/**
 * FPGA-based Out-of-Tree Controller - Interrupt-Driven Version
 *
 * This controller communicates with an external FPGA over SPI to compute
 * control commands. Communication is ASYNCHRONOUS and NON-BLOCKING.
 *
 * Hardware Setup:
 *   - CS:  IO1 (directly controls chip select)
 *   - IRQ: IO2 (FPGA asserts when computation done)
 *   - SPI: SCK, MOSI, MISO (deck SPI bus)
 *
 * Data Flow:
 *   1. Controller updates state buffer (fast, non-blocking)
 *   2. FPGA raises IRQ when done computing
 *   3. High-priority task does SPI exchange (receive results, send new state)
 *   4. Controller reads latest motor commands (fast, non-blocking)
 *
 * The control loop NEVER blocks waiting for the FPGA!
 * There is 1 cycle of latency (motor commands reflect state from previous cycle).
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "app.h"
#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "FPGA_CTRL"
#include "debug.h"

#include "controller.h"
#include "controller_pid.h"
#include "log.h"
#include "param.h"
#include "deck.h"

#include "fpga_spi.h"
#include "param_logic.h"

//--------------------------------------------------------------
// Configuration
//--------------------------------------------------------------

// Hardware pins
#define FPGA_CS_PIN   DECK_GPIO_IO1
#define FPGA_IRQ_PIN  DECK_GPIO_IO2

//--------------------------------------------------------------
// State Variables
//--------------------------------------------------------------

// Controller state
static bool fpgaInitialized = false;
static bool useFpgaController = true;
static bool fpgaFallbackToPid = true;

// Latest FPGA control output (for logging and hold)
static fpgaControl_t lastFpgaControl = {0};
static bool haveValidControl = false;

// Motor command clamping
static float motorMin = 0.0f;
static float motorMax = 1.0f;

// Statistics for logging
static uint32_t fpgaSuccessCount = 0;
static uint32_t fpgaFailCount = 0;
static uint32_t fpgaIrqCount = 0;
static uint32_t lastTransactionUs = 0;
static uint32_t maxTransactionUs = 0;

//--------------------------------------------------------------
// Application Main Task
//--------------------------------------------------------------

void appMain() {
    DEBUG_PRINT("FPGA Controller app starting...\n");
    
    // Wait for system to be ready
    vTaskDelay(M2T(500));
    
    // Force switch to PID controller (1) at boot
    // User can switch to FPGA (6) with: stabilizer.controller = 6
    paramVarId_t controllerParam = paramGetVarId("stabilizer", "controller");
    paramSetInt(controllerParam, 1);  // 1 = ControllerTypePID
    DEBUG_PRINT("Set stabilizer.controller=1 (PID)\n");
    DEBUG_PRINT("  Use stabilizer.controller=6 for FPGA\n");
    
    // Initialize FPGA SPI communication with CS and IRQ pins
    if (fpgaSpiInit(FPGA_CS_PIN, FPGA_IRQ_PIN)) {
        fpgaInitialized = true;
        DEBUG_PRINT("FPGA SPI ready (CS=IO1, IRQ=IO2)\n");
    } else {
        DEBUG_PRINT("ERROR: Failed to initialize FPGA SPI\n");
    }
    
    // Monitoring loop - update statistics
    while (1) {
        vTaskDelay(M2T(500));
        
        if (fpgaInitialized) {
            const fpgaStats_t *stats = fpgaSpiGetStats();
            fpgaSuccessCount = stats->successCount;
            fpgaFailCount = stats->failCount;
            fpgaIrqCount = stats->interruptCount;
            lastTransactionUs = stats->avgTransactionUs;
            maxTransactionUs = stats->maxTransactionUs;
        }
    }
}

//--------------------------------------------------------------
// Out-of-Tree Controller Interface
//--------------------------------------------------------------

void controllerOutOfTreeInit(void) {
    DEBUG_PRINT("FPGA out-of-tree controller init\n");
    
    // Initialize PID controller as fallback
    controllerPidInit();
}

bool controllerOutOfTreeTest(void) {
    return true;
}

void controllerOutOfTree(control_t *control,
                         const setpoint_t *setpoint,
                         const sensorData_t *sensors,
                         const state_t *state,
                         const uint32_t tick) {
    
    //----------------------------------------------------------
    // Check if FPGA controller is enabled and ready
    //----------------------------------------------------------
    if (!fpgaInitialized || !useFpgaController) {
        controllerPid(control, setpoint, sensors, state, tick);
        return;
    }
    
    //----------------------------------------------------------
    // Update state buffer for FPGA (non-blocking, ~1us)
    // This just copies data into a buffer - the actual SPI
    // transfer happens asynchronously when FPGA triggers IRQ
    //----------------------------------------------------------
    fpgaSpiUpdateState(state, sensors);
    
    //----------------------------------------------------------
    // Get latest control output (non-blocking, ~0.1us)
    //----------------------------------------------------------
    fpgaControl_t fpgaResult;
    
    if (fpgaSpiGetControl(&fpgaResult)) {
        // Got new/valid control data from FPGA
        lastFpgaControl = fpgaResult;
        haveValidControl = true;
    }
    
    //----------------------------------------------------------
    // Apply control output
    //----------------------------------------------------------
    if (haveValidControl) {
        control->controlMode = controlModeForce;
        
        // Clamp motor values to valid range
        control->normalizedForces[0] = fmaxf(motorMin, fminf(motorMax, lastFpgaControl.motor1));
        control->normalizedForces[1] = fmaxf(motorMin, fminf(motorMax, lastFpgaControl.motor2));
        control->normalizedForces[2] = fmaxf(motorMin, fminf(motorMax, lastFpgaControl.motor3));
        control->normalizedForces[3] = fmaxf(motorMin, fminf(motorMax, lastFpgaControl.motor4));
    } else {
        // No valid FPGA data yet - use fallback
        if (fpgaFallbackToPid) {
            controllerPid(control, setpoint, sensors, state, tick);
        } else {
            // Zero output for safety
            control->controlMode = controlModeForce;
            control->normalizedForces[0] = 0.0f;
            control->normalizedForces[1] = 0.0f;
            control->normalizedForces[2] = 0.0f;
            control->normalizedForces[3] = 0.0f;
        }
    }
}

//--------------------------------------------------------------
// Logging Variables
//--------------------------------------------------------------

LOG_GROUP_START(fpga)
LOG_ADD(LOG_UINT8, init, &fpgaInitialized)
LOG_ADD(LOG_UINT8, enabled, &useFpgaController)
LOG_ADD(LOG_UINT8, valid, &haveValidControl)
LOG_ADD(LOG_UINT32, success, &fpgaSuccessCount)
LOG_ADD(LOG_UINT32, fail, &fpgaFailCount)
LOG_ADD(LOG_UINT32, irqCnt, &fpgaIrqCount)
LOG_ADD(LOG_UINT32, txTimeUs, &lastTransactionUs)
LOG_ADD(LOG_UINT32, txMaxUs, &maxTransactionUs)
LOG_ADD(LOG_FLOAT, m1, &lastFpgaControl.motor1)
LOG_ADD(LOG_FLOAT, m2, &lastFpgaControl.motor2)
LOG_ADD(LOG_FLOAT, m3, &lastFpgaControl.motor3)
LOG_ADD(LOG_FLOAT, m4, &lastFpgaControl.motor4)
LOG_GROUP_STOP(fpga)

//--------------------------------------------------------------
// Parameters
//--------------------------------------------------------------

PARAM_GROUP_START(fpga)
PARAM_ADD(PARAM_UINT8, enable, &useFpgaController)
PARAM_ADD(PARAM_UINT8, fallback, &fpgaFallbackToPid)
PARAM_ADD(PARAM_FLOAT, motorMin, &motorMin)
PARAM_ADD(PARAM_FLOAT, motorMax, &motorMax)
PARAM_GROUP_STOP(fpga)
