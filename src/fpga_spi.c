/**
 * FPGA SPI Communication Implementation - Interrupt-Driven Version
 *
 * Non-blocking SPI communication with FPGA controller.
 * 
 * Hardware Configuration:
 *   - CS:  IO1 (PB8)
 *   - IRQ: IO2 (PB5) - FPGA asserts this when computation is done
 *   - SPI: Standard deck SPI (SCK, MOSI, MISO)
 *
 * Flow:
 *   1. Controller calls fpgaSpiUpdateState() - stores state in buffer (fast)
 *   2. FPGA asserts IRQ (IO2/PB5 rising edge) when computation done
 *   3. EXTI5 ISR signals high-priority task
 *   4. Task does SPI transfer: receive results, send new state
 *   5. Controller calls fpgaSpiGetControl() - gets latest results (fast)
 *
 * The SPI transfer happens asynchronously, NOT in the control loop!
 */

#include "fpga_spi.h"
#include "deck.h"
#include "deck_spi.h"
#include "sleepus.h"
#include "usec_time.h"
#include "exti.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "stm32fxxx.h"

#define DEBUG_MODULE "FPGA_SPI"
#include "debug.h"

// Buffer sizes
#define TX_BUFFER_SIZE  (3 * FPGA_N_STATE)    // 36 bytes state data
#define RX_BUFFER_SIZE  (3 * FPGA_N_RESULT)   // 12 bytes result data

// Task configuration
#define FPGA_TASK_NAME       "FPGA_SPI"
#define FPGA_TASK_STACKSIZE  (2 * configMINIMAL_STACK_SIZE)
#define FPGA_TASK_PRI        (configMAX_PRIORITIES - 1)  // Highest priority!

//--------------------------------------------------------------
// Double Buffers for Lock-Free Operation
//--------------------------------------------------------------

typedef struct {
    uint8_t data[TX_BUFFER_SIZE];
    volatile bool valid;
} stateBuffer_t;

typedef struct {
    fpgaControl_t control;
    volatile bool valid;
    volatile bool newData;
} controlBuffer_t;

// State double buffer (controller writes, SPI task reads)
static stateBuffer_t stateBuffers[2];
static volatile uint8_t stateActiveBuffer = 0;  // Buffer being read by SPI task

// Control double buffer (SPI task writes, controller reads)  
static controlBuffer_t controlBuffers[2];
static volatile uint8_t controlActiveBuffer = 0;  // Buffer being read by controller

// RX buffer for SPI transaction
static uint8_t rxBuffer[RX_BUFFER_SIZE];

//--------------------------------------------------------------
// Module State
//--------------------------------------------------------------

static bool isInit = false;
static deckPin_t csPinConfig;
static deckPin_t irqPinConfig;

// Synchronization
static SemaphoreHandle_t irqSemaphore = NULL;
static TaskHandle_t fpgaTaskHandle = NULL;

// Statistics
static volatile fpgaStats_t stats = {0};

//--------------------------------------------------------------
// Fixed-Point Conversion
//--------------------------------------------------------------

uint32_t fpgaFloatToFixed24(float value) {
    int32_t scaled = (int32_t)(value * FPGA_FIXED_POINT_SCALE);
    
    // Clamp to 24-bit signed range
    if (scaled > 8388607) scaled = 8388607;
    if (scaled < -8388608) scaled = -8388608;
    
    return (uint32_t)(scaled & 0xFFFFFF);
}

float fpgaFixed24ToFloat(uint32_t fixed) {
    int32_t signed_val = (int32_t)(fixed & 0xFFFFFF);
    if (signed_val & 0x800000) {
        signed_val |= 0xFF000000;  // Sign extend
    }
    return (float)signed_val / FPGA_FIXED_POINT_SCALE;
}

//--------------------------------------------------------------
// Buffer Packing/Unpacking
//--------------------------------------------------------------

static void packStateToBuffer(uint8_t *buffer, const state_t *state, const sensorData_t *sensors) {
    uint32_t stateWords[FPGA_N_STATE];
    
    // Position (meters)
    stateWords[0] = fpgaFloatToFixed24(state->position.x);
    stateWords[1] = fpgaFloatToFixed24(state->position.y);
    stateWords[2] = fpgaFloatToFixed24(state->position.z);
    
    // Velocity (m/s)
    stateWords[3] = fpgaFloatToFixed24(state->velocity.x);
    stateWords[4] = fpgaFloatToFixed24(state->velocity.y);
    stateWords[5] = fpgaFloatToFixed24(state->velocity.z);
    
    // Attitude (degrees)
    stateWords[6] = fpgaFloatToFixed24(state->attitude.roll);
    stateWords[7] = fpgaFloatToFixed24(state->attitude.pitch);
    stateWords[8] = fpgaFloatToFixed24(state->attitude.yaw);
    
    // Angular rates (deg/s)
    stateWords[9]  = fpgaFloatToFixed24(sensors->gyro.x);
    stateWords[10] = fpgaFloatToFixed24(sensors->gyro.y);
    stateWords[11] = fpgaFloatToFixed24(sensors->gyro.z);
    
    // Pack 24-bit words LSB first
    for (int i = 0; i < FPGA_N_STATE; i++) {
        uint32_t v = stateWords[i];
        buffer[3*i + 0] = (v >> 0)  & 0xFF;
        buffer[3*i + 1] = (v >> 8)  & 0xFF;
        buffer[3*i + 2] = (v >> 16) & 0xFF;
    }
}

static void unpackControlFromBuffer(const uint8_t *buffer, fpgaControl_t *control) {
    uint32_t resultWords[FPGA_N_RESULT];
    
    // Unpack 24-bit words (MSB first)
    for (int i = 0; i < FPGA_N_RESULT; i++) {
        resultWords[i] = ((uint32_t)buffer[3*i + 0] << 16) |
                         ((uint32_t)buffer[3*i + 1] << 8)  |
                         ((uint32_t)buffer[3*i + 2] << 0);
    }
    
    control->motor1 = fpgaFixed24ToFloat(resultWords[0]);
    control->motor2 = fpgaFixed24ToFloat(resultWords[1]);
    control->motor3 = fpgaFixed24ToFloat(resultWords[2]);
    control->motor4 = fpgaFixed24ToFloat(resultWords[3]);
}

//--------------------------------------------------------------
// EXTI5 Callback (IO2 / PB5 interrupt)
// This is called from the EXTI ISR when FPGA signals "done"
//--------------------------------------------------------------

void __attribute__((used)) EXTI5_Callback(void) {
    if (irqSemaphore != NULL) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(irqSemaphore, &xHigherPriorityTaskWoken);
        stats.interruptCount++;
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

//--------------------------------------------------------------
// FPGA Communication Task (High Priority)
//--------------------------------------------------------------

static void fpgaSpiTask(void *param) {
    DEBUG_PRINT("FPGA SPI task started (priority %d)\n", FPGA_TASK_PRI);
    
    while (1) {
        // Wait for interrupt from FPGA (computation done)
        if (xSemaphoreTake(irqSemaphore, portMAX_DELAY) == pdTRUE) {
            
            uint64_t startTime = usecTimestamp();
            
            //--------------------------------------------------
            // Get the latest valid state buffer
            //--------------------------------------------------
            // Find the most recently written valid buffer
            uint8_t readIdx = 1 - stateActiveBuffer;  // Try inactive buffer first
            if (!stateBuffers[readIdx].valid) {
                readIdx = stateActiveBuffer;  // Fall back to active
            }
            
            if (!stateBuffers[readIdx].valid) {
                // No valid state yet, skip this cycle
                stats.failCount++;
                continue;
            }
            
            // Mark this buffer as being used
            stateActiveBuffer = readIdx;
            
            //--------------------------------------------------
            // SPI Transaction: Full-duplex exchange
            // - Send: new state data
            // - Receive: motor commands (result of previous computation)
            //--------------------------------------------------
            spiBeginTransaction(FPGA_SPI_BAUDRATE);
            digitalWrite(csPinConfig, LOW);
            
            // Full-duplex exchange - happens in ~20us at 21MHz
            spiExchange(TX_BUFFER_SIZE, stateBuffers[readIdx].data, rxBuffer);
            
            digitalWrite(csPinConfig, HIGH);
            spiEndTransaction();
            
            //--------------------------------------------------
            // Unpack received control data
            //--------------------------------------------------
            fpgaControl_t newControl;
            unpackControlFromBuffer(rxBuffer, &newControl);
            
            // Write to inactive control buffer
            uint8_t writeIdx = 1 - controlActiveBuffer;
            controlBuffers[writeIdx].control = newControl;
            controlBuffers[writeIdx].valid = true;
            controlBuffers[writeIdx].newData = true;
            
            // Atomic pointer swap - make new data visible
            controlActiveBuffer = writeIdx;
            
            //--------------------------------------------------
            // Update statistics
            //--------------------------------------------------
            uint64_t endTime = usecTimestamp();
            uint32_t elapsed = (uint32_t)(endTime - startTime);
            
            stats.lastTransactionUs = elapsed;
            stats.successCount++;
            
            if (elapsed > stats.maxTransactionUs) {
                stats.maxTransactionUs = elapsed;
            }
            
            if (stats.avgTransactionUs == 0) {
                stats.avgTransactionUs = elapsed;
            } else {
                stats.avgTransactionUs = (stats.avgTransactionUs * 7 + elapsed) / 8;
            }
        }
    }
}

//--------------------------------------------------------------
// EXTI Configuration for IO2 (PB5)
//--------------------------------------------------------------

static void configureExtiForIO2(void) {
    EXTI_InitTypeDef EXTI_InitStructure;
    
    // Connect EXTI Line 5 to PB5
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource5);
    
    // Configure EXTI Line 5 for rising edge (FPGA asserts high when done)
    EXTI_InitStructure.EXTI_Line = EXTI_Line5;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
}

//--------------------------------------------------------------
// Public API
//--------------------------------------------------------------

bool fpgaSpiInit(deckPin_t csPin, deckPin_t irqPin) {
    if (isInit) {
        return true;
    }
    
    csPinConfig = csPin;
    irqPinConfig = irqPin;
    
    // Initialize buffers
    for (int i = 0; i < 2; i++) {
        stateBuffers[i].valid = false;
        controlBuffers[i].valid = false;
        controlBuffers[i].newData = false;
    }
    
    // Create binary semaphore for interrupt signaling
    irqSemaphore = xSemaphoreCreateBinary();
    if (irqSemaphore == NULL) {
        DEBUG_PRINT("ERROR: Failed to create semaphore\n");
        return false;
    }
    
    // Initialize SPI subsystem
    spiBegin();
    
    // Configure CS pin as output, initially high (deselected)
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);
    
    // Configure IRQ pin as input
    pinMode(irqPin, INPUT);
    
    // Configure EXTI for IO2 (PB5 = EXTI5)
    // Note: If using a different IRQ pin, you'd need different EXTI config
    configureExtiForIO2();
    
    // Create high-priority task for SPI transfers
    BaseType_t result = xTaskCreate(
        fpgaSpiTask, 
        FPGA_TASK_NAME, 
        FPGA_TASK_STACKSIZE,
        NULL, 
        FPGA_TASK_PRI, 
        &fpgaTaskHandle
    );
    
    if (result != pdPASS) {
        DEBUG_PRINT("ERROR: Failed to create FPGA task\n");
        return false;
    }
    
    isInit = true;
    DEBUG_PRINT("FPGA SPI initialized (CS=IO%d, IRQ=IO%d)\n", 
                csPin.id - 3, irqPin.id - 3);  // Convert to IO number
    
    return true;
}

bool fpgaSpiIsReady(void) {
    return isInit && controlBuffers[controlActiveBuffer].valid;
}

void fpgaSpiUpdateState(const state_t *state, const sensorData_t *sensors) {
    if (!isInit) return;
    
    // Write to inactive buffer
    uint8_t writeIdx = 1 - stateActiveBuffer;
    
    // Pack state data into buffer
    packStateToBuffer(stateBuffers[writeIdx].data, state, sensors);
    stateBuffers[writeIdx].valid = true;
    
    // No need to swap - SPI task will read whichever is valid
}

bool fpgaSpiGetControl(fpgaControl_t *control) {
    if (!isInit) return false;
    
    uint8_t readIdx = controlActiveBuffer;
    
    if (!controlBuffers[readIdx].valid) {
        return false;
    }
    
    // Copy control data (fast - just 16 bytes)
    *control = controlBuffers[readIdx].control;
    
    // Clear new data flag
    controlBuffers[readIdx].newData = false;
    
    return true;
}

bool fpgaSpiHasNewData(void) {
    if (!isInit) return false;
    return controlBuffers[controlActiveBuffer].newData;
}

const fpgaStats_t* fpgaSpiGetStats(void) {
    return (const fpgaStats_t*)&stats;
}

//--------------------------------------------------------------
// Blocking Transaction (for testing only!)
//--------------------------------------------------------------

bool fpgaSpiTransactBlocking(const state_t *state,
                             const sensorData_t *sensors,
                             fpgaControl_t *control,
                             uint32_t timeoutMs) {
    if (!isInit) return false;
    
    // Update state buffer
    fpgaSpiUpdateState(state, sensors);
    
    // Wait for new control data
    TickType_t startTick = xTaskGetTickCount();
    TickType_t timeoutTicks = pdMS_TO_TICKS(timeoutMs);
    
    while ((xTaskGetTickCount() - startTick) < timeoutTicks) {
        if (fpgaSpiHasNewData()) {
            return fpgaSpiGetControl(control);
        }
        vTaskDelay(1);
    }
    
    DEBUG_PRINT("Blocking transaction timeout\n");
    return false;
}
