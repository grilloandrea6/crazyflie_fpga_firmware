/**
 * FPGA SPI Communication Interface - Interrupt-Driven Version
 *
 * This module provides non-blocking SPI communication with an FPGA controller.
 * 
 * Protocol:
 *   1. FPGA asserts interrupt pin when computation is complete
 *   2. On interrupt: receive 4 motor commands, send 12 state values
 *   3. FPGA begins computing on new state
 *   4. Controller uses latest motor commands (1-cycle latency)
 *
 * Data format: 24-bit fixed-point Q16.8 (16 integer bits, 8 fractional bits)
 */

#ifndef FPGA_SPI_H
#define FPGA_SPI_H

#include <stdint.h>
#include <stdbool.h>
#include "stabilizer_types.h"
#include "deck_constants.h"

// Number of 24-bit state words to send to FPGA
#define FPGA_N_STATE    12

// Number of 24-bit result words to receive from FPGA
#define FPGA_N_RESULT   4

// Fixed-point scaling for Q16.8 format
#define FPGA_FIXED_POINT_SCALE  256.0f   // 2^8

// SPI configuration
#define FPGA_SPI_BAUDRATE       SPI_BAUDRATE_21MHZ  // Adjust based on FPGA capabilities

// Result structure from FPGA - direct motor commands
typedef struct {
    float motor1;      // Motor 1 normalized thrust (0.0 to 1.0)
    float motor2;      // Motor 2 normalized thrust (0.0 to 1.0)
    float motor3;      // Motor 3 normalized thrust (0.0 to 1.0)
    float motor4;      // Motor 4 normalized thrust (0.0 to 1.0)
} fpgaControl_t;

// Transaction statistics
typedef struct {
    uint32_t successCount;
    uint32_t failCount;
    uint32_t interruptCount;
    uint32_t lastTransactionUs;
    uint32_t avgTransactionUs;
    uint32_t maxTransactionUs;
} fpgaStats_t;

/**
 * Initialize the FPGA SPI interface with interrupt support
 * @param csPin The chip select pin (e.g., DECK_GPIO_IO1)
 * @param irqPin The interrupt pin from FPGA (e.g., DECK_GPIO_IO2)
 * @return true if initialization successful
 */
bool fpgaSpiInit(deckPin_t csPin, deckPin_t irqPin);

/**
 * Check if FPGA SPI is initialized and ready
 * @return true if ready
 */
bool fpgaSpiIsReady(void);

/**
 * Update the state buffer with current state data.
 * This should be called from the controller to provide fresh state.
 * The actual SPI transfer happens asynchronously on FPGA interrupt.
 * 
 * @param state Current state from state estimator
 * @param sensors Current sensor data
 */
void fpgaSpiUpdateState(const state_t *state, const sensorData_t *sensors);

/**
 * Get the latest control output from FPGA (non-blocking).
 * Returns the most recent motor commands received from FPGA.
 * 
 * @param control Output control structure to fill
 * @return true if valid data available, false if no data yet
 */
bool fpgaSpiGetControl(fpgaControl_t *control);

/**
 * Check if new control data is available since last call to fpgaSpiGetControl
 * @return true if new data available
 */
bool fpgaSpiHasNewData(void);

/**
 * Get transaction statistics
 * @return Pointer to statistics structure
 */
const fpgaStats_t* fpgaSpiGetStats(void);

/**
 * Convert float to 24-bit fixed-point Q16.8
 * @param value Float value to convert
 * @return 24-bit fixed-point value (stored in uint32_t)
 */
uint32_t fpgaFloatToFixed24(float value);

/**
 * Convert 24-bit fixed-point Q16.8 to float
 * @param fixed 24-bit fixed-point value
 * @return Float value
 */
float fpgaFixed24ToFloat(uint32_t fixed);

/**
 * Force a synchronous transaction (for testing/initialization).
 * This WILL block! Only use during startup or testing.
 * 
 * @param state Current state
 * @param sensors Current sensor data  
 * @param control Output control structure
 * @param timeoutMs Timeout in milliseconds
 * @return true if successful
 */
bool fpgaSpiTransactBlocking(const state_t *state,
                             const sensorData_t *sensors,
                             fpgaControl_t *control,
                             uint32_t timeoutMs);

#endif // FPGA_SPI_H
