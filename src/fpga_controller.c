/**
 * FPGA-based Out-of-Tree Controller
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
#include "log.h"
#include "param.h"
#include "deck.h"
#include "sleepus.h"

#include "param_logic.h"

//--------------------------------------------------------------
// Configuration
//--------------------------------------------------------------

// Hardware pins
#define FPGA_CS_PIN       DECK_GPIO_IO1
#define FPGA_IRQ_PIN      DECK_GPIO_IO2
#define FPGA_SPI_BAUDRATE SPI_BAUDRATE_12MHZ

#define TX_LEN 39
#define RX_LEN 14


#define FRAC_BITS 22
#define SCALE     (1 << FRAC_BITS)

/* 24-bit signed range for Q2.22: about -2.0 to +2.0 */
#define FP24_MIN  (-(1 << 23))
#define FP24_MAX  ((1 << 23) - 1)

static bool fpgaControllerInitialized = false;
static uint8_t emptyBuffer[TX_LEN] = {0};
static uint8_t txBuffer[TX_LEN] = {0};
static uint8_t rxBuffer[RX_LEN] = {0};
static uint8_t dummyBuffer[RX_LEN] = {0};

static uint8_t runTimes = 0;

#include "math3d.h"
// #ifndef M_PI_F
//   #define M_PI_F   (3.14159265358979323846f)
// #endif
// struct vec {
// 	float x; float y; float z;
// };
// static inline float radians(float degrees) { return (M_PI_F / 180.0f) * degrees; }


void appMain() {
    DEBUG_PRINT("FPGA Controller app started.\n");
    
    while (1) {
        vTaskDelay(M2T(20000));
    }
}

//--------------------------------------------------------------
// Out-of-Tree Controller Interface
//--------------------------------------------------------------





void controllerOutOfTreeInit(void) {
    DEBUG_PRINT("FPGA out-of-tree controller init...\n");
    
    pinMode(FPGA_CS_PIN, OUTPUT);
    pinMode(FPGA_IRQ_PIN, INPUT_PULLDOWN);
    digitalWrite(FPGA_CS_PIN, HIGH);
    spiBegin();
    spiBeginTransaction(FPGA_SPI_BAUDRATE);

    DEBUG_PRINT("FPGA out-of-tree controller initialized.\n");

    vTaskDelay(M2T(2000)); // Wait for FPGA to be ready

    DEBUG_PRINT("Sending dummy first transaction.\n");
    
    digitalWrite(FPGA_CS_PIN, LOW);

    uint8_t localTxBuffer[TX_LEN] = {0}; localTxBuffer[2] = 0xAA; 
    uint8_t localRxBuffer[TX_LEN];

    spiExchange(TX_LEN, localTxBuffer, localRxBuffer);
    DEBUG_PRINT("Dummy transaction sent.\n");

    fpgaControllerInitialized = true;   
}

bool controllerOutOfTreeTest(void) {
    return true;
}
static inline quaternion_t normalize_quat(quaternion_t q)
{
    float inv_mag = 1.0f / sqrtf(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    return (quaternion_t){
        .x = q.x * inv_mag,
        .y = q.y * inv_mag,
        .z = q.z * inv_mag,
        .w = q.w * inv_mag
    };
}

static inline struct vec quat_2_rp(quaternion_t q)
{
  struct vec v;
  v.x = q.x / q.w;
  v.y = q.y / q.w;
  v.z = q.z / q.w;
  return v;
}

void float_to_24bit_fixed_at(float value, uint8_t *buf, size_t offset)
{
    double scaled = (double)value * (double)SCALE;
    int32_t fp24;

    if (scaled <= (double)FP24_MIN) {
        fp24 = FP24_MIN;
    } else if (scaled >= (double)FP24_MAX) {
        fp24 = FP24_MAX;
    } else {
        fp24 = (int32_t)(scaled < 0 ? scaled - 0.5 : scaled + 0.5);
    }

    uint32_t u = (uint32_t)fp24 & 0xFFFFFFu;
    buf[offset + 0] = (uint8_t)(u >> 0);
    buf[offset + 1] = (uint8_t)(u >> 8);
    buf[offset + 2] = (uint8_t)(u >> 16);
}

float fixed_24bit_to_float_at(const uint8_t *buf, size_t offset)
{
    /* Assemble 24-bit value: first byte = MSB (bits 16–23), same as receive_val_24bit_from_buf */
    uint32_t u24 = ((uint32_t)buf[offset]     << 16)
                 | ((uint32_t)buf[offset + 1] << 8)
                 | ((uint32_t)buf[offset + 2] << 0);

    /* Sign-extend from bit 23 to 32 bits */
    int32_t s24 = (int32_t)((u24 << 8) >> 8);

    return (float)s24 * (1.0f / (float)SCALE);
}

void stateToTxBuffer(const state_t *state, const sensorData_t *sensors, uint8_t *buffer) {

    // ToDo difference from setpoint

    struct vec phi = quat_2_rp(normalize_quat(state->attitudeQuaternion)); // quaternion to Rodrigues parameters
    DEBUG_PRINT("phi: (%.2f, %.2f, %.2f)\n", (double)phi.x, (double)phi.y, (double)phi.z);

    buffer[0] = 0x00;
    buffer[1] = 0x00;
    buffer[2] = 0xAA;

    float_to_24bit_fixed_at(state->position.x, buffer, 3);
    float_to_24bit_fixed_at(state->position.y, buffer, 6);
    float_to_24bit_fixed_at(state->position.z - 1, buffer, 9); // ToDo tmp hover at 1m height
    float_to_24bit_fixed_at(phi.x, buffer, 12);
    float_to_24bit_fixed_at(phi.y, buffer, 15);
    float_to_24bit_fixed_at(phi.z, buffer, 18);
    float_to_24bit_fixed_at(state->velocity.x, buffer, 21);
    float_to_24bit_fixed_at(state->velocity.y, buffer, 24);
    float_to_24bit_fixed_at(state->velocity.z, buffer, 27);
    float_to_24bit_fixed_at(radians(sensors->gyro.x), buffer, 30);
    float_to_24bit_fixed_at(radians(sensors->gyro.y), buffer, 33);
    float_to_24bit_fixed_at(radians(sensors->gyro.z), buffer, 36);
}

void rxBufferToControl(const uint8_t *buffer, control_t *control) {
    control->controlMode = controlModeForce;
    control->normalizedForces[0] = fixed_24bit_to_float_at(buffer, 0);
    control->normalizedForces[1] = fixed_24bit_to_float_at(buffer, 3);
    control->normalizedForces[2] = fixed_24bit_to_float_at(buffer, 6);
    control->normalizedForces[3] = fixed_24bit_to_float_at(buffer, 9);
}

void controllerOutOfTree(control_t *control,
                         const setpoint_t *setpoint,
                         const sensorData_t *sensors,
                         const state_t *state,
                         const uint32_t tick) {
    if(!fpgaControllerInitialized) {
        return;
    }
    runTimes++;

    stateToTxBuffer(state, sensors, txBuffer);
  
    // uint64_t start = usecTimestamp();

    for(int i = 0; i < 50000; i++) {
        if(digitalRead(FPGA_IRQ_PIN) == HIGH) {
            break;
        }
    }

    for(int i = 0; i < 1000; i++) {
       spiExchange(1, emptyBuffer, rxBuffer);
        if(rxBuffer[0] == 0xFF) {
            break;
        }
    }

    spiExchange(RX_LEN, emptyBuffer, rxBuffer);

    digitalWrite(FPGA_CS_PIN, HIGH);
    sleepus(10);
    digitalWrite(FPGA_CS_PIN, LOW);

    txBuffer[0] = 0x00;
    txBuffer[1] = 0x00;
    txBuffer[2] = 0xAA; 
    // State words (24-bit, LSB first)
    for (int i = 0; i < 12; i++) {
        txBuffer[3 + 3*i + 0] = (runTimes + i) & 0xFF;
        txBuffer[3 + 3*i + 1] = 0xBB;
        txBuffer[3 + 3*i + 2] = 0x03;
    }

    spiExchange(39, txBuffer, dummyBuffer);

    // uint64_t end = usecTimestamp();


    rxBufferToControl(rxBuffer, control);

    
    // DEBUG_PRINT("TIME: %lld us\n", end - start);
    // for(volatile int i = 0; i < 14; i++) {
    //     DEBUG_PRINT("  RX[%d] = 0x%02X\n", i, rxBuffer[i]);
    // }
}

//--------------------------------------------------------------
// Logging Variables
//--------------------------------------------------------------

// LOG_GROUP_START(fpga)
// LOG_ADD(LOG_UINT8, init, &fpgaInitialized)
// LOG_GROUP_STOP(fpga)
//--------------------------------------------------------------
// Parameters
//--------------------------------------------------------------

// PARAM_GROUP_START(fpga)
// PARAM_GROUP_STOP(fpga)
