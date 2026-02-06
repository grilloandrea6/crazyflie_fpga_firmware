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


static bool fpgaControllerInitialized = false;

// static uint32_t fpgaSuccessCount = 0;
// static uint32_t fpgaFailCount = 0;
// static uint32_t fpgaIrqCount = 0;
// static uint32_t lastTransactionUs = 0;
// static uint32_t maxTransactionUs = 0;

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
    vTaskDelay(M2T(2000));
    pinMode(FPGA_CS_PIN, OUTPUT);
    pinMode(FPGA_IRQ_PIN, INPUT_PULLDOWN);

    digitalWrite(FPGA_CS_PIN, HIGH);

    DEBUG_PRINT("Status of FPGA IRQ pin: %d\n", digitalRead(FPGA_IRQ_PIN));

    spiBegin();

    sleepus(300);
    DEBUG_PRINT("FPGA out-of-tree controller initialized. Now sending dummy first transaction.\n");
    spiBeginTransaction(FPGA_SPI_BAUDRATE);
    digitalWrite(FPGA_CS_PIN, LOW);
    sleepus(50);

    uint8_t txBuffer[64] = {0};
    uint8_t rxBuffer[64] = {0};

    txBuffer[0] = 0x00;
    txBuffer[1] = 0x00;
    txBuffer[2] = 0xAA; 
    // State words (24-bit, LSB first)
    for (int i = 0; i < 12; i++) {
        txBuffer[3 + 3*i + 0] = 0xFF;
        txBuffer[3 + 3*i + 1] = 0xBB;
        txBuffer[3 + 3*i + 2] = 0x03;
    }
    // for (int i = 0; i < 64; i++) {
    //     DEBUG_PRINT("  TX[%d] = 0x%02X\n", i, txBuffer[i]);
    // }
    spiExchange(39, txBuffer, rxBuffer);
    DEBUG_PRINT("SENT\n");

    fpgaControllerInitialized = true;   
    // sleepus(10000000);
}

bool controllerOutOfTreeTest(void) {
    return true;
}
static uint8_t rxBuffer[64] = {0};
static uint8_t rxBuffer2[64] = {0};
static uint8_t txBuffer[64] = {0};
static uint8_t runTimes = 0;

void controllerOutOfTree(control_t *control,
                         const setpoint_t *setpoint,
                         const sensorData_t *sensors,
                         const state_t *state,
                         const uint32_t tick) {
    if(!fpgaControllerInitialized) {
        return;
    }
    if(runTimes > 30) {
        return;
    }
    runTimes++;
  
    uint64_t start = usecTimestamp();
    // DEBUG_PRINT("FPGA OOTOOTOOTOOTOOTOOTOOTOOTOOT\n");


    for(int i = 0; i < 50000; i++) {

        if(digitalRead(FPGA_IRQ_PIN) == HIGH) {
            // DEBUG_PRINT("  cycle %d, FPGA IRQ went HIGH!\n", i);
            break;
        }
        
        // DEBUG_PRINT("  Dummy SPI byte %d, RX = 0x%02X\n", i, rxBuffer[0]);
        // sleepus(10);
    }
    // DEBUG_PRINT("we managed to break!\n");
bool ready = false;
    for(int i = 0; i < 1000; i++) {
       spiExchange(1, txBuffer, rxBuffer);
        if(rxBuffer[0] == 0xFF) {
            // DEBUG_PRINT("  Dummy SPI byte %d, RX = 0x%02X\n", i, rxBuffer[0]);
            ready = true;
            break;
        }
    }
    if(!ready) {
        // DEBUG_PRINT("  FPGA DUMMY NOT 0xFF timeout - no data received\n");
    } else {
        // DEBUG_PRINT("  FPGA 0xFF received!\n");
    }

    for(int i = 0; i < 64; i++) {
        txBuffer[i] = 0x00;
    }

    spiExchange(14, txBuffer, rxBuffer);


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
    // for (int i = 0; i < 64; i++) {
    //     DEBUG_PRINT("  TX[%d] = 0x%02X\n", i, txBuffer[i]);
    // }
    spiExchange(39, txBuffer, rxBuffer2);

    uint64_t end = usecTimestamp();
    DEBUG_PRINT("SENT\n");
    DEBUG_PRINT("TIME: %lld us\n", end - start);


    for(volatile int i = 0; i < 14; i++) {
        DEBUG_PRINT("  RX[%d] = 0x%02X\n", i, rxBuffer[i]);
    }
    // if(RATE_DO_EXECUTE((RATE_100_HZ/100), tick)) {
    //     DEBUG_PRINT("FPGA out-of-tree controller transaction...\n");
        
    //     while(digitalRead(FPGA_IRQ_PIN) == LOW);

    //     while(1) {
    //         spiExchange(39, txBuffer, rxBuffer);
    //         for(uint8_t i = 0; i < 39; i++) {
    //             DEBUG_PRINT("  RX[%d] = 0x%02X\n", i, rxBuffer[i]);
    //         }
    //     }

    //     digitalWrite(FPGA_CS_PIN, HIGH);
    //     sleepus(50);
    //     digitalWrite(FPGA_CS_PIN, HIGH);

    //     spiExchange(39, txBuffer, rxBuffer);
    
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
