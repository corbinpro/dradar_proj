#ifndef CC2500_H
#define CC2500_H

#include "stm32l4xx_hal.h"

// SPI & GPIO control
#define CC2500_CS_PORT GPIOA
#define CC2500_CS_PIN  GPIO_PIN_4

#define CC2500_CS_LOW()    HAL_GPIO_WritePin(CC2500_CS_PORT, CC2500_CS_PIN, GPIO_PIN_RESET)
#define CC2500_CS_HIGH()   HAL_GPIO_WritePin(CC2500_CS_PORT, CC2500_CS_PIN, GPIO_PIN_SET)

// Command strobes
#define CC2500_SRES    0x30
#define CC2500_SRX     0x34

// Public API
void CC2500_Init(void);
void CC2500_SetChannel(uint8_t channel);
uint8_t CC2500_ReadRSSI(void);
void CC2500_Strobe(uint8_t cmd);
void CC2500_SweepAndDetect(void);
void CC2500_ApplyConfig(void);
void CC2500_RecalibrateNoiseFloor(void);

// Shared values
extern int8_t CC2500_NoiseFloor;
extern int8_t CC2500_DetectionThreshold;

#endif
