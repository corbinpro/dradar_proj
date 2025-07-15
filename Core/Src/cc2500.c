#include "cc2500.h"
#include "main.h"
#include <stdio.h>
#include "stm32l4xx_hal_conf.h"
#include "char_lcd.h"

// Exposed values
int8_t CC2500_NoiseFloor = -100;
int8_t CC2500_DetectionThreshold = -80;
char rssiString[16];


extern SPI_HandleTypeDef hspi1;

//write registers to config
void CC2500_WriteRegister(uint8_t addr, uint8_t value) {
    CC2500_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &addr, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, &value, 1, HAL_MAX_DELAY);
    CC2500_CS_HIGH();
}

//read current config to write back unmodified values
uint8_t CC2500_ReadRegister(uint8_t addr) {
    uint8_t value;
    addr |= 0x80;
    CC2500_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &addr, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, &value, 1, HAL_MAX_DELAY);
    CC2500_CS_HIGH();
    return value;
}

//??? TODO
void CC2500_Strobe(uint8_t cmd) {
    CC2500_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    CC2500_CS_HIGH();
}

//set channel ?? TODO
void CC2500_SetChannel(uint8_t channel) {
    CC2500_WriteRegister(0x0A, channel);
    CC2500_Strobe(CC2500_SRX);
}

//Chech if packets are being recieved ?? TODO
uint8_t CC2500_ReadRSSI(void) {
    uint8_t raw = CC2500_ReadRegister(0x34);
    uint8_t calcValue = (raw >= 128) ? ((int8_t)(raw - 256) / 2 - 74) : (raw / 2 - 74);
    return calcValue;
}

//Config registers
/*
 * Setup Table:
| Address (Hex) | Register Name | Value (Hex) | Purpose                       |
| ------------- | ------------- | ----------- | ----------------------------- |
| `0x00`        | IOCFG2        | `0x29`      | GDO2 output config (optional) |
| `0x02`        | IOCFG0        | `0x06`      | GDO0 as sync word received    |
| `0x0B`        | FSCTRL1       | `0x06`      | Frequency synthesizer control |
| `0x0C`        | FSCTRL0       | `0x00`      |                               |
| `0x0D`        | FREQ2         | `0x5D`      | 2.433 GHz                     |
| `0x0E`        | FREQ1         | `0x93`      |                               |
| `0x0F`        | FREQ0         | `0xB1`      |                               |
| `0x10`        | MDMCFG4       | `0x2D`      | Modem config                  |
| `0x11`        | MDMCFG3       | `0x3B`      |                               |
| `0x12`        | MDMCFG2       | `0x73`      |                               |
| `0x15`        | DEVIATN       | `0x01`      |                               |
| `0x18`        | MCSM0         | `0x18`      | Auto-calibrate                |
| `0x19`        | FOCCFG        | `0x1D`      | Frequency Offset Correction   |
| `0x1A`        | BSCFG         | `0x1C`      | Bit synchronization           |
| `0x21`        | FREND0        | `0x11`      | Front end RX configuration    |
| `0x22`        | FSCAL3        | `0xE9`      | Frequency synthesizer cal     |
| `0x23`        | FSCAL2        | `0x2A`      |                               |
| `0x24`        | FSCAL1        | `0x00`      |                               |
| `0x25`        | FSCAL0        | `0x1F`      |                               |
| `0x07`        | PKTCTRL1      | `0x04`      | No address check              |
| `0x08`        | PKTCTRL0      | `0x05`      | Infinite packet length, CRC   |
| `0x3E`        | PATABLE       | `0xC0`      | Max output power              |
 * */
void CC2500_ApplyConfig(void) {
    CC2500_WriteRegister(0x00, 0x29);
    CC2500_WriteRegister(0x02, 0x06);
    CC2500_WriteRegister(0x03, 0x07);
    CC2500_WriteRegister(0x06, 0x00);
    CC2500_WriteRegister(0x07, 0x04);
    CC2500_WriteRegister(0x08, 0x05);
    CC2500_WriteRegister(0x0A, 0x00);
    CC2500_WriteRegister(0x0B, 0x06);
    CC2500_WriteRegister(0x0C, 0x00);
    CC2500_WriteRegister(0x0D, 0x5D);
    CC2500_WriteRegister(0x0E, 0x93);
    CC2500_WriteRegister(0x0F, 0xB1);
    CC2500_WriteRegister(0x10, 0x2D);
    CC2500_WriteRegister(0x11, 0x3B);
    CC2500_WriteRegister(0x12, 0x73);
    CC2500_WriteRegister(0x15, 0x01);
    CC2500_WriteRegister(0x18, 0x18);
    CC2500_WriteRegister(0x19, 0x1D);
    CC2500_WriteRegister(0x1A, 0x1C);
    CC2500_WriteRegister(0x21, 0x11);
    CC2500_WriteRegister(0x22, 0xE9);
    CC2500_WriteRegister(0x23, 0x2A);
    CC2500_WriteRegister(0x24, 0x00);
    CC2500_WriteRegister(0x25, 0x1F);
    CC2500_WriteRegister(0x3E, 0xC0);
}

//apply configurations and establish first noise floor
void CC2500_Init(void) {
    HAL_Delay(100);
    CC2500_CS_HIGH(); HAL_Delay(1);
    CC2500_CS_LOW();  HAL_Delay(1);
    CC2500_CS_HIGH(); HAL_Delay(1);

    CC2500_Strobe(CC2500_SRES);
    HAL_Delay(1);

    CC2500_ApplyConfig();
    CC2500_Strobe(CC2500_SRX);

    // Initial noise floor calibration
    CC2500_RecalibrateNoiseFloor();
}

//re calibrate average noise floor.
void CC2500_RecalibrateNoiseFloor(void) {
	CharLCD_Set_Cursor(0,7); // Set cursor to row 1, column 0
	CharLCD_Write_String("CALBRATNG");

	int32_t sum = 0;
    const uint8_t ch_min = 0;
    const uint8_t ch_max = 100;
    const int sweep_count = ch_max - ch_min + 1;

    for (uint8_t ch = ch_min; ch <= ch_max; ch++) {
        CC2500_SetChannel(ch);
        HAL_Delay(3);
        int8_t rssi = CC2500_ReadRSSI();
        sum += rssi;
    }

    CC2500_NoiseFloor = sum / sweep_count;
    //starting threshold value:10 increase or deacrease to desired sensitivity. TODO Possibly itegrate button to change this value.
    CC2500_DetectionThreshold = CC2500_NoiseFloor + 10;

}

// Use sweep mode and read rssi to see if packets are being recieved on any channels, read strength, alert on noise floor threshold
void CC2500_SweepAndDetect(void) {
    for (uint8_t ch = 0; ch <= 100; ch++) {
        CC2500_SetChannel(ch);
        HAL_Delay(3);
        int8_t rssi = CC2500_ReadRSSI();

        //OUTPUT ON DETECTION
        if (rssi > CC2500_DetectionThreshold) {
        	CharLCD_Set_Cursor(1,7); // Set cursor to row 1, column 0
        	CharLCD_Write_String("DETECTED");

        	//TODO ADD Trigger alarm
        }
        else {
        	sprintf(rssiString, "RSSI:%d", rssi);
        	CharLCD_Set_Cursor(1,7); // Set cursor to row 1, column 0
        	CharLCD_Write_String(rssiString);

        	//TODO ADD turn off alarm
        }
    }
}
