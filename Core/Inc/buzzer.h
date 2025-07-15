#ifndef BUZZER_H
#define BUZZER_H

#include "stm32l4xx_hal.h"

void Buzzer_Init(void);
void Buzzer_On(uint16_t frequency);  // frequency in Hz
void Buzzer_Off(void);

#endif
