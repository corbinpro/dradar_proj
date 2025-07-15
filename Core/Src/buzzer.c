/*
 * buzzer.c
 *
 *  Created on: Jul 15, 2025
 *      Author: corbinpro
 */
#include "buzzer.h"

extern TIM_HandleTypeDef htim1; // Change TIM1 to your timer

void Buzzer_Init(void) {
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

void Buzzer_On(uint16_t frequency) {
    uint32_t timerClock = 80000000; // 80 MHz default APB2
    uint32_t prescaler = 79;        // Must match CubeMX setting
    uint32_t period = (timerClock / (prescaler + 1)) / frequency - 1;

    __HAL_TIM_SET_AUTORELOAD(&htim1, period);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, period / 2); // 50% duty
    __HAL_TIM_SET_COUNTER(&htim1, 0);
}

void Buzzer_Off(void) {
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
}

