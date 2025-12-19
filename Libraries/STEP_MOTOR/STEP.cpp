/*
 * STEP.cpp
 *
 *  Created on: Dec 5, 2025
 *      Author: CHINH
 */


#include "STEP.h"

void STEP::update_ISR(){
    if(step_period_us == 0) return;
    step_timer++;
    if(step_timer >= step_period_us) {
        step_timer = 0;
        step_state ^= 1;
        HAL_GPIO_WritePin(_step_port, _step_pin, step_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
        if(step_state) position_count++;
    }
}


void STEP::setStepPeriod(uint32_t period) {

    const uint32_t MIN_PERIOD_US = 50; // giới hạn min
    if(period < MIN_PERIOD_US) period = MIN_PERIOD_US;
    step_period_us = period/50;
}


uint32_t STEP::getPosition() { return position_count; }
