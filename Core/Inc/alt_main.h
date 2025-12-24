/*
 * alt_main.h
 *
 *  Created on: Dec 3, 2025
 *      Author: CHINH
 */

#ifndef ALT_MAIN_H_
#define ALT_MAIN_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"

int alt_main();

//RTOS
void Sensor_AS5600_RTOS(void);

void Dwin_RTOS(void);
void DwinUsartTask_RTOS(void);
void Control_motor_RTOS(void);
void Control_Dwin_get_theta_RTOS(void);

//TIM
void TIM2_CALLBACK_STEP(void);


#ifdef __cplusplus
}
#endif

#endif
