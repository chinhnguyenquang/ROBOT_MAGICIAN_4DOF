/*
 * Sethome.c
 *
 *  Created on: Dec 9, 2025
 *      Author: CHINH
 */



#include "Sethome.h"


#define SET_HOME_COUNT 4


volatile flag_sethome_t _Flag;

SetHome SetHomeArr[SET_HOME_COUNT] = {
    { GPIOE, GPIOE, 5, 4},   // STEP 1
    { GPIOE, GPIOE, 3, 2},   // STEP 2
    { GPIOE, GPIOE, 0, 1},   // STEP 3
    { GPIOB, GPIOB, 9, 8}    // DC MOTOR
};

void Home_Update_All(void)
{
    for (int i = 0; i < SET_HOME_COUNT; i++)
    {
    	 uint8_t status =(SetHomeArr[i].port_input->IDR >> SetHomeArr[i].pin_input) & 0x01;
        status=(status==0)?1:0;
        // Update bit-field
        switch(i)
        {
            case 0: _Flag.flag1 = status; break;
            case 1: _Flag.flag2 = status; break;
            case 2: _Flag.flag3 = status; break;
            case 3: _Flag.flag4 = status; break;
        }

        if (status) {
            // RESET pin (LOW)
            SetHomeArr[i].port_output->BSRR =
                (1U << (SetHomeArr[i].pin_output + 16));
        } else {
            // SET pin (HIGH)
            SetHomeArr[i].port_output->BSRR =
                (1U << SetHomeArr[i].pin_output);
        }
    }
}


void Sethome_link4(TIM_HandleTypeDef *htim,TIM_HandleTypeDef *tim_pwm){


	//BD QUAY 45 DO: 4*11*270/8 (11 TỈ SỐ DÂY ĐAI ROBOT)
	if (!_Flag.flag4_bd){
		uint32_t encoder=32768-__HAL_TIM_GET_COUNTER(htim);
		if(encoder < 1485){
			__HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_3, 50);
			__HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_4, 0);
		}
		else {
  			__HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_3, 0);
  			__HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_4, 0);
		}
	}
	else {
		if(!_Flag.flag4){
			__HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_3, 0);
			__HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_4, 50);
		}
		else {
  			__HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_3, 0);
  			__HAL_TIM_SET_COMPARE(tim_pwm, TIM_CHANNEL_4, 0);
  			__HAL_TIM_SET_COUNTER(htim, 32768);  //SET LAI DIEM GIUA
		}
	}
}


