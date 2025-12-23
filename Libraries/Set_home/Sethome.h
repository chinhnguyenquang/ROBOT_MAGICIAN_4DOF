/*
 * Sethome.h
 *
 *  Created on: Dec 9, 2025
 *      Author: CHINH
 */

#ifndef LIBRARIES_SET_HOME_SETHOME_H_
#define LIBRARIES_SET_HOME_SETHOME_H_



#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"



typedef struct {
    GPIO_TypeDef *port_input;
    GPIO_TypeDef *port_output;
    uint8_t pin_input;
    uint8_t pin_output;
} SetHome;



typedef struct {
	uint8_t flag1:1;
	uint8_t flag2:1;
	uint8_t flag3:1;
	uint8_t flag4:1;
	uint8_t flag4_bd:1;
	uint8_t :3;
}flag_sethome_t;



typedef enum {
    ROBOT_SETHOME = 0,   // Robot đang về vị trí gốc (home)
    ROBOT_ACTIVE  = 1,    // Robot di chuyển các khớp để hút vật
	ROBOT_IDLE=2
} Robot_State_t;



//flag=1 đang chạm sethome
extern volatile flag_sethome_t _Flag;


// Hàm xử lý
void Home_Update_All(void);

void Sethome_link4(TIM_HandleTypeDef *htim,TIM_HandleTypeDef *tim_pwm);




#ifdef __cplusplus
}
#endif





#endif /* LIBRARIES_SET_HOME_SETHOME_H_ */
