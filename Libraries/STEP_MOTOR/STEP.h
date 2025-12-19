/*
 * STEP.h
 *
 *  Created on: Dec 5, 2025
 *      Author: CHINH
 */

#ifndef STEP_H_
#define STEP_H_



#ifdef __cplusplus
extern "C"
{
#endif


#include "stm32f4xx_hal.h"
#include "alt_main.h"

struct Resolution {
    unsigned char M0 :1;
    unsigned char M1 :1;
    unsigned char M2 :1;
    unsigned char P0 :4; // pin M0 (0-15)
    unsigned char P1 :4; // pin M1 (0-15)
    unsigned char P2 :4; // pin M2 (0-15)

    unsigned char :1;
};

class STEP {
private :
	Resolution x;
	GPIO_TypeDef* ports[3];
	inline void write_pin(GPIO_TypeDef* port, uint8_t pin, bool val)
	{
	    port->BSRR = val ? (1U << pin) : (1U << (pin + 16));
	}

	inline void update_Resolution()
	{
	    write_pin(ports[0], x.P0, x.M0);
	    write_pin(ports[1], x.P1, x.M1);
	    write_pin(ports[2], x.P2, x.M2);
	}
	GPIO_TypeDef* _step_port;
	uint16_t _step_pin;
	GPIO_TypeDef* _dir_port;
	uint16_t _dir_pin;


	uint32_t angle_init=0;
	uint32_t angle_target=0;
	uint32_t angle_current=0;

private:
    void update_ISR();
    volatile uint32_t step_period_us;
    volatile uint32_t step_timer;
    volatile bool step_state;
    volatile uint32_t position_count=0;
public:
    friend void TIM2_CALLBACK_STEP();
    friend void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c);
    void setStepPeriod(uint32_t period);
    uint32_t getPosition();
    friend void Sensor_AS5600_RTOS();


public:
	STEP(uint8_t M0, uint8_t M1, uint8_t M2,
	         uint8_t pin0, GPIO_TypeDef* port0,
	         uint8_t pin1, GPIO_TypeDef* port1,
	         uint8_t pin2, GPIO_TypeDef* port2,
	         uint16_t step_pin, GPIO_TypeDef* step_port,
	         uint16_t dir_pin, GPIO_TypeDef* dir_port)
	        : _step_port(step_port), _step_pin(step_pin),
	          _dir_port(dir_port), _dir_pin(dir_pin), step_state(0)
	  {
        x.M0 = M0;
        x.M1 = M1;
        x.M2 = M2;

        x.P0= pin0;
        x.P1 = pin1;
        x.P2 = pin2;

        ports[0] = port0;
        ports[1] = port1;
        ports[2] = port2;

        update_Resolution();
    }
};









#ifdef __cplusplus
}
#endif

#endif


/* STEP_H_ */
