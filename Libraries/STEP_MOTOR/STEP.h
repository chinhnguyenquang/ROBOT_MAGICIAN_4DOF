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

typedef enum
{
	STEP_SETHOME,
    STEP_OPEN_LOOP,       // DK LAN 1 (OPEN LOOP)
	STEP_CLOSED,		//DK LAN 2 (SD AS5600)
    STEP_DONE,          // Step hoàn thành
//    STEP_ERROR          // Step gặp lỗi
} StepState_t;



typedef enum{
	STEP_DUONG,
	STEP_AM,
}Chieuquay_state_t;

typedef struct
{
	/*Angle */

	int32_t angle_as56_init;
	int32_t angle_as56_cal; //bien khoi tao de luu diem 0_
	int32_t angle_as56_last;
	int32_t angle_as56_cur; //bien luu giu da quay bao nhieu goc
	int32_t angle_as56_tar;//so goc can quay
	int32_t angle_kc_candat;
	int32_t angle_est;

    /* Encoder */

    int32_t angle_cur; //goc hien tai
    int32_t angle_tar;//goc mong muon

    /* Step */
    int32_t target_step;
    volatile int32_t current_step;

    Chieuquay_state_t Chieuquayhientai;


} STEP_Handle_t;

typedef struct {
	//CAC CO KHOI TAO 1 LAN
	bool flag_angle_init; // khoi tao de luu gia tri ban dau cua goc dau tien (angle_init)
	bool flag_trang_thaidau_vehome; //KHI CHAM VAO HOME THI CONG THEM KHOANG DE VE VI TRI 0
	bool flag_set_dir;//SET DIR DE CHON CHIEU 1 LAN
	bool flag_set_chieu_openloop; //khi set openloop

}FLAG_STEPx_Handle_t;

class STEP {
private :
	Resolution x;
	GPIO_TypeDef* ports[3];
	inline void write_pin(GPIO_TypeDef* port, uint8_t pin, bool val)
	{
	    port->BSRR = val ? (1U << pin) : (1U << (pin + 16));
	}


	GPIO_TypeDef* _step_port;
	uint16_t _step_pin;
	GPIO_TypeDef* _dir_port;
	uint16_t _dir_pin;
	FLAG_STEPx_Handle_t Flags;
	STEP_Handle_t STEPx;
	bool dir_Stepx;
	bool dir_AS;


	bool Flag_step_oke;
	StepState_t Status_Step;
	uint8_t update_closed_loop;

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

        Flags.flag_angle_init=false;
        Flags.flag_set_dir=false;
        Flags.flag_trang_thaidau_vehome=false;
        Flags.flag_set_chieu_openloop=false;

        STEPx.target_step=0;
        STEPx.current_step=0;

        STEPx.angle_as56_cur=0;
        STEPx.angle_as56_tar=0;

        Ki_acc=0;
        stable_cnt=0;
        last_error=0;
        remain=0;

        Flag_step_oke=false;

        Status_Step=STEP_DONE;

	    write_pin(ports[0], x.P0, x.M0);
	    write_pin(ports[1], x.P1, x.M1);
	    write_pin(ports[2], x.P2, x.M2);
    }


private:

    void update_ISR();
    volatile uint32_t step_timer;
    volatile bool step_state;

    volatile bool is_enable_step=false;

    int32_t remain;

public:
    void STEP_set_target_as(int32_t target);
	 void update_As5600_cur(int32_t value);
	 void STEP_Process();

	 void dir_step(uint8_t dir);



	 friend void TIM2_CALLBACK_STEP();
	 friend void Control_motor_RTOS();
	 friend void Control_Dwin_get_theta_RTOS();
	 friend void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

	 void STEP_set_dir_as_step(bool dir,bool dir_As); //chon chieu true (set) chieu cua encoder (false)
	 void STEP_set_home_trigger(void);

	 void STEP_CLOSEDLOOP();
	 int32_t last_error;
	 int32_t stable_cnt;
	 float Ki_acc;
	 void update_goc_debug();
	//SET TOC DO QUAY STEP DUA VAO HAM ISR
private:
	uint32_t step_period_us;
public:
	void setStepPeriod(uint32_t period);


public:
	void STEP_OPENLOOP();
};









#ifdef __cplusplus
}
#endif

#endif


/* STEP_H_ */
