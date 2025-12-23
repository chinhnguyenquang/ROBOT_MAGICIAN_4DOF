/*
 * alt_main.cpp
 *
 *  Created on: Dec 3, 2025
 *      Author: CHINH
 */

#include "alt_main.h"
#include "string.h"


#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"

#include "Dwin.h"
#include "STEP.h"
#include "Sethome.h"

extern "C" {
    #include "FreeRTOS.h"
    #include "task.h"
    #include "queue.h"
}


#include "Robot_xyz.h"

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;


extern osMessageQueueId_t Queue_DWINHandle;
extern osThreadId_t Task_DMA_HMIHandle;
extern osSemaphoreId_t Seme_Tx_dwinHandle;


extern volatile flag_sethome_t _Flag;

Robot_State_t _Robot_state;




uint8_t data[64];

#define HUT(x) ((x) ? (GPIOE->BSRR = GPIO_BSRR_BS6) : (GPIOE->BSRR = GPIO_BSRR_BR6))


DwinDgus* _Dwin = nullptr;
STEP * _STEP1=nullptr;
STEP* _STEP2=nullptr;
STEP* _STEP3=nullptr;

uint64_t pos_e=0;

uint8_t pos_dbg[64];

void pos_state_to_array(uint64_t state, uint8_t *arr)
{
    for (uint8_t i = 0; i < 64; i++)
    {
        arr[i] = (state >> i) & 0x01;
    }
}





void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  UNUSED(Size);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UARTEx_RxEventCallback can be implemented in the user file.
   */
  if (huart->Instance==USART3){
	  memset(data,'0',64);
	  memcpy ( data, _Dwin->RxData, Size );
	  _Dwin->handled_data_interrupt();
	  pos_state_to_array(_Dwin->pos_state, pos_dbg);

		HAL_UARTEx_ReceiveToIdle_DMA(_Dwin->_huart, _Dwin->RxData, 32);
		__HAL_DMA_DISABLE_IT(_Dwin->_hdma_rx,DMA_IT_HT);

  }

}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_5) {   _Flag.flag1 = 1;}
    else if (GPIO_Pin == GPIO_PIN_3) { _Flag.flag2 = 1;}
    else if (GPIO_Pin == GPIO_PIN_0) { _Flag.flag3 = 1;}
    else if (GPIO_Pin == GPIO_PIN_9) { _Flag.flag4 = 1;}
}



uint8_t data_i2c1[2];
uint8_t data_i2c2[2];
uint8_t data_i2c3[2];
uint32_t raw[3];
uint8_t flag_init_step=0;


int32_t hello=0;

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {

    if (hi2c->Instance == I2C1) {
        raw[0] = (((uint16_t)data_i2c1[0] << 8) | data_i2c1[1]) & 0x0FFF;
        _STEP3->update_As5600_cur(raw[0]);

    }
    else if (hi2c->Instance == I2C2) {
        raw[1] = (((uint16_t)data_i2c2[0] << 8) | data_i2c2[1]) & 0x0FFF;
        _STEP1->update_As5600_cur(raw[1]);


    }
    else if (hi2c->Instance == I2C3) {
        raw[2] = (((uint16_t)data_i2c3[0] << 8) | data_i2c3[1]) & 0x0FFF;
        _STEP2->update_As5600_cur(raw[2]);
        if (flag_init_step==0){
        	hello=raw[2]; flag_init_step=1;
        }

    }
}


#define AS5600_ADDR 0x36  // I2C address sensor






void readAS5600_DMA() {
    uint8_t reg = 0x0C;

    if (hi2c1.State == HAL_I2C_STATE_READY) {
        HAL_I2C_Master_Transmit(&hi2c1, AS5600_ADDR<<1, &reg, 1, 30);
        HAL_I2C_Master_Receive_DMA(&hi2c1, AS5600_ADDR<<1, data_i2c1, 2);
    }

    if (hi2c2.State == HAL_I2C_STATE_READY) {
        HAL_I2C_Master_Transmit(&hi2c2, AS5600_ADDR<<1, &reg, 1, 30);
        HAL_I2C_Master_Receive_DMA(&hi2c2, AS5600_ADDR<<1, data_i2c2, 2);
    }

    if (hi2c3.State == HAL_I2C_STATE_READY) {
        HAL_I2C_Master_Transmit(&hi2c3, AS5600_ADDR<<1, &reg, 1, 30);
        HAL_I2C_Master_Receive_DMA(&hi2c3, AS5600_ADDR<<1, data_i2c3, 2);
    }
}


uint8_t buffer_rs485[16];


void Sensor_AS5600_RTOS(){


	for(;;)
	  {

		readAS5600_DMA();
		 osDelay(5);
	  }
}






void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3)
    {

    	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    	osSemaphoreRelease(Seme_Tx_dwinHandle);

    	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}





void DwinUsartTask_RTOS(){
	uint8_t req;
	for(;;)
	{
	    osMessageQueueGet(Queue_DWINHandle, &req, NULL, osWaitForever);

	    osSemaphoreAcquire(Seme_Tx_dwinHandle, osWaitForever); // <-- chặn trước khi gửi

	    if (req != 255){
	        uint16_t add=0x1000+(uint16_t)req*4;
	        _Dwin->setVP(add,0); // gọi DMA sau khi đã "chiếm" semaphore
	    }
	    else {
	        _Dwin->beepHMI();


	        _Robot_state=ROBOT_ACTIVE;
	        _STEP1->STEP_set_Target(90);
	        _STEP2->STEP_set_Target(45);
	        _STEP3->STEP_set_Target(35);
	    }
	}
}




void Dwin_RTOS(void)
{
    uint8_t req;

    _Dwin= new DwinDgus(&huart3,&hdma_usart3_rx);
    uint8_t count=0;
    for (;;)
    {
        if (_Dwin->Flag_send)
        {
            for (uint8_t i = 0; i < Count_Point; ++i)
            {
                    if (_Dwin->pos_state & (1ULL << i))
                    {
                        req = i;
                        osMessageQueuePut(Queue_DWINHandle, &req, 0, 0);
                        ++count;
                    }

                    if (count==(_Dwin->point_count_new-1)) break;

            }
            count=0;
            _Dwin->point_count_new=0;
            _Dwin->point_count_old=0;
            req = 255; // beep
            osMessageQueuePut(Queue_DWINHandle, &req, 0, 0);

            _Dwin->Flag_send = false;
            _Dwin->pos_state = 0;
        }

        osDelay(50);
    }
}




uint16_t a=0,b=0;
int16_t c=0;
int32_t d=0;


int16_t e=0,f=0,g=0,j=0;
int16_t Q=0,W=0,R=0,T=0;



bool Flag_cho_set_home=false;
void TIM2_CALLBACK_STEP(void)
{
	if(Flag_cho_set_home){
		if(!_Flag.flag1) _STEP1->update_ISR();
		if(!_Flag.flag2) _STEP2->update_ISR();
		if(!_Flag.flag3) _STEP3->update_ISR();
	}

	else if(_Robot_state==ROBOT_ACTIVE){
		_STEP1->update_ISR(); //BEN TRONG DA CO BIEN ENABLE
		_STEP2->update_ISR();
		_STEP3->update_ISR();


	}

}

int32_t mang1[10],mang2[10],mang3[10];

bool Set_dir_sethome;

void Control_motor_RTOS(){
	for (;;)
    {
		Home_Update_All();  //UPDATE DEN BÁO SETHOME
		if (_Robot_state==ROBOT_SETHOME){

				if (_Flag.flag1) _STEP1->STEP_set_home_trigger(); //NEU CO QUAY THEM SE THEM 1 LAN TU CABLI
				if (_Flag.flag2) _STEP2->STEP_set_home_trigger();
				if (_Flag.flag3) _STEP3->STEP_set_home_trigger();

				////MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM


				if ((_Flag.flag1)&&(_Flag.flag2)&&(_Flag.flag3)&&(_Flag.flag4)) {_Robot_state=ROBOT_IDLE; }//DUNG IM

				//MUON QUAY VE SET HOME PHAI UPDATE THEM SET DIR SETHOME
				if((!Set_dir_sethome)){
					if(!_Flag.flag1)  {HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15,GPIO_PIN_RESET);}
					if(!_Flag.flag2)  {HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,GPIO_PIN_RESET);}
					if(!_Flag.flag3)  {HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3,GPIO_PIN_SET);}
					_Flag.flag4_bd=false;
					Set_dir_sethome=true;
					_STEP1->is_enable_step=true;_STEP1->STEPx.Chieuquayhientai=STEP_SETHOME;
					_STEP2->is_enable_step=true;_STEP2->STEPx.Chieuquayhientai=STEP_SETHOME;
					_STEP3->is_enable_step=true;_STEP3->STEPx.Chieuquayhientai=STEP_SETHOME;

					Flag_cho_set_home=true; //bien trang thai isr

				}


				if ((_Flag.flag1)&&(_Flag.flag2)&&(_Flag.flag3)&&(Set_dir_sethome)){
					_Flag.flag4_bd=true;
					Flag_cho_set_home=false;
				}
				Sethome_link4(&htim3,&htim4);
		}

		else if (_Robot_state==ROBOT_ACTIVE){

			_STEP1->STEP_Process();
			_STEP2->STEP_Process();
			_STEP3->STEP_Process();

			mang1[0]=_STEP1->STEPx.angle_as56_tar;
			mang1[1]=_STEP1->STEPx.angle_as56_cur;
			mang1[2]=_STEP1->STEPx.target_step;
			mang1[3]=_STEP1->STEPx.current_step;
			mang1[4]=_STEP1->STEPx.angle_kc_candat;
			mang1[5]=_STEP1->i;

			mang2[0]=_STEP2->STEPx.angle_as56_tar;
			mang2[1]=_STEP2->STEPx.angle_as56_cur;
			mang2[2]=_STEP2->STEPx.target_step;
			mang2[3]=_STEP2->STEPx.current_step;
			mang2[4]=_STEP2->STEPx.angle_kc_candat;
			mang2[5]=_STEP2->i;

			mang3[0]=_STEP3->STEPx.angle_as56_tar;
			mang3[1]=_STEP3->STEPx.angle_as56_cur;
			mang3[2]=_STEP3->STEPx.target_step;
			mang3[3]=_STEP3->STEPx.current_step;
			mang3[4]=_STEP3->STEPx.angle_kc_candat;
			mang3[5]=_STEP3->i;

		}

		 osDelay(1);

    }


}





int alt_main()
{
	/* Initialization */

	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
	__HAL_TIM_SET_COUNTER(&htim3, 32768);


	_STEP2 = new STEP(
	    0, 0, 1,                 // M0, M1, M2
	    12, GPIOB,               // M0 pin/port
	    13, GPIOB,               // M1 pin/port
	    14, GPIOB,               // M2 pin/port
		3, GPIOA,       // STEP pin/port
	    15, GPIOB       // DIR pin/port
	);


	_STEP1 = new STEP(
	    0, 0, 1,                 // M0, M1, M2
	    12, GPIOE,               // M0 pin/port
	    13, GPIOE,               // M1 pin/port
	    14, GPIOE,               // M2 pin/port
	    2, GPIOA,       // STEP pin/port
	    15, GPIOE       // DIR pin/port
	);


	_STEP3 = new STEP(
	    0, 0, 1,                 // M0, M1, M2
	    0, GPIOC,                // M0 pin/port
	    1, GPIOC,                // M1 pin/port
	    2, GPIOC,                // M2 pin/port
	    1, GPIOA,       // STEP pin/port
	    3, GPIOC        // DIR pin/port
	);


	_STEP1->STEP_set_cal(2707,true,false);
	_STEP2->STEP_set_cal(3249, true,true);
	_STEP3->STEP_set_cal(1338, false,false);
	Home_Update_All();
	_STEP1->setStepPeriod(50);
	_STEP2->setStepPeriod(50);
	_STEP3->setStepPeriod(50);
	//drv8871
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start (&htim4,TIM_CHANNEL_4);


	//DE TIEN HANH SET HOME CAN TAC DONG
	_Robot_state=ROBOT_SETHOME;
	Set_dir_sethome=false;
	//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
	HUT(0);

	return 0;
}


