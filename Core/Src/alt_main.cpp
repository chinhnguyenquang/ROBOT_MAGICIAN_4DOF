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
#include "Dc_motor.h"


extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;


extern osMessageQueueId_t Queue_DWINHandle;
extern osThreadId_t Task_DMA_HMIHandle;
extern osSemaphoreId_t Seme_Tx_dwinHandle;
extern osSemaphoreId_t Theta_xyzHandle;;


extern volatile flag_sethome_t _Flag;

Robot_State_t _Robot_state;


RobotTheta_t *Angle_4_theta=nullptr;

uint8_t data[64];

#define HUT(x) ((x) ? (GPIOE->BSRR = GPIO_BSRR_BS6) : (GPIOE->BSRR = GPIO_BSRR_BR6))


DwinDgus* _Dwin = nullptr;
STEP * _STEP1=nullptr;
STEP* _STEP2=nullptr;
STEP* _STEP3=nullptr;

bool Set_dir_sethome;

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
    if (GPIO_Pin == GPIO_PIN_5) {   _Flag.flag1 = 1;_STEP1->is_enable_step=false;}
    else if (GPIO_Pin == GPIO_PIN_3) { _Flag.flag2 = 1;_STEP2->is_enable_step=false;}
    else if (GPIO_Pin == GPIO_PIN_0) { _Flag.flag3 = 1;_STEP3->is_enable_step=false;}
    else if (GPIO_Pin == GPIO_PIN_9) { _Flag.flag4 = 1;}
}



uint8_t data_i2c1[2];
uint8_t data_i2c2[2];
uint8_t data_i2c3[2];
uint32_t raw[3];
uint8_t flag_init_step=0;




uint16_t yyy[3];


#define AS5600_ADDR 0x36  // I2C address sensor


static void I2C_BusRecover(I2C_HandleTypeDef *hi2c,
                           GPIO_TypeDef *SCL_Port, uint16_t SCL_Pin,
                           GPIO_TypeDef *SDA_Port, uint16_t SDA_Pin)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    HAL_I2C_DeInit(hi2c);

    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull  = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = SCL_Pin;
    HAL_GPIO_Init(SCL_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = SDA_Pin;
    HAL_GPIO_Init(SDA_Port, &GPIO_InitStruct);

    /* Release bus */
    HAL_GPIO_WritePin(SCL_Port, SCL_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SDA_Port, SDA_Pin, GPIO_PIN_SET);
    osDelay(1);

    /* Clock out 9 pulses */
    for (int i = 0; i < 9; i++)
    {
        HAL_GPIO_WritePin(SCL_Port, SCL_Pin, GPIO_PIN_RESET);
        osDelay(1);
        HAL_GPIO_WritePin(SCL_Port, SCL_Pin, GPIO_PIN_SET);
        osDelay(1);
    }

    /* STOP */
    HAL_GPIO_WritePin(SDA_Port, SDA_Pin, GPIO_PIN_RESET);
    osDelay(1);
    HAL_GPIO_WritePin(SCL_Port, SCL_Pin, GPIO_PIN_SET);
    osDelay(1);
    HAL_GPIO_WritePin(SDA_Port, SDA_Pin, GPIO_PIN_SET);

    HAL_I2C_Init(hi2c);
}


static HAL_StatusTypeDef AS5600_Read(I2C_HandleTypeDef *hi2c,
                                     uint16_t *value,
                                     uint8_t *buf,
                                     GPIO_TypeDef *SCL_Port, uint16_t SCL_Pin,
                                     GPIO_TypeDef *SDA_Port, uint16_t SDA_Pin)
{
    if (HAL_I2C_Mem_Read(hi2c,
                         AS5600_ADDR << 1,
                         0x0C,
                         I2C_MEMADD_SIZE_8BIT,
                         buf,
                         2,
                         10) == HAL_OK)
    {
        *value = ((buf[0] << 8) | buf[1]) & 0x0FFF;
        return HAL_OK;
    }

    uint32_t err = HAL_I2C_GetError(hi2c);
    if (err == HAL_I2C_ERROR_TIMEOUT ||
        err == HAL_I2C_ERROR_BERR ||
        err == HAL_I2C_ERROR_AF)
    {
        I2C_BusRecover(hi2c,
                       SCL_Port, SCL_Pin,
                       SDA_Port, SDA_Pin);
    }

    return HAL_ERROR;
}


/* ================= I2C1 ================= */
#define I2C1_SCL_Port GPIOB
#define I2C1_SCL_Pin  GPIO_PIN_8
#define I2C1_SDA_Port GPIOB
#define I2C1_SDA_Pin  GPIO_PIN_9

/* ================= I2C2 ================= */
#define I2C2_SCL_Port GPIOB
#define I2C2_SCL_Pin  GPIO_PIN_10
#define I2C2_SDA_Port GPIOB
#define I2C2_SDA_Pin  GPIO_PIN_11

/* ================= I2C3 ================= */
#define I2C3_SCL_Port GPIOA
#define I2C3_SCL_Pin  GPIO_PIN_8
#define I2C3_SDA_Port GPIOC
#define I2C3_SDA_Pin  GPIO_PIN_9

#define AS5600_ADDR   0x36

void Sensor_AS5600_RTOS()
{
    uint16_t val1 = 0, val2 = 0, val3 = 0;
    uint8_t buf1[2], buf2[2], buf3[2];

    /* Chờ AS5600 ổn định sau power-on */
    osDelay(50);

    for (;;)
    {
        /* ===== I2C1 ===== */
        if (AS5600_Read(&hi2c1, &val1, buf1,
                        I2C1_SCL_Port, I2C1_SCL_Pin,
                        I2C1_SDA_Port, I2C1_SDA_Pin) == HAL_OK)
        {
            _STEP3->update_As5600_cur(val1);
            yyy[0] = val1;
        }

        /* ===== I2C2 ===== */
        if (AS5600_Read(&hi2c2, &val2, buf2,
                        I2C2_SCL_Port, I2C2_SCL_Pin,
                        I2C2_SDA_Port, I2C2_SDA_Pin) == HAL_OK)
        {
            _STEP1->update_As5600_cur(val2);
            yyy[1] = val2;
        }

        /* ===== I2C3 ===== */
        if (AS5600_Read(&hi2c3, &val3, buf3,
                        I2C3_SCL_Port, I2C3_SCL_Pin,
                        I2C3_SDA_Port, I2C3_SDA_Pin) == HAL_OK)
        {
            _STEP2->update_As5600_cur(val3);
            yyy[2] = val3;
        }

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




void Update_Theta_Robot(uint8_t req){Angle_4_theta=Robot_GetTheta(req);}

uint16_t yui=0;
uint16_t theta[4];
extern RobotTheta_t QH_QD[POINT_QHQD];
uint8_t bien_co_set_theta=0;
void Control_Dwin_get_theta_RTOS(void){



    for (;;)
    {
    	if(Angle_4_theta != nullptr){
    		if(bien_co_set_theta==0){
    			_Robot_state=ROBOT_ACTIVE;

				_STEP1->STEP_set_target_as(QH_QD[0].theta1);
				_STEP2->STEP_set_target_as(QH_QD[0].theta2);
				_STEP3->STEP_set_target_as(QH_QD[0].theta3);



				bien_co_set_theta =1;

    		}
    		if (bien_co_set_theta==1){
					if ((_STEP1->Status_Step==STEP_DONE)&&(_STEP2->Status_Step==STEP_DONE)&&(_STEP3->Status_Step==STEP_DONE)){

						HUT(1);
						_STEP1->STEP_set_target_as(Angle_4_theta->theta1);
						//_STEP1->STEP_set_target_as(200);
						_STEP2->STEP_set_target_as(Angle_4_theta->theta2);
						_STEP3->STEP_set_target_as(Angle_4_theta->theta3);

						bien_co_set_theta=2;

				}
    		}

			if (bien_co_set_theta==2){
				if ((_STEP1->Status_Step==STEP_DONE)&&(_STEP2->Status_Step==STEP_DONE)&&(_STEP3->Status_Step==STEP_DONE)){

					osDelay(1000);

					_STEP1->STEP_set_target_as(QH_QD[0].theta1);
					_STEP2->STEP_set_target_as(QH_QD[0].theta2);
					_STEP3->STEP_set_target_as(QH_QD[0].theta3);

					bien_co_set_theta=3;

				}
			}
			if (bien_co_set_theta==3){
				if ((_STEP1->Status_Step==STEP_DONE)&&(_STEP2->Status_Step==STEP_DONE)&&(_STEP3->Status_Step==STEP_DONE)){

//
						_STEP1->STEP_set_target_as(QH_QD[1].theta1);
						_STEP2->STEP_set_target_as(QH_QD[1].theta2);
						_STEP3->STEP_set_target_as(QH_QD[1].theta3);

						bien_co_set_theta=4;

				}
			}
    		    if(bien_co_set_theta==4){
					if ((_STEP1->Status_Step==STEP_DONE)&&(_STEP2->Status_Step==STEP_DONE)&&(_STEP3->Status_Step==STEP_DONE)){

//						osDelay(30);
						_STEP1->STEP_set_target_as(-1332);
						_STEP2->STEP_set_target_as(5249);
						_STEP3->STEP_set_target_as(-3890);


						bien_co_set_theta=5;
							}
    		    		}
				if(bien_co_set_theta==5){
					if ((_STEP1->Status_Step==STEP_DONE)&&(_STEP2->Status_Step==STEP_DONE)&&(_STEP3->Status_Step==STEP_DONE)){
						HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);
						bien_co_set_theta=6;
					}

				}

				if(bien_co_set_theta==6){

						Angle_4_theta=nullptr;
						bien_co_set_theta=0;
						yui=10;
						osSemaphoreRelease(Theta_xyzHandle);


				}

    		}






        osDelay(100);
    }
}




void DwinUsartTask_RTOS(){
	uint8_t req;
	for(;;)
	{
	    osMessageQueueGet(Queue_DWINHandle, &req, NULL, osWaitForever);

	    osSemaphoreAcquire(Seme_Tx_dwinHandle, osWaitForever); // <-- chặn trước khi gửi

	    if ((req != 255)){
	        uint16_t add=0x1000+(uint16_t)req*4;
	        Update_Theta_Robot(req);
	        osSemaphoreAcquire(Theta_xyzHandle, osWaitForever);
	        _Dwin->setVP(add,0); // gọi DMA sau khi đã "chiếm" semaphore
	    }
	    else {
	        _Dwin->beepHMI();
	        _Robot_state=ROBOT_SETHOME;
	        Set_dir_sethome=false;


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

        osDelay(1000);
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

int32_t bien_init1[3];
int32_t gttt[3];
int32_t bien_init[3];
int32_t bien_init3[3];
void Control_motor_RTOS(){
	for (;;)
    {
		Home_Update_All();  //UPDATE DEN BÁO SETHOME
		gttt[0]=_STEP1->STEPx.angle_as56_cur;
		gttt[1]=_STEP2->STEPx.angle_as56_cur;
		gttt[2]=_STEP3->STEPx.angle_as56_cur;



		if (_Robot_state==ROBOT_SETHOME){


				////MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
				if (_Flag.flag1) _STEP1->STEP_set_home_trigger(); //NEU CO QUAY THEM SE THEM 1 LAN TU CABLI
				if (_Flag.flag2) _STEP2->STEP_set_home_trigger();
				if (_Flag.flag3) _STEP3->STEP_set_home_trigger();

				if ((_Flag.flag1)&&(_Flag.flag2)&&(_Flag.flag3)) {_Robot_state=ROBOT_IDLE;Flag_cho_set_home=false; }//DUNG IM

				//MUON QUAY VE SET HOME PHAI UPDATE THEM SET DIR SETHOME
				if((!Set_dir_sethome)){
					if(!_Flag.flag1)  {HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15,GPIO_PIN_RESET);}
					if(!_Flag.flag2)  {HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,GPIO_PIN_RESET);}
					if(!_Flag.flag3)  {HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3,GPIO_PIN_SET);}
					//_Flag.flag4_bd=false;
					Set_dir_sethome=true;
					_STEP1->is_enable_step=true;_STEP1->Status_Step=STEP_SETHOME;
					_STEP2->is_enable_step=true;_STEP2->Status_Step=STEP_SETHOME;
					_STEP3->is_enable_step=true;_STEP3->Status_Step=STEP_SETHOME;

					Flag_cho_set_home=true; //bien trang thai isr

				}


				if ((_Flag.flag1)&&(_Flag.flag2)&&(_Flag.flag3)&&(Set_dir_sethome)){
					//_Flag.flag4_bd=true;
					Flag_cho_set_home=false;
				}
				//Sethome_link4(&htim3,&htim4);
		}

		else if (_Robot_state==ROBOT_ACTIVE){

			_STEP1->STEP_CLOSEDLOOP();

			_STEP2->STEP_CLOSEDLOOP();

			_STEP3->STEP_CLOSEDLOOP();




		}

		 osDelay(5);

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



	// SET CHIEU CHO STEP /////////////////////////
	_STEP1->STEP_set_dir_as_step(2885,true,false);
	_STEP2->STEP_set_dir_as_step(1800,true,true);
	_STEP3->STEP_set_dir_as_step(3000,false,false);

	//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM



	Home_Update_All();

	_STEP1->setStepPeriod(50);
	_STEP2->setStepPeriod(50);
	_STEP3->setStepPeriod(50);
	//drv8871
//	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
//	HAL_TIM_PWM_Start (&htim4,TIM_CHANNEL_4);


	//DE TIEN HANH SET HOME CAN TAC DONG
	_Robot_state=ROBOT_SETHOME;
	Set_dir_sethome=false;
	//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
	//HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);
	HUT(0);

	return 0;
}


