/*
 * Dwin.h
 *
 *  Created on: Dec 4, 2025
 *      Author: CHINH
 */

#ifndef DWIN_H_
#define DWIN_H_


#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32f4xx_hal.h"        // Main HAL
#include "stm32f4xx_hal_uart.h"   // UART HAL
#include "stm32f4xx_hal_dma.h"    // DMA HAL
#include "usart.h"



#define Count_Point 37
#define Add_Send 0x1200

class DwinDgus {
private :
	UART_HandleTypeDef *_huart;
	DMA_HandleTypeDef *_hdma_rx;
	uint8_t RxData[32];
	uint8_t _sendBuffer[8];
	volatile uint64_t pos_state = 0;
	bool Flag_send=false; // Bao hieu khi nao an gui de xu ly du lieu

	//double check lai du lieu nhan ve co dung khong
	uint8_t point_count_old=0;
	uint8_t point_count_new=0;
public:

    DwinDgus(UART_HandleTypeDef *huartx,DMA_HandleTypeDef *hdma_usartx_rx);



    void handled_data_interrupt();

    friend void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
    friend void Dwin_RTOS();
    friend void USART3_IRQ_INIT();
    void setVP(uint16_t address,uint16_t data);
    void setPage(uint8_t Page);
    void beepHMI();
private:
    void CMD_READ_Func();
    void set_pos_from_value(uint16_t add,uint16_t data);
};










#ifdef __cplusplus
}
#endif


#endif /* DWIN_H_ */
