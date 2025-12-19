/*
 * Dwin.cpp
 *
 *  Created on: Dec 4, 2025
 *      Author: CHINH
 */


#include "Dwin.h"


#define CMD_HEAD1           0x5A
#define CMD_HEAD2           0xA5
#define CMD_WRITE           0x82
#define CMD_READ            0x83

DwinDgus::DwinDgus(UART_HandleTypeDef *huartx,DMA_HandleTypeDef *hdma_usartx_rx)
    : _huart(huartx), _hdma_rx(hdma_usartx_rx)
{
	HAL_UARTEx_ReceiveToIdle_DMA(this->_huart,this->RxData, 32);
	__HAL_DMA_DISABLE_IT(this->_hdma_rx,DMA_IT_HT);

}




#define BASE_VALUE 0x1000


void DwinDgus::set_pos_from_value(uint16_t add,uint16_t data)
{
	uint16_t index = (add - 0x1000) >> 2;
	if (index > Count_Point ) return;
	if (data){
		this->pos_state |= (1ULL << index);
	}
	else {
		this->pos_state &= ~(1ULL << index);
	}

}

void DwinDgus::CMD_READ_Func(){
	if(this->RxData[2]==0x06){
		uint16_t Add=this->RxData[4]<<8|this->RxData[5];
		uint16_t Data=this->RxData[8];
		if ((Add >=BASE_VALUE)&&(Add != Add_Send)){
			if (Data) this->point_count_new++;
			else {
				this->point_count_old-=2;
				this->point_count_new--;
			}
			set_pos_from_value(Add,Data);
		}
		else if (Add==Add_Send){
			this->point_count_new++;
			this->Flag_send=true;
		}
	}
}


void DwinDgus::handled_data_interrupt(){
	this->point_count_old++;
	if ((this->RxData[0]==CMD_HEAD1)&&(this->RxData[1]==CMD_HEAD2)){
		switch(this->RxData[3]){
		case CMD_WRITE:
			break;
		case CMD_READ:
			CMD_READ_Func();
			break;
		}
	}
}

void DwinDgus::beepHMI() {
    _sendBuffer[0] = 0x5A;
    _sendBuffer[1] = 0xA5;
    _sendBuffer[2] = 0x05;
    _sendBuffer[3] = 0x82;
    _sendBuffer[4] = 0x00;
    _sendBuffer[5] = 0xA0;
    _sendBuffer[6] = 0x00;
    _sendBuffer[7] = 0x7D;
    HAL_UART_Transmit_DMA(this->_huart, _sendBuffer, 8);

}

void DwinDgus::setVP(uint16_t address, uint16_t data) {
    _sendBuffer[0] = CMD_HEAD1;
    _sendBuffer[1] = CMD_HEAD2;
    _sendBuffer[2] = 0x05;
    _sendBuffer[3] = CMD_WRITE;
    _sendBuffer[4] = (address >> 8) & 0xFF;
    _sendBuffer[5] = address & 0xFF;
    _sendBuffer[6] = (data >> 8) & 0xFF;
    _sendBuffer[7] = data & 0xFF;
    HAL_UART_Transmit_DMA(this->_huart, _sendBuffer, 8);

}
