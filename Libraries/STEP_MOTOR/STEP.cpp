/*
 * STEP.cpp
 *
 *  Created on: Dec 5, 2025
 *      Author: CHINH
 */


#include "STEP.h"


static inline int32_t iabs(int32_t x)
{
    return (x < 0) ? -x : x;
}



void STEP::update_ISR(){
	if(!is_enable_step) return;

    if(this->STEPx.Chieuquayhientai==STEP_CLOSED){
    	if(this->remain==0) return;
    	this->remain--;
    }

    if(step_period_us == 0) return;
    step_timer++;
    if(step_timer >= step_period_us) {
        step_timer = 0;
        step_state ^= 1;
        this->_step_port->BSRR = step_state ? (1U << this->_step_pin) : (1U << (this->_step_pin+16));

        if(this->STEPx.Chieuquayhientai==STEP_SETHOME) return;




        if (this->STEPx.Chieuquayhientai==STEP_DUONG){
        	if(step_state) this->STEPx.current_step++;
        }
        else if (this->STEPx.Chieuquayhientai==STEP_AM){
        	if(step_state) this->STEPx.current_step--;
        }
        if (this->STEPx.current_step==this->STEPx.target_step) {this->is_enable_step=false;this->Status_Step=STEP_RUNNING_2; this->STEPx.Chieuquayhientai=STEP_CLOSED;}

    }
}







void STEP::dir_step(uint8_t dir){this->_dir_port->BSRR = dir ? (1U << this->_dir_pin ) : (1U << (this->_dir_pin + 16));}



void STEP::setStepPeriod(uint32_t period) {

    const uint32_t MIN_PERIOD_US = 50; // giới hạn min
    if(period < MIN_PERIOD_US) period = MIN_PERIOD_US;
    step_period_us = period/50;
}


uint32_t STEP::getPosition() { return this->STEPx.current_step; }


#define AS5600_RESOLUTION 4096
#define AS5600_HALF       2048

void STEP::update_As5600_cur(int32_t value)
{

	if (!(this->Flags.flag_angle_init)){
		this->Flags.flag_angle_init=true;
		this->STEPx.angle_as56_init=value;
		this->STEPx.angle_as56_last=value;
		return;
	}



    int32_t delta = (int32_t)value - (int32_t)this->STEPx.angle_as56_last;

    // xử lý wrap
    if (delta > AS5600_HALF)
        delta -= AS5600_RESOLUTION;
    else if (delta < -AS5600_HALF)
        delta += AS5600_RESOLUTION;

    this->STEPx.angle_as56_cur += delta;
    this->STEPx.angle_as56_last = value;

}





/* ===== CẤU HÌNH HỆ ===== */
#define STEP_PER_REV     19200    // step 1.8°, microstep 1/16 vong dai x6
#define ENC_PER_REV      4096    // AS5600


/* ===== FILTER + FEEDBACK ===== */
#define ENC_DEADBAND     3       // rung encoder (microstep)
#define STABLE_COUNT     3       // số lần ổn định liên tiếp
#define FEEDBACK_DEADBAND 2      // deadband feedback
#define MAX_CORRECT_STEP 2       // step bù mỗi lần



void STEP::STEP_set_cal(uint16_t a,bool dir,bool dir_As)
{
	this->STEPx.angle_as56_cal=a;
	this->dir_Stepx=dir;
	this->dir_AS=dir_As;
}



void STEP::STEP_set_Target(uint8_t a){ //a la goc


	this->STEPx.angle_kc_candat=this->STEPx.angle_as56_cal-this->STEPx.angle_as56_init; //TH NGUY HIEM KHI 2 DIEM NAY NAM TAI BIEN

	this->Flags.flag_set_dir=false; //set chieu ban dau truoc khi dieu khien

	if (this->dir_AS){
		this->STEPx.angle_as56_tar=(int32_t)a*6*4096/360 +this->STEPx.angle_kc_candat; //chieu cam bien duong
	}
	else {
		this->STEPx.angle_as56_tar=(-1)*(int32_t)a*6*4096/360 + this->STEPx.angle_kc_candat;
	}


	if(!(this->Flags.flag_trang_thaidau_vehome)){
		//NGHIA LA HIEN DANG SET HOME CAN + THEM 1 LUONG DE VE 0
		this->STEPx.angle_cur=0;

		this->STEPx.target_step=(int32_t)a*STEP_PER_REV/360+iabs(this->STEPx.angle_kc_candat)*STEP_PER_REV/ENC_PER_REV/6;

		this->Flags.flag_trang_thaidau_vehome=true;

	}
	else {
		//TARGET AM HOAC DUONG
		this->STEPx.target_step=(int32_t)a*STEP_PER_REV/360 - this->STEPx.angle_cur*STEP_PER_REV/ENC_PER_REV;

	}
	this->Status_Step=STEP_RUNNING_1;

}


void STEP::STEP_set_home_trigger(void){
	this->Flags.flag_trang_thaidau_vehome=false;
	this->Flags.flag_angle_init=false;
	this->is_enable_step=false;
}





void STEP::STEP_Process(){
	if(this->Status_Step==STEP_DONE) return;

	//OPEN LOOP
	if(this->Status_Step==STEP_RUNNING_1){
		//QUAY THEO CHIEU DUONG THIEU GOC
		if (this->STEPx.current_step < this->STEPx.target_step){
			if(!(this->Flags.flag_set_dir)){
				//THIEU GOC THI QUAY THEO CHIEU DUONG
				if(this->dir_Stepx) dir_step(true);
				else dir_step(false);
				this->Flags.flag_set_dir=true;
				this->STEPx.Chieuquayhientai=STEP_DUONG;
				this->is_enable_step=true;
			}

		}
		else if (this->STEPx.current_step > this->STEPx.target_step){
			if(!(this->Flags.flag_set_dir)){
				//DU GOC GOC THI QUAY THEO CHIEU AM
				if(this->dir_Stepx) dir_step(false);
				else dir_step(true);
				this->Flags.flag_set_dir=true;
				this->STEPx.Chieuquayhientai=STEP_AM;
				this->is_enable_step=true;
			}
		}
	}
	//DA SET CO STATUS_STEP TRONG ISR
	else if (this->Status_Step==STEP_RUNNING_2){
		this->STEPx.angle_cur=this->STEPx.angle_as56_cur;
		int32_t tmp=this->STEPx.angle_as56_tar-this->STEPx.angle_cur;
		if (iabs(tmp) <7) {
			this->Status_Step=STEP_DONE;
			i=3;
			return;
		}
		//DIR AS LA CHIEU DUONG NEU TMP DUONG NGHIA LA QUAY THIEU
		if (this->dir_AS){
			if(tmp > 0){
				if(this->dir_Stepx) dir_step(true);
				else dir_step(false);

			}
			else {
				if(this->dir_Stepx) dir_step(false);
				else dir_step(true);
			}

			i=2;
		}
		else{
		//DIR AS LA CHIEU AM NEN NEU TMP DUONG  LA QUAY DU
			if(tmp > 0){
				if(this->dir_Stepx) dir_step(false);
				else dir_step(true);
			}
			else {
				if(this->dir_Stepx) dir_step(true);
				else dir_step(false);
			}


			i=1;
		}
		this->is_enable_step=true;
		this->remain=2;


	}
}





void STEP::set_enable_step(bool val){
	this->is_enable_step=val;
}


