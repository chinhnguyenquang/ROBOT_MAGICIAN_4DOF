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




//////////////////////////////OKKKKK///////////////////////////////////////
void STEP::dir_step(uint8_t dir){this->_dir_port->BSRR = dir ? (1U << this->_dir_pin ) : (1U << (this->_dir_pin + 16));}


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


void STEP::STEP_set_dir_as_step(uint16_t a,bool dir,bool dir_As)
{
	this->dir_Stepx=dir;
	this->dir_AS=dir_As;
	this->STEPx.angle_as56_cal=a;
}


////////MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM


/////////////////// CO THE CHINH SUA UPDATE//////////////////////////////
void STEP::setStepPeriod(uint32_t period) {

    const uint32_t MIN_PERIOD_US = 50; // giới hạn min
    if(period < MIN_PERIOD_US) period = MIN_PERIOD_US;
    step_period_us = period/50;
}
////////MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM

void STEP::update_ISR(){
	if(!is_enable_step) return;

	if((remain == 0)&&(this->Status_Step==STEP_CLOSED)){
		is_enable_step=false;
		return;
	}

    if(step_period_us == 0) return;
    step_timer++;
    if(step_timer >= step_period_us) {
        step_timer = 0;
        step_state ^= 1;
        this->_step_port->BSRR = step_state ? (1U << this->_step_pin) : (1U << (this->_step_pin+16));


        if (!step_state) return;

        if(this->Status_Step==STEP_SETHOME) return;


        //CLOSED
        if(this->Status_Step==STEP_CLOSED){
            	this->remain--;
            	return;
        }

        //OPEN LOOP

        if (this->STEPx.Chieuquayhientai==STEP_DUONG)   this->STEPx.current_step++;
        else if (this->STEPx.Chieuquayhientai==STEP_AM) this->STEPx.current_step--;

        if (this->STEPx.current_step==this->STEPx.target_step) {this->is_enable_step=false;this->Status_Step=STEP_CLOSED;}

    }
}






















/* ===== CẤU HÌNH HỆ ===== */
#define STEP_PER_REV     19200    // step 1.8°, microstep 1/16 vong dai x6
#define ENC_PER_REV      4096    // AS5600


/* ===== FILTER + FEEDBACK ===== */
#define ENC_DEADBAND     3       // rung encoder (microstep)
#define STABLE_COUNT     3       // số lần ổn định liên tiếp
#define FEEDBACK_DEADBAND 5      // deadband feedback
#define MAX_CORRECT_STEP 2       // step bù mỗi lần







//void STEP::STEP_set_Target(uint16_t a){ //a la goc
//
//
//	this->STEPx.angle_kc_candat=this->STEPx.angle_as56_cal-this->STEPx.angle_as56_init; //TH NGUY HIEM KHI 2 DIEM NAY NAM TAI BIEN
//
//	this->Flags.flag_set_dir=false; //set chieu ban dau truoc khi dieu khien
//
//	this->STEPx.current_step=0;
//
//	if (this->dir_AS){
//		this->STEPx.angle_as56_tar=(int32_t)a*6*4096/360 +this->STEPx.angle_kc_candat; //chieu cam bien duong
//	}
//	else {
//		this->STEPx.angle_as56_tar=(-1)*(int32_t)a*6*4096/360 + this->STEPx.angle_kc_candat;
//	}
//
//
//	if(!(this->Flags.flag_trang_thaidau_vehome)){
//		//NGHIA LA HIEN DANG SET HOME CAN + THEM 1 LUONG DE VE 0
//		this->STEPx.angle_cur=0;
//
//		this->STEPx.target_step=(int32_t)a*STEP_PER_REV/360+iabs(this->STEPx.angle_kc_candat)*STEP_PER_REV/ENC_PER_REV/6;
//
//		this->Flags.flag_trang_thaidau_vehome=true;
//
//	}
//	else {
//		//TARGET AM HOAC DUONG
//		this->STEPx.target_step=(int32_t)a*STEP_PER_REV/360 - iabs(this->STEPx.angle_cur)*3200/ENC_PER_REV + iabs(this->STEPx.angle_kc_candat)*3200/ENC_PER_REV;
//
//	}
//	this->Status_Step=STEP_OPEN_LOOP;
//
//}

// encoder → step
#define ENC_TO_STEP(e)   ((int32_t)((int64_t)(e) * 19200 / (4096 * 6)))

void STEP::STEP_set_target_as(int32_t target){


	this->STEPx.angle_kc_candat=this->STEPx.angle_as56_init; //TH NGUY HIEM KHI 2 DIEM NAY NAM TAI BIEN

	this->STEPx.angle_kc_candat=this->STEPx.angle_as56_cal-this->STEPx.angle_as56_init;

	this->STEPx.angle_as56_tar=target;//+this->STEPx.angle_kc_candat;

	this->STEPx.current_step=0;

	int32_t bient_target_step_tmp=target;
	if (target>0){
		bient_target_step_tmp=target-100;
	}
	else {
		bient_target_step_tmp=target+100;
	}

	if(!(this->Flags.flag_trang_thaidau_vehome)){

		this->STEPx.angle_as56_cur=0;
		this->STEPx.angle_cur=0;

		this->STEPx.target_step=(int32_t)(bient_target_step_tmp * 25/32);


		//this->STEPx.target_step=500;
		this->Flags.flag_trang_thaidau_vehome=true;

	}
	else {
		//TARGET AM HOAC DUONG
		this->STEPx.target_step=(int32_t)((bient_target_step_tmp-this->STEPx.angle_cur)*25/32);

		//this->STEPx.target_step=500;
	}
	this->Status_Step=STEP_OPEN_LOOP;
	//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
	this->Flags.flag_set_dir=false;
	this->Flags.flag_set_chieu_openloop=false;
	//MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM

	STEP_OPENLOOP();
}

void STEP::STEP_set_home_trigger(void){
	this->Flags.flag_trang_thaidau_vehome=false;
	this->Flags.flag_angle_init=false;
	this->is_enable_step=false;
	this->STEPx.angle_as56_cur=0;
}






#define FABS(x)        ((x) < 0.0f ? -(x) : (x))

#define STEP_PER_ENC   (25.0f / 32.0f)   // encoder -> step
#define ENC_PER_STEP   (32.0f / 25.0f)   // step -> encoder

/////////////////////////// CLOSED LOOP ///////////////////////////
void STEP::STEP_CLOSEDLOOP()
{
    if (Status_Step != STEP_CLOSED) return;

    int32_t tmp;
    __disable_irq();
    tmp = remain;
    __enable_irq();

    int32_t enc_raw = STEPx.angle_as56_cur;

    // step -> encoder
    tmp = (int32_t)(tmp * ENC_PER_STEP);

    if (dir_AS) STEPx.angle_est = enc_raw + tmp;
    else        STEPx.angle_est = enc_raw - tmp;

    int32_t error_enc = STEPx.angle_as56_tar - STEPx.angle_est;
    int32_t error_ee  = STEPx.angle_as56_tar - enc_raw;

    /* ===== DONE CHECK (ENCODER THẬT) ===== */
    if (iabs(error_ee) < 4) {
        stable_cnt++;
        if (stable_cnt >= 20) {
            Status_Step = STEP_DONE;
            this->STEPx.angle_cur=enc_raw;
            is_enable_step = false;
            remain = 0;
            stable_cnt = 0;
        }
        return;
    } else {
        stable_cnt = 0;
    }

    /* ===== CONTROL ===== */
    float error_step = error_enc * STEP_PER_ENC;

    int32_t step_cmd = 0;
    if (error_step > 1.0f)       step_cmd = 1;
    else if (error_step < -1.0f) step_cmd = -1;
    else return;

    bool dir = this->dir_Stepx;
    if (step_cmd < 0)  dir = !dir;
    if (!this->dir_AS) dir = !dir;

    dir_step(dir);

    __disable_irq();
    remain += iabs(step_cmd);
    __enable_irq();

    is_enable_step = true;
}




void STEP::STEP_OPENLOOP(){
	//OPEN LOOP
	if(!Flags.flag_set_chieu_openloop){
		if(this->Status_Step==STEP_OPEN_LOOP){
			//QUAY THEO CHIEU DUONG THIEU GOC
			if (this->STEPx.current_step < this->STEPx.target_step){
				//THIEU GOC THI QUAY THEO CHIEU DUONG
				if (this->dir_AS){
					if(this->dir_Stepx) dir_step(true);
					else dir_step(false);
				}
				else {
					if(this->dir_Stepx) dir_step(false);
					else dir_step(true);
				}

				this->STEPx.Chieuquayhientai=STEP_DUONG;
				this->is_enable_step=true;

			}
			else if (this->STEPx.current_step > this->STEPx.target_step){

				//DU GOC GOC THI QUAY THEO CHIEU AM
				if (this->dir_AS){
					if(this->dir_Stepx) dir_step(false);
					else dir_step(true);
				}
				else {
					if(this->dir_Stepx) dir_step(true);
					else dir_step(false);
				}

				this->STEPx.Chieuquayhientai=STEP_AM;
				this->is_enable_step=true;

			}
			Flags.flag_set_chieu_openloop=true;
		}
	}

}










