/*
 * Robot_xyz.h
 *
 *  Created on: Dec 19, 2025
 *      Author: CHINH
 */

#ifndef ROBOT_XYZ_H_
#define ROBOT_XYZ_H_


#ifdef __cplusplus
extern "C"
{
#endif

#include "stdint.h"
#include "stm32f4xx_hal.h"


/* ================== CONSTANT ================== */
#define ROBOT_MAX_BIT   37
#define POINT_QHQD 7

/* ================== TYPE DEFINITIONS ================== */



/* 4 góc cho robot */
typedef struct
{
    int16_t theta1;
    int16_t theta2;
    int16_t theta3;
    int16_t theta4;
} RobotTheta_t;



/* ================== EXTERN VARIABLES ================== */


extern RobotTheta_t thetaTable[ROBOT_MAX_BIT];
extern RobotTheta_t QH_QD[POINT_QHQD];
/* ================== FUNCTION PROTOTYPES ================== */

RobotTheta_t* Robot_GetTheta(uint8_t index);


#ifdef __cplusplus
}
#endif


#endif /* ROBOT_XYZ_H_ */
