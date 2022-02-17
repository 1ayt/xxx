/*
 * Balance.h
 *
 *  Created on: 2022��2��4��
 *      Author: xqx
 */

#ifndef CODE_BALANCE_H_
#define CODE_BALANCE_H_

#include "Angle_Calculate.h"
#include "SEEKFREE_MPU6050.h"
#include "zf_gtm_pwm.h"
#include "zf_gpio.h"

void Balance(void);
float X_Balance_Control(float Angle,float Angle_Zero,float Gyro);
float Speed_Control(float encoder);
int SBB_Get_BalancePID(float Angle,float Gyro);

#endif /* CODE_BALANCE_H_ */
