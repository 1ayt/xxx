/*
 * Motor_A.c
 *
 *  Created on: 2022��2��4��
 *      Author: xx
 */

#include "Motor_A.h"

void Motor_A_Init(void)
{
    //������ATOM3_CH7_P13_2
    gtm_pwm_init(ATOM3_CH5_P11_10,125,0);         //��ʼ��pwm���ڿ��Ƶ��ת��
    gtm_pwm_init(ATOM2_CH3_P11_6,125,0);

    //���PWM
    gpio_init(P00_9, GPO, 0, PUSHPULL);
    gtm_pwm_init(ATOM3_CH7_P13_2,125,0);

    //���PWM
    gtm_pwm_init(ATOM3_CH4_P14_1,125,1600);
}

