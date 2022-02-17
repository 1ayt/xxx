/*
 * PIT_Init.c
 *
 *  Created on: 2022��2��16��
 *      Author: xqx
 */
#include "PIT_Init.h"

void PIT_Init(void)
{
    //���ڴ���Balance����
    pit_init(CCU6_1,PIT_CH0,9000);                  //ÿ��9ms��ȡһ������
    pit_enable_interrupt(CCU6_1,PIT_CH0);           //ʹ��pit�ж�
    pit_start(CCU6_1,PIT_CH0);                      //pit�жϿ�ʼ

    //���ڴ���mpu6050����
    pit_init(CCU6_0,PIT_CH1,9000);                  //ÿ��9ms��ȡһ������
    pit_enable_interrupt(CCU6_0,PIT_CH1);           //ʹ��pit�ж�
    pit_start(CCU6_0,PIT_CH1);                      //pit�жϿ�ʼ

}

