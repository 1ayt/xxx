/*
 * encoder.c
 *
 *  Created on: 2022年1月15日
 *      Author: xqx
 */

#include "encoder.h"

void encoder_init(void)
{
    eru_init(ERU_CH5_REQ1_P15_8,RISING);            //初始化eru中断用于读取编码器脉冲数
    gpio_init(P15_7, GPI, 0, NO_PULL);             //初始化P00_12引脚用于读取编码器旋转方向
    eru_enable_interrupt(ERU_CH5_REQ1_P15_8);       //使能eru中断

    pit_init(CCU6_0,PIT_CH0,100000);                //初始化pit中断用于处理编码器数值
    pit_enable_interrupt(CCU6_0,PIT_CH0);           //使能pit中断
    pit_start(CCU6_0,PIT_CH0);                      //pit中断开始
}



void encoder_read_right(int* a,int count_right)
{
    *a = count_right;
    count_right = 0;
}


void encoder_read_left(int* a,int count_left)
{
    *a = count_left;
    count_left = 0;
}

void encoder_count_reset(int* count)
{
    *count = 0;
}
