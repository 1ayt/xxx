/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2020,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ����Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		main
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		ADS v1.2.2
 * @Target core		TC264D
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2020-3-23
 ********************************************************************************************************************/

#include "headfile.h"
#include "Angle_Calculate.h"
#include "oled_iic.h"
#include "Motor_A.h"
#include "Balance.h"
#include "encoder.h"
#include "image.h"
#include "PIT_Init.h"
#include "SEEKFREE_MT9V03X.h"
#include "SEEKFREE_MPU6050.h"
#pragma section all "cpu0_dsram"

//��̬�ǲ���
extern float Roll;

//ͼ������(����)
uint8 data[120][188];
uint8 img[120*188];
int i,j,k=0,m=0;

int core0_main(void)
{
	disableInterrupts();
    get_clk();//��ȡʱ��Ƶ��  ��ر���

    //�û��ڴ˴����ø��ֳ�ʼ������
    PIT_Init();
    mpu6050_init();
    encoder_init();
    Motor_A_Init();
    OLED_Init();
    mt9v03x_init();

	enableInterrupts();                             //ʹ��ȫ���ж�

	//OLED��ʾ��������
	OLED_ShowStr(0,0,(unsigned char*)"Roll :",2);
	OLED_ShowStr(0,2,(unsigned char*)"PWM1 :",2);
	OLED_ShowStr(0,4,(unsigned char*)"PWM2 :",2);
	OLED_ShowStr(0,6,(unsigned char*)"Speed :",2);

    while (TRUE)
    {

        OLED_ShowNum(56,0,Roll);
        systick_delay_ms(STM0,50);
        //Send_Gray_image(mt9v03x_image,188,120);

    }
}

#pragma section all restore
