/*
 * oled_iic.h
 *
 *  Created on: 2022��2��6��
 *      Author: xqx
 */

#ifndef CODE_OLED_IIC_H_
#define CODE_OLED_IIC_H_

#include "iic.h"
#include "zf_stm_systick.h"

#define low 0
//#define Brightness  0xCF
#define X_WIDTH     128
#define Y_WIDTH     64

/*OLED IICͨѶ��غ궨��*/
#define OLED_CMD                0       //д����
#define OLED_DATA               1       //д����
#define IIC_ADD_WC              0x78    //IICͨѶд����ʱ�ĵ�ַ
#define IIC_ADD_WD              0x78    //IICͨѶд����ʱ�ĵ�ַ
#define IIC_CONC                0x00    //�����ֽ�ָ����һ���ֽ�Ϊ����
#define IIC_COND                0x40    //�����ֽ�ָ����һ���ֽ�Ϊ����SS

/*SSD1306 ���������*/
#define SET_CONTRAST_CONTROL                0x81    //�Աȶ��������ȡֵ��Χ1-255����λֵ��0x7F
#define SET_DISPLAY_ON              0xAF    //����ʾ����
#define SET_DISPLAY_OFF             0xAE    //����ʾ����
#define ENTIRE_DISPLAY_ON_A4                0xA4    //ȫ����ʾ�ر�
#define ENTIRE_DISPLAY_ON_A5                0xA5    //ȫ����ʾ����
#define NORMAL_DISPLAY              0xA6    //��GDDRAMֵΪ0��Ϩ�����أ���GDDRAMֵΪ1����������
#define INVERSE_DISPLAY             0xA7    //��GDDRAMֵΪ0���������أ���GDDRAMֵΪ1��Ϩ������

void OLED_WR_Byte(uint8 dat,uint8 cmd);
void OLED_Clear(void);
void OLED_Init(void);
void OLED_Draw_Point(uint8 x,uint8 y,uint8 c);
void OLED_Fill(void);
void OLED_Display_Off(void);
void OLED_Display_On(void);
void OLED_SetCursorAddrese(uint8 x,uint8 y);
void OLED_PageClear(uint8 page);
void OLED_ShowStr(unsigned char x, unsigned char y, unsigned char ch[], unsigned char TextSize);
void OLED_ShowNum(unsigned char x, unsigned char y, int Num);

#endif /* CODE_OLED_IIC_H_ */
