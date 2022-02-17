/*
 * iic.c
 *
 *  Created on: 2022��2��6��
 *      Author: xqx
 */
#include "iic.h"
#include "zf_gpio.h"
#include "zf_stm_systick.h"

#define SDA_vice             gpio_get (AYT_SDA)
#define SDA0_vice()          gpio_set (AYT_SDA, 0)      //IO������͵�ƽ
#define SDA1_vice()          gpio_set (AYT_SDA, 1)      //IO������ߵ�ƽ
#define SCL0_vice()          gpio_set (AYT_SCL, 0)      //IO������͵�ƽ
#define SCL1_vice()          gpio_set (AYT_SCL, 1)      //IO������ߵ�ƽ
#define DIR_OUT_vice()       gpio_dir (AYT_SDA, GPO, PUSHPULL)   //�������
#define DIR_IN_vice()        gpio_dir (AYT_SDA, GPI, NO_PULL)    //���뷽��

uint16 iic_delay_time = AYT_DELAY_TIME;   //ICM�ȴ�����Ӧ����Ϊ20


//-------------------------------------------------------------------------------------------------------------------
//  @brief      ģ��IIC��ʱ ʱ������
//  @return     void
//  @since      v1.0
//  Sample usage:               ���IICͨѶʧ�ܿ��Գ�������simiic_delay_time��ֵ
//-------------------------------------------------------------------------------------------------------------------
void iic_delay_set(uint16 time)
{
    iic_delay_time = time;           //ICM�ȴ�����Ӧ����Ϊ100
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ģ��IIC��ʱ
//  @return     void
//  @since      v1.0
//  Sample usage:               ���IICͨѶʧ�ܿ��Գ�������simiic_delay_time��ֵ
//-------------------------------------------------------------------------------------------------------------------
void iic_delay(void)
{
    uint16 delay_time;
    delay_time = iic_delay_time;
    while(delay_time--);
}

//�ڲ�ʹ�ã��û��������
void iic_start(void)
{
    SDA1_vice();
    SCL1_vice();
    iic_delay();
    SDA0_vice();
    iic_delay();
    SCL0_vice();
}

//�ڲ�ʹ�ã��û��������
void iic_stop(void)
{
    SDA0_vice();
    SCL0_vice();
    iic_delay();
    SCL1_vice();
    iic_delay();
    SDA1_vice();
    iic_delay();
}

int iic_waitack(void)
{
    SCL0_vice();
    DIR_IN_vice();
    iic_delay();

    SCL1_vice();
    iic_delay();

    if(SDA_vice)           //Ӧ��Ϊ�ߵ�ƽ���쳣��ͨ��ʧ��
    {
        DIR_OUT_vice();
        SCL0_vice();
        return 0;
    }
    DIR_OUT_vice();
    SCL0_vice();
    iic_delay();
    return 1;
}

//�ֽڷ��ͳ���
//����c(����������Ҳ���ǵ�ַ)���������մ�Ӧ��
//�����Ǵ�Ӧ��λ
//�ڲ�ʹ�ã��û��������
void iic_send_ch(uint8 c)
{
    uint8 i = 8;
    while(i--)
    {
        if(c & 0x80)    SDA1_vice();//SDA �������
        else            SDA0_vice();
        c <<= 1;
        iic_delay();
        SCL1_vice();                //SCL ���ߣ��ɼ��ź�
        iic_delay();
        SCL0_vice();                //SCL ʱ��������
    }

}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      ģ��IIC�˿ڳ�ʼ��
//  @param      NULL
//  @return     void
//  @since      v1.0
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
void iic_init(void)
{
    gpio_init (AYT_SCL, GPO, 1, PUSHPULL);
    gpio_init (AYT_SDA, GPO, 1, PUSHPULL);
}
