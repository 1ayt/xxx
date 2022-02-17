/*
 * Angle_Calculate.c
 *
 *  Created on: 2022��1��14��
 *      Author: xqx
 */

#include "Angle_Calculate.h"
#include "SEEKFREE_MPU6050.h"

//Yaw,Pitch,Roll �Ƕ�ֵ
float Yaw,Pitch,Roll;

//����mpu6050.h�ļ��еı���
extern int16 mpu_gyro_x,mpu_gyro_y,mpu_gyro_z;
extern int16 mpu_acc_x,mpu_acc_y,mpu_acc_z;

/* brief  : ����x��Ǽ��ٶ�
 * Param1 : mpu_acc_x : mpu6050���ص�x��Ǽ��ٶ�ֵ
 */
float Accel_x(float mpu_acc_x)
{
    float Accel_x;

    Accel_x = 8*9.8*(mpu_acc_x)/32768;

    return Accel_x;
}

/* brief  : ����y��Ǽ��ٶ�
 * Param1 : mpu_acc_y : mpu6050���ص�y��Ǽ��ٶ�ֵ
 */
float Accel_y(float mpu_acc_y)
{
    float Accel_y;

    Accel_y = 8*9.8*(mpu_acc_y)/32768;

    return Accel_y;
}

/* brief  : ����z��Ǽ��ٶ�
 * Param1 : mpu_acc_z : mpu6050���ص�z��Ǽ��ٶ�ֵ
 */
float Accel_z(float mpu_acc_z)
{
    float Accel_z;

    Accel_z = 8*9.8*(mpu_acc_z)/32768;

    return Accel_z;
}

/* brief  : ����x����ٶ�
 * Param1 : mpu_gyro_x : mpu6050���ص�x����ٶ�ֵ
 */
float Gyro_x(float mpu_gyro_x)
{
    float Gyro_x;

    Gyro_x = 2000*mpu_gyro_x/32768;

    return Gyro_x;
}

/* brief  : ����y����ٶ�
 * Param1 : mpu_gyro_y : mpu6050���ص�y����ٶ�ֵ
 */
float Gyro_y(float mpu_gyro_y)
{
    float Gyro_y;

    Gyro_y = 2000*mpu_gyro_y/32768;

    return Gyro_y;
}

/* brief  : ����z����ٶ�
 * Param1 : mpu_gyro_z : mpu6050���ص�z����ٶ�ֵ
 */
float Gyro_z(float mpu_gyro_z)
{
    float Gyro_z;

    Gyro_z = 2000*mpu_gyro_z/32768;

    return Gyro_z;
}
/* brief  : ����Yaw�Ƕ�
 * Param1 : mpu_gyro_z : mpu6050���ص�z����ٶ�ֵ
 */



/*--------------------------------------------------------
 * float Kalman_filter_Pitch(float Gyro ,float Angle_See)
 * brief:�������˲�
 * param:
 *-------------------------------------------------------*/


float dt = 0.008;     //����ʱ��

float Angle;              //�Ƕȵ��������ֵ
float Gyro_bias = 0.1;          //���ٶ�ƫ��ֵ

float Cov_angle = 0.01;      //�����ǲ����Ľ��ٶȵ�Э����
float Cov_bias = 0.01;       //������Ư��ֵ��Э����
float Cov_accel = 0.01;       //���ٶȼƲ����ļ��ٶ�Э����

//P����
float P00 = 1.0;
float P01 = 0.0;
float P10 = 0.0;
float P11 = 1.0;

//�������������
float K0;
float K1;

float Angle_err;


//���������ǽ��ٶȺͼ���Ƕ�
float Kalman_filter_Pitch(float Gyro ,float Angle_See)
{
    /*״̬����
     * | Angle     |   | 1   -dt| | Angle     |   | dt|
     * |           | = |        | |           | + |   | Gyro;
     * | Gyro_bias |   | 0     1| | Gyro_bias |   | 0 |
     */
    Angle = Angle - Gyro_bias * dt + Gyro * dt;
    Gyro_bias = Gyro_bias;

    //P�������
    P00 = P00 - P10 * dt + (P11 * dt - P01) * dt + Cov_angle;
    P01 = P01 - P11 * dt;
    P10 = P10 - P11 * dt;
    P11 = P11 + Cov_bias;

    //��⿨�����������
    K0 = P00 / (P00 + Cov_accel);
    K1 = P10 / (P00 + Cov_accel);

    Angle_err = Angle_See - Angle;

    //��������
    Angle = Angle + K0 * Angle_err;
    Gyro_bias = Gyro_bias + K1 * Angle_err;

    //����P����
    P00 = P00 - K0 * P00;
    P01 = P01 - K0 * P01;
    P10 = P10 - K1 * P00;
    P11 = P11 - K1 * P01;

    return Angle;
}

/*Roll�˲�����
 *
 *
 */
float dt_1 = 0.009;     //����ʱ��

float Angle_1;              //�Ƕȵ��������ֵ
float Gyro_bias_1 = 0.1;          //���ٶ�ƫ��ֵ

float Cov_angle_1 = 0.001;      //�����ǲ����Ľ��ٶȵ�Э����
float Cov_bias_1 = 0.001;       //������Ư��ֵ��Э����
float Cov_accel_1 = 1500;       //���ٶȼƲ����ļ��ٶ�Э����

//P����
static float P001 = 1.0;
static float P011 = 0.0;
static float P101 = 0.0;
static float P111 = 1.0;

//�������������
float K0_1;
float K1_1;

float Angle_err_1;

float Kalman_filter_Roll(float Gyro_1 ,float Angle_See_1)
{
    Angle_1 = Angle_1 - Gyro_bias_1 * dt_1 + Gyro_1 * dt;
    Gyro_bias_1 = Gyro_bias_1;

    //P�������
    P001 = P001 - P101 * dt_1 + (P111 * dt_1 - P011) * dt_1 + Cov_angle_1;
    P011 = P011 - P111 * dt_1;
    P101 = P101 - P111 * dt_1;
    P111 = P111 + Cov_bias_1;

    //��⿨�����������
    K0_1 = P001 / (P001 + Cov_accel_1);
    K1_1 = P101 / (P001 + Cov_accel_1);

    Angle_err_1 = Angle_See_1 - Angle_1;

    //��������
    Angle_1 = Angle_1 + K0_1 * Angle_err_1;
    Gyro_bias_1 = Gyro_bias + K1_1 * Angle_err_1;

    //����P����
    P001 = P001 - K0_1 * P001;
    P011 = P011 - K0_1 * P011;
    P101 = P101 - K1_1 * P001;
    P111 = P111 - K1_1 * P011;

    return Angle_1;
}

//Yaw�˲�����
float dt_2 = 0.008;     //����ʱ��

float Angle_2;              //�Ƕȵ��������ֵ
float Gyro_bias_2 = 0.1;          //���ٶ�ƫ��ֵ

float Cov_angle_2 = 0.01;      //�����ǲ����Ľ��ٶȵ�Э����
float Cov_bias_2 = 0.01;       //������Ư��ֵ��Э����
float Cov_accel_2 = 0.01;       //���ٶȼƲ����ļ��ٶ�Э����

//P����
static float P002 = 1.0;
static float P012 = 0.0;
static float P102 = 0.0;
static float P112 = 1.0;

//�������������
float K0_2;
float K1_2;

float Angle_err_2;
float Gyro_calculate; //�˲���Ľ��ٶ�

float Kalman_filter_Yaw(float Gyro_2 ,float Angle_See_2) //���������ǽ��ٶȺͼ���Ƕ�
{
    /*״̬����
     * | Angle     |   | 1   -dt| | Angle     |   | dt|
     * |           | = |        | |           | + |   | Gyro;
     * | Gyro_bias |   | 0     1| | Gyro_bias |   | 0 |
     */
    Angle_2 = Angle_2 - Gyro_bias_2 * dt_2 + Gyro_2 * dt_2;
    Gyro_bias_2 = Gyro_bias_2;

    //P�������
    P002 = P002 - P102 * dt_2 + (P112 * dt_2 - P012) * dt_2 + Cov_angle_2;
    P012 = P012 - P112 * dt_2;
    P102 = P102 - P112 * dt_2;
    P112 = P112 + Cov_bias_2;

    //��⿨�����������
    K0_2 = P002 / (P002 + Cov_accel_2);
    K1_2 = P102 / (P002 + Cov_accel_2);

    Angle_err_2 = Angle_See_2 - Angle_2;

    //��������
    Angle_2 = Angle_2 + K0_2 * Angle_err_2;
    Gyro_bias_2 = Gyro_bias_2 + K1_2 * Angle_err_2;
    Gyro_calculate = Gyro_2 - Gyro_bias_2;

    //����P����
    P002 = P002 - K0_2 * P002;
    P012 = P012 - K0_2 * P012;
    P102 = P102 - K1_2 * P002;
    P112 = P112 - K1_2 * P012;

    return Angle_2;
}

void Angle_Yaw(void)
{
    Yaw = Yaw + (2000*mpu_gyro_z/32768) * 0.008;
}


/* brief  : ����Pitch�Ƕ�
 * Param1 : mpu_acc_z : mpu6050���ص�z��Ǽ��ٶ�ֵ
 * Param2 : mpu_acc_x : mpu6050���ص�x��Ǽ��ٶ�ֵ
 */
void Angle_Pitch(void)
{
    Pitch = atan2(8*9.8*(mpu_acc_z)/32768,8*9.8*(mpu_acc_x)/32768)*180.0/3.14;
}


/* brief  : ����Roll�Ƕ�
 * Param1 : mpu_acc_z : mpu6050���ص�z��Ǽ��ٶ�ֵ
 * param2 : mpu_acc_y : mpu6050���ص�y��Ǽ��ٶ�ֵ
 */
void Angle_Roll(void)
{
    static float x;
    static float y;
    y = 2000*mpu_gyro_x/32768;
    x = atan2(8*9.8*(mpu_acc_y)/32768,8*9.8*(mpu_acc_z)/32768)*180.0/3.14;
    Roll = Kalman_filter_Roll(y,x);
}
