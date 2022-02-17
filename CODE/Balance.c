/*
 * Balance.c
 *
 *  Created on: 2022��2��4��
 *      Author: xqx
 */
#include "Balance.h"

int Start_Flag=0;

/*����������*/
extern int count_r;
extern int count_l;

/*mpu6050����*/
extern int16 mpu_gyro_x;
extern float Roll;

/*PIDx����Ʋ���*/
float X_Balance_KP = 700.001;
float X_Balance_KI = 30.001;//3.0001;
float X_Balance_KD = 3.001;//30.001;

/*PI�����ֿ��Ʋ���*/
float X_Speed_KP = 90;
float X_Speed_KI = 90;

/*���ƽ����Ʋ���*/
float Zero_error;
float Integration;
int Servo_PWM;
#define Servo_Center_Mid  1600
int PWMServo;
float Balance_Kp = 32.0;
float Balance_Ki = 0.03;
float Balance_Kd = 0.002;

/*����ƽ���е���*/
float Roll_Zero =0.0;

/*PWM�м���*/
int PWM_X;
int PWM_Accel;

void Balance(void)
{
    Angle_Roll();

    //--------------------�����ֿ���----------------------//
    PWM_X = X_Balance_Control(Roll,Roll_Zero,Gyro_x((float)mpu_gyro_x));
    if(PWM_X>8000) PWM_X=8000;        // �����ֵ���޷�
    else if(PWM_X<-8000) PWM_X=-8000; // �����ֵ���޷�
    if(PWM_X < 0 && PWM_X > -1000) PWM_X = -800;
    if(PWM_X > 0 && PWM_X < +1000) PWM_X = +800;
    //if((PWM_X<1000)&&(PWM_X>-1000)) PWM_X=0;
    if(Roll < -10 | Roll > +10 ) PWM_X = 0;

    if(Roll > 0)
    {
        pwm_duty(ATOM3_CH5_P11_10,PWM_X);
        pwm_duty(ATOM2_CH3_P11_6,0);
    }
    else
    {
        pwm_duty(ATOM3_CH5_P11_10,(-PWM_X));
        pwm_duty(ATOM2_CH3_P11_6,PWM_X);
    }

    //---------------------�������-----------------------//
    Servo_PWM = SBB_Get_BalancePID(Roll,Gyro_x((float)mpu_gyro_x));
    if(Servo_PWM > +400) Servo_PWM = +400;
    if(Servo_PWM < -400) Servo_PWM = -400;
    PWMServo = Servo_Center_Mid + Servo_PWM;
    pwm_duty(ATOM3_CH4_P14_1,PWMServo);
}

//x��PIDƽ�����
float X_Balance_Control(float Angle,float Angle_Zero,float Gyro)
{
    float PWM,Bias;
    static float error;
    Bias = Angle - Angle_Zero;
    error = error + Bias;
    if(error > +30) error = +30;
    if(error < -30) error = -30;
    PWM = X_Balance_KP * Bias + X_Balance_KI * error + Gyro * X_Balance_KD;
    return PWM;
}

//�������ٶ�PI����
float Speed_Control(float encoder)
{
    static float Encoder,Encoder_Integral;
    float Speed,Encoder_Least;

    Encoder_Least = encoder;
    Encoder = Encoder * 0.7;
    Encoder = Encoder + Encoder_Least * 0.3;
    Encoder_Integral = Encoder_Integral + Encoder;
    if(Encoder_Integral > +2000) Encoder_Integral = +2000;
    if(Encoder_Integral < -2000) Encoder_Integral = -2000;
    Speed = Encoder * X_Speed_KP + Encoder_Integral * X_Speed_KI/100;
    if(Start_Flag==0) Encoder_Integral=0,Encoder=0,Speed=0;
    return Speed;
}

int SBB_Get_BalancePID(float Angle,float Gyro)
{
    float  Bias;
    int SBB_BalancePID;
    Bias = Angle - Zero_error;     // ���ƽ��ĽǶ���ֵ�ʹ�ʱ����ǵ�ƫ��
    Integration += Bias;           // ����
    if(Integration<-380)      Integration=-380; //�޷�
    else if(Integration>380)  Integration= 380; //�޷�
    //===����ƽ����ƵĶ��PWM  PID���� kp��Pϵ�� kiʽIϵ�� kd��Dϵ��
    SBB_BalancePID = Balance_Kp * Bias + Balance_Ki*Integration + Balance_Kd*Gyro;
    return SBB_BalancePID;
}
