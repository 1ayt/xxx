/*
 * Balance.c
 *
 *  Created on: 2022年2月4日
 *      Author: xqx
 */
#include "Balance.h"

int Start_Flag=0;

/*编码器参数*/
extern int count_r;
extern int count_l;

/*mpu6050参数*/
extern int16 mpu_gyro_x;
extern float Roll;

/*PIDx轴控制参数*/
float X_Balance_KP = 700.001;
float X_Balance_KI = 30.001;//3.0001;
float X_Balance_KD = 3.001;//30.001;

/*PI动量轮控制参数*/
float X_Speed_KP = 90;
float X_Speed_KI = 90;

/*舵机平衡控制参数*/
float Zero_error;
float Integration;
int Servo_PWM;
#define Servo_Center_Mid  1600
int PWMServo;
float Balance_Kp = 32.0;
float Balance_Ki = 0.03;
float Balance_Kd = 0.002;

/*车体平衡机械零点*/
float Roll_Zero =0.0;

/*PWM中间量*/
int PWM_X;
int PWM_Accel;

void Balance(void)
{
    Angle_Roll();

    //--------------------动量轮控制----------------------//
    PWM_X = X_Balance_Control(Roll,Roll_Zero,Gyro_x((float)mpu_gyro_x));
    if(PWM_X>8000) PWM_X=8000;        // 动量轮电机限幅
    else if(PWM_X<-8000) PWM_X=-8000; // 动量轮电机限幅
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

    //---------------------舵机控制-----------------------//
    Servo_PWM = SBB_Get_BalancePID(Roll,Gyro_x((float)mpu_gyro_x));
    if(Servo_PWM > +400) Servo_PWM = +400;
    if(Servo_PWM < -400) Servo_PWM = -400;
    PWMServo = Servo_Center_Mid + Servo_PWM;
    pwm_duty(ATOM3_CH4_P14_1,PWMServo);
}

//x轴PID平衡控制
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

//动量轮速度PI控制
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
    Bias = Angle - Zero_error;     // 求出平衡的角度中值和此时横滚角的偏差
    Integration += Bias;           // 积分
    if(Integration<-380)      Integration=-380; //限幅
    else if(Integration>380)  Integration= 380; //限幅
    //===计算平衡控制的舵机PWM  PID控制 kp是P系数 ki式I系数 kd是D系数
    SBB_BalancePID = Balance_Kp * Bias + Balance_Ki*Integration + Balance_Kd*Gyro;
    return SBB_BalancePID;
}
