/*
 * Angle_Calculate.h
 *
 *  Created on: 2022��1��14��
 *      Author: xqx
 */

#ifndef USER_ANGLE_CALCULATE_H_
#define USER_ANGLE_CALCULATE_H_

//���ٶȺͽ��ٶȻ�ȡ
float Accel_x(float mpu_acc_x);
float Accel_y(float mpu_acc_y);
float Accel_z(float mpu_acc_z);
float Gyro_x(float mpu_gyro_x);
float Gyro_y(float mpu_gyro_y);
float Gyro_z(float mpu_gyro_z);

//�Ƕȼ���
void Angle_Yaw(void);
void Angle_Pitch(void);
void Angle_Roll(void);

//�������˲�
float Kalman_filter_Pitch(float Gyro ,float Angle_See);
float Kalman_filter_Roll(float Gyro_1 ,float Angle_See_1);
float Kalman_filter_Yaw(float Gyro_2 ,float Angle_See_2);

#endif /* USER_ANGLE_CALCULATE_H_ */
