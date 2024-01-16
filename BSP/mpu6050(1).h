#ifndef __MPU6050_H_
#define __MPU6050_H_

#include "stdint.h"
#include "filter.h"

#define 	DEV_ADDR			0xD0

#define 	Kp 					2.0f//2
#define 	Ki 					0.2f//0.2
#define  	T 					0.003f
#define 	Rad					(180.f/3.14f)
#define		PI					3.1416f

/* ���������� */
typedef struct 
{
	float gyro[3];					//���ٶ�
	float accel[3];					//���ٶ�
	LowPassFilter acc_filter[3];	//���ٶȵ�ͨ�˲�
	float quat[4];					//��Ԫ��
	int temp;						//�¶�
}MPU6050_DATA;

/* ������У׼���� */
typedef struct 
{
	float gyro_offset[3];			//���ٶȲ���
	float accel_offset[3];			//���ٶȲ���
	unsigned int count;				//У׼�ƴ�
}MPU6050_CALI;

/* �����Ƕ�ֵ */
typedef struct
{
	float roll, yaw, pitch; 
}MPU6050_ANGLE;

/* �������ٶ�ֵ */
typedef struct
{
	float roll, yaw, pitch; 
}MPU6050_SPEED;

extern MPU6050_ANGLE angle;	
void MPU6050_Write(uint8_t reg, uint8_t data);
uint8_t MPU6050_Read(uint8_t reg, uint8_t num);
void MPU6050_Init(void);
void MPU6050_DataUpdate(void);

#endif
