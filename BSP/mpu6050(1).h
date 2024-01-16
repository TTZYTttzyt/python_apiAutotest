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

/* 陀螺仪数据 */
typedef struct 
{
	float gyro[3];					//角速度
	float accel[3];					//加速度
	LowPassFilter acc_filter[3];	//加速度低通滤波
	float quat[4];					//四元数
	int temp;						//温度
}MPU6050_DATA;

/* 陀螺仪校准数据 */
typedef struct 
{
	float gyro_offset[3];			//角速度补偿
	float accel_offset[3];			//加速度补偿
	unsigned int count;				//校准计次
}MPU6050_CALI;

/* 解算后角度值 */
typedef struct
{
	float roll, yaw, pitch; 
}MPU6050_ANGLE;

/* 解算后角速度值 */
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
