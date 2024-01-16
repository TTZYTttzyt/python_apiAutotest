#include "main.h"
#include "i2c.h"
#include "math.h"
#include "mpu6050.h"

/* 选择是否校准 */
#define MPU6050_DO_CALI
//#define MPU6050_NO_CALI

volatile unsigned int cali_finish_flag = 0;		//校准完成标志位
MPU6050_DATA data;								//MPU6050数据
MPU6050_CALI cali;								//MPU6050校准参数
MPU6050_ANGLE angle;							//解算后的角度值
MPU6050_SPEED speed;							//解算后的速度值

/**
  * @brief  向MPU6050写入数据
  * @param  reg：寄存器号
  * @param  num：写入字节数
  * @retval 无
  * @attention 无
  */
void MPU6050_Write(uint8_t reg, uint8_t data)
{
	HAL_I2C_Mem_Write(&hi2c1, DEV_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 0xFF);
}

/**
  * @brief  从MPU6050读取数据
  * @param  reg：寄存器号
  * @param  num：读取字节数
  * @retval 读取值
  * @attention 无
  */
uint8_t MPU6050_Read(uint8_t reg, uint8_t num)
{
	uint8_t data = 0;
	HAL_I2C_Mem_Read(&hi2c1, DEV_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, num, 0xFF);
	return data;
}

/**
  * @brief  MPU6050初始化
  * @param  无
  * @retval 无
  * @attention 无
  */
void MPU6050_Init(void)
{
	HAL_Delay(1000);
 
	/* 解除休眠 */
	MPU6050_Write(0x6B, 0X00);
 
	/* 陀螺仪采样率 = 1kHz / (1 + 采样率分频) = 100Hz */
	MPU6050_Write(0x19, 0x09);
 
	/* 采样频率fs = 100Hz，带宽 = fs/2 = 50Hz */
	MPU6050_Write(0x1A, 0X03);
 
	/* 配置加速度计量程范围（最小计量范围）：±2g */
	MPU6050_Write(0x1C, 0x00);
 
	/* 配置陀螺仪量程范围（最小计量范围）：±250°/s */
	MPU6050_Write(0x1B, 0X00);
	
	/* 四元数初始化 */
	data.quat[0] = 1;
	
	/* 初始化加速度计低通滤波 */
	Filter_LowPassInit(&data.acc_filter[0], 0.2);
	Filter_LowPassInit(&data.acc_filter[1], 0.2);
	Filter_LowPassInit(&data.acc_filter[2], 0.2);
}

/**
  * @brief  MPU6050校准
  * @param  无
  * @retval 无
  * @attention 需静置于水平面，注意陀螺仪安装姿态
  */
void MPU6050_Cali(void)
{
	/* 统计前500个数据，得出偏移量，统计完毕后将校准标志位置1 */
	if(cali.count < 200)
	{
		cali.gyro_offset[0] += data.gyro[0];
		cali.gyro_offset[1] += data.gyro[1];
		cali.gyro_offset[2] += data.gyro[2];
		
		cali.accel_offset[0] += data.accel[0];
		cali.accel_offset[1] += data.accel[1];
		cali.accel_offset[2] += data.accel[2] - 9.8f;
		
		cali.count++;
	}
	else if(cali.count == 200)
	{
		cali_finish_flag = 1;
	}
	
	/* 若校准标志位已置1，则计算平均值，将标志位置2，并点亮LED，结束校准程序 */
	if(cali_finish_flag == 1)
	{
		cali.gyro_offset[0] /= (float)cali.count;
		cali.gyro_offset[1] /= (float)cali.count;
		cali.gyro_offset[2] /= (float)cali.count;
		
		cali.accel_offset[0] /= (float)cali.count;
		cali.accel_offset[1] /= (float)cali.count;
		cali.accel_offset[2] /= (float)cali.count;
		
		cali_finish_flag = 2;
		cali.count++;
		
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	}
}

/**
  * @brief  四元数计算
  * @param  无
  * @retval 无
  * @attention 无
  */
void MPU6050_QuatCalc(void)
{
	double norm = 0;
	double vx = 0, vy = 0, vz = 0;
	double ex = 0, ey = 0, ez = 0;
	double exInt = 0, eyInt = 0, ezInt = 0;
 
	/* 估计重力分量 */
	vx = 2.0 * (data.quat[1] * data.quat[3] - data.quat[0] * data.quat[2]);
	vy = 2.0 * (data.quat[0] * data.quat[1] + data.quat[2] * data.quat[3]);
	vz = data.quat[0] * data.quat[0] - data.quat[1] * data.quat[1] - data.quat[2] * data.quat[2] + data.quat[3] * data.quat[3];
 
	/* 利用叉积计算估计重力和实际重力之间的误差 */
	ex = (data.accel[1] * vz - data.accel[2] * vy);
	ey = (data.accel[2] * vx - data.accel[0] * vz);
	ez = (data.accel[0] * vy - data.accel[1] * vx);
 
	/* 将重力误差进行积分运算 */
	exInt = exInt + ex * Ki;
	eyInt = eyInt + ey * Ki;
	ezInt = ezInt + ez * Ki;
 
	/* 修正陀螺仪数据 */
	data.gyro[0] = data.gyro[0] + Kp * ex + exInt;
	data.gyro[1] = data.gyro[1] + Kp * ey + eyInt;
	data.gyro[2] = data.gyro[2] + Kp * ez + ezInt;
	
	/* 将四元数进行微分运算 */
	data.quat[0] = data.quat[0] + (-data.quat[1] * data.gyro[0] - data.quat[2] * data.gyro[1] - data.quat[3] * data.gyro[2]) * T * 0.5f;
	data.quat[1] = data.quat[1] + (data.quat[0] * data.gyro[0] + data.quat[2] * data.gyro[2] - data.quat[3] * data.gyro[1]) * T * 0.5f;
	data.quat[2] = data.quat[2] + (data.quat[0] * data.gyro[1] - data.quat[1] * data.gyro[2] + data.quat[3] * data.gyro[0]) * T * 0.5f;
	data.quat[3] = data.quat[3] + (data.quat[0] * data.gyro[2] + data.quat[1] * data.gyro[1] - data.quat[2] * data.gyro[0]) * T * 0.5f;
	
	/* 将四元数归一化处理 */
	norm = sqrt(data.quat[0] * data.quat[0] + data.quat[1] * data.quat[1] + data.quat[2] * data.quat[2] + data.quat[3] * data.quat[3]);
	data.quat[0] = data.quat[0] / norm;
	data.quat[1] = data.quat[1] / norm;
	data.quat[2] = data.quat[2] / norm;
	data.quat[3] = data.quat[3] / norm;
	
	/* 计算欧拉角 */
	angle.roll = atan2(2 * data.quat[2] * data.quat[3] + 2* data.quat[0]*data.quat[1],	\
							 -2 * data.quat[1] * data.quat[1]- 2 * data.quat[2] * data.quat[2] + 1) * Rad;
	angle.yaw = atan2(2 * data.quat[1] * data.quat[2] + 2 * data.quat[0]*data.quat[3],	\
							 -2 * data.quat[2] * data.quat[2] - 2 * data.quat[3] * data.quat[3] + 1) * Rad;
    angle.pitch = asin(-2 * data.quat[1] * data.quat[3] + 2 * data.quat[0] * data.quat[2]) * Rad;
	
	/* 计算角速度 */
	speed.roll = data.gyro[0] / PI * 180.f;
	speed.yaw = data.gyro[2] / PI * 180.f;
	speed.pitch = data.gyro[1] / PI * 180.f;
	/*计算加速度*/
	
}

/**
  * @brief  更新陀螺仪数值
  * @param  无
  * @retval 无
  * @attention 角度、角速度、温度
  */
void MPU6050_DataUpdate(void)
{	
	/* 读取陀螺仪原始值 */
	data.gyro[0] = (int16_t)((MPU6050_Read(0x43, 1) << 8) | (MPU6050_Read(0x44, 1))) / (131.068 * 57.30);
	data.gyro[1] = (int16_t)((MPU6050_Read(0x45, 1) << 8) | (MPU6050_Read(0x46, 1))) / (131.068 * 57.30);
	data.gyro[2] = (int16_t)((MPU6050_Read(0x47, 1) << 8) | (MPU6050_Read(0x48, 1))) / (131.068 * 57.30);
	data.temp = (int16_t)((MPU6050_Read(0x41, 1) << 8) | (MPU6050_Read(0x42, 1))) / 340 + 36.53;
	data.accel[0] = (int16_t)((MPU6050_Read(0x3B, 1) << 8) | (MPU6050_Read(0x3C, 1))) / 16384.f * 9.8f;
	data.accel[1] = (int16_t)((MPU6050_Read(0x3D, 1) << 8) | (MPU6050_Read(0x3E, 1))) / 16384.f * 9.8f;
	data.accel[2] = (int16_t)((MPU6050_Read(0x3F, 1) << 8) | (MPU6050_Read(0x40, 1))) / 16384.f * 9.8f;
	
	/* 若校准，则执行校准程序，计算补偿值 */
	#ifdef MPU6050_DO_CALI
		MPU6050_Cali();
		
		/* 待校准已完成，对原始值进行补偿，计算角度值 */
		if(cali_finish_flag)
		{
			data.gyro[0] -= cali.gyro_offset[0];
			data.gyro[1] -= cali.gyro_offset[1];
			data.gyro[2] -= cali.gyro_offset[2];
			
//			data.accel[0] -= cali.accel_offset[0];
//			data.accel[1] -= cali.accel_offset[1];
//			data.accel[2] -= cali.accel_offset[2];
			
			data.accel[0] = Filter_LowPass(&data.acc_filter[0], data.accel[0]);
			data.accel[1] = Filter_LowPass(&data.acc_filter[1], data.accel[1]);
			data.accel[2] = Filter_LowPass(&data.acc_filter[2], data.accel[2]);
		
			MPU6050_QuatCalc();
		}

	#endif
	
	/* 若不校准，则直接使用读取值*/
	#ifdef MPU6050_NO_CALI
		
		/* 计算角度值 */
		MPU6050_QuatCalc();
	#endif
		
}
