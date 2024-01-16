#include "main.h"
#include "i2c.h"
#include "math.h"
#include "mpu6050.h"

/* ѡ���Ƿ�У׼ */
#define MPU6050_DO_CALI
//#define MPU6050_NO_CALI

volatile unsigned int cali_finish_flag = 0;		//У׼��ɱ�־λ
MPU6050_DATA data;								//MPU6050����
MPU6050_CALI cali;								//MPU6050У׼����
MPU6050_ANGLE angle;							//�����ĽǶ�ֵ
MPU6050_SPEED speed;							//�������ٶ�ֵ

/**
  * @brief  ��MPU6050д������
  * @param  reg���Ĵ�����
  * @param  num��д���ֽ���
  * @retval ��
  * @attention ��
  */
void MPU6050_Write(uint8_t reg, uint8_t data)
{
	HAL_I2C_Mem_Write(&hi2c1, DEV_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 0xFF);
}

/**
  * @brief  ��MPU6050��ȡ����
  * @param  reg���Ĵ�����
  * @param  num����ȡ�ֽ���
  * @retval ��ȡֵ
  * @attention ��
  */
uint8_t MPU6050_Read(uint8_t reg, uint8_t num)
{
	uint8_t data = 0;
	HAL_I2C_Mem_Read(&hi2c1, DEV_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, num, 0xFF);
	return data;
}

/**
  * @brief  MPU6050��ʼ��
  * @param  ��
  * @retval ��
  * @attention ��
  */
void MPU6050_Init(void)
{
	HAL_Delay(1000);
 
	/* ������� */
	MPU6050_Write(0x6B, 0X00);
 
	/* �����ǲ����� = 1kHz / (1 + �����ʷ�Ƶ) = 100Hz */
	MPU6050_Write(0x19, 0x09);
 
	/* ����Ƶ��fs = 100Hz������ = fs/2 = 50Hz */
	MPU6050_Write(0x1A, 0X03);
 
	/* ���ü��ٶȼ����̷�Χ����С������Χ������2g */
	MPU6050_Write(0x1C, 0x00);
 
	/* �������������̷�Χ����С������Χ������250��/s */
	MPU6050_Write(0x1B, 0X00);
	
	/* ��Ԫ����ʼ�� */
	data.quat[0] = 1;
	
	/* ��ʼ�����ٶȼƵ�ͨ�˲� */
	Filter_LowPassInit(&data.acc_filter[0], 0.2);
	Filter_LowPassInit(&data.acc_filter[1], 0.2);
	Filter_LowPassInit(&data.acc_filter[2], 0.2);
}

/**
  * @brief  MPU6050У׼
  * @param  ��
  * @retval ��
  * @attention �農����ˮƽ�棬ע�������ǰ�װ��̬
  */
void MPU6050_Cali(void)
{
	/* ͳ��ǰ500�����ݣ��ó�ƫ������ͳ����Ϻ�У׼��־λ��1 */
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
	
	/* ��У׼��־λ����1�������ƽ��ֵ������־λ��2��������LED������У׼���� */
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
  * @brief  ��Ԫ������
  * @param  ��
  * @retval ��
  * @attention ��
  */
void MPU6050_QuatCalc(void)
{
	double norm = 0;
	double vx = 0, vy = 0, vz = 0;
	double ex = 0, ey = 0, ez = 0;
	double exInt = 0, eyInt = 0, ezInt = 0;
 
	/* ������������ */
	vx = 2.0 * (data.quat[1] * data.quat[3] - data.quat[0] * data.quat[2]);
	vy = 2.0 * (data.quat[0] * data.quat[1] + data.quat[2] * data.quat[3]);
	vz = data.quat[0] * data.quat[0] - data.quat[1] * data.quat[1] - data.quat[2] * data.quat[2] + data.quat[3] * data.quat[3];
 
	/* ���ò���������������ʵ������֮������ */
	ex = (data.accel[1] * vz - data.accel[2] * vy);
	ey = (data.accel[2] * vx - data.accel[0] * vz);
	ez = (data.accel[0] * vy - data.accel[1] * vx);
 
	/* �����������л������� */
	exInt = exInt + ex * Ki;
	eyInt = eyInt + ey * Ki;
	ezInt = ezInt + ez * Ki;
 
	/* �������������� */
	data.gyro[0] = data.gyro[0] + Kp * ex + exInt;
	data.gyro[1] = data.gyro[1] + Kp * ey + eyInt;
	data.gyro[2] = data.gyro[2] + Kp * ez + ezInt;
	
	/* ����Ԫ������΢������ */
	data.quat[0] = data.quat[0] + (-data.quat[1] * data.gyro[0] - data.quat[2] * data.gyro[1] - data.quat[3] * data.gyro[2]) * T * 0.5f;
	data.quat[1] = data.quat[1] + (data.quat[0] * data.gyro[0] + data.quat[2] * data.gyro[2] - data.quat[3] * data.gyro[1]) * T * 0.5f;
	data.quat[2] = data.quat[2] + (data.quat[0] * data.gyro[1] - data.quat[1] * data.gyro[2] + data.quat[3] * data.gyro[0]) * T * 0.5f;
	data.quat[3] = data.quat[3] + (data.quat[0] * data.gyro[2] + data.quat[1] * data.gyro[1] - data.quat[2] * data.gyro[0]) * T * 0.5f;
	
	/* ����Ԫ����һ������ */
	norm = sqrt(data.quat[0] * data.quat[0] + data.quat[1] * data.quat[1] + data.quat[2] * data.quat[2] + data.quat[3] * data.quat[3]);
	data.quat[0] = data.quat[0] / norm;
	data.quat[1] = data.quat[1] / norm;
	data.quat[2] = data.quat[2] / norm;
	data.quat[3] = data.quat[3] / norm;
	
	/* ����ŷ���� */
	angle.roll = atan2(2 * data.quat[2] * data.quat[3] + 2* data.quat[0]*data.quat[1],	\
							 -2 * data.quat[1] * data.quat[1]- 2 * data.quat[2] * data.quat[2] + 1) * Rad;
	angle.yaw = atan2(2 * data.quat[1] * data.quat[2] + 2 * data.quat[0]*data.quat[3],	\
							 -2 * data.quat[2] * data.quat[2] - 2 * data.quat[3] * data.quat[3] + 1) * Rad;
    angle.pitch = asin(-2 * data.quat[1] * data.quat[3] + 2 * data.quat[0] * data.quat[2]) * Rad;
	
	/* ������ٶ� */
	speed.roll = data.gyro[0] / PI * 180.f;
	speed.yaw = data.gyro[2] / PI * 180.f;
	speed.pitch = data.gyro[1] / PI * 180.f;
	/*������ٶ�*/
	
}

/**
  * @brief  ������������ֵ
  * @param  ��
  * @retval ��
  * @attention �Ƕȡ����ٶȡ��¶�
  */
void MPU6050_DataUpdate(void)
{	
	/* ��ȡ������ԭʼֵ */
	data.gyro[0] = (int16_t)((MPU6050_Read(0x43, 1) << 8) | (MPU6050_Read(0x44, 1))) / (131.068 * 57.30);
	data.gyro[1] = (int16_t)((MPU6050_Read(0x45, 1) << 8) | (MPU6050_Read(0x46, 1))) / (131.068 * 57.30);
	data.gyro[2] = (int16_t)((MPU6050_Read(0x47, 1) << 8) | (MPU6050_Read(0x48, 1))) / (131.068 * 57.30);
	data.temp = (int16_t)((MPU6050_Read(0x41, 1) << 8) | (MPU6050_Read(0x42, 1))) / 340 + 36.53;
	data.accel[0] = (int16_t)((MPU6050_Read(0x3B, 1) << 8) | (MPU6050_Read(0x3C, 1))) / 16384.f * 9.8f;
	data.accel[1] = (int16_t)((MPU6050_Read(0x3D, 1) << 8) | (MPU6050_Read(0x3E, 1))) / 16384.f * 9.8f;
	data.accel[2] = (int16_t)((MPU6050_Read(0x3F, 1) << 8) | (MPU6050_Read(0x40, 1))) / 16384.f * 9.8f;
	
	/* ��У׼����ִ��У׼���򣬼��㲹��ֵ */
	#ifdef MPU6050_DO_CALI
		MPU6050_Cali();
		
		/* ��У׼����ɣ���ԭʼֵ���в���������Ƕ�ֵ */
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
	
	/* ����У׼����ֱ��ʹ�ö�ȡֵ*/
	#ifdef MPU6050_NO_CALI
		
		/* ����Ƕ�ֵ */
		MPU6050_QuatCalc();
	#endif
		
}
