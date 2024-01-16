#ifndef __FILER_H
#define __FILER_H

#include "main.h"

//��ͨ�˲��ṹ��
typedef struct
{
	float rate;
	float last_data;
} LowPassFilter;

//һ�׿������˲��ṹ��
typedef struct
{
	float X_last; //��һʱ�̵����Ž��  X(k|k-1)
	float X_pre;  //��ǰʱ�̵�Ԥ����  X(k|k-1)
	float X_now;  //��ǰʱ�̵����Ž��  X(k|k)
	float P_pre;  //��ǰʱ��Ԥ������Э����  P(k|k-1)
	float P_now;  //��ǰʱ�����Ž����Э����  P(k|k)
	float P_last; //��һʱ�����Ž����Э����  P(k-1|k-1)
	float kg;     //kalman����
	float Q;
	float R;
} KalmanFilter;


//��ֵ�˲��ṹ��
typedef struct
{
	uint8_t index;
	float *value_arr;
	uint8_t size;
	float sum;
} MeanFilter;

void  Filter_LowPassInit(LowPassFilter *p, float rate);
float Filter_LowPass(LowPassFilter *p, float new_data);
void  Filter_KalmanInit(KalmanFilter *p, float T_Q, float T_R);
float Filter_Kalman(KalmanFilter *p, float dat);
void Filter_MeanInit(MeanFilter *x,void *value_arr, uint16_t size);
float Filter_Mean(MeanFilter *x, float get);

#endif /* __FILER_H */
