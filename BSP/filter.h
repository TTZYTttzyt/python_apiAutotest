#ifndef __FILER_H
#define __FILER_H

#include "main.h"

//低通滤波结构体
typedef struct
{
	float rate;
	float last_data;
} LowPassFilter;

//一阶卡尔曼滤波结构体
typedef struct
{
	float X_last; //上一时刻的最优结果  X(k|k-1)
	float X_pre;  //当前时刻的预测结果  X(k|k-1)
	float X_now;  //当前时刻的最优结果  X(k|k)
	float P_pre;  //当前时刻预测结果的协方差  P(k|k-1)
	float P_now;  //当前时刻最优结果的协方差  P(k|k)
	float P_last; //上一时刻最优结果的协方差  P(k-1|k-1)
	float kg;     //kalman增益
	float Q;
	float R;
} KalmanFilter;


//均值滤波结构体
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
