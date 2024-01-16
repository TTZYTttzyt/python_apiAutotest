#include "filter.h"

/**
  * @brief  初始化一个低通滤波器
  * @param  p:  滤波器
  * @param  rate：滤波系数
  * @retval 无
  * @attention rate越小，延迟越大，滤波效果越好，
			   rate越大，延迟越小，滤波效果越不好，
			   rate应该等于0与1之间的小数
  */
void Filter_LowPassInit(LowPassFilter *p, float rate)
{
	p->rate = rate;
	p->last_data = 0;
}


/**
  * @brief  低通滤波器
  * @param  p:  滤波器
  * @param  new_data:待滤波数据
  * @retval 滤波后的数据
  * @attention 无
  */
float Filter_LowPass(LowPassFilter *p, float new_data)
{
	p->last_data = p->last_data * (1 - p->rate) + new_data * p->rate;
	return p->last_data;
}

/**
  * @brief  初始化一个卡尔曼滤波器
  * @param  p:  滤波器
  * @param  T_Q:系统噪声协方差
  * @param  T_R:测量噪声协方差
  * @retval 无
  * @attention R固定，Q越大，代表越信任侧量值，Q无穷代表只用测量值
  *           反之，Q越小代表越信任模型预测值，Q为零则是只用模型预测
  */
void Filter_KalmanInit(KalmanFilter *p, float T_Q, float T_R)
{
	p->X_last = (float)0;
	p->P_last = 0;
	p->Q = T_Q;
	p->R = T_R;
}


/**
  * @brief  卡尔曼滤波器
  * @param  p:  滤波器
  * @param  dat:待滤波数据
  * @retval 滤波后的数据
  * @attention Z(k)是系统输入,即测量值   X(k|k)是卡尔曼滤波后的值,即最终输出
  *            A=1 B=0 H=1 I=1  W(K)  V(k)是高斯白噪声,叠加在测量值上了,可以不用管
  *            以下是卡尔曼的5个核心公式
  *            一阶H'即为它本身,否则为转置矩阵
  */
float Filter_Kalman(KalmanFilter *p, float dat)
{
	p->X_pre = p->X_last;                            //x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
	p->P_pre = p->P_last + p->Q;                     //p(k|k-1) = A*p(k-1|k-1)*A'+Q
	p->kg = p->P_pre / (p->P_pre + p->R);            //kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
	p->X_now = p->X_pre + p->kg * (dat - p->X_pre);  //x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
	p->P_now = (1 - p->kg) * p->P_pre;               //p(k|k) = (I-kg(k)*H)*P(k|k-1)
	p->P_last = p->P_now;                            //状态更新
	p->X_last = p->X_now;                            //状态更新
	return p->X_now;                                 //输出预测结果x(k|k)
}




/**
  * @brief  初始化一个均值滤波器
  * @param  滤波器结构体
  * @param  需要关联到滤波器结构体的数组
	* @param  滤波器缓存区大小（滤波器内部用于存储数据的数组的大小）
	* @retval 无
  * @attention  因为没有用到malloc动态内存申请，所以必须关联一个全局float数组
  */
void Filter_MeanInit(MeanFilter *x,void *value_arr, uint16_t size)
{
	x->size = size;
	x->index = 0;
	x->sum = 0;
	x->value_arr = value_arr;
	for(uint16_t i=0;i<size;i++)
	{
		x->value_arr[i] = 0;
	}
}

/**
  * @brief  均值滤波函数
  * @param  滤波器结构体，需要滤波的数据
  * @retval 滤波输出量
  * @attention 无
  */
float Filter_Mean(MeanFilter *x, float data)
{
	float retval;
	
	x->sum -= x->value_arr[x->index];
	x->sum += data;
	retval = x->sum / (float)x->size;
	
	x->value_arr[x->index] = data;
	x->index++;
	if(x->index >= x->size)
	{
		x->index = 0;
	}
	
	return retval;
}
