#include "filter.h"

/**
  * @brief  ��ʼ��һ����ͨ�˲���
  * @param  p:  �˲���
  * @param  rate���˲�ϵ��
  * @retval ��
  * @attention rateԽС���ӳ�Խ���˲�Ч��Խ�ã�
			   rateԽ���ӳ�ԽС���˲�Ч��Խ���ã�
			   rateӦ�õ���0��1֮���С��
  */
void Filter_LowPassInit(LowPassFilter *p, float rate)
{
	p->rate = rate;
	p->last_data = 0;
}


/**
  * @brief  ��ͨ�˲���
  * @param  p:  �˲���
  * @param  new_data:���˲�����
  * @retval �˲��������
  * @attention ��
  */
float Filter_LowPass(LowPassFilter *p, float new_data)
{
	p->last_data = p->last_data * (1 - p->rate) + new_data * p->rate;
	return p->last_data;
}

/**
  * @brief  ��ʼ��һ���������˲���
  * @param  p:  �˲���
  * @param  T_Q:ϵͳ����Э����
  * @param  T_R:��������Э����
  * @retval ��
  * @attention R�̶���QԽ�󣬴���Խ���β���ֵ��Q�������ֻ�ò���ֵ
  *           ��֮��QԽС����Խ����ģ��Ԥ��ֵ��QΪ������ֻ��ģ��Ԥ��
  */
void Filter_KalmanInit(KalmanFilter *p, float T_Q, float T_R)
{
	p->X_last = (float)0;
	p->P_last = 0;
	p->Q = T_Q;
	p->R = T_R;
}


/**
  * @brief  �������˲���
  * @param  p:  �˲���
  * @param  dat:���˲�����
  * @retval �˲��������
  * @attention Z(k)��ϵͳ����,������ֵ   X(k|k)�ǿ������˲����ֵ,���������
  *            A=1 B=0 H=1 I=1  W(K)  V(k)�Ǹ�˹������,�����ڲ���ֵ����,���Բ��ù�
  *            �����ǿ�������5�����Ĺ�ʽ
  *            һ��H'��Ϊ������,����Ϊת�þ���
  */
float Filter_Kalman(KalmanFilter *p, float dat)
{
	p->X_pre = p->X_last;                            //x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
	p->P_pre = p->P_last + p->Q;                     //p(k|k-1) = A*p(k-1|k-1)*A'+Q
	p->kg = p->P_pre / (p->P_pre + p->R);            //kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
	p->X_now = p->X_pre + p->kg * (dat - p->X_pre);  //x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
	p->P_now = (1 - p->kg) * p->P_pre;               //p(k|k) = (I-kg(k)*H)*P(k|k-1)
	p->P_last = p->P_now;                            //״̬����
	p->X_last = p->X_now;                            //״̬����
	return p->X_now;                                 //���Ԥ����x(k|k)
}




/**
  * @brief  ��ʼ��һ����ֵ�˲���
  * @param  �˲����ṹ��
  * @param  ��Ҫ�������˲����ṹ�������
	* @param  �˲�����������С���˲����ڲ����ڴ洢���ݵ�����Ĵ�С��
	* @retval ��
  * @attention  ��Ϊû���õ�malloc��̬�ڴ����룬���Ա������һ��ȫ��float����
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
  * @brief  ��ֵ�˲�����
  * @param  �˲����ṹ�壬��Ҫ�˲�������
  * @retval �˲������
  * @attention ��
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
