#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "stdint.h"
#include "PID.h"
#include "usart.h"
#include "cmsis_os.h"

#define RR 30u    //������ٱ�
#define RELOADVALUE1 __HAL_TIM_GetAutoreload(&htim3)    //��ȡ�Զ�װ��ֵ,������Ϊ20000
#define COUNTERNUM1 __HAL_TIM_GetCounter(&htim3)        //��ȡ��������ʱ���еļ���ֵ
#define RELOADVALUE2 __HAL_TIM_GetAutoreload(&htim4)    //��ȡ�Զ�װ��ֵ,������Ϊ20000
#define COUNTERNUM2 __HAL_TIM_GetCounter(&htim4)        
#define RELOADVALUE3 __HAL_TIM_GetAutoreload(&htim5)    //��ȡ�Զ�װ��ֵ,������Ϊ20000
#define COUNTERNUM3 __HAL_TIM_GetCounter(&htim5)        
#define RELOADVALUE4 __HAL_TIM_GetAutoreload(&htim8)    //��ȡ�Զ�װ��ֵ,������Ϊ20000
#define COUNTERNUM4 __HAL_TIM_GetCounter(&htim8)        
#define IN11(state) HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,(GPIO_PinState)(state))    //M1
#define IN21(state) HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,(GPIO_PinState)(state))    //M2
#define IN31(state) HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,(GPIO_PinState)(state))    //M3
#define IN41(state) HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,(GPIO_PinState)(state))    //M4
#define WHEELBASE 165    //���
#define WHEELTRACK 200 //�־�
#define OFFSET_X  0    //x����
#define OFFSET_Y  0    //y����
#define PI  3.1415927f
#define WHEEL_RADIUS 40  //�ְ뾶

#ifndef ABS
#define ABS(x) ((x)>=0?(x):-(x))
#define LIMIT(x,min,max) (x)=(((x)<=(min))?(min):(((x)>=(max))?(max):(x)))
#endif

//����ṹ��
typedef struct _Motor
{
	int32_t lastAngle;        //��10msת���ĽǶ�
	int32_t totalAngle;       //�ܵĽǶ�
	int16_t loopNum;          //�����������ֵ
	
	float speed;	//��������Ŀǰת��,��λΪRPM
	float targetspeed;
	int output;
//	int32_t angle;
	int32_t targetAngle;//Ŀ��Ƕ�(������ֵ)//���ϽǶ�����ʱ�������޴�
	
	PID speedPID;//�ٶ�pid(����)
	CascadePID anglePID;//�Ƕ�pid������
}Motor;

typedef struct _Chassis
{
	struct Move
	{
		float vx;//��ǰ����ƽ���ٶ� mm/s
		float vy;//��ǰǰ���ƶ��ٶ� mm/s
		float vw;//��ǰ��ת�ٶ� rad/s
		
		float maxVx,maxVy,maxVw; //������������ٶ�
	
	}move;
	
	float angle;
	float lastAngle;
	float totalAngle;
	float targetAngle;
	
	PID wPID;
	
}Chassis;

enum Mode
{
	STOP=0,
	START,
	END,
	GO,
	LEFT,
	RIGHT,
	UPHILL,
	DOWNHILL,
	UPSTAIR,
	DOWNSTAIR,
	WHITE,
};

void Motor_Init(void);
void Motor_send(enum Mode mode);
void MotorStop(void);
void JY901_UART(void);
void Output_judge(float output);
void SetSpeed(int speed1,int speed2,int speed3,int speed4);
extern Motor motor[4];
extern int motorStat;
extern Motor motor1;   
extern Motor motor2;   
extern Motor motor3;
extern Motor motor4;
extern int state;
extern uint8_t buff_JY[2];
extern double roll;
extern double pitch;
extern double yaw;

#endif


