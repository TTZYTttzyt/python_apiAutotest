#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "stdint.h"
#include "PID.h"
#include "usart.h"
#include "cmsis_os.h"

#define RR 30u    //电机减速比
#define RELOADVALUE1 __HAL_TIM_GetAutoreload(&htim3)    //获取自动装载值,本例中为20000
#define COUNTERNUM1 __HAL_TIM_GetCounter(&htim3)        //获取编码器定时器中的计数值
#define RELOADVALUE2 __HAL_TIM_GetAutoreload(&htim4)    //获取自动装载值,本例中为20000
#define COUNTERNUM2 __HAL_TIM_GetCounter(&htim4)        
#define RELOADVALUE3 __HAL_TIM_GetAutoreload(&htim5)    //获取自动装载值,本例中为20000
#define COUNTERNUM3 __HAL_TIM_GetCounter(&htim5)        
#define RELOADVALUE4 __HAL_TIM_GetAutoreload(&htim8)    //获取自动装载值,本例中为20000
#define COUNTERNUM4 __HAL_TIM_GetCounter(&htim8)        
#define IN11(state) HAL_GPIO_WritePin(GPIOC,GPIO_PIN_0,(GPIO_PinState)(state))    //M1
#define IN21(state) HAL_GPIO_WritePin(GPIOC,GPIO_PIN_1,(GPIO_PinState)(state))    //M2
#define IN31(state) HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2,(GPIO_PinState)(state))    //M3
#define IN41(state) HAL_GPIO_WritePin(GPIOC,GPIO_PIN_3,(GPIO_PinState)(state))    //M4
#define WHEELBASE 165    //轴距
#define WHEELTRACK 200 //轮距
#define OFFSET_X  0    //x抵消
#define OFFSET_Y  0    //y抵消
#define PI  3.1415927f
#define WHEEL_RADIUS 40  //轮半径

#ifndef ABS
#define ABS(x) ((x)>=0?(x):-(x))
#define LIMIT(x,min,max) (x)=(((x)<=(min))?(min):(((x)>=(max))?(max):(x)))
#endif

//电机结构体
typedef struct _Motor
{
	int32_t lastAngle;        //上10ms转过的角度
	int32_t totalAngle;       //总的角度
	int16_t loopNum;          //溢出次数计数值
	
	float speed;	//电机输出轴目前转速,单位为RPM
	float targetspeed;
	int output;
//	int32_t angle;
	int32_t targetAngle;//目标角度(编码器值)//加上角度增量时可以无限大
	
	PID speedPID;//速度pid(单级)
	CascadePID anglePID;//角度pid，串级
}Motor;

typedef struct _Chassis
{
	struct Move
	{
		float vx;//当前左右平移速度 mm/s
		float vy;//当前前后移动速度 mm/s
		float vw;//当前旋转速度 rad/s
		
		float maxVx,maxVy,maxVw; //三个分量最大速度
	
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


