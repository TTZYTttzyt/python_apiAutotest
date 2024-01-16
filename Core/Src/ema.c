#include "ema.h"
#include "cmsis_os.h"
#include "tim.h"
#include "math.h"
#include "gpio.h"

#define PI 3.1415926
#define LIMIT(x,min,max) (x)=(((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

#define L1 150.0
#define L2 200.0

uint16_t compare_yaw=1500;
uint16_t compare_up=1450;
uint16_t compare_front=1400;
uint16_t compare_catch=2500;
uint16_t compare_open=500;

Coordinate cord;
MchArmAngle angle;

int x,y,z;
        
        
    //直角坐标转为球坐标
void CordTF(Coordinate* cord , double x , double y , double z)//int16_t
{
	cord->x = x;
	cord->y = y;
	cord->z = z;
	cord->sqr_rho = x*x + y*y +z*z;
	cord->rho = sqrt((float)cord->sqr_rho);
	cord->phi = (float)atan2((double)y,(double)x);//正负派   base
	cord->theta = (float)PI/2.0f - acos(z/cord->rho);//  正负二分之派，在正半轴完美，负半轴要换算二分之派
}
//机械臂角度解算
void AngleCalc(MchArmAngle* angle,Coordinate* cord)
{
	if((cord->sqr_rho<(L1+L2)*(L1+L2))&&(cord->sqr_rho>(L1-L2)*(L1-L2)))
	{
		angle->phi = cord->phi * 180/PI;//角度
		float temp1,temp2;
		temp1 = (cord->sqr_rho + L1*L1 - L2*L2)/(2*L1*cord->rho);
		angle->alpha = cord->theta + acos(temp1);       //right
		temp2 = ((float)cord->z - L1*sin(angle->alpha))/L2;  
		angle->gamma = asin(temp2);                 //正负二分之派  left

		angle->alpha = angle->alpha * 180/PI;
		angle->gamma =angle->gamma * 180/PI;
		
//		LIMIT(angle->phi,0,360);  
		LIMIT(angle->alpha,0,90);
//		LIMIT(angle->gamma,-75,10);//可正负
	}
}
void ema_task(void const * argument)
{
  /* USER CODE BEGIN ema_task */
    
    HAL_TIM_PWM_Start(&htim10,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim11,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim13,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim14,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_1);
      __HAL_TIM_SET_COMPARE (&htim10,TIM_CHANNEL_1,compare_yaw);
      __HAL_TIM_SET_COMPARE (&htim11,TIM_CHANNEL_1,compare_catch);
      __HAL_TIM_SET_COMPARE (&htim13,TIM_CHANNEL_1,compare_front);
      __HAL_TIM_SET_COMPARE (&htim14,TIM_CHANNEL_1,compare_up);
      __HAL_TIM_SET_COMPARE (&htim9,TIM_CHANNEL_1,compare_open);
    x=0;
    y=200;
    z=150;
    
    CordTF(&cord,x,y,z);
  /* Infinite loop */
  for(;;)
  {
      AngleCalc(&angle,&cord);
      __HAL_TIM_SET_COMPARE (&htim10,TIM_CHANNEL_1,compare_yaw);
      __HAL_TIM_SET_COMPARE (&htim11,TIM_CHANNEL_1,compare_catch);
      __HAL_TIM_SET_COMPARE (&htim13,TIM_CHANNEL_1,compare_front);
      __HAL_TIM_SET_COMPARE (&htim14,TIM_CHANNEL_1,compare_up);
      __HAL_TIM_SET_COMPARE (&htim9,TIM_CHANNEL_1,compare_open);
      
      if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_3)==GPIO_PIN_RESET)
      {
        z++;
        CordTF(&cord,x,y,z);
      }
      if(HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4)==GPIO_PIN_RESET)
      {
        z--;
        CordTF(&cord,x,y,z);
      }
      compare_up=1450-11.11*angle.gamma;
      compare_front=1400+11.11*(angle.alpha-90);
    osDelay(50);
  }
  /* USER CODE END ema_task */
}
