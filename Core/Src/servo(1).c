#include "servo.h"
#include "tim.h"
#include "math.h"

ServoDef Servo;
_Bool Enable=1;

void Servo_Init()
{
	Servo.media1=2400;
	Servo.media2=1800;
	Servo.angle1=Servo.media1;
	Servo.angle2=Servo.media2;
	
	Servo.TargetX=60;
	Servo.TargetY=74;
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	__HAL_TIM_SET_COMPARE(&htim2 ,TIM_CHANNEL_2,Servo.angle1);
	__HAL_TIM_SET_COMPARE(&htim2 ,TIM_CHANNEL_3,Servo.angle2);
	
	PID_Init(&Servo.pidX,3.5,0.003,80,0,2500);//i=0.01  p4,d90   2,0.003,45,50,2500   5,80
	PID_Init(&Servo.pidY,3.5,0.001,80,0,2500);
}
int t =0;
int draw_round=0;
double p=0;
double k=0.13;
void  ServoCallback(void const * argument)
{
  /* USER CODE BEGIN LEDCallback */
	Servo_Init();
  /* Infinite loop */
  for(;;)
  {
		if(Enable)
		{
			PID_SingleCalc(&Servo.pidX,Servo.TargetX,Servo.RecX);
			PID_SingleCalc(&Servo.pidY,Servo.TargetY,Servo.RecY);
			Servo.angle1=Servo.media1+Servo.pidX.output;
			Servo.angle2=Servo.media2+Servo.pidY.output;
		}
		else if(Enable==0)
		{
			Servo.angle1=Servo.media1;
			Servo.angle2=Servo.media2;
		}
		LIMIT(Servo.angle1,1500,2500);
		LIMIT(Servo.angle2,1000,2000);
		
		__HAL_TIM_SET_COMPARE(&htim2 ,TIM_CHANNEL_2,Servo.angle1);
		__HAL_TIM_SET_COMPARE(&htim2 ,TIM_CHANNEL_3,Servo.angle2);
		
    osDelay(10);
		
		//		if(draw_round == 1)  //ÕýÏò»­Ô²
//		{
//		t++;
//		p = k * t;
//    Servo.TargetX=60+(20 * _sin(p) );
//		Servo.TargetY=74+(20 * _cos(p));
//		if( p >= 6.28)
//			t = 0; 
// }
// 
//	 if(draw_round == 2)  //ÄæÏò»­Ô²
//	 {
//		 t++;
//		 p = 0.13 * t;
//		 Servo.TargetX=60+(20 * _sin(p) );
//		 Servo.TargetY=74-(20 * _cos(p));
//		 if( p >= 6.28)
//				t = 0; 
//		}

  }
  /* USER CODE END LEDCallback */
}






