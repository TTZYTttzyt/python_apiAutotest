#include "motor.h"  
#include "tim.h"
//1  2
//3  4
Motor motor[4];
Chassis chassis={0};

void Motor_Init()
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);      //开启编码器定时器
  __HAL_TIM_ENABLE_IT(&htim3,TIM_IT_UPDATE);           //开启编码器定时器更新中断,防溢出处理
	__HAL_TIM_SET_COUNTER(&htim3, 10000);                //编码器定时器初始值设定为10000
	
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);      //开启编码器定时器
  __HAL_TIM_ENABLE_IT(&htim4,TIM_IT_UPDATE);           //开启编码器定时器更新中断,防溢出处理
	__HAL_TIM_SET_COUNTER(&htim4, 10000);
	
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);      //开启编码器定时器
  __HAL_TIM_ENABLE_IT(&htim5,TIM_IT_UPDATE);           //开启编码器定时器更新中断,防溢出处理
	__HAL_TIM_SET_COUNTER(&htim5, 10000);
	
	HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);      //开启编码器定时器
  __HAL_TIM_ENABLE_IT(&htim8,TIM_IT_UPDATE);           //开启编码器定时器更新中断,防溢出处理
	__HAL_TIM_SET_COUNTER(&htim8, 10000);
	
	HAL_TIM_Base_Start_IT(&htim12);                       //开启10ms定时器中断                                  //溢出计数
}

void ChassisInit()
{
	PID_Init(&motor[0].speedPID,50,0, 0, 700, 10000);	//30
	PID_Init(&motor[1].speedPID,50,0, 0, 700, 10000);	//
  PID_Init(&motor[2].speedPID,50,0, 0, 700, 10000);
	PID_Init(&motor[3].speedPID,50,0, 0, 700, 10000);
	PID_Init(&chassis.wPID,0.05,0,0,0,1.5);
	
	PID_Init(&motor[0].anglePID.inner,10,0.001,0,500,10000);
  PID_Init(&motor[0].anglePID.outer,2,0.0,0,0,1000);
	motor[0].targetAngle=995;
	
}
//底盘电机麦轮解算
void ChassisSpeedCalc()
{
	float rotateRatio[4];
	rotateRatio[0]=(WHEELBASE+WHEELTRACK)/2.0f-OFFSET_Y+OFFSET_X;
	rotateRatio[1]=(WHEELBASE+WHEELTRACK)/2.0f-OFFSET_Y-OFFSET_X;
	rotateRatio[2]=(WHEELBASE+WHEELTRACK)/2.0f+OFFSET_Y+OFFSET_X;
	rotateRatio[3]=(WHEELBASE+WHEELTRACK)/2.0f+OFFSET_Y-OFFSET_X;
	
	motor[0].targetspeed=(chassis.move.vx+chassis.move.vy-chassis.move.vw*rotateRatio[0])*60/(2*PI*WHEEL_RADIUS)*30;//FL
	motor[3].targetspeed=-(-chassis.move.vx+chassis.move.vy+chassis.move.vw*rotateRatio[1])*60/(2*PI*WHEEL_RADIUS)*30;//FR
	motor[1].targetspeed=(-chassis.move.vx+chassis.move.vy-chassis.move.vw*rotateRatio[2])*60/(2*PI*WHEEL_RADIUS)*30;//BL
	motor[2].targetspeed=-(chassis.move.vx+chassis.move.vy+chassis.move.vw*rotateRatio[3])*60/(2*PI*WHEEL_RADIUS)*30;//BR
}


/*********底盘电机速度更新**********/
void ChassisSend()
{
	PID_SingleCalc(&motor[0].speedPID,motor[0].targetspeed,motor[0].speed);		
	motor[0].output =motor[0].speedPID.output;		
	PID_SingleCalc(&motor[1].speedPID,motor[1].targetspeed,motor[1].speed);		
	motor[1].output = motor[1].speedPID.output;
	PID_SingleCalc(&motor[2].speedPID,motor[2].targetspeed,motor[2].speed);		
	motor[2].output = motor[2].speedPID.output;
	PID_SingleCalc(&motor[3].speedPID,motor[3].targetspeed,motor[3].speed);		
	motor[3].output = motor[3].speedPID.output;
}

void Output_judge1(int output)
{
	if(output>=0)//正转
		{
			LIMIT(output,0,6200);//6167
			__HAL_TIM_SET_COMPARE(&htim1 ,TIM_CHANNEL_1,(uint32_t)(output));
			IN11(0);
		}
	else
		{
			LIMIT(output,-6200,0);
			__HAL_TIM_SET_COMPARE(&htim1 ,TIM_CHANNEL_1,(uint32_t)(10000+output));
			IN11(1);
		}	
}
void Output_judge2(int output)
{
	if(output>=0)//正转
		{
			LIMIT(output,0,6200);
			__HAL_TIM_SET_COMPARE(&htim1 ,TIM_CHANNEL_2,(uint32_t)(output));
			IN21(0);
		}
	else
		{
			LIMIT(output,-6200,0);
			__HAL_TIM_SET_COMPARE(&htim1 ,TIM_CHANNEL_2,(uint32_t)(10000+output));
			IN21(1);
		}	
}
void Output_judge3(int output)
{
	if(output>=0)//正转
		{
			LIMIT(output,0,6200);
			__HAL_TIM_SET_COMPARE(&htim1 ,TIM_CHANNEL_3,(uint32_t)(output));
			IN31(0);
		}
	else
		{
			LIMIT(output,-6200,0);
			__HAL_TIM_SET_COMPARE(&htim1 ,TIM_CHANNEL_3,(uint32_t)(10000+output));
			IN31(1);
		}	
}
void Output_judge4(int output)
{
	if(output>=0)//正转
	{
		LIMIT(output,0,6200);
		__HAL_TIM_SET_COMPARE(&htim1 ,TIM_CHANNEL_4,(uint32_t)(output));
		IN41(0);
	}
	else
	{
		LIMIT(output,-6200,0);
		__HAL_TIM_SET_COMPARE(&htim1 ,TIM_CHANNEL_4,(uint32_t)(10000+output));
		IN41(1);
	}	
}

void MotorStop()
{
	__HAL_TIM_SET_COMPARE(&htim1 ,TIM_CHANNEL_1,0);
	IN11(0);
	__HAL_TIM_SET_COMPARE(&htim1 ,TIM_CHANNEL_2,0);
	IN21(0);
	__HAL_TIM_SET_COMPARE(&htim1 ,TIM_CHANNEL_3,0);
	IN31(0);
	__HAL_TIM_SET_COMPARE(&htim1 ,TIM_CHANNEL_4,0);
	IN41(0);
}

void SetSpeed(int speed1,int speed2,int speed3,int speed4)
{
	motor[0].targetspeed=speed1;
	motor[1].targetspeed=speed2;
	motor[2].targetspeed=speed3;
	motor[3].targetspeed=speed4;
}

void MotorCallback(void const * argument)
{
  /* USER CODE BEGIN MotorCallback */
  /* Infinite loop */
  for(;;)
  {
		
    osDelay(5);
  }
  /* USER CODE END MotorCallback */
}
int motorStat=0;
void ChassisCallback(void const * argument)
{
  /* USER CODE BEGIN ChassisCallback */
	Motor_Init();
	ChassisInit();
  /* Infinite loop */
  for(;;)
  {
		if(motorStat==1)//forward
		{
			chassis.move.vy=30; 
		}
		else if(motorStat==2)//back
		{
			chassis.move.vy=-30; 
		}
		else if(motorStat==3)//right
		{
			chassis.move.vx=30; 
			
		}
		else if(motorStat==4)//
		{
			chassis.move.vx=-30; 
			
		}
		else if(motorStat==5)//rightR
		{
			chassis.move.vw=-0.05; 
		}
		else if(motorStat==6)//leftR
		{
			chassis.move.vw=0.05; 
		}
		else if(motorStat==0)
		{
			chassis.move.vx=0; 
			chassis.move.vy=0; 
			chassis.move.vw=0; 
			MotorStop();
		}
		else if(motorStat==10)//CascadeCalc
		{
			PID_CascadeCalc(&motor[0].anglePID,motor[0].targetAngle,motor[0].totalAngle,motor[0].speed);
			Output_judge1(motor[0].anglePID.output);
		}
		if(motorStat && motorStat!=10)
		{
			ChassisSpeedCalc();
			ChassisSend();
			Output_judge1(motor[0].output);
			Output_judge2(motor[1].output);
			Output_judge3(motor[2].output);
			Output_judge4(motor[3].output);
		}
					
    osDelay(5);
  }
  /* USER CODE END ChassisCallback */
}





