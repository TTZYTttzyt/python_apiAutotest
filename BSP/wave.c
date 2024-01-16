/*超声波模块,单位为cm*/
#include "wave.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include "stdlib.h"
#include "tim.h"
#include "usart.h"
#include "main.h"
#include <string.h>

double now_time=0;
char echo_i=0;
char echo_j=0;
double distant;      //测量距离

double last_time;
int flag[2]={0};
void USER_TIM_IRQHandlre(void)
{
	now_time=0.00001+now_time;//时间累计
}
//void WaveCallback(void const * argument)
//{
//  /* USER CODE BEGIN Wave_task */
//    HAL_TIM_Base_Start_IT(&htim11);  //开启定时器11计时 
//  /* Infinite loop */
//   for(;;)
//   {
//    TRIG_L;//PG13
//    osDelay(1);
//    TRIG_H;
//    osDelay(10);
//    TRIG_L;          
//    while(1)
//		{
//      flag[0]=HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_14);
//			if(flag[0])
//			{
//				if(echo_i==0)
//				{
//					last_time=now_time;
//					echo_i++;
//				}
//				break;
//			}
//		}
//	while(1)
//	{
//		flag[0]=HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_14);
//		if(!flag[0])
//		{
//			if(echo_j==0)
//			{
//				distant=(now_time*1.0-last_time*1.0)*1.0*34000.0*0.5;
//				echo_j++;
//			}
//			echo_i=0;
//			echo_j=0;
//			break;
//			}
//	}
//    osDelay(20);
//    }
//  /* USER CODE END Wave_task */
//}

