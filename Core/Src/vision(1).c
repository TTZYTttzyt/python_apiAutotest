/*mpu6050”ÎOLEDœ‘ æ*/
#include "cmsis_os.h"
#include <stdio.h>
#include <math.h>
#include "usart.h"
#include "servo.h"
#include "JY901.h"

int t3=0;
void BlueteethCallback(void const * argument)
{
  /* USER CODE BEGIN BlueteethCallback */
  /* Infinite loop */
  for(;;)
  {

		osDelay(1);
  }
  /* USER CODE END BlueteethCallback */
}

void OledCallback(void const * argument)
{
  /* USER CODE BEGIN OledCallback */
	
  /* Infinite loop */
  for(;;)
  {
		
	
    osDelay(10);
  }
  /* USER CODE END OledCallback */
}




