/*摇柄：ADC采样*/
#include "rocker.h"
#include "cmsis_os.h"
#include "adc.h"
uint16_t adc_value[4];

void Rocker_callback(void const * argument)
{
  /* USER CODE BEGIN Rocker_callback */
 HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_value,4);

  /* Infinite loop */
  for(;;)
  { 

		
    osDelay(500);
  }
  /* USER CODE END Rocker_callback */
}








//#include "rocker.h"   单通道轮询
//#include "cmsis_os.h"
//#include "stdio.h"
//#include "adc.h"
//	float adc_value[2];
//uint16_t Get_adc_2()
//{
//    //开启ADC1
//  HAL_ADC_Start(&hadc1);
//    //等待ADC转换完成，超时为100ms
//    HAL_ADC_PollForConversion(&hadc1,100);
//    //判断ADC是否转换成功
//	HAL_Delay(0);
//    if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1),HAL_ADC_STATE_REG_EOC)){
//         //读取值
//       return HAL_ADC_GetValue(&hadc1);
//    }
//    return 0;
//}

//      char i;
//      for(i=0;i<4;i++)
//    {
//    HAL_ADC_Start(&hadc1);
//    HAL_ADC_PollForConversion(&hadc1,10);
//    adc_value[i]=HAL_ADC_GetValue(&hadc1);
//    }


//void Rocker_callback(void const * argument)
//{
//  /* USER CODE BEGIN Rocker_callback */

//  /* Infinite loop */
//  for(;;)
//  {
//      uint16_t i,j;
//        for(j=0;j<2;j++)
//    {	
//        adc_value[j]=Get_adc_2();//分别存放通道1、2的ADC值
//    }		
//      
//    osDelay(1);
//  }
//  /* USER CODE END Rocker_callback */
//}


//#include "rocker.h"   单通道中断
//#include "cmsis_os.h"
//#include "stdio.h"
//#include "adc.h"
//	float adc_value[2];
//uint16_t Get_adc_2()
//{
//    //开启ADC1
//    HAL_ADC_Start(&hadc1);
//    //等待ADC转换完成，超时为100ms
//    HAL_ADC_PollForConversion(&hadc1,100);
//    //判断ADC是否转换成功
//	HAL_Delay(0);
//    if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1),HAL_ADC_STATE_REG_EOC)){
//         //读取值
//       return HAL_ADC_GetValue(&hadc1);
//    }
//    return 0;
//}

//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//{
//    if(hadc==&hadc1)
//    {
//        adc_value[0]=HAL_ADC_GetValue(&hadc1);//分别存放通道1、2的ADC值
//    }
//}

//void Rocker_callback(void const * argument)
//{
//  /* USER CODE BEGIN Rocker_callback */
//    __HAL_ADC_ENABLE_IT(&hadc1,ADC_IT_EOC);//用于打开ADC转换完成中断
//        HAL_ADC_Start(&hadc1);
//        adc_value[0]=HAL_ADC_GetValue(&hadc1);
//  /* Infinite loop */
//  for(;;)
//  {
//      
//    osDelay(1);
//  }
//  /* USER CODE END Rocker_callback */
//}
