/*ҡ����ADC����*/
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








//#include "rocker.h"   ��ͨ����ѯ
//#include "cmsis_os.h"
//#include "stdio.h"
//#include "adc.h"
//	float adc_value[2];
//uint16_t Get_adc_2()
//{
//    //����ADC1
//  HAL_ADC_Start(&hadc1);
//    //�ȴ�ADCת����ɣ���ʱΪ100ms
//    HAL_ADC_PollForConversion(&hadc1,100);
//    //�ж�ADC�Ƿ�ת���ɹ�
//	HAL_Delay(0);
//    if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1),HAL_ADC_STATE_REG_EOC)){
//         //��ȡֵ
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
//        adc_value[j]=Get_adc_2();//�ֱ���ͨ��1��2��ADCֵ
//    }		
//      
//    osDelay(1);
//  }
//  /* USER CODE END Rocker_callback */
//}


//#include "rocker.h"   ��ͨ���ж�
//#include "cmsis_os.h"
//#include "stdio.h"
//#include "adc.h"
//	float adc_value[2];
//uint16_t Get_adc_2()
//{
//    //����ADC1
//    HAL_ADC_Start(&hadc1);
//    //�ȴ�ADCת����ɣ���ʱΪ100ms
//    HAL_ADC_PollForConversion(&hadc1,100);
//    //�ж�ADC�Ƿ�ת���ɹ�
//	HAL_Delay(0);
//    if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1),HAL_ADC_STATE_REG_EOC)){
//         //��ȡֵ
//       return HAL_ADC_GetValue(&hadc1);
//    }
//    return 0;
//}

//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//{
//    if(hadc==&hadc1)
//    {
//        adc_value[0]=HAL_ADC_GetValue(&hadc1);//�ֱ���ͨ��1��2��ADCֵ
//    }
//}

//void Rocker_callback(void const * argument)
//{
//  /* USER CODE BEGIN Rocker_callback */
//    __HAL_ADC_ENABLE_IT(&hadc1,ADC_IT_EOC);//���ڴ�ADCת������ж�
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
