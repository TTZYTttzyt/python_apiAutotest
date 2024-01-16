/*TOF模块，可用于测距0-30cm*/
#include "distance.h"
#include "cmsis_os.h"
#include "usart.h"

uint8_t uart3Buf[10];
uint16_t tof_distance;
uint8_t tof_command[4][3]		//tof控制信号
	={0xA5,0x50,0xF5,
		0xa5,0x51,0xf6,
		0xa5,0x52,0xf7,
		0xa5,0x53,0xf8
		};

void TOF_decode(uint8_t *buf)//tof数据解码为距离
{
	tof_distance = (buf[4]<<8)|buf[5];
}

void TOF_UART3_IRQHandler()//TOF串口接收处理函数
{
	static uint16_t rxCnt=0;
	if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE))
	{
		uart3Buf[rxCnt++] = huart3.Instance->DR;
	}
  if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE))
  {
    /* clear idle it flag avoid idle interrupt all the time */
    __HAL_UART_CLEAR_IDLEFLAG(&huart3);
		TOF_decode(uart3Buf);
	rxCnt = 0;
  }
}

void DistanceCallback(void const * argument)
{
  /* USER CODE BEGIN Distance_task */
	
  __HAL_UART_ENABLE_IT(&huart3,UART_IT_RXNE);//初始化tof串口
	__HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);  
  /* Infinite loop */
  for(;;)
  {

		osDelay(100);
  }

  /* USER CODE END Distance_task */
}

