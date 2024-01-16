/*陀螺仪JY901*/
#include "JY901.h"
#include "cmsis_os.h"
#include "usart.h"
uint8_t buff_JY[2];

JY901 jy901;

void JY901_UART()
{
  static unsigned char JY901RxBuffer[250];
	static unsigned char JY901RxCn = 0;	
	JY901RxBuffer[JY901RxCn++]=buff_JY[0];	//将收到的数据存入缓冲区中
	if (JY901RxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
	{
		JY901RxCn=0;
		return;
	}
	if (JY901RxCn<11) {return;}//数据不满11个，则返回
	else
	{
		switch(JY901RxBuffer[1])//判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
		{
			case 0x53:
            jy901.roll=(float)(JY901RxBuffer[3]<<8|JY901RxBuffer[2])*180/32768;
            jy901.pitch=(float)(JY901RxBuffer[5]<<8|JY901RxBuffer[4])*180/32768;
            jy901.yaw=(float)(JY901RxBuffer[7]<<8|JY901RxBuffer[6])*180/32768;
		}
		JY901RxCn=0;//清空缓存区
	}
}

void JY901Callback(void const * argument)
{
  /* USER CODE BEGIN JY901_task */
   HAL_UART_Receive_IT(&huart4,buff_JY,1);
  /* Infinite loop */
  for(;;)
  {
		
    osDelay(2);
  }
  /* USER CODE END JY901_task */
}
