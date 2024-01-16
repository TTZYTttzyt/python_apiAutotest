/*������JY901*/
#include "JY901.h"
#include "cmsis_os.h"
#include "usart.h"
uint8_t buff_JY[2];

JY901 jy901;

void JY901_UART()
{
  static unsigned char JY901RxBuffer[250];
	static unsigned char JY901RxCn = 0;	
	JY901RxBuffer[JY901RxCn++]=buff_JY[0];	//���յ������ݴ��뻺������
	if (JY901RxBuffer[0]!=0x55) //����ͷ���ԣ������¿�ʼѰ��0x55����ͷ
	{
		JY901RxCn=0;
		return;
	}
	if (JY901RxCn<11) {return;}//���ݲ���11�����򷵻�
	else
	{
		switch(JY901RxBuffer[1])//�ж��������������ݣ�Ȼ���俽������Ӧ�Ľṹ���У���Щ���ݰ���Ҫͨ����λ���򿪶�Ӧ������󣬲��ܽ��յ�������ݰ�������
		{
			case 0x53:
            jy901.roll=(float)(JY901RxBuffer[3]<<8|JY901RxBuffer[2])*180/32768;
            jy901.pitch=(float)(JY901RxBuffer[5]<<8|JY901RxBuffer[4])*180/32768;
            jy901.yaw=(float)(JY901RxBuffer[7]<<8|JY901RxBuffer[6])*180/32768;
		}
		JY901RxCn=0;//��ջ�����
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
