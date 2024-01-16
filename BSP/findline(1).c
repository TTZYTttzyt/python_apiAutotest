/*灰度传感器循迹模块，阻塞式收发则使用串口，否则使用7个GPIO口*/
#include "findline.h"
#include "cmsis_os.h"
#include "usart.h"

void Find_line_task(void const * argument)
{
  /* USER CODE BEGIN Find_line_task */

  /* Infinite loop */
  for(;;)
  {  
		
    osDelay(1);
  }
  /* USER CODE END Find_line_task */
}


