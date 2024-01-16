#ifndef _WAVE_H_
#define _WAVE_H_
#include "main.h"

#define TRIG_H  HAL_GPIO_WritePin(Trig_GPIO_Port,Trig_Pin,GPIO_PIN_SET)
#define TRIG_L  HAL_GPIO_WritePin(Trig_GPIO_Port,Trig_Pin,GPIO_PIN_RESET)
void USER_TIM_IRQHandlre(void);


#endif



