#ifndef _SERVO_H_
#define _SERVO_H_

#include "cmsis_os.h"
#include "PID.h"

typedef struct __SERVO 
{
	
	float TargetX,TargetY;//Ä¿±êÖµ
	uint8_t RecX,RecY;
	float media1,media2;
	PID pidX,pidY;

	uint32_t angle1;//  
	uint32_t angle2;//  
	
}ServoDef;

extern ServoDef Servo;
extern _Bool Enable; 
extern int draw_round;
#endif

