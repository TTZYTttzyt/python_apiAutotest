#ifndef _JY901_H_
#define _JY901_H_
#include "stdint.h"

typedef struct
{
	float roll, yaw, pitch; 
}JY901;

extern JY901 jy901;
extern uint8_t buff_JY[2];
void JY901_UART(void);


#endif

