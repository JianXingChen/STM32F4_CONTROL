#ifndef _UART2_CAMERA_H_
#define _UART2_CAMERA_H_


#include "stm32f4xx.h" 
#include "sys.h"

void UART2_CAMERA_Configuration(void);

extern float yaw_diff , pitch_diff;
extern uint32_t Camera_cnt;
extern float yaw_f , pitch_f;
extern float yaw_diff_inter ;

#endif

