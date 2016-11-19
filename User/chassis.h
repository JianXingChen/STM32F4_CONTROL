#ifndef __CHASSIS_H
#define __CHASSIS_H


#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 
#include "USART3.h"

extern float Chassis_angle;//����ƫ��
extern float Chassis_angle_raw;//����ƫ��ԭʼֵ
extern float Chassis_out;

extern struct PID_PARA Chassis_para;//PID����

/*��������*/
uint8_t Chassis_Control(uint8_t flag);
#endif


