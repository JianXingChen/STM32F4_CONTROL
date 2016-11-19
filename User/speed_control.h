#ifndef __SPEED_CONTROL_H
#define	__SPEED_CONTROL_H

#include "stm32f4xx_conf.h"
#include "USART3.h"



#define ENABLE_MOTOR_OUT 1
#define DISABLE_MOTOR_OUT 0

#define CAR_MOTOR_NUM_0 0
#define CAR_MOTOR_NUM_1 1
#define CAR_MOTOR_NUM_2 2
#define CAR_MOTOR_NUM_3 3

#define NO_OUT 0
#define ALL_OUT 1
#define SIGNAL_OUT 2


extern float wheel_position[4];

extern float wheel_position_old[4];

extern int16_t SPEED_COT_CNT;

extern float read_speed[4];

extern struct PID_PARA Wheel_para;//轮子速度闭环

extern int16_t speed_start_flag;

extern float Target_speed[4];//四只轮子给定速度
	
uint8_t Speed_control(float * target,  uint8_t flag);

void Wheel_out(uint8_t out_mode , float * speed_list ,uint8_t wheel_id ,  int16_t speed );



#endif


