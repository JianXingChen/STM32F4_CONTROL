#ifndef _HOLDER_H_
#define _HOLDER_H_


#include "stm32f4xx.h" 
#include "sys.h"

#define GYRO_FILTER_NUM 5

#define Z_GYRO_FILTER_NUM 30


#define GYRO_FILTER_K (1.0f /GYRO_FILTER_NUM )

struct Hold_Info
{
	
	uint8_t angle_flag ;
	
	float can_angle;
	
	float angle;
	float angle_old;
	
	float angle_delta_interal;
	
	float angle_speed;
	float angle_speed_delta_interal;
	
	float angle_target;
	float shell_out;
	float out;
	
};
extern struct Hold_Info Pitch_Hold_Info;
extern struct Hold_Info yaw_Hold_Info;	
extern uint8_t TransmitMailbox;

uint8_t Pitch_pid_cal( uint8_t flag );
uint8_t Yaw_pid_cal( uint8_t flag);




uint8_t Holder_out(uint8_t flag);



#endif

