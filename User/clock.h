#ifndef _CLOCK_H
#define _CLOCK_H
#include "sys.h"

void TIM3_Int_Init(void);

extern int32_t clock_cnt;

extern int8_t sys_flag ;//系统运行标志位  0： 正常


#define MPU6050_ERROR 1
#define MPU6050_OK 0

#define HOLD_INIT_OK 1
#define HOLD_INIT_NOT_OK 0

struct T_SYS_FLAG
{
	uint8_t mpu6050;//系统运行标志位  0： 正常
	
	uint8_t Hold_init;

	int32_t Hold_init_cnt;
	
	int32_t clock_cnt;
	
};

extern struct T_SYS_FLAG Car_sys;





#endif
