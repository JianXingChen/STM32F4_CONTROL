#ifndef _IMU_H_
#define _IMU_H_
#include "stm32f4xx.h"


typedef struct{
				float ROL;
				float PIT;
				float YAW;}T_FLOAT_ANGEL;

extern T_FLOAT_ANGEL Q_ANGLE;				
				
struct _angle{
        float pitch;
				float roll;
        float yaw;};


extern struct _angle angle;
				
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az,float mx,float my,float mz);
				
		void Get_Attitude(void);		
				
#endif
