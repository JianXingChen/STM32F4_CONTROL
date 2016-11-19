#include "vision.h"
#include "UART2_CAMERA.h"

#define Camera_Filter_NUM  10

#define Camera_Filter_k  (1.0f / Camera_Filter_NUM);

float Camera_Filter_Buff[2][Camera_Filter_NUM];



void Vision_target_filter(void)
{
	static uint8_t F_NUM = 0;
	int16_t i ;
	float sum0 , sum1;
	
	Camera_Filter_Buff[0][F_NUM] = yaw_f;
	
	sum0 = 0;
	sum1 = 0;
	for( i = 0;i< Camera_Filter_NUM;i++)
	{
		sum0 += Camera_Filter_Buff[0][i];
	}
	yaw_f = sum0 * Camera_Filter_k;
	
	
	F_NUM  = (F_NUM + 1) % Camera_Filter_NUM;
	
	
	
}
