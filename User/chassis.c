#include "chassis.h"

//#define CHASSIS_VAL  2480
#define CHASSIS_VAL  4264
//#define CHASSIS_TO_ANGLE_K 0.04186
#define CHASSIS_TO_ANGLE_K 0.0504

float Chassis_angle;//����ƫ��
float Chassis_angle_raw;//����ƫ��ԭʼֵ
float Chassis_out;

struct PID_PARA Chassis_para = 
{
	105 , 0 , 0,
	0, 0 , 0
}
	;//PID����


//����PID����
uint8_t Chassis_Control(uint8_t flag)
{
	float p_part , d_part;
	
	//1. ��ȡ�Ƕ�
	Chassis_angle = (Chassis_angle_raw - CHASSIS_VAL) * CHASSIS_TO_ANGLE_K;//
	
	//2. ����PID����
	p_part = Chassis_para.shell_P * 0.1* (0 - Chassis_angle);
	//p_part = 1 * (0 - Chassis_angle);
	Chassis_out = p_part;
		
	#if 0
	Send_data[0] = Chassis_angle;
	Send_data[1] = Chassis_out;
	#endif
	
	return 1;
}