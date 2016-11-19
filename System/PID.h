#include "stm32f4xx.h" 
#include <stdio.h>

struct PID_InitTypeDef      
{
	float Kp;               //Kp����
	float Ki;               //Ki����
	float Kd;
	float Bias;             //�������
	float Integral_Bias;    //������
	float Last_Bias;  
	float Pre_Bias; 
	float Derivative_Bias;   //���΢��
	
	float Target;       //Ŀ��
	float Out;          //���
};

float  PID_Control(float FeedBack,float Target,struct PID_InitTypeDef *PID,int Inner);


