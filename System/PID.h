#include "stm32f4xx.h" 
#include <stdio.h>

struct PID_InitTypeDef      
{
	float Kp;               //Kp参数
	float Ki;               //Ki参数
	float Kd;
	float Bias;             //当次误差
	float Integral_Bias;    //误差积分
	float Last_Bias;  
	float Pre_Bias; 
	float Derivative_Bias;   //误差微分
	
	float Target;       //目标
	float Out;          //输出
};

float  PID_Control(float FeedBack,float Target,struct PID_InitTypeDef *PID,int Inner);


