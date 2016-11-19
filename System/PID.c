
#include "PID.h"
#include "USART3.h"


/***************************************************************************************
  * @函数描述：  限幅.
  * @入口参数：  无.
  * @返回值  :   无.
****************************************************************************************/
float AmplitudeCheck(float Angle)
{
	if(Angle>4000)  Angle=4000;
	if(Angle<-4000)  Angle=-4000;
	return Angle;
}

/***************************************************************************************
  * @函数描述：  PID处理函数.
  * @入口参数：  FeedBack----反馈值，Target----给定值，PID----待运算PID结构体，
                 Inner----1,内环；0，外环.
  * @返回值  :   无.
****************************************************************************************/
float  PID_Control(float FeedBack,float Target,struct PID_InitTypeDef *PID,int Inner)
{
	if(Inner==1)
	{
		PID->Kp=Pitch_para.core_P/1000; //   0.52;
		PID->Ki=Pitch_para.core_I/1000;//  0;
		PID->Kd=Pitch_para.core_D/1000;//  0;
	}
	else
	{
		PID->Kp=-Pitch_para.shell_P/10;//    120;
		PID->Ki=-Pitch_para.shell_I/1000;//  0;
		PID->Kd=-Pitch_para.shell_D/1000;//  0;
	}
	
	PID->Bias = Target - FeedBack;
/*--------------------------  增量式PID  --------------------*/
//	PID->Out += PID->Kp * (PID->Bias - PID->Last_Bias) +	PID->Ki * PID->Bias + PID->Kd *(PID->Bias - 2*PID->Last_Bias + PID->Pre_Bias);
//	PID->Pre_Bias = PID->Last_Bias;
/*-------------------------  绝对式PD  ---------------------*/
	PID->Integral_Bias += PID->Bias;
	PID->Derivative_Bias = PID->Bias - PID->Last_Bias;
	PID->Out = PID->Kp * PID->Bias + PID->Ki * PID->Integral_Bias  + PID->Kd *PID->Derivative_Bias;
	
	PID->Last_Bias = PID->Bias ;
	
	return AmplitudeCheck(PID->Out);
}


