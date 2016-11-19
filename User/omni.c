
#include "omni.h"
#include "chassis.h"
#include "speed_control.h"
#include "USART1.h"

#define SPEED_MAX 150
#define SPEED_K 0.01



u8 OmniData[8];

float AmplitudeCheck(float Angle , float limit)
{
	if(Angle>limit)  Angle=limit;
	if(Angle<-limit)  Angle=-limit;
	return Angle;
}



/*
 * 函数名：omni
 * 描述  ：处理收到的遥控器信息，将摇杆位置转化为电机的电流值
 * 输入  : 无
 * 输出  : 无
 * 调用  ：外部调用
 */
void omni(uint8_t flag)
{
	
	u8 i;
	CanTxMsg TxMessage;
	
	
	short x=7,y=7;//发送数据电流值范围 -5000~+5000, 10000/1320=8	x是速度	                          
	short	direction[4]={0,0,0,0};//电机1，电机2，电机3，电机4	
	
  short	    f0=RC_Ctl.rc.ch1-1024,
				    r0=RC_Ctl.rc.ch0-1024,
				    y0=0; //-720--+720      90 * 0.1 * 80
	

	
	y0 = Chassis_out;
		
	direction[0]=x*(-f0+r0)+y*y0;
	direction[1]=x*(f0+r0)+y*y0;
	direction[2]=x*(f0-r0)+y*y0;
	direction[3]=x*(-f0-r0)+y*y0;
	
	for(i=0;i<4;i++)//限幅输出
	{
		direction[i] = AmplitudeCheck( SPEED_K * direction[i] , SPEED_MAX);
		
		Target_speed[i] =  direction[i];
	}	
	
	
	
//	
//	#if 1
//	Send_data[0] = direction[0];
//	Send_data[1] = direction[1];
//	Send_data[2] = direction[2];
//	Send_data[3] = direction[3];
//	#endif
//	
//	
//	//2. 电机输出
//	TxMessage.StdId = 0X200;            //本地ID
//	TxMessage.IDE = CAN_ID_STD; 
//	TxMessage.RTR = CAN_RTR_DATA;
//	TxMessage.DLC = 8; 
//	

//	if( flag == 1 )//正常输出
//	{
//		for( i = 0;i<4;i++ )
//		{
//			TxMessage.Data[i*2+0] = (uint8_t)( direction[i] >> 8 );
//			TxMessage.Data[i*2+1] = (uint8_t)( direction[i] );
//		}
//	
//	}
//	else///无输出
//	{
//		for( i = 0;i<4;i++ )
//		{
//			TxMessage.Data[i*2+0] = 0;
//			TxMessage.Data[i*2+1] = 0;
//		}
//	}
//	
//	CAN_Transmit(CAN1,&TxMessage);//确认发送

	
	
//	for(i=0;i<4;i++)
//	{
//		direction[i]=AmplitudeCheck( SPEED_K *direction[i]);
//		
//		Target_speed[i] = direction[i];	
//	}

}

