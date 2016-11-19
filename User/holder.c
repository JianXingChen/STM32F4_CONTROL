#include "holder.h"
#include "spi.h"
#include "USART3.h"
#include "6050.h"
#include "clock.h"
#include "math.h"
#include "Prepare_data.h"


/****宏定义***/

#define PITCH_VAL 1710
#define PITCH_ANGLE_K 0.06122


#define GYRO_GAP 15
#define PITCH_OUT_MAX 4500
#define YAW_OUT_MAX 4500


/*******全局变量******/


struct PID_PARA Pitch_para=
{
	300,0,50,
	220,0,0
};



struct PID_PARA Yaw_para = 
{
	300,0,50,
	220,0,0
};



uint8_t TransmitMailbox;


struct Hold_Info Pitch_Hold_Info;
struct Hold_Info yaw_Hold_Info;





uint8_t Pitch_pid_cal( uint8_t flag )
{
	float core_p_part,core_d_part;
	float core_delta_new,core_d_delta;
	
	float shell_delta_new,shell_d_delta;
		
	float shell_p_part , shell_d_part;
	
	//1.shell
	shell_delta_new = Pitch_Hold_Info.angle_target - Pitch_Hold_Info.angle;
	
	shell_p_part = shell_delta_new * Pitch_para.shell_P;//P SHELL
	
	shell_d_part = Pitch_Hold_Info.angle_speed * Pitch_para.shell_D * 0.01;//D SHELL
	
	Pitch_Hold_Info.shell_out = shell_p_part + shell_d_part;
	
	//2.core
	core_delta_new = Pitch_Hold_Info.shell_out  - Pitch_Hold_Info.angle_speed;
	//core_delta_new = 0  - Pitch_Hold_Info.angle_speed;
	
	core_p_part = Pitch_para.core_P *  0.01f *core_delta_new;// CORE P
	

	if( flag  == 1 )
	{
		Pitch_Hold_Info.out = core_p_part;
	
		Pitch_Hold_Info.out = Pitch_Hold_Info.out - 700;
	}
	
	else
	{
			Pitch_Hold_Info.out = 0;
	}
	
	#if 0

	Send_data[0] = Pitch_Hold_Info.angle * 100;

	Send_data[1] = Pitch_Hold_Info.out;

	Send_data[2] = shell_p_part ;

	Send_data[3] = shell_d_part;

	#endif
	
	
	
	return 1;
	
	

}


uint8_t Yaw_pid_cal( uint8_t flag)
{
	float core_p_part,core_d_part;
	float core_delta_new,core_d_delta;
	
	float shell_delta_new,shell_d_delta;
		
	float shell_p_part , shell_d_part;
	
	//1.shell
	shell_delta_new = yaw_Hold_Info.angle_target - yaw_Hold_Info.angle;
	
	shell_p_part = shell_delta_new * Yaw_para.shell_P;//P SHELL
	
	shell_d_part = yaw_Hold_Info.angle_speed * Yaw_para.shell_D * 0.01;//D SHELL
	
	yaw_Hold_Info.shell_out = shell_p_part + shell_d_part;
	
	//2.core
	
	core_delta_new = yaw_Hold_Info.shell_out  - yaw_Hold_Info.angle_speed;
	
	core_p_part = Yaw_para.core_P *  0.01f *core_delta_new;// CORE P
	

	if( flag == 1 )
	{
			yaw_Hold_Info.out = core_p_part;
	
	}
	else
	{
			yaw_Hold_Info.out = 0;
	
	}
	
	#if 0

	Send_data[0] = yaw_Hold_Info.angle;

	Send_data[1] = yaw_Hold_Info.out;
	
	Send_data[2] = yaw_Hold_Info.angle_speed;

	#endif
	
	
	return 1;
}





uint8_t Holder_out(uint8_t flag)
{
	uint8_t c1,c2,c3,c4;
	
	int16_t yaw_out_s16;
  int16_t pitch_out_s16;
	CanTxMsg TxMessage;
	uint16_t t;
	

	
	TxMessage.StdId = 0x1ff;	
	TxMessage.IDE=CAN_ID_STD;					 //STANDARD MODE
	TxMessage.RTR=CAN_RTR_DATA;				 //发送的是数据
	TxMessage.DLC=8;							 //数据长度为2字节

	//转换到int16类型，方便后续CAN发送
	yaw_out_s16 = (int16_t)yaw_Hold_Info.out;
	pitch_out_s16 = (int16_t)Pitch_Hold_Info.out;
	
	
	//yaw轴输出限幅
	if( yaw_out_s16 < -YAW_OUT_MAX)
	{
		yaw_out_s16 = -YAW_OUT_MAX;
	}
	else if( yaw_out_s16 > YAW_OUT_MAX )
	{
		yaw_out_s16 = YAW_OUT_MAX;
	}
	
	//Pitch轴限幅
	if( pitch_out_s16 < -PITCH_OUT_MAX)
	{
		pitch_out_s16 = -PITCH_OUT_MAX;
	}
	else if( pitch_out_s16 > PITCH_OUT_MAX )
	{
		pitch_out_s16 = PITCH_OUT_MAX;
	}
	
	
	if( flag == 1 )
	{
		c1 = (uint8_t)( pitch_out_s16 >> 8);
	
		c2	 = (uint8_t)pitch_out_s16;
		
		c3 = (uint8_t)( yaw_out_s16>> 8);
	
		c4	 = (uint8_t)yaw_out_s16;	
	}
	else
	{
		c1= 0 ;
		c2 = 0;
		c3 = 0;
		c4 = 0;
	}
	
	
  TxMessage.Data[0]=c3;
  TxMessage.Data[1]=c4 ; 
	

	TxMessage.Data[2]=c1;
	TxMessage.Data[3]=c2;
	
	TxMessage.Data[4]=0;
	TxMessage.Data[5]=0;
	TxMessage.Data[6]=0;
	TxMessage.Data[7]=0;
	//CAN_Transmit(CAN1, &TxMessage);
	
	TransmitMailbox=CAN_Transmit(CAN1,&TxMessage);
	t=0;
	while((CAN_TransmitStatus(CAN1,TransmitMailbox)!=CANTXOK)&&(t!= 0xFF))
	{
		t++;
	}

	
	//示波器发送显示
	#if 0
	Send_data[3] = yaw_out_s16;
	#endif
	
	
	return 1;
}



