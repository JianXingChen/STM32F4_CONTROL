
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
 * ��������omni
 * ����  �������յ���ң������Ϣ����ҡ��λ��ת��Ϊ����ĵ���ֵ
 * ����  : ��
 * ���  : ��
 * ����  ���ⲿ����
 */
void omni(uint8_t flag)
{
	
	u8 i;
	CanTxMsg TxMessage;
	
	
	short x=7,y=7;//�������ݵ���ֵ��Χ -5000~+5000, 10000/1320=8	x���ٶ�	                          
	short	direction[4]={0,0,0,0};//���1�����2�����3�����4	
	
  short	    f0=RC_Ctl.rc.ch1-1024,
				    r0=RC_Ctl.rc.ch0-1024,
				    y0=0; //-720--+720      90 * 0.1 * 80
	

	
	y0 = Chassis_out;
		
	direction[0]=x*(-f0+r0)+y*y0;
	direction[1]=x*(f0+r0)+y*y0;
	direction[2]=x*(f0-r0)+y*y0;
	direction[3]=x*(-f0-r0)+y*y0;
	
	for(i=0;i<4;i++)//�޷����
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
//	//2. ������
//	TxMessage.StdId = 0X200;            //����ID
//	TxMessage.IDE = CAN_ID_STD; 
//	TxMessage.RTR = CAN_RTR_DATA;
//	TxMessage.DLC = 8; 
//	

//	if( flag == 1 )//�������
//	{
//		for( i = 0;i<4;i++ )
//		{
//			TxMessage.Data[i*2+0] = (uint8_t)( direction[i] >> 8 );
//			TxMessage.Data[i*2+1] = (uint8_t)( direction[i] );
//		}
//	
//	}
//	else///�����
//	{
//		for( i = 0;i<4;i++ )
//		{
//			TxMessage.Data[i*2+0] = 0;
//			TxMessage.Data[i*2+1] = 0;
//		}
//	}
//	
//	CAN_Transmit(CAN1,&TxMessage);//ȷ�Ϸ���

	
	
//	for(i=0;i<4;i++)
//	{
//		direction[i]=AmplitudeCheck( SPEED_K *direction[i]);
//		
//		Target_speed[i] = direction[i];	
//	}

}

