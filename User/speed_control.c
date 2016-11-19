
#include "speed_control.h"
#include "CAN1.h"

#include "holder.h"


#define SPEED_FILTER_NUM 10
#define SPEED_OUT_MAX 5000


float wheel_position[4];
float wheel_position_old[4];
float read_speed[4];

float Target_speed[4] = {0,0,0,0};//��ֻ���Ӹ����ٶ�
	
float speed_filter_buff[4][SPEED_FILTER_NUM];
	
int16_t SPEED_COT_CNT = 0;

int16_t speed_start_flag = 0;

struct PID_PARA Wheel_para = 
{
	1500,0,0,
	0,0,0
};//�����ٶȱջ�


void Wheel_out(uint8_t out_mode , float * speed_list ,uint8_t wheel_id ,  int16_t speed );//���̵�����


void Speed_read(void)
{
	
}


uint8_t Speed_control(float * target,  uint8_t flag)
{
	float d_error ,cur_speed;
	
	float p_part,d_part,i_part;
	
	float wheel_out[4];
	
	static float speed_error[4][2];
		
	static int8_t num = 0;
	
	int8_t i,wheel_cnt;
	
	float sum[4];
	
	/***********NUM1 �����ĸ����ӵĵ�����*************/
	for( wheel_cnt = 0;wheel_cnt<4;wheel_cnt++ )
	{
		//1.��ȡ��ת���ٶ�
		read_speed[wheel_cnt] = wheel_position[wheel_cnt] -  wheel_position_old[wheel_cnt];
		
		if( abs(read_speed[wheel_cnt]) > 3000 )
		{
			if( read_speed[wheel_cnt] < 0 )
			{
				read_speed[wheel_cnt] = wheel_position[wheel_cnt] + 8191 - wheel_position_old[wheel_cnt];//�����ٶ�
			}
			else
			{
				read_speed[wheel_cnt] =  wheel_position[wheel_cnt] - wheel_position_old[wheel_cnt] - 8191;//�����ٶ�
			}
		}
		
			/*�����ϴεĵ��λ��*/
		wheel_position_old[wheel_cnt] = wheel_position[wheel_cnt];
		
		//2. �Բɼ����ٶȽ����˲�
		
		speed_filter_buff[wheel_cnt][num] = read_speed[wheel_cnt];
		sum[wheel_cnt] = 0;
		for( i=0;i< SPEED_FILTER_NUM;i++)
		{
			sum[wheel_cnt] +=  speed_filter_buff[wheel_cnt][i];
		}
		read_speed[wheel_cnt] = sum[wheel_cnt] / SPEED_FILTER_NUM;
		
		//3. ����PID���
			/*(1)����΢С�ٶȶ���*/
		if( abs(read_speed[wheel_cnt])<=0.5 )
		{
			read_speed[wheel_cnt] = 0;
		}
		
			/*(2)����PID*/
		speed_error[wheel_cnt][0] = speed_error[wheel_cnt][1];
		speed_error[wheel_cnt][1] = (*(target+wheel_cnt)) - read_speed[wheel_cnt];

		p_part = ( Wheel_para.shell_P ) * 0.1 * speed_error[wheel_cnt][1];//P PART ��С10��
		
		d_part = ( Wheel_para.shell_D ) * ( speed_error[wheel_cnt][1] - speed_error[wheel_cnt][0] );// D PART
		
		wheel_out[wheel_cnt] = p_part + d_part;
		
			/*(3)����޷�*/
		if( wheel_out[wheel_cnt]  > SPEED_OUT_MAX )
		{
			wheel_out[wheel_cnt]  = SPEED_OUT_MAX;
		}

		else if( wheel_out[wheel_cnt]  < -SPEED_OUT_MAX )
		{
			wheel_out[wheel_cnt]  = -SPEED_OUT_MAX;
		}
	}
	
	num = (num + 1 )%SPEED_FILTER_NUM;//����
	
	
	/***********NUM2 �������ֵ�����*************/
	
	
	//1.��������ʱ�ĸ���
	if( speed_start_flag < 20 )
	{
		speed_start_flag ++;
		
		return 0;
	}
	
	else
	{
		//��������
		if( flag == DISABLE_MOTOR_OUT)
		{
			//Wheel_speed(2 , 0);
			Wheel_out( NO_OUT , wheel_out,0,0 );
		}
		
		else
		{
			//Wheel_speed(2 , wheel_out[2]);
			Wheel_out( ALL_OUT , wheel_out,0,0 );
		}
		
			
		//ʾ�����۲�
		#if 0
		Send_data[0] = wheel_out[0];
		Send_data[1] = wheel_out[1];
		Send_data[2] = wheel_out[2];
		Send_data[3] = wheel_out[3];
		#endif
		
		return 1;
	}

}

/*
 * ��������Wheel_out
 * ����  �������ĸ����̵�������
 * ����  ��1. out_mode
		��ѡ������NO_OUT  : �޵�����ģʽ
							ALL_OUT  :���е�����ģʽ
							SIGNAL_OUT	:����������ģʽ
							
					2. speed_list :����������
					3. wheel_id:������  ��ѡ������ 0-3
					4. speed	:��ֻ���ָ���ٶ�

 * ���  : ��
 * ����  ���ڲ�����
 */	 
void Wheel_out(uint8_t out_mode , float * speed_list ,uint8_t wheel_id ,  int16_t speed )
{
	int16_t i16_speed_list[4];
	int8_t i;
	
	uint16_t t;
	
	CanTxMsg TxMessage;
	
	TxMessage.StdId = 0x200;	
	TxMessage.IDE=CAN_ID_STD;					 //STANDARD MODE
  TxMessage.RTR=CAN_RTR_DATA;				 //���͵�������
  TxMessage.DLC=8;							 //���ݳ���Ϊ2�ֽ�
	

	
	if( out_mode == ALL_OUT )//���е�����ģʽ
	{
		for( i=0;i<4;i++ )
		{
			i16_speed_list[i] = (int16_t)(*(speed_list+i));
			
			TxMessage.Data[i*2+0] = (uint8_t)(i16_speed_list[i]>>8);
			TxMessage.Data[i*2+1] = (uint8_t)(i16_speed_list[i]);
		}
	}
	
	else if( out_mode == SIGNAL_OUT )//����������ģʽ
	{
		TxMessage.Data[0]=0;
		TxMessage.Data[1]=0;
		TxMessage.Data[2]=0;
		TxMessage.Data[3]=0;
		TxMessage.Data[4]=0;
		TxMessage.Data[5]=0;
		TxMessage.Data[6]=0;
		TxMessage.Data[7]=0;
		
		TxMessage.Data[wheel_id*2+0] = (uint8_t)(speed>>8);
		TxMessage.Data[wheel_id*2+1] = (uint8_t)(speed);
	}
	
	else
	{
		TxMessage.Data[0]=0;
		TxMessage.Data[1]=0;
		TxMessage.Data[2]=0;
		TxMessage.Data[3]=0;
		TxMessage.Data[4]=0;
		TxMessage.Data[5]=0;
		TxMessage.Data[6]=0;
		TxMessage.Data[7]=0;
	}
	

	TransmitMailbox=CAN_Transmit(CAN1,&TxMessage);
	t=0;
	while((CAN_TransmitStatus(CAN1,TransmitMailbox)!=CANTXOK)&&(t!= 0xFF))
	{
		t++;
	}

}