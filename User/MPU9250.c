#include "MPU9250.h"
#include "I2C.h"
#include "stm32f4xx.h"
#include "delay.h"
#include "math.h"
#include "USART3.h"
//float TX_DATA[4];  	 //��ʾ�ݻ�����
unsigned char BUF[10];       //�������ݻ�����
short T_X,T_Y,T_Z,T_T;		 //X,Y,Z�ᣬ�¶�



//��ʼ��MPU9250��������Ҫ��ο�pdf�����޸�************************
void Init_MPU9250(void)
{
	int addr;
	while(1){
		addr = Single_Read(GYRO_ADDRESS,WHO_AM_I);
		if(addr == 0x71) break;
	}
Single_Write(GYRO_ADDRESS,PWR_MGMT_1, 0x00);	//�������״̬
	Single_Write(GYRO_ADDRESS,SMPLRT_DIV, 0x07);
	Single_Write(GYRO_ADDRESS,CONFIG, 0x06);
	Single_Write(GYRO_ADDRESS,GYRO_CONFIG, 0x18);
	Single_Write(GYRO_ADDRESS,ACCEL_CONFIG, 0x01);
}

//******��ȡMPU9250����****************************************
void READ_MPU9250_ACCEL(void)
{ 

   BUF[0]=Single_Read(ACCEL_ADDRESS,ACCEL_XOUT_L); 
   BUF[1]=Single_Read(ACCEL_ADDRESS,ACCEL_XOUT_H);
   T_X=	(BUF[1]<<8)|BUF[0];
   T_X/=164; 						   //��ȡ����X������

   BUF[2]=Single_Read(ACCEL_ADDRESS,ACCEL_YOUT_L);
   BUF[3]=Single_Read(ACCEL_ADDRESS,ACCEL_YOUT_H);
   T_Y=	(BUF[3]<<8)|BUF[2];
   T_Y/=164; 						   //��ȡ����Y������
   BUF[4]=Single_Read(ACCEL_ADDRESS,ACCEL_ZOUT_L);
   BUF[5]=Single_Read(ACCEL_ADDRESS,ACCEL_ZOUT_H);
   T_Z=	(BUF[5]<<8)|BUF[4];
   T_Z/=164; 					       //��ȡ����Z������
 
}

void READ_MPU9250_GYRO(void)
{ 

   BUF[0]=Single_Read(GYRO_ADDRESS,GYRO_XOUT_L); 
   BUF[1]=Single_Read(GYRO_ADDRESS,GYRO_XOUT_H);
   T_X=	(BUF[1]<<8)|BUF[0];
   T_X/=16.4; 						   //��ȡ����X������

   BUF[2]=Single_Read(GYRO_ADDRESS,GYRO_YOUT_L);
   BUF[3]=Single_Read(GYRO_ADDRESS,GYRO_YOUT_H);
   T_Y=	(BUF[3]<<8)|BUF[2];
   T_Y/=16.4; 						   //��ȡ����Y������
   BUF[4]=Single_Read(GYRO_ADDRESS,GYRO_ZOUT_L);
   BUF[5]=Single_Read(GYRO_ADDRESS,GYRO_ZOUT_H);
   T_Z=	(BUF[5]<<8)|BUF[4];
   T_Z/=16.4; 					       //��ȡ����Z������
 
 
  // BUF[6]=Single_Read(GYRO_ADDRESS,TEMP_OUT_L); 
  // BUF[7]=Single_Read(GYRO_ADDRESS,TEMP_OUT_H); 
  // T_T=(BUF[7]<<8)|BUF[6];
  // T_T = 35+ ((double) (T_T + 13200)) / 280;// ��ȡ������¶�
}

extern float Send_data[4];

void READ_MPU9250_MAG(void)
{ 
   Single_Write(GYRO_ADDRESS,0x37,0x02);//turn on Bypass Mode 
   delay_ms(20);	
   Single_Write(MAG_ADDRESS,0x0A,0x01);
   delay_ms(20);	
   BUF[0]=Single_Read (MAG_ADDRESS,MAG_XOUT_L);
   BUF[1]=Single_Read (MAG_ADDRESS,MAG_XOUT_H);
   T_X=(BUF[1]<<8)|BUF[0];


   BUF[2]=Single_Read(MAG_ADDRESS,MAG_YOUT_L);
   BUF[3]=Single_Read(MAG_ADDRESS,MAG_YOUT_H);
   T_Y=	(BUF[3]<<8)|BUF[2];

   						   //��ȡ����Y������
	 
   BUF[4]=Single_Read(MAG_ADDRESS,MAG_ZOUT_L);
   BUF[5]=Single_Read(MAG_ADDRESS,MAG_ZOUT_H);
   T_Z=	(BUF[5]<<8)|BUF[4];
								//��ȡ����Z������
	
	Send_data[0] = atan2((double)T_Y,(double)T_X);
	Send_data[1] = T_X;
	Send_data[2] = T_Y;
 					       
}

void mag(void)
{
	int i;  float sumX=0,sumY=0;
	static float MAG_dataX[8]={0,0,0,0,0,0,0,0},MAG_dataY[8]={0,0,0,0,0,0,0,0};
	READ_MPU9250_MAG();

		for(i=7;i>=1;i--)
		{
			MAG_dataX[i] = MAG_dataX[i-1];
			sumX+=MAG_dataX[i];
		}
		MAG_dataX[0] = T_X;

		for(i=7;i>=1;i--)
		{
			MAG_dataY[i] = MAG_dataY[i-1];
			sumY+=MAG_dataY[i];
		}
		MAG_dataY[0] = T_Y;

}
