#include "6050.h"

#define RtA 		57.324841		
#define AtR    	0.0174533			
#define Acc_G 	0.0011963			
#define Gyro_G 	0.03051756		
#define Gyro_Gr	0.0005327   

#define UART_Put_Char(x)  UART1_Put_Char(x)
#define uint8_t u8
#define uint16_t u16

struct _sensor sensor;	
u8		 mpu6050_buffer[14];					

u8		ACC_OFFSET_OK = 0;
u8    ACC_OFFSET=1,GYRO_OFFSET=1;

extern vs16 Moto_duty[4];
extern float Scope3;
/***************************************************************************************
  * @����������  MPU6050��ʼ��
  * @��ڲ�����  ��.
  * @����ֵ  :   ��ʼ����ɱ�־��0----��ɣ�!0----δ���.
****************************************************************************************/
u8 InitMPU6050(void)
{
	unsigned char Addr;
	u8 date,count=0;
	do
	{
		Addr=Single_Read(MPU6050_ADDRESS,WHO_AM_I); count++;
		if(count>8) return 0;                                      //MPU6050����ʧ��
	}while(Addr!=0x68);
	date = Single_Write(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);  	   //�������״̬
	date += Single_Write(MPU6050_ADDRESS, SMPLRT_DIV, 0x07);     //����Ƶ�ʣ�1KHz��
	date += Single_Write(MPU6050_ADDRESS, CONFIG, 0x00);         //��ͨ�˲�
	date += Single_Write(MPU6050_ADDRESS, GYRO_CONFIG, 0x08);    //���������� 
	date += Single_Write(MPU6050_ADDRESS, ACCEL_CONFIG, 0x08);   //���ٶ����� 
	
	return date;
}
/***************************************************************************************
  * @����������  ��ȡ���м��ٶȼơ�������
  * @��ڲ�����  ��.
  * @����ֵ  :   ��.
****************************************************************************************/
void MPU6050_Read(void)
{
	mpu6050_buffer[0]=Single_Read(MPU6050_ADDRESS, ACCEL_XOUT_H);
	mpu6050_buffer[1]=Single_Read(MPU6050_ADDRESS, ACCEL_XOUT_L);
	
	
	mpu6050_buffer[2]=Single_Read(MPU6050_ADDRESS, ACCEL_YOUT_H);
	mpu6050_buffer[3]=Single_Read(MPU6050_ADDRESS, ACCEL_YOUT_L);
	
	mpu6050_buffer[4]=Single_Read(MPU6050_ADDRESS, ACCEL_ZOUT_H);
	mpu6050_buffer[5]=Single_Read(MPU6050_ADDRESS, ACCEL_ZOUT_L);
	
	mpu6050_buffer[8]=Single_Read(MPU6050_ADDRESS, GYRO_XOUT_H);
	mpu6050_buffer[9]=Single_Read(MPU6050_ADDRESS, GYRO_XOUT_L);
	
	
	mpu6050_buffer[10]=Single_Read(MPU6050_ADDRESS, GYRO_YOUT_H);
	mpu6050_buffer[11]=Single_Read(MPU6050_ADDRESS, GYRO_YOUT_L);
	
	
	mpu6050_buffer[12]=Single_Read(MPU6050_ADDRESS, GYRO_ZOUT_H);
	mpu6050_buffer[13]=Single_Read(MPU6050_ADDRESS, GYRO_ZOUT_L);
	
}


/***************************************************************************************
  * @����������  ���������У׼
  * @��ڲ�����  ��.
  * @����ֵ  :   ��.
****************************************************************************************/

void Gyro_OFFEST(void)
{
   int cnt_g=1000;
	 int32_t  tempgx=0,tempgy=0,tempgz=0;
	 sensor.gyro.averag.x=0;    //���ƫ������
	 sensor.gyro.averag.y=0;  
	 sensor.gyro.averag.z=0;
	 while(cnt_g--)       //ѭ���ɼ�2000��   ��ƽ��
	 {
		  MPU6050_Read();
		 
		  sensor.gyro.origin.x = ((((int16_t)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9]);
	    sensor.gyro.origin.y = ((((int16_t)mpu6050_buffer[10]) << 8)| mpu6050_buffer[11]);
	    sensor.gyro.origin.z = ((((int16_t)mpu6050_buffer[12]) << 8)| mpu6050_buffer[13]);
      tempgx+= sensor.gyro.origin.x;
			tempgy+= sensor.gyro.origin.y;
			tempgz+= sensor.gyro.origin.z;
   }
	 sensor.gyro.quiet.x=tempgx/1000;
	 sensor.gyro.quiet.y=tempgy/1000;
	 sensor.gyro.quiet.z=tempgz/1000;
}
/***************************************************************************************
  * @����������  ��ȡ������ԭʼ����
  * @��ڲ�����  ����ȡ��.
  * @����ֵ  :   ��ǰ���ٶ�.
****************************************************************************************/
float Get_Gyro_data(u8 TargetAxis)
{
	int i;  float sum=0;
	int16_t buffer=0;
	u8 Axis=TargetAxis,tempH=0,tempL=0;
	static float GyroY_data[12]={0,0,0,0,0,0,0,0,0,0,0,0},GyroZ_data[12]={0,0,0,0,0,0,0,0,0,0,0,0};

	tempH=Single_Read(MPU6050_ADDRESS, Axis);
	tempL=Single_Read(MPU6050_ADDRESS, Axis+1);

	if(Axis==GYRO_Y)
	{
		buffer = ((((int16_t)tempH) << 8)|tempL) - sensor.gyro.quiet.y;
	}
	else
	{
		buffer = ((((int16_t)tempH) << 8)|tempL) - sensor.gyro.quiet.z;
	}
	Scope3 = buffer;
	
	if((buffer>-30)&(buffer<30)) buffer=0;
	
	if(Axis==GYRO_Y)
	{
		for(i=11;i>=1;i--)
		{
			GyroY_data[i] = GyroY_data[i-1];
			sum+=GyroY_data[i];
		}
		GyroY_data[0] = buffer;
		return ((sum+GyroY_data[0])/12);
	}
	else
	{
		for(i=11;i>=1;i--)
		{
			GyroZ_data[i] = GyroZ_data[i-1];
			sum+=GyroZ_data[i];
		}
		GyroZ_data[0] = buffer;
		return ((sum+GyroZ_data[0])/12);
	}
}

float getGyroY(void)
{
	int16_t buff[5];
	int16_t max,min,sum;
	int i;
	float out;
	for(i=0;i<5;i++)
	{
		mpu6050_buffer[12]=Single_Read(MPU6050_ADDRESS, GYRO_YOUT_H);
		mpu6050_buffer[13]=Single_Read(MPU6050_ADDRESS, GYRO_YOUT_L);
		buff[i] = ((((int16_t)mpu6050_buffer[12]) << 8)| mpu6050_buffer[13]) - sensor.gyro.quiet.y;
	}
	max = buff[0];
	min = buff[0];
	sum = 0;
	for( i = 0;i<5;i++ )
	{
		if( buff[i] > max  )
		{
			max = buff[i];
		}
		if( buff[i]<min )
		{
			min = buff[i];
		}
		sum += buff[i];
	}
	out = (sum-max-min)*0.3333;
	if((out>-20)&(out<20)) out=0;
	return (out);
}

float getGyroZ(void)
{
	int16_t buff[5];
	int16_t max,min,sum;
	int i;
	float out;
	for(i=0;i<5;i++)
	{
		mpu6050_buffer[12]=Single_Read(MPU6050_ADDRESS, GYRO_ZOUT_H);
		mpu6050_buffer[13]=Single_Read(MPU6050_ADDRESS, GYRO_ZOUT_L);
		buff[i] = ((((int16_t)mpu6050_buffer[12]) << 8)| mpu6050_buffer[13]) - sensor.gyro.quiet.z;
	}
	max = buff[0];
	min = buff[0];
	sum = 0;
	for( i = 0;i<5;i++ )
	{
		if( buff[i] > max  )
		{
			max = buff[i];
		}
		if( buff[i]<min )
		{
			min = buff[i];
		}
		sum += buff[i];
	}
	out = (sum-max-min)*0.3333;
	if((out>-20)&(out<20)) out=0;
	return (out);
}




