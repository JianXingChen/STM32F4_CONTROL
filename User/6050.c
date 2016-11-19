#include "6050.h"
 #include "holder.h"
#include "USART3.h"
#include "delay.h"
#include "math.h"

#define UART_Put_Char(x)  UART1_Put_Char(x)
#define uint8_t u8
#define uint16_t u16

struct _sensor sensor;	


u8		 mpu6050_buffer[20];					

u8		ACC_OFFSET_OK = 0;
u8    ACC_OFFSET=1,GYRO_OFFSET=1;

extern vs16 Moto_duty[4];
/***************************************************************************************
  * @函数描述：  MPU6050初始化
  * @入口参数：  无.
  * @返回值  :   初始化完成标志，0----完成，!0----未完成.
****************************************************************************************/
u8 InitMPU6050(void)
{
	unsigned char Addr;
	u8 date,count=0;
	do
	{
		Addr=Single_Read(MPU6050_ADDRESS,WHO_AM_I); //count++;
		//if(count>8) return 0;                                      //MPU6050连接失败
	}while(Addr!=0x68);
	
	count = 0;
//	do
//	{
//		Addr=Single_Read(MPU6050_ADDRESS2,WHO_AM_I); //count++;
//		//if(count>8) return 0;                                      //MPU6050连接失败
//	}while(Addr!=0x68);
	
	
	//初始化MPU6050 1
	date = Single_Write(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);  	   //解除休眠状态0x00
	delay_ms(500);
	
	
	date = Single_Write(MPU6050_ADDRESS, PWR_MGMT_1, 0x00);  	   //解除休眠状态0x00
	delay_ms(50);
	date += Single_Write(MPU6050_ADDRESS, SMPLRT_DIV, 0x00);     //采样频率（1KHz）
	delay_ms(50);
	date += Single_Write(MPU6050_ADDRESS, CONFIG, 0x01);         //低通滤波0x00
	delay_ms(50);
	date += Single_Write(MPU6050_ADDRESS, GYRO_CONFIG, 0x10);    //陀螺仪量程 
	delay_ms(50);
	date += Single_Write(MPU6050_ADDRESS, ACCEL_CONFIG, 0x09);   //加速度量程 
	delay_ms(50);
	
	return date;
}
/***************************************************************************************
  * @函数描述：  读取所有加速度计、陀螺仪
  * @入口参数：  无.
  * @返回值  :   无.
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
	
	
	mpu6050_buffer[12]=Single_Read(MPU6050_ADDRESS, GYRO_ZOUT_H);//传感器2的z轴陀螺仪
	mpu6050_buffer[13]=Single_Read(MPU6050_ADDRESS, GYRO_ZOUT_L);//
	
	HMC5883_read();
	
}


/***************************************************************************************
  * @函数描述：  陀螺仪零点校准
  * @入口参数：  无.
  * @返回值  :   无.
****************************************************************************************/

void Gyro_OFFEST(void)
{
   int cnt_g=1000;
	 int cnt = cnt_g;
	 float  tempgx=0,tempgy=0,tempgz=0;
	 sensor.gyro.averag.x=0;    //零点偏移清零
	 sensor.gyro.averag.y=0;  
	 sensor.gyro.averag.z=0;
	 while(cnt_g--)       //循环采集1000次   求平均
	 {
		  MPU6050_Read();
		 
		  sensor.gyro.origin.x = ((((int16_t)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9]);
	    sensor.gyro.origin.y = ((((int16_t)mpu6050_buffer[10]) << 8)| mpu6050_buffer[11]);
	    sensor.gyro.origin.z = ((((int16_t)mpu6050_buffer[12]) << 8)| mpu6050_buffer[13]);
      tempgx+= sensor.gyro.origin.x;
			tempgy+= sensor.gyro.origin.y;
			tempgz+= sensor.gyro.origin.z;
   }
	 sensor.gyro.quiet.x=tempgx/cnt;
	 sensor.gyro.quiet.y=tempgy/cnt;
	 sensor.gyro.quiet.z=tempgz/cnt;
}

