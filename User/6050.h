#ifndef __MPU6050_H_
#define __MPU6050_H_
#include<stdint.h>
#include "stm32f4xx.h" 
#include "I2C.h"

//****************************************
// 定义MPU6050内部地址
//****************************************

#define	SMPLRT_DIV					0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIG			  			0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG					0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG				0x1C	//加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)
#define MOT_THR 						0x1F
#define FIFO_EN							0x23
#define I2C_MST_CTRL				0x24
#define I2C_SLV0_ADDR 			0x25
#define I2C_SLV0_REG				0x26
#define I2C_SLV0_CTRL			  0x27
#define I2C_SLV1_ADDR 			0x28
#define I2C_SLV1_REG 				0x29
#define I2C_SLV1_CTRL 			0x2A
#define I2C_SLV4_CTRL 			0x34
#define INT_PIN_CFG					0x37
#define EXT_SENS_DATA_00		0x49
#define EXT_SENS_DATA_01		0x4A
#define EXT_SENS_DATA_02		0x4B
#define EXT_SENS_DATA_03		0x4C
#define EXT_SENS_DATA_04		0x4D
#define EXT_SENS_DATA_05		0x4E
#define EXT_SENS_DATA_06		0x4F
#define I2C_SLV0_DO					0x63
#define	I2C_MST_DELAY_CTRL	0x67
#define USER_CTRL						0x6A


#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40

#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42

#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48

#define MAG_XOUT_L		0x03
#define MAG_XOUT_H		0x04
#define MAG_YOUT_L		0x05
#define MAG_YOUT_H		0x06
#define MAG_ZOUT_L		0x07
#define MAG_ZOUT_H		0x08

#define	PWR_MGMT_1		0x6B	 //电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I		  0x75	   //IIC地址寄存器(默认数值0x68，只读)

#define	MPU6050_ADDRESS  0xD0	  //陀螺地址
#define MAG_ADDRESS    0x18       //磁场地址
#define ACCEL_ADDRESS  0xD0 

#define	MPU6050_ADDRESS2  0xD2	



#define	GYRO_Y		0x45
#define	GYRO_Z		0x47


typedef struct{
				float X;
				float Y;
				float Z;}FLOAT_XYZ;

struct _float{
	      float x;
				float y;
				float z;};

struct _int16{
       int16_t x;
	     int16_t y;
	     int16_t z;};		

struct _trans{
     struct _int16 origin;  //原始值
	   struct _int16 averag;  //平均值
	   struct _int16 histor;  //历史值
	   struct _int16 quiet;   //静态值
	   struct _float radian;  //弧度值 
          };

struct _sensor{   
	struct _trans acc;
	struct _trans gyro;
              };

extern struct _sensor sensor;					

extern u8		 mpu6050_buffer[20];					
							
u8 InitMPU6050(void);					   
void MPU6050_Read(void);
void MPU6050_Dataanl(void);
void Gyro_OFFEST(void);							

void UART1_ReportIMU(void);
void MPU6050_CalOff_Acc(void);
void MPU6050_CalOff_Gyr(void);	
void HMC5883_read(void);							
		
#endif // __MPU6050_H__
