
#include "spi.h"
#include "SysTickDelay.h"
#include "holder.h"



float position_e[3];
uint8_t ahrs_buf[12];
	
unsigned char spi_send(unsigned char dat);


void Check_MPU9250_Offeset(void)
{
	short tempPicth,tempYaw,tempRoll;     //单次读取陀螺仪值缓存
	float sumPicth=0,sumYaw=0,sumRoll=0;
	int i;
	for(i=0;i<200;i++)
	{
		spi_read_gyro(&tempPicth,&tempRoll,&tempYaw);
		sumPicth += tempPicth;
		sumRoll += tempRoll;
		sumYaw += tempYaw;
	}
	Pitch_Offeset = sumPicth/200;
	Yaw_Offeset = sumYaw/200;
	Roll_Offeset = sumRoll/200;
}


void spi_delay()
{
	int t;
	for(t = 0; t < 30; t++)
		__nop();
}

void spi_start()
{
	SPI_NCS_L();
	spi_delay();
}

void spi_stop()
{
	SPI_NCS_H();
}

//写数据
unsigned char spi_send(unsigned char dat)	 
{
	unsigned char i;
	unsigned char rcv = 0;
	
	for(i = 0; i < 8; i++)
	{
		spi_delay();
		SPI_SCL_L();

		if(dat & 0x80)
			SPI_SDI_H();
		else
			SPI_SDI_L();
		dat <<= 1;
				
		spi_delay();
		SPI_SCL_H();
				
		rcv <<= 1;
		if(SPI_SDO_READ())
			rcv++;
	}
	
	return rcv;
}

void spi_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/*************F4*************/
	RCC_AHB1PeriphClockCmd(RCC_AHBPeriph_SPI_CTRL_PORT, ENABLE);//
  
  GPIO_InitStructure.GPIO_Pin = SPI_SDI | SPI_SCL | SPI_NCS;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//50MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//
  GPIO_Init(GPIOA, &GPIO_InitStructure);//

	
	GPIO_InitStructure.GPIO_Pin =  SPI_SDO;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//50MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//
	
	GPIO_Init(SPI_CTRL_PORT, &GPIO_InitStructure);
	
	
	SPI_NCS_H();
	SPI_SCL_H();
	SPI_SDI_H();
}


void spi_test_comm(void)
{
	uint8_t ahrs_id;
	
	spi_init();
	
	delay_ms(20);
	do
	{
		delay_ms(10);
		spi_start();
		spi_send(0x4E);
		ahrs_id = spi_send(0xFF);
		spi_stop();
	}while(ahrs_id != 0x12);
	
}

void spi_test_read_angle(void)//读取欧拉角
{
	uint8_t i;

	spi_start();
	spi_send(0x00);
	for(i = 0; i < 12; i++)
	{
		ahrs_buf[i] = spi_send(0xFF);
	}
	spi_stop();
	
	position_e[0] = *(float *)(&ahrs_buf[0]);
	position_e[1] = *(float *)(&ahrs_buf[4]);
	position_e[2] = *(float *)(&ahrs_buf[8]);


}


uint8_t IMU_Init(void)
{
	uint8_t ahrs_id , i,flag = 0;
	
	spi_init();//spi引脚初始化
	
	delay_ms(20);
	
	for( i=0;i<5;i++ )
	{
		delay_ms(10);
		spi_start();
		spi_send(0x4E);
		ahrs_id = spi_send(0xFF);
		spi_stop();
		
		if( ahrs_id == 0x12 )
		{
			flag = 1;
			
			Check_MPU9250_Offeset();
			break;
		
		}
	}
	
	return flag;
	
}


