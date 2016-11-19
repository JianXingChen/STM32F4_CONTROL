
#ifndef _SPI_H_
#define _SPI_H_

#include "stm32f4xx.h" 
#include "sys.h"



/*自己定义*/
#define SPI_CTRL_PORT GPIOA

#define RCC_AHBPeriph_SPI_CTRL_PORT RCC_AHB1Periph_GPIOA


#define SPI_SCL	GPIO_Pin_6
#define SPI_SDI	GPIO_Pin_8
#define SPI_SDO	GPIO_Pin_5//In
#define SPI_NCS	GPIO_Pin_7



#define SPI_NCS_PIN PAout(7)
#define SPI_NCS_H()	SPI_NCS_PIN = 1
#define SPI_NCS_L()	SPI_NCS_PIN = 0



#define SPI_SCL_PIN	PAout(6)
#define SPI_SCL_H()	SPI_SCL_PIN = 1
#define SPI_SCL_L()	SPI_SCL_PIN = 0



#define SPI_SDI_PIN	PAout(8)
#define SPI_SDI_H()	SPI_SDI_PIN = 1
#define SPI_SDI_L()	SPI_SDI_PIN = 0


#define SPI_SDO_PIN PAin(5)
#define SPI_SDO_READ() SPI_SDO_PIN



void spi_init(void);
void spi_test_comm(void);
void spi_start(void);

void spi_stop(void);
unsigned char spi_send(unsigned char dat)	;

void spi_test_read_angle(void);//读取欧拉角
uint8_t IMU_Init(void);


extern float position_e[3];//存储欧拉角
extern uint8_t ahrs_buf[12];
#endif //_SPI_H_
