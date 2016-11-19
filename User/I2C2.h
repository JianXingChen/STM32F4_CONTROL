#ifndef __I2C2_H__
#define __I2C2_H__
#include "stm32f4xx.h"

#define FALSE 0
#define TRUE  1

//#define SCL_PIN GPIO_Pin_8
//#define SDA_PIN GPIO_Pin_9

//#define SCL_PIN2 GPIO_Pin_14 
//#define SDA_PIN2 GPIO_Pin_15


#define SCL_PIN2 			PBout(6)
#define SDA_PIN2			PBout(7)

#define SCL_HH         SCL_PIN2 = 1  
#define SCL_LL         SCL_PIN2 = 0

#define SDA_HH         SDA_PIN2 = 1 
#define SDA_LL         SDA_PIN2 = 0  

#define SCL_read2      PBin(6) 
#define SDA_read2     PBin(7)


void I2C2_INIT(void);
void I2C2_delay(void);
void delay5ms2(void);
uint16_t I2C2_Start(void);
void I2C2_Stop(void);
void I2C2_Ack(void); 
void I2C2_NoAck(void);
uint16_t I2C2_WaitAck(void);
void I2C_SendByte(unsigned char SendByte);
unsigned char I2C2_RadeByte(void);
uint16_t Single_Write2(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data);
unsigned char Single_Read2(unsigned char SlaveAddress,unsigned char REG_Address);


#endif // __MYI2C_H__
