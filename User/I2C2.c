#include "stm32f4xx.h"
#include "I2C2.h"

/*******************************************************************************
* Function Name  : I2C_GPIO_Config
* Description    : Configration Simulation IIC GPIO
* Input          : None 
* Output         : None
* Return         : None
****************************************************************************** */
void I2C2_INIT(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure; 
 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);  //使能GPIOB时钟
	
  //GPIO_PinRemapConfig(RCC_APB2Periph_AFIO | GPIO_Remap_I2C1, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin =  SCL_PIN2 | SDA_PIN2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	

}
/*******************************************************************************
* Function Name  : I2C_delay
* Description    : Simulation IIC Timing series delay
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C2_delay(void)
{
//	__NOP();
	int a = 6;//6
		while(a--);
}

void delay5ms(void)
{
		
   int i=5000;  
   while(i) 
   { 
     i--; 
   }  
}
/*******************************************************************************
* Function Name  : I2C_Start
* Description    : Master Start Simulation IIC Communication
* Input          : None
* Output         : None
* Return         : Wheather	 Start
****************************************************************************** */
uint16_t I2C2_Start(void)
{
	SDA_HH;
	SCL_HH;
	I2C_delay();
	if(!SDA_read2)return FALSE;	//SDA线为低电平则总线忙,退出
	SDA_LL;
	I2C_delay();
	if(SDA_read2) return FALSE;	//SDA线为高电平则总线出错,退出
	SDA_LL;
	I2C_delay();
	return TRUE;
}
/*******************************************************************************
* Function Name  : I2C_Stop
* Description    : Master Stop Simulation IIC Communication
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C2_Stop(void)
{
	SCL_LL;
	I2C2_delay();
	SDA_LL;
	I2C2_delay();
	SCL_HH;
	I2C2_delay();
	SDA_HH;
	I2C2_delay();
} 
/*******************************************************************************
* Function Name  : I2C_Ack
* Description    : Master Send Acknowledge Single
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C2_Ack(void)
{	
	SCL_L2;
	I2C2_delay();
	SDA_L2;
	I2C2_delay();
	SCL_H2;
	I2C2_delay();
	SCL_L2;
	I2C2_delay();
}   
/*******************************************************************************
* Function Name  : I2C_NoAck
* Description    : Master Send No Acknowledge Single
* Input          : None
* Output         : None
* Return         : None
****************************************************************************** */
void I2C2_NoAck(void)
{	
	SCL_L2;
	I2C2_delay();
	SDA_H2;
	I2C2_delay();
	SCL_H2;
	I2C2_delay();
	SCL_L2;
	I2C2_delay();
} 
/*******************************************************************************
* Function Name  : I2C_WaitAck
* Description    : Master Reserive Slave Acknowledge Single
* Input          : None
* Output         : None
* Return         : Wheather	 Reserive Slave Acknowledge Single
****************************************************************************** */
uint16_t I2C2_WaitAck(void) 	 //返回为:=1有ACK,=0无ACK
{
	SCL_L2;
	I2C2_delay();
	SDA_H2;			
	I2C2_delay();
	SCL_H2;
	I2C2_delay();
	if(SDA_read2)
	{
      SCL_L2;
	  I2C2_delay();
      return FALSE;
	}
	SCL_L2;
	I2C2_delay();
	return TRUE;
}
/*******************************************************************************
* Function Name  : I2C_SendByte
* Description    : Master Send a Byte to Slave
* Input          : Will Send Date
* Output         : None
* Return         : None
****************************************************************************** */
void I2C2_SendByte(unsigned char SendByte) //数据从高位到低位//
{
    u8 i=8;
    while(i--)
    {
        SCL_L2;
        I2C2_delay();
      if(SendByte&0x80)
              SDA_H2;  
      else 
        SDA_L2;   
        SendByte<<=1;
        I2C2_delay();
		SCL_H2;
        I2C2_delay();
    }
    SCL_L2;
}  
/*******************************************************************************
* Function Name  : I2C_RadeByte
* Description    : Master Reserive a Byte From Slave
* Input          : None
* Output         : None
* Return         : Date From Slave 
****************************************************************************** */
unsigned char I2C2_RadeByte(void)  //数据从高位到低位//
{ 
    u8 i=8;
    u8 ReceiveByte=0;

    SDA_H2;				
    while(i--)
    {
      ReceiveByte<<=1;      
      SCL_L2;
      I2C2_delay();
	  SCL_H2;
      I2C2_delay();	
      if(SDA_read2)
      {
        ReceiveByte|=0x01;
      }
    }
    SCL_L2;
    return ReceiveByte;
} 
//ZRX          
//单字节写入*******************************************

uint16_t Single_Write2(unsigned char SlaveAddress,unsigned char REG_Address,unsigned char REG_data)		     //void
{
  	if(!I2C2_Start())return FALSE;
    I2C2_SendByte(SlaveAddress);   //发送设备地址+写信号//I2C_SendByte(((REG_Address & 0x0700) >>7) | SlaveAddress & 0xFFFE);//设置高起始地址+器件地址 
    if(!I2C2_WaitAck()){I2C2_Stop(); return FALSE;}
    I2C2_SendByte(REG_Address );   //设置低起始地址      
    I2C2_WaitAck();	
    I2C2_SendByte(REG_data);
    I2C2_WaitAck();   
    I2C2_Stop(); 
    delay5ms2();
    return TRUE;
}


//单字节读取*****************************************
unsigned char Single_Read2(unsigned char SlaveAddress,unsigned char REG_Address)
{  
	unsigned char REG_data;     	
	if(!I2C2_Start())
		return FALSE;
    I2C2_SendByte(SlaveAddress); //I2C_SendByte(((REG_Address & 0x0700) >>7) | REG_Address & 0xFFFE);//设置高起始地址+器件地址 
    if(!I2C2_WaitAck())
    {
      I2C2_Stop();return FALSE;
    }
    I2C2_SendByte((u8) REG_Address);   //设置低起始地址      
    I2C2_WaitAck();
    I2C2_Start();
    I2C2_SendByte(SlaveAddress+1);
    I2C2_WaitAck();
		
	  REG_data= I2C2_RadeByte();
    I2C2_NoAck();
    I2C2_Stop();
    //return TRUE;
	return REG_data;
}	

