#include "stm32f4xx.h"
#include "stdio.h"
#include "CAN1.h"
#include "chassis.h"

#include "usart1.h"

#include "speed_control.h"

#include "holder.h"



CanRxMsg RxMessage_205;
CanRxMsg RxMessage_206;
CanRxMsg RxMessage_201;
CanRxMsg RxMessage_202;
CanRxMsg RxMessage_203;
CanRxMsg RxMessage_204;
/***********************************************************************************
  * @函数描述：  CAN1通信初始化，波特率1MHz. CAN1_TX----PA12,CAN1_RX-----PA11
  * @入口参数：  无.
  * @返回值  :   无.
***********************************************************************************/
void CAN1_Configuration(void)
{
    CAN_InitTypeDef        can;
    CAN_FilterInitTypeDef  can_filter;
    GPIO_InitTypeDef       gpio;
    NVIC_InitTypeDef       nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);
		
    gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOA, &gpio);
	
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0); 
	
    nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 3;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    
    CAN_DeInit(CAN1);
    CAN_StructInit(&can);
    
    can.CAN_TTCM = DISABLE;
    can.CAN_ABOM = DISABLE;
    can.CAN_AWUM = DISABLE;
    can.CAN_NART = DISABLE;
    can.CAN_RFLM = DISABLE;
    can.CAN_TXFP = DISABLE;
    can.CAN_Mode = CAN_Mode_Normal;
    can.CAN_SJW  = CAN_SJW_1tq;
    can.CAN_BS1 = CAN_BS1_9tq;
    can.CAN_BS2 = CAN_BS2_4tq;
    can.CAN_Prescaler = 3;   //CAN BaudRate 42/(1+9+4)/3=1Mbps
    CAN_Init(CAN1, &can);

	can_filter.CAN_FilterNumber=0;
	can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
	can_filter.CAN_FilterScale=CAN_FilterScale_32bit;
	can_filter.CAN_FilterIdHigh=0x0000;
	can_filter.CAN_FilterIdLow=0x0000;
	can_filter.CAN_FilterMaskIdHigh=0x0000;
	can_filter.CAN_FilterMaskIdLow=0x0000;
	can_filter.CAN_FilterFIFOAssignment=CAN_FIFO0;
	can_filter.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&can_filter);
    
  CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);   //接收完成

}

/***************************************************************************************
  * @函数描述：  CAN1发送函数.
  * @入口参数：  ID----本地ID，Data----待发送数组地址.
  * @返回值  :   无.
****************************************************************************************/
void CAN1_Send(uint32_t ID,u8 *Data)         
{
	u8 TransmitMailbox=0,i=0;
	CanTxMsg TxMessage;
	
	TxMessage.StdId = ID;            //本地ID
	TxMessage.IDE = CAN_ID_STD; 
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8; 
	
	for(i=0;i<8;i++)
	{
		TxMessage.Data[i] = *Data++; 
	}
	
	TransmitMailbox=CAN_Transmit(CAN1,&TxMessage);
	
	while((CAN_TransmitStatus(CAN1,TransmitMailbox)!=CANTXOK)&&(i!=0xFF))
	{
		i++;
	}


}

void CAN1_TX_IRQHandler(void)   
{
    if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET) 
	{
	   CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
    }
}

/***************************************************************************************
  * @函数描述：  CAN1接收中断函数,依据ID将数据送入不同全局变量
  * @入口参数：  无.
  * @返回值  :   无.
****************************************************************************************/
void CAN1_RX0_IRQHandler(void)
{
	uint16_t can_pitch_angle , can_yaw_angle;
	
  CanRxMsg RxMessage;
    
  if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
	{
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
  
		/*------------------------Rx接收设置-------------------------*/
		RxMessage.StdId=0x00;
		RxMessage.IDE =CAN_ID_STD;
		RxMessage.RTR=CAN_RTR_DATA;
		RxMessage.DLC = 8; 
	
		CAN_Receive(CAN1,CAN_FIFO0,&RxMessage);
		
		if (RxMessage.StdId==0x205) 
		{
			RxMessage_205=RxMessage;
			
			 can_yaw_angle = RxMessage_205.Data[0] *256 + RxMessage_205.Data[1];//获取底盘偏角
		
			 Chassis_angle_raw = can_yaw_angle;
			
			 yaw_Hold_Info.can_angle = (can_yaw_angle - 2820) * 0.045454f;
			
		}
		else if( RxMessage.StdId == 0x206 )//Pitch
		{
			RxMessage_206=RxMessage;
			
			can_pitch_angle = RxMessage_206.Data[0] *256 + RxMessage_206.Data[1];

			Pitch_Hold_Info.can_angle = 0.0408f * (can_pitch_angle-1600) ;
			
		}
		
		
		
		else if( (RxMessage.StdId >= 0x201)&&( RxMessage.StdId <= 0x204 ) )
		{
			wheel_position[RxMessage.StdId - 0x201] = RxMessage.Data[0]*256 +RxMessage.Data[1];
		}
		
				//示波器观察
			#if 0
			Send_data[0] = wheel_position[0];
			Send_data[1] = wheel_position[1];
			Send_data[2] = wheel_position[2];
			Send_data[3] = wheel_position[3];
			#endif
//		else if (RxMessage.StdId==0x206) RxMessage_206=RxMessage;
//		else if (RxMessage.StdId==0x201) RxMessage_201=RxMessage;
//		else if (RxMessage.StdId==0x202) RxMessage_202=RxMessage;
//		else if (RxMessage.StdId==0x203) RxMessage_203=RxMessage;
//		else if (RxMessage.StdId==0x204) RxMessage_204=RxMessage;
    }

		
		
		CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0);
		
		

}



