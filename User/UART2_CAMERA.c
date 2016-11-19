#include "UART2_CAMERA.h"
#include "holder.h"
#include "USART3.h"
#include "chassis.h"
#include "vision.h"

float yaw_diff = 0 , pitch_diff=0;
float yaw_f , pitch_f;
float yaw_diff_inter = 0;


float yaw_k =  0.00001f;
float pitch_k = 0;
		
#define CAMERA_REV_NUM_MAX 12

uint8_t camera_rev_buff[CAMERA_REV_NUM_MAX];

uint32_t Camera_cnt = 0;

void UART2_CAMERA_Configuration(void)
{
		USART_InitTypeDef USART2_InitStructure;
		GPIO_InitTypeDef  gpio;
		NVIC_InitTypeDef  nvic;
		DMA_InitTypeDef   dma;
	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA1,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource2 ,GPIO_AF_USART2);
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource3 ,GPIO_AF_USART2);
	
		gpio.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 ;
		gpio.GPIO_Mode = GPIO_Mode_AF;
		gpio.GPIO_OType = GPIO_OType_PP;
		gpio.GPIO_Speed = GPIO_Speed_100MHz;
		gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOA,&gpio);
			
		USART_DeInit(USART2);
		USART2_InitStructure.USART_BaudRate = 9600;   //D-BUS 100K baudrate
		USART2_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART2_InitStructure.USART_StopBits = USART_StopBits_1;
		USART2_InitStructure.USART_Parity = USART_Parity_No;
		USART2_InitStructure.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
		USART2_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(USART2,&USART2_InitStructure);
    
		USART_Cmd(USART2,ENABLE);
    USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
    
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0); 
    nvic.NVIC_IRQChannel = DMA1_Stream5_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 6;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    
    DMA_DeInit(DMA1_Stream5);
    dma.DMA_Channel= DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);
    dma.DMA_Memory0BaseAddr = (uint32_t)camera_rev_buff;
    dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize = CAMERA_REV_NUM_MAX;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Circular;
    dma.DMA_Priority = DMA_Priority_High;
    dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst = DMA_Mode_Normal;
    dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream5,&dma);

    DMA_ITConfig(DMA1_Stream5,DMA_IT_TC,ENABLE);
    DMA_Cmd(DMA1_Stream5,ENABLE);

}

void DMA1_Stream5_IRQHandler(void)
{
		int16_t yaw_temp,pitch_temp;

		uint16_t rev_cnt = 0;
	
	
    if(DMA_GetITStatus(DMA1_Stream5, DMA_IT_TCIF5))
    {
			Camera_cnt = 0;
			yaw_diff_inter = 0;
			
			DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);
			DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);
			
			yaw_f = 0;
			pitch_f = 0;
			
			for( rev_cnt = 0 ; rev_cnt < CAMERA_REV_NUM_MAX - 5; rev_cnt ++)
			{
				if( (camera_rev_buff[rev_cnt] == 0xaa)&( camera_rev_buff[rev_cnt + 5] == 0xbb ) )// valid data
				{
					
					yaw_temp =   ( camera_rev_buff[rev_cnt + 1] + camera_rev_buff[rev_cnt + 2]  *256  );
				
					//pitch_temp =  ( camera_rev_buff[rev_cnt + 3] + camera_rev_buff[rev_cnt + 4] * 256 );
		
					yaw_f = yaw_k * 1.36f* yaw_temp;
					
					#if 1
					Vision_target_filter();//滑动滤波处理
					#endif
					
					yaw_diff = yaw_f /20.0f;
					
							
					//示波器发送显示
					#if 0
					Send_data[0] = yaw_f ;//示波器发送
					Send_data[1] = pitch_f;
					//Send_data[2] = yaw_angel_target;
					//Send_data[3] = ywa_angle_speed;
					//Send_data[3] = pitch_out;
					#endif			
					break;	
				}
			}
	
			
    }
		

}
