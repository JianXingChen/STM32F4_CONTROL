
#include "USART3.h"

/*-----USART3_TX-----PB10-----*/
/*-----USART3_RX-----PB11-----*/


_PARA_REV Para_s_rev;

uint8_t para_rev[PARA_REV_NUM];

u8 EnemyData[6];

void uart2_Rstart_dma(void);//??DMA??,????
uint8_t Updata_PID(struct PID_PARA *PARA_INDEX);//??PID??,????


float Send_data[4];

void USART3_Configuration(void)
{
    USART_InitTypeDef usart3;
		GPIO_InitTypeDef  gpio;
    NVIC_InitTypeDef  nvic;
		DMA_InitTypeDef   DMA_InitStructure;
	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA1,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
		
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); 
		
		gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
		gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOB,&gpio);
		
		usart3.USART_BaudRate = 9600;
		usart3.USART_WordLength = USART_WordLength_8b;
		usart3.USART_StopBits = USART_StopBits_1;
		usart3.USART_Parity = USART_Parity_No;
		usart3.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
    usart3.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(USART3,&usart3);

    USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);
		USART_Cmd(USART3,ENABLE);
    
    nvic.NVIC_IRQChannel = DMA1_Stream1_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 3;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
		
		DMA_DeInit(DMA1_Stream1);
    DMA_InitStructure.DMA_Channel= DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART3->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)para_rev;//(uint32_t)EnemyData;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = PARA_REV_NUM;//6;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream1,&DMA_InitStructure);

    DMA_Cmd(DMA1_Stream1,ENABLE);
}

void uart2_Rstart_dma(void)
{  
//	DMA_InitTypeDef       DMA_InitStructure;
//	DMA_InitStructure.DMA_BufferSize = 100;            
//	DMA_Init(DMA2_Stream0, &DMA_InitStructure);
//	
//	DMA_Cmd(DMA2_Stream0, ENABLE);         
	uint16_t x;
	DMA1_Stream1->CR &= ~(uint32_t)DMA_SxCR_EN;//Ê§ÄÜdma
	DMA1_Stream1->M0AR =(uint32_t)para_rev;
	DMA1_Stream1->NDTR = PARA_REV_NUM;
	
	for( x = 0;x< PARA_REV_NUM;x++)
	{
		para_rev[x] = 0;
	}
	Para_s_rev.data_num_last = 0;
	Para_s_rev.data_num_now = 0;
	Para_s_rev.rev_overtime_cnt = 0;
	Para_s_rev.rev_state = 0;
	
	DMA1_Stream1->CR |= (uint32_t)DMA_SxCR_EN;//Ê¹ÄÜdma
  
}

uint8_t Detect_para_rev_dma(uint16_t cnt , struct PID_PARA *PARA_INDEX)
{
	Para_s_rev.data_num_now = PARA_REV_NUM - DMA_SxNDT_1;//??para????????
	
	if( Para_s_rev.rev_state == 0 )//??????
	{
		if(  Para_s_rev.data_num_now > 0 )
		{
			Para_s_rev.rev_state = 1;
			Para_s_rev.data_num_last = Para_s_rev.data_num_now;
			
		}
	}
	else if( Para_s_rev.rev_state == 1 )//??????
	{
		if (Para_s_rev.data_num_now == Para_s_rev.data_num_last)//??????
		{
			Para_s_rev.rev_overtime_cnt ++;
			if( Para_s_rev.rev_overtime_cnt >= cnt )//??????
			{
				Para_s_rev.rev_state = 2;	
			}
			Para_s_rev.data_num_last = Para_s_rev.data_num_now;
		}
		else if( Para_s_rev.data_num_now > Para_s_rev.data_num_last )
		{
			Para_s_rev.rev_overtime_cnt = 0;
			Para_s_rev.data_num_last = Para_s_rev.data_num_now;
		}
	}

	
	if( Para_s_rev.rev_state ==2 )
	{
		Updata_PID(PARA_INDEX);
		
		uart2_Rstart_dma();//??DMA??
	}
	else
	{
		
	}
	return 1;
}

uint8_t Updata_PID(struct PID_PARA *PARA_INDEX)
{	
	if( (para_rev[0]==0x8a)&&(para_rev[1]==0x8b)&&(para_rev[2]==0x1c)&&(para_rev[3]==0xae) )
	{
		(*PARA_INDEX).core_P = para_rev[4] * 256 + para_rev[5];
		(*PARA_INDEX).core_I = para_rev[6] * 256 + para_rev[7];
		(*PARA_INDEX).core_D = para_rev[8] * 256 + para_rev[9];
		
		(*PARA_INDEX).shell_P = para_rev[10] * 256 + para_rev[11];
		(*PARA_INDEX).shell_I = para_rev[12] * 256 + para_rev[13];
		(*PARA_INDEX).shell_D = para_rev[14] * 256 + para_rev[15];
		
	}
	
	return 1;
}

void uart3_send_multi_data(float *data)
{
	uint16_t x;
	uint16_t i;
	uint8_t sum;
	
	uint8_t data_buff[32];
	
	data_buff[0] = 0x88;
	data_buff[1] = 0xa1;
	data_buff[2] = 28;
	
	for( i=0;i<4;i++ )
	{
		data_buff[i*4+0+3] = BYTE3(*(data+i));
		data_buff[i*4+1+3] = BYTE2(*(data+i));
		data_buff[i*4+2+3] = BYTE1(*(data+i));
		data_buff[i*4+3+3] = BYTE0(*(data+i));
	}
	
	for( x=0;x<31;x++ )
	{
		sum = sum + data_buff[x];
	}
	
	data_buff[31] = sum;
	
	
	for( x = 0;x<32;x++ )
	{
		while ((USART3->SR & USART_FLAG_TC) == (uint16_t)RESET);
		USART_SendData(USART3,data_buff[x]);
	}
	
}

void uart_putchar(uint8_t data)
{
		while ((USART1->SR & USART_FLAG_TC) == (uint16_t)RESET);
		USART_SendData(USART1,data);
	  while ((USART1->SR & USART_FLAG_TC) == (uint16_t)RESET);
}




//void USART3_SendChar(unsigned char b)
//{
//    while (USART_GetFlagStatus(USART3,USART_FLAG_TC) == RESET);
//		USART_SendData(USART3,b);
//}

//int fputc(int ch, FILE *f)
//{
//    while (USART_GetFlagStatus(USART3,USART_FLAG_TC) == RESET);
//    USART_SendData(USART3, (uint8_t)ch);
//    return ch;
//}

//char Rx_data = 0;
//void USART3_IRQHandler(void)
//{
//    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
//    {
//        USART_ClearITPendingBit(USART3,USART_IT_RXNE);
//        Rx_data  = USART3->DR;
//    }
//}
