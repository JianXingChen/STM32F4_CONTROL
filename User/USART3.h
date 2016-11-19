#ifndef __USART3_H__
#define __USART3_H__


#include "stm32f4xx.h"
#include <stdio.h>

#define PARA_REV_NUM 32      //?????
#define PRAR_REV_DMA_CNT (DMA1_Stream1->MA_CNDTR)   //DMA?????

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
	
#define abs(x) ((x)>0? (x):(-(x)))


struct PID_PARA
{
	float shell_P;
	float shell_I;
	float shell_D;
	
	float core_P;
	float core_I;
	float core_D;
};
	
extern struct PID_PARA Pitch_para;
extern struct PID_PARA Yaw_para;

typedef struct
{
	uint8_t  rev_state;
	uint16_t data_num_now;
	uint16_t data_num_last;
	uint16_t rev_overtime_cnt;
	
}_PARA_REV;

extern _PARA_REV Para_s_rev;
extern uint8_t para_rev[32];

extern float Send_data[4];

/*********????*******/

void USART3_Configuration(void);
void uart3_send_multi_data(float *data);
void USART3_Configuration(void);



uint8_t Detect_para_rev_dma(uint16_t cnt , struct PID_PARA *PARA_INDEX);//????PID??



#endif
