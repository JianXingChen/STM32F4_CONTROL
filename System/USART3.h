#ifndef __USART1_H__
#define __USART1_H__

void USART3_Configuration(void);
#define	__USART1_H

#include "stm32f4xx.h"
#include <stdio.h>

#define PARA_REV_NUM 32      //接收总字节
#define PRAR_REV_DMA_CNT (DMA1_Stream1->MA_CNDTR)   //DMA已接收字节

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
	

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

/*********函数声明*******/

void USART3_Configuration(void);
void uart3_send_multi_data(float *data);

/*
检测上位机发送的PID参数进行更新

参数说明：cnt：定义超时中断周期个数

				PARA_INDEX：结构体指针  
				可选值：Pitch_para 、Yaw_para

*/
uint8_t Detect_para_rev_dma(uint16_t cnt , struct PID_PARA *PARA_INDEX);//检测更新PID参数



#endif
