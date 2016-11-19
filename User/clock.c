#include "clock.h"
#include "led.h"
#include "chassis.h"
#include "omni.h"
#include "speed_control.h"
#include "holder.h"
#include "UART2_CAMERA.h"
#include "Prepare_data.h"
#include "imu.h"
#include "6050.h"
#include "HMC5883L.h"

struct T_SYS_FLAG Car_sys;


/*
yaw encode

right max : 840

left max: 4800

*/


void TIM3_Int_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///
	
  TIM_TimeBaseInitStructure.TIM_Period = 9; 	//
	TIM_TimeBaseInitStructure.TIM_Prescaler=8400-1;  //
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //
	TIM_Cmd(TIM3,ENABLE); //
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0); 
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0; //
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=2; //
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

//
void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //
	{
		Car_sys.clock_cnt ++;		
//		Camera_cnt ++;
		
		if( Car_sys.clock_cnt >=1000 )
		{
			Car_sys.clock_cnt = 0;
			
			if( Car_sys.Hold_init == 1 )
			{
				LED0= ~LED0;
			}
			
			
			if( Car_sys.Hold_init == 0 )
			{
				Car_sys.Hold_init_cnt ++;
			}
			
			if( Car_sys.Hold_init_cnt >= 4 )// 4s 后 ，角度解算稳定，判定云台初始化完成，开始进入稳定控制
			{
				Car_sys.Hold_init = 1;
			}
			
			
			if( Car_sys.mpu6050 == MPU6050_ERROR )// 6050读取错误，第二个LED闪烁
			{
				LED1= ~LED1;
			}
		
		}
		
		if( Car_sys.mpu6050 == MPU6050_ERROR )
		{
			Holder_out(0);//云台电机输出
			Speed_control( Target_speed , 0 );//控制电机闭环输出
			
		}
		else
		{
			#if 1
		
			//Detect_para_rev_dma(100 , &Yaw_para);//更新参数，上位机发送
				
			//底盘控制
			//Chassis_Control(1);//底盘控制
			
			//omni(ALL_MOTOR_OUT);//遥控器控制
			
			//Speed_control( Target_speed , 0 );//控制电机闭环输出
			
			
			/*****************云台控制*************/
	
			Prepare_data_mode5();// get sensor data
		
			HMC5883_read();
			
			Get_Attitude();
			
			#endif
		}
		
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //
}
