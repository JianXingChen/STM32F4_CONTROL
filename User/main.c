#include "stm32f4xx.h" 
#include "sys.h" 
#include "delay.h" 
//#include "led.h" 
//#include "usart1.h"	
#include "clock.h"
//#include "SDS.h"
//#include "spi.h"
//#include "CAN1.h"
//#include "chassis.h"

#include "USART3.h"
#include "I2C.h"
#include "6050.h"
//#include "holder.h"
//#include "USART1.h"
//#include "UART2_CAMERA.h"
//#include "speed_control.h"
//#include "math.h"
#include "HMC5883L.h"


	
/*
2016 1.30

底盘和轮子速度闭环

控制周期0.5ms

底盘PID参数：只有P环节  P = 70

轮子PID参数：只有P环节  P = 1500

*/


/*
2016.2.1

实现整车功能，底盘跟随云台

1. 控制周期 1ms，实验发现0.5ms周期处理器速度跟不上

2. 程序默认开启FPU功能，加快浮点运算

3. 小数表示需要加上f  ，如1.2f

4. 程序初始化需要给定遥控器四个通道的值为1024，防止跑飞

5. 


*/

//	__FPU_PRESENT=1,__FPU_USED =1
//  添加以上代码，开启FPU


/*

2016.2.25

和摄像头结合功能完成，效果差，视觉输出频率有问题
*/


/*
2016.3.29

重新进行姿态解算

串级控制外环加入D，以陀螺仪输出作为外环的D，效果良好，消除了之前的慢速回复的问题

角度解算初期需要时间收敛，所以设定4秒后才进行云台控制

yaw轴陀螺仪滑动滤波深度和其他轴不同，消除震动干扰

下一步：
1. 需要配合整车，重新运行

2. 需要配合视觉

*/



float a  =1.1f , b = 1.1f , c;

int32_t num;

uint8_t data[8];

int16_t speed ;

float xx  = 371.1 , x2;


float acc_temp[4];
float angle_temp[3];


int main(void)
{
	//x2 = xx % 360.0;

	delay_init(168);//延时前要调用
	

//	RC_Ctl.rc.ch0 = 1024;
//	RC_Ctl.rc.ch1 = 1024;
//	RC_Ctl.rc.ch2 = 1024;
//	RC_Ctl.rc.ch3 = 1024;
	
//	LED_Init();//Pc13,Pc14
	
	USART3_Configuration();//上位机调试使用
	
	
	I2C_INIT();//MPU6050
	delay_ms(500);
	
	InitMPU6050();//读取gyro z
	Gyro_OFFEST();
	initHMC5883();
	TIM3_Int_Init();// 1ms
	

	while(1)
	{

		uart3_send_multi_data(Send_data);
		
	}
}


