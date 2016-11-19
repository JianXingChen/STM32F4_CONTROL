#ifndef _PREPARE_DATA_H_
#define _PREPARE_DATA_H_
#include "holder.h"

#include "stm32f4xx.h" 
#include "sys.h"

#define ACC_FILTER_NUM 20

#define RtA 		57.324841		//  180/3.1415  角度制 转化为弧度制		
#define AtR    	0.0174533		//  1/RtA             RtA倒数		
#define Acc_G 	0.0011963		//  1/32768/4/9.8     加速度量程为4G		
#define Gyro_G 	0.03051756	//  1/32768/1000      陀螺仪量程为 +—1000			
#define Gyro_Gr	0.0005327   //  1/32768/1000/57.3 



typedef struct{
				int16_t X;
				int16_t Y;
				int16_t Z;}T_INT16_XYZ;

typedef struct{
				int16_t X;
				int16_t Y;
				int16_t Z;}T_FLOAT_XYZ;

				
				
extern T_INT16_XYZ ACC_AVG ;
extern T_FLOAT_XYZ GYRO_AVG;				
				
extern float Gyro_File_Buf[3][GYRO_FILTER_NUM];
extern int16_t	ACC_X_BUF[ACC_FILTER_NUM],ACC_Y_BUF[ACC_FILTER_NUM],ACC_Z_BUF[ACC_FILTER_NUM];	

				
				
				
void Prepare_data_mode4(void);
void Update_Sensor_data(void);
void Prepare_data_mode5(void);

				
				

#endif

