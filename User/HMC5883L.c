#include "HMC5883L.h"
#include "stm32f4xx.h"
#include "I2C.h"
#include "SysTickDelay.h"
#include "math.h"

u8		 HMC5883[6];	

extern float Send_data[4];

float mx , my , mz ;

void initHMC5883(void){
	
	Single_Write(HMC58X3_ADDR,HMC58X3_R_MODE,0x00);
	Single_Write(HMC58X3_ADDR,HMC58X3_R_CONFA,0x78);
	Single_Write(HMC58X3_ADDR,HMC58X3_R_CONFB,0x20);

}
int16_t X_BUFF[100],Y_BUFF[100],Z_BUFF[100]; 
float max=0,min=0,max2=0,min2=0,max3=0,min3=0;
void HMC5883_read(void){
	int i;
//	uint16_t tempx,tempy,tempz;
	 static uint8_t filter_cnt=0;
	 int32_t temp1=0,temp2=0,temp3=0;

	HMC5883[0]=Single_Read(HMC58X3_ADDR, HMC58X3_R_XH);
	HMC5883[1]=Single_Read(HMC58X3_ADDR, HMC58X3_R_XL);
	
	HMC5883[2]=Single_Read(HMC58X3_ADDR, HMC58X3_R_YH);
	HMC5883[3]=Single_Read(HMC58X3_ADDR, HMC58X3_R_YL);
	
	HMC5883[4]=Single_Read(HMC58X3_ADDR, HMC58X3_R_ZH);
	HMC5883[5]=Single_Read(HMC58X3_ADDR, HMC58X3_R_ZL);
	
		X_BUFF[filter_cnt] = (((int16_t)HMC5883[0]) << 8 | HMC5883[1]); //Combine MSB and LSB of X Data output register;
	  Y_BUFF[filter_cnt] = (((int16_t)HMC5883[2]) << 8 | HMC5883[3]); //Combine MSB and LSB of Y Data output register;
	  Z_BUFF[filter_cnt] = (((int16_t)HMC5883[4]) << 8 | HMC5883[5]); //Combine MSB and LSB of Z Data output register;
	
#if 0
	if(X_BUFF[filter_cnt] > max ){
		max = X_BUFF[filter_cnt];			//511
	}
	else if(X_BUFF[filter_cnt] < min){
		min = X_BUFF[filter_cnt];			//-252
	}
	
	if(Y_BUFF[filter_cnt] > max2 ){
		max2 = Y_BUFF[filter_cnt];		//427
	}
	else if(Y_BUFF[filter_cnt] < min2){
		min2 = Y_BUFF[filter_cnt];		//-322
	}
	
	if(Z_BUFF[filter_cnt] > max3 ){
		max3 = Z_BUFF[filter_cnt];		//312
	}
	else if(Z_BUFF[filter_cnt] < min3){
		min3 = Z_BUFF[filter_cnt];		//-499
	}
	
	Send_data[0] = max;
	Send_data[1] = min;
	Send_data[2] = max3;
	Send_data[3] = min3;
	
#endif

	  for(i=0;i<100;i++)  //10深度的滑动滤波
	  {
		   temp1 += X_BUFF[i] ;
		   temp2 += Y_BUFF[i];
		   temp3 += Z_BUFF[i];
	  }
#if 1
	  mx = temp1 / 100 - 129.5f;
	  my = temp2 / 100 - 52.5 ;
	  mz = temp3 / 100 + 93.5 ;
	  filter_cnt++;
	  if(filter_cnt==100)	filter_cnt=0;
#endif
		
#if 0
		mx = temp1 / 100.0f ;
	  my = temp2 / 100.0f  ;
	  mz = temp3 / 100.0f  ;
		
		filter_cnt++;
	  if(filter_cnt==100)	filter_cnt=0;
		
		Send_data[0] =mx;
		Send_data[1] = my;
		Send_data[2] = mz;
#endif
//	
//		/*   地磁椭圆矫正                              */  
//    /*  关于如何进行椭圆矫正 ，将在下版本 详细说明 */		
//    mx = 1 *(mx -163);
//    my = (double)((my + 303));
//	  mz = (double)((mz + 635));
		
		
	
}


void  SelfTest(void){

	Single_Write(HMC58X3_ADDR, HMC58X3_R_CONFA, 0x79);
	delay_ms(50);
	Single_Write(HMC58X3_ADDR, HMC58X3_R_CONFB, 0x40);
	delay_ms(50);
	Single_Write(HMC58X3_ADDR, HMC58X3_R_MODE, 0x01);
	delay_ms(50);
	
	HMC5883[0]=Single_Read(HMC58X3_ADDR, HMC58X3_R_XH);
	HMC5883[1]=Single_Read(HMC58X3_ADDR, HMC58X3_R_XL);
	
	HMC5883[2]=Single_Read(HMC58X3_ADDR, HMC58X3_R_YH);
	HMC5883[3]=Single_Read(HMC58X3_ADDR, HMC58X3_R_YL);
	
	HMC5883[4]=Single_Read(HMC58X3_ADDR, HMC58X3_R_ZH);
	HMC5883[5]=Single_Read(HMC58X3_ADDR, HMC58X3_R_ZL);
	
	X_BUFF[0] = (((int16_t)HMC5883[0]) << 8 | HMC5883[1]); //Combine MSB and LSB of X Data output register;
	Y_BUFF[0] = (((int16_t)HMC5883[2]) << 8 | HMC5883[3]); //Combine MSB and LSB of Y Data output register;
	Z_BUFF[0] = (((int16_t)HMC5883[4]) << 8 | HMC5883[5]); //Combine MSB and LSB of Z Data output register;
		
}

	
