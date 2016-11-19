#include "Prepare_data.h"
#include "6050.h"
#include "holder.h"
#include "clock.h"
#include "math.h"

#include "USART3.h"
#include "imu.h"

#define K_K ( 4.0f / 65536.0f )



/*	
	Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
	R:测量噪声，R增大，动态响应变慢，收敛稳定性变好	
*/

#define KALMAN_Q        0.02
#define KALMAN_R        6.0000

extern float mx,my,mz;

T_INT16_XYZ ACC_AVG ;
T_FLOAT_XYZ GYRO_AVG;


int16_t	ACC_X_BUF[ACC_FILTER_NUM],ACC_Y_BUF[ACC_FILTER_NUM],ACC_Z_BUF[ACC_FILTER_NUM];					
				

float Gyro_File_Buf[3][GYRO_FILTER_NUM];

float Z_Gyro_File_Buf[Z_GYRO_FILTER_NUM];


static double KalmanFilter_x(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R);
static double KalmanFilter_y(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R);
static double KalmanFilter_z(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R);



void Prepare_data_mode4(void)
{
	static int num = 0;
	static uint8_t filter_cnt=0;
	
	
	float sum0,sum1,sum2;
	int32_t temp1=0,temp2=0,temp3=0;

	int i;
	
	int16_t acc_16[3];
	int16_t gyro_16[3];

	unsigned char Addr;
	
	//******************测试读取芯片错误************************//
	#if 1
	Addr=Single_Read(MPU6050_ADDRESS,WHO_AM_I); //count++;
	
	if( Addr != 0x68 )
	{
		Car_sys.mpu6050 = MPU6050_ERROR;
		
		return;
	}
	
	Addr=Single_Read(MPU6050_ADDRESS2,WHO_AM_I); //count++;
	
	if( Addr != 0x68 )
	{
		Car_sys.mpu6050 = MPU6050_ERROR;
		
		return;
	}
	#endif
	//******************测试读取芯片错误************************//
	
	
	
	//1.获取数据
	MPU6050_Read();
	
	//2. 计算加速度计角度
	acc_16[0] =  ((((int16_t)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1]) ;
	acc_16[1] = ((((int16_t)mpu6050_buffer[2]) << 8) | mpu6050_buffer[3]);
	acc_16[2] = ((((int16_t)mpu6050_buffer[4]) << 8) | mpu6050_buffer[5]) ;
	
	ACC_X_BUF[filter_cnt] = acc_16[0];
	ACC_Y_BUF[filter_cnt] = acc_16[1];
	ACC_Z_BUF[filter_cnt] = acc_16[2];
	
	temp1 = 0;
	temp2 = 0;
	temp3 = 0;
	
	for(i=0;i<ACC_FILTER_NUM;i++)
	{
		temp1 += ACC_X_BUF[i];
		temp2 += ACC_Y_BUF[i];
		temp3 += ACC_Z_BUF[i];
	}
	
	ACC_AVG.X = temp1 / ACC_FILTER_NUM;
	ACC_AVG.Y = temp2 / ACC_FILTER_NUM;
	ACC_AVG.Z = temp3 / ACC_FILTER_NUM;
	
	filter_cnt = (filter_cnt + 1)%ACC_FILTER_NUM;
	
	
	//3. 计算陀螺仪角速度
	gyro_16[0] = ((((int16_t)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9]) - sensor.gyro.quiet.x;
	gyro_16[1] = ((((int16_t)mpu6050_buffer[10]) << 8) | mpu6050_buffer[11]) - sensor.gyro.quiet.y;
	gyro_16[2] = ((((int16_t)mpu6050_buffer[12]) << 8) | mpu6050_buffer[13]) - sensor.gyro.quiet.z;
	
//	Gyro_File_Buf[0][num] = gyro_16[0] * Gyro_rate[0] * 0.0010653f	;
//	Gyro_File_Buf[1][num] = gyro_16[1] * Gyro_rate[1] * 0.0010653f	;
//	Gyro_File_Buf[2][num] = gyro_16[2] * Gyro_rate[2] * 0.0010653f	;
	
	Gyro_File_Buf[0][num] = gyro_16[0]* 0.0010653f 	;
	Gyro_File_Buf[1][num] = gyro_16[1]* 0.0010653f 	;
	Gyro_File_Buf[2][num] = gyro_16[2] * 0.0010653f	;


	//4. 陀螺仪滤波
	sum0 = 0;
	sum1 = 0;
	sum2 = 0;

	for(i=0;i<GYRO_FILTER_NUM;i++)
	{
		sum0 += Gyro_File_Buf[0][i];
		sum1 += Gyro_File_Buf[1][i];
		sum2 += Gyro_File_Buf[2][i];
	}
	
	
	GYRO_AVG.X = sum0 * GYRO_FILTER_K;
	GYRO_AVG.Y = sum1 * GYRO_FILTER_K;
	GYRO_AVG.Z = sum2 * GYRO_FILTER_K;

	
	num = (num + 1) % GYRO_FILTER_NUM;



}

void Prepare_data_mode5(void)
{
	static uint8_t num = 0;
	
	static uint8_t z_num = 0;
	
	float sum0,sum1,sum2;

	int i;
	unsigned char Addr;
		
	//******************测试读取芯片错误************************//
	#if 1
	Addr=Single_Read(MPU6050_ADDRESS,WHO_AM_I); //count++;
	
	if( Addr != 0x68 )
	{
		Car_sys.mpu6050 = MPU6050_ERROR;
		
		return;
	}
	
//	Addr=Single_Read(MPU6050_ADDRESS2,WHO_AM_I); //count++;
//	
//	if( Addr != 0x68 )
//	{
//		Car_sys.mpu6050 = MPU6050_ERROR;
//		
//		return;
//	}
	#endif
	//******************测试读取芯片错误************************//
	
	
	
	
		//1.获取数据
	MPU6050_Read();
	
	sensor.acc.origin.x = ((((int16_t)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1]) ;
	sensor.acc.origin.y = ((((int16_t)mpu6050_buffer[2]) << 8) | mpu6050_buffer[3]) ;
	sensor.acc.origin.z = ((((int16_t)mpu6050_buffer[4]) << 8) | mpu6050_buffer[5]);

	sensor.gyro.origin.x = ((((int16_t)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9]);
	sensor.gyro.origin.y = ((((int16_t)mpu6050_buffer[10]) << 8)| mpu6050_buffer[11]);
	sensor.gyro.origin.z = ((((int16_t)mpu6050_buffer[12]) << 8)| mpu6050_buffer[13]);
  
	sensor.gyro.radian.x = sensor.gyro.origin.x * Gyro_Gr - sensor.gyro.quiet.x * Gyro_Gr;
	sensor.gyro.radian.y = sensor.gyro.origin.y * Gyro_Gr - sensor.gyro.quiet.y * Gyro_Gr;
	sensor.gyro.radian.z = sensor.gyro.origin.z * Gyro_Gr - sensor.gyro.quiet.z * Gyro_Gr;

	
	//2. acc 滤波
	sensor.acc.averag.x = KalmanFilter_x(sensor.acc.origin.x,KALMAN_Q,KALMAN_R);  // ACC X轴卡尔曼滤波
	sensor.acc.averag.y = KalmanFilter_y(sensor.acc.origin.y,KALMAN_Q,KALMAN_R);  // ACC Y轴卡尔曼滤波
	sensor.acc.averag.z = KalmanFilter_z(sensor.acc.origin.z,KALMAN_Q,KALMAN_R);  // ACC Z轴卡尔曼滤波
	
	Gyro_File_Buf[0][num] = sensor.gyro.radian.x;
	Gyro_File_Buf[1][num] = sensor.gyro.radian.y;
	//Gyro_File_Buf[2][num] = sensor.gyro.radian.z;
	
	Z_Gyro_File_Buf[z_num]  = sensor.gyro.radian.z;
	
	
	//3. 陀螺仪滤波
	sum0 = 0;
	sum1 = 0;
	sum2 = 0;

	for(i=0;i<GYRO_FILTER_NUM;i++)
	{
		sum0 += Gyro_File_Buf[0][i];
		sum1 += Gyro_File_Buf[1][i];
		//sum2 += Gyro_File_Buf[2][i];
	}
	
	
	sensor.gyro.radian.x = sum0 * GYRO_FILTER_K;
	sensor.gyro.radian.y = sum1 * GYRO_FILTER_K;
	//sensor.gyro.radian.z = sum2 * GYRO_FILTER_K;
	
	
	//单独为z轴计算滑动滤波，滑动深度比其他轴大
	for(i=0;i<Z_GYRO_FILTER_NUM;i++)
	{
		sum2 += Z_Gyro_File_Buf[i];
	}
	sensor.gyro.radian.z = sum2 / Z_GYRO_FILTER_NUM;
	
	
	num = (num + 1) % GYRO_FILTER_NUM;
	z_num = (z_num + 1) % Z_GYRO_FILTER_NUM;
}


void Update_Sensor_data(void)
{
	IMUupdate( GYRO_AVG.X , GYRO_AVG.Y  , GYRO_AVG.Z  , ACC_AVG.X, ACC_AVG.Y , -ACC_AVG.Z ,mx,my,mz );//update the data
	
	
//	if( Car_sys.Hold_init == HOLD_INIT_NOT_OK )
//	{
//		//1.初始化时取云台角度信息使其稳定
//			Pitch_Hold_Info.angle = Pitch_Hold_Info.can_angle;
//			yaw_Hold_Info.angle = yaw_Hold_Info.can_angle;
//	}
//	
//	else
	{
		Pitch_Hold_Info.angle = -Q_ANGLE.PIT - 20;
		yaw_Hold_Info.angle = Q_ANGLE.YAW;
	}

	Pitch_Hold_Info.angle_speed = GYRO_AVG.X;
	yaw_Hold_Info.angle_speed  = GYRO_AVG.Y;
	
}





/*	
	Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
	R:测量噪声，R增大，动态响应变慢，收敛稳定性变好	
*/
/*           卡尔曼对三个轴加速度进行滤波处理           */
static double KalmanFilter_x(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
   kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声
   x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
                
   p_now=(1-kg)*p_mid;//最优值对应的covariance       
   p_last = p_now; //更新covariance值
   x_last = x_now; //更新系统状态值
   return x_now;                
 }
static double KalmanFilter_y(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
   kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声
   x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
                
   p_now=(1-kg)*p_mid;//最优值对应的covariance       
   p_last = p_now; //更新covariance值
   x_last = x_now; //更新系统状态值
   return x_now;                
 }
static double KalmanFilter_z(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
   kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声
   x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值
                
   p_now=(1-kg)*p_mid;//最优值对应的covariance       
   p_last = p_now; //更新covariance值
   x_last = x_now; //更新系统状态值
   return x_now;                
 }


