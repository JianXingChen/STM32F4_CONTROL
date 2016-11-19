#include "imu.h"
#include "math.h"
#include "6050.h"
#include "holder.h"
#include "USART3.h"
#include "Prepare_data.h"
#include "clock.h"
			
	
#define FILTER_NUM 10


////////////////////////////////////////////////////////////////////////////////
#define Kp 20.0f                        // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.0005f                     // integral gain governs rate of convergence of gyroscope biases
#define halfT 0.0005f                   // half the sample period???????

T_FLOAT_ANGEL Q_ANGLE;

struct _angle angle;

extern float mx,my,mz;

float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error


//   快速求平方根倒数
float Q_rsqrt(float number)
{
	long i;
	float x2, y;
	const float threehalfs = 1.5f;
 
	x2 = number * 0.5F;
	y  = number;
	i  = * ( long * ) &y;                      
	i  = 0x5f3759df - ( i >> 1 );               
	y  = * ( float * ) &i;
	y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration （第一次牛顿迭代）
	return y;
} 
//  求float型数据绝对值
float FL_ABS(float x)
{
   if(x < 0)  return -x;
	 else return x; 
}
/*   采用三角函数的泰勒展开式 求近似值*/
float COS(float x)
{
	float result;
  result = 1 - x * x/2;
	return result; 
}

float SIN(float y)
{
	float result;
  result = y - y * y * y /6;
	return result; 
}
/***********************************************
  * @brief  可变增益自适应参数
************************************************/
float VariableParameter(float error)
{
	float  result = 0;
	
	if(error < 0)
	{
	   error = -error;
	}
  if(error >0.8f)
	{
	   error = 0.8f;
	}
	result = 1 - 1.28f * error;
	if(result < 0)
	{
	   result = 0;
	}
	return result;
}


void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az,float mx,float my,float mz)
{
  float norm;
	int16_t Xr,Yr;
  float vx, vy, vz;
	float hx, hy, bx, bz,wx, wy, wz;
  float ex, ey, ez;

  // 先把这些用得到的值算好
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
	float q0q3 = q0*q3;//
  float q1q1 = q1*q1;
  float q1q2 = q1*q2;//
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;
	
	if(ax*ay*az==0)
 		return;
	
#if 1
	hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
  hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
  bx = sqrt(hx * hx + hy * hy);
  bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
	
	wx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
  wy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
  wz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2); 
#endif
		
  norm = Q_rsqrt(ax*ax + ay*ay + az*az);       //acc数据归一化
  ax = ax *norm;
  ay = ay * norm;
  az = az * norm;

#if 1
	norm = Q_rsqrt(wx*wx + wy*wy + wz*wz);       
  wx = wx *norm;
  wy = wy * norm;
  wz = wz * norm;
	
	norm = Q_rsqrt(mx*mx + my*my + mz*mz);       
  mx = mx *norm;
  my = my * norm;
  mz = mz * norm;
	
#endif 

  // estimated direction of gravity and flux (v and w)              估计重力方向和流量/变迁
  vx = 2*(q1q3 - q0q2);												//四元素中xyz的表示
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3 ;

  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy) + (my*wz - mz*wy) ;                           					 //向量外积在相减得到差分就是误差
  ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
  ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

  exInt = exInt + VariableParameter(ex) * ex * Ki;								  //对误差进行积分
  eyInt = eyInt + VariableParameter(ey) * ey * Ki;
  ezInt = ezInt + VariableParameter(ez) * ez * Ki;
// adjusted gyroscope measurements

  gx = gx + Kp *  VariableParameter(ex) * ex + exInt;	
	gy = gy + Kp *  VariableParameter(ey) * ey + eyInt;	
	gz = gz + Kp *  VariableParameter(ez) * ez + ezInt;	
  								
  // integrate quaternion rate and normalise						   //四元素的微分方程
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

  // normalise quaternion
  norm = Q_rsqrt(q0q0 + q1q1 + q2q2 + q3q3);
  q0 = q0 * norm;
  q1 = q1 * norm;
  q2 = q2 * norm;
  q3 = q3 * norm;
	
  angle.pitch = atan2(2*q2q3 + 2*q0q1, -2*q1q1 - 2*q2q2 + 1);//-AngleOffset_Pit*AtR; // roll
	angle.roll = asin(-2*q1q3 + 2*q0q2);//-AngleOffset_Rol*AtR; // pitch
	
#if 0
	Xr = mx * COS(angle.pitch/AtR) + my * SIN(-angle.pitch/AtR) * SIN(-angle.roll/AtR) - mz * COS(angle.roll/AtR) * SIN(-angle.pitch/AtR);
	Yr = my * COS(angle.roll/AtR) + mz * SIN(-angle.roll/AtR);

	angle.yaw = atan2((double)Yr,(double)Xr) * RtA;
#endif

#if 1
	angle.yaw  = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1)* 57.3;
#endif
	
	angle.roll= angle.roll* RtA;
	
	angle.pitch = -angle.pitch * RtA;
	

#if 1 	
	Send_data[0] = angle.roll;
		
	Send_data[1] = angle.pitch;
	
	Send_data[2] = angle.yaw;
#endif
	if( angle.pitch > 0  )
	{
		angle.pitch = angle.pitch - 360.0f;
	}
	
	angle.pitch = angle.pitch + 160;
	
}

/**************************************
 * 函数名：Get_Attitude
 * 描述  ：得到当前姿态
 *************************************/
void Get_Attitude(void)
{
	//1.
	IMUupdate(sensor.gyro.radian.x,
						sensor.gyro.radian.y,
						sensor.gyro.radian.z,
						sensor.acc.averag.x,
	          sensor.acc.averag.y,
	          sensor.acc.averag.z,
						my,-mx,mz);	
	
	
//	if( Car_sys.Hold_init == 1 )
//	{
//			////////////pitch
//		Pitch_Hold_Info.angle = angle.pitch;
//		Pitch_Hold_Info.angle_speed = -sensor.gyro.radian.x / Gyro_Gr;
//		
//		if( abs( Pitch_Hold_Info.angle_speed )< 30 )
//		{
//			Pitch_Hold_Info.angle_speed = 0;
//		}
//		
//		///////////////yaw 
//		angle.yaw = sensor.gyro.radian.z / Gyro_Gr;
//		
//		if( abs( angle.yaw  ) < 50 )
//		{
//			angle.yaw = 0;
//		}
//		yaw_Hold_Info.angle = yaw_Hold_Info.angle + angle.yaw * 0.00003272f;//3.272727272727273e-5v
//		yaw_Hold_Info.angle_speed = angle.yaw ;
//		
//	}


	
	
	
}




