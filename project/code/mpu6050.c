#include "mpu6050.h"
#include "math.h"
#include "stdio.h"

float time=0.001;  	//时间间隔1ms

float gyro_pitch, acc_pitch, pitch, pitch_offset=3.0;
float yaw, yaw_offset=0;
int16 AX, AY, AZ;

KalmanFilter KF;		//卡尔曼滤波参数结构体


/*初始化卡尔曼滤波*/
//三个参数：角度过程噪声，零偏过程噪声，测量噪声
void Kalman_Init(KalmanFilter* kf,float Q_angle,float Q_bias,float R_measure) {
    kf->Q_angle = Q_angle;
    kf->Q_bias = Q_bias;
    kf->R_measure = R_measure;

    kf->angle = 0.0f;
    kf->bias = 0.0f;
    
    kf->P[0][0] = 0.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 0.0f;
}

/* 卡尔曼滤波计算 */
float Kalman_Calculate(KalmanFilter* kf, float newAngle, float newRate, float time, float* corrected_rate) {

    kf->rate = newRate - kf->bias;
    kf->angle += time * kf->rate;
    
    kf->P[0][0] += time * (time * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= time * kf->P[1][1];
    kf->P[1][0] -= time * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * time;
    
    float S = kf->P[0][0] + kf->R_measure;
    float K[2];
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;
    
    float y = newAngle - kf->angle;
    kf->angle += K[0] * y;
    kf->bias += K[1] * y;
    
    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];
    
    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;
		
		
		if(corrected_rate != NULL) {
      *corrected_rate = kf->rate;
    }
    
    return kf->angle;
}

/*姿态解算*/
void Calculate_Attitude(void) 
{
	static int16 cnt = 0;
	
	mpu6050_get_gyro();
	mpu6050_get_acc();
	
	//yaw
	yaw += (mpu6050_gyro_transition(mpu6050_gyro_z) * time)-yaw_offset;
	
	if(yaw<0){//校准yaw角偏移
		if(cnt<=1000){				
			if(cnt==1000){
			yaw_offset = yaw / 1000.0;
			yaw = 0;
			}
			cnt++;
		}
	}
	
	//pitch
	AX = mpu6050_acc_x;
	AY = mpu6050_acc_y;
	AZ = mpu6050_acc_z;
	// 计算加速度计角度
	float acc_pitch = -atan2(AX, sqrt(AY*AY + AZ*AZ)) * 57.29578f; // 180/π ≈ 57.29578
	// 获取陀螺仪原始角速度（转换为°/s）
	float gyro_rate_raw = (mpu6050_gyro_y / 32768.0f) * 2000.0f;  // 假设量程为±2000°/s
	
	pitch=Kalman_Calculate(&KF, acc_pitch, gyro_rate_raw, time, &gyro_pitch) + pitch_offset;
}
