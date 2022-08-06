//
// Created by insppp on 2022/8/4.
//

#include "datasource.h"
#include "attitude.h"
#include "imu.h"
#include "my_math.h"

void datasource_update_attitude (DataSource source, Attitude* attitude, float dt){
	IMU* imu = source;

	// copy data
	volatile fp32 accX = imu->accel[1];
	volatile fp32 accY = imu->accel[0];
	volatile fp32 accZ = imu->accel[2];
	volatile fp32 gyroX = imu->gyro[1];
	volatile fp32 gyroY = imu->gyro[0];
	volatile fp32 gyroZ = imu->gyro[2];

	volatile struct V
	{
		float x;
		float y;
		float z;
	} Gravity,Acc,Gyro,AccGravity;

	static struct V GyroIntegError = {0};
	static  float KpDef = 8.0f ;
	static  float KiDef = 0.00003f;
	static Quaternion NumQ = {1, 0, 0, 0};
	//float NormAcc;
	volatile float NormQuat;
	float HalfTime = dt * 0.5f;

	// 提取等效旋转矩阵中的重力分量
	Gravity.x = 2.0f*(NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);
	Gravity.y = 2.0f*(NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);
	Gravity.z = 1.0f-2.0f*(NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);

	// 加速度归一化
	fp32 NormAcc = 1.0f/sqrtf(squa(accX) + squa(accY) + squa(accZ));
	Acc.x = accX * NormAcc;
	Acc.y = accY * NormAcc;
	Acc.z = accZ * NormAcc;

	//向量差乘得出的值
	AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
	AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
	AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);
	//再做加速度积分补偿角速度的补偿值
	GyroIntegError.x += AccGravity.x * KiDef;
	GyroIntegError.y += AccGravity.y * KiDef;
	GyroIntegError.z += AccGravity.z * KiDef;

	//角速度融合加速度积分补偿值
	Gyro.x = gyroX * Gyro_Gr + KpDef * AccGravity.x  +  GyroIntegError.x;//弧度制
	Gyro.y = gyroY * Gyro_Gr + KpDef * AccGravity.y  +  GyroIntegError.y;
	Gyro.z = gyroZ * Gyro_Gr + KpDef * AccGravity.z  +  GyroIntegError.z;

	// 一阶龙格库塔法, 更新四元数

	NumQ.q0 += (-NumQ.q1*Gyro.x - NumQ.q2*Gyro.y - NumQ.q3*Gyro.z) * HalfTime;
	NumQ.q1 += ( NumQ.q0*Gyro.x - NumQ.q3*Gyro.y + NumQ.q2*Gyro.z) * HalfTime;
	NumQ.q2 += ( NumQ.q3*Gyro.x + NumQ.q0*Gyro.y - NumQ.q1*Gyro.z) * HalfTime;
	NumQ.q3 += (-NumQ.q2*Gyro.x + NumQ.q1*Gyro.y + NumQ.q0*Gyro.z) * HalfTime;

	// 四元数归一化
	NormQuat = 1.0f/sqrtf(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3));
	NumQ.q0 *= NormQuat;
	NumQ.q1 *= NormQuat;
	NumQ.q2 *= NormQuat;
	NumQ.q3 *= NormQuat;

	// 四元数转欧拉角
	{
	#ifdef	YAW_GYRO
		*(float *)pAngE = atan2f(2 * NumQ.q1 *NumQ.q2 + 2 * NumQ.q0 * NumQ.q3, 1 - 2 * NumQ.q2 *NumQ.q2 - 2 * NumQ.q3 * NumQ.q3) * RtA;  //yaw
	#else
		float yaw_G = gyroZ * Gyro_G;
		if((yaw_G > 3.0f) || (yaw_G < -3.0f)) //数据太小可以认为是干扰，不是偏航动作
		{
			attitude->yaw  += yaw_G * dt;
		}
	#endif
		attitude->pitch  =  asinf(2 * NumQ.q0 *NumQ.q2 - 2 * NumQ.q1 * NumQ.q3) * RtA;
		attitude->roll	= atan2f(2 * NumQ.q2 *NumQ.q3 + 2 * NumQ.q0 * NumQ.q1, 1 - 2 * NumQ.q1 *NumQ.q1 - 2 * NumQ.q2 * NumQ.q2) * RtA;	//PITCH
	}
}