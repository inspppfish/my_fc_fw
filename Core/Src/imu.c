//
// Created by insppp on 2022/8/4.
//

#include "imu.h"

bool datasource_imu_bind(IMU *imu, fp32 *gyro, fp32 *accel, fp32 *temprate) {
	imu->accel = accel;
	imu->gyro = gyro;
	imu->temprate = temprate;
	return true;
}
