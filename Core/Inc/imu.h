//
// Created by insppp on 2022/8/4.
//

#ifndef MY_FC_FW_IMU_H
#define MY_FC_FW_IMU_H

#include "struct_typedef.h"

typedef struct {
	fp32 *accel;
	fp32 *gyro;
	fp32 *temprate;
}IMU;//implement datasource for IMU

bool datasource_imu_bind(IMU *imu, fp32 *gyro, fp32 *accel, fp32* temprate);

#endif //MY_FC_FW_IMU_H
