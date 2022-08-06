//
// Created by insppp on 2022/8/4.
//

#ifndef MY_FC_FW_ATTITUDE_H
#define MY_FC_FW_ATTITUDE_H

#include "struct_typedef.h"
#include "datasource.h"

#define Gyro_G 0.03051756f*2
#define Gyro_Gr 0.0005326f*2
#define RtA 57.2957795f


typedef struct {
	fp32 roll;
	fp32 pitch;
	fp32 yaw;
}Attitude;

void datasource_update_attitude (DataSource source, Attitude* attitude, float dt);

#endif //MY_FC_FW_ATTITUDE_H
