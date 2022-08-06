//
// Created by insppp on 2022/8/4.
//
#include "main.h"
#include "cmsis_os.h"
#include "attitude.h"
#include "imu.h"
#include "sbus_receiver/remote_control.h"

const RC_ctrl_t *local_rc_ctrl;
void debug_output_entry(void const * argument)
{
	remote_control_init();
	local_rc_ctrl = get_remote_control_point();
	/* USER CODE BEGIN debug_output_entry */
	/* Infinite loop */
	for(;;)
	{
//		extern fp32 accel[3];
//		extern fp32 gyro[3];
//		#define X 1
//		#define Y 0
//		#define Z 2
//		printf("/*%.3f,%.3f,%.3f,%.3f,%.3f,%.3f*/\r\n", accel[X], accel[Y], accel[Z], gyro[0], gyro[1], gyro[2]);
//		printf("/*%f,%f,%f*/\r\n", gyro[0], gyro[1], gyro[2]);
//		extern Attitude attitude;
//		printf("/*%f,%f,%f*/\r\n", attitude.roll, attitude.pitch, attitude.yaw);
//		extern IMU imu;
//		printf("/*%f,%f,%f*/\r\n", imu.accel[0], imu.accel[1], imu.accel[2]);
		printf(
"**********\r\n\
ch0:%d\r\n\
ch1:%d\r\n\
ch2:%d\r\n\
ch3:%d\r\n\
",
				local_rc_ctrl->rc.ch[0],
				local_rc_ctrl->rc.ch[1],
				local_rc_ctrl->rc.ch[2],
				local_rc_ctrl->rc.ch[3]
				);
		osDelay(1000);
	}
	/* USER CODE END debug_output_entry */
}