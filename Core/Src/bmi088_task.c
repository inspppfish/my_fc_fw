//
// Created by insppp on 2022/8/4.
//

#include "main.h"
#include "cmsis_os.h"
#include "imu.h"
#include "attitude.h"

fp32 accel[3], gyro[3], temp;
IMU imu;
Attitude attitude;
void bmi088process_entry(void const * argument)
{
	/* USER CODE BEGIN bmi088process_entry */
	while (BMI088_init()) {};
	datasource_imu_bind(&imu, gyro, accel, &temp);
	/* Infinite loop */
	for(;;)
	{
		BMI088_read(gyro, accel, &temp);
		datasource_update_attitude(&imu, &attitude, 10.0f/1000.0f);
		osDelay(10);
	}
	/* USER CODE END bmi088process_entry */
}