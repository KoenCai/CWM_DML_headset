#include "icm_42607_port.h"


extern float acc_in[3] ;
extern float gyr_in[3] ;
void put_imu_data(float *accel_g, float *gyro_dps)
{
		CustomSensorData data;
	
    acc_in[0] = accel_g[0] * 9.80665f;
    acc_in[1] = accel_g[1] * 9.80665f;
    acc_in[2] = accel_g[2] * 9.80665f;

    gyr_in[0] = gyro_dps[0] * 3.14159f / 180;
    gyr_in[1] = gyro_dps[1] * 3.14159f / 180;
    gyr_in[2] = gyro_dps[2] * 3.14159f / 180;
	
//		printf("acc[m/s2]= %.4f %.4f %.4f\r\n", acc_in[0], acc_in[1], acc_in[2]);
//		printf("gyr[rad/s]= %.4f %.4f %.4f\r\n", gyr_in[0], gyr_in[1], gyr_in[2]);	
    
    memset(&data, 0, sizeof(data));
    data.sensorType = CUSTOM_ACC;
    memcpy(data.fData, acc_in, sizeof(float) *3);
    CWM_CustomSensorInput(&data);	
	
    memset(&data, 0, sizeof(data));
    data.sensorType = CUSTOM_GYRO;
    memcpy(data.fData, gyr_in, sizeof(float) *3);
    CWM_CustomSensorInput(&data);

		CWM_process();

}

void inv_imu_sleep_us(uint32_t us)
{
    CWM_OS_uSleep(us);
}

uint64_t inv_imu_get_time_us(void)
{
    uint64_t time;
    time = CWM_OS_GetTimeNs();
    return time / 1000;
}



