#ifndef __CONFIG_H__
#define __CONFIG_H__

#define aikemu52832

#define IMU_SCL_PIN    NRF_GPIO_PIN_MAP(1,14)
#define IMU_SDA_PIN    NRF_GPIO_PIN_MAP(1,13)

#define sensor_address 0x6b  //lsm6dsow_address 0x6b   42607_address 0x69

/*sensorÑ¡Ôñ*/
#define LSM6DSOW
//#define ICM_42607

/*sensor input mode*/
#define FIFO_mode

/* set activity mode */
#define CWM_ACTIVITY_NORMAL													1001
//#define CWM_ACTIVITY_TREADMILL										1002
//#define CWM_ACTIVITY_OUTDOOR_RUNNING							1003
//#define CWM_ACTIVITY_CLIMBING_STAIRS							1004
//#define CWM_ACTIVITY_HIKING												1005
//#define CWM_ACTIVITY_INDOOR_RUNNING								1006
//#define CWM_ACTIVITY_INDOOR_SWIMMING							2001
//#define CWM_ACTIVITY_OUTDOOR_BIKING								3001
//#define CWM_ACTIVITY_FREETRAINING									5001
//#define CWM_ACTIVITY_WORKOUT_MACHINE							6001


/*sensor enable */
#define USE_ACC
#define USE_GYRO


#if 0
#define nothing 						//²»¿ªÆô¾²Ì¬Ëã·¨
#define inactivity_mode 0
#endif

#if 1

#define sedentary 					//¾Ã×ø
#define inactivity_mode 1
#endif

#if 0

#define sleeping						//Ë¯Ãß
#define inactivity_mode 2
#endif

#if 0

#define Sedentary_and_Nap  //¾Ã×øºÍË¯Ãß
#define inactivity_mode 3
#endif

#if 0

#define Nap  								//Ð¡Ë¯
#define inactivity_mode 4
#endif



#endif //__CONFIG_H__
