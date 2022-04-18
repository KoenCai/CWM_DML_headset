#ifndef ___CWM_LIB__H_
#define ___CWM_LIB__H_

#include <stdint.h>
#include "os_api.h"
#include "CWM_EventStructure.h"
#include "CWM_CustomDataStruct.h"

#define SCL_LOG                 0
#define SCL_DATE_TIME           2
#define SCL_GET_CHIP_INFO       8
#define SCL_LIB_DEBUG           10
#define SCL_GET_LIB_INFO        12
#define SCL_CHIP_VENDOR_CONFIG  17
#define SCL_ABS_STATIC_CONFIG   19
#define SCL_INPUT_SENSOR_CONFIG 101
#define SCL_INPUT_DT_CONFIG     102
#define SCL_SENS_CALI_CONFIG    110
#define SCL_SENS_CALI_CTRL_MAG  111
#define SCL_SENS_CALI_CTRL_A    112
#define SCL_SPV_CONFIG          113
#define SCL_SPV_MODE            114
#define SCL_GET_HEAP_INFO       120
#define SCL_SEC_VERIFY_CLIENT   200
#define SCL_SEC_VERIFY_SERVER   201

#define SCL_HS_ORIEN_CONFIG     300
#define SCL_HS_ORIEN_CTRL_EXC   301
#define SCL_HS_ORIEN_RESET      302
#define SCL_HS_PHONE_CONFIG     303

/**
  * @brief Custom Sensor Input sensorId
  * @api CWM_CustomSensorInput
  * @param CustomSensorData.sensorType extSensorId
  * @
  */
#define CUSTOM_ACC              0
#define CUSTOM_GYRO             1
#define CUSTOM_MAG              2
#define CUSTOM_BARO             3
#define CUSTOM_TEMP             4
#define CUSTOM_HEARTRATE        5
#define CUSTOM_GNSS             6
#define CUSTOM_OFFBODY_DETECT   7
#define CUSTOM_ON_CHARGING      8
#define CUSTOM_ACC_ANY_MOTION   9
#define CUSTOM_SENS1            10


/**
  * @brief sensor enable/disable
  * @api CWM_Sensor_Enable CWM_Sensor_Disable
  * @param sensorId
  */
#define IDX_ACCEL                    0
#define IDX_GYRO                     1
#define IDX_MAG                      2
#define IDX_BARO                     3
#define IDX_HEARTRATE                5
#define IDX_REQUEST_SENSOR           11
#define IDX_ALGO_ABSOLUTE_STATIC     19
#define IDX_ALGO_SPV                 20
#define IDX_ALGO_SENS_CALIBRATION    21

typedef struct {
    int iData[16];
} SettingControl_t;


typedef void (*FN_SENSOR_LISTEN)(pSensorEVT_t);

/**
  * @brief  CyweeMotion internal process Initial
  */
void CWM_LibPreInit(os_api *api);

/**
  * @brief  Sensor Event register callback, when sensor event is triggered,
  *         this function will be called to notify
  */
void CWM_LibPostInit(FN_SENSOR_LISTEN pfn);

/**
  * @brief  Sensor enable based on sensor index provided
  * @param  sensorId: sensor register handle, if not setup this handle don't care
  * @retval 0 is success, otherwise is failed
  */
int CWM_Sensor_Enable(uint32_t sensorId);

/**
  * @brief  Sensor disable based on sensor index provided
  * @param  sensorId: sensor register handle, if not setup this handle don't care
  * @retval 0 is success, otherwise is failed
  */
int CWM_Sensor_Disable(uint32_t sensorId);

/**
  * @brief  Read sensor data and output through sensor call-back function
  */
int CWM_process2(int dt_us);

/**
  * @brief  Put custom sensor data.
  * @param  SensorDataInput: sensor data input handle, input format as below:
  *                          sensorType= CustomSensorDataIndex
  *                          fData     = sensor raw data
  * @retval 0 is success, otherwise is failed
  */
int CWM_CustomSensorInput(CustomSensorData *SensorDataInput);

/**
  * @brief  Switch log output by input parameters
  * @param  Control ID
  * @param  Setting control array
  * @retval 0 is success, otherwise is failed
  */
int CWM_SettingControl(int CtrId, SettingControl_t *pData);


#endif //___CWM_LIB__H_
