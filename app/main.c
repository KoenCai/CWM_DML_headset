#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "boards.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"

#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"
#include "board_nordic.h"
#include "lsm6dso_reg.h"
#include "nrf_uart.h"
//#include "w25q128.h"

#include "cwm_lib.h"
#include "cwm_lib_dml.h"

/* Variables Defines */

static int m_req_sensor_state = 0;

/* Function Implementations */
int getReqSensor(int index) {
    if (m_req_sensor_state & (1<<index))
        return 1;
    return 0;
}

typedef union{
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

//static axis3bit16_t data_raw_acceleration;
//static axis3bit16_t data_raw_angular_rate;
//static float acceleration_mg[3];
//static float angular_rate_mdps[3];
//static uint8_t whoamI, rst;

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
//static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
//static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static void platform_delay(uint32_t ms);
static void platform_init(void);


SettingControl_t scl;	
CustomSensorData csd ;

float acc_in[3] = {0};
float gyr_in[3] = {0};
float m_acc_bias[3] = {0};
float m_gyr_bias[3] = {0};

//uint8_t Flash_Tx_Buffer[256];
//uint8_t Flash_Rx_Buffer[256];


///写入Flash Sensor bias数据
//void f2u(void)
//{
//    uint8_t i;
//    for (i = 0; i < 4; i++)
//        Flash_Tx_Buffer[i] = ((uint8_t *)&(m_gyr_bias[0]))[i];

//    for (i = 0; i < 4; i++)
//        Flash_Tx_Buffer[i + 4] = ((uint8_t *)&(m_gyr_bias[1]))[i];

//    for (i = 0; i < 4; i++)
//        Flash_Tx_Buffer[i + 8] = ((uint8_t *)&(m_gyr_bias[2]))[i];
//}
////读取Flash Sensor bias数据
//void u2f(void)
//{
//    uint8_t i;
//    for (i = 0; i < 4; i++)
//        ((uint8_t *)&(m_gyr_bias[0]))[i] = Flash_Rx_Buffer[i];

//    for (i = 0; i < 4; i++)
//        ((uint8_t *)&(m_gyr_bias[1]))[i] = Flash_Rx_Buffer[i + 4];

//    for (i = 0; i < 4; i++)
//        ((uint8_t *)&(m_gyr_bias[2]))[i] = Flash_Rx_Buffer[i + 8];
//}

//void Read_BiasData(void)
//{
//    SpiFlash_Read(Flash_Rx_Buffer, 0, 4 * 3);
//		//for(uint8_t i=0; i<12;i++)  CWM_OS_dbgPrintf("Flash_Rx_Buffer[%d] = \r%x\r\n",i,(uint8_t)Flash_Rx_Buffer[i]);
//    u2f();
//    if (Flash_Rx_Buffer[0] == 0xFF && Flash_Rx_Buffer[1] == 0xFF && Flash_Rx_Buffer[2] == 0xFF && Flash_Rx_Buffer[3] == 0xFF)
//    {
//        m_gyr_bias[0] = 0;
//        m_gyr_bias[1] = 0;
//        m_gyr_bias[2] = 0;
//    }
//    CWM_OS_dbgPrintf("Flash read: acc_bias=%.4f, %.4f, %.4f gyr_bias=%.4f, %.4f, %.4f\n",
//                     m_acc_bias[0], m_acc_bias[1], m_acc_bias[2],
//                     m_gyr_bias[0], m_gyr_bias[1], m_gyr_bias[2]);	
//}


//void BUTTON1_check(void)
//{
//	/*  */
//	if(nrf_gpio_pin_read(BUTTON_1) == 0)
//	{
//		platform_delay(10);
//		if(nrf_gpio_pin_read(BUTTON_1) == 0){
//			CWM_OS_dbgPrintf("Button1 Down\n\r");
//			
//			/* 按键操作 */
//			Read_BiasData();
//			
//			while(nrf_gpio_pin_read(BUTTON_1) == 0)
//					;
//		}
//	}
//}
//void BUTTON2_check(void){
//	/*  */
//	if(nrf_gpio_pin_read(BUTTON_2) == 0)
//	{
//		platform_delay(10);
//		if(nrf_gpio_pin_read(BUTTON_2) == 0){
//			CWM_OS_dbgPrintf("Button2 Down\n\r");
//			
//			memset(&scl, 0, sizeof(scl));
//			scl.iData[0] = 1;
//			scl.iData[3] = -1;
//			CWM_SettingControl(SCL_SENS_CALI_CONFIG, &scl);

//			memset(&scl, 0, sizeof(scl));
//			scl.iData[0] = 1;
//			scl.iData[1] = 5;        //gravity axis 1:x+,2:x-,3:y+,4:y-,5:z+,6:z-
//			scl.iData[2] = 5;        //vaildation time
//			scl.iData[3] = 50;       //odr
//			scl.iData[4] = 16383750; //lsb dps     LSB/dps
//			scl.iData[5] = 1800;     //acc noise   g-rms
//			scl.iData[6] = 170000;   //gyro noise  dps-rms
//			scl.iData[7] = 112500;   //acc bias    g
//			scl.iData[8] = 3300000;  //gyro bias   dps
//			CWM_SettingControl(SCL_SPV_CONFIG, &scl);

//			memset(&scl, 0, sizeof(scl));
//			scl.iData[0] = 1;
//			scl.iData[1] = 1;
//			CWM_SettingControl(SCL_SPV_MODE, &scl);

//			CWM_Sensor_Disable(IDX_ALGO_SPV);
//			CWM_Sensor_Enable(IDX_ALGO_SPV);

//			CWM_Sensor_Disable(100);
//			CWM_Sensor_Enable(100);

//			m_gyr_bias[0] = 0;
//			m_gyr_bias[1] = 0;
//			m_gyr_bias[2] = 0;
//			CWM_OS_dbgPrintf("SPV start!\n");			
//			
//			while(nrf_gpio_pin_read(BUTTON_2) == 0)
//					;
//		}
//	}
//}
//void BUTTON3_check(void){
//	/*  */
//	if(nrf_gpio_pin_read(BUTTON_3) == 0)
//	{
//		platform_delay(10);
//		if(nrf_gpio_pin_read(BUTTON_3) == 0){
//			
//			/* 按键操作 */
//			CWM_OS_dbgPrintf("Button3 Down\n\r");
//			SPIFlash_Erase_Sector(0);  //擦除Flash
//			CWM_OS_dbgPrintf("Flash erase succeed!\n\r");
//			Read_BiasData();              //读Flash BiosData		
//			
//			while(nrf_gpio_pin_read(BUTTON_3) == 0)
//					;
//		}
//	}	
//}


void CWM_AP_SensorListen(pSensorEVT_t sensorEVT)
{
    switch (sensorEVT->sensorType)
    {
    case IDX_ALGO_SPV:
    {
        float *f = sensorEVT->fData;
        if (f[0] == 1)
        {
            if (f[1] == 1)
            {
                CWM_OS_dbgPrintf("SPV ok: acc_bias=%.4f, %.4f, %.4f gyr_bias=%.4f, %.4f, %.4f\n",
                                 f[4], f[5], f[6], f[7], f[8], f[9]);

                m_gyr_bias[0] = f[7];
                m_gyr_bias[1] = f[8];
                m_gyr_bias[2] = f[9];
						 
//                SPIFlash_Erase_Sector(0);								
//                f2u();
								//for(uint8_t i=0; i<12;i++)  CWM_OS_dbgPrintf("[%d] = \r%x\r\n",i,Flash_Tx_Buffer[i]);
//                SpiFlash_Write_Page(Flash_Tx_Buffer, 0, 4 * 3);
//								Read_BiasData();
								
							
                CWM_Sensor_Disable(100);
                CWM_Sensor_Enable(100);
            }
            else
            {
                CWM_OS_dbgPrintf("SPV faile: reason=%.1f\n", f[2]);
            }
        }
        else if (f[0] == 2)
        {
            CWM_OS_dbgPrintf("SPV dbg: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
                             f[0], f[1], f[2], f[3], f[4], f[5], f[6], f[7], f[8], f[9], f[10], f[11], f[12], f[13], f[14], f[15]);
        }
				CWM_OS_dbgPrintf("Press key 4 to continue\r\n");
				while(nrf_gpio_pin_read(BUTTON_4) == 1)
							;
    }
    break;

    case 100:
        if (nrf_gpio_pin_read(BUTTON_4) == 0)
				{
            CWM_OS_dbgPrintf("orientation: yaw=%.3f, pitch=%.3f, row=%.3f\n",
                             sensorEVT->fData[0],
                             sensorEVT->fData[1],
                             sensorEVT->fData[2]);
        }
        break;
				

		
    default:
        CWM_OS_dbgPrintf("IDX[%d]: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",
                         sensorEVT->sensorType,
                         sensorEVT->fData[0],
                         sensorEVT->fData[1],
                         sensorEVT->fData[2],
                         sensorEVT->fData[3],
                         sensorEVT->fData[4],
                         sensorEVT->fData[5],
                         sensorEVT->fData[6],
                         sensorEVT->fData[7]);
        break;
    }

}

int main(void)
{	
		/* CWM接口 */
		OsAPI device_func = 
		{
        .GetTimeNs    = CWM_OS_GetTimeNs,
        .dbgOutput    = CWM_OS_dbgOutput,
        .i2cRead      = CWM_OS_i2cRead,
        .i2cWrite     = CWM_OS_i2cWrite,
        .uSleep       = CWM_OS_uSleep,
		};

		/* MCU初始化 */
    platform_init();
	
    /* Wait sensor boot time */
    platform_delay(10);
		
		/* CWM_LibPreInit */
		CWM_LibPreInit(&device_func);

		
//		Read_BiasData();     //读Flash BiosData 	
		
	
		/* 获取Lib版本信息 */
    memset(&scl, 0, sizeof(scl));
    scl.iData[0] = 1;
    CWM_SettingControl(SCL_GET_LIB_INFO, &scl);
    CWM_OS_dbgPrintf("version:%d.%d.%d.%d product:%d\n", 
											scl.iData[1],
											scl.iData[2], 
											scl.iData[3], 
											scl.iData[4], 
											scl.iData[5]);
																						
		/* 设置MCU芯片信息, 必须在CWM_LibPostInit()之前设置 */
    memset(&scl, 0, sizeof(scl));
    scl.iData[0] = 1;
    scl.iData[1] = 2; //0: mcu_auto_detect 2: skip_mcu_auto_detect
    CWM_SettingControl(SCL_CHIP_VENDOR_CONFIG, &scl);											
											
		/* CWM_LibPostInit初始化 */
    CWM_LibPostInit(CWM_AP_SensorListen);											
											
		/* 获取芯片信息 */
		char chipInfo[64];
		memset(&scl, 0, sizeof(scl));
    scl.iData[0] = 1;
    scl.iData[1] = 1;
    scl.iData[2] = (int)chipInfo;
    scl.iData[3] = sizeof(chipInfo);
    scl.iData[4] = 0;
    scl.iData[5] = 0;
    scl.iData[6] = 0;
    CWM_SettingControl(SCL_GET_CHIP_INFO, &scl);	
    CWM_OS_dbgPrintf("have_security = %d.%d ret_buff_size = %d  chipInfo = %s\n", 
											scl.iData[5], 
											scl.iData[6], 
											scl.iData[4], 
											chipInfo);
    CWM_OS_dbgPrintf("chip_settings = %d, %d, %d\n", 
											scl.iData[9], 
											scl.iData[10], 
											scl.iData[11]);
											
		CWM_Dml_LibInit();
		
		
		uint8_t whomi;
		CWM_OS_i2cRead(0x6b,0x0f,1,&whomi,1,100);
		CWM_OS_dbgPrintf("whomi = %d\r\n",whomi);
		
		
		memset(&scl, 0, sizeof(scl));
    scl.iData[0] = 1;
    CWM_SettingControl(SCL_DML_GET_LIB_INFO, &scl);
    CWM_OS_dbgPrintf("dml version:%d.%d.%d.%d product:%d model:%d\n", scl.iData[1], scl.iData[2], scl.iData[3], scl.iData[4], scl.iData[5], scl.iData[6]);
		

#if 0
    memset(&scl, 0, sizeof(scl));
    scl.iData[0] = 1;
    scl.iData[1] = 4;
    CWM_SettingControl(SCL_DML_DEBUG, &scl);
#endif

		memset(&scl, 0, sizeof(scl));
    scl.iData[0] = 1;
    scl.iData[1] = 1;
    scl.iData[2] = 9;
    CWM_SettingControl(SCL_DML_DRV_HW_CONFIG, &scl);		

		memset(&scl, 0, sizeof(scl));
    scl.iData[0] = 1;
    scl.iData[3] = 1;
    scl.iData[4] = 2;
    scl.iData[8] = 52;
    CWM_SettingControl(SCL_DML_DRV_AG_CONFIG, &scl);		

    memset(&scl, 0, sizeof(scl));
    scl.iData[0] = 1;
    scl.iData[1] = 1;
    CWM_SettingControl(SCL_DML_DRV_INIT, &scl); 
		
		CWM_OS_dbgPrintf("DML device=%d\n", scl.iData[2]);
		
    memset(&scl, 0, sizeof(scl));
    scl.iData[0] = 1;
    scl.iData[1] = 1;
    scl.iData[2] = 3;
    CWM_SettingControl(SCL_DML_DRV_ENABLE, &scl);		
		
		/* 打开log输出 */
    memset(&scl, 0, sizeof(scl));
    scl.iData[0] = 1;
    scl.iData[3] = 3;
    scl.iData[4] = 5;
    CWM_SettingControl(SCL_LOG, &scl);
		
//	  CWM_Sensor_Enable(CUSTOM_ACC);
//    CWM_Sensor_Enable(CUSTOM_GYRO);						


    memset(&scl, 0, sizeof(scl));
    scl.iData[0] = 1;
    scl.iData[2] = 4; //low speed 4dps	
		scl.iData[9] = 30;
//		CWM_SettingControl(SCL_HS_ORIEN_CONFIG, &scl);	

		CWM_Sensor_Enable(100);

  while(1)
  {		

			CWM_Dml_process();
				
  }
}
	
			



//static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
//{
//		
//		CWM_OS_i2cWrite(LSM6DSO_I2C_ADD_H, reg, 1, bufp, (uint16_t)len);
//		return 0;
//}


//static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
//{
//		CWM_OS_i2cRead(LSM6DSO_I2C_ADD_H, reg, 1, bufp, (uint16_t)len);
//		return 0;
//}


static void platform_delay(uint32_t ms)
{
    nrf_delay_ms(ms);
}


static void platform_init(void)
{
    board_init();
}



