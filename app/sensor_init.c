//#include "cwm_lib_api.h"
//#include "sensor_init.h"
//#include "board_nordic.h"
//#include "lsm6dso_reg.h"




//typedef union{
//  int16_t i16bit[3];
//  uint8_t u8bit[6];
//} axis3bit16_t;

///* Private macro -------------------------------------------------------------*/

///* Private variables ---------------------------------------------------------*/

//static axis3bit16_t data_raw_acceleration;
//static axis3bit16_t data_raw_angular_rate;
//static float acceleration_mg[3];
//static float angular_rate_mdps[3];
//static uint8_t whoamI, rst;

//float acc_in[3] = {0};
//float gyr_in[3] = {0};

///* Extern variables ----------------------------------------------------------*/

///* Private functions ---------------------------------------------------------*/
//static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
//static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
//static void platform_delay(uint32_t ms);
////static void platform_init(void);


//stmdev_ctx_t dev_ctx;

//void lsm6dso_init(void)
//{
//    /* Initialize mems driver interface */
//		stmdev_ctx_t dev_ctx;
//    dev_ctx.write_reg = platform_write;
//    dev_ctx.read_reg = platform_read;
//    dev_ctx.handle = NULL;	
//			
//    /* Wait sensor boot time */
//			platform_delay(10);			

//		/* LSM6DSOW 参数设置 */	
//		/* 检查设备ID */
//    lsm6dso_device_id_get(&dev_ctx, &whoamI);
//    if (whoamI != LSM6DSO_ID)
//        while (1)
//            ;
//		printf("LSM6DSO Sensor ID： %d \r\n",LSM6DSO_ID);
//		platform_delay(1000);		
//		
//    /* LSM6DSOW reset配置 */
//    lsm6dso_reset_set(&dev_ctx, PROPERTY_ENABLE);
//    do
//    {
//        lsm6dso_reset_get(&dev_ctx, &rst);
//    } while (rst);			
//		
//		/* Disable I3C interface */
//    lsm6dso_i3c_disable_set(&dev_ctx, LSM6DSO_I3C_DISABLE);
//		

//    /* 启用BDU块更新 */
//    lsm6dso_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

//    /* 加速度计量程 */
//    lsm6dso_xl_full_scale_set(&dev_ctx, LSM6DSO_16g);
//		
//		/* 陀螺仪量程 */
//    lsm6dso_gy_full_scale_set(&dev_ctx, LSM6DSO_2000dps);

//    /* 设置输出 加速度计ODR 为26Hz */
//    lsm6dso_xl_data_rate_set(&dev_ctx, LSM6DSO_XL_ODR_52Hz);
//		
//		/* 设置输出 陀螺仪ODR 为26Hz */
//    lsm6dso_gy_data_rate_set(&dev_ctx, LSM6DSO_GY_ODR_52Hz);
//		
//		
//    /* 设置 FIFO watermark (number of unread sensor data TAG + 6 bytes stored in FIFO) to 10 samples （set val = 20 -> COUNT = 10 )*/
//    lsm6dso_fifo_watermark_set(&dev_ctx, 20);

//    /* 设置 加速度计 FIFO ODR为26Hz */
//    lsm6dso_fifo_xl_batch_set(&dev_ctx, LSM6DSO_XL_BATCHED_AT_52Hz);
//		
//		/* 设置 陀螺仪 FIFO ODR为26Hz */
//    lsm6dso_fifo_gy_batch_set(&dev_ctx, LSM6DSO_GY_BATCHED_AT_52Hz);

//    /* 将FIFO模式设置为流模式(即连续模式)   */
//    lsm6dso_fifo_mode_set(&dev_ctx, LSM6DSO_STREAM_MODE);

//	
//}

//void lsm6dso_getfifo(void)
//{
//		
//			uint16_t num = 0;
//			uint8_t wmflag = 0;
//			lsm6dso_fifo_tag_t reg_tag;
//			axis3bit16_t dummy;
//			
//			//int64_t next_duration = CWM_GetNextActionDuration_ns();
//					
//			/* Read watermark flag */
//			lsm6dso_fifo_wtm_flag_get(&dev_ctx, &wmflag);			
//		  if (wmflag > 0)
//			{		

//						
//								
//				  /* Read number of samples in FIFO */
//					lsm6dso_fifo_data_level_get(&dev_ctx, &num);
//					//printf("num: %d\r\n",num);
//					/*CWM FIFO START*/
//							Cwm_fifo_start(CUSTOM_ACC,20);
//							Cwm_fifo_start(CUSTOM_GYRO,20);
//					while(num--)
//					{
//						 lsm6dso_fifo_sensor_tag_get(&dev_ctx, &reg_tag);
//						 switch(reg_tag)
//							{
//								case LSM6DSO_XL_NC_TAG:
//									memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
//									lsm6dso_fifo_out_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
//									acceleration_mg[0] = lsm6dso_from_fs16_to_mg(data_raw_acceleration.i16bit[0]);
//									acceleration_mg[1] = lsm6dso_from_fs16_to_mg(data_raw_acceleration.i16bit[1]);
//									acceleration_mg[2] = lsm6dso_from_fs16_to_mg(data_raw_acceleration.i16bit[2]);

//									acc_in[0] = (float)acceleration_mg[0] * 9.81f / 1000;
//									acc_in[1] = (float)acceleration_mg[1] * 9.81f / 1000;
//									acc_in[2] = (float)acceleration_mg[2] * 9.81f / 1000;
//									
//									//将数据传入LIB
//									DataInput(CUSTOM_ACC,acc_in[0],acc_in[1],acc_in[2]);	

//									break;
//									

//								
//								case LSM6DSO_GYRO_NC_TAG:
//									memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));
//									lsm6dso_fifo_out_raw_get(&dev_ctx, data_raw_angular_rate.u8bit);
//									angular_rate_mdps[0] = lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[0]);
//									angular_rate_mdps[1] = lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[1]);
//									angular_rate_mdps[2] = lsm6dso_from_fs2000_to_mdps(data_raw_angular_rate.i16bit[2]);

//									gyr_in[0] = (float)angular_rate_mdps[0] * 3.1416f / 180 / 1000;
//									gyr_in[1] = (float)angular_rate_mdps[1] * 3.1416f / 180 / 1000;
//									gyr_in[2] = (float)angular_rate_mdps[2] * 3.1416f / 180 / 1000;
//								
//									//将数据传入LIB			 
//									DataInput(CUSTOM_GYRO,gyr_in[0],gyr_in[1],gyr_in[2]);								
//																						
//									break;
//									

//								
//								default:
//									/* Flush unused samples */
//									memset(dummy.u8bit, 0x00, 3 * sizeof(int16_t));
//									lsm6dso_fifo_out_raw_get(&dev_ctx, dummy.u8bit);
//									break;
//								
//							}
//									Cwm_fifo_end(CUSTOM_ACC);
//									Cwm_fifo_end(CUSTOM_GYRO);
//							
//					}

//					/* 算法计算 */
//					CWM_process();								
//		}		
//	}		


////void icm42607_getpolling(float *accel_g, float *gyro_dps)
////{
////    acc_in[0] = accel_g[0] * 9.80665f;
////    acc_in[1] = accel_g[1] * 9.80665f;
////    acc_in[2] = accel_g[2] * 9.80665f;

////    gyr_in[0] = gyro_dps[0] * 3.14159f / 180;
////    gyr_in[1] = gyro_dps[1] * 3.14159f / 180;
////    gyr_in[2] = gyro_dps[2] * 3.14159f / 180;	
////		
////		DataInput(CUSTOM_ACC,acc_in[0],acc_in[1],acc_in[2]);
////		DataInput(CUSTOM_GYRO,gyr_in[0],gyr_in[1],gyr_in[2]);	
////		CWM_process();	
////}


//static void platform_delay(uint32_t ms)
//{
//    nrf_delay_ms(ms);
//}

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


