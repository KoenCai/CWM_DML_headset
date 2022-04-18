#ifndef __CWM_LIB_API_H__
#define __CWM_LIB_API_H__

#include "board_nordic.h"
#include "cwm_lib.h"
#include "config.h"

#define SIGNAL_SIMULATE_FIFO_COUNT 10

void cwm_init(void);
void DataInput(int SensorTyp,float data_x,float data_y,float data_z);
void Cwm_fifo_start(int SensorTyp,int DTms);
void Cwm_fifo_end(int SensorTyp);


#endif //__CWM_LIB_API_H__
