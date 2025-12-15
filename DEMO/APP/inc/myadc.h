#ifndef MYADC_H
#define MYADC_H

#include "myadc.h"


#define ADC_DATA_SIZE 4096U // ADC采样数据大小
#define ADC_BITS 14U       
#define ADC_RESOLUTION ((1 << ADC_BITS) - 1) // ADC分辨率对应的最大值

void ADC_DMA_Start(void);
void ADC_UpdateCache(void);
void usb_send_adc3_data(void);

#endif

