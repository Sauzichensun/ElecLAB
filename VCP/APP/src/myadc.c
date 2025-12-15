#include "myadc.h"
#include "adc.h"
#include "main.h"
#include "tim.h"
#include "adc.h"
#include "myusb.h"

/*
*ADC的DMA传输正常，不循环
1M采样率 TM3触发-84M时钟*/



volatile uint8_t half_flag = 0; // DMA传输一半完成标志位
volatile uint8_t full_flag = 0; // DMA传输全部完成标志位

uint16_t adc_PB0_data[ADC_DATA_SIZE]; // ADC采样数据数组


void ADC_DMA_Start(void) 
{ 
    // 启动ADC DMA传输
    
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_PB0_data, ADC_DATA_SIZE);
    HAL_TIM_Base_Start(&htim3);
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) half_flag = 1;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) full_flag = 1;
}

