#include "myadc.h"
#include "adc.h"
#include "main.h"
#include "adc.h"
#include "myusb.h"
#include "tim.h"

/*
ADC配置：adc3  PC0  TIM6触发  2M采样率   转换率
ADC时钟50M以下，cubemax配置要改 必须检查adc时钟
*/

extern JustFloatVofa adc_frame;


volatile uint8_t half_flag = 0; // DMA传输一半完成标志位
volatile uint8_t full_flag = 0; // DMA传输全部完成标志位

//STM32H7 的 ADC3 位于 D3 域，它只能使用 BDMA (Basic DMA)，且 BDMA 只能访问 SRAM4 (地址 0x38000000)。
//如果内存分配错误，DMA 将无法搬运数据（结果全为0）
//这块划为MPU单元，防止用cache，导致数据更新不及时问题

#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
  uint16_t adc_PC0_data[ADC_DATA_SIZE] __attribute__((at(0x38000000)));
#else
  #error "Please specify buffer location for your compiler!"
#endif

float adc_PC0_voltage[ADC_DATA_SIZE]; // 用于存储转换成电压后的数据

void ADC_DMA_Start(void) 
{ 
    
    // 启动ADC DMA传输
    HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    HAL_Delay(100);//等待校准
    HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc_PC0_data, ADC_DATA_SIZE);
    HAL_TIM_Base_Start(&htim6);
}


void ADC_UpdateCache(void)
{
    SCB_InvalidateDCache_by_Addr((uint32_t*)adc_PC0_data, sizeof(adc_PC0_data));
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    // 判断是否是 ADC3 触发的中断
    if (hadc->Instance == ADC3)
    {
      if(half_flag==0) half_flag = 1;
    }
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    // 判断是否是 ADC3 触发的中断
    if (hadc->Instance == ADC3)
    {
      //避免丢帧 
      if(full_flag==0) full_flag = 1;
      
      
    }
}

void usb_send_adc3_data(void)
{
  if(half_flag == 1)
  {
    half_flag = 0;
    for(uint16_t i=0;i<ADC_DATA_SIZE/2;i++)
    {
        adc_PC0_voltage[i] = ((float)adc_PC0_data[i]) * 3.3f / ADC_RESOLUTION; // 转换为电压值
    }
    JustFloatVofa_setData(&adc_frame, (const float*)adc_PC0_voltage, NULL, ADC_DATA_SIZE/2);
    JustFloatVofa_send(&adc_frame);
  }
  if(full_flag == 1)
  {
    full_flag = 0;
    for(uint16_t i=0;i<ADC_DATA_SIZE/2;i++)
    {
        adc_PC0_voltage[i+ADC_DATA_SIZE/2] = ((float)adc_PC0_data[i+ADC_DATA_SIZE/2]) * 3.3f / ADC_RESOLUTION; // 转换为电压值
    }

    JustFloatVofa_setData(&adc_frame, (const float*)&adc_PC0_voltage[ADC_DATA_SIZE/2], NULL, ADC_DATA_SIZE/2);
    JustFloatVofa_send(&adc_frame);
  }
}

/**
  * @brief  ADC 错误回调（可选，用于调试）
  */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
    // 如果发生 Overrun (OVR)，通常是因为数据处理太慢，或者定时器没及时停
    // 可以在这里打断点调试
}
