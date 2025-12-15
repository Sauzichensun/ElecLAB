#include "main.h"
#include "mydac.h"
#include "tim.h"
#include "dac.h"
#include "const.h"

//dac1 PA4  TIM7触发  1MHz输出频率

// 固定长度的查找表，避免反复 malloc
#define DAC_TABLE_LEN 4000U          
#define DAC_MAX_FS    1e6f       //

#if defined(__CC_ARM)
//定义在SRAM1区，加速访问速度
uint16_t dac_PA4_lut[DAC_TABLE_LEN] __attribute__((at(0x30000000)));
#else
#error "Please specify buffer location for your compiler!"
#endif


void mydac_set_data(void) 
{
    for (uint32_t i = 0; i < DAC_TABLE_LEN; ++i) 
    {
        float f =200e3f;
        float x = 2.0f * pi * (float)i * f / (float)DAC_MAX_FS;
        dac_PA4_lut[i] = (uint16_t)((sinf(x) * 0.5f + 0.5f) * 4095.0f);  // 0..4095
    }
    //把缓存刷到内存
    SCB_CleanDCache_by_Addr((uint32_t*)dac_PA4_lut, sizeof(dac_PA4_lut));
}





uint8_t mydac_start_out(void)
{
    // 先启动 DMA（循环模式），再启动 TIM6
    if (HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)dac_PA4_lut, DAC_TABLE_LEN, DAC_ALIGN_12B_R) != HAL_OK)
        return 0;
    if (HAL_TIM_Base_Start(&htim7) != HAL_OK)
        return 0;
    return 1;
}
