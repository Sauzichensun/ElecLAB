#include "main.h"
#include "mydac.h"
#include "tim.h"
#include "dac.h"


// 固定长度的查找表，避免反复 malloc
#define DAC_TABLE_LEN 1024U          // 256/512/1024 均可
#define DAC_MAX_FS    800000U       // DAC 实际能跑的更新率上限，保守 400~800 kSa/s
static uint16_t dac_lut[DAC_TABLE_LEN];

static inline uint32_t tim6_get_timerclk_hz(void) {
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    // APB1 分频>1 时，定时器时钟翻倍
    if ((RCC->CFGR & RCC_CFGR_PPRE1_Msk) != RCC_CFGR_PPRE1_DIV1) return pclk1 * 2U;
    return pclk1;
}

void mydac_build_sine(void) 
{
    for (uint32_t i = 0; i < DAC_TABLE_LEN; ++i) 
    {
        float x = 2.0f * 3.1415926f * (float)i / (float)DAC_TABLE_LEN;
        dac_lut[i] = (uint16_t)((sinf(x) * 0.5f + 0.5f) * 4095.0f);  // 0..4095
    }
}

// 设定输出频率（Hz）
void mydac_set_freq(uint32_t f_out_hz) {
    if (f_out_hz < 1U) f_out_hz = 1U;
    // 采样率 = f_out * N；受 DAC 最大更新率限制
    uint32_t fs_req = f_out_hz * DAC_TABLE_LEN;
    if (fs_req > DAC_MAX_FS) { fs_req = DAC_MAX_FS; f_out_hz = fs_req / DAC_TABLE_LEN; }

    // 已设 TIM6 PSC=83 -> 计数时钟=timclk/(PSC+1)
    uint32_t timclk = tim6_get_timerclk_hz();
    uint32_t psc    = __HAL_TIM_GET_PRESCALER(&htim6) + 1U;
    uint32_t cntclk = timclk / psc;
    uint32_t arr    = cntclk / fs_req;
    if (arr < 2U) arr = 2U;  // 防止 ARR 太小不稳定

    __HAL_TIM_DISABLE(&htim6);
    __HAL_TIM_SET_AUTORELOAD(&htim6, arr - 1U);
    __HAL_TIM_SET_COUNTER(&htim6, 0U);
    __HAL_TIM_ENABLE(&htim6);
}

void mydac_tim_init(void) {
    // CubeMX 已配置 TIM6: TRGO=Update, PSC=83 等
    // 在启动 DAC DMA 之后再启动计时器
}

uint8_t DAC_DMA_Start(void) {
    // 先启动 DMA（循环模式），再启动 TIM6
    if (HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)dac_lut, DAC_TABLE_LEN, DAC_ALIGN_12B_R) != HAL_OK)
        return 0;
    if (HAL_TIM_Base_Start(&htim6) != HAL_OK)
        return 0;
    return 1;
}
