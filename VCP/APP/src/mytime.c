#include "mytime.h"
#include "main.h"
#include "tim.h"
#include "myusb.h"

extern TIM_HandleTypeDef htim14;
extern JustFloatVofa adc_frame;
extern float adc_frame_values[CH_COUNT];

//启动定时器
void mytime_start(TIM_HandleTypeDef *htim)
{
    /* 启动 TIM14 基本定时器并使能中断 */
    HAL_TIM_Base_Start_IT(htim);
}

void mytime_stop(TIM_HandleTypeDef *htim)
{
    HAL_TIM_Base_Stop_IT(htim);
}



/* HAL 的定时器周期回调，转发给 TIM14 专用回调 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim14 || htim->Instance == htim14.Instance) {
        //mytime_tim14_callback();

    }
}

