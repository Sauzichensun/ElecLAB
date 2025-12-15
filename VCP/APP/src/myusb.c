#include "myusb.h"
#include "main.h"
#include "usbd_cdc_if.h"
#include <string.h>

JustFloatVofa adc_frame;
float adc_frame_values[CH_COUNT] = {0.0f};

void JustFloatVofa_init(JustFloatVofa* frame)
{
    for (uint8_t i = 0; i < CH_COUNT; ++i) {
        frame->fdata[i] = 0.0f;
    }
    /* 数据帧尾初始化 （IEEE754 浮点数 0x7F800000 表示 +Inf, 但这里保持之前的字节顺序）*/
    frame->tail[0] = 0x00;
    frame->tail[1] = 0x00;
    frame->tail[2] = 0x80;
    frame->tail[3] = 0x7f;
}

void JustFloatVofa_setData(JustFloatVofa* frame, const float* data, uint8_t len)
{
    if (data == NULL || frame == NULL) return;
    if (len > CH_COUNT) len = CH_COUNT;
    for (uint8_t i = 0; i < len; ++i) {
        frame->fdata[i] = data[i];
    }
}

uint8_t JustFloatVofa_send(JustFloatVofa* frame)
{
    if (frame == NULL) return 1;
    uint16_t payload_len = (uint16_t)(CH_COUNT * sizeof(float) + 4);
    uint8_t buf[CH_COUNT * sizeof(float) + 4];
    /* 将 float 数据与尾部按字节打包 */
    memcpy(buf, frame->fdata, CH_COUNT * sizeof(float));
    memcpy(buf + CH_COUNT * sizeof(float), frame->tail, 4);

    return CDC_Transmit_FS(buf, payload_len);
}

void mytime_tim14_callback(void)
{
    adc_frame_values[0] += 1.0f;  // 示例：简单递增
    if(adc_frame_values[0] > 100.0f) {
        adc_frame_values[0] = 0.0f; // 重置
    }
    JustFloatVofa_setData(&adc_frame, adc_frame_values, CH_COUNT);
    JustFloatVofa_send(&adc_frame);

}
