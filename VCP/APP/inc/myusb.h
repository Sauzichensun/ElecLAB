#ifndef __MYUSB_H
#define __MYUSB_H

#include <stdint.h>

/* 通道数，可按需修改 */
#define CH_COUNT 4096


typedef struct {
	float fdata[CH_COUNT];
	uint8_t tail[4];
} JustFloatVofa;



/* 初始化帧（清零数据并设置帧尾） */
void JustFloatVofa_init(JustFloatVofa* frame);

/* 设置数据：从 data 复制最多 len 个 float 到帧中 */
void JustFloatVofa_setData(JustFloatVofa* frame, const float* data, uint8_t len);

/* 发送帧，通过 CDC_Transmit_FS 返回值：0=OK, 其它为错误 */
uint8_t JustFloatVofa_send(JustFloatVofa* frame);

void mytime_tim14_callback(void);

#endif
