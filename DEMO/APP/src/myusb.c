#include "myusb.h"
#include "main.h"
#include "usbd_cdc_if.h"
#include <string.h>


JustFloatVofa adc_frame;


uint8_t buf[100000];

void JustFloatVofa_init(JustFloatVofa* frame)
{

    frame->Data_Length=0;//初始化数据长度0
    /* 数据帧尾初始化 （IEEE754 浮点数 0x7F800000 表示 +Inf, 但这里保持之前的字节顺序）*/
    frame->tail[0] = 0x00;
    frame->tail[1] = 0x00;
    frame->tail[2] = 0x80;
    frame->tail[3] = 0x7f;
}

void JustFloatVofa_setData(JustFloatVofa* frame, const float* data1,const float* data2,uint16_t len)
{
    if (data1 == NULL || frame == NULL) return;
    if (len > DATA_LEN_MAX) len = DATA_LEN_MAX;
    frame->Data_Length = len;
	uint16_t i=0;
    if(data1!=NULL)
    {
        for (i = 0; i < len; i++) 
        {
            frame->fdata[0][i] = data1[i];
        } 
        SCB_CleanDCache_by_Addr((uint32_t*)frame->fdata[0], sizeof(frame->fdata[0]));
    }
    
    if(CH_COUNT==2)
    {
        for(i = 0; i < len; i++) 
        {
            frame->fdata[1][i] = data2[i];
        }
        SCB_CleanDCache_by_Addr((uint32_t*)frame->fdata[1], sizeof(frame->fdata[1]));
    }
    
}

uint8_t JustFloatVofa_send(JustFloatVofa* frame)
{
    if (frame == NULL) return 1;
    uint16_t offset=0;
    uint16_t payload_len = (uint16_t)(CH_COUNT * (sizeof(float) + 4) * frame->Data_Length);
    
    for(uint16_t i=0;i<frame->Data_Length;i++)
    {
        if(CH_COUNT==1)
        {
            memcpy(buf+offset,(uint8_t*)&(frame->fdata[0][i]),sizeof(float));
            offset+=sizeof(float);
            memcpy(buf+offset,(uint8_t*)(frame->tail),4);
						offset+=4;
        }
        else if(CH_COUNT==2)
        {
            memcpy(buf+offset,(uint8_t*)&(frame->fdata[0][i]),sizeof(float));
            offset+=sizeof(float);
            memcpy(buf+offset,(uint8_t*)&(frame->fdata[1][i]),sizeof(float));
            offset+=sizeof(float);
            memcpy(buf+offset,(uint8_t*)(frame->tail),4);
						offset+=4;
        }
    }

    return CDC_Transmit_FS(buf, payload_len);
}



