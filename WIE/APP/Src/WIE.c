/*******************************************************************************
* 版    权：HUAV
* 文 件 名：drv_wis100.c
* 修订日期：2021/09/17
* 描    述：WIS100(IMU)驱动程序，协议解析
*******************************************************************************/

#include <string.h>
#include "WIE.h"

#define WIS100_START_CODE_1         0xAA                    //起始符号定义
#define WIS100_START_CODE_2         0x55                    //起始符号定义
#define WIS100_PROTO_LEN            34
#define WIS100_RECV_BUFF_LEN        (WIS100_PROTO_LEN * 32)
#define WIS100_CMD_MAX_LEN          512

#pragma pack(1)
/* WIS100协议数据结构体 */
typedef struct
{
    uint8_t msg_cnt;        /* 循环计数，0-255 */
    
    int32_t gyr_x;          /* x轴陀螺，单位：deg/s，比例系数1e-5 */
    int32_t gyr_y;          /* y轴陀螺，单位：deg/s，比例系数1e-5 */
    int32_t gyr_z;          /* z轴陀螺，单位：deg/s，比例系数1e-5 */
    
    int32_t acc_x;          /* x轴加表，单位：g，比例系数1e-6 */
    int32_t acc_y;          /* y轴加表，单位：g，比例系数1e-6 */
    int32_t acc_z;          /* z轴加表，单位：g，比例系数1e-6 */
    
    int16_t gyr_temp;       /* 陀螺温度  单位：℃，比例系数1/256 */
    int16_t acc_temp;       /* 加表温度，单位：℃，比例系数1/256 */
    
    uint8_t reserve;        /* 预留 */
} WIS100ProtoData_TypeDef;


#pragma pack()


/* 外部调用接口函数 */
//static void WIS100_Data_Recv(uint8_t *p_recv, uint16_t recv_len);
static uint8_t WIS100_Data_Ready(void);
static void WIS100_Get_Origin_Data(WIS100Data_TypeDef *p_data);
static uint8_t WIS100_Init(void);

/* 需要用户实现的接口函数 */
__weak uint32_t WIS100_GetTick(void){return 0;};            //获取系统时间
__weak uint64_t WIS100_GetRealMs(void){return 0;};          //获取绝对时间

uint8_t WIS100RecvBuff[WIS100_RECV_BUFF_LEN];               //用于协议组包的临时缓存
uint32_t WIS100RecvCnt = 0;                                 //用于统计数据丢包率
uint8_t WIS100DataReady = 0;
uint8_t WIS100ReadFlag = 0;
uint8_t WIS100WriteFlag = 0;
WIS100ProtoData_TypeDef WIS100ProtoData;
WIS100Data_TypeDef WIS100Data;

WIS100_Typedef DrvWIS100 = 
{
    .init            = WIS100_Init,
    .data_recv       = NULL,                //WIS100_Data_Recv,
    .data_ready      = WIS100_Data_Ready,    
    .get_origin_data = WIS100_Get_Origin_Data
};

uint32_t WIS100CheckSum_ErrCnt = 0;
uint32_t WIS100BuffFull_ErrCnt = 0;
uint32_t Read_ErrCnt = 0;
uint32_t Write_ErrCnt = 0;
uint32_t WIS100CurrentRecvLen = 0;
uint32_t WIS100TotalRecvCnt = 0;

void WIS100_Data_Recv(uint8_t *p_recv, uint16_t recv_len,uint8_t *flag)
{
    static uint16_t read_index = 0, write_index = 0;
    uint8_t *p_buff = WIS100RecvBuff, *p_proto = NULL, check_sum;
    uint16_t i, j;
    
    WIS100CurrentRecvLen = recv_len;
    WIS100TotalRecvCnt += WIS100CurrentRecvLen;
    
    /* 参数检查 */
    if (NULL == p_recv || 0 == recv_len || recv_len > sizeof(WIS100RecvBuff))
    {
        return;
    }
    
    /* 接收的数据存入缓存剩余空间，超出部分舍弃 */
    if (recv_len > (sizeof(WIS100RecvBuff) - write_index))
    {
        WIS100BuffFull_ErrCnt ++;
        
        memset(WIS100RecvBuff, 0, sizeof(WIS100RecvBuff));//清空临时缓存
        read_index = 0;
        write_index = 0;
        return;
    }
    memcpy(p_buff + write_index, p_recv, recv_len);
    write_index += recv_len;
    
    /* 协议组包 */
    for(i = read_index; i < (write_index - read_index); )
    {
        /* 读写位置异常 */
        if (write_index <= read_index)
        {
            break;
        }
        
        /* 剩余部分不够一个完整帧 */
        if (write_index - read_index < WIS100_PROTO_LEN)
        {
            break;
        }
        
        /* 找到帧头 */
        if (WIS100_START_CODE_1 == p_buff[i] && \
            WIS100_START_CODE_2 == p_buff[i + 1])
        {
            p_proto = p_buff + i;
            
            /* 剩余部分不够一个完整帧 */
            if (write_index - i < WIS100_PROTO_LEN)
            {
                break;
            }
            
            /* 检查校验位 */
            check_sum = 0;
            for(j = 0; j < WIS100_PROTO_LEN - 1; j++)
            {
                check_sum += p_proto[j];
            }
            if (check_sum == p_proto[WIS100_PROTO_LEN - 1])
            {
                /* 找到完整数据帧 */
                memcpy(&WIS100ProtoData, p_proto + 3, sizeof(WIS100ProtoData_TypeDef));
                
                /* 应用层数据读取完成后再更新，避免读写冲突 */
                if (!WIS100ReadFlag)
                {
                    WIS100WriteFlag = 1;
                    
                    /* IMU数据更新 */
                    WIS100Data.time_stamp = WIS100_GetTick();
                    
                    WIS100Data.msg_cnt = WIS100ProtoData.msg_cnt;
                    
                    WIS100Data.gyr_x = (double)WIS100ProtoData.gyr_x / 1e5*3.1415926/180.0;
                    WIS100Data.gyr_y = (double)WIS100ProtoData.gyr_y / 1e5*3.1415926/180.0;
                    WIS100Data.gyr_z = (double)WIS100ProtoData.gyr_z / 1e5*3.1415926/180.0;
                    
                    WIS100Data.acc_x = (double)WIS100ProtoData.acc_x / 1e6;
                    WIS100Data.acc_y = (double)WIS100ProtoData.acc_y / 1e6;
                    WIS100Data.acc_z = (double)WIS100ProtoData.acc_z / 1e6;
                    
                    WIS100Data.gyr_temp = (float)WIS100ProtoData.gyr_temp / 256.0f;
                    WIS100Data.acc_temp = (float)WIS100ProtoData.acc_temp / 256.0f;
                    
                    WIS100Data.real_time_stamp = WIS100_GetRealMs();
                    
                    WIS100WriteFlag = 0;
                    WIS100DataReady = 1;
                    WIS100RecvCnt ++;
                    *flag=1;
                }
                else
                {
                    Write_ErrCnt ++;
                }
                
                i += WIS100_PROTO_LEN;
                p_proto = NULL;
            }
            else
            {
                i ++;
                p_proto = NULL;
                WIS100CheckSum_ErrCnt ++;
            }
        }
        else
        {
            i ++;
        }
    }
    
    read_index = i;
    
    /* 数据全部已读或者读写索引位置异常 */
    if ((read_index >= write_index) || \
        (write_index == sizeof(WIS100RecvBuff)))
    {
        memset(WIS100RecvBuff, 0, sizeof(WIS100RecvBuff));
        read_index = 0;
        write_index = 0;
    }
}

static uint8_t WIS100_Data_Ready(void)
{
    if (WIS100DataReady)
    {
        WIS100DataReady = 0;
        return 1;
    }
    else
    {
        return 0;
    }
}

static void WIS100_Get_Origin_Data(WIS100Data_TypeDef *p_data)
{
    if (WIS100WriteFlag)
    {
        Read_ErrCnt ++;
        return;
    }
    
    WIS100ReadFlag = 1;
    memcpy(p_data, &WIS100Data, sizeof(WIS100Data_TypeDef));
    WIS100ReadFlag = 0;
}

static uint8_t WIS100_Init(void)
{
    DrvWIS100.data_recv = WIS100_Data_Recv;
    return 1;
}
