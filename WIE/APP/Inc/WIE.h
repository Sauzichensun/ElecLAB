/*******************************************************************************
* 版    权：HUAV
* 文 件 名：drv_wis100.h
* 创建日期：2021/09/17
* 修订日期：2021/09/17
* 描    述：WIS100(IMU)驱动程序，协议解析
*******************************************************************************/

#ifndef WIE_H
#define WIE_H

#include <stdint.h>
#include <stdbool.h>

/* WIS100传感器数据结构体 */
#pragma pack(1)
typedef struct
{
    uint32_t time_stamp;        /* 数据获取时间戳 系统时间 */
    uint64_t real_time_stamp;   /* 数据绝对时间戳 UTC毫秒时间 */
    
    uint8_t msg_cnt;            /* 消息序号，0~255循环 */
    
    double gyr_x;               /* x轴陀螺  deg/s */
    double gyr_y;               /* y轴陀螺 */
    double gyr_z;               /* z轴陀螺 */
 
    double acc_x;               /* x轴加表  g */
    double acc_y;               /* y轴加表 */
    double acc_z;               /* z轴加表 */
    
    float gyr_temp;             /* 陀螺温度  ℃ */
    float acc_temp;             /* 加表温度 */
} WIS100Data_TypeDef;
#pragma pack()

/* WIS100驱动对象结构体 */
typedef struct
{
    uint8_t (*init)(void);
    void (*data_recv)(uint8_t *p_recv, uint16_t recv_len);
    uint8_t (*data_ready)(void);
    void (*get_origin_data)(WIS100Data_TypeDef *p_data);
    void (*dr_trigger)(uint64_t real_time, uint32_t sys_time);
} WIS100_Typedef;


extern WIS100_Typedef DrvWIS100;
void WIS100_Data_Recv(uint8_t *p_recv, uint16_t recv_len,uint8_t *flag);

#endif

