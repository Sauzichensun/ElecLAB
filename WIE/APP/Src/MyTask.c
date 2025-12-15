#include "WIE.h"
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "stm32h7xx_hal_usart.h"
#include "stdio.h"
#include "math.h"
#include "jiesuan.h"
#include "MyTask.h"
#include "othertask.h"


#define TIM3_500US 500//定时器3的500us中断
#define NoValidSPC 0xffff//无有效数据

double gyr_x;               /* x轴陀螺  deg/s */
double gyr_y;               /* y轴陀螺 */
double gyr_z;               /* z轴陀螺 */
double acc_x;              /* x轴加速度 */
double acc_y;              /* y轴加速度 */
double acc_z;              /* z轴加速度 */
Imu_Typedef Imu1={0,0,0,5,0.01,0,0.001};

/* 定时器中断回调函数 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static uint32_t count=0;
    static uint16_t CntOmitData=0;
    if(htim->Instance==htim1.Instance)//1000hz-1ms
    {
        count++;
        
        if(count>=1000)
        {
            count=0;
            // printf("CheckSum_ErrCnt:%d,BuffFull_ErrCnt:%d,Read_ErrCnt:%d,Write_ErrCnt:%d,TotalRecvCnt:%d,CurrentRecvLen:%d\n",
            // WIS100CheckSum_ErrCnt,WIS100BuffFull_ErrCnt,Read_ErrCnt,Write_ErrCnt,WIS100TotalRecvCnt,WIS100CurrentRecvLen);
        }
        
        gyr_x=WIS100Data.gyr_x;
        gyr_y=WIS100Data.gyr_y;
        gyr_z=WIS100Data.gyr_z;
        acc_x=WIS100Data.acc_x;
        acc_y=WIS100Data.acc_y;
        acc_z=WIS100Data.acc_z;
        //解算姿态
        IMU(&WIS100Data,&Imu1);
        if(CntOmitData>=5000 && CntOmitData!=NoValidSPC)//5000ms
        {
          CntOmitData=NoValidSPC;
          if(InitPitchAngle==INIFITE && InitRollAngle==INIFITE)
          {
            InitPitchAngle=Imu1.pitch;
            InitRollAngle=Imu1.roll;
          }
        }
        else if(CntOmitData!=NoValidSPC) CntOmitData++;
        // printf("%.3f,%.3f,%.3f\n",Imu1.pitch,Imu1.roll,Imu1.yaw);
        //printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",gyr_x,gyr_y,gyr_z,acc_x,acc_y,acc_z);
        
    }
    else if(htim->Instance==htim3.Instance)//2000hz-500us
    {
      //SD卡写入任务
      static uint16_t Cnt10ms=0;
      static uint8_t Cnt2ms=0;
      Cnt10ms++;
      Cnt2ms++;

      //10msSD卡写任务
      if(Cnt10ms>=(10000/TIM3_500US))
      {
        Cnt10ms=0;
        if(TotalWriteCnt<10000)
        {
          SDCardWriteImuTask();
        }
        else
        {
          SDCardCloseImuLog();
        }
      }
      //入队检查任务
      if(Cnt2ms>=(2000/TIM3_500US))
      {
        Cnt2ms=0;
        PushImuSque(&Imu1);//压入队列
        StartCheckTask(&WIS100Data);//启动检查任务
        if(ImuSque.circular==1 && InitPitchAngle!=INIFITE && InitRollAngle!=INIFITE)  FireCheckTask();//检查点火任务
      }
    }
}

/*串口回调*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  static uint8_t WIE100Pre=0;
  if(huart==&huart1)
  {
    if(WIE100Pre>=34)
    {
        WIE100Pre=0;//防止数组越界
        WIS100_Data_Recv(WIE100PreRecvBuff,34,&ReceiveFlag);
        //HAL_UART_Transmit_DMA(&huart1,&WIE100PreRecvBuff,34*32);//回显
    }
    WIE100PreRecvBuff[WIE100Pre++]=TempReceiveData;
    
    //HAL_UART_Receive_DMA(&huart1,&TempReceiveData,1);//重新开启接收中断
  }
}
