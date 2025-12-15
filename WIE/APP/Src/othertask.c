#include "main.h"
#include "tim.h"
#include "usart.h"
#include "stdio.h"
#include "othertask.h"
#include "fatfs.h"
#include "jiesuan.h"
#include "string.h"
#include "WIE.h"

void SDCardTaskInit(void)
{
  if(SDFlag==1) printf("SD Card Init Success!\n");
  else printf("SD Card Init Failed!\n");
  FRESULT res;
  res = f_mount(&SDFatFS, (TCHAR const*)SDPath, 0);
  if (res != FR_OK) printf("SD card mount failed\n");
  else  printf("SD card mount success\n");

  //create file
  printf("Creating file Attitude.txt\n");
  res = f_open(&SDFile, "Attitude.txt",FA_WRITE | FA_CREATE_ALWAYS);
  if (res != FR_OK) printf("Error creating file\n");
  else printf("File created successfully\n");
  f_close(&SDFile);

}

uint32_t TotalWriteCnt=0;
uint32_t WriteErrorCnt=0;


#define IMU_BUFFER_SIZE 30  // 缓存10条数据
#define SYNC_INTERVAL 1000   // 每1000ms强制同步一次

typedef struct {
    double gyr_x;               /* x轴陀螺  deg/s */
    double gyr_y;               /* y轴陀螺 */
    double gyr_z;               /* z轴陀螺 */
 
    double acc_x;               /* x轴加表  g */
    double acc_y;               /* y轴加表 */
    double acc_z;               /* z轴加表 */
    uint32_t timestamp; // 可选：添加时间戳
} WIS100DataBufPoint;

static WIS100DataBufPoint imu_buffer[IMU_BUFFER_SIZE];
static uint16_t buffer_index = 0;
static uint8_t file_opened = 0;
static uint32_t last_sync_time = 0;
volatile uint8_t IsWriting=0;
void SDCardWriteImuTask(void)
{
    static uint8_t is_first = 0;
    
    // 第一次调用：创建文件并写入表头
    if (is_first == 0)
    {
        is_first = 1;
        FRESULT res;
        
        res = f_open(&SDFile, "Attitude.txt", FA_WRITE | FA_CREATE_ALWAYS);
        if (res != FR_OK) {
            printf("Error opening file: %d\n", res);
            IsWriting=0;
            return;
        }
        
        // 写入表头
        const char *header = "Timestamp(ms),gyrox,gyroy,gyroz,accx,accy,accz\n";
        UINT bw;
        res = f_write(&SDFile, header, strlen(header), &bw);
        if (res != FR_OK) {
            printf("Error writing header: %d\n", res);
            f_close(&SDFile);
            IsWriting=0;
            return;
        }
        
        file_opened = 1; // 保持文件打开状态
        printf("IMU logging started\n");
        IsWriting=0;
        return;
    }
    
    // 添加数据到缓存
    imu_buffer[buffer_index].gyr_x = WIS100Data.gyr_x;
    imu_buffer[buffer_index].gyr_y = WIS100Data.gyr_y;
    imu_buffer[buffer_index].gyr_z = WIS100Data.gyr_z;
    imu_buffer[buffer_index].acc_x = WIS100Data.acc_x;
    imu_buffer[buffer_index].acc_y = WIS100Data.acc_y;
    imu_buffer[buffer_index].acc_z = WIS100Data.acc_z;
    imu_buffer[buffer_index].timestamp = HAL_GetTick(); // 或使用RTC时间
    buffer_index++;
    
    // 缓存满或超时，批量写入
    uint32_t current_time = HAL_GetTick();
    if ((buffer_index >= IMU_BUFFER_SIZE || 
        (current_time - last_sync_time) >= SYNC_INTERVAL) && IsWriting==0)
    {
        if (!file_opened) 
        {
            // 重新打开文件（异常恢复）
            FRESULT res = f_open(&SDFile, "Attitude.txt", FA_WRITE | FA_OPEN_APPEND);
            if (res != FR_OK)
            {
                printf("Reopen failed: %d\n", res);
                buffer_index = 0; // 丢弃缓存数据
                IsWriting=0;
                return;
            }
            file_opened = 1;
        }
        
        // 批量写入
        char write_buf[2048]; // 大缓存区
        int offset = 0;
        IsWriting=1;
        for (uint16_t i = 0; i < buffer_index; i++) 
        {
            offset += sprintf(write_buf + offset, "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                            imu_buffer[i].timestamp,
                            imu_buffer[i].gyr_x,
                            imu_buffer[i].gyr_y,
                            imu_buffer[i].gyr_z,
                            imu_buffer[i].acc_x,
                            imu_buffer[i].acc_y,
                            imu_buffer[i].acc_z);
        }
        
        UINT bytes_written;
        FRESULT res = f_write(&SDFile, write_buf, offset, &bytes_written);
        
        if (res != FR_OK || bytes_written != offset) 
        {
            WriteErrorCnt++;
            printf("Write error: res=%d, written=%d/%d\n", res, bytes_written, offset);
        } 
        else 
        {
            TotalWriteCnt += buffer_index;
        }
        
        // 同步到SD卡（确保数据落盘）
        f_sync(&SDFile);
        IsWriting = 0;
        // 重置缓存
        buffer_index = 0;
        last_sync_time = current_time;
    }
}

// 系统关闭或卸载SD卡前调用
void SDCardCloseImuLog(void)
{
    if (file_opened)
    {
        // 写入剩余数据
        if (buffer_index > 0) 
        {
            // 批量写入
            char write_buf[2048]; // 大缓存区
            int offset = 0;
            
            for (uint16_t i = 0; i < buffer_index; i++)
            {
                offset += sprintf(write_buf + offset, "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                        imu_buffer[i].timestamp,
                        imu_buffer[i].gyr_x,
                        imu_buffer[i].gyr_y,
                        imu_buffer[i].gyr_z,
                        imu_buffer[i].acc_x,
                        imu_buffer[i].acc_y,
                        imu_buffer[i].acc_z);
            }
        
            UINT bytes_written;
            FRESULT res = f_write(&SDFile, write_buf, offset, &bytes_written);
            
            if (res != FR_OK || bytes_written != offset)
            {
                WriteErrorCnt++;
                printf("Write error: res=%d, written=%d/%d\n", res, bytes_written, offset);
            } 
            else
            {
                TotalWriteCnt += buffer_index;
            }
        
            // 同步到SD卡（确保数据落盘）
            f_sync(&SDFile);
        }
        f_close(&SDFile);
        file_opened = 0;
        printf("IMU log closed, total: %lu records\n", TotalWriteCnt);
    }
}

volatile uint8_t IsStartFlag=0;//是否起飞标志位
volatile uint8_t IsFirfeFlag=0;//是否点火标志位

double InitPitchAngle=INIFITE;//初始俯仰角
double InitRollAngle=INIFITE;//初始横滚角

//GPS数据5ms
//2ms
void StartCheckTask(WIS100Data_TypeDef *data)
{
    //起飞检查--垂直于地面的加速度大于3g
    if(data->acc_z<-3) IsStartFlag=1;
}
void FireCheckTask()
{
    //点火检查
    uint8_t TempCheckPitch=0;
    uint8_t TempCheckRoll=0;

    ImuSqueElement CheckImu, CheckImu1, CheckImu2;
    PopImuSque(&CheckImu);
    PopImuSque(&CheckImu1);
    PopImuSque(&CheckImu2);
    if(fabs(CheckImu.pitch-InitPitchAngle)>60 &&
     fabs(CheckImu1.pitch-InitPitchAngle)>60 &&
      fabs(CheckImu2.pitch-InitPitchAngle)>60) TempCheckPitch=1;
    if(fabs(CheckImu.roll-InitRollAngle)>60 &&
     fabs(CheckImu1.roll-InitRollAngle)>60 &&
      fabs(CheckImu2.roll-InitRollAngle)>60) TempCheckRoll=1;
    if(TempCheckPitch==1 || TempCheckRoll==1) IsFirfeFlag=1;
}

