#ifndef OTHERTASK_H
#define OTHERTASK_H 

#include "jiesuan.h"

extern volatile uint8_t SDFlag;
extern Imu_Typedef Imu1;
extern WIS100Data_TypeDef WIS100Data;

void SDCardTaskInit(void);
//void SDCardWriteImuTask(void);
void SDCardWriteImuTask(void);
void SDCardCloseImuLog(void);
void FireCheckTask(void);
void StartCheckTask(WIS100Data_TypeDef *data);
#endif
