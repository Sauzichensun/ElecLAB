#ifndef JIESUAN_H
#define JIESUAN_H   

#include "WIE.h"
typedef struct
{
  double pitch;
  double roll;
  double yaw;
  double kp;
  double ki;
  double kd;
  double dt;
}Imu_Typedef; 

typedef struct _q
{
  double pitch;
  double roll;
}ImuSqueElement;

typedef struct _imu
{
  ImuSqueElement ele[3];
  uint8_t squePointer;
  uint8_t circular;
}IMuuSque;

#define INIFITE 0xFFFF
void IMU(WIS100Data_TypeDef *p_data,Imu_Typedef *p_imu);
void PushImuSque(Imu_Typedef *p_imu);
void PopImuSque(ImuSqueElement *p_imu);

#endif
