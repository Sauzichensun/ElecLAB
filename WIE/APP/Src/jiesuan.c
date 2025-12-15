#include "WIE.h"
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "stm32h7xx_hal_usart.h"
#include "stdio.h"
#include "math.h"
#include "jiesuan.h"


//四元数
double q[4] = {1,0,0,0};
#define IMUProid 1//ms
IMuuSque ImuSque={0};
void PushImuSque(Imu_Typedef *p_imu)
{
  ImuSque.ele[ImuSque.squePointer].pitch=p_imu->pitch;
  ImuSque.ele[ImuSque.squePointer].roll=p_imu->roll;
  if((ImuSque.squePointer++)>=3)
  {
    ImuSque.squePointer=0;
    ImuSque.circular=1;
  }
}

void PopImuSque(ImuSqueElement *p_imu)
{
  uint8_t index = ImuSque.squePointer;
  if(index==0)
  {
    index=2;
    ImuSque.ele[index].pitch=ImuSque.ele[index].pitch;
    ImuSque.ele[index].roll=ImuSque.ele[index].roll;
    ImuSque.squePointer=index;
  }
  else 
  {
    index--;
    p_imu->pitch=ImuSque.ele[index].pitch;
    p_imu->roll=ImuSque.ele[index].roll;
    ImuSque.squePointer=index;
  }
}
void IMU(WIS100Data_TypeDef *p_data,Imu_Typedef *p_imu)
{
  double imu_gyr_x=p_data->gyr_x;
  double imu_gyr_y=p_data->gyr_y;
  double imu_gyr_z=p_data->gyr_z;
  double imu_acc_x=p_data->acc_x;
  double imu_acc_y=p_data->acc_y;
  double imu_acc_z=p_data->acc_z;

  double kp,ki,kd,dt;
  kp = p_imu->kp;
  ki = p_imu->ki; 
  kd = p_imu->kd;
  dt = p_imu->dt;

  //归一化加速度计
  double norm_acc = sqrt(imu_acc_x*imu_acc_x+imu_acc_y*imu_acc_y+imu_acc_z*imu_acc_z);
  if (norm_acc==0) return;
  imu_acc_x=imu_acc_x/norm_acc;
  imu_acc_y=imu_acc_y/norm_acc; 
  imu_acc_z=imu_acc_z/norm_acc;

  //计算预测重力方向
  double ex=0,ey=0,ez=0;
  static double integral_ex=0,integral_ey=0,integral_ez=0;
  double vx = 2*(q[1]*q[3]-q[0]*q[2]);
  double vy = 2*(q[0]*q[1]+q[2]*q[3]);
  double vz = q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3];
  
  ex = imu_acc_y*vz - imu_acc_z*vy;
  ey = imu_acc_z*vx - imu_acc_x*vz;
  ez = imu_acc_x*vy - imu_acc_y*vx;

  //误差积分
  integral_ex += ex*dt;
  integral_ey += ey*dt;
  integral_ez += ez*dt;

  //补偿陀螺仪数据误差
  imu_gyr_x = imu_gyr_x + kp*ex + ki*integral_ex + kd*(ez-vz);
  imu_gyr_y = imu_gyr_y + kp*ey + ki*integral_ey + kd*(vx-ex);  
  imu_gyr_z = imu_gyr_z + kp*ez + ki*integral_ez + kd*(vy-ey);
  
  
  //积分四元数
  double q0,q1,q2,q3;
  q0 = q[0];
  q1 = q[1];
  q2 = q[2];
  q3 = q[3];

  q[0] = q[0] + (-q1*imu_gyr_x - q2*imu_gyr_y - q3*imu_gyr_z)*dt*0.5;
  q[1] = q[1] + ( q0*imu_gyr_x + q2*imu_gyr_z - q3*imu_gyr_y)*dt*0.5;
  q[2] = q[2] + ( q0*imu_gyr_y - q1*imu_gyr_z + q3*imu_gyr_x)*dt*0.5;
  q[3] = q[3] + ( q0*imu_gyr_z + q1*imu_gyr_y - q2*imu_gyr_x)*dt*0.5;

  //四元数归一化
  double norm_q = sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
  if (norm_q==0) return;
  q[0]=q[0]/norm_q;
  q[1]=q[1]/norm_q; 
  q[2]=q[2]/norm_q;
  q[3]=q[3]/norm_q;
  //计算欧拉角
  double temp_pitch = asin(-2*q[1]*q[3]+2*q[0]*q[2])*57.3;
  double temp_roll= atan2(2*q[2]*q[3]+2*q[0]*q[1],-2*q[1]*q[1]-2*q[2]*q[2]+1)*57.3+180.0;
  double temp_yaw = atan2(2*q[1]*q[2]+2*q[0]*q[3],-2*q[2]*q[2]-2*q[3]*q[3]+1)*57.3+180.0;

  p_imu->pitch = temp_pitch;
  if(temp_yaw>=180) p_imu->yaw = temp_yaw-360;
  else p_imu->yaw = temp_yaw;
  if(temp_roll>=180) p_imu->roll = temp_roll-360;
  else p_imu->roll = temp_roll;

}


