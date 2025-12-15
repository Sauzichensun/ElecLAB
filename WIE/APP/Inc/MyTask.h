#ifndef MYTASK_H
#define MYTASK_H

#include "main.h"
#include "WIE.h"
#include "jiesuan.h"

extern WIS100Data_TypeDef WIS100Data;
extern uint8_t TempReceiveData;
extern uint8_t WIE100PreRecvBuff[];
extern uint8_t ReceiveFlag;
extern uint32_t WIS100CheckSum_ErrCnt;
extern uint32_t WIS100BuffFull_ErrCnt;
extern uint32_t Read_ErrCnt;
extern uint32_t Write_ErrCnt;
extern uint32_t WIS100CurrentRecvLen;
extern uint32_t WIS100TotalRecvCnt;
extern uint32_t TotalWriteCnt;
extern double InitPitchAngle;//³õÊ¼¸©Ñö½Ç
extern double InitRollAngle;//³õÊ¼ºá¹ö½Ç
extern IMuuSque ImuSque;

#endif
