#ifndef SENDRECV_H
#define SENDRECV_H
#include "unitreeMotor.h"
#include "transform.h"
#include "crc32.h"  

typedef __PACKED_STRUCT
{
    float Pos,T,W;
}Can_Recv;

extern uint8_t SendData[34],RecvData[78];

extern void DataSend(MOTOR_send *motor);
extern bool DataRecv(MOTOR_recv *motor);
extern void modify_data(MOTOR_send *motor_s);
extern bool extract_data(MOTOR_recv* motor_r);
#endif

