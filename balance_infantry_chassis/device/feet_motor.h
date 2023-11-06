
#ifndef __MOTOR_H
#define __MOTOR_H

#include "struct_typedef.h"
#include "main.h"

//rm motor data

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    uint16_t last_ecd;
	
    fp32 angle;
	fp32 speed;
} motor_measure_t;

#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
        (ptr)->angle = (ptr)->ecd / 8191.0f *360 - 180;          		\
		(ptr)->speed = (ptr)->speed_rpm * 360.0f;						\
    }


extern void MotorProcess(uint32_t MotorID, CAN_HandleTypeDef *hcan, uint8_t* message);
extern void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void Chassis_Control(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

extern motor_measure_t LeftFootMotorMeasure;
extern motor_measure_t RightFootMotorMeasure;
extern motor_measure_t LeftSliderMotorMeasure;
extern motor_measure_t RightSliderMotorMeasure;
extern motor_measure_t YawMotorMeasure;
	
#endif
