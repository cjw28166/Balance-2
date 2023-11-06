#ifndef __CalculateThread_H
#define __CalculateThread_H
#include "struct_typedef.h"
#include "feet_motor.h"

typedef enum
{
	NOFORCE,
	STOP,
	FALLOW,
	ROTING,
	HIGHSPEED
}ChassisMode_e;


typedef struct
{
	const motor_measure_t *FeetMotor[2];
	const motor_measure_t *SliderMotor[2];
	const motor_measure_t *YawMotor;
	ChassisMode_e Mode;
	
	float vx;
	float vy;
	float wz;
	pid_type_def XYPid[4];
	pid_type_def WZPid;
	float Current[4];
	float WheelSpeed[4];
}Chassis_t;

extern Chassis_t Chassis;
extern int left_flag,right_flag;

void CalculateThread(void const * pvParameters);





#endif



