#ifndef SETTING_H
#define SETTING_H

#include "struct_typedef.h"


#define LENGTH 1000

#define MOTOR_OFFLINE_TIMEMAX   50
#define REMOTE_OFFLINE_TIMEMAX  550
#define AIMBOT_OFFLINE_TIMEMAX  550
#define REFEREE_OFFLINE_TIMEMAX 3000

#define PARAMETER_FILE "Setting.h"



// imu安装方向
#define IMU_DIRECTION_yrxz_XYZ
// imu yaw轴零飘偏置
#define IMU_GYRO_YAW_BIAS    							0.00165f
// 电机ID分配

#define FollowAngle              						-115.0f

#define CAN_FEET_ALL_ID									0x1FF

#define YawMotorId                                     	0x205

#define FEET_MOTOR1_TRANSMIT_ID         			    0x201
#define FEET_MOTOR2_TRANSMIT_ID         			    0x282

#define SLIDER_MOTOR1_RECEIVE_ID         	  			0x206
#define SLIDER_MOTOR2_RECEIVE_ID         	 			0x205

#define FEET_MOTOR1_RECEIVE_ID         	  				0x207
#define FEET_MOTOR2_RECEIVE_ID         	 			    0x208

#define DRAMA_ANGLE						1.8f

#define FOLLOW_KP						2.0f
#define FOLLOW_KI						0.0f
#define FOLLOW_KD						100.0f

#define POSITION_KP                   	900.0f	
#define POSITION_KI                		0.0f
#define POSITION_KD                		25000.0f

#define POSITION_FAST_KP				1800.0f
#define POSITION_FAST_KI				0.0f
#define POSITION_FAST_KD				25000.0f

#define SPEED_KP						3.0f
#define SPEED_KI						0.07f
#define SPEED_KD						0.0f

#define SLIDER_SPEED_KP					0.8f
#define SLIDER_SPEED_KI					0.0f
#define SLIDER_SPEED_KD					0.0f

#define DRAMA_KP						1.0f //8000
#define DRAMA_KI						0.0f
#define DRAMA_KD						0.0f//300.0f

#define TURN_KP							3000.0f //8000
#define TURN_KI							0.0f//0.0f
#define TURN_KD							0.0f//50000.0f
 
#define SLIDER_KP						1.75f
#define SLIDER_KI						0.0f//0.002f
#define SLIDER_KD						0.0f

#define POSITION_DRAMA_KP						0.014f
#define POSITION_DRAMA_KI						0.0f
#define POSITION_DRAMA_KD						0.00f

#define POSITION_SLIDER1_KP						0.4f
#define POSITION_SLIDER1_KI						0.0f
#define POSITION_SLIDER1_KD						0.0f

#define SPEED_SLIDER1_KP						40.0f
#define SPEED_SLIDER1_KI						0.0f
#define SPEED_SLIDER1_KD						0.0f

#define POSITION_SLIDER2_KP						0.4f
#define POSITION_SLIDER2_KI						0.0f
#define POSITION_SLIDER2_KD						0.0f

#define SPEED_SLIDER2_KP						40.0f
#define SPEED_SLIDER2_KI						0.0f
#define SPEED_SLIDER2_KD						0.0f

#define PITCH_KP						-0.345f
#define PITCH_KI						0.0f
#define PITCH_KD						0.001f


#define SPEED_FOLLOWING_KP				0.1f
#define SPEED_FOLLOWING_KI				0.00f
#define SPEED_FOLLOWING_KD				0.00f
// 通信can总线位置
#define COMMUNICATE_CANPORT         hcan1

#define ROTING_SPEED_60					1.5f
#define ROTING_SPEED_80					2.0f
#define ROTING_SPEED_100				3.0f




#endif

