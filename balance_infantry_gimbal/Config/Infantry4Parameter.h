#ifndef __INFANTRY4_PARAMETER_H
#define __INFANTRY4_PARAMETER_H

#include "struct_typedef.h"

#define GM6020_MAX_OUTPUT                       30000
#define GM6020_MAX_IOUTPUT                      10000
#define M3508_MAX_OUTPUT                        16384
#define M3508_MAX_IOUTPUT                       6000
#define M2006_MAX_OUTPUT                        10000
#define M2006_MAX_IOUTPUT                       5000



#define PITCH_MAX_SPEED                         300
#define PITCH_MAX_ISPEED                        30
#define YAW_MAX_SPEED                           600 //300
#define YAW_MAX_ISPEED                          30
//射速 15M 18m 30m
#define AMMO_SPEEDSET_10MS                      2000
#define AMMO_SPEEDSET_12MS                      5000
#define AMMO_SPEEDSET_14MS                      5000
#define AMMO_SPEEDSET_15MS                      6400//4170//4215
#define AMMO_SPEEDSET_16MS                      5000
#define AMMO_SPEEDSET_18MS                      4550
#define AMMO_SPEEDSET_22MS                      5000
#define AMMO_SPEEDSET_30MS                      6400
//摩擦轮参数
#define ROTOR_SPEEDSET_FORWARD                  6000.0f//5700.0f //7200
#define ROTOR_SPEEDSET_BACKWARD                 2000.0f
//5000
//#define HIGH_FREQ_COOLING_COEFFICIENT						1550//440
//#define LOW_FREQ_COOLING_COEFFICIENT						1500//440
#define HIGH_FREQ_COOLING_COEFFICIENT						400//440
#define LOW_FREQ_COOLING_COEFFICIENT						400//440
#define ROTOR_TIMESET_BUSY                      68//68//拨盘的旋转时间
#define ROTOR_TIMESET_COOLING                   2//2//第一档射频
#define ROTOR_TIMESET_COOLING_SECOND            120//80//第二档
#define ROTOR_TIMESET_RESERVE                   35 // 35//堵转以后倒转的时间
#define ROTOR_LAGGING_COUNTER_MAX               42 //42堵转计时
// cooling150 busy 65 COUNTER_MAX 70  160
#define ROTOR_TIMESET_COOLING_MAXEXTRA          70
//200

#define duoji_init 2245
#define duoji_final 500


#define DELTA_HEAT_MAX													30




//  无力云台参数
//  YAW轴角速度环
#define YAW_SPEED_NO_FORCE_KP                   0.0f
#define YAW_SPEED_NO_FORCE_KI                   0.0f
#define YAW_SPEED_NO_FORCE_KD                   0.0f
fp32 YAW_SPEED_NO_FORCE[3] = {YAW_SPEED_NO_FORCE_KP, YAW_SPEED_NO_FORCE_KI, YAW_SPEED_NO_FORCE_KD};
//  YAW轴角度环
#define YAW_ANGLE_NO_FORCE_KP                   0.0f
#define YAW_ANGLE_NO_FORCE_KI                   0.0f
#define YAW_ANGLE_NO_FORCE_KD                   0.0f
fp32 YAW_ANGLE_NO_FORCE[3] = {YAW_ANGLE_NO_FORCE_KP, YAW_ANGLE_NO_FORCE_KI, YAW_ANGLE_NO_FORCE_KD};
//  PITCH轴角速度环
#define PITCH_SPEED_NO_FORCE_KP                 0.0f
#define PITCH_SPEED_NO_FORCE_KI                 0.0f
#define PITCH_SPEED_NO_FORCE_KD                 0.0f
fp32 PITCH_SPEED_NO_FORCE[3] = {PITCH_SPEED_NO_FORCE_KP, PITCH_SPEED_NO_FORCE_KI, PITCH_SPEED_NO_FORCE_KD};
//  PITCH轴角度环
#define PITCH_ANGLE_NO_FORCE_KP                 0.0f
#define PITCH_ANGLE_NO_FORCE_KI                 0.0f
#define PITCH_ANGLE_NO_FORCE_KD                 0.0f
fp32 PITCH_ANGLE_NO_FORCE[3] = {PITCH_ANGLE_NO_FORCE_KP, PITCH_ANGLE_NO_FORCE_KI, PITCH_ANGLE_NO_FORCE_KD};


//  归位云台参数
//  YAW轴角速度环
#define YAW_SPEED_RESET_POSITION_KP            	40.0f//36.0f
#define YAW_SPEED_RESET_POSITION_KI             0.0f
#define YAW_SPEED_RESET_POSITION_KD             0.0f
fp32 YAW_SPEED_RESET_POSITION[3] = {YAW_SPEED_RESET_POSITION_KP, YAW_SPEED_RESET_POSITION_KI, YAW_SPEED_RESET_POSITION_KD};
//  YAW轴角度环
#define YAW_ANGLE_RESET_POSITION_KP             18.0f
#define YAW_ANGLE_RESET_POSITION_KI             0.0f
#define YAW_ANGLE_RESET_POSITION_KD             0.0f
fp32 YAW_ANGLE_RESET_POSITION[3] = {YAW_ANGLE_RESET_POSITION_KP, YAW_ANGLE_RESET_POSITION_KI, YAW_ANGLE_RESET_POSITION_KD};
//  PITCH轴角速度环
#define PITCH_SPEED_RESET_POSITION_KP           160.0f
#define PITCH_SPEED_RESET_POSITION_KI           0.0f
#define PITCH_SPEED_RESET_POSITION_KD           0.0f
fp32 PITCH_SPEED_RESET_POSITION[3] = {PITCH_SPEED_RESET_POSITION_KP, PITCH_SPEED_RESET_POSITION_KI, PITCH_SPEED_RESET_POSITION_KD};
//  PITCH轴角度环
#define PITCH_ANGLE_RESET_POSITION_KP           60.0f
#define PITCH_ANGLE_RESET_POSITION_KI           0.0f
#define PITCH_ANGLE_RESET_POSITION_KD           0.0f
fp32 PITCH_ANGLE_RESET_POSITION[3] = {PITCH_ANGLE_RESET_POSITION_KP, PITCH_ANGLE_RESET_POSITION_KI, PITCH_ANGLE_RESET_POSITION_KD};


//  手动控制云台参数
//  YAW轴角速度环
#define YAW_SPEED_MANUAL_OPERATE_KP             360.0f
#define YAW_SPEED_MANUAL_OPERATE_KI             0.0f
#define YAW_SPEED_MANUAL_OPERATE_KD             0.0f
fp32 YAW_SPEED_MANUAL_OPERATE[3] = {YAW_SPEED_MANUAL_OPERATE_KP, YAW_SPEED_MANUAL_OPERATE_KI, YAW_SPEED_MANUAL_OPERATE_KD};
//  YAW轴角度环
#define YAW_ANGLE_MANUAL_OPERATE_KP             36.0f //30.0f
#define YAW_ANGLE_MANUAL_OPERATE_KI             0.0f
#define YAW_ANGLE_MANUAL_OPERATE_KD             0.0f
fp32 YAW_ANGLE_MANUAL_OPERATE[3] = {YAW_ANGLE_MANUAL_OPERATE_KP, YAW_ANGLE_MANUAL_OPERATE_KI, YAW_ANGLE_MANUAL_OPERATE_KD};
//  PITCH轴角速度环
#define PITCH_SPEED_MANUAL_OPERATE_KP           160.0f
#define PITCH_SPEED_MANUAL_OPERATE_KI           0.0f
#define PITCH_SPEED_MANUAL_OPERATE_KD           0.0f
fp32 PITCH_SPEED_MANUAL_OPERATE[3] = {PITCH_SPEED_MANUAL_OPERATE_KP, PITCH_SPEED_MANUAL_OPERATE_KI, PITCH_SPEED_MANUAL_OPERATE_KD};
//  PITCH轴角度环
#define PITCH_ANGLE_MANUAL_OPERATE_KP           57.0f //67
#define PITCH_ANGLE_MANUAL_OPERATE_KI           0.00999f
#define PITCH_ANGLE_MANUAL_OPERATE_KD           0.0f
fp32 PITCH_ANGLE_MANUAL_OPERATE[3] = {PITCH_ANGLE_MANUAL_OPERATE_KP, PITCH_ANGLE_MANUAL_OPERATE_KI, PITCH_ANGLE_MANUAL_OPERATE_KD};



//  自瞄控制云台参数
//  YAW轴角速度环
#define YAW_SPEED_AIMBOT_OPERATE_KP             180.0f
#define YAW_SPEED_AIMBOT_OPERATE_KI             0.0f
#define YAW_SPEED_AIMBOT_OPERATE_KD             0.0f
fp32 YAW_SPEED_AIMBOT_OPERATE[3] = {YAW_SPEED_AIMBOT_OPERATE_KP, YAW_SPEED_AIMBOT_OPERATE_KI, YAW_SPEED_AIMBOT_OPERATE_KD};
//  YAW轴角度环
#define YAW_ANGLE_AIMBOT_OPERATE_KP             36.0f
#define YAW_ANGLE_AIMBOT_OPERATE_KI             0.0f
#define YAW_ANGLE_AIMBOT_OPERATE_KD             0.0f
fp32 YAW_ANGLE_AIMBOT_OPERATE[3] = {YAW_ANGLE_AIMBOT_OPERATE_KP, YAW_ANGLE_AIMBOT_OPERATE_KI, YAW_ANGLE_AIMBOT_OPERATE_KD};
//  PITCH轴角速度环
#define PITCH_SPEED_AIMBOT_OPERATE_KP           160.0f
#define PITCH_SPEED_AIMBOT_OPERATE_KI           0.0f
#define PITCH_SPEED_AIMBOT_OPERATE_KD           0.0f
fp32 PITCH_SPEED_AIMBOT_OPERATE[3] = {PITCH_SPEED_AIMBOT_OPERATE_KP, PITCH_SPEED_AIMBOT_OPERATE_KI, PITCH_SPEED_AIMBOT_OPERATE_KD};
//  PITCH轴角度环
#define PITCH_ANGLE_AIMBOT_OPERATE_KP           30.0f
#define PITCH_ANGLE_AIMBOT_OPERATE_KI           0.00999f
#define PITCH_ANGLE_AIMBOT_OPERATE_KD           0.0f
fp32 PITCH_ANGLE_AIMBOT_OPERATE[3] = {PITCH_ANGLE_AIMBOT_OPERATE_KP, PITCH_ANGLE_AIMBOT_OPERATE_KI, PITCH_ANGLE_AIMBOT_OPERATE_KD};


//  打符云台参数
//  YAW轴角速度环
#define YAW_SPEED_AIMBOT_RUNES_KP               180.0f
#define YAW_SPEED_AIMBOT_RUNES_KI               0.0f
#define YAW_SPEED_AIMBOT_RUNES_KD               0.0f
fp32 YAW_SPEED_AIMBOT_RUNES[3] = {YAW_SPEED_AIMBOT_RUNES_KP, YAW_SPEED_AIMBOT_RUNES_KI, YAW_SPEED_AIMBOT_RUNES_KD};
//  YAW轴角度环
#define YAW_ANGLE_AIMBOT_RUNES_KP               36.0f
#define YAW_ANGLE_AIMBOT_RUNES_KI               0.0f
#define YAW_ANGLE_AIMBOT_RUNES_KD               0.0f
fp32 YAW_ANGLE_AIMBOT_RUNES[3] = {YAW_ANGLE_AIMBOT_RUNES_KP, YAW_ANGLE_AIMBOT_RUNES_KI, YAW_ANGLE_AIMBOT_RUNES_KD};
//  PITCH轴角速度环
#define PITCH_SPEED_AIMBOT_RUNES_KP             160.0f
#define PITCH_SPEED_AIMBOT_RUNES_KI             0.0f
#define PITCH_SPEED_AIMBOT_RUNES_KD             0.0f
fp32 PITCH_SPEED_AIMBOT_RUNES[3] = {PITCH_SPEED_AIMBOT_RUNES_KP, PITCH_SPEED_AIMBOT_RUNES_KI, PITCH_SPEED_AIMBOT_RUNES_KD};
//  PITCH轴角度环
#define PITCH_ANGLE_AIMBOT_RUNES_KP             30.0f
#define PITCH_ANGLE_AIMBOT_RUNES_KI             0.00999f
#define PITCH_ANGLE_AIMBOT_RUNES_KD             0.0f
fp32 PITCH_ANGLE_AIMBOT_RUNES[3] = {PITCH_ANGLE_AIMBOT_RUNES_KP, PITCH_ANGLE_AIMBOT_RUNES_KI, PITCH_ANGLE_AIMBOT_RUNES_KD};


//  摩擦轮参数

#define AMMO_LEFT_SPEED_10MS_KP                 13.0f
#define AMMO_LEFT_SPEED_10MS_KI                 0.0f
#define AMMO_LEFT_SPEED_10MS_KD                 0.0f
fp32 AMMO_LEFT_SPEED_10MS[3] = {AMMO_LEFT_SPEED_10MS_KP, AMMO_LEFT_SPEED_10MS_KI, AMMO_LEFT_SPEED_10MS_KD};

#define AMMO_RIGHT_SPEED_10MS_KP                13.0f
#define AMMO_RIGHT_SPEED_10MS_KI                0.0f
#define AMMO_RIGHT_SPEED_10MS_KD                0.0f
fp32 AMMO_RIGHT_SPEED_10MS[3] = {AMMO_RIGHT_SPEED_10MS_KP, AMMO_RIGHT_SPEED_10MS_KI, AMMO_RIGHT_SPEED_10MS_KD};

#define AMMO_LEFT_SPEED_12MS_KP                 20.0f
#define AMMO_LEFT_SPEED_12MS_KI                 0.0f
#define AMMO_LEFT_SPEED_12MS_KD                 0.0f
fp32 AMMO_LEFT_SPEED_12MS[3] = {AMMO_LEFT_SPEED_12MS_KP, AMMO_LEFT_SPEED_12MS_KI, AMMO_LEFT_SPEED_12MS_KD};

#define AMMO_RIGHT_SPEED_12MS_KP                20.0f
#define AMMO_RIGHT_SPEED_12MS_KI                0.0f
#define AMMO_RIGHT_SPEED_12MS_KD                0.0f
fp32 AMMO_RIGHT_SPEED_12MS[3] = {AMMO_RIGHT_SPEED_12MS_KP, AMMO_RIGHT_SPEED_12MS_KI, AMMO_RIGHT_SPEED_12MS_KD};

#define AMMO_LEFT_SPEED_14MS_KP                 20.0f
#define AMMO_LEFT_SPEED_14MS_KI                 0.0f
#define AMMO_LEFT_SPEED_14MS_KD                 0.0f
fp32 AMMO_LEFT_SPEED_14MS[3] = {AMMO_LEFT_SPEED_14MS_KP, AMMO_LEFT_SPEED_14MS_KI, AMMO_LEFT_SPEED_14MS_KD};

#define AMMO_RIGHT_SPEED_14MS_KP                20.0f
#define AMMO_RIGHT_SPEED_14MS_KI                0.0f
#define AMMO_RIGHT_SPEED_14MS_KD                0.0f
fp32 AMMO_RIGHT_SPEED_14MS[3] = {AMMO_RIGHT_SPEED_14MS_KP, AMMO_RIGHT_SPEED_14MS_KI, AMMO_RIGHT_SPEED_14MS_KD};

#define AMMO_LEFT_SPEED_15MS_KP                 20.0f
#define AMMO_LEFT_SPEED_15MS_KI                 0.0f
#define AMMO_LEFT_SPEED_15MS_KD                 0.0f
fp32 AMMO_LEFT_SPEED_15MS[3] = {AMMO_LEFT_SPEED_15MS_KP, AMMO_LEFT_SPEED_15MS_KI, AMMO_LEFT_SPEED_15MS_KD};

#define AMMO_RIGHT_SPEED_15MS_KP                8.0f//13.0f
#define AMMO_RIGHT_SPEED_15MS_KI                0.0f
#define AMMO_RIGHT_SPEED_15MS_KD                0.0f
fp32 AMMO_RIGHT_SPEED_15MS[3] = {AMMO_RIGHT_SPEED_15MS_KP, AMMO_RIGHT_SPEED_15MS_KI, AMMO_RIGHT_SPEED_15MS_KD};

#define AMMO_LEFT_SPEED_16MS_KP                 20.0f//13.0f
#define AMMO_LEFT_SPEED_16MS_KI                 0.0f
#define AMMO_LEFT_SPEED_16MS_KD                 0.0f
fp32 AMMO_LEFT_SPEED_16MS[3] = {AMMO_LEFT_SPEED_16MS_KP, AMMO_LEFT_SPEED_16MS_KI, AMMO_LEFT_SPEED_16MS_KD};

#define AMMO_RIGHT_SPEED_16MS_KP                20.0f
#define AMMO_RIGHT_SPEED_16MS_KI                0.0f
#define AMMO_RIGHT_SPEED_16MS_KD                0.0f
fp32 AMMO_RIGHT_SPEED_16MS[3] = {AMMO_RIGHT_SPEED_16MS_KP, AMMO_RIGHT_SPEED_16MS_KI, AMMO_RIGHT_SPEED_16MS_KD};

#define AMMO_LEFT_SPEED_18MS_KP                 20.0f
#define AMMO_LEFT_SPEED_18MS_KI                 0.0f
#define AMMO_LEFT_SPEED_18MS_KD                 0.0f
fp32 AMMO_LEFT_SPEED_18MS[3] = {AMMO_LEFT_SPEED_18MS_KP, AMMO_LEFT_SPEED_18MS_KI, AMMO_LEFT_SPEED_18MS_KD};

#define AMMO_RIGHT_SPEED_18MS_KP                20.0f
#define AMMO_RIGHT_SPEED_18MS_KI                0.0f
#define AMMO_RIGHT_SPEED_18MS_KD                0.0f
fp32 AMMO_RIGHT_SPEED_18MS[3] = {AMMO_RIGHT_SPEED_18MS_KP, AMMO_RIGHT_SPEED_18MS_KI, AMMO_RIGHT_SPEED_18MS_KD};

#define AMMO_LEFT_SPEED_22MS_KP                 20.0f
#define AMMO_LEFT_SPEED_22MS_KI                 0.0f
#define AMMO_LEFT_SPEED_22MS_KD                 0.0f
fp32 AMMO_LEFT_SPEED_22MS[3] = {AMMO_LEFT_SPEED_22MS_KP, AMMO_LEFT_SPEED_22MS_KI, AMMO_LEFT_SPEED_22MS_KD};

#define AMMO_RIGHT_SPEED_22MS_KP                20.0f
#define AMMO_RIGHT_SPEED_22MS_KI                0.0f
#define AMMO_RIGHT_SPEED_22MS_KD                0.0f
fp32 AMMO_RIGHT_SPEED_22MS[3] = {AMMO_RIGHT_SPEED_22MS_KP, AMMO_RIGHT_SPEED_22MS_KI, AMMO_RIGHT_SPEED_22MS_KD};

#define AMMO_LEFT_SPEED_30MS_KP                 20.0f
#define AMMO_LEFT_SPEED_30MS_KI                 0.0f
#define AMMO_LEFT_SPEED_30MS_KD                 0.0f
fp32 AMMO_LEFT_SPEED_30MS[3] = {AMMO_LEFT_SPEED_30MS_KP, AMMO_LEFT_SPEED_30MS_KI, AMMO_LEFT_SPEED_30MS_KD};

#define AMMO_RIGHT_SPEED_30MS_KP                20.0f
#define AMMO_RIGHT_SPEED_30MS_KI                0.0f
#define AMMO_RIGHT_SPEED_30MS_KD                0.0f
fp32 AMMO_RIGHT_SPEED_30MS[3] = {AMMO_RIGHT_SPEED_30MS_KP, AMMO_RIGHT_SPEED_30MS_KI, AMMO_RIGHT_SPEED_30MS_KD};



#define ROTOR_UNABLE_KP                         0.0f
#define ROTOR_UNABLE_KI                         0.0f
#define ROTOR_UNABLE_KD                         0.0f
fp32 ROTOR_UNABLE[3] = {ROTOR_UNABLE_KP, ROTOR_UNABLE_KI, ROTOR_UNABLE_KD};
//120
#define ROTOR_FORWARD_KP                        200.0f//100.0f//230.0f
#define ROTOR_FORWARD_KI                        0.0f
#define ROTOR_FORWARD_KD                        0.0f
fp32 ROTOR_FORWARD[3] = {ROTOR_FORWARD_KP, ROTOR_FORWARD_KI, ROTOR_FORWARD_KD};

#define ROTOR_STOP_KP                           30.0f
#define ROTOR_STOP_KI                           0.0f
#define ROTOR_STOP_KD                           0.0f
fp32 ROTOR_STOP[3] = {ROTOR_STOP_KP, ROTOR_STOP_KI, ROTOR_STOP_KD};

#define ROTOR_BACK_KP                           100.0f//60.0f
#define ROTOR_BACK_KI                           0.0f
#define ROTOR_BACK_KD                           0.0f
fp32 ROTOR_BACK[3] = {ROTOR_BACK_KP, ROTOR_BACK_KI, ROTOR_BACK_KD};

// speed 75 150 200 
// cooling 50 100 250
// fire 150 280 400

#endif
