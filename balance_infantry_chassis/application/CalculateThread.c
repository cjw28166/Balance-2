#include "CalculateThread.h"
#include "feet_motor.h"
#include "Remote.h"
#include "AttitudeThread.h"
#include "cmsis_os.h"
#include "pid.h"
#include "Setting.h"
#include "user_lib.h"
#include "arm_math.h"
#include "CanPacket.h"
#include "stdio.h"
#include "InterruptService.h"
#include "RefereeBehaviour.h"

//#define printf(...)  HAL_UART_Transmit_DMA(&huart6,\
//																				(uint8_t  *)u1_buf,\
//																				sprintf((char*)u1_buf,__VA_ARGS__))
//uint8_t u1_buf[32];

//1. 结构体类型定义
typedef struct 
{
    float LastP;//上次估算协方差 初始化值为0.02
    float Now_P;//当前估算协方差 初始化值为0
    float out;//卡尔曼滤波器输出 初始化值为0
    float Kg;//卡尔曼增益 初始化值为0
    float Q;//过程噪声协方差 初始化值为0.001
    float R;//观测噪声协方差 初始化值为0.543
}KFP;//Kalman Filter parameter

//2. 以高度为例 定义卡尔曼结构体并初始化参数
KFP KFP_pspeed={0.02,0,0,0,0.0001,0.543};
KFP KFP_slider={0.02,0,0,0,0.0001,0.543};
KFP KFP_drama={0.02,0,0,0,0.0001,0.543};
/**
 *卡尔曼滤波器
 *@param KFP *kfp 卡尔曼结构体参数
 *   float input 需要滤波的参数的测量值（即传感器的采集值）
 *@return 滤波后的参数（最优值）
 */
 float KalmanFilter(KFP *kfp,float input)
 {
     //预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
     kfp->Now_P = kfp->LastP + kfp->Q;
     //卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
     kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
     //更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
     kfp->out = kfp->out + kfp->Kg * (input -kfp->out);//因为这一次的预测值就是上一次的输出值
     //更新协方差方程: 本次的系统协方差付给 kfp->LastP 威下一次运算准备。
     kfp->LastP = (1-kfp->Kg) * kfp->Now_P;
     return kfp->out;
 }

																				
																				
Chassis_t Chassis;
RC_ctrl_t Remote;
EulerSystemMeasure_t    Imu;
Aim_t Aim;
PTZ_t PTZ;
ext_game_robot_status_t Referee;

pid_type_def follow;
pid_type_def slider_speed;
pid_type_def drama_pid_left;
pid_type_def drama_pid_right;
pid_type_def position_pid_left;
pid_type_def position_pid_right;
pid_type_def position_pid_left_fast;
pid_type_def position_pid_right_fast;
pid_type_def turn_pid;
pid_type_def slider_pid;
pid_type_def speed_pid_left;
pid_type_def speed_pid_right;
pid_type_def position_slider1_pid;
pid_type_def position_slider2_pid;
pid_type_def speed_slider1_pid;
pid_type_def speed_slider2_pid;
pid_type_def speed_pitch_pid;
pid_type_def speed_folowing_pid;
pid_type_def position_drama_pid;

first_order_filter_type_t joint_filter;
first_order_filter_type_t turn_filter;
first_order_filter_type_t pitch_filter;
first_order_filter_type_t position_filter;
fp32 joint_paramater[1];
fp32 turn_paramater[1];
fp32 pitch_paramater[1];
fp32 position_paramater[1];
fp32 left_foot_position;
fp32 right_foot_position;
fp32 left_foot_speed;
fp32 right_foot_speed;
fp32 left_slider_foot_current,right_slider_foot_current;
fp32 left_slider_foot_speed,right_slider_foot_speed;
fp32 slider_foot_position,slider_foot_position_left,slider_foot_position_right;
fp32 erro_angle;
fp32 pitch_speed;

fp32 wz_left;
fp32 wz_right;
fp32 wz_current;
fp32 wz_current_left;
fp32 wz_current_right;
fp32 pitch_last;
fp32 t=0;
fp32 speed_x,ps,po;
fp32 speed_folowing;
fp32 sp;

int mode=10;
extern int left_flag;
extern int right_flag;


fp32 x_left=0,x_right=0; 
fp32 angle_zero = -0.22;//-0.22 -0.32  0.05
fp32 roting_speed;
fp32 follow_angle;
fp32 follow_PID[3]={FOLLOW_KP,FOLLOW_KI,FOLLOW_KD};
fp32 slider_speed_PID[3]={SLIDER_SPEED_KP,SLIDER_SPEED_KI,SLIDER_SPEED_KD};
fp32 drama_PID[3]={DRAMA_KP,DRAMA_KI,DRAMA_KD};
fp32 position_PID[3]={POSITION_KP,POSITION_KI,POSITION_KD};
fp32 position_fast_PID[3]={POSITION_FAST_KP,POSITION_FAST_KI,POSITION_FAST_KD};
fp32 speed_PID[3]={SPEED_KP,SPEED_KI,SPEED_KD};
fp32 speed_pitch_PID[3]={PITCH_KP,PITCH_KI,PITCH_KD};
fp32 turn_PID[3]={TURN_KP,TURN_KI,TURN_KD};
fp32 slider_PID[3]={SLIDER_KP,SLIDER_KI,SLIDER_KD};
fp32 position_slider1_PID[3]={POSITION_SLIDER1_KP,POSITION_SLIDER1_KI,POSITION_SLIDER1_KD};
fp32 position_slider2_PID[3]={POSITION_SLIDER2_KP,POSITION_SLIDER2_KI,POSITION_SLIDER2_KD};
fp32 speed_slider1_PID[3]={SPEED_SLIDER1_KP,SPEED_SLIDER1_KI,SPEED_SLIDER1_KD};
fp32 speed_slider2_PID[3]={SPEED_SLIDER2_KP,SPEED_SLIDER2_KI,SPEED_SLIDER2_KD};
fp32 speed_folowing_PID[3]={SPEED_FOLLOWING_KP,SPEED_FOLLOWING_KI,SPEED_FOLLOWING_KD};
fp32 position_drama_PID[3]={POSITION_DRAMA_KP,POSITION_DRAMA_KI,POSITION_DRAMA_KD};

uint16_t init_time;
uint8_t Mode_last;
uint8_t Mode_now;

void ChassisInit();
void ChassisModeUpdate();
void ChassisPidUpadte();
void ChassisCommandUpdate();
void RefereeInfUpdate(ext_game_robot_status_t* referee);

void CalculateThread(void const * pvParameters)
{
	
	ChassisInit();
	
	
	while(1)
	{
			

        ChassisModeUpdate();
		RefereeInfUpdate(&Referee);
        GimbalEulerSystemMeasureUpdate(&Imu);	
		ChassisCommandUpdate();
		
		
		Chassis_Control(right_slider_foot_current,
						left_slider_foot_current,
						right_foot_position + wz_current_right +wz_current,
						left_foot_position + wz_current_left+wz_current);
					
	
	
		osDelay(1);
		
	}
	
	
}

void ChassisInit()
{
	PID_init(&slider_speed,PID_POSITION,slider_speed_PID,0.5,0.3);
	PID_init(&drama_pid_right,PID_POSITION,drama_PID,16384,16384);		
	PID_init(&drama_pid_left,PID_POSITION,drama_PID,16384,16384);
	PID_init(&position_pid_left,PID_POSITION,position_PID,16384,16384);
	PID_init(&position_pid_right,PID_POSITION,position_PID,16384,16384);
	PID_init(&position_pid_left_fast,PID_POSITION,position_fast_PID,16384,16384);
	PID_init(&position_pid_right_fast,PID_POSITION,position_fast_PID,16384,16384);
	PID_init(&speed_pid_left,PID_POSITION,speed_PID,7,5);
	PID_init(&speed_pid_right,PID_POSITION,speed_PID,7,5);
	PID_init(&turn_pid,PID_POSITION,turn_PID,16384,16384);
	PID_init(&slider_pid,PID_POSITION,slider_PID,0.5,0.3);
	PID_init(&position_slider1_pid,PID_POSITION,position_slider1_PID,720,100);
	PID_init(&position_slider2_pid,PID_POSITION,position_slider2_PID,720,100);
	PID_init(&speed_slider1_pid,PID_POSITION,speed_slider1_PID,30000,10000);
	PID_init(&speed_slider2_pid,PID_POSITION,speed_slider2_PID,30000,10000);
	PID_init(&speed_pitch_pid,PID_POSITION,speed_pitch_PID,2,1);
	PID_init(&speed_folowing_pid,PID_POSITION,speed_folowing_PID,0.5,0.2);
	PID_init(&follow,PID_POSITION,follow_PID,2,2);
	PID_init(&position_drama_pid,PID_POSITION,position_drama_PID,10000,10000);
	
	first_order_filter_init(&joint_filter, 0.001, joint_paramater);
	first_order_filter_init(&turn_filter,0.001,turn_paramater);
	first_order_filter_init(&pitch_filter,0.001,pitch_paramater);
	first_order_filter_init(&position_filter,0.001,position_paramater);
}

void ChassisModeUpdate()
{
	
	switch(PTZ.ChassisStatueRequest)
	{
		case 0x01:
			Chassis.Mode = NOFORCE; break;
		case 0x12:
		case 0x92:
			Chassis.Mode = ROTING; break;
		case 0x0A:
		case 0x8A:
			Chassis.Mode = FALLOW; break;
		case 0x06:
		case 0x86:
			Chassis.Mode = STOP; break;
		case 0x2A:
		case 0xAA:
			Chassis.Mode = HIGHSPEED; break;
		default:
			break;
	}	
	
}

fp32 sig(fp32 x)
{
	if(x>0)	return 1;
	return -1;
}
fp32 kp=1.67,kd=1,ki=0.2,k0=0.008,k1=0.0005;
double Fabs(double a)
{
		if(a<0)	return -a;
	return a;
}
void ChassisCommandUpdate()
{
	//printf("%lf,%lf\r\n",Imu.PitchAngle,Imu.PitchSpeed);
	//ps=KalmanFilter(&KFP_pspeed,Imu.PitchSpeed);
	//printf("%lf,%lf\r\n",Imu.PitchAngle,ps);
	if(Chassis.Mode == NOFORCE  )
	{		
		right_slider_foot_current = 0;
		left_slider_foot_current = 0;
		right_foot_position = 0;
		wz_current_right = 0;
		left_foot_position = 0;
		wz_current_left = 0;
		wz_current = 0;
		
	}
	if(Chassis.Mode == FALLOW || Chassis.Mode == ROTING || Chassis.Mode == STOP || Chassis.Mode == HIGHSPEED)//
	{
		po=KalmanFilter(&KFP_drama,Imu.PitchSpeed);
		if(Mode_last == NOFORCE)
		{
			init_time = 500;
			while(init_time)
			{
				Chassis_Control(10000,-10000,0,0);
				init_time -= 1;
				osDelay(1);
			}
			left_flag = 0;
			right_flag = 1;
		}
			
			follow_angle = loop_fp32_constrain(FollowAngle, YawMotorMeasure.angle - 180.0f,YawMotorMeasure.angle + 180.0f);
			
			if(Chassis.Mode == FALLOW){
				Chassis.vx = - PTZ.FBSpeed/32767.0f;
				Chassis.wz = PID_calc(&follow,YawMotorMeasure.angle,follow_angle);
				right_foot_speed = -PID_calc(&speed_pid_right,RightFootMotorMeasure.speed_rpm/5000.0f,Chassis.vx);	
				left_foot_speed = PID_calc(&speed_pid_left,LeftFootMotorMeasure.speed_rpm/5000.0f,-Chassis.vx);	
				if( Fabs(Imu.PitchAngle-(right_foot_speed + left_foot_speed)/2.0) < DRAMA_ANGLE){
					right_foot_position = PID_calc(&position_pid_left_fast,Imu.PitchAngle,(right_foot_speed + left_foot_speed)/2.0);//(right_foot_angle + left_foot_angle)/2.0
					left_foot_position = -PID_calc(&position_pid_right_fast,Imu.PitchAngle,(right_foot_speed + left_foot_speed)/2.0);//(right_foot_angle + left_foot_angle)/2.0
				}
				else{
					right_foot_position = PID_calc(&position_pid_left,Imu.PitchAngle,(right_foot_speed + left_foot_speed)/2.0);//(right_foot_angle + left_foot_angle)/2.0
					left_foot_position = -PID_calc(&position_pid_right,Imu.PitchAngle,(right_foot_speed + left_foot_speed)/2.0);//(right_foot_angle + left_foot_angle)/2.0	
					
				}
					
			}
			else if(Chassis.Mode == ROTING){
					
				Chassis.vx = - PTZ.FBSpeed/32767.0f;
				Chassis.wz = roting_speed;
				right_foot_speed = -PID_calc(&speed_pid_right,RightFootMotorMeasure.speed_rpm/3000.0f,0);	
				left_foot_speed = PID_calc(&speed_pid_left,LeftFootMotorMeasure.speed_rpm/3000.0f,0);		
				if( Fabs(Imu.PitchAngle-(right_foot_speed + left_foot_speed)/2.0) < DRAMA_ANGLE){
					right_foot_position = PID_calc(&position_pid_left_fast,Imu.PitchAngle,(right_foot_speed + left_foot_speed)/2.0);//(right_foot_angle + left_foot_angle)/2.0
					left_foot_position = -PID_calc(&position_pid_right_fast,Imu.PitchAngle,(right_foot_speed + left_foot_speed)/2.0);//(right_foot_angle + left_foot_angle)/2.0
				}
				else{
					right_foot_position = PID_calc(&position_pid_left,Imu.PitchAngle,(right_foot_speed + left_foot_speed)/2.0);//(right_foot_angle + left_foot_angle)/2.0
					left_foot_position = -PID_calc(&position_pid_right,Imu.PitchAngle,(right_foot_speed + left_foot_speed)/2.0);//(right_foot_angle + left_foot_angle)/2.0	
					
				}
			}
			else if(Chassis.Mode == STOP){
				Chassis.vx =  PTZ.LRSpeed/32767.0f;
				Chassis.wz = PID_calc(&follow,YawMotorMeasure.angle,follow_angle + 90.0);//Chassis.vy = - PTZ.LRSpeed/32767.0f;
				right_foot_speed = -PID_calc(&speed_pid_right,RightFootMotorMeasure.speed_rpm/3000.0f,Chassis.vx);	
				left_foot_speed = PID_calc(&speed_pid_left,LeftFootMotorMeasure.speed_rpm/3000.0f,-Chassis.vx);			
				if(Fabs(Imu.PitchAngle-(right_foot_speed + left_foot_speed)/2.0) < DRAMA_ANGLE){
					right_foot_position = PID_calc(&position_pid_left_fast,Imu.PitchAngle,(right_foot_speed + left_foot_speed)/2.0);//(right_foot_angle + left_foot_angle)/2.0
					left_foot_position = -PID_calc(&position_pid_right_fast,Imu.PitchAngle,(right_foot_speed + left_foot_speed)/2.0);//(right_foot_angle + left_foot_angle)/2.0
				}
				else{
					right_foot_position = PID_calc(&position_pid_left,Imu.PitchAngle,(right_foot_speed + left_foot_speed)/2.0);//(right_foot_angle + left_foot_angle)/2.0
					left_foot_position = -PID_calc(&position_pid_right,Imu.PitchAngle,(right_foot_speed + left_foot_speed)/2.0);//(right_foot_angle + left_foot_angle)/2.0	
					
				}
			}
			else if(Chassis.Mode == HIGHSPEED){
				
				Chassis.vx = - PTZ.FBSpeed/32767.0f;
				Chassis.wz = PID_calc(&follow,YawMotorMeasure.angle,follow_angle);
				right_foot_position = PID_calc(&drama_pid_right,RightFootMotorMeasure.speed_rpm, Chassis.vx * 10000 );
				left_foot_position = PID_calc(&drama_pid_left,LeftFootMotorMeasure.speed_rpm, -Chassis.vx * 10000);
				
			}
			
			//Chassis.wz=0;
			wz_left = Chassis.wz;//sqrt(Chassis.vx*Chassis.vx+Chassis.wz*Chassis.wz-2*Chassis.vx*Chassis.wz*cos(follow_angle))*sig(Chassis.wz) ;//
			wz_right = Chassis.wz;//sqrt(Chassis.vx*Chassis.vx+Chassis.wz*Chassis.wz+2*Chassis.vx*Chassis.wz*cos(follow_angle))*sig(Chassis.wz) ;//
			wz_current_right = PID_calc(&turn_pid,RightFootMotorMeasure.speed_rpm/3000.0f,wz_right);	
			wz_current_left = PID_calc(&turn_pid,LeftFootMotorMeasure.speed_rpm/3000.0f,wz_left);	
//			if(Chassis.Mode == ROTING)
//				right_foot_position=0,left_foot_position=0;
	//		first_order_filter_cali(&turn_filter,YawMotorMeasure.angle - erro_angle);	

			x_left=LeftSliderMotorMeasure.ecd+left_flag*8200;//2330 5490+8200
			x_right=RightSliderMotorMeasure.ecd+right_flag*8200;//1350 4525+8200 
			if(x_left<1000)	x_left+=8200;
			if(x_right<1000)	x_right+=8200;
			if(Chassis.Mode == ROTING)
				slider_foot_position = 0;
			else
				slider_foot_position = -PID_calc(&slider_speed,(RightFootMotorMeasure.speed_rpm/3000.0 - LeftFootMotorMeasure.speed_rpm/3000.0)/2.0,Chassis.vx)-PID_calc(&position_drama_pid,po,0);
			
			sp=KalmanFilter(&KFP_slider,slider_foot_position);
			if(Chassis.Mode == HIGHSPEED)
			{
				slider_foot_position_left=(-0.5)*(4360+8200-1230)+(4360+8200-1230)/2+1230;
				slider_foot_position_right=-(-0.5)*(4520+8200-1360)+(4520+8200-1360)/2+1360;
				
			}
			else
			{
				slider_foot_position_left=(sp+angle_zero)*(4360+8200-1230)+(4360+8200-1230)/2+1230;
				slider_foot_position_right=-(sp+angle_zero)*(4520+8200-1360)+(4520+8200-1360)/2+1360;
			}
			left_slider_foot_speed = PID_calc(&position_slider1_pid,x_left,slider_foot_position_left);
			right_slider_foot_speed = PID_calc(&position_slider2_pid,x_right,slider_foot_position_right);
			left_slider_foot_current = PID_calc(&speed_slider1_pid,LeftSliderMotorMeasure.speed_rpm,left_slider_foot_speed);
			right_slider_foot_current = PID_calc(&speed_slider2_pid,RightSliderMotorMeasure.speed_rpm,right_slider_foot_speed);
			
			
			
			
	}
	
	Mode_last = Mode_now;
	Mode_now = Chassis.Mode;
	
}

void RefereeInfUpdate(ext_game_robot_status_t* referee)
{
	memcpy(referee,&robot_state,sizeof(ext_game_robot_status_t));
	
	switch(Referee.chassis_power_limit){
		case 60: roting_speed = ROTING_SPEED_60;break;
		
		case 80: roting_speed = ROTING_SPEED_80;break;
		
		case 100:roting_speed = ROTING_SPEED_100;break;
		
		default: roting_speed = ROTING_SPEED_60;break;
		
	}
				
		
		
}