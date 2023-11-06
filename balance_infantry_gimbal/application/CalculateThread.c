#include "CalculateThread.h"
#include "AttitudeThread.h"
#include "InterruptService.h"
#include "Remote.h"
#include "AimbotCan.h"
#include "user_lib.h"
#include "pid.h"
#include "Motor.h"
#include "RefereeCan.h"
#include "tim.h"
#include "bsp_can.h"
#include "stdio.h"
#include "loop_fifo.h"
#include "usart.h"
#include "cmsis_os.h"
#include <string.h>
#include "Infantry4KeyMap.h"
#include "Setting.h"
#include PARAMETER_FILE
#include KEYMAP_FILE

//#define printf(...)  HAL_UART_Transmit_DMA(&huart1,\
//																				(uint8_t  *)u1_buf,\
//																				sprintf((char*)u1_buf,__VA_ARGS__))
//uint8_t u1_buf[10];

Gimbal_t                Gimbal;//云台状态结构
Chassis_t               Chassis;//底盘状态
RC_ctrl_t               Remote;//遥控器数据
AimbotCommand_t         Aimbot;//自瞄数据
OfflineMonitor_t        Offline;//离线检测结构体
RefereeInformation_t    Referee;//裁判系统数据

first_order_filter_type_t  pitch_aimbot_filter;
fp32 pitch_aimbot_filter_param = 0.10f;
fp32 ecd_count=0;
bool_t single_shoot_flag=1;//单发开关
bool_t auto_fire_flag=0;//自动开火开关
bool_t switch_flag=0;//打符切换开关
int16_t dealta_heat=0;
int32_t onelasttime=0;
int16_t onelastheat=0;
uint16_t count=0;

int32_t    gimbal_init_countdown = 0;          //  云台初始化倒计时器
int32_t    gimbal_fire_countdown = 0;          //  云台射击拨盘转动倒计时器
int32_t    gimbal_lagging_counter = 0;         //  云台堵转计数器


void GimbalStateMachineUpdate(void);
void ChassisStateMachineUpdate(void);
void GimbalControlModeUpdate(void);
void GimbalFireModeUpdate(void);
void SetGimbalDisable(void);
void GimbalPIDUpdate(void);
void RotorPIDUpdate(void);
void AmmoPIDUpdate(void);
void GimbalMeasureUpdate(void);
void GimbalCommandUpdate(void);
void ChassisCommandUpdate(void);
void RotorCommandUpdate(void);
void AmmoCommandUpdate(void);
void DebugLEDShow(void);

void BoomBayCover(void);

fp32 LimitNormalization(fp32 input);
extern ImuPacketNormal_t ImuPacket;
extern ImuPacketMini_t ImuPackageMini;
int16_t minus = 0;

void CalculateThread(void const * pvParameters)
{
    osDelay(500);
    PID_init(&Gimbal.Pid.AmmoLeft, PID_POSITION, AMMO_LEFT_SPEED_10MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);//左右摩擦轮pid初始化
    PID_init(&Gimbal.Pid.AmmoRight, PID_POSITION, AMMO_RIGHT_SPEED_10MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
    LoopFifoFp32_init(&Gimbal.ImuBuffer.YawLoopPointer, Gimbal.ImuBuffer.YawAddress, 64);//自瞄数据fifo初始化
    LoopFifoFp32_init(&Gimbal.ImuBuffer.PitchLoopPointer, Gimbal.ImuBuffer.PitchAddress, 64);
    first_order_filter_init(&pitch_aimbot_filter, 1000, &pitch_aimbot_filter_param);//滤波器初始化

    while(1)
    {
        Remote = *get_remote_control_point();//更新遥控器数据
        GetAimbotCommand(&Aimbot);//获取自瞄指令
        GetRefereeInformation(&Referee);//获取裁判系统信息 包括枪口的限制
        DeviceOfflineMonitorUpdate(&Offline);//获取模块离线信息
        
        LoopFifoFp32_push(&Gimbal.ImuBuffer.YawLoopPointer, Gimbal.Imu.YawAngle);
        LoopFifoFp32_push(&Gimbal.ImuBuffer.PitchLoopPointer, Gimbal.Imu.PitchAngle);//陀螺仪数据入栈
        
        GimbalStateMachineUpdate();//根据遥控器拨杆决定当前状态（无力，初始化，测试，比赛）
        ChassisStateMachineUpdate();//底盘状态改变
        GimbalControlModeUpdate();//控制权
        GimbalFireModeUpdate();//开火状态转换
        GimbalPIDUpdate();//云台pid重装载
        RotorPIDUpdate();//拨盘pid重装载
        AmmoPIDUpdate();//射击pid重装载
        GimbalMeasureUpdate();//获取电机和imu数据
        GimbalCommandUpdate();//指令的转换
        ChassisCommandUpdate();//底盘指令转换
        RotorCommandUpdate();//拨盘控制转换
        AmmoCommandUpdate();//发射部分控制转化

        
        DebugLEDShow();
        BoomBayCover();//弹舱盖开合
				
				minus = Aimbot.CommandTimer - ImuPacket.TimeStamp;


				GimbalMotorControl( Gimbal.Output.Yaw * YAW_MOTOR_DIRECTION ,
                            Gimbal.Output.Pitch * PITCH_MOTOR_DIRECTION, 
                            Gimbal.Output.Rotor, //Gimbal.Output.Rotor
                            Gimbal.Output.AmmoLeft, 
                            Gimbal.Output.AmmoRight
                        );
				osDelay(1);
    }
}




void GimbalStateMachineUpdate(void)
{
    // 电机离线保护
    if (Offline.PitchMotor == DEVICE_OFFLINE  ||  Offline.YawMotor == DEVICE_OFFLINE){
        if (Gimbal.StateMachine != GM_NO_FORCE){
                Gimbal.StateMachine = GM_NO_FORCE;
            }
        return;
    }
    
    // 遥控器离线保护
    if (Offline.Remote == DEVICE_OFFLINE){
        if (Gimbal.StateMachine != GM_NO_FORCE){
                Gimbal.StateMachine = GM_NO_FORCE;
            }
        return;
    }
        
    
    
    // 云台状态机
    switch (Remote.rc.s[0]){
        // 右拨杆打到最上，云台复位后进入比赛模式，该模式下开摩擦轮
        case RC_SW_UP:
            if (Gimbal.StateMachine == GM_NO_FORCE){
                Gimbal.StateMachine = GM_INIT;
                gimbal_init_countdown = 2000;
            }
            else if (Gimbal.StateMachine == GM_INIT)
            {
                if (gimbal_init_countdown > 0){
                    gimbal_init_countdown--;
                }
                else{
                    Gimbal.StateMachine = GM_MATCH;//比赛模式
                }
            }
            else{
                Gimbal.StateMachine = GM_MATCH;
            }
            break;
        
        // 右拨杆打到中间，云台复位后进入调试模式
        case RC_SW_MID:
            if (Gimbal.StateMachine == GM_NO_FORCE){
                Gimbal.StateMachine = GM_INIT;
                gimbal_init_countdown = 2000;
            }
            else if (Gimbal.StateMachine == GM_INIT)
            {
                if (gimbal_init_countdown > 0){
                    gimbal_init_countdown--;
                }
                else{
                    Gimbal.StateMachine = GM_TEST;
                }
            }
            else{
                Gimbal.StateMachine = GM_TEST;
            }
            break;
            
        // 右拨杆打到最下，或遥控器数据出错，云台进入无力模式
        case RC_SW_DOWN:
            if (Gimbal.StateMachine != GM_NO_FORCE){
                Gimbal.StateMachine = GM_NO_FORCE;
            }
            break;
        default:
            if (Gimbal.StateMachine != GM_NO_FORCE){
                Gimbal.StateMachine = GM_NO_FORCE;
            }
            break;
    }
}

void ChassisStateMachineUpdate(void)
{
    //if ((Gimbal.StateMachine == GM_NO_FORCE)  ||  (Gimbal.StateMachine == GM_INIT)) {
		if ((Gimbal.StateMachine == GM_NO_FORCE) ) {
        Chassis.ChassisState = CHASSIS_NO_FORCE;
    }//云台无力，底盘强制进入无力状态
		 else if(Gimbal.StateMachine == GM_INIT)
		 {
			if(CHASSIS_ENABLE_KEYMAP)
					Chassis.ChassisState = CHASSIS_FOLLOW;
			else
					Chassis.ChassisState = CHASSIS_NO_FORCE;//CHASSIS_FOLLOW;
		 }
    else {
        if (CHASSIS_ENABLE_KEYMAP/*||(Remote.rc.s[1] == RC_SW_UP)*/) {//左侧拨杆在最下面是底盘有力
            if (CHASSIS_ROTATE_SWITCH_KEYMAP) {//小陀螺模式
                Chassis.ChassisState = CHASSIS_ROTATE;
            }
				else if (CHASSIS_STOP_KEYMAP) {
                Chassis.ChassisState = CHASSIS_NO_MOVE;
				
            }
            else {
                Chassis.ChassisState = CHASSIS_FOLLOW;
            }
            
            if (CHASSIS_HIGH_SPEED_KEYMAP) {
                Chassis.ChassisSpeed = CHASSIS_FAST_SPEED;
            }
            else {
                Chassis.ChassisSpeed = CHASSIS_NORMAL_SPEED;
            }
        }
        else {
            Chassis.ChassisState = CHASSIS_NO_FORCE;
        }
    }
}
void SetGimbalDisable(void)
{
    Gimbal.StateMachine = GM_NO_FORCE;
    Gimbal.ControlMode = GM_NO_CONTROL;
    Gimbal.FireMode = GM_FIRE_UNABLE;
}


void GimbalControlModeUpdate(void)
{
    // 比赛模式下
    if(Gimbal.StateMachine == GM_MATCH){
        // 如果按下鼠标右键并且视觉发现目标，进入自瞄控制
        if (((Remote.mouse.press_r == PRESS)  ||  (Remote.rc.s[1] == RC_SW_UP))   
            &&  (Offline.AimbotStateNode == DEVICE_ONLINE)  &&  (Offline.AimbotDataNode == DEVICE_ONLINE)  
            &&  (Aimbot.State & AIMBOT_TARGET_INSIDE_OFFSET)//大幅和车的切换？
            ) {
						if(single_shoot_flag)
								Gimbal.ControlMode = GM_AIMBOT_RUNES;
						else
								Gimbal.ControlMode = GM_AIMBOT_OPERATE;//自瞄状态
        }
        else{
            Gimbal.ControlMode = GM_MANUAL_OPERATE;//手动状态
        }
    }
    else if (Gimbal.StateMachine == GM_TEST) {
        if (((Remote.mouse.press_r == PRESS)  ||  (Remote.rc.s[1] == RC_SW_UP))  
            &&  (Offline.AimbotStateNode == DEVICE_ONLINE)  &&  (Offline.AimbotDataNode == DEVICE_ONLINE)  
            &&  (Aimbot.State & AIMBOT_TARGET_INSIDE_OFFSET)
            ){
            if(single_shoot_flag)
								Gimbal.ControlMode = GM_AIMBOT_RUNES;
						else
								Gimbal.ControlMode = GM_AIMBOT_OPERATE;//自瞄状态
        }
        else{
            Gimbal.ControlMode = GM_MANUAL_OPERATE;
        }
    }
    else if (Gimbal.StateMachine == GM_INIT) {
        Gimbal.ControlMode = GM_RESET_POSITION;
    }
    else {
        Gimbal.ControlMode = GM_NO_CONTROL;
    }
}


// qylann: 以下代码太乱

void GimbalFireModeUpdate(void)
{		
		//大小符切换
		if(SUPER_CAP_SWITCH_KEYMAP)
				switch_flag=(switch_flag+1)%2;
	
    //自动开火开关
	  if(FIRE_MODE_KEYMAP) 
        auto_fire_flag=(auto_fire_flag+1)%2;   
	
		//单发开关
		if(SINGLE_SHOOT_KEMAP)
				single_shoot_flag=(single_shoot_flag+1)%2;	
    
		dealta_heat=Referee.Ammo0Limit.Heat-Referee.Realtime.Ammo0Heat;
		if(GetSystemTimer()-onelasttime>=1000)
		{
				if(count==0)	rotor_flag=0,ecd_count=RotorMotorMeasure.ecd/10000.0;
				onelasttime=GetSystemTimer(),
				onelastheat=dealta_heat,
				count=0; 
		}

		if(Gimbal.StateMachine!=GM_MATCH)
		{
				Gimbal.FireMode=GM_FIRE_UNABLE;
				gimbal_fire_countdown=0;
				rotor_flag=0,
				ecd_count=RotorMotorMeasure.ecd/10000.0;
		}
    if(Gimbal.StateMachine==GM_MATCH) 
		{
        if(Gimbal.FireMode==GM_FIRE_UNABLE)
            Gimbal.FireMode=GM_FIRE_READY;
        if (Gimbal.FireMode==GM_FIRE_READY) 
				{						
						if((SHOOT_COMMAND_KEYMAP||(CHASSIS_ROTATE_SWITCH_KEYMAP&&Chassis.ChassisState==CHASSIS_NO_FORCE))
							&&((Gimbal.ControlMode==GM_AIMBOT_OPERATE&&(Aimbot.State==0x03||auto_fire_flag==0))||Gimbal.ControlMode==GM_MANUAL_OPERATE)
								/*	&&(count*10<=Referee.Ammo0Limit.Cooling+onelastheat&&(dealta_heat>10||Referee.Ammo0Limit.Heat==0xFFFF))*/	) 
						{
								Gimbal.FireMode=GM_FIRE_BUSY;
								ecd_count+=0.8192*36/8;										
								gimbal_fire_countdown=ROTOR_TIMESET_BUSY; 
								count++;
						}
        }
				if(Gimbal.FireMode==GM_FIRE_BUSY&&gimbal_fire_countdown<=0)
				{
						if(single_shoot_flag==1||(CHASSIS_ROTATE_SWITCH_KEYMAP&&Chassis.ChassisState==CHASSIS_NO_FORCE))
								gimbal_fire_countdown=1000;
						else 
								gimbal_fire_countdown=(int)(10000.0/(dealta_heat/1.7+Referee.Ammo0Limit.Cooling/1.5+5));
						Gimbal.FireMode=GM_FIRE_COOLING;
				}

				if(Gimbal.FireMode==GM_FIRE_COOLING&&gimbal_fire_countdown<=0) 
						Gimbal.FireMode=GM_FIRE_READY;    
				
        //  异常射击模式的状态机，用于反堵转
				if(Gimbal.FireMode==GM_FIRE_LAGGING)
				{
						if(gimbal_fire_countdown<=0)
						{
								Gimbal.FireMode=GM_FIRE_READY;  
								rotor_flag=0,
								ecd_count=RotorMotorMeasure.ecd/10000.0;
						}
				}
				else
				{
						if(Gimbal.Pid.Rotor.s_set-Gimbal.Pid.Rotor.s_fdb>1&&Gimbal.Pid.Rotor.v_fdb<=0)
								gimbal_lagging_counter++;
						else
								gimbal_lagging_counter=0;      
						if (gimbal_lagging_counter>ROTOR_LAGGING_COUNTER_MAX)//ROTOR_LAGGING_COUNTER_MAX
						{        
								gimbal_lagging_counter=0;
								gimbal_fire_countdown=ROTOR_TIMESET_RESERVE;
								Gimbal.FireMode=GM_FIRE_LAGGING;
						}
				}
				
				gimbal_fire_countdown--;
		}
}

// qylann: "     "






GimbalControlMode_e CMthis = GM_NO_CONTROL;
GimbalControlMode_e CMlast = GM_NO_CONTROL;

void GimbalPIDUpdate(void)
{
    CMthis = Gimbal.ControlMode;
    
    if (CMthis == CMlast){
        return;
    }
    
    
    //  
    
    if (CMthis == GM_MANUAL_OPERATE){
        cascade_PID_init(   &Gimbal.Pid.Yaw, 
                            YAW_ANGLE_MANUAL_OPERATE, 
                            YAW_SPEED_MANUAL_OPERATE, 
                            YAW_MAX_SPEED, 
                            YAW_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
        cascade_PID_init(   &Gimbal.Pid.Pitch, 
                            PITCH_ANGLE_MANUAL_OPERATE, 
                            PITCH_SPEED_MANUAL_OPERATE, 
                            PITCH_MAX_SPEED, 
                            PITCH_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
    }
    else if (CMthis == GM_AIMBOT_OPERATE){
        cascade_PID_init(   &Gimbal.Pid.Yaw, 
                            YAW_ANGLE_AIMBOT_OPERATE, 
                            YAW_SPEED_AIMBOT_OPERATE, 
                            YAW_MAX_SPEED, 
                            YAW_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
        cascade_PID_init(   &Gimbal.Pid.Pitch, 
                            PITCH_ANGLE_AIMBOT_OPERATE, 
                            PITCH_SPEED_AIMBOT_OPERATE, 
                            PITCH_MAX_SPEED, 
                            PITCH_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
    }
    else if (CMthis == GM_AIMBOT_RUNES){
        cascade_PID_init(   &Gimbal.Pid.Yaw, 
                            YAW_ANGLE_AIMBOT_RUNES, 
                            YAW_SPEED_AIMBOT_RUNES, 
                            YAW_MAX_SPEED, 
                            YAW_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
        cascade_PID_init(   &Gimbal.Pid.Pitch, 
                            PITCH_ANGLE_AIMBOT_RUNES, 
                            PITCH_SPEED_AIMBOT_RUNES, 
                            PITCH_MAX_SPEED, 
                            PITCH_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
    }
    else if (CMthis == GM_RESET_POSITION){
        cascade_PID_init(   &Gimbal.Pid.Yaw, 
                            YAW_ANGLE_RESET_POSITION, 
                            YAW_SPEED_RESET_POSITION, 
                            YAW_MAX_SPEED, 
                            YAW_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
        cascade_PID_init(   &Gimbal.Pid.Pitch, 
                            PITCH_ANGLE_RESET_POSITION, 
                            PITCH_SPEED_RESET_POSITION, 
                            PITCH_MAX_SPEED, 
                            PITCH_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
    }
    else{
        cascade_PID_init(   &Gimbal.Pid.Yaw, 
                            YAW_ANGLE_NO_FORCE, 
                            YAW_SPEED_NO_FORCE, 
                            YAW_MAX_SPEED, 
                            YAW_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
        cascade_PID_init(   &Gimbal.Pid.Pitch, 
                            PITCH_ANGLE_NO_FORCE, 
                            PITCH_SPEED_NO_FORCE, 
                            PITCH_MAX_SPEED, 
                            PITCH_MAX_ISPEED, 
                            GM6020_MAX_OUTPUT, 
                            GM6020_MAX_IOUTPUT
                            );
    }
    
    CMlast = CMthis;
}





GimbalFireMode_e FMthis = GM_FIRE_UNABLE;
GimbalFireMode_e FMlast = GM_FIRE_UNABLE;
void RotorPIDUpdate(void)
{
    FMthis = Gimbal.FireMode;
    
    if (FMthis == FMlast){
        return;
    }
    
    //
    
//    if ((FMthis == GM_FIRE_READY)  ||  (FMthis == GM_FIRE_COOLING)){
//        PID_init(&Gimbal.Pid.Rotor, PID_POSITION, ROTOR_STOP, M2006_MAX_OUTPUT, M2006_MAX_IOUTPUT);
//    }
//    else if (FMthis == GM_FIRE_BUSY){
//        PID_init(&Gimbal.Pid.Rotor, PID_POSITION, ROTOR_FORWARD, M2006_MAX_OUTPUT, M2006_MAX_IOUTPUT);
//    }
//    else if (FMthis == GM_FIRE_LAGGING){
//        PID_init(&Gimbal.Pid.Rotor, PID_POSITION, ROTOR_BACK, M2006_MAX_OUTPUT, M2006_MAX_IOUTPUT);
//    }
//    else{
//        PID_init(&Gimbal.Pid.Rotor, PID_POSITION, ROTOR_UNABLE, M2006_MAX_OUTPUT, M2006_MAX_IOUTPUT);
//    }
		if(single_shoot_flag||(CHASSIS_ROTATE_SWITCH_KEYMAP&&Chassis.ChassisState==CHASSIS_NO_FORCE))
				cascade_PID_init(&Gimbal.Pid.Rotor, ROTOR_SPEED, ROTOR_SINGLE, ROTOR_MAX_SPEED, ROTOR_MAX_ISPEED,M2006_MAX_OUTPUT, M2006_MAX_IOUTPUT);
		else
				cascade_PID_init(&Gimbal.Pid.Rotor, ROTOR_SPEED, ROTOR_FORWARD, ROTOR_MAX_SPEED, ROTOR_MAX_ISPEED,M2006_MAX_OUTPUT, M2006_MAX_IOUTPUT);
    FMlast = FMthis;
}


uint8_t MSthis = 0;
uint8_t MSlast = 0;
void AmmoPIDUpdate(void)
{
    MSthis = Referee.Ammo0Limit.Speed;//Gimbal.Referee.MaxSpeed;
    
    if (MSthis != MSlast){
        switch (MSthis){
            case 10:
                PID_init(&Gimbal.Pid.AmmoLeft, PID_POSITION, AMMO_LEFT_SPEED_10MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                PID_init(&Gimbal.Pid.AmmoRight, PID_POSITION, AMMO_RIGHT_SPEED_10MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                break;
            case 12:
                PID_init(&Gimbal.Pid.AmmoLeft, PID_POSITION, AMMO_LEFT_SPEED_12MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                PID_init(&Gimbal.Pid.AmmoRight, PID_POSITION, AMMO_RIGHT_SPEED_12MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                break;
            case 14:
                PID_init(&Gimbal.Pid.AmmoLeft, PID_POSITION, AMMO_LEFT_SPEED_14MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                PID_init(&Gimbal.Pid.AmmoRight, PID_POSITION, AMMO_RIGHT_SPEED_14MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                break;
            case 15:
                PID_init(&Gimbal.Pid.AmmoLeft, PID_POSITION, AMMO_LEFT_SPEED_15MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                PID_init(&Gimbal.Pid.AmmoRight, PID_POSITION, AMMO_RIGHT_SPEED_15MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                break;
            case 16:
                PID_init(&Gimbal.Pid.AmmoLeft, PID_POSITION, AMMO_LEFT_SPEED_16MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                PID_init(&Gimbal.Pid.AmmoRight, PID_POSITION, AMMO_RIGHT_SPEED_16MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                break;
            case 18:
                PID_init(&Gimbal.Pid.AmmoLeft, PID_POSITION, AMMO_LEFT_SPEED_18MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                PID_init(&Gimbal.Pid.AmmoRight, PID_POSITION, AMMO_RIGHT_SPEED_18MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                break;
            case 22:
                PID_init(&Gimbal.Pid.AmmoLeft, PID_POSITION, AMMO_LEFT_SPEED_22MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                PID_init(&Gimbal.Pid.AmmoRight, PID_POSITION, AMMO_RIGHT_SPEED_22MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                break;
            case 30:
                PID_init(&Gimbal.Pid.AmmoLeft, PID_POSITION, AMMO_LEFT_SPEED_30MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                PID_init(&Gimbal.Pid.AmmoRight, PID_POSITION, AMMO_RIGHT_SPEED_30MS, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                break;
            default:
                PID_init(&Gimbal.Pid.AmmoLeft, PID_POSITION, DEFAULT_AMMOL_PID, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                PID_init(&Gimbal.Pid.AmmoRight, PID_POSITION, DEFAULT_AMMOR_PID, M3508_MAX_OUTPUT, M3508_MAX_IOUTPUT);
                break;
        }
    }
    
    MSlast = MSthis;
}


void GimbalMeasureUpdate(void)
{
    GimbalMotorMeasureUpdate(&Gimbal.MotorMeasure.GimbalMotor);
    ShootMotorMeasureUpdate(&Gimbal.MotorMeasure.ShootMotor);
    GimbalEulerSystemMeasureUpdate(&Gimbal.Imu);
}

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
KFP KFP_yaw={0.003,0,0,0,0.001,0.543};
KFP KFP_pitch={0.02,0,0,0,0.001,0.45};
//KFP KFP_slider={0.02,0,0,0,0.0001,0.543};
/**
 *卡尔曼滤波器
 *@param KFP *kfp 卡尔曼结构体参数
 *   float input 需要滤波的参数的测量值（即传感器的采集值）
 *@return 滤波后的参数（最优值）
 */
 float KalmanFilter(KFP *kfp,float input)
 {
     float tinput=input;
		 if(tinput-kfp->out>15||tinput-kfp->out<-15)
					tinput=kfp->out;
		 if(tinput-kfp->out>0.8||tinput-kfp->out<-0.8)		
					tinput=kfp->out*0.5+input*0.5;
	   //预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
     kfp->Now_P = kfp->LastP + kfp->Q;
     //卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
     kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
     //更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
     kfp->out = kfp->out + kfp->Kg * (tinput -kfp->out);//因为这一次的预测值就是上一次的输出值
     //更新协方差方程: 本次的系统协方差付给 kfp->LastP 威下一次运算准备。
     kfp->LastP = (1-kfp->Kg) * kfp->Now_P;
     return kfp->out;
 }

float c_y=0,c_y_l=0,k=0.45;
float c_p=0,c_p_l=0;
void GimbalCommandUpdate(void)
{
    if (Gimbal.ControlMode == GM_MANUAL_OPERATE){
				Gimbal.Command.Yaw += GIMBAL_CMD_YAW_KEYMAP;
        Gimbal.Command.Pitch += GIMBAL_CMD_PITCH_KEYMAP;
        Gimbal.Command.Yaw = loop_fp32_constrain(Gimbal.Command.Yaw, Gimbal.Imu.YawAngle - 180.0f, Gimbal.Imu.YawAngle + 180.0f);
        Gimbal.Command.Pitch = fp32_constrain(Gimbal.Command.Pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
				c_y_l=c_y;
				c_y=KalmanFilter(&KFP_yaw,Gimbal.Command.Yaw);
				c_y=(c_y-c_y_l)*k+c_y;
			
				c_p_l=c_p;
				c_p=KalmanFilter(&KFP_pitch,Gimbal.Command.Pitch);
				c_p=(c_p-c_p_l)*k+c_p;
				if(Chassis.ChassisState == CHASSIS_ROTATE)
						Gimbal.Output.Yaw = YAW_cascade_PID_calc(&Gimbal.Pid.Yaw, Gimbal.Imu.YawAngle-Gimbal.MotorMeasure.GimbalMotor.YawMotorSpeed*0.0192 , Gimbal.Imu.YawSpeed, Gimbal.Command.Yaw);
				else
						Gimbal.Output.Yaw = YAW_cascade_PID_calc(&Gimbal.Pid.Yaw, Gimbal.Imu.YawAngle-Gimbal.MotorMeasure.GimbalMotor.YawMotorSpeed*0.0192, Gimbal.Imu.YawSpeed, Gimbal.Command.Yaw);
        Gimbal.Output.Pitch = cascade_PID_calc(&Gimbal.Pid.Pitch, Gimbal.Imu.PitchAngle, Gimbal.Imu.PitchSpeed, Gimbal.Command.Pitch);
        pitch_aimbot_filter.out = Gimbal.Command.Pitch;
    }
    else if (Gimbal.ControlMode == GM_AIMBOT_OPERATE ){
//        Gimbal.Command.Yaw = Gimbal.Imu.YawAngle + Aimbot.YawRelativeAngle;
//        Gimbal.Command.Pitch = Gimbal.Imu.PitchAngle + Aimbot.PitchRelativeAngle;
        Gimbal.Command.Yaw = LoopFifoFp32_read(&Gimbal.ImuBuffer.YawLoopPointer, (GetSystemTimer() - Aimbot.CommandTimer)) + Aimbot.YawRelativeAngle;
        Gimbal.Command.Pitch = LoopFifoFp32_read(&Gimbal.ImuBuffer.PitchLoopPointer, (GetSystemTimer() - Aimbot.CommandTimer)) + Aimbot.PitchRelativeAngle;
				
			//        fp32 pitch_command = LoopFifoFp32_read(&Gimbal.ImuBuffer.PitchLoopPointer, (GetSystemTimer() - Aimbot.CommandTimer)) + Aimbot.PitchRelativeAngle;
//        first_order_filter_cali(&pitch_aimbot_filter, pitch_command);
//        Gimbal.Command.Pitch = pitch_aimbot_filter.out;
        Gimbal.Command.Yaw = loop_fp32_constrain(Gimbal.Command.Yaw, Gimbal.Imu.YawAngle - 180.0f, Gimbal.Imu.YawAngle + 180.0f);
				Gimbal.Command.Pitch = fp32_constrain(Gimbal.Command.Pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
				c_y_l=c_y;
				c_y=KalmanFilter(&KFP_yaw,Gimbal.Command.Yaw);
				c_y=(c_y-c_y_l)*k+c_y;        
				c_p_l=c_p;
				c_p=KalmanFilter(&KFP_pitch,Gimbal.Command.Pitch);
				c_p=(c_p-c_p_l)*k+c_p;
				
				if(Chassis.ChassisState == CHASSIS_ROTATE)
						Gimbal.Output.Yaw = YAW_cascade_PID_calc(&Gimbal.Pid.Yaw, Gimbal.Imu.YawAngle-Gimbal.MotorMeasure.GimbalMotor.YawMotorSpeed*0.0192 , Gimbal.Imu.YawSpeed, Gimbal.Command.Yaw);
				else
						Gimbal.Output.Yaw = YAW_cascade_PID_calc(&Gimbal.Pid.Yaw, Gimbal.Imu.YawAngle-Gimbal.MotorMeasure.GimbalMotor.YawMotorSpeed*0.0192, Gimbal.Imu.YawSpeed, Gimbal.Command.Yaw);
        Gimbal.Output.Pitch = cascade_PID_calc(&Gimbal.Pid.Pitch, Gimbal.Imu.PitchAngle, Gimbal.Imu.PitchSpeed, Gimbal.Command.Pitch);
    }
    else if (Gimbal.ControlMode == GM_AIMBOT_RUNES){
        
//        Gimbal.Command.Yaw = Gimbal.Imu.YawAngle + Aimbot.YawRelativeAngle;
//        Gimbal.Command.Pitch = Gimbal.Imu.PitchAngle + Aimbot.PitchRelativeAngle;
        Gimbal.Command.Yaw = LoopFifoFp32_read(&Gimbal.ImuBuffer.YawLoopPointer, (GetSystemTimer() - Aimbot.CommandTimer)) + Aimbot.YawRelativeAngle;
        Gimbal.Command.Pitch = LoopFifoFp32_read(&Gimbal.ImuBuffer.PitchLoopPointer, (GetSystemTimer() - Aimbot.CommandTimer)) + Aimbot.PitchRelativeAngle;
//        fp32 pitch_command = LoopFifoFp32_read(&Gimbal.ImuBuffer.PitchLoopPointer, (GetSystemTimer() - Aimbot.CommandTimer)) + Aimbot.PitchRelativeAngle;
//        first_order_filter_cali(&pitch_aimbot_filter, pitch_command);
//        Gimbal.Command.Pitch = pitch_aimbot_filter.out;
        Gimbal.Command.Yaw = loop_fp32_constrain(Gimbal.Command.Yaw, Gimbal.Imu.YawAngle - 180.0f, Gimbal.Imu.YawAngle + 180.0f);
        Gimbal.Command.Pitch = fp32_constrain(Gimbal.Command.Pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);
				c_y_l=c_y;
				c_y=KalmanFilter(&KFP_yaw,Gimbal.Command.Yaw);
				c_y=(c_y-c_y_l)*k+c_y;        
				c_p_l=c_p;
				c_p=KalmanFilter(&KFP_pitch,Gimbal.Command.Pitch);
				c_p=(c_p-c_p_l)*k+c_p;
				if(Chassis.ChassisState == CHASSIS_ROTATE)
						Gimbal.Output.Yaw = YAW_cascade_PID_calc(&Gimbal.Pid.Yaw, Gimbal.Imu.YawAngle-Gimbal.MotorMeasure.GimbalMotor.YawMotorSpeed*0.0192 , Gimbal.Imu.YawSpeed, Gimbal.Command.Yaw);
				else
						Gimbal.Output.Yaw = YAW_cascade_PID_calc(&Gimbal.Pid.Yaw, Gimbal.Imu.YawAngle-Gimbal.MotorMeasure.GimbalMotor.YawMotorSpeed*0.0192, Gimbal.Imu.YawSpeed, Gimbal.Command.Yaw);
        Gimbal.Output.Pitch = cascade_PID_calc(&Gimbal.Pid.Pitch, Gimbal.Imu.PitchAngle, Gimbal.Imu.PitchSpeed, Gimbal.Command.Pitch);

    }
    else if (Gimbal.ControlMode == GM_RESET_POSITION){
        Gimbal.Command.Yaw = Gimbal.Imu.YawAngle;
        Gimbal.Command.Pitch = Gimbal.Imu.PitchAngle;
        fp32 YawTempCommand = loop_fp32_constrain(YAW_ZERO_ECDANGLE, Gimbal.MotorMeasure.GimbalMotor.YawMotorAngle - 180.0f, Gimbal.MotorMeasure.GimbalMotor.YawMotorAngle + 180.0f);
//        Gimbal.Output.Yaw = YAW_MOTOR_DIRECTION * cascade_PID_calc(&Gimbal.Pid.Yaw, Gimbal.MotorMeasure.GimbalMotor.YawMotorAngle, Gimbal.MotorMeasure.GimbalMotor.YawMotorSpeed, YAW_ZERO_ECDANGLE);
        Gimbal.Pid.Yaw.v_set = PID_calc(&Gimbal.Pid.Yaw.pid_outside, Gimbal.MotorMeasure.GimbalMotor.YawMotorAngle, YawTempCommand);
        Gimbal.Output.Yaw = YAW_MOTOR_DIRECTION * PID_calc(&Gimbal.Pid.Yaw.pid_inside, Gimbal.MotorMeasure.GimbalMotor.YawMotorSpeed, Gimbal.Pid.Yaw.v_set);  
        Gimbal.Output.Pitch = cascade_PID_calc(&Gimbal.Pid.Pitch, Gimbal.Imu.PitchAngle, Gimbal.Imu.PitchSpeed, 0);
        pitch_aimbot_filter.out = Gimbal.Command.Pitch;
    }
    else{
        Gimbal.Command.Yaw = Gimbal.Imu.YawAngle;
        Gimbal.Command.Pitch = Gimbal.Imu.PitchAngle;
        Gimbal.Output.Yaw = 0;
        Gimbal.Output.Pitch = 0;
        pitch_aimbot_filter.out = Gimbal.Command.Pitch;
    }
}

void ChassisCommandUpdate(void)
{
    if ((Chassis.ChassisState == CHASSIS_NO_FORCE) ) {
        Chassis.ChassisCommandX = 0.0f;
        Chassis.ChassisCommandY = 0.0f;
    }
    else {
        Chassis.ChassisCommandX = CHASSIS_CMD_X_KEYMAP;
        Chassis.ChassisCommandY = CHASSIS_CMD_Y_KEYMAP;
    }
}
/*
void RotorCommandUpdate(void)
{
		Gimbal.Command.Rotor = ecd_count;
		if(Gimbal.FireMode == GM_FIRE_UNABLE)
		{
        Gimbal.Output.Rotor = 0;
        return;
    }
    Gimbal.Output.Rotor = cascade_PID_calc(&Gimbal.Pid.Rotor, Gimbal.MotorMeasure.ShootMotor.RotorMotorEcd/10000.0+rotor_flag*0.8192,Gimbal.MotorMeasure.ShootMotor.RotorMotorSpeed, Gimbal.Command.Rotor);
		
}
*/
void RotorCommandUpdate(void)
{
    if(Gimbal.FireMode == GM_FIRE_BUSY||Gimbal.FireMode == GM_FIRE_READY||Gimbal.FireMode == GM_FIRE_COOLING)
		{
        //Gimbal.Command.Rotor = ROTOR_SPEEDSET_FORWARD * ROTOR_MOTOR_DIRECTION;
				Gimbal.Output.Rotor = cascade_PID_calc(&Gimbal.Pid.Rotor, RotorMotorMeasure.ecd/10000.0+0.8192*rotor_flag, RotorMotorMeasure.speed_rpm,ecd_count);
    
    }
    if(Gimbal.FireMode == GM_FIRE_LAGGING)
		{
        //Gimbal.Command.Rotor = ROTOR_SPEEDSET_BACKWARD * (-ROTOR_MOTOR_DIRECTION);
				Gimbal.Output.Rotor = -5000;
    }
    if(Gimbal.FireMode == GM_FIRE_UNABLE)
		{
        //Gimbal.Command.Rotor = 0;
        Gimbal.Output.Rotor = 0;
    }

		//ecd_count=RotorMotorMeasure.ecd/10000.0+0.8192*rotor_flag;
    //Gimbal.Output.Rotor = PID_calc(&Gimbal.Pid.Rotor, Gimbal.MotorMeasure.ShootMotor.RotorMotorSpeed, Gimbal.Command.Rotor);
}




void AmmoCommandUpdate(void)
{
    if (Gimbal.FireMode == GM_FIRE_UNABLE){
        Gimbal.Command.AmmoLeft = 0;
        Gimbal.Command.AmmoRight = 0;
        Gimbal.Output.AmmoLeft = 0;
        Gimbal.Output.AmmoRight = 0;
        return;
    }
    switch (MSthis){
        case 10:
            Gimbal.Output.AmmoLeft = PID_calc(  &Gimbal.Pid.AmmoLeft,
                                                Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed, 
                                                AMMO_SPEEDSET_10MS * AMMO_LEFT_MOTOR_DIRECTION
                                                );
            Gimbal.Output.AmmoRight = PID_calc( &Gimbal.Pid.AmmoRight, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoRightMotorSpeed, 
                                                AMMO_SPEEDSET_10MS * AMMO_RIGHT_MOTOR_DIRECTION
                                                );
            break;
        
        case 12:
            Gimbal.Output.AmmoLeft = PID_calc(  &Gimbal.Pid.AmmoLeft, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed, 
                                                AMMO_SPEEDSET_12MS * AMMO_LEFT_MOTOR_DIRECTION
                                                );
            Gimbal.Output.AmmoRight = PID_calc( &Gimbal.Pid.AmmoRight, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoRightMotorSpeed, 
                                                AMMO_SPEEDSET_12MS * AMMO_RIGHT_MOTOR_DIRECTION
                                                );
            break;
        
        case 14:
            Gimbal.Output.AmmoLeft = PID_calc(  &Gimbal.Pid.AmmoLeft, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed, 
                                                AMMO_SPEEDSET_14MS * AMMO_LEFT_MOTOR_DIRECTION
                                                );
            Gimbal.Output.AmmoRight = PID_calc( &Gimbal.Pid.AmmoRight, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoRightMotorSpeed, 
                                                AMMO_SPEEDSET_14MS * AMMO_RIGHT_MOTOR_DIRECTION
                                                );
            break;
        
        case 15:
			
            Gimbal.Output.AmmoLeft = PID_calc(  &Gimbal.Pid.AmmoLeft, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed, 
                                                AMMO_SPEEDSET_15MS * AMMO_LEFT_MOTOR_DIRECTION
                                                );
            Gimbal.Output.AmmoRight = PID_calc( &Gimbal.Pid.AmmoRight, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoRightMotorSpeed, 
                                                AMMO_SPEEDSET_15MS * AMMO_RIGHT_MOTOR_DIRECTION
                                                );
			
			
            break;
        
        case 16:
            Gimbal.Output.AmmoLeft = PID_calc(  &Gimbal.Pid.AmmoLeft, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed, 
                                                AMMO_SPEEDSET_16MS * AMMO_LEFT_MOTOR_DIRECTION
                                                );
            Gimbal.Output.AmmoRight = PID_calc( &Gimbal.Pid.AmmoRight, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoRightMotorSpeed, 
                                                AMMO_SPEEDSET_16MS * AMMO_RIGHT_MOTOR_DIRECTION
                                                );
            break;
        
        case 18:
            Gimbal.Output.AmmoLeft = PID_calc(  &Gimbal.Pid.AmmoLeft, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed, 
                                                AMMO_SPEEDSET_18MS * AMMO_LEFT_MOTOR_DIRECTION
                                                );
            Gimbal.Output.AmmoRight = PID_calc( &Gimbal.Pid.AmmoRight, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoRightMotorSpeed, 
                                                AMMO_SPEEDSET_18MS * AMMO_RIGHT_MOTOR_DIRECTION
                                                );
            break;
        
        case 22:
            Gimbal.Output.AmmoLeft = PID_calc(  &Gimbal.Pid.AmmoLeft, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed, 
                                                AMMO_SPEEDSET_22MS * AMMO_LEFT_MOTOR_DIRECTION
                                                );
            Gimbal.Output.AmmoRight = PID_calc( &Gimbal.Pid.AmmoRight, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoRightMotorSpeed, 
                                                AMMO_SPEEDSET_22MS * AMMO_RIGHT_MOTOR_DIRECTION
                                                );
            break;
        
        case 30:
            Gimbal.Output.AmmoLeft = PID_calc(  &Gimbal.Pid.AmmoLeft, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed, 
                                                AMMO_SPEEDSET_30MS * AMMO_LEFT_MOTOR_DIRECTION
                                                );
            Gimbal.Output.AmmoRight = PID_calc( &Gimbal.Pid.AmmoRight, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoRightMotorSpeed, 
                                                AMMO_SPEEDSET_30MS * AMMO_RIGHT_MOTOR_DIRECTION
                                                );
            break;
        
        default:
            Gimbal.Output.AmmoLeft = PID_calc(  &Gimbal.Pid.AmmoLeft, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoLeftMotorSpeed, 
                                                DEFAULT_AMMO_SPEEDSET * AMMO_LEFT_MOTOR_DIRECTION
                                                );
            Gimbal.Output.AmmoRight = PID_calc( &Gimbal.Pid.AmmoRight, 
                                                Gimbal.MotorMeasure.ShootMotor.AmmoRightMotorSpeed, 
                                                DEFAULT_AMMO_SPEEDSET * AMMO_RIGHT_MOTOR_DIRECTION
                                                );
            break;
    }
    
}


void GetGimbalMotorOutput(GimbalOutput_t *out)
{
    memcpy(out, &Gimbal.Output, sizeof(GimbalOutput_t));
}

bool_t cover_flag = 0;
void BoomBayCover(void)
{
		__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 2000);
		/*
    if (Gimbal.StateMachine == GM_MATCH) {
        if (cover_flag == 0) {
            cover_flag = 2;
            
        }
        
        if (COVER_SWITCH_KEYMAP) {
            if (cover_flag == 1) {
                cover_flag = 2;
                HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_SET);
            }
            else if (cover_flag == 2) {
                cover_flag = 1;
                HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_RESET);
            }
        }
    }
    else {
        if (SHOOT_COMMAND_KEYMAP) {
              cover_flag = 1;
                HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_RESET);
        }
		else{
		 cover_flag = 2;
                HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_SET);
		}

    }
    
		
   
    if ((cover_flag == 1)  ||  (cover_flag == 0)) {
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 2000);//四号：2245 三号 1250    打开
     }
    if (cover_flag == 2) {
         __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 650);//四号：500   关闭
    }
    
    */
    
}
uint8_t t=5;

void GetGimbalRequestState(GimbalRequestState_t *RequestState)
{
	   if (Gimbal.StateMachine == GM_NO_FORCE) {
         RequestState->GimbalState |= (uint8_t)(1 << 0);
    }
	
	
		
    RequestState->AimbotRequest = 0x00;
		if(single_shoot_flag == 1)
		{
				if(switch_flag)
						RequestState->AimbotRequest |= (uint8_t)(1 << 4);
				else
						RequestState->AimbotRequest |= (uint8_t)(1 << 5);
		}    
		else
				RequestState->AimbotRequest |= (uint8_t) (1 << 0);
	  //RequestState->AimbotRequest |= (uint8_t) (1 << t);
    
    
    
    RequestState->ChassisMoveXRequest = Chassis.ChassisCommandX * 32767;
    RequestState->ChassisMoveYRequest = Chassis.ChassisCommandY * 32767;
    RequestState->ChassisStateRequest = 0x00;

    
    
    if (Chassis.ChassisState != CHASSIS_NO_FORCE) {
        RequestState->ChassisStateRequest |= (uint8_t)(1 << 1);
        // 运动状态
        if (Chassis.ChassisState == CHASSIS_NO_MOVE) {
            RequestState->ChassisStateRequest |= (uint8_t)(1 << 2);
        }
        else if (Chassis.ChassisState == CHASSIS_FOLLOW) {
            RequestState->ChassisStateRequest |= (uint8_t)(1 << 3);
        }
        else if (Chassis.ChassisState == CHASSIS_ROTATE) {
            RequestState->ChassisStateRequest |= (uint8_t)(1 << 4);
        }
       
				if(CHASSIS_HIGH_SPEED_KEYMAP){
					RequestState->ChassisStateRequest |= (uint8_t)(1<<5);
				}
				if(CHASSIS_STOP_KEYMAP){
					RequestState->ChassisStateRequest |= (uint8_t)(1<<2);
				}
        
//        if (Chassis.ChassisSpeed == CHASSIS_NORMAL_SPEED) {
//            RequestState->ChassisStateRequest |= (uint8_t)(1 << 5);
//        }
//        else if (Chassis.ChassisSpeed == CHASSIS_FAST_SPEED) {
//            RequestState->ChassisStateRequest |= (uint8_t)(1 << 6);
//        }
//        else if (Chassis.ChassisSpeed == CHASSIS_LOW_SPEED) {
//            RequestState->ChassisStateRequest |= (uint8_t)(1 << 7);
//        }
    }
    else {
        RequestState->ChassisStateRequest |= (uint8_t)(1 << 0);
    }
    
    RequestState->GimbalState = 0x00;
    
    
    
    if ((Remote.mouse.press_r == PRESS)  ||  (Remote.rc.s[1] == RC_SW_UP)) {
        RequestState->GimbalState |= (uint8_t) (1 << 1);
    }
    
    if (cover_flag == 2) {
        RequestState->GimbalState |= (uint8_t) (1 << 3);
    }
    else if (cover_flag == 1) {
        RequestState->GimbalState |= (uint8_t) (1 << 4);
    }
	
	if(auto_fire_flag == 1){
		RequestState->GimbalState |= (uint8_t) (1 << 6);
	}
	
    

    
        
    
    
    RequestState->Reserve = 0x00;
    
}



void DebugLEDShow(void)
{
    if ((Offline.AimbotStateNode == DEVICE_ONLINE)  &&  (Offline.AimbotDataNode == DEVICE_ONLINE)){
        HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
        if ((Aimbot.State & AIMBOT_TARGET_INSIDE_OFFSET)){
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
        }
        else{
            HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
        }
    }
    else{
        HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
    }
    
    
}


fp32 LimitNormalization(fp32 input)
{
    if (input > 1.0f){
        return 1.0f;
    }
    else if (input < -1.0f){
        return -1.0f;
    }
    else{
        return input;
    }
}



void RefereeHeatInterpolation(void)
{
    Referee.Realtime.Ammo0Heat -= Referee.Ammo0Limit.Cooling / 10;
    if (Referee.Realtime.Ammo0Heat < 0) {
        Referee.Realtime.Ammo0Heat = 0;
    }
}



