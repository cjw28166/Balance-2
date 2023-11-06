
#include "feet_motor.h"
#include "bsp_can.h"
#include "CalculateThread.h"
#include "Setting.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
static uint32_t             send_mail_box;
static CAN_TxHeaderTypeDef  can_tx_message;
static uint8_t              MotorSendBuffer[16];
static uint8_t				JointSendBuffer[8];


void MotorProcess(uint32_t MotorID, CAN_HandleTypeDef *hcan, uint8_t* message);

void Chassis_Control(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    can_tx_message.StdId = CAN_FEET_ALL_ID;
    can_tx_message.IDE = CAN_ID_STD;
    can_tx_message.RTR = CAN_RTR_DATA;
    can_tx_message.DLC = 0x08;
    MotorSendBuffer[0] = motor1 >> 8;
    MotorSendBuffer[1] = motor1;
    MotorSendBuffer[2] = motor2 >> 8;
    MotorSendBuffer[3] = motor2;
    MotorSendBuffer[4] = motor3 >> 8;
    MotorSendBuffer[5] = motor3;
    MotorSendBuffer[6] = motor4 >> 8;
    MotorSendBuffer[7] = motor4;
    HAL_CAN_AddTxMessage(&hcan2, &can_tx_message, MotorSendBuffer, &send_mail_box);
}

motor_measure_t LeftFootMotorMeasure;
motor_measure_t RightFootMotorMeasure;
motor_measure_t LeftSliderMotorMeasure;
motor_measure_t RightSliderMotorMeasure;
motor_measure_t YawMotorMeasure;

//motor data read

void MotorProcess(uint32_t MotorID, CAN_HandleTypeDef *hcan, uint8_t* message)
{
    switch (MotorID){
		/*case YawMotorId:
			get_motor_measure(&YawMotorMeasure,message);
			break; */
        case FEET_MOTOR1_RECEIVE_ID:
            get_motor_measure(&RightFootMotorMeasure, message);
            break;
        case FEET_MOTOR2_RECEIVE_ID:
            get_motor_measure(&LeftFootMotorMeasure, message);
            break;
		case SLIDER_MOTOR1_RECEIVE_ID:
            get_motor_measure(&LeftSliderMotorMeasure, message);
            break;
        case SLIDER_MOTOR2_RECEIVE_ID:
            get_motor_measure(&RightSliderMotorMeasure, message);
            break;
        default:
            break;
    }
}













