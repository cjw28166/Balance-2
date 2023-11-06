#include "InterruptService.h"
#include "main.h"
#include "feet_motor.h"
#include "CanPacket.h"
#include "struct_typedef.h"
#include "Setting.h"

double t0=0,tk[4];
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;

int left_flag,right_flag;

CAN_RxHeaderTypeDef rx_header;
uint8_t rx_data[8];
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
 
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	
	if(hcan->Instance == CAN2)
	{
		switch (rx_header.StdId)
		{
			
			case SLIDER_MOTOR1_RECEIVE_ID:
			{
            MotorProcess(rx_header.StdId, hcan, rx_data);
			if(LeftSliderMotorMeasure.ecd-LeftSliderMotorMeasure.last_ecd>8000)	left_flag=0;
			if(LeftSliderMotorMeasure.last_ecd-LeftSliderMotorMeasure.ecd>8000)	left_flag=1;	
			if(RightSliderMotorMeasure.ecd-RightSliderMotorMeasure.last_ecd>8000)	right_flag=0;
			if(RightSliderMotorMeasure.last_ecd-RightSliderMotorMeasure.ecd>8000)	right_flag=1;	
//			t[0]++;
//			tk[0]=t[0]/(t0+0.0);				
            break;

			}
			case SLIDER_MOTOR2_RECEIVE_ID:
			{
            MotorProcess(rx_header.StdId, hcan, rx_data);
			if(LeftSliderMotorMeasure.ecd-LeftSliderMotorMeasure.last_ecd>8000)	left_flag=0;
			if(LeftSliderMotorMeasure.last_ecd-LeftSliderMotorMeasure.ecd>8000)	left_flag=1;	
			if(RightSliderMotorMeasure.ecd-RightSliderMotorMeasure.last_ecd>8000)	right_flag=0;
			if(RightSliderMotorMeasure.last_ecd-RightSliderMotorMeasure.ecd>8000)	right_flag=1;	
//			t[1]++;
//			tk[1]=t[1]/(t0+0.0);				
            break;
			}			
			case FEET_MOTOR1_RECEIVE_ID:
			{
            MotorProcess(rx_header.StdId, hcan, rx_data);
//			t[2]++;
//			tk[2]=t[2]/(t0+0.0);
            break;
			}
			case FEET_MOTOR2_RECEIVE_ID:
			{
            MotorProcess(rx_header.StdId, hcan, rx_data);
//			t[3]++;
//			tk[3]=t[3]/(t0+0.0);
            break;
			}

			default:
			{
			break;
			}
		}
	}
	else if(hcan->Instance == CAN1)
	{
		switch(rx_header.StdId)
		{
			case YawMotorId:
			{
			//MotorProcess(rx_header.StdId,hcan,rx_data);
				get_motor_measure(&YawMotorMeasure,rx_data);
			break;
			}
			/*-------------------------------------云台数据下发接收-------------------------------------*/
			case DefaultAimStatusAndTargetId:
			{
				memcpy(&Aim,rx_data,sizeof(Aim_t));
				break;
			}
			case SentryAimStatusAndTargetId:
			{
				break;//哨兵相关，暂时不管
			}
			case DefaulPTZRequestAndStatusId:
			{
				memcpy(&PTZ,rx_data,sizeof(PTZ_t));
				break;
			}
			case SentryPTZRequestAndStatusId:
			{
				break;//哨兵相关，不管
			}
			default:
			{
				break;
			}
		
		
		}
	
	
	
	}
	
}





void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
    TimerTaskLoop1000Hz();
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
    TimerTaskLoop500Hz_1();
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
    TimerTaskLoop500Hz_2();
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */
    TimerTaskLoop100Hz();
  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}



void TimerTaskLoop1000Hz()
{
	
		
	t0++;
	
	



}

void TimerTaskLoop500Hz_1()
{
	




}

void TimerTaskLoop500Hz_2()
{
			
		
	
	
}

void TimerTaskLoop100Hz()
{


}


