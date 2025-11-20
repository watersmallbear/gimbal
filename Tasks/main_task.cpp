/**
*******************************************************************************
* @file      :main_task.cpp
* @brief     :
* @history   :
*  Version     Date            Author          Note
*  V0.9.0      yyyy-mm-dd      <author>        1. <note>
*******************************************************************************
* @attention :
*******************************************************************************
*  Copyright (c) 2024 Hello World Team，Zhejiang University.
*  All Rights Reserved.
*******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "main_task.hpp"
#include "system_user.hpp"

#include "DT7.hpp"
#include "HW_fdcan.hpp"
#include "dm4310_drv.hpp"
#include "iwdg.h"
#include "math.h"
#include "imu_task.hpp"
#include "PID.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

#define DISABLEMOTOR false
#define ENABLEMOTOR true

struct Pid_Parameter{
  float kp;
  float ki;
  float kd;
};

static FDCAN_RxHeaderTypeDef rx_header1,rx_header3;
static uint8_t can1_rx_data[8],can1_rx_data_1[8],can1_tx_data[8],can3_rx_data[8];

uint32_t tick = 0;

namespace remote_control = hello_world::devices::remote_control;
static const uint8_t kRxBufLen = remote_control::kRcRxDataLen;
static uint8_t rx_buf[kRxBufLen] __attribute__((section(".RAM_D1")));
remote_control::DT7 *rc_ptr;
float chassis_euler_angles[3] = {0};
float gimbal_last_euler_angles[3] = {0};
float max_pitch = 0;
float pitch_omega = 0;
float torque = 0;
float rc_rv_int = 0;
int16_t count_tick = 0;
float max_pitch_angle = 1.6;
float min_pitch_angle = 0.01;
float horizon_pitch_angle = 0.36;
float torque_feedforward = 4;
float position = horizon_pitch_angle;
float temp_pitch_angle = horizon_pitch_angle;
bool wether_enable_motor = true;

motor_fbpara_t pitch_motor_para;
Joint_Motor_t pitch_motor_t;
motor_fbpara_t yaw_motor_para;
Joint_Motor_t yaw_motor_t;

Pid pitch_pid;
Pid torque_pid;

Pid_Parameter pitch_pid_parameter = {15,0,0};
Pid_Parameter torch_pid_parameter = {5, 0, 0};

bool Wether_Enable_Motor();
void Send_To_Chassis();
void Pitch_Motor_Control();
void Decode_Chassis();

void RobotInit(void) { rc_ptr = new remote_control::DT7(); }

void MainInit(void) {
  RobotInit();

  ImuInit();

  pitch_pid.Init(pitch_pid_parameter.kp, pitch_pid_parameter.ki, pitch_pid_parameter.kd, 0.001);
  torque_pid.Init(torch_pid_parameter.kp, torch_pid_parameter.ki, torch_pid_parameter.kd, 0.001);

  torque_pid.Derivative_Feedforward(0.01);

  // 开启FDCAN
  FdcanFilter_Init(&hfdcan1);
  HAL_FDCAN_Start(&hfdcan1);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  FdcanFilter_Init(&hfdcan2);
  HAL_FDCAN_Start(&hfdcan2);
  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  FdcanFilter_Init(&hfdcan3);
  HAL_FDCAN_Start(&hfdcan3);
  HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0);

  enable_motor_mode(&hfdcan3, 9, MIT_MODE);
  joint_motor_init(&pitch_motor_t, 9, MIT_MODE);

  // 开启遥控器接收
  HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rx_buf, kRxBufLen);

  // 开启定时器
  HAL_TIM_Base_Start_IT(&htim6);
}

void MainTask(void) {
  tick++; 
  ImuUpdate();

  if (tick >= 2000 && wether_enable_motor){

    Send_To_Chassis();
    Pitch_Motor_Control();

  }else if(wether_enable_motor == false){
    position = 0;
    temp_pitch_angle = 0;

    mit_ctrl(&hfdcan3, 9, 0, 0, 0, 0, 0);
  }else{

    mit_ctrl(&hfdcan3, 9, 0, 0, 0, 0, 0);
  }
  
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

  if (htim == &htim6) {
    MainTask();
  }
}

uint8_t rx_data = 0;

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs) {
  if (hfdcan == &hfdcan1) {
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header1,can1_rx_data) ==HAL_OK) // 获得接收到的数据头和数据
    {
      if (rx_header1.Identifier == 0x510) { // 帧头校验
        HAL_IWDG_Refresh(&hiwdg1);
        
        Decode_Chassis();
      }
      if (rx_header1.Identifier == 0x13){

        dm4310_fbdata(&yaw_motor_t, can1_rx_data, 8);
      }
    }
  } else if (hfdcan == &hfdcan2) {
  }
  HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0); // 再次使能FIFO0接收中断
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo1ITs) {
  if (hfdcan == &hfdcan3) {
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rx_header3,
                               can3_rx_data) ==
        HAL_OK) // 获得接收到的数据头和数据
    {
      if (rx_header3.Identifier == 0x19) { // 帧头校验
        dm4310_fbdata(&pitch_motor_t, can3_rx_data, 8);
      }
    }
  }
  HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE,
                                 0); // 再次使能FIFO0接收中断
}

void Pitch_Motor_Control(){

  pitch_omega = pitch_motor_t.para.vel;

  if (fabs(rc_rv_int) > 0.1){
    position += rc_rv_int * 0.0005;

    if (position < min_pitch_angle){position = min_pitch_angle;}
    else if(position > max_pitch_angle){position = max_pitch_angle;}

    temp_pitch_angle = position;

    pitch_pid.Control(position, pitch_motor_t.para.pos);
    torque_pid.Control(pitch_pid.Result(), pitch_omega);

    torque = torque_pid.Result() + torque_feedforward;
    mit_ctrl(&hfdcan3, 9, 0, 0, 0, 0, torque);

  }else{

    pitch_pid.Control(temp_pitch_angle, pitch_motor_t.para.pos);
    torque_pid.Control(pitch_pid.Result(), pitch_omega);

    torque = torque_pid.Result() + torque_feedforward;
    mit_ctrl(&hfdcan3, 9, 0, 0, 0, 0, torque);

  }
}

bool Wether_Enable_Motor(){

  if (rc_ptr->rc_l_switch() == 1){return DISABLEMOTOR;}
  else                           {return ENABLEMOTOR;}
}

void Send_To_Chassis(){
    // 将浮点数转换为整数（假设需要精度到0.01度）
    int32_t angle_int = (int32_t)(gimbal_euler_angles[0] * 10000.0f);
    int8_t yaw_motor_angle = float_to_uint(yaw_motor_t.para.pos, -3.14, 3.14, 8);
    
    // 拆分为3个字节
    can1_tx_data[0] = (angle_int >> 16) & 0xFF;  // 高字节
    can1_tx_data[1] = (angle_int >> 8) & 0xFF;   // 中字节
    can1_tx_data[2] = angle_int & 0xFF;          // 低字节
    
    can1_tx_data[3] = yaw_motor_angle;
    can1_tx_data[4] = 0;
    can1_tx_data[5] = 0;
    can1_tx_data[6] = 0;
    can1_tx_data[7] = 0;

    FDCAN_Send_Msg(&hfdcan1, can1_tx_data, 0x114, 8);
}

void Decode_Chassis(){
    
    rc_rv_int = uint_to_float(can1_rx_data[3], -1, 1, 8);

    wether_enable_motor = can1_rx_data[2];
}
