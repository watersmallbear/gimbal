/**
 *******************************************************************************
 * @file      :HW_fdcan.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2023 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "HW_fdcan.hpp"
#include "stdint.h"

#include "dm4310_drv.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

uint32_t pTxMailbox;
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/**
 * @brief
 * @param        *hcan:
 * @retval       None
 * @note        None
 */
void FdcanFilter_Init(FDCAN_HandleTypeDef *hfdcan) {

  FDCAN_FilterTypeDef filter_config{
      .IdType = FDCAN_STANDARD_ID,
      .FilterIndex = 0,
      .FilterType = FDCAN_FILTER_MASK,
      .FilterID1 = 0x000,
      .FilterID2 = 0x000,
      .RxBufferIndex = 0,
      .IsCalibrationMsg = 0,
  };

  if (hfdcan == &hfdcan1 || hfdcan == &hfdcan2) {
    filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  } else if (hfdcan == &hfdcan3) {
    filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
  }

  if (HAL_FDCAN_ConfigFilter(hfdcan, &filter_config) != HAL_OK) {
    Error_Handler();
  }
}

uint32_t can_rec_times = 0;
uint32_t can_success_times = 0;
uint32_t can_receive_data = 0;

/**
 * @brief   CAN中断的回调函数，全部数据解析都在该函数中
 * @param   hcan为CAN句柄
 * @retval  none
 * @note
 **/
// void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan,
//                                 uint32_t RxFifo1ITs) {
//    if (hfdcan == &hfdcan3) {
//      if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rx_header3,
//                                 can3_rx_data) ==
//          HAL_OK) // 获得接收到的数据头和数据
//      {
//        if (rx_header3.Identifier == 0x200) { // 帧头校验
//         dm4310_fbdata(&pitch_motor_t,can3_rx_data,8); // 校验通过进行具体数据处理
//        }
//      }
//    }
//    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE,
//                                   0); // 再次使能FIFO0接收中断
//  }

/**
 * @brief   CAN中断的回调函数，全部数据解析都在该函数中
 * @param   hcan为CAN句柄
 * @retval  none
 * @note
 **/
// void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan,
//                                uint32_t RxFifo1ITs) {
//   if (hfdcan == &hfdcan3) {
//     if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rx_header3,
//                                can3_rx_data) ==
//         HAL_OK) // 获得接收到的数据头和数据
//     {
//       if (rx_header3.Identifier == 0x200) { // 帧头校验
//         // 校验通过进行具体数据处理
//       }
//     }
//   }
//   HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE,
//                                  0); // 再次使能FIFO0接收中断
// }

/**
 * @brief   向can总线发送数据，抄官方的
 * @param   hcan为CAN句柄
 * @param	msg为发送数组首地址
 * @param	id为发送报文
 * @param	len为发送数据长度（字节数）
 * @retval  none
 * @note    主控发送都是len=8字节，再加上帧间隔3位，理论上can总线1ms最多传输9帧
 **/
void FDCAN_Send_Msg(FDCAN_HandleTypeDef *hfdcan, uint8_t *msg, uint32_t id,
                    uint8_t len) {
  FDCAN_TxHeaderTypeDef TxMessageHeader = {0};

  TxMessageHeader.Identifier = id;                // 32位ID
  TxMessageHeader.IdType = FDCAN_STANDARD_ID;     // 标准ID
  TxMessageHeader.TxFrameType = FDCAN_DATA_FRAME; // 数据帧
  TxMessageHeader.DataLength = len;               // 数据长度
  TxMessageHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxMessageHeader.BitRateSwitch = FDCAN_BRS_OFF;           // 关闭速率切换
  TxMessageHeader.FDFormat = FDCAN_CLASSIC_CAN;            // 传统的CAN模式
  TxMessageHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // 无发送事件
  TxMessageHeader.MessageMarker = 0;
  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxMessageHeader, msg) != HAL_OK) {
  }
}
