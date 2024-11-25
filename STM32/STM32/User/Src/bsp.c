/*
 * bsp.c
 *
 *  Created on: May 22, 2024
 *      Author: 23877
 */

#include "bsp.h"
#include "board.h"
#include "can.h"
#include "main.h"
#include "unitree_a1.h"
#include "unitree_go.h"
#include "usbd_cdc_if.h"
#include "cmsis_os.h"

RS485_port_info unitree_port[2];
CAN_info CAN_port[2];

JogCmd temp_jog_cmd;
RobotFBData robot_fb_data;

void BSP_Init() {
	unitree_port[0].port = RS485_DIR1_GPIO_Port;  // A1 motor
	unitree_port[0].pin = RS485_DIR1_Pin;
	unitree_port[0].uart.handle = &huart1;
	unitree_port[1].port = RS485_DIR2_GPIO_Port;  // GO motor
	unitree_port[1].pin = RS485_DIR2_Pin;
	unitree_port[1].uart.handle = &huart6;
	for (uint8_t i = 0; i < 2; ++i) {
		// 使能串口空闲中断
		__HAL_UART_ENABLE_IT(unitree_port[i].uart.handle, UART_IT_IDLE);
		// 开启DMA接收
		HAL_UART_Receive_DMA(unitree_port[i].uart.handle, unitree_port[i].uart.rx_buff, UART_BUFFER_SIZE);
	}

	CAN_port[0].handle = &hcan1;
	CAN_port[1].handle = &hcan2;
}

void RS485_Send(uint8_t ind, const uint8_t *data, uint16_t size, uint32_t timeout) {
//	HAL_GPIO_WritePin(unitree_port[ind].port, unitree_port[ind].pin, GPIO_PIN_SET);
	if (HAL_UART_Transmit(unitree_port[ind].uart.handle, data, size, timeout) != HAL_OK)  // 判断是否发送正常，如果出现异常则进入异常中断函数
		Error_Handler();
//	HAL_GPIO_WritePin(unitree_port[ind].port, unitree_port[ind].pin, GPIO_PIN_RESET);
}

void RS485_Send_DMA(uint8_t ind, const uint8_t *data, uint16_t size) {
	// 串口1有板载485芯片，需要自己控制数据流向，发送时需要将控制口SET；串口6外接TTL转485模块，自动控制数据流向，无需SET。
	// 为保证代码一致性和美观性，以下代码中二者不做区分
	HAL_GPIO_WritePin(unitree_port[ind].port, unitree_port[ind].pin, GPIO_PIN_SET);
	if (HAL_UART_Transmit_DMA(unitree_port[ind].uart.handle, data, size) != HAL_OK)  // 判断是否发送正常，如果出现异常则进入异常中断函数
		Error_Handler();
	while (__HAL_UART_GET_FLAG(unitree_port[ind].uart.handle, UART_FLAG_TC) != SET) {
	}  // 等待数据发送完成
	HAL_GPIO_WritePin(unitree_port[ind].port, unitree_port[ind].pin, GPIO_PIN_RESET);
}

/* 放在"stm32f4xx_it.c"里形如"void USART2_IRQHandler(void)"类的函数中，只要用了DMA接收的串口都放 */
void USER_UART_IRQHandler(UART_HandleTypeDef *huart) {
	// 判断是否进入空闲中断
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET) {
		__HAL_UART_CLEAR_IDLEFLAG(huart);  // 清楚空闲中断标志
		HAL_UART_DMAStop(huart);           // 暂停本次DMA传输，进行数据处理
		uint8_t data_length = UART_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);  // 计算接收到的数据长度
		// 数据处理回调函数
		if (huart == unitree_port[0].uart.handle) {
			A1_Motor_RecvData_Process(board.roll_motor, unitree_port[0].uart.rx_buff, data_length);
			HAL_UART_Receive_DMA(huart, unitree_port[0].uart.rx_buff, UART_BUFFER_SIZE);  // 重启DMA接收
		} else if (huart == unitree_port[1].uart.handle) {
			GO_Motor_RecvData_Process(board.steer_motor, unitree_port[1].uart.rx_buff, data_length);
			HAL_UART_Receive_DMA(huart, unitree_port[1].uart.rx_buff, UART_BUFFER_SIZE);  // 重启DMA接收
		}
	}
}

/* CAN过滤器初始化 */
void CAN_Filter_Init() {
	CAN_FilterTypeDef fcan = { 0 };

	fcan.FilterBank = 0;
	fcan.FilterMode = CAN_FILTERMODE_IDMASK;
	fcan.FilterScale = CAN_FILTERSCALE_32BIT;
	fcan.FilterIdHigh = 0;
	fcan.FilterIdLow = 0;
	fcan.FilterMaskIdHigh = 0;
	fcan.FilterMaskIdLow = 0;
	fcan.FilterFIFOAssignment = CAN_RX_FIFO0;
	fcan.FilterActivation = ENABLE;
	fcan.SlaveStartFilterBank = 0;

	if (HAL_CAN_ConfigFilter(&hcan1, &fcan) != HAL_OK)
		Error_Handler();
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
		Error_Handler();
	if (HAL_CAN_Start(&hcan1) != HAL_OK)
		Error_Handler();

	if (HAL_CAN_ConfigFilter(&hcan2, &fcan) != HAL_OK)
		Error_Handler();
	if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
		Error_Handler();
	if (HAL_CAN_Start(&hcan2) != HAL_OK)
		Error_Handler();
}

/* CAN发送数据函数 */
void CAN_Send_Msg(uint8_t can_ind, uint8_t id, uint8_t *msg, uint8_t len) {
	CAN_port[can_ind].TxHeader.StdId = id;
	CAN_port[can_ind].TxHeader.ExtId = 0;
	CAN_port[can_ind].TxHeader.IDE = CAN_ID_STD;
	CAN_port[can_ind].TxHeader.RTR = CAN_RTR_DATA;
	CAN_port[can_ind].TxHeader.DLC = len;
	CAN_port[can_ind].TxHeader.TransmitGlobalTime = DISABLE;

	//	while(HAL_CAN_GetTxMailboxesFreeLevel(CAN_port[can_ind].handle) == 0) {}

	/*找到空的发送邮箱，把数据发送出去*/
	if (HAL_CAN_AddTxMessage(CAN_port[can_ind].handle, &CAN_port[can_ind].TxHeader, msg, (uint32_t*) CAN_TX_MAILBOX0)
	        != HAL_OK) {
		if (HAL_CAN_AddTxMessage(CAN_port[can_ind].handle, &CAN_port[can_ind].TxHeader, msg,
		        (uint32_t*) CAN_TX_MAILBOX1) != HAL_OK) {
			if (HAL_CAN_AddTxMessage(CAN_port[can_ind].handle, &CAN_port[can_ind].TxHeader, msg,
			        (uint32_t*) CAN_TX_MAILBOX2) != HAL_OK)
				Error_Handler();
		}
	}
}

/* CAN接收中断回调函数 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	uint8_t rxdata[8] = { 0 };
	if (hcan->Instance == CAN1) {
		HAL_CAN_GetRxMessage(CAN_port[0].handle, CAN_RX_FIFO0, &CAN_port[0].RxHeader, rxdata);
		RE35_Motor_RecvData_Process(board.cable_motor, CAN_port[0].RxHeader.StdId, rxdata);
	} else if (hcan->Instance == CAN2) {
		HAL_CAN_GetRxMessage(CAN_port[1].handle, CAN_RX_FIFO0, &CAN_port[1].RxHeader, rxdata);
		RE35_Motor_RecvData_Process(board.archor_motor, CAN_port[1].RxHeader.StdId, rxdata);
	}
}

/* 虚拟串口接收数据处理函数 */
void CDC_Process_Recv_Data(uint8_t *data, uint32_t Len) {
	if (data[0] == ARCHOR_RESET) {
		board.archor_motor->reset_flag = false;
	}
	temp_jog_cmd = JogCmd_fromCharArray(data, Len);
	osMessagePut(JogCmdQueueHandle, (uint32_t) &temp_jog_cmd, 0);
}

void Process_Cmd(JogCmd *cmd) {
	RE35_Motor_SetCmd(board.cable_motor, cmd->cable_cmd.motor_mode, (int32_t) cmd->cable_cmd.vel,
	        (int32_t) cmd->cable_cmd.pos); //  接收的速度：RPM，接收的位置：qc
	RE35_Motor_SetCmd(board.archor_motor, cmd->archor_cmd.motor_mode, (int32_t) cmd->archor_cmd.vel,
	        (int32_t) cmd->archor_cmd.pos);  //  接收的速度：RPM，接收的位置：qc
	GO_Motor_SetCmd(board.steer_motor, cmd->steer_cmd.motor_mode, 0, cmd->steer_cmd.vel, cmd->steer_cmd.pos); //  接收的速度：rad/s，接收的位置：rad
	A1_Motor_SetCmd(board.roll_motor, cmd->roll_cmd.motor_mode, 0, cmd->roll_cmd.vel, cmd->roll_cmd.pos); //  接收的速度：rad/s，接收的位置：rad

	RE35_Motor_Send(board.cable_motor);
	RE35_Motor_Send(board.archor_motor);
	GO_Motor_Send(board.steer_motor);
	A1_Motor_Send(board.roll_motor);
}

