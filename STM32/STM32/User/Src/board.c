/*
 * board.c
 *
 *  Created on: May 23, 2024
 *      Author: 23877
 */

#include "board.h"

Board board;

void Board_Init() {
	// A1接串口1，波特率4.8M，使用转接板时485的GND和B口线序需调换
//	 GO接串口6，波特率4M，使用转接板时485的GND和B口线序需调换
//	RE35_Config config3 = { 0, 1, 1, 0 };
//	board.cable_motor = RE35_Motor_Create(config3);
//	RE35_Config config4 = { 1, 5, 1, 5 };
//	board.archor_motor = RE35_Motor_Create(config4);
//	GO_Config config2 = { 1, 0, -1 };
//	board.steer_motor = GO_Motor_Create(config2);
//	A1_Config config1 = { 0, 0, -1 };
//	board.roll_motor = A1_Motor_Create(config1);

//	RE35_Config config3 = { 0, 2, -1, 0 };
//	board.cable_motor = RE35_Motor_Create(config3);
//	RE35_Config config4 = { 1, 6, 1, 10 };
//	board.archor_motor = RE35_Motor_Create(config4);
//	GO_Config config2 = { 1, 0, -1 };
//	board.steer_motor = GO_Motor_Create(config2);
//	A1_Config config1 = { 0, 0, 1 };
//	board.roll_motor = A1_Motor_Create(config1);

//	RE35_Config config3 = { 0, 3, -1, 0 };
//	board.cable_motor = RE35_Motor_Create(config3);
//	RE35_Config config4 = { 1, 7, 1, 5 };
//	board.archor_motor = RE35_Motor_Create(config4);
//	GO_Config config2 = { 1, 0, -1 };
//	board.steer_motor = GO_Motor_Create(config2);
//	A1_Config config1 = { 0, 0, -1 };
//	board.roll_motor = A1_Motor_Create(config1);

	RE35_Config config3 = { 0, 4, 1, 0 };
	board.cable_motor = RE35_Motor_Create(config3);
	RE35_Config config4 = { 1, 8, 1, 5 };
	board.archor_motor = RE35_Motor_Create(config4);
	GO_Config config2 = { 1, 0, -1 };
	board.steer_motor = GO_Motor_Create(config2);
	A1_Config config1 = { 0, 0, 1 };
	board.roll_motor = A1_Motor_Create(config1);

	RE35_Motor_Init(board.cable_motor, VEL_POS);
	RE35_Motor_Init(board.archor_motor, VEL_POS);
	GO_Motor_SetCmd(board.steer_motor, GO_MODE_W, 0, 0, 0);	//获取零位
	GO_Motor_Send(board.steer_motor);
	A1_Motor_SetCmd(board.roll_motor, A1_MODE_W, 0, 0, 0);	//获取零位
	A1_Motor_Send(board.roll_motor);
}

