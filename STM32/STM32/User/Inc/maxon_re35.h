/*
 * maxon_re35.h
 *
 *  Created on: May 23, 2024
 *      Author: 23877
 */

#ifndef MAXON_RE35_H_
#define MAXON_RE35_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
	ZERO_RESET = 0,		// 复位
	VEL = 0X03,     // 速度模式
	VEL_POS = 0X05  // 速度位置模式
} RE35_Run_Mode;

typedef struct {
	uint8_t can_ind;
	uint8_t driver_id;
	int8_t dir; // 旋转方向，顺时针为负，逆时针为正
	uint8_t period;
} RE35_Config;

typedef struct {
	uint8_t cmd_id;
	RE35_Run_Mode mode;
	uint8_t tx_data[8];		// 发送的数据
} RE35_Cmd;

typedef struct {
	int16_t current;	// 当前电流（mA）
	int16_t speed;      // 当前角速度（rpm）
	int32_t angle;      // 当前角度值（qc）
} RE35_Data;

typedef struct {
	RE35_Config config;
	RE35_Cmd cmd;
	RE35_Data data;

	bool reset_flag;
} RE35_Motor;

RE35_Motor* RE35_Motor_Create(RE35_Config config);
void RE35_Motor_Init(RE35_Motor *obj, RE35_Run_Mode mode);
void RE35_Motor_SetCmd(RE35_Motor *obj, RE35_Run_Mode mode, int32_t speed, int32_t angle);
void RE35_Motor_RecvData_Process(RE35_Motor *obj, uint32_t recv_id, uint8_t *data);
void RE35_Motor_Send(RE35_Motor *obj);

int32_t Cable_Convert_Pos(float pos);
int32_t Archor_Convert_Pos(float pos);

#endif /* MAXON_RE35_H_ */
