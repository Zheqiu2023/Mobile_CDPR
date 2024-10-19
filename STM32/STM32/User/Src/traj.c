/*
 * traj.c
 *
 *  Created on: May 30, 2024
 *      Author: 23877
 */

#include "traj.h"
#include "maxon_re35.h"
#include "board.h"

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"

void Start_Local_Traj(float traj[][2], uint16_t rows, float period) {
	float target_pos[2];
	int32_t cmd_pos[2], cmd_vel[2], cmd_pos_prev[2];

	const TickType_t xFrequency = pdMS_TO_TICKS((uint32_t )1000 * period); // 周期period秒
	TickType_t xLastWakeTime = xTaskGetTickCount(); // 初始化上次唤醒时刻

	for (size_t i = 0; i < rows; ++i) {
		for (size_t j = 0; j < 2; ++j) {
			target_pos[j] = traj[i][j];
			cmd_pos_prev[j] = cmd_pos[j];
			cmd_pos[j] = Cable_Convert_Pos(target_pos[j]);
			cmd_vel[j] = (int32_t) ceil(fabs((cmd_pos[j] - cmd_pos_prev[j]) * 60 / period));
		}
		RE35_Motor_SetCmd(board.cable_motor, VEL_POS, cmd_vel[0], cmd_pos[0]);
		RE35_Motor_SetCmd(board.archor_motor, VEL_POS, cmd_vel[1], cmd_pos[1]);
		RE35_Motor_Send(board.cable_motor);
		RE35_Motor_Send(board.archor_motor);
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}

void Start_Global_Traj(float traj[][4], uint16_t rows, float period) {
	float target_pos[2];
	int32_t cmd_pos[2], cmd_vel[2], cmd_pos_prev[2];

	const TickType_t xFrequency = pdMS_TO_TICKS((uint32_t )1000 * period); // 周期period秒
	TickType_t xLastWakeTime = xTaskGetTickCount(); // 初始化上次唤醒时刻

	for (size_t i = 0; i < rows; ++i) {
		for (size_t j = 0; j < 2; ++j) {
			target_pos[j] = traj[i][j];
			cmd_pos_prev[j] = cmd_pos[j];
			cmd_pos[j] = Cable_Convert_Pos(target_pos[j]);
			cmd_vel[j] = (int32_t) ceil(fabs((cmd_pos[j] - cmd_pos_prev[j]) * 60 / period));
		}
		RE35_Motor_SetCmd(board.cable_motor, VEL_POS, cmd_vel[0], cmd_pos[0]);
		RE35_Motor_SetCmd(board.archor_motor, VEL_POS, cmd_vel[1], cmd_pos[1]);
		GO_Motor_SetCmd(board.steer_motor, GO_MODE_POS, 0, 0, traj[i][2]);
		A1_Motor_SetCmd(board.roll_motor, A1_MODE_W, 0, A1_Convert_Vel(traj[i][3]), 0);
		RE35_Motor_Send(board.cable_motor);
		RE35_Motor_Send(board.archor_motor);
		GO_Motor_Send(board.steer_motor);
		A1_Motor_Send(board.roll_motor);
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}

// local traj
float updown_traj[][2] = {};
float line_traj[][2] = {};
float circle_traj[][2] = {};

// global traj
float no_obs_traj[][4] = {};
float obs_traj[][4] = {};
