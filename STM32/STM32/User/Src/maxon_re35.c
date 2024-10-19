/*
 * maxon_re35.c
 *
 *  Created on: May 23, 2024
 *      Author: 23877
 */

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "cmsis_os.h"
#include "maxon_re35.h"
#include "bsp.h"
#include "utilities.h"
#include "usbd_cdc_if.h"

#define RE35_REDUCTION_RATIO 35	// 减速比
#define ENCODER_LINES_NUM 2000	// 编码器线数
#define PWM_LIM 5000	// pwm限制值
#define LEAD 5	// 丝杠导程（mm）
#define REEL_D 30	// 绕线轮直径（mm）

RE35_Motor* RE35_Motor_Create(RE35_Config config) {
	RE35_Motor *obj = (RE35_Motor*) malloc(sizeof(RE35_Motor));
	if (obj == NULL) {
		// 如果内存分配失败，返回 NULL
		return NULL;
	}
	memset(obj, 0, sizeof(RE35_Motor));

	obj->config = config;

	return obj;
}

void RE35_Motor_Init(RE35_Motor *obj, RE35_Run_Mode mode) {
	obj->reset_flag = false;
	obj->cmd.mode = mode;
	// 发送复位指令
	obj->cmd.cmd_id = 0x000 | (obj->config.driver_id << 4);  // 复位指令（帧ID，由驱动器编号和功能序号决定）
	for (uint8_t i = 0; i < 8; ++i)
		obj->cmd.tx_data[i] = 0x55;
	RE35_Motor_Send(obj);
	HAL_Delay(500);
	// 发送模式选择指令
	obj->cmd.cmd_id = 0x001 | (obj->config.driver_id << 4);   // 模式选择指令
	obj->cmd.tx_data[0] = mode;  // 选择mode对应模式
	RE35_Motor_Send(obj);
	HAL_Delay(500);
	// 发送配置指令
	obj->cmd.cmd_id = 0x00A | (obj->config.driver_id << 4);  // 配置指令
	obj->cmd.tx_data[0] = 0x05;    // 以 1 毫秒为周期对外发送电流、速度、位置等信息
	obj->cmd.tx_data[1] = obj->config.period;  // 以 period 毫秒为周期对外发送CTL1/CTL2的电平状态
	RE35_Motor_Send(obj);
	HAL_Delay(500);
}

void RE35_Motor_SetCmd(RE35_Motor *obj, RE35_Run_Mode mode, int32_t speed, int32_t angle) {
	if (obj->cmd.mode != mode)
		RE35_Motor_Init(obj, mode);

	switch (mode) {
		case VEL:
			int32_t temp_speed = speed * obj->config.dir;
			obj->cmd.cmd_id = 0x004 | (obj->config.driver_id << 4);
			obj->cmd.tx_data[0] = (uint8_t) ((PWM_LIM >> 8) & 0xff);
			obj->cmd.tx_data[1] = (uint8_t) (PWM_LIM & 0xff);
			obj->cmd.tx_data[2] = (uint8_t) ((temp_speed >> 8) & 0xff);
			obj->cmd.tx_data[3] = (uint8_t) (temp_speed & 0xff);
			for (uint8_t i = 4; i < 8; ++i)
				obj->cmd.tx_data[i] = 0x55;
			break;
		case VEL_POS:
			int32_t temp_angle = angle * obj->config.dir;
			obj->cmd.cmd_id = 0x006 | (obj->config.driver_id << 4);
			obj->cmd.tx_data[0] = (uint8_t) ((PWM_LIM >> 8) & 0xff);
			obj->cmd.tx_data[1] = (uint8_t) (PWM_LIM & 0xff);
			obj->cmd.tx_data[2] = (uint8_t) ((speed >> 8) & 0xff);
			obj->cmd.tx_data[3] = (uint8_t) (speed & 0xff);
			obj->cmd.tx_data[4] = (uint8_t) ((temp_angle >> 24) & 0xff);
			obj->cmd.tx_data[5] = (uint8_t) ((temp_angle >> 16) & 0xff);
			obj->cmd.tx_data[6] = (uint8_t) ((temp_angle >> 8) & 0xff);
			obj->cmd.tx_data[7] = (uint8_t) (temp_angle & 0xff);
			break;
		default:
			break;
	}
}

void RE35_Motor_Send(RE35_Motor *obj) {
	CAN_Send_Msg(obj->config.can_ind, obj->cmd.cmd_id, obj->cmd.tx_data, sizeof(obj->cmd.tx_data));
}

void RE35_Motor_RecvData_Process(RE35_Motor *obj, uint32_t recv_id, uint8_t *data) {
	uint8_t temp_id = recv_id & 0x0f;

	if (temp_id == 0x0b) {
		// 接收到电流、速度、位置等信息
//		obj->data.current = (data[0] << 8) | data[1];
		obj->data.speed = (data[2] << 8) | data[3];
		obj->data.angle = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7];

		Buffer_Put(&motor_fb_buffer, MAXON_RE35, (recv_id-0x0b)>>4, obj->data.angle, obj->data.speed);
	} else if (temp_id == 0x0c && data[0] == 0x01) {
		// 接收到驱动器CTL1/CTL2的电平状态(针对锚点座电机)
		obj->reset_flag = true;
		RE35_Motor_SetCmd(obj, obj->cmd.mode, 0, 0);
		RE35_Motor_Send(obj);
	}
}

int32_t Cable_Convert_Pos(float pos) {
	return round(pos * 1000 * RE35_REDUCTION_RATIO * ENCODER_LINES_NUM / (M_PI * REEL_D));	// m转换为qc
}

int32_t Archor_Convert_Pos(float pos) {
	return round(pos * 1000 * RE35_REDUCTION_RATIO * ENCODER_LINES_NUM / LEAD);	// m转换为qc
}
