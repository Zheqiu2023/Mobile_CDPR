/*
 * CDC_msg.h
 *
 *  Created on: May 30, 2024
 *      Author: 23877
 */

#ifndef CDC_MSG_H_
#define CDC_MSG_H_

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

/* 电机指令格式**************************************/
typedef enum {
	MAXON_RE35, UNITREE_A1, UNITREE_GO
} MotorType;

typedef enum {
	ARCHOR_RESET,	// 锚点座电机复位
	J,  // 点动指令
	T   // 轨迹指令
} CmdMode;

// 单片机虚拟串口最大接收字节数为64，因此需要字节对齐，防止超出限制
#pragma pack(1)
typedef struct {
	unsigned char motor_mode;
	float vel;
	float pos;
} MotorCmd;
#pragma pack()

/* 点动指令格式**************************************/
#pragma pack(1)
typedef struct {
	unsigned char cmd_mode;
	MotorCmd cable_cmd;
	MotorCmd archor_cmd;
	MotorCmd steer_cmd;
	MotorCmd roll_cmd;
} JogCmd;
#pragma pack()

/* 电机反馈数据**************************************/
#pragma pack(1)
typedef struct {
	uint8_t id;
	float vel;
	float pos;
} MotorFBData;

typedef struct {
	MotorFBData cable_fb_data;
	MotorFBData archor_fb_data;
	MotorFBData steer_fb_data;
	MotorFBData roll_fb_data;
} RobotFBData;
#pragma pack()

// 将结构体转换为char数组
void MotorCmd_toCharArray(const MotorCmd *msg, uint8_t *buffer, size_t size);
void JogCmd_toCharArray(const JogCmd *msg, uint8_t *buffer, size_t size);

// 从char数组转换回结构体
MotorCmd MotorCmd_fromCharArray(const uint8_t *data, size_t size);
JogCmd JogCmd_fromCharArray(const uint8_t *data, size_t size);

#endif /* CDC_MSG_H_ */
