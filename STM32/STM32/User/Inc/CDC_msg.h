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
typedef enum{
	MAXON_RE35,
	UNITREE_A1,
	UNITREE_GO
} MotorType;

typedef enum {
	M,  // 电机模式选择指令
	J,  // 点动指令
	T   // 轨迹指令
} CmdMode;

// 单片机虚拟串口最大接收字节数为64，因此需要字节对齐，防止超出限制
#pragma pack(1)
typedef struct {
	unsigned char motor_mode;
	float vel;
	float pos;
} MotorMsg;
#pragma pack()

enum NewCmd {
	CABLE, ARCHOR, STEER, ROLL
};

/* 点动指令格式**************************************/
#pragma pack(1)
typedef struct {
	unsigned char cmd_mode;
	unsigned char new_cmd;
	MotorMsg cable_cmd;
	MotorMsg archor_cmd;
	MotorMsg go_cmd;
	MotorMsg a1_cmd;
} JogMsg;
#pragma pack()

/* 轨迹指令格式**************************************/
enum LocalTrajType {
	EMPTY1, UPDOWN, LINE, CIRCLE
};

enum GlobalTrajType {
	EMPTY2, NO_OBS, OBS
};

#pragma pack(1)
typedef struct {
	unsigned char cmd_mode;
	bool start;
	unsigned char cdpr_traj;
	unsigned char chassis_traj;
	float period;
} TrajMsg;
#pragma pack()

/* 电机反馈数据**************************************/
#pragma pack(1)
typedef struct {
	MotorType motor_type;
	uint8_t id;
//	float current;
	float vel;
	float pos;
} MotorFBData;
#pragma pack()

// 将结构体转换为char数组
void MotorMsg_toCharArray(const MotorMsg *msg, uint8_t *buffer, size_t size);
void JogMsg_toCharArray(const JogMsg *msg, uint8_t *buffer, size_t size);
void TrajMsg_toCharArray(const TrajMsg *msg, uint8_t *buffer, size_t size);

// 从char数组转换回结构体
MotorMsg MotorMsg_fromCharArray(const uint8_t *data, size_t size);
JogMsg JogMsg_fromCharArray(const uint8_t *data, size_t size);
TrajMsg TrajMsg_fromCharArray(const uint8_t *data, size_t size);

#endif /* CDC_MSG_H_ */
