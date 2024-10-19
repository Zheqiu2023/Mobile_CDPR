/*
 * unitree_go.h
 *
 *  Created on: May 21, 2024
 *      Author: 23877
 */

#ifndef UNITREE_GO_H_
#define UNITREE_GO_H_

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include "unitree_go_msg.h"

typedef enum {
	GO_MODE_STOP, 	  //电机停止
	GO_MODE_T,        //力矩模式
	GO_MODE_W,        //速度模式
	GO_MODE_POS,      //位置模式
	GO_MODE_HB        //混控模式
} GO_Ctrl_Mode;

typedef enum {
	BRAKE, 	  //电机停止
	FOC,	//foc
	CALIBRATE	//校准
} GO_Run_Mode;

typedef struct {
	uint8_t rs485_ind;
	uint8_t id;
	int8_t dir; //旋转方向，顺时针为负，逆时针为正
} GO_Config;

typedef struct {
	GO_Config config;
	GOData motor_data;        //收到的数据
	GOCmd motor_cmd;	//发送的数据

	float init_pos;            //零位（rad），上电时的位置
} GO_Motor;

GO_Motor* GO_Motor_Create(GO_Config config);
void GO_Motor_SetCmd(GO_Motor *obj, GO_Ctrl_Mode mode, float T, float W, float Pos);

void GO_Modify_Data(GOCmd *motor_s);
void GO_Get_Send_Data(GOCmd *motor_s, uint8_t *buf);
void GO_Motor_Send(GO_Motor *obj);

bool GO_Extract_Data(GOData *motor_r, uint8_t *data);
void GO_Get_Recv_Data(GOData *motor_r, uint8_t *buf);
void GO_Motor_RecvData_Process(GO_Motor *obj, uint8_t *data, uint8_t len);

uint16_t crc_ccitt(uint16_t crc, uint8_t const *buffer, size_t len);

#endif /* UNITREE_GO_H_ */
