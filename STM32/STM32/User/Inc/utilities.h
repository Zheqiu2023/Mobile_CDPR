/*
 * common.h
 *
 *  Created on: May 21, 2024
 *      Author: 23877
 */

#ifndef UTILITIES_H_
#define UTILITIES_H_

#include <stdio.h>
#include <stdbool.h>

#include "CDC_msg.h"
#include "cmsis_os.h"

float deg2rad(float degree);
float rad2deg(float radian);

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

/**
 * 定义环形缓冲区，用于存储电机反馈数据
 */
#define RING_BUFFER_SIZE 256

typedef struct {
	float data[RING_BUFFER_SIZE][2]; // 存储电机反馈数据
	uint8_t id[RING_BUFFER_SIZE];     // 存储电机ID
	MotorType motor_type[RING_BUFFER_SIZE];	// 电机类型
	uint8_t head;
	uint8_t tail;
	bool full;
} RingBuffer;

extern osMutexId bufferMutexHandle;

void Buffer_Put(RingBuffer *buffer, MotorType type, uint8_t id, float pos, float vel);
bool Buffer_Get(RingBuffer *buffer, MotorType *type, uint8_t *id, float *pos, float *vel);

#endif /* UTILITIES_H_ */
