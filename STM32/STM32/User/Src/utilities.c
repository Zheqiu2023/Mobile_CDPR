/*
 * common.c
 *
 *  Created on: May 21, 2024
 *      Author: 23877
 */

#include <math.h>

#include "utilities.h"
#include "usart.h"
#include "cmsis_os.h"

float deg2rad(float degree) {
	return degree * (M_PI / 180.0);
}

float rad2deg(float radian) {
	return radian * (180.0 / M_PI);
}

PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart5, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
	return ch;
}


RingBuffer motor_fb_buffer = {{{0}}, {0}, {0}, 0, 0, 0};

void Buffer_Put(RingBuffer *buffer, MotorType type, uint8_t id, float pos, float vel) {
//	osMutexWait(bufferMutexHandle, 0);	// 加锁后代码会卡死在RE35_Motor_RecvData_Process函数中

	buffer->motor_type[buffer->head] = type;
	buffer->id[buffer->head] = id;
    buffer->data[buffer->head][0] = pos;
	buffer->data[buffer->head][1] = vel;
    buffer->head = (buffer->head + 1) % RING_BUFFER_SIZE;

    if (buffer->head == buffer->tail)
        buffer->full = true;

//    osMutexRelease(bufferMutexHandle);
}

bool Buffer_Get(RingBuffer *buffer, MotorType *type, uint8_t *id, float *pos, float *vel) {
//	osMutexWait(bufferMutexHandle, osWaitForever);

	if (buffer->head == buffer->tail && !buffer->full)
		return false;	// 缓冲区为空

	*type = buffer->motor_type[buffer->tail];
	*id = buffer->id[buffer->tail];
	*pos = buffer->data[buffer->tail][0];
	*vel = buffer->data[buffer->tail][1];

    buffer->tail = (buffer->tail + 1) % RING_BUFFER_SIZE;
    buffer->full = false;

//    osMutexRelease(bufferMutexHandle);

    return true;
}


