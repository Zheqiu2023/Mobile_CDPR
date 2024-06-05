/*
 * board.h
 *
 *  Created on: May 23, 2024
 *      Author: 23877
 */

#ifndef BOARD_H_
#define BOARD_H_

#include "bsp.h"
#include "unitree_a1.h"
#include "unitree_go.h"
#include "maxon_re35.h"

typedef struct {
	A1_Motor *roll_motor;
	GO_Motor *steer_motor;
	RE35_Motor *cable_motor;
	RE35_Motor *archor_motor;
} Board;

extern Board board;

void Board_Init();

#endif /* BOARD_H_ */
