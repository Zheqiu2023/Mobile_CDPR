/*
 * traj.h
 *
 *  Created on: May 30, 2024
 *      Author: 23877
 */

#ifndef TRAJ_H_
#define TRAJ_H_

#include <stdint.h>

extern float updown_traj[][2];
extern float line_traj[][2];
extern float circle_traj[][2];

extern float no_obs_traj[][4];
extern float obs_traj[][4];

void Start_Local_Traj(float traj[][2], uint16_t rows, float period);
void Start_Global_Traj(float traj[][4], uint16_t rows, float period);

#endif /* TRAJ_H_ */
