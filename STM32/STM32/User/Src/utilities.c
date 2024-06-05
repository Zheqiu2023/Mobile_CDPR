/*
 * common.c
 *
 *  Created on: May 21, 2024
 *      Author: 23877
 */

#include <math.h>
#include <utilities.h>

float deg2rad(float degree) {
	return degree * (M_PI / 180.0);
}

float rad2deg(float radian) {
	return radian * (180.0 / M_PI);
}

