/**
 * @File Name: tension_sensors_node.cpp
 * @brief
 * @author Zhe Qiu (zheqiu2021@163.com)
 * @version 0.1
 * @date 2023-06-20
 *
 * *  ***********************************************************************************
 * *  @copyright Copyright (c) 2023  by Zhe Qiu. All rights reserved.
 * *  Use of this source code is governed by the BSD 3-Clause license, see LICENSE.
 * *  ***********************************************************************************
 */
#include "tension_sensors.hpp"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "tension_sensors_node");

    tension_sensors::TensionSensors tension_sensors;
    tension_sensors.start_read();

    return 0;
}