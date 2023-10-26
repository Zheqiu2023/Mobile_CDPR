/**
 * @File Name: sim_control_node.cpp
 * @brief
 * @author Zhe Qiu (zheqiu2021@163.com)
 * @version 0.1
 * @date 2023-10-18
 *
 * *  ***********************************************************************************
 * *  @copyright Copyright (c) 2023  by Zhe Qiu. All rights reserved.
 * *  Use of this source code is governed by the BSD 3-Clause license, see LICENSE.
 * *  ***********************************************************************************
 */
#include "sim_control.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cdpr_gazebo_node");

    sim_control::SimControl sim_ctrl;
    sim_ctrl.moveAround();

    return 0;
}