/**
 * @File Name: go_controller.cpp
 * @brief
 * @author Zhe Qiu (zheqiu2021@163.com)
 * @version 0.1
 * @date 2023-05-26
 *
 *  ***********************************************************************************
 *  @copyright Copyright (c) 2023  by Zhe Qiu. All rights reserved.
 *  Use of this source code is governed by the BSD 3-Clause license, see LICENSE.
 *  ***********************************************************************************
 */

#include "unitree_motor_go/go.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "unitree_motor_go");
    ros::NodeHandle nh("~");

    ros::AsyncSpinner spinner(2);
    spinner.start();

    motor_go::GoControl go_control(nh);
    go_control();

    return 0;
}