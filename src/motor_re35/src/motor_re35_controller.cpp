/**
 * @File Name: motor_re35_controller.cpp
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

#include <pthread.h>
#include <ros/ros.h>

#include "motor_re35.hpp"
#include "usb_can.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motor_re35");

    can_init::CanInit can_handler;
    // 打开设备：注意一个设备只能打开一次
    if (VCI_OpenDevice(VCI_USBCAN2, DEV_IND, 0) != 1)
    {
        ROS_ERROR_STREAM("Failed to open USBCAN!");
        exit(EXIT_FAILURE);
    }
    can_handler.initCAN(VCI_USBCAN2, DEV_IND, CAN_IND1, MotorType::MOTOR_RE35);  // 打开CAN通道2

    motor_re35::Re35Run re35_run;

    VCI_CloseDevice(VCI_USBCAN2, DEV_IND);  // 关闭设备
    ROS_INFO_STREAM("Close USBCAN Successfully!");

    return 0;
}