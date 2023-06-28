/**
 * @File Name: usb_can_node.cpp
 * @brief
 * @author Zhe Qiu (zheqiu2021@163.com)
 * @version 0.1
 * @date 2023-05-30
 *
 *  ***********************************************************************************
 *  @copyright Copyright (c) 2023  by Zhe Qiu. All rights reserved.
 *  Use of this source code is governed by the BSD 3-Clause license, see LICENSE.
 *  ***********************************************************************************
 */
#include "usb_can_node.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "usb_can_controller");

    // 打开设备：注意一个设备只能打开一次
    if (VCI_OpenDevice(VCI_USBCAN2, DEV_IND, 0) != 1)
    {
        ROS_ERROR_STREAM("Failed to open USBCAN!");
        exit(EXIT_FAILURE);
    }
    can_init::CanInit can_handler;
    can_handler.initCAN(VCI_USBCAN2, DEV_IND, CAN_IND0, MotorType::STEPPER_MOTOR);  // 打开CAN通道1
    can_handler.initCAN(VCI_USBCAN2, DEV_IND, CAN_IND1, MotorType::MOTOR_RE35);     // 打开CAN通道2

    usb_can::TransferStation transfer_station;

    ros::Rate loop_rate(10000);
    while (ros::ok())
    {
        transfer_station.publishMsgs();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}