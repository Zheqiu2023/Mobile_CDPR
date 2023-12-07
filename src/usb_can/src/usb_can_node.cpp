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
#include "usb_can/usb_can_node.hpp"

using namespace usb_can;

int main(int argc, char** argv) {
    ros::init(argc, argv, "usb_can_controller");
    ros::NodeHandle nh("~");

    // 打开设备：注意一个设备只能打开一次
    if (VCI_OpenDevice(VCI_USBCAN2, DEV_IND0, 0) != 1 || VCI_OpenDevice(VCI_USBCAN2, DEV_IND1, 0) != 1) {
        ROS_WARN("Failed to open at least one USBCAN!");
    }
    CanInit can_handler;
    can_handler.initCAN(VCI_USBCAN2, DEV_IND0, CAN_IND0, MotorType::STEPPER_MOTOR);  // open USBCAN0 CNA1
    can_handler.initCAN(VCI_USBCAN2, DEV_IND0, CAN_IND1, MotorType::MAXON_RE35);     // open USBCAN0 CNA2
    can_handler.initCAN(VCI_USBCAN2, DEV_IND1, CAN_IND0, MotorType::STEPPER_MOTOR);  // open USBCAN1 CNA1
    can_handler.initCAN(VCI_USBCAN2, DEV_IND1, CAN_IND1, MotorType::MAXON_RE35);     // open USBCAN1 CNA2

    // 开启2条并发线程处理订阅话题回调函数，保证及时接收到每条消息
    ros::AsyncSpinner spinner(2);
    spinner.start();

    TransferStation transfer_station(nh);

    ros::Rate loop_rate(500);
    while (ros::ok()) {
        transfer_station.publishMsg();
        loop_rate.sleep();
    }

    VCI_CloseDevice(VCI_USBCAN2, DEV_IND0);
    VCI_CloseDevice(VCI_USBCAN2, DEV_IND1);
    return 0;
}