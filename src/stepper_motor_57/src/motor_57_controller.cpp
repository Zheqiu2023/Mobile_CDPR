/**
 * @File Name: motor_57_controller.cpp
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

// 开发库中的样例只是提供一个简单的调用so库的方法供参考，程序接收与发送函数设置在两个线程中，并且线程没有同步。
// 现实中客户编程中，发送与接收函数不能同时调用（不支持多线程），如果在多线程中，一定需要互锁。需要客户自行完善代码。
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <ros/ros.h>
#include <ros/assert.h>
#include <std_msgs/Bool.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <cstdlib>
#include <ctime>

#include "stepper_motor_57.hpp"
#include "unistd.h"
#include "usb_can.hpp"

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "stepper_motor_57");
    // ros::NodeHandle nh;
    // ros::Publisher pub = nh.advertise<std_msgs::Bool>("can_off", 10);
    // std_msgs::Bool can_off_flag;  // CAN关闭标志
    // can_off_flag.data = true;

    can_init::CanInit can_handler;
    // 打开设备：注意一个设备只能打开一次
    if (VCI_OpenDevice(VCI_USBCAN2, DEV_IND, 0) != 1)
    {
        ROS_ERROR_STREAM("Failed to open USBCAN!");
        exit(EXIT_FAILURE);
    }
    can_handler.initCAN(VCI_USBCAN2, DEV_IND, CAN_IND0, MotorType::STEPPER_MOTOR);  // 打开CAN通道1

    motor_57::Motor57Run m57_run;

    // for (auto i = 0; i < 10; ++i)
    // {
    //     pub.publish(can_off_flag);
    //     usleep(10000);
    // }
    VCI_CloseDevice(VCI_USBCAN2, DEV_IND);  // 关闭设备
    ROS_INFO_STREAM("Close USBCAN Successfully!");

    return 0;
}
