/**
 * @File Name: motor_57_controller.cpp
 * @brief
 * @author Zhe Qiu (zheqiu2021@163.com)
 * @version 0.1
 * @date 2023-05-26
 *
 *  ***********************************************************************************
 *  BSD 3-Clause License
 *
 *  @copyright Copyright (c) 2023  by Zhe Qiu. All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  1.Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  2.Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  3.Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 *  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
