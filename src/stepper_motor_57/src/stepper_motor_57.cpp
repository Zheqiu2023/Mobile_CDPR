/**
 * @File Name: stepper_motor_57.cpp
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

#include "stepper_motor_57/stepper_motor_57.hpp"
#include "usb_can/usb_can.hpp"

#include <pthread.h>
#include <stdio.h>

#include <array>
#include <string>
#include <vector>
using namespace motor_57;

/**
 * @brief Construct a new MotorRun::MotorRun object
 */
MotorRun::MotorRun(ros::NodeHandle& nh) : nh_(nh)
{
    name_space_ = nh_.getNamespace();
    pub_ = nh_.advertise<cdpr_bringup::CanFrame>("/usbcan/motor_57", 100);
    sub_ =
        nh_.subscribe<cdpr_bringup::CanFrame>("/usbcan/can_pub", 100, boost::bind(&MotorRun::recvCallback, this, _1));

    ros::Duration(0.4).sleep();  // 休眠0.4s，保证发出的第一条消息能被usbcan接收
}

void MotorRun::recvCallback(const cdpr_bringup::CanFrame::ConstPtr& msg)
{
    if (msg->Data[2] == 0x41 && msg->Data[7] == 0)
        is_reset_ = true;
}

void MotorRun::publishCmd(const cdpr_bringup::CanFrame& cmd)
{
    pub_.publish(cmd);
}

/**
 * @brief 根据电机运行模式设置相应控制指令
 * @param  cmd_mode
 */
void MotorRun::setCmd(StepperMotorRunMode cmd_mode, std::vector<int> data_vec)
{
    pub_cmd_.ID = 0xc1;  // 帧ID，与驱动器地址相同
    pub_cmd_.SendType = 1;  // 单次发送（只发送一次，发送失败不会自动重发，总线只产生一帧数据）
    pub_cmd_.RemoteFlag = 0;                                          // 0为数据帧，1为远程帧
    pub_cmd_.ExternFlag = 0;                                          // 0为标准帧，1为拓展帧
    pub_cmd_.DataLen = 8;                                             // 数据长度8字节
    pub_cmd_.Data[0] = static_cast<unsigned char>(pub_cmd_.ID >> 3);  // CAN 驱动器地址0x0c1(11bits)的高 8bits
    pub_cmd_.Data[1] = static_cast<unsigned char>(pub_cmd_.ID << 5);  // CAN 驱动器地址的低 3bits，后五位一般设置为 0
    switch (cmd_mode)
    {
        case StepperMotorRunMode::RESET:
            pub_cmd_.Data[2] = static_cast<unsigned char>((CMD_REQUEST << 5) | 0X01);  // 复位命令(需要返回)
            pub_cmd_.Data[7] = 1;                                                      // 复位完毕，返回命令
            break;
        case StepperMotorRunMode::POS:
            pub_cmd_.Data[2] = static_cast<unsigned char>((CMD_REQUEST << 5) | 0X02);  // 定位命令(需要返回)
            pub_cmd_.Data[7] = 1;                                                      // 到位后，返回命令
            break;
        case StepperMotorRunMode::VEL_FORWARD:
            pub_cmd_.Data[2] = static_cast<unsigned char>((CMD_REQUEST << 5) | 0X03);  // 正转命令(需要返回)
            pub_cmd_.Data[7] = 3;  // 正方向转 IntDate 步后命令返回
            break;
        case StepperMotorRunMode::VEL_REVERSE:
            pub_cmd_.Data[2] = static_cast<unsigned char>((CMD_REQUEST << 5) | 0X04);  // 反转命令(需要返回)
            pub_cmd_.Data[7] = 3;  // 反方向转 IntDate 步后命令返回
            break;
        case StepperMotorRunMode::STALL:
            pub_cmd_.Data[2] = static_cast<unsigned char>((CMD_REQUEST << 5) | 0X05);  // 停转命令(需要返回)
            pub_cmd_.Data[7] = 1;                                                      // 减速停止，返回命令
            break;
        default:
            ROS_WARN_NAMED(name_space_, "Motor operating mode error!");
            break;
    }
    for (size_t i = 3; i < 7; ++i)
    {
        pub_cmd_.Data[i] = static_cast<unsigned char>(data_vec[i - 3]);
    }
}

void MotorRun::run()
{
    // writeParam();
    // readParam();

    std::string is_stall{}, cmd_type{};
    std::vector<int> data_vec(4, 0);
    nh_.getParam("motor_cmd_type", cmd_type);

    if (cmd_type == "position")
    {
        ROS_INFO_NAMED(name_space_, "Reset before localization!");
        // Reset: find zero position
        while (ros::ok())
        {
            setCmd(StepperMotorRunMode::RESET, data_vec);
            publishCmd(pub_cmd_);
            ros::spinOnce();
            if (is_reset_)
                break;
        }
        ROS_INFO_NAMED(name_space_, "Reset done!");
        // Run to the specified position
        nh_.getParam("motor_ctrl_data/goal_pos_arr", data_vec);
        setCmd(StepperMotorRunMode::POS, data_vec);
        publishCmd(pub_cmd_);
    }
    else if (cmd_type == "vel_forward")
    {
        nh_.getParam("motor_ctrl_data/goal_vel_forward_arr", data_vec);
        ROS_INFO_NAMED(name_space_, "Forward rotation!");

        setCmd(StepperMotorRunMode::VEL_FORWARD, data_vec);
        publishCmd(pub_cmd_);
        while (ros::ok())
        {
            getline(std::cin, is_stall);
            if (is_stall == "p")
                break;
        }
        setCmd(StepperMotorRunMode::STALL, data_vec);
        publishCmd(pub_cmd_);
    }
    else if (cmd_type == "vel_backward")
    {
        nh_.getParam("motor_ctrl_data/goal_vel_reverse_arr", data_vec);
        ROS_INFO_NAMED(name_space_, "Reverse rotation!");

        setCmd(StepperMotorRunMode::VEL_REVERSE, data_vec);
        publishCmd(pub_cmd_);
        while (ros::ok())
        {
            getline(std::cin, is_stall);
            if (is_stall == "p")
                break;
        }
        setCmd(StepperMotorRunMode::STALL, data_vec);
        publishCmd(pub_cmd_);
    }
    else
    {
        ROS_WARN_NAMED(name_space_, "Motor control type error!");
    }
}

/**
 * @brief 发送运行参数,保存到flash后掉电数据不会丢失，因此只需设置一次
 */
void MotorRun::writeParam()
{
    ros::Publisher pub = nh_.advertise<cdpr_bringup::CanFrame>("/usbcan/motor_57", 100);
    std::array<cdpr_bringup::CanFrame, 8> data_arr;  // data_arr[0]:启动周期   data_arr[1]:恒速周期 data_arr[2]:加速步数
                                                     // data_arr[3]:加速系数   data_arr[4]:细分    data_arr[5]：工作模式
                                                     // data_arr[6]：相电流    data_arr[7]：CAN ID
    for (auto& data : data_arr)
    {
        data.ID = 0xc1;  // 帧ID，与驱动器地址相同
        data.SendType = 1;  // 单次发送（只发送一次，发送失败不会自动重发，总线只产生一帧数据）
        data.RemoteFlag = 0;                                      // 0为数据帧，1为远程帧
        data.ExternFlag = 0;                                      // 0为标准帧，1为拓展帧
        data.DataLen = 8;                                         // 数据长度8字节
        data.Data[0] = static_cast<unsigned char>(data.ID >> 3);  // CAN 驱动器地址(11bits)的高 8bits
        data.Data[1] = static_cast<unsigned char>(data.ID << 5);  // CAN 驱动器地址的低 3bits，后五位一般设置为 0
        data.Data[2] = static_cast<unsigned char>((CMD_REQUEST << 5) | 0X13);  // 运行参数保存到内存
    }
    // 小端模式：低地址存放低位
    std::vector<std::vector<int>> data_vec(8, std::vector<int>{ 0 });
    nh_.getParam("operating_param/plus_start_time", data_vec[0]);
    nh_.getParam("operating_param/plus_constant_time", data_vec[1]);
    nh_.getParam("operating_param/acc_steps", data_vec[2]);
    nh_.getParam("operating_param/acc_cof", data_vec[3]);
    nh_.getParam("operating_param/sub_divide", data_vec[4]);
    nh_.getParam("operating_param/reset_mode", data_vec[5]);
    nh_.getParam("operating_param/phase_current", data_vec[6]);
    nh_.getParam("operating_param/can_id", data_vec[7]);
    for (size_t i = 0; i < data_vec.size(); ++i)
    {
        for (size_t j = 0; j < 5; ++j)
        {
            data_arr[i].Data[j + 3] = static_cast<unsigned char>(data_vec[i][j]);
        }
    }

    ROS_INFO_NAMED(name_space_, "Sending operating parameters!");
    for (auto& data : data_arr)
    {
        pub.publish(data);
        usleep(100000);
    }

    ROS_INFO_NAMED(name_space_, "********************************");
    data_arr[0].Data[2] = static_cast<unsigned char>((CMD_REQUEST << 5) | 0X14);  // 运行参数保存到flash，掉电数据不丢失
    pub.publish(data_arr[0]);
    usleep(100000);
    ROS_INFO_NAMED(name_space_, "Parameters have been saved in flash!");
}

/**
 * @brief 从flash中读取电机运行参数
 */
void MotorRun::readParam()
{
    ros::Publisher pub = nh_.advertise<cdpr_bringup::CanFrame>("/usbcan/motor_57", 100);
    std::array<cdpr_bringup::CanFrame, 8> data_arr;  // data_arr[0]:启动周期   data_arr[1]:恒速周期 data_arr[2]:加速步数
                                                     // data_arr[3]:加速系数   data_arr[4]:细分    data_arr[5]：工作模式
                                                     // data_arr[6]：相电流    data_arr[7]：CAN ID
    for (auto& data : data_arr)
    {
        data.ID = 0xc1;  // 帧ID，与驱动器地址相同
        data.SendType = 1;  // 单次发送（只发送一次，发送失败不会自动重发，总线只产生一帧数据）
        data.RemoteFlag = 0;                                      // 0为数据帧，1为远程帧
        data.ExternFlag = 0;                                      // 0为标准帧，1为拓展帧
        data.DataLen = 8;                                         // 数据长度8字节
        data.Data[0] = static_cast<unsigned char>(data.ID >> 3);  // CAN 驱动器地址(11bits)的高 8bits
        data.Data[1] = static_cast<unsigned char>(data.ID << 5);  // CAN 驱动器地址的低 3bits，后五位一般设置为 0
        data.Data[2] = static_cast<unsigned char>((CMD_REQUEST << 5) | 0X13);  // 运行参数读取
        for (size_t i = 3; i < data.DataLen; ++i)
        {
            data.Data[i] = 0;
        }
    }
    data_arr[0].Data[7] = 17;
    data_arr[1].Data[7] = 15;
    data_arr[2].Data[7] = 1;
    data_arr[3].Data[7] = 19;
    data_arr[4].Data[7] = 11;
    data_arr[5].Data[7] = 13;
    data_arr[6].Data[7] = 23;
    data_arr[7].Data[7] = 9;

    for (auto& data : data_arr)
    {
        pub.publish(data);
        usleep(100000);
    }
}
