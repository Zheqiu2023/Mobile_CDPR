/**
 * @File Name: motor_re35.cpp
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
#include "motor_re35/motor_re35.hpp"
#include "usb_can/usb_can.hpp"
#include "cdpr_bringup/filters/filters.hpp"

#include <pthread.h>
#include <vector>
#include <unistd.h>

using namespace motor_re35;

/**
 * @brief Construct a new MotorRun::MotorRun object
 */
MotorRun::MotorRun(ros::NodeHandle& nh)
{
    pub_ = nh_.advertise<cdpr_bringup::CanCmd>("/usbcan/motor_re35", 10);
    sub_ =
        nh_.subscribe<cdpr_bringup::CanFrame>("/usbcan/can_pub", 100, boost::bind(&MotorRun::recvCallback, this, _1));
    ros::Duration(1.0).sleep();  // 休眠1s，保证发出的第一条消息能被usbcan接收
}

/**
 * @brief 订阅CAN话题回调函数
 * @param  msg
 */
void MotorRun::recvCallback(const cdpr_bringup::CanFrame::ConstPtr& msg)
{
    recv_msg_ = std::move(*msg);
}

void MotorRun::publishCmd(const cdpr_bringup::CanCmd& cmd_struct)
{
    pub_.publish(cmd_struct);
}

/**
 * @brief 初始化电机
 */
void MotorRun::init()
{
    pub_cmd_.cmd.SendType = 1;  // 单次发送（只发送一次，发送失败不会自动重发，总线只产生一帧数据）
    pub_cmd_.cmd.RemoteFlag = 0;  // 0为数据帧，1为远程帧
    pub_cmd_.cmd.ExternFlag = 0;  // 0为标准帧，1为拓展帧
    pub_cmd_.cmd.DataLen = 8;     // 数据长度8字节

    // 发送复位指令
    pub_cmd_.cmd.ID = 0x000;  // 广播复位指令（帧ID，由驱动器编号和功能序号决定）
    for (auto& data : pub_cmd_.cmd.Data)
        data = 0x55;
    publishCmd(pub_cmd_);
    usleep(500000);
    // 发送配置指令
    pub_cmd_.cmd.ID = 0x00A;  // 广播配置指令
    for (auto& data : pub_cmd_.cmd.Data)
        data = 0x55;
    pub_cmd_.cmd.Data[0] = 0xc8;  // 以 200 毫秒为周期对外发送电流、速度、位置等信息
    pub_cmd_.cmd.Data[1] = 0x00;
    publishCmd(pub_cmd_);
    usleep(500000);
    // 发送模式选择指令
    int run_mode = 0;
    nh_.getParam("run_mode", run_mode);

    pub_cmd_.cmd.ID = 0x001;  // 广播模式选择指令
    for (auto& data : pub_cmd_.cmd.Data)
        data = 0x55;
    if (run_mode == 0)
    {
        pub_cmd_.cmd.Data[0] = 0x03;  // 选择速度模式
        publishCmd(pub_cmd_);
    }
    else if (run_mode == 1)
    {
        pub_cmd_.cmd.Data[0] = 0x05;  // 选择速度位置模式
        publishCmd(pub_cmd_);
    }
    usleep(500000);
}

/**
 * @brief 按指定模式运行电机
 */
void MotorRun::run()
{
    int run_mode = 0;
    nh_.getParam("run_mode", run_mode);

    if (run_mode == 0)
    {
        // 速度模式：力控
        short temp_vel = 0;

        pub_cmd_.cmd.ID = 0x014;  // 速度模式下的参数指令，广播
        pub_cmd_.cmd.Data[0] = static_cast<unsigned char>((PWM_LIM >> 8) & 0xff);
        pub_cmd_.cmd.Data[1] = static_cast<unsigned char>(PWM_LIM & 0xff);
        pub_cmd_.cmd.Data[2] = static_cast<unsigned char>((temp_vel >> 8) & 0xff);
        pub_cmd_.cmd.Data[3] = static_cast<unsigned char>(temp_vel & 0xff);
        for (size_t i = 4; i < 8; ++i)
        {
            pub_cmd_.cmd.Data[i] = 0x55;
        }

        publishCmd(pub_cmd_);
    }
    else if (run_mode == 1)
    {
        // 速度位置模式
        std::vector<int> target_pos_vec{};
        int temp_pos = 0, history_pos = 0, reduction_ratio = 0, encoder_lines_num = 0;
        unsigned short temp_vel = 10000;  // 速度限制值(RPM)：0~32767

        nh_.getParam("target_data/target_pos_vec", target_pos_vec);
        nh_.getParam("reduction_ratio", reduction_ratio);
        nh_.getParam("encoder_lines_num", encoder_lines_num);

        pub_cmd_.cmd.ID = 0x016;  // 速度位置模式下的参数指令，非广播
        pub_cmd_.cmd.Data[0] = static_cast<unsigned char>((PWM_LIM >> 8) & 0xff);
        pub_cmd_.cmd.Data[1] = static_cast<unsigned char>(PWM_LIM & 0xff);
        pub_cmd_.cmd.Data[2] = static_cast<unsigned char>((temp_vel >> 8) & 0xff);
        pub_cmd_.cmd.Data[3] = static_cast<unsigned char>(temp_vel & 0xff);
        for (auto& temp_pos : target_pos_vec)
        {
            history_pos += temp_pos;
            temp_pos = history_pos * reduction_ratio * encoder_lines_num / 360;  // °转换为qc
            pub_cmd_.cmd.Data[4] = static_cast<unsigned char>((temp_pos >> 24) & 0xff);
            pub_cmd_.cmd.Data[5] = static_cast<unsigned char>((temp_pos >> 16) & 0xff);
            pub_cmd_.cmd.Data[6] = static_cast<unsigned char>((temp_pos >> 8) & 0xff);
            pub_cmd_.cmd.Data[7] = static_cast<unsigned char>(temp_pos & 0xff);

            publishCmd(pub_cmd_);
            sleep(1);
        }
    }
    else
    {
        ROS_ERROR("Motor RE35 run mode error!");
    }
}