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
#include "motor_re35.hpp"
#include "usb_can.hpp"
#include "filter.hpp"

#include <pthread.h>
#include <vector>
#include <unistd.h>

using namespace motor_re35;

/**
 * @brief Construct a new MsgBox::MsgBox object
 */
MsgBox::MsgBox()
{
    tension_vec_.reserve(7);
    subs_.resize(2);
    pub_ = nh_.advertise<general_file::can_msgs>("/usbcan/motor_re35", 10);
    subs_[0] =
        nh_.subscribe<general_file::can_msgs>("/usbcan/can_pub", 100, boost::bind(&MsgBox::recvCANMsgs, this, _1));
    subs_[1] = nh_.subscribe<std_msgs::Float32>("/tension_val", 10, boost::bind(&MsgBox::recvTension, this, _1));
    ros::Duration(0.4).sleep();  // 休眠0.4s，保证发出的第一条消息能被usbcan接收
}

/**
 * @brief 订阅CAN话题回调函数
 * @param  msg
 */
void MsgBox::recvCANMsgs(const general_file::can_msgs::ConstPtr& msg)
{
    general_file::can_msgs recv_msgs = *msg;
}

/**
 * @brief 订阅张力传感器话题，获取张力数据
 * @param  tension
 */
void MsgBox::recvTension(const std_msgs::Float32::ConstPtr& tension)
{
    tension_vec_.push_back(tension->data);
    ++times_;
    if (times_ == 7)
    {
        force_ = filter::medianMeanFilter(tension_vec_);
        times_ = 0;
        tension_vec_.clear();
        ROS_INFO("Tension:%.3lf", force_);
    }
}

float MsgBox::getTension()
{
    return force_;
}

/**
 * @brief 发送函数
 */
void MsgBox::publishCmd(const general_file::can_msgs& cmd)
{
    pub_.publish(cmd);
}

MotorRun::MotorRun()
{
    float kp, ki, kd, integral_lim, frequency;
    ros::param::get("/re35/motor_ctrl_data/kp", kp);
    ros::param::get("/re35/motor_ctrl_data/ki", ki);
    ros::param::get("/re35/motor_ctrl_data/kd", kd);
    ros::param::get("/re35/motor_ctrl_data/integral_lim", integral_lim);
    ros::param::get("/re35/motor_ctrl_data/frequency", frequency);

    pid_.pidConfig(kp, ki, kd, integral_lim, frequency);
}

/**
 * @brief 初始化电机
 */
void MotorRun::init()
{
    pub_cmd_.SendType = 1;  // 单次发送（只发送一次，发送失败不会自动重发，总线只产生一帧数据）
    pub_cmd_.RemoteFlag = 0;  // 0为数据帧，1为远程帧
    pub_cmd_.ExternFlag = 0;  // 0为标准帧，1为拓展帧
    pub_cmd_.DataLen = 8;     // 数据长度8字节

    // 发送复位指令
    pub_cmd_.ID = 0x000;  // 广播复位指令（帧ID，由驱动器编号和功能序号决定）
    for (auto& data : pub_cmd_.Data)
        data = 0x55;
    msg_box_.publishCmd(pub_cmd_);
    usleep(500000);
    // 发送配置指令
    pub_cmd_.ID = 0x00A;  // 广播配置指令
    for (auto& data : pub_cmd_.Data)
        data = 0x55;
    pub_cmd_.Data[0] = 0xc8;  // 以 200 毫秒为周期对外发送电流、速度、位置等信息
    pub_cmd_.Data[1] = 0x00;
    msg_box_.publishCmd(pub_cmd_);
    usleep(500000);
    // 发送模式选择指令
    int run_mode = 0;
    ros::param::get("/re35/run_mode", run_mode);

    pub_cmd_.ID = 0x001;  // 广播模式选择指令
    for (auto& data : pub_cmd_.Data)
        data = 0x55;
    if (run_mode == 0)
    {
        pub_cmd_.Data[0] = 0x03;  // 选择速度模式
        msg_box_.publishCmd(pub_cmd_);
    }
    else if (run_mode == 1)
    {
        pub_cmd_.Data[0] = 0x05;  // 选择速度位置模式
        msg_box_.publishCmd(pub_cmd_);
    }
    usleep(500000);
}

/**
 * @brief 按指定模式运行电机
 */
void MotorRun::run()
{
    int run_mode = 0;
    ros::param::get("/re35/run_mode", run_mode);

    if (run_mode == 0)
    {
        // 速度模式：力控
        short temp_vel = 0;
        float tension = msg_box_.getTension();
        float goal_force;
        ros::param::get("/re35/motor_ctrl_data/goal_force", goal_force);

        if (tension != goal_force)
        {
            temp_vel = static_cast<short>(pid_.pidProcess(goal_force, tension));
            temp_vel = temp_vel > 32767 ? 32767 : temp_vel;
            temp_vel = temp_vel < -32768 ? -32768 : temp_vel;
        }

        pub_cmd_.ID = 0x014;  // 速度模式下的参数指令，广播
        pub_cmd_.Data[0] = static_cast<unsigned char>((PWM_LIM >> 8) & 0xff);
        pub_cmd_.Data[1] = static_cast<unsigned char>(PWM_LIM & 0xff);
        pub_cmd_.Data[2] = static_cast<unsigned char>((temp_vel >> 8) & 0xff);
        pub_cmd_.Data[3] = static_cast<unsigned char>(temp_vel & 0xff);
        for (size_t i = 4; i < 8; ++i)
        {
            pub_cmd_.Data[i] = 0x55;
        }

        msg_box_.publishCmd(pub_cmd_);
    }
    else if (run_mode == 1)
    {
        // 速度位置模式
        std::vector<int> goal_pos_vec{};
        int temp_pos = 0, history_pos = 0, reduction_ratio = 0, encoder_lines_num = 0;
        unsigned short temp_vel = 10000;  // 速度限制值(RPM)：0~32767

        ros::param::get("/re35/motor_ctrl_data/goal_pos_vec", goal_pos_vec);
        ros::param::get("/re35/reduction_ratio", reduction_ratio);
        ros::param::get("/re35/encoder_lines_num", encoder_lines_num);

        pub_cmd_.ID = 0x016;  // 速度位置模式下的参数指令，非广播
        pub_cmd_.Data[0] = static_cast<unsigned char>((PWM_LIM >> 8) & 0xff);
        pub_cmd_.Data[1] = static_cast<unsigned char>(PWM_LIM & 0xff);
        pub_cmd_.Data[2] = static_cast<unsigned char>((temp_vel >> 8) & 0xff);
        pub_cmd_.Data[3] = static_cast<unsigned char>(temp_vel & 0xff);
        for (auto& temp_pos : goal_pos_vec)
        {
            history_pos += temp_pos;
            temp_pos = history_pos * reduction_ratio * encoder_lines_num / 360;  // °转换为qc
            pub_cmd_.Data[4] = static_cast<unsigned char>((temp_pos >> 24) & 0xff);
            pub_cmd_.Data[5] = static_cast<unsigned char>((temp_pos >> 16) & 0xff);
            pub_cmd_.Data[6] = static_cast<unsigned char>((temp_pos >> 8) & 0xff);
            pub_cmd_.Data[7] = static_cast<unsigned char>(temp_pos & 0xff);

            msg_box_.publishCmd(pub_cmd_);
            sleep(1);
        }
    }
    else
    {
        ROS_ERROR("Motor RE35 run mode error!");
    }
}