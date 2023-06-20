/**
 * @File Name: stepper_motor_42.cpp
 * @brief
 * @author Zhe Qiu (zheqiu2021@163.com)
 * @version 0.1
 * @date 2023-05-29
 *
 *  ***********************************************************************************
 *  @copyright Copyright (c) 2023  by Zhe Qiu. All rights reserved.
 *  Use of this source code is governed by the BSD 3-Clause license, see LICENSE.
 *  ***********************************************************************************
 */
#include "stepper_motor_42.hpp"
#include "usb_can.hpp"

#include <pthread.h>
#include <stdio.h>

#include <array>
#include <string>
#include <vector>

using namespace motor_42;

/**
 * @brief 创建发送线程，属性设为可分离
 */
void MotorRun::creatThread()
{
    pthread_t thread_trans;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    int ret_trans = pthread_create(&thread_trans, &attr, transmitFunc, &msg_box_);
    ROS_ASSERT_MSG(ret_trans == 0, "Failed to create a sending thread!");
}

/**
 * @brief 发送线程函数
 * @param  arg
 * @return void*
 */
void* MotorRun::transmitFunc(void* arg)
{
    MsgBox* msg_box = (MsgBox*)arg;
    msg_box->run();

    // MotorParam m_param;
    // m_param.writeParam();
    // m_param.readParam();
    // usleep(100000);

    ROS_INFO_STREAM("Sending thread is closed!");
    pthread_exit(0);
}

/**
 * @brief Construct a new MsgBox::MsgBox object
 */
MsgBox::MsgBox()
{
    sem_init(&sem_trans_, 0, 0);  // 将信号量sem_trans设为线程间通信，初值为0
    pub_ = nh_.advertise<general_file::can_msgs>("/usbcan/motor_42", 100);
    sub_ = nh_.subscribe<general_file::can_msgs>("/usbcan/can_pub", 100, boost::bind(&MsgBox::recvCallback, this, _1));

    ros::Duration(0.4).sleep();  // 休眠0.4s，保证发出的第一条消息能被usbcan接收
}

MsgBox::~MsgBox()
{
    sem_destroy(&sem_trans_);
}

void MsgBox::recvCallback(const general_file::can_msgs::ConstPtr& msg)
{
    general_file::can_msgs temp_msgs_ = *msg;
    memcpy(&recv_msgs_, &temp_msgs_, sizeof(temp_msgs_));
    if (recv_msgs_.Data[2] == 0x41 && recv_msgs_.Data[7] == 0)
        sem_post(&sem_trans_);
}

/**
 * @brief 根据电机运行模式设置相应控制指令
 * @param  cmd_mode
 */
void MsgBox::setCmd(StepperMotorRunMode cmd_mode)
{
    std::vector<int> data_vec{ 0, 0, 0, 0 };

    send_cmd_.ID = 0xc1;  // 帧ID，与驱动器地址相同
    send_cmd_.SendType = 1;  // 单次发送（只发送一次，发送失败不会自动重发，总线只产生一帧数据）
    send_cmd_.RemoteFlag = 0;                                   // 0为数据帧，1为远程帧
    send_cmd_.ExternFlag = 0;                                   // 0为标准帧，1为拓展帧
    send_cmd_.DataLen = 8;                                      // 数据长度8字节
    send_cmd_.Data[0] = static_cast<unsigned char>(0xc1 >> 3);  // CAN 驱动器地址0x0c1(11bits)的高 8bits
    send_cmd_.Data[1] = static_cast<unsigned char>(0xc1 << 5);  // CAN 驱动器地址的低 3bits，后五位一般设置为 0
    switch (cmd_mode)
    {
        case StepperMotorRunMode::RESET:
            send_cmd_.Data[2] = static_cast<unsigned char>((CMD_REQUEST << 5) | 0X01);  // 复位命令(需要返回)
            send_cmd_.Data[7] = 1;                                                      // 复位完毕，返回命令
            break;
        case StepperMotorRunMode::POS:
            ros::param::get("/motor_42/motor_ctrl_data/goal_pos_arr", data_vec);
            send_cmd_.Data[2] = static_cast<unsigned char>((CMD_REQUEST << 5) | 0X02);  // 定位命令(需要返回)
            send_cmd_.Data[7] = 1;                                                      // 到位后，返回命令
            break;
        case StepperMotorRunMode::VEL_FORWARD:
            ros::param::get("/motor_42/motor_ctrl_data/goal_vel_forward_arr", data_vec);
            send_cmd_.Data[2] = static_cast<unsigned char>((CMD_REQUEST << 5) | 0X03);  // 正转命令(需要返回)
            send_cmd_.Data[7] = 3;  // 正方向转 IntDate 步后命令返回
            break;
        case StepperMotorRunMode::VEL_REVERSE:
            ros::param::get("/motor_42/motor_ctrl_data/goal_vel_reverse_arr", data_vec);
            send_cmd_.Data[2] = static_cast<unsigned char>((CMD_REQUEST << 5) | 0X04);  // 反转命令(需要返回)
            send_cmd_.Data[7] = 3;  // 反方向转 IntDate 步后命令返回
            break;
        case StepperMotorRunMode::STALL:
            send_cmd_.Data[2] = static_cast<unsigned char>((CMD_REQUEST << 5) | 0X05);  // 停转命令(需要返回)
            send_cmd_.Data[7] = 1;                                                      // 减速停止，返回命令
            break;
        default:
            ROS_WARN_STREAM("Motor operating mode error!");
            break;
    }
    for (size_t i = 3; i < 7; ++i)
    {
        send_cmd_.Data[i] = static_cast<unsigned char>(data_vec[i - 3]);
    }
}

/**
 * @brief 发送函数
 */
void MsgBox::publishCmd()
{
    memcpy(&pub_cmd_, &send_cmd_, sizeof(send_cmd_));
    pub_.publish(pub_cmd_);
}

void MsgBox::run()
{
    std::string is_stall{ "" }, cmd_type{ "" };
    ros::param::get("/motor_42/motor_cmd_type", cmd_type);

    if (cmd_type == "position")
    {
        ROS_INFO_STREAM("Reset before positioning!");
        // 先复位后定位
        setCmd(StepperMotorRunMode::RESET);
        publishCmd();
        sem_wait(&sem_trans_);
        ROS_INFO_STREAM("Reset done!");
        setCmd(StepperMotorRunMode::POS);
        publishCmd();
    }
    else if (cmd_type == "vel_forward")
    {
        ROS_INFO_STREAM("Forward rotation!");

        setCmd(StepperMotorRunMode::VEL_FORWARD);
        publishCmd();
        while (ros::ok())
        {
            getline(std::cin, is_stall);
            if (is_stall == "p")
                break;
        }
        setCmd(StepperMotorRunMode::STALL);
        publishCmd();
    }
    else if (cmd_type == "vel_backward")
    {
        ROS_INFO_STREAM("Reverse rotation!");

        setCmd(StepperMotorRunMode::VEL_REVERSE);
        publishCmd();
        while (ros::ok())
        {
            getline(std::cin, is_stall);
            if (is_stall == "p")
                break;
        }
        setCmd(StepperMotorRunMode::STALL);
        publishCmd();
    }
    else
    {
        ROS_WARN_STREAM("Motor control type error!");
    }
}

/**
 * @brief 发送运行参数,保存到flash后掉电数据不会丢失，因此只需设置一次
 */
void MotorParam::writeParam()
{
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<general_file::can_msgs>("/usbcan/motor_42", 100);
    std::array<general_file::can_msgs, 6> data_arr;  // data_arr[0]:启动周期   data_arr[1]:恒速周期 data_arr[2]:加速步数
                                                     // data_arr[3]:加速系数   data_arr[4]:细分    data_arr[5]：工作模式
    for (auto& data : data_arr)
    {
        data.ID = 0xc1;  // 帧ID，与驱动器地址相同
        data.SendType = 1;  // 单次发送（只发送一次，发送失败不会自动重发，总线只产生一帧数据）
        data.RemoteFlag = 0;                                   // 0为数据帧，1为远程帧
        data.ExternFlag = 0;                                   // 0为标准帧，1为拓展帧
        data.DataLen = 8;                                      // 数据长度8字节
        data.Data[0] = static_cast<unsigned char>(0xc1 >> 3);  // CAN 驱动器地址0x0c1(11bits)的高 8bits
        data.Data[1] = static_cast<unsigned char>(0xc1 << 5);  // CAN 驱动器地址的低 3bits，后五位一般设置为 0
        data.Data[2] = static_cast<unsigned char>((CMD_REQUEST << 5) | 0X13);  // 运行参数保存到内存
    }
    // 小端模式：低地址存放低位
    std::vector<std::vector<int>> data_vec{ { 0 }, { 0 }, { 0 }, { 0 }, { 0 }, { 0 } };
    ros::param::get("/motor_42/operating_param/plus_start_time", data_vec[0]);
    ros::param::get("/motor_42/operating_param/plus_constant_time", data_vec[1]);
    ros::param::get("/motor_42/operating_param/acc_steps", data_vec[2]);
    ros::param::get("/motor_42/operating_param/acc_cof", data_vec[3]);
    ros::param::get("/motor_42/operating_param/sub_divide", data_vec[4]);
    ros::param::get("/motor_42/operating_param/reset_mode", data_vec[5]);
    for (size_t i = 0; i < 6; ++i)
    {
        for (size_t j = 0; j < 5; ++j)
        {
            data_arr[i].Data[j + 3] = static_cast<unsigned char>(data_vec[i][j]);
        }
    }

    ROS_INFO_STREAM("Sending operating parameters!");
    for (auto& data : data_arr)
    {
        pub.publish(data);
        usleep(100000);
    }
    ROS_INFO_STREAM("********************************");
    data_arr[0].Data[2] = static_cast<unsigned char>((CMD_REQUEST << 5) | 0X14);  // 运行参数保存到flash，掉电数据不丢失
    pub.publish(data_arr[0]);
    usleep(100000);
    ROS_INFO_STREAM("Parameters have been saved in flash!");
}

/**
 * @brief 从flash中读取电机运行参数
 */
void MotorParam::readParam()
{
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<general_file::can_msgs>("/usbcan/motor_42", 100);
    std::array<general_file::can_msgs, 7> data_arr;  // data_arr[0]:启动周期   data_arr[1]:恒速周期 data_arr[2]:加速步数
                                                     // data_arr[3]:加速系数   data_arr[4]:细分    data_arr[5]：工作模式
                                                     // data_arr[6]：相电流
    for (auto& data : data_arr)
    {
        data.ID = 0xc1;  // 帧ID，与驱动器地址相同
        data.SendType = 1;  // 单次发送（只发送一次，发送失败不会自动重发，总线只产生一帧数据）
        data.RemoteFlag = 0;                                   // 0为数据帧，1为远程帧
        data.ExternFlag = 0;                                   // 0为标准帧，1为拓展帧
        data.DataLen = 8;                                      // 数据长度8字节
        data.Data[0] = static_cast<unsigned char>(0xc1 >> 3);  // CAN 驱动器地址0x0c1(11bits)的高 8bits
        data.Data[1] = static_cast<unsigned char>(0xc1 << 5);  // CAN 驱动器地址的低 3bits，后五位一般设置为 0
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

    ROS_INFO_STREAM("Reading operating parameters!");
    for (auto& data : data_arr)
    {
        pub.publish(data);
        usleep(100000);
    }
}
