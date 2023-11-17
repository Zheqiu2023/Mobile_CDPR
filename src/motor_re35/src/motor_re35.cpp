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

#include <vector>
#include <unistd.h>

using namespace motor_re35;

/**
 * @brief Construct a new MotorRun::MotorRun object
 */
MotorRun::MotorRun(ros::NodeHandle& nh) : nh_(nh)
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

void MotorRun::init(const int& run_mode)
{
    XmlRpc::XmlRpcValue can_config;
    nh_.getParam("can_config", can_config);
    ROS_ASSERT(can_config.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    driver_id_.resize(can_config.size());
    pub_cmd_.resize(can_config.size());
    auto iter = can_config.begin();
    for (int i = 0; i < can_config.size(); ++i)
    {
        ROS_ASSERT(iter->second.hasMember("dev_ind") && iter->second.hasMember("can_ind") &&
                   iter->second.hasMember("driver_id"));
        driver_id_[i] = static_cast<unsigned int>((int)iter->second["driver_id"]);
        pub_cmd_[i].dev_ind = static_cast<unsigned int>((int)iter->second["dev_ind"]);
        pub_cmd_[i].can_ind = static_cast<unsigned int>((int)iter->second["can_ind"]);
        pub_cmd_[i].cmd.SendType = 1;  // Single send (sends only once, does not automatically retransmit after a failed
                                       // send, CAN bus generates only one frame of data)
        pub_cmd_[i].cmd.RemoteFlag = 0;  // 0 for data frame, 1 for remote frame
        pub_cmd_[i].cmd.ExternFlag = 0;  // 0 for standard frame, 1 for expanded frame
        pub_cmd_[i].cmd.DataLen = 8;     // Data length 8 bytes
        ++iter;
    }

    for (int i = 0; i < can_config.size(); ++i)
    {
        // 发送复位指令
        pub_cmd_[i].cmd.ID = 0x000 || (driver_id_[i] << 4);  // 复位指令（帧ID，由驱动器编号和功能序号决定）
        for (auto& data : pub_cmd_[i].cmd.Data)
            data = 0x55;
        publishCmd(pub_cmd_[i]);
    }
    usleep(500000);
    for (int i = 0; i < can_config.size(); ++i)
    {
        // 发送配置指令
        pub_cmd_[i].cmd.ID = 0x00A || (driver_id_[i] << 4);  // 配置指令
        for (auto& data : pub_cmd_[i].cmd.Data)
            data = 0x55;
        pub_cmd_[i].cmd.Data[0] = 0x0a;  // 以 10 毫秒为周期对外发送电流、速度、位置等信息
        pub_cmd_[i].cmd.Data[1] = 0x00;
        publishCmd(pub_cmd_[i]);
    }
    usleep(500000);
    for (int i = 0; i < can_config.size(); ++i)
    {
        // 发送模式选择指令
        pub_cmd_[i].cmd.ID = 0x001 || (driver_id_[i] << 4);  // 模式选择指令
        for (auto& data : pub_cmd_[i].cmd.Data)
            data = 0x55;
        switch (run_mode)
        {
            case 5:
                pub_cmd_[i].cmd.Data[0] = 0x05;  // 选择速度位置模式
                publishCmd(pub_cmd_[i]);
                break;
            case 7:
                pub_cmd_[i].cmd.Data[0] = 0x07;  // 选择电流位置模式
                publishCmd(pub_cmd_[i]);
                break;
            case 8:
                pub_cmd_[i].cmd.Data[0] = 0x08;  // 选择电流速度位置模式
                publishCmd(pub_cmd_[i]);
                break;
            default:
                ROS_ERROR("Undefined run mode!");
                break;
        }
    }
    usleep(500000);
}

/**
 * @brief 按指定模式运行电机
 */
void MotorRun::run()
{
    int run_mode = nh_.param("run_mode", 5);
    init(run_mode);

    switch (run_mode)
    {
        case 5: {  // 速度位置模式
            std::vector<int> target_pos_vec{};
            int temp_pos = 0, history_pos = 0, reduction_ratio = 0, encoder_lines_num = 0;
            unsigned short temp_vel = 10000;  // 速度限制值(RPM)：0~32767

            nh_.getParam("target_data/target_pos_vec", target_pos_vec);
            nh_.getParam("reduction_ratio", reduction_ratio);
            nh_.getParam("encoder_lines_num", encoder_lines_num);

            for (size_t i = 0; i < pub_cmd_.size(); ++i)
            {
                pub_cmd_[i].cmd.ID = 0x006 || (driver_id_[i] << 4);  // 速度位置模式下的参数指令，非广播
                pub_cmd_[i].cmd.Data[0] = static_cast<unsigned char>((PWM_LIM >> 8) & 0xff);
                pub_cmd_[i].cmd.Data[1] = static_cast<unsigned char>(PWM_LIM & 0xff);
                pub_cmd_[i].cmd.Data[2] = static_cast<unsigned char>((temp_vel >> 8) & 0xff);
                pub_cmd_[i].cmd.Data[3] = static_cast<unsigned char>(temp_vel & 0xff);
            }
            for (auto& temp_pos : target_pos_vec)
            {
                history_pos += temp_pos;
                temp_pos = history_pos * reduction_ratio * encoder_lines_num / 360;  // °转换为qc
                for (auto& pub_cmd : pub_cmd_)
                {
                    pub_cmd.cmd.Data[4] = static_cast<unsigned char>((temp_pos >> 24) & 0xff);
                    pub_cmd.cmd.Data[5] = static_cast<unsigned char>((temp_pos >> 16) & 0xff);
                    pub_cmd.cmd.Data[6] = static_cast<unsigned char>((temp_pos >> 8) & 0xff);
                    pub_cmd.cmd.Data[7] = static_cast<unsigned char>(temp_pos & 0xff);

                    publishCmd(pub_cmd);
                }
                sleep(1);
            }
            break;
        }
        case 8: {  // 电流速度位置模式
            std::vector<int> target_pos_vec{};
            int temp_pos = 0, history_pos = 0, reduction_ratio = 0, encoder_lines_num = 0;
            unsigned short temp_vel = 10000;      // 速度限制值(RPM)：0~32767
            unsigned short temp_current = 10000;  // 电流限制值(mA)：0~32767

            nh_.getParam("target_data/target_pos_vec", target_pos_vec);
            nh_.getParam("reduction_ratio", reduction_ratio);
            nh_.getParam("encoder_lines_num", encoder_lines_num);

            for (size_t i = 0; i < pub_cmd_.size(); ++i)
            {
                pub_cmd_[i].cmd.ID = 0x009 || (driver_id_[i] << 4);  // 速度位置模式下的参数指令，非广播
                pub_cmd_[i].cmd.Data[0] = static_cast<unsigned char>((temp_current >> 8) & 0xff);
                pub_cmd_[i].cmd.Data[1] = static_cast<unsigned char>(temp_current & 0xff);
                pub_cmd_[i].cmd.Data[2] = static_cast<unsigned char>((temp_vel >> 8) & 0xff);
                pub_cmd_[i].cmd.Data[3] = static_cast<unsigned char>(temp_vel & 0xff);
            }
            for (auto& temp_pos : target_pos_vec)
            {
                history_pos += temp_pos;
                temp_pos = history_pos * reduction_ratio * encoder_lines_num / 360;  // °转换为qc
                for (auto& pub_cmd : pub_cmd_)
                {
                    pub_cmd.cmd.Data[4] = static_cast<unsigned char>((temp_pos >> 24) & 0xff);
                    pub_cmd.cmd.Data[5] = static_cast<unsigned char>((temp_pos >> 16) & 0xff);
                    pub_cmd.cmd.Data[6] = static_cast<unsigned char>((temp_pos >> 8) & 0xff);
                    pub_cmd.cmd.Data[7] = static_cast<unsigned char>(temp_pos & 0xff);

                    publishCmd(pub_cmd);
                }
                sleep(1);
            }
            break;
        }
        default:
            break;
    }
}