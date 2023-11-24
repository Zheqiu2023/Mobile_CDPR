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
#include <algorithm>
#include <unistd.h>
#include <std_msgs/Bool.h>

using namespace motor_re35;

MotorDriver::MotorDriver(ros::NodeHandle& nh) : nh_(nh)
{
    XmlRpc::XmlRpcValue can_config;
    if (!(nh_.getParam("reduction_ratio", reduction_ratio_) && nh_.getParam("encoder_lines_num", encoder_lines_num_) &&
          nh_.getParam("can_config", can_config)))
        ROS_ERROR("Some motor params doesn't given in namespace: '%s')", nh_.getNamespace().c_str());
    ROS_ASSERT(can_config.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    for (auto iter = can_config.begin(); iter != can_config.end(); ++iter)
    {
        ROS_ASSERT(iter->second.hasMember("dev_ind") && iter->second.hasMember("can_ind") &&
                   iter->second.hasMember("driver_id"));

        cdpr_bringup::CanCmd pub_cmd{};
        pub_cmd.dev_ind = static_cast<unsigned int>((int)iter->second["dev_ind"]);
        pub_cmd.can_ind = static_cast<unsigned int>((int)iter->second["can_ind"]);
        pub_cmd.cmd.SendType = 1;    // Single send (sends only once, does not automatically retransmit after a failed
                                     // send, CAN bus generates only one frame of data)
        pub_cmd.cmd.RemoteFlag = 0;  // 0 for data frame, 1 for remote frame
        pub_cmd.cmd.ExternFlag = 0;  // 0 for standard frame, 1 for expanded frame
        pub_cmd.cmd.DataLen = 8;     // Data length 8 bytes

        motor_data_.push_back(MotorData{ .driver_id_ = (int)iter->second["driver_id"],
                                         .target_pos_ = 0.0,
                                         .target_force_ = 0.0,
                                         .pub_cmd_ = std::move(pub_cmd) });
    }

    pubs_.emplace_back(nh_.advertise<cdpr_bringup::CanCmd>("/usbcan/motor_re35", 10));
    pubs_.emplace_back(nh_.advertise<std_msgs::Bool>("/motor_re35/reset_flag", 1));
    subs_.emplace_back(nh_.subscribe("/cable_length", 10, &MotorDriver::recvCableLengthCB, this));
    subs_.emplace_back(nh_.subscribe("/cable_force", 10, &MotorDriver::recvCableForceCB, this));
    subs_.emplace_back(nh_.subscribe("/usb_can/motor_state", 10, &MotorDriver::recvStateCB, this));
    ros::Duration(1.0).sleep();  // Sleep for 1s to ensure that the first message sent is received by USBCAN
}

void MotorDriver::recvStateCB(const cdpr_bringup::CanFrame::ConstPtr& state)
{
    int id = (state->ID - 0x00B) >> 4;
    for (const auto& motor_data : motor_data_)
    {
        if (motor_data.driver_id_ == id)
        {
            short real_current = (state->Data[0] << 8) | state->Data[1];
            short real_velocity = (state->Data[2] << 8) | state->Data[3];
            int real_position =
                (state->Data[4] << 24) | (state->Data[5] << 16) | (state->Data[6] << 8) | state->Data[7];
            ROS_INFO("driver id:%d, current:%d, velocity:%d, position:%d", id, real_current, real_velocity,
                     real_position);
            break;
        }
    }
}

void MotorDriver::recvCableLengthCB(const std_msgs::Float32MultiArray::ConstPtr& length)
{
    std::lock_guard<std::mutex> guard(mtx1_);
    for (size_t i = 0; i < motor_data_.size(); ++i)
    {
        motor_data_[i].target_pos_ = length->data[i];
    }
}

void MotorDriver::recvCableForceCB(const std_msgs::Float32MultiArray::ConstPtr& force)
{
    std::lock_guard<std::mutex> guard(mtx2_);
    for (size_t i = 0; i < motor_data_.size(); ++i)
    {
        motor_data_[i].target_force_ = force->data[i];
    }
}

void MotorDriver::publishCmd(const cdpr_bringup::CanCmd& cmd_struct)
{
    pubs_[0].publish(cmd_struct);
}

void MotorDriver::init(const int& run_mode)
{
    for (auto& motor_data : motor_data_)
    {
        // 发送复位指令
        motor_data.pub_cmd_.cmd.ID =
            0x000 | (motor_data.driver_id_ << 4);  // 复位指令（帧ID，由驱动器编号和功能序号决定）
        for (auto& data : motor_data.pub_cmd_.cmd.Data)
            data = 0x55;
        publishCmd(motor_data.pub_cmd_);
    }
    usleep(500000);
    for (auto& motor_data : motor_data_)
    {
        // 发送配置指令
        motor_data.pub_cmd_.cmd.ID = 0x00A | (motor_data.driver_id_ << 4);  // 配置指令
        for (auto& data : motor_data.pub_cmd_.cmd.Data)
            data = 0x55;
        motor_data.pub_cmd_.cmd.Data[0] = 0x0a;  // 以 10 毫秒为周期对外发送电流、速度、位置等信息
        motor_data.pub_cmd_.cmd.Data[1] = 0x00;
        publishCmd(motor_data.pub_cmd_);
    }
    usleep(500000);
    for (auto& motor_data : motor_data_)
    {
        // 发送模式选择指令
        motor_data.pub_cmd_.cmd.ID = 0x001 | (motor_data.driver_id_ << 4);  // 模式选择指令
        for (auto& data : motor_data.pub_cmd_.cmd.Data)
            data = 0x55;
        switch (run_mode)
        {
            case 5:
                motor_data.pub_cmd_.cmd.Data[0] = 0x05;  // 选择速度位置模式
                publishCmd(motor_data.pub_cmd_);
                break;
            case 7:
                motor_data.pub_cmd_.cmd.Data[0] = 0x07;  // 选择电流位置模式
                publishCmd(motor_data.pub_cmd_);
                break;
            case 8:
                motor_data.pub_cmd_.cmd.Data[0] = 0x08;  // 选择电流速度位置模式
                publishCmd(motor_data.pub_cmd_);
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
void MotorDriver::run()
{
    int run_mode = nh_.param("run_mode", 5);
    init(run_mode);

    switch (run_mode)
    {
        // 速度位置模式
        case 5: {
            std::vector<int> target_pos_vec{}, target_pos(motor_data_.size(), 0), history_pos(motor_data_.size(), 0);
            unsigned short temp_vel = 1000;  // 速度限制值(RPM)：0~32767

            nh_.getParam("target_data/target_pos_vec", target_pos_vec);

            for (auto& motor_data : motor_data_)
            {
                motor_data.pub_cmd_.cmd.ID = 0x006 | (motor_data.driver_id_ << 4);  // 速度位置模式下的参数指令，非广播
                motor_data.pub_cmd_.cmd.Data[0] = static_cast<unsigned char>((PWM_LIM >> 8) & 0xff);
                motor_data.pub_cmd_.cmd.Data[1] = static_cast<unsigned char>(PWM_LIM & 0xff);
                motor_data.pub_cmd_.cmd.Data[2] = static_cast<unsigned char>((temp_vel >> 8) & 0xff);
                motor_data.pub_cmd_.cmd.Data[3] = static_cast<unsigned char>(temp_vel & 0xff);
            }
            for (auto& temp_pos : target_pos_vec)
            {
                for (size_t i = 0; i < motor_data_.size(); ++i)
                {
                    history_pos[i] += temp_pos;
                    target_pos[i] = history_pos[i] * reduction_ratio_ * encoder_lines_num_ / 360;  // °转换为qc
                    motor_data_[i].pub_cmd_.cmd.Data[4] = static_cast<unsigned char>((target_pos[i] >> 24) & 0xff);
                    motor_data_[i].pub_cmd_.cmd.Data[5] = static_cast<unsigned char>((target_pos[i] >> 16) & 0xff);
                    motor_data_[i].pub_cmd_.cmd.Data[6] = static_cast<unsigned char>((target_pos[i] >> 8) & 0xff);
                    motor_data_[i].pub_cmd_.cmd.Data[7] = static_cast<unsigned char>(target_pos[i] & 0xff);

                    publishCmd(motor_data_[i].pub_cmd_);
                }
                sleep(1);
            }
            break;
        }
        // 电流速度位置模式
        case 8: {
            std::vector<int> cmd_pos{}, cmd_force{};
            unsigned short temp_vel = 1000;      // 速度限制值(RPM)：0~32767
            unsigned short temp_current = 1000;  // 电流限制值(mA)：0~32767
            for (auto& motor_data : motor_data_)
            {
                motor_data.pub_cmd_.cmd.ID =
                    0x009 | (motor_data.driver_id_ << 4);  // 电流速度位置模式下的参数指令，非广播
                motor_data.pub_cmd_.cmd.Data[2] = static_cast<unsigned char>((temp_vel >> 8) & 0xff);
                motor_data.pub_cmd_.cmd.Data[3] = static_cast<unsigned char>(temp_vel & 0xff);
            }

            std_msgs::Bool reset_done{};
            reset_done.data = true;
            pubs_[1].publish(reset_done);

            while (ros::ok())
            {
                std::lock(mtx1_, mtx2_);
                std::lock_guard<std::mutex> lck1(mtx1_, std::adopt_lock);
                std::lock_guard<std::mutex> lck2(mtx2_, std::adopt_lock);
                for (size_t i = 0; i < motor_data_.size(); ++i)
                {
                    cmd_pos[i] = motor_data_[i].target_pos_ * 1000 * reduction_ratio_ * encoder_lines_num_ /
                                 (2 * M_PI * reel_radius_);  // °转换为qc
                    motor_data_[i].pub_cmd_.cmd.Data[0] = static_cast<unsigned char>((temp_current >> 8) & 0xff);
                    motor_data_[i].pub_cmd_.cmd.Data[1] = static_cast<unsigned char>(temp_current & 0xff);
                    motor_data_[i].pub_cmd_.cmd.Data[4] = static_cast<unsigned char>((cmd_pos[i] >> 24) & 0xff);
                    motor_data_[i].pub_cmd_.cmd.Data[5] = static_cast<unsigned char>((cmd_pos[i] >> 16) & 0xff);
                    motor_data_[i].pub_cmd_.cmd.Data[6] = static_cast<unsigned char>((cmd_pos[i] >> 8) & 0xff);
                    motor_data_[i].pub_cmd_.cmd.Data[7] = static_cast<unsigned char>(cmd_pos[i] & 0xff);

                    publishCmd(motor_data_[i].pub_cmd_);
                }
            }
            // Return to initial position
            for (auto& motor_data : motor_data_)
            {
                for (size_t i = 4; i < 8; ++i)
                    motor_data.pub_cmd_.cmd.Data[i] = 0;
                publishCmd(motor_data.pub_cmd_);
            }
            break;
        }
        default:
            break;
    }
}