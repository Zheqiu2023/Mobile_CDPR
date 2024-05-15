/**
 * @File Name: maxon_re35.cpp
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
#include "maxon_re35/maxon_re35.hpp"

#include <std_msgs/Bool.h>
#include <unistd.h>

#include <algorithm>
#include <vector>

#include "usb_can/usb_can.hpp"

using namespace maxon_re35;

MotorDriver::MotorDriver(ros::NodeHandle& nh) : nh_(nh)
{
    XmlRpc::XmlRpcValue cmd_config;
    if (!(nh_.getParam("reduction_ratio", reduction_ratio_) && nh_.getParam("encoder_lines_num", encoder_lines_num_) &&
          nh_.getParam("cmd_config", cmd_config) && nh_.getParam("reel_diameter", reel_diameter_)))
        ROS_ERROR("Some motor params are not given in namespace: '%s')", nh_.getNamespace().c_str());

    ROS_ASSERT(cmd_config.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    for (auto iter = cmd_config.begin(); iter != cmd_config.end(); ++iter)
    {
        ROS_ASSERT(iter->second.hasMember("dev_ind") && iter->second.hasMember("can_ind") &&
                   iter->second.hasMember("driver_id") && iter->second.hasMember("direction"));

        cdpr_bringup::CanCmd pub_cmd{};
        pub_cmd.dev_ind = static_cast<unsigned int>((int)iter->second["dev_ind"]);
        pub_cmd.can_ind = static_cast<unsigned int>((int)iter->second["can_ind"]);
        pub_cmd.cmd.SendType = 1;    // Single send (sends only once, does not automatically retransmit after a failed
                                     // send, CAN bus generates only one frame of data)
        pub_cmd.cmd.RemoteFlag = 0;  // 0 for data frame, 1 for remote frame
        pub_cmd.cmd.ExternFlag = 0;  // 0 for standard frame, 1 for expanded frame
        pub_cmd.cmd.DataLen = 8;     // Data length 8 bytes

        motor_data_.push_back(MotorData{ .driver_id_ = static_cast<int>(iter->second["driver_id"]),
                                         .direction_ = static_cast<int>(iter->second["direction"]),
                                         .target_pos_ = 0.0,
                                         .last_pos_ = 0.0,
                                         .pub_cmd_ = std::move(pub_cmd) });
    }

    pubs_.emplace_back(nh_.advertise<cdpr_bringup::CanCmd>("motor_cmd", 10));
    pubs_.emplace_back(nh_.advertise<std_msgs::Bool>("ready_state", 1));
    cur_pos_pubs_.push_back(nh_.advertise<std_msgs::Float64>("/cable0_cur_pos", 10));
    cur_pos_pubs_.push_back(nh_.advertise<std_msgs::Float64>("/cable1_cur_pos", 10));
    cur_pos_pubs_.push_back(nh_.advertise<std_msgs::Float64>("/cable2_cur_pos", 10));
    cur_pos_pubs_.push_back(nh_.advertise<std_msgs::Float64>("/cable3_cur_pos", 10));
    subs_.emplace_back(nh_.subscribe("/cable_length", 50, &MotorDriver::cmdCableLengthCB, this));
    ros::Duration(1.0).sleep();  // Sleep for 1s to ensure that the first message sent is received by USBCAN
}

void MotorDriver::motorStateCB(const cdpr_bringup::CanFrame::ConstPtr& state)
{
    int id = state->ID - 0x0B;
    for (const auto& motor_data : motor_data_)
    {
        if (id == (motor_data.driver_id_ << 4))
        {
            short real_velocity = (state->Data[2] << 8) | state->Data[3];
            int real_position =
                (state->Data[4] << 24) | (state->Data[5] << 16) | (state->Data[6] << 8) | state->Data[7];
            cur_pos_.data =
                motor_data.direction_ * real_position * M_PI * reel_diameter_ / (reduction_ratio_ * encoder_lines_num_);
            cur_pos_pubs_[motor_data.driver_id_ - 1].publish(cur_pos_);

            // ROS_INFO("driver id:%d, velocity:%d, position:%d, cur_pos:%.5f", motor_data.driver_id_, real_velocity,
            //          real_position, cur_pos_.data);
            return;
        }
    }
}

void MotorDriver::cmdCableLengthCB(const cdpr_bringup::TrajCmd::ConstPtr& length)
{
    std::lock_guard<std::mutex> guard(mutex_);
    is_traj_end_ = length->is_traj_end;
    for (size_t i = 0; i < motor_data_.size(); ++i)
    {
        motor_data_[i].last_pos_ = motor_data_[i].target_pos_;
        motor_data_[i].target_pos_ = length->target[i] * motor_data_[i].direction_;
    }
    // ROS_INFO("target pos: %.5f, %.5f, %.5f, %.5f", length->target[0], length->target[1], length->target[2],
    //          length->target[3]);
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
        std::fill(motor_data.pub_cmd_.cmd.Data.begin(), motor_data.pub_cmd_.cmd.Data.end(), 0x55);
        publishCmd(motor_data.pub_cmd_);
    }
    ros::Duration(0.5).sleep();
    for (auto& motor_data : motor_data_)
    {
        // 发送模式选择指令
        motor_data.pub_cmd_.cmd.ID = 0x001 | (motor_data.driver_id_ << 4);  // 模式选择指令
        motor_data.pub_cmd_.cmd.Data[1] = 0x55;
        switch (run_mode)
        {
            case 0:
            case 1:
                motor_data.pub_cmd_.cmd.Data[0] = 0x05;  // 选择速度位置模式
                break;
            case 2:
            case 3:
                motor_data.pub_cmd_.cmd.Data[0] = 0x03;  // 选择速度模式
                break;
            default:
                ROS_ERROR("Undefined run mode!");
                break;
        }
        publishCmd(motor_data.pub_cmd_);
    }
    ros::Duration(0.5).sleep();
    for (auto& motor_data : motor_data_)
    {
        // 发送配置指令
        motor_data.pub_cmd_.cmd.ID = 0x00A | (motor_data.driver_id_ << 4);  // 配置指令
        motor_data.pub_cmd_.cmd.Data[0] = 0x01;  // 以 1 毫秒为周期对外发送电流、速度、位置等信息
        motor_data.pub_cmd_.cmd.Data[1] = 0x00;
        publishCmd(motor_data.pub_cmd_);
    }
    ros::Duration(0.5).sleep();
}

/**
 * @brief 按指定模式运行电机
 */
void MotorDriver::run()
{
    int run_mode = nh_.param("run_mode", 1);
    init(run_mode);

    switch (run_mode)
    {
        // 回零位
        case 0: {
            int cmd_pos = 0, cmd_vel = nh_.param("target_data/target_vel", 1000);  // 速度限制值(RPM)：0~32767

            for (auto& motor_data : motor_data_)
            {
                motor_data.pub_cmd_.cmd.ID = 0x006 | (motor_data.driver_id_ << 4);  // 速度位置模式下的参数指令，非广播
                motor_data.pub_cmd_.cmd.Data[0] = static_cast<unsigned char>((PWM_LIM >> 8) & 0xff);
                motor_data.pub_cmd_.cmd.Data[1] = static_cast<unsigned char>(PWM_LIM & 0xff);
                motor_data.pub_cmd_.cmd.Data[2] = static_cast<unsigned char>((cmd_vel >> 8) & 0xff);
                motor_data.pub_cmd_.cmd.Data[3] = static_cast<unsigned char>(cmd_vel & 0xff);
                for (size_t j = 4; j < 8; ++j)
                {
                    motor_data.pub_cmd_.cmd.Data[j] = 0;
                }

                publishCmd(motor_data.pub_cmd_);
            }

            break;
        }
        // 轨迹模式
        case 1: {
            int cmd_pos = 0.0, cmd_vel = 0.0;  // 速度限制值(RPM)：0~32767
            if (!(nh_.getParam("/traj/cdpr_period", traj_period_)))
                ROS_ERROR("Some motor params are not given in namespace: 'traj'");

            for (auto& motor_data : motor_data_)
            {
                motor_data.pub_cmd_.cmd.ID = 0x006 | (motor_data.driver_id_ << 4);  // 速度位置模式下的参数指令，非广播
                motor_data.pub_cmd_.cmd.Data[0] = static_cast<unsigned char>((PWM_LIM >> 8) & 0xff);
                motor_data.pub_cmd_.cmd.Data[1] = static_cast<unsigned char>(PWM_LIM & 0xff);
            }

            std_msgs::Bool is_ready{};
            is_ready.data = true;
            pubs_[1].publish(is_ready);
            ROS_INFO("Motor RE35 Reset done, ready to follow the trajectory!");
            subs_.emplace_back(nh_.subscribe("/usbcan/motor_state", 10, &MotorDriver::motorStateCB, this));

            // follow the trajectory
            while (ros::ok())
            {
                std::lock_guard<std::mutex> guard(mutex_);
                for (auto& motor_data : motor_data_)
                {
                    cmd_pos = std::round(motor_data.target_pos_ * reduction_ratio_ * encoder_lines_num_ /
                                         (M_PI * reel_diameter_));  // m转换为qc
                    cmd_vel = std::ceil(std::fabs((motor_data.target_pos_ - motor_data.last_pos_) * 60 *
                                                  reduction_ratio_ / (traj_period_ * M_PI * reel_diameter_)));

                    motor_data.pub_cmd_.cmd.Data[2] = static_cast<unsigned char>((cmd_vel >> 8) & 0xff);
                    motor_data.pub_cmd_.cmd.Data[3] = static_cast<unsigned char>(cmd_vel & 0xff);
                    motor_data.pub_cmd_.cmd.Data[4] = static_cast<unsigned char>((cmd_pos >> 24) & 0xff);
                    motor_data.pub_cmd_.cmd.Data[5] = static_cast<unsigned char>((cmd_pos >> 16) & 0xff);
                    motor_data.pub_cmd_.cmd.Data[6] = static_cast<unsigned char>((cmd_pos >> 8) & 0xff);
                    motor_data.pub_cmd_.cmd.Data[7] = static_cast<unsigned char>(cmd_pos & 0xff);

                    publishCmd(motor_data.pub_cmd_);
                }
                if (is_traj_end_)
                    break;
            }

            ros::Duration(2.0).sleep();  // buffering time for motors moving back to zero position
            break;
        }
        default:
            break;
    }
}