/**
 * @File Name: movable_archor.cpp
 * @brief
 * @author Zhe Qiu (zheqiu2021@163.com)
 * @version 0.1
 * @date 2023-12-14
 *
 * *  ***********************************************************************************
 * *  @copyright Copyright (c) 2023  by Zhe Qiu. All rights reserved.
 * *  Use of this source code is governed by the BSD 3-Clause license, see LICENSE.
 * *  ***********************************************************************************
 */
#include "cdpr_movable_archor/movable_archor.hpp"

#include <std_msgs/Bool.h>
#include <unistd.h>

#include <algorithm>
#include <vector>

#include "usb_can/usb_can.hpp"

using namespace movable_archor;

ArchorDriver::ArchorDriver(ros::NodeHandle& nh) : nh_(nh) {
    XmlRpc::XmlRpcValue cmd_config;
    if (!(nh_.getParam("reduction_ratio", reduction_ratio_) && nh_.getParam("encoder_lines_num", encoder_lines_num_) &&
          nh_.getParam("cmd_config", cmd_config)))
        ROS_ERROR("Some motor params are not given in namespace: '%s')", nh_.getNamespace().c_str());

    ROS_ASSERT(cmd_config.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    for (auto iter = cmd_config.begin(); iter != cmd_config.end(); ++iter) {
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

        motor_data_.push_back(MotorData{.is_reset_ = false,
                                        .driver_id_ = static_cast<int>(iter->second["driver_id"]),
                                        .direction_ = static_cast<int>(iter->second["direction"]),
                                        .target_pos_ = 0.0,
                                        .last_pos_ = 0.0,
                                        .pub_cmd_ = std::move(pub_cmd)});
    }

    pubs_.push_back(nh_.advertise<cdpr_bringup::CanCmd>("motor_cmd", 10));
    pubs_.push_back(nh_.advertise<std_msgs::Bool>("ready_state", 1));
    cur_pos_pubs_.push_back(nh_.advertise<std_msgs::Float64>("/archor0_cur_pos", 10));
    cur_pos_pubs_.push_back(nh_.advertise<std_msgs::Float64>("/archor1_cur_pos", 10));
    cur_pos_pubs_.push_back(nh_.advertise<std_msgs::Float64>("/archor2_cur_pos", 10));
    cur_pos_pubs_.push_back(nh_.advertise<std_msgs::Float64>("/archor3_cur_pos", 10));
    subs_.push_back(nh_.subscribe("/archor_coor_z", 50, &ArchorDriver::cmdPosCallback, this));
    subs_.push_back(nh_.subscribe("/usbcan/motor_state", 10, &ArchorDriver::motorStateCB, this));
    ros::Duration(1.0).sleep();  // Sleep for 1s to ensure that the first message sent is received by USBCAN
}

void ArchorDriver::motorStateCB(const cdpr_bringup::CanFrame::ConstPtr& state) {
    int id1 = state->ID - 0x0b;
    int id2 = state->ID - 0x0c;

    for (auto& motor_data : motor_data_) {
        if (id1 == (motor_data.driver_id_ << 4)) {
            // 接收到电流、速度、位置等信息
            short real_velocity = (state->Data[2] << 8) | state->Data[3];
            int real_position =
                (state->Data[4] << 24) | (state->Data[5] << 16) | (state->Data[6] << 8) | state->Data[7];
            cur_pos_.data = (double)real_position * lead_ / (1000 * reduction_ratio_ * encoder_lines_num_);
            cur_pos_pubs_[motor_data.driver_id_ - 5].publish(cur_pos_);

            // ROS_INFO("driver id:%d, velocity:%d, position:%d, cur_pos:%.5f", motor_data.driver_id_, real_velocity,
            //          real_position, cur_pos_.data);
            return;
        } else if (id2 == (motor_data.driver_id_ << 4) && state->Data[0] == 0x01) {
            // 接收到CTL1/CTL2的电平状态
            std::lock_guard<std::mutex> guard(state_mutex_);
            motor_data.is_reset_ = true;
            return;
        }
    }
}

void ArchorDriver::cmdPosCallback(const cdpr_bringup::TrajCmd::ConstPtr& pos) {
    std::lock_guard<std::mutex> guard(pos_mutex_);
    is_traj_end_ = pos->is_traj_end;
    for (size_t i = 0; i < motor_data_.size(); ++i) {
        motor_data_[i].last_pos_ = motor_data_[i].target_pos_;
        motor_data_[i].target_pos_ = -pos->target[i] * motor_data_[i].direction_;
    }
    // ROS_INFO("target pos: %.5f, %.5f, %.5f, %.5f", pos->target[0], pos->target[1], pos->target[2], pos->target[3]);
}

void ArchorDriver::publishCmd(const cdpr_bringup::CanCmd& cmd_struct) { pubs_[0].publish(cmd_struct); }

void ArchorDriver::init(RunMode mode) {
    for (auto& motor_data : motor_data_) {
        // 发送复位指令
        motor_data.pub_cmd_.cmd.ID =
            0x000 | (motor_data.driver_id_ << 4);  // 复位指令（帧ID，由驱动器编号和功能序号决定）
        std::fill(motor_data.pub_cmd_.cmd.Data.begin(), motor_data.pub_cmd_.cmd.Data.end(), 0x55);
        publishCmd(motor_data.pub_cmd_);
    }
    ros::Duration(0.6).sleep();
    for (auto& motor_data : motor_data_) {
        // 发送模式选择指令
        motor_data.pub_cmd_.cmd.ID = 0x001 | (motor_data.driver_id_ << 4);   // 模式选择指令
        motor_data.pub_cmd_.cmd.Data[0] = static_cast<unsigned char>(mode);  // 选择mode对应模式
        publishCmd(motor_data.pub_cmd_);
    }
    ros::Duration(0.6).sleep();
    for (auto& motor_data : motor_data_) {
        // 发送配置指令
        motor_data.pub_cmd_.cmd.ID = 0x00A | (motor_data.driver_id_ << 4);  // 配置指令
        motor_data.pub_cmd_.cmd.Data[0] = 0x05;  // 以 5 毫秒为周期对外发送电流、速度、位置等信息
        motor_data.pub_cmd_.cmd.Data[1] = 0x05;  // 以 5 毫秒为周期对外发送CTL1/CTL2的电平状态
        publishCmd(motor_data.pub_cmd_);
    }
    ros::Duration(0.6).sleep();
}

/**
 * @brief 按指定模式运行电机
 */
void ArchorDriver::run() {
    init(RunMode::VEL);

    int run_mode = nh_.param("run_mode", 0);
    switch (run_mode) {
        // reset
        case 0: {
            int target_vel = nh_.param("target_vel", 1000);  // 速度限制值(RPM)：0~32767

            for (auto& motor_data : motor_data_) {
                motor_data.pub_cmd_.cmd.ID = 0x004 | (motor_data.driver_id_ << 4);  // 速度模式下的参数指令，非广播
                motor_data.pub_cmd_.cmd.Data[0] = static_cast<unsigned char>((PWM_LIM >> 8) & 0xff);
                motor_data.pub_cmd_.cmd.Data[1] = static_cast<unsigned char>(PWM_LIM & 0xff);
                motor_data.pub_cmd_.cmd.Data[2] =
                    static_cast<unsigned char>(((target_vel * motor_data.direction_) >> 8) & 0xff);
                motor_data.pub_cmd_.cmd.Data[3] =
                    static_cast<unsigned char>((target_vel * motor_data.direction_) & 0xff);
                publishCmd(motor_data.pub_cmd_);
            }

            ROS_INFO("Anchors reseting...");
            // Reset: move to bottom
            while (ros::ok()) {
                std::lock_guard<std::mutex> guard(state_mutex_);
                for (auto& motor_data : motor_data_) {
                    if (true == motor_data.is_reset_) {
                        for (int i = 0; i < 4; ++i) motor_data.pub_cmd_.cmd.Data[i] = 0;
                        publishCmd(motor_data.pub_cmd_);
                    }
                }

                if (std::all_of(motor_data_.begin(), motor_data_.end(), [](const MotorData& motor_data) {
                        return motor_data.is_reset_;
                    }))  // all archores reset successfully
                    break;
            }

            break;
        }
        // 轨迹模式
        case 1: {
            int target_vel = nh_.param("target_vel", 1000);  // 速度限制值(RPM)：0~32767
            for (auto& motor_data : motor_data_) {
                motor_data.pub_cmd_.cmd.ID = 0x004 | (motor_data.driver_id_ << 4);  // 速度模式下的参数指令，非广播
                motor_data.pub_cmd_.cmd.Data[0] = static_cast<unsigned char>((PWM_LIM >> 8) & 0xff);
                motor_data.pub_cmd_.cmd.Data[1] = static_cast<unsigned char>(PWM_LIM & 0xff);
                motor_data.pub_cmd_.cmd.Data[2] =
                    static_cast<unsigned char>(((target_vel * motor_data.direction_) >> 8) & 0xff);
                motor_data.pub_cmd_.cmd.Data[3] =
                    static_cast<unsigned char>((target_vel * motor_data.direction_) & 0xff);
                publishCmd(motor_data.pub_cmd_);
            }

            ROS_INFO("Anchors reseting...");
            // Reset: move to bottom
            while (ros::ok()) {
                std::lock_guard<std::mutex> guard(state_mutex_);
                for (auto& motor_data : motor_data_) {
                    if (true == motor_data.is_reset_) {
                        for (int i = 0; i < 4; ++i) motor_data.pub_cmd_.cmd.Data[i] = 0;
                        publishCmd(motor_data.pub_cmd_);
                    }
                }

                if (std::all_of(motor_data_.begin(), motor_data_.end(), [](const MotorData& motor_data) {
                        return motor_data.is_reset_;
                    }))  // all archores reset successfully
                    break;
            }

            int cmd_pos = 0.0, cmd_vel = 0.0;  // 速度限制值(RPM)：0~32767
            if (!(nh_.getParam("/traj/traj_period", traj_period_) && nh_.getParam("lead", lead_)))
                ROS_ERROR("Undefined parameter: traj_period, lead!");

            init(RunMode::VEL_POS);
            for (auto& motor_data : motor_data_) {
                motor_data.pub_cmd_.cmd.ID = 0x006 | (motor_data.driver_id_ << 4);  // 速度位置模式下的参数指令，非广播
                motor_data.pub_cmd_.cmd.Data[0] = static_cast<unsigned char>((PWM_LIM >> 8) & 0xff);
                motor_data.pub_cmd_.cmd.Data[1] = static_cast<unsigned char>(PWM_LIM & 0xff);
            }

            std_msgs::Bool is_ready{};
            is_ready.data = true;
            pubs_[1].publish(is_ready);
            ROS_INFO("Movable archor reset done, ready to follow the trajectory!");

            // follow the trajectory
            while (ros::ok()) {
                std::lock_guard<std::mutex> guard(pos_mutex_);
                for (auto& motor_data : motor_data_) {
                    cmd_pos =
                        motor_data.target_pos_ * 1000 * reduction_ratio_ * encoder_lines_num_ / lead_;  // m转换为qc
                    cmd_vel = std::fabs((motor_data.target_pos_ - motor_data.last_pos_) * 60 * 1000 * reduction_ratio_ /
                                        (traj_period_ * lead_));
                    // ROS_INFO("Velocity: %d, Position: %d", cmd_vel, cmd_pos);

                    motor_data.pub_cmd_.cmd.Data[2] = static_cast<unsigned char>((cmd_vel >> 8) & 0xff);
                    motor_data.pub_cmd_.cmd.Data[3] = static_cast<unsigned char>(cmd_vel & 0xff);
                    motor_data.pub_cmd_.cmd.Data[4] = static_cast<unsigned char>((cmd_pos >> 24) & 0xff);
                    motor_data.pub_cmd_.cmd.Data[5] = static_cast<unsigned char>((cmd_pos >> 16) & 0xff);
                    motor_data.pub_cmd_.cmd.Data[6] = static_cast<unsigned char>((cmd_pos >> 8) & 0xff);
                    motor_data.pub_cmd_.cmd.Data[7] = static_cast<unsigned char>(cmd_pos & 0xff);

                    publishCmd(motor_data.pub_cmd_);
                }

                if (is_traj_end_) break;
            }

            ros::Duration(2.0).sleep();  // buffering time for archors moving back to zero position
            break;
        }
        // 靠近电机
        case 2: {
            std::string is_stall{};
            short target_vel = nh_.param("target_vel", 1000);  // 速度限制值(RPM)：-32768 ~ +32767
            for (auto& motor_data : motor_data_) {
                motor_data.pub_cmd_.cmd.ID = 0x004 | (motor_data.driver_id_ << 4);  // 速度模式下的参数指令，非广播
                motor_data.pub_cmd_.cmd.Data[0] = static_cast<unsigned char>((PWM_LIM >> 8) & 0xff);
                motor_data.pub_cmd_.cmd.Data[1] = static_cast<unsigned char>(PWM_LIM & 0xff);
                motor_data.pub_cmd_.cmd.Data[2] =
                    static_cast<unsigned char>(((target_vel * motor_data.direction_) >> 8) & 0xff);
                motor_data.pub_cmd_.cmd.Data[3] =
                    static_cast<unsigned char>((target_vel * motor_data.direction_) & 0xff);
                publishCmd(motor_data.pub_cmd_);
            }
            ROS_INFO("Press p to stop moving: ");
            while (ros::ok()) {
                getline(std::cin, is_stall);
                if (is_stall == "p") break;
            }

            for (int i = 0; i < 5; ++i) {
                for (auto& motor_data : motor_data_) {
                    for (int i = 0; i < 4; ++i) motor_data.pub_cmd_.cmd.Data[i] = 0;
                    publishCmd(motor_data.pub_cmd_);
                }
            }
            break;
        }
        // 远离电机
        case 3: {
            std::string is_stall{};
            short target_vel = -nh_.param("target_vel", 1000);  // 速度限制值(RPM)：-32768 ~ +32767

            for (auto& motor_data : motor_data_) {
                motor_data.pub_cmd_.cmd.ID = 0x004 | (motor_data.driver_id_ << 4);  // 速度模式下的参数指令，非广播
                motor_data.pub_cmd_.cmd.Data[0] = static_cast<unsigned char>((PWM_LIM >> 8) & 0xff);
                motor_data.pub_cmd_.cmd.Data[1] = static_cast<unsigned char>(PWM_LIM & 0xff);
                motor_data.pub_cmd_.cmd.Data[2] =
                    static_cast<unsigned char>(((target_vel * motor_data.direction_) >> 8) & 0xff);
                motor_data.pub_cmd_.cmd.Data[3] =
                    static_cast<unsigned char>((target_vel * motor_data.direction_) & 0xff);
                publishCmd(motor_data.pub_cmd_);
            }

            ROS_INFO("Press p to stop moving: ");
            while (ros::ok()) {
                getline(std::cin, is_stall);
                if (is_stall == "p") break;
            }

            for (int i = 0; i < 5; ++i) {
                for (auto& motor_data : motor_data_) {
                    for (int i = 0; i < 4; ++i) motor_data.pub_cmd_.cmd.Data[i] = 0;
                    publishCmd(motor_data.pub_cmd_);
                }
            }
            break;
        }
        default:
            break;
    }
}