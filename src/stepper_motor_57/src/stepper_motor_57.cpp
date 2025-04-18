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

#include <std_msgs/Bool.h>

#include <string>
#include <vector>

#include "usb_can/usb_can.hpp"

using namespace stepper_57;

MotorDriver::MotorDriver(ros::NodeHandle& nh) : nh_(nh), is_traj_end_(false) {
    // Configure command waited to be sent
    XmlRpc::XmlRpcValue cmd_config;
    name_space_ = nh_.getNamespace();
    if (!(nh_.getParam("cmd_config", cmd_config) && nh_.getParam("lead", lead_) &&
          nh_.getParam("step_angle", step_angle_) && nh_.getParam("sub_divide", sub_divide_)))
        ROS_ERROR("Some motor params are not given in namespace: '%s')", name_space_.c_str());
    ROS_ASSERT(cmd_config.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    for (auto iter = cmd_config.begin(); iter != cmd_config.end(); ++iter) {
        ROS_ASSERT(iter->second.hasMember("dev_ind") && iter->second.hasMember("can_ind") &&
                   iter->second.hasMember("can_id") && iter->second.hasMember("direction"));

        cdpr_bringup::CanCmd pub_cmd;
        pub_cmd.dev_ind = static_cast<unsigned int>((int)iter->second["dev_ind"]);
        pub_cmd.can_ind = static_cast<unsigned int>((int)iter->second["can_ind"]);
        pub_cmd.cmd.ID =
            static_cast<unsigned int>((int)iter->second["can_id"]);  // CAN frame ID, same as CAN driver's address
        pub_cmd.cmd.SendType = 1;    // Single send (sends only once, does not automatically retransmit after a failed
                                     // send, CAN bus generates only one frame of data)
        pub_cmd.cmd.RemoteFlag = 0;  // 0 for data frame, 1 for remote frame
        pub_cmd.cmd.ExternFlag = 0;  // 0 for standard frame, 1 for expanded frame
        pub_cmd.cmd.DataLen = 8;     // Data length 8 bytes
        pub_cmd.cmd.Data[0] =
            static_cast<unsigned char>(pub_cmd.cmd.ID >> 3);  // high 8 bits of CAN driver's address (11bits)
        pub_cmd.cmd.Data[1] =
            static_cast<unsigned char>(pub_cmd.cmd.ID << 5);  // Lower 3 bits of the CAN driver's address, the last five
                                                              // bits are normally set to 0.

        motor_data_.push_back(MotorData{.is_reset_ = false,
                                        .direction_ = static_cast<int>(iter->second["direction"]),
                                        .target_pos_ = 0.0,
                                        .pub_cmd_ = std::move(pub_cmd)});
    }

    pubs_.emplace_back(nh_.advertise<cdpr_bringup::CanCmd>("motor_cmd", 10));
    pubs_.emplace_back(nh_.advertise<std_msgs::Bool>("ready_state", 1));
    subs_.emplace_back(nh_.subscribe("/usbcan/motor_state", 10, &MotorDriver::motorStateCallback, this));
    subs_.emplace_back(nh_.subscribe("/archor_coor_z", 10, &MotorDriver::cmdPosCallback, this));

    ros::Duration(1.0).sleep();  // Sleep for 1s to ensure that the first message sent is received by USBCAN
}

void MotorDriver::cmdPosCallback(const cdpr_bringup::TrajCmd::ConstPtr& pos) {
    std::lock_guard<std::mutex> guard(mutex_);
    is_traj_end_ = pos->is_traj_end;
    for (size_t i = 0; i < motor_data_.size(); ++i) motor_data_[i].target_pos_ = pos->target[i];
    ROS_INFO("target pos: %.5f, %.5f, %.5f, %.5f", motor_data_[0].target_pos_, motor_data_[1].target_pos_,
             motor_data_[2].target_pos_, motor_data_[3].target_pos_);
}

void MotorDriver::motorStateCallback(const cdpr_bringup::CanFrame::ConstPtr& state) {
    if (state->Data[2] == 0x41 && state->Data[7] == 0) {
        for (auto& motor_data : motor_data_) {
            if (motor_data.pub_cmd_.cmd.ID == state->ID) motor_data.is_reset_ = true;  // reset successfully
        }
    }
}

void MotorDriver::publishCmd(const cdpr_bringup::CanCmd& cmd_struct) { pubs_[0].publish(cmd_struct); }

void MotorDriver::setCmd(cdpr_bringup::CanFrame& cmd, RunMode cmd_mode, const std::vector<int>& pos_vec) {
    switch (cmd_mode) {
        case RunMode::RESET:
            cmd.Data[2] = (CMD_REQUEST << 5) | 0X01;  // Reset command (return required)
            cmd.Data[7] = 1;                          // When reset complete, return command
            break;
        case RunMode::POS:
            cmd.Data[2] = (CMD_REQUEST << 5) | 0X02;  // Positioning command (return required)
            cmd.Data[7] = 1;                          // When in place, return command
            break;
        case RunMode::VEL_FORWARD:
            cmd.Data[2] = (CMD_REQUEST << 5) | 0X03;  // Forward rotating command (return required)
            cmd.Data[7] = 3;  // The command returns after turning IntDate step in the positive direction.
            break;
        case RunMode::VEL_REVERSE:
            cmd.Data[2] = (CMD_REQUEST << 5) | 0X04;  // Reverse command (return required)
            cmd.Data[7] = 3;  // The command returns after turning IntDate step in the opposite direction.
            break;
        case RunMode::STALL:
            cmd.Data[2] = (CMD_REQUEST << 5) | 0X05;  // Stop command (return required)
            cmd.Data[7] = 1;                          // Deceleration till stop, return command
            break;
        default:
            ROS_WARN_NAMED(name_space_, "Motor run mode error!");
            break;
    }
    for (size_t i = 3; i < 7; ++i) {
        cmd.Data[i] = static_cast<unsigned char>(pos_vec[i - 3]);
    }
}

void MotorDriver::run() {
    std::string is_stall{};
    int run_mode = nh_.param("run_mode", 0);
    std::vector<int> pos_vec(4, 0);

    switch (run_mode) {
        case 0: {  // position
            ROS_INFO_NAMED(name_space_, "Reset before localization!");
            // Reset: move to zero position
            while (ros::ok()) {
                for (auto& motor_data : motor_data_) {
                    if (true == motor_data.is_reset_) continue;
                    setCmd(motor_data.pub_cmd_.cmd, RunMode::RESET, pos_vec);
                    publishCmd(motor_data.pub_cmd_);
                }

                if (std::all_of(motor_data_.begin(), motor_data_.end(),
                                [](const MotorData& motor_data) {
                                    return motor_data.is_reset_;
                                }))  // all archores reset successfully
                {
                    subs_[0].shutdown();
                    break;
                }
            }

            std_msgs::Bool is_ready{};
            is_ready.data = true;
            pubs_[1].publish(is_ready);
            ROS_INFO_NAMED(name_space_, "Stepper motor57 reset done, ready to follow the trajectory!");

            // follow the trajectory
            while (ros::ok()) {
                std::lock_guard<std::mutex> guard(mutex_);
                for (auto& motor_data : motor_data_) {
                    int cmd_pos =
                        std::round((motor_data.target_pos_ * 1000 * 360 / lead_) / (step_angle_ / sub_divide_));
                    if (motor_data.direction_ == 1) {
                        for (size_t i = 0; i < pos_vec.size(); ++i) pos_vec[i] = 0xff - (cmd_pos >> (8 * i)) & 0xff;
                    } else if (motor_data.direction_ == -1) {
                        for (size_t i = 0; i < pos_vec.size(); ++i) pos_vec[i] = (cmd_pos >> (8 * i)) & 0xff;
                    }

                    setCmd(motor_data.pub_cmd_.cmd, RunMode::POS, pos_vec);
                    publishCmd(motor_data.pub_cmd_);
                }
                if (is_traj_end_) break;
            }

            ros::Duration(2.0).sleep();  // buffering time for motors moving back to zero position
            break;
        }
        case 1: {  // close to motor
            nh_.getParam("target_data/target_vel_close_arr", pos_vec);
            ROS_INFO_NAMED(name_space_, "Move close to motor!");

            ROS_INFO_NAMED(name_space_, "Press p to stop moving: ");
            while (ros::ok()) {
                for (auto& motor_data : motor_data_) {
                    if (motor_data.direction_ == 1)
                        setCmd(motor_data.pub_cmd_.cmd, RunMode::VEL_FORWARD, pos_vec);
                    else if (motor_data.direction_ == -1)
                        setCmd(motor_data.pub_cmd_.cmd, RunMode::VEL_REVERSE, pos_vec);
                    publishCmd(motor_data.pub_cmd_);
                }
                getline(std::cin, is_stall);
                if (is_stall == "p") break;
            }

            for (auto& motor_data : motor_data_) {
                setCmd(motor_data.pub_cmd_.cmd, RunMode::STALL, pos_vec);
                publishCmd(motor_data.pub_cmd_);
            }
            break;
        }
        case 2: {  // away from motor
            nh_.getParam("target_data/target_vel_away_arr", pos_vec);
            ROS_INFO_NAMED(name_space_, "Move away from motor!");

            for (auto& motor_data : motor_data_) {
                if (motor_data.direction_ == 1)
                    setCmd(motor_data.pub_cmd_.cmd, RunMode::VEL_REVERSE, pos_vec);
                else if (motor_data.direction_ == -1)
                    setCmd(motor_data.pub_cmd_.cmd, RunMode::VEL_FORWARD, pos_vec);
                publishCmd(motor_data.pub_cmd_);
            }

            ROS_INFO_NAMED(name_space_, "Press p to stop moving: ");
            while (ros::ok()) {
                getline(std::cin, is_stall);
                if (is_stall == "p") break;
            }

            for (auto& motor_data : motor_data_) {
                setCmd(motor_data.pub_cmd_.cmd, RunMode::STALL, pos_vec);
                publishCmd(motor_data.pub_cmd_);
            }
            break;
        }
        case 3: {  // reset
            ROS_INFO_NAMED(name_space_, "Reset!");
            // Reset: move to zero position
            while (ros::ok()) {
                for (auto& motor_data : motor_data_) {
                    if (true == motor_data.is_reset_) continue;
                    setCmd(motor_data.pub_cmd_.cmd, RunMode::RESET, pos_vec);
                    publishCmd(motor_data.pub_cmd_);
                }

                if (std::all_of(motor_data_.begin(), motor_data_.end(),
                                [](const MotorData& motor_data) {
                                    return motor_data.is_reset_;
                                }))  // all archores reset successfully
                {
                    subs_[0].shutdown();
                    break;
                }
            }
            break;
        }
        case 4: {  // configure motor parameters
            XmlRpc::XmlRpcValue value;
            nh_.getParam("operating_param", value);
            auto iter = value.begin();

            for (auto& motor_data : motor_data_) {
                readParam(motor_data.pub_cmd_);
                writeParam(motor_data.pub_cmd_, iter->second);
                ++iter;
            }
            break;
        }
        default: {
            ROS_WARN_NAMED(name_space_, "Motor control type error!");
            break;
        }
    }
}

/**
 * @brief Sending operation parameters and save it to flash, data will not be lost even if power down, so only need to
 * send once.
 */
void MotorDriver::writeParam(cdpr_bringup::CanCmd& cmd_struct, XmlRpc::XmlRpcValue& value) {
    std::vector<std::string> param_type{"plus_start_time", "plus_constant_time", "acc_steps",     "acc_cof",
                                        "sub_divide",      "reset_mode",         "phase_current", "can_id"};

    ROS_INFO_NAMED(name_space_, "Sending operating parameters!");

    cmd_struct.cmd.Data[2] = static_cast<unsigned char>((CMD_REQUEST << 5) | 0X13);  // 运行参数保存到内存
    for (const auto& type : param_type) {
        ROS_ASSERT(value[type].size() == 5);
        for (int j = 0; j < value[type].size(); ++j) {
            // 小端模式：低地址存放低位
            cmd_struct.cmd.Data[j + 3] = static_cast<unsigned char>(int(value[type][j]));
        }
        publishCmd(cmd_struct);
        usleep(100000);
    }

    ROS_INFO_NAMED(name_space_, "********************************");
    cmd_struct.cmd.Data[2] =
        static_cast<unsigned char>((CMD_REQUEST << 5) | 0X14);  // 运行参数保存到flash，掉电数据不丢失
    publishCmd(cmd_struct);
    usleep(100000);
    ROS_INFO_NAMED(name_space_, "Parameters have been saved in flash!");
}

/**
 * @brief Reading motor operating parameters from flash
 */
void MotorDriver::readParam(cdpr_bringup::CanCmd& cmd_struct) {
    cmd_struct.cmd.Data[2] = static_cast<unsigned char>((CMD_REQUEST << 5) | 0X13);  // 运行参数读取
    for (size_t i = 3; i < cmd_struct.cmd.DataLen; ++i) {
        cmd_struct.cmd.Data[i] = 0;
    }

    std::vector<unsigned char> bytedata{17, 15, 1, 19, 11, 13, 23, 9};
    for (const auto& i : bytedata) {
        cmd_struct.cmd.Data[7] = i;
        publishCmd(cmd_struct);
        usleep(100000);
    }
}
