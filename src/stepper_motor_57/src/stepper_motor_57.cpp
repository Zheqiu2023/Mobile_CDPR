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

#include <string>
#include <vector>

using namespace motor_57;

/**
 * @brief Construct a new MotorRun::MotorRun object
 */
MotorRun::MotorRun(ros::NodeHandle& nh) : nh_(nh)
{
    // Configure command waited to be sent
    XmlRpc::XmlRpcValue can_config;
    nh_.getParam("can_config", can_config);
    ROS_ASSERT(can_config.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    pub_cmd_.resize(can_config.size());
    is_reset_.resize(can_config.size());
    direction_.resize(can_config.size());
    auto iter = can_config.begin();
    for (int i = 0; i < can_config.size(); ++i)
    {
        is_reset_[i] = false;
        ROS_ASSERT(iter->second.hasMember("dev_ind") && iter->second.hasMember("can_ind") &&
                   iter->second.hasMember("can_id") && iter->second.hasMember("direction"));
        direction_[i] = static_cast<int>(iter->second["direction"]);
        pub_cmd_[i].dev_ind = static_cast<unsigned int>((int)iter->second["dev_ind"]);
        pub_cmd_[i].can_ind = static_cast<unsigned int>((int)iter->second["can_ind"]);
        pub_cmd_[i].cmd.ID =
            static_cast<unsigned int>(int(iter->second["can_id"]));  // CAN frame ID, same as CAN driver's address
        pub_cmd_[i].cmd.SendType = 1;  // Single send (sends only once, does not automatically retransmit after a failed
                                       // send, CAN bus generates only one frame of data)
        pub_cmd_[i].cmd.RemoteFlag = 0;  // 0 for data frame, 1 for remote frame
        pub_cmd_[i].cmd.ExternFlag = 0;  // 0 for standard frame, 1 for expanded frame
        pub_cmd_[i].cmd.DataLen = 8;     // Data length 8 bytes
        pub_cmd_[i].cmd.Data[0] =
            static_cast<unsigned char>(pub_cmd_[i].cmd.ID >> 3);  // high 8 bits of CAN driver's address (11bits)
        pub_cmd_[i].cmd.Data[1] =
            static_cast<unsigned char>(pub_cmd_[i].cmd.ID << 5);  // Lower 3 bits of the CAN driver's address, the last
                                                                  // five bits are normally set to 0.
        ++iter;
    }

    name_space_ = nh_.getNamespace();
    pub_ = nh_.advertise<cdpr_bringup::CanCmd>("/usbcan/motor_57", 10);
    sub_ = nh_.subscribe<cdpr_bringup::CanFrame>("/usbcan/can_pub", 10, boost::bind(&MotorRun::recvCallback, this, _1));

    ros::Duration(2.0).sleep();  // Sleep for 1s to ensure that the first message sent is received by USBCAN
}

void MotorRun::recvCallback(const cdpr_bringup::CanFrame::ConstPtr& msg)
{
    if (0xc1 == msg->ID || 0xc2 == msg->ID || 0xc3 == msg->ID || 0xc4 == msg->ID)
        if (msg->Data[2] == 0x41 && msg->Data[7] == 0)
            is_reset_[msg->ID - (unsigned int)0xc1] = true;
    // is_reset_[0] = true;
}

void MotorRun::publishCmd(const cdpr_bringup::CanCmd& cmd_struct)
{
    pub_.publish(cmd_struct);
}

void MotorRun::setCmd(cdpr_bringup::CanFrame& cmd, StepperMotorRunMode cmd_mode, std::vector<int>& data_vec)
{
    switch (cmd_mode)
    {
        case StepperMotorRunMode::RESET:
            cmd.Data[2] = static_cast<unsigned char>((CMD_REQUEST << 5) | 0X01);  // Reset command (return required)
            cmd.Data[7] = 1;                                                      // When reset complete, return command
            break;
        case StepperMotorRunMode::POS:
            cmd.Data[2] =
                static_cast<unsigned char>((CMD_REQUEST << 5) | 0X02);  // Positioning command (return required)
            cmd.Data[7] = 1;                                            // When in place, return command
            break;
        case StepperMotorRunMode::VEL_FORWARD:
            cmd.Data[2] =
                static_cast<unsigned char>((CMD_REQUEST << 5) | 0X03);  // Forward rotating command (return required)
            cmd.Data[7] = 3;  // The command returns after turning IntDate step in the positive direction.
            break;
        case StepperMotorRunMode::VEL_REVERSE:
            cmd.Data[2] = static_cast<unsigned char>((CMD_REQUEST << 5) | 0X04);  // Reverse command (return required)
            cmd.Data[7] = 3;  // The command returns after turning IntDate step in the opposite direction.
            break;
        case StepperMotorRunMode::STALL:
            cmd.Data[2] = static_cast<unsigned char>((CMD_REQUEST << 5) | 0X05);  // Stop command (return required)
            cmd.Data[7] = 1;  // Deceleration till stop, return command
            break;
        default:
            ROS_WARN_NAMED(name_space_, "Motor run mode error!");
            break;
    }
    for (size_t i = 3; i < 7; ++i)
    {
        cmd.Data[i] = static_cast<unsigned char>(data_vec[i - 3]);
    }
}

void MotorRun::getPos(cdpr_bringup::CanCmd& cmd_struct)
{
    cmd_struct.cmd.Data[2] =
        static_cast<unsigned char>((CMD_REQUEST << 5) | 0);  // Communication test command (return current position)
    for (size_t i = 3; i < 8; ++i)
    {
        cmd_struct.cmd.Data[i] = 0;
    }
    publishCmd(cmd_struct);
}

void MotorRun::run()
{
    std::string is_stall{};
    int run_type = nh_.param("motor_run_type", 0);
    std::vector<int> data_vec(4, 0);

    switch (run_type)
    {
        case 0: {  // position
            ROS_INFO_NAMED(name_space_, "Reset before localization!");
            // Reset: find zero position
            while (ros::ok())
            {
                std::size_t cnt = 0;
                for (size_t i = 0; i < pub_cmd_.size(); ++i)
                {
                    if (true == is_reset_[i])
                    {
                        ++cnt;
                        continue;
                    }
                    setCmd(pub_cmd_[i].cmd, StepperMotorRunMode::RESET, data_vec);
                    publishCmd(pub_cmd_[i]);
                }
                if (pub_cmd_.size() == cnt)
                    break;
                ros::spinOnce();
            }
            sleep(1);
            ROS_INFO_NAMED(name_space_, "Reset done!");
            // Run to the specified position
            nh_.getParam("target_data/target_pos_arr", data_vec);
            for (size_t i = 0; i < pub_cmd_.size(); ++i)
            {
                if (direction_[i] == 1)
                    setCmd(pub_cmd_[i].cmd, StepperMotorRunMode::VEL_REVERSE, data_vec);
                else if (direction_[i] == -1)
                    setCmd(pub_cmd_[i].cmd, StepperMotorRunMode::VEL_FORWARD, data_vec);
                publishCmd(pub_cmd_[i]);
            }

            ros::spin();
            break;
        }
        case 1: {  // foward rotation
            nh_.getParam("target_data/target_vel_forward_arr", data_vec);
            ROS_INFO_NAMED(name_space_, "Forward rotation!");

            for (auto& pub_cmd : pub_cmd_)
            {
                setCmd(pub_cmd.cmd, StepperMotorRunMode::VEL_FORWARD, data_vec);
                publishCmd(pub_cmd);
            }
            ROS_INFO_NAMED(name_space_, "Press p to stop rotation: ");
            while (ros::ok())
            {
                getline(std::cin, is_stall);
                if (is_stall == "p")
                    break;
            }
            for (auto& pub_cmd : pub_cmd_)
            {
                setCmd(pub_cmd.cmd, StepperMotorRunMode::STALL, data_vec);
                publishCmd(pub_cmd);
            }
            break;
        }
        case 2: {  // reverse rotation
            nh_.getParam("target_data/target_vel_reverse_arr", data_vec);
            ROS_INFO_NAMED(name_space_, "Reverse rotation!");

            for (auto& pub_cmd : pub_cmd_)
            {
                setCmd(pub_cmd.cmd, StepperMotorRunMode::VEL_REVERSE, data_vec);
                publishCmd(pub_cmd);
            }
            ROS_INFO_NAMED(name_space_, "Press p to stop rotation: ");
            while (ros::ok())
            {
                getline(std::cin, is_stall);
                if (is_stall == "p")
                    break;
            }
            for (auto& pub_cmd : pub_cmd_)
            {
                setCmd(pub_cmd.cmd, StepperMotorRunMode::STALL, data_vec);
                publishCmd(pub_cmd);
            }
            break;
        }
        case 3: {  // configure motor parameters
            XmlRpc::XmlRpcValue value;
            nh_.getParam("operating_param", value);
            auto iter = value.begin();

            for (size_t i = 0; i < pub_cmd_.size(); ++i)
            {
                readParam(pub_cmd_[i]);
                writeParam(pub_cmd_[i], iter->second);
                ++iter;
            }
            break;
        }
        default:
            ROS_WARN_NAMED(name_space_, "Motor control type error!");
            break;
    }
}

/**
 * @brief Sending operation parameters and save it to flash, data will not be lost even if power down, so only need to
 * send once.
 */
void MotorRun::writeParam(cdpr_bringup::CanCmd& cmd_struct, XmlRpc::XmlRpcValue& value)
{
    ROS_ASSERT(value["plus_start_time"].size() == 5 && value["plus_constant_time"].size() == 5 &&
               value["acc_steps"].size() == 5 && value["acc_cof"].size() == 5 && value["sub_divide"].size() == 5 &&
               value["reset_mode"].size() == 5 && value["phase_current"].size() == 5 && value["can_id"].size() == 5);

    // dirty code for loading and writing parameters
    ROS_INFO_NAMED(name_space_, "Sending operating parameters!");
    cmd_struct.cmd.Data[2] = static_cast<unsigned char>((CMD_REQUEST << 5) | 0X13);  // 运行参数保存到内存
    for (int j = 0; j < value["plus_start_time"].size(); ++j)
    {
        // 小端模式：低地址存放低位
        cmd_struct.cmd.Data[j + 3] = static_cast<unsigned char>(int(value["plus_start_time"][j]));
    }
    publishCmd(cmd_struct);
    usleep(100000);
    for (int j = 0; j < value["plus_constant_time"].size(); ++j)
    {
        cmd_struct.cmd.Data[j + 3] = static_cast<unsigned char>(int(value["plus_constant_time"][j]));
    }
    publishCmd(cmd_struct);
    usleep(100000);
    for (int j = 0; j < value["acc_steps"].size(); ++j)
    {
        cmd_struct.cmd.Data[j + 3] = static_cast<unsigned char>(int(value["acc_steps"][j]));
    }
    publishCmd(cmd_struct);
    usleep(100000);
    for (int j = 0; j < value["acc_cof"].size(); ++j)
    {
        cmd_struct.cmd.Data[j + 3] = static_cast<unsigned char>(int(value["acc_cof"][j]));
    }
    publishCmd(cmd_struct);
    usleep(100000);
    for (int j = 0; j < value["sub_divide"].size(); ++j)
    {
        cmd_struct.cmd.Data[j + 3] = static_cast<unsigned char>(int(value["sub_divide"][j]));
    }
    publishCmd(cmd_struct);
    usleep(100000);
    for (int j = 0; j < value["reset_mode"].size(); ++j)
    {
        cmd_struct.cmd.Data[j + 3] = static_cast<unsigned char>(int(value["reset_mode"][j]));
    }
    publishCmd(cmd_struct);
    usleep(100000);
    for (int j = 0; j < value["phase_current"].size(); ++j)
    {
        cmd_struct.cmd.Data[j + 3] = static_cast<unsigned char>(int(value["phase_current"][j]));
    }
    publishCmd(cmd_struct);
    usleep(100000);
    for (int j = 0; j < value["can_id"].size(); ++j)
    {
        cmd_struct.cmd.Data[j + 3] = static_cast<unsigned char>(int(value["can_id"][j]));
    }
    publishCmd(cmd_struct);
    usleep(100000);

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
void MotorRun::readParam(cdpr_bringup::CanCmd& cmd_struct)
{
    cmd_struct.cmd.Data[2] = static_cast<unsigned char>((CMD_REQUEST << 5) | 0X13);  // 运行参数读取
    for (size_t i = 3; i < cmd_struct.cmd.DataLen; ++i)
    {
        cmd_struct.cmd.Data[i] = 0;
    }
    // dirty code for reading parameters
    cmd_struct.cmd.Data[7] = 17;
    publishCmd(cmd_struct);
    usleep(100000);
    cmd_struct.cmd.Data[7] = 15;
    publishCmd(cmd_struct);
    usleep(100000);
    cmd_struct.cmd.Data[7] = 1;
    publishCmd(cmd_struct);
    usleep(100000);
    cmd_struct.cmd.Data[7] = 19;
    publishCmd(cmd_struct);
    usleep(100000);
    cmd_struct.cmd.Data[7] = 11;
    publishCmd(cmd_struct);
    usleep(100000);
    cmd_struct.cmd.Data[7] = 13;
    publishCmd(cmd_struct);
    usleep(100000);
    cmd_struct.cmd.Data[7] = 23;
    publishCmd(cmd_struct);
    usleep(100000);
    cmd_struct.cmd.Data[7] = 9;
    publishCmd(cmd_struct);
    usleep(100000);
}
