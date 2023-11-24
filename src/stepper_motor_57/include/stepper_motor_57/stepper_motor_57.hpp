/**
 * @File Name: stepper_motor_57.hpp
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
#pragma once

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <mutex>

#include "cdpr_bringup/usb_can/controlcan.h"
#include "cdpr_bringup/CanCmd.h"
#include "cdpr_bringup/CanFrame.h"
#include "stepper_motor.hpp"

namespace motor_57
{
struct MotorData
{
    bool is_reset_;
    int direction_;
    float target_pos_;
    cdpr_bringup::CanCmd pub_cmd_;
};

class MotorDriver
{
  public:
    MotorDriver(ros::NodeHandle& nh);
    void run();

  private:
    void setCmd(cdpr_bringup::CanFrame& cmd, StepperMotorRunMode cmd_mode, const std::vector<int>& pos_vec);
    void publishCmd(const cdpr_bringup::CanCmd& cmd_struct);

    void recvPosCallback(const std_msgs::Float32MultiArray::ConstPtr& pos);
    void recvStateCallback(const cdpr_bringup::CanFrame::ConstPtr& state);

    void readParam(cdpr_bringup::CanCmd& cmd_struct);
    void writeParam(cdpr_bringup::CanCmd& cmd_struct, XmlRpc::XmlRpcValue& value);

    int lead_ = 0, sub_divide_ = 0;
    float step_angle_ = 0.0;

    std::vector<MotorData> motor_data_{};

    std::string name_space_{};
    ros::NodeHandle nh_;
    ros::V_Publisher pubs_;
    ros::V_Subscriber subs_;

    std::mutex mutex_;
};
}  // namespace motor_57
