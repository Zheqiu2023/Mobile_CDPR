/**
 * @File Name: a1.hpp
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
#include <std_msgs/Float64MultiArray.h>

#include "serialPort/SerialPort.h"

namespace motor_a1
{
struct MotorParam
{
    int serial_num_;
    double zero_position_, last_pos_;
    MotorCmd init_param_, motor_cmd_;
    MotorData motor_recv_;

    SerialPort* port_;
};

class A1Control
{
  public:
    A1Control(ros::NodeHandle nh);
    void init();
    void drive();
    void stall();
    void operator()();

  private:
    void setCmd(const std::vector<double>& cmd);
    void setCommandCB(const std_msgs::Float64MultiArray::ConstPtr& cmd_vel);
    void setPosCB(const std_msgs::Float64MultiArray::ConstPtr& pos);

    double reduction_ratio_ = 0.0, wheel_radius_ = 0.0;
    std::vector<int> id_{};
    std::vector<std::string> port_name_{};
    std::vector<MotorParam> motor_param_;

    ros::V_Subscriber subs_;
    ros::Publisher pub_;
    ros::NodeHandle nh_;
};
}  // namespace motor_a1