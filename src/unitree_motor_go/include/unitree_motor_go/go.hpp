/**
 * @File Name: go.hpp
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

namespace motor_go
{
class GoControl
{
  public:
    GoControl(ros::NodeHandle nh);
    void init(std::vector<SerialPort*>& port);
    void drive(std::vector<SerialPort*>& port);
    void stall(std::vector<SerialPort*>& port);
    void operator()();

  private:
    void setCmd(const std::vector<float>& cmd);
    void setCommandCB(const std_msgs::Float64MultiArray::ConstPtr& cmd_vel);

    int motor_num_ = 0;
    std::vector<int> id_{};
    std::vector<float> motor_zero_position_{};
    std::vector<MotorCmd> init_param_{}, motor_cmd_{};
    std::vector<MotorData> motor_recv_{};
    std::vector<std::string> serial_port_{};

    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::NodeHandle nh_;
};
}  // namespace motor_go