/**
 * @File Name: motor_re35.hpp
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
#include <set>
#include <std_msgs/Float64.h>

#include "general_file/can_msgs.h"
#include "controlcan.h"
#include "ctrl_algorithm.hpp"

namespace motor_re35
{
constexpr unsigned short PWM_LIM = 5000;  // pwm限制值：0~5000，若供电电压与额定电压一致，设为5000

class MsgBox
{
  public:
    MsgBox();
    void publishCmd(const general_file::can_msgs& cmd);
    void recvCANMsgs(const general_file::can_msgs::ConstPtr& msg);
    void recvTension(const std_msgs::Float64::ConstPtr& tension);
    double getTension();

  private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::V_Subscriber subs_;
    std::vector<double> tension_vec_{};
    double force_ = 0;
    int times = 0;
};

class MotorRun
{
  public:
    MotorRun();
    void run();
    void init();

  private:
    general_file::can_msgs pub_cmd_{};
    ctrl_algorithm::PID pid_;
    MsgBox msg_box_;
};
}  // namespace motor_re35
