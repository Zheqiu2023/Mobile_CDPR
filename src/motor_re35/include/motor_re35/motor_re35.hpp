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
#include <std_msgs/Float32.h>

#include "general_file/CanFrame.h"
#include "general_file/usb_can/controlcan.h"
#include "general_file/ctrl_algorithm.hpp"

namespace motor_re35
{
constexpr unsigned short PWM_LIM = 5000;  // pwm限制值：0~5000，若供电电压与额定电压一致，设为5000

class MsgBox
{
  public:
    MsgBox();
    void publishCmd(const general_file::CanFrame& cmd);
    void recvCANMsgs(const general_file::CanFrame::ConstPtr& msg);
    void recvTension(const std_msgs::Float32::ConstPtr& tension);
    float getTension();

  private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::V_Subscriber subs_;
    std::vector<float> tension_vec_{};
    float force_ = 0;
    int times_ = 0;
};

class MotorRun
{
  public:
    MotorRun();
    void run();
    void init();

  private:
    general_file::CanFrame pub_cmd_{};
    ctrl_algorithm::PID pid_;
    MsgBox msg_box_;
};
}  // namespace motor_re35
