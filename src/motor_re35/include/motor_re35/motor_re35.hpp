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

#include "cdpr_bringup/CanFrame.h"
#include "cdpr_bringup/CanCmd.h"
#include "cdpr_bringup/usb_can/controlcan.h"

namespace motor_re35
{
constexpr unsigned short PWM_LIM = 5000;  // pwm限制值：0~5000，若供电电压与额定电压一致，设为5000

class MotorRun
{
  public:
    MotorRun(ros::NodeHandle& nh);
    void run();

  private:
    void init(const int& run_mode);
    void publishCmd(const cdpr_bringup::CanCmd& cmd_struct);

    std::vector<int> driver_id_{};
    std::vector<cdpr_bringup::CanCmd> pub_cmd_{};

    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
};
}  // namespace motor_re35
