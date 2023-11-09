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
#include <semaphore.h>

#include "cdpr_bringup/usb_can/controlcan.h"
#include "cdpr_bringup/CanFrame.h"
#include "stepper_motor.hpp"

namespace motor_57
{
class MotorRun
{
  public:
    MotorRun(ros::NodeHandle& nh);
    void run();

  private:
    void setCmd(StepperMotorRunMode cmd_mode, std::vector<int> data_vec);
    void publishCmd(const cdpr_bringup::CanFrame& cmd);
    void recvCallback(const cdpr_bringup::CanFrame::ConstPtr& msg);
    void readParam();
    void writeParam();

    bool is_reset_ = false;
    cdpr_bringup::CanFrame pub_cmd_{};

    std::string name_space_{};
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
};
}  // namespace motor_57
