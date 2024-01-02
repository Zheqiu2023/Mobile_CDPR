/**
 * @File Name: maxon_re35.hpp
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

#include <mutex>

#include "cdpr_bringup/CanCmd.h"
#include "cdpr_bringup/CanFrame.h"
#include "cdpr_bringup/TrajCmd.h"
#include "cdpr_bringup/usb_can/controlcan.h"

namespace maxon_re35 {
constexpr unsigned short PWM_LIM = 5000;  // pwm限制值：0~5000，供电电压24V，额定电压48V

struct MotorData {
    int driver_id_, direction_;
    double target_pos_, last_pos_;
    cdpr_bringup::CanCmd pub_cmd_;
};

class MotorDriver {
   public:
    MotorDriver(ros::NodeHandle& nh);
    void run();

   private:
    void init(const int& run_mode);
    void publishCmd(const cdpr_bringup::CanCmd& cmd_struct);

    void cmdCableLengthCB(const cdpr_bringup::TrajCmd::ConstPtr& length);
    void motorStateCB(const cdpr_bringup::CanFrame::ConstPtr& state);

    int reduction_ratio_ = 0, encoder_lines_num_ = 0;
    double reel_diameter_ = 0.0, traj_period_ = 0.0;

    bool is_traj_end_ = false;
    std::vector<MotorData> motor_data_{};

    ros::NodeHandle nh_;
    ros::V_Publisher pubs_;
    ros::V_Subscriber subs_;

    std::mutex mutex_;
};
}  // namespace maxon_re35
