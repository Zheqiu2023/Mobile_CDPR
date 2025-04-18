/**
 * @File Name: movable_archor.hpp
 * @brief
 * @author Zhe Qiu (zheqiu2021@163.com)
 * @version 0.1
 * @date 2023-12-14
 *
 * *  ***********************************************************************************
 * *  @copyright Copyright (c) 2023  by Zhe Qiu. All rights reserved.
 * *  Use of this source code is governed by the BSD 3-Clause license, see LICENSE.
 * *  ***********************************************************************************
 */
#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <mutex>

#include "cdpr_bringup/CanCmd.h"
#include "cdpr_bringup/CanFrame.h"
#include "cdpr_bringup/TrajCmd.h"
#include "cdpr_bringup/usb_can/controlcan.h"

namespace movable_archor {
constexpr unsigned short PWM_LIM = 5000;  // pwm限制值

// maxon re35电机运行模式
enum class RunMode {
    VEL = 0X03,     // 速度模式
    VEL_POS = 0X05  // 速度位置模式
};

struct MotorData {
    bool is_reset_;
    int driver_id_, direction_;
    double target_pos_, last_pos_;
    cdpr_bringup::CanCmd pub_cmd_;
};

class ArchorDriver {
   public:
    ArchorDriver(ros::NodeHandle& nh);
    void run();

   private:
    void init(RunMode mode, const int& period);
    void publishCmd(const cdpr_bringup::CanCmd& cmd_struct);

    void cmdPosCallback(const cdpr_bringup::TrajCmd::ConstPtr& pos);
    void motorStateCB(const cdpr_bringup::CanFrame::ConstPtr& state);
    void resetStateCB(const cdpr_bringup::CanFrame::ConstPtr& state);

    int lead_ = 0, reduction_ratio_ = 0, encoder_lines_num_ = 0;
    double traj_period_ = 0.0;

    bool is_traj_end_ = false;
    std::vector<MotorData> motor_data_{};
    std_msgs::Float64 cur_pos_;

    ros::NodeHandle nh_;
    ros::V_Publisher pubs_, cur_pos_pubs_;
    ros::V_Subscriber subs_;

    std::mutex pos_mutex_, state_mutex_;
};
}  // namespace movable_archor
