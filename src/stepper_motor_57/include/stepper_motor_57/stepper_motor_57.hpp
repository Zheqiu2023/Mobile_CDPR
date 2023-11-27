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
#include <mutex>

#include "cdpr_bringup/usb_can/controlcan.h"
#include "cdpr_bringup/CanCmd.h"
#include "cdpr_bringup/CanFrame.h"
#include "cdpr_bringup/TrajCmd.h"

namespace stepper_57
{
// 命令码(功能码)类型
constexpr unsigned char CMD_BROADCAST = 0x00;  // 广播命令(不需要返回)
constexpr unsigned char CMD_REQUEST = 0x01;    // 请求命令(需要返回)
constexpr unsigned char CMD_ACK = 0x02;        // 请求命令的正确返回
constexpr unsigned char CMD_NAK = 0x03;        // 请求命令的不正确返回
constexpr unsigned char CMD_NOCMD = 0x05;      // 无此命令码
constexpr unsigned char CMD_ER_PARA = 0x06;    // 命令参数错误

// 步进电机运行模式
enum class RunMode
{
    RESET,                // 复位模式
    POS,                  // 定位模式
    VEL_FORWARD,          // 正转，速度模式
    VEL_REVERSE,          // 反转，速度模式
    STALL,                // 停转
    PARAM2MEMORY = 0x13,  // 运行参数设置到内存及读取内存
    PARAM2FLASH = 0x14    // 运行参数从内存存储到 flash
};

struct MotorData
{
    bool is_reset_;
    int direction_;
    double target_pos_;
    cdpr_bringup::CanCmd pub_cmd_;
};

class MotorDriver
{
  public:
    MotorDriver(ros::NodeHandle& nh);
    void run();

  private:
    void setCmd(cdpr_bringup::CanFrame& cmd, RunMode cmd_mode, const std::vector<int>& pos_vec);
    void publishCmd(const cdpr_bringup::CanCmd& cmd_struct);

    void cmdPosCallback(const cdpr_bringup::TrajCmd::ConstPtr& pos);
    void motorStateCallback(const cdpr_bringup::CanFrame::ConstPtr& state);

    void readParam(cdpr_bringup::CanCmd& cmd_struct);
    void writeParam(cdpr_bringup::CanCmd& cmd_struct, XmlRpc::XmlRpcValue& value);

    int lead_ = 0, sub_divide_ = 0;  // 导程，细分
    double step_angle_ = 0.0;        // 步距角

    bool is_traj_end_;
    std::vector<MotorData> motor_data_{};

    std::string name_space_{};
    ros::NodeHandle nh_;
    ros::V_Publisher pubs_;
    ros::V_Subscriber subs_;

    std::mutex mutex_;
};
}  // namespace stepper_57
