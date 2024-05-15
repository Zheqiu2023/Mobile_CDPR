/**
 * @File Name: cable_archor.hpp
 * @brief
 * @author Zhe Qiu (zheqiu2021@163.com)
 * @version 0.1
 * @date 2024-01-30
 *
 * *  ***********************************************************************************
 * *  @copyright Copyright (c) 2023  by Zhe Qiu. All rights reserved.
 * *  Use of this source code is governed by the BSD 3-Clause license, see LICENSE.
 * *  ***********************************************************************************
 */
#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <stdio.h>
#include <termio.h>

#include "cdpr_bringup/usb_can/controlcan.h"

namespace cable_archor
{
constexpr unsigned short PWM_LIM = 5000;  // pwm限制值
// maxon re35电机运行模式
enum class RunMode
{
    VEL = 0X03,     // 速度模式
    VEL_POS = 0X05  // 速度位置模式
};

struct CanCmd
{
    int dev_ind;
    int can_ind;
    VCI_CAN_OBJ cmd;
};

class BaseDriver
{
  protected:
    void sendCmd(CanCmd& cmd_struct);
    void setVel(VCI_CAN_OBJ& cmd, const int& driver_id, const int& target_vel);
    void setVelPos(VCI_CAN_OBJ& cmd, const int& driver_id, const int& target_vel, const int& target_pos);

    int reduction_ratio_ = 0, encoder_lines_num_ = 0;
    double traj_period_ = 0.0;
    bool start_traj_tracking_ = false;

    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::V_Subscriber subs_;
};

struct CableData
{
    int driver_id_, direction_;
    double target_pos_, last_pos_;
    CanCmd pub_cmd_;
};

class CableDriver : public BaseDriver
{
    friend class UsbCan;

  public:
    CableDriver(ros::NodeHandle& nh);
    void run();
    void creatThread();

  private:
    static void* threadFunc(void* arg);
    void init(const int& run_mode);
    int scanKeyboard();

    void cmdCableLengthCB(const std_msgs::Float64MultiArray::ConstPtr& length);
    void startTrajCB(const std_msgs::Bool::ConstPtr& flag);

    double reel_diameter_ = 0.0;
    std::vector<double> traj_{};
    std::vector<CableData> motor_data_{};
};

struct ArchorData
{
    bool is_reset_;
    int driver_id_, direction_;
    double target_pos_, last_pos_;
    CanCmd pub_cmd_;
};

class ArchorDriver : public BaseDriver
{
    friend class UsbCan;

  public:
    ArchorDriver(ros::NodeHandle& nh);
    void run();
    void creatThread();

  private:
    static void* threadFunc(void* arg);
    void init(RunMode mode, const int& period);

    void cmdPosCallback(const std_msgs::Float64MultiArray::ConstPtr& pos);
    void startTrajCB(const std_msgs::Bool::ConstPtr& flag);

    int lead_ = 0;
    std::vector<double> traj_{};
    std::vector<ArchorData> motor_data_{};
};

enum class MotorType
{
    STEPPER_MOTOR,
    MAXON_RE35
};

class UsbCan
{
  public:
    UsbCan(ros::NodeHandle& nh);
    ~UsbCan();
    void can_receive(CableDriver& cable_driver, ArchorDriver& archor_driver);

  private:
    void initCAN(const int& dev_type, const int& dev_ind, const int& can_ind, const MotorType& m_type);
    void setCANParam(const MotorType& m_type);
    void printMsg(uint32_t dev_ind, uint32_t can_ind, const VCI_CAN_OBJ& msg) const;

    ros::NodeHandle nh_;
    ros::V_Publisher archor_pos_pubs_, cable_pos_pubs_;

    std_msgs::Float64 archor_pos_, cable_pos_;
    VCI_INIT_CONFIG config_;                     // 初始化参数，参考二次开发函数库说明书
    std::array<VCI_CAN_OBJ, 3000> recv_msgs_{};  // 接收缓存，设为3000为佳
};
}  // namespace cable_archor