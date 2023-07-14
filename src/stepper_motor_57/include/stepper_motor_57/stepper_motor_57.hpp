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

#include "controlcan.h"
#include "general_file/can_msgs.h"
#include "stepper_motor.hpp"

namespace motor_57
{
class MotorParam
{
  public:
    void readParam();
    void writeParam();
};

class MsgBox
{
  public:
    MsgBox();
    ~MsgBox();
    void publishCmd(const general_file::can_msgs& cmd);
    void recvCallback(const general_file::can_msgs::ConstPtr& msg);
    sem_t& getSemaphore();

  private:
    sem_t sem_trans_{};
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
};

class MotorRun
{
  public:
    void creatThread();
    static void* transmitFunc(void* arg);
    void setCmd(StepperMotorRunMode cmd_mode);
    void run();

  private:
    MsgBox msg_box_{};
    general_file::can_msgs pub_cmd_{};
};
}  // namespace motor_57
