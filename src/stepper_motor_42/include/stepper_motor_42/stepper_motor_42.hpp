/**
 * @File Name: stepper_motor_42.hpp
 * @brief
 * @author Zhe Qiu (zheqiu2021@163.com)
 * @version 0.1
 * @date 2023-05-29
 *
 *  ***********************************************************************************
 *  @copyright Copyright (c) 2023  by Zhe Qiu. All rights reserved.
 *  Use of this source code is governed by the BSD 3-Clause license, see LICENSE.
 *  ***********************************************************************************
 */
#include <ros/ros.h>
#include <semaphore.h>

#include "controlcan.h"
#include "general_file/can_msgs.h"
#include "stepper_motor.hpp"

namespace motor_42
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
    void setCmd(StepperMotorRunMode cmd_mode);
    void run();
    void publishCmd();
    void recvCallback(const general_file::can_msgs::ConstPtr& msg);

  private:
    general_file::can_msgs pub_cmd_{};
    VCI_CAN_OBJ send_cmd_{}, recv_msgs_{};
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

  private:
    MsgBox msg_box_{};
};
}  // namespace motor_42