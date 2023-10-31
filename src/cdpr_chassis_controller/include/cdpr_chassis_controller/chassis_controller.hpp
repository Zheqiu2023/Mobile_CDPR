/**
 * @File Name: chassis_controller.hpp
 * @brief
 * @author Zhe Qiu (zheqiu2021@163.com)
 * @version 0.1
 * @date 2023-10-22
 *
 * *  ***********************************************************************************
 * *  @copyright Copyright (c) 2023  by Zhe Qiu. All rights reserved.
 * *  Use of this source code is governed by the BSD 3-Clause license, see LICENSE.
 * *  ***********************************************************************************
 */
#pragma once

#include <boost/scoped_ptr.hpp>
#include <ros/node_handle.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>

#include <control_toolbox/pid.h>
#include <realtime_tools/realtime_buffer.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <effort_controllers/joint_position_controller.h>
#include <effort_controllers/joint_velocity_controller.h>

#include "general_file/ChassisCmd.h"
#include "general_file/eigen_types.hpp"
#include "general_file/filters/filters.hpp"

namespace cdpr_chassis_controller
{
struct Wheelset
{
    Vec2<double> position_;
    double steer_offset_, wheel_radius_;
    effort_controllers::JointPositionController* ctrl_steer_;
    effort_controllers::JointVelocityController* ctrl_roll_;
};

class ChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
  public:
    ChassisController() = default;
    ~ChassisController();
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
    void update(const ros::Time& time, const ros::Duration& period) override;

  private:
    void moveJoint(const ros::Time& time, const ros::Duration& period);
    void cmdChassisCallback(const general_file::ChassisCmd::ConstPtr& msg);
    void publishJointState(const ros::Time& time);

    double wheel_radius_{}, publish_rate_{}, timeout_{};
    std::string name_space_;
    std::vector<Wheelset> wheelsets_;
    RampFilter<double>*ramp_x_{}, *ramp_y_{}, *ramp_w_{};

    geometry_msgs::Vector3 cmd_vel_{};
    general_file::ChassisCmd chassis_cmd_;
    realtime_tools::RealtimeBuffer<general_file::ChassisCmd> cmd_rt_buffer_;
    ros::Subscriber cmd_chassis_sub_;

    control_toolbox::Pid pid_follow_;
    sensor_msgs::JointState joint_state_;
    std::vector<hardware_interface::JointHandle> joint_handles_{};
    hardware_interface::EffortJointInterface* effort_joint_interface_{};

    ros::Time last_publish_time_;
    boost::scoped_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState>> chassis_state_publisher_;
};
}  // namespace cdpr_chassis_controller