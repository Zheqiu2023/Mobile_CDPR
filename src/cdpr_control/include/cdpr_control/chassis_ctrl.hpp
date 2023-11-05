#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>

#include "cdpr_bringup/ChassisCmd.h"
#include "cdpr_bringup/filters/filters.hpp"

namespace chassis_ctrl
{
struct Wheelset
{
    Vec2<double> position_;
    double steer_offset_, wheel_radius_;
};

class ChassisCtrl
{
  public:
    ChassisCtrl(ros::NodeHandle& nh);
    void update(const ros::Time& time);

  private:
    void moveJoint();
    void chassisCmdCB(const cdpr_bringup::ChassisCmd::ConstPtr& msg);
    void steerStateCB(const std_msgs::Float64MultiArray::ConstPtr& state);

    double timeout_{};
    std::string name_space_;
    std::vector<Wheelset> wheelsets_;
    RampFilter<double>*ramp_x_{}, *ramp_y_{}, *ramp_w_{};
    std_msgs::Float64MultiArray steer_cmd_{}, roll_cmd_{};

    geometry_msgs::Vector3 cmd_vel_{};
    std_msgs::Float64MultiArray steer_state_;
    cdpr_bringup::ChassisCmd chassis_cmd_;
    ros::Subscriber cmd_chassis_sub_, state_steer_sub_;
    ros::V_Publisher cmd_wheelset_pubs_;
    ros::NodeHandle nh_;
};
}  // namespace chassis_ctrl