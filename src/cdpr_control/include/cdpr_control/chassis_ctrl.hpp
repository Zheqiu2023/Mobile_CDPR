#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/TwistStamped.h>

#include "cdpr_bringup/filters/filters.hpp"

namespace chassis_ctrl
{
struct Wheelset
{
    Vec2<double> position_;
    double roll_direction_, steer_direction_, wheel_radius_;
};

class ChassisCtrl
{
  public:
    ChassisCtrl(ros::NodeHandle& nh);
    void update(const ros::Time& time);

  private:
    void moveJoint();
    void chassisCmdCB(const geometry_msgs::TwistStampedConstPtr& msg);
    void steerStateCB(const std_msgs::Float64MultiArray::ConstPtr& state);

    double timeout_{};
    std::string name_space_{};
    std::vector<Wheelset> wheelsets_{};
    std::vector<RampFilter<double>*> ramp_angle_{}, ramp_vel_{};

    geometry_msgs::Vector3 cmd_vel_{};
    geometry_msgs::TwistStamped chassis_cmd_{};
    std_msgs::Float64MultiArray steer_cmd_{}, roll_cmd_{};
    std_msgs::Float64MultiArray steer_state_{};

    ros::Subscriber cmd_chassis_sub_, state_steer_sub_;
    ros::V_Publisher cmd_wheelset_pubs_;
    ros::NodeHandle nh_;
};
}  // namespace chassis_ctrl