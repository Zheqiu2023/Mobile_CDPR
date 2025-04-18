/**
 * @File Name: chassis_controller.cpp
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
#include <pluginlib/class_list_macros.hpp>
#include <angles/angles.h>

#include "cdpr_chassis_controller/chassis_controller.hpp"

using namespace cdpr_chassis_controller;

ChassisController::~ChassisController()
{
    cmd_chassis_sub_.shutdown();
}

/**
 * @brief set the chassis command
 * @param  msg
 */
void ChassisController::cmdChassisCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    chassis_cmd_ = *msg;
    // the writeFromNonRT can be used in RT, if you have the guarantee that
    //  * no non-rt thread is calling the same function (we're not subscribing to ros callbacks)
    //  * there is only one single rt thread
    cmd_rt_buffer_.writeFromNonRT(chassis_cmd_);
}

/**
 * @brief Controller initialization in non-realtime
 * @param  robot_hw
 * @param  root_nh
 * @param  controller_nh
 * @return true
 * @return false
 */
bool ChassisController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
                             ros::NodeHandle& controller_nh)
{
    XmlRpc::XmlRpcValue wheelsets;
    XmlRpc::XmlRpcValue accel;
    name_space_ = controller_nh.getNamespace();
    ROS_INFO_STREAM("ChassisController namespace: " << name_space_);

    if (!controller_nh.getParam("publish_rate", publish_rate_) || controller_nh.getParam("wheelsets", wheelsets) ||
        !controller_nh.getParam("timeout", timeout_) || !controller_nh.getParam("accel", accel))
    {
        ROS_ERROR("Some chassis params are not given in namespace: '%s')", name_space_.c_str());
        return false;
    }
    ROS_ASSERT(wheelsets.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_ASSERT(accel.getType() == XmlRpc::XmlRpcValue::TypeStruct);

    effort_joint_interface_ = robot_hw->get<hardware_interface::EffortJointInterface>();
    for (const auto& wheelset : wheelsets)
    {
        ROS_ASSERT(wheelset.second.hasMember("position"));
        ROS_ASSERT(wheelset.second["position"].getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(wheelset.second["position"].size() == 2);
        ROS_ASSERT(wheelset.second.hasMember("steer"));
        ROS_ASSERT(wheelset.second["steer"].getType() == XmlRpc::XmlRpcValue::TypeStruct);
        ROS_ASSERT(wheelset.second.hasMember("roll"));
        ROS_ASSERT(wheelset.second["roll"].getType() == XmlRpc::XmlRpcValue::TypeStruct);
        ROS_ASSERT(wheelset.second["roll"].hasMember("radius"));

        Wheelset w{ .position_ = Vec2<double>((double)wheelset.second["position"][0], (double)wheelset.second["positio"
                                                                                                              "n"][1]),
                    .steer_offset_ = wheelset.second["steer"]["offset"],
                    .wheel_radius_ = wheelset.second["roll"]["radius"],
                    .ctrl_steer_ = new effort_controllers::JointPositionController(),
                    .ctrl_roll_ = new effort_controllers::JointVelocityController() };

        ros::NodeHandle nh_steer = ros::NodeHandle(controller_nh, "wheelsets/" + wheelset.first + "/steer");
        ros::NodeHandle nh_roll = ros::NodeHandle(controller_nh, "wheelsets/" + wheelset.first + "/roll");

        if (!w.ctrl_steer_->init(effort_joint_interface_, nh_steer) ||
            !w.ctrl_roll_->init(effort_joint_interface_, nh_roll))
            return false;
        if (wheelset.second["steer"].hasMember("offset"))
            w.steer_offset_ = wheelset.second["steer"]["offset"];

        joint_handles_.push_back(w.ctrl_steer_->joint_);
        joint_handles_.push_back(w.ctrl_roll_->joint_);
        wheelsets_.push_back(w);
    }

    if (controller_nh.hasParam("pid_follow"))
        if (!pid_follow_.init(ros::NodeHandle(controller_nh, "pid_follow")))
            return false;

    ramp_x_ = new RampFilter<double>((double)accel["x"], 0.001);
    ramp_y_ = new RampFilter<double>((double)accel["y"], 0.001);
    ramp_w_ = new RampFilter<double>((double)accel["z"], 0.001);

    // connect the joint state interface
    // joint_state_.position.resize(joint_handles_.size());
    // joint_state_.velocity.resize(joint_handles_.size());
    // joint_state_.effort.resize(joint_handles_.size());
    // for (size_t i = 0; i < joint_handles_.size(); ++i)
    // {
    //     std::string joint_name = joint_handles_[i].getName();
    //     joint_state_.name.push_back(joint_name);
    //     robot_hw->get<hardware_interface::JointStateInterface>()->registerHandle(hardware_interface::JointStateHandle(
    //         joint_name, &joint_state_.position[i], &joint_state_.velocity[i], &joint_state_.effort[i]));
    // }

    // Start command subscriber
    cmd_chassis_sub_ = controller_nh.subscribe<geometry_msgs::TwistStamped>(
        "/cmd_chassis", 1, &ChassisController::cmdChassisCallback, this);
    // Start realtime state publisher
    chassis_state_publisher_.reset(
        new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(controller_nh, "/chassis_state", 1));

    return true;
}

// Controller update loop in realtime
void ChassisController::update(const ros::Time& time, const ros::Duration& period)
{
    geometry_msgs::Twist cmd_twist = cmd_rt_buffer_.readFromRT()->twist;

    if ((time - cmd_rt_buffer_.readFromRT()->header.stamp).toSec() > timeout_)
    {
        cmd_vel_.x = 0.;
        cmd_vel_.y = 0.;
        cmd_vel_.z = 0.;
    }
    else
    {
        ramp_x_->input(cmd_twist.linear.x);
        ramp_y_->input(cmd_twist.linear.y);
        ramp_w_->input(cmd_twist.angular.z);
        cmd_vel_.x = ramp_x_->output();  // velocity of car center
        cmd_vel_.y = ramp_y_->output();
        cmd_vel_.z = ramp_w_->output();
    }

    moveJoint(time, period);
    // publishJointState(time);
}

// Ref: https://dominik.win/blog/programming-swerve-drive/
// Ref: https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383
void ChassisController::moveJoint(const ros::Time& time, const ros::Duration& period)
{
    Vec2<double> vel_center(cmd_vel_.x, cmd_vel_.y);  // velocity of wheelset
    for (auto& wheelset : wheelsets_)
    {
        // Calculate the speed and angle of each wheelset based on the speed of car center
        Vec2<double> vel = vel_center + cmd_vel_.z * Vec2<double>(-wheelset.position_.y(), wheelset.position_.x());
        double vel_angle = std::atan2(vel.y(), vel.x()) + wheelset.steer_offset_;

        // Direction flipping and Stray wheelset mitigation
        double a = angles::shortest_angular_distance(wheelset.ctrl_steer_->joint_.getPosition(), vel_angle);
        double b = angles::shortest_angular_distance(wheelset.ctrl_steer_->joint_.getPosition(), vel_angle + M_PI);
        wheelset.ctrl_steer_->setCommand(std::abs(a) < std::abs(b) ? vel_angle : vel_angle + M_PI);
        wheelset.ctrl_roll_->setCommand(vel.norm() / wheelset.wheel_radius_ * std::cos(a));
        wheelset.ctrl_steer_->update(time, period);
        wheelset.ctrl_roll_->update(time, period);
    }
}

void ChassisController::publishJointState(const ros::Time& time)
{
    if (last_publish_time_ + ros::Duration(1.0 / publish_rate_) < time)
    {
        if (chassis_state_publisher_->trylock())
        {
            joint_state_.header.stamp = ros::Time::now();
            chassis_state_publisher_->msg_ = joint_state_;
        }
        last_publish_time_ = time;
        chassis_state_publisher_->unlockAndPublish();
    }
}

// Register controller to pluginlib
PLUGINLIB_EXPORT_CLASS(cdpr_chassis_controller::ChassisController, controller_interface::ControllerBase)