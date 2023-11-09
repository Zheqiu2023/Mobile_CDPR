#include <angles/angles.h>

#include "cdpr_bringup/eigen_types.hpp"
#include "cdpr_control/chassis_ctrl.hpp"

using namespace chassis_ctrl;

/**
 * @brief set the chassis command
 * @param  cmd
 */
void ChassisCtrl::chassisCmdCB(const geometry_msgs::TwistStampedConstPtr& cmd)
{
    chassis_cmd_ = std::move(*cmd);
}

void ChassisCtrl::steerStateCB(const std_msgs::Float64MultiArray::ConstPtr& state)
{
    steer_state_ = std::move(*state);
}

ChassisCtrl::ChassisCtrl(ros::NodeHandle& nh) : nh_(nh)
{
    name_space_ = nh_.getNamespace();
    ROS_INFO_STREAM("class ChassisCtrl namespace: " << name_space_);

    // get parameters from .yaml file
    XmlRpc::XmlRpcValue wheelsets;
    double accel = 0.0;
    if (!nh_.getParam("wheelsets", wheelsets) || !nh_.getParam("accel", accel) || !nh_.getParam("timeout", timeout_))
    {
        ROS_ERROR("Some chassis params doesn't given in namespace: '%s')", name_space_.c_str());
        return;
    }
    ROS_ASSERT(wheelsets.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    for (const auto& wheelset : wheelsets)
    {
        ROS_ASSERT(wheelset.second.hasMember("position"));
        ROS_ASSERT(wheelset.second["position"].getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(wheelset.second["position"].size() == 2);
        ROS_ASSERT(wheelset.second.hasMember("steer"));
        ROS_ASSERT(wheelset.second["steer"].getType() == XmlRpc::XmlRpcValue::TypeStruct);
        ROS_ASSERT(wheelset.second["steer"].hasMember("direction"));
        ROS_ASSERT(wheelset.second.hasMember("roll"));
        ROS_ASSERT(wheelset.second["roll"].getType() == XmlRpc::XmlRpcValue::TypeStruct);
        ROS_ASSERT(wheelset.second["roll"].hasMember("direction"));
        ROS_ASSERT(wheelset.second["roll"].hasMember("radius"));

        wheelsets_.push_back(Wheelset{
            .position_ = Vec2<double>{ (double)wheelset.second["position"][0], (double)wheelset.second["position"][1] },
            .roll_direction_ = wheelset.second["roll"]["direction"],
            .steer_direction_ = wheelset.second["steer"]["direction"],
            .wheel_radius_ = wheelset.second["roll"]["radius"] });
    }

    // initialiize member variables
    steer_state_.data.resize(wheelsets_.size());
    steer_cmd_.data.resize(wheelsets_.size());
    roll_cmd_.data.resize(wheelsets_.size());

    ramp_vel_.resize(wheelsets_.size());
    ramp_angle_.resize(wheelsets_.size());
    for (std::size_t i = 0; i < wheelsets_.size(); ++i)
    {
        ramp_vel_[i] = new RampFilter<double>(accel, 0.005);
        ramp_angle_[i] = new RampFilter<double>(M_PI, 0.05);
    }

    // Start command subscriber and publisher
    cmd_chassis_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>("/cmd_chassis", 1,
                                                                  boost::bind(&ChassisCtrl::chassisCmdCB, this, _1));
    state_steer_sub_ = nh_.subscribe<std_msgs::Float64MultiArray>("/steer_pos_state", 10,
                                                                  boost::bind(&ChassisCtrl::steerStateCB, this, _1));
    cmd_wheelset_pubs_.push_back(nh_.advertise<std_msgs::Float64MultiArray>("/steer_pos_cmd", 100));
    cmd_wheelset_pubs_.push_back(nh_.advertise<std_msgs::Float64MultiArray>("/roll_vel_cmd", 100));
}

void ChassisCtrl::update(const ros::Time& time)
{
    cmd_vel_.x = chassis_cmd_.twist.linear.x;
    cmd_vel_.y = chassis_cmd_.twist.linear.y;
    cmd_vel_.z = chassis_cmd_.twist.angular.z;

    moveJoint();
}

// Ref: https://dominik.win/blog/programming-swerve-drive/
// Ref: https://www.chiefdelphi.com/t/paper-4-wheel-independent-drive-independent-steering-swerve/107383
void ChassisCtrl::moveJoint()
{
    Vec2<double> vel_center(cmd_vel_.x, cmd_vel_.y);  // velocity of wheelset
    for (size_t i = 0; i < wheelsets_.size(); ++i)
    {
        // Calculate the speed and angle of each wheelset based on the speed of car center
        Vec2<double> vel =
            vel_center + cmd_vel_.z * Vec2<double>(-wheelsets_[i].position_.y(), wheelsets_[i].position_.x());
        double vel_angle = std::atan2(vel.y(), vel.x());

        // Direction flipping and Stray wheelset mitigation
        double a = angles::shortest_angular_distance(steer_state_.data[i], vel_angle);
        double b = angles::shortest_angular_distance(steer_state_.data[i], vel_angle + M_PI);
        double target_angle = 0.0, target_vel = 0.0;
        if (std::abs(a) > std::abs(b))
            target_angle = (vel_angle + M_PI) > M_PI ? angles::normalize_angle(vel_angle + M_PI) : vel_angle + M_PI;
        else
            target_angle = vel_angle;
        target_vel = vel.norm() / wheelsets_[i].wheel_radius_ * std::cos(a) * wheelsets_[i].roll_direction_;

        // Smoothing steering angle and rolling velocity
        ramp_angle_[i]->input(target_angle);
        ramp_vel_[i]->input(target_vel);
        steer_cmd_.data[i] = ramp_angle_[i]->output();
        roll_cmd_.data[i] = ramp_vel_[i]->output();
    }
    cmd_wheelset_pubs_[0].publish(steer_cmd_);
    cmd_wheelset_pubs_[1].publish(roll_cmd_);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "chassis_ctrl");
    ros::NodeHandle nh("~");

    ChassisCtrl chassis_ctrl(nh);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ROS_ASSERT(nh.hasParam("update_rate"));
    int update_rate = nh.param("update_rate", 200);

    ros::Rate loop_rate(update_rate);
    while (ros::ok())
    {
        chassis_ctrl.update(ros::Time::now());
        loop_rate.sleep();
    }

    return 0;
}