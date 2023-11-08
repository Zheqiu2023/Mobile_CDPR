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
    chassis_cmd_ = *cmd;
}

void ChassisCtrl::steerStateCB(const std_msgs::Float64MultiArray::ConstPtr& state)
{
    steer_state_ = *state;
}

ChassisCtrl::ChassisCtrl(ros::NodeHandle& nh) : nh_(nh)
{
    name_space_ = nh_.getNamespace();
    ROS_INFO_STREAM("class ChassisCtrl namespace: " << name_space_);

    XmlRpc::XmlRpcValue wheelsets;
    XmlRpc::XmlRpcValue accel;
    if (!nh_.getParam("wheelsets", wheelsets) || !nh_.getParam("accel", accel) || !nh_.getParam("timeout", timeout_))
    {
        ROS_ERROR("Some chassis params doesn't given in namespace: '%s')", name_space_.c_str());
        return;
    }
    ROS_ASSERT(wheelsets.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_ASSERT(accel.getType() == XmlRpc::XmlRpcValue::TypeStruct);
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
    steer_state_.data.resize(wheelsets_.size());

    ramp_x_ = new RampFilter<double>(accel["x"], 0.005);
    ramp_y_ = new RampFilter<double>(accel["y"], 0.005);
    ramp_w_ = new RampFilter<double>(accel["w"], 0.005);

    // Start command subscriber and publisher
    cmd_chassis_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>("/cmd_chassis", 10,
                                                                  boost::bind(&ChassisCtrl::chassisCmdCB, this, _1));
    state_steer_sub_ = nh_.subscribe<std_msgs::Float64MultiArray>("/steer_pos_state", 10,
                                                                  boost::bind(&ChassisCtrl::steerStateCB, this, _1));
    cmd_wheelset_pubs_.push_back(nh_.advertise<std_msgs::Float64MultiArray>("/steer_pos_cmd", 10));
    cmd_wheelset_pubs_.push_back(nh_.advertise<std_msgs::Float64MultiArray>("/roll_vel_cmd", 10));
}

void ChassisCtrl::update(const ros::Time& time)
{
    if ((time - chassis_cmd_.header.stamp).toSec() > timeout_)
    {
        cmd_vel_.x = 0.;
        cmd_vel_.y = 0.;
        cmd_vel_.z = 0.;
    }
    else
    {
        ramp_x_->input(chassis_cmd_.twist.linear.x);
        ramp_y_->input(chassis_cmd_.twist.linear.y);
        ramp_w_->input(chassis_cmd_.twist.angular.z);
        cmd_vel_.x = ramp_x_->output();  // velocity of car center
        cmd_vel_.y = ramp_y_->output();
        cmd_vel_.z = ramp_w_->output();
    }

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
        steer_cmd_.data.emplace_back(std::abs(a) < std::abs(b) ? vel_angle : vel_angle + M_PI);
        roll_cmd_.data.emplace_back(vel.norm() / wheelsets_[i].wheel_radius_ * std::cos(a) *
                                    wheelsets_[i].roll_direction_);
    }
    cmd_wheelset_pubs_[0].publish(steer_cmd_);
    cmd_wheelset_pubs_[1].publish(roll_cmd_);
    steer_cmd_.data.clear();
    roll_cmd_.data.clear();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "chassis_ctrl");
    ros::NodeHandle nh("~");

    ChassisCtrl chassis_ctrl(nh);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ROS_ASSERT(nh.hasParam("update_rate"));
    int update_rate = nh.param("update_rate", 500);

    ros::Rate loop_rate(update_rate);
    while (ros::ok())
    {
        chassis_ctrl.update(ros::Time::now());
        loop_rate.sleep();
    }

    return 0;
}