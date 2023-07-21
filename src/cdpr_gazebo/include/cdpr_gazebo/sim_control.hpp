#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <vector>

namespace sim_control
{
class SimControl
{
  public:
    SimControl();
    void recvTwist(const geometry_msgs::Twist::ConstPtr& msg);
    void calcWheelAngVel(std::vector<double>* roll_vec, std::vector<double>* steer_vec, const float& wheelbase,
                         const float& tread, const float& wheel_radius);
    void moveAround();
    void skew();
    void spin();

  private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::V_Publisher pubs_;
    double linear_vel_ = 0, angular_vel_ = 0, turn_flag_ = 0;
};
}  // namespace sim_control
