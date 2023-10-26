/**
 * @File Name: sim_control.hpp
 * @brief
 * @author Zhe Qiu (zheqiu2021@163.com)
 * @version 0.1
 * @date 2023-10-18
 *
 * *  ***********************************************************************************
 * *  @copyright Copyright (c) 2023  by Zhe Qiu. All rights reserved.
 * *  Use of this source code is governed by the BSD 3-Clause license, see LICENSE.
 * *  ***********************************************************************************
 */
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
