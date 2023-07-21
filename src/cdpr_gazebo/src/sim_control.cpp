#include <sim_control.hpp>
#include <std_msgs/Float64.h>
#include <math.h>

using namespace sim_control;

SimControl::SimControl()
{
    sub_ = nh_.subscribe("/cmd_vel", 10, &SimControl::recvTwist, this);
    pubs_.reserve(8);
    pubs_.push_back(nh_.advertise<std_msgs::Float64>("/cdpr/roll_lf_controller/command", 10));
    pubs_.push_back(nh_.advertise<std_msgs::Float64>("/cdpr/roll_rf_controller/command", 10));
    pubs_.push_back(nh_.advertise<std_msgs::Float64>("/cdpr/roll_lb_controller/command", 10));
    pubs_.push_back(nh_.advertise<std_msgs::Float64>("/cdpr/roll_rb_controller/command", 10));
    pubs_.push_back(nh_.advertise<std_msgs::Float64>("/cdpr/steer_lf_controller/command", 10));
    pubs_.push_back(nh_.advertise<std_msgs::Float64>("/cdpr/steer_rf_controller/command", 10));
    pubs_.push_back(nh_.advertise<std_msgs::Float64>("/cdpr/steer_lb_controller/command", 10));
    pubs_.push_back(nh_.advertise<std_msgs::Float64>("/cdpr/steer_rb_controller/command", 10));
    // pubs_.push_back(nh_.advertise<std_msgs::Float64>("/LF0_controller/command", 10));
    // pubs_.push_back(nh_.advertise<std_msgs::Float64>("/RF0_controller/command", 10));
    // pubs_.push_back(nh_.advertise<std_msgs::Float64>("/LB0_controller/command", 10));
    // pubs_.push_back(nh_.advertise<std_msgs::Float64>("/RB0_controller/command", 10));
    // pubs_.push_back(nh_.advertise<std_msgs::Float64>("/LF2_controller/command", 10));
    // pubs_.push_back(nh_.advertise<std_msgs::Float64>("/RF2_controller/command", 10));
    // pubs_.push_back(nh_.advertise<std_msgs::Float64>("/LB2_controller/command", 10));
    // pubs_.push_back(nh_.advertise<std_msgs::Float64>("/RB2_controller/command", 10));
}

/**
 * @brief 接收键盘输入的速度
 * @param  msg
 */
void SimControl::recvTwist(const geometry_msgs::Twist::ConstPtr& msg)
{
    linear_vel_ = msg->linear.x;
    angular_vel_ = msg->angular.z;
    turn_flag_ = msg->angular.x;
}

/**
 * @brief 根据车身线速度和角速度计算车轮转速和转角
 * @param  roll_vec
 * @param  steer_vec
 * @param  wheelbase
 * @param  tread
 * @param  wheel_radius
 */
void SimControl::calcWheelAngVel(std::vector<double>* roll_vec, std::vector<double>* steer_vec, const float& wheelbase,
                                 const float& tread, const float& wheel_radius)
{
    float turning_radius = linear_vel_ / angular_vel_;

    // 0、1、2、3分别对应lf、rf、lb、rb
    if (angular_vel_ == 0)
    {
        for (size_t i = 0; i < 4; ++i)
        {
            (*roll_vec)[i] = static_cast<double>(linear_vel_ / wheel_radius);
            (*steer_vec)[i] = 0;
        }
    }
    else if (turn_flag_ == 0)
    {
        // 四轮转向：转向瞬心位于机器人横向对称轴的延长线，内侧两轮转角和速度相等，外侧两轮转角和速度相等
        // 车轮角速度
        (*roll_vec)[0] = (*roll_vec)[2] =
            angular_vel_ * sqrt(pow(turning_radius - tread / 2, 2) + pow(wheelbase / 2, 2)) / wheel_radius;
        (*roll_vec)[1] = (*roll_vec)[3] =
            angular_vel_ * sqrt(pow(turning_radius + tread / 2, 2) + pow(wheelbase / 2, 2)) / wheel_radius;
        // 转向角度
        (*steer_vec)[0] = atan(wheelbase / (2 * turning_radius - tread));
        (*steer_vec)[1] = atan(wheelbase / (2 * turning_radius + tread));
        (*steer_vec)[2] = -atan(wheelbase / (2 * turning_radius - tread));
        (*steer_vec)[3] = -atan(wheelbase / (2 * turning_radius + tread));
    }
    else if (turn_flag_ == 1)
    {
        // 车轮角速度：四轮角速度相同，会产生误差
        for (size_t i = 0; i < 4; ++i)
        {
            (*roll_vec)[i] = static_cast<double>(linear_vel_ / wheel_radius);
        }
        // 转向角度
        (*steer_vec)[0] = angular_vel_;
        (*steer_vec)[1] = atan(wheelbase / (wheelbase / tan(angular_vel_) + 2 * tread));
        (*steer_vec)[2] = -angular_vel_;
        (*steer_vec)[3] = -atan(wheelbase / (wheelbase / tan(angular_vel_) + 2 * tread));
    }

    // ROS_INFO("steer_vec[0] = %f, steer_vec[1] = %f, steer_vec[2] = %f, steer_vec[3] = %f", (*steer_vec)[0],
    //          (*steer_vec)[1], (*steer_vec)[2], (*steer_vec)[3]);
    // ROS_INFO("roll_vec[0] = %f, roll_vec[1] = %f, roll_vec[2] = %f, roll_vec[3] = %f", (*roll_vec)[0],
    // (*roll_vec)[1],
    //          (*roll_vec)[2], (*roll_vec)[3]);
}

/**
 * @brief 直行、转弯
 */
void SimControl::moveAround()
{
    // 获取轴距、轮距、轮胎半径
    float wheelbase = 0, tread = 0, wheel_radius = 0;
    ros::param::get("/cdpr_gazebo/wheelbase", wheelbase);
    ros::param::get("/cdpr_gazebo/tread", tread);
    ros::param::get("/cdpr_gazebo/wheel_radius", wheel_radius);

    std::vector<double> roll_vec(4, 0), steer_vec(4, 0);
    std::vector<std_msgs::Float64> roll_msg, steer_msg;
    roll_msg.resize(4);
    steer_msg.resize(4);

    ros::Rate loop_rate(500);
    while (ros::ok())
    {
        calcWheelAngVel(&roll_vec, &steer_vec, wheelbase, tread, wheel_radius);
        for (size_t i = 0; i < 4; ++i)
        {
            roll_msg[i].data = roll_vec[i];
            steer_msg[i].data = steer_vec[i];
        }

        for (size_t i = 0; i < 4; ++i)
        {
            pubs_[i].publish(roll_msg[i]);
            pubs_[i + 4].publish(steer_msg[i]);
        }

        loop_rate.sleep();
        ros::spinOnce();
    }
}

/**
 * @brief 横移
 */
void SimControl::skew()
{
    // 获取轮胎半径
    float wheel_radius = 0;
    ros::param::get("/cdpr_gazebo/wheel_radius", wheel_radius);

    std::vector<std_msgs::Float64> roll_msg, steer_msg;
    roll_msg.resize(4);
    steer_msg.resize(4);

    ros::Rate loop_rate(500);
    while (ros::ok())
    {
        for (size_t i = 0; i < 4; ++i)
        {
            // 四轮角速度相同
            roll_msg[i].data = linear_vel_ / wheel_radius;
            // 斜移时四轮角度相同
            steer_msg[i].data = angular_vel_;
        }

        for (size_t i = 0; i < 4; ++i)
        {
            pubs_[i].publish(roll_msg[i]);
            pubs_[i + 4].publish(steer_msg[i]);
        }

        loop_rate.sleep();
        ros::spinOnce();
    }
}

/**
 * @brief 自旋
 */
void SimControl::spin()
{
    // 获取轴距、轮距、轮胎半径
    float wheelbase = 0, tread = 0, wheel_radius = 0;
    ros::param::get("/cdpr_gazebo/wheelbase", wheelbase);
    ros::param::get("/cdpr_gazebo/tread", tread);
    ros::param::get("/cdpr_gazebo/wheel_radius", wheel_radius);

    std::vector<double> roll_vec(4, 0), steer_vec(4, 0);
    std::vector<std_msgs::Float64> roll_msg, steer_msg;
    roll_msg.resize(4);
    steer_msg.resize(4);

    ros::Rate loop_rate(500);
    while (ros::ok())
    {
        // 四轮角速度
        roll_msg[0].data = linear_vel_ / wheel_radius;
        roll_msg[1].data = -linear_vel_ / wheel_radius;
        roll_msg[2].data = linear_vel_ / wheel_radius;
        roll_msg[3].data = -linear_vel_ / wheel_radius;
        // 四轮角度
        steer_msg[0].data = angular_vel_;
        steer_msg[1].data = -angular_vel_;
        steer_msg[2].data = -angular_vel_;
        steer_msg[3].data = angular_vel_;

        for (size_t i = 0; i < 4; ++i)
        {
            pubs_[i].publish(roll_msg[i]);
            pubs_[i + 4].publish(steer_msg[i]);
        }

        loop_rate.sleep();
        ros::spinOnce();
    }
}