/**
 * @File Name: a1.cpp
 * @brief
 * @author Zhe Qiu (zheqiu2021@163.com)
 * @version 0.1
 * @date 2023-05-26
 *
 *  ***********************************************************************************
 *  @copyright Copyright (c) 2023  by Zhe Qiu. All rights reserved.
 *  Use of this source code is governed by the BSD 3-Clause license, see LICENSE.
 *  ***********************************************************************************
 */
#include "unitree_motor_a1/a1.hpp"

#include <std_msgs/Float64.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include "cdpr_bringup/interpolation.hpp"
#include "cdpr_bringup/math_utilities.hpp"

using namespace motor_a1;

A1Control::A1Control(ros::NodeHandle& nh) : nh_(nh) {
    subs_.push_back(nh_.subscribe("/remote_roll_vel", 10, &A1Control::remoteControlCB, this));
    subs_.push_back(nh_.subscribe("/traj_roll_vel", 10, &A1Control::trajTrackingCB, this));
    subs_.push_back(nh_.subscribe("/start_traj_tracking", 10, &A1Control::startTrajCB, this));
    pub_ = nh_.advertise<std_msgs::Bool>("ready_state", 1);
    traj_cmd_pubs_.push_back(nh_.advertise<std_msgs::Float64>("/traj_vel_cmd0", 100));
    traj_cmd_pubs_.push_back(nh_.advertise<std_msgs::Float64>("/traj_vel_cmd1", 100));
    traj_cmd_pubs_.push_back(nh_.advertise<std_msgs::Float64>("/traj_vel_cmd2", 100));
    traj_cmd_pubs_.push_back(nh_.advertise<std_msgs::Float64>("/traj_vel_cmd3", 100));
    traj_state_pubs_.push_back(nh_.advertise<std_msgs::Float64>("/traj_vel_state0", 100));
    traj_state_pubs_.push_back(nh_.advertise<std_msgs::Float64>("/traj_vel_state1", 100));
    traj_state_pubs_.push_back(nh_.advertise<std_msgs::Float64>("/traj_vel_state2", 100));
    traj_state_pubs_.push_back(nh_.advertise<std_msgs::Float64>("/traj_vel_state3", 100));

    ros::Duration(2).sleep();  // 延时，保证pub_注册成功(必须有)
}

/**
 * @brief receive target velocity(Used in keyboard remote control)
 * @param  vel
 */
void A1Control::remoteControlCB(const std_msgs::Float64MultiArray::ConstPtr& vel) {
    for (size_t i = 0; i < motor_param_.size(); ++i) motor_param_[i].motor_cmd_.W = vel->data[i] * reduction_ratio_;
}

/**
 * @brief receive target velocity(Used in trajectory tracking)
 * @param  vel
 */
void A1Control::trajTrackingCB(const std_msgs::Float64MultiArray::ConstPtr& vel) { traj_ = std::move(vel->data); }

void A1Control::startTrajCB(const std_msgs::Bool::ConstPtr& flag) { start_traj_tracking_ = flag->data; }

/**
 * @brief set control parameters
 * @param  cmd
 */
void A1Control::setControlParam(const std::vector<double>& cmd) {
    int mode = nh_.param("motor_ctrl_data/mode", 0);

    for (auto& motor_param : motor_param_) {
        // actual torque command: PD controller
        // K_P*delta_Pos + K_W*delta_W + T
        motor_param.motor_cmd_.mode = mode;
        motor_param.motor_cmd_.K_P = cmd[0];
        motor_param.motor_cmd_.K_W = cmd[1];
    }
}

/**
 * @brief initialize the motor
 * @param  port
 */
void A1Control::init() {
    for (auto& motor_param : motor_param_) {
        // initial parameters
        motor_param.init_cmd_.mode = 0;
        motor_param.port_->sendRecv(&motor_param.init_cmd_, &motor_param.motor_recv_);
        // get current position and set it as zero point
        motor_param.zero_position_ = motor_param.motor_recv_.Pos;
        motor_param.last_pos_ = motor_param.motor_recv_.Pos;

        ROS_INFO("Zero position of motor A1[%d]: %f", motor_param.serial_num_, motor_param.zero_position_);
    }

    ROS_INFO("Motor A1 initialization complete, start running!");
}

/**
 * @brief  set K_W and K_P and run the motor according to the control mode specified
 */
void A1Control::drive() {
    std::vector<double> control_param_vec(2, 0);
    int cmd_type = nh_.param("cmd_type", 0);
    int ctrl_frequency = nh_.param("motor_ctrl_data/ctrl_frequency", 200);
    ros::Rate loop_rate(ctrl_frequency);

    switch (cmd_type) {
        case 0: {  // keyboard remote control
            // set K_W、K_P
            nh_.getParam("motor_ctrl_data/vel_kp_kw", control_param_vec);
            setControlParam(control_param_vec);

            // receive target velocity from topic "/remote_roll_vel"
            while (ros::ok()) {
                // send the command
                for (auto& motor_param : motor_param_) {
                    motor_param.port_->sendRecv(&motor_param.motor_cmd_, &motor_param.motor_recv_);
                    ROS_INFO("Received velocity of motor A1[%d]: %f", motor_param.serial_num_,
                             motor_param.motor_recv_.W);
                }

                ros::spinOnce();
                loop_rate.sleep();
            }
            break;
        }
        case 1: {  // follow trajectory
            if (!(nh_.getParam("/traj/chassis_period", traj_period_) &&
                  nh_.getParam("motor_ctrl_data/vel_kp_kw", control_param_vec)))
                ROS_ERROR("chassis_period or vel_kp_kw are not given");

            // set K_W、K_P
            setControlParam(control_param_vec);

            std_msgs::Bool is_ready{};
            is_ready.data = true;
            pub_.publish(is_ready);
            ROS_INFO("Unitree motor A1 reset done, ready to follow the trajectory!");

            while (ros::ok() && !start_traj_tracking_) ros::spinOnce();

            // receive target position from topic "/traj_roll_vel"
            std_msgs::Float64 temp_val{};
            for (size_t i = 0; i < traj_.size() / 4; ++i) {
                if (false == start_traj_tracking_) break;

                for (size_t j = 0; j < motor_param_.size(); ++j) {
                    motor_param_[j].motor_cmd_.W = traj_[i * 4 + j] * reduction_ratio_ * motor_param_[j].direction_ /
                                                   wheel_radius_;  //   m/s转换为rad/s
                    motor_param_[j].port_->sendRecv(&motor_param_[j].motor_cmd_, &motor_param_[j].motor_recv_);
                    temp_val.data =
                        motor_param_[j].motor_cmd_.W * wheel_radius_ / (reduction_ratio_ * motor_param_[j].direction_);
                    traj_cmd_pubs_[j].publish(temp_val);
                    temp_val.data =
                        motor_param_[j].motor_recv_.W * wheel_radius_ / (reduction_ratio_ * motor_param_[j].direction_);
                    traj_state_pubs_[j].publish(temp_val);
                }
                ros::Duration(traj_period_).sleep();
            }

            break;
        }
        case 2: {  // position control
            // set K_W、K_P
            nh_.getParam("motor_ctrl_data/pos_kp_kw", control_param_vec);
            setControlParam(control_param_vec);
            // set target position
            double add_goal_position = 0.0;
            std::vector<double> goal_position_vec{};
            nh_.getParam("motor_ctrl_data/goal_pos_vec", goal_position_vec);

            for (auto goal_position : goal_position_vec) {
                // Unitree motors use a single-turn absolute encoder, the encoder position will not return to zero
                // after restarting. Therefore, the encoder position at the time of powering up  is used as
                // compensation, i.e., the motor position at the time of powering up is used as the zero
                // point, and the angle sent is the angle of the zero point plus the target angle (multi-turn
                // totalization).
                add_goal_position += goal_position;
                for (auto& motor_param : motor_param_) {
                    motor_param.motor_cmd_.Pos =
                        motor_param.zero_position_ + deg2rad(add_goal_position * reduction_ratio_);
                    motor_param.port_->sendRecv(&motor_param.motor_cmd_, &motor_param.motor_recv_);
                    ROS_DEBUG("Received position of motor A1[%d]: %f", motor_param.serial_num_,
                              motor_param.motor_recv_.Pos);
                }
                ros::Duration(0.5).sleep();
            }
            break;
        }
        case 3: {  // bessel curve trajectory planning：no need to set a timer in while(ros::ok())
            // set K_W、K_P
            nh_.getParam("motor_ctrl_data/traj_kp_kw", control_param_vec);
            setControlParam(control_param_vec);

            double total_run_time = 0, end_pos = 0, phase = 0, goal_position = 0;
            std::vector<double> bezier_plan_result(3, 0), goal_traj{}, traj_start_pos(motor_param_.size(), 0);
            std::vector<int> current_traj_point(motor_param_.size(), 0);

            nh_.getParam("motor_ctrl_data/interpolation/total_run_time", total_run_time);
            nh_.getParam("motor_ctrl_data/goal_traj", goal_traj);
            double per_traj_seg_run_time = total_run_time / goal_traj.size();  // run time of each trajectory segment

            // get the start time
            double start_time = ros::Time::now().toSec();
            // current time
            double right_now = 0;
            while (ros::ok()) {
                for (size_t i = 0; i < motor_param_.size(); ++i) {
                    goal_position = 0;
                    right_now = ros::Time::now().toSec();
                    if ((right_now - start_time) < 0) {
                        ROS_FATAL("System Time Error!");
                    } else if ((right_now - start_time) >= total_run_time)  // the entire trajectory finished
                    {
                        motor_param_[i].motor_cmd_.Pos =
                            motor_param_[i].motor_recv_.Pos;  // stay at the end of the trajectory
                        phase = (right_now - start_time - (per_traj_seg_run_time * (current_traj_point[i] - 1))) /
                                per_traj_seg_run_time;  // any number greater than 1 is acceptable
                        ROS_INFO("Motor A1[%d] run complete!", motor_param_[i].serial_num_);
                    } else if ((right_now - start_time) >
                               (double)(current_traj_point[i] * per_traj_seg_run_time))  // one trajectory segment
                                                                                         // finished, update for next
                                                                                         // segment
                    {
                        ++current_traj_point[i];
                        traj_start_pos[i] = motor_param_[i].motor_recv_.Pos;
                    }

                    if (phase >= 0 && phase <= 1) {
                        // one segment isn't finished, continue planning
                        for (size_t j = 0; j < current_traj_point[i]; ++j) {
                            goal_position += goal_traj[j];
                        }
                        end_pos = motor_param_[i].zero_position_ + deg2rad(goal_position * reduction_ratio_);
                        phase = (right_now - start_time - (per_traj_seg_run_time * (current_traj_point[i] - 1))) /
                                per_traj_seg_run_time;  // phase of each trajectory segment�?0~1

                        bezier_plan_result =
                            interpolate::cubicBezierTrajPlanner(traj_start_pos[i], end_pos, phase, total_run_time);
                        ROS_INFO("Bezier planning result: pos:%f, vel:%f, acc:%f", bezier_plan_result[0],
                                 bezier_plan_result[1], bezier_plan_result[2]);
                        motor_param_[i].motor_cmd_.Pos = bezier_plan_result[0];
                        motor_param_[i].motor_cmd_.W = bezier_plan_result[1];
                    }

                    motor_param_[i].port_->sendRecv(&motor_param_[i].motor_cmd_, &motor_param_[i].motor_recv_);
                    ROS_INFO("Motor A1[%d] run time:%.6lf, phase:%f, pos:%f, vel:%f, startpos:%f, endpos:%f",
                             motor_param_[i].serial_num_, (right_now - start_time), phase,
                             motor_param_[i].motor_recv_.Pos, motor_param_[i].motor_recv_.W, traj_start_pos[i],
                             end_pos);
                }
            }
            break;
        }
        case 4: {  // quintic polynomial trajectory planning
            // set K_W、K_P
            nh_.getParam("motor_ctrl_data/traj_kp_kw", control_param_vec);
            setControlParam(control_param_vec);

            std::vector<int> current_traj_point(motor_param_.size(), 0);
            int total_run_time = 2, goal_position = 360;
            int point_num = ceil(ctrl_frequency * total_run_time) +
                            101;  // ensure that the control period is greater than the running time of each segment
            double per_traj_seg_run_time = total_run_time / (point_num - 1);

            std::vector<std::vector<std::vector<double>>> qp_plan_result{};
            for (size_t i = 0; i < motor_param_.size(); ++i) {
                double end_pos = motor_param_[i].zero_position_ + deg2rad(goal_position * reduction_ratio_);
                std::vector<double> pos{motor_param_[i].zero_position_, end_pos}, vel{0, 0},
                    acc{0, 0};  // start，end position data
                qp_plan_result.push_back(interpolate::quinticPolynomial(total_run_time, pos, vel, acc, point_num));
            }

            // get the start time
            double start_time = ros::Time::now().toSec();
            // current time
            double right_now = 0;
            while (ros::ok()) {
                for (size_t i = 0; i < motor_param_.size(); ++i) {
                    right_now = ros::Time::now().toSec();
                    if ((right_now - start_time) < 0) {
                        ROS_FATAL("Time Error");
                    } else if (current_traj_point[i] >= point_num)  // the entire trajectory finished
                    {
                        ROS_INFO("Motor A1[%d] run complete!", motor_param_[i].serial_num_);
                        motor_param_[i].motor_cmd_.Pos = qp_plan_result[i].back()[0];
                    }
                    // If you don't use timer in while(ros::ok()), you need to use time to judge whether the waypoint is
                    // completed or not; if you use timer, you don't need to judge whether the waypoint is completed or
                    // not, but you need to make sure that the control period is greater than the running time of each
                    // segment of the trajectory. else if ((right_now - start_time) >
                    // static_cast<double>(current_traj_point * per_traj_seg_run_time))
                    // // one trajectory segment finished, update for next segment
                    // {
                    ++current_traj_point[i];

                    if (current_traj_point[i] < point_num) {
                        motor_param_[i].motor_cmd_.Pos = qp_plan_result[i][current_traj_point[i]][0];
                        motor_param_[i].motor_cmd_.W = qp_plan_result[i][current_traj_point[i]][1];
                        // instead of using the planned speeds, use the average of the speeds for each segment
                        // motor_param_[i].motor_cmd_.W = (motor_param_[i].motor_cmd_.Pos -
                        // motor_param_[i].motor_recv_.Pos) / per_traj_seg_run_time;
                    }
                    // }
                    motor_param_[i].port_->sendRecv(&motor_param_[i].motor_cmd_, &motor_param_[i].motor_recv_);
                    ROS_INFO("Motor A1[%d] run time:%.6lf, pos:%f, Vel:%f", motor_param_[i].serial_num_,
                             (right_now - start_time), motor_param_[i].motor_recv_.Pos, motor_param_[i].motor_recv_.W);
                }
                loop_rate.sleep();
            }
            break;
        }
        default: {
            ROS_ERROR("Undefined motor control type!");
            break;
        }
    }
}

/**
 * @brief stop the motor
 * @param  port
 */
void A1Control::stall() {
    for (auto& motor_param : motor_param_) {
        while (!(motor_param.port_->sendRecv(&motor_param.init_cmd_, &motor_param.motor_recv_)))
            ros::Duration(0.1).sleep();

        ROS_DEBUG_STREAM("position: " << motor_param.motor_recv_.Pos << motor_param.motor_recv_.T);
    }

    ROS_INFO("Motor A1 run over!");
}

/**
 * @brief open serial port and drive the motor
 */
void A1Control::operator()() {
    // get motor parameters
    std::vector<int> id{}, direction{};
    std::vector<std::string> port_name{};

    if (!(nh_.getParam("id", id) && nh_.getParam("direction", direction) && nh_.getParam("port_name", port_name) &&
          nh_.getParam("reduction_ratio", reduction_ratio_) && nh_.getParam("wheel_radius", wheel_radius_)))
        ROS_ERROR("Some motor params are not given in namespace: '%s')", nh_.getNamespace().c_str());

    // open serial port
    std::array<SerialPort, 4> serial_port{SerialPort(port_name[0]), SerialPort(port_name[1]), SerialPort(port_name[2]),
                                          SerialPort(port_name[3])};
    motor_param_.resize(port_name.size());

    for (int i = 0; i < port_name.size(); ++i) {
        motor_param_[i].serial_num_ = i;
        motor_param_[i].direction_ = direction[i];
        motor_param_[i].init_cmd_.id = static_cast<unsigned short>(id[i]);
        motor_param_[i].motor_cmd_.id = static_cast<unsigned short>(id[i]);
        motor_param_[i].port_ = &serial_port[i];
    }

    init();
    drive();
    stall();
}