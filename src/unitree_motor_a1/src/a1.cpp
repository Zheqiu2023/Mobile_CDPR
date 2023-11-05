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
#include "cdpr_bringup/interpolation.hpp"

#include <vector>
#include <array>
#include <math.h>
#include <algorithm>

using namespace motor_a1;

A1Control::A1Control(ros::NodeHandle nh) : nh_(nh)
{
    sub_ = nh_.subscribe<std_msgs::Float64MultiArray>("/roll_vel_cmd", 10,
                                                      boost::bind(&A1Control::setCommandCB, this, _1));
    // get motor parameters
    if (!(nh_.getParam("id", id_) && nh_.getParam("port_name", serial_port_)))
        ROS_ERROR("Some motor params doesn't given in namespace: '%s')", nh_.getNamespace().c_str());
    motor_num_ = id_.size();
}

/**
 * @brief receive and set target velocity
 * @param  cmd_vel
 */
void A1Control::setCommandCB(const std_msgs::Float64MultiArray::ConstPtr& cmd_vel)
{
    float reduction_ratio = nh_.param("reduction_ratio", 9.0);
    for (size_t i = 0; i < motor_num_; ++i)
    {
        motor_cmd_[i].W = cmd_vel->data.at(i) * reduction_ratio;
    }
}

/**
 * @brief set control parameters
 * @param  cmd
 */
void A1Control::setCmd(const std::vector<float>& cmd)
{
    int mode = 0;
    nh_.getParam("motor_ctrl_data/mode", mode);

    for (size_t i = 0; i < motor_num_; ++i)
    {
        // actual torque command: PD controller
        // K_P*delta_Pos + K_W*delta_W + T
        motor_cmd_[i].id = id_[i];
        motor_cmd_[i].mode = mode;
        motor_cmd_[i].K_P = cmd[0];
        motor_cmd_[i].K_W = cmd[1];
    }
}

/**
 * @brief initialize the motor
 * @param  port
 */
void A1Control::init(std::vector<SerialPort*>& port)
{
    init_param_.resize(motor_num_);
    motor_cmd_.resize(motor_num_);
    motor_recv_.resize(motor_num_);
    motor_zero_position_.resize(motor_num_);
    for (size_t i = 0; i < motor_num_; ++i)
    {
        // initialization parameters
        init_param_[i].id = id_[i];
        init_param_[i].mode = 0;
        port[i]->sendRecv(&init_param_[i], &motor_recv_[i]);
        // get current position and set it as zero point
        motor_zero_position_[i] = motor_recv_[i].Pos;

        ROS_INFO("Zero position of motor A1[%lu]: %f", i, motor_zero_position_[i]);
    }

    ROS_INFO("Motor A1 initialization complete, start running!");
}

/**
 * @brief  set K_W and K_P and run the motor according to the control mode specified
 */
void A1Control::drive(std::vector<SerialPort*>& port)
{
    std::vector<float> control_param_vec(2, 0);
    float reduction_ratio = nh_.param("reduction_ratio", 9.0);
    int cmd_type = nh_.param("cmd_type", 0);
    int ctrl_frequency = nh_.param("motor_ctrl_data/ctrl_frequency", 200);
    ros::Rate loop_rate(ctrl_frequency);

    if (cmd_type == 0)
    {
        // position control
        // set K_W、K_P
        nh_.getParam("motor_ctrl_data/pos_kp_kw", control_param_vec);
        setCmd(control_param_vec);
        // set target position
        float add_goal_position = 0.0;
        std::vector<float> goal_position_vec{};
        nh_.getParam("motor_ctrl_data/goal_pos_vec", goal_position_vec);

        for (auto goal_position : goal_position_vec)
        {
            // 由于宇树电机使用单圈绝对值编码器，重启后编码器位置不会归零
            // 因此使用给电机上电时的编码器位置作为补偿,即用给电机上电时的电机位置作为零点，发送的角度为零点角度加上目标角度(多圈累加)
            add_goal_position += goal_position;
            for (size_t i = 0; i < motor_num_; ++i)
            {
                motor_cmd_[i].Pos = motor_zero_position_[i] + add_goal_position * reduction_ratio * M_PI / 180.0;
                port[i]->sendRecv(&motor_cmd_[i], &motor_recv_[i]);
                ROS_DEBUG("Received position of motor A1[%lu]: %f", i, motor_recv_[i].Pos);
            }
            usleep(500000);
        }
    }
    else if (cmd_type == 1)
    {
        // velocity control
        // set K_W、K_P
        nh_.getParam("motor_ctrl_data/vel_kp_kw", control_param_vec);
        setCmd(control_param_vec);
        // set target velocity
        // std::for_each(motor_cmd_.begin(), motor_cmd_.end(), [&](MotorCmd& cmd) {
        //     nh_.getParam("motor_ctrl_data/goal_vel", cmd.W);
        //     cmd.W *= reduction_ratio;
        // });

        while (ros::ok())
        {
            // send the command
            for (size_t i = 0; i < motor_num_; ++i)
            {
                port[i]->sendRecv(&motor_cmd_[i], &motor_recv_[i]);
                ROS_INFO("Received velocity of motor A1[%lu]: %f", i, motor_recv_[i].W);
            }

            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    else if (cmd_type == 2)
    {
        // torque control
        // set K_W、K_P
        nh_.getParam("motor_ctrl_data/trq_kp_kw", control_param_vec);
        setCmd(control_param_vec);
        // set target torque
        std::for_each(motor_cmd_.begin(), motor_cmd_.end(),
                      [&](MotorCmd& cmd) { nh_.getParam("motor_ctrl_data/goal_trq", cmd.T); });

        while (ros::ok())
        {
            for (size_t i = 0; i < motor_num_; ++i)
            {
                port[i]->sendRecv(&motor_cmd_[i], &motor_recv_[i]);
                ROS_INFO("Received torque of motor A1[%lu]: %f", i, motor_recv_[i].T);
            }
            loop_rate.sleep();
        }
    }
    else if (cmd_type == 3)
    {
        // bessel curve trajectory planning：no need to set a timer in while(ros::ok())
        // set K_W、K_P
        nh_.getParam("motor_ctrl_data/traj_kp_kw", control_param_vec);
        setCmd(control_param_vec);

        float total_run_time = 0, end_pos = 0, phase = 0, goal_position = 0;
        std::vector<float> bezier_plan_result(3, 0), goal_traj{}, traj_start_pos(motor_num_, 0);
        std::vector<int> current_traj_point(motor_num_, 0);

        nh_.getParam("interpolation/total_run_time", total_run_time);
        nh_.getParam("motor_ctrl_data/goal_traj", goal_traj);
        float per_traj_seg_run_time = total_run_time / goal_traj.size();  // run time of each trajectory segment

        // get the start time
        double start_time = ros::Time::now().toSec();
        // current time
        double right_now = 0;
        while (ros::ok())
        {
            for (size_t i = 0; i < motor_num_; ++i)
            {
                goal_position = 0;
                right_now = ros::Time::now().toSec();
                if ((right_now - start_time) < 0)
                {
                    ROS_FATAL("System Time Error!");
                }
                else if ((right_now - start_time) >= total_run_time)  // the entire trajectory finished
                {
                    motor_cmd_[i].Pos = motor_recv_[i].Pos;  // stay at the end of the trajectory
                    phase = (right_now - start_time - (per_traj_seg_run_time * (current_traj_point[i] - 1))) /
                            per_traj_seg_run_time;  // any number greater than 1 is acceptable
                    ROS_INFO("Motor A1[%lu] run complete!", i);
                }
                else if ((right_now - start_time) >
                         (double)(current_traj_point[i] * per_traj_seg_run_time))  // one trajectory segment finished,
                                                                                   // update for next segment
                {
                    ++current_traj_point[i];
                    traj_start_pos[i] = motor_recv_[i].Pos;
                }

                if (phase >= 0 && phase <= 1)
                {
                    // one segment isn't finished, continue planning
                    for (size_t j = 0; j < current_traj_point[i]; ++j)
                    {
                        goal_position += goal_traj[j];
                    }
                    end_pos = motor_zero_position_[i] + goal_position * reduction_ratio * M_PI / 180.0;
                    phase = (right_now - start_time - (per_traj_seg_run_time * (current_traj_point[i] - 1))) /
                            per_traj_seg_run_time;  // phase of each trajectory segment，0~1

                    bezier_plan_result =
                        interpolate::cubicBezierTrajPlanner(traj_start_pos[i], end_pos, phase, total_run_time);
                    ROS_INFO("Bezier planning result: pos:%f, vel:%f, acc:%f", bezier_plan_result[0],
                             bezier_plan_result[1], bezier_plan_result[2]);
                    motor_cmd_[i].Pos = bezier_plan_result[0];
                    motor_cmd_[i].W = bezier_plan_result[1];
                }

                port[i]->sendRecv(&motor_cmd_[i], &motor_recv_[i]);
                ROS_INFO("Motor A1[%lu] run time:%.6lf, phase:%f, pos:%f, vel:%f, startpos:%f, endpos:%f", i,
                         (right_now - start_time), phase, motor_recv_[i].Pos, motor_recv_[i].W, traj_start_pos[i],
                         end_pos);
            }
        }
    }
    else if (cmd_type == 4)
    {
        // quintic polynomial trajectory planning
        // set K_W、K_P
        nh_.getParam("motor_ctrl_data/traj_kp_kw", control_param_vec);
        setCmd(control_param_vec);

        std::vector<int> current_traj_point(motor_num_, 0);
        float total_run_time = 2, goal_position = 360;
        int point_num = ceil(ctrl_frequency * total_run_time) +
                        101;  // ensure that the control period is greater than the running time of each segment
        float per_traj_seg_run_time = total_run_time / (point_num - 1);

        std::vector<std::vector<std::vector<float>>> qp_plan_result{};
        for (size_t i = 0; i < motor_num_; ++i)
        {
            float end_pos = motor_zero_position_[i] + goal_position * reduction_ratio * M_PI / 180.0;
            std::vector<float> pos{ motor_zero_position_[i], end_pos }, vel{ 0, 0 },
                acc{ 0, 0 };  // start，end position data
            qp_plan_result.push_back(interpolate::quinticPolynomial(total_run_time, pos, vel, acc, point_num));
        }

        // get the start time
        double start_time = ros::Time::now().toSec();
        // current time
        double right_now = 0;
        while (ros::ok())
        {
            for (size_t i = 0; i < motor_num_; ++i)
            {
                right_now = ros::Time::now().toSec();
                if ((right_now - start_time) < 0)
                {
                    ROS_FATAL("Time Error");
                }
                else if (current_traj_point[i] >= point_num)  // the entire trajectory finished
                {
                    ROS_INFO("Motor A1[%lu] run complete!", i);
                    motor_cmd_[i].Pos = qp_plan_result[i].back()[0];
                }
                // If you don't use timer in while(ros::ok()), you need to use time to judge whether the waypoint is
                // completed or not; if you use timer, you don't need to judge whether the waypoint is completed or not,
                // but you need to make sure that the control period is greater than the running time of each segment of
                // the trajectory.
                // else if ((right_now - start_time) > static_cast<double>(current_traj_point * per_traj_seg_run_time))
                // // one trajectory segment finished, update for next segment
                // {
                ++current_traj_point[i];

                if (current_traj_point[i] < point_num)
                {
                    motor_cmd_[i].Pos = qp_plan_result[i][current_traj_point[i]][0];
                    motor_cmd_[i].W = qp_plan_result[i][current_traj_point[i]][1];
                    // instead of using the planned speeds, use the average of the speeds for each segment
                    // motor_cmd_.W = (motor_cmd_.Pos - motor_recv_.Pos) / per_traj_seg_run_time;
                }
                // }
                port[i]->sendRecv(&motor_cmd_[i], &motor_recv_[i]);
                ROS_INFO("Motor A1[%lu] run time:%.6lf, pos:%f, Vel:%f", i, (right_now - start_time),
                         motor_recv_[i].Pos, motor_recv_[i].W);
            }
            loop_rate.sleep();
        }
    }
    else
    {
        ROS_ERROR("Motor control type error!");
    }
}

/**
 * @brief stop the motor
 * @param  port
 */
void A1Control::stall(std::vector<SerialPort*>& port)
{
    for (size_t i = 0; i < motor_num_; ++i)
    {
        while (!(port[i]->sendRecv(&init_param_[i], &motor_recv_[i])))
        {
            usleep(100000);
        }
        ROS_DEBUG_STREAM("position: " << motor_recv_[i].Pos << motor_recv_[i].T);
    }

    ROS_INFO("End of motor A1 operation!");
}

/**
 * @brief open serial port and drive the motor
 */
void A1Control::operator()()
{
    // open serial port
    std::array<SerialPort, 4> serial_port{ SerialPort("/dev/ttyUSB0"), SerialPort("/dev/ttyUSB1"),
                                           SerialPort("/dev/ttyUSB2"), SerialPort("/dev/ttyUSB3") };

    std::vector<SerialPort*> serial{};
    for (auto iter = serial_port.begin(); iter != serial_port.end(); ++iter)
    {
        serial.push_back(iter);
    }

    init(serial);

    drive(serial);

    stall(serial);
}