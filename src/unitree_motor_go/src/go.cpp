/**
 * @File Name: go.cpp
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
#include "go.hpp"
#include "interpolation.hpp"

#include <ros/ros.h>
#include <vector>
#include <math.h>

using namespace motor_go;

/**
 * @brief 设置控制参数
 * @param  cmd
 */
void GoControl::setCmd(const std::vector<float>& cmd)
{
    // 电机控制时使用的参数
    int mode = 0;
    ros::param::get("/go/motor_ctrl_data/mode", mode);

    motor_cmd_.id = id_;
    motor_cmd_.mode = mode;
    motor_cmd_.K_P = cmd[0];
    motor_cmd_.K_W = cmd[1];
}

/**
 * @brief 初始化电机
 * @param  port
 */
void GoControl::init(SerialPort& port)
{
    // 电机初始化参数
    init_param_.id = id_;
    init_param_.mode = 0;  // 初始化时电机停转
    // 电机初始化
    port.sendRecv(&init_param_, &motor_recv_);
    motor_zero_position_ = motor_recv_.Pos;  // 获取当前位置并作为零点
    start_time_ = ros::Time::now().toSec();  // 获取开始运行时间

    ROS_INFO_STREAM("Initialization completed, start running!");
    ROS_DEBUG_STREAM("Zero position: " << motor_zero_position_);
}

/**
 * @brief 根据控制模式设置参数K_W、K_P并运行电机
 * @param  port
 */
void GoControl::drive(SerialPort& port, const int& ctrl_frequency)
{
    std::vector<float> control_param_vec{ 0, 0 };
    float reduction_ratio = 0.0;
    std::string is_stall = "";
    int cmd_type = 0;
    ros::param::get("/go/cmd_type", cmd_type);
    ros::param::get("/go/reduction_ratio", reduction_ratio);

    if (cmd_type == 0)
    {
        // 位置控制
        // 设置参数K_P、K_W
        ros::param::get("/go/motor_ctrl_data/pos_kp_kw", control_param_vec);
        setCmd(control_param_vec);
        // 设置目标角度
        float add_goal_position = 0.0;
        std::vector<float> goal_position_vec;
        ros::param::get("/go/motor_ctrl_data/goal_pos_vec", goal_position_vec);

        for (auto goal_position : goal_position_vec)
        {
            // 由于宇树电机使用单圈绝对值编码器，重启后编码器位置不会归零
            // 因此使用给电机上电时的编码器位置作为补偿,即用给电机上电时的电机位置作为零点，发送的角度为零点角度加上目标角度(多圈累加)
            add_goal_position += goal_position;
            motor_cmd_.Pos = motor_zero_position_ + add_goal_position * reduction_ratio * M_PI / 180.0;

            for (int i = 0; i < 10; ++i)
            {
                port.sendRecv(&motor_cmd_, &motor_recv_);
                usleep(50000);
            }
            ROS_INFO("Received position:%f", motor_recv_.Pos);
        }
    }
    else if (cmd_type == 1)
    {
        // 速度控制
        // 设置参数K_W、K_P
        ros::param::get("/go/motor_ctrl_data/vel_kp_kw", control_param_vec);
        setCmd(control_param_vec);

        // 设置目标速度
        ros::param::get("/go/motor_ctrl_data/goal_vel", motor_cmd_.W);
        motor_cmd_.W *= reduction_ratio;

        while (ros::ok())
        {
            port.sendRecv(&motor_cmd_, &motor_recv_);
            ROS_INFO_STREAM("Received velocity: " << motor_recv_.W);
        }
    }
    else if (cmd_type == 2)
    {
        // 力矩控制
        // 设置参数K_W、K_P
        ros::param::get("/go/motor_ctrl_data/trq_kp_kw", control_param_vec);
        setCmd(control_param_vec);

        // 设置目标力矩
        ros::param::get("/go/motor_ctrl_data/goal_trq", motor_cmd_.T);

        while (ros::ok())
        {
            port.sendRecv(&motor_cmd_, &motor_recv_);
            ROS_INFO_STREAM("Receiving torque: " << motor_recv_.T);
        }
    }
    else if (cmd_type == 3)
    {
        // 贝塞尔曲线轨迹规划：while(ros::ok())内无需定时器
        // 设置参数K_W、K_P
        ros::param::get("/go/motor_ctrl_data/traj_kp_kw", control_param_vec);
        setCmd(control_param_vec);

        float total_run_time = 0, end_pos = 0, phase = 0, goal_position = 0;
        std::vector<float> bezier_plan_result(3, 0), goal_traj{};

        ros::param::get("/go/interpolation/total_run_time", total_run_time);
        ros::param::get("/go/motor_ctrl_data/goal_traj", goal_traj);
        float per_traj_seg_run_time = total_run_time / goal_traj.size();  // 每段轨迹运行时间

        double right_now = ros::Time::now().toSec();
        if ((right_now - start_time_) < 0)
        {
            ROS_FATAL("System Time Error!");
        }
        else if ((right_now - start_time_) >= total_run_time)  // 表示整条路径已经走完
        {
            motor_cmd_.Pos = motor_recv_.Pos;                  // 保持在终点位置不动
            phase = (right_now - start_time_ - (per_traj_seg_run_time * (current_traj_point_ - 1))) /
                    per_traj_seg_run_time;                     // 任意大于1的数均可
            ROS_INFO("Motor Go run complete!");
        }
        else if ((right_now - start_time_) >
                 static_cast<double>(current_traj_point_ * per_traj_seg_run_time))  // 表示一个路点已完成,准备下一个路点
        {
            ++current_traj_point_;
            traj_start_pos_ = motor_recv_.Pos;
        }

        if (phase >= 0 && phase <= 1)
        {
            // 轨迹未走完，继续规划
            for (size_t i = 0; i < current_traj_point_; ++i)
            {
                goal_position += goal_traj[i];
            }
            end_pos = motor_zero_position_ + goal_position * reduction_ratio * M_PI / 180.0;
            phase = (right_now - start_time_ - (per_traj_seg_run_time * (current_traj_point_ - 1))) /
                    per_traj_seg_run_time;  // 此处phase为整段路径中的各段的phase，0~1

            bezier_plan_result = interpolate::cubicBezierTrajPlanner(traj_start_pos_, end_pos, phase, total_run_time);
            ROS_INFO("Bezier planning result: pos:%f, vel:%f, acc:%f", bezier_plan_result[0], bezier_plan_result[1],
                     bezier_plan_result[2]);
            motor_cmd_.Pos = bezier_plan_result[0];
            motor_cmd_.W = bezier_plan_result[1];
        }

        port.sendRecv(&motor_cmd_, &motor_recv_);
        ROS_INFO("run time:%.6lf, phase:%f, pos:%f, vel:%f, startpos:%f, endpos:%f", (right_now - start_time_), phase,
                 motor_recv_.Pos, motor_recv_.W, traj_start_pos_, end_pos);
    }
    else if (cmd_type == 4)
    {
        // 五次多项式轨迹规划
        // 设置参数K_W、K_P
        ros::param::get("/go/motor_ctrl_data/traj_kp_kw", control_param_vec);
        setCmd(control_param_vec);

        float total_run_time = 2, goal_position = 360;
        float end_pos = motor_zero_position_ + goal_position * reduction_ratio * M_PI / 180.0;
        std::vector<float> pos{ motor_zero_position_, end_pos }, vel{ 0, 0 }, acc{ 0, 0 };  // 起始，结束位置信息
        int point_num = ceil(ctrl_frequency * total_run_time) + 101;  // 保证控制周期大于每小段轨迹运行时间
        std::vector<std::vector<float>> bezier_plan_result =
            interpolate::quinticPolynomial(total_run_time, pos, vel, acc, point_num);

        double right_now = ros::Time::now().toSec();
        float per_traj_seg_run_time = total_run_time / (point_num - 1);
        if ((right_now - start_time_) < 0)
        {
            ROS_FATAL("Time Error");
        }
        else if (current_traj_point_ >= point_num)  // 表示整条路径已经走完
        {
            ROS_INFO("Motor Go run complete!");
            motor_cmd_.Pos = bezier_plan_result.back()[0];
        }
        // 若在while(ros::ok())内不使用定时器，需用时间判断路点是否完成；若使用定时器，无需判断路点是否完成，但需保证控制周期大于每小段轨迹运行时间
        // else if ((right_now - start_time_) > static_cast<double>(current_traj_point_ * per_traj_seg_run_time))
        // // 表示一个路点已完成, 准备下一个路点
        // {
        ++current_traj_point_;

        if (current_traj_point_ < point_num)
        {
            motor_cmd_.Pos = bezier_plan_result[current_traj_point_][0];
            motor_cmd_.W = bezier_plan_result[current_traj_point_][1];
            ROS_INFO("send pos:%f, vel:%f", motor_cmd_.Pos, motor_cmd_.W);
            // 不用规划的速度，用每段速度平均值
            // motor_cmd_.W = (motor_cmd_.Pos - motor_recv_.Pos) / per_traj_seg_run_time;
        }
        // }
        port.sendRecv(&motor_cmd_, &motor_recv_);
        ROS_INFO("run time:%.6lf, pos:%f, vel:%f", (right_now - start_time_), motor_recv_.Pos, motor_recv_.W);
    }
    else
    {
        ROS_ERROR_STREAM("Motor control type error!");
    }
}

/**
 * @brief 停转
 * @param  port
 */
void GoControl::stall(SerialPort& port)
{
    while (!port.sendRecv(&init_param_, &motor_recv_))
    {
        usleep(100000);
    }

    ROS_DEBUG_STREAM("position: " << motor_recv_.Pos << motor_recv_.T);
    ROS_INFO_STREAM("End of run!");
}

/**
 * @brief 设置ID
 * @param  id
 */
void GoControl::setID(int id)
{
    id_ = id;
}

/**
 * @brief Construct a new motor go::GoRun::GoRun object
 * @param  id
 */
GoRun::GoRun(int id)
{
    if (!ros::param::get("/go/port_name", serial_port_))
        ROS_WARN_STREAM("The serial port isn't set in the launch file, the default setting is " << serial_port_);
    go_control_.setID(id);
}

/**
 * @brief 开串口并驱动电机
 */
void GoRun::operator()()
{
    SerialPort serial(serial_port_);  // 打开串口
    go_control_.init(serial);

    ros::param::get("/go/motor_ctrl_data/ctrl_frequency", ctrl_frequency_);

    ros::Rate loop_rate(ctrl_frequency_);
    while (ros::ok())
    {
        go_control_.drive(serial, ctrl_frequency_);
        loop_rate.sleep();
    }

    go_control_.stall(serial);
}