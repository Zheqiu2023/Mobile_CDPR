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
#include "a1.hpp"
#include "general_file/interpolation.hpp"

#include <vector>
#include <array>
#include <ros/ros.h>
#include <math.h>
#include <algorithm>

using namespace motor_a1;

/**
 * @brief 设置控制参数
 * @param  cmd
 */
void A1Control::setCmd(const std::vector<float>& cmd)
{
    int mode = 0;
    ros::param::get("/a1/motor_ctrl_data/mode", mode);

    for (size_t i = 0; i < motor_num_; ++i)
    {
        // 实际给FOC的指令力矩为：PD控制器
        // K_P*delta_Pos + K_W*delta_W + T
        motor_cmd_[i].id = id_[i];
        motor_cmd_[i].mode = mode;
        motor_cmd_[i].K_P = cmd[0];
        motor_cmd_[i].K_W = cmd[1];
    }
}

/**
 * @brief 初始化电机
 * @param  port
 */
void A1Control::init(std::vector<SerialPort*>& port)
{
    // 设置电机ID
    ros::param::get("/a1/id", id_);
    // 初始化
    ros::param::get("/a1/motor_num", motor_num_);
    init_param_.resize(motor_num_);
    motor_cmd_.resize(motor_num_);
    motor_recv_.resize(motor_num_);
    motor_zero_position_.resize(motor_num_);
    for (size_t i = 0; i < motor_num_; ++i)
    {
        // 电机初始化参数
        init_param_[i].id = id_[i];
        init_param_[i].mode = 0;
        // 电机初始化
        (*port[i]).sendRecv(&init_param_[i], &motor_recv_[i]);
        // 获取当前位置并作为零点
        motor_zero_position_[i] = motor_recv_[i].Pos;

        ROS_INFO("Zero position of motor A1[%lu]: %f", i, motor_zero_position_[i]);
    }

    ROS_INFO("Motor A1 initialization complete, start running!");
}

/**
 * @brief  根据控制模式设置参数K_W、K_P并运行电机
 */
void A1Control::drive(std::vector<SerialPort*>& port)
{
    std::vector<float> control_param_vec(2, 0);
    float reduction_ratio = 0.0;
    int cmd_type = 0, ctrl_frequency = 0;
    ros::param::get("/a1/cmd_type", cmd_type);
    ros::param::get("/a1/reduction_ratio", reduction_ratio);
    ros::param::get("/a1/motor_ctrl_data/ctrl_frequency", ctrl_frequency);
    ros::Rate loop_rate(ctrl_frequency);

    if (cmd_type == 0)
    {
        // 位置控制
        // 设置参数K_W、K_P
        ros::param::get("/a1/motor_ctrl_data/pos_kp_kw", control_param_vec);
        setCmd(control_param_vec);
        // 设置目标角度
        float add_goal_position = 0.0;
        std::vector<float> goal_position_vec{};
        ros::param::get("/a1/motor_ctrl_data/goal_pos_vec", goal_position_vec);

        for (auto goal_position : goal_position_vec)
        {
            // 由于宇树电机使用单圈绝对值编码器，重启后编码器位置不会归零
            // 因此使用给电机上电时的编码器位置作为补偿,即用给电机上电时的电机位置作为零点，发送的角度为零点角度加上目标角度(多圈累加)
            add_goal_position += goal_position;
            for (size_t i = 0; i < motor_num_; ++i)
            {
                motor_cmd_[i].Pos = motor_zero_position_[i] + add_goal_position * reduction_ratio * M_PI / 180.0;
                (*port[i]).sendRecv(&motor_cmd_[i], &motor_recv_[i]);
                ROS_DEBUG("Received position of motor A1[%lu]: %f", i, motor_recv_[i].Pos);
            }
            usleep(500000);
        }
    }
    else if (cmd_type == 1)
    {
        // 速度控制
        // 设置参数K_W、K_P
        ros::param::get("/a1/motor_ctrl_data/vel_kp_kw", control_param_vec);
        setCmd(control_param_vec);
        // 设置目标速度
        std::for_each(motor_cmd_.begin(), motor_cmd_.end(), [=](MotorCmd& cmd) {
            ros::param::get("/a1/motor_ctrl_data/goal_vel", cmd.W);
            cmd.W *= reduction_ratio;
        });

        while (ros::ok())
        {
            // 发送指令
            for (size_t i = 0; i < motor_num_; ++i)
            {
                (*port[i]).sendRecv(&motor_cmd_[i], &motor_recv_[i]);
                ROS_INFO("Received velocity of motor A1[%lu]: %f", i, motor_recv_[i].W);
            }
            loop_rate.sleep();
        }
    }
    else if (cmd_type == 2)
    {
        // 力矩控制
        // 设置参数K_W、K_P
        ros::param::get("/a1/motor_ctrl_data/trq_kp_kw", control_param_vec);
        setCmd(control_param_vec);
        // 设置目标力矩
        std::for_each(motor_cmd_.begin(), motor_cmd_.end(),
                      [](MotorCmd& cmd) { ros::param::get("/a1/motor_ctrl_data/goal_trq", cmd.T); });

        while (ros::ok())
        {
            for (size_t i = 0; i < motor_num_; ++i)
            {
                (*port[i]).sendRecv(&motor_cmd_[i], &motor_recv_[i]);
                ROS_INFO("Received torque of motor A1[%lu]: %f", i, motor_recv_[i].T);
            }
            loop_rate.sleep();
        }
    }
    else if (cmd_type == 3)
    {
        // 贝塞尔曲线轨迹规划：while(ros::ok())内无需定时器
        // 设置参数K_W、K_P
        ros::param::get("/a1/motor_ctrl_data/traj_kp_kw", control_param_vec);
        setCmd(control_param_vec);

        float total_run_time = 0, end_pos = 0, phase = 0, goal_position = 0;
        std::vector<float> bezier_plan_result(3, 0), goal_traj{}, traj_start_pos(motor_num_, 0);
        std::vector<int> current_traj_point(motor_num_, 0);

        ros::param::get("/a1/interpolation/total_run_time", total_run_time);
        ros::param::get("/a1/motor_ctrl_data/goal_traj", goal_traj);
        float per_traj_seg_run_time = total_run_time / goal_traj.size();  // 每段轨迹运行时间

        // 获取开始运行时间
        double start_time = ros::Time::now().toSec();
        // 当前时间
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
                else if ((right_now - start_time) >= total_run_time)  // 表示整条路径已经走完
                {
                    motor_cmd_[i].Pos = motor_recv_[i].Pos;  // 保持在终点位置不动
                    phase = (right_now - start_time - (per_traj_seg_run_time * (current_traj_point[i] - 1))) /
                            per_traj_seg_run_time;  // 任意大于1的数均可
                    ROS_INFO("Motor A1[%lu] run complete!", i);
                }
                else if ((right_now - start_time) >
                         (double)(current_traj_point[i] * per_traj_seg_run_time))  // 表示一个路段已完成,准备下一个路段
                {
                    ++current_traj_point[i];
                    traj_start_pos[i] = motor_recv_[i].Pos;
                }

                if (phase >= 0 && phase <= 1)
                {
                    // 一个路段未走完，继续规划
                    for (size_t j = 0; j < current_traj_point[i]; ++j)
                    {
                        goal_position += goal_traj[j];
                    }
                    end_pos = motor_zero_position_[i] + goal_position * reduction_ratio * M_PI / 180.0;
                    phase = (right_now - start_time - (per_traj_seg_run_time * (current_traj_point[i] - 1))) /
                            per_traj_seg_run_time;  // 此处phase为整段路径中的各段的phase，0~1

                    bezier_plan_result =
                        interpolate::cubicBezierTrajPlanner(traj_start_pos[i], end_pos, phase, total_run_time);
                    ROS_INFO("Bezier planning result: pos:%f, vel:%f, acc:%f", bezier_plan_result[0],
                             bezier_plan_result[1], bezier_plan_result[2]);
                    motor_cmd_[i].Pos = bezier_plan_result[0];
                    motor_cmd_[i].W = bezier_plan_result[1];
                }

                (*port[i]).sendRecv(&motor_cmd_[i], &motor_recv_[i]);
                ROS_INFO("Motor A1[%lu] run time:%.6lf, phase:%f, pos:%f, vel:%f, startpos:%f, endpos:%f", i,
                         (right_now - start_time), phase, motor_recv_[i].Pos, motor_recv_[i].W, traj_start_pos[i],
                         end_pos);
            }
        }
    }
    else if (cmd_type == 4)
    {
        // 五次多项式轨迹规划
        // 设置参数K_W、K_P
        ros::param::get("/a1/motor_ctrl_data/traj_kp_kw", control_param_vec);
        setCmd(control_param_vec);

        std::vector<int> current_traj_point(motor_num_, 0);
        float total_run_time = 2, goal_position = 360;
        int point_num = ceil(ctrl_frequency * total_run_time) + 101;  // 保证控制周期大于每小段轨迹运行时间
        float per_traj_seg_run_time = total_run_time / (point_num - 1);

        std::vector<std::vector<std::vector<float>>> qp_plan_result{};
        for (size_t i = 0; i < motor_num_; ++i)
        {
            float end_pos = motor_zero_position_[i] + goal_position * reduction_ratio * M_PI / 180.0;
            std::vector<float> pos{ motor_zero_position_[i], end_pos }, vel{ 0, 0 }, acc{ 0, 0 };  // 起始，结束位置信息
            qp_plan_result.push_back(interpolate::quinticPolynomial(total_run_time, pos, vel, acc, point_num));
        }

        // 获取开始运行时间
        double start_time = ros::Time::now().toSec();
        // 当前时间
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
                else if (current_traj_point[i] >= point_num)  // 表示整条路径已经走完
                {
                    ROS_INFO("Motor A1[%lu] run complete!", i);
                    motor_cmd_[i].Pos = qp_plan_result[i].back()[0];
                }
                // 若在while(ros::ok())内不使用定时器，需用时间判断路点是否完成；若使用定时器，无需判断路点是否完成，但需保证控制周期大于每小段轨迹运行时间
                // else if ((right_now - start_time) > static_cast<double>(current_traj_point *
                // per_traj_seg_run_time))
                // // 表示一个路点已完成, 准备下一个路点
                // {
                ++current_traj_point[i];

                if (current_traj_point[i] < point_num)
                {
                    motor_cmd_[i].Pos = qp_plan_result[i][current_traj_point[i]][0];
                    motor_cmd_[i].W = qp_plan_result[i][current_traj_point[i]][1];
                    // 不用规划的速度，用每段速度平均值
                    // motor_cmd_.W = (motor_cmd_.Pos - motor_recv_.Pos) / per_traj_seg_run_time;
                }
                // }
                (*port[i]).sendRecv(&motor_cmd_[i], &motor_recv_[i]);
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
 * @brief 停转
 * @param  port
 */
void A1Control::stall(std::vector<SerialPort*>& port)
{
    for (size_t i = 0; i < motor_num_; ++i)
    {
        while (!(*port[i]).sendRecv(&init_param_[i], &motor_recv_[i]))
        {
            usleep(100000);
        }
        ROS_DEBUG_STREAM("position: " << motor_recv_[i].Pos << motor_recv_[i].T);
    }

    ROS_INFO("End of motor A1 operation!");
}

/**
 * @brief 开串口并驱动电机
 */
void A1Control::operator()()
{
    if (!ros::param::get("/a1/port_name", serial_port_))
        ROS_ERROR("The serial port isn't set in the launch file!");

    // 打开串口
    std::array<SerialPort, 4> serial_port{ SerialPort("/dev/ttyUSB0"), SerialPort("/dev/ttyUSB1"),
                                           SerialPort("/dev/ttyUSB2"), SerialPort("/dev/ttyUSB3") };

    std::vector<SerialPort*> serial{};
    for (auto iter = serial_port.begin(); iter != serial_port.end(); ++iter)
    {
        serial.push_back(iter);
    }

    // 电机初始化
    init(serial);
    // 运行电机
    drive(serial);
    // 电机停转
    stall(serial);
}