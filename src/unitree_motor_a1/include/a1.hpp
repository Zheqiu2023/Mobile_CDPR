/**
 * @File Name: a1.hpp
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

#pragma once
#include "serialPort/SerialPort.h"

namespace motor_a1
{
constexpr float PI = 3.1415926;

class A1Control
{
  public:
    void init(SerialPort& port);
    void drive(SerialPort& port, const int& ctrl_frequency);
    void stall(SerialPort& port);
    void setID(int id);

  private:
    void setCmd(const std::vector<float>& cmd);
    int id_ = 0, current_traj_point_ = 0;  // 表示a1电机当前处于哪个路点
    float motor_zero_position_ = 0, traj_start_pos_ = 0;
    double start_time_ = 0;
    MotorCmd init_param_{}, motor_cmd_{};
    MotorData motor_recv_{};
};

class A1Run
{
  public:
    A1Run(int id);
    void operator()();

  private:
    int ctrl_frequency_ = 0;
    std::string serial_port_{ "/dev/ttyUSB0" };
    A1Control a1_control_{};
};
}  // namespace motor_a1