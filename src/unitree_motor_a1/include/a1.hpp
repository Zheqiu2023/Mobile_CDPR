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
class A1Control
{
  public:
    void init(std::vector<SerialPort*>& port);
    void drive(std::vector<SerialPort*>& port);
    void stall(std::vector<SerialPort*>& port);

  private:
    void setCmd(const std::vector<float>& cmd);
    int motor_num_ = 0, ctrl_frequency_ = 0;
    std::vector<int> id_{};
    std::vector<float> motor_zero_position_{};
    std::vector<MotorCmd> init_param_{}, motor_cmd_{};
    std::vector<MotorData> motor_recv_{};
};

class A1Run
{
  public:
    void operator()();

  private:
    // std::vector<std::string> serial_port_{};
    A1Control a1_control_{};
};
}  // namespace motor_a1