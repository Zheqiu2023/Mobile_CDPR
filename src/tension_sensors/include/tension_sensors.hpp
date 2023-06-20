/**
 * @File Name: tension_sensors.hpp
 * @brief
 * @author Zhe Qiu (zheqiu2021@163.com)
 * @version 0.1
 * @date 2023-06-20
 *
 * *  ***********************************************************************************
 * *  @copyright Copyright (c) 2023  by Zhe Qiu. All rights reserved.
 * *  Use of this source code is governed by the BSD 3-Clause license, see LICENSE.
 * *  ***********************************************************************************
 */
#pragma once

#include <string>
#include <vector>
#include <ros/ros.h>

namespace tension_sensors
{
class TensionSensors
{
  public:
    TensionSensors();
    ~TensionSensors();
    int openPort(int& fd, const std::string& dev) const;
    int setPort(const int& fd, const int& nSpeed, const int& nBits, const char& nEvent, const int& nStop) const;
    void start_read();

  private:
    int fd_;
    int baud_rate_;
    std::string port_name_;
    ros::NodeHandle nh_;
    ros::Publisher pub_;
};
}  // namespace tension_sensors