/**
 * @File Name: tension_sensors.cpp
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
#include "tension_sensors.hpp"
#include "filter.hpp"

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>
#include <ros/ros.h>
#include <ros/assert.h>
#include <stdlib.h>
#include <set>
#include <std_msgs/Float64.h>

using namespace tension_sensors;

TensionSensors::TensionSensors()
{
    ros::param::get("/tension_sensors/port_name", port_name_);
    ros::param::get("/tension_sensors/baud_rate", baud_rate_);
    pub_ = nh_.advertise<std_msgs::Float64>("/tension_val", 100);
}

TensionSensors::~TensionSensors()
{
    close(fd_);
}

int TensionSensors::openPort(int& fd, const std::string& dev) const
{
    // O_NONBLOCK设置为非阻塞模式，在read时不会阻塞住，在读的时候要将read函数放在while循环中
    fd = open(dev.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);

    if (fd == -1)
    {
        perror("Can't open serialport");
        return -1;
    }

    // 测试是否为终端设备
    if (isatty(STDIN_FILENO) == 0)
        ROS_INFO("Standard input is not a terminal device!");
    else
        ROS_INFO("isatty success!");
    ROS_INFO("fd-open=%d", fd);

    return 0;
}

int TensionSensors::configPort(const int& fd, const int& baud_rate, const int& data_bytes, const char& check_bit,
                               const int& stop_bits) const
{
    struct termios newtio, oldtio;
    // 保存测试现有串口参数设置，在这里如果串口号等出错，会有相关的出错信息
    if (tcgetattr(fd, &oldtio) != 0)
    {
        perror("SetupSerial");
        ROS_INFO("tcgetattr(fd, &oldtio) -> %d", tcgetattr(fd, &oldtio));
        return -1;
    }
    memset(&newtio, 0, sizeof(newtio));
    // 步骤一，设置字符大小
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;
    // 设置数据位数
    switch (data_bytes)
    {
        case 7:
            newtio.c_cflag |= CS7;
            break;
        case 8:
            newtio.c_cflag |= CS8;
            break;
        default:
            break;
    }
    // 设置奇偶校验位
    switch (check_bit)
    {
        case 'o':
        case 'O':  // 奇数
            newtio.c_cflag |= PARENB;
            newtio.c_cflag |= PARODD;
            newtio.c_iflag |= (INPCK | ISTRIP);
            break;
        case 'e':
        case 'E':  // 偶数
            newtio.c_iflag |= (INPCK | ISTRIP);
            newtio.c_cflag |= PARENB;
            newtio.c_cflag &= ~PARODD;
            break;
        case 'n':
        case 'N':  // 无奇偶校验位
            newtio.c_cflag &= ~PARENB;
            break;
        default:
            break;
    }
    // 设置波特率
    switch (baud_rate)
    {
        case 2400:
            cfsetispeed(&newtio, B2400);
            cfsetospeed(&newtio, B2400);
            break;
        case 4800:
            cfsetispeed(&newtio, B4800);
            cfsetospeed(&newtio, B4800);
            break;
        case 9600:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
        case 115200:
            cfsetispeed(&newtio, B115200);
            cfsetospeed(&newtio, B115200);
            break;
        case 460800:
            cfsetispeed(&newtio, B460800);
            cfsetospeed(&newtio, B460800);
            break;
        default:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
    }
    // 设置停止位
    if (stop_bits == 1)
        newtio.c_cflag &= ~CSTOPB;
    else if (stop_bits == 2)
        newtio.c_cflag |= CSTOPB;
    // 设置等待时间和最小接收字符
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;
    // 处理未接收字符
    tcflush(fd, TCIFLUSH);
    // 激活新配置
    if ((tcsetattr(fd, TCSANOW, &newtio)) != 0)
    {
        perror("Serialport set error");
        return -1;
    }
    ROS_INFO("Serialport set done!\n");
    return 0;
}

void TensionSensors::startRead()
{
    int is_open = openPort(fd_, port_name_);
    ROS_ASSERT(is_open == 0);

    int is_set = configPort(fd_, baud_rate_, 8, 'N', 1);
    ROS_ASSERT(is_set == 0);

    unsigned char buf[10];  // 包括小数点、小数位、换行符在内不超过10位
    bool is_complete_val = false;
    std_msgs::Float64 tension{};
    std::string val = "";
    ros::Rate loop_rate(1000);
    while (ros::ok())
    {
        memset(buf, 0, sizeof(buf));
        if (read(fd_, buf, sizeof(buf)) > 0)
        {
            for (size_t i = 0; i < sizeof(buf); ++i)
            {
                // printf("%c", buf[i]);
                if (buf[i] == '\n')
                {
                    is_complete_val = true;
                    if (val != "")
                    {
                        // printf("val:%s\n", val.c_str());
                        tension.data = atof(val.c_str());
                        pub_.publish(tension);
                        // printf("%.1lf\n", tension.data);
                        val = "";
                        break;
                    }
                }
                else if (is_complete_val == true)
                {
                    val += buf[i];
                }
            }
        }
        loop_rate.sleep();
    }
}
