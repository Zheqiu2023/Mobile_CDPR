#include "tension_sensors.hpp"
#include "general_file/tension_msgs.h"

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>
#include <ros/ros.h>
#include <ros/assert.h>
#include <stdlib.h>

using namespace tension_sensors;

TensionSensors::TensionSensors()
{
    ros::param::get("/tension_sensors/port_name", port_name_);
    ros::param::get("/tension_sensors/baud_rate", baud_rate_);
    pub_ = nh_.advertise<general_file::tension_msgs>("/tension_val", 100);
}

TensionSensors::~TensionSensors()
{
    close(fd_);
}

int TensionSensors::openPort(int& fd, const std::string& dev) const
{
    // O_NONBLOCK设置为非阻塞模式，在read时不会阻塞住，在读的时候要将read放在while循环中
    fd = open(dev.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);

    if (fd == -1)
    {
        perror("Can't Open SerialPort");
        return -1;
    }

    /*测试是否为终端设备*/
    if (isatty(STDIN_FILENO) == 0)
        printf("standard input is not a terminal device\n");
    else
        printf("isatty success!\n");
    printf("fd-open=%d\n", fd);

    return 0;
}

int TensionSensors::setPort(const int& fd, const int& nSpeed, const int& nBits, const char& nEvent,
                            const int& nStop) const
{
    struct termios newtio, oldtio;
    /*保存测试现有串口参数设置，在这里如果串口号等出错，会有相关的出错信息*/
    if (tcgetattr(fd, &oldtio) != 0)
    {
        perror("SetupSerial");
        printf("tcgetattr(fd, &oldtio) -> %d\n", tcgetattr(fd, &oldtio));
        return -1;
    }
    memset(&newtio, 0, sizeof(newtio));
    /*步骤一，设置字符大小*/
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;
    /*设置数据位数*/
    switch (nBits)
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
    /*设置奇偶校验位*/
    switch (nEvent)
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
    /*设置波特率*/
    switch (nSpeed)
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
    /*设置停止位*/
    if (nStop == 1)
        newtio.c_cflag &= ~CSTOPB;
    else if (nStop == 2)
        newtio.c_cflag |= CSTOPB;
    /*设置等待时间和最小接收字符*/
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;
    /*处理未接收字符*/
    tcflush(fd, TCIFLUSH);
    /*激活新配置*/
    if ((tcsetattr(fd, TCSANOW, &newtio)) != 0)
    {
        perror("com set error");
        return -1;
    }
    printf("com set done!\n");
    return 0;
}

void TensionSensors::start_read()
{
    int is_open = openPort(fd_, port_name_);
    ROS_ASSERT(is_open == 0);

    int is_set = setPort(fd_, baud_rate_, 8, 'N', 1);
    ROS_ASSERT(is_set == 0);

    char buf[10] = { 0 };  // 包括小数点、小数位、换行符在内不超过10位
    int is_complete_val = 0;
    general_file::tension_msgs tension{};
    std::string val = "";

    ros::Rate loop_rate(1000);
    while (ros::ok())
    {
        memset(buf, 0, sizeof(buf));
        if (read(fd_, buf, sizeof(buf)) > 0)
        {
            for (size_t i = 0; i < sizeof(buf); ++i)
            {
                if (buf[i] == '\n')
                {
                    is_complete_val = 1;
                    if (val != "")
                    {
                        tension.Force = atof(val.c_str());
                        pub_.publish(tension);
                        // printf("%.1lf\n", tension.Force);
                        val = "";
                        break;
                    }
                }
                else if (is_complete_val == 1)
                {
                    val += buf[i];
                }
            }
        }
        loop_rate.sleep();
    }
}
