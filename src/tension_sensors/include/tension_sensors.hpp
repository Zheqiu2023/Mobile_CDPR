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