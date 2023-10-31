#pragma once

#include "cdpr_hw/hw/types.hpp"
#include "unitree_motor_go/serialPort/SerialPort.h"

#include <chrono>
#include <mutex>
#include <thread>

namespace cdpr_hw
{

struct GoRecvMsgStamp
{
    MotorData recv_msg;
    ros::Time stamp;
};

class UnitreeMotorGo
{
  public:
    /** \brief Open serial port
     *
     * \param port_name /dev/ttyUSB*.
     * \param data_ptr Pointer which point to CAN data.
     */
    UnitreeMotorGo(const std::string& port_name, ActDataPtr data_ptr);
    /** \brief Read active data from read_buffer_ to data_ptr_, such as position, velocity, torque and so on. Clear
     * read_buffer_ after reading.
     *
     * \param time ROS time, but it doesn't be used.
     */
    void read(ros::Time time);
    /** \brief Write commands to can bus.
     *
     */
    void write();

  private:
    const std::string port_name_;
    std::unique_ptr<SerialPort> serial_;

    ActDataPtr data_ptr_;
    MotorCmd send_cmd_{};
    MotorData recv_msg_{};
    std::vector<GoRecvMsgStamp> recv_msg_stamp_buffer_{};

    mutable std::mutex mutex_;
};

}  // namespace cdpr_hw
