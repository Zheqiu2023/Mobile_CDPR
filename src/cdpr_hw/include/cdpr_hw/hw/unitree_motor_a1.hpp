#pragma once

#include "cdpr_hw/hw/types.hpp"
#include "unitree_motor_a1/serialPort/SerialPort.h"

#include <chrono>
#include <mutex>
#include <thread>

namespace cdpr_hw
{

struct A1RecvMsgStamp
{
    MotorData recv_msg;
    ros::Time stamp;
};

class UnitreeMotorA1
{
  public:
    /** \brief Open serial port
     *
     * \param port_name /dev/ttyUSB*.
     * \param data_ptr Pointer which point to CAN data.
     */
    UnitreeMotorA1(const std::string& port_name, ActDataPtr data_ptr);
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
    void createSerial();

    const std::string port_name_;
    std::shared_ptr<SerialPort> serial_;

    ActDataPtr data_ptr_;
    MotorCmd send_cmd_{};
    MotorData recv_msg_{};
    std::vector<A1RecvMsgStamp> recv_msg_stamp_buffer_{};

    mutable std::mutex mutex_;
};

}  // namespace cdpr_hw
