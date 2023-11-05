#pragma once

#include "cdpr_bringup/CanFrame.h"
#include "cdpr_hw/hw/types.hpp"
#include "cdpr_bringup/usb_can/controlcan.h"

#include <chrono>
#include <mutex>
#include <thread>

/**
 * @brief communication interface between CdprHW and real robot through CAN
 */
namespace cdpr_hw
{

class CanBus
{
  public:
    /** \brief Start can device.
     *
     * \param dev_ind device index(0, 1).
     * \param can_ind can index(0, 1).
     * \param baud_rate baud rate(125k, 1M).
     * \param data_ptr Pointer which point to CAN data.
     */
    CanBus(const int& dev_ind, const int& can_ind, const int& baud_rate, ActDataPtr data_ptr);
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
    /** \brief This function will be called when CAN bus receive message. It push frame which received into a vector:
     * read_buffer_.
     *
     * @param frame The frame which socketcan receive.
     */
    void frameCallback(const cdpr_bringup::CanFrame& frame);

    const int dev_ind_ = 0, can_ind_ = 0, baud_rate_ = 0;
    VCI_INIT_CONFIG config_;

    ActDataPtr data_ptr_;
    cdpr_bringup::CanFrame cmd_frame_{};
    std::vector<cdpr_bringup::CanFrame> read_buffer_;

    mutable std::mutex mutex_;
};

}  // namespace cdpr_hw