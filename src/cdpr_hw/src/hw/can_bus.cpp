#include "cdpr_hw/hw/can_bus.hpp"
#include "general_file/math_utilities.hpp"

#include <string>
#include <ros/ros.h>

using namespace cdpr_hw;

CanBus::CanBus(const int& dev_ind, const int& can_ind, const int& baud_rate, ActDataPtr data_ptr)
  : dev_ind_(dev_ind), can_ind_(can_ind), baud_rate_(baud_rate), data_ptr_(data_ptr)
{
    // configure CAN
    if (baud_rate_ == 125000)
    {
        config_.Timing0 = 0x03;
        config_.Timing1 = 0x1C;
    }
    else if (baud_rate_ == 1000000)
    {
        config_.Timing0 = 0x00;
        config_.Timing1 = 0x14;
    }
    config_.AccCode = 0;
    config_.AccMask = 0xFFFFFFFF;  // recommend setting: 0xFFFFFFFF，that is receiving all messages
    config_.Filter = 2;            // receive standard frame
    config_.Mode = 0;              // normal mode

    // initialize CAN
    if (VCI_InitCAN(VCI_USBCAN2, dev_ind_, can_ind_, &config_) != 1)
    {
        VCI_CloseDevice(VCI_USBCAN2, dev_ind_);
        ROS_ERROR("Failed to initialize device[%d] CAN[%d]!", dev_ind_, can_ind_);
    }
    VCI_ClearBuffer(VCI_USBCAN2, dev_ind_, can_ind_);

    // start CAN
    if (VCI_StartCAN(VCI_USBCAN2, dev_ind_, can_ind_) != 1)
    {
        VCI_CloseDevice(VCI_USBCAN2, dev_ind_);
        ROS_ERROR("Failed to start device[%d] CAN[%d]!", dev_ind_, can_ind_);
    }
    ros::Duration(0.5).sleep();  // sleep for 0.5s to ensure can device function properly

    ROS_INFO_STREAM("Start device " << dev_ind_ << " can " << can_ind_ << " successfully.");
}

void CanBus::read(ros::Time time)
{
}

void CanBus::write()
{
}

void CanBus::frameCallback(const general_file::CanFrame& frame)
{
}
