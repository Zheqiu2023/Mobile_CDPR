#include "cdpr_hw/hw/comm_protocol.hpp"
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

    // open usb_can device：each device can only be open once
    if (dev_open_flag_[dev_ind_] == false)
    {
        if (VCI_OpenDevice(VCI_USBCAN2, dev_ind_, 0) != 1)
            ROS_ERROR_STREAM("Failed to open USB_CAN!");
        else
            dev_open_flag_[dev_ind_] = true;
    }

    // initialize CAN
    if (VCI_InitCAN(VCI_USBCAN2, dev_ind_, can_ind_, &config_) != 1)
    {
        VCI_CloseDevice(VCI_USBCAN2, dev_ind_);
        if (can_ind_ == CAN_IND0)
            ROS_ERROR_STREAM("Failed to initialize CAN1!");
        else
            ROS_ERROR_STREAM("Failed to initialize CAN2!");
        return;
    }
    VCI_ClearBuffer(VCI_USBCAN2, dev_ind_, can_ind_);

    // start CAN
    if (VCI_StartCAN(VCI_USBCAN2, dev_ind_, can_ind_) != 1)
    {
        VCI_CloseDevice(VCI_USBCAN2, dev_ind_);
        if (can_ind_ == CAN_IND0)
            ROS_ERROR_STREAM("Failed to open CAN1!");
        else
            ROS_ERROR_STREAM("Failed to open CAN2!");
        return;
    }
    ros::Duration(0.5).sleep();  // sleep for 0.5s to ensure can device function properly

    ROS_INFO_STREAM("Start device index " << dev_ind_ << " can index " << can_ind_ << " successfully.");
}

void CanBus::write()
{
    // safety first
    std::fill(std::begin(cmd_frame_.Data), std::end(cmd_frame_.Data), 0);

    for (auto& item : *data_ptr_.id2act_data_)
    {
        if (item.second.type.find("maxon") != std::string::npos)
        {  // maxon re35 motor
            if (item.second.halted)
                continue;
            const ActCoeff& act_coeff = data_ptr_.type2act_coeffs_->find(item.second.type)->second;
            int id = item.first - 0x201;
            double cmd = minAbs(act_coeff.effort2act * item.second.exe_effort,
                                act_coeff.max_out);  // add max_range to act_data
        }
        else if (item.second.type.find("stepper") != std::string::npos)
        {  // 57 stepper motor
            const ActCoeff& act_coeff = data_ptr_.type2act_coeffs_->find(item.second.type)->second;
            cmd_frame_.can_id = item.first;
            cmd_frame_.can_dlc = 8;
            uint16_t q_des = static_cast<int>(act_coeff.pos2act * (item.second.cmd_pos - act_coeff.act2pos_offset));
            uint16_t qd_des = static_cast<int>(act_coeff.vel2act * (item.second.cmd_vel - act_coeff.act2vel_offset));
            uint16_t kp = 0.;
            uint16_t kd = 0.;
            uint16_t tau =
                static_cast<int>(act_coeff.effort2act * (item.second.exe_effort - act_coeff.act2effort_offset));

            cmd_frame_.Data[0] = q_des >> 8;
            cmd_frame_.Data[1] = q_des & 0xFF;
            cmd_frame_.Data[2] = qd_des >> 4;
            cmd_frame_.Data[3] = ((qd_des & 0xF) << 4) | (kp >> 8);
            cmd_frame_.Data[4] = kp & 0xFF;
            cmd_frame_.Data[5] = kd >> 4;
            cmd_frame_.Data[6] = ((kd & 0xF) << 4) | (tau >> 8);
            cmd_frame_.Data[7] = tau & 0xff;
            socket_can_.write(&frame);
        }
    }

    if (has_write_frame0)
        socket_can_.write(&rm_frame0_);
    if (has_write_frame1)
        socket_can_.write(&rm_frame1_);
}

void CanBus::read(ros::Time time)
{
    std::lock_guard<std::mutex> guard(mutex_);
    for (const auto& frame_stamp : read_buffer_)
    {
        can_frame frame = frame_stamp.frame;
        // Check if robomaster motor
        if (data_ptr_.id2act_data_->find(frame.can_id) != data_ptr_.id2act_data_->end())
        {
            ActData& act_data = data_ptr_.id2act_data_->find(frame.can_id)->second;
            if ((frame_stamp.stamp - act_data.stamp).toSec() < 0.0005)
                continue;
            const ActCoeff& act_coeff = data_ptr_.type2act_coeffs_->find(act_data.type)->second;
            if (act_data.type.find("rm") != std::string::npos)
            {
                act_data.q_raw = (frame.data[0] << 8u) | frame.data[1];
                act_data.qd_raw = (frame.data[2] << 8u) | frame.data[3];
                int16_t cur = (frame.data[4] << 8u) | frame.data[5];
                act_data.temp = frame.data[6];

                // Multiple circle
                if (act_data.seq != 0)  // not the first receive
                {
                    if (act_data.q_raw - act_data.q_last > 4096)
                        act_data.q_circle--;
                    else if (act_data.q_raw - act_data.q_last < -4096)
                        act_data.q_circle++;
                }
                try
                {  // Duration will be out of dual 32-bit range while motor failure
                    act_data.frequency = 1. / (frame_stamp.stamp - act_data.stamp).toSec();
                }
                catch (std::runtime_error& ex)
                {
                }
                act_data.stamp = frame_stamp.stamp;
                act_data.seq++;
                act_data.q_last = act_data.q_raw;
                // Converter raw CAN data to position velocity and effort.
                act_data.pos = act_coeff.act2pos * static_cast<double>(act_data.q_raw + 8191 * act_data.q_circle) +
                               act_data.offset;
                act_data.vel = act_coeff.act2vel * static_cast<double>(act_data.qd_raw);
                act_data.effort = act_coeff.act2effort * static_cast<double>(cur);
                // Low pass filter
                act_data.lp_filter->input(act_data.vel, frame_stamp.stamp);
                act_data.vel = act_data.lp_filter->output();
                continue;
            }
        }
        // Check MIT Cheetah motor
        else if (frame.can_id == static_cast<unsigned int>(0x000))
        {
            if (data_ptr_.id2act_data_->find(frame.data[0]) != data_ptr_.id2act_data_->end())
            {
                ActData& act_data = data_ptr_.id2act_data_->find(frame.data[0])->second;
                const ActCoeff& act_coeff = data_ptr_.type2act_coeffs_->find(act_data.type)->second;
                if (act_data.type.find("cheetah") != std::string::npos)
                {  // MIT Cheetah Motor
                    act_data.q_raw = (frame.data[1] << 8) | frame.data[2];
                    uint16_t qd = (frame.data[3] << 4) | (frame.data[4] >> 4);
                    uint16_t cur = ((frame.data[4] & 0xF) << 8) | frame.data[5];
                    // Multiple cycle
                    // NOTE: The raw data range is -4pi~4pi
                    if (act_data.seq != 0)  // not the first receive
                    {
                        double pos_new = act_coeff.act2pos * static_cast<double>(act_data.q_raw) +
                                         act_coeff.act2pos_offset + static_cast<double>(act_data.q_circle) * 8 * M_PI +
                                         act_data.offset;
                        if (pos_new - act_data.pos > 4 * M_PI)
                            act_data.q_circle--;
                        else if (pos_new - act_data.pos < -4 * M_PI)
                            act_data.q_circle++;
                    }
                    try
                    {  // Duration will be out of dual 32-bit range while motor failure
                        act_data.frequency = 1. / (frame_stamp.stamp - act_data.stamp).toSec();
                    }
                    catch (std::runtime_error& ex)
                    {
                    }
                    act_data.stamp = frame_stamp.stamp;
                    act_data.seq++;
                    act_data.pos = act_coeff.act2pos * static_cast<double>(act_data.q_raw) + act_coeff.act2pos_offset +
                                   static_cast<double>(act_data.q_circle) * 8 * M_PI + act_data.offset;
                    // Converter raw CAN data to position velocity and effort.
                    act_data.vel = act_coeff.act2vel * static_cast<double>(qd) + act_coeff.act2vel_offset;
                    act_data.effort = act_coeff.act2effort * static_cast<double>(cur) + act_coeff.act2effort_offset;
                    // Low pass filter
                    act_data.lp_filter->input(act_data.vel);
                    act_data.vel = act_data.lp_filter->output();
                    continue;
                }
            }
        }

        if (frame.can_id != 0x0)
            ROS_ERROR_STREAM_ONCE("Can not find defined device, id: 0x" << std::hex << frame.can_id
                                                                        << " on bus: " << bus_name_);
    }
    read_buffer_.clear();
}

void CanBus::frameCallback(const can_frame& frame)
{
    std::lock_guard<std::mutex> guard(mutex_);
    CanFrameStamp can_frame_stamp{ .frame = frame, .stamp = ros::Time::now() };
    read_buffer_.push_back(can_frame_stamp);
}
