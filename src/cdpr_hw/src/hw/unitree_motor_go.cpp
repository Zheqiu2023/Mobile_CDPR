#include "cdpr_hw/hw/unitree_motor_go.hpp"
#include "general_file/math_utilities.hpp"

#include <string>
#include <ros/ros.h>

using namespace cdpr_hw;

UnitreeMotorGo::UnitreeMotorGo(const std::string& port_name, ActDataPtr data_ptr)
  : port_name_(port_name), data_ptr_(data_ptr)
{
    SerialPort serial(port_name);
    serial_ = std::make_unique<SerialPort>(serial);
    for (auto& item : *data_ptr_.id2act_data_)
    {
        send_cmd_.id = item.first;
        send_cmd_.mode = 0;
        // intialize motor
        serial_->sendRecv(&send_cmd_, &recv_msg_);
        // obtain the current position and use it as the zero point
        item.second.zero_point = recv_msg_.Pos;
        // intialize other parameters
        item.second.stamp = ros::Time::now();
        item.second.temp = recv_msg_.Temp;
        item.second.pos = recv_msg_.Pos;
        item.second.vel = recv_msg_.W;
        item.second.effort = recv_msg_.T;
        ROS_INFO("Zero point of usb port[%s] motor Go[%d]: %f", port_name_.c_str(), item.first, recv_msg_.Pos);
    }
}

void UnitreeMotorGo::read(ros::Time time)
{
    for (const auto& msg_stamp : recv_msg_stamp_buffer_)
    {
        MotorData data = msg_stamp.recv_msg;
        if (!data.correct)
        {
            ROS_WARN("data received from usb port[%s] unitree motor Go[%d] is not correct.", port_name_.c_str(),
                     data.motor_id);
            continue;
        }

        if (data_ptr_.id2act_data_->find(data.motor_id) != data_ptr_.id2act_data_->end())
        {
            ActData& act_data = data_ptr_.id2act_data_->find(data.motor_id)->second;
            if (act_data.seq == 0)
                continue;

            const ActCoeff& act_coeff = data_ptr_.type2act_coeffs_->find(act_data.type)->second;
            if (act_data.type.find("go") != std::string::npos)
            {
                try
                {  // Duration will be out of dual 32-bit range while motor failure
                    act_data.frequency = 1. / (msg_stamp.stamp - act_data.stamp).toSec();
                }
                catch (std::runtime_error& ex)
                {
                }
                act_data.stamp = msg_stamp.stamp;
                ++act_data.seq;
                act_data.temp = data.Temp;
                act_data.pos_last = act_data.pos;
                act_data.pos = data.Pos;
                act_data.vel = data.W;
                act_data.effort = data.T;
            }
        }
    }
    recv_msg_stamp_buffer_.clear();
}

void UnitreeMotorGo::write()
{
    for (auto& item : *data_ptr_.id2act_data_)
    {
        if (item.second.type.find("go") != std::string::npos)
        {
            if (item.second.halted)
                continue;
            const ActCoeff& act_coeff = data_ptr_.type2act_coeffs_->find(item.second.type)->second;

            send_cmd_.id = item.first;
            send_cmd_.K_P = act_coeff.kp;
            send_cmd_.K_W = act_coeff.kw;
            send_cmd_.mode = act_coeff.mode;
            // send_cmd_.Pos = item.second.cmd_pos;
            // send_cmd_.W = item.second.cmd_vel;
            send_cmd_.T = item.second.exe_effort;

            serial_->sendRecv(&send_cmd_, &recv_msg_);
            GoRecvMsgStamp recv_msg_stamp{ .recv_msg = recv_msg_, .stamp = ros::Time::now() };
            recv_msg_stamp_buffer_.push_back(recv_msg_stamp);
        }
    }
}