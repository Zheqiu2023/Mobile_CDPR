/**
 * @File Name: usb_can_node.hpp
 * @brief
 * @author Zhe Qiu (zheqiu2021@163.com)
 * @version 0.1
 * @date 2023-06-20
 *
 * ***********************************************************************************
 * @copyright Copyright (c) 2023  by Zhe Qiu. All rights reserved.
 * Use of this source code is governed by the BSD 3-Clause license, see LICENSE.
 * ***********************************************************************************
 */
#pragma once

#include <ros/ros.h>

#include <array>
#include <string>

#include "cdpr_bringup/CanCmd.h"
#include "cdpr_bringup/CanFrame.h"
#include "cdpr_bringup/usb_can/controlcan.h"
#include "usb_can.hpp"

namespace usb_can {
/**
 * @brief
 * USBCAN只能在一个进程内使用，而多个节点都需要使用CAN通信，所以使用此节点作为中转站，发送和接收的CAN消息都需先经过此节点
 */
class TransferStation {
   public:
    TransferStation(ros::NodeHandle& nh);
    void publishMsg();

   private:
    void cmdCallback(const cdpr_bringup::CanCmd::ConstPtr& msg);
    void printMsg(uint32_t dev_ind, uint32_t can_ind, const VCI_CAN_OBJ& msg) const;

    ros::NodeHandle nh_;
    ros::V_Subscriber subs_;
    ros::Publisher pub_;

    VCI_CAN_OBJ send_msg_{};  // 待发送消息(接收其他节点的消息然后发给电机)
    cdpr_bringup::CanCmd temp_msg_{};
    std::array<VCI_CAN_OBJ, 3000> recv_msgs_{};  // 接收缓存，设为3000为佳
};

TransferStation::TransferStation(ros::NodeHandle& nh) : nh_(nh) {
    // subs_.push_back(
    //     nh_.subscribe<cdpr_bringup::CanCmd>("/stepper_57/motor_cmd", 100, &TransferStation::cmdCallback, this));
    subs_.push_back(
        nh_.subscribe<cdpr_bringup::CanCmd>("/maxon_re35/motor_cmd", 100, &TransferStation::cmdCallback, this));
    subs_.push_back(
        nh_.subscribe<cdpr_bringup::CanCmd>("/movable_archor/motor_cmd", 100, &TransferStation::cmdCallback, this));
    pub_ = nh_.advertise<cdpr_bringup::CanFrame>("motor_state", 10);
}

void TransferStation::cmdCallback(const cdpr_bringup::CanCmd::ConstPtr& msg) {
    temp_msg_ = std::move(*msg);
    memcpy(&send_msg_, &temp_msg_.cmd, sizeof(temp_msg_.cmd));
    if (VCI_Transmit(VCI_USBCAN2, temp_msg_.dev_ind, temp_msg_.can_ind, &send_msg_, 1) <= 0) {
        ROS_ERROR("USBCAN%d CAN%d failed to send command!", temp_msg_.dev_ind, temp_msg_.can_ind);
    }
}

void TransferStation::publishMsg() {
    int recv_len = 0;                   // 接收到的消息长度
    cdpr_bringup::CanFrame pub_msgs{};  // 待发布消息(接收电机反馈的消息然后发布给其他节点)

    for (uint32_t dev_ind = 0; dev_ind < 2; ++dev_ind)
        for (uint32_t can_ind = 0; can_ind < 2; ++can_ind) {
            if ((recv_len = VCI_Receive(VCI_USBCAN2, dev_ind, can_ind, recv_msgs_.begin(), 3000, 100)) > 0) {
                for (size_t j = 0; j < recv_len; ++j) {
                    // printMsg(dev_ind, can_ind, recv_msgs_[j]);
                    memcpy(&pub_msgs, &recv_msgs_[j], sizeof(recv_msgs_[j]));
                    pub_.publish(pub_msgs);
                }
            }
            memset(&recv_msgs_, 0, sizeof(recv_msgs_));
            usleep(30000);
        }
}

/**
 * @brief 按固定格式打印收到的消息
 * @param  msg
 */
void TransferStation::printMsg(uint32_t dev_ind, uint32_t can_ind, const VCI_CAN_OBJ& msg) const {
    printf("DEV%d CAN%d RX ID:0x%08X", dev_ind, can_ind, msg.ID);  // ID
    if (msg.ExternFlag == 0)
        printf(" Standard ");  // 帧格式：标准帧
    else if (msg.ExternFlag == 1)
        printf(" Extend   ");  // 帧格式：扩展帧
    if (msg.RemoteFlag == 0)
        printf(" Data   ");  // 帧类型：数据帧
    else if (msg.RemoteFlag == 1)
        printf(" Remote ");             // 帧类型：远程帧
    printf("DLC:0x%02X", msg.DataLen);  // 帧长度
    printf(" data:0x");                 // 数据
    for (size_t i = 0; i < msg.DataLen; ++i) printf(" %02X", msg.Data[i]);
    printf(" TimeStamp:0x%08X", msg.TimeStamp);  // 时间戳
    printf("\n");
}
}  // namespace usb_can