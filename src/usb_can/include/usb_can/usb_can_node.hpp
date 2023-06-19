#include <ros/ros.h>
#include <string>
#include <array>

#include "general_file/can_msgs.h"
#include "controlcan.h"
#include "usb_can.hpp"

namespace usb_can
{
/**
 * @brief
 * USBCAN只能在一个进程内使用，而多个节点都需要使用CAN通信，所以使用此节点作为中转站，发送和接收的CAN消息都需先经过此节点（尝试使用nodelet失败）
 */
class TransferStation
{
  public:
    TransferStation();
    void canCallback(const general_file::can_msgs::ConstPtr& msg, const int& can_ind);
    void printMsgs(const VCI_CAN_OBJ& msg) const;
    void publishMsgs();

  private:
    ros::NodeHandle nh_;
    ros::V_Subscriber subs_;
    ros::Publisher pub_;
    std::array<VCI_CAN_OBJ, 3000> recv_msgs_{};  // 接收缓存，设为3000为佳
};

TransferStation::TransferStation()
{
    subs_.push_back(nh_.subscribe<general_file::can_msgs>(
        "/usbcan/motor_42", 100, boost::bind(&TransferStation::canCallback, this, _1, CAN_IND0)));
    pub_ = nh_.advertise<general_file::can_msgs>("/usbcan/can_pub", 100);
}

void TransferStation::canCallback(const general_file::can_msgs::ConstPtr& msg, const int& can_ind)
{
    VCI_CAN_OBJ send_msgs{};  // 待发送消息(接收其他节点的消息然后发给电机)
    general_file::can_msgs temp_msgs{};

    temp_msgs = *msg;
    memcpy(&send_msgs, &temp_msgs, sizeof(temp_msgs));
    if (VCI_Transmit(VCI_USBCAN2, DEV_IND, can_ind, &send_msgs, 1) <= 0)
    {
        ROS_ERROR_STREAM("Failed to send command!");
    }
}

void TransferStation::publishMsgs()
{
    int recv_len = 0;                   // 接收到的消息长度
    general_file::can_msgs pub_msgs{};  // 待发布消息(接收电机的消息然后发布给其他节点)

    if ((recv_len = VCI_Receive(VCI_USBCAN2, DEV_IND, CAN_IND0, recv_msgs_.begin(), 3000, 100)) > 0)
    {
        for (size_t j = 0; j < recv_len; ++j)
        {
            printMsgs(recv_msgs_[j]);
            memcpy(&pub_msgs, &recv_msgs_[j], sizeof(recv_msgs_[j]));
            pub_.publish(pub_msgs);
        }
    }
    memset(&recv_msgs_, 0, sizeof(recv_msgs_));
}

/**
 * @brief 按固定格式打印收到的消息
 * @param  msg
 */
void TransferStation::printMsgs(const VCI_CAN_OBJ& msg) const
{
    printf("CAN%d RX ID:0x%08X", CAN_IND0 + 1, msg.ID);  // ID
    if (msg.ExternFlag == 0)
        printf(" Standard ");                            // 帧格式：标准帧
    if (msg.ExternFlag == 1)
        printf(" Extend   ");                            // 帧格式：扩展帧
    if (msg.RemoteFlag == 0)
        printf(" Data   ");                              // 帧类型：数据帧
    if (msg.RemoteFlag == 1)
        printf(" Remote ");                              // 帧类型：远程帧
    printf("DLC:0x%02X", msg.DataLen);                   // 帧长度
    printf(" data:0x");                                  // 数据
    for (size_t i = 0; i < msg.DataLen; ++i)
        printf(" %02X", msg.Data[i]);
    printf(" TimeStamp:0x%08X", msg.TimeStamp);  // 时间标识
    printf("\n");
}
}  // namespace usb_can