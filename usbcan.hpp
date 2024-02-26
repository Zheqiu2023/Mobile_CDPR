#pragma once

#include <QThread>
#include <array>

#include "common.hpp"

namespace usb_can
{
class UsbCan : public QObject
{
    Q_OBJECT

  public:
    UsbCan(QObject* parent = nullptr);
    ~UsbCan();
    void can_receive();

  private:
    void initCAN(const int& dev_type, const int& dev_ind, const int& can_ind, const MotorType& m_type);
    void setCANParam(const MotorType& m_type);
    void printMsg(unsigned int dev_ind, unsigned int can_ind, const VCI_CAN_OBJ& msg) const;

    double archor_pos_, cable_pos_;
    VCI_INIT_CONFIG config_;                     // 初始化参数，参考二次开发函数库说明书
    std::array<VCI_CAN_OBJ, 3000> recv_msgs_{};  // 接收缓存，设为3000为佳
    TrajParams params_;

  signals:
    void resetMsg(int id);
};
}  // namespace usb_can
