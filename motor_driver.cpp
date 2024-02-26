#include <unistd.h>
#include <algorithm>
#include <vector>
#include <QDebug>
#include <unistd.h>
#include <QtGlobal>
#include <cmath>
#include <QTime>

#include "motor_driver.hpp"

using namespace motor_driver;

void BaseDriver::stop()
{
    is_stop_ = true;
}

void BaseDriver::sleep_ms(unsigned int msec)
{
    QTime reachTime = QTime::currentTime().addMSecs(msec);
    while (QTime::currentTime() < reachTime)
        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
}

void BaseDriver::setVel(VCI_CAN_OBJ& cmd, const int& driver_id, const int& target_vel)
{
    cmd.ID = 0x004 | (driver_id << 4);  // 速度模式下的参数指令，非广播
    cmd.Data[0] = static_cast<unsigned char>((PWM_LIM >> 8) & 0xff);
    cmd.Data[1] = static_cast<unsigned char>(PWM_LIM & 0xff);
    cmd.Data[2] = static_cast<unsigned char>((target_vel >> 8) & 0xff);
    cmd.Data[3] = static_cast<unsigned char>(target_vel & 0xff);
}

void BaseDriver::setVelPos(VCI_CAN_OBJ& cmd, const int& driver_id, const int& target_vel, const int& target_pos)
{
    cmd.ID = 0x006 | (driver_id << 4);  // 速度位置模式下的参数指令，非广播
    cmd.Data[0] = static_cast<unsigned char>((PWM_LIM >> 8) & 0xff);
    cmd.Data[1] = static_cast<unsigned char>(PWM_LIM & 0xff);
    cmd.Data[2] = static_cast<unsigned char>((target_vel >> 8) & 0xff);
    cmd.Data[3] = static_cast<unsigned char>(target_vel & 0xff);
    cmd.Data[4] = static_cast<unsigned char>((target_pos >> 24) & 0xff);
    cmd.Data[5] = static_cast<unsigned char>((target_pos >> 16) & 0xff);
    cmd.Data[6] = static_cast<unsigned char>((target_pos >> 8) & 0xff);
    cmd.Data[7] = static_cast<unsigned char>(target_pos & 0xff);
}

void BaseDriver::sendCmd(CanCmd& cmd_struct)
{
    if (VCI_Transmit(VCI_USBCAN2, cmd_struct.dev_ind, cmd_struct.can_ind, &cmd_struct.cmd, 1) <= 0)
    {
        qCritical() << "USBCAN" << cmd_struct.dev_ind << " CAN" << cmd_struct.can_ind << " failed to send command!";
    }
}

ArchorDriver::ArchorDriver(QObject* parent)
{
    Q_ASSERT(4 == dev_ind_.size() && 4 == can_ind_.size() && 4 == params_.archor_id_.size() &&
             4 == params_.archor_direction_.size());
    for (size_t i = 0; i < 4; ++i)
    {
        CanCmd pub_cmd{};
        pub_cmd.dev_ind = dev_ind_[i];
        pub_cmd.can_ind = can_ind_[i];
        pub_cmd.cmd.SendType = 0;    // automatically retransmit after a failed send
        pub_cmd.cmd.RemoteFlag = 0;  // 0 for data frame, 1 for remote frame
        pub_cmd.cmd.ExternFlag = 0;  // 0 for standard frame, 1 for expanded frame
        pub_cmd.cmd.DataLen = 8;     // Data length 8 bytes

        motor_data_.push_back(ArchorData{ .is_reset_ = false,
                                          .driver_id_ = params_.archor_id_[i],
                                          .direction_ = params_.archor_direction_[i],
                                          .target_pos_ = 0.0,
                                          .last_pos_ = 0.0,
                                          .pub_cmd_ = std::move(pub_cmd) });
    }

    sleep_ms(1000);  // Sleep for 1s to ensure that the first message sent is received by USBCAN
}

void ArchorDriver::init(RunMode mode, const int& period)
{
    for (auto& motor_data : motor_data_)
    {
        // 发送复位指令
        motor_data.pub_cmd_.cmd.ID =
            0x000 | (motor_data.driver_id_ << 4);  // 复位指令（帧ID，由驱动器编号和功能序号决定）
        for (auto& data : motor_data.pub_cmd_.cmd.Data)
            data = 0x55;
        sendCmd(motor_data.pub_cmd_);
    }
    sleep_ms(500);
    for (auto& motor_data : motor_data_)
    {
        // 发送模式选择指令
        motor_data.pub_cmd_.cmd.ID = 0x001 | (motor_data.driver_id_ << 4);   // 模式选择指令
        motor_data.pub_cmd_.cmd.Data[0] = static_cast<unsigned char>(mode);  // 选择mode对应模式
        sendCmd(motor_data.pub_cmd_);
    }
    sleep_ms(500);
    for (auto& motor_data : motor_data_)
    {
        // 发送配置指令
        motor_data.pub_cmd_.cmd.ID = 0x00A | (motor_data.driver_id_ << 4);  // 配置指令
        motor_data.pub_cmd_.cmd.Data[0] = 0x01;    // 以 1 毫秒为周期对外发送电流、速度、位置等信息
        motor_data.pub_cmd_.cmd.Data[1] = period;  // 以 period 毫秒为周期对外发送CTL1/CTL2的电平状态
        sendCmd(motor_data.pub_cmd_);
    }
    sleep_ms(500);
}

void ArchorDriver::setSendVel(const unsigned int& i, const int& vel)
{
    setVel(motor_data_[i].pub_cmd_.cmd, motor_data_[i].driver_id_, vel);
    sendCmd(motor_data_[i].pub_cmd_);
}

void ArchorDriver::setSendVelPos(const unsigned int& i, const int& vel, const int& pos)
{
    setVelPos(motor_data_[i].pub_cmd_.cmd, motor_data_[i].driver_id_, vel, pos);
    sendCmd(motor_data_[i].pub_cmd_);
}

void ArchorDriver::recvResetMsg(const int& id)
{
    for (auto& motor_data : motor_data_)
    {
        if (id == motor_data.driver_id_)
            motor_data.is_reset_ = true;
    }
}

void ArchorDriver::reset(const int& vel)
{
    init(RunMode::VEL, 0x05);

    for (auto& motor_data : motor_data_)
    {
        setVel(motor_data.pub_cmd_.cmd, motor_data.driver_id_, vel * motor_data.direction_);
        sendCmd(motor_data.pub_cmd_);
    }

    qInfo() << "Anchors reseting...";
    // Reset: move to bottom
    while (!is_stop_)
    {
        for (auto& motor_data : motor_data_)
        {
            if (true == motor_data.is_reset_)
            {
                for (int i = 0; i < 4; ++i)
                    motor_data.pub_cmd_.cmd.Data[i] = 0;
                sendCmd(motor_data.pub_cmd_);
            }
        }

        if (std::all_of(motor_data_.begin(), motor_data_.end(), [](const ArchorData& motor_data) {
                return motor_data.is_reset_;
            }))  // all archores reset successfully
            break;
    }
}

/**
 * @brief 运行轨迹
 */
void ArchorDriver::run_traj(const int& vel, const double& period)
{
    init(RunMode::VEL, 0x05);

    for (auto& motor_data : motor_data_)
    {
        setVel(motor_data.pub_cmd_.cmd, motor_data.driver_id_, vel * motor_data.direction_);
        sendCmd(motor_data.pub_cmd_);
    }

    qInfo() << "Anchors reseting...";
    // Reset: move to bottom
    while (!is_stop_)
    {
        for (auto& motor_data : motor_data_)
        {
            if (true == motor_data.is_reset_)
            {
                for (int i = 0; i < 4; ++i)
                    motor_data.pub_cmd_.cmd.Data[i] = 0;
                sendCmd(motor_data.pub_cmd_);
            }
        }

        if (std::all_of(motor_data_.begin(), motor_data_.end(), [](const ArchorData& motor_data) {
                return motor_data.is_reset_;
            }))  // all archores reset successfully
            break;
    }

    int cmd_pos = 0.0, cmd_vel = 0.0;  // 速度限制值(RPM)：0~32767
    init(RunMode::VEL_POS, 0x00);

    emit resetFinished();
    qInfo() << "Movable archor reset done, ready to follow the trajectory!";

    // follow the trajectory
    while (!is_stop_)
    {
        for (auto& motor_data : motor_data_)
        {
            cmd_pos = motor_data.target_pos_ * 1000 * params_.reduction_ratio_ * params_.encoder_lines_num_ /
                      params_.lead_;  // m转换为qc
            cmd_vel = std::fabs((motor_data.target_pos_ - motor_data.last_pos_) * 60 * 1000 * params_.reduction_ratio_ /
                                (period * params_.lead_));

            setVelPos(motor_data.pub_cmd_.cmd, motor_data.driver_id_, cmd_vel, cmd_pos);
            sendCmd(motor_data.pub_cmd_);
        }

        if (is_traj_end_)
            break;
    }

    sleep_ms(2000);  // buffering time for archors moving back to zero position
}

CableDriver::CableDriver(QObject* parent)
{
    Q_ASSERT(4 == dev_ind_.size() && 4 == can_ind_.size() && 4 == params_.cable_id_.size() &&
             4 == params_.cable_direction_.size());
    for (size_t i = 0; i < 4; ++i)
    {
        CanCmd pub_cmd{};
        pub_cmd.dev_ind = dev_ind_[i];
        pub_cmd.can_ind = can_ind_[i];
        pub_cmd.cmd.SendType = 0;    // automatically retransmit after a failed send
        pub_cmd.cmd.RemoteFlag = 0;  // 0 for data frame, 1 for remote frame
        pub_cmd.cmd.ExternFlag = 0;  // 0 for standard frame, 1 for expanded frame
        pub_cmd.cmd.DataLen = 8;     // Data length 8 bytes

        motor_data_.push_back(CableData{ .driver_id_ = params_.cable_id_[i],
                                         .direction_ = params_.cable_direction_[i],
                                         .target_pos_ = 0.0,
                                         .last_pos_ = 0.0,
                                         .pub_cmd_ = std::move(pub_cmd) });
    }

    sleep_ms(1000);  // Sleep for 1s to ensure that the first message sent is received by USBCAN
}

void CableDriver::init(RunMode mode)
{
    for (auto& motor_data : motor_data_)
    {
        // 发送复位指令
        motor_data.pub_cmd_.cmd.ID =
            0x000 | (motor_data.driver_id_ << 4);  // 复位指令（帧ID，由驱动器编号和功能序号决定）
        for (auto& data : motor_data.pub_cmd_.cmd.Data)
            data = 0x55;
        sendCmd(motor_data.pub_cmd_);
    }
    sleep_ms(500);
    for (auto& motor_data : motor_data_)
    {
        // 发送模式选择指令
        motor_data.pub_cmd_.cmd.ID = 0x001 | (motor_data.driver_id_ << 4);   // 模式选择指令
        motor_data.pub_cmd_.cmd.Data[0] = static_cast<unsigned char>(mode);  // 选择mode对应模式
        sendCmd(motor_data.pub_cmd_);
    }
    sleep_ms(500);
    for (auto& motor_data : motor_data_)
    {
        // 发送配置指令
        motor_data.pub_cmd_.cmd.ID = 0x00A | (motor_data.driver_id_ << 4);  // 配置指令
        motor_data.pub_cmd_.cmd.Data[0] = 0x01;  // 以 1 毫秒为周期对外发送电流、速度、位置等信息
        motor_data.pub_cmd_.cmd.Data[1] = 0x00;
        sendCmd(motor_data.pub_cmd_);
    }
    sleep_ms(500);
}

void CableDriver::setSendVel(const unsigned int& i, const int& vel)
{
    setVel(motor_data_[i].pub_cmd_.cmd, motor_data_[i].driver_id_, vel);
    sendCmd(motor_data_[i].pub_cmd_);
}

void CableDriver::setSendVelPos(const unsigned int& i, const int& vel, const int& pos)
{
    setVelPos(motor_data_[i].pub_cmd_.cmd, motor_data_[i].driver_id_, vel, pos);
    sendCmd(motor_data_[i].pub_cmd_);
}

/**
 * @brief 运行轨迹
 */
void CableDriver::run_traj(const double& period)
{
    init(RunMode::VEL_POS);

    int cmd_pos = 0.0, cmd_vel = 0.0;  // 速度限制值(RPM)：0~32767

    emit resetFinished();
    qInfo() << "Cables reset done, ready to follow the trajectory!";

    // follow the trajectory
    while (!is_stop_)
    {
        for (auto& motor_data : motor_data_)
        {
            cmd_pos = std::round(motor_data.target_pos_ * params_.reduction_ratio_ * params_.encoder_lines_num_ /
                                 (M_PI * params_.reel_diameter_));  // m转换为qc
            cmd_vel = std::ceil(std::fabs((motor_data.target_pos_ - motor_data.last_pos_) * 60 *
                                          params_.reduction_ratio_ / (period * M_PI * params_.reel_diameter_)));

            setVelPos(motor_data.pub_cmd_.cmd, motor_data.driver_id_, cmd_vel, cmd_pos);
            sendCmd(motor_data.pub_cmd_);
        }
        if (is_traj_end_)
            break;
    }

    sleep_ms(2000);  // buffering time for motors moving back to zero position
}
