#include <vector>
#include <QDebug>
#include <QtGlobal>
#include <cmath>
#include <QTime>
#include <QDir>
#include <QElapsedTimer>

#include "motor_driver.hpp"

using namespace motor_driver;

void BaseDriver::stopRun()
{
    stop_run_ = true;
}

void BaseDriver::setCur(VCI_CAN_OBJ& cmd, const int& driver_id, const int& target_cur)
{
    cmd.ID = 0x003 | (driver_id << 4);  // 电流模式下的参数指令，非广播
    cmd.Data[0] = static_cast<unsigned char>((PWM_LIM >> 8) & 0xff);
    cmd.Data[1] = static_cast<unsigned char>(PWM_LIM & 0xff);
    cmd.Data[2] = static_cast<unsigned char>((target_cur >> 8) & 0xff);
    cmd.Data[3] = static_cast<unsigned char>(target_cur & 0xff);
    for (int i = 4; i < 8; ++i)
        cmd.Data[i] = 0x55;
}

void BaseDriver::setVel(VCI_CAN_OBJ& cmd, const int& driver_id, const int& target_vel)
{
    cmd.ID = 0x004 | (driver_id << 4);  // 速度模式下的参数指令，非广播
    cmd.Data[0] = static_cast<unsigned char>((PWM_LIM >> 8) & 0xff);
    cmd.Data[1] = static_cast<unsigned char>(PWM_LIM & 0xff);
    cmd.Data[2] = static_cast<unsigned char>((target_vel >> 8) & 0xff);
    cmd.Data[3] = static_cast<unsigned char>(target_vel & 0xff);
    for (int i = 4; i < 8; ++i)
        cmd.Data[i] = 0x55;
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
    if (VCI_Transmit(VCI_USBCAN2, cmd_struct.dev_ind_, cmd_struct.can_ind_, &cmd_struct.cmd_, 1) <= 0)
        qCritical() << "USBCAN" << cmd_struct.dev_ind_ << " CAN" << cmd_struct.can_ind_ << " failed to send command!";
}

ArchorDriver::ArchorDriver(QObject* parent)
{
    Q_ASSERT(4 == dev_ind_.size() && 4 == can_ind_.size() && 4 == params_.archor_id_.size() &&
             4 == params_.archor_direction_.size());
    for (size_t i = 0; i < 4; ++i)
    {
        CanCmd pub_cmd{};
        pub_cmd.dev_ind_ = dev_ind_[i];
        pub_cmd.can_ind_ = can_ind_[i];
        pub_cmd.cmd_.SendType = 0;    // automatically retransmit after a failed send
        pub_cmd.cmd_.RemoteFlag = 0;  // 0 for data frame, 1 for remote frame
        pub_cmd.cmd_.ExternFlag = 0;  // 0 for standard frame, 1 for expanded frame
        pub_cmd.cmd_.DataLen = 8;     // Data length 8 bytes

        motor_data_.push_back(ArchorData{ .is_reset_ = false,
                                          .group_id_ = group_id_,
                                          .driver_id_ = params_.archor_id_[i],
                                          .direction_ = params_.archor_direction_[i],
                                          .target_pos_ = 0.0,
                                          .last_pos_ = 0.0,
                                          .pub_cmd_ = std::move(pub_cmd) });
    }
}

void ArchorDriver::init(RunMode mode)
{
    for (auto& motor_data : motor_data_)
    {
        // 发送复位指令
        motor_data.pub_cmd_.cmd_.ID = 0x000 | (motor_data.driver_id_ << 4);  // 复位指令（）
        for (auto& data : motor_data.pub_cmd_.cmd_.Data)
            data = 0x55;
        sendCmd(motor_data.pub_cmd_);
    }
    QThread::msleep(500);
    for (auto& motor_data : motor_data_)
    {
        // 发送模式选择指令
        motor_data.pub_cmd_.cmd_.ID = 0x001 | (motor_data.driver_id_ << 4);   // 模式选择指令
        motor_data.pub_cmd_.cmd_.Data[0] = static_cast<unsigned char>(mode);  // 选择mode对应模式
        sendCmd(motor_data.pub_cmd_);
    }
    QThread::msleep(500);
    for (auto& motor_data : motor_data_)
    {
        // 发送配置指令
        motor_data.pub_cmd_.cmd_.ID = 0x00A | (motor_data.driver_id_ << 4);  // 配置指令
        motor_data.pub_cmd_.cmd_.Data[0] = 0x01;  // 以 1 毫秒为周期对外发送电流、速度、位置等信息
        motor_data.pub_cmd_.cmd_.Data[1] = 0x0A;  // 以 10 毫秒为周期对外发送CTL1/CTL2的电平状态
        sendCmd(motor_data.pub_cmd_);
    }
    QThread::msleep(500);
}

void ArchorDriver::setSendVel(const unsigned int& i, const int& vel)
{
    setVel(motor_data_[i].pub_cmd_.cmd_, motor_data_[i].driver_id_, vel);
    sendCmd(motor_data_[i].pub_cmd_);
}

void ArchorDriver::setSendVelPos(const unsigned int& i, const int& vel, const int& pos)
{
    setVelPos(motor_data_[i].pub_cmd_.cmd_, motor_data_[i].driver_id_, vel, pos);
    sendCmd(motor_data_[i].pub_cmd_);
}

void ArchorDriver::stopReset()
{
    stop_reset_ = true;
    qInfo() << "Anchors stop reset";
}

void ArchorDriver::reset(const int& vel)
{
    stop_reset_ = false;
    for (auto& motor_data : motor_data_)
    {
        motor_data.is_reset_ = false;
        setVel(motor_data.pub_cmd_.cmd_, motor_data.driver_id_, vel * motor_data.direction_);
        sendCmd(motor_data.pub_cmd_);
    }

    int recv_len = 0;
    std::array<VCI_CAN_OBJ, 3000> recv_msgs{};
    qInfo() << "Anchors reseting...";
    // Reset: move to bottom
    while (!stop_reset_)
    {
        for (uint32_t dev_ind = 0; dev_ind < 2; ++dev_ind)
        {
            if ((recv_len = VCI_Receive(VCI_USBCAN2, dev_ind, 0, recv_msgs.begin(), 3000, 100)) > 0)
            {
                for (int j = 0; j < recv_len; ++j)
                {
                    int id = recv_msgs[j].ID - 0x0c;
                    for (auto& motor_data : motor_data_)
                    {
                        if (id == (motor_data.driver_id_ << 4) && recv_msgs[j].Data[0] == 0x01)
                        {
                            motor_data.is_reset_ = true;
                            setVel(motor_data.pub_cmd_.cmd_, motor_data.driver_id_, 0);
                            sendCmd(motor_data.pub_cmd_);
                            break;
                        }
                    }
                }
            }
            memset(&recv_msgs, 0, sizeof(recv_msgs));
        }

        if (std::all_of(motor_data_.begin(), motor_data_.end(),
                        [](const ArchorData& motor_data) { return motor_data.is_reset_; }))
        {
            qInfo() << "All archores reset successfully";
            break;
        }
        QThread::msleep(30);
    }
    for (size_t i = 0; i < motor_data_.size(); ++i)
        setSendVel(i, 0);
}

/**
 * @brief 单位m转换为qc
 * @param pos
 * @return
 */
int ArchorDriver::convertPos(double pos)
{
    return std::round(pos * 1000 * params_.reduction_ratio_ * params_.encoder_lines_num_ / params_.lead_);
}

/**
 * @brief 运行轨迹
 */
void ArchorDriver::runTraj(const double& period, const QList<QList<double>>& traj)
{
    int cmd_pos = 0.0, cmd_vel = 0.0;  // 速度限制值(RPM)：0~32767

    qint64 cur_time = 0;
    QString message{};
    // path to save .csv file
    QString dir_path = QCoreApplication::applicationDirPath() + "/../data";
    // check if file directory exists
    QDir dir;
    if (!dir.exists(dir_path))
        dir.mkpath(dir_path);
    // .csv file path
    QString file_path =
        dir_path + "/" + QString("archorPos%1.csv").arg(QDateTime::currentDateTime().toString("MM_dd_hh_mm_ss"));
    // create and open .csv file
    QFile file(file_path);
    file.open(QIODevice::WriteOnly | QIODevice::Text);
    QTextStream out(&file);

    // follow the trajectory
    stop_run_ = false;
    for (auto iter = traj.begin(); iter != traj.end(); ++iter)
    {
        if (stop_run_)
        {
            for (size_t i = 0; i < motor_data_.size(); ++i)
                setSendVelPos(i, 0, 0);
            return;
        }
        for (size_t i = 0; i < motor_data_.size(); ++i)
        {
            motor_data_[i].last_pos_ = motor_data_[i].target_pos_;
            motor_data_[i].target_pos_ = -iter->at(i) * motor_data_[i].direction_;
            cmd_pos = convertPos(motor_data_[i].target_pos_);  // m转换为qc
            cmd_vel = std::fabs((motor_data_[i].target_pos_ - motor_data_[i].last_pos_) * 60 * 1000 *
                                params_.reduction_ratio_ / (period * params_.lead_));
            setSendVelPos(i, cmd_vel, cmd_pos);

            cur_time = QDateTime::currentMSecsSinceEpoch();
            message = QString(u8"%1, %2, %3").arg(cur_time).arg(motor_data_[i].driver_id_).arg(iter->at(i));
            out << message << '\n';
        }
        QThread::msleep(static_cast<int>(period * 1000));
    }

    QThread::msleep(2000);  // buffering time for archors moving back to zero position
    file.close();
    qInfo() << "Archors: run traj over";
}

CableDriver::CableDriver(QObject* parent)
{
    Q_ASSERT(4 == dev_ind_.size() && 4 == can_ind_.size() && 4 == params_.cable_id_.size() &&
             4 == params_.cable_direction_.size());
    for (size_t i = 0; i < 4; ++i)
    {
        CanCmd pub_cmd{};
        pub_cmd.dev_ind_ = dev_ind_[i];
        pub_cmd.can_ind_ = can_ind_[i];
        pub_cmd.cmd_.SendType = 0;    // automatically retransmit after a failed send
        pub_cmd.cmd_.RemoteFlag = 0;  // 0 for data frame, 1 for remote frame
        pub_cmd.cmd_.ExternFlag = 0;  // 0 for standard frame, 1 for expanded frame
        pub_cmd.cmd_.DataLen = 8;     // Data length 8 bytes

        motor_data_.push_back(CableData{ .group_id_ = group_id_,
                                         .driver_id_ = params_.cable_id_[i],
                                         .direction_ = params_.cable_direction_[i],
                                         .target_pos_ = 0.0,
                                         .last_pos_ = 0.0,
                                         .pub_cmd_ = std::move(pub_cmd) });
    }
}

void CableDriver::init(RunMode mode)
{
    for (auto& motor_data : motor_data_)
    {
        // 发送复位指令
        motor_data.pub_cmd_.cmd_.ID =
            0x000 | (motor_data.driver_id_ << 4);  // 复位指令（帧ID，由驱动器编号和功能序号决定）
        for (auto& data : motor_data.pub_cmd_.cmd_.Data)
            data = 0x55;
        sendCmd(motor_data.pub_cmd_);
    }
    QThread::msleep(500);
    for (auto& motor_data : motor_data_)
    {
        // 发送模式选择指令
        motor_data.pub_cmd_.cmd_.ID = 0x001 | (motor_data.driver_id_ << 4);   // 模式选择指令
        motor_data.pub_cmd_.cmd_.Data[0] = static_cast<unsigned char>(mode);  // 选择mode对应模式
        sendCmd(motor_data.pub_cmd_);
    }
    QThread::msleep(500);
    for (auto& motor_data : motor_data_)
    {
        // 发送配置指令
        motor_data.pub_cmd_.cmd_.ID = 0x00A | (motor_data.driver_id_ << 4);  // 配置指令
        motor_data.pub_cmd_.cmd_.Data[0] = 0x01;  // 以 1 毫秒为周期对外发送电流、速度、位置等信息
        motor_data.pub_cmd_.cmd_.Data[1] = 0x00;
        sendCmd(motor_data.pub_cmd_);
    }
    QThread::msleep(500);
}

void CableDriver::setSendCur(const unsigned int& i, const int& cur)
{
    setCur(motor_data_[i].pub_cmd_.cmd_, motor_data_[i].driver_id_, cur);
    sendCmd(motor_data_[i].pub_cmd_);
}

void CableDriver::setSendVelPos(const unsigned int& i, const int& vel, const int& pos)
{
    setVelPos(motor_data_[i].pub_cmd_.cmd_, motor_data_[i].driver_id_, vel, pos);
    sendCmd(motor_data_[i].pub_cmd_);
}

/**
 * @brief 单位m转换为qc
 * @param pos
 * @return
 */
int CableDriver::convertPos(double pos)
{
    return std::round(pos * params_.reduction_ratio_ * params_.encoder_lines_num_ / (M_PI * params_.reel_diameter_));
}

/**
 * @brief 运行轨迹
 */
void CableDriver::runTraj(const double& period, const QList<QList<double>>& traj)
{
    int cmd_pos = 0.0, cmd_vel = 0.0;  // 速度限制值(RPM)：0~32767

    qint64 cur_time = 0;
    QString message{};
    // path to save .csv file
    QString dir_path = QCoreApplication::applicationDirPath() + "/../data";
    // check if file directory exists
    QDir dir;
    if (!dir.exists(dir_path))
        dir.mkpath(dir_path);
    // .csv file path
    QString file_path =
        dir_path + "/" + QString("cablePos%1.csv").arg(QDateTime::currentDateTime().toString("MM_dd_hh_mm_ss"));
    // create and open .csv file
    QFile file(file_path);
    file.open(QIODevice::WriteOnly | QIODevice::Text);
    QTextStream out(&file);

    // follow the trajectory
    stop_run_ = false;
    for (auto iter = traj.begin(); iter != traj.end(); ++iter)
    {
        if (stop_run_)
        {
            for (size_t i = 0; i < motor_data_.size(); ++i)
                setSendVelPos(i, 0, 0);
            return;
        }
        for (size_t i = 0; i < motor_data_.size(); ++i)
        {
            motor_data_[i].last_pos_ = motor_data_[i].target_pos_;
            motor_data_[i].target_pos_ = iter->at(i) * motor_data_[i].direction_;
            cmd_pos = convertPos(motor_data_[i].target_pos_);  // m转换为qc
            cmd_vel = std::ceil(std::fabs((motor_data_[i].target_pos_ - motor_data_[i].last_pos_) * 60 *
                                          params_.reduction_ratio_ / (period * M_PI * params_.reel_diameter_)));
            setSendVelPos(i, cmd_vel, cmd_pos);

            cur_time = QDateTime::currentMSecsSinceEpoch();
            message = QString(u8"%1, %2, %3").arg(cur_time).arg(motor_data_[i].driver_id_).arg(iter->at(i));
            out << message << '\n';
        }
        QThread::msleep(static_cast<int>(period * 1000));
    }

    QThread::msleep(2000);  // buffering time for motors moving back to zero position
    file.close();
    qInfo() << "Cables: run traj over";
}
