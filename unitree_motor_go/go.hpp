#pragma once

#include <QThread>

#include "unitree_motor_go/serialPort/SerialPort.h"

namespace motor_go
{
enum class RunMode
{
    BRAKE = 0,        // 空闲
    CLOSED_LOOP = 1,  // FOC闭环
    CALIBRATION = 2   // 电机标定
};

struct MotorParam
{
    int serial_num_;
    double zero_position_;
    MotorCmd init_cmd_, motor_cmd_;
    MotorData motor_recv_;

    SerialPort* port_;
};

class GoDriver : public QObject
{
    Q_OBJECT

  public:
    GoDriver(QObject* parent = nullptr);
    void setCmd(const std::vector<double>& cmd, const RunMode& mode = RunMode::CLOSED_LOOP);
    void stopRun();
    void runTraj(const double& period, const QList<QList<double>>& traj);
    void recvAngle(const std::vector<double>& cmd_angle);

  private:
    void init();
    void stall();

    const double reduction_ratio_ = 6.33;
    const std::vector<unsigned short> id_{ 0, 0, 0, 0 };
    const std::vector<std::string> port_name_{ "/dev/ttyUSB4", "/dev/ttyUSB5", "/dev/ttyUSB6", "/dev/ttyUSB7" };

    bool stop_run_ = false;
    std::vector<double> pos_state_{};
    std::vector<MotorParam> motor_param_{};

  signals:
    void sendSteerState(std::vector<double> state);
};
}  // namespace motor_go
