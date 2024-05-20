#pragma once

#include <QThread>

#include "unitree_motor_a1/serialPort/SerialPort.h"

namespace motor_a1
{
enum class RunMode
{
    FREE = 0,         // 空闲
    OPEN_LOOP = 5,    // 开环
    CLOSED_LOOP = 10  // FOC闭环
};

struct MotorParam
{
    int serial_num_;
    double zero_position_, last_pos_;
    MotorCmd init_cmd_, motor_cmd_;
    MotorData motor_recv_;

    SerialPort* port_;
};

class A1Driver : public QObject
{
    Q_OBJECT

  public:
    A1Driver(QObject* parent = nullptr);
    void setCmd(const std::vector<double>& cmd, const RunMode& mode = RunMode::CLOSED_LOOP);
    void stopRun();
    void runTraj(const double& period, const QList<QList<double>>& traj);
    void recvVel(const std::vector<double>& cmd_vel);

  private:
    void init();
    void stall();

    const double reduction_ratio_ = 9.1, wheel_radius_ = 0.1175;
    const std::vector<int> id_{ 0, 0, 0, 0 };
    const std::vector<std::string> port_name_{ "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB3" };

    bool stop_run_ = false;
    std::vector<MotorParam> motor_param_{};
};
}  // namespace motor_a1
