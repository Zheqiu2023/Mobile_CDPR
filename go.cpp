#include "unitree_motor_go/go.hpp"
#include "qdebug.h"
#include "qlogging.h"

#include <vector>
#include <array>
#include <QDebug>
#include <QtGlobal>
#include <QTime>
#include <QDir>
#include <QElapsedTimer>
#include <QCoreApplication>

using namespace motor_go;

GoDriver::GoDriver(QObject* parent)
{
    // open serial port
    Q_ASSERT(4 == port_name_.size());
    static std::array<SerialPort, 4> serial_port{ SerialPort(port_name_[0]), SerialPort(port_name_[1]),
                                                  SerialPort(port_name_[2]), SerialPort(port_name_[3]) };
    motor_param_.resize(serial_port.size());
    pos_state_.resize(serial_port.size());
    for (size_t i = 0; i < motor_param_.size(); ++i)
    {
        motor_param_[i].serial_num_ = i;
        motor_param_[i].init_param_.id = static_cast<unsigned short>(id_[i]);
        motor_param_[i].motor_cmd_.id = static_cast<unsigned short>(id_[i]);
        motor_param_[i].port_ = &serial_port[i];
    }
    init();
}

void GoDriver::setCmd(const std::vector<double>& cmd, const RunMode& mode)
{
    for (auto& motor_param : motor_param_)
    {
        // actual torque command: PD controller
        // K_P*delta_Pos + K_W*delta_W + T
        motor_param.motor_cmd_.mode = static_cast<unsigned short>(mode);
        motor_param.motor_cmd_.K_P = cmd[0];
        motor_param.motor_cmd_.K_W = cmd[1];
    }
}

void GoDriver::init()
{
    for (auto& motor_param : motor_param_)
    {
        // initial parameters
        motor_param.init_param_.mode = 0;
        motor_param.port_->sendRecv(&motor_param.init_param_, &motor_param.motor_recv_);
        // record the position at each motor power-up and set it as zero point
        motor_param.zero_position_ = motor_param.motor_recv_.Pos;
        motor_param.motor_cmd_.Pos = motor_param.motor_recv_.Pos;

        qInfo("Zero position of motor Go[%d]: %f", motor_param.serial_num_, motor_param.zero_position_);
    }
}

void GoDriver::runTraj(const double& period, const QList<QList<double>>& traj)
{
    qint64 cur_time = 0;
    QString message{};
    // path to save .csv file
    QString dir_path = QCoreApplication::applicationDirPath() + "/../data";
    // check if file directory exists
    QDir dir;
    if (!dir.exists(dir_path))
        dir.mkpath(dir_path);
    // .csv file path
    QString file_path1 =
        dir_path + "/" + QString("goTargetPos%1.csv").arg(QDateTime::currentDateTime().toString("MM_dd_hh_mm_ss"));
    QString file_path2 =
        dir_path + "/" + QString("goRecvPos%1.csv").arg(QDateTime::currentDateTime().toString("MM_dd_hh_mm_ss"));
    // create and open .csv file
    QFile file1(file_path1);
    QFile file2(file_path2);
    file1.open(QIODevice::WriteOnly | QIODevice::Text);
    file2.open(QIODevice::WriteOnly | QIODevice::Text);
    QTextStream out1(&file1);
    QTextStream out2(&file2);

    // follow the trajectory
    stop_run_ = false;
    for (auto iter = traj.begin(); iter != traj.end(); ++iter)
    {
        if (stop_run_)
        {
            stall();
            return;
        }
        for (size_t i = 0; i < motor_param_.size(); ++i)
        {
            motor_param_[i].motor_cmd_.Pos = motor_param_[i].zero_position_ + iter->at(i) * reduction_ratio_;
            motor_param_[i].motor_cmd_.T = 0.2;  // feed-forward torque(0.2 is an estimated value)
            motor_param_[i].port_->sendRecv(&motor_param_[i].motor_cmd_, &motor_param_[i].motor_recv_);
            pos_state_[i] = (motor_param_[i].motor_recv_.Pos - motor_param_[i].zero_position_) / reduction_ratio_;

            cur_time = QDateTime::currentMSecsSinceEpoch();
            message = QString(u8"%1, %2, %3").arg(cur_time).arg(motor_param_[i].serial_num_).arg(iter->at(i));
            out1 << message << '\n';
            message = QString(u8"%1, %2, %3")
                          .arg(cur_time)
                          .arg(motor_param_[i].serial_num_)
                          .arg(motor_param_[i].motor_recv_.Pos);
            out2 << message << '\n';
        }
        QThread::msleep(static_cast<int>(period * 1000));
    }

    QThread::msleep(2000);
    file1.close();
    file2.close();
    qInfo() << "GO: run traj over";
}

void GoDriver::recvAngle(const std::vector<double>& cmd_angle)
{
    for (size_t i = 0; i < cmd_angle.size(); ++i)
    {
        motor_param_[i].motor_cmd_.Pos = motor_param_[i].zero_position_ + cmd_angle[i] * reduction_ratio_;
        motor_param_[i].motor_cmd_.T = 0.2;  // feed-forward torque(0.2 is an estimated value)
        motor_param_[i].port_->sendRecv(&motor_param_[i].motor_cmd_, &motor_param_[i].motor_recv_);
        pos_state_.push_back((motor_param_[i].motor_recv_.Pos - motor_param_[i].zero_position_) / reduction_ratio_);
    }
    emit sendSteerState(pos_state_);
    pos_state_.clear();
}

void GoDriver::stall()
{
    for (auto& motor_param : motor_param_)
    {
        while (!(motor_param.port_->sendRecv(&motor_param.init_param_, &motor_param.motor_recv_)))
            QThread::msleep(100);
        qDebug() << "position: " << motor_param.motor_recv_.Pos << motor_param.motor_recv_.T;
    }

    qInfo() << "Motor GO stop running!";
}

void GoDriver::stopRun()
{
    stop_run_ = true;
}
