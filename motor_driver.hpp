#pragma once

#include <QReadWriteLock>
#include <QMutex>
#include <QMutexLocker>
#include <QThread>
#include <termio.h>
#include <vector>

#include "common.hpp"

namespace motor_driver
{
class BaseDriver
{
  public:
    void sendCmd(CanCmd& cmd_struct);
    void stop();
    void sleep_ms(unsigned int msec);

  protected:
    void setVel(VCI_CAN_OBJ& cmd, const int& driver_id, const int& target_vel);
    void setVelPos(VCI_CAN_OBJ& cmd, const int& driver_id, const int& target_vel, const int& target_pos);

    double traj_period_ = 0.0;
    bool is_stop_ = false;
};

class ArchorDriver : public QObject, public BaseDriver
{
    Q_OBJECT

  public:
    ArchorDriver(QObject* parent = nullptr);

    void run_traj(const double& period, const QList<QList<double>>& traj);
    void reset(const int& vel);
    void init(RunMode mode);
    void setSendVel(const unsigned int& i, const int& vel);
    void setSendVelPos(const unsigned int& i, const int& vel, const int& pos);

  private:
    TrajParams params_;
    const std::vector<int> dev_ind_ = { 0, 0, 1, 1 }, can_ind_ = { 0, 0, 0, 0 };
    std::vector<ArchorData> motor_data_{};

    QReadWriteLock rwlock_;
    QMutex mutex_;
};

class CableDriver : public QObject, public BaseDriver
{
    Q_OBJECT

  public:
    CableDriver(QObject* parent = nullptr);

    void run_traj(const double& period, const QList<QList<double>>& traj);
    void init(RunMode mode);
    void setSendVelPos(const unsigned int& i, const int& vel, const int& pos);
    double convertPos(double pos);

  private:
    TrajParams params_;
    const std::vector<int> dev_ind_ = { 0, 0, 1, 1 }, can_ind_ = { 1, 1, 1, 1 };
    std::vector<CableData> motor_data_{};
};
}  // namespace motor_driver
