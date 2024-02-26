#pragma once

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
    bool is_traj_end_ = false, is_stop_ = false;
};

class ArchorDriver : public QObject, public BaseDriver
{
    Q_OBJECT

  public:
    ArchorDriver(QObject* parent = nullptr);

    void run_traj(const int& vel, const double& period);
    void reset(const int& vel);
    void init(RunMode mode, const int& period);

    void setSendVel(const unsigned int& i, const int& vel);
    void setSendVelPos(const unsigned int& i, const int& vel, const int& pos);

  private:
    TrajParams params_;
    const std::vector<int> dev_ind_ = { 0, 0, 1, 1 }, can_ind_ = { 0, 0, 0, 0 };
    std::vector<ArchorData> motor_data_{};

  public slots:
    void recvResetMsg(const int& id);

  signals:
    void resetFinished();
};

class CableDriver : public QObject, public BaseDriver
{
    Q_OBJECT

  public:
    CableDriver(QObject* parent = nullptr);

    void run_traj(const double& period);
    void init(RunMode mode);

    void setSendVel(const unsigned int& i, const int& vel);
    void setSendVelPos(const unsigned int& i, const int& vel, const int& pos);

  private:
    TrajParams params_;
    const std::vector<int> dev_ind_ = { 0, 0, 1, 1 }, can_ind_ = { 1, 1, 1, 1 };
    std::vector<CableData> motor_data_{};

  signals:
    void resetFinished();
};
}  // namespace motor_driver
