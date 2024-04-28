#pragma once

#include <QThread>

#include "utilities/common.hpp"

namespace motor_driver
{
class BaseDriver
{
  public:
    void sendCmd(CanCmd& cmd_struct);
    void stopRun();
    void sleepMS(unsigned int msec);

  protected:
    void setCur(VCI_CAN_OBJ& cmd, const int& driver_id, const int& target_cur);
    void setVel(VCI_CAN_OBJ& cmd, const int& driver_id, const int& target_vel);
    void setVelPos(VCI_CAN_OBJ& cmd, const int& driver_id, const int& target_vel, const int& target_pos);

    bool stop_run_ = false;
};

class ArchorDriver : public QObject, public BaseDriver
{
    Q_OBJECT

  public:
    ArchorDriver(QObject* parent = nullptr);

    void runTraj(const double& period, const QList<QList<double>>& traj);
    void reset(const int& vel);
    void init(RunMode mode);
    void setSendVel(const unsigned int& i, const int& vel);
    void setSendVelPos(const unsigned int& i, const int& vel, const int& pos);
    void stopReset();
    int convertPos(double pos);

  private:
    const int group_id_ = 0;
    const std::vector<int> dev_ind_ = { 0, 0, 1, 1 }, can_ind_ = { 0, 0, 0, 0 };
    TrajParams params_;

    bool stop_reset_ = false;
    std::vector<ArchorData> motor_data_{};
};

class CableDriver : public QObject, public BaseDriver
{
    Q_OBJECT

  public:
    CableDriver(QObject* parent = nullptr);

    void runTraj(const double& period, const QList<QList<double>>& traj);
    void init(RunMode mode);
    void setSendCur(const unsigned int& i, const int& cur);
    void setSendVelPos(const unsigned int& i, const int& vel, const int& pos);
    int convertPos(double pos);

  private:
    const int group_id_ = 0;
    const std::vector<int> dev_ind_ = { 0, 0, 1, 1 }, can_ind_ = { 1, 1, 1, 1 };
    TrajParams params_;

    std::vector<CableData> motor_data_{};
};
}  // namespace motor_driver
