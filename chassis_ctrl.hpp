#pragma once

#include <memory>
#include <QThread>

#include "utilities/filters.hpp"
#include "utilities/eigen_types.hpp"
#include "utilities/common.hpp"

namespace chassis_ctrl
{
struct Wheelset
{
    Vec2<double> position_;
    int roll_direction_, steer_direction_;
    double wheel_radius_;
    double last_angle_, steer_state_;
    std::shared_ptr<RampFilter<double>> ramp_angle_, ramp_vel_;
};

struct Vel
{
    double x, y, turn;
};

class ChassisCtrl : public QObject
{
    Q_OBJECT

  public:
    ChassisCtrl(QObject* parent = nullptr);
    void updateVel(const double& x, const double& y, const double& turn);
    void updateSteerState(const std::vector<double>& state);
    void move();
    void stopRun();

  private:
    ChassisParams params_;

    bool stop_run_ = false;
    Vel cmd_vel_{};
    std::vector<double> steer_cmd_{}, roll_cmd_{};
    std::vector<Wheelset> wheelsets_;

  signals:
    void sendA1Vel(std::vector<double> roll);
    void sendGOVel(std::vector<double> steer);
};
}  // namespace chassis_ctrl
