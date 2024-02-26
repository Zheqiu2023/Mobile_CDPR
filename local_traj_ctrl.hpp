#pragma once

#include <QThread>
#include <QString>

namespace local_traj_ctrl
{
class LocalTrajCtrl
{
  public:
    LocalTrajCtrl() = default;
    void readTraj(const QString& path, QList<QList<double>>& archor_traj, QList<QList<double>>& cable_traj);

  private:
    double archor_coor_z_{}, cable_length_{};
};
}  // namespace local_traj_ctrl
