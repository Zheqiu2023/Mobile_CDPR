#pragma once

#include <QThread>
#include <QString>

namespace local_traj_ctrl
{
class LocalTrajCtrl
{
  public:
    void readTraj(const QString& path, QList<QList<double>>& archor_traj, QList<QList<double>>& cable_traj);
};
}  // namespace local_traj_ctrl
