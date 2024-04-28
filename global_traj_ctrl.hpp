#pragma once

#include <QThread>
#include <QString>

namespace global_traj_ctrl
{
class GlobalTrajCtrl
{
  public:
    void readTraj(const QString& path1, const QString& path2, QList<QList<double>>& archor_traj,
                  QList<QList<double>>& cable_traj, QList<QList<double>>& a1_traj, QList<QList<double>>& go_traj);
};
}  // namespace global_traj_ctrl
