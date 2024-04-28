#include "global_traj_ctrl.hpp"

#include <QFile>
#include <QDebug>
#include <QByteArray>
#include <QTextCodec>
#include <QMessageBox>

using namespace global_traj_ctrl;

void GlobalTrajCtrl::readTraj(const QString& path1, const QString& path2, QList<QList<double>>& archor_traj,
                              QList<QList<double>>& cable_traj, QList<QList<double>>& a1_traj,
                              QList<QList<double>>& go_traj)
{
    QFile file1(path1), file2(path2);
    if (!file1.open(QIODevice::ReadOnly) || !file2.open(QIODevice::ReadOnly))
    {
        QMessageBox::critical(nullptr, "Traj", "Read global trajectory failed.", QMessageBox::Ok);
        return;
    }
    QTextCodec* codec = QTextCodec::codecForName("UTF-8");  // 使用UTF-8编码
    QByteArray content1 = file1.readAll();
    QString text1 = codec->toUnicode(content1);
    QStringList lines1 = text1.split('\n');
    QByteArray content2 = file2.readAll();
    QString text2 = codec->toUnicode(content2);
    QStringList lines2 = text2.split('\n');

    // cabel&archor
    for (int i = 0; i < (lines1.size() - 1); ++i)
    {
        QList<double> data1, data2;
        if (!lines1.at(i).isEmpty())
        {
            QStringList row = lines1.at(i).split(',');
            Q_ASSERT(8 == row.size());
            for (int i = 0; i < 4; ++i)
                data1.append(row[i].toDouble());
            for (int i = 4; i < 8; ++i)
                data2.append(row[i].toDouble());
        }
        archor_traj.append(data1);
        cable_traj.append(data2);
    }
    // chassis
    for (int i = 0; i < (lines2.size() - 1); ++i)
    {
        QList<double> data1, data2;
        if (!lines2.at(i).isEmpty())
        {
            QStringList row = lines2.at(i).split(',');
            Q_ASSERT(8 == row.size());
            for (int i = 0; i < 4; ++i)
                data1.append(row[i].toDouble());
            for (int i = 4; i < 8; ++i)
                data2.append(row[i].toDouble());
        }
        a1_traj.append(data1);
        go_traj.append(data2);
    }
    file1.close();
    file2.close();
    QMessageBox::information(nullptr, "Traj", "Read local trajectory finished.", QMessageBox::Ok);
}
