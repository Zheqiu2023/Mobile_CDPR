#include "local_traj_ctrl.hpp"

#include <QFile>
#include <QDebug>
#include <QByteArray>
#include <QTextCodec>
#include <QMessageBox>

using namespace local_traj_ctrl;

void LocalTrajCtrl::readTraj(const QString& path, QList<QList<double>>& archor_traj, QList<QList<double>>& cable_traj)
{
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly))
        QMessageBox::critical(nullptr, "Traj", "Read local trajectory failed.", QMessageBox::Ok);

    QTextCodec* codec = QTextCodec::codecForName("UTF-8");  // 使用UTF-8编码
    QByteArray content = file.readAll();
    QString text = codec->toUnicode(content);
    QStringList lines = text.split('\n');
    qDebug() << lines.size();

    for (int i = 0; i < lines.size(); ++i)
    {
        QList<double> data1, data2;
        if (!lines.at(i).isEmpty())
        {
            QStringList row = lines.at(i).split(',');
            Q_ASSERT(8 == row.size());
            for (int i = 0; i < 4; ++i)
            {
                data1.append(row[i].toDouble());
            }
            for (int i = 4; i < 8; ++i)
            {
                data2.append(row[i].toDouble());
            }
        }
        archor_traj.append(data1);
        cable_traj.append(data2);
    }

    file.close();
    QMessageBox::information(nullptr, "Traj", "Read local trajectory finished.", QMessageBox::Ok);
}
