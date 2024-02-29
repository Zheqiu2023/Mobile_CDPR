#pragma once

#include <QWidget>
#include <QButtonGroup>

#include "motor_driver.hpp"
#include "usbcan.hpp"
#include "local_traj_ctrl.hpp"

QT_BEGIN_NAMESPACE
namespace Ui
{
class Widget;
}
QT_END_NAMESPACE

class Widget : public QWidget
{
    Q_OBJECT

  public:
    Widget(QWidget* parent = nullptr);
    ~Widget();

  signals:
    void startArchorReset(int vel);
    void startArchorRunTraj(double period, QList<QList<double>> traj);
    void startCableRunTraj(double period, QList<QList<double>> traj);
    void startCanRecvPos();

  private slots:
    void on_lfCableJogButton_toggled(bool checked);

    void on_rfCableJogButton_toggled(bool checked);

    void on_lbCableJogButton_toggled(bool checked);

    void on_rbCableJogButton_toggled(bool checked);

    void on_cableResetButton_clicked();

    void on_lfArchorUp_toggled(bool checked);

    void on_rfArchorUp_toggled(bool checked);

    void on_lbArchorUp_toggled(bool checked);

    void on_rbArchorUp_toggled(bool checked);

    void on_lfArchorDown_toggled(bool checked);

    void on_rfArchorDown_toggled(bool checked);

    void on_lbArchorDown_toggled(bool checked);

    void on_rbArchorDown_toggled(bool checked);

    void on_archorResetButton_clicked();

    void on_stopButton_clicked();

    void on_startLocalTraj_clicked();

    void on_readLocalTraj_clicked();

    void selectCableMode(bool checked);

    void selectArchorMode(bool checked);

  private:
    Ui::Widget* ui;
    QButtonGroup* cableModeGroup;
    QButtonGroup* archorModeGroup;

    QList<QList<double>> archor_traj_{}, cable_traj_{};

    motor_driver::ArchorDriver* archors_;
    motor_driver::CableDriver* cables_;
    usb_can::UsbCan* usbcan_;
    local_traj_ctrl::LocalTrajCtrl local_traj_;

    QThread *archor_thread_, *cable_thread_, *can_thread_;
};
