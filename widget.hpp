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
    void startArchorRunTraj(int vel, double period);
    void startCableRunTraj(double period);
    void startCanRecv();

  private slots:
    void on_lfPullCableButton_toggled(bool checked);

    void on_rfPullCableButton_toggled(bool checked);

    void on_lbPullCableButton_toggled(bool checked);

    void on_rbPullCableButton_toggled(bool checked);

    void on_lfReleaseCableButton_toggled(bool checked);

    void on_rfReleaseCableButton_toggled(bool checked);

    void on_lbReleaseCableButton_toggled(bool checked);

    void on_rbReleaseCableButton_toggled(bool checked);

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

  private:
    Ui::Widget* ui;

    QList<QList<double>> archor_traj_, cable_traj_;

    motor_driver::ArchorDriver* archors_;
    motor_driver::CableDriver* cables_;
    usb_can::UsbCan* usbcan_;
    local_traj_ctrl::LocalTrajCtrl local_traj_;

    QThread *archor_thread_, *cable_thread_, *can_thread_;
};
