#pragma once

#include <QWidget>
#include <QButtonGroup>
#include <QKeyEvent>

#include "motor_driver.hpp"
#include "usbcan.hpp"
#include "local_traj_ctrl.hpp"
#include "unitree_motor_a1/a1.hpp"
#include "unitree_motor_go/go.hpp"
#include "global_traj_ctrl.hpp"
#include "chassis_ctrl.hpp"

QT_BEGIN_NAMESPACE
namespace Ui
{
class Widget;
constexpr auto REMOTE_CONTROL_MSG = R"delimiter(
        Control Your CDPR!
        Reading from the keyboard and Sending to Chassis!
        ---------------------------
        Holonomic mode (strafing):
           q    w    e
           a    s    d
           z    x    c

        For Steering mode, hold down the shift key:
        ---------------------------
           Q    E
           A    D
           Z    C

        anything else : stop

        u/j : increase/decrease only linear speed in x axis by 10%
        i/k : increase/decrease only linear speed in y axis by 10%
        o/l : increase/decrease only angular speed by 10%
    )delimiter";
}  // namespace Ui
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
    void startA1RunTraj(double period, QList<QList<double>> traj);
    void startGORunTraj(double period, QList<QList<double>> traj);
    void startCanRecvPos();

    void startRemoteControl();
    void sendChassisVel(double x, double y, double turn);

  private slots:
    void on_lfCableJogButton_toggled(bool checked);

    void on_rfCableJogButton_toggled(bool checked);

    void on_lbCableJogButton_toggled(bool checked);

    void on_rbCableJogButton_toggled(bool checked);

    void on_lfArchorUp_toggled(bool checked);

    void on_rfArchorUp_toggled(bool checked);

    void on_lbArchorUp_toggled(bool checked);

    void on_rbArchorUp_toggled(bool checked);

    void on_lfArchorDown_toggled(bool checked);

    void on_rfArchorDown_toggled(bool checked);

    void on_lbArchorDown_toggled(bool checked);

    void on_rbArchorDown_toggled(bool checked);

    void on_startLocalTraj_clicked();

    void on_readLocalTraj_clicked();

    void selectCableMode(bool checked);

    void selectArchorMode(bool checked);

    void on_archorStopResetButton_clicked();

    void on_stopLocalTraj_clicked();

    void on_lfArchorLocateButton_toggled(bool checked);

    void on_rfArchorLocateButton_toggled(bool checked);

    void on_lbArchorLocateButton_toggled(bool checked);

    void on_rbArchorLocateButton_toggled(bool checked);

    void on_lfCableLocateButton_toggled(bool checked);

    void on_rfCableLocateButton_toggled(bool checked);

    void on_lbCableLocateButton_toggled(bool checked);

    void on_rbCableLocateButton_toggled(bool checked);

    void on_cableZeroPoint_clicked();

    void on_archorBottom_clicked();

    void on_archorZeroPoint_clicked();

    void selectChassisMode(bool checked);

    void on_readGlobalTraj_clicked();

    void on_startGlobalTraj_clicked();

    void on_stopGlobalTraj_clicked();

    void keyPressEvent(QKeyEvent* event);  //键盘按下事件

    void keyReleaseEvent(QKeyEvent* event);  //键盘松开事件

  private:
    Ui::Widget* ui;
    QButtonGroup* cableModeGroup;
    QButtonGroup* archorModeGroup;
    QButtonGroup* chassisModeGroup;

    QList<QList<double>> archor_traj_{}, cable_traj_{}, a1_traj_{}, go_traj_{};

    motor_driver::ArchorDriver* archors_;
    motor_driver::CableDriver* cables_;
    usb_can::UsbCan* usbcan_;
    local_traj_ctrl::LocalTrajCtrl local_traj_;
    motor_a1::A1Driver* a1_;
    motor_go::GoDriver* go_;
    global_traj_ctrl::GlobalTrajCtrl global_traj_;
    chassis_ctrl::ChassisCtrl* chassis_;

    QThread *archor_thread_, *cable_thread_, *can_thread_, *a1_thread_, *go_thread_, *chassis_thread_;
};
