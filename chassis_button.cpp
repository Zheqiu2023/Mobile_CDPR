#include "widget.hpp"
#include "ui_widget.h"

#include <QDebug>
#include <QSpinBox>
#include <QRadioButton>
#include <QPushButton>
#include <QMessageBox>

// 键盘按下事件
void Widget::keyPressEvent(QKeyEvent* event)
{
    if (!ui->remoteControl->isChecked())
        return;

    double x = 0.0, y = 0.0, th = 0.0, x_vel = ui->x_vel->value(), y_vel = ui->y_vel->value(),
           turn_vel = ui->turn_vel->value(), x_vel_limit = ui->x_vel->maximum(), y_vel_limit = ui->y_vel->maximum(),
           turn_vel_limit = ui->turn_vel->maximum();
    static int status = 0;
    std::map<int, std::vector<int>> moveBindings = { { Qt::Key_W, { 1, 0, 0 } },  { Qt::Key_A, { 0, 1, 0 } },
                                                     { Qt::Key_S, { 0, 0, 0 } },  { Qt::Key_D, { 0, -1, 0 } },
                                                     { Qt::Key_X, { -1, 0, 0 } }, { Qt::Key_Q, { 1, 1, 0 } },
                                                     { Qt::Key_E, { 1, -1, 0 } }, { Qt::Key_Z, { -1, 1, 0 } },
                                                     { Qt::Key_C, { -1, -1, 0 } }

    },
                                    turnBindings = { { Qt::Key_A, { 0, 0, 1 } },  { Qt::Key_D, { 0, 0, -1 } },
                                                     { Qt::Key_Q, { 1, 0, 1 } },  { Qt::Key_E, { 1, 0, -1 } },
                                                     { Qt::Key_Z, { -1, 0, 1 } }, { Qt::Key_C, { -1, 0, -1 } } };
    std::map<int, std::vector<double>> speedBindings = { { Qt::Key_U, { 1.1, 1, 1 } }, { Qt::Key_J, { 0.9, 1, 1 } },
                                                         { Qt::Key_I, { 1, 1.1, 1 } }, { Qt::Key_K, { 1, 0.9, 1 } },
                                                         { Qt::Key_O, { 1, 1, 1.1 } }, { Qt::Key_L, { 1, 1, 0.9 } } };

    auto print_vels = [&]() {
        qInfo("currently:  speed in x axis %.3f, in y axis %.3f, turn speed %.3f ", x_vel, y_vel, turn_vel);
    };

    if (event->modifiers() == Qt::ShiftModifier)
    {  // 如果按下了SHIFT键
        switch (event->key())
        {
            // 转向键
            case Qt::Key_A:
            case Qt::Key_D:
            case Qt::Key_Q:
            case Qt::Key_E:
            case Qt::Key_Z:
            case Qt::Key_C:
                x = turnBindings[event->key()][0];
                y = turnBindings[event->key()][1];
                th = turnBindings[event->key()][2];
                break;
            default:
                x = 0;
                y = 0;
                th = 0;
                break;
        }
    }
    else
    {
        switch (event->key())
        {
            // 移动键
            case Qt::Key_W:
            case Qt::Key_A:
            case Qt::Key_S:
            case Qt::Key_D:
            case Qt::Key_X:
            case Qt::Key_Q:
            case Qt::Key_E:
            case Qt::Key_Z:
            case Qt::Key_C:
                x = moveBindings[event->key()][0];
                y = moveBindings[event->key()][1];
                th = moveBindings[event->key()][2];
                break;
            // 速度键
            case Qt::Key_U:
            case Qt::Key_J:
            case Qt::Key_I:
            case Qt::Key_K:
            case Qt::Key_O:
            case Qt::Key_L:
                x_vel = fmin(x_vel_limit, x_vel * speedBindings[event->key()][0]);
                y_vel = fmin(y_vel_limit, y_vel * speedBindings[event->key()][1]);
                turn_vel = fmin(turn_vel_limit, turn_vel * speedBindings[event->key()][2]);
                ui->x_vel->setValue(x_vel);
                ui->y_vel->setValue(y_vel);
                ui->turn_vel->setValue(turn_vel);
                if (x_vel == x_vel_limit)
                    qInfo("Linear speed limit in x axis reached!");
                if (x_vel == x_vel_limit)
                    qInfo("Linear speed limit in y axis reached!");
                if (turn_vel == turn_vel_limit)
                    qInfo("Angular speed limit reached!");
                print_vels();
                if (status == 14)
                    qInfo() << Ui::REMOTE_CONTROL_MSG;
                status = (status + 1) % 15;
                break;
            default:
                x = 0;
                y = 0;
                th = 0;
                break;
        }
    }

    emit sendChassisVel(x * x_vel, y * y_vel, th * turn_vel);
}

// 键盘释放事件
void Widget::keyReleaseEvent(QKeyEvent* event)
{
    if (!ui->remoteControl->isChecked())
        return;
    emit sendChassisVel(0, 0, 0);
}
