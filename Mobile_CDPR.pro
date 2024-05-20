QT       += core gui

greaterThan(QT_MAJOR_VERSION,5): QT += core5compat
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    a1.cpp \
    cable_archor_button.cpp \
    chassis_button.cpp \
    chassis_ctrl.cpp \
    filters.cpp \
    global_traj_ctrl.cpp \
    go.cpp \
    local_traj_ctrl.cpp \
    main.cpp \
    motor_driver.cpp \
    usbcan.cpp \
    widget.cpp

HEADERS += \
    chassis_ctrl.hpp \
    controlcan.hpp \
    utilities/angles.hpp \
    utilities/common.hpp \
    utilities/eigen_types.hpp \
    utilities/filters.hpp \
    utilities/interpolation.hpp \
    utilities/math_utilities.hpp \
    global_traj_ctrl.hpp \
    local_traj_ctrl.hpp \
    motor_driver.hpp \
    unitree_motor_a1/IOPort/IOPort.h \
    unitree_motor_a1/a1.hpp \
    unitree_motor_a1/crc/crc32.h \
    unitree_motor_a1/serialPort/SerialPort.h \
    unitree_motor_a1/serialPort/include/errorClass.h \
    unitree_motor_a1/unitreeMotor/include/motor_msg.h \
    unitree_motor_a1/unitreeMotor/unitreeMotor.h \
    unitree_motor_go/IOPort/IOPort.h \
    unitree_motor_go/crc/crc_ccitt.h \
    unitree_motor_go/go.hpp \
    unitree_motor_go/serialPort/SerialPort.h \
    unitree_motor_go/serialPort/include/errorClass.h \
    unitree_motor_go/unitreeMotor/include/motor_msg.h \
    unitree_motor_go/unitreeMotor/unitreeMotor.h \
    usbcan.hpp \
    widget.hpp

FORMS += \
    widget.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target


unix:!macx: LIBS += -L$$PWD/./ -lcontrolcan

INCLUDEPATH += $$PWD/.
DEPENDPATH += $$PWD/.

unix:!macx: LIBS += -L$$PWD/./ -lunitreeMotorSDK_Linux64

INCLUDEPATH += $$PWD/.
DEPENDPATH += $$PWD/.

unix:!macx: LIBS += -L$$PWD/./ -lUnitreeMotorSDK_M80106_Linux64

INCLUDEPATH += $$PWD/''
DEPENDPATH += $$PWD/''
