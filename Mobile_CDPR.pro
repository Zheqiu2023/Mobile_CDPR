QT       += core gui

greaterThan(QT_MAJOR_VERSION,5): QT += core5compat
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    local_traj_ctrl.cpp \
    main.cpp \
    motor_driver.cpp \
    usbcan.cpp \
    widget.cpp

HEADERS += \
    common.hpp \
    controlcan.hpp \
    local_traj_ctrl.hpp \
    motor_driver.hpp \
    usbcan.hpp \
    widget.hpp

FORMS += \
    widget.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/./release/ -lcontrolcan
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/./debug/ -lcontrolcan
else:unix: LIBS += -L$$PWD/./ -lcontrolcan

INCLUDEPATH += $$PWD/''
DEPENDPATH += $$PWD/''
