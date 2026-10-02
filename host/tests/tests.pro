QT += core gui widgets serialport testlib
CONFIG += testcase console c++11
CONFIG -= app_bundle
TEMPLATE = app
TARGET = host_tests
SOURCES += tst_environment.cpp tst_protocol.cpp tst_messages.cpp tst_session.cpp tst_firmware.cpp
SOURCES += tst_dispatcher.cpp ../../firmware/core/command_dispatcher.cpp
SOURCES += tst_control_service.cpp ../../firmware/core/control_service.cpp
HEADERS += ../../firmware/core/control_service.h
HEADERS += ../../firmware/core/command_dispatcher.h
INCLUDEPATH += $$PWD/../../firmware/core
SOURCES += $$PWD/../../firmware/core/protocol_v1.cpp $$PWD/../../firmware/core/control_gate.cpp
HEADERS += $$PWD/../../firmware/core/protocol_v1.h $$PWD/../../firmware/core/control_gate.h
include(../common.pri)
INCLUDEPATH += $$PWD/../src
SOURCES += ../src/ui/MainWindow.cpp
HEADERS += ../src/ui/MainWindow.h
QMAKE_CXXFLAGS += -Wall -Wextra
win32-g++:QMAKE_CXXFLAGS += -finput-charset=UTF-8 -fexec-charset=UTF-8
