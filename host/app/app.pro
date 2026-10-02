QT += core gui widgets serialport
CONFIG += c++11
TEMPLATE = app
TARGET = StepperHost
include(../common.pri)
INCLUDEPATH += $$PWD/../src
SOURCES += main.cpp ../src/ui/MainWindow.cpp
HEADERS += ../src/ui/MainWindow.h
QMAKE_CXXFLAGS += -Wall -Wextra
win32-g++:QMAKE_CXXFLAGS += -finput-charset=UTF-8 -fexec-charset=UTF-8
