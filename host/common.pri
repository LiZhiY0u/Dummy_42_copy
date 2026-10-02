INCLUDEPATH += $$PWD/src
SOURCES += $$PWD/src/protocol/ProtocolCodec.cpp $$PWD/src/protocol/ProtocolMessages.cpp \
    $$PWD/src/transport/SerialTransport.cpp $$PWD/src/transport/MockTransport.cpp \
    $$PWD/src/device/SessionController.cpp $$PWD/src/device/DeviceService.cpp $$PWD/src/device/DeviceStateModel.cpp
HEADERS += $$PWD/src/protocol/ProtocolTypes.h $$PWD/src/protocol/ProtocolCodec.h $$PWD/src/protocol/ProtocolMessages.h \
    $$PWD/src/transport/ITransport.h $$PWD/src/transport/SerialTransport.h $$PWD/src/transport/MockTransport.h \
    $$PWD/src/device/SessionController.h $$PWD/src/device/DeviceService.h $$PWD/src/device/DeviceStateModel.h
