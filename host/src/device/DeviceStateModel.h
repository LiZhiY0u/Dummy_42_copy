#pragma once
#include "SessionController.h"
class DeviceStateModel : public QObject {
    Q_OBJECT
public:
    explicit DeviceStateModel(QObject *parent=nullptr);
    ConnectionState state=ConnectionState::Disconnected;
    Snapshot snapshot;
    DeviceInfo info;
    bool fresh=false;
public slots:
    void updateState(ConnectionState value);
    void updateInfo(DeviceInfo value);
    void updateSnapshot(Snapshot value);
signals:
    void changed();
private:
    QTimer timer;
    QElapsedTimer age;
};
