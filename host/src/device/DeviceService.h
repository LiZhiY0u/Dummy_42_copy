#pragma once
#include "SessionController.h"
class DeviceService : public QObject {
    Q_OBJECT
public:
    explicit DeviceService(QObject *parent=nullptr):QObject(parent) {}
public slots:
    void connectEndpoint(QString endpoint);
    void shutdown();
    void stop();
signals:
    void stateChanged(ConnectionState state);
    void infoReceived(DeviceInfo info);
    void snapshotReceived(Snapshot snapshot);
    void logMessage(QString message);
    void transactionMessage(QString message);
private:
    ITransport *transport=nullptr;
    SessionController *session=nullptr;
};
