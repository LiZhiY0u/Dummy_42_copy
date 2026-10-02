#pragma once
#include "ITransport.h"
#include <QSerialPort>
class SerialTransport : public ITransport {
    Q_OBJECT
public:
    explicit SerialTransport(QObject *parent=nullptr);
    void open(const QString &endpoint) override;
    void close() override;
    bool send(const QByteArray &bytes) override;
    bool isOpen() const override { return port.isOpen(); }
private:
    void pump();
    QSerialPort port;
    QByteArray outgoing;
};
