#pragma once
#include <QObject>
#include <QByteArray>
class ITransport : public QObject {
    Q_OBJECT
public:
    explicit ITransport(QObject *parent=nullptr):QObject(parent) {}
    virtual void open(const QString &endpoint)=0;
    virtual void close()=0;
    virtual bool send(const QByteArray &bytes)=0;
    virtual bool isOpen() const=0;
signals:
    void opened();
    void closed();
    void bytesReceived(QByteArray bytes);
    void transportError(QString message);
    void frameQueued(QByteArray bytes);
};
