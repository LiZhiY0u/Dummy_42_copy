#pragma once
#include "ITransport.h"
#include "protocol/ProtocolCodec.h"
#include <QTimer>
#include <QElapsedTimer>
#include <QHash>
#include <QMap>
class MockTransport : public ITransport {
    Q_OBJECT
public:
    explicit MockTransport(QObject *parent=nullptr);
    void open(const QString &endpoint) override;
    void close() override;
    bool send(const QByteArray &bytes) override;
    bool isOpen() const override {return connected;}
    void setResponseDelay(Command command,int ms) {delays[quint16(command)]=qMax(0,ms);}
    void setDropResponses(Command command,bool drop) {drops[quint16(command)]=drop;}
    void setChunkSize(int size) {chunkSize=qBound(1,size,144);}
    void reboot();
private:
    void handle(const Frame &request);
    void deliver(const Frame &frame,int delay=0);
    void pump();
    QByteArray sampleSnapshot();
    Snapshot snapshot;
    DeviceInfo info;
    ProtocolCodec codec;
    QElapsedTimer clock;
    QTimer telemetry;
    QTimer watchdog;
    QMap<quint16,QByteArray> parameters;
    QHash<quint16,int> delays;
    QHash<quint16,bool> drops;
    struct Cached {Frame request,response; qint64 time=0;};
    QList<Cached> cache;
    QByteArray incoming;
    quint32 session=0,generation=0;
    quint16 highWater=0;
    qint64 heartbeat=0;
    int chunkSize=144;
    bool connected=false,pumping=false;
};
