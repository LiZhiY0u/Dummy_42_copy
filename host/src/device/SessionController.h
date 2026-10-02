#pragma once
#include <QObject>
#include <QTimer>
#include <QElapsedTimer>
#include <QQueue>
#include <QHash>
#include "protocol/ProtocolCodec.h"
#include "transport/ITransport.h"
enum class ConnectionState { Disconnected, Opening, Handshaking, Ready, Error };
struct Response { quint64 requestId=0; Command command=Command::Hello; Status status=Status::Ok; QByteArray data; };
Q_DECLARE_METATYPE(ConnectionState)
Q_DECLARE_METATYPE(Snapshot)
Q_DECLARE_METATYPE(DeviceInfo)
Q_DECLARE_METATYPE(Response)
class SessionController : public QObject {
    Q_OBJECT
public:
    explicit SessionController(ITransport *transport,QObject *parent=nullptr);
    void connectDevice(const QString &endpoint,quint32 token=0);
    void disconnectDevice();
    quint64 submit(Command command,const QByteArray &payload={});
    ConnectionState state() const {return currentState;}
    quint32 sessionId() const {return activeSession;}
    int pendingCount() const {return pending.size()+queue.size();}
signals:
    void stateChanged(ConnectionState state);
    void infoReceived(DeviceInfo info);
    void snapshotReceived(Snapshot snapshot);
    void responseReceived(Response response);
    void requestTimedOut(quint64 requestId);
    void requestCanceled(quint64 requestId);
    void errorOccurred(QString message);
    void eventReceived(quint16 command,QByteArray payload);
private:
    struct Request {quint64 id=0; Command command=Command::Hello; QByteArray payload; bool bootstrap=false;};
    struct Pending {Request request; Frame frame; qint64 deadline=0; int retries=0;};
    void changeState(ConnectionState state);
    void receive(const QByteArray &bytes);
    void tick();
    void dispatch();
    void sendRequest(const Request &request);
    void bootstrap(Command command,const QByteArray &payload={});
    void fail(const QString &message);
    void reset();
    bool has(Command command) const;
    ITransport *transport;
    ProtocolCodec codec;
    QTimer timer;
    QElapsedTimer clock;
    QHash<quint16,Pending> pending;
    QQueue<Request> queue;
    DeviceInfo deviceInfo;
    ConnectionState currentState=ConnectionState::Disconnected;
    quint32 activeSession=0,proposedSession=0;
    quint16 nextSequence=1;
    quint64 nextId=1;
    qint64 lastHeartbeat=0,lastHeartbeatSent=0;
    quint64 receivedBytes=0,parsedFrames=0;
    bool resetting=false;
};
