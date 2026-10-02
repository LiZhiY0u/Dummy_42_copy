#include "SessionController.h"
#include "protocol/ProtocolMessages.h"
#include <QRandomGenerator>
#include <QtEndian>
namespace {
template<class T> QByteArray packed(T value) {value=qToLittleEndian(value);return QByteArray(reinterpret_cast<const char*>(&value),sizeof(value));}
bool readOnly(Command c) {return c==Command::GetInfo||c==Command::GetStatus||c==Command::ReadParams||c==Command::TaskQuery||c==Command::CalibrateQuery;}
// HELLO is safe only as an identical token/sequence retransmission. Never
// allocate a new request here or treat actuator commands as retryable.
bool retryAllowed(Command c) {return c==Command::Hello||readOnly(c);}
bool motion(Command c) {return c==Command::Enable||c==Command::MoveAbsolute||c==Command::SetVelocity||c==Command::SetCurrent||c==Command::CalibrateStart||c==Command::SetZero;}
}
SessionController::SessionController(ITransport *value,QObject *parent):QObject(parent),transport(value),timer(this) {
    qRegisterMetaType<ConnectionState>();qRegisterMetaType<Snapshot>();qRegisterMetaType<DeviceInfo>();qRegisterMetaType<Response>();
    connect(transport,&ITransport::opened,this,[this]{if(currentState==ConnectionState::Opening) {changeState(ConnectionState::Handshaking);bootstrap(Command::Hello,packed(proposedSession));}});
    connect(transport,&ITransport::bytesReceived,this,&SessionController::receive);
    connect(transport,&ITransport::transportError,this,[this](const QString &s){if(!resetting) fail(s);});
    connect(transport,&ITransport::closed,this,[this]{if(!resetting) fail(QStringLiteral("连接已关闭"));});
    timer.setInterval(10);connect(&timer,&QTimer::timeout,this,&SessionController::tick);
}
void SessionController::changeState(ConnectionState s) {currentState=s;emit stateChanged(s);}
void SessionController::reset() {timer.stop();pending.clear();queue.clear();codec.reset();activeSession=0;nextSequence=1;deviceInfo=DeviceInfo();receivedBytes=0;parsedFrames=0;}
void SessionController::connectDevice(const QString &endpoint,quint32 token) {
    disconnectDevice();proposedSession=token?token:QRandomGenerator::global()->generate();if(!proposedSession) proposedSession=1;
    clock.start();lastHeartbeat=0;lastHeartbeatSent=0;timer.start();changeState(ConnectionState::Opening);transport->open(endpoint);
}
void SessionController::disconnectDevice() {resetting=true;reset();transport->close();resetting=false;changeState(ConnectionState::Disconnected);}
void SessionController::fail(const QString &message) {if(resetting)return;resetting=true;reset();transport->close();resetting=false;changeState(ConnectionState::Error);emit errorOccurred(message);}
bool SessionController::has(Command command) const {for(const auto &p:pending) if(p.request.command==command)return true;return false;}
void SessionController::bootstrap(Command command,const QByteArray &payload) {Request r;r.command=command;r.payload=payload;r.bootstrap=true;sendRequest(r);}
void SessionController::sendRequest(const Request &request) {
    Pending p;p.request=request;p.frame.command=quint16(request.command);p.frame.payload=request.payload;
    p.frame.session=request.command==Command::Hello?0:activeSession;p.frame.sequence=nextSequence++;
    if(!nextSequence)nextSequence=1;
    p.deadline=clock.elapsed()+200;
    pending.insert(p.frame.sequence,p);
    if(!transport->send(ProtocolCodec::encode(p.frame)) && currentState!=ConnectionState::Error) fail(QStringLiteral("发送失败"));
}
quint64 SessionController::submit(Command command,const QByteArray &payload) {
    if(currentState!=ConnectionState::Ready || command==Command::Hello || command==Command::Heartbeat)return 0;
    Frame f;f.command=quint16(command);f.payload=payload;f.session=activeSession;f.sequence=1;
    if(ProtocolMessages::validateRequest(f)!=Status::Ok)return 0;
    int bit=-1;if(command==Command::MoveAbsolute)bit=0;if(command==Command::SetVelocity)bit=1;if(command==Command::SetCurrent)bit=2;
    if(command==Command::Enable)bit=quint8(payload[0]);
    if(command==Command::WriteParams||command==Command::ReadParams)bit=3;
    if(command==Command::SaveParams)bit=4;
    if(command==Command::CalibrateStart)bit=5;
    if(command==Command::SetZero)bit=6;
    if(command==Command::ClearFault)bit=7;
    if(command==Command::TelemetryConfig)bit=8;
    if(bit>=0 && !(deviceInfo.capabilities&(1u<<bit)))return 0;
    if(motion(command)&&(has(Command::Stop)||has(Command::Disable)))return 0;
    Request r;r.command=command;r.payload=payload;r.id=nextId++;
    if(command==Command::Stop || command==Command::Disable) {
        if(has(Command::Stop)||has(Command::Disable))return 0;
        QQueue<Request> retained;
        while(!queue.isEmpty()){const Request q=queue.dequeue();if(motion(q.command))emit requestCanceled(q.id);else retained.enqueue(q);}queue=retained;
        // Old query snapshots must not overwrite a later confirmed stop.
        const auto keys=pending.keys();for(auto key:keys) if(pending[key].request.command!=Command::Heartbeat){emit requestCanceled(pending[key].request.id);pending.remove(key);}
        sendRequest(r);
    } else {if(queue.size()>=16)return 0;queue.enqueue(r);dispatch();}
    return r.id;
}
void SessionController::dispatch() {
    if(currentState!=ConnectionState::Ready || has(Command::Stop)||has(Command::Disable))return;
    for(const auto &p:pending)if(p.request.command!=Command::Heartbeat)return;
    if(!queue.isEmpty())sendRequest(queue.dequeue());
}
void SessionController::receive(const QByteArray &bytes) {
    receivedBytes+=quint64(bytes.size());
    for(const Frame &f:codec.feed(bytes,clock.isValid()?clock.elapsed():0)) {
        ++parsedFrames;
        if(f.type!=FrameType::Response) {
            if(f.session==activeSession && activeSession && ProtocolMessages::validateUnsolicited(f)) {
                if(f.type==FrameType::Telemetry){Snapshot s;ProtocolMessages::decodeSnapshot(f.payload,s);emit snapshotReceived(s);}
                else emit eventReceived(f.command,f.payload);
            }continue;
        }
        if(!pending.contains(f.sequence))continue;
        const Pending p=pending.value(f.sequence);const Command c=p.request.command;
        if(f.command!=quint16(c) || (c==Command::Hello ? (f.session!=proposedSession&&f.session!=0) : f.session!=activeSession))continue;
        Status status;QByteArray data;if(!ProtocolMessages::decodeResponse(f,status,data))continue;
        if(status==Status::Ok && (c==Command::TaskQuery||c==Command::CalibrateQuery||c==Command::CalibrateCancel)
            && data.left(4)!=p.request.payload)continue;
        pending.remove(f.sequence);
        if(status==Status::BadSession){fail(QStringLiteral("设备会话失效，需重新连接"));return;}
        if(c==Command::Heartbeat){if(status==Status::Ok)lastHeartbeat=clock.elapsed();continue;}
        if(p.request.bootstrap && status!=Status::Ok){fail(QStringLiteral("握手被设备拒绝：%1").arg(quint16(status)));return;}
        if(status==Status::Ok) {
            if(c==Command::Hello){activeSession=proposedSession;lastHeartbeat=clock.elapsed();lastHeartbeatSent=lastHeartbeat;bootstrap(Command::GetInfo);}
            if(c==Command::GetInfo){ProtocolMessages::decodeInfo(data,deviceInfo);emit infoReceived(deviceInfo);if(p.request.bootstrap)bootstrap(Command::GetStatus);}
            if(c==Command::GetStatus||c==Command::Disable||c==Command::Stop||c==Command::ClearFault){Snapshot s;ProtocolMessages::decodeSnapshot(data,s);emit snapshotReceived(s);}
        }
        if(p.request.bootstrap) {
            if(c==Command::GetStatus){if(deviceInfo.capabilities&(1u<<3))bootstrap(Command::ReadParams);else if(deviceInfo.capabilities&(1u<<8))bootstrap(Command::TelemetryConfig,packed<quint16>(20));else changeState(ConnectionState::Ready);}
            if(c==Command::ReadParams){if(deviceInfo.capabilities&(1u<<8))bootstrap(Command::TelemetryConfig,packed<quint16>(20));else changeState(ConnectionState::Ready);}
            if(c==Command::TelemetryConfig)changeState(ConnectionState::Ready);
        }else {Response r;r.requestId=p.request.id;r.command=c;r.status=status;r.data=data;emit responseReceived(r);}
        dispatch();
    }
}
void SessionController::tick() {
    receive({});const qint64 now=clock.elapsed();
    if(activeSession && now-lastHeartbeat>=500){fail(QStringLiteral("心跳超过500ms无确认"));return;}
    const auto keys=pending.keys();
    for(auto key:keys){if(!pending.contains(key))continue;Pending &p=pending[key];if(now<p.deadline)continue;
        if(retryAllowed(p.request.command)&&p.retries==0){p.retries++;p.deadline=now+200;const QByteArray bytes=ProtocolCodec::encode(p.frame);if(!transport->send(bytes))return;}
        else {const Request r=p.request;pending.remove(key);if(r.bootstrap){fail(QStringLiteral("握手超时：cmd=0x%1 seq=%2 RX=%3 bytes frames=%4（CRC有效帧）")
            .arg(quint16(r.command),4,16,QChar('0')).arg(key).arg(receivedBytes).arg(parsedFrames));return;}if(r.command!=Command::Heartbeat)emit requestTimedOut(r.id);}
    }
    if(activeSession&&now-lastHeartbeatSent>=100&&!has(Command::Heartbeat)){Request r;r.command=Command::Heartbeat;lastHeartbeatSent=now;sendRequest(r);}
    dispatch();
}
