#include "MockTransport.h"
#include "protocol/ProtocolMessages.h"
#include <QtEndian>
namespace {
template<class T> QByteArray packed(T value) {value=qToLittleEndian(value);return QByteArray(reinterpret_cast<const char*>(&value),sizeof(value));}
template<class T> T read(const QByteArray &bytes,int offset=0) {return qFromLittleEndian<T>(reinterpret_cast<const uchar*>(bytes.constData()+offset));}
}
MockTransport::MockTransport(QObject *parent):ITransport(parent),telemetry(this),watchdog(this) {
    info.uid="MOCK00000001";info.firmwareMinor=1;info.capabilities=(1u<<1)|(1u<<3)|(1u<<8);
    info.unitsPerRev=51200;info.currentMaxMa=1000;info.calibrationMaxMa=2000;
    info.velocityMax=1536000;info.accelerationMax=1000000;info.parameterRevision=1;
    snapshot.calibrated=true;snapshot.encoderValid=true;
    parameters.insert(0x0101,QByteArray::fromHex("010420000000"));
    watchdog.setInterval(10);connect(&watchdog,&QTimer::timeout,this,[this]{if(session&&clock.elapsed()-heartbeat>=500)reboot();});
    connect(&telemetry,&QTimer::timeout,this,[this]{
        if(!connected||!session)return;
        Frame f;f.type=FrameType::Telemetry;f.command=quint16(Command::Telemetry);f.session=session;
        f.payload=sampleSnapshot();deliver(f);
    });
}
void MockTransport::open(const QString &endpoint) {Q_UNUSED(endpoint);close();connected=true;clock.start();watchdog.start();emit opened();}
void MockTransport::close() {const bool was=connected;connected=false;watchdog.stop();reboot();if(was)emit closed();}
void MockTransport::reboot() {
    generation++;session=0;highWater=0;cache.clear();incoming.clear();pumping=false;codec.reset();telemetry.stop();
    snapshot=Snapshot();snapshot.calibrated=true;snapshot.encoderValid=true;
}
bool MockTransport::send(const QByteArray &bytes) {
    if(!connected)return false;
    emit frameQueued(bytes);
    for(const Frame &f:codec.feed(bytes,clock.elapsed()))handle(f);
    return true;
}
void MockTransport::handle(const Frame &request) {
    const Command c=Command(request.command);
    Frame response=request;response.type=FrameType::Response;
    Status status=ProtocolMessages::validateRequest(request);QByteArray body;
    if(c!=Command::Hello && request.session!=session)status=Status::BadSession;
    if(status==Status::Ok && c!=Command::Hello) {
        for(const Cached &entry:cache)if(entry.request.sequence==request.sequence && clock.elapsed()-entry.time<2000) {
            if(entry.request.command==request.command&&entry.request.payload==request.payload){if(!drops.value(request.command))deliver(entry.response,delays.value(request.command));return;}
            status=Status::SequenceConflict;break;
        }
        const quint16 delta=quint16(request.sequence-highWater);
        if(status==Status::Ok && highWater && (delta==0||delta>32767))status=Status::SequenceConflict;
    }
    if(status==Status::Ok) {
        if(c!=Command::Hello)highWater=request.sequence;
        switch(c) {
        case Command::Hello: {
            const quint32 token=read<quint32>(request.payload);
            if(token!=session && snapshot.state!=DeviceState::Disabled){status=Status::WrongState;response.session=0;break;}
            if(token!=session){session=token;cache.clear();highWater=request.sequence;heartbeat=clock.elapsed();telemetry.stop();}
            response.session=session;body=packed<quint16>(1);break;
        }
        case Command::GetInfo:body=ProtocolMessages::encodeInfo(info);break;
        case Command::GetStatus:body=sampleSnapshot();break;
        case Command::Heartbeat:heartbeat=clock.elapsed();body=packed<quint32>(quint32(clock.elapsed()));break;
        case Command::ReadParams: {
            body=packed(info.parameterRevision)+packed<quint16>(1)+packed<quint16>(quint16(parameters.size()));
            for(auto it=parameters.cbegin();it!=parameters.cend();++it)body+=packed(it.key())+it.value();
            break;
        }
        case Command::WriteParams: {
            if(snapshot.state!=DeviceState::Disabled){status=Status::WrongState;break;}
            auto next=parameters;int offset=4;
            while(offset<request.payload.size()) {
                const quint16 id=read<quint16>(request.payload,offset);const int length=quint8(request.payload[offset+3]);
                if(id>=1&&id<=4){const quint32 maximum=id==1?info.currentMaxMa:id==2?info.velocityMax:id==3?info.accelerationMax:info.calibrationMaxMa;
                    if(read<quint32>(request.payload,offset+4)>maximum){status=Status::OutOfRange;break;}}
                next[id]=request.payload.mid(offset+2,length+2);offset+=length+4;
            }
            if(status==Status::Ok){parameters=next;info.parameterRevision++;body=packed(info.parameterRevision)+request.payload;}break;
        }
        case Command::TelemetryConfig:telemetry.stop();if(read<quint16>(request.payload))telemetry.start(read<quint16>(request.payload));body=request.payload;break;
        case Command::Stop:case Command::Disable:
            snapshot.state=DeviceState::Disabled;snapshot.mode=ControlMode::None;snapshot.velocity=0;snapshot.targetVelocity=0;
            body=sampleSnapshot();break;
        case Command::Enable:
            if(quint8(request.payload[0])!=1)status=Status::Unsupported;
            else {snapshot.state=DeviceState::Ready;snapshot.mode=ControlMode::Velocity;body=request.payload;}break;
        case Command::SetVelocity:
            if(snapshot.mode!=ControlMode::Velocity)status=Status::WrongState;
            else if(qAbs(qint64(read<qint32>(request.payload)))>info.velocityMax||read<quint32>(request.payload,4)>info.accelerationMax)status=Status::OutOfRange;
            else {snapshot.targetVelocity=read<qint32>(request.payload);snapshot.state=DeviceState::Running;body=request.payload;}break;
        default:status=Status::Unsupported;break;
        }
    }
    if(status!=Status::Ok)body.clear();
    response.payload=packed<quint16>(quint16(status))+body;
    if(c!=Command::Hello&&request.session==session&&status!=Status::SequenceConflict) {
        Cached entry;entry.request=request;entry.response=response;entry.time=clock.elapsed();cache.append(entry);while(cache.size()>4)cache.removeFirst();
    }
    if(!drops.value(request.command))deliver(response,delays.value(request.command));
}
void MockTransport::deliver(const Frame &frame,int delay) {
    const auto epoch=generation;const QByteArray bytes=ProtocolCodec::encode(frame);
    QTimer::singleShot(delay,this,[this,epoch,bytes]{
        if(!connected||generation!=epoch)return;
        if(incoming.size()+bytes.size()>4096){emit transportError(QStringLiteral("模拟接收缓存满"));return;}
        incoming.append(bytes);if(!pumping){pumping=true;pump();}
    });
}
void MockTransport::pump() {
    if(!connected||incoming.isEmpty()){pumping=false;return;}
    const QByteArray bytes=incoming.left(chunkSize);incoming.remove(0,bytes.size());emit bytesReceived(bytes);
    const auto epoch=generation;QTimer::singleShot(0,this,[this,epoch]{if(epoch==generation)pump();});
}
QByteArray MockTransport::sampleSnapshot() {
    snapshot.timestampMs=quint32(clock.elapsed());snapshot.sampleCounter++;
    return ProtocolMessages::encodeSnapshot(snapshot);
}
