#include "ProtocolMessages.h"
#include <QtEndian>
#include <QSet>
#include <limits>

namespace {
const quint32 MaxSigned = quint32(std::numeric_limits<qint32>::max());
template<typename T> T read(const QByteArray &bytes, int offset) {
    return qFromLittleEndian<T>(reinterpret_cast<const uchar *>(bytes.constData()+offset));
}
template<typename T> void append(QByteArray &bytes, T value) {
    const T little=qToLittleEndian(value);
    bytes.append(reinterpret_cast<const char *>(&little), sizeof(T));
}
bool positive32(quint32 value) { return value > 0 && value <= MaxSigned; }
bool signedValue(qint32 value) { return value != std::numeric_limits<qint32>::min(); }
bool validInfo(const DeviceInfo &info) {
    return info.protocolRevision==1 && info.uid.size()==12 && positive32(info.unitsPerRev)
        && positive32(info.currentMaxMa) && positive32(info.calibrationMaxMa)
        && positive32(info.velocityMax) && positive32(info.accelerationMax);
}
}

Status ProtocolMessages::validateParameters(const QByteArray &bytes)
{
    if (bytes.size()<4 || bytes.size()>128 || read<quint16>(bytes,0)!=1) return Status::BadPayload;
    const quint16 count=read<quint16>(bytes,2);
    if (count==0 || count>15) return Status::BadPayload;
    QSet<quint16> ids;
    int offset=4;
    for (int index=0; index<count; ++index) {
        if (offset+4>bytes.size()) return Status::BadPayload;
        const quint16 id=read<quint16>(bytes,offset);
        const quint8 type=quint8(bytes[offset+2]), size=quint8(bytes[offset+3]);
        if (ids.contains(id)) return Status::BadPayload;
        ids.insert(id);
        quint8 expectedType=0;
        if ((id>=1 && id<=4) || (id>=0x0401 && id<=0x0403)) expectedType=2;
        else if ((id>=0x0101 && id<=0x0103) || (id>=0x0201 && id<=0x0204)) expectedType=1;
        else if (id==0x0301) expectedType=3;
        else return Status::BadPayload;
        if (type!=expectedType || size!=(type==3 ? 1 : 4) || offset+4+size>bytes.size()) return Status::BadPayload;
        offset+=4;
        if (type==1) {
            const qint32 gain=read<qint32>(bytes,offset);
            const qint32 maximum=(id<=0x0103 ? 255 : 4095);
            if (gain<0 || gain>maximum) return Status::OutOfRange;
        } else if (type==3) {
            if (quint8(bytes[offset])>1) return Status::OutOfRange;
        } else {
            const quint32 value=read<quint32>(bytes,offset);
            if (id<=4 && !positive32(value)) return Status::OutOfRange;
            if (id==0x0401 && (value==0 || value>51200)) return Status::OutOfRange;
            if (id==0x0402 && value>51200) return Status::OutOfRange;
            if (id==0x0403 && (value<10 || value>2000)) return Status::OutOfRange;
        }
        offset+=size;
    }
    return offset==bytes.size() ? Status::Ok : Status::BadPayload;
}

Status ProtocolMessages::validateRequest(const Frame &frame)
{
    if (frame.type!=FrameType::Request || frame.sequence==0 || frame.payload.size()>128) return Status::BadPayload;
    const auto command=Command(frame.command);
    const auto &bytes=frame.payload;
    if (command==Command::Hello) {
        if (frame.session!=0) return Status::BadSession;
        return bytes.size()==4 && read<quint32>(bytes,0)!=0 ? Status::Ok : Status::BadPayload;
    }
    if (frame.session==0) return Status::BadSession;
    switch (command) {
    case Command::GetInfo: case Command::GetStatus: case Command::Heartbeat:
    case Command::Disable: case Command::Stop: case Command::SetZero:
    case Command::ClearFault: case Command::ReadParams: case Command::CalibrateStart:
        return bytes.isEmpty() ? Status::Ok : Status::BadPayload;
    case Command::Enable:
        if (bytes.size()!=1) return Status::BadPayload;
        return quint8(bytes[0])<=2 ? Status::Ok : Status::OutOfRange;
    case Command::MoveAbsolute:
        if (bytes.size()!=12) return Status::BadPayload;
        return positive32(read<quint32>(bytes,4)) && positive32(read<quint32>(bytes,8)) ? Status::Ok : Status::OutOfRange;
    case Command::SetVelocity:
        if (bytes.size()!=8) return Status::BadPayload;
        return signedValue(read<qint32>(bytes,0)) && positive32(read<quint32>(bytes,4)) ? Status::Ok : Status::OutOfRange;
    case Command::SetCurrent:
        if (bytes.size()!=4) return Status::BadPayload;
        return signedValue(read<qint32>(bytes,0)) ? Status::Ok : Status::OutOfRange;
    case Command::SaveParams:
        return bytes.size()==4 ? Status::Ok : Status::BadPayload;
    case Command::TaskQuery: case Command::CalibrateQuery: case Command::CalibrateCancel:
        if (bytes.size()!=4) return Status::BadPayload;
        return read<quint32>(bytes,0)!=0 ? Status::Ok : Status::OutOfRange;
    case Command::WriteParams: return validateParameters(bytes);
    case Command::TelemetryConfig: {
        if (bytes.size()!=2) return Status::BadPayload;
        const quint16 period=read<quint16>(bytes,0);
        return period==0 || period==10 || period==20 || period==50 || period==100 ? Status::Ok : Status::OutOfRange;
    }
    default: return Status::UnknownCommand;
    }
}

QByteArray ProtocolMessages::encodeSnapshot(const Snapshot &value)
{
    if (quint8(value.state)>4 || quint8(value.mode)>3) return {};
    QByteArray bytes;
    append(bytes,value.timestampMs); append(bytes,value.sampleCounter);
    append(bytes,value.position); append(bytes,value.targetPosition);
    append(bytes,value.velocity); append(bytes,value.targetVelocity);
    append(bytes,value.currentCommandMa); append(bytes,value.targetCurrentMa);
    append(bytes,value.positionError); append(bytes,value.faultBits);
    bytes.append(char(value.state)); bytes.append(char(value.mode));
    bytes.append(char(value.calibrated)); bytes.append(char(value.encoderValid));
    append(bytes,value.encoderErrorCount); append(bytes,value.txDropCount);
    return bytes;
}

bool ProtocolMessages::decodeSnapshot(const QByteArray &bytes, Snapshot &value)
{
    if (bytes.size()!=48 || quint8(bytes[40])>4 || quint8(bytes[41])>3
        || quint8(bytes[42])>1 || quint8(bytes[43])>1) return false;
    Snapshot next;
    next.timestampMs=read<quint32>(bytes,0); next.sampleCounter=read<quint32>(bytes,4);
    next.position=read<qint32>(bytes,8); next.targetPosition=read<qint32>(bytes,12);
    next.velocity=read<qint32>(bytes,16); next.targetVelocity=read<qint32>(bytes,20);
    next.currentCommandMa=read<qint32>(bytes,24); next.targetCurrentMa=read<qint32>(bytes,28);
    next.positionError=read<qint32>(bytes,32); next.faultBits=read<quint32>(bytes,36);
    next.state=DeviceState(quint8(bytes[40])); next.mode=ControlMode(quint8(bytes[41]));
    next.calibrated=bytes[42]!=0; next.encoderValid=bytes[43]!=0;
    next.encoderErrorCount=read<quint16>(bytes,44); next.txDropCount=read<quint16>(bytes,46);
    value=next;
    return true;
}

QByteArray ProtocolMessages::encodeInfo(const DeviceInfo &info)
{
    if (!validInfo(info)) return {};
    QByteArray bytes;
    append(bytes,info.protocolRevision); append(bytes,info.firmwareMajor);
    append(bytes,info.firmwareMinor); append(bytes,info.firmwarePatch); append(bytes,info.hardwareRevision);
    bytes.append(info.uid); append(bytes,info.capabilities); append(bytes,info.unitsPerRev);
    append(bytes,info.currentMaxMa); append(bytes,info.calibrationMaxMa);
    append(bytes,info.velocityMax); append(bytes,info.accelerationMax); append(bytes,info.parameterRevision);
    return bytes;
}

bool ProtocolMessages::decodeInfo(const QByteArray &bytes, DeviceInfo &info)
{
    if (bytes.size()!=50) return false;
    DeviceInfo next;
    next.protocolRevision=read<quint16>(bytes,0); next.firmwareMajor=read<quint16>(bytes,2);
    next.firmwareMinor=read<quint16>(bytes,4); next.firmwarePatch=read<quint16>(bytes,6);
    next.hardwareRevision=read<quint16>(bytes,8); next.uid=bytes.mid(10,12);
    next.capabilities=read<quint32>(bytes,22); next.unitsPerRev=read<quint32>(bytes,26);
    next.currentMaxMa=read<quint32>(bytes,30); next.calibrationMaxMa=read<quint32>(bytes,34);
    next.velocityMax=read<quint32>(bytes,38); next.accelerationMax=read<quint32>(bytes,42);
    next.parameterRevision=read<quint32>(bytes,46);
    if (!validInfo(next)) return false;
    info=next;
    return true;
}

bool ProtocolMessages::decodeResponse(const Frame &frame, Status &status, QByteArray &data)
{
    if (frame.type!=FrameType::Response || frame.sequence==0
        || frame.payload.size()<2 || frame.payload.size()>128) return false;
    const quint16 raw=read<quint16>(frame.payload,0);
    if (raw>13) return false;
    const Status result=Status(raw);
    const QByteArray body=frame.payload.mid(2);
    const Command command=Command(frame.command);
    if (frame.session==0 && !(command==Command::Hello && raw>=2)) return false;
    bool valid=false;
    if (result!=Status::Ok && result!=Status::Accepted) valid=body.isEmpty();
    else if (result==Status::Accepted) {
        int length=-1;
        if (command==Command::MoveAbsolute) length=16;
        if (command==Command::SaveParams) length=8;
        if (command==Command::CalibrateStart) length=4;
        valid=body.size()==length && length>=4 && read<quint32>(body,0)!=0;
        if (valid && command==Command::MoveAbsolute) {
            Frame echo; echo.session=1; echo.sequence=1; echo.command=frame.command; echo.payload=body.mid(4);
            valid=validateRequest(echo)==Status::Ok;
        }
    } else {
        switch (command) {
        case Command::Hello: valid=body.size()==2 && read<quint16>(body,0)==1; break;
        case Command::GetInfo: { DeviceInfo info; valid=decodeInfo(body,info); break; }
        case Command::GetStatus: case Command::Disable: case Command::Stop: case Command::ClearFault: {
            Snapshot snapshot; valid=decodeSnapshot(body,snapshot); break;
        }
        case Command::Heartbeat: case Command::SetZero: valid=body.size()==4; break;
        case Command::Enable: valid=body.size()==1 && quint8(body[0])<=2; break;
        case Command::SetVelocity: case Command::SetCurrent: case Command::TelemetryConfig: {
            Frame echo; echo.session=1; echo.sequence=1; echo.command=frame.command; echo.payload=body;
            valid=validateRequest(echo)==Status::Ok; break;
        }
        case Command::ReadParams: case Command::WriteParams:
            valid=body.size()>=8 && validateParameters(body.mid(4))==Status::Ok; break;
        case Command::CalibrateCancel: valid=body.size()==4 && read<quint32>(body,0)!=0; break;
        case Command::TaskQuery:
            valid=body.size()==12 && read<quint32>(body,0)!=0 && quint8(body[4])>=1
                && quint8(body[4])<=3 && quint8(body[5])<=6 && read<quint16>(body,6)<=13; break;
        case Command::CalibrateQuery:
            valid=body.size()==13 && read<quint32>(body,0)!=0 && quint8(body[4])<=6
                && quint8(body[5])<=100 && read<quint16>(body,6)<=13 && quint8(body[8])<=1; break;
        default: break;
        }
    }
    if (!valid) return false;
    status=result; data=body;
    return true;
}

bool ProtocolMessages::validateUnsolicited(const Frame &frame)
{
    if (frame.session==0) return false;
    const QByteArray &body=frame.payload;
    if (frame.type==FrameType::Telemetry && Command(frame.command)==Command::Telemetry) {
        Snapshot snapshot;
        return decodeSnapshot(body,snapshot);
    }
    if (frame.type!=FrameType::Event) return false;
    if (Command(frame.command)==Command::TaskEvent) {
        return body.size()==12 && read<quint32>(body,0)!=0 && quint8(body[4])>=1
            && quint8(body[4])<=6 && body[5]==0 && read<quint16>(body,6)<=13;
    }
    if (Command(frame.command)==Command::FaultEvent) {
        return body.size()==10 && quint8(body[4])<=4 && quint8(body[5])<=3;
    }
    return false;
}
