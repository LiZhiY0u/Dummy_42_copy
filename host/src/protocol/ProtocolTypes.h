#ifndef STEPPER_PROTOCOL_TYPES_H
#define STEPPER_PROTOCOL_TYPES_H
#include <QByteArray>
#include <QtGlobal>

enum class FrameType : quint8 { Request = 1, Response = 2, Event = 3, Telemetry = 4 };
enum class Command : quint16 {
    Hello=0x0001, GetInfo=0x0002, GetStatus=0x0003, Heartbeat=0x0004, TaskQuery=0x0005,
    Enable=0x0101, Disable=0x0102, MoveAbsolute=0x0103, SetVelocity=0x0104,
    SetCurrent=0x0105, Stop=0x0106, SetZero=0x0107, ClearFault=0x0108,
    ReadParams=0x0201, WriteParams=0x0202, SaveParams=0x0203,
    CalibrateStart=0x0301, CalibrateQuery=0x0302, CalibrateCancel=0x0303,
    TelemetryConfig=0x0401, Telemetry=0x0402, TaskEvent=0x0501, FaultEvent=0x0502
};
enum class Status : quint16 {
    Ok=0, Accepted=1, UnknownCommand=2, BadPayload=3, OutOfRange=4, WrongState=5,
    NotCalibrated=6, EncoderFault=7, Busy=8, StorageError=9, Unsupported=10,
    BadSession=11, SequenceConflict=12, InternalError=13
};
enum class ControlMode : quint8 { Position=0, Velocity=1, Current=2, None=3 };
enum class DeviceState : quint8 { Disabled=0, Ready=1, Running=2, Calibrating=3, Fault=4 };
struct Snapshot {
    quint32 timestampMs=0, sampleCounter=0;
    qint32 position=0, targetPosition=0, velocity=0, targetVelocity=0;
    qint32 currentCommandMa=0, targetCurrentMa=0, positionError=0;
    quint32 faultBits=0;
    DeviceState state=DeviceState::Disabled;
    ControlMode mode=ControlMode::None;
    bool calibrated=false, encoderValid=false;
    quint16 encoderErrorCount=0, txDropCount=0;
};
struct DeviceInfo {
    quint16 protocolRevision=1, firmwareMajor=0, firmwareMinor=0, firmwarePatch=0, hardwareRevision=0;
    QByteArray uid;
    quint32 capabilities=0, unitsPerRev=0, currentMaxMa=0, calibrationMaxMa=0;
    quint32 velocityMax=0, accelerationMax=0, parameterRevision=0;
};
struct Frame
{
    FrameType type = FrameType::Request;
    quint32 session = 0;
    quint16 sequence = 0;
    quint16 command = 0;
    QByteArray payload;
};
#endif
