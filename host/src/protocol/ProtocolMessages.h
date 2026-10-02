#ifndef STEPPER_PROTOCOL_MESSAGES_H
#define STEPPER_PROTOCOL_MESSAGES_H
#include "ProtocolTypes.h"

class ProtocolMessages
{
public:
    static Status validateRequest(const Frame &frame);
    static bool decodeResponse(const Frame &frame, Status &status, QByteArray &data);
    static QByteArray encodeSnapshot(const Snapshot &snapshot);
    static bool decodeSnapshot(const QByteArray &bytes, Snapshot &snapshot);
    static QByteArray encodeInfo(const DeviceInfo &info);
    static bool decodeInfo(const QByteArray &bytes, DeviceInfo &info);
    static Status validateParameters(const QByteArray &bytes);
    static bool validateUnsolicited(const Frame &frame);
};
#endif
