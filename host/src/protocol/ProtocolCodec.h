#ifndef STEPPER_PROTOCOL_CODEC_H
#define STEPPER_PROTOCOL_CODEC_H
#include "ProtocolTypes.h"
#include <QVector>

class ProtocolCodec
{
public:
    static const int MaximumPayload = 128;
    static const int PartialTimeoutMs = 100;
    static quint16 crc16(const QByteArray &bytes);
    static QByteArray encode(const Frame &frame);
    QVector<Frame> feed(const QByteArray &bytes, qint64 nowMs);
    void reset();
    int bufferedBytes() const { return buffer_.size(); }
private:
    void parse(qint64 nowMs, QVector<Frame> &frames);
    QByteArray buffer_;
    qint64 candidateSince_ = -1;
};
#endif
