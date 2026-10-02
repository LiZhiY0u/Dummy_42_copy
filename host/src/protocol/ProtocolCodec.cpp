#include "ProtocolCodec.h"
#include <QtEndian>

namespace {
template<typename T> void appendLittleEndian(QByteArray &bytes, T value)
{
    const T little = qToLittleEndian(value);
    bytes.append(reinterpret_cast<const char *>(&little), sizeof(T));
}
template<typename T> T readLittleEndian(const QByteArray &bytes, int offset)
{
    return qFromLittleEndian<T>(reinterpret_cast<const uchar *>(bytes.constData() + offset));
}
bool validType(quint8 type) { return type >= 1 && type <= 4; }
}

quint16 ProtocolCodec::crc16(const QByteArray &bytes)
{
    quint16 crc = 0xFFFF;
    for (char byte : bytes) {
        crc ^= quint16(quint8(byte)) << 8;
        for (int bit = 0; bit < 8; ++bit)
            crc = quint16((crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1);
    }
    return crc;
}

QByteArray ProtocolCodec::encode(const Frame &frame)
{
    if (frame.payload.size() > MaximumPayload || !validType(quint8(frame.type))) return {};
    QByteArray bytes = QByteArray::fromHex("AA55");
    bytes.reserve(16 + frame.payload.size());
    bytes.append(char(1));
    bytes.append(char(frame.type));
    appendLittleEndian(bytes, quint16(frame.payload.size()));
    appendLittleEndian(bytes, frame.session);
    appendLittleEndian(bytes, frame.sequence);
    appendLittleEndian(bytes, frame.command);
    bytes.append(frame.payload);
    appendLittleEndian(bytes, crc16(bytes.mid(2)));
    return bytes;
}

QVector<Frame> ProtocolCodec::feed(const QByteArray &bytes, qint64 nowMs)
{
    QVector<Frame> frames;
    if (candidateSince_ >= 0 && nowMs - candidateSince_ >= PartialTimeoutMs) reset();
    // Incremental parsing keeps retained input bounded, even for huge noise bursts.
    for (char byte : bytes) {
        buffer_.append(byte);
        parse(nowMs, frames);
    }
    return frames;
}

void ProtocolCodec::parse(qint64 nowMs, QVector<Frame> &frames)
{
    while (!buffer_.isEmpty()) {
        if (quint8(buffer_[0]) != 0xAA) {
            buffer_.remove(0, 1);
            candidateSince_ = -1;
            continue;
        }
        if (candidateSince_ < 0) candidateSince_ = nowMs;
        if (buffer_.size() < 2) return;
        if (quint8(buffer_[1]) != 0x55) {
            buffer_.remove(0, 1);
            candidateSince_ = -1;
            continue;
        }
        if (buffer_.size() < 14) return;
        const quint16 length = readLittleEndian<quint16>(buffer_, 4);
        if (quint8(buffer_[2]) != 1 || !validType(quint8(buffer_[3])) || length > MaximumPayload) {
            buffer_.remove(0, 1);
            candidateSince_ = -1;
            continue;
        }
        const int total = 16 + length;
        if (buffer_.size() < total) return;
        const quint16 expected = readLittleEndian<quint16>(buffer_, total - 2);
        if (crc16(buffer_.mid(2, total - 4)) != expected) {
            buffer_.remove(0, 1);
            candidateSince_ = -1;
            continue;
        }
        Frame frame;
        frame.type = FrameType(quint8(buffer_[3]));
        frame.session = readLittleEndian<quint32>(buffer_, 6);
        frame.sequence = readLittleEndian<quint16>(buffer_, 10);
        frame.command = readLittleEndian<quint16>(buffer_, 12);
        frame.payload = buffer_.mid(14, length);
        frames.append(frame);
        buffer_.remove(0, total);
        candidateSince_ = -1;
    }
}

void ProtocolCodec::reset()
{
    buffer_.clear();
    candidateSince_ = -1;
}
