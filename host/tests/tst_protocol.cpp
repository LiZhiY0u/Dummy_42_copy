#include <QtTest>
#include "protocol/ProtocolCodec.h"

class ProtocolTest : public QObject
{
    Q_OBJECT
    QByteArray golden() const { return QByteArray::fromHex("AA5501010C00443322110100030100C8000000C8000000900100FCA7"); }
    Frame move() const
    {
        Frame frame;
        frame.type = FrameType::Request;
        frame.session = 0x11223344;
        frame.sequence = 1;
        frame.command = 0x0103;
        frame.payload = QByteArray::fromHex("00C8000000C8000000900100");
        return frame;
    }
private slots:
    void crcStandardVector() { QCOMPARE(ProtocolCodec::crc16("123456789"), quint16(0x29B1)); }
    void encodesGoldenFrame() { QCOMPARE(ProtocolCodec::encode(move()), golden()); }
    void everySplitPoint()
    {
        const QByteArray bytes = golden();
        for (int split = 1; split < bytes.size(); ++split) {
            ProtocolCodec codec;
            QVERIFY(codec.feed(bytes.left(split), 0).isEmpty());
            const auto frames = codec.feed(bytes.mid(split), 1);
            QCOMPARE(frames.size(), 1);
            QCOMPARE(frames.first().session, quint32(0x11223344));
            QCOMPARE(frames.first().payload, move().payload);
        }
    }
    void concatenatedFrames()
    {
        ProtocolCodec codec;
        QCOMPARE(codec.feed(golden() + golden(), 0).size(), 2);
    }
    void payloadContainsHeaderAndNegativeValue()
    {
        Frame frame = move();
        frame.command = 0x0104;
        frame.payload = QByteArray::fromHex("0038FFFFAA550000"); // -51200, 21930
        ProtocolCodec codec;
        const auto frames = codec.feed(ProtocolCodec::encode(frame), 0);
        QCOMPARE(frames.size(), 1);
        QCOMPARE(frames.first().payload, frame.payload);
    }
    void corruptCrcRecovers()
    {
        QByteArray bad = golden();
        bad[20] = char(bad[20] ^ 1);
        ProtocolCodec codec;
        const auto frames = codec.feed(bad + golden(), 0);
        QCOMPARE(frames.size(), 1);
        QCOMPARE(frames.first().payload, move().payload);
    }
    void oversizeLengthRecovers()
    {
        QByteArray bad = golden().left(14);
        bad[4] = char(129);
        ProtocolCodec codec;
        QCOMPARE(codec.feed(bad + golden(), 0).size(), 1);
        Frame tooBig = move();
        tooBig.payload = QByteArray(129, 0);
        QVERIFY(ProtocolCodec::encode(tooBig).isEmpty());
    }
    void noiseAndMaximumPayload()
    {
        Frame frame = move();
        frame.payload = QByteArray(128, char(0xAA));
        ProtocolCodec codec;
        const auto frames = codec.feed(QByteArray(4096, char(0xAA)) + ProtocolCodec::encode(frame), 0);
        QCOMPARE(frames.size(), 1);
        QCOMPARE(frames.first().payload.size(), 128);
        QVERIFY(codec.bufferedBytes() <= 144);
    }
    void partialFrameTimeout()
    {
        ProtocolCodec codec;
        QVERIFY(codec.feed(golden().left(20), 0).isEmpty());
        QVERIFY(codec.feed({}, 100).isEmpty());
        QCOMPARE(codec.feed(golden(), 101).size(), 1);
    }
    void rejectsVersionAndType()
    {
        for (int field : {2, 3}) {
            QByteArray bad = golden();
            bad[field] = char(0x7F);
            const quint16 crc = ProtocolCodec::crc16(bad.mid(2, bad.size() - 4));
            bad[bad.size()-2] = char(crc & 0xFF);
            bad[bad.size()-1] = char(crc >> 8);
            ProtocolCodec codec;
            QCOMPARE(codec.feed(bad + golden(), 0).size(), 1);
        }
    }
    void resetDropsPartialFrame()
    {
        ProtocolCodec codec;
        codec.feed(golden().left(20), 0);
        codec.reset();
        QCOMPARE(codec.bufferedBytes(), 0);
        QCOMPARE(codec.feed(golden(), 1).size(), 1);
    }
};
int runProtocolTests(int argc, char **argv)
{
    ProtocolTest test;
    return QTest::qExec(&test, argc, argv);
}
#include "tst_protocol.moc"
