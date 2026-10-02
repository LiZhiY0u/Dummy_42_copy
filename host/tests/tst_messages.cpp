#include <QtTest>
#include <QFile>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include "protocol/ProtocolMessages.h"
#include "protocol/ProtocolCodec.h"

class MessageTest : public QObject
{
    Q_OBJECT
    Frame request(Command command, const QByteArray &payload = {}) const
    {
        Frame frame;
        frame.session = 0x11223344;
        frame.sequence = 1;
        frame.command = quint16(command);
        frame.payload = payload;
        return frame;
    }
private slots:
    void handshakeAndSessionRules()
    {
        Frame hello = request(Command::Hello, QByteArray::fromHex("44332211"));
        hello.session = 0;
        QCOMPARE(ProtocolMessages::validateRequest(hello), Status::Ok);
        hello.payload.fill(0);
        QCOMPARE(ProtocolMessages::validateRequest(hello), Status::BadPayload);
        Frame query = request(Command::GetStatus);
        query.session = 0;
        QCOMPARE(ProtocolMessages::validateRequest(query), Status::BadSession);
        query.session = 1; query.sequence = 0;
        QCOMPARE(ProtocolMessages::validateRequest(query), Status::BadPayload);
    }
    void exactLengthsAndModes()
    {
        QCOMPARE(ProtocolMessages::validateRequest(request(Command::GetInfo, "x")), Status::BadPayload);
        QCOMPARE(ProtocolMessages::validateRequest(request(Command::Enable, QByteArray(1, 3))), Status::OutOfRange);
        QCOMPARE(ProtocolMessages::validateRequest(request(Command::Enable, QByteArray(1, 2))), Status::Ok);
        QCOMPARE(ProtocolMessages::validateRequest(request(Command::MoveAbsolute, QByteArray(11, 0))), Status::BadPayload);
        QCOMPARE(ProtocolMessages::validateRequest(request(Command::TelemetryConfig, QByteArray::fromHex("1400"))), Status::Ok);
        QCOMPARE(ProtocolMessages::validateRequest(request(Command::TelemetryConfig, QByteArray::fromHex("0F00"))), Status::OutOfRange);
    }
    void signedAndUnsignedBounds()
    {
        QCOMPARE(ProtocolMessages::validateRequest(request(Command::SetVelocity, QByteArray::fromHex("0038FFFF00900100"))), Status::Ok);
        QCOMPARE(ProtocolMessages::validateRequest(request(Command::SetVelocity, QByteArray::fromHex("0000008000900100"))), Status::OutOfRange);
        QCOMPARE(ProtocolMessages::validateRequest(request(Command::MoveAbsolute, QByteArray::fromHex("00C800000000000000900100"))), Status::OutOfRange);
        QCOMPARE(ProtocolMessages::validateRequest(request(Command::SetVelocity, QByteArray(8, 0))), Status::OutOfRange);
    }
    void parameterBatchValidation()
    {
        const QByteArray good = QByteArray::fromHex("010001000101010405000000"); // schema1, count1, PID kp i32=5
        QCOMPARE(ProtocolMessages::validateRequest(request(Command::WriteParams, good)), Status::Ok);
        QByteArray duplicate = QByteArray::fromHex("01000200") + good.mid(4) + good.mid(4);
        QCOMPARE(ProtocolMessages::validateRequest(request(Command::WriteParams, duplicate)), Status::BadPayload);
        QByteArray badType = good; badType[6] = 2;
        QCOMPARE(ProtocolMessages::validateRequest(request(Command::WriteParams, badType)), Status::BadPayload);
        QByteArray highGain = good; highGain[8] = 0; highGain[9] = 1;
        QCOMPARE(ProtocolMessages::validateRequest(request(Command::WriteParams, highGain)), Status::OutOfRange);
        QCOMPARE(ProtocolMessages::validateRequest(request(Command::WriteParams, good + 'x')), Status::BadPayload);
    }
    void rejectsUnknownCommandsAndWrongDirection()
    {
        Frame unknown = request(Command::GetStatus); unknown.command = 0x6666;
        QCOMPARE(ProtocolMessages::validateRequest(unknown), Status::UnknownCommand);
        unknown.type = FrameType::Response;
        QCOMPARE(ProtocolMessages::validateRequest(unknown), Status::BadPayload);
    }
    void responseStatusAndLengths()
    {
        Status status = Status::InternalError; QByteArray data("unchanged");
        Frame response = request(Command::GetStatus); response.type = FrameType::Response;
        response.payload = QByteArray::fromHex("0700");
        QVERIFY(ProtocolMessages::decodeResponse(response, status, data));
        QCOMPARE(status, Status::EncoderFault); QVERIFY(data.isEmpty());
        response.payload = QByteArray(49, 0);
        QVERIFY(!ProtocolMessages::decodeResponse(response, status, data));
        response.payload = QByteArray(50, 0);
        QVERIFY(ProtocolMessages::decodeResponse(response, status, data));
        response.payload = QByteArray::fromHex("0100");
        QVERIFY(!ProtocolMessages::decodeResponse(response, status, data));
        response.command = quint16(Command::CalibrateStart);
        response.payload = QByteArray::fromHex("010001000000");
        QVERIFY(ProtocolMessages::decodeResponse(response, status, data));
        response.payload = QByteArray::fromHex("010000000000");
        QVERIFY(!ProtocolMessages::decodeResponse(response, status, data));
    }
    void telemetryRoundtripAndAtomicFailure()
    {
        Snapshot original;
        original.timestampMs = 1234; original.sampleCounter = 17;
        original.position = -51200; original.velocity = -25600;
        original.mode = ControlMode::Velocity;
        original.encoderValid = true; original.calibrated = true;
        const QByteArray bytes = ProtocolMessages::encodeSnapshot(original);
        QCOMPARE(bytes.size(), 48);
        Snapshot decoded;
        QVERIFY(ProtocolMessages::decodeSnapshot(bytes, decoded));
        QCOMPARE(decoded.position, qint32(-51200));
        QCOMPARE(decoded.velocity, qint32(-25600));
        QCOMPARE(decoded.sampleCounter, quint32(17));
        QByteArray corrupt = bytes; corrupt[42] = 2;
        QVERIFY(!ProtocolMessages::decodeSnapshot(corrupt, decoded));
        QCOMPARE(decoded.position, qint32(-51200));
        QVERIFY(!ProtocolMessages::decodeSnapshot(bytes + 'x', decoded));
    }
    void infoLayoutAndCompatibility()
    {
        DeviceInfo info;
        info.uid = QByteArray::fromHex("00112233445566778899AABB");
        info.capabilities = 1; info.unitsPerRev = 51200;
        info.currentMaxMa = 1000; info.calibrationMaxMa = 2000;
        info.velocityMax = 1536000; info.accelerationMax = 1000000;
        const QByteArray bytes = ProtocolMessages::encodeInfo(info);
        QCOMPARE(bytes.size(), 50);
        DeviceInfo decoded;
        QVERIFY(ProtocolMessages::decodeInfo(bytes, decoded));
        QCOMPARE(decoded.uid, info.uid); QCOMPARE(decoded.unitsPerRev, quint32(51200));
        QByteArray future = bytes; future[0] = 2;
        QVERIFY(!ProtocolMessages::decodeInfo(future, decoded));
        QCOMPARE(decoded.uid, info.uid);
    }
    void independentGoldenFixtures()
    {
        QFile file(QCoreApplication::applicationDirPath() + "/../../../../docs/protocol/golden-frames.json");
        QVERIFY2(file.open(QIODevice::ReadOnly), qPrintable(file.errorString()));
        const QJsonArray vectors = QJsonDocument::fromJson(file.readAll()).object().value("vectors").toArray();
        QVERIFY(vectors.size() >= 6);
        for (const auto &value : vectors) {
            const auto object = value.toObject();
            Frame frame;
            frame.type = FrameType(object.value("type").toInt());
            frame.session = quint32(object.value("session").toDouble());
            frame.sequence = quint16(object.value("sequence").toInt());
            frame.command = quint16(object.value("command").toInt());
            frame.payload = QByteArray::fromHex(object.value("payload").toString().toLatin1());
            QCOMPARE(ProtocolCodec::encode(frame), QByteArray::fromHex(object.value("hex").toString().toLatin1()));
            ProtocolCodec codec;
            QCOMPARE(codec.feed(QByteArray::fromHex(object.value("hex").toString().toLatin1()), 0).size(), 1);
        }
    }
    void fullParameterResponseFitsFrame()
    {
        const QByteArray all = QByteArray::fromHex(
            "01000F00"
            "01000204E8030000" "0200020400701700" "0300020440420F00" "04000204D0070000"
            "0101010405000000" "020101041E000000" "0301010400000000"
            "01020104C8000000" "0202010450000000" "030201042C010000" "04020104FA000000"
            "0103030101" "0104020464000000" "0204020464000000" "0304020464000000");
        QCOMPARE(all.size(), 121);
        QCOMPARE(ProtocolMessages::validateParameters(all), Status::Ok);
        Frame response=request(Command::ReadParams); response.type=FrameType::Response;
        response.payload=QByteArray::fromHex("000001000000")+all;
        QCOMPARE(response.payload.size(), 127);
        Status status; QByteArray data;
        QVERIFY(ProtocolMessages::decodeResponse(response,status,data));
        QCOMPARE(ProtocolCodec::encode(response).size(), 143);
    }
    void handshakeErrorsBeforeSessionAssignment()
    {
        Frame response=request(Command::Hello); response.type=FrameType::Response;
        response.session=0; response.payload=QByteArray::fromHex("0300");
        Status status; QByteArray data;
        QVERIFY(ProtocolMessages::decodeResponse(response,status,data));
        response.payload=QByteArray::fromHex("00000100");
        QVERIFY(!ProtocolMessages::decodeResponse(response,status,data));
    }
    void unsolicitedDirectionAndPayloadRules()
    {
        Frame event=request(Command::TaskEvent, QByteArray::fromHex("0100000003000000D2040000"));
        event.type=FrameType::Event;
        QVERIFY(ProtocolMessages::validateUnsolicited(event));
        event.payload[4]=char(7);
        QVERIFY(!ProtocolMessages::validateUnsolicited(event));
        event=request(Command::FaultEvent,QByteArray::fromHex("010000000403D2040000")); event.type=FrameType::Event;
        QVERIFY(ProtocolMessages::validateUnsolicited(event));
        event.type=FrameType::Telemetry;
        QVERIFY(!ProtocolMessages::validateUnsolicited(event));
        event=request(Command::Telemetry,ProtocolMessages::encodeSnapshot(Snapshot())); event.type=FrameType::Telemetry;
        QVERIFY(ProtocolMessages::validateUnsolicited(event));
        event.session=0;
        QVERIFY(!ProtocolMessages::validateUnsolicited(event));
    }
    void taskQueryRecoversLostTerminalEvent()
    {
        QCOMPARE(ProtocolMessages::validateRequest(request(Command::TaskQuery,QByteArray::fromHex("01000000"))),Status::Ok);
        QCOMPARE(ProtocolMessages::validateRequest(request(Command::TaskQuery,QByteArray(4,0))),Status::OutOfRange);
        Frame response=request(Command::TaskQuery); response.type=FrameType::Response;
        response.payload=QByteArray::fromHex("00000100000001030000D2040000");
        Status status; QByteArray data;
        QVERIFY(ProtocolMessages::decodeResponse(response,status,data));
        response.payload[7]=char(7);
        QVERIFY(!ProtocolMessages::decodeResponse(response,status,data));
    }
};
int runMessageTests(int argc, char **argv) { MessageTest test; return QTest::qExec(&test, argc, argv); }
#include "tst_messages.moc"
