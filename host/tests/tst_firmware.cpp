#include <QtTest>
#include <QFile>
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include "protocol/ProtocolCodec.h"
#include "protocol/ProtocolMessages.h"
#include "protocol_v1.h"
#include "control_gate.h"
class FirmwareTest : public QObject {
    Q_OBJECT
    static void collect(void *context,const stepper::Frame &frame) {static_cast<QVector<stepper::Frame>*>(context)->append(frame);}
    QByteArray request(quint16 command=3) {Frame f;f.session=7;f.sequence=2;f.command=command;return ProtocolCodec::encode(f);}
private slots:
    void independentGoldenVectors() {
        QFile file(QCoreApplication::applicationDirPath()+"/../../../../docs/protocol/golden-frames.json");QVERIFY(file.open(QIODevice::ReadOnly));
        const auto root=QJsonDocument::fromJson(file.readAll()).object();
        const auto vectors=root.value("vectors").toArray();QVERIFY(vectors.size()>=8);
        for(const auto &item:vectors) {
            const auto bytes=QByteArray::fromHex(item.toObject().value("hex").toString().toLatin1());
            QVector<stepper::Frame> frames;stepper::Parser parser;
            parser.feed(reinterpret_cast<const uint8_t*>(bytes.constData()),size_t(bytes.size()),0,collect,&frames);
            QCOMPARE(frames.size(),1);uint8_t encoded[144];
            const size_t size=stepper::encode(frames[0],encoded,sizeof(encoded));
            QCOMPARE(QByteArray(reinterpret_cast<const char*>(encoded),int(size)),bytes);
        }
    }
    void everySplitAndBytewiseInput() {
        const auto bytes=request();
        for(int split=0;split<=bytes.size();++split) {
            QVector<stepper::Frame> frames;stepper::Parser parser;
            parser.feed(reinterpret_cast<const uint8_t*>(bytes.constData()),split,0,collect,&frames);
            parser.feed(reinterpret_cast<const uint8_t*>(bytes.constData()+split),size_t(bytes.size()-split),1,collect,&frames);
            QCOMPARE(frames.size(),1);QCOMPARE(frames[0].session,uint32_t(7));
        }
        QVector<stepper::Frame> frames;stepper::Parser parser;
        for(int i=0;i<bytes.size();++i)parser.feed(reinterpret_cast<const uint8_t*>(bytes.constData()+i),1,uint32_t(i),collect,&frames);
        QCOMPARE(frames.size(),1);
    }
    void corruptNoiseAndOversizeRecoverWithinFixedStorage() {
        auto bad=request();bad[14]=char(bad[14]^1);
        QByteArray bytes(10000,char(0xAA));bytes+=bad+QByteArray::fromHex("aa5501018100")+request()+request();
        QVector<stepper::Frame> frames;stepper::Parser parser;
        parser.feed(reinterpret_cast<const uint8_t*>(bytes.constData()),size_t(bytes.size()),10,collect,&frames);
        QCOMPARE(frames.size(),2);QVERIFY(parser.buffered()<=144);
        QVERIFY(sizeof(stepper::Parser)<=168);
    }
    void partialExpiryAndClockWrap() {
        QVector<stepper::Frame> frames;stepper::Parser parser;const auto bytes=request();
        parser.feed(reinterpret_cast<const uint8_t*>(bytes.constData()),10,0xfffffff0,collect,&frames);
        parser.feed(nullptr,0,0x54,collect,&frames);QCOMPARE(parser.buffered(),size_t(0));
        parser.feed(reinterpret_cast<const uint8_t*>(bytes.constData()),size_t(bytes.size()),0x55,collect,&frames);QCOMPARE(frames.size(),1);
    }
    void maximumPayloadEmbeddedHeaderAndCapacity() {
        stepper::Frame frame={};frame.type=1;frame.session=7;frame.sequence=3;frame.command=0x0202;frame.length=128;
        for(unsigned i=0;i<128;++i)frame.payload[i]=uint8_t(i);
        frame.payload[50]=0xaa;frame.payload[51]=0x55;
        uint8_t bytes[144];QCOMPARE(stepper::encode(frame,bytes,sizeof(bytes)),size_t(144));
        QVector<stepper::Frame> frames;stepper::Parser parser;parser.feed(bytes,sizeof(bytes),0,collect,&frames);
        QCOMPARE(frames.size(),1);QCOMPARE(frames[0].length,uint16_t(128));
        QCOMPARE(QByteArray(reinterpret_cast<const char*>(frames[0].payload),128),QByteArray(reinterpret_cast<const char*>(frame.payload),128));
        uint8_t small[16];memset(small,0xcd,sizeof(small));QCOMPARE(stepper::encode(frame,small,sizeof(small)),size_t(0));
        QCOMPARE(small[0],uint8_t(0xcd));frame.length=129;QCOMPARE(stepper::encode(frame,bytes,sizeof(bytes)),size_t(0));
    }
    void rejectedVersionTypeAndNullInput() {
        auto version=request();version[2]=2;auto type=request();type[3]=0;
        auto updateCrc=[](QByteArray &bytes){const auto crc=ProtocolCodec::crc16(bytes.mid(2,bytes.size()-4));bytes[bytes.size()-2]=char(crc);bytes[bytes.size()-1]=char(crc>>8);};
        updateCrc(version);updateCrc(type); // Valid CRC isolates version/type rejection.
        const auto bytes=version+type+request();QVector<stepper::Frame> frames;stepper::Parser parser;
        parser.feed(nullptr,10,0,collect,&frames);QCOMPARE(parser.buffered(),size_t(0));
        parser.feed(reinterpret_cast<const uint8_t*>(bytes.constData()),size_t(bytes.size()),1,collect,&frames);QCOMPARE(frames.size(),1);
        const uint8_t reference[]={ '1','2','3','4','5','6','7','8','9' };QCOMPARE(stepper::crc16(reference,9),uint16_t(0x29b1));
    }
    void switchingModeRequiresStop() {
        stepper::ControlGate gate;gate.setSensors(true,true);gate.hello(7,0);gate.enable(1,1);
        QCOMPARE(gate.enable(0,2),stepper::Result::WrongState);QCOMPARE(gate.mode(),uint8_t(1));
        gate.stop();QCOMPARE(gate.enable(0,3),stepper::Result::Ok);
        gate.setSensors(true,false);QVERIFY(!gate.enabled());QCOMPARE(gate.enable(0,4),stepper::Result::NotCalibrated);
    }
    void encoderAndCalibrationGateEnable() {
        stepper::ControlGate gate;QVERIFY(!gate.enabled());
        QCOMPARE(gate.enable(1,0),stepper::Result::BadSession);
        QCOMPARE(gate.hello(7,0),stepper::Result::Ok);
        QCOMPARE(gate.enable(1,0),stepper::Result::EncoderFault);
        gate.setSensors(true,false);gate.clearFault();
        QCOMPARE(gate.enable(1,0),stepper::Result::NotCalibrated);
        gate.setSensors(true,true);QCOMPARE(gate.enable(1,1),stepper::Result::Ok);QVERIFY(gate.enabled());
    }
    void heartbeatExpiryAndFaultRecoveryNeverEnable() {
        stepper::ControlGate gate;gate.setSensors(true,true);gate.hello(7,0);gate.enable(1,1);
        gate.tick(499);QVERIFY(gate.enabled());gate.tick(500);QVERIFY(!gate.enabled());QCOMPARE(gate.session(),uint32_t(0));
        QVERIFY(gate.faults()&4);gate.hello(8,501);QCOMPARE(gate.enable(1,501),stepper::Result::WrongState);
        QCOMPARE(gate.clearFault(),stepper::Result::Ok);QVERIFY(!gate.enabled());
        gate.enable(1,502);gate.setSensors(false,true);QVERIFY(!gate.enabled());QVERIFY(gate.faults()&1);
        QCOMPARE(gate.clearFault(),stepper::Result::EncoderFault);
    }
    void oldHeartbeatsAndRunningTakeoverAreRejected() {
        stepper::ControlGate gate;gate.setSensors(true,true);gate.hello(7,0);gate.enable(1,1);
        QCOMPARE(gate.hello(8,100),stepper::Result::WrongState);
        QCOMPARE(gate.heartbeat(8,499),stepper::Result::BadSession);
        QCOMPARE(gate.hello(7,499),stepper::Result::Ok);gate.tick(500);QVERIFY(!gate.enabled());
        gate.clearFault();gate.hello(8,0xfffffff0);gate.enable(1,0xfffffff0);gate.heartbeat(8,0x100);
        gate.tick(0x2f3);QVERIFY(gate.enabled());gate.tick(0x2f4);QVERIFY(!gate.enabled());
    }
    void stopAndOwnershipAreExplicit() {
        stepper::ControlGate gate;gate.setSensors(true,true);QVERIFY(gate.canAcceptCanMotion());gate.hello(7,0);
        QVERIFY(!gate.canAcceptCanMotion());gate.enable(1,1);gate.stop();QVERIFY(!gate.enabled());
        QCOMPARE(gate.mode(),uint8_t(3));QCOMPARE(gate.session(),uint32_t(7));
        QCOMPARE(gate.enable(3,2),stepper::Result::OutOfRange);QVERIFY(!gate.enabled());
    }
};
int runFirmwareTests(int argc,char **argv) {FirmwareTest test;return QTest::qExec(&test,argc,argv);}
#include "tst_firmware.moc"
