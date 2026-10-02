#include <QtTest>
#include "command_dispatcher.h"
#include "control_gate.h"
#include "protocol/ProtocolCodec.h"
#include "protocol/ProtocolMessages.h"
#include "device/SessionController.h"
#include <QElapsedTimer>
#include <QTimer>
class TestBackend : public stepper::CommandBackend {
public:
    stepper::ControlGate gate;
    int executions=0;
    bool checkWatchdog=true;
    bool oversizedReply=false;
    int forcedStatus=-1;
    TestBackend(){gate.setSensors(true,true);}
    uint32_t sessionId() const override {return gate.session();}
    void poll(uint32_t now) override {if(checkWatchdog)gate.tick(now);}
    uint16_t beginSession(uint32_t token,uint32_t now) override {return uint16_t(gate.hello(token,now));}
    uint16_t execute(const stepper::Frame &request,uint32_t now,stepper::Frame &response) override {
        executions++;
        if(oversizedReply){response.length=127;return 0;}
        if(forcedStatus>=0)return uint16_t(forcedStatus);
        switch(request.command) {
        case 2: {DeviceInfo info;info.uid=QByteArray(12,1);info.unitsPerRev=51200;info.currentMaxMa=1000;info.calibrationMaxMa=2000;info.velocityMax=1536000;info.accelerationMax=1000000;
            const auto bytes=ProtocolMessages::encodeInfo(info);response.length=uint16_t(bytes.size());memcpy(response.payload,bytes.constData(),size_t(bytes.size()));return 0;}
        case 4: {const auto status=gate.heartbeat(request.session,now);response.length=4;for(unsigned i=0;i<4;++i)response.payload[i]=uint8_t(now>>(8*i));return uint16_t(status);}
        case 0x0101:response.length=1;response.payload[0]=request.payload[0];return uint16_t(gate.enable(request.payload[0],now));
        case 0x0102:case 0x0106:gate.stop(); // Test fixture confirms a logical stop only.
            // fall through
        case 3: {Snapshot s;s.state=gate.enabled()?DeviceState::Ready:DeviceState::Disabled;s.mode=ControlMode(gate.mode());s.calibrated=true;s.encoderValid=true;
            const auto bytes=ProtocolMessages::encodeSnapshot(s);response.length=uint16_t(bytes.size());memcpy(response.payload,bytes.constData(),size_t(bytes.size()));return 0;}
        default:return 10;
        }
    }
};
class DispatcherTest : public QObject {
    Q_OBJECT
    stepper::Frame request(uint16_t command,uint16_t sequence,uint32_t session=7,QByteArray payload={}) {
        stepper::Frame frame={};frame.type=1;frame.command=command;frame.sequence=sequence;frame.session=session;frame.length=uint16_t(payload.size());memcpy(frame.payload,payload.constData(),size_t(payload.size()));return frame;
    }
    uint16_t status(const stepper::Frame &response){return uint16_t(response.payload[0])|(uint16_t(response.payload[1])<<8);}
    void hello(stepper::CommandDispatcher &d,uint32_t now=0,uint16_t seq=1) {stepper::Frame response;QVERIFY(d.handle(request(1,seq,0,QByteArray::fromHex("07000000")),now,response));QCOMPARE(status(response),uint16_t(0));}
private slots:
    void duplicatesReturnIdenticalResponseAndNeverExecuteTwice() {
        TestBackend b;stepper::CommandDispatcher d(b);hello(d);stepper::Frame first,second;
        const auto r=request(0x0101,2,7,QByteArray(1,1));d.handle(r,1,first);d.handle(r,2,second);
        QCOMPARE(b.executions,1);QCOMPARE(status(second),uint16_t(0));QCOMPARE(second.length,first.length);
        QCOMPARE(QByteArray(reinterpret_cast<const char*>(second.payload),second.length),QByteArray(reinterpret_cast<const char*>(first.payload),first.length));
        auto changed=r;changed.payload[0]=0;d.handle(changed,3,second);QCOMPARE(status(second),uint16_t(12));QCOMPARE(b.executions,1);
        d.handle(request(0x0106,3),4,second);QVERIFY(!b.gate.enabled());
        d.handle(r,5,second);QCOMPARE(status(second),uint16_t(0));QVERIFY(!b.gate.enabled());QCOMPARE(b.executions,2);
    }
    void cacheEvictionAndTtlPreserveWatermark() {
        TestBackend b;b.checkWatchdog=false;stepper::CommandDispatcher d(b);hello(d);stepper::Frame response;
        for(uint16_t seq=2;seq<=6;++seq)d.handle(request(3,seq),seq,response);
        d.handle(request(3,2),20,response);QCOMPARE(status(response),uint16_t(12));QCOMPARE(b.executions,5);
        d.handle(request(3,6),2006,response);QCOMPARE(status(response),uint16_t(12));QCOMPARE(b.executions,5);
        d.handle(request(3,7),2007,response);QCOMPARE(status(response),uint16_t(0));QCOMPARE(b.executions,6);
    }
    void sequenceAndTtlWrap() {
        TestBackend b;b.checkWatchdog=false;stepper::CommandDispatcher d(b);hello(d,0xfffffff0,65534);stepper::Frame response;
        d.handle(request(3,65535),0xfffffff1,response);d.handle(request(3,1),0x10,response);QCOMPARE(status(response),uint16_t(0));
        d.handle(request(3,1),0x20,response);QCOMPARE(b.executions,2);
        d.handle(request(3,1),0x7e0,response);QCOMPARE(status(response),uint16_t(12));
        d.handle(request(3,32769),0x7e1,response);QCOMPARE(status(response),uint16_t(12));
        d.handle(request(3,2),0x7e2,response);QCOMPARE(status(response),uint16_t(0));
    }
    void expiredSessionAndNewSessionRejectOldMotion() {
        TestBackend b;stepper::CommandDispatcher d(b);hello(d);stepper::Frame response;
        d.handle(request(0x0101,2,7,QByteArray(1,1)),1,response);b.gate.tick(500);
        d.handle(request(0x0101,3,7,QByteArray(1,1)),501,response);QCOMPARE(status(response),uint16_t(11));QCOMPARE(b.executions,1);
        auto h=request(1,1,0,QByteArray::fromHex("08000000"));d.handle(h,502,response);QCOMPARE(response.session,uint32_t(8));
        d.handle(request(0x0101,2,7,QByteArray(1,1)),503,response);QCOMPARE(status(response),uint16_t(11));QVERIFY(!b.gate.enabled());
    }
    void malformedRequestsNeverReachBackendAndConsumeSequence() {
        TestBackend b;stepper::CommandDispatcher d(b);hello(d);stepper::Frame response;
        d.handle(request(0x0101,2,7,QByteArray(1,3)),1,response);QCOMPARE(status(response),uint16_t(4));
        d.handle(request(0x0101,2,7,QByteArray(1,1)),2,response);QCOMPARE(status(response),uint16_t(12));
        d.handle(request(3,3,7,QByteArray(1,0)),3,response);QCOMPARE(status(response),uint16_t(3));
        d.handle(request(0xaaaa,4),4,response);QCOMPARE(status(response),uint16_t(2));QCOMPARE(b.executions,0);
        auto r=request(3,5);r.type=4;QVERIFY(!d.handle(r,5,response));r.type=1;r.sequence=0;QVERIFY(!d.handle(r,6,response));
    }
    void helloReplayDoesNotResetWatermarkOrRenewHeartbeat() {
        TestBackend b;stepper::CommandDispatcher d(b);hello(d);stepper::Frame response;
        d.handle(request(0x0101,2,7,QByteArray(1,1)),1,response);
        d.handle(request(1,1,0,QByteArray::fromHex("08000000")),2,response);QCOMPARE(status(response),uint16_t(5));
        d.handle(request(1,1,0,QByteArray::fromHex("07000000")),499,response);QCOMPARE(status(response),uint16_t(0));
        b.gate.tick(500);QVERIFY(!b.gate.enabled());
    }
    void malformedHelloCannotChangeAProcessedSequence() {
        TestBackend b;stepper::CommandDispatcher d(b);hello(d);stepper::Frame response;
        d.handle(request(1,1,0,QByteArray::fromHex("0700000000")),1,response);QCOMPARE(status(response),uint16_t(12));
        d.handle(request(1,2,0,QByteArray::fromHex("0700000000")),2,response);QCOMPARE(status(response),uint16_t(3));
        d.handle(request(1,2,0,QByteArray::fromHex("07000000")),3,response);QCOMPARE(status(response),uint16_t(12));
        d.handle(request(1,2,0,QByteArray::fromHex("0700000000")),4,response);QCOMPARE(status(response),uint16_t(3));
    }
    void invalidBackendReplyBecomesCachedInternalError() {
        TestBackend b;stepper::CommandDispatcher d(b);hello(d);stepper::Frame response;
        b.oversizedReply=true;d.handle(request(3,2),1,response);QCOMPARE(status(response),uint16_t(13));QCOMPARE(response.length,uint16_t(2));
        d.handle(request(3,2),2,response);QCOMPARE(b.executions,1);
        b.oversizedReply=false;b.forcedStatus=14;d.handle(request(3,3),3,response);QCOMPARE(status(response),uint16_t(13));
    }
    void responseWireLayoutsMatchQtDecoder() {
        TestBackend b;stepper::CommandDispatcher d(b);stepper::Frame response;
        const QVector<stepper::Frame> requests={request(1,1,0,QByteArray::fromHex("07000000")),request(2,2),request(3,3),request(4,4),request(0x0101,5,7,QByteArray(1,1)),request(0x0106,6),request(0x0201,7)};
        for(const auto &r:requests){QVERIFY(d.handle(r,r.sequence,response));uint8_t bytes[144];const auto count=stepper::encode(response,bytes,sizeof(bytes));
            ProtocolCodec codec;const auto frames=codec.feed(QByteArray(reinterpret_cast<const char*>(bytes),int(count)),0);QCOMPARE(frames.size(),1);
            Status s;QByteArray data;QVERIFY(ProtocolMessages::decodeResponse(frames[0],s,data));}
    }
    void allRequestLengthsMatchQtValidation() {
        TestBackend b;stepper::CommandDispatcher d(b);hello(d);stepper::Frame response;uint16_t seq=2;
        const QVector<uint16_t> commands={2,3,4,5,0x0101,0x0102,0x0103,0x0104,0x0105,0x0106,0x0107,0x0108,0x0201,0x0202,0x0203,0x0301,0x0302,0x0303,0x0401,0xaaaa};
        for(uint16_t command:commands)for(int length=0;length<=128;++length) {
            const QByteArray payload(length,0);Frame qt;qt.session=7;qt.sequence=seq;qt.command=command;qt.payload=payload;
            const auto expected=ProtocolMessages::validateRequest(qt);d.handle(request(command,seq++,7,payload),0,response);
            if(expected!=Status::Ok)QCOMPARE(status(response),uint16_t(expected));
            else QVERIFY(status(response)!=2&&status(response)!=3&&status(response)!=4);
        }
    }
    void parameterBatchesMatchQtValidation() {
        TestBackend b;stepper::CommandDispatcher d(b);hello(d);stepper::Frame response;uint16_t seq=2;
        const QList<QByteArray> payloads={QByteArray::fromHex("010001000101010420000000"),QByteArray::fromHex("0100020001010104200000000101010421000000"),
            QByteArray::fromHex("0100010001010104ffffffff"),QByteArray::fromHex("010001000103030102"),QByteArray::fromHex("010001000104020400c80000"),
            QByteArray::fromHex("0100010003040204d0070000"),QByteArray::fromHex("0100010003040204d1070000")};
        for(const auto &payload:payloads) {
            Frame qt;qt.session=7;qt.sequence=seq;qt.command=0x0202;qt.payload=payload;const auto expected=ProtocolMessages::validateRequest(qt);
            d.handle(request(0x0202,seq++,7,payload),0,response);
            QCOMPARE(status(response),expected==Status::Ok?uint16_t(10):uint16_t(expected));
        }
    }
    void qtSessionHandshakesWithPortableDispatcher() {
        class WireTransport : public ITransport {
        public:
            TestBackend backend;stepper::CommandDispatcher dispatcher;
            stepper::Parser parser;QElapsedTimer clock;bool connected=false;uint32_t generation=0;
            WireTransport():dispatcher(backend) {}
            void open(const QString &) override {connected=true;clock.start();emit opened();}
            void close() override {generation++;if(connected){connected=false;emit closed();}}
            bool isOpen() const override {return connected;}
            static void receive(void *context,const stepper::Frame &request) {
                auto self=static_cast<WireTransport*>(context);stepper::Frame response;
                if(!self->dispatcher.handle(request,uint32_t(self->clock.elapsed()),response))return;
                uint8_t bytes[144];const auto count=stepper::encode(response,bytes,sizeof(bytes));
                const QByteArray packet(reinterpret_cast<const char*>(bytes),int(count));const auto epoch=self->generation;
                QTimer::singleShot(0,self,[self,packet,epoch]{if(self->connected&&self->generation==epoch)emit self->bytesReceived(packet);});
            }
            bool send(const QByteArray &bytes) override {
                if(!connected)return false;
                emit frameQueued(bytes);
                parser.feed(reinterpret_cast<const uint8_t*>(bytes.constData()),size_t(bytes.size()),uint32_t(clock.elapsed()),receive,this);return true;
            }
        } transport;
        SessionController session(&transport);QSignalSpy responses(&session,&SessionController::responseReceived);
        session.connectDevice("portable",7);QTRY_COMPARE(session.state(),ConnectionState::Ready);
        QTest::qWait(350);QCOMPARE(session.state(),ConnectionState::Ready);
        QVERIFY(session.submit(Command::Stop)!=0);QTRY_COMPARE(responses.size(),1);
        QCOMPARE(qvariant_cast<Response>(responses[0][0]).status,Status::Ok);
        session.disconnectDevice();QVERIFY(!transport.isOpen());
    }
};
int runDispatcherTests(int argc,char **argv){DispatcherTest test;return QTest::qExec(&test,argc,argv);}
#include "tst_dispatcher.moc"
