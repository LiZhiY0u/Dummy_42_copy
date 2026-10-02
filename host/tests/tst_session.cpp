#include <QtTest>
#include <QSignalSpy>
#include "device/SessionController.h"
#include "device/DeviceStateModel.h"
#include "transport/MockTransport.h"
#include "transport/SerialTransport.h"
#include "protocol/ProtocolCodec.h"
#include "protocol/ProtocolMessages.h"

class SessionTest : public QObject
{
    Q_OBJECT
    QVector<Frame> sent(const QSignalSpy &spy, Command command) const {
        QVector<Frame> result;
        for (const auto &args : spy) {
            ProtocolCodec codec;
            for (const auto &frame : codec.feed(args[0].toByteArray(),0))
                if (frame.command==quint16(command)) result.append(frame);
        }
        return result;
    }
private slots:
    void lostHelloReplyRetriesIdenticalRequestOnce() {
        MockTransport transport;
        transport.setDropResponses(Command::Hello,true);
        SessionController session(&transport);
        QSignalSpy output(&transport,&ITransport::frameQueued);
        QSignalSpy errors(&session,&SessionController::errorOccurred);
        connect(&transport,&ITransport::frameQueued,&transport,[&transport](const QByteArray &bytes){
            ProtocolCodec codec;
            const auto frames=codec.feed(bytes,0);
            if(!frames.isEmpty() && frames.first().command==quint16(Command::Hello))
                QTimer::singleShot(0,&transport,[&transport]{transport.setDropResponses(Command::Hello,false);});
        });
        session.connectDevice("mock",0x12345678);
        QTRY_COMPARE_WITH_TIMEOUT(session.state(),ConnectionState::Ready,1500);
        const auto hellos=sent(output,Command::Hello);
        QCOMPARE(hellos.size(),2);
        QCOMPARE(ProtocolCodec::encode(hellos[0]),ProtocolCodec::encode(hellos[1]));
        QCOMPARE(hellos[0].session,quint32(0));
        QCOMPARE(hellos[0].sequence,quint16(1));
        QVERIFY(sent(output,Command::Enable).isEmpty());
        QCOMPARE(errors.size(),0);
    }
    void helloRetryIsBoundedWhenAllRepliesAreLost() {
        MockTransport transport;
        transport.setDropResponses(Command::Hello,true);
        SessionController session(&transport);
        QSignalSpy output(&transport,&ITransport::frameQueued);
        session.connectDevice("mock",7);
        QTRY_COMPARE_WITH_TIMEOUT(session.state(),ConnectionState::Error,1500);
        const auto hellos=sent(output,Command::Hello);
        QCOMPARE(hellos.size(),2);
        QCOMPARE(ProtocolCodec::encode(hellos[0]),ProtocolCodec::encode(hellos[1]));
    }
    void realSerialReadyTelemetryStopAndReconnect() {
        const QString endpoint=qEnvironmentVariable("STEPPER_TEST_PORT");
        if(endpoint.isEmpty()) QSKIP("Opt-in physical test: set STEPPER_TEST_PORT, motor remains disabled");
        SerialTransport transport;
        SessionController session(&transport);
        QSignalSpy errors(&session,&SessionController::errorOccurred);
        QSignalSpy infos(&session,&SessionController::infoReceived);
        QSignalSpy snapshots(&session,&SessionController::snapshotReceived);
        QSignalSpy responses(&session,&SessionController::responseReceived);
        QSignalSpy output(&transport,&ITransport::frameQueued);
        for(unsigned round=0;round<2;++round) {
            snapshots.clear();infos.clear();responses.clear();output.clear();
            session.connectDevice(endpoint,0x34567810+round);
            QTRY_COMPARE_WITH_TIMEOUT(session.state(),ConnectionState::Ready,2000);
            QCOMPARE(infos.size(),1);
            QCOMPARE(qvariant_cast<DeviceInfo>(infos.first()[0]).uid.size(),12);
            QTest::qWait(5000);
            QCOMPARE(session.state(),ConnectionState::Ready);
            QVERIFY(snapshots.size()>=200);
            quint32 previous=0;
            for(const auto &args:snapshots) {
                const Snapshot s=qvariant_cast<Snapshot>(args[0]);
                QVERIFY(s.sampleCounter>previous);
                previous=s.sampleCounter;
                QCOMPARE(s.mode,ControlMode::None);
                QCOMPARE(s.currentCommandMa,qint32(0));
                QCOMPARE(s.txDropCount,quint16(0));
            }
            QVERIFY(session.submit(Command::Stop)!=0);
            QTRY_COMPARE(responses.size(),1);
            QCOMPARE(qvariant_cast<Response>(responses.first()[0]).status,Status::Ok);
            QVERIFY(sent(output,Command::Enable).isEmpty());
            QVERIFY(sent(output,Command::MoveAbsolute).isEmpty());
            QVERIFY(sent(output,Command::SetVelocity).isEmpty());
            qInfo("PHYSICAL round=%u snapshots=%d heartbeatRequests=%d stop=confirmed",
                  round+1,snapshots.size(),sent(output,Command::Heartbeat).size());
            session.disconnectDevice();
            QTest::qWait(600); // Let the board watchdog expire before reconnecting.
        }
        QCOMPARE(errors.size(),0);
    }
    void helloTimeoutIdentifiesNoReplyVersusLoopback() {
        for (bool loopback : {false, true}) {
            MockTransport transport;
            transport.setDropResponses(Command::Hello,true);
            SessionController session(&transport);
            QSignalSpy errors(&session,&SessionController::errorOccurred);
            QSignalSpy output(&transport,&ITransport::frameQueued);
            session.connectDevice("mock",7);
            const QByteArray hello=output.first()[0].toByteArray();
            QCOMPARE(hello.size(),20);
            if(loopback) emit transport.bytesReceived(hello);
            QTRY_COMPARE(session.state(),ConnectionState::Error);
            QCOMPARE(errors.size(),1);
            const QString message=errors.first()[0].toString();
            QVERIFY(message.contains("cmd=0x0001"));
            QVERIFY(message.contains(loopback?"RX=20":"RX=0"));
            QVERIFY(message.contains(loopback?"frames=1":"frames=0"));
        }
    }
    void mockHandshakeRemainsDisabled() {
        MockTransport transport;
        SessionController session(&transport);
        QSignalSpy info(&session,&SessionController::infoReceived);
        QSignalSpy snapshots(&session,&SessionController::snapshotReceived);
        session.connectDevice("mock",0x11223344);
        QTRY_COMPARE(session.state(),ConnectionState::Ready);
        QCOMPARE(info.size(),1);
        QTRY_VERIFY(snapshots.size()>1);
        const Snapshot value=qvariant_cast<Snapshot>(snapshots.last()[0]);
        QCOMPARE(value.state,DeviceState::Disabled);
        QCOMPARE(session.sessionId(),quint32(0x11223344));
        session.disconnectDevice();
        QCOMPARE(session.state(),ConnectionState::Disconnected);
        QCOMPARE(session.pendingCount(),0);
    }
    void heartbeatRunsDuringSlowHandshake() {
        MockTransport transport; transport.setResponseDelay(Command::GetInfo,300);
        SessionController session(&transport);
        QSignalSpy output(&transport,&ITransport::frameQueued);
        session.connectDevice("mock",7);
        QTRY_COMPARE(session.state(),ConnectionState::Ready);
        QVERIFY(sent(output,Command::Heartbeat).size()>=2);
        session.disconnectDevice();
    }
    void queryRetriesOnceWithSameSequence() {
        MockTransport transport; SessionController session(&transport);
        session.connectDevice("mock",7); QTRY_COMPARE(session.state(),ConnectionState::Ready);
        transport.setDropResponses(Command::GetStatus,true);
        QSignalSpy output(&transport,&ITransport::frameQueued);
        QSignalSpy unknown(&session,&SessionController::requestTimedOut);
        session.submit(Command::GetStatus);
        QTRY_COMPARE(unknown.size(),1);
        const auto frames=sent(output,Command::GetStatus);
        QCOMPARE(frames.size(),2); QCOMPARE(frames[0].sequence,frames[1].sequence);
        QTest::qWait(80); QCOMPARE(sent(output,Command::GetStatus).size(),2);
    }
    void motionIsNeverAutomaticallyRetried() {
        MockTransport transport; SessionController session(&transport);
        session.connectDevice("mock",7); QTRY_COMPARE(session.state(),ConnectionState::Ready);
        transport.setDropResponses(Command::SetVelocity,true);
        QSignalSpy output(&transport,&ITransport::frameQueued);
        QSignalSpy unknown(&session,&SessionController::requestTimedOut);
        QVERIFY(session.submit(Command::SetVelocity,QByteArray::fromHex("0038FFFF00900100"))!=0);
        QTRY_COMPARE(unknown.size(),1);
        QCOMPARE(sent(output,Command::SetVelocity).size(),1);
    }
    void staleSessionAndWrongCommandCannotCompleteRequest() {
        MockTransport transport; SessionController session(&transport);
        session.connectDevice("mock",7); QTRY_COMPARE(session.state(),ConnectionState::Ready);
        transport.setDropResponses(Command::GetStatus,true);
        QSignalSpy output(&transport,&ITransport::frameQueued);
        QSignalSpy responses(&session,&SessionController::responseReceived);
        session.submit(Command::GetStatus);
        Frame response=sent(output,Command::GetStatus).first(); response.type=FrameType::Response;
        response.payload=QByteArray::fromHex("0000")+ProtocolMessages::encodeSnapshot(Snapshot());
        response.session=6; emit transport.bytesReceived(ProtocolCodec::encode(response));
        QCOMPARE(responses.size(),0);
        response.session=7; response.command=quint16(Command::Disable);
        emit transport.bytesReceived(ProtocolCodec::encode(response)); QCOMPARE(responses.size(),0);
        response.command=quint16(Command::GetStatus);
        emit transport.bytesReceived(ProtocolCodec::encode(response)); QCOMPARE(responses.size(),1);
    }
    void stopClearsQueuedMotionAndBypassesQuery() {
        MockTransport transport; SessionController session(&transport);
        session.connectDevice("mock",7); QTRY_COMPARE(session.state(),ConnectionState::Ready);
        transport.setDropResponses(Command::GetStatus,true);
        QSignalSpy output(&transport,&ITransport::frameQueued);
        session.submit(Command::GetStatus);
        QVERIFY(session.submit(Command::SetVelocity,QByteArray::fromHex("0038FFFF00900100"))!=0);
        QVERIFY(session.submit(Command::Stop)!=0);
        QCOMPARE(sent(output,Command::Stop).size(),1);
        QCOMPARE(sent(output,Command::SetVelocity).size(),0);
        QTest::qWait(450);
        QCOMPARE(sent(output,Command::SetVelocity).size(),0);
    }
    void queueIsBoundedAndSequenceAssignedWhenSent() {
        MockTransport transport; SessionController session(&transport);
        session.connectDevice("mock",7); QTRY_COMPARE(session.state(),ConnectionState::Ready);
        transport.setResponseDelay(Command::GetStatus,300);
        QSignalSpy output(&transport,&ITransport::frameQueued);
        session.submit(Command::GetStatus);
        for(int i=0;i<16;++i) QVERIFY(session.submit(Command::GetStatus)!=0);
        QCOMPARE(session.submit(Command::GetStatus),quint64(0));
        QTRY_VERIFY(sent(output,Command::GetStatus).size()>=3);
        const auto queries=sent(output,Command::GetStatus);
        QVERIFY(queries[2].sequence>queries[0].sequence);
        session.disconnectDevice();
        QCOMPARE(session.pendingCount(),0);
    }
    void rebootStopsSessionAndClearsQueue() {
        MockTransport transport; SessionController session(&transport);
        session.connectDevice("mock",7); QTRY_COMPARE(session.state(),ConnectionState::Ready);
        transport.setDropResponses(Command::GetStatus,true);
        session.submit(Command::GetStatus);
        session.submit(Command::SetVelocity,QByteArray::fromHex("0038FFFF00900100"));
        transport.reboot();
        QTRY_COMPARE(session.state(),ConnectionState::Error);
        QCOMPARE(session.pendingCount(),0); QCOMPARE(session.sessionId(),quint32(0));
    }
    void rejectsMotionBeforeReadyAndUnknownCapabilities() {
        MockTransport transport; SessionController session(&transport);
        QCOMPARE(session.submit(Command::Enable,QByteArray(1,1)),quint64(0));
        session.connectDevice("mock",7); QTRY_COMPARE(session.state(),ConnectionState::Ready);
        QCOMPARE(session.submit(Command::SetCurrent,QByteArray(4,0)),quint64(0));
    }
    void invalidPortReportsErrorWithoutOpeningAnything() {
        SerialTransport transport; QSignalSpy errors(&transport,&ITransport::transportError);
        transport.open("__STEPPER_INVALID_PORT__");
        QVERIFY(!errors.isEmpty()); QVERIFY(!transport.isOpen());
        transport.close();
    }
    void lostHeartbeatClosesSessionWithoutResumingMotion() {
        MockTransport transport; SessionController session(&transport);
        session.connectDevice("mock",7); QTRY_COMPARE(session.state(),ConnectionState::Ready);
        transport.setDropResponses(Command::Heartbeat,true);
        QTRY_COMPARE(session.state(),ConnectionState::Error);
        QVERIFY(!transport.isOpen()); QCOMPARE(session.sessionId(),quint32(0));
        QCOMPARE(session.submit(Command::SetVelocity,QByteArray::fromHex("0038FFFF00900100")),quint64(0));
    }
    void modelMarksOldSamplesStale() {
        DeviceStateModel model; model.updateState(ConnectionState::Ready);
        Snapshot newer;newer.sampleCounter=1;model.updateSnapshot(newer); QVERIFY(model.fresh);
        model.updateSnapshot(Snapshot());QCOMPARE(model.snapshot.sampleCounter,quint32(1));
        QTRY_VERIFY(!model.fresh);
        newer.sampleCounter=2;model.updateSnapshot(newer); QVERIFY(model.fresh);
        model.updateState(ConnectionState::Disconnected); QVERIFY(!model.fresh);
    }
    void disableAlsoCancelsMotionAndBlocksEnableUntilConfirmed() {
        MockTransport transport;SessionController session(&transport);
        session.connectDevice("mock",7);QTRY_COMPARE(session.state(),ConnectionState::Ready);
        transport.setDropResponses(Command::GetStatus,true);transport.setResponseDelay(Command::Disable,100);
        QSignalSpy output(&transport,&ITransport::frameQueued);
        session.submit(Command::GetStatus);session.submit(Command::Enable,QByteArray(1,1));
        QVERIFY(session.submit(Command::Disable)!=0);
        QCOMPARE(session.submit(Command::Enable,QByteArray(1,1)),quint64(0));
        QTest::qWait(250);QCOMPARE(sent(output,Command::Disable).size(),1);QCOMPARE(sent(output,Command::Enable).size(),0);
    }
    void taskResponseMustMatchRequestedId() {
        MockTransport transport;SessionController session(&transport);
        session.connectDevice("mock",7);QTRY_COMPARE(session.state(),ConnectionState::Ready);
        transport.setDropResponses(Command::TaskQuery,true);
        QSignalSpy output(&transport,&ITransport::frameQueued);QSignalSpy responses(&session,&SessionController::responseReceived);
        session.submit(Command::TaskQuery,QByteArray::fromHex("01000000"));
        Frame f=sent(output,Command::TaskQuery).first();f.type=FrameType::Response;
        f.payload=QByteArray::fromHex("0000020000000103000000000000");
        emit transport.bytesReceived(ProtocolCodec::encode(f));QCOMPARE(responses.size(),0);
        f.payload[2]=1;emit transport.bytesReceived(ProtocolCodec::encode(f));QCOMPARE(responses.size(),1);
    }
    void mockHelloPreservesWatermarkAndRejectsActiveTakeover() {
        MockTransport transport;transport.open("mock");QSignalSpy input(&transport,&ITransport::bytesReceived);
        auto exchange=[&](Command c,quint16 seq,quint32 session,const QByteArray &payload){
            input.clear();Frame request;request.command=quint16(c);request.sequence=seq;request.session=session;request.payload=payload;
            transport.send(ProtocolCodec::encode(request));QTest::qWait(5);ProtocolCodec codec;QVector<Frame> frames;
            for(const auto &args:input)frames+=codec.feed(args[0].toByteArray(),0);
            return frames.isEmpty()?Frame():frames.first();
        };
        QCOMPARE(exchange(Command::Hello,1,0,QByteArray::fromHex("07000000")).session,quint32(7));
        exchange(Command::GetStatus,100,7,{});
        exchange(Command::Hello,1,0,QByteArray::fromHex("07000000"));
        QCOMPARE(exchange(Command::GetStatus,2,7,{}).payload.left(2),QByteArray::fromHex("0c00"));
        exchange(Command::Enable,101,7,QByteArray(1,1));
        QCOMPARE(exchange(Command::Hello,1,0,QByteArray::fromHex("08000000")).payload.left(2),QByteArray::fromHex("0500"));
        const Frame f=exchange(Command::GetStatus,102,7,{});Snapshot s;
        QVERIFY(ProtocolMessages::decodeSnapshot(f.payload.mid(2),s));QCOMPARE(s.state,DeviceState::Ready);
    }
    void mockParameterWriteIsAtomicAndReadable() {
        MockTransport transport;SessionController session(&transport);
        session.connectDevice("mock",7);QTRY_COMPARE(session.state(),ConnectionState::Ready);
        QSignalSpy responses(&session,&SessionController::responseReceived);
        session.submit(Command::WriteParams,QByteArray::fromHex("010001000101010429000000"));
        QTRY_COMPARE(responses.size(),1);QCOMPARE(qvariant_cast<Response>(responses[0][0]).status,Status::Ok);
        session.submit(Command::ReadParams);QTRY_COMPARE(responses.size(),2);
        const auto response=qvariant_cast<Response>(responses[1][0]);
        QVERIFY(response.data.endsWith(QByteArray::fromHex("010001000101010429000000")));
        session.submit(Command::WriteParams,QByteArray::fromHex("0100010001000204e9030000"));
        QTRY_COMPARE(responses.size(),3);QCOMPARE(qvariant_cast<Response>(responses[2][0]).status,Status::OutOfRange);
        session.submit(Command::WriteParams,QByteArray::fromHex("0100010003000204a0860100"));
        QTRY_COMPARE(responses.size(),4);QCOMPARE(qvariant_cast<Response>(responses[3][0]).status,Status::Ok);
    }
    void snapshotsAdvanceWithoutTelemetryAndStopIsVisible() {
        MockTransport transport;SessionController session(&transport);DeviceStateModel model;
        connect(&session,&SessionController::stateChanged,&model,&DeviceStateModel::updateState);
        connect(&session,&SessionController::snapshotReceived,&model,&DeviceStateModel::updateSnapshot);
        session.connectDevice("mock",7);QTRY_COMPARE(session.state(),ConnectionState::Ready);
        QSignalSpy responses(&session,&SessionController::responseReceived);
        session.submit(Command::TelemetryConfig,QByteArray(2,0));QTRY_COMPARE(responses.size(),1);
        session.submit(Command::Enable,QByteArray(1,1));QTRY_COMPARE(responses.size(),2);
        session.submit(Command::GetStatus);QTRY_COMPARE(model.snapshot.state,DeviceState::Ready);
        session.submit(Command::Stop);QTRY_COMPARE(model.snapshot.state,DeviceState::Disabled);QVERIFY(model.fresh);
    }
    void splitBytesAndCloseCancelDelayedReplies() {
        MockTransport transport; transport.setChunkSize(1);
        SessionController session(&transport);
        session.connectDevice("mock",7); QTRY_COMPARE(session.state(),ConnectionState::Ready);
        transport.setResponseDelay(Command::GetStatus,250);
        QSignalSpy responses(&session,&SessionController::responseReceived);
        session.submit(Command::GetStatus); session.disconnectDevice();
        QTest::qWait(300); QCOMPARE(responses.size(),0);
    }
};
int runSessionTests(int argc,char **argv) { SessionTest test; return QTest::qExec(&test,argc,argv); }
#include "tst_session.moc"
