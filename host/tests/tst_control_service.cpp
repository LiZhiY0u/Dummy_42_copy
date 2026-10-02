#include <QtTest>
#include "control_service.h"
class TestCritical : public stepper::CriticalSection {
public:
    int depth=0,enters=0;
    uint32_t enter() override {enters++;return uint32_t(depth++);}
    void leave(uint32_t saved) override {depth=int(saved);}
};
class TestDriver : public stepper::ControlDriver {
public:
    int stops=0,enables=0;bool output=false,failEnable=false;uint8_t supported=7,mode=3;int32_t positionTarget=0;
    uint8_t supportedModes() const override {return supported;}
    void disableAndReset() override {stops++;output=false;mode=3;positionTarget=0;}
    bool enable(uint8_t requested,int32_t position) override {enables++;if(failEnable)return false;output=true;mode=requested;positionTarget=position;return true;}
};
class ControlServiceTest : public QObject {
    Q_OBJECT
    stepper::ControlCommand command(stepper::ControlKind kind,uint32_t id,uint32_t session=7,uint8_t mode=1) {
        stepper::ControlCommand c={};c.kind=kind;c.requestId=id;c.session=session;c.mode=mode;return c;
    }
    stepper::Measurement sample(int32_t position=1234) {stepper::Measurement m={};m.position=position;m.encoderValid=true;m.calibrated=true;return m;}
private slots:
    void ackWaitsForControlBoundaryAndUnreadResultsAreNotOverwritten() {
        TestCritical critical;stepper::ControlMailbox mailbox(critical);TestDriver driver;stepper::ControlService service(mailbox,critical,driver);
        QVERIFY(mailbox.submit(command(stepper::ControlKind::Hello,1)));stepper::ControlCompletion result;
        QVERIFY(!mailbox.takeCompletion(result));QVERIFY(!driver.output);
        service.tick(0,sample());QVERIFY(mailbox.takeCompletion(result));QCOMPARE(result.status,uint16_t(0));
        QVERIFY(mailbox.submit(command(stepper::ControlKind::Enable,2)));service.tick(1,sample());QVERIFY(driver.output);
        QVERIFY(!mailbox.submit(command(stepper::ControlKind::ReadState,3)));QVERIFY(mailbox.takeCompletion(result));
        QCOMPARE(result.snapshot.state,uint8_t(1));QCOMPARE(driver.positionTarget,int32_t(0));QCOMPARE(critical.depth,0);
    }
    void stopPreemptsAndCancelsPendingEnable() {
        TestCritical critical;stepper::ControlMailbox mailbox(critical);TestDriver driver;stepper::ControlService service(mailbox,critical,driver);
        mailbox.submit(command(stepper::ControlKind::Hello,1));service.tick(0,sample());stepper::ControlCompletion result;mailbox.takeCompletion(result);
        mailbox.submit(command(stepper::ControlKind::Enable,2));mailbox.submit(command(stepper::ControlKind::Stop,3));service.tick(1,sample());
        QVERIFY(!driver.output);QCOMPARE(driver.enables,0);
        QVERIFY(mailbox.takeCompletion(result));QCOMPARE(result.requestId,uint32_t(3));QCOMPARE(result.status,uint16_t(0));
        QVERIFY(mailbox.takeCompletion(result));QCOMPARE(result.requestId,uint32_t(2));QVERIFY(result.canceled);QCOMPARE(result.status,uint16_t(5));
    }
    void wrongSessionStopDoesNotCancelValidEnable() {
        TestCritical critical;stepper::ControlMailbox mailbox(critical);TestDriver driver;stepper::ControlService service(mailbox,critical,driver);
        mailbox.submit(command(stepper::ControlKind::Hello,1));service.tick(0,sample());stepper::ControlCompletion r;mailbox.takeCompletion(r);
        mailbox.submit(command(stepper::ControlKind::Enable,2));mailbox.submit(command(stepper::ControlKind::Stop,3,8));service.tick(1,sample());
        mailbox.takeCompletion(r);QCOMPARE(r.status,uint16_t(11));QVERIFY(driver.output);mailbox.takeCompletion(r);QCOMPARE(r.status,uint16_t(0));
    }
    void watchdogAndEncoderFaultCallDriverStopWithoutAnyRequest() {
        TestCritical critical;stepper::ControlMailbox mailbox(critical);TestDriver driver;stepper::ControlService service(mailbox,critical,driver);
        mailbox.submit(command(stepper::ControlKind::Hello,1));service.tick(0,sample());stepper::ControlCompletion r;mailbox.takeCompletion(r);
        mailbox.submit(command(stepper::ControlKind::Enable,2));service.tick(1,sample());mailbox.takeCompletion(r);
        service.tick(500,sample());QVERIFY(!driver.output);auto s=service.snapshot();QVERIFY(s.faultBits&4);QCOMPARE(s.session,uint32_t(0));
        mailbox.submit(command(stepper::ControlKind::Hello,3,8));service.tick(501,sample());mailbox.takeCompletion(r);
        mailbox.submit(command(stepper::ControlKind::ClearFault,4,8));service.tick(502,sample());mailbox.takeCompletion(r);QVERIFY(!driver.output);
        mailbox.submit(command(stepper::ControlKind::Enable,5,8));service.tick(503,sample());mailbox.takeCompletion(r);QVERIFY(driver.output);
        auto invalid=sample();invalid.encoderValid=false;service.tick(504,invalid);QVERIFY(!driver.output);QVERIFY(service.snapshot().faultBits&1);
    }
    void positionEnableLatchesMeasuredPositionAndNeverRelatchesWhileEnabled() {
        TestCritical critical;stepper::ControlMailbox mailbox(critical);TestDriver driver;stepper::ControlService service(mailbox,critical,driver);
        mailbox.submit(command(stepper::ControlKind::Hello,1));service.tick(0,sample());stepper::ControlCompletion r;mailbox.takeCompletion(r);
        mailbox.submit(command(stepper::ControlKind::Enable,2,7,0));service.tick(1,sample(-1234));mailbox.takeCompletion(r);QCOMPARE(driver.positionTarget,int32_t(-1234));
        mailbox.submit(command(stepper::ControlKind::Enable,3,7,0));service.tick(2,sample(1234));mailbox.takeCompletion(r);QCOMPARE(driver.enables,1);QCOMPARE(driver.positionTarget,int32_t(-1234));
        QCOMPARE(service.snapshot().positionError,int32_t(-2468));
        service.tick(3,sample(INT32_MAX));QCOMPARE(service.snapshot().positionError,int32_t(INT32_MIN));
    }
    void unsupportedModeAndDriverFailureRemainDisabled() {
        TestCritical critical;stepper::ControlMailbox mailbox(critical);TestDriver driver;stepper::ControlService service(mailbox,critical,driver);
        mailbox.submit(command(stepper::ControlKind::Hello,1));service.tick(0,sample());stepper::ControlCompletion r;mailbox.takeCompletion(r);
        driver.supported=2;mailbox.submit(command(stepper::ControlKind::Enable,2,7,0));service.tick(1,sample());mailbox.takeCompletion(r);QCOMPARE(r.status,uint16_t(10));
        driver.failEnable=true;mailbox.submit(command(stepper::ControlKind::Enable,3));service.tick(2,sample());mailbox.takeCompletion(r);QCOMPARE(r.status,uint16_t(13));QVERIFY(!driver.output);QCOMPARE(service.snapshot().mode,uint8_t(3));
    }
    void threeLanesAreBoundedAndCompletionPriorityIsStopHeartbeatOrdinary() {
        TestCritical critical;stepper::ControlMailbox mailbox(critical);TestDriver driver;stepper::ControlService service(mailbox,critical,driver);
        mailbox.submit(command(stepper::ControlKind::Hello,1));service.tick(0,sample());stepper::ControlCompletion r;mailbox.takeCompletion(r);
        QVERIFY(mailbox.submit(command(stepper::ControlKind::ReadState,2)));QVERIFY(mailbox.submit(command(stepper::ControlKind::Heartbeat,3)));QVERIFY(mailbox.submit(command(stepper::ControlKind::Stop,4)));
        QVERIFY(!mailbox.submit(command(stepper::ControlKind::Heartbeat,5)));service.tick(1,sample());
        mailbox.takeCompletion(r);QCOMPARE(r.requestId,uint32_t(4));mailbox.takeCompletion(r);QCOMPARE(r.requestId,uint32_t(3));mailbox.takeCompletion(r);QCOMPARE(r.requestId,uint32_t(2));QVERIFY(!mailbox.takeCompletion(r));QCOMPARE(critical.depth,0);
    }
    void stopInFlightBlocksNewEnableUntilItsCompletionIsRead() {
        TestCritical critical;stepper::ControlMailbox mailbox(critical);TestDriver driver;stepper::ControlService service(mailbox,critical,driver);
        mailbox.submit(command(stepper::ControlKind::Hello,1));service.tick(0,sample());stepper::ControlCompletion r;mailbox.takeCompletion(r);
        QVERIFY(mailbox.submit(command(stepper::ControlKind::Stop,2)));QVERIFY(!mailbox.submit(command(stepper::ControlKind::Enable,3)));
        service.tick(1,sample());QVERIFY(!mailbox.submit(command(stepper::ControlKind::Enable,3)));
        mailbox.takeCompletion(r);QVERIFY(mailbox.submit(command(stepper::ControlKind::Enable,3)));service.tick(2,sample());QVERIFY(driver.output);
    }
    void correlationIdsAreUniqueAcrossAllOccupiedLanes() {
        TestCritical critical;stepper::ControlMailbox mailbox(critical);
        QVERIFY(!mailbox.submit(command(stepper::ControlKind::Enable,0)));
        QVERIFY(mailbox.submit(command(stepper::ControlKind::ReadState,1)));
        QVERIFY(!mailbox.submit(command(stepper::ControlKind::Heartbeat,1)));
        QVERIFY(!mailbox.submit(command(stepper::ControlKind::Stop,1)));
        QCOMPARE(critical.depth,0);
    }
    void nestedCriticalStateAndDisabledSnapshotArePreserved() {
        TestCritical critical;stepper::ControlMailbox mailbox(critical);TestDriver driver;stepper::ControlService service(mailbox,critical,driver);
        const auto saved=critical.enter();auto measurement=sample();measurement.currentCommandMa=900;
        service.tick(0,measurement);const auto state=service.snapshot();QCOMPARE(critical.depth,1);
        QCOMPARE(state.currentCommandMa,int32_t(0));QCOMPARE(state.targetCurrentMa,int32_t(0));QCOMPARE(state.targetVelocity,int32_t(0));
        critical.leave(saved);QCOMPARE(critical.depth,0);
    }
};
int runControlServiceTests(int argc,char **argv){ControlServiceTest test;return QTest::qExec(&test,argc,argv);}
#include "tst_control_service.moc"
