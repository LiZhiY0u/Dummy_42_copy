#include <QtTest>
#include <QLabel>
#include <QSerialPort>
#include <QPushButton>
#include "ui/MainWindow.h"

class EnvironmentTest : public QObject
{
    Q_OBJECT
private slots:
    void serialModuleIsAvailable()
    {
        QSerialPort port;
        QVERIFY(!port.isOpen());
        QVERIFY(port.setBaudRate(115200));
        QCOMPARE(port.baudRate(), 115200);
    }
    void startsDisconnected()
    {
        MainWindow window;
        QLabel *status = window.findChild<QLabel *>("connectionStatus");
        QVERIFY(status);
        QCOMPARE(status->text(), QString::fromUtf8("未连接"));
        QVERIFY(window.findChild<QLabel *>("environmentInfo"));
        window.show();
        QTest::qWait(50);
        QVERIFY(window.isVisible());
        const QString screenshot = qEnvironmentVariable("STEPPER_TEST_SCREENSHOT");
        if (!screenshot.isEmpty()) QVERIFY(window.grab().save(screenshot));
        QVERIFY(window.close());
    }
    void mockConnectionUpdatesUiAndReconnects()
    {
        MainWindow window;
        auto button=window.findChild<QPushButton *>("connectButton");
        QVERIFY(button);
        button->click();
        auto status=window.findChild<QLabel *>("connectionStatus");
        QTRY_COMPARE(status->text(),QString::fromUtf8("已就绪"));
        auto snapshot=window.findChild<QLabel *>("snapshotInfo");
        QTRY_VERIFY(snapshot->text().contains(QString::fromUtf8("禁用")));
        const QString initial=snapshot->text();
        QTRY_VERIFY(snapshot->text()!=initial);
        window.findChild<QPushButton *>("stopButton")->click();
        QTRY_VERIFY(window.findChild<QLabel *>("transactionInfo")->text().contains(QString::fromUtf8("确认")));
        button->click();QTRY_COMPARE(status->text(),QString::fromUtf8("未连接"));
        button->click();QTRY_COMPARE(status->text(),QString::fromUtf8("已就绪"));
        const QString screenshot=qEnvironmentVariable("STEPPER_TEST_SCREENSHOT");
        if(!screenshot.isEmpty()){window.show();QTest::qWait(80);QVERIFY(window.grab().save(screenshot));}
        // Destructor shuts the active worker down while it still owns its timers.
    }
};

int runProtocolTests(int argc, char **argv);
int runMessageTests(int argc, char **argv);
int runSessionTests(int argc, char **argv);
int runFirmwareTests(int argc, char **argv);
int runDispatcherTests(int argc, char **argv);
int runControlServiceTests(int argc, char **argv);
int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    EnvironmentTest environment;
    const int environmentResult = QTest::qExec(&environment, argc, argv);
    return environmentResult | runProtocolTests(argc, argv) | runMessageTests(argc, argv) | runSessionTests(argc, argv) | runFirmwareTests(argc, argv) | runDispatcherTests(argc, argv) | runControlServiceTests(argc, argv);
}
#include "tst_environment.moc"
