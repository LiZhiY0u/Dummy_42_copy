#include "MainWindow.h"
#include "device/DeviceService.h"
#include "device/DeviceStateModel.h"
#include <QLabel>
#include <QStatusBar>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QComboBox>
#include <QCheckBox>
#include <QPushButton>
#include <QPlainTextEdit>
#include <QSerialPortInfo>

MainWindow::MainWindow(QWidget *parent):QMainWindow(parent),service(new DeviceService) {
    qRegisterMetaType<ConnectionState>();qRegisterMetaType<Snapshot>();qRegisterMetaType<DeviceInfo>();qRegisterMetaType<Response>();
    setWindowTitle(QStringLiteral("步进电机调试上位机"));resize(900,620);
    auto body=new QWidget(this);auto layout=new QVBoxLayout(body);
    auto title=new QLabel(QStringLiteral("步进电机调试上位机"),body);auto font=title->font();font.setPointSize(20);title->setFont(font);layout->addWidget(title);
    auto environment=new QLabel(QStringLiteral("Qt %1 · MinGW %2.%3.%4 · %5 位").arg(qVersion()).arg(__GNUC__).arg(__GNUC_MINOR__).arg(__GNUC_PATCHLEVEL__).arg(sizeof(void*)*8),body);
    environment->setObjectName("environmentInfo");layout->addWidget(environment);
    auto row=new QHBoxLayout;auto mode=new QComboBox(body);mode->addItems({QStringLiteral("模拟设备"),QStringLiteral("串口设备（协议V1）")});row->addWidget(mode);
    auto port=new QComboBox(body);row->addWidget(port);auto refresh=new QPushButton(QStringLiteral("刷新串口"),body);row->addWidget(refresh);
    auto connectButton=new QPushButton(QStringLiteral("连接"),body);connectButton->setObjectName("connectButton");row->addWidget(connectButton);
    auto stopButton=new QPushButton(QStringLiteral("停止并禁用"),body);stopButton->setObjectName("stopButton");stopButton->setEnabled(false);row->addWidget(stopButton);layout->addLayout(row);
    auto revision=new QCheckBox(QStringLiteral("已确认下位机刷入协议V1固件（原P9旧固件不能直接连接）"),body);layout->addWidget(revision);
    auto status=new QLabel(QStringLiteral("未连接"),body);status->setObjectName("connectionStatus");layout->addWidget(status);
    auto info=new QLabel(QStringLiteral("设备信息：等待握手"),body);layout->addWidget(info);
    auto snapshot=new QLabel(QStringLiteral("设备确认状态：暂无数据"),body);snapshot->setObjectName("snapshotInfo");snapshot->setWordWrap(true);layout->addWidget(snapshot);
    auto transaction=new QLabel(QStringLiteral("暂无请求"),body);transaction->setObjectName("transactionInfo");layout->addWidget(transaction);
    auto log=new QPlainTextEdit(body);log->setReadOnly(true);log->setMaximumBlockCount(200);layout->addWidget(log);
    layout->addWidget(new QLabel(QStringLiteral("P3通讯验证：模拟遥测50Hz；运动控制、参数和真实固件联调待后续阶段。"),body));setCentralWidget(body);
    auto model=new DeviceStateModel(this);
    auto enumerate=[port]{port->clear();for(const auto &item:QSerialPortInfo::availablePorts())port->addItem(item.portName());};enumerate();
    connect(refresh,&QPushButton::clicked,this,enumerate);
    auto controls=[=]{const bool serial=mode->currentIndex()==1;const bool idle=model->state==ConnectionState::Disconnected||model->state==ConnectionState::Error;
        port->setEnabled(serial&&idle);refresh->setEnabled(serial&&idle);revision->setEnabled(serial&&idle);mode->setEnabled(idle);
        connectButton->setEnabled(!idle||!serial||(revision->isChecked()&&!port->currentText().isEmpty()));};
    connect(mode,QOverload<int>::of(&QComboBox::currentIndexChanged),this,[controls](int){controls();});
    connect(port,QOverload<int>::of(&QComboBox::currentIndexChanged),this,[controls](int){controls();});
    connect(revision,&QCheckBox::toggled,this,[controls](bool){controls();});controls();
    connect(connectButton,&QPushButton::clicked,this,[=]{
        if(model->state!=ConnectionState::Disconnected&&model->state!=ConnectionState::Error){emit disconnectRequested();return;}
        if(mode->currentIndex()==1&&(!revision->isChecked()||port->currentText().isEmpty()))return;
        connectButton->setEnabled(false);emit connectRequested(mode->currentIndex()==0?QStringLiteral("mock"):port->currentText());
    });
    connect(stopButton,&QPushButton::clicked,this,&MainWindow::stopRequested);
    connect(model,&DeviceStateModel::changed,this,[=]{
        const QStringList states={QStringLiteral("未连接"),QStringLiteral("正在打开连接"),QStringLiteral("正在握手"),QStringLiteral("已就绪"),QStringLiteral("连接异常")};
        status->setText(states[int(model->state)]);connectButton->setText(model->state==ConnectionState::Disconnected||model->state==ConnectionState::Error?QStringLiteral("连接"):QStringLiteral("断开"));controls();
        stopButton->setEnabled(model->state==ConnectionState::Ready);
        if(!model->info.uid.isEmpty()) info->setText(QStringLiteral("UID %1 · 固件 %2.%3.%4 · %5单位/圈 · 能力0x%6").arg(QString::fromLatin1(model->info.uid.toHex())).arg(model->info.firmwareMajor).arg(model->info.firmwareMinor).arg(model->info.firmwarePatch).arg(model->info.unitsPerRev).arg(model->info.capabilities,0,16));
        else info->setText(QStringLiteral("设备信息：等待握手"));
        const QStringList deviceStates={QStringLiteral("禁用"),QStringLiteral("使能待命"),QStringLiteral("运行"),QStringLiteral("校准"),QStringLiteral("故障")};
        if(!model->info.uid.isEmpty()) snapshot->setText(QStringLiteral("设备确认：%1 · %2\n位置 %3 · 速度 %4 · 指令电流 %5 mA\n采样 %6 · 设备时间 %7 ms · 故障位0x%8")
            .arg(deviceStates[int(model->snapshot.state)]).arg(model->fresh?QStringLiteral("数据有效"):QStringLiteral("数据陈旧/未就绪"))
            .arg(model->snapshot.position).arg(model->snapshot.velocity).arg(model->snapshot.currentCommandMa).arg(model->snapshot.sampleCounter).arg(model->snapshot.timestampMs).arg(model->snapshot.faultBits,0,16));
        else snapshot->setText(QStringLiteral("设备确认状态：暂无数据"));
    });
    service->moveToThread(&worker);
    connect(&worker,&QThread::finished,service,&QObject::deleteLater);
    connect(this,&MainWindow::connectRequested,service,&DeviceService::connectEndpoint);
    connect(this,&MainWindow::disconnectRequested,service,&DeviceService::shutdown);
    connect(this,&MainWindow::stopRequested,service,&DeviceService::stop);
    connect(service,&DeviceService::stateChanged,model,&DeviceStateModel::updateState);
    connect(service,&DeviceService::infoReceived,model,&DeviceStateModel::updateInfo);
    connect(service,&DeviceService::snapshotReceived,model,&DeviceStateModel::updateSnapshot);
    connect(service,&DeviceService::logMessage,log,&QPlainTextEdit::appendPlainText);
    connect(service,&DeviceService::transactionMessage,transaction,&QLabel::setText);
    worker.start();statusBar()->showMessage(QStringLiteral("连接不会自动使能；停止结果以设备确认应答为准。"));
}
MainWindow::~MainWindow() {
    if(worker.isRunning()){QMetaObject::invokeMethod(service,"shutdown",Qt::BlockingQueuedConnection);worker.quit();worker.wait();}
}
