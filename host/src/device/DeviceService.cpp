#include "DeviceService.h"
#include "transport/MockTransport.h"
#include "transport/SerialTransport.h"
void DeviceService::connectEndpoint(QString endpoint) {
    shutdown();
    transport=endpoint=="mock"?static_cast<ITransport*>(new MockTransport(this)):static_cast<ITransport*>(new SerialTransport(this));
    session=new SessionController(transport,this);
    // Log only bootstrap traffic; continuous telemetry must not flood the UI.
    connect(transport,&ITransport::frameQueued,this,[this](const QByteArray &bytes){
        if(session && session->state()==ConnectionState::Handshaking)
            emit logMessage(QStringLiteral("TX排队 [%1B] %2").arg(bytes.size()).arg(QString::fromLatin1(bytes.toHex(' ').toUpper())));
    });
    connect(transport,&ITransport::bytesReceived,this,[this](const QByteArray &bytes){
        if(session && session->state()!=ConnectionState::Ready && !bytes.isEmpty())
            emit logMessage(QStringLiteral("RX [%1B] %2").arg(bytes.size()).arg(QString::fromLatin1(bytes.left(144).toHex(' ').toUpper())));
    });
    connect(session,&SessionController::stateChanged,this,&DeviceService::stateChanged);
    connect(session,&SessionController::infoReceived,this,&DeviceService::infoReceived);
    connect(session,&SessionController::snapshotReceived,this,&DeviceService::snapshotReceived);
    connect(session,&SessionController::errorOccurred,this,&DeviceService::logMessage);
    connect(session,&SessionController::responseReceived,this,[this](const Response &r){
        const QString text=r.status==Status::Ok?QStringLiteral("设备确认停止：驱动已禁用"):QStringLiteral("设备拒绝请求：状态码%1").arg(quint16(r.status));
        emit transactionMessage(text);emit logMessage(text);
    });
    connect(session,&SessionController::requestTimedOut,this,[this](quint64 id){
        const QString text=QStringLiteral("请求%1超时，结果未知；请检查设备确认状态").arg(id);emit transactionMessage(text);emit logMessage(text);
    });
    emit transactionMessage(QStringLiteral("等待设备确认"));
    emit logMessage(endpoint=="mock"?QStringLiteral("连接模拟设备；模拟数据不代表实际电机测量"):QStringLiteral("打开串口%1，115200/8N1，协议V1").arg(endpoint));
    session->connectDevice(endpoint);
}
void DeviceService::shutdown() {
    if(session){session->disconnectDevice();delete session;session=nullptr;}
    delete transport;transport=nullptr;
    emit stateChanged(ConnectionState::Disconnected);
    emit transactionMessage(QStringLiteral("连接已关闭；未确认硬件停机"));
}
void DeviceService::stop() {
    if(!session||!session->submit(Command::Stop))emit transactionMessage(QStringLiteral("当前状态不能发送STOP"));
    else emit transactionMessage(QStringLiteral("STOP已发送，等待设备确认"));
}
