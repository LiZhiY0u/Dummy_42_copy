#include "SerialTransport.h"
SerialTransport::SerialTransport(QObject *parent):ITransport(parent),port(this) {
    port.setReadBufferSize(4096);
    connect(&port,&QSerialPort::readyRead,this,[this]{emit bytesReceived(port.readAll());});
    connect(&port,&QSerialPort::bytesWritten,this,[this](qint64){pump();});
    connect(&port,&QSerialPort::errorOccurred,this,[this](QSerialPort::SerialPortError error){
        if(error!=QSerialPort::NoError) emit transportError(port.errorString());
    });
}
void SerialTransport::open(const QString &endpoint) {
    close(); port.setPortName(endpoint);
    port.setBaudRate(115200); port.setDataBits(QSerialPort::Data8);
    port.setParity(QSerialPort::NoParity); port.setStopBits(QSerialPort::OneStop);
    port.setFlowControl(QSerialPort::NoFlowControl);
    if(port.open(QIODevice::ReadWrite)) emit opened();
}
void SerialTransport::close() {
    outgoing.clear();
    if(port.isOpen()) {port.close();emit closed();}
}
bool SerialTransport::send(const QByteArray &bytes) {
    if(!isOpen()) return false;
    if(outgoing.size()+port.bytesToWrite()+bytes.size()>432) {
        emit transportError(QStringLiteral("串口发送积压超过432字节")); return false;
    }
    outgoing.append(bytes);emit frameQueued(bytes);pump();return true;
}
void SerialTransport::pump() {
    if(outgoing.isEmpty() || !isOpen()) return;
    const qint64 count=port.write(outgoing);
    if(count>0) outgoing.remove(0,int(count));
}
