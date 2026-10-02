#include "DeviceStateModel.h"
DeviceStateModel::DeviceStateModel(QObject *parent):QObject(parent),timer(this) {
    timer.setInterval(100);connect(&timer,&QTimer::timeout,this,[this]{if(fresh&&age.elapsed()>=500){fresh=false;emit changed();}});timer.start();
}
void DeviceStateModel::updateState(ConnectionState value) {
    state=value;if(value!=ConnectionState::Ready){fresh=false;if(value==ConnectionState::Opening){info=DeviceInfo();snapshot=Snapshot();age.invalidate();}}emit changed();
}
void DeviceStateModel::updateInfo(DeviceInfo value) {info=value;emit changed();}
void DeviceStateModel::updateSnapshot(Snapshot value) {
    if(age.isValid()) {
        const qint32 sampleDelta=qint32(value.sampleCounter-snapshot.sampleCounter);
        const qint32 timeDelta=qint32(value.timestampMs-snapshot.timestampMs);
        if(sampleDelta<0 || (sampleDelta==0 && timeDelta<=0))return;
    }
    snapshot=value;age.restart();fresh=state==ConnectionState::Ready;emit changed();
}
