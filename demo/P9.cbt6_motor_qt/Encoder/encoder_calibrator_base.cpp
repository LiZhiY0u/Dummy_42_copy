
#include <valarray>
#include "encoder_calibrator_base.h"
#include "stockpile_f103cb.h"
#include "mt6816_base.h"
#include "tb67h450_base.h"
#include "motor.h"
#include "calibration_record.h"
#include "calibration_lifecycle.h"

namespace {
volatile uint32_t storageTask=0, storageProgress=0;
volatile stepper::calibration::Outcome storageOutcome=stepper::calibration::Outcome::Unknown;
volatile bool storageWorking=false, storageCancel=false;
bool calibrationWriteOk = false;
uint32_t expectedCalibrationCrc = 0xffffffffU;
uint32_t calibrationCrcWord(uint32_t crc,uint16_t value) {
    for(unsigned byte=0;byte<2;++byte) {
        crc ^= uint8_t(value>>(8*byte));
        for(unsigned bit=0;bit<8;++bit)
            crc=(crc>>1)^(0xedb88320U & (0U-(crc&1U)));
    }
    return crc;
}
bool calibrationWriting(const EncoderCalibratorBase &cal) {
    const volatile EncoderCalibratorBase &live=cal;
    return live.isTriggered && live.state==EncoderCalibratorBase::CALI_CALCULATING;
}
struct CalibrationWriter {
    uint8_t phase=0; // 0 idle, 1 erase, 2 write, 3 verify, 4 invalidate, 5 header, 6 commit
    uint32_t page=0, verified=0, actualCrc=0xffffffffU;
    int32_t x=0,y=0,limit=0,delta=0;
    bool stepReady=false;
} calibrationWriter;
Stockpile_FLASH_Typedef calibrationMetadataArea() {
    Stockpile_FLASH_Typedef area=stockpile_quick_cali;
    area.begin_add=uintptr_t(P9_CALIBRATION_RECORD_ADDRESS);
    area.area_size=Stockpile_Page_Size;area.page_num=1;area.asce_write_add=0;
    return area;
}
bool writeMetadataWord(EncoderCalibratorBase &cal,Stockpile_FLASH_Typedef &area,uint16_t value) {
    if(!calibrationWriting(cal))return false;
    const uintptr_t address=uintptr_t(area.asce_write_add);
    if(address<area.begin_add || address+2>area.begin_add+area.area_size)return false;
    Stockpile_Flash_Data_Write_Data16(&area,&value,1);
    return uintptr_t(area.asce_write_add)==address+2 &&
        *reinterpret_cast<volatile const uint16_t *>(address)==value;
}
void finishCalibrationStorage(EncoderCalibratorBase &cal,bool valid) {
    const uint32_t saved=__get_PRIMASK();
    __disable_irq();
    // A full verified physical commit linearizes before a later cancellation.
    const bool durable=calibrationWriter.phase==6 &&
        cal.resultNum==uint32_t(mt6816_base.RESOLUTION) &&
        calibrationWriter.actualCrc==expectedCalibrationCrc &&
        stepper::calibration::committed(stepper::calibration::stored()) &&
        stepper::calibration::stored().crc==~expectedCalibrationCrc;
    const bool accepted=(valid && calibrationWriting(cal)) || durable;
    const bool cancelled=storageCancel || !calibrationWriting(cal);
    mt6816_base.angleData.rectifyValid=accepted;
    storageOutcome=accepted ? stepper::calibration::Outcome::Verified :
        cancelled ? stepper::calibration::Outcome::Cancelled : stepper::calibration::Outcome::Failed;
    storageProgress=accepted ? (5U<<8)|100U : (6U<<8)|(storageProgress&255U);
    storageWorking=false;
    cal.state=EncoderCalibratorBase::CALI_DISABLE;
    cal.isTriggered=false;
    calibrationWriter.phase=0;
    __set_PRIMASK(saved);
}
bool nextCalibrationValue(EncoderCalibratorBase &cal,uint16_t &value) {
    CalibrationWriter &w=calibrationWriter;
    // At most 201 source intervals exist, even when an endpoint is empty.
    for(unsigned skipped=0;skipped<202;++skipped) {
        if(cal.goDirection ? w.x>cal.rcdX+200 : w.x<cal.rcdX) return false;
        if(!w.stepReady) {
            const int32_t a=int32_t(cal.sampleDataAverageForward[cal.CycleMod(w.x,200)]);
            const int32_t b=int32_t(cal.sampleDataAverageForward[cal.CycleMod(w.x+1,200)]);
            w.delta=cal.goDirection ? cal.CycleSubtract(b,a,mt6816_base.RESOLUTION)
                                    : cal.CycleSubtract(a,b,mt6816_base.RESOLUTION);
            const bool first=cal.goDirection ? w.x==cal.rcdX : w.x==cal.rcdX+200;
            const bool last=cal.goDirection ? w.x==cal.rcdX+200 : w.x==cal.rcdX;
            w.y=first?cal.rcdY:0;w.limit=last?cal.rcdY:w.delta;
            if(w.delta<=0 || w.y<0 || w.y>w.limit || w.limit>w.delta) {
                calibrationWriteOk=false;return false;
            }
            w.stepReady=true;
        }
        if(w.y<w.limit) {
            value=uint16_t(cal.CycleMod(cal.goDirection ? 256*w.x+256*w.y/w.delta
                : 256*(w.x+1)-256*w.y/w.delta,51200));
            ++w.y;return true;
        }
        w.stepReady=false;w.x+=cal.goDirection?1:-1;
    }
    calibrationWriteOk=false;return false;
}
}

bool stepper::calibration::beginTask(uint32_t task) {
    if(!task || storageWorking || calibrationWriter.phase ||
        (storageTask && storageOutcome==Outcome::Pending))return false;
    storageTask=task;storageOutcome=Outcome::Pending;storageProgress=1U<<8;
    storageCancel=false;return true;
}
void stepper::calibration::requestCancel(uint32_t task) {
    if(task!=storageTask || storageOutcome!=Outcome::Pending)return;
    storageCancel=true;
    if(!storageWorking && !calibrationWriter.phase) {
        storageOutcome=Outcome::Cancelled;storageProgress=(6U<<8)|(storageProgress&255U);
    }
}
stepper::calibration::Outcome stepper::calibration::outcome(uint32_t task) {
    return task && task==storageTask ? storageOutcome : Outcome::Unknown;
}
uint16_t stepper::calibration::phasePercent(uint32_t task) {
    return task && task==storageTask ? uint16_t(storageProgress) : 0;
}
bool stepper::calibration::storageBusy() {return storageWorking || calibrationWriter.phase;}

void EncoderCalibratorBase::BeginWriteFlash()
{
    calibrationWriteOk = true;
    expectedCalibrationCrc = 0xffffffffU;
    Stockpile_Flash_Data_Begin(&stockpile_quick_cali);
}

void EncoderCalibratorBase::EndWriteFlash()
{
    Stockpile_Flash_Data_End(&stockpile_quick_cali);
}

void EncoderCalibratorBase::ClearFlash()
{
    Stockpile_Flash_Data_Empty(&stockpile_quick_cali);
}

// 记录编码器从0开始的对应细分数
void EncoderCalibratorBase::WriteFlash16bitsAppend(uint16_t _data)
{
    const uintptr_t address = uintptr_t(stockpile_quick_cali.begin_add) + resultNum * 2U;
    if (!calibrationWriting(*this) || !calibrationWriteOk || resultNum >= uint32_t(mt6816_base.RESOLUTION) ||
        _data >= 51200U || uintptr_t(stockpile_quick_cali.asce_write_add) != address)
    {
        calibrationWriteOk = false;
        return;
    }
    Stockpile_Flash_Data_Write_Data16(&stockpile_quick_cali, &_data, 1);
    if (uintptr_t(stockpile_quick_cali.asce_write_add) != address + 2U ||
        *reinterpret_cast<volatile const uint16_t *>(address) != _data)
        calibrationWriteOk = false;
    else
        expectedCalibrationCrc=calibrationCrcWord(expectedCalibrationCrc,_data);
}

// 获取样本数据循环平均值
int32_t EncoderCalibratorBase::CycleDataAverage(const uint16_t *_data, uint16_t _length, int32_t _cyc)
{
    int32_t sumData = 0;
    int32_t subData;
    int32_t diffData;

    sumData += (int32_t)_data[0];
    for (uint16_t i = 1; i < _length; i++)
    {
        diffData = (int32_t)_data[i];
        subData = (int32_t)_data[i] - (int32_t)_data[0]; // mt6816样本数据总差
        if (subData > (_cyc >> 1))
            diffData = (int32_t)_data[i] - _cyc;
        if (subData < (-_cyc >> 1))
            diffData = (int32_t)_data[i] + _cyc;
        sumData += diffData;
    }

    sumData = sumData / _length;

    if (sumData < 0)
        sumData += _cyc;
    if (sumData > _cyc)
        sumData -= _cyc;

    return sumData;
}

// 进行步距检测，通过后得到零点前的最后一步步数及零点后第一步与零点的编码器距离
void EncoderCalibratorBase::CalibrationDataCheck()
{
    uint32_t count;
    int32_t subData;

    int32_t calibSampleResolution = mt6816_base.RESOLUTION / MOTOR_ONE_CIRCLE_HARD_STEPS; // 81
    for (count = 0; count < MOTOR_ONE_CIRCLE_HARD_STEPS + 1; count++)                     // 由正转值和反转值再求平均值，存放在正转数组里
    {
        sampleDataAverageForward[count] = (uint16_t)CycleAverage((int32_t)sampleDataAverageForward[count],
                                                                 (int32_t)sampleDataAverageBackward[count],
                                                                 mt6816_base.RESOLUTION);
    }
    subData = CycleSubtract((int32_t)sampleDataAverageForward[0],
                            (int32_t)sampleDataAverageForward[MOTOR_ONE_CIRCLE_HARD_STEPS - 1],
                            mt6816_base.RESOLUTION); // 由第一次和最后一次的数据的差值得到正负值代表方向
    if (subData == 0)                                // 第一次和最后一次重叠，返回错误
    {
        errorCode = CALI_ERROR_AVERAGE_DIR;
        return;
    }
    else
    {
        goDirection = subData > 0; // 1代表顺时针
    }

    for (count = 1; count < MOTOR_ONE_CIRCLE_HARD_STEPS; count++)
    {
        subData = CycleSubtract((int32_t)sampleDataAverageForward[count],
                                (int32_t)sampleDataAverageForward[count - 1],
                                mt6816_base.RESOLUTION);     // 取每相邻的两个数据的差值进行判断
        if (fabs(subData) > (calibSampleResolution * 3 / 2)) // delta-data too large，121
        {
            errorCode = CALI_ERROR_AVERAGE_CONTINUTY;
            return;
        }
        if (fabs(subData) < (calibSampleResolution * 1 / 2)) // delta-data too small，40
        {
            errorCode = CALI_ERROR_AVERAGE_CONTINUTY;
            return;
        }
        if (subData == 0)
        {
            errorCode = CALI_ERROR_AVERAGE_DIR;
            return;
        }
        if ((subData > 0) && (!goDirection)) // 方向不同
        {
            errorCode = CALI_ERROR_AVERAGE_DIR;
            return;
        }
        if ((subData < 0) && (goDirection))
        {
            errorCode = CALI_ERROR_AVERAGE_DIR;
            return;
        }
    }

    // 200步数据全部通过后进行零点判断

    uint32_t step_num = 0;
    if (goDirection) // 顺时针
    {
        for (count = 0; count < MOTOR_ONE_CIRCLE_HARD_STEPS; count++)
        {
            subData = (int32_t)sampleDataAverageForward[CycleMod(count + 1, MOTOR_ONE_CIRCLE_HARD_STEPS)] -
                      (int32_t)sampleDataAverageForward[CycleMod(count, MOTOR_ONE_CIRCLE_HARD_STEPS)];
            if (subData < 0) // 当有一次穿过时钟零点
            {
                step_num++;
                rcdX = (int32_t)count; // 记录零点对应步数位置
                rcdY = (mt6816_base.RESOLUTION - 1) -
                       sampleDataAverageForward[CycleMod(rcdX, MOTOR_ONE_CIRCLE_HARD_STEPS)]; // 16383-零点对应的mt6816校准数据
            }
        }
        if (step_num != 1)
        {
            errorCode = CALI_ERROR_PHASE_STEP;
            return;
        }
    }
    else // 逆时针
    {
        for (count = 0; count < MOTOR_ONE_CIRCLE_HARD_STEPS; count++)
        {
            subData = (int32_t)sampleDataAverageForward[CycleMod(count + 1, MOTOR_ONE_CIRCLE_HARD_STEPS)] -
                      (int32_t)sampleDataAverageForward[CycleMod(count, MOTOR_ONE_CIRCLE_HARD_STEPS)];
            if (subData > 0)
            {
                step_num++;
                rcdX = (int32_t)count;
                rcdY = (mt6816_base.RESOLUTION - 1) -
                       sampleDataAverageForward[CycleMod(rcdX + 1, MOTOR_ONE_CIRCLE_HARD_STEPS)];
            }
        }
        if (step_num != 1)
        {
            errorCode = CALI_ERROR_PHASE_STEP;
            return;
        }
    }

    errorCode = CALI_NO_ERROR;
}

// 默认1000电流，获取到每1.8度对应的编码器数据，包括正转和反转
void EncoderCalibratorBase::Tick20kHz()
{
    // Bring-up policy: configuration is the single current source, capped at
    // the previous 1A software command until actual coil current is validated.
    const int32_t current = ::motor.config.motionParams.caliCurrent;
    if (!isTriggered || current <= 0 || current > 1000)
    {
        tb67h450_base.Sleep();
        state = CALI_DISABLE;
        isTriggered = false;
        return;
    }
    if (state == CALI_CALCULATING)
    {
        // The main-loop writer owns Flash; no table access or output from ISR.
        tb67h450_base.Sleep();
        return;
    }
    if (!mt6816_base.UpdateAngle())
    {
        tb67h450_base.Sleep();
        errorCode = CALI_ERROR_ENCODER;
        state = CALI_DISABLE;
        isTriggered = false;
        return;
    }

    switch (state)
    {
    case CALI_DISABLE:
        if (isTriggered)
        {
            tb67h450_base.SetFocCurrentVector(goPosition, current);
            goPosition = 51200;
            sampleCount = 0;
            state = CALI_FORWARD_PREPARE;
            errorCode = CALI_NO_ERROR;
        }
        break;
    case CALI_FORWARD_PREPARE:
        goPosition += AUTO_CALIB_SPEED;
        tb67h450_base.SetFocCurrentVector(goPosition, current);
        if (goPosition == 2 * 51200) // 细分2转一圈
        {
            goPosition = 51200;
            state = CALI_FORWARD_MEASURE;
        }
        break;
    case CALI_FORWARD_MEASURE:       // 从51200开始
        if ((goPosition % 256) == 0) // 记录每1.8度的数据16次
        {
            sampleDataRaw[sampleCount++] = mt6816_base.angleData.rawAngle;
            if (sampleCount == EncoderCalibratorBase::SAMPLE_COUNTS_PER_STEP)
            {
                // 取平均值放到数组里
                sampleDataAverageForward[(goPosition - 51200) / 256] = CycleDataAverage(sampleDataRaw, EncoderCalibratorBase::SAMPLE_COUNTS_PER_STEP, mt6816_base.RESOLUTION);
                sampleCount = 0;
                goPosition += FINE_TUNE_CALIB_SPEED;
            }
        }
        else
        {
            goPosition += FINE_TUNE_CALIB_SPEED;
        }

        tb67h450_base.SetFocCurrentVector(goPosition, current);

        if (goPosition > (2 * 51200)) // 跑完一圈
        {
            state = CALI_BACKWARD_RETURN;
        }
        break;
    case CALI_BACKWARD_RETURN: // 从2*51200开始
        goPosition += FINE_TUNE_CALIB_SPEED;
        tb67h450_base.SetFocCurrentVector(goPosition, current);

        if (goPosition ==
            (2 * 51200 + 256 * 20))
        { // 继续跑20个1.8度后退出
            state = CALI_BACKWARD_GAP_DISMISS;
        }
        break;
    case CALI_BACKWARD_GAP_DISMISS: // 从2 * 51200 + 256 * 20开始
        goPosition -= FINE_TUNE_CALIB_SPEED;
        tb67h450_base.SetFocCurrentVector(goPosition, current);
        if (goPosition == (2 * 51200))
        { // 回到起点
            state = CALI_BACKWARD_MEASURE;
        }
        break;
    case CALI_BACKWARD_MEASURE: // 从2*51200开始
        if ((goPosition % 256) == 0)
        {
            sampleDataRaw[sampleCount++] = mt6816_base.angleData.rawAngle;
            if (sampleCount == EncoderCalibratorBase::SAMPLE_COUNTS_PER_STEP)
            {
                sampleDataAverageBackward[(goPosition - 51200) /
                                          256] = CycleDataAverage(sampleDataRaw, EncoderCalibratorBase::SAMPLE_COUNTS_PER_STEP,
                                                                  mt6816_base.RESOLUTION);

                sampleCount = 0;
                goPosition -= FINE_TUNE_CALIB_SPEED;
            }
        }
        else
        {
            goPosition -= FINE_TUNE_CALIB_SPEED;
        }
        tb67h450_base.SetFocCurrentVector(goPosition, current);
        if (goPosition < 51200)
        {
            state = CALI_CALCULATING;
        }
        break;
    default:
        break;
    }
}

void EncoderCalibratorBase::TickMainLoop()
{
    CalibrationWriter &writer=calibrationWriter;
    if (!calibrationWriting(*this)) {
        if(writer.phase) finishCalibrationStorage(*this,false);
        return;
    }
    if(storageCancel){finishCalibrationStorage(*this,false);return;}
    storageWorking=true;
    tb67h450_base.Sleep();
    mt6816_base.angleData.rectifyValid=false;
    if(!writer.phase) {
        storageProgress=(2U<<8)|50U;
        if(*reinterpret_cast<volatile const uint16_t *>(uintptr_t(FLASHSIZE_BASE))!=128U) {
            finishCalibrationStorage(*this,false);return;
        }
        CalibrationDataCheck();
        if(errorCode!=CALI_NO_ERROR || rcdX<0 || rcdX>=200 || rcdY<0) {
            finishCalibrationStorage(*this,false);return;
        }
        writer=CalibrationWriter();
        writer.phase=4;storageProgress=(3U<<8)|50U; // invalidate before touching table
        resultNum=0;calibrationWriteOk=true;expectedCalibrationCrc=0xffffffffU;
        return;
    }
    if(writer.phase==4) {
        Stockpile_FLASH_Typedef metadata=calibrationMetadataArea();
        Stockpile_Flash_Data_Empty(&metadata);
        bool erased=calibrationWriting(*this);
        for(unsigned i=0;i<Stockpile_Page_Size/2U && erased;++i)
            erased=*reinterpret_cast<volatile const uint16_t *>(uintptr_t(metadata.begin_add)+2U*i)==0xffffU;
        if(!erased){finishCalibrationStorage(*this,false);return;}
        writer.phase=1;return;
    }
    if(writer.phase==1) {
        storageProgress=(3U<<8)|(50U+5U*writer.page/32U);
        // One physical page per call. The HAL call itself is still blocking.
        Stockpile_FLASH_Typedef page=stockpile_quick_cali;
        const uint32_t pages=uint32_t(mt6816_base.RESOLUTION)*2U/Stockpile_Page_Size;
        if(writer.page>=pages || stockpile_quick_cali.area_size<uint32_t(mt6816_base.RESOLUTION)*2U) {
            finishCalibrationStorage(*this,false);return;
        }
        page.begin_add+=writer.page*Stockpile_Page_Size;
        page.area_size=Stockpile_Page_Size;page.page_num=1;page.asce_write_add=0;
        Stockpile_Flash_Data_Empty(&page);
        if(!calibrationWriting(*this)) {finishCalibrationStorage(*this,false);return;}
        for(uint32_t i=0;i<Stockpile_Page_Size/2U;++i)
            if(*reinterpret_cast<volatile const uint16_t *>(uintptr_t(page.begin_add)+i*2U)!=0xffffU) {
                finishCalibrationStorage(*this,false);return;
            }
        if(++writer.page==pages) {
            writer.phase=2;writer.x=goDirection?rcdX:rcdX+200;
            writer.stepReady=false;
        }
        return;
    }
    if(writer.phase==2) {
        storageProgress=(3U<<8)|(55U+35U*resultNum/uint32_t(mt6816_base.RESOLUTION));
        // Unlock only for this bounded chunk, and relock before yielding.
        Stockpile_Flash_Data_Begin(&stockpile_quick_cali);
        if(stockpile_quick_cali.asce_write_add!=stockpile_quick_cali.begin_add) {
            EndWriteFlash();finishCalibrationStorage(*this,false);return;
        }
        Stockpile_Flash_Data_Set_Write_Add(&stockpile_quick_cali,
            stockpile_quick_cali.begin_add+resultNum*2U);
        for(unsigned n=0;n<32 && resultNum<uint32_t(mt6816_base.RESOLUTION);++n) {
            uint16_t value=0;
            if(!nextCalibrationValue(*this,value)) {calibrationWriteOk=false;break;}
            WriteFlash16bitsAppend(value);
            if(!calibrationWriting(*this) || !calibrationWriteOk) break;
            ++resultNum;
        }
        EndWriteFlash();
        if(!calibrationWriting(*this) || !calibrationWriteOk) {
            finishCalibrationStorage(*this,false);return;
        }
        if(resultNum==uint32_t(mt6816_base.RESOLUTION)) {
            uint16_t extra=0;
            if(nextCalibrationValue(*this,extra) || !calibrationWriteOk) {
                errorCode=CALI_ERROR_ANALYSIS_QUANTITY;
                finishCalibrationStorage(*this,false);return;
            }
            writer.phase=3;writer.verified=0;writer.actualCrc=0xffffffffU;
            storageProgress=(4U<<8)|90U;
        }
        return;
    }
    if(writer.phase==5 || writer.phase==6) {
        storageProgress=(4U<<8)|99U;
        using namespace stepper::calibration;
        Stockpile_FLASH_Typedef metadata=calibrationMetadataArea();
        const uint32_t fields[4]={Magic,Version,Bytes,~expectedCalibrationCrc};
        if(writer.phase==6) {
            const volatile Record &record=stored();
            if(record.magic!=Magic || record.version!=Version || record.bytes!=Bytes ||
                record.crc!=~expectedCalibrationCrc || record.commit!=0xffffffffU) {
                finishCalibrationStorage(*this,false);return;
            }
        }
        Stockpile_Flash_Data_Begin(&metadata);
        bool ok=metadata.asce_write_add==metadata.begin_add;
        if(writer.phase==6)
            Stockpile_Flash_Data_Set_Write_Add(&metadata,metadata.begin_add+16U);
        const unsigned words=writer.phase==5?8U:2U;
        for(unsigned i=0;i<words && ok;++i) {
            const uint32_t value=writer.phase==5?fields[i/2]:Commit;
            ok=writeMetadataWord(*this,metadata,uint16_t(value>>(16*(i%2))));
        }
        EndWriteFlash();
        if(!ok || !calibrationWriting(*this)){finishCalibrationStorage(*this,false);return;}
        if(writer.phase==5){writer.phase=6;return;}
        finishCalibrationStorage(*this,committed(stored()) && stored().crc==~expectedCalibrationCrc);return;
    }
    // Read at most 128 halfwords, then return to the outer UART loop.
    storageProgress=(4U<<8)|(90U+9U*writer.verified/uint32_t(mt6816_base.RESOLUTION));
    for(unsigned n=0;n<128 && writer.verified<resultNum;++n,++writer.verified)
        writer.actualCrc=calibrationCrcWord(writer.actualCrc,
            *reinterpret_cast<volatile const uint16_t *>(uintptr_t(stockpile_quick_cali.begin_add)+2U*writer.verified));
    if(writer.verified==resultNum) {
        if(writer.actualCrc!=expectedCalibrationCrc)finishCalibrationStorage(*this,false);
        else writer.phase=5;
    }
}

// 取余，把_a限制在_b内
uint32_t EncoderCalibratorBase::CycleMod(uint32_t _a, uint32_t _b)
{
    return (_a + _b) % _b;
}

// 取循环距离差，返回两者在圆上的距离，指在圆上距离小的那段值，有正负，例如(11,1,12)=-2,(1,11,12)=2
int32_t EncoderCalibratorBase::CycleSubtract(int32_t _a, int32_t _b, int32_t _cyc)
{
    int32_t sub_data;

    sub_data = _a - _b;
    if (sub_data > (_cyc >> 1))
        sub_data -= _cyc;
    if (sub_data < (-_cyc >> 1))
        sub_data += _cyc;
    return sub_data;
}

// 返回a,b中值，取两者距离小的那段的中值
int32_t EncoderCalibratorBase::CycleAverage(int32_t _a, int32_t _b, int32_t _cyc)
{
    int32_t sub_data;
    int32_t ave_data;

    sub_data = _a - _b;
    ave_data = (_a + _b) >> 1;

    if (fabs(sub_data) > (_cyc >> 1)) //
    {
        if (ave_data >= (_cyc >> 1))
            ave_data -= (_cyc >> 1);
        else
            ave_data += (_cyc >> 1);
    }
    return ave_data;
}
