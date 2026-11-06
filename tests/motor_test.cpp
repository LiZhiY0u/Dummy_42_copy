#include "Motor/motor.h"
#include "configurations.h"
#include <cassert>
#include <cmath>
#include <cstdio>
BoardConfig_t boardConfig;
class Encoder : public EncoderBase {
public:
    bool valid = true;
    bool Init() override { return true; }
    uint16_t UpdateAngle() override { angleData.sampleValid = valid; angleData.rectifyValid = true; return 0; }
    bool IsCalibrated() override { return true; }
};
class Driver : public DriverBase {
public:
    int outputs = 0, sleeps = 0;
    void Init() override {}
    void SetFocCurrentVector(uint32_t, int32_t) override { ++outputs; }
    void Sleep() override { ++sleeps; }
    void Brake() override {}
    void SetTwoCoilsCurrent(uint16_t, uint16_t) override {}
};
int main() {
    Motor motor; Encoder encoder; Driver driver;
    motor.AttachDriver(&driver); motor.AttachEncoder(&encoder); motor.controller->Init();
    motor.controller->SetCtrlMode(Motor::MODE_COMMAND_CURRENT);
    motor.controller->SetCurrentSetPoint(100);
    for (int n=0; n<20; ++n) motor.Tick20kHz();
    assert(driver.outputs > 0);
    encoder.valid = false;
    motor.Tick20kHz();
    int before = driver.outputs;
    assert(motor.HasFault() && driver.sleeps > 0);
    encoder.valid = true;
    motor.controller->SetCtrlMode(Motor::MODE_COMMAND_CURRENT);
    for (int n=0; n<20; ++n) motor.Tick20kHz();
    assert(driver.outputs == before && motor.controller->state == Motor::STATE_ENCODER_FAULT);
    Motor planner; planner.AttachDriver(&driver); planner.AttachEncoder(&encoder); planner.controller->Init();
    auto limit = planner.config.motionParams.ratedVelocity;
    assert(!planner.controller->SetPositionSetPointWithTime(100, NAN));
    assert(planner.config.motionParams.ratedVelocity == limit);
    assert(!planner.controller->SetPositionSetPointWithTime(100, -1));
    assert(planner.controller->SetPositionSetPointWithTime(1, 3600));
    assert(planner.config.motionParams.ratedVelocity >= 1 && planner.config.motionParams.ratedVelocity <= boardConfig.velocityLimit);
    Motor maintenance; maintenance.AttachDriver(&driver); maintenance.AttachEncoder(&encoder); maintenance.controller->Init();
    maintenance.BeginMaintenance();
    before = driver.outputs;
    maintenance.controller->SetCtrlMode(Motor::MODE_COMMAND_CURRENT);
    maintenance.controller->SetCurrentSetPoint(100);
    for (int n=0; n<20; ++n) maintenance.Tick20kHz();
    assert(driver.outputs == before);
    maintenance.EndMaintenance();
    for (int n=0; n<20; ++n) maintenance.Tick20kHz();
    assert(driver.outputs == before && maintenance.controller->requestMode == Motor::MODE_STOP);
    std::puts("motor: real output before fault, latched shutdown, timing/rate limits passed");
}
