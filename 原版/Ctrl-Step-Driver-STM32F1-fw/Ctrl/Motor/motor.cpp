#include "configurations.h"
#include "motor.h"

#include <cmath>
#include "interrupt_guard.h"

void Motor::Tick20kHz()
{
    // 1.Encoder data Update，获取mt6816校准值
    if (maintenance) { Stop(); return; }
    if (storageFault) {
        Stop();
        controller->state = STATE_STORAGE_FAULT;
        return;
    }
    encoder->UpdateAngle();
    if (!encoder->angleData.sampleValid || encoderFault)
    {
        LatchEncoderFault();
        return;
    }

    // 2.Motor Control Update
    CloseLoopControlTick();
}

void Motor::BeginMaintenance()
{
    InterruptGuard guard;
    maintenance = true;
    Stop();
}

void Motor::EndMaintenance()
{
    InterruptGuard guard;
    Stop();
    maintenance = false;
}

void Motor::Stop()
{
    controller->requestMode = MODE_STOP;
    controller->modeRunning = MODE_STOP;
    controller->goalVelocity = controller->goalCurrent = 0;
    controller->softVelocity = controller->softCurrent = 0;
    controller->focCurrent = 0;
    controller->softNewCurve = true;
    controller->ClearIntegral();
    if (driver) driver->Sleep();
}

void Motor::LatchStorageFault()
{
    storageFault = true;
    Stop();
    controller->state = STATE_STORAGE_FAULT;
}

void Motor::LatchEncoderFault()
{
    encoderFault = true;
    controller->requestMode = MODE_STOP;
    controller->modeRunning = MODE_STOP;
    controller->goalVelocity = controller->goalCurrent = 0;
    controller->focCurrent = 0;
    controller->ClearIntegral();
    controller->state = STATE_ENCODER_FAULT;
    if (driver) driver->Sleep();
}

void Motor::AttachEncoder(EncoderBase *_encoder)
{
    encoder = _encoder;
}

void Motor::AttachDriver(DriverBase *_driver)
{
    driver = _driver;
}

void Motor::CloseLoopControlTick()
{
    /************************************ First Called ************************************/
    static bool isFirstCalled = true; // static只赋值一次，下次调用不再赋值，并在值变化时保持变量的持久性
    if (isFirstCalled)                // 初次跑闭环，把大半圈位置置负数，比如0到10中把7变-3
    {
        int32_t angle; // 整形有负数
        if (config.motionParams.encoderHomeOffset < MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS / 2)
        {
            angle =
                encoder->angleData.rectifiedAngle >
                        config.motionParams.encoderHomeOffset + MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS / 2
                    ? encoder->angleData.rectifiedAngle - MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS
                    : encoder->angleData.rectifiedAngle;
        }
        else
        {
            angle =
                encoder->angleData.rectifiedAngle <
                        config.motionParams.encoderHomeOffset - MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS / 2
                    ? encoder->angleData.rectifiedAngle + MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS
                    : encoder->angleData.rectifiedAngle;
        }

        controller->realLapPosition = angle;
        controller->realLapPositionLast = angle;
        controller->realPosition = angle;
        controller->realPositionLast = angle;

        isFirstCalled = false;
        return;
    }

    /********************************* Update Data *********************************/
    int32_t deltaLapPosition; // 超半圈的运动变反向

    // Read Encoder data
    controller->realLapPositionLast = controller->realLapPosition;   // 上次细分数
    controller->realLapPosition = encoder->angleData.rectifiedAngle; // 当前细分数

    // 注意以下都是以细分数进行
    //  Lap-Position calculate
    deltaLapPosition = controller->realLapPosition - controller->realLapPositionLast; // 与上次位置差
    if (deltaLapPosition > MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS >> 1)
        deltaLapPosition -= MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
    else if (deltaLapPosition < -MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS >> 1)
        deltaLapPosition += MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;

    // Naive-Position calculate
    controller->realPositionLast = controller->realPosition;
    controller->realPosition += deltaLapPosition;

    /********************************* Estimate Data *********************************/
    // Estimate Velocity
    controller->estVelocityIntegral += ((controller->realPosition - controller->realPositionLast) * motionPlanner.CONTROL_FREQUENCY + ((controller->estVelocity << 5) - controller->estVelocity)); // 位置差值*20000+a*31累加
    controller->estVelocity = controller->estVelocityIntegral >> 5;
    controller->estVelocityIntegral -= (controller->estVelocity << 5);

    // Estimate Position
    controller->estLeadPosition = Controller::CompensateAdvancedAngle(controller->estVelocity);
    controller->estPosition = controller->realPosition + controller->estLeadPosition; // 由估计速度和补偿表得到估计位置

    // Estimate Error
    controller->estError = controller->softPosition - controller->estPosition;

    /************************************ Ctrl Loop ************************************/
    if (controller->isStalled ||
        controller->softDisable ||
        !encoder->IsCalibrated()) // 首次会进入
    {
        controller->ClearIntegral(); // clear integrals
        controller->focPosition = 0; // clear outputs
        controller->focCurrent = 0;
        driver->Sleep();
    }
    else if (controller->softBrake)
    {
        controller->ClearIntegral();
        controller->focPosition = 0;
        controller->focCurrent = 0;
        driver->Brake();
    }
    else
    {
        switch (controller->modeRunning)
        {
        case MODE_STEP_DIR:
            controller->CalcDceToOutput(controller->softPosition, controller->softVelocity);
            break;
        case MODE_STOP:
            driver->Sleep();
            break;
        case MODE_COMMAND_Trajectory:
            controller->CalcDceToOutput(controller->softPosition, controller->softVelocity);
            break;
        case MODE_COMMAND_CURRENT:
            controller->CalcCurrentToOutput(controller->softCurrent);
            break;
        case MODE_COMMAND_VELOCITY:
            controller->CalcPidToOutput(controller->softVelocity);
            break;
        case MODE_COMMAND_POSITION:
            controller->CalcDceToOutput(controller->softPosition, controller->softVelocity);
            break;
        case MODE_PWM_CURRENT:
            controller->CalcCurrentToOutput(controller->softCurrent);
            break;
        case MODE_PWM_VELOCITY:
            controller->CalcPidToOutput(controller->softVelocity);
            break;
        case MODE_PWM_POSITION:
            controller->CalcDceToOutput(controller->softPosition, controller->softVelocity);
            break;
        default:
            break;
        }
    }

    /******************************* Mode Change Handle *******************************/
    if (controller->modeRunning != controller->requestMode) //
    {
        controller->modeRunning = controller->requestMode;
        controller->softNewCurve = true;
    }

    /******************************* Update Hard-Goal，限制速度和电流 *******************************/
    if (controller->goalVelocity > config.motionParams.ratedVelocity)
        controller->goalVelocity = config.motionParams.ratedVelocity;
    else if (controller->goalVelocity < -config.motionParams.ratedVelocity)
        controller->goalVelocity = -config.motionParams.ratedVelocity;
    if (controller->goalCurrent > config.motionParams.ratedCurrent)
        controller->goalCurrent = config.motionParams.ratedCurrent;
    else if (controller->goalCurrent < -config.motionParams.ratedCurrent)
        controller->goalCurrent = -config.motionParams.ratedCurrent;

    /******************************** Motion Plan，好像没用上 *********************************/
    if ((controller->softDisable && !controller->goalDisable) ||
        (controller->softBrake && !controller->goalBrake))
    {
        controller->softNewCurve = true;
    }

    if (controller->softNewCurve) // 只有模式变化时才执行一次，由估计速度和估计位置得到跟踪器速度和位置
    {
        controller->softNewCurve = false;
        controller->ClearIntegral();
        controller->ClearStallFlag();

        switch (controller->modeRunning) // 更新tracker变量
        {
        case MODE_STOP:
            break;
        case MODE_COMMAND_POSITION:
            motionPlanner.positionTracker.NewTask(controller->estPosition, controller->estVelocity);
            break;
        case MODE_COMMAND_VELOCITY:
            motionPlanner.velocityTracker.NewTask(controller->estVelocity);
            break;
        case MODE_COMMAND_CURRENT:
            motionPlanner.currentTracker.NewTask(controller->focCurrent);
            break;
        case MODE_COMMAND_Trajectory:
            motionPlanner.trajectoryTracker.NewTask(controller->estPosition, controller->estVelocity);
            break;
        case MODE_PWM_POSITION:
            motionPlanner.positionTracker.NewTask(controller->estPosition, controller->estVelocity);
            break;
        case MODE_PWM_VELOCITY:
            motionPlanner.velocityTracker.NewTask(controller->estVelocity);
            break;
        case MODE_PWM_CURRENT:
            motionPlanner.currentTracker.NewTask(controller->focCurrent);
            break;
        case MODE_STEP_DIR:
            motionPlanner.positionInterpolator.NewTask(controller->estPosition, controller->estVelocity);
            // step/dir mode uses delta-position, so stay where we are
            controller->goalPosition = controller->estPosition;
            break;
        default:
            break;
        }
    }

    /******************************* Update Soft Goal *******************************/
    switch (controller->modeRunning) // 必定进入，更新soft变量
    {
    case MODE_STOP:
        break;
    case MODE_COMMAND_POSITION:
        motionPlanner.positionTracker.CalcSoftGoal(controller->goalPosition);
        controller->softPosition = motionPlanner.positionTracker.go_location;
        controller->softVelocity = motionPlanner.positionTracker.go_velocity;
        break;
    case MODE_COMMAND_VELOCITY:
        motionPlanner.velocityTracker.CalcSoftGoal(controller->goalVelocity);
        controller->softVelocity = motionPlanner.velocityTracker.goVelocity;
        break;
    case MODE_COMMAND_CURRENT:
        motionPlanner.currentTracker.CalcSoftGoal(controller->goalCurrent);
        controller->softCurrent = motionPlanner.currentTracker.goCurrent;
        break;
    case MODE_COMMAND_Trajectory:
        motionPlanner.trajectoryTracker.CalcSoftGoal(controller->goalPosition, controller->goalVelocity);
        controller->softPosition = motionPlanner.trajectoryTracker.goPosition;
        controller->softVelocity = motionPlanner.trajectoryTracker.goVelocity;
        break;
    case MODE_PWM_POSITION:
        motionPlanner.positionTracker.CalcSoftGoal(controller->goalPosition);
        controller->softPosition = motionPlanner.positionTracker.go_location;
        controller->softVelocity = motionPlanner.positionTracker.go_velocity;
        break;
    case MODE_PWM_VELOCITY:
        motionPlanner.velocityTracker.CalcSoftGoal(controller->goalVelocity);
        controller->softVelocity = motionPlanner.velocityTracker.goVelocity;
        break;
    case MODE_PWM_CURRENT:
        motionPlanner.currentTracker.CalcSoftGoal(controller->goalCurrent);
        controller->softCurrent = motionPlanner.currentTracker.goCurrent;
        break;
    case MODE_STEP_DIR:
        motionPlanner.positionInterpolator.CalcSoftGoal(controller->goalPosition);
        controller->softPosition = motionPlanner.positionInterpolator.goPosition;
        controller->softVelocity = motionPlanner.positionInterpolator.goVelocity;
        break;
    default:
        break;
    }

    controller->softDisable = controller->goalDisable;
    controller->softBrake = controller->goalBrake;

    /******************************** State Check ********************************/
    int32_t current = abs(controller->focCurrent);

    // Stall detect
    if (controller->config->stallProtectSwitch)
    {
        if ( // Current Mode
            ((controller->modeRunning == MODE_COMMAND_CURRENT ||
              controller->modeRunning == MODE_PWM_CURRENT) &&
             (current != 0)) || // Other Mode
            current == config.motionParams.ratedCurrent)
        {
            if (abs(controller->estVelocity) < MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS / 5)
            {
                if (controller->stalledTime >= 1000 * 1000)
                    controller->isStalled = true;
                else
                    controller->stalledTime += motionPlanner.CONTROL_PERIOD;
            }
        }
        else // can ONLY clear stall flag  MANUALLY
        {
            controller->stalledTime = 0;
        }
    }

    // Overload detect
    if ((controller->modeRunning != MODE_COMMAND_CURRENT) &&
        (controller->modeRunning != MODE_PWM_CURRENT) &&
        (current == config.motionParams.ratedCurrent))
    {
        if (controller->overloadTime >= 1000 * 1000)
            controller->overloadFlag = true;
        else
            controller->overloadTime += motionPlanner.CONTROL_PERIOD;
    }
    else // auto clear overload flag when released
    {
        controller->overloadTime = 0;
        controller->overloadFlag = false;
    }

    /******************************** Update State ********************************/
    if (!encoder->IsCalibrated())
        controller->state = STATE_NO_CALIB;
    else if (controller->modeRunning == MODE_STOP)
        controller->state = STATE_STOP;
    else if (controller->isStalled)
        controller->state = STATE_STALL;
    else if (controller->overloadFlag)
        controller->state = STATE_OVERLOAD;
    else
    {
        if (controller->modeRunning == MODE_COMMAND_POSITION)
        {
            if ((controller->softPosition == controller->goalPosition) && (controller->softVelocity == 0))
                controller->state = STATE_FINISH;
            else
                controller->state = STATE_RUNNING;
        }
        else if (controller->modeRunning == MODE_COMMAND_VELOCITY)
        {
            if (controller->softVelocity == controller->goalVelocity)
                controller->state = STATE_FINISH;
            else
                controller->state = STATE_RUNNING;
        }
        else if (controller->modeRunning == MODE_COMMAND_CURRENT)
        {
            if (controller->softCurrent == controller->goalCurrent)
                controller->state = STATE_FINISH;
            else
                controller->state = STATE_RUNNING;
        }
        else
        {
            controller->state = STATE_FINISH;
        }
    }
}

// 由pid计算得到运动方向（正负），实际运动看估计位置加256？
void Motor::Controller::CalcCurrentToOutput(int32_t current)
{
    focCurrent = current;

    if (focCurrent > 0)
        focPosition = estPosition + context->SOFT_DIVIDE_NUM; // ahead phase of 90°
    else if (focCurrent < 0)
        focPosition = estPosition - context->SOFT_DIVIDE_NUM; // behind phase of 90°
    else
        focPosition = estPosition;

    context->driver->SetFocCurrentVector(focPosition, focCurrent);
}

void Motor::Controller::CalcPidToOutput(int32_t _speed)
{
    config->pid.vErrorLast = config->pid.vError;
    config->pid.vError = _speed - estVelocity;
    if (config->pid.vError > (1024 * 1024))
        config->pid.vError = (1024 * 1024);
    if (config->pid.vError < (-1024 * 1024))
        config->pid.vError = (-1024 * 1024);
    config->pid.outputKp = ((config->pid.kp) * (config->pid.vError));

    config->pid.integralRound += (config->pid.ki * config->pid.vError);
    config->pid.integralRemainder = config->pid.integralRound >> 10;
    config->pid.integralRound -= (config->pid.integralRemainder << 10);
    config->pid.outputKi += config->pid.integralRemainder;
    // integralRound limitation is  ratedCurrent*1024
    if (config->pid.outputKi > context->config.motionParams.ratedCurrent << 10)
        config->pid.outputKi = context->config.motionParams.ratedCurrent << 10;
    else if (config->pid.outputKi < -(context->config.motionParams.ratedCurrent << 10))
        config->pid.outputKi = -(context->config.motionParams.ratedCurrent << 10);

    config->pid.outputKd = config->pid.kd * (config->pid.vError - config->pid.vErrorLast);

    config->pid.output = (config->pid.outputKp + config->pid.outputKi + config->pid.outputKd) >> 10;
    if (config->pid.output > context->config.motionParams.ratedCurrent)
        config->pid.output = context->config.motionParams.ratedCurrent;
    else if (config->pid.output < -context->config.motionParams.ratedCurrent)
        config->pid.output = -context->config.motionParams.ratedCurrent;

    CalcCurrentToOutput(config->pid.output);
}

void Motor::Controller::CalcDceToOutput(int32_t _location, int32_t _speed)
{
    config->dce.pError = _location - estPosition;
    if (config->dce.pError > (3200))
        config->dce.pError = (3200); // limited pError to 1/16r (51200/16)
    if (config->dce.pError < (-3200))
        config->dce.pError = (-3200);
    config->dce.vError = (_speed - estVelocity) >> 7;
    if (config->dce.vError > (4000))
        config->dce.vError = (4000); // limited vError
    if (config->dce.vError < (-4000))
        config->dce.vError = (-4000);

    config->dce.outputKp = config->dce.kp * config->dce.pError;

    config->dce.integralRound += (config->dce.ki * config->dce.pError + config->dce.kv * config->dce.vError);
    config->dce.integralRemainder = config->dce.integralRound >> 7;
    config->dce.integralRound -= (config->dce.integralRemainder << 7);
    config->dce.outputKi += config->dce.integralRemainder;
    // limited to ratedCurrent * 1024, should be TUNED when use different scene
    if (config->dce.outputKi > context->config.motionParams.ratedCurrent << 10) // 1024000
        config->dce.outputKi = context->config.motionParams.ratedCurrent << 10;
    else if (config->dce.outputKi < -(context->config.motionParams.ratedCurrent << 10))
        config->dce.outputKi = -(context->config.motionParams.ratedCurrent << 10);

    config->dce.outputKd = ((config->dce.kd) * (config->dce.vError));

    config->dce.output = (config->dce.outputKp + config->dce.outputKi + config->dce.outputKd) >> 10;
    if (config->dce.output > context->config.motionParams.ratedCurrent)
        config->dce.output = context->config.motionParams.ratedCurrent;
    else if (config->dce.output < -context->config.motionParams.ratedCurrent)
        config->dce.output = -context->config.motionParams.ratedCurrent;

    CalcCurrentToOutput(config->dce.output);
}

void Motor::Controller::SetCtrlMode(Motor::Mode_t _mode)
{
    requestMode = _mode;
}

void Motor::Controller::AddTrajectorySetPoint(int32_t _pos, int32_t _vel)
{
    SetPositionSetPoint(_pos);
    SetVelocitySetPoint(_vel);
}

void Motor::Controller::SetPositionSetPoint(int32_t _pos)
{
    goalPosition = _pos + context->config.motionParams.encoderHomeOffset;
}

bool Motor::Controller::SetPositionSetPointWithTime(int32_t _pos, float _time)
{
    if (!std::isfinite(_time) || _time <= 0 || _time > 3600 ||
        context->config.motionParams.ratedVelocityAcc <= 0) return false;
    const double delta = std::fabs(static_cast<double>(_pos) - realPosition +
                                   context->config.motionParams.encoderHomeOffset);
    const double acceleration = context->config.motionParams.ratedVelocityAcc;
    const double time = _time;
    const double discriminant = time*time - 4.0*delta/acceleration;
    bool feasible = discriminant >= 0;
    double velocity = boardConfig.velocityLimit;
    if (feasible && delta > 0)
        velocity = 2.0 * delta / (time + std::sqrt(discriminant));
    if (velocity > boardConfig.velocityLimit) { velocity = boardConfig.velocityLimit; feasible = false; }
    if (velocity < 1) velocity = 1;
    InterruptGuard guard;
    if (context->HasFault()) return false;
    context->config.motionParams.ratedVelocity = static_cast<int32_t>(velocity);
    SetPositionSetPoint(_pos);
    SetCtrlMode(MODE_COMMAND_POSITION);
    return feasible;
}

void Motor::Controller::SetVelocitySetPoint(int32_t _vel)
{
    if ((_vel >= -context->config.motionParams.ratedVelocity) &&
        (_vel <= context->config.motionParams.ratedVelocity))
    {
        goalVelocity = _vel;
    }
}

float Motor::Controller::GetPosition(bool _isLap)
{
    return _isLap ? (float)(realLapPosition - context->config.motionParams.encoderHomeOffset) /
                        (float)(context->MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS)
                  : (float)(realPosition - context->config.motionParams.encoderHomeOffset) /
                        (float)(context->MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
}

float Motor::Controller::GetVelocity()
{
    return (float)estVelocity / (float)context->MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
}

float Motor::Controller::GetFocCurrent()
{
    return (float)focCurrent / 1000.f;
}

// 限制正负1000内
void Motor::Controller::SetCurrentSetPoint(int32_t _cur)
{
    if (_cur > context->config.motionParams.ratedCurrent)
        goalCurrent = context->config.motionParams.ratedCurrent;
    else if (_cur < -context->config.motionParams.ratedCurrent)
        goalCurrent = -context->config.motionParams.ratedCurrent;
    else
        goalCurrent = _cur;
}

void Motor::Controller::SetDisable(bool _disable)
{
    goalDisable = _disable;
}

void Motor::Controller::SetBrake(bool _brake)
{
    goalBrake = _brake;
}

void Motor::Controller::ClearStallFlag()
{
    stalledTime = 0;
    isStalled = false;
}

int32_t Motor::Controller::CompensateAdvancedAngle(int32_t _vel)
{
    /*
     * The code is for DPS series sensors, need to measured and renew for TLE5012/MT6816.
     */

    int32_t compensate;

    if (_vel < 0)
    {
        if (_vel > -100000)
            compensate = 0;
        else if (_vel > -1300000)
            compensate = (((_vel + 100000) * 262) >> 20) - 0;
        else if (_vel > -2200000)
            compensate = (((_vel + 1300000) * 105) >> 20) - 300;
        else
            compensate = (((_vel + 2200000) * 52) >> 20) - 390;

        if (compensate < -430)
            compensate = -430;
    }
    else
    {
        if (_vel < 100000)
            compensate = 0;
        else if (_vel < 1300000)
            compensate = (((_vel - 100000) * 262) >> 20) + 0;
        else if (_vel < 2200000)
            compensate = (((_vel - 1300000) * 105) >> 20) + 300;
        else
            compensate = (((_vel - 2200000) * 52) >> 20) + 390;

        if (compensate > 430)
            compensate = 430;
    }

    return compensate;
}

void Motor::Controller::Init()
{
    requestMode = boardConfig.enableMotorOnBoot ? static_cast<Mode_t>(boardConfig.defaultMode) : MODE_STOP;

    modeRunning = MODE_STOP;
    state = STATE_STOP;

    realLapPosition = 0;
    realLapPositionLast = 0;
    realPosition = 0;
    realPositionLast = 0;

    estVelocityIntegral = 0;
    estVelocity = 0;
    estLeadPosition = 0;
    estPosition = 0;
    estError = 0;

    goalPosition = context->config.motionParams.encoderHomeOffset;
    goalVelocity = 0;
    goalCurrent = 0;
    goalDisable = false;
    goalBrake = false;

    softPosition = 0;
    softVelocity = 0;
    softCurrent = 0;
    softDisable = false;
    softBrake = false;
    softNewCurve = false;

    focPosition = 0;
    focCurrent = 0;

    stalledTime = 0;
    isStalled = false;

    overloadTime = 0;
    overloadFlag = false;

    config->pid.vError = 0;
    config->pid.vErrorLast = 0;
    config->pid.outputKp = 0;
    config->pid.outputKi = 0;
    config->pid.outputKd = 0;
    config->pid.integralRound = 0;
    config->pid.integralRemainder = 0;
    config->pid.output = 0;

    config->dce.pError = 0;
    config->dce.vError = 0;
    config->dce.outputKp = 0;
    config->dce.outputKi = 0;
    config->dce.outputKd = 0;
    config->dce.integralRound = 0;
    config->dce.integralRemainder = 0;
    config->dce.output = 0;
}

void Motor::Controller::ApplyPosAsHomeOffset()
{
    context->config.motionParams.encoderHomeOffset = realPosition %
                                                     context->MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
}

void Motor::Controller::AttachConfig(Motor::Controller::Config_t *_config)
{
    config = _config;
}

void Motor::Controller::ClearIntegral() const
{
    config->pid.integralRound = 0;
    config->pid.integralRemainder = 0;
    config->pid.outputKi = 0;

    config->dce.integralRound = 0;
    config->dce.integralRemainder = 0;
    config->dce.outputKi = 0;
}
