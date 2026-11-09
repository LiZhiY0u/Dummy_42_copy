#include "motion_planner.h"
#include "math.h"

void MotionPlanner::CurrentTracker::Init()
{
    SetCurrentAcc(context->config->ratedCurrentAcc);
}

void MotionPlanner::CurrentTracker::NewTask(int32_t _realCurrent)
{
    currentIntegral = 0;
    trackCurrent = _realCurrent;
}

void MotionPlanner::CurrentTracker::CalcSoftGoal(int32_t _goalCurrent)
{
    int32_t deltaCurrent = _goalCurrent - trackCurrent;

    if (deltaCurrent == 0)
    {
        trackCurrent = _goalCurrent;
    }
    else if (deltaCurrent > 0)
    {
        if (trackCurrent >= 0)
        {
            CalcCurrentIntegral(currentAcc);
            if (trackCurrent >= _goalCurrent)
            {
                currentIntegral = 0;
                trackCurrent = _goalCurrent;
            }
        }
        else
        {
            CalcCurrentIntegral(currentAcc);
            if ((int32_t)trackCurrent >= 0)
            {
                currentIntegral = 0;
                trackCurrent = 0;
            }
        }
    }
    else if (deltaCurrent < 0)
    {
        if (trackCurrent <= 0)
        {
            CalcCurrentIntegral(-currentAcc);
            if ((int32_t)trackCurrent <= (int32_t)_goalCurrent)
            {
                currentIntegral = 0;
                trackCurrent = _goalCurrent;
            }
        }
        else
        {
            CalcCurrentIntegral(-currentAcc);
            if ((int32_t)trackCurrent <= 0)
            {
                currentIntegral = 0;
                trackCurrent = 0;
            }
        }
    }

    goCurrent = (int32_t)trackCurrent;
}

void MotionPlanner::CurrentTracker::CalcCurrentIntegral(int32_t _current)
{
    currentIntegral += _current;
    trackCurrent += currentIntegral / context->CONTROL_FREQUENCY;
    currentIntegral = currentIntegral % context->CONTROL_FREQUENCY;
}

void MotionPlanner::CurrentTracker::SetCurrentAcc(int32_t _currentAcc)
{
    currentAcc = _currentAcc;
}

void MotionPlanner::VelocityTracker::Init()
{
    SetVelocityAcc(context->config->ratedVelocityAcc);
}

void MotionPlanner::VelocityTracker::SetVelocityAcc(int32_t _velocityAcc)
{
    velocityAcc = _velocityAcc;
}

void MotionPlanner::VelocityTracker::NewTask(int32_t _realVelocity)
{
    velocityIntegral = 0;
    trackVelocity = _realVelocity;
}

// 由加速度得到跟踪速度与目标速度比较，得到真实速度
void MotionPlanner::VelocityTracker::CalcSoftGoal(int32_t _goalVelocity)
{
    int32_t deltaVelocity = _goalVelocity - trackVelocity;// 目标速度减跟踪速度

    if (deltaVelocity == 0)
    {
        trackVelocity = _goalVelocity;
    }
    else if (deltaVelocity > 0)
    {
        if (trackVelocity >= 0)// 跟踪速度大于或等于0但小于目标速度，进行正向加速度直到超过目标速度
        {
            CalcVelocityIntegral(velocityAcc);
            if (trackVelocity >= _goalVelocity)
            {
                velocityIntegral = 0;
                trackVelocity = _goalVelocity;
            }
        }
        else// 跟踪速度小于0但目标速度大于0，先把跟踪速度正向加速到大于0，再把跟踪速度和加速度的累加值0，回到从0开始的加速阶段
        {
            CalcVelocityIntegral(velocityAcc);
            if (trackVelocity >= 0)
            {
                velocityIntegral = 0;
                trackVelocity = 0;
            }
        }
    }
    else if (deltaVelocity < 0)
    {
        if (trackVelocity <= 0)// 跟踪速度比目标速度大，且都是负的，使用负加速度进行加速
        {
            CalcVelocityIntegral(-velocityAcc);
            if (trackVelocity <= _goalVelocity)
            {
                velocityIntegral = 0;
                trackVelocity = _goalVelocity;
            }
        }
        else// 跟踪速度为正，目标速度为负，先把跟踪速度减到0，再进行上面步骤
        {
            CalcVelocityIntegral(-velocityAcc);
            if (trackVelocity <= 0)
            {
                velocityIntegral = 0;
                trackVelocity = 0;
            }
        }
    }

    goVelocity = (int32_t)trackVelocity;
}

// 由加速度计算跟踪速度算法
void MotionPlanner::VelocityTracker::CalcVelocityIntegral(int32_t _velocity)
{
    velocityIntegral += _velocity;                                    // 加速度累加得到变化速度
    trackVelocity += velocityIntegral / context->CONTROL_FREQUENCY;   // 取整并累加得到跟踪速度，单位是50us
    velocityIntegral = velocityIntegral % context->CONTROL_FREQUENCY; // 取余并累计得到变化速度
}

void MotionPlanner::PositionTracker::Init()
{
    SetVelocityAcc(context->config->ratedVelocityAcc);

}

void MotionPlanner::PositionTracker::SetVelocityAcc(int32_t value)
{
    velocityUpAcc = value;
    velocityDownAcc = value;
    quickVelocityDownAcc = 0.5f / (float)velocityDownAcc;
}

void MotionPlanner::PositionTracker::NewTask(int32_t real_location, int32_t real_speed)
{
    velocityIntegral = 0;
    trackVelocity = real_speed;
    positionIntegral = 0;
    trackPosition = real_location;
}

void MotionPlanner::PositionTracker::CalcSoftGoal(int32_t _goalPosition)
{
    int32_t deltaPosition = _goalPosition - trackPosition; // 目标位置与跟踪位置的差值
    // Position is quantized to one subdivision. Once inside that final quantum,
    // keep the published position while ramping velocity down; otherwise the
    // fractional integral can carry it one step past the goal on every approach.
    const bool holdFinalQuantum=deltaPosition==0 && velocityDownAcc>0 &&
        int64_t(trackVelocity)*trackVelocity<=int64_t(2)*velocityDownAcc;

    if (deltaPosition == 0)
    {
        if (trackVelocity == 0)
        {
            velocityIntegral = 0;
            trackVelocity = 0;
            positionIntegral = 0;
        }
        else if (trackVelocity > 0)
        {
            CalcVelocityIntegral(-velocityDownAcc);
            if (trackVelocity <= 0)
            {
                velocityIntegral = 0;
                trackVelocity = 0;
            }
        }
        else if (trackVelocity < 0)
        {
            CalcVelocityIntegral(velocityDownAcc);
            if (trackVelocity >= 0)
            {
                velocityIntegral = 0;
                trackVelocity = 0;
            }
        }
    }
    else
    {
        if (trackVelocity == 0)
        {
            if (deltaPosition > 0)
            {
                CalcVelocityIntegral(velocityUpAcc);
            }
            else
            {
                CalcVelocityIntegral(-velocityUpAcc);
            }
        }
        else if ((deltaPosition > 0) && (trackVelocity > 0))
        {
            if (trackVelocity <= context->config->ratedVelocity)
            {
                auto need_down_location = (int32_t)((float)trackVelocity *
                                                    (float)trackVelocity *
                                                    (float)quickVelocityDownAcc);
                if (fabs(deltaPosition) > need_down_location)
                {
                    if (trackVelocity < context->config->ratedVelocity)
                    {
                        CalcVelocityIntegral(velocityUpAcc);
                        if (trackVelocity >= context->config->ratedVelocity)
                        {
                            velocityIntegral = 0;
                            trackVelocity = context->config->ratedVelocity;
                        }
                    }
                    else if (trackVelocity > context->config->ratedVelocity)
                    {
                        CalcVelocityIntegral(-velocityDownAcc);
                    }
                }
                else
                {
                    CalcVelocityIntegral(-velocityDownAcc);
                    if (trackVelocity <= 0)
                    {
                        velocityIntegral = 0;
                        trackVelocity = 0;
                    }
                }
            }
            else
            {
                CalcVelocityIntegral(-velocityDownAcc);
                if (trackVelocity <= 0)
                {
                    velocityIntegral = 0;
                    trackVelocity = 0;
                }
            }
        }
        else if ((deltaPosition < 0) && (trackVelocity < 0))
        {
            if (trackVelocity >= -context->config->ratedVelocity)
            {
                auto need_down_location = (int32_t)((float)trackVelocity *
                                                    (float)trackVelocity *
                                                    (float)quickVelocityDownAcc);
                if (fabs(deltaPosition) > need_down_location)
                {
                    if (trackVelocity > -context->config->ratedVelocity)
                    {
                        CalcVelocityIntegral(-velocityUpAcc);
                        if (trackVelocity <= -context->config->ratedVelocity)
                        {
                            velocityIntegral = 0;
                            trackVelocity = -context->config->ratedVelocity;
                        }
                    }
                    else if (trackVelocity < -context->config->ratedVelocity)
                    {
                        CalcVelocityIntegral(velocityDownAcc);
                    }
                }
                else
                {
                    CalcVelocityIntegral(velocityDownAcc);
                    if (trackVelocity >= 0)
                    {
                        velocityIntegral = 0;
                        trackVelocity = 0;
                    }
                }
            }
            else
            {
                CalcVelocityIntegral(velocityDownAcc);
                if (trackVelocity >= 0)
                {
                    velocityIntegral = 0;
                    trackVelocity = 0;
                }
            }
        }
        else if ((deltaPosition < 0) && (trackVelocity > 0))
        {
            CalcVelocityIntegral(-velocityDownAcc);
            if (trackVelocity <= 0)
            {
                velocityIntegral = 0;
                trackVelocity = 0;
            }
        }
        else if (((deltaPosition > 0) && (trackVelocity < 0)))
        {
            CalcVelocityIntegral(velocityDownAcc);
            if (trackVelocity >= 0)
            {
                velocityIntegral = 0;
                trackVelocity = 0;
            }
        }
    }

    if(holdFinalQuantum)positionIntegral=0;
    else CalcPositionIntegral(trackVelocity);

    go_location = (int32_t)trackPosition;
    go_velocity = (int32_t)trackVelocity;
}

void MotionPlanner::PositionTracker::CalcPositionIntegral(int32_t value)
{
    positionIntegral += value;
    trackPosition += positionIntegral / context->CONTROL_FREQUENCY;
    positionIntegral = positionIntegral % context->CONTROL_FREQUENCY;
}

void MotionPlanner::PositionTracker::CalcVelocityIntegral(int32_t value)
{
    velocityIntegral += value;
    trackVelocity += velocityIntegral / context->CONTROL_FREQUENCY;
    velocityIntegral = velocityIntegral % context->CONTROL_FREQUENCY;
}

void MotionPlanner::PositionInterpolator::Init()
{
}

void MotionPlanner::PositionInterpolator::NewTask(int32_t _realPosition, int32_t _realVelocity)
{
    recordPosition = _realPosition;
    recordPositionLast = _realPosition;
    estPosition = _realPosition;
    estVelocity = _realVelocity;
}

void MotionPlanner::PositionInterpolator::CalcSoftGoal(int32_t _goalPosition)
{
    recordPositionLast = recordPosition;
    recordPosition = _goalPosition;

    estPositionIntegral += (((recordPosition - recordPositionLast) * context->CONTROL_FREQUENCY) + ((estVelocity << 6) - estVelocity));
    estVelocity = estPositionIntegral >> 6;
    estPositionIntegral -= (estVelocity << 6);

    estPosition = recordPosition;

    goPosition = estPosition;
    goVelocity = estVelocity;
}

void MotionPlanner::TrajectoryTracker::SetSlowDownVelocityAcc(int32_t value)
{
    velocityDownAcc = value;
}

void MotionPlanner::TrajectoryTracker::NewTask(int32_t real_location, int32_t real_speed)
{
    updateTime = 0;
    overtimeFlag = false;
    dynamicVelocityAccRemainder = 0;
    velocityNow = real_speed;
    velovityNowRemainder = 0;
    positionNow = real_location;
}

void MotionPlanner::TrajectoryTracker::CalcSoftGoal(int32_t _goalPosition, int32_t _goalVelocity)
{
    if (_goalVelocity != recordVelocity || _goalPosition != recordPosition)
    {
        updateTime = 0;
        recordVelocity = _goalVelocity;
        recordPosition = _goalPosition;

        dynamicVelocityAcc = (int32_t)((float)(_goalVelocity + velocityNow) *
                                       (float)(_goalVelocity - velocityNow) /
                                       (float)(2 * (_goalPosition - positionNow)));

        overtimeFlag = false;
    }
    else
    {
        if (updateTime >= (updateTimeout * 1000))
            overtimeFlag = true;
        else
            updateTime += context->CONTROL_PERIOD;
    }

    if (overtimeFlag)
    {
        if (velocityNow == 0)
        {
            dynamicVelocityAccRemainder = 0;
        }
        else if (velocityNow > 0)
        {
            CalcVelocityIntegral(-velocityDownAcc);
            if (velocityNow <= 0)
            {
                dynamicVelocityAccRemainder = 0;
                velocityNow = 0;
            }
        }
        else
        {
            CalcVelocityIntegral(velocityDownAcc);
            if (velocityNow >= 0)
            {
                dynamicVelocityAccRemainder = 0;
                velocityNow = 0;
            }
        }
    }
    else
    {
        CalcVelocityIntegral(dynamicVelocityAcc);
    }

    CalcPositionIntegral(velocityNow);

    goPosition = positionNow;
    goVelocity = velocityNow;
}

void MotionPlanner::TrajectoryTracker::CalcVelocityIntegral(int32_t value)
{
    dynamicVelocityAccRemainder += value; // sum up last remainder
    velocityNow += dynamicVelocityAccRemainder / context->CONTROL_FREQUENCY;
    dynamicVelocityAccRemainder = dynamicVelocityAccRemainder % context->CONTROL_FREQUENCY; // calc remainder
}

void MotionPlanner::TrajectoryTracker::CalcPositionIntegral(int32_t value)
{
    velovityNowRemainder += value;
    positionNow += velovityNowRemainder / context->CONTROL_FREQUENCY;
    velovityNowRemainder = velovityNowRemainder % context->CONTROL_FREQUENCY;
}

void MotionPlanner::TrajectoryTracker::Init(int32_t _updateTimeout)
{
    // SetSlowDownVelocityAcc(context->config->ratedVelocityAcc / 10);
    SetSlowDownVelocityAcc(context->config->ratedVelocityAcc);
    updateTimeout = _updateTimeout;
}

void MotionPlanner::AttachConfig(MotionPlanner::Config_t *_config)
{
    config = _config;

    currentTracker.Init();
    velocityTracker.Init();
    positionTracker.Init();
    positionInterpolator.Init();
    trajectoryTracker.Init(200);
}
