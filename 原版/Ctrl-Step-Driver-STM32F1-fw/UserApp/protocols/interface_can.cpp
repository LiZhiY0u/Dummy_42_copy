#include "common_inc.h"
#include "configurations.h"
#include <can.h>
#include "protocol_safety.h"
#include "interrupt_guard.h"
using namespace ProtocolSafety;

extern Motor motor;
extern EncoderCalibrator encoderCalibrator;

CAN_TxHeaderTypeDef txHeader =
    {
        .StdId = 0x00,
        .ExtId = 0x00,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = 8,
        .TransmitGlobalTime = DISABLE};

void OnCanCmd(uint8_t _cmd, uint8_t *_data, uint32_t _len)
{
    if (!ValidCommand(_cmd, _data, _len)) return;
    InterruptGuard guard(_cmd != 0x06);
    // Ignore motion and configuration writes while calibrating or faulted.
    if ((encoderCalibrator.isTriggered || motor.HasFault() || motor.IsMaintaining()) && _cmd < 0x20)
    {
        if (_cmd == 0x01 && ReadUint(_data) == 0) encoderCalibrator.Cancel();
        return;
    }
    // Reject speed goals above the configured limit before changing mode.
    if (_cmd == 0x04 && std::fabs(ReadFloat(_data) * 51200.0f) > boardConfig.velocityLimit) return;
    if (_cmd >= 0x11 && _cmd <= 0x1b &&
        (motor.controller->requestMode != Motor::MODE_STOP || motor.controller->modeRunning != Motor::MODE_STOP)) return;
    float tmpF;
    int32_t tmpI;

    switch (_cmd)
    {
    // 0x00~0x0F No Memory CMDs
    case 0x01: // Enable Motor
        if (ReadUint(_data) == 0) motor.Stop();
        else motor.controller->SetCtrlMode(Motor::MODE_COMMAND_VELOCITY);
        break;
    case 0x02: // Do Calibration
        motor.Stop();
        encoderCalibrator.isTriggered = true;
        break;
    case 0x03: // Set Current SetPoint
        if (motor.controller->modeRunning != Motor::MODE_COMMAND_CURRENT)
            motor.controller->SetCtrlMode(Motor::MODE_COMMAND_CURRENT);
        motor.controller->SetCurrentSetPoint((int32_t)(ReadFloat(_data) * 1000));
        break;
    case 0x04: // Set Velocity SetPoint
        if (motor.controller->modeRunning != Motor::MODE_COMMAND_VELOCITY)
        {
            motor.config.motionParams.ratedVelocity = boardConfig.velocityLimit;
            motor.controller->SetCtrlMode(Motor::MODE_COMMAND_VELOCITY);
        }
        motor.controller->SetVelocitySetPoint(
            (int32_t)(ReadFloat(_data) *
                      (float)motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS));
        break;
    case 0x05: // Set Position SetPoint
        if (motor.controller->modeRunning != Motor::MODE_COMMAND_POSITION)
        {
            motor.config.motionParams.ratedVelocity = boardConfig.velocityLimit;
            motor.controller->SetCtrlMode(Motor::MODE_COMMAND_POSITION);
        }
        motor.controller->SetPositionSetPoint(
            (int32_t)(ReadFloat(_data) * (float)motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS));
        if (_len >= 5 && _data[4]) // Need Position & Finished ACK
        {
            tmpF = motor.controller->GetPosition();
            auto *b = (unsigned char *)&tmpF;
            for (int i = 0; i < 4; i++)
                _data[i] = *(b + i);
            _data[4] = motor.controller->state == Motor::STATE_FINISH ? 1 : 0;
            txHeader.StdId = (boardConfig.canNodeId << 7) | 0x23;
            { uint8_t reply[8]{};
                std::memcpy(reply, _data, 5);
                CAN_Send(&txHeader, reply); }
        }
        break;
    case 0x06: // Set Position with Time (calculation outside IRQ mask)
        motor.controller->SetPositionSetPointWithTime(
            (int32_t)(ReadFloat(_data) * (float)motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS),
            ReadFloat(_data + 4));
        // All eight bytes belong to position and time; no ACK flag exists.
        break;
    case 0x07: // Set Position with Velocity-Limit
    {
        if (motor.controller->modeRunning != Motor::MODE_COMMAND_POSITION)
        {
            motor.config.motionParams.ratedVelocity = boardConfig.velocityLimit;
            motor.controller->SetCtrlMode(Motor::MODE_COMMAND_POSITION);
        }
        motor.config.motionParams.ratedVelocity =
            (int32_t)(ReadFloat(_data + 4) * (float)motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
        if (motor.config.motionParams.ratedVelocity > boardConfig.velocityLimit)
            motor.config.motionParams.ratedVelocity = boardConfig.velocityLimit;
        motor.controller->SetPositionSetPoint(
            (int32_t)(ReadFloat(_data) * (float)motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS));
        // Always Need Position & Finished ACK
        tmpF = motor.controller->GetPosition();
        auto *b = (unsigned char *)&tmpF;
        for (int i = 0; i < 4; i++)
            _data[i] = *(b + i);
        _data[4] = motor.controller->state == Motor::STATE_FINISH ? 1 : 0;
        txHeader.StdId = (boardConfig.canNodeId << 7) | 0x23;
        { uint8_t reply[8]{};
                std::memcpy(reply, _data, 5);
                CAN_Send(&txHeader, reply); }
    }
    break;

        // 0x10~0x1F CMDs with Memory
    case 0x11: // Set Node-ID and Store to EEPROM
        boardConfig.canNodeId = ReadUint(_data);
        if (_len >= 5 && _data[4])
            boardConfig.configStatus = CONFIG_COMMIT;
        break;
    case 0x12: // Set Current-Limit and Store to EEPROM
        motor.config.motionParams.ratedCurrent = (int32_t)(ReadFloat(_data) * 1000);
        boardConfig.currentLimit = motor.config.motionParams.ratedCurrent;
        if (_len >= 5 && _data[4])
            boardConfig.configStatus = CONFIG_COMMIT;
        break;
    case 0x13: // Set Velocity-Limit and Store to EEPROM
        motor.config.motionParams.ratedVelocity =
            (int32_t)(ReadFloat(_data) *
                      (float)motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
        boardConfig.velocityLimit = motor.config.motionParams.ratedVelocity;
        if (_len >= 5 && _data[4])
            boardConfig.configStatus = CONFIG_COMMIT;
        break;
    case 0x14: // Set Acceleration （and Store to EEPROM）
        tmpF = ReadFloat(_data) * (float)motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;

        motor.config.motionParams.ratedVelocityAcc = (int32_t)tmpF;
        motor.motionPlanner.velocityTracker.SetVelocityAcc((int32_t)tmpF);
        motor.motionPlanner.positionTracker.SetVelocityAcc((int32_t)tmpF);
        boardConfig.velocityAcc = motor.config.motionParams.ratedVelocityAcc;
        if (_len >= 5 && _data[4])
            boardConfig.configStatus = CONFIG_COMMIT;
        break;
    case 0x15: // Apply Home-Position and Store to EEPROM
        motor.controller->ApplyPosAsHomeOffset();
        boardConfig.encoderHomeOffset = motor.config.motionParams.encoderHomeOffset %
                                        motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
        boardConfig.configStatus = CONFIG_COMMIT;
        break;
    case 0x16: // Set Auto-Enable and Store to EEPROM
        boardConfig.enableMotorOnBoot = (ReadUint(_data) == 1);
        if (_len >= 5 && _data[4])
            boardConfig.configStatus = CONFIG_COMMIT;
        break;
    case 0x17: // Set DCE Kp
        motor.config.ctrlParams.dce.kp = ReadInt(_data);
        boardConfig.dce_kp = motor.config.ctrlParams.dce.kp;
        if (_len >= 5 && _data[4])
            boardConfig.configStatus = CONFIG_COMMIT;
        break;
    case 0x18: // Set DCE Kv
        motor.config.ctrlParams.dce.kv = ReadInt(_data);
        boardConfig.dce_kv = motor.config.ctrlParams.dce.kv;
        if (_len >= 5 && _data[4])
            boardConfig.configStatus = CONFIG_COMMIT;
        break;
    case 0x19: // Set DCE Ki
        motor.config.ctrlParams.dce.ki = ReadInt(_data);
        boardConfig.dce_ki = motor.config.ctrlParams.dce.ki;
        if (_len >= 5 && _data[4])
            boardConfig.configStatus = CONFIG_COMMIT;
        break;
    case 0x1A: // Set DCE Kd
        motor.config.ctrlParams.dce.kd = ReadInt(_data);
        boardConfig.dce_kd = motor.config.ctrlParams.dce.kd;
        if (_len >= 5 && _data[4])
            boardConfig.configStatus = CONFIG_COMMIT;
        break;
    case 0x1B: // Set Enable Stall-Protect
        motor.config.ctrlParams.stallProtectSwitch = (ReadUint(_data) == 1);
        boardConfig.enableStallProtect = motor.config.ctrlParams.stallProtectSwitch;
        if (_len >= 5 && _data[4])
            boardConfig.configStatus = CONFIG_COMMIT;
        break;

        // 0x20~0x2F Inquiry CMDs
    case 0x21: // Get Current
    {
        tmpF = motor.controller->GetFocCurrent();
        auto *b = (unsigned char *)&tmpF;
        for (int i = 0; i < 4; i++)
            _data[i] = *(b + i);
        _data[4] = (motor.controller->state == Motor::STATE_FINISH ? 1 : 0);

        txHeader.StdId = (boardConfig.canNodeId << 7) | 0x21;
        { uint8_t reply[8]{};
                std::memcpy(reply, _data, 5);
                CAN_Send(&txHeader, reply); }
    }
    break;
    case 0x22: // Get Velocity
    {
        tmpF = motor.controller->GetVelocity();
        auto *b = (unsigned char *)&tmpF;
        for (int i = 0; i < 4; i++)
            _data[i] = *(b + i);
        _data[4] = (motor.controller->state == Motor::STATE_FINISH ? 1 : 0);

        txHeader.StdId = (boardConfig.canNodeId << 7) | 0x22;
        { uint8_t reply[8]{};
                std::memcpy(reply, _data, 5);
                CAN_Send(&txHeader, reply); }
    }
    break;
    case 0x23: // Get Position
    {
        tmpF = motor.controller->GetPosition();
        auto *b = (unsigned char *)&tmpF;
        for (int i = 0; i < 4; i++)
            _data[i] = *(b + i);
        // Finished ACK
        _data[4] = motor.controller->state == Motor::STATE_FINISH ? 1 : 0;
        txHeader.StdId = (boardConfig.canNodeId << 7) | 0x23;
        { uint8_t reply[8]{};
                std::memcpy(reply, _data, 5);
                CAN_Send(&txHeader, reply); }
    }
    break;
    case 0x24: // Get Offset
    {
        tmpI = motor.config.motionParams.encoderHomeOffset;
        auto *b = (unsigned char *)&tmpI;
        for (int i = 0; i < 4; i++)
            _data[i] = *(b + i);
        txHeader.StdId = (boardConfig.canNodeId << 7) | 0x24;
        { uint8_t reply[8]{};
                std::memcpy(reply, _data, 5);
                CAN_Send(&txHeader, reply); }
    }
    break;

    case 0x7e: // Erase Configs
        boardConfig.configStatus = CONFIG_RESTORE;
        break;
    case 0x7f: // Reboot
        motor.Stop();
        HAL_NVIC_SystemReset();
        break;
    default:
        break;
    }
}
