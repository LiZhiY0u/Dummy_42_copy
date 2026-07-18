#include "common_inc.h"
#include "configurations.h"
#include <can.h>
#include <cstring>

namespace
{
float ReadFloat32(const uint8_t *data)
{
    float value;
    std::memcpy(&value, data, sizeof(value));
    return value;
}

int32_t ReadInt32(const uint8_t *data)
{
    int32_t value;
    std::memcpy(&value, data, sizeof(value));
    return value;
}

uint32_t ReadUint32(const uint8_t *data)
{
    uint32_t value;
    std::memcpy(&value, data, sizeof(value));
    return value;
}

bool HasCommandLength(uint8_t command, uint32_t length)
{
    switch (command)
    {
    case 0x06:
    case 0x07:
        return length >= 8;
    case 0x01:
    case 0x03:
    case 0x04:
    case 0x05:
    case 0x11:
    case 0x12:
    case 0x13:
    case 0x14:
    case 0x16:
    case 0x17:
    case 0x18:
    case 0x19:
    case 0x1A:
    case 0x1B:
        return length >= 4;
    default:
        return true;
    }
}
}

extern Motor motor;

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;


CAN_TxHeaderTypeDef txHeader =
    {
        .StdId = 0x00,
        .ExtId = 0x00,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = 8,
        .TransmitGlobalTime = DISABLE};

void CAN_Send(CAN_TxHeaderTypeDef* pHeader, uint8_t* data)
{
    if (HAL_CAN_AddTxMessage(&hcan, pHeader, data, &TxMailbox) != HAL_OK)
    {
        Error_Handler();
    }
}

// void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* CanHandle)
// {
//     /* Get RX message */
//     if (HAL_CAN_GetRxMessage(CanHandle, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
//     {
//         /* Reception Error */
//         Error_Handler();
//     }

//     uint8_t id = (RxHeader.StdId >> 7); // 4Bits ID & 7Bits Msg
//     uint8_t cmd = RxHeader.StdId & 0x7F; // 4Bits ID & 7Bits Msg
//     if (id == 0 || id == 1)
//     {
//         OnCanCmd(cmd, RxData, RxHeader.DLC);
//     }
// }



void OnCanCmd(uint8_t _cmd, uint8_t *_data, uint32_t _len)
{
    if (_data == nullptr || !HasCommandLength(_cmd, _len))
        return;

    float tmpF;
    int32_t tmpI;

    switch (_cmd)
    {
    // 0x00~0x0F No Memory CMDs
    case 0x01: // Enable Motor
        motor.controller->requestMode = (ReadUint32(_data) == 1) ? Motor::MODE_COMMAND_VELOCITY : Motor::MODE_STOP;
        break;
    case 0x02: // Do Calibration
        motor.controller->SetCtrlMode(Motor::MODE_STOP);
        tb67h450_base.Sleep();
        encoder_calibrator_base.isTriggered = true;
        break;
    case 0x03: // Set Current SetPoint
        if (motor.controller->modeRunning != Motor::MODE_COMMAND_CURRENT)
            motor.controller->SetCtrlMode(Motor::MODE_COMMAND_CURRENT);
        motor.controller->SetCurrentSetPoint((int32_t)(ReadFloat32(_data) * 1000));
        break;
    case 0x04: // Set Velocity SetPoint
        if (motor.controller->modeRunning != Motor::MODE_COMMAND_VELOCITY)
        {
            motor.config.motionParams.ratedVelocity = boardConfig.velocityLimit;
            motor.controller->SetCtrlMode(Motor::MODE_COMMAND_VELOCITY);
        }
        motor.controller->SetVelocitySetPoint(
            (int32_t)(ReadFloat32(_data) *
                      (float)motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS));
        break;
    case 0x05: // Set Position SetPoint
        if (motor.controller->modeRunning != Motor::MODE_COMMAND_POSITION)
        {
            motor.config.motionParams.ratedVelocity = boardConfig.velocityLimit;
            motor.controller->SetCtrlMode(Motor::MODE_COMMAND_POSITION);
        }
        motor.controller->SetPositionSetPoint(
            (int32_t)(ReadFloat32(_data) * (float)motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS));
        if (_len >= 5 && _data[4]) // Need Position & Finished ACK
        {
            tmpF = motor.controller->GetPosition();
            auto *b = (unsigned char *)&tmpF;
            for (int i = 0; i < 4; i++)
                _data[i] = *(b + i);
            _data[4] = motor.controller->state == Motor::STATE_FINISH ? 1 : 0;
            txHeader.StdId = (boardConfig.canNodeId << 7) | 0x23;
            CAN_Send(&txHeader, _data);
        }
        break;
    case 0x06: // Set Position with Time
        if (motor.controller->modeRunning != Motor::MODE_COMMAND_POSITION)
            motor.controller->SetCtrlMode(Motor::MODE_COMMAND_POSITION);
        motor.controller->SetPositionSetPointWithTime(
            (int32_t)(ReadFloat32(_data) * (float)motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS),
            ReadFloat32(_data + 4));
        break;
    case 0x07: // Set Position with Velocity-Limit
    {
        if (motor.controller->modeRunning != Motor::MODE_COMMAND_POSITION)
        {
            motor.config.motionParams.ratedVelocity = boardConfig.velocityLimit;
            motor.controller->SetCtrlMode(Motor::MODE_COMMAND_POSITION);
        }
        motor.config.motionParams.ratedVelocity =
            (int32_t)(ReadFloat32(_data + 4) * (float)motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
        motor.controller->SetPositionSetPoint(
            (int32_t)(ReadFloat32(_data) * (float)motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS));
        // Always Need Position & Finished ACK
        tmpF = motor.controller->GetPosition();
        auto *b = (unsigned char *)&tmpF;
        for (int i = 0; i < 4; i++)
            _data[i] = *(b + i);
        _data[4] = motor.controller->state == Motor::STATE_FINISH ? 1 : 0;
        txHeader.StdId = (boardConfig.canNodeId << 7) | 0x23;
        CAN_Send(&txHeader, _data);
    }
    break;

        // 0x10~0x1F CMDs with Memory
    case 0x11: // Set Node-ID and Store to EEPROM
    {
        const uint32_t nodeId = ReadUint32(_data);
        if (nodeId > 0x0FU)
            break;
        boardConfig.canNodeId = nodeId;
        if (_len >= 5 && _data[4])
            boardConfig.configStatus = CONFIG_COMMIT;
        break;
    }
    case 0x12: // Set Current-Limit and Store to EEPROM
        motor.config.motionParams.ratedCurrent = (int32_t)(ReadFloat32(_data) * 1000);
        boardConfig.currentLimit = motor.config.motionParams.ratedCurrent;
        if (_len >= 5 && _data[4])
            boardConfig.configStatus = CONFIG_COMMIT;
        break;
    case 0x13: // Set Velocity-Limit and Store to EEPROM
        motor.config.motionParams.ratedVelocity =
            (int32_t)(ReadFloat32(_data) *
                      (float)motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS);
        boardConfig.velocityLimit = motor.config.motionParams.ratedVelocity;
        if (_len >= 5 && _data[4])
            boardConfig.configStatus = CONFIG_COMMIT;
        break;
    case 0x14: // Set Acceleration （and Store to EEPROM）
        tmpF = ReadFloat32(_data) * (float)motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;

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
        boardConfig.enableMotorOnBoot = (ReadUint32(_data) == 1);
        if (_len >= 5 && _data[4])
            boardConfig.configStatus = CONFIG_COMMIT;
        break;
    case 0x17: // Set DCE Kp
        motor.config.ctrlParams.dce.kp = ReadInt32(_data);
        boardConfig.dce_kp = motor.config.ctrlParams.dce.kp;
        if (_len >= 5 && _data[4])
            boardConfig.configStatus = CONFIG_COMMIT;
        break;
    case 0x18: // Set DCE Kv
        motor.config.ctrlParams.dce.kv = ReadInt32(_data);
        boardConfig.dce_kv = motor.config.ctrlParams.dce.kv;
        if (_len >= 5 && _data[4])
            boardConfig.configStatus = CONFIG_COMMIT;
        break;
    case 0x19: // Set DCE Ki
        motor.config.ctrlParams.dce.ki = ReadInt32(_data);
        boardConfig.dce_ki = motor.config.ctrlParams.dce.ki;
        if (_len >= 5 && _data[4])
            boardConfig.configStatus = CONFIG_COMMIT;
        break;
    case 0x1A: // Set DCE Kd
        motor.config.ctrlParams.dce.kd = ReadInt32(_data);
        boardConfig.dce_kd = motor.config.ctrlParams.dce.kd;
        if (_len >= 5 && _data[4])
            boardConfig.configStatus = CONFIG_COMMIT;
        break;
    case 0x1B: // Set Enable Stall-Protect
        motor.config.ctrlParams.stallProtectSwitch = (ReadUint32(_data) == 1);
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
        CAN_Send(&txHeader, _data);
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
        CAN_Send(&txHeader, _data);
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
        CAN_Send(&txHeader, _data);
    }
    break;
    case 0x24: // Get Offset
    {
        tmpI = motor.config.motionParams.encoderHomeOffset;
        auto *b = (unsigned char *)&tmpI;
        for (int i = 0; i < 4; i++)
            _data[i] = *(b + i);
        txHeader.StdId = (boardConfig.canNodeId << 7) | 0x24;
        CAN_Send(&txHeader, _data);
    }
    break;

    case 0x7e: // Erase Configs
        boardConfig.configStatus = CONFIG_RESTORE;
        break;
    case 0x7f: // Reboot
        HAL_NVIC_SystemReset();
        break;
    default:
        break;
    }
}
