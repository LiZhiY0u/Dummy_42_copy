#include "common_inc.h"
#include "configurations.h"
#include "Platform/Utils/st_hardware.h"
#include <tim.h>
#include "protocol_safety.h"
#include "storage_integrity.h"
#include <cstring>
#include <cstddef>


/* Component Definitions -----------------------------------------------------*/
BoardConfig_t boardConfig;
Motor motor;
TB67H450 tb67H450;
MT6816 mt6816;
EncoderCalibrator encoderCalibrator(&motor);
Button button1(1, 1000), button2(2, 3000);
void OnButton1Event(Button::Event _event);
void OnButton2Event(Button::Event _event);
Led statusLed;


namespace {
constexpr uint32_t ConfigMagic = 0x31474643; // CFG1
struct ConfigRecord {
    uint32_t magic, version;
    BoardConfig_t config;
    uint32_t crc;
};
bool ValidConfig(const BoardConfig_t& c) {
    using namespace ProtocolSafety;
    return c.configStatus == CONFIG_OK && c.canNodeId >= 1 && c.canNodeId <= 15 &&
        c.defaultMode >= Motor::MODE_COMMAND_POSITION && c.defaultMode <= Motor::MODE_COMMAND_CURRENT &&
        c.currentLimit > 0 && c.currentLimit <= MaxCurrent &&
        c.velocityLimit > 0 && c.velocityLimit <= MaxVelocity &&
        c.velocityAcc > 0 && c.velocityAcc <= MaxAcceleration &&
        c.calibrationCurrent > 0 && c.calibrationCurrent <= MaxCurrent &&
        c.encoderHomeOffset > -51200 && c.encoderHomeOffset < 51200 &&
        c.dce_kp >= 0 && c.dce_kp <= 1000 && c.dce_kv >= 0 && c.dce_kv <= 1000 &&
        c.dce_ki >= 0 && c.dce_ki <= 1000 && c.dce_kd >= 0 && c.dce_kd <= 1000;
}
}

extern "C" void FirmwareSafeStop()
{
    // Fatal errors may occur before C++ motor initialization. Use direct IO.
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIOA->BRR = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    if (__HAL_RCC_TIM2_IS_CLK_ENABLED()) {
        TIM2->CCR3 = 0;
        TIM2->CCR4 = 0;
    }
}

/* Main Entry ----------------------------------------------------------------*/
void Main()
{
    uint64_t serialNum = GetSerialNumber();
    uint16_t defaultNodeID = 1;
    // Change below to fit your situation
    switch (serialNum)
    {
        case 431466563640: //J1
            defaultNodeID = 1;
            break;
        case 384624576568: //J2
            defaultNodeID = 2;
            break;
        case 384290670648: //J3
            defaultNodeID = 3;
            break;
        case 431531051064: //J4
            defaultNodeID = 4;
            break;
        case 431466760248: //J5
            defaultNodeID = 5;
            break;
        case 431484848184: //J6
            defaultNodeID = 6;
            break;
        default:
            break;
    }

    
    /*---------- Apply EEPROM Settings ----------*/
    // Setting priority is EEPROM > Motor.h
    EEPROM eeprom;
    ConfigRecord record{};
    eeprom.get(0, record);
    boardConfig = record.config;
    if (record.magic != ConfigMagic || record.version != 1 ||
        record.crc != StorageIntegrity::Crc32(&record, offsetof(ConfigRecord, crc)) ||
        !ValidConfig(boardConfig)) // 没接收过任何数据的时候用默认配置，use default settings
    {
        boardConfig = BoardConfig_t{
            .configStatus = CONFIG_OK,
            .canNodeId = defaultNodeID,
            .encoderHomeOffset = 0,
            .defaultMode = Motor::MODE_COMMAND_POSITION,
            .currentLimit = 1 * 1000,    // A
            .velocityLimit = 30 * motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS, // r/s,51200*30
            .velocityAcc = 100 * motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS,   // r/s^2,5120000
            .calibrationCurrent=2000,
            .dce_kp = 200,
            .dce_kv = 80,
            .dce_ki = 300,
            .dce_kd = 250,
            .enableMotorOnBoot=false,
            .enableStallProtect=false
        };
        // Persist lazily after initialization; legacy records reset to defaults.
        boardConfig.configStatus = CONFIG_COMMIT;
    }
    /*--------------把配置加载到motor专属配置内，每次复位可能不一样----------------------*/
    motor.config.motionParams.encoderHomeOffset = boardConfig.encoderHomeOffset;
    motor.config.motionParams.ratedCurrent = boardConfig.currentLimit;
    motor.config.motionParams.ratedVelocity = boardConfig.velocityLimit;
    motor.config.motionParams.ratedVelocityAcc = boardConfig.velocityAcc;
    motor.motionPlanner.velocityTracker.SetVelocityAcc(boardConfig.velocityAcc);
    motor.motionPlanner.positionTracker.SetVelocityAcc(boardConfig.velocityAcc);
    motor.config.motionParams.caliCurrent = boardConfig.calibrationCurrent;
    motor.config.ctrlParams.dce.kp = boardConfig.dce_kp;
    motor.config.ctrlParams.dce.kv = boardConfig.dce_kv;
    motor.config.ctrlParams.dce.ki = boardConfig.dce_ki;
    motor.config.ctrlParams.dce.kd = boardConfig.dce_kd;
    motor.config.ctrlParams.stallProtectSwitch = boardConfig.enableStallProtect;


    /*---------------- Init Motor ----------------*/
    motor.AttachDriver(&tb67H450);
    motor.AttachEncoder(&mt6816);
    motor.controller->Init();
    motor.driver->Init();//io和pwm初始化
    motor.encoder->Init();//spi初始化，获取一次mt6816数据


    /*------------- Init peripherals -------------*/
    button1.SetOnEventListener(OnButton1Event);
    button2.SetOnEventListener(OnButton2Event);


    /*------- Start Close-Loop Control Tick，初始化结束才开启tim中断 ------*/
    HAL_Delay(100);
    HAL_TIM_Base_Start_IT(&htim1);  // 100Hz
    HAL_TIM_Base_Start_IT(&htim4);  // 20kHz

    if (button1.IsPressed() && button2.IsPressed()) {
        motor.Stop();
        encoderCalibrator.isTriggered = true;
    }


    for (;;)
    {
        encoderCalibrator.TickMainLoop();//完成校准后便没用了


        CAN_ProcessPending();
        UartProcess();

        const bool commit = boardConfig.configStatus == CONFIG_COMMIT;
        const bool restore = boardConfig.configStatus == CONFIG_RESTORE;
        if ((commit || restore) && !encoderCalibrator.isTriggered && !motor.HasFault() &&
            motor.controller->requestMode == Motor::MODE_STOP && motor.controller->modeRunning == Motor::MODE_STOP)
        {
            motor.BeginMaintenance();
            if (restore) {
                // Invalidate the record, then reboot into default settings.
                record = ConfigRecord{};
            } else {
                boardConfig.configStatus = CONFIG_OK;
                record = ConfigRecord{};
                record.magic = ConfigMagic;
                record.version = 1;
                record.config = boardConfig;
                record.crc = StorageIntegrity::Crc32(&record, offsetof(ConfigRecord, crc));
            }
            eeprom.put(0, record);
            // Read physical Flash, not EEPROM's RAM cache.
            if (std::memcmp(reinterpret_cast<const void*>(0x0801fc00), &record, sizeof record) != 0)
                motor.LatchStorageFault();
            else if (restore)
                HAL_NVIC_SystemReset();
            motor.EndMaintenance();
        }
    }
}


/* Event Callbacks -----------------------------------------------------------*/
extern "C" void Tim1Callback100Hz()
{
    __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);

    button1.Tick(10);
    button2.Tick(10);
    statusLed.Tick(10, motor.controller->state);
}

//完成校准后只进行20khz的电机任务
extern "C" void Tim4Callback20kHz()
{
    __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);

    if (motor.IsMaintaining()) { motor.driver->Sleep(); return; }
    if (encoderCalibrator.isTriggered)
        encoderCalibrator.Tick20kHz();
    else
        motor.Tick20kHz();
}


void OnButton1Event(Button::Event _event)
{
    if (motor.IsMaintaining()) return;
    switch (_event)
    {
        case ButtonBase::UP:
            break;
        case ButtonBase::DOWN:
            break;
        case ButtonBase::LONG_PRESS:
            HAL_NVIC_SystemReset();
            break;
        case ButtonBase::CLICK:
            if (motor.controller->modeRunning != Motor::MODE_STOP)
            {
                boardConfig.defaultMode = motor.controller->modeRunning;
                motor.controller->requestMode = Motor::MODE_STOP;
            } else
            {
                motor.controller->requestMode = static_cast<Motor::Mode_t>(boardConfig.defaultMode);
            }
            break;
    }
}


void OnButton2Event(Button::Event _event)
{
    if (motor.IsMaintaining()) return;
    switch (_event)
    {
        case ButtonBase::UP:
            break;
        case ButtonBase::DOWN:
            break;
        case ButtonBase::LONG_PRESS:
            switch (motor.controller->modeRunning)
            {
                case Motor::MODE_COMMAND_CURRENT:
                case Motor::MODE_PWM_CURRENT:
                    motor.controller->SetCurrentSetPoint(0);
                    break;
                case Motor::MODE_COMMAND_VELOCITY:
                case Motor::MODE_PWM_VELOCITY:
                    motor.controller->SetVelocitySetPoint(0);
                    break;
                case Motor::MODE_COMMAND_POSITION:
                case Motor::MODE_PWM_POSITION:
                    motor.controller->SetPositionSetPoint(0);
                    break;
                case Motor::MODE_COMMAND_Trajectory:
                case Motor::MODE_STEP_DIR:
                case Motor::MODE_STOP:
                    break;
            }
            break;
        case ButtonBase::CLICK:
            motor.controller->ClearStallFlag();
            break;
    }
}