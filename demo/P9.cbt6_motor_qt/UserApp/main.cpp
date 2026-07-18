#include "common_inc.h"
#include "configurations.h"

/*
sqrtf测试，计算平方根
*/

// extern "C"  void tim4callback(void);

extern uint8_t TxData[8];
extern uint8_t RxData[8];

extern CAN_TxHeaderTypeDef TxHeader;
extern CAN_RxHeaderTypeDef RxHeader;

uint16_t _b = 0, _g = 0, _p = 0;
uint8_t rx_buffer[128] = {0}, rxLen = 0;

BoardConfig_t boardConfig;
Motor motor;
MT6816Base mt6816_base((uint16_t *)(0x08017C00));
TB67H450Base tb67h450_base;
EncoderCalibratorBase encoder_calibrator_base;
ButtonBase button1(GPIOB, GPIO_PIN_2, 1, 1000), button2(GPIOB, GPIO_PIN_12, 2, 1000);

extern DMA_HandleTypeDef hdma_usart1_rx;
extern uint8_t _a;
extern uint16_t _v;

uint16_t aaa = 0;
uint8_t rec_buff[100] = {0};

extern "C" void Main()
{
    boardConfig = BoardConfig_t{
        .configStatus = CONFIG_OK,
        .canNodeId = 1,
        .encoderHomeOffset = 0,
        .defaultMode = Motor::MODE_COMMAND_POSITION,
        .currentLimit = 1 * 1000,
        .velocityLimit = 30 * motor.MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS,
        .velocityAcc = 1000000,
        .calibrationCurrent = 2000,
        .dce_kp = 200,
        .dce_kv = 80,
        .dce_ki = 300,
        .dce_kd = 250,
        .enableMotorOnBoot = false,
        .enableStallProtect = false};

    encoder_calibrator_base.isTriggered = false;
    encoder_calibrator_base.errorCode = EncoderCalibratorBase::CALI_NO_ERROR;
    encoder_calibrator_base.state = EncoderCalibratorBase::CALI_DISABLE;
    encoder_calibrator_base.goPosition = 0;
    encoder_calibrator_base.rcdX = 0;
    encoder_calibrator_base.rcdY = 0;
    encoder_calibrator_base.resultNum = 0;

    motor.AttachEncoder(&mt6816_base);
    motor.config.motionParams.encoderHomeOffset = boardConfig.encoderHomeOffset;
    motor.config.motionParams.caliCurrent = boardConfig.calibrationCurrent;
    motor.config.motionParams.ratedCurrent = boardConfig.currentLimit;
    motor.config.motionParams.ratedVelocity = boardConfig.velocityLimit;
    motor.config.motionParams.ratedVelocityAcc = boardConfig.velocityAcc;
    motor.config.ctrlParams.dce.kp = boardConfig.dce_kp;
    motor.config.ctrlParams.dce.kv = boardConfig.dce_kv;
    motor.config.ctrlParams.dce.ki = boardConfig.dce_ki;
    motor.config.ctrlParams.dce.kd = boardConfig.dce_kd;
    motor.config.ctrlParams.stallProtectSwitch = boardConfig.enableStallProtect;
    motor.motionPlanner.velocityTracker.SetVelocityAcc(boardConfig.velocityAcc);
    motor.motionPlanner.positionTracker.SetVelocityAcc(boardConfig.velocityAcc);

    mt6816_base.Init();
    motor.controller->Init();

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    tb67h450_base.Sleep();

    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rec_buff, sizeof(rec_buff)) == HAL_OK)
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);

    HAL_TIM_Base_Start_IT(&htim1);
    HAL_TIM_Base_Start_IT(&htim4);

    for (;;)
    {

        encoder_calibrator_base.TickMainLoop();

        // static uint32_t _a = 0;
        // _a++;
        // if (_a == 51200)
        // {
        //     _a = 0;
        // }
        // tb67h450_base.SetFocCurrentVector(_a, 1000);
        // HAL_Delay(100);

        // test1();
        // _g = mt6816_base.test1();
        // motor.encoder->test();
    }
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim1)
    {
        Upload_estvelocity();
    }
    if (htim == &htim4)
    {
        // if (goPosition <= 51200)
        // {
        //     // 测试：以2000mA电流跑一圈
        //     tb67h450_base.SetFocCurrentVector(goPosition, 1000);
        //     goPosition += 2;
        // }
        // mt6816_base.UpdateAngle();

        if (encoder_calibrator_base.isTriggered)
        {
            encoder_calibrator_base.Tick20kHz();
        }
        else
        {

            motor.Tick20kHz();
        }

        // tim4callback();
    }
}

extern "C" void tim4callback()
{
    // mt6816_base.UpdateAngle();
    _b++;
    if (_b == 1000)
    {
        _b = 0;
    }
    if (_b == 1)
    {
        _p++;
        if (_p == 1000)
        {
            _p = 0;
        }
    }
}

void test2()
{
    _g++;
    if (_g == 100)
    {
        _g = 0;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint32_t temp = __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
    rxLen = 128 - temp;

    HAL_UART_Receive_DMA(&huart1, rx_buffer, 128);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
    /* Get RX message */
    if (HAL_CAN_GetRxMessage(CanHandle, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
        /* Reception Error */
        Error_Handler();
    }

    uint8_t id = (RxHeader.StdId >> 7);  // 4Bits ID & 7Bits Msg
    uint8_t cmd = RxHeader.StdId & 0x7F; // 4Bits ID & 7Bits Msg
    if (id == 0 || id == boardConfig.canNodeId)
    {
        OnCanCmd(cmd, RxData, RxHeader.DLC);
    }
}
