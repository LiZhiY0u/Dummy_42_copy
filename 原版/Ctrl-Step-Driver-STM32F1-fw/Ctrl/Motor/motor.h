#ifndef CTRL_STEP_FW_MOTOR_H
#define CTRL_STEP_FW_MOTOR_H

#include "Motor/motion_planner.h"
#include "Sensor/Encoder/encoder_base.h"
#include "Driver/driver_base.h"

class Motor
{
public:
    Motor() : controller(&controllerInstance)
    {
        /****************** Default Configs *******************/
        config.motionParams.encoderHomeOffset = 0;
        config.motionParams.caliCurrent = 2000;         // (mA)
        config.motionParams.ratedCurrent = 1000;        // (mA)
        config.motionParams.ratedCurrentAcc = 2 * 1000; // (mA/s)
        config.motionParams.ratedVelocity = 30 * MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;
        config.motionParams.ratedVelocityAcc = 1000 * MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS;

        config.ctrlParams.stallProtectSwitch = false;
        config.ctrlParams.pid = {};
        config.ctrlParams.pid.kp = 5;
        config.ctrlParams.pid.ki = 30;
        config.ctrlParams.dce = {};
        config.ctrlParams.dce.kp = 200;
        config.ctrlParams.dce.kv = 80;
        config.ctrlParams.dce.ki = 300;
        config.ctrlParams.dce.kd = 250;

        /*****************************************************/

        motionPlanner.AttachConfig(&config.motionParams);
        controller->AttachConfig(&config.ctrlParams);
    }

    const int32_t MOTOR_ONE_CIRCLE_HARD_STEPS = 200; // for 1.8° step-motors
    const int32_t SOFT_DIVIDE_NUM = 256;//256
    const int32_t MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS = MOTOR_ONE_CIRCLE_HARD_STEPS * SOFT_DIVIDE_NUM;//200*256

    typedef enum
    {
        MODE_STOP,
        MODE_COMMAND_POSITION,
        MODE_COMMAND_VELOCITY,
        MODE_COMMAND_CURRENT,
        MODE_COMMAND_Trajectory,
        MODE_PWM_POSITION,
        MODE_PWM_VELOCITY,
        MODE_PWM_CURRENT,
        MODE_STEP_DIR,
    } Mode_t;

    typedef enum
    {
        STATE_STOP,
        STATE_FINISH,
        STATE_RUNNING,
        STATE_OVERLOAD,
        STATE_STALL,
        STATE_NO_CALIB,
        STATE_ENCODER_FAULT,
        STATE_STORAGE_FAULT
    } State_t;

    class Controller
    {
    public:
        friend Motor;

        typedef struct
        {
            bool kpValid, kiValid, kdValid;
            int32_t kp, ki, kd;
            int32_t vError, vErrorLast;
            int32_t outputKp, outputKi, outputKd;
            int32_t integralRound;
            int32_t integralRemainder;
            int32_t output;
        } PID_t;

        typedef struct
        {
            int32_t kp, kv, ki, kd;
            int32_t pError, vError;//目标位置差和目标速度差
            int32_t outputKp, outputKi, outputKd;
            int32_t integralRound;
            int32_t integralRemainder;
            int32_t output;//pid计算输出，只决定输出正负
        } DCE_t;

        typedef struct
        {
            PID_t pid;
            DCE_t dce;

            bool stallProtectSwitch;
        } Config_t;

        explicit Controller(Motor *_context)
        {
            context = _context;

            requestMode = MODE_STOP;
            modeRunning = MODE_STOP;
        }

        Config_t *config = nullptr;
        Mode_t requestMode;
        Mode_t modeRunning;
        State_t state = STATE_STOP;
        bool isStalled = false;

        void Init();
        void SetCtrlMode(Mode_t _mode);
        void SetCurrentSetPoint(int32_t _cur);
        void SetVelocitySetPoint(int32_t _vel);
        void SetPositionSetPoint(int32_t _pos);
        bool SetPositionSetPointWithTime(int32_t _pos, float _time);
        float GetPosition(bool _isLap = false);
        float GetVelocity();
        float GetFocCurrent();
        void AddTrajectorySetPoint(int32_t _pos, int32_t _vel);
        void SetDisable(bool _disable);
        void SetBrake(bool _brake);
        void ApplyPosAsHomeOffset();
        void ClearStallFlag();

    private:
        Motor *context;
        int32_t realLapPosition{};
        int32_t realLapPositionLast{};//上次细分数记录
        int32_t realPosition{};//目前实际位置，由mt6816校准得到
        int32_t realPositionLast{};
        int32_t estVelocity{};
        int32_t estVelocityIntegral{};
        int32_t estLeadPosition{};//由估计速度通过补偿表得到的估计位移
        int32_t estPosition{};//由估计速度和补偿表得到估计位置
        int32_t estError{};//估计误差
        int32_t focCurrent{};
        int32_t goalPosition{};//目标位置
        int32_t goalVelocity{};
        int32_t goalCurrent{};
        bool goalDisable{};
        bool goalBrake{};
        int32_t softPosition{};
        int32_t softVelocity{};
        int32_t softCurrent{};
        bool softDisable{};
        bool softBrake{};
        bool softNewCurve{};
        int32_t focPosition{};
        uint32_t stalledTime{};
        uint32_t overloadTime{};
        bool overloadFlag{};

        void AttachConfig(Config_t *_config);
        void CalcCurrentToOutput(int32_t current);
        void CalcPidToOutput(int32_t _speed);
        void CalcDceToOutput(int32_t _location, int32_t _speed);
        void ClearIntegral() const;

        static int32_t CompensateAdvancedAngle(int32_t _vel);
    };

    struct Config_t
    {
        MotionPlanner::Config_t motionParams{};
        Controller::Config_t ctrlParams{};
    };
    Config_t config;

    MotionPlanner motionPlanner;
    Controller *controller = nullptr;
    EncoderBase *encoder = nullptr;
    DriverBase *driver = nullptr;

    void Tick20kHz();
    void Stop();
    void BeginMaintenance();
    void EndMaintenance();
    bool IsMaintaining() const { return maintenance; }
    void LatchEncoderFault();
    bool HasEncoderFault() const { return encoderFault; }
    bool HasFault() const { return encoderFault || storageFault; }
    void LatchStorageFault();
    void AttachEncoder(EncoderBase *_encoder);
    void AttachDriver(DriverBase *_driver);

private:
    volatile bool maintenance = false;
    bool encoderFault = false;
    bool storageFault = false;
    Controller controllerInstance = Controller(this);//指向类自身

    void CloseLoopControlTick();
};

#endif
