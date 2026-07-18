#ifndef CTRL_STEP_FW_ENCODER_CALIBRATOR_BASE_H
#define CTRL_STEP_FW_ENCODER_CALIBRATOR_BASE_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "motor.h"

    class EncoderCalibratorBase
    {
    public:
        static const int32_t MOTOR_ONE_CIRCLE_HARD_STEPS = 200; // for 1.8° step-motors，200
        static const uint8_t SAMPLE_COUNTS_PER_STEP = 16;       // 16
        static const uint8_t AUTO_CALIB_SPEED = 2;
        static const uint8_t FINE_TUNE_CALIB_SPEED = 1;

        typedef enum
        {
            CALI_NO_ERROR = 0x00,
            CALI_ERROR_AVERAGE_DIR,
            CALI_ERROR_AVERAGE_CONTINUTY,
            CALI_ERROR_PHASE_STEP,
            CALI_ERROR_ANALYSIS_QUANTITY,
            CALI_ERROR_ENCODER,
        } Error_t;

        typedef enum
        {
            CALI_DISABLE = 0x00,
            CALI_FORWARD_PREPARE, // 准备阶段，试转一圈
            CALI_FORWARD_MEASURE, // 正转记录阶段
            CALI_BACKWARD_RETURN,
            CALI_BACKWARD_GAP_DISMISS,
            CALI_BACKWARD_MEASURE, // 反转记录阶段
            CALI_CALCULATING,
        } State_t;

        // explicit EncoderCalibratorBase(Motor* _motor)
        // {
        //     motor = _motor;

        //     isTriggered = false;
        //     errorCode = CALI_NO_ERROR;
        //     state = CALI_DISABLE;
        //     goPosition = 0;
        //     rcdX = 0;
        //     rcdY = 0;
        //     resultNum = 0;
        // }

        bool isTriggered; // true则重新开启校准流程

        void Tick20kHz();
        void TickMainLoop();

        // private:
        Motor *motor;

        Error_t errorCode;
        State_t state;
        uint32_t goPosition;
        bool goDirection;                                                      // 由计算得到方向定义
        uint16_t sampleCount = 0;                                              // 16次计数记录
        uint16_t sampleDataRaw[SAMPLE_COUNTS_PER_STEP]{};                      // 记录每1.8度的mt6816数据16次的缓存区
        uint16_t sampleDataAverageForward[MOTOR_ONE_CIRCLE_HARD_STEPS + 1]{};  // 正转每1.8度的数据记录
        uint16_t sampleDataAverageBackward[MOTOR_ONE_CIRCLE_HARD_STEPS + 1]{}; // 反转每1.8度的数据记录
        int32_t rcdX;                                                          // X:最接近零点的步数（下一步就越过零点），正数
        int32_t rcdY;                                                          // Y:X+1步距零点的编码器距离，正数
        uint32_t resultNum;

        void CalibrationDataCheck();
        static uint32_t CycleMod(uint32_t _a, uint32_t _b);
        static int32_t CycleSubtract(int32_t _a, int32_t _b, int32_t _cyc);
        static int32_t CycleAverage(int32_t _a, int32_t _b, int32_t _cyc);
        static int32_t CycleDataAverage(const uint16_t *_data, uint16_t _length, int32_t _cyc);

        /***** Port Specified Implements *****/
        void BeginWriteFlash();                      // 解锁flash，准备写入
        void EndWriteFlash();                        // 上锁flash，结束写入
        void ClearFlash();                           // 清空flash
        void WriteFlash16bitsAppend(uint16_t _data); // flash写入数据，需要在解锁和上锁之间使用
    };
    extern EncoderCalibratorBase encoder_calibrator_base;
#ifdef __cplusplus
}
#endif

#endif
