# Flash驱动内部防护影响记录

2026-11-08。按用户直接开发验证的执行要求，限定内部实现修复；不改变公开函数签名/结构体、HAL SDK、IRQ、构建系统或分区布局。

直接产品修改：demo/P9.cbt6_motor_qt/Memory/stockpile_f103cb.c。public header stockpile_f103cb.h保持不变。

调用者：Encoder/encoder_calibrator_base.cpp、历史encoder_calibrator_stm32.cpp。既有Empty/Begin/End/SetWriteAdd/Write16/32/64入口保留；错误时提前退出，写游标只在成功后递增。void接口不新增状态返回，不能声称具备完整错误上报契约。

目标：现有STM32F103CB 128KB，Keil ARMClang6.7正常配置；内存校验限制0x08000000..0x08020000，不宣称此修改兼容其它容量芯片。宏、配置、原始分区布局未改。

测试：真实C文件由MinGW GCC以C99编译，HAL边界替身；新编译错误路径、溢出及空指针测试通过；正常ARM全量0错误/0告警和静态栈门槛通过。公开符号无计划删除，头文件Git差异为空；未进行跨板矩阵或实机回滚演练，不把它们记作已过。

本文件记录实际影响而非框架SPEC FROZEN声明。后续分步存储接口或RAM执行/链接布局若需要扩大范围，须单独记录真实影响与验证，不将内部修复推定为许可。
