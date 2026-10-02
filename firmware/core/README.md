# 下位机平台无关核心

此目录是P4新增核心的当前源码位置。代码无需Qt/HAL，可用现有MinGW行为测试和ARM Compiler6.7模块交叉编译。

- protocol_v1：固定缓冲二进制协议解析/编码。调用只产生Frame，不能直接控制电机；命令分发还须校验会话/SEQ/命令参数。
- control_gate：单一上下文拥有的逻辑使能/故障门控。未来控制周期适配层必须执行真实停机；enabled=false本身不会改变GPIO或PWM。
- command_dispatcher：四项请求/应答去重缓存与持久序号水位、请求格式校验和Backend分发。当前生产控制后端未接入；仅测试夹具完成Qt会话对接。
- control_service：三槽有界邮箱、控制周期执行确认、驱动抽象接口与一致快照。ARM临界区、P9真实驱动和协议异步完成适配尚未接入。

目前尚未加入P9 Keil工程。测试入口在host/tests/tst_firmware.cpp；交叉编译命令为scripts/check-firmware-core.ps1。接入调查和验证详见docs/validation/2026-10-02-P4-core.md。
