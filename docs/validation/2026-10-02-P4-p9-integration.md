# P4：P9 下位机首批协议接入验证

日期：2026-10-02。固件分支：`codex/qt-host-firmware`。基线提交：`05c6690`。

## 已接入

- revision 1 帧解析、CRC-16/CCITT-FALSE、最大 128B payload。
- UART Receive-to-idle DMA 分块接收、512B 环形缓冲，解析在主循环执行。
- 四槽 DMA TX 队列；应答优先于遥测，活动 DMA 缓冲不复用。
- ARM PRIMASK 临界区、三槽控制邮箱、20kHz 周期执行确认和一致快照。
- `HELLO/GET_INFO/GET_STATUS/HEARTBEAT/ENABLE/DISABLE/STOP/CLEAR_FAULT/TELEMETRY_CONFIG`。
- 500ms 心跳超时停机、清会话并锁存通信故障；重新握手或清故障不会自动使能。
- UART 会话期间阻止 CAN 运动/校准命令；CAN 正在运动或校准时拒绝新 UART 会话。
- 未接入的 revision 1 运动目标、参数、保存与校准命令明确返回 `UNSUPPORTED`。

## 验证证据

平台无关 Qt/固件测试基线：92 passed、0 failed；涵盖帧恢复、会话、去重、心跳、STOP 优先级、邮箱边界与控制服务。P9 接入后使用本机现有 Keil MDK 5.24 / ARM Compiler 6.7 单任务完整构建：

```text
Program Size: Code=46252 RO-data=5108 RW-data=80 ZI-data=7696
FromELF: creating hex file...
0 Error(s), 0 Warning(s)
```

链接 map：RO 总计 51,360B；RW+ZI 总计 7,776B；ROM 总计 51,440B。目标 STM32F103CBT6 为 128KiB Flash / 20KiB SRAM，静态区未超限。基线构建为 Code=23078、RO-data=2346、RW-data=68、ZI-data=4004；新增协议、缓存与安全服务增加约 25.9KiB ROM、3.7KiB 静态 RAM。

## 尚未验证

- 未烧录、未打开真实串口、未与 Qt 应用完成握手/遥测/断线实测。
- 未测 UART 高负载下丢帧、DMA 错误恢复和 TX 队列高水位。
- 未测 20kHz ISR 最坏执行时间、心跳失联到实际驱动关断延迟。
- MOVE/速度/电流目标、参数持久化与 UART 校准仍未接入。

因此本轮状态是“完整构建通过，待禁用状态实机联调”，不是 P4 完成或整机验收通过。
