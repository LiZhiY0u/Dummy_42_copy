# COM4基础实机复核

日期2026-11-10，用户确认硬件已接好并要求检查。仅运行握手/STOP/心跳/信息/状态，没有烧录、使能、清故障或校准。

## 当前识别与命令

沙箱外只读PnP识别：USB-SERIAL CH340(COM4)、STM32 STLink均Status OK。此前沙箱CIM受限不能识别的缺口本轮补齐，不是此前已识别。

在项目根目录运行：

```powershell
& './scripts/check-device-basics.ps1' -Port COM4
```

脚本退出0。七个请求HELLO、STOP、HEARTBEAT、GET_INFO、GET_STATUS、HEARTBEAT、GET_STATUS均收到CRC/类型/会话/序号/命令匹配的STATUS0。应答长度依次4/50/6/52/50/6/50。STOP反馈及两次状态均state0、mode3、calibrated0、encoderValid1、fault_bits0。

UID原始小端字节：49-FF-6A-06-71-85-51-56-60-28-18-87；能力0x180。它不是最新正常候选预期0x1A0，但不能仅靠能力断言准确HEX或Flash容量，未读取调试器身份。当前候选7D5932DF仍没有本轮烧录证据。

## 结论与限制

基础串口、STOP应答与禁用状态反馈正常，当前采样有效而校准无效。仅三份状态响应，不是持续编码器稳定性测试、校准有效性验证或物理停机证明。断开后未继续读取，心跳超时可能随后锁存通信故障，不能把窗口内fault0外推到断开后。

没有新编译ARM/Qt、没有输出/运动、没有提交推送。脚本关闭并释放COM4。下一步需要明确授权后烧录已核对配置及哈希的新候选，再重复此基础检查；低电流校准另作受控步骤，不自动跟随烧录启动。
