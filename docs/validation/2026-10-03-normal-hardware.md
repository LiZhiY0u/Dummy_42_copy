# 2026-10-03：正常候选烧录与禁用通信实测

用户明确授权本地提交、烧录正常候选及禁用状态测试；未授权运动或自动使能。本轮使用STLink、现有Keil 5.24及CH340 COM8，115200/8N1。

## 版本与固件身份

- 上位机测试/文档提交1f5456a；下位机源码提交e58a194。
- 正常配置三个UART诊断开关均0。候选全量Code46444、RO5336、RW92、ZI8556；静态估算992/1024B。
- HEX SHA256：854E904B2E3B95C4EE4529B9967AEAA7A73F4453849D30E936EA4F0E40DD1FA6。
- 烧录来源是全量临时副本的AXF，与上述HEX同一链接生成：`C:/Users/Cainiao/AppData/Local/Temp/p9-stack-green-92c2c22b13314a8488e5d03a30fdf2cf/P9.cbt6_motor_qt/MDK-ARM/cbt6_demo3/cbt6_demo3.axf`，不是原目录旧候选。

## 烧录结果

既有STLink USB VID0483/PID3748，状态OK。首轮设备名过滤漏掉STM32 STLink；按VID重新枚举后确认。JLink7.96的-ShowEmuList参数不受支持，没有因此操作目标或安装工具。

通过Keil命令行`UV4.exe -f <候选uvprojx> -o <flash-normal.log>`，进程退出0；日志11:35:36记录：

```text
Erase Done.Programming Done.Verify OK.Application running ...
```

flash-normal.log SHA256：E40D25247CC1E172A62BD37BB9F2F93C90CA3E012E28C7CFE8646B7E6C881356。日志显示擦除/编程/校验成功；没有独立回读校准存储区，不据此宣称校准数据未变。命令行为依据[Keil Program Flash](https://www.keil.com/support/man/docs/uv4cl/uv4cl_cl_programflash.htm)。

## Qt真实通信

执行`scripts/test-real-serial.ps1 -Port COM8 -BuildName host-p4`，退出0；全部软件及实机套件99 passed、0 failed、0 skipped（含初始化/清理）。

| 场景 | 第一轮 | 第二轮 |
| --- | --- | --- |
| HELLO→INFO→STATUS→遥测配置→Ready | 通过 | 通过 |
| 连续5秒快照 | 255 | 255 |
| 心跳请求 | 48 | 48 |
| STOP软件应答 | Ok | Ok |
| 断开后等待600ms再连接 | 通过 | 通过 |

测试断言：样本计数递增、mode=None、指令电流0、TXdrop0，会话错误0。未发送ENABLE、位置/速度/电流目标或校准命令。当前用例验证等待600ms后重连，不是亚500ms快速接管压力测试，也没有直接测量断开后瞬间的驱动关断。

原始日志`build/host-p4/tests.log`，SHA256：48D3C32A31ECA43292623A1F09278FD98DB74A9C844FD8CB601E8003F9608BAF。本次覆盖旧日志，所以本记录保存独立摘要和身份。

## 尚未验收

正常配置禁用通信已得到新实机证据；没有测栈哨兵/SP最小值、ISR连续最坏时序、编码器精度、物理停机或功率输出。STLink被枚举及Keil烧录成功不等于已具备ETM连续追踪。未在没有确认当前有效SP的情况下盲填栈区，没有修改启动/HAL/IRQ来插桩。

[测量方案](2026-10-03-stack-debug-measurement.md)仍待实施。32B静态余量不是安全标准，P4仍进行中、P5运动仍未开放；本轮未推送/合并，也未清理历史Keil产物。
