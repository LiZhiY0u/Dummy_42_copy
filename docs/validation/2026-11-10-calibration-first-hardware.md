# 新候选烧录与首次受控校准

2026-11-10，用户明确“烧录并开始吧”，本轮授权包含烧录和校准尝试，不包含普通运动目标或自动重复校准。硬件条件沿用用户确认3A额定、12V、空载、可断电；实际供电/电流/机械转动没有仪器或视觉测量。

## 候选与烧录

从当前独立固件仓库复制至build/calibration-flash-2026-11-10/P9.cbt6_motor_qt，三个UART诊断开关全0；仅隔离副本复用已验证ST-Link uvoptx，未改main调试器设置。现有Keil全量0错0告警，Code51052/RO5480/RW140/ZI9020；静态992/1024B、32B余量通过。

HEX SHA256：7D5932DF3019B4E482277C82AF02AE4CA99E3B5F496E74C1FDE6D65C964BF5D7，与前轮候选一致。Keil -f日志：Erase Done.Programming Done.Verify OK.Application running。成功加载该候选；未独立读回全片或测物理容量，应用运行时128KB守卫仍在。日志见[evidence](evidence/2026-11-10-calibration-first-hardware)。

烧录后COM4基础脚本七请求全成功，UID49-FF-6A-06-71-85-51-56-60-28-18-87，cap0x1A0。STOP和两状态均state0/mode3、calibrated0、encoderValid1、fault0。

## 工具与实际执行

新增scripts/run-device-calibration.ps1，必须显式-Port和-StartCalibration，白名单无普通ENABLE/运动目标；启动前STOP、能力/1..1000mA/采样有效/禁用/无故障检查，仅允许单独通信超时故障自动清除。循环心跳与任务查询，60秒上限，失败或结束finally STOP。基础脚本新增LibraryOnly供复用，不放宽原命令白名单。

首次自检因点加载覆盖同名参数导致SelfTest分支未进入，保留外层Port/SelfTest后修复，新自检通过查询帧和拒绝ENABLE；基础自检亦通过。没有安装工具。实际命令：

```powershell
& './scripts/run-device-calibration.ps1' -Port COM4 -StartCalibration
```

本轮串口原始结果摘要：

```text
START calibration command-current=1000mA (not measured phase current)
ACCEPTED task=1
EVENT cmd=0501 payload=01-00-00-00-01-00-00-00-0B-69-01-00
task=1 phase=1 percent=0 result=1 verified=0 elapsedMs=31
EVENT cmd=0501 payload=01-00-00-00-05-00-0D-00-39-85-01-00
task=1 phase=6 percent=50 result=13 verified=0 elapsedMs=7303
FINAL STOP acknowledged
```

脚本退出1（明确报告校准失败），没有重试。任务约7.3秒后分析阶段失败，result13为通用INTERNAL_ERROR，verified0。源码中分析错误统一映射13，具体errorCode/样本数组没有通过协议读出，**不能确定方向/连续性/零点哪项失败，也不能据此断言接线、磁铁或机械原因**。50%是软件阶段标记，不是完成了一半校准的物理测量。没有完整校准提交成功证据。

## 失败后复核与当前状态

再次基础探针七请求成功、STOP确认：state4(FAULT)、mode3(禁用)、calibrated0、encoderValid1、fault0x24。bit5校准失败锁存，bit2通信超时与两次会话之间断开相符；不是存储bit3或编码器bit0。没有清除这些故障，也没有再次启动。脚本关闭释放COM4。软件STOP应答不是物理停机证明，实际轴是否转动/转动方向/异常声响需要用户观察。

本轮未跑Qt全套/21UART/联合测试，未提交推送。下一步先获取本次机械转动现象，补只读校准诊断（具体校准错误与样本统计或停止状态下调试器读取），定位后再决定修复和受控复测；不提高电流、不放宽校验、不自动重试。
