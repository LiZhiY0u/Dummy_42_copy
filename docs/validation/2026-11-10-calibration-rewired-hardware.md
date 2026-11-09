# 修正电机接线后校准成功

2026-11-10。用户明确报告电机线接错、已修正并要求重新开始。该报告是接线修正来源，未通过视觉或电气测量独立核对。保留[首次失败记录](2026-11-10-calibration-first-hardware.md)，不将未知分析errorCode倒推为已读到的具体错误。

## 条件与动作

同一已烧录7D5932DF正常候选，无重建/重烧录，PnP重新确认CH340 COM4与STLink均OK。额定3A、12V、空载/可断电为用户此前确认条件；本轮校准指令1000mA，不是实测相电流。

校准脚本增加显式-ClearCalibrationFault：仅在STOP/禁用与编码器有效检查后，允许清除仅bit5校准失败（可伴bit2通信超时，即32/36）的既有锁存。存储/编码器等其他故障仍拒绝，不自动扩大到任意清故障。默认不清校准故障。SelfTest本轮通过。

```powershell
& './scripts/run-device-calibration.ps1' -Port COM4 -StartCalibration -ClearCalibrationFault
```

运行退出0：前置检查后显式清除fault36，查询确认无故障，单次启动task2，没有自动重试或普通ENABLE/运动目标。

## 实机反馈

```text
ACCEPTED task=2
EVENT 0501: 02-00-00-00-01-00-00-00-F3-44-04-00
phase1 percent0 result1 verified0 elapsedMs29
phase3 percent50 result1 verified0 elapsedMs7232
phase3 percent89 result1 verified0 elapsedMs10040
phase4 percent93 result1 verified0 elapsedMs10163
phase4 percent97 result1 verified0 elapsedMs10255
EVENT 0501: 02-00-00-00-06-00-00-00-65-6A-04-00
phase5 percent100 result0 verified1 elapsedMs10362
PASS: calibration verified and disabled
FINAL STOP acknowledged
```

最终状态检查通过state0/mode3、calibrated1、encoderValid1、fault0，STOP确认，关闭串口。相较首次失败，这次通过采集数据分析、表写入/回读校验/提交及任务终态。可确认本次设备报告存储验证成功，不能据此宣称真实掉电/复位后恢复、Flash耐久或全运动功能已验收。

## 当前边界与下一步

未测物理轴角度/转速/相电流/停止响应时延、未复位或断电，未做普通闭环目标运动；没有提交推送。串口关闭后可能重新出现通信超时故障，本轮最终fault0仅指关闭前窗口。

下一步先在用户允许复位/断电后验证校准重启保持（不重烧录覆盖校准区），再开发/验证一个受限低速短程运动目标及STOP/失联退出。现有普通目标命令仍未接入，不能因校准成功直接宣称已具备完整运动控制。
