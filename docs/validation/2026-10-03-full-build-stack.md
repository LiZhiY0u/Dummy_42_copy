# 2026-10-03：正常配置全量构建与栈基线

源码基线：P9 a14d8d6，上位机1ff3a3b；本轮尚未修改产品代码或工程配置。

为保留原目录历史Keil产物，在临时副本调用现有Keil 5.24：

```powershell
Start-Process 'E:/keil5_2_4/UV4/UV4.exe' -ArgumentList @('-r','<副本>/MDK-ARM/cbt6_demo3.uvprojx','-j0','-o','<临时目录>/full-build.log') -WindowStyle Hidden
```

副本路径：`C:/Users/Cainiao/AppData/Local/Temp/p9-fullbuild-9af33c92ba9d46dfb4a19b56fed7d850/P9.cbt6_motor_qt`。使用原工程cbt6_demo3目标、ARMClang6.7，三个UART诊断开关均0。没有清理原目录、修改配置、安装工具或烧录。

实际输出：全量编译/链接，0 Error(s)、0 Warning(s)，耗时12秒；Code46416、RO5316、RW92、ZI8540。RO与手工增量候选5148不同，不混用两份身份或告警记录。

- 正常栈脚本：main312 + receiver200 + IRQ368 + allowance128 = 1008/1024B。报告生成时间与本次构建一致。
- HEX SHA256：2FBCCD8A75887D28E5B9165EED17075A97CEF837129073EA3A2E4301D62F9F37。
- full-build.log SHA256：3B8443758EC55EA88E3193CA06D98B35797A70C6674926993E0489241DE3C01A。

这确认小余量不是增量旧对象混用造成。旧库异常调用仍存在Unknown尾部，脚本预留不是绝对上界；只有16B账面余量，未达到运行安全验收。未重跑Qt回归，不将此前98/0/1写为本轮新结果。

后续有界方案：仅重排UART主循环已有快照读取路径以缩短临时对象与发送/校验链的栈重叠，保持snapshot公共接口、协议与中断所有权不变；补非空规则空指针测试；制定调试器栈高水位/ISR测量步骤，不添加启动或IRQ测量钩子。改动后重新软件回归、全量构建并报告栈变化；若需要HAL/启动/链接改动，另行确认范围。

## 确认方案后的实施结果

- UART回调先完成快照捕获，再进入不内联dispatchFrame；快照及请求时间由非重入主循环回调工作区持有。公共snapshot API、载荷规则、会话/去重顺序及IRQ未改变。代价：当前ZI增加16B，主循环非重入所有权仍是必要约束。
- 余量回归RED：旧1008B报告剩16B，要求至少32B失败。首轮分离后1000B仍失败；请求时间不再跨捕获调用保留于局部栈，最终992B通过。未降低检查门槛或扩大栈。
- 最终全量正常配置0错误/0告警，Code46444、RO5336、RW92、ZI8556；main312 + receiver184 + IRQ368 + allowance128 = 992B/1024B，剩32B，仅是改进回归指标，不是安全认证。
- 临时副本：`C:/Users/Cainiao/AppData/Local/Temp/p9-stack-green-92c2c22b13314a8488e5d03a30fdf2cf/P9.cbt6_motor_qt`。
- 最终HEX SHA256：854E904B2E3B95C4EE4529B9967AEAA7A73F4453849D30E936EA4F0E40DD1FA6；full-build.log SHA256：800C5BCDFED1DB20026871859091C9BEE69FC4280A4655E9DE76A5DDB3519108。
- Qt完整回归98通过/0失败/1真实串口跳过；新增断言放在既有测试方法，计数未增加。HELLO/null4、WRITE_PARAMS/null4及非空129B超长已覆盖。
- 空指针临时副本突变探针：首轮因未设置MinGW PATH无法生成exe，不能作为RED；补齐现有工具PATH后删除公共空指针保护退出-1073741819，恢复原头文件退出0。产品两份契约未修改，副本门槛及ARM核心布局验证通过。
- P9六UART场景与编码器/TX/echo全部重新编译执行通过；栈fixture已知/未知及最小余量边界436B通过、437B拒绝。
- [栈高水位与ISR测量方案](2026-10-03-stack-debug-measurement.md)已制定，未执行。原工作区历史产物未覆盖，没有安装、烧录、串口或电机操作。
