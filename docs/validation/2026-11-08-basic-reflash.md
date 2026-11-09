# 2026-11-08 正常固件重新烧录及基础复测

用户接好硬件并明确允许重新烧录。当前USB-SERIAL CH340为COM4（不是历史COM8），STM32 STLink枚举OK。本次不发送ENABLE、运动目标或校准；要求电机功率物理隔离，软件没有独立测量隔离状态。

## 固件身份与构建

下位机main28c40a6，固件源码基线e58a194，仓库干净。复制至build/flash-check-2026-11-08/P9.cbt6_motor_qt，现有Keil5.24全量`-r -j0`构建，8秒，0错误/0告警；Code46444、RO5336、RW92、ZI8556。三个UART诊断开关均0，正常模式。

HEX SHA256：`854E904B2E3B95C4EE4529B9967AEAA7A73F4453849D30E936EA4F0E40DD1FA6`，与此前正常候选一致。静态栈脚本normal、MinimumHeadroom32通过：312+184+368+128=992/1024B；不是最坏运行栈证明。

首次Keil `-f`退出2，Target DLL has been cancelled。调查发现uvoptx默认nTsel3、Segger/JL2CM3.dll，而实际接ST-Link。只改隔离副本为nTsel5、STLink/ST-LINKIII-KEIL_SWO.dll后，同一候选烧录退出0，`Erase Done.Programming Done.Verify OK.Application running ...`。没有修改main中的工程设置或源码；未独立读回校准区，不宣称校准数据已验证保留。

## 基础验证

上位机3a88900工作区的既有host-p4测试程序，Qt5.14.2/MinGW7.3，`STEPPER_TEST_PORT=COM4`，`host_tests.exe -txt`。

- 烧录前板上固件版本不确定：99通过/0失败/0跳过；两轮255/256快照，48/47心跳，STOP确认。不能据此确定板上构建身份。
- 烧录后：进程退出0，七套件合计99通过/0失败/0跳过；两轮各255快照，47/48心跳，STOP确认，600ms后重连，会话错误0。
- 实机用例验证Ready、设备UID长度12、5秒遥测、采样计数递增、mode=None、电流指令0、TXdrop0及STOP返回OK；断言没有ENABLE、MOVE_ABSOLUTE和SET_VELOCITY请求。

本轮只确认基础禁用通信。未测实际运动、使能/禁用输出、物理急停、失联物理停机、ISR最坏耗时、栈高水位、Flash断电及长期压力；不据99项结果宣称上述验收完成。

原始证据：[本次日志目录](evidence/2026-11-08)，包含全量构建、首次烧录失败、成功烧录、烧录前/后测试和stderr。构建产物留忽略build目录。记录暂未提交/推送。
