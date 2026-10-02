# 2026-10-03：命令契约与P9框架同步验证

范围：20个请求的命令/载荷校验、真实支持清单及能力声明；不增加运动、参数或校准后端。执行依据：[设计](../superpowers/specs/2026-10-03-firmware-command-contract-design.md)、[计划](../superpowers/plans/2026-10-03-firmware-command-contract.md)。

## 变更

- 权威契约为firmware/core/command_contract.h；P9 Uart目录保留逐字节一致分发副本。Qt消息校验、通用分发器及实际P9均使用同一规则。
- P9 command_support.h明确9个真实处理命令；GET_INFO从该清单计算能力，仍为0x180。未实现请求先校验，合法时返回10，不合法时返回3/4；未知请求和事件号请求返回2。
- 帧结构、会话、缓存/pending、序号水位检查仍先行；控制命令仍经邮箱并在完成后应答，STOP/心跳优先级未改。
- 不使用同步Backend替代P9，不在IRQ中做契约校验，不新增堆分配/阻塞/Frame副本。

## RED → GREEN 证据

| 项目 | RED | GREEN |
| --- | --- | --- |
| 独立契约 | Qt构建失败：command_contract.h不存在 | 新增三项契约测试、20命令合法向量、20×129长度矩阵、15项参数及数值边界通过 |
| 实际P9拒绝顺序 | TASK_QUERY长度5，期望3，实际10，assert退出1 | request-contract覆盖11个未实现请求的合法/错误长度，越界、未知号、会话/去重优先及无控制副作用通过 |
| 副本门槛 | 旧脚本接受故意不一致副本，测试throw | 不一致副本/缺头文件/缺项目拒绝，真实匹配副本接受 |

Qt：`./scripts/build-host.ps1 -Test -BuildName host-p4`最后一轮退出0。7套件98 passed、0 failed、1 skipped，含初始化/清理；唯一跳过为真实串口。相较基线95增加3个测试方法，矩阵循环不另算通过条数。

P9：用tests/README.md的既有MinGW命令重新构建UART测试；基线和request-contract、cached-takeover、pending-takeover、running-takeover、info-capabilities共6次执行均通过。编码器、UART TX及echo重新构建执行均通过。硬件双精确限定在UART/HAL/寄存器边界，实际parser、协议处理和控制服务参与测试。

副本门槛：`./scripts/tests/test-contract-copy.ps1 -P9Project <P9-root>`退出0；真实副本逐字节一致，ARM核心对象与原布局assert通过。测试fixture只在scripts/tests，不进入产品include目录。

## 正常候选构建与栈

使用既有ARMClang6.7，重编译P9 uart_protocol.cpp，Cortex-M3/Thumb、-Oz、C++11、无异常/RTTI、STM32F103xB；与既有正常模式对象增量链接，fromelf导出HEX。不是清空目录后的全量构建。

| 项目 | 结果 |
| --- | --- |
| Code | 46400B（比上一候选增加968B） |
| RO-data | 5148B（增加132B） |
| RW-data / ZI-data | 92B / 8540B（不变） |
| 静态栈门槛 | main280 + receiver88 + IRQ344 + SysTick0 + allowance128 = 840B/1024B |
| HEX SHA256 | E4AFB735B23EB63CBA9A2ADD188F691B4F8AE5B5C64DAAA6447F7502B6BAD38B |
| 既有告警 | motor.h的MT6816_base.h包含名与磁盘大小写不一致，1项；未为本轮改动修正 |

代码/只读数据增加，没有通过扩大栈或RAM区域规避门槛；静态估算仍不能替代运行高水位。候选位于P9/MDK-ARM/cbt6_demo3/cbt6_demo3.hex。已验证诊断serial-diagnostic-verified.hex未覆盖。

新增契约/支持头文件未检出malloc/free、printf、HAL_Delay或阻塞UART发送；UART RX/TX ISR入口和控制服务未修改。该文本检查及调用边界核对不是所有ISR执行时间的证明。

## 实施裁决（偏离与代价）

1. 沿批准计划中的现有codex开发分支/目录执行，不再新建worktree：避免移动用户的联调环境；代价是源码共用当前分支，依靠显式路径提交保护。
2. Windows PowerShell/apply_patch台账替代POSIX任务脚本：保留BASE、RED/GREEN及任务完成记录；代价是人工台账可能遗漏，需与Git核对。
3. 原info-capabilities用例空载荷运动请求改为合法向量：它验证的是能力/未实现后端而非载荷错误；错误路径由新用例独立覆盖。
4. 副本负例保留为scripts/tests永久fixture而非build临时文件：checkout后仍可回归；代价是该假头文件必须始终保持在测试目录。
5. 用户后续明确要求两端Git管理并批准含提交步骤的计划，覆盖最初设计“不提交”的边界：只做本地里程碑，仍不推送/合并。

## Git与尚未验收

上位机产品提交09067ca（契约）、7a253cd（Qt/通用核心接入）；P9产品提交b796746（真实接入及回归）。副本检查和文档作为后续本地里程碑提交，ID可查询包含本记录的提交。

本轮未烧录、未打开硬件串口、未使能/运动、未安装环境、未推送/合并。旧诊断实机结果不用于认定新候选实机通过。正常模式通信、运行栈高水位、ISR耗时、异常链路及物理停机仍待验收。独立只读审查结果另行追加，不将软件测试等同于审查或硬件认证。
