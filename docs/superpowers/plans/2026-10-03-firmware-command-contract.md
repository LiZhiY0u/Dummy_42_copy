# 下位机命令契约同步 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans（本任务建议当前会话顺序执行）or superpowers:subagent-driven-development to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 统一Qt、通用固件核心及实际P9的20个请求载荷校验，使未实现后端在校验后明确拒绝，保留现有控制执行边界。

**Architecture:** C++11只读契约头文件提供描述、载荷校验及显式支持清单的能力映射。P9使用逐字节检查的版本化副本；会话、去重、邮箱和完成应答留在原有模块，不引入同步等待或新运动后端。

**Tech Stack:** Windows PowerShell、Qt 5.14.2、MinGW 7.3.0、ARMClang 6.7、现有P9项目；不安装新环境。

**Spec:** [命令契约设计](../specs/2026-10-03-firmware-command-contract-design.md)。当前为实施计划待评审，未修改产品代码。

## Global Constraints

- V1/revision1、128B载荷上限、既有0～13状态码不变。
- P9仍为9个已接入、11个未接入请求，capabilities仍为0x180。
- 新token HELLO、缓存与pending作用域、拒绝请求水位、STOP/HB优先、完成后才应答均不改变。
- 契约只在主循环调用，无HAL、堆分配、递归、异常、RTTI、阻塞等待；参数ID临时空间最多15个u16。
- 队列/缓存容量、1024B栈及现有Flash/RAM区域不变，不修改IRQ、HAL、Keil工程或链接脚本。
- 不烧录、不连接硬件、不使能/运动、不推送/合并；保留下位机84个历史Keil产物/用户状态修改。
- 上位机基线abc6776，下位机基线f57c14a。受控源码可按已授权Git管理提交本地里程碑，不暂存生成物。

## Review Focus

- 非零长度空指针/129B/尾随字节：读取前拒绝，不访问越界（任务1）。
- 对比双方共享同一实现导致假通过：独立合法向量和字面状态断言（任务1/2）。
- 未实现命令混入合法帧错误会话/重复序号：会话和去重优先，不执行副作用（任务3）。
- 能力位只从枚举推断或缺少组合依赖：显式后端支持及不完整组合测试（任务1/3）。
- 副本漂移或新增主循环校验使栈超限：不一致强制失败，完整链栈门槛（任务4）。

## 文件职责

以H表示上位机仓库根目录，以P表示下位机仓库`demo/P9.cbt6_motor_qt`；所有路径相对于对应根目录。

| 文件 | 职责 |
| --- | --- |
| H/firmware/core/command_contract.h（新增） | 权威无平台契约、描述、校验、能力映射 |
| P/Uart/command_contract.h（新增） | 权威文件的逐字节一致副本 |
| P/Uart/command_support.h（新增） | P9静态9命令支持清单，不包含执行代码 |
| H/host/src/protocol/ProtocolMessages.cpp | 保留帧/会话检查，转调契约 |
| H/firmware/core/command_dispatcher.cpp | 移除重复校验，不改Backend接口 |
| P/Uart/uart_protocol.cpp | 校验→支持→原执行路径、能力声明 |
| H/host/tests/tst_dispatcher.cpp、tst_messages.cpp | 独立向量、长度矩阵、原分发回归 |
| P/tests/uart_handshake_test.cpp | 实际P9拒绝行为与无副作用验证 |
| H/scripts/check-firmware-core.ps1 | 副本一致性门槛、既有ARM对象验证 |
| H/docs、P/tests/README.md | 行为变化、结果与扩展规则 |

### Task 1: 无平台契约及独立测试

**Interfaces:** 命名空间`stepper::contract`。`enum class Rule : uint8_t`列出Empty、Hello、Enable、MoveAbsolute、Velocity、Current、TaskId、Parameters、Save、Telemetry；`enum class Group : uint8_t`列出Core、Motion、Parameters、Calibration、Telemetry。

`struct Description { uint16_t command; Rule rule; Group group; uint8_t ordinal; };`，ordinal为协议20个请求按命令号升序的0～19序号。`struct Support { uint32_t commandBits; };`仅保留低20位。

产生接口：`const Description *describe(uint16_t command)`、`uint16_t validatePayload(uint16_t command,const uint8_t *payload,size_t length)`、`bool supports(Support support,uint16_t command)`、`uint32_t capabilities(Support support)`；全部inline函数，const描述只读，不新增翻译单元。

- [ ] 在`tst_dispatcher.cpp`添加`contractIndependentVectors()`：20条独立合法载荷（直接从协议基线编写字节，不用被测编码函数生成）全部返回0；每条加尾随字节返回3，WriteParams计数不变时也拒绝尾随数据。枚举20项及事件/遥测号缺席独立断言。
- [ ] 添加`contractBoundaryFailures()`，至少包含以下字面断言，并涵盖参数15项、重复ID、未知ID、错误type/length和上下限。

```cpp
QCOMPARE(validatePayload(2,nullptr,0),uint16_t(0));
QCOMPARE(validatePayload(2,nullptr,1),uint16_t(3));
QCOMPARE(validatePayload(2,nullptr,129),uint16_t(3));
QCOMPARE(validatePayload(0x0501,nullptr,0),uint16_t(2));
const uint8_t zero[4]={0,0,0,0};
QCOMPARE(validatePayload(1,zero,4),uint16_t(3));
QCOMPARE(validatePayload(5,zero,4),uint16_t(4));
const uint8_t minimum[4]={0,0,0,0x80};
QCOMPARE(validatePayload(0x0105,minimum,4),uint16_t(4));
```

- [ ] 添加`contractCapabilitiesRequireBackendSupport()`：空支持返回0；P9九项返回字面0x180；只有READ_PARAMS或WRITE_PARAMS不能置bit3；位置/速度/电流需ENABLE与对应目标命令；保存需SAVE_PARAMS与TASK_QUERY；校准需START/QUERY/CANCEL/TASK_QUERY；逻辑零点、清故障、遥测分别需对应命令。未知支持位不产生能力。
- [ ] 运行`./scripts/build-host.ps1 -Test -BuildName host-p4`确认新增头文件/符号缺失导致RED，保存具体失败到日期化验证记录，不把环境失败视为RED。
- [ ] 用apply_patch实现权威头文件，迁入当前校验规则，读取字段前做长度/指针检查，参数重复检测有界。
- [ ] 同命令运行全部Qt测试确认GREEN；提交H端契约及测试里程碑，不声称P9已接入。

### Task 2: Qt与通用核心接入

**Interfaces:** 消费任务1的describe/validatePayload；保留`ProtocolMessages::validateRequest(const Frame&)`、`validateParameters(const QByteArray&)`和`CommandDispatcher::handle(...)`公开接口。

- [ ] 扩展`allRequestLengthsMatchQtValidation()`涵盖20命令×129长度，包括HELLO的SESSION0单独路径；增加合法向量以保证有成功样本。新增错误TYPE、SEQ0、SESSION0/错误HELLO SESSION及非法参数的独立期望测试。
- [ ] 运行Qt回归，确认原校验与独立期望一致；复用RED阶段任务1作为提取校验的行为防护，不伪造必须失败的等价重构测试。
- [ ] 在ProtocolMessages.cpp以相对路径`../../../firmware/core/command_contract.h`包含权威头文件；帧/会话检查先于调用。validateParameters直接调用WRITE_PARAMS规则，不修改参数响应编解码。
- [ ] command_dispatcher.cpp包含同目录头文件，删除匿名validate/parameters重复函数并调用统一校验，保留所有会话、缓存、水位及Backend行为。
- [ ] 运行Qt全部回归和`./scripts/check-firmware-core.ps1`，确认公开对象布局门槛不变；提交H端接入里程碑。

### Task 3: 实际P9接入与拒绝优先级

**Interfaces:** 消费任务1接口。新增`stepper::p9::supportedCommands()`返回九项组成的contract::Support；不是运行时注册器。原控制服务、ControlKind和mailbox接口不改变。

- [ ] 实际`uart_handshake_test.cpp`新增命令行场景`request-contract`。在成功HELLO后对11条未实现请求逐项发送独立合法载荷，断言10；逐项错误长度断言3，适用数值越界断言4。发送事件/遥测号和0xAAAA断言2；错误会话优先11、同SEQ变载荷优先12。
- [ ] 每个拒绝请求前后比较snapshot、控制目标及邮箱占用，没有新pending、使能或心跳续期；重复拒绝得到相同响应。另覆盖支持清单恰好9项、GET_INFO能力字面0x180。
- [ ] 按P9/tests/README.md的g++握手构建命令运行`tests/uart_handshake_test.exe request-contract`，观察旧实现对非法未实现请求返回10的RED。
- [ ] 用apply_patch添加契约副本和command_support.h。在handleFrame原会话/事务检查之后统一校验；错误沿用sendAndRemember；合法但目标未支持返回10。删除knownButUnsupported/重复载荷校验，保留HELLO限制、各执行分支及finishControls；GET_INFO调用真实支持映射。
- [ ] 重新编译运行基线、request-contract、cached-takeover、pending-takeover、running-takeover、info-capabilities；回归Qt控制服务的STOP优先/心跳/完成后应答及P9编码器/TX/echo。
- [ ] 提交P端两个头文件、UART接入、测试和README，排除Keil输出；此时仍未完成ARM/硬件验收。

### Task 4: 副本门槛、ARM构建与文档交付

**Interfaces:** `check-firmware-core.ps1`新增可选参数`[string]$P9Project`；提供时校验`$P9Project/Uart/command_contract.h`与权威文件的字节一致性，再执行原有对象验证；缺失路径/副本或不一致均throw，不得跳过。不提供时保留原独立核心检查行为。

- [ ] 用临时测试目录和apply_patch创建故意不同的副本，运行脚本带P9Project验证RED；换成真实P9路径后必须GREEN。临时测试文件仅在build下，不删除用户文件。
- [ ] 在脚本添加文件一致性检查，不修改编译器选项、构建工程或链接配置。
- [ ] 在P目录使用既有ARM工具重编译uart_protocol.cpp。所有执行命令必须检查LASTEXITCODE；包括Core/Inc、HAL/Legacy、CMSIS及各业务头文件目录。

```powershell
$compiler = 'E:/keil5_2_4/ARM/ARMCLANG/bin/armclang.exe'
$includes = @('Core/Inc','Drivers/STM32F1xx_HAL_Driver/Inc','Drivers/STM32F1xx_HAL_Driver/Inc/Legacy','Drivers/CMSIS/Device/ST/STM32F1xx/Include','Drivers/CMSIS/Include','UserApp','Motor','Encoder','Driver','Memory','Button','Uart') | ForEach-Object { '-I' + $_ }
& $compiler --target=arm-arm-none-eabi -mcpu=cortex-m3 -mthumb -Oz -std=c++11 -fno-exceptions -fno-rtti -fshort-enums -fshort-wchar -ffunction-sections -fdata-sections -DUSE_HAL_DRIVER -DSTM32F103xB @includes -c Uart/uart_protocol.cpp -o MDK-ARM/cbt6_demo3/uart_protocol.o
Push-Location MDK-ARM
try { & 'E:/keil5_2_4/ARM/ARMCLANG/bin/armlink.exe' --via cbt6_demo3/cbt6_demo3.lnp } finally { Pop-Location }
./tests/check-stack.ps1 -Mode normal
```

- [ ] 链接成功后用既有fromelf导出候选HEX并记录SHA256、Code/RO/RW/ZI、告警及新调用链栈估算；超过1024B即失败，不通过增大栈绕过。明确增量链接、未硬件测量，不将旧840B当新结果。
- [ ] 运行完整Qt/P9回归、契约副本检查、静态ISR禁用API检查及55个既有文档链接的更新检查，记录实际新计数。
- [ ] 更新进度台账I16、功能说明、协议手册及新增`docs/validation/2026-10-03-command-contract.md`。记录新错误优先级但保持9/11支持数量及硬件未验收状态，补充以后接入后端的清单规则。
- [ ] 检查两端暂存diff、生成物排除、回归结果和Git状态；提交脚本/文档里程碑，列出两个最终提交ID。最终报告必须明确没有烧录、运动、推送或合并。

## 自审与交接

上述四任务对应设计第3～8节：独立契约、三个接入点、异步执行保护、副本及栈门槛均有归属。设计的未来运动/存储/校准不列入任务，禁止为了“框架完整”添加成功占位后端。

建议Native顺序执行：任务间共享契约接口，四个阶段需要连续验证实际P9路径。计划需用户评审并确认执行方式后开始产品实现；无需新建用户聊天。
