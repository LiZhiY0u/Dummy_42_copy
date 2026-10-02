# P4第三步：控制周期邮箱与服务框架

日期2026-10-02。P4进行中。新增`firmware/core/control_service.h/.cpp`与`host/tests/tst_control_service.cpp`。

## 已完成

- 三槽邮箱：普通、心跳、STOP各1项；命令与完成结果均按STOP→心跳→普通优先。状态为Empty/Pending/Processing/Completed，未读完成结果占用槽，不能覆盖。
- 主循环提交只表示入队；控制owner的tick调用驱动接口并产生完成结果后才能取出确认。每tick最多处理三条命令，无CRC、IO、Flash、动态内存或递归。
- 请求编号必须非零且在所有占用槽间唯一。STOP在途或完成未读时，拒绝新Enable。
- 先校验STOP会话，再禁用驱动/清目标积分接口，并取消待执行Enable；错误会话的STOP不取消当前合法Enable。
- 每tick检查编码器/校准和500ms心跳；无新请求也会在失联/编码器故障时调用驱动禁用接口。重新建立会话、清故障都不自动使能。
- 位置使能锁存测量位置；速度/电流初始目标0；同模式重复使能不重新锁存，改变模式需先STOP。驱动使能失败回到禁用并报告13 INTERNAL_ERROR。
- 一致快照经临界区复制；位置差用int64计算并饱和到int32。禁用后指令电流和速度/电流目标为0，实测速度仍来自输入测量，不把关闭驱动等同于转轴已静止。

目前只支持Hello/Heartbeat/Enable/Stop/ClearFault/ReadState的控制生命周期，运动目标、参数/校准/存储尚未加入Mailbox。

## 接入约束与未完成部分

CriticalSection是抽象接口。真实ARM实现必须保存/恢复PRIMASK并含编译器内存屏障，不使用阻塞锁；当前测试替身只验证嵌套状态恢复和访问顺序，不能证明真实中断并发安全。

ControlDriver也是抽象接口。disableAndReset必须真正禁用驱动、清目标、规划器和积分；enable必须有界且ISR安全。当前TestDriver只有逻辑状态，不连接GPIO/PWM或实际电机。

ControlService由一个控制周期上下文拥有；Measurement必须由既有控制器提供一致的细分位置、速度和指令电流，避免主循环与ISR同时修改Gate。Snapshot含本机会话元数据，ARM52B；线上Snapshot仍为48B，必须显式编码，不能直接发送struct。

当前CommandDispatcher/CommandBackend是同步接口，尚未把Mailbox完成结果接回协议。后续需要异步适配：入队不得先返回OK，重复待确认请求不能二次入队，完成后才能缓存最终应答。严禁通过阻塞等待ISR完成来掩盖接口差异。

核对P9发现MODE_STOP分支只调用Sleep；ClearIntegral为独立路径且Controller::ClearIntegral是私有成员。GetPosition返回float，不能用它替代高精度原始细分位置快照。真实适配需明确提供整型状态读取和完整停机接口，现有用户代码/Keil改动本轮未改写。

## 验证记录

1. `build/p4-control-red.log`：先增加测试，核心头/源码未实现时构建失败。
2. `build/p4-control-green.log`：首批7项行为通过。
3. `build/p4-control-boundary-red.log`：STOP完成未读仍可提交Enable、不同lane允许同关联ID，两项边界断言失败。新增STOP屏障与跨槽ID唯一性检查后通过。
4. 独立只读评审未发现重要核心缺陷，结论不涵盖真实CriticalSection/ControlDriver。

最终命令：

```powershell
.\scripts\build-host.ps1 -Test -BuildName host-p4
.\scripts\check-firmware-core.ps1
```

环境5、Qt帧13、Qt消息15、Qt会话20、固件核心13、分发14、控制服务12，总92 passed、0 failed（含14个init/cleanup项，78个行为测试），无编译或跨线程警告。日志`build/p4-control-verified.log`。

ARM Compiler6.7四个核心cpp用Cortex-M3/C++11/Wall/Wextra/Werror交叉编译通过。静态断言：Mailbox232B、Service92B、内部Snapshot52B；Parser156B、Dispatcher1196B保持不变。Parser+Dispatcher+Mailbox+Service持久对象合计1676B，Service已包含Gate，不能再次加算。实际CriticalSection/Driver对象、UART缓冲、完成应答和栈不在此数内。日志`build/p4-control-arm-layout.log`。

未测ISR WCET、临界区屏蔽时间、完整链接RAM/Flash、实际停机延时。未打开串口/烧录，未修改P9或安装工具，未提交/推送。

下一步：协议异步完成适配、P9整型快照/完整停止接口与真实ControlDriver；随后统一DMA/TX、Keil构建和禁用状态联调。
