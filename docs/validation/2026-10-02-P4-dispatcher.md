# P4第二步：请求去重与命令分发

日期2026-10-02。P4仍进行中，本记录不证明P9硬件已经兼容V1。

## 实现与边界

新增`firmware/core/command_dispatcher.h/.cpp`，无Qt/HAL依赖、无动态内存。Backend接口负责更新安全状态、提供当前会话、建立会话及实际执行命令；目前只有QtTest逻辑后端，尚无连接电机的生产适配层。

- 保存最近4项完整请求/应答，TTL2000ms；完全相同请求直接回放，不再次调用Backend。相同SEQ不同内容返回12 SEQ_CONFLICT。
- 过期与淘汰不清序号水位；只接受模65536前向距离1～32767，跳过SEQ0；时间TTL支持uint32回绕。
- 会话失效/切换清缓存，旧session命令返回11 BAD_SESSION。新token HELLO通过Backend.beginSession判定禁用状态接管；同token重试保留缓存/水位且不续心跳。
- 新请求即使被长度/数值拒绝，也消耗SEQ并缓存错误，不能修改相同SEQ后再执行。HELLO同token也先做序号检查；符合格式的新token才进入新会话逻辑。
- TYPE错误、SEQ0、LEN大于128无应答。合法请求按会话→序号→命令布局→全局数值→Backend处理；覆盖所有V1命令长度及ParameterList结构/范围。设备profile、能力和运行状态仍由真实后端检查。
- Backend Data超过126B或STATUS非法转13 INTERNAL_ERROR，不拷贝超长Data；错误也缓存，防止再次执行。不能据这项长度保护宣称所有Backend应答语义已经验证。
- STOP后重放旧ENABLE仅返回旧确认，不改变禁用状态；Qt UI必须继续以当前状态快照判断使能状态，不能据历史应答恢复运动。
- 适配层需要周期调用Backend.poll，即使串口无新数据；Dispatcher会在每个合法结构请求前poll，不能替代20kHz安全更新。接口单上下文使用，未来ISR邮箱/命令确认异步适配尚未完成。

## 验证

先新增7项行为测试，`build/p4-dispatch-red.log`因新头/源码尚不存在而构建失败。实现后通过，再补充请求长度与参数批次对照、Qt会话整合、异常后端应答。

独立只读评审发现HELLO布局校验先于序号冲突：同token/SEQ修改payload长度会返回BAD_PAYLOAD而非SEQ_CONFLICT。已调整校验顺序，并回归同序号改payload、非法新序号消耗水位及完全相同错误重试。修正一处误导缩进编译警告。

最终命令：

```powershell
.\scripts\build-host.ps1 -Test -BuildName host-p4
.\scripts\check-firmware-core.ps1
```

环境5、Qt帧13、Qt消息15、Qt会话20、固件核心13、分发14，共80 passed、0 failed（含12个init/cleanup项，68个行为测试）。无编译/跨线程警告。最新日志`build/p4-dispatch-complete.log`。

分发12项行为覆盖：重复/修改内容、STOP后旧ENABLE回放、缓存淘汰、精确TTL边界、SEQ与时钟回绕、会话失效/接管、非法请求消耗SEQ、HELLO重复和冲突、后端非法应答、响应布局、20种命令×129种长度共2580个组合、参数批次，以及Qt SessionController通过真实portable Parser/Dispatcher完成HELLO→INFO→STATUS、保持心跳、STOP应答与断开。

整合后端是测试夹具：能力位0，没有遥测/运动/参数的产品实现，不能当成V1下位机完成。它确认两端帧/消息/会话接口能配合运行。

现有ARM Compiler6.7三份cpp在Cortex-M3、C++11、Wall/Wextra/Werror、无异常/RTTI配置下交叉编译通过。ARM静态断言：Dispatcher1196B（含四项缓存）、Parser156B、ControlGate16B；这些持久对象合计1368B，Frame144B另计。尚不包含UART缓冲、后端、调用栈和其他产品对象，最终RAM/Flash须完整链接验证。日志`build/p4-dispatch-arm-final.log`。

没有改P9工程或构建产物，没有打开真实串口、烧录、安装环境、提交/推送。下一步控制周期邮箱、一致快照和真实Backend，然后UART DMA/TX与Keil接入。
