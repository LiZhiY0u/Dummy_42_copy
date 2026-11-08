# 2026-10-06 栈哨兵与调试计数实测

## 结果与边界

P4仍进行中。正常禁用通信回归99通过/0失败/0跳过；取得一份有效的main入口填充至首个协议STOP入口栈快照：低地址连续368B保持0xA5，观察写入跨度656B。不是最坏栈深、不是SP最小值测量，也不包含该STOP后续执行的全部路径。ISR连续最坏耗时、运动和物理停机未验收。

本轮没有修改产品代码、HAL、IRQ、启动、链接或原Keil工程；没有烧录、使能、运动、校准、安装环境、提交或推送。只在隔离的临时工程关闭自动烧录，配置调试初始化文件。用户确认环境已准备；功率输出隔离按本轮确认条件处理，软件没有独立测量物理隔离。

## 固件与方法

- 源码基线：下位机e58a194，上位机97d01a2；使用既有全量候选AXF/map，不重新构建。
- 候选HEX SHA256：`854E904B2E3B95C4EE4529B9967AEAA7A73F4453849D30E936EA4F0E40DD1FA6`；本轮核对磁盘文件，不是Flash独立读回校验。
- 现有Keil5.24/ST-Link，COM8 CH340，115200/8N1。栈区`[0x20001DC8,0x200021C8)`，1024B。
- 临时副本：`C:/Users/Cainiao/AppData/Local/Temp/p9-measure-77fa5e90cd64488ababe5d3be215bc2f/P9.cbt6_motor_qt`。
- 临时工程`tLdApp=0`、`UpdateFlashBeforeDebugging=0`；`LOAD %L NOCODE`仅加载符号，`G,main`在启动库完成后停止。
- main入口PC=0x0800A7A8，MSP/SP=0x200021C8，CONTROL=0。先记录256个32位栈字，再仅向当前MSP以下空闲区填0xA5；记录256字回读全部匹配。条件不满足则拒绝填充。
- 控制服务stopDriver地址0x08008468，计数2：首次初始化停机后，第二次在禁用会话的STOP入口暂停。根据源码调用条件与Qt实际STOP提交路径对应；不把函数名称本身当作物理停机证明。
- 断点处MSP/SP=0x20002028，uwTick=76261；完整256字快照中首次变化字地址0x20001F38，因此连续未改前缀368B、写入跨度656B。这是字级保守观察，不是峰值SP。覆盖启动后初始化、握手/信息/状态、约5秒遥测/心跳和STOP入口，未覆盖完整STOP、突发/畸形帧及所有嵌套分支。

## ISR探针

使用只读DWT/寄存器探针，未写计数器控制寄存器。main入口DWT_CTRL=0x40000001，CYCCNT=6002；TIM4第100次入口CYCCNT=604798，随后入口607199，SP均0x20002120。计数有递增，Keil STATES三个采样均0，不能依赖其时间显示。

两个IRQ入口计数相差2401周期，仅是受断点干扰的入口间隔，不是完整ISR耗时，不换算或宣称WCET。main入口SystemCoreClock变量16000000是配置时钟之前的值，不能用于换算运行ISR。反汇编确认TIM4_IRQHandler尾调用HAL_TIM_IRQHandler，0x08004786的NOP不是可用的ISR出口，故未使用它作为计时终点。

后续须设计DWT/GPIO有界插桩并评审实际入口/异常返回边界及核心时钟；若涉及IRQ/HAL改动，先走框架变更确认。现有证据不足以打开运动能力。

## 失败与修正（保留）

1. 首次绝对AXF路径LOAD报error56；改用项目`%L`后符号加载通过。未证明具体路径解析限制，不归因于硬件。
2. 调试函数定义在LOAD之前，LOAD后调用报error34；移到LOAD之后恢复。未执行到填充的日志不算测量。
3. 复位入口哨兵被启动初始化清零，整栈变化；该轮无效，不据其宣称溢出。改为main入口填充。
4. 聚合Qt测试不能直接用单套件函数名启动，报Unknown test function；改为完整套件并重定向日志。
5. 两轮带断点完整套件均98通过/1失败，STOP应答等待失败。不能算正常通信回归；暂停/栈转储改变调度与心跳时序。脱离调试器后重新验证成功，未据此修改协议产品代码。

## 脱离调试器回归

执行既有`build/host-p4/tests/debug/host_tests.exe -txt`，PATH使用已有Qt/MinGW，`STEPPER_TEST_PORT=COM8`。该轮进程退出0，七套件总计99通过/0失败/0跳过（含初始化/清理）。两轮各255快照、48心跳、STOP确认、600ms后重连，会话错误0；测试断言未发送ENABLE、MOVE_ABSOLUTE、SET_VELOCITY。无重新编译，验证的是现有构建测试程序。

结束时本任务Keil进程已关闭，没有残留调试断点会话；随后完成上述回归。关闭仅本任务PID，不影响用户进程。

## 原始证据

位于[evidence/2026-10-06](evidence/2026-10-06)：[有效栈日志](evidence/2026-10-06/stack-load.log)、[栈脚本](evidence/2026-10-06/probe.ini)、[ISR日志](evidence/2026-10-06/isr-probe.log)、[ISR脚本](evidence/2026-10-06/isr-probe.ini)、[干净回归](evidence/2026-10-06/qt-clean.log)。失败及无效轮日志一并保存。

| 文件 | SHA256 |
|---|---|
| stack-load.log | BEACD4487B4C59BF07EDC98BC40480B6CB5F785A532043C615B5BAAF2B6CB0E7 |
| isr-probe.log | D3ABF74ACC331F9FEA3EC1B5F47EB8C0901D2C50988878B64627747D2BCE96AF |
| qt-clean.log | 875D9982B7DAF0D80CF5A183B9E6A09FE08EED3D12E58030A0485FD27D2DC2A8 |

哨兵可能漏掉仅移动SP未写内存的深度，也可能遇到数据恰为0xA5；单次未改前缀不能替代静态未知调用审查或安全阈值。仍保留992B/1024B静态回归门槛及其假设，不用656B将它下调。
