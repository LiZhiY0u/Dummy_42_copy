# 2026-11-08 编码器第一批增量

用户明确要求减少设计审核、直接开发验证。沿用既有CLEAR_FAULT后显式ENABLE的语义；不改HAL、IRQ、公开类型或协议，不安装工具、不烧录、不操作输出。

下位机main28c40a6上未提交改动：

- Encoder/mt6816_base.cpp校准表合法条件从仅拒绝65535改为拒绝>=51200。每圈200*256细分，合法值0..51199；初始化坏表保持rectifyValid=false，不改Flash内容。
- tests/encoder_spi_test.cpp新增51199接受、51200及65535拒绝、重新初始化恢复；TXE/RXNE/BSY超时、CS释放、有效采样恢复、连续错误计数65535饱和及恢复归零。
- 新增测试先对旧实现执行，51200拒绝断言失败（退出3）；最小修复后同测试退出0。既有寄存器替身仅模拟命令回读为raw=0，不模拟真实奇偶/无磁序列；该部分后续需要脚本采样替身，不能据本轮测试宣称覆盖。

已有MinGW7.3，按固件tests/README.md编译组合新编译编码器和UART集成测试。编码器通过；UART默认/cached-takeover/pending-takeover/running-takeover/info-capabilities五场景均退出0。运行时异常计数饱和测试循环65537次。

隔离副本build/encoder-check-2026-11-08/P9.cbt6_motor_qt，现有Keil -r/-j0全量构建：0错误/0告警，Code46440、RO5340、RW92、ZI8556。对基线Code-4/RO+4，RW/ZI不变，合计大小无增长。正常静态栈992/1024B、MinimumHeadroom32通过，估计路径不变，仍不是运行安全证明。[构建日志](evidence/2026-11-08-encoder/full-build.log)。

本轮新候选没有烧录；板上仍是前一次基础复测候选。没有重新跑Qt/COM4，本轮UART结果是离线替身集成，不是实机通信结果。完整采样异常序列、底层Motor恢复与V1 Gate一致性、实际输出关闭及ISR时间仍待完成。

下一步继续补奇偶/无磁序列测试和控制路径恢复测试；若暴露需改公开接口/HAL/IRQ的缺口再报告影响。源码和文档未提交/推送。

## 后续增量：序列与恢复路径验证

本次已完成此前缺少的脚本采样：仅修改tests/encoder_mocks/spi.h，将SR替身扩为RXNE读取时按脚本注入DR回复；产品寄存器路径不变。编码器测试真实编译Encoder/mt6816_base.cpp，原有边界/超时/计数测试仍通过，新增：

- raw5（角度1、有效偶校验）映射校准值123，首轮成功只消耗两次回复；
- raw4连续三次奇校验失败，raw3连续三次无磁，均退出失败、sampleValid=false，不将保留旧值当作新有效采样；
- raw4→3→5在第三轮恢复成功且清零连续错误计数；
- 缺RXNE时不消耗脚本回复、样本无效；随后RX恢复，下一轮成功。

UART集成增加encoder-recovery场景及测试驱动停止/使能调用计数。通过真实UartProtocolControlTick及ControlService/ControlGate，确认失效调用StopAndReset、fault保持、无效状态CLEAR_FAULT失败、采样恢复不清fault、ENABLE仍拒绝；有效时CLEAR_FAULT成功但不使能，后续独立ENABLE才调用驱动。命令从控制邮箱提交，不冒充本场景覆盖了完整线帧解码；完整RX/parser由原有场景验证。

本轮新编译后编码器测试退出0；UART默认、encoder-recovery、request-contract、三takeover及info-capabilities共7个场景全部退出0。使用与上节一致的已有MinGW和编译参数，测试README已补入口。新增代码全部是测试/文档，没有继续修改产品代码，因此此前ARM候选身份仍适用；本轮未重跑ARM构建或板端测试。

源码检查发现正常TIM4路径先Motor::Tick20kHz再UartProtocolControlTick；采样失败时底层Sleep，后续服务锁存Gate并停止驱动。该检查不等于测试了真实Motor的全部内部状态；完整底层恢复/输出时序仍需要真实Motor替身驱动集成或隔离硬件测试。不要将FakeController调用计数宣称为物理停止证据。

当前剩余：底层Motor恢复路径完整集成、异常时序/栈板端验证。未烧录、未运动、未提交推送。
