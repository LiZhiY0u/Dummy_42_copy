# 2026-11-08 统一校准电流及真实校准Tick测试

本轮继续安全校准准备，不启动实机。已知用户条件3A额定/12V/空载/可断电；首次软件指令上限沿用已有校准循环的1000mA，不能据此宣称线圈实测电流。没有提高到3A。

## 修改

- 校准六个输出阶段不再硬编码1000，统一读取全局motor.config.motionParams.caliCurrent。
- Tick20kHz入口要求isTriggered且电流在1..1000mA；取消、<=0或>1000时驱动Sleep、状态CALI_DISABLE、清触发，不继续采样或输出。原编码器失败路径仍停止。
- main的BoardConfig默认calibrationCurrent及Motor构造默认caliCurrent从2000改为1000，与实际指令来源一致。不改结构体布局、HAL/IRQ分发、协议或Flash布局。
- 计算状态输出电流0；本轮不更改主循环Flash算法，也未接入V1校准启动命令。

## 新测试与RED/GREEN

新增tests/calibration_current_test.cpp，编译真实Encoder/encoder_calibrator_base.cpp，Motor配置、编码器读取、驱动及Flash边界为替身。测试控制所有六个输出状态，在250/500/1000指令下检查实际驱动调用参数；非法配置-1/0/1001/3000不输出且取消；取消后即使再次调用Tick也休眠；编码器失败无输出；计算阶段输出0，所有被测IRQ路径Flash计数0。

第一轮测试缺MT6816及initializer_list头文件，编译失败，仅属测试构建错误；补头文件后旧实现真实断言失败退出1（配置250而输出1000）。产品最小修复后新编译测试退出0。编译命令已加入固件tests/README.md，使用现有MinGW7.3，-std=c++11 -Wall -Wextra及既有头告警抑制。

同轮运行既有encoder_spi_test及12个UART离线场景，全部退出0；这两类exe来自前轮已有构建，本轮没有声称重新编译它们。新增校准测试则本轮新编译。三类测试都不连接硬件。

## ARM验证

隔离副本build/calibration-current-check-2026-11-08/P9.cbt6_motor_qt，Keil正常配置全量重建，0错误/0告警：Code46564/RO5344/RW92/ZI8620。对前轮取消保护Code+8、RO-8、RW/ZI不变，合计不增加。静态栈992/1024B、MinimumHeadroom32通过，未改变门槛。[完整构建日志](evidence/2026-11-08-calibration-current/full-build.log)。

## 剩余边界

输出阶段使用的是配置指令，不是实测电流；低电流是否足以校准尚未实机验证。非法配置在当前层取消但未新增独立错误码，后续V1启动必须先校验并返回明确失败，不能将CALI_DISABLE/旧errorCode==0当任务成功。

V1安全启动、TASK_QUERY/终态、Flash写入阶段取消竞争和写入验证仍未完成。未满足前不触发旧校准，不用调试器置触发。板上仍为79182D编码器候选，未烧录本轮校准电流/取消改动；未使能/运动/校准、未提交推送。
