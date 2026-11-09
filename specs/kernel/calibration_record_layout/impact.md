# 校准记录及程序区边界影响

2026-11-08，按用户继续开发/减少反复设计审核要求实施。本文件记录实际影响，不伪造SPEC FROZEN或已完成人工ABI/回滚验收。

产品文件：Encoder/calibration_record.h（新20B固定格式）；mt6816_base.cpp启动校验；encoder_calibrator_base.cpp元数据阶段；Uart/uart_protocol.cpp容量拒绝；Memory/stockpile_config.h程序区宏；MDK-ARM/cbt6_demo3.uvprojx两个ROM配置尺寸。

现有Stockpile API/结构体、ControlService API及串口帧/revision不变；新增模块内部记录类型，未改HAL SDK/IRQ/启动向量/工具链。

程序区0x08000000..0x080177FF（94KB），记录页0x08017800..0x08017BFF，表0x08017C00..0x0801FBFF，配置0x0801FC00..0x0801FFFF。两个链接ROM尺寸改0x17800，物理CPU描述仍128KB。原固件区域宏47KB与已增长程序不符，改为94KB，未挪动表与配置地址。新程序不能接受旧无记录校准表。

主机新编译四类模块回归及16UART场景通过；正常/通信诊断/echo/TX四配置Keil全量均0错误0告警，正常栈992B/32B门槛通过。map LR_IROM1 Max0x17800且Size0xD360，未进入记录区。容量依据本地CMSIS FLASHSIZE_BASE0x1FFFF7E0，启动/写入只接受128KB。

旧链接器L6776W告警已复现、记录并通过将committed检查改内部inline链接解决，没有抑制告警。未烧录该布局、未做真实Flash断电/重启或跨容量板验证；主机64KB替身仅证明拒绝逻辑。新增记录API的物理提交/取消语义见日期化记录，后续外部分发不能声称取消会回滚已完成提交。
