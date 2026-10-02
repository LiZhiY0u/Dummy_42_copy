# 串口 V1：协议 revision 1

2026-10-03 状态更新：UART/DMA 与 P9 基础控制边界已接入，诊断固件实机通信闭环已验证；正常模式尚未实机验收，运动目标/参数/校准后端仍未接入。新源码能力位为 0x180；HELLO 允许原帧有界重试一次。以下首段保留为历史说明，不代表最新实现状态。当前说明见 [使用手册](串口协议V1使用手册.md) 和 [参考与加固记录](2026-10-03-协议参考与加固.md)。

日期：2026-10-02。状态：revision 1为两端实现基线。Qt会话/模拟通信、固件平台无关帧层/逻辑安全Gate/去重分发已软件验证；生产控制后端、DMA与P9控制环尚未接入，不能宣称真实设备已兼容。修改字段长度、编号或语义必须更新revision与共同测试向量。

## 已实现约定

`AA55 | version:u8 | type:u8 | length:u16 | session:u32 | sequence:u16 | command:u16 | payload:N | crc:u16`。

头长14B，整体16+N，N最大128。版本1；type=1请求/2应答/3事件/4遥测。多字节小端序，payload不通过struct内存隐式编码。

CRC-16/CCITT-FALSE：poly0x1021、init0xFFFF、无反射、XorOut0；覆盖偏移2至CRC前一字节，线上低字节先发。标准向量123456789→0x29B1。

ProtocolCodec将任意接收块拆成完整Frame，保留数据最大144B；完整Payload内的AA55不被当成边界。错误CRC/版本/类型/长度后向后移动1B寻找帧头；候选半帧从开始接收起达到100ms时清空，下一帧可重新同步。feed使用单调时钟ms；上层须以定时器调用feed(empty, nowMs)执行无新数据时的超时清理。

encode遇到payload超过128或非法帧类型返回空数组；传输层须将其当成本地错误，不发送。帧层只检验结构和CRC，不负责会话合法性、命令范围与是否允许运动；这些由后续SessionController和固件CommandDispatcher处理。

## 测试向量

MOVE_ABS示例：session=0x11223344、sequence=1、command=0x0103、position=51200、max_velocity=51200、acceleration=102400。

`AA 55 01 01 0C 00 44 33 22 11 01 00 03 01 00 C8 00 00 00 C8 00 00 00 90 01 00 FC A7`

CRC=0xA7FC；这不是对真实电机发出的指令。

负速度Payload示例：`00 38 FF FF AA 55 00 00`，前4B为int32 -51200，后4B为uint32 21930；该向量同时验证Payload内帧头不破坏解析。

## 会话与事务

HELLO请求SESSION=0，Payload为上位机新生成的非零u32 token。请求SEQ非零。设备只有输出禁用时允许新token接管；成功应答SESSION等于token，STATUS=OK，Data为u16 revision=1。尚未分配会话时的HELLO错误应答可SESSION=0，且仅有2B错误STATUS。重复同token的HELLO不能重置已处理序号或清故障。

除HELLO外，请求SESSION必须等于活动会话。下位机重启后session失效且禁用；旧会话请求不能执行。上位机重连生成新token，清空旧事务和运动队列。session token不是安全认证。

请求SEQ为1～65535，同会话递增，65535后为1。设备保存最近4个已处理请求的完整内容及响应，TTL=2000ms；相同session/seq/cmd/payload返回已有响应，相同seq但内容变化返回SEQ_CONFLICT。TTL到期/缓存淘汰后仍保留最近新请求的序号水位：仅接受模65536前向距离1～32767的新SEQ，旧SEQ返回SEQ_CONFLICT，不能重新执行。4项完整最大请求/响应缓存约1.2KiB；P4必须结合其他缓冲核对链接map。

HELLO成功应开始500ms心跳计时。上位机在握手后就每100ms发送HEARTBEAT，包括INFO/PARAM查询尚未完成时；只有当前会话有效HEARTBEAT续期。STOP优先于心跳，心跳优先于普通请求。最多同时等待1个普通事务、1个心跳、1个STOP应答，应用层普通待发队列上限16；不在系统串口缓冲预先堆积运动命令。

普通应答超时200ms；只读GET_INFO/GET_STATUS/TASK_QUERY/READ_PARAMS/CALIBRATE_QUERY重试1次，使用原SEQ及完全相同内容。HEARTBEAT下一周期使用新SEQ；运动、校准、保存不自动重试。超时记录为结果未知，通过状态/任务查询恢复结论。事件与遥测SEQ为独立u16计数，允许回绕0；不参与请求水位。

## 消息布局与错误码

以下u8/u16/u32/i32均固定宽度、小端序，按顺序紧密排列，没有隐式padding。所有应答Payload=`u16 STATUS + Data`。错误STATUS（2～13）Data为空；所有长度都必须精确匹配，不能接受尾随字节。

STATUS：0 OK；1 ACCEPTED；2 UNKNOWN_CMD；3 BAD_PAYLOAD；4 OUT_OF_RANGE；5 WRONG_STATE；6 NOT_CALIBRATED；7 ENCODER_FAULT；8 BUSY；9 STORAGE_ERROR；10 UNSUPPORTED；11 BAD_SESSION；12 SEQ_CONFLICT；13 INTERNAL_ERROR。

CRC/帧结构错误直接丢弃。合法帧的处理顺序：会话/序号 → 命令与长度/type → 数值范围 → 能力 → 控制权/状态 → 执行。CMD事件号/遥测号不能作为请求。状态校验失败不得修改目标或参数。

| CMD | 请求Payload | 成功STATUS与Data（长度不含STATUS） |
|---|---|---|
| 0001 HELLO | u32 token（4B） | OK；u16 revision（2B） |
| 0002 GET_INFO | 空 | OK；DeviceInfo（50B） |
| 0003 GET_STATUS | 空 | OK；Snapshot（48B） |
| 0004 HEARTBEAT | 空 | OK；u32 uptime_ms（4B） |
| 0005 TASK_QUERY | u32 task_id（4B） | OK；u32 task_id,u8 kind,u8 phase,u16 result,u32 timestamp（12B） |
| 0101 ENABLE | u8 mode（1B） | OK；u8 accepted_mode（1B） |
| 0102 DISABLE | 空 | OK；Snapshot（48B） |
| 0103 MOVE_ABS | i32 position,u32 max_velocity,u32 acceleration（12B） | ACCEPTED；u32 task_id，接受的上述12B（16B） |
| 0104 SET_VELOCITY | i32 velocity,u32 acceleration（8B） | OK；实际接受的同格式（8B） |
| 0105 SET_CURRENT | i32 current_mA（4B） | OK；i32 accepted_current_mA（4B） |
| 0106 STOP | 空 | OK；Snapshot（48B） |
| 0107 SET_ZERO | 空 | OK；i32 new_home_offset（4B） |
| 0108 CLEAR_FAULT | 空 | OK；Snapshot（48B） |
| 0201 READ_PARAMS | 空 | OK；u32 revision, ParameterList（变长） |
| 0202 WRITE_PARAMS | ParameterList | OK；u32 new_revision, 实际接受的ParameterList（变长） |
| 0203 SAVE_PARAMS | u32 expected_revision（4B） | ACCEPTED；u32 task_id,u32 revision（8B） |
| 0301 CALIBRATE_START | 空 | ACCEPTED；u32 task_id（4B） |
| 0302 CALIBRATE_QUERY | u32 task_id（4B） | OK；u32 task_id,u8 phase,u8 percent,u16 result,u8 storage_verified,u32 timestamp（13B） |
| 0303 CALIBRATE_CANCEL | u32 task_id（4B） | OK；u32 task_id（4B），表示已停止运动并接收取消，存储收尾另报 |
| 0401 TELEMETRY_CONFIG | u16 period_ms（2B） | OK；u16 accepted_period_ms（2B） |

位置与设定值可为负；速度/电流不接受INT32_MIN，避免取绝对值溢出。限速与加速度为1～INT32_MAX；电流和实际运行上限再对照DeviceInfo。Telemetry周期只支持0/10/20/50/100ms，0关闭。任务ID为非零u32，仅当前会话有效。

STOP/DISABLE取消排队运动、清目标和积分并关闭输出；不是受控减速或硬件急停。CLEAR_FAULT后仍禁用。ENABLE mode只能0/1/2，位置使能锁存当前实际位置，速度/电流使能目标为0。只有已使能匹配模式允许设目标；参数写入、SET_ZERO、SAVE_PARAMS与校准启动需要输出禁用。校准中STOP与失联立即停止运动。

## DeviceInfo（50B，不含STATUS）

| 偏移 | 字段 | 类型 |
|---:|---|---|
| 0 | protocol_revision=1 | u16 |
| 2/4/6 | firmware major/minor/patch | 各u16 |
| 8 | hardware_revision | u16 |
| 10 | UID，原始12B，不是字符串 | 12B |
| 22 | capabilities | u32 |
| 26 | units_per_rev | u32 |
| 30 | current_max_mA | u32 |
| 34 | calibration_current_max_mA | u32 |
| 38 | velocity_max（细分/s） | u32 |
| 42 | acceleration_max（细分/s²） | u32 |
| 46 | parameter_revision | u32 |

units及四项最大值必须1～INT32_MAX。未知revision拒绝接管设备；未知能力位保留但不启用未知功能。

能力bit0位置、bit1速度、bit2电流、bit3参数读写、bit4保存、bit5校准、bit6逻辑零点、bit7清故障、bit8遥测。HELLO/INFO/STATUS/HEARTBEAT/STOP/DISABLE为核心命令。设备必须按真实实现报告能力，不能因为枚举存在就宣称支持。

上限来自经过配置的固件设备profile，未通过硬件验证前不能把它宣传为硬件额定能力。运行限流与校准限流分开，避免把现有1000mA运行默认值误作校准2000mA的同一边界。

## Snapshot（48B）与主动消息

| 偏移 | 字段 | 类型 |
|---:|---|---|
| 0/4 | timestamp_ms/sample_counter | 各u32 |
| 8/12 | position/target_position | 各i32 |
| 16/20 | velocity/target_velocity | 各i32 |
| 24/28 | current_command_mA/target_current_mA | 各i32 |
| 32 | position_error（目标减实际，宽整数计算后检查） | i32 |
| 36 | fault_bits | u32 |
| 40/41 | state/mode | 各u8 |
| 42/43 | calibrated/encoder_valid | 各u8，只能0/1 |
| 44/46 | encoder_error_count/tx_drop_count | 各u16，饱和计数 |

状态：0 DISABLED、1 READY、2 RUNNING、3 CALIBRATING、4 FAULT。模式：0位置、1速度、2电流、3无模式。故障bit0编码器、1堵转、2通信失联、3存储、4过载、5校准失败；未知故障位仍视为故障并显示原始码。

TASK_QUERY kind=1位置运动、2参数保存、3校准；phase=0等待、1运行、2规划完成、3实际完成、4取消、5失败、6保存验证完成。保留当前活动任务及最近4个终态任务，直到会话切换或新结果挤出最旧项；未知/淘汰task_id返回OUT_OF_RANGE，Qt保留“结果未知”，不能当成成功或自动重发。这个任务结果历史独立于2000ms请求去重缓存。

遥测：TYPE=4、CMD=0402、Payload=Snapshot。任务事件：TYPE=3、CMD=0501、Payload=`u32 task_id,u8 event,u8 reserved=0,u16 result,u32 timestamp`（12B）。event=1启动、2规划完成、3实际到位、4取消、5失败、6保存验证完成；只有3/4/5/6是终态。规划完成不能被UI当作实际到位。result使用STATUS编码。

故障事件：TYPE=3、CMD=0502、Payload=`u32 fault_bits,u8 state,u8 mode,u32 timestamp`（10B），故障位是当前完整快照，不能仅发送新增位导致丢事件后信息不全。CALIBRATE_QUERY phase=0等待、1采集、2分析、3写入、4验证、5完成、6失败/取消；percent=0～100，result为STATUS，storage_verified只能0/1。

消息校验只保证格式合法；P3还必须检验活动会话、命令/SEQ与原请求匹配及任务ID关联，才能更新业务状态。

## ParameterList与数值边界

格式：`u16 schema=1,u16 count`，随后count项`u16 id,u8 type,u8 length,value`。type=1 i32/4B、2 u32/4B、3 bool/1B（值0/1）。count=1～15，ID唯一，未知ID、类型/长度不匹配、尾随字节、越界均整批拒绝。WRITE可提交子集；READ返回实现的参数集，不支持参数时返回UNSUPPORTED。

| ID | 参数 | 类型 | 协议边界/设备二次校验 |
|---|---|---|---|
| 0001 | 运行电流上限mA | u32 | 1～DeviceInfo.current_max_mA |
| 0002 | 速度上限 | u32 | 1～DeviceInfo.velocity_max |
| 0003 | 加速度 | u32 | 1～DeviceInfo.acceleration_max |
| 0004 | 校准电流mA | u32 | 1～DeviceInfo.calibration_current_max_mA |
| 0101/0102/0103 | 速度PID kp/ki/kd | i32 | 0～255 |
| 0201/0202/0203/0204 | 位置DCE kp/kv/ki/kd | i32 | 0～4095 |
| 0301 | 堵转保护 | bool | 0/1 |
| 0401 | 实际到位位置容差 | u32 | 1～51200细分 |
| 0402 | 实际到位速度阈值 | u32 | 0～51200细分/s |
| 0403 | 到位连续保持时间 | u32 | 10～2000ms |

15项完整ParameterList为121B（14个4B整数条目+1个bool条目+4B列表头）；READ应答增加revision4B及STATUS2B，共127B，低于128B限制。此版不采用分包参数事务；新增第16项必须设计分页/新schema，不能直接塞入现有报文。

validateRequest只做协议全局边界检查（四个u32上限1～INT32_MAX），不假装已知道设备profile。设备/业务服务需要再检查GET_INFO上限、组合约束和状态。完整列表示例中的100/100/100到位阈值仅是测试数据；实际默认值由P5机械验收确定。

系数范围包括现有默认PID5/30/0及DCE200/80/300/250。依据现有限幅，PID最大单项P/I乘积约255×1048576，D项最坏速度误差变化约255×2097152；DCE单项最大约4095×4000。但既有积分累加、位置差与负数移位仍不能据此证明无溢出/无未定义行为。P4必须把差值、乘积、积分及求和升级为int64中间值，再饱和/检查回写，不以限制系数替代该修复。

MOVE任务超过120000ms未实际到位则故障终止并禁用；实际到位必须规划结束且误差/速度连续符合0401～0403。这个超时是第一版软件策略，不是机械响应时间的实测承诺。事件丢失时以GET_STATUS和TASK_QUERY恢复结论。

## 软件交付范围

已实现ProtocolCodec、ProtocolMessages：请求布局校验、响应STATUS/布局校验、Info/Snapshot编解码、主动事件校验。解码失败保持输出对象原值，避免半更新状态。黄金帧文件为golden-frames.json，独立CRC生成后由C++编码和解析双向核对。

已软件验证Qt运行会话、固件平台无关去重分发和逻辑心跳门控；尚未实现生产控制后端/ISR邮箱/UART DMA、设备profile动态限值的实机应用、参数生效、运动和存储。软件夹具通过不能代替P4完整固件构建与硬件验收。
