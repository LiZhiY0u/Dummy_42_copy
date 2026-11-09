# 校准复位保持与只读容量检查

2026-11-10，用户“下一步”续接校准后的复位保持验证。本轮未烧录、清故障、校准、使能或发送目标运动。

使用已安装OpenOCD0.12.0（E:/clion_for_stm32/OpenOCD-20230621-0.12.0），不是新安装环境。COM4复位前基础七请求成功：calibrated1、encoderValid1、mode3、fault4（通信超时），STOP应答正常。

ST-Link接口执行init、reset halt、只读容量/记录、resume、shutdown。复位停于pc0x08000194、msp0x200023c8，随后恢复运行；没有Flash写入/擦除。第一条mdh/mdw未在工具返回中显示数据，不以其作为容量证据。

复位后基础七请求成功：UID不变，cap0x1A0；STOP与两份状态均state0/mode3、calibrated1、encoderValid1、fault0。说明本次复位后启动校验接受已提交校准表，并保持禁用，没有自动恢复运动。未做断电保持或耐久测试。

随后不再复位，用OpenOCD只读并显式echo read_memory得到：

```text
FLASH_KIB=0x80
CAL_RECORD=0x314c4143 0x1 0x8000 0x38608726 0x54494d43
```

分别对应芯片容量寄存器128KB、CAL1/version1/32768B/CRC32=38608726/完整提交标记；容量地址0x1FFFF7E0，记录地址0x08017800。没有独立导出整表重算CRC，本次整表有效依据为实际启动校验和calibrated1，不夸大为外部独立回读全片。

## 新发现与下一步

两次OpenOCD均报告STLINK V2J45S7、Target voltage:6.525897。该目标电压读数异常，可能来自探针测量/接线/供电等，未测定原因，也不能将其当作真实MCU电压。先不继续实际运动，需用户用万用表核对MCU3.3V电源与GND，确认探针接线/参考电压。用户此前12V是电机供电，不可混为MCU电源或ST-Link测量值。

当前产品仍已烧录7D5932DF。未新构建ARM/Qt或运行软件套件，未提交推送。下一步先核实电压异常，再开发/验证单一受限低速短程运动目标；当前运动目标命令仍未实现，不能直接用ENABLE代替完整目标流程。
