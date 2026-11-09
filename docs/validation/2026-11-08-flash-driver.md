# 2026-11-08 Flash底层遇错退出与参数防护

本轮仅修改P9 Memory/stockpile_f103cb.c内部，保留原void API、结构体、分区、HAL SDK及IRQ/构建配置。未开始硬件写入或运动。

## 变更与证据

- HAL解锁失败不继续擦除；擦除状态或page_error失败即退出页循环并上锁，不继续后续页。
- HAL_FLASHEx_Erase已包含等待；删除驱动紧随其后的额外FLASH_WaitForLastOperation(HAL_MAX_DELAY)。不是将HAL内等待也改成非阻塞。
- 半字/字/双字编程一旦失败立即退出，不把后续数据继续写到同一游标；仅成功推进游标。
- Begin解锁失败置无效游标；新增空指针、区域、对齐和计数检查，用减法/除法避免num*width及地址加法溢出。
- 128KB边界固定于当前F103CB目标；其它容量平台未验证。结束函数显式标记未用参数，主机C编译可保持-Werror而非新增告警抑制。

新增tests/flash_driver_test.cpp及HAL替身，实际Memory/stockpile_f103cb.c由GCC -std=c99编译，再由G++链接测试。首轮旧C代码未用参数在-Werror下编译失败，先用原告警级别得到真实RED：擦除失败仍继续且调用额外等待，退出1。修复后C/C++均-Wall -Wextra -Werror，新编译运行退出0。

测试擦除失败/解锁失败/完整三页、16/32/64编程失败不前进、成功前进、边界、0x80000000计数溢出及null输入。HAL仅为替身，不能据此证明实际Flash时序/控制服务响应。

正常Keil全量0错误/0告警：Code47348/RO5352/RW96/ZI8728；相比上一轮Code+272，其他不变。静态栈992/1024B、MinimumHeadroom32通过。[全量日志](evidence/2026-11-08-flash-driver/full-build.log)。未冒称本轮重跑Qt、实机或所有上层测试。

## 仍不满足实机校准开放条件

源码Drivers/.../Inc/stm32f1xx_hal_flash.h定义FLASH_TIMEOUT_VALUE=50000（50秒），FLASH_WaitForLastOperation依赖HAL_GetTick；HAL自身等待及同bank Flash停顿问题仍在。删除额外HAL_MAX_DELAY不等于得到可靠的短时硬件超时，更不是全流程非阻塞。

旧void存储接口不能向调用者返回完整错误原因；上层依赖游标/擦除与回读检查发现错误。整区操作仍同步，主循环需要真正的分步状态机才能在页/小块之间返回UART轮询；不能递归调用UartProtocolPoll而破坏非重入栈假设。

下一步实现校准专用分步擦除/写入/验证状态机，明确每次调用工作量、错误状态、取消收尾及HAL等待限制，再处理持久化提交标记。公开V1启动/能力仍未开。本轮未烧录/校准/使能/运动，源码文档未提交推送。
