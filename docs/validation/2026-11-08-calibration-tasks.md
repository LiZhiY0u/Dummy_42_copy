# 2026-11-08 校准任务记录核心（尚未接启动）

本轮继续V1校准接入，先补纯记录核心，不打开未完整验证的启动能力。新增P9 Uart/calibration_tasks.h与tests/calibration_tasks_test.cpp，不变更已发布线协议、公共ControlService接口或支持列表。

## 已实现与新跑验证

- 同会话单活动任务，task_id非零，错误会话不接受/查询；会话切换清记录，同会话刷新保留，ID分配器不随会话重置。
- 校准phase遵循协议0等待/1采集/2分析/3写入/4验证/5完成/6失败取消，通用任务phase按0等待/1运行/4取消/5失败/6保存验证完成编码。
- 进度不回退、不跳阶段，99以下为过程，只有显式验证阶段后才允许100%成功和storage_verified=1；失败/取消storage_verified=0。
- 最近4个终态历史不依赖2秒请求缓存时间，超过4个淘汰最旧；取消后迟到progress/complete不能覆盖已保留的取消结果。
- TASK_QUERY 12B及CALIBRATE_QUERY 13B响应体（不含公共STATUS）用独立字节向量验证小端和字段顺序。

MinGW7.3，-std=c++11 -Wall -Wextra -Werror，新编译任务测试退出0；本机sizeof任务管理器112B（固定数组，无堆）。暂未实例化进生产后端，因此这不是目标固件RAM增长，也未声称ARM布局验证。既有真实校准Tick电流测试本轮运行退出0，没有冒称新编译该测试。

突变验证：临时移除completeVerified的phase==4保护，新编译测试退出1（未验证即完成被抓到），恢复保护再编译后退出0，突变未留在源码。

## 明确限制与下一步

这是顺序调用的状态记录，不执行校准/Flash，不替代IRQ与主循环的互斥设计；header要求控制周期单写者，主循环复制时排除IRQ。需要接入时再用实际critical boundary验证，不能把顺序取消测试当并发Flash验收。

本轮没有接CALIBRATE_START/QUERY/CANCEL/TASK_QUERY分发，没有改变9命令清单或0x180能力；Qt仍会得到当前未实现命令UNSUPPORTED。后端必须真正读回验证存储后才调用completeVerified，不能仅置phase4就代表硬件已验证。

下一步：用内部控制周期请求接入安全启动/停止与任务归属，保留既有STOP优先级；再将校准阶段/Flash验证结果接入终态。Flash收尾、原始表损坏/断电及取消竞争仍未解决。未重跑全量ARM（当前新增模块不参与生产编译）、未烧录/使能/运动，未提交推送。
