# 文件导览

[完整目录树与逐文件说明](../README.md#文件导览)

| 要改什么 | 入口 |
| --- | --- |
| 主控阶段、起飞或降落 | [主控代码与配置](../src/fly_mission/README.md) |
| 建图、覆盖路线或返航 | [探索规划器代码与配置](../src/exploration_planner/README.md) |
| 点云裁剪、杆体或圆环识别 | [感知模块](../src/perception/README.md) |
| 任务消息字段 | [消息定义](../src/acfly_interfaces/msg/)；修改后同步核对主控、规划器及 [跨节点测试](../src/acfly_bringup/test/) |
| 调整参数默认值 | [按用途查找 YAML](../config/README.md) |
| 修改配图 | [Mermaid 源文件与导出方法](assets/README.md) |
| 排查异常或验证修改 | [故障定位表](MAINTENANCE.md)、[构建与验证](PROJECT_OVERVIEW.md#验证) |
