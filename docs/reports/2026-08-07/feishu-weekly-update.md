# Robonix-Go2 组会简要汇报（2026-08-07）

## 工作内容

- 完成 MiniCPM-RobotTrack 官方模型、D435i、RTX 4070 工作站和 Go2 底盘的视觉跟随集成，模型依赖与权重均已在本地校验。
- 将 RobotTrack 封装为 Robonix Primitive `go2_robottrack`，完成 Robonix codegen、lifecycle 和 manifest 接入；provider 由 Robonix 全栈统一初始化、激活和停止。
- 打通“D435i 画面 → RobotTrack 推理 → Robonix provider → 速度源选择 → 原有平滑/guard/Go2 单控制器底盘链”，并完成网页监控，沿用停止、取消、看门狗和遥控器接管能力。
- generation 1 地图、初始位姿、语义地标及已通过的中文语音/Nav2 导航成果全部保留，跟随功能作为独立模式接入。

## 当前进度

- 已完成 45 秒和 75 秒两轮正式实机跟随；运行时不是模型脚本直接调用 Unitree SDK，而是由 Robonix manifest 启动 provider，并通过原有 Go2 chassis owner 执行。最新 75 秒复测累计行走约 `12.36 m`，持续前进及左右转向均正常。
- 正式窗口内前进速度不超过 `0.50 m/s`、转向速度不超过 `0.30 rad/s`，Go2 全程保持经典步态 `2010`；D435i 画面持续实时更新。
- 测试结束后约 `0.61 s` 明确停止并进入 `DISARMED`。本次测试现已结束，机器人由现场人员手动趴下，跟随全栈、模型服务和 D435i 桥接均已关闭，日志、录包和视频材料已保留。

## 风险/阻碍/问题

- 当前跟随主要依赖视觉模型，尚未专门验收目标长时间丢失、遮挡、多人干扰及复杂光照场景。
- RobotTrack 速度未经过 Nav2 障碍层，视觉跟随不等同于已完成 LiDAR 动态避障融合。

## 下周计划

- 根据演示需求，继续验证目标丢失后停止与重新进入画面后的恢复，并补充遮挡、多人及动态场景记录。
- 整理跟随功能的一键启动与结果材料；原 generation 1 导航保持可直接复用，不重复已通过的实机阶段。

## 相关 PR

- [RobotTrack 跟随 Draft PR #6](https://github.com/syswonder/robot-unitree-go2/pull/6)：新增 `go2_robottrack` Primitive、D435i/模型推理链、速度源选择及 45/75 秒实测记录；分支 `agent/minicpm-robottrack-follow`，基于下方完整全栈 PR #1。
- [Go2 完整全栈 PR #1](https://github.com/syswonder/robot-unitree-go2/pull/1)
- [Mapping PR #15](https://github.com/syswonder/service-map-rbnx/pull/15)
- [Navigation PR #9](https://github.com/syswonder/service-navigation-rbnx/pull/9)：已补充 RobotTrack 所需的可选原始速度输出提交 `d10fe3d`，默认 Nav2 路径不变。
- [Client PR #10](https://github.com/syswonder/robonix-client/pull/10)

RobotTrack 已作为独立 Draft PR 提交，没有扩写已进入评审状态的 PR #1；模型权重、录包、日志、截图和视频继续保留在本地，不进入 Git。
