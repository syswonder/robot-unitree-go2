# Robonix-Go2 组会简要汇报（2026-08-09 更新）

## 工作内容

- 在已通过的 MiniCPM-RobotTrack 视觉跟随基础上接入 D435i 对齐深度，实现按人物远近自动调节前进速度的固定距离跟随。
- 新增 Robonix `follow/distance` 能力和中文语音调距：默认每次启动保持 `5 m`，支持“离我远一点 / 靠近一点”每次增减 `1 m`，也支持“设置为 1 米”等绝对距离指令。
- 固定距离模式使用独立启动入口；普通跟随、generation 1 地图、初始位姿、语义地标和中文语音/Nav2 导航均保持原样，可分别直接启动。

## 当前进度

- 已完成约 5 分钟固定距离与语音调距实机验收，现场确认跟随可随目标远近自动控速，持续前进和左右转向效果正常，整体验收通过。
- 测试中成功完成 `5 m → 6 m → 5 m → 1 m → 6 m` 切换；`1 m` 近距离与 `6 m` 远距离跟随均达到预期。下次启动仍从默认 `5 m` 开始，运行中可随时切换。
- 5 分钟窗口内底盘实际运行约 `300.16 s`、累计运动约 `17.0 m`，窗口内无底盘故障或 OOM，结束后进入 `DISARMED`。

## 风险/阻碍/问题

- 当前距离估计依赖 D435i 深度与视觉人物框，遮挡、多人和复杂光照仍需后续专项验证。
- 当前只允许向前追近；人物距离小于设定值时停止前进，不主动倒退拉开距离。

## 下周计划

- 完成固定距离/语音调距代码、实机报告和启动说明的 PR 整理，保证后续可直接进入实机复测。
- 根据演示需要补充遮挡、多人和目标短时丢失场景；不重复已通过的 generation 1 导航及普通跟随阶段。

## 相关 PR

- [RobotTrack 跟随 Draft PR #6](https://github.com/syswonder/robot-unitree-go2/pull/6)：补充固定距离跟随、D435i RGB-D 测距、Robonix 距离能力、中文语音调距和实机验收记录。
- [Go2 完整全栈 PR #1](https://github.com/syswonder/robot-unitree-go2/pull/1)
- [Mapping PR #15](https://github.com/syswonder/service-map-rbnx/pull/15)
- [Navigation PR #9](https://github.com/syswonder/service-navigation-rbnx/pull/9)
- [Client PR #10](https://github.com/syswonder/robonix-client/pull/10)

模型权重、地图、录包、日志、截图和视频继续保留在本地，不进入 Git。
