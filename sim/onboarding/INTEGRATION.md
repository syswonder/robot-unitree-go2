# Go2 本体接口与仿真负责人交接

## 接入链路

```text
Robonix 任务 / Navigation / 跟随能力
                 ↓ 速度意图（前进、侧移、转向）
仿真 chassis Primitive → ROS 2 / Bridge → Go2 locomotion 控制器
                                                ↓ 关节动作 / 力矩
                                  MuJoCo 的 Go2 MJCF + 场景
                                                ↓
                              真正的仿真状态、传感器、仿真时钟
                                                ↓
                                    Bridge → ROS 2 / TF → Robonix
```

本目录完成最下面本体模型及其输入输出约定，不实现图中的完整运行闭环。
真机链路的 `ROS adapter → SDK daemon → SportClient.Move` 不在仿真中复用；
仿真应由 locomotion 控制器把速度意图转换成腿部动作。不得用直接平移/旋转 base qpos
冒充真实足式动力学行走。

## 必须按名映射的本体事实

| 项目 | 固定上游模型的内容 |
| --- | --- |
| 根刚体 | `base_link`，一个无名称 freejoint |
| 关节数 | 12 个 hinge；qpos 为 19、qvel 为 18（Native 检查另行验证） |
| SDK / actuator 顺序 | FR → FL → RR → RL；每条腿 hip → thigh → calf |
| XML joint 遍历顺序 | FL → FR → RL → RR；不能直接按数组位置与 SDK 对齐 |
| 角度、角速度、力矩 | rad、rad/s、N·m |
| motor 控制范围 | hip/thigh ±23.7，calf ±45.43；以编译后 `actuator_ctrlrange` 为准 |
| 浮基姿态 | MuJoCo quaternion 为 w,x,y,z；ROS 消息为 x,y,z,w |
| 质量 | XML 显式惯性质量合计约 15.206408 kg；不是 EDU 附加 Orin/雷达/相机后的实测总质量 |
| 初始姿态 | `home` keyframe；腿部角度 0、0.9、-1.8；不是步态策略 |
| `home.ctrl` | 原文件含类似角度的数值，但 motor 仍是力矩输入；不能据此推断 position servo |
| IMU site | `imu` 相对 base_link 为 (-0.02557, 0, 0.04232) m |
| frame_pos / frame_vel | 绑定 IMU site，不是 base_link 原点；做 odom 时需处理刚体偏移 |
| 渲染 / 碰撞 | visual group 2、collision group 3；对接参考框架时重新确认射线排除分组 |

XML motor 名为 `FR_hip` 等，joint 名为 `FR_hip_joint` 等；控制输入不能依赖“名称相似所以
数组顺序相同”。`inspect_model.py` 输出静态映射；`check_native.py` 输出真实编译地址。

官方 Python bridge 的控制语义是力矩前馈加位置/速度误差反馈；它只说明如何执行给定关节命令，
不会从前进速度自行生成行走动作。新策略需核对观测顺序、动作归一化、默认关节角、PD 增益、
控制周期和 reset 隐状态。不要直接使用 Ranger 的轮式控制器或照抄另一种机器人的策略配置。

## 现有代码如何复用

1. 从本目录取得固定来源模型、许可证、资产索引；模型和环境分开维护。
2. 将本体插入仿真负责人已有框架的 `assets/robots/<id>/`。使用生成的原模型入口
   `vendor/go2.xml` 组装场景；独立编译检查使用 `vendor/scene.xml`。
3. 在 Native 的机器人注册表选择 Go2 控制器；控制器就绪前不要填一个虚假的 Web
   `controller.module`，也不要把不存在的 `controller.js` 注册进默认目录。
4. 对齐 `command / step / state / reset`。Bridge 是否仍硬编码 Piper arm/pick 状态，
   是否允许 Go2 无机械臂，必须在实际框架中检查；不可仅加一条 robot ID 就算接入。
5. 速度使用 m/s、rad/s，约定 base 前 x、左 y、上 z；明确前进、侧移、原地转向支持范围。
   过期、停止、reset、掉线应清理速度目标与策略状态，沿用已有单控制器所有权。
6. 复用本仓库 chassis 的上层 capability 语义，但使用仿真驱动。仿真 primitive
   首帧初始化应等待真正的 odom/传感器，不可常量返回 ACTIVE。

## ROS / 感知对接目标（不是本目录已发布的话题）

| 接口 | 消息和约定 |
| --- | --- |
| 仿真速度命令 | `geometry_msgs/Twist`；由仿真专用 remap/namespace 接收，不连接真机 |
| odom / TF | `nav_msgs/Odometry`、`odom → base_link`；位姿在 odom，twist 在 child frame |
| joint_states | `sensor_msgs/JointState`；按名称，rad/rad/s，URDF 使用同名关节 |
| IMU | `sensor_msgs/Imu`；注明 frame、单位、重力/加速度语义 |
| 时钟 | `/clock` 来源于 `data.time`；下游 `use_sim_time=true`，不混系统时钟 |
| 2D / 3D 雷达 | LaserScan / PointCloud2；射线看到环境碰撞、不错误过滤全部障碍或打到自己 |
| RGB-D | Image + CameraInfo；深度 encoding/单位、光学 frame、内参、时间同步明确 |

原始 Go2 MJCF 已有 joint、IMU 传感器，但没有可直接宣称为 MID-360 / D435i 的完整仿真输出。
雷达和相机需要实际生成数据及 TF；不要用空图像/空点云通过感知初始化。
EDU 载荷及相机安装坐标需要本体实际配置确认，通用 Go2 模型不能替代实测标定。

## 控制器与厂家资源

- 厂家模型：[unitree_mujoco 固定提交](https://github.com/unitreerobotics/unitree_mujoco/tree/1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d)。
- 社区候选：[rl_sar 的 Go2 配置及策略目录](https://github.com/fan-ziqi/rl_sar/tree/376d42c9b128f963ab08579762d5a216a976ce39/policy/go2)。
  仅供控制器负责人评估，不在本次工具中安装、执行或重新分发；应核对策略来源与许可证、
  与该 MJCF 的匹配关系，并实际验证行走后再引入。
- 不应将 `unitree_rl_gym` 的 G1/H1 部署示例写成 Go2 现成策略。
- 厂家技术支持：[Unitree 联系页面](https://www.unitree.com/contact/)，技术邮箱
  `support@unitree.com`。可索取 Go2 EDU 的 MuJoCo locomotion 工程、支持的依赖版本、
  Twist/SDK 兼容接口、策略文件及许可、载荷与传感器安装参数；不要索要或分享其他人的账号密码。

## 验收与交接清单

| 项目 | 本目录交付状态 | 最终接入需要的证据 |
| --- | --- | --- |
| 固定源模型、mesh、许可证、SHA256 | 已提供获取与静态检查工具 | 18 个上游文件校验成功 |
| 关节/执行器/根节点映射 | 已提供静态报告和 Native 检查工具 | 编译后按名匹配 qpos/qvel/transmission |
| Native 物理加载 | 提供命令，未作为行走验收 | MuJoCo 版本、实际加载日志、无数值异常 |
| 稳定站立、前后/侧移/转向 | 未实现于本目录 | 控制器/策略版本、轨迹及方向/单位测试 |
| stop、超时、reset、断连 | 待完整 runtime 集成 | 清理命令及策略状态，停止后无残留目标 |
| ROS/TF/仿真时间 | 接口约定已交付，未实现 Bridge | 同时钟、有真数据、单一 TF 发布者 |
| 雷达、RGB-D 和 EDU 载荷 | 需要仿真/本体联合核对 | 测距、遮挡、碰撞、相机内外参、质量配置 |
| `rbnx boot`、移动和感知任务 | 待仿真 runtime | capability 发现、初始化、执行及停止日志 |
| Mapping / Nav2 / Scene | 有数据和底盘后再验收 | 仿真环境内实际任务结果，不照搬真机通过记录 |
| catalog 登记 | 不注册未完成部署 | 默认分支完整 manifest，随后提交 catalog.yaml |

本目录不会替仿真负责人声明他尚未公开的代码已完成，也不会覆盖其已有实现。
