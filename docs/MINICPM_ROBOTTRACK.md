# MiniCPM-RobotTrack 跟随集成

更新时间：2026-08-06（Asia/Shanghai）

## 当前结论

本仓库已经完成 MiniCPM-RobotTrack 的工作站侧静态集成、策略 checkpoint
离线验证、D435i 全图像 dry-run、完整正式 45 秒 Go2 实机跟随和后续 75 秒复测。
当前路径是：D435i
只提供 RGB，RTX 4070 工作站运行官方推理服务，Robonix provider 把官方速度响应
接入已经实机验证的 Go2 单控制器速度链。
上传前严格采用官方 `center_crop_height`：按比例缩到 384 像素高，再中央裁成
`384 x 384`，不会把 D435i 的 4:3 人像直接横向拉伸。

最新正式实机结果需要精确表述：RobotTrack 前进上限为 `0.50 m/s`、转向上限为
`0.30 rad/s` 时，Go2 已完成持续前进和灵敏左右转向跟随。2026-08-07 的复测正式
窗口为 `75.0298 s`，累计路径 `12.3602 m`；窗口内 mode 始终为 `0`、marker 始终
为 `2010`，D435i 375 帧全部实时变化，并在结束后 `0.612 s` 明确进入
`DISARMED`。这项结果继续确认了早期“只确认转向、前进不明显”的阶段性
结论，但早期录包和现场观察仍作为历史证据保留。

受 DINOv3 License 约束的 ViT-S/16 snapshot 已从 ModelScope 同名镜像补齐。权重
SHA-256、大小以及两个配置文件均与项目固定的 Hugging Face 参考内容一致；许可证
也保存在模型目录中。资产校验器通过逐文件 SHA-256 验证镜像，不伪造 Hugging Face
revision marker，也不会读取或保存 Hugging Face token。

最新正式报告和 rosbag：

- `docs/reports/2026-08-07/robottrack-speed50-yaw30-classic2010-full75-result.md`；
- `docs/reports/2026-08-07/robottrack-speed50-yaw30-classic2010-full75-20260807T103423CST-rosbag/`。

## 与原有导航的关系

原 generation 1 导航完整保留，没有重建地图或重做任何实机阶段：

- map ID：`go2_second_location_refined_20260724_01`，generation `1`；
- 原始初始位姿 sidecar、语义地标、RViz、Mapping/Navigation dirty submodule、
  日志、截图、视频、PR worktree 和组会材料均未改写；
- 普通 persistent voice/Nav2 manifest 的默认行为不变；
- RobotTrack 只通过显式 `GO2_ROBOTTRACK_MODE=true` 选择新的 manifest；
- `100`、`2010` 不进入模型，也不被 RobotTrack 当作错误；ClassicWalk 仍由已经
  跑通的 chassis owner 保持，RobotTrack 不调用步态 API；
- Go2 原生随身定位器/遥控器跟随与视觉模型是两条独立方案，本实现不联动它。

跟随 profile 的速度所有权是：

```text
Nav2 controller  -> /go2/robottrack/nav_cmd_vel_raw --+
                                                        +-> source mux
RobotTrack        -> /go2/robottrack/cmd_vel_raw -------+      |
                                                               v
                                                        /cmd_vel_nav
                                                               |
                                               existing velocity_smoother
                                                               |
                                               /cmd_vel_guard_input
                                                               |
                                                 existing velocity_guard
                                                               |
                                             /go2/staged_nav2/cmd_vel
                                                               |
                                  existing chassis + SDK daemon + watchdog
```

RobotTrack profile 固定选择 `robottrack`；mux 是 `/cmd_vel_nav` 的唯一新增发布者。
RobotTrack live manifest 当前固定 `max_vx=0.50`、`max_wz=0.30`。普通导航/底盘的
既有通用转向 envelope 仍可为 `0.40`，但 RobotTrack 在进入公共底盘链之前已夹到
`0.30`，两者不矛盾。
关闭、HTTP 取消、1.5 秒旧计划、选中源超过 0.25 秒未刷新都会产生零速，之后仍经过
原有平滑、guard、底盘 command watchdog 和遥控器人工接管链。没有增加另一个
RobotTrack armed/disarmed 状态、一次性审批文件或仓库硬门。

需要准确区分：RobotTrack 速度不经过 Nav2 controller/costmap，因此已有 MID-360
动态障碍层不会二次改写 RobotTrack 的 Twist。本路线忠实采用官方视觉跟随控制方式，
但“复杂动态环境持续跟随”和目标丢失后的行为仍必须用 D435i dry-run 与现场记录验收。

## 固定的官方源码与资产

- OpenBMB 源码：`upstream/MiniCPM-Robot`，commit
  `f7dc15a016b1b2a7e48a559072521e347658de11`；
- RobotTrack checkpoint revision：
  `80d32d0b091c3b340a1e4a6c78d441e8af19590e`；
- checkpoint 权重 SHA-256：
  `27cc7e58fd0797ea2dfa03b250847a7142eeedc7babbadd7bc63401d051ec7f3`；
- SigLIP revision：`9fdffc58afc957d1a03a25b10dba0329ab15c2a3`；
- DINOv3 目标 revision：`114c1379950215c8b35dfcd4e90a5c251dde0d32`；
- DINOv3 ModelScope 镜像 revision：
  `2e601320d0545509ab03374e2f8707f303e1de7a`；
- DINOv3 权重 SHA-256：
  `4610ad75edef83e75afdebf162d148dc628045ea6cbb83d67d4708c709c4f91d`；
- 已缓存但未安装的 Jetson ARM PyTorch wheel：
  `upstream/MiniCPM-Robot/MiniCPM-RobotTrack/vendor/torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl`，
  size `806950107`，SHA-256
  `6f75fd2d2ef840ede1a90dbcf40a5458214bee26cc803fa510cda2e8978d972a`。

固定值和文件 hash 在 `config/robottrack_assets.yaml`。离线检查：

```bash
cd /path/to/robonix-go2/packages/robot-unitree-go2
python3 scripts/verify_robottrack_assets.py
```

当前期望结果是 source、checkpoint、SigLIP、DINOv3 全部为 `READY`。DINOv3
使用显式 `sha256_files` 校验模式，输出中会保留 ModelScope repo 和 revision，实际
Hugging Face revision 保持 `unknown`，不会把镜像 revision 冒充为官方缓存标记。

下载和逐文件校验记录：
`docs/reports/2026-08-06/robottrack-dinov3-modelscope-asset.json`。

官方资料：

- <https://github.com/OpenBMB/MiniCPM-Robot>
- <https://huggingface.co/openbmb/MiniCPM-RobotTrack>
- <https://github.com/OpenBMB/MiniCPM-Robot/blob/main/MiniCPM-RobotTrack/docs/GO2_DEPLOYMENT_zh-CN.md>
- <https://modelscope.cn/models/facebook/dinov3-vits16-pretrain-lvd1689m>

## 已完成的离线验证

依赖被隔离在工作区内的 `.tools/robottrack-python`，没有改系统 Python。

策略 checkpoint smoke：

```bash
REPO_ROOT="$(git rev-parse --show-toplevel)"
WORKSPACE_ROOT="$(cd "$REPO_ROOT/../.." && pwd)"
cd "$REPO_ROOT"
PYTHONPATH="$WORKSPACE_ROOT/.tools/robottrack-python" \
  "$WORKSPACE_ROOT/upstream/robonix-go2-build/services/speech/rbnx-build/venv/bin/python" \
  scripts/robottrack_checkpoint_smoke.py --device cuda:0 --warmup 1 --iterations 5
```

2026-08-05 保存结果：输出 shape `1 x 8 x 3`，24/24 finite，revision 与权重
SHA-256 一致；RTX 4070 五次暖推理 median `13.00 ms`，峰值 allocated VRAM
`1353513472` bytes。完整 JSON：
`docs/reports/2026-08-05/robottrack-checkpoint-smoke-cuda.json`。

离线测试：

```bash
bash packages/go2_robottrack/tests/run_offline_tests.sh
python3 -m unittest -v \
  tests.test_verify_robottrack_assets \
  tests.test_robottrack_offline_tools \
  tests.test_robottrack_manifest \
  tests.test_robottrack_launcher
```

新 provider 已正式完成 Robonix codegen、`lifecycle` ROS 2 overlay 和
`go2_robottrack` colcon build；`start.sh` 所需的 IDL、protobuf stub 和安装 overlay
均已生成。没有执行 `start.sh`。

补齐 DINOv3 后，官方 HTTP 服务已在本机 `127.0.0.1:5801` 成功加载，并用官方仓库
自带的跟随样例图完成了全图像链测试。首次请求曾明确暴露缺少 `torchvision`；历史
记录保存在 `docs/reports/2026-08-06/robottrack-full-image-smoke-preinstall.json`。
经单独批准后，匹配当前工作站 torch `2.11.0+cu130` 的
`torchvision-0.26.0+cu130` 已安装到工作区隔离依赖目录，其 CUDA NMS 编译算子
验证通过。

随后官方四帧跟随序列均返回 HTTP 200、有限的 `8 x 3` waypoints 和有界
`base_velocity`。5 次暖态测量的 `e2e_ms` mean `60.84`、p50 `60.76`、p95
`63.95`，官方 benchmark 报告 mean `16.437 FPS`。测试结束后服务已停止，完整结果：
`docs/reports/2026-08-06/robottrack-full-image-smoke-cuda.json`。这只证明工作站静态/
序列图像推理链，不是 D435i 或实机跟随结果。

## 已完成的 D435i 与实机验证

2026-08-06 的 D435i dry-run 已验证：`384 x 384` 模型输入持续更新，人物横移时
速度/waypoint 响应灵敏。后续 preview upload 已补齐监控页左侧 `Raw camera`；最新
实测中 raw、模型输入和 overlay 三路均实时变化。

最新 75 秒复测正式窗口为 `10:34:39.374584885` 至 `10:35:54.404337106`，长度
`75.029752221 s`。RobotTrack raw 与 `/cmd_vel_nav` 各 3,751 条、约 50 Hz，
`99.813%` 非零；staged 1,501 条、20 Hz，`99.800%` 非零。三路命令均严格位于
`vx=[0,0.50] m/s`、`vy=0`、`wz=[-0.30,+0.30] rad/s`。正式窗口内 D435i
375 帧约 5 Hz 且 375 个内容 hash 全部不同，scan 约 7.62 Hz，marker 全程
`2010`，mode 全程 `0`。

窗口内 odom 的 2D 净位移为 `2.7471 m`、累计路径为 `12.3602 m`，航向约变化
`179.4 deg`，符合包含大量转向的跟随路线而非直线距离测试。正式结束后
`0.612235 s` 首次明确进入 `DISARMED`，bag 结束时仍为 `DISARMED`。完整数值、
bag hash、启动证据和 caveat 以最新正式报告为准。

## 完整图像推理服务

DINOv3、SigLIP 和 RobotTrack checkpoint 已完整放在工作区。Hugging Face 访问申请
被拒后没有借用他人账号、Token 或创建小号；按照面壁工程师建议，改用 ModelScope
同名镜像并完成逐文件内容校验。下载不改变 DINOv3 License 的适用范围。

工作站完整图像服务所需的匹配版 `torchvision` 已安装到隔离依赖目录，没有改系统
Python。这个 x86 CUDA wheel 只用于 RTX 工作站，不能用于后续 Jetson 机载环境。

资产完整后，从工作站本地启动官方服务：

```bash
cd /path/to/robonix-go2/packages/robot-unitree-go2
bash scripts/start_robottrack_inference_server.sh
```

默认只监听 `127.0.0.1:5801`，使用 DINO/SigLIP torch backend 和官方
`/eval_dual`。服务启动器只负责模型，不会启动 ROS、D435i 或 Go2。

## 后续通电时的使用顺序

1. 按原 handoff 恢复已经跑通的 D435i RGB ROS topic
   `/go2/d435i/color/image_raw`；depth 和 IMU 不输入模型。
2. 先运行官方推理服务，确认 D435i raw、模型输入和 overlay 更新。已通过的
   full-image、转向和完整 45 秒前进跟随不需要机械重复；只核对本次启动状态。
3. 使用原 generation 1 persistent 全栈的同一组当前 map/interface/runtime 参数，
   运行 `scripts/start_workstation_robottrack_follow.sh`。这个 wrapper 只选择 RobotTrack
   manifest，不创建新审批或新 armed 状态。
4. 继续使用现有遥控器/App 接管、StopMove、取消和底盘 watchdog。前进、转向、
   模型加载、网页画面和速度链均已有正式证据，不必从零重做。只有用户继续要求时，
   再测目标丢失、重入、遮挡、光照和多人动态场景。
5. 完成后保存 Robonix run directory、provider/server stdout、相机片段和底盘状态，
   再决定是否做专门的目标存在判定或 LiDAR 融合。

官方机载方案验证环境是 JetPack 6.2.2 / L4T R36.5 / Python 3.10；本机记录的
Orin 当前是 JetPack 5.1.1 / L4T R35.3.1。不要把已缓存的 JetPack 6 ARM wheel
直接装入当前 JetPack 5 系统。机载部署需要单独的备份、刷机和回退工作，不属于本轮
关机状态下的工作站实现。

## 主要代码位置

- Robonix/ROS provider：`packages/go2_robottrack/`；
- manifest renderer：`deploy/time-sync/render_workstation_robottrack_manifest.py`；
- 跟随 wrapper：`scripts/start_workstation_robottrack_follow.sh`；
- 官方服务 wrapper：`scripts/start_robottrack_inference_server.sh`；
- checkpoint smoke：`scripts/robottrack_checkpoint_smoke.py`；
- 资产清单与验证：`config/robottrack_assets.yaml`、
  `scripts/verify_robottrack_assets.py`；
- Nav2 raw-output opt-in：
  `third_party/service-navigation-rbnx/nav2_wrapper/guarded_launch.py` 和
  `configuration.py`。
