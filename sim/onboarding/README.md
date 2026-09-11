# Go2 MuJoCo 本体接入交付 / Body onboarding kit

本目录提交 **Go2 本体负责人可独立交付的模型获取、资产清单、静态检查和接口约定**。
不是完整仿真部署包，不改变仓库根目录的真机部署，也不宣称已完成仿真行走、导航、
Web/WASM、Webots 或 `rbnx boot` 仿真验收。

按 [Robonix MuJoCo 接入指南](https://book.robonix.ai/integration-guide/mujoco-simulation-onboarding)
优先采用厂商原生 MJCF 的 Native 路线。仿真负责人可以在已有工程中复用这些文件；
无需重新导入整套真机运行环境，更不需要复制真机地图、账号、标定或运动许可文件。

## 1. 获取和检查（无需机器狗，无需安装 Python 依赖）

在本仓库根目录运行，Python 3.10+：

```bash
python3 sim/onboarding/prepare.py
python3 sim/onboarding/inspect_model.py
python3 -m unittest discover -s sim/onboarding/tests -v
```

首次准备只下载固定提交的 18 个模型/许可证文件到 `.runtime/mujoco-onboarding/unitree_go2/`。
每个文件在落盘前检查 SHA256，后续运行复用相同字节；已有文件被修改时不会覆盖。
不执行下载源码、不启动 SDK、DDS、ROS 或任何机器人进程。不下载策略、不安装依赖。

如已收到此前的本体资源包，可以完全离线准备：

```bash
python3 sim/onboarding/prepare.py \
  --source-root /path/to/go2-simulation-2026-09-09/vendor/unitree_mujoco
```

产物：

```text
unitree_go2/
├── index.json                # 18 个模型运行资产：原 XML + 16 个 mesh + 平地场景
├── upstream.lock.json        # 提交与 SHA256；包含许可证校验
└── vendor/
    ├── LICENSE
    ├── go2.xml               # 官方原文件，不改质量、惯量、执行器
    ├── assets/*.obj
    └── scene.xml             # 本项目生成的最小平地测试场景
```

`scene.xml` 与原始 `go2.xml` 位于同一层级，保持原 `meshdir="assets"` 路径语义。
`index.json` 中的路径均相对于 `unitree_go2/`，无绝对路径或 `..`。
许可证及 provenance 随包交付，但不作为 MuJoCo 运行资产加载。
未复制官方地形场景，避免把地形路径问题混入本体接入。

## 2. Native 编译检查（有 MuJoCo 的环境才执行）

```bash
python3 sim/onboarding/check_native.py
```

仅在本进程中加载模型、按名称核对 qpos/qvel/actuator 地址，并执行 20 次零力矩被动步进。
不创建 ROS/DDS 网络、不调用真机 API、不打开 viewer、不要求 GPU。
该检查通过仅代表编译、传动映射和短时数值检查通过，**不代表站立/行走通过**。
上游 C++ README 使用 MuJoCo 3.3.6；这里将其作为首次复现候选版本，而非本机已验证版本。
缺少依赖时脚本报明确信息退出，不自动安装或修改系统 Python。

项目操作者批准安装后，可以在独立 venv 中使用：

```bash
python3 -m venv .runtime/go2-mujoco-venv
.runtime/go2-mujoco-venv/bin/python -m pip install 'mujoco==3.3.6'
.runtime/go2-mujoco-venv/bin/python sim/onboarding/check_native.py
```

此依赖用于模型检查，不是完整 Robonix 仿真依赖集合。

## 3. 本体和控制接口

详见 [INTEGRATION.md](INTEGRATION.md)。最重要的差异：

- 官方 `unitree_mujoco` 是关节级仿真；发布运动状态不等于实现了 `SportClient.Move`。
- 12 路 motor 输入是力矩，不能把 Twist 或目标关节角直接写入 `data.ctrl`。
- SDK/执行器顺序为 FR、FL、RR、RL；模型关节遍历顺序为 FL、FR、RL、RR，均为 hip/thigh/calf。
- 策略观测/动作顺序仍由策略自己的配置决定，不能由 SDK 顺序推断。
- `check_native.py` 按名称解析地址，避免把 freejoint 的 7 个位置量当成电机关节。

## 4. SDK、URDF 和可交付源码

| 资源 | 固定来源 / 本仓库入口 | 用途 |
| --- | --- | --- |
| Go2 MJCF/mesh | [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco/tree/1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d/unitree_robots/go2) | 本工具直接获取并校验 |
| SDK2 C++ | [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2/tree/21d0a3b2c46ee48c8fdf2783becb6be3beb0a59b) | 真机协议参考；不是 MuJoCo 步态控制器 |
| SDK2 Python | [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python/tree/65691c8a8bc53b98d3976dba4dbf9d5d20b2e7f5) | 官方 Python 仿真通信参考；无需为静态检查安装 |
| ROS2 消息 | [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2/tree/668d1ec5a05d1c38d3306bdca7d59f2ba3581a88) | 与官方消息定义对接 |
| Go2 URDF / DAE | [go2_description](../../packages/go2_description/) | 本仓库原有模型与 TF，不在本次重复复制 |
| 真机底盘能力 | [go2_chassis](../../packages/go2_chassis/CAPABILITY.md) | 对照上层 capability，不能用真机 daemon 控制仿真 |
| 感知能力 | [go2_sensors](../../packages/go2_sensors/CAPABILITY.md) | 对照 scan/cloud/IMU 语义 |
| RGB-D 能力 | [go2_d435i](../../packages/go2_d435i/CAPABILITY.md) | 对照相机消息与 frame，不代表 MJCF 已有 D435i |

模型及许可证来源由 `upstream.lock.json` 锁定。原始 XML/mesh 保持 BSD-3-Clause；本目录原创
工具及文档随本仓库 Apache-2.0 发布。生成包应连同 `vendor/LICENSE`、锁文件及本目录说明一起交付。

## 5. 提交及验收边界

本次提交放在真实本体仓库的隔离目录，便于本体负责人维护及仿真负责人取用。
不修改根 `robonix_manifest.yaml`，不向 catalog 注册尚未实现的 `go2_mujoco` 部署，
不把模型资料 PR 描述成可执行仿真 Primitive。

最终仿真集成 PR 应在承载 **实际 runtime** 的仓库提交，包含模型注册、行走控制器、
Bridge、传感器、capability/manifest 和验证结果。若师兄已有实现，直接引用本 PR 并
补齐 [验收清单](INTEGRATION.md#验收与交接清单)，无需重复造一套 runtime。
若最终采用独立 Go2 仿真部署仓库，应在默认分支存在可运行根 manifest 后，按
[catalog 指南](https://book.robonix.ai/integration-guide/package-catalog)
另向 `syswonder/robonix-package-catalog` 提交仅含 `catalog.yaml` 的注册 PR。
