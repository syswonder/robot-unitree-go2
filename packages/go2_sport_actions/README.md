# Go2 动作 Skill · Robonix

通过 Robonix Client 中文语音或标准能力调用控制 Go2 的 8 项动作。
技能不依赖地图、摄像头、RobotTrack 模型或特定演示场景，可加入其他任务流程；
执行端需要支持对应动作的 Go2 固件与本项目的 SDK daemon。

包名：`robonix.skill.unitree.go2.sport_actions`，版本：`0.1.0`。

## 动作与完整语音流程

按顺序一次说一句。等当前 Skill run 结束、机器人自然收尾并站稳，再说下一句。
监督窗口包含固件开始动作的延迟，不等于实测动画时长。

| 顺序 | 语音口令 | execute 的 name | 实现 | 监督窗口 |
| --- | --- | --- | --- | --- |
| 1 | 打个招呼 | hello | Hello | 15 秒 |
| 2 | 伸展一下 | stretch | Stretch | 20 秒 |
| 3 | 拜年 | new_year_greeting | Scrape | 30 秒 |
| 4 | 卧倒再起身 | crouch | StandDown → StandUp | 8 + 8 秒 |
| 5 | 跳第一支舞 | dance_1 | Dance1 | 45 秒 |
| 6 | 跳第二支舞 | dance_2 | Dance2 | 55 秒 |
| 7 | 做个倒立 | handstand | HandStand(true) → HandStand(false) | 8 + 4 秒 |
| 8 | 倒立向前走 | handstand_walk | 倒立 → Move → StopMove → 退出倒立 | 8 + 5 + 4 秒 |

倒立前进的命令速度为 **0.06 m/s**、持续 **5 秒**，实际位移由固件和地面决定。
没有 Bow/前跳/左右摆动/后空翻的执行入口。
2026-09-27 单项测试、2026-09-28 操作者确认完整语音流程通过，
并确认已断开连接、机器人关机。验收范围与日志见 [验收记录](docs/ACCEPTANCE.md)；
其他机器人或固件需要各自验证。

## 控制链路

Client 麦克风 → Speech → Pilot → Executor → 此 Skill → 私有 Unix socket
→ Go2 chassis SDK daemon → Unitree SportClient → 机器人固件动作控制器。

Skill 负责动作名称、顺序、监督时长、运行状态和取消。
固件负责舞蹈、倒立等姿态和关节运动；本包不训练模型、不发送低层电机力矩。
现有单一控制者、状态新鲜度、300 ms watchdog 和 StopMove 路径保持有效。
普通模式 100 已验证可以直接启动两支舞，无需为舞蹈先手动切到 2010。

## 获取与构建

需要 Linux、Python 3.10+、CMake/C++ 编译器、已配置的 Robonix Python API/
MCP/codegen 环境，以及 Go2 同网段的网卡。构建脚本不自动安装依赖。

```bash
git clone --recurse-submodules https://github.com/syswonder/robot-unitree-go2.git
git clone https://github.com/syswonder/skill-unitree-go2-sport-actions-rbnx.git
export GO2_ROBOT_DIR="$(realpath robot-unitree-go2)"
export GO2_SKILL_DIR="$(realpath skill-unitree-go2-sport-actions-rbnx)"
bash "$GO2_SKILL_DIR/scripts/build_daemon.sh"
rbnx validate "$GO2_SKILL_DIR"
rbnx build "$GO2_SKILL_DIR"
```

必须使用含动作协议的 `robot-unitree-go2` 版本；旧 daemon 不认识 SportAction。
SDK2 使用该仓库固定的 gitlink，勿在复现时随意切换版本。
也可直接使用 robot 仓库内的 `packages/go2_sport_actions`，无需重复下载技能。
独立技能库是该目录的发布快照；修改源码请向 robot 仓库对应目录提 PR。

若 Python 环境需要指定，使用 `RBNX_CODEGEN_PYTHON`（构建）和
`RBNX_RUNTIME_PYTHON`（运行）。`UNITREE_SDK2_DIR` 可指定已有 SDK2 checkout。

## 接入自己的 Robonix 部署

将以下条目加入部署 manifest 的 `skill:`，设置路径和网卡环境变量：

```yaml
skill:
  - name: go2_sport_actions
    path: ${GO2_SKILL_DIR}
    config:
      backend: managed
      daemon_socket: ${GO2_SDK_SOCKET}
      daemon_binary: ${GO2_ROBOT_DIR}/packages/go2_chassis/rbnx-build/sdk/install/bin/go2_sport_daemon
      network_interface: ${GO2_NETWORK_INTERFACE}
```

`GO2_SDK_SOCKET` 使用可写的绝对路径，父目录需已创建。
默认仅查询目录/预览不会连接或启动机器人动作。只有明确执行请求才启动托管 daemon。
在本体现场准备好、具体测试获准后，沿用 Go2 部署现有运行时配置：

```bash
export GO2_OPERATOR_PRESENT=1
export GO2_STAGED_NAV2_RUNTIME_ACK=I_APPROVE_GO2_STAGED_NAV2_MOTION
rbnx boot -f /absolute/path/to/robonix_manifest.yaml
```

同一机器人只运行一个底盘控制者。切换到导航/跟随前，先结束动作 Skill 部署。
紧急情况由现场遥控器接管；StopMove 收到回执不等于能立即中断每种固件动画。

Client 完整示例位于 robot 仓库的
[`deploy/sport-actions`](https://github.com/syswonder/robot-unitree-go2/tree/main/deploy/sport-actions)。
该示例只加载动作、Speech、音频桥和 Robonix 系统组件。
设置 Speech 模型路径和音频桥路径后即可接入自己的 Client。
可用包内本地确定性 Pilot 路由器：

```bash
cd "$GO2_SKILL_DIR"
PYTHONPATH=. python3 -m go2_sport_actions.intent_router
```

将 Pilot 的 OpenAI 兼容上游设为 `http://127.0.0.1:18081/v1`，
模型名 `go2-sport-router`。它只处理本包口令；若复用已有通用 Pilot，
保留其上游并让它发现本 Skill 的能力即可。运行 `rbnx tools` 检查发现结果。
路由器只创建 RTDL 调用，收到 Executor 反馈后不会再次发送同一动作。

## 编程接口与串行编排

| 能力（前缀 robonix/skill/go2_sport_actions/） | 参数 / 返回 |
| --- | --- |
| list | 无参数；动作目录及已验证范围 |
| preview | name；无运动预览、后端是否配置 |
| execute | name、request_id；accepted、run_id、message |
| execute_utterance | text、request_id；语音文本映射到相同执行路径 |
| status | run_id；result_json 内含状态、已下发步骤、stop_acknowledged |
| cancel | run_id；请求取消，随后查询 status |

示例参数：

```json
{"name":"dance_1","request_id":"demo-001"}
```

或 `execute_utterance`：

```json
{"text":"跳第一支舞","request_id":"demo-001"}
```

相同非空 `request_id` 在当前进程生命周期内返回同一 run，不重复启动。
重启后此去重记录不保留。新一次有意执行应使用新的 ID。
动作进行中会返回 `action_in_progress`，不会插入或抢占动作。

完整编排可读取 [demo_sequence.json](examples/demo_sequence.json)：
逐项 execute → accepted 后用 run_id 查询 status → 等到终态再进入下一项。
不要在仅收到 accepted 时继续下一项。
`SDK_ACCEPTED_COMPLETION_UNVERIFIED` 是程序流程结束状态；
`FAILED` / `CANCELLED` 时终止后续序列。
即使 `stop_acknowledged=true`，实际姿态仍由操作者/上层感知确认。
单次 run 不会因历史验收记录而宣称实时物理完成。
语音“停止动作”或“停下”走同一个 cancel 路径。

## 离线验证

```bash
cd "$GO2_SKILL_DIR"
PYTHONPATH=. python3 -m unittest discover -s tests -q
rbnx validate .
bash -n build.sh start.sh scripts/build_daemon.sh
```

Provider 与无运动 daemon IPC 检查见 `scripts/`；
这些脚本不初始化运动 SDK，不向真机发动作。
主仓库 `scripts/validate_offline.sh` 还验证协议、watchdog 和导航/跟随回归。

## 许可与维护

Apache-2.0，见 [LICENSE](LICENSE)。
本包为 Go2 适配和编排代码；Unitree SDK2 与机器人固件由上游提供，
不在独立技能包中重新分发。维护者：张修齐 / Origamii520。
