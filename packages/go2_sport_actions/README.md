# 宇树 Go2 动作技能包 · Robonix

**一个 package，7 个独立注册的 Skill，复用 8 项已实机验收的底层动作。**
包名保持 `robonix.skill.unitree.go2.sport_actions`，版本 **0.2.0**。
网站保留同一个包入口，不拆成七个代码仓库。

## 七个技能

| Skill / 初始化 action | 对应宇树 Go2 动作 | 示例说法 |
| --- | --- | --- |
| `unitree_go2_hello` / `hello` | Hello，打招呼/挥手 | 打个招呼 |
| `unitree_go2_stretch` / `stretch` | Stretch，伸展身体 | 伸展一下 |
| `unitree_go2_new_year_greeting` / `new_year_greeting` | Scrape，遥控器上的拜年 | 拜个年 |
| `unitree_go2_crouch` / `crouch` | StandDown → StandUp，卧倒再起身 | 卧倒再起身 |
| `unitree_go2_dance` / `dance` | 从 Dance1、Dance2 随机选一支 | 跳支舞 / 跳个舞吧 |
| `unitree_go2_handstand` / `handstand` | 原地倒立并恢复四足 | 做个倒立 |
| `unitree_go2_handstand_walk` / `handstand_walk` | 倒立前进并恢复四足 | 倒立向前走 |

每项有自己的 Skill ID、初始化参数、动作说明、execute/status/cancel 合约。
用户无需说第一支/第二支舞，大模型也不需要填写动作名、SDK 名或舞蹈编号。
自然语言由所接入的 Pilot 模型理解，表中口令只是例子，不是生产入口的关键词白名单。

舞蹈候选由共享服务的 `dance_variants: [dance_1, dance_2]` 配置，
执行时使用系统随机源等概率选择；不是固定第一支、固定轮换或大模型猜编号。
独立请求可能连续抽到同一支，这是正常随机结果。
同一非空 request_id 的重试复用原 run，不重复抽取/执行。
返回和状态记录中可以查看 selected_action / name。

## 架构与验收边界

Client 语音 → Speech → Pilot 选择具名 Skill → Executor →
对应 Skill → **共享动作服务** → Go2 SDK daemon → Unitree SportClient → 固件动作控制器。

7 个 Skill 是同一包的 7 个配置实例，不是 7 个底盘控制者。
额外的 `unitree_go2_sport_runtime` 是后台 Service，不是第 8 个动作 Skill，
不对大模型开放通用动作选择入口；共享原来的 ActionManager、停止、取消和 watchdog。
不依赖地图、相机或 RobotTrack，不接管原导航/跟随部署。

底层 8 项动作已在本机 Go2 EDU 通过单项及完整语音流程验收，见
[原始验收记录](docs/ACCEPTANCE.md)。本次仅拆分注册和合并舞蹈入口：
Dance1/2 监督窗口仍为 45/55 秒，倒立保持 8+4 秒，
倒立前进仍为 0.06 m/s × 5 秒，结束后恢复四足。
SDK daemon、动作 ID 和这些运动计划没有变更。
**0.2.0 注册/随机选择链路的验证为离线验证，不宣称重新完成真机验收。**
其他 Go2 或固件仍需验证兼容性；不包含鞠躬、前跳、后空翻。

## 下载与构建

需要已配置的 Robonix、Python 3.10+ API/MCP/codegen 环境、CMake/C++ 与兼容 Go2。
脚本不自动安装依赖。

```bash
git clone --recurse-submodules https://github.com/syswonder/robot-unitree-go2.git
git clone https://github.com/syswonder/skill-unitree-go2-sport-actions-rbnx.git
export GO2_ROBOT_DIR="$(realpath robot-unitree-go2)"
export GO2_SKILL_DIR="$(realpath skill-unitree-go2-sport-actions-rbnx)"
bash "$GO2_SKILL_DIR/scripts/build_daemon.sh"
rbnx validate "$GO2_SKILL_DIR"
rbnx build "$GO2_SKILL_DIR"
```

也可直接用 robot 仓库内的 `packages/go2_sport_actions`。
独立仓库是这个目录的发布快照；请向 robot 仓库提交源码改动，再同步发布。
指定现有 Python 环境可用 `RBNX_CODEGEN_PYTHON` 和 `RBNX_RUNTIME_PYTHON`。
`UNITREE_SDK2_DIR` 可指定已有的固定版本 SDK2。

## 一包七 Skill 的初始化与部署

完整配置见 [seven-skills.yaml](examples/seven-skills.yaml)，包含
1 个共享服务和 7 个独立 `skill:` 条目，每项都引用相同 `GO2_SKILL_DIR`，
但选择自己的 manifest 和 action 初始化参数。例如：

```yaml
skill:
  - name: unitree_go2_dance
    path: ${GO2_SKILL_DIR}
    manifest: package_manifest.dance.yaml
    config:
      action: dance
      runtime_provider_id: unitree_go2_sport_runtime
```

不要只复制这一项而遗漏共享服务。完整配置已列齐七项，无需复制源码。
`go2_sport_actions/skills.json` 汇总七个身份、中文说明、SDK 对应关系及初始化参数。
manifest 变体只限定各实例的公开接口，实际启动和运行逻辑复用同一份实现。

将配置的 service/skill 条目加入已有 Client 部署，并保留该本体的 Soma 配置。
`examples/offline-soma.yaml` 和 `offline.urdf` 仅为独立注册测试夹具，不是 Go2 物理模型。
完整 Go2 Client 配置在 robot 仓库的
[deploy/sport-actions](https://github.com/syswonder/robot-unitree-go2/tree/main/deploy/sport-actions)。

设置 `GO2_SDK_SOCKET`（父目录存在的私有绝对 socket 路径）、
`GO2_NETWORK_INTERFACE`、`GO2_ROBOT_DIR`，以及已有 Speech/音频桥/Pilot 环境。
现场准备好并批准动作测试后，沿用现有运行时确认：

```bash
export GO2_OPERATOR_PRESENT=1
export GO2_STAGED_NAV2_RUNTIME_ACK=I_APPROVE_GO2_STAGED_NAV2_MOTION
rbnx boot -f /absolute/path/to/your/robonix_manifest.yaml
rbnx tools
```

默认不启用运动。后台服务只在明确执行请求时启动 SDK daemon。
同一只狗的导航/跟随与动作部署不能同时占用底盘。

## 自然语言与直接调用

默认接入已有通用 Pilot 模型，让它读取七个具名技能的独立说明并选取对应技能；
**不要求启用包内 intent_router**。没有为七项动作再写七份自然语言规则。
`intent_router` 仅保留为原有离线固定口令演示的可选兼容工具，输出已改为七个具名入口；
它不是大模型泛化能力的验证，也不会决定舞蹈编号。

以跳舞为例：
- Provider：`unitree_go2_dance`
- Contract：`robonix/skill/unitree_go2_dance/execute`
- 参数：`{"request_id":"my-demo-001"}`
- 返回：`accepted`、`run_id`、`message`（内含 selected_action）
- 用同一命名空间的 `status` 查询 `{"run_id":"..."}`。
- `cancel` 请求取消；空 run_id 表示取消共享执行器当前动作。

其他六项输入相同，只需选择对应 Skill。request_id 可留空由 Skill 生成，
需要显式重试时应复用返回的 request_id，不要无意生成一个新动作。
不同动作不可复用相同 request_id；去重记录仅在当前运行服务进程中保留。
网络错误会返回 dispatch_unconfirmed，不会自动重发或声称机器狗没有动作。

完整展示见 [demo_sequence.json](examples/demo_sequence.json)。
按任意顺序逐项说话，等前项自然结束且站稳后再下发下一项。
不同技能共享单一执行器，运行中不会互相抢占；不新增动作间固定等待时间。
SDK_ACCEPTED_COMPLETION_UNVERIFIED 只是程序流程结束，不是物理姿态证明。
FAILED/CANCELLED 时停止后续序列；实际停止仍需现场确认。

## 离线验证

```bash
cd "$GO2_SKILL_DIR"
PYTHONPATH=. python3 -m unittest discover -s tests -q
rbnx validate .
# 该配置 backend=catalog，不连接机器人、不启动 SDK daemon：
rbnx boot -f examples/offline-seven-skills.yaml
```

启动后 Atlas 应注册 7 个 KIND_SKILL 和 1 个共享动作 Service。
`scripts/check_registration.py` 只查询注册信息，不发动作；
生成绑定后可运行 `scripts/check_split_provider_offline.py` 检查七个初始化/调用适配。
原 `scripts/check_provider_offline.py` 检查兼容接口。
全仓 `scripts/validate_offline.sh` 继续验证旧导航/跟随与底盘协议。

## 兼容与许可

0.1.0 通用接口保留给原部署的程序兼容，默认七技能部署不会注册它，
其 user_invocable 标记已关闭，不向模型重复暴露两个舞蹈编号。
历史验收文档保留原样，不把八项旧验收改写为七项新链路实测。

Apache-2.0；Unitree SDK2 与固件由上游提供，遵守各自许可。
