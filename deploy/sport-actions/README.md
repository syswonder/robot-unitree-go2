# Go2 特技 Skill 的独立 Client 配置

此配置让 Go2 动作作为 Robonix Skill 出现在工具目录中，供别人的 Client、
Pilot 或直接能力调用复用；不是把动作写死在一段录屏脚本里。

入口是 `bash deploy/sport-actions/start.sh`。它使用已经配置好的 Robonix、
Speech、音频桥和 FunASR，不安装依赖；默认 `GO2_OPERATOR_PRESENT=0`。
按以下步骤设置环境变量并启动本地 Pilot 路由器，再运行该入口。
官方 Robonix Client 可连接本部署的 Atlas/音频桥；常用网页端口为 7860。
本次实测使用与现有 Speech 兼容的 v0.1.0 (178fd2a+) 构建；
完整复用教程与 8 项语音口令见 [Skill README](../../packages/go2_sport_actions/README.md)。

1. 在 `robot-unitree-go2` 仓库根目录运行
   `bash packages/go2_sport_actions/scripts/build_daemon.sh` 与
   `bash packages/go2_sport_actions/build.sh`。先用 `rbnx validate packages/go2_sport_actions`
   和该包单元测试验证；构建只使用已经安装的依赖。
2. 停止原导航/跟随的 motion-enabled 实例。设置
   `ROBONIX_DEPLOY_DIR` 为本仓库绝对路径、`ROBONIX_SOURCE_PATH` 为
   Robonix 源码目录、`ROBONIX_AUDIO_BRIDGE_PATH` 为已有的 Client 音频桥包目录、
   `ROBONIX_SPEECH_PATH` 为已经构建的语音服务包目录、
   `GO2_SDK_SOCKET` 为这只狗既有的私有 SDK socket 路径、
   `GO2_NETWORK_INTERFACE` 为 Go2 专用网卡；另提供本地 FunASR 模型路径
   `FUNASR_MODEL_DIR` 和 Pilot 所需的 `VLM_BASE_URL`、`VLM_MODEL`、
   `VLM_API_KEY`。不要把凭据提交到仓库。
3. 启动本包的本地 Pilot 路由器：
   `PYTHONPATH=packages/go2_sport_actions python3 -m go2_sport_actions.intent_router`
   （只监听 `127.0.0.1:18081`，不连接机器狗）。设置
   `VLM_BASE_URL=http://127.0.0.1:18081/v1`、`VLM_MODEL=go2-sport-router`；
   它将明确的中文动作口令交给 Skill，Executor 反馈不会重复下发动作。
4. 在现场满足运动条件、明确批准具体动作阶段后，沿用现有
   `GO2_OPERATOR_PRESENT` 和 `GO2_STAGED_NAV2_RUNTIME_ACK` 运行时确认。
   以 `rbnx boot -f deploy/sport-actions/robonix_manifest.yaml` 加载专用配置，
   `rbnx tools` 应列出 `robonix/skill/go2_sport_actions/*`。
5. Robonix Client 连到此配置的 Liaison/音频桥，明确说“跳第一支舞”
   “跳第二支舞”“拜年”等；或者直接调用
   `robonix/skill/go2_sport_actions/execute_utterance`，请求字段为
   `text`、`request_id`。返回 `run_id` 后轮询 `status(run_id)`；“停止动作”
   或 `cancel(run_id)` 请求取消。只报告 SDK 接受与停止确认，不把它写成
   姿态或落地已验收。

此配置没有 Nav2、RobotTrack 或 D435i。若要恢复导航/跟随，先关闭此
配置，再按原流程启动旧配置；两者不能共同争抢运动控制。后空翻仍不能
执行。2026-09-27 已在这只 Go2 EDU 上由 Client 语音分别触发两支舞，
操作者确认完整/自然收尾、站姿正常；Hello、Stretch、拜年、卧倒再起身、
原地倒立与 2 秒、5 秒倒立前进也已分别观察到动作及恢复，5 秒版明显更远。
2026-09-28 操作者确认 8 项完整语音流程符合预期并已关机。
发布版只提供这 8 项动作；日志记录了 StopMove 响应，但未测定
StopMove 在各种固件动画中途的中断延迟。
