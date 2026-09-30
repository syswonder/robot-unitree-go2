# Go2 七项动作 Skill 的 Client 配置

一个代码包注册七个独立 Skill，后台共用一个 Service 和 SDK daemon。
本配置已包含七套初始化参数以及 Soma 中对应的能力导出，不加载导航或跟随控制器。
无需相机。普通模式直接舞蹈、倒立前进 5 秒等已验收参数保持原样。

## 准备

1. 按 [技能包 README](../../packages/go2_sport_actions/README.md) 构建 daemon 和技能包。
2. 设置 `ROBONIX_DEPLOY_DIR`、`ROBONIX_SOURCE_PATH`、
   `ROBONIX_AUDIO_BRIDGE_PATH`、`ROBONIX_SPEECH_PATH`、
   `FUNASR_MODEL_DIR` 为已有环境的路径。
3. 设置 `GO2_SDK_SOCKET`、`GO2_NETWORK_INTERFACE`。
   沿用原 Go2 的现场确认变量；默认不启用运动。
4. 将 `VLM_BASE_URL`、`VLM_MODEL`、`VLM_API_KEY` 指向已有的通用 Pilot 模型服务。
   不在文件中保存凭据。**七个技能由模型按独立动作说明选择，不需要关键词路由器。**
5. 运行 `bash deploy/sport-actions/start.sh`，Client 连接当前 Liaison/音频桥。
   `rbnx tools` 应看到 `unitree_go2_hello` 等七个 Skill 的独立能力。

## 语音展示

打个招呼 → 伸展一下 → 拜年 → 卧倒再起身 → 跳个舞吧 → 做个倒立 → 倒立向前走。

顺序可以按需改变；每次等动作自然收尾、站稳后再说下一句。
舞蹈服务从 `dance_variants: [dance_1, dance_2]` 随机选择；
再次说“跳支舞”是一次新的随机选择，可能与上次相同，不要求舞蹈编号。
“停止动作”由 Pilot 调用任一具名 Skill 的 cancel（空 run_id 取消当前动作）。
不新增动作之间的固定等待；沿用已验收的动作监督窗口和单一底盘所有者。

## 离线兼容演示（可选）

旧的 `python3 -m go2_sport_actions.intent_router` 仍可作为固定口令演示工具，
监听 localhost:18081。它已改为输出七个具名 Skill 调用；
它不是通用大模型，不能用于证明开放词汇理解能力，也不选择舞蹈编号。
正式接入使用上一节的模型端点。

## 验证范围

2026-09-28 前的 8 项底层动作已经现场验收。
0.2.0 的七实例注册、MCP 调用、随机选择与去重在离线环境验证；
没有因注册改造而重跑物理动作或变更固件动作参数。
如需恢复导航/跟随，先关闭本专用动作部署，避免共同占用底盘。
