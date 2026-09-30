# 0.2.0：一包七技能验证（2026-09-30）

## 改动

- 同一 package 注册七个 unitree_go2 前缀的独立 Skill。
- 每个实例有专属 manifest、动作说明、初始化 action 和独立 execute/status/cancel。
- 一个内部 Service 复用既有 ActionManager 与 SDK daemon，模型不再填写通用动作名。
- 舞蹈合并为 dance；从配置的 Dance1/Dance2 中随机选择，每次 run 固定选择结果。
- 默认通用 Pilot 按技能描述选择，无需固定关键词路由；旧口令路由仅作可选兼容工具。

## 离线结果

- 36 项包测试通过，覆盖原 8 项运动计划、两种随机舞蹈分支、去重、互斥和取消。
- 七个 provider 的真实生成绑定、初始化配置、委托执行与异常不重发检查通过。
- 使用真实 Atlas/Soma 启动，注册为 7 个 KIND_SKILL 和 1 个 KIND_SERVICE。
- 七条 MCP execute 调用经内部 gRPC 到达 backend=catalog 的共享服务，并返回
  physical_backend_not_configured；没有连接 SDK 或发送真机指令。
- 全仓 scripts/validate_offline.sh 通过；底盘代码未变更。
- 共享服务只注册 gRPC，不发布 MCP，避免旧版 Pilot 忽略 user_invocable
  标记而把通用动作入口重复暴露给模型；rbnx tools 仅显示七项技能的执行入口。
- 代码生成沿用现有环境，上游有 4 个无关 IDL 依赖跳过提示；本包全部合约生成成功。
- 旧 Atlas 会对同一包的重复合约根目录打印重复加载提示；最终合约 ID 唯一，七个实例注册正常。

## 验收边界

2026-09-28 的原始 8 项物理验收保留在 ACCEPTANCE.md。
本次没有重新进行真机或开放词汇语音实测，不把离线通过写成再次实机验收。
实际动作调用、时长与倒立前进参数保持不变；新注册结构后续可按原展示流程复测。
