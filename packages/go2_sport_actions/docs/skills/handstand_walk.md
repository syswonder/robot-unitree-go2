---
description: "宇树 Go2 EDU 倒立后以 0.06 m/s 向前走 5 秒，停止并恢复四足。"
---

# 宇树 Go2 倒立前进

宇树 Go2 EDU 倒立后以 0.06 m/s 向前走 5 秒，停止并恢复四足。

仅适用于支持相应动作的宇树 Go2 EDU 固件，不是其他本体或 MuJoCo 仿真动作。
底层接口：HandStand + Move + StopMove + HandStand(false)。执行动作无需填动作名称或速度参数，request_id 用于请求去重。
只执行操作者当前要求的这一项；accepted 是受理，不是动作完成。
等待 status 返回终态且实际站稳后再切换动作；需要中止时调用 cancel。
