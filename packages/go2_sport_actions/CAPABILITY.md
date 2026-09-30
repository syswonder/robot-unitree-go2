# 宇树 Go2 七项动作技能包

一个 package，七个独立注册 Skill。各自定义与初始化参数见 go2_sport_actions/skills.json；
各 Skill 的专属中文说明见 docs/skills/，不会要求模型填写通用 action_name。

默认使用 package_manifest.<action>.yaml 注册 hello、stretch、new_year_greeting、
crouch、dance、handstand、handstand_walk。舞蹈由共享服务随机选择已验收的 Dance1/Dance2。
所有动作仅适用于兼容的宇树 Go2 EDU，不是跨本体动作或 MuJoCo 策略。
七个 Skill 共用一个 Service 和底盘所有者，后者不是另一个用户动作技能。

0.1.0 的通用接口仅作旧部署兼容，不向模型公开。接受不代表物理完成；
完成状态、实际姿态确认、停止与取消规则见 README.md。
