# SPDX-License-Identifier: Apache-2.0
"""Describe official high-level SDK calls, without importing or invoking SDKs.

An SDK declaration is not device/firmware support or a physical acceptance.
In particular, HandStand(bool) is not a handstand-walking command.
"""
from dataclasses import asdict, dataclass
import json


@dataclass(frozen=True)
class Action:
    name: str
    label: str
    sdk_method: str | None
    sdk_arguments: tuple
    note: str


ACTIONS = (
    Action("hello", "打招呼", "Hello", (), "2026-09-27 本机 Client 语音实测动作明显"),
    Action("stretch", "伸展", "Stretch", (), "2026-09-27 本机 Client 语音实测动作明显"),
    Action("dance_1", "舞蹈一", "Dance1", (), "2026-09-27 本机 Client 语音实测完整跳完、站姿正常"),
    Action("dance_2", "舞蹈二", "Dance2", (), "2026-09-27 本机 Client 语音实测自然收尾、站姿正常"),
    Action("new_year_greeting", "拜年", "Scrape", (), "2026-09-27 本机 Client 语音实测与遥控器拜年一致，结束后正常静止"),
    Action("crouch", "卧倒再起身", None, (), "2026-09-27 本机 Client 语音实测先卧倒、再自行站起并静止"),
    Action("handstand", "倒立展示", "HandStand", (True, False), "2026-09-27 本机 Client 语音实测进入倒立并完整退出"),
    Action("handstand_walk", "倒立行走", None, (), "2026-09-27 本机 Client 语音分别实测 0.06 m/s × 2 s 和 5 s 前进，并自行恢复"),
)
BY_NAME = {action.name: action for action in ACTIONS}
PHYSICALLY_VERIFIED = frozenset({
    "dance_1", "dance_2", "hello", "stretch", "new_year_greeting", "crouch",
    "handstand", "handstand_walk",
})


def describe(action: Action) -> dict:
    return {
        **asdict(action),
        "sdk_declared": action.sdk_method is not None,
        "firmware_verified": action.name in PHYSICALLY_VERIFIED,
        "physical_tested": action.name in PHYSICALLY_VERIFIED,
        "physical_plan_prepared": False,
        "backend": "existing_go2_chassis_daemon",
        "operator_reported_manual_support": action.name in {
            "dance_1", "dance_2", "new_year_greeting", "handstand", "handstand_walk",
        },
        "manual_support_scope": (
            "2026-09-27 本机 Go2 EDU 的 Client 语音分别触发 Dance1/Dance2，"
            "操作者确认完整/自然收尾且站姿正常；仅适用于这两项"
            if action.name in {"dance_1", "dance_2"} else
            "2026-09-27 本机 Go2 EDU 的 Client 语音触发，操作者确认动作明显；"
            "日志有 StopMove 响应，不代表其他固件支持"
            if action.name in {"hello", "stretch"} else
            "2026-09-27 本机 Go2 EDU 的 Client 语音触发，操作者确认与遥控器拜年一致，"
            "结束后正常静止；不代表其他固件支持"
            if action.name == "new_year_greeting" else
            "2026-09-27 本机 Go2 EDU 的 Client 语音触发，操作者确认先卧倒再站起，"
            "结束后静止；不代表其他固件支持"
            if action.name == "crouch" else
            "2026-09-27 本机 Go2 EDU 的 Client 语音触发，操作者确认完整进入及退出倒立；"
            "不代表其他固件支持"
            if action.name == "handstand" else
            "2026-09-27 本机 Go2 EDU 的 Client 语音分别触发 0.06 m/s × 2 s 和 5 s 倒立前进；"
            "操作者确认 5 s 版明显更远、自行恢复正常站姿且完全静止"
            if action.name == "handstand_walk" else
            ""
        ),
    }


def catalog() -> dict:
    return {
        "schema_version": 1,
        "scope": "unitree_go2_named_sport_actions",
        "physical_validation_complete": True,
        "validation_scope": "Operator-observed eight-action Client voice sequence on one Go2 EDU; confirmed 2026-09-28. Other robots and firmware need their own acceptance.",
        "actions": [describe(action) for action in ACTIONS],
    }


def preview(name: str) -> dict:
    # No fuzzy matching or free-form method dispatch in a motion-facing API.
    if not isinstance(name, str) or name not in BY_NAME:
        return {"known": False, "physical_plan_prepared": False,
                "reason": "unknown_action"}
    return {"known": True, **describe(BY_NAME[name])}


def dumps(value: dict) -> str:
    return json.dumps(value, ensure_ascii=False, allow_nan=False)


if __name__ == "__main__":
    print(dumps(catalog()))
