# SPDX-License-Identifier: Apache-2.0
"""Small explicit Chinese command vocabulary for the Client-facing Skill.

An utterance is mapped to one canonical plan, never to an arbitrary SDK name.
Negations and stop/cancel phrases are not interpreted as execute commands.
"""

from __future__ import annotations

import re


NEGATIONS = ("不要", "别", "不许", "停止", "取消", "不想", "不能", "别再")
STOP_UTTERANCES = frozenset({"停下", "停下来", "停止动作", "停止表演",
                             "结束表演", "取消当前动作", "停止跳舞", "停止舞蹈"})


def is_cancel_utterance(text: str) -> bool:
    return isinstance(text, str) and re.sub(r"[\s，。！？,.!?]+", "", text) in STOP_UTTERANCES


def action_from_utterance(text: str) -> str | None:
    if not isinstance(text, str):
        return None
    utterance = re.sub(r"[\s，。！？,.!?]+", "", text)
    if (not utterance or any(word in utterance for word in NEGATIONS)
            or utterance.endswith(("吗", "么")) or any(word in utterance for word in
                ("鞠躬", "鞠个躬", "左右摆动", "摇摆", "前跳", "向前跳", "后空翻"))):
        return None
    matches: list[str] = []
    if "倒立" in utterance and any(word in utterance for word in ("走", "前进", "移动")):
        matches.append("handstand_walk")
    elif "倒立" in utterance:
        matches.append("handstand")
    if "拜年" in utterance or "拜个年" in utterance:
        matches.append("new_year_greeting")
    if any(word in utterance for word in (
        "蹲下再起来", "卧下再起身", "卧倒再起身", "卧倒再起来",
    )):
        matches.append("crouch")
    if any(word in utterance for word in ("舞蹈", "跳舞", "支舞")):
        matches.append("dance_2" if any(word in utterance for word in
                    ("第二", "二号", "另一支")) else "dance_1")
    if "伸展" in utterance or "拉伸" in utterance:
        matches.append("stretch")
    if any(word in utterance for word in ("打招呼", "打个招呼", "挥手")):
        matches.append("hello")
    # Do not silently execute one half of a multi-action sentence.
    return matches[0] if len(matches) == 1 else None
