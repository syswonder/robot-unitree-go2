"""Pure Chinese intent parsing for RobotTrack follow-distance requests.

This module deliberately has no Robonix, ROS, camera, or controller imports.
It only describes the user's requested setpoint change; a follow provider may
apply that change later through an explicit capability contract.
"""

from __future__ import annotations

from dataclasses import dataclass
import re
import unicodedata
from typing import Literal


DEFAULT_DISTANCE_STEP_M = 1.0

_DIGITS = {
    "零": 0,
    "〇": 0,
    "一": 1,
    "二": 2,
    "两": 2,
    "三": 3,
    "四": 4,
    "五": 5,
    "六": 6,
    "七": 7,
    "八": 8,
    "九": 9,
}
_UNITS = {"十": 10, "百": 100}
_NUMBER = r"(?:\d+(?:\.\d+)?|[零〇一二两三四五六七八九十百]+(?:点[零〇一二两三四五六七八九]+)?)"
_TRAILING_PUNCTUATION = " ,，。.!！?？、;；:：'\"“”‘’"

_SET_PATTERNS = tuple(
    re.compile(pattern)
    for pattern in (
        rf"^(?:请)?(?:把)?(?:跟随)?距离(?:设为|设置为|设定为|调到|调整到)(?P<number>{_NUMBER})米$",
        rf"^(?:请)?(?:保持|固定|设为|设置为|设定为)(?P<number>{_NUMBER})米(?:的)?(?:跟随)?(?:距离)?$",
        rf"^(?:请)?(?:跟我|离我)保持(?P<number>{_NUMBER})米(?:的)?距离$",
    )
)
_FARTHER_PATTERN = re.compile(r"^(?:请)?(?:再)?离我远(?:一)?点(?:儿)?$")
_CLOSER_PATTERN = re.compile(
    r"^(?:请)?(?:再)?(?:靠近我|向我靠近|靠我近|靠近)(?:一)?点(?:儿)?$"
)


@dataclass(frozen=True)
class FollowDistanceIntent:
    """One absolute setpoint or relative setpoint adjustment.

    ``meters`` is the requested absolute distance for ``set`` and a signed
    delta for ``adjust``. Positive adjustment means farther from the person.
    """

    operation: Literal["set", "adjust"]
    meters: float


def _normalize_utterance(text: str) -> str:
    normalized = unicodedata.normalize("NFKC", str(text)).casefold()
    normalized = re.sub(r"\s+", "", normalized)
    return normalized.strip(_TRAILING_PUNCTUATION)


def _parse_chinese_integer(text: str) -> int | None:
    if not text:
        return None
    if not any(character in _UNITS for character in text):
        if not all(character in _DIGITS for character in text):
            return None
        return int("".join(str(_DIGITS[character]) for character in text))

    total = 0
    pending: int | None = None
    previous_unit = 1000
    for character in text:
        if character in _DIGITS:
            pending = _DIGITS[character]
            continue
        unit = _UNITS.get(character)
        if unit is None or unit >= previous_unit:
            return None
        total += (1 if pending is None else pending) * unit
        pending = None
        previous_unit = unit
    return total + (pending or 0)


def _parse_distance(text: str) -> float | None:
    try:
        value = float(text)
    except ValueError:
        integer_text, separator, decimal_text = text.partition("点")
        integer = _parse_chinese_integer(integer_text)
        if integer is None:
            return None
        if separator:
            if not decimal_text or not all(char in _DIGITS for char in decimal_text):
                return None
            decimal = "".join(str(_DIGITS[char]) for char in decimal_text)
            value = float(f"{integer}.{decimal}")
        else:
            value = float(integer)
    return value if value > 0.0 else None


def parse_follow_distance_intent(text: str) -> FollowDistanceIntent | None:
    """Parse a bounded follow-distance utterance without changing runtime state.

    Relative requests represent exactly one default step. Repeating the same
    utterance therefore produces the same delta for the controller to apply to
    its current setpoint. Unrelated navigation or conversational text returns
    ``None``.
    """

    command = _normalize_utterance(text)
    if _FARTHER_PATTERN.fullmatch(command):
        return FollowDistanceIntent("adjust", DEFAULT_DISTANCE_STEP_M)
    if _CLOSER_PATTERN.fullmatch(command):
        return FollowDistanceIntent("adjust", -DEFAULT_DISTANCE_STEP_M)
    for pattern in _SET_PATTERNS:
        match = pattern.fullmatch(command)
        if match is None:
            continue
        meters = _parse_distance(match.group("number"))
        if meters is not None:
            return FollowDistanceIntent("set", meters)
    return None
