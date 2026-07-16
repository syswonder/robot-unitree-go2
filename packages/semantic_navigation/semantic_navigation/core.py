"""ROS/Robonix-independent landmark loading and matching."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import re
import unicodedata
from typing import Any

import yaml

from .map_lifecycle import LifecycleBindingError, parse_generation


class LandmarkError(ValueError):
    """A landmark configuration or lookup failed closed."""


_SEPARATORS = re.compile(r"[\s\u3000,，。.!！?？、;；:：'\"“”‘’()（）\[\]【】<>《》_-]+")


def normalize_text(value: str) -> str:
    """Normalize common Chinese ASR spacing/punctuation without translating."""

    return _SEPARATORS.sub("", unicodedata.normalize("NFKC", value).casefold())


def normalize_yaw(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


@dataclass(frozen=True)
class Landmark:
    id: str
    name: str
    aliases: tuple[str, ...]
    map_id: str
    map_generation: int
    frame_id: str
    x: float
    y: float
    yaw: float
    verified: bool
    metadata: dict[str, Any]

    @property
    def terms(self) -> tuple[str, ...]:
        return (self.name, *self.aliases)


class LandmarkStore:
    def __init__(
        self,
        map_id: str,
        map_generation: int,
        frame_id: str,
        landmarks: tuple[Landmark, ...],
    ):
        if not map_id.strip():
            raise LandmarkError("landmark file has an empty map_id")
        if frame_id != "map":
            raise LandmarkError(f"landmark frame must be 'map', got {frame_id!r}")
        if not landmarks:
            raise LandmarkError("landmark file contains no landmarks")
        ids = [item.id for item in landmarks]
        if len(ids) != len(set(ids)):
            raise LandmarkError("landmark ids must be unique")
        self.map_id = map_id
        self.map_generation = map_generation
        self.frame_id = frame_id
        self.landmarks = landmarks

    @classmethod
    def from_path(cls, path: str | Path) -> "LandmarkStore":
        source = Path(path)
        if not source.is_file():
            raise LandmarkError(f"landmark file not found: {source}")
        try:
            raw = yaml.safe_load(source.read_text(encoding="utf-8")) or {}
        except (OSError, yaml.YAMLError) as exc:
            raise LandmarkError(f"cannot read landmark file {source}: {exc}") from exc
        return cls.from_mapping(raw)

    @classmethod
    def from_mapping(cls, raw: Any) -> "LandmarkStore":
        if not isinstance(raw, dict):
            raise LandmarkError("landmark document must be a mapping")
        if raw.get("schema_version") != 2:
            raise LandmarkError("unsupported semantic landmark schema_version")
        map_id = str(raw.get("map_id", "")).strip()
        try:
            map_generation = parse_generation(
                raw.get("map_generation"), label="landmark map_generation"
            )
        except LifecycleBindingError as exc:
            raise LandmarkError(str(exc)) from exc
        frame_id = str(raw.get("frame_id", "map")).strip()
        rows = raw.get("landmarks")
        if not isinstance(rows, list):
            raise LandmarkError("landmarks must be a list")

        items: list[Landmark] = []
        seen_terms: dict[str, str] = {}
        for index, row in enumerate(rows):
            if not isinstance(row, dict):
                raise LandmarkError(f"landmarks[{index}] must be a mapping")
            landmark_id = str(row.get("id", "")).strip()
            name = str(row.get("name", "")).strip()
            aliases_raw = row.get("aliases") or []
            pose = row.get("pose") or {}
            verified = row.get("verified", False)
            metadata = row.get("metadata") or {}
            if not landmark_id or not name:
                raise LandmarkError(f"landmarks[{index}] requires id and name")
            if not isinstance(aliases_raw, list) or not isinstance(pose, dict):
                raise LandmarkError(f"landmark {landmark_id!r} aliases/pose have invalid types")
            if not isinstance(verified, bool):
                raise LandmarkError(
                    f"landmark {landmark_id!r} verified must be a YAML boolean"
                )
            if not isinstance(metadata, dict):
                raise LandmarkError(f"landmark {landmark_id!r} metadata must be a mapping")
            aliases = tuple(str(value).strip() for value in aliases_raw if str(value).strip())
            try:
                x = float(pose["x"])
                y = float(pose["y"])
                yaw = normalize_yaw(float(pose["yaw"]))
            except (KeyError, TypeError, ValueError) as exc:
                raise LandmarkError(f"landmark {landmark_id!r} has an invalid pose") from exc
            if not all(math.isfinite(value) for value in (x, y, yaw)):
                raise LandmarkError(f"landmark {landmark_id!r} pose must be finite")
            item = Landmark(
                id=landmark_id,
                name=name,
                aliases=aliases,
                map_id=map_id,
                map_generation=map_generation,
                frame_id=frame_id,
                x=x,
                y=y,
                yaw=yaw,
                verified=verified,
                metadata=dict(metadata),
            )
            for term in item.terms:
                normalized = normalize_text(term)
                if not normalized:
                    raise LandmarkError(f"landmark {landmark_id!r} contains an empty term")
                owner = seen_terms.get(normalized)
                if owner is not None and owner != landmark_id:
                    raise LandmarkError(
                        f"ambiguous normalized landmark term {term!r}: {owner!r} and {landmark_id!r}"
                    )
                seen_terms[normalized] = landmark_id
            items.append(item)
        return cls(
            map_id=map_id,
            map_generation=map_generation,
            frame_id=frame_id,
            landmarks=tuple(items),
        )

    def resolve(
        self,
        utterance: str,
        *,
        expected_map_id: str,
        expected_generation: int,
        require_verified: bool = True,
    ) -> Landmark:
        query = normalize_text(utterance)
        if not query:
            raise LandmarkError("landmark name is empty")
        if self.map_id != expected_map_id:
            raise LandmarkError(
                f"landmark map_id {self.map_id!r} does not match active map {expected_map_id!r}"
            )
        if self.map_generation != expected_generation:
            raise LandmarkError(
                f"landmark map generation {self.map_generation} does not match active map "
                f"generation {expected_generation}"
            )

        matches: list[tuple[int, Landmark, str]] = []
        for item in self.landmarks:
            for term in item.terms:
                normalized = normalize_text(term)
                if normalized and normalized in query:
                    matches.append((len(normalized), item, term))
        if not matches:
            raise LandmarkError(f"unknown semantic landmark in {utterance!r}")
        best_length = max(length for length, _, _ in matches)
        best = {item.id: item for length, item, _ in matches if length == best_length}
        if len(best) != 1:
            raise LandmarkError(f"ambiguous semantic landmark in {utterance!r}: {sorted(best)}")
        item = next(iter(best.values()))
        if require_verified and not item.verified:
            raise LandmarkError(
                f"landmark {item.name!r} has no physically verified approach pose"
            )
        return item
