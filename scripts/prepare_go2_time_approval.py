#!/usr/bin/env python3
"""Build a reviewable clock-reference approval from original evidence.

This is an offline transformation.  It neither contacts ROS nor activates the
chrony bridge.  It refuses to trust a correlation JSON on its own: the writer
GID is reread from topic-info and the RTPS correlation is recomputed from the
classic PCAP before a schema-v2 approval is emitted.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "deploy" / "time-sync"))
sys.path.insert(0, str(ROOT / "scripts"))

from evidence_bundle import (  # noqa: E402
    ALLOWED_TOPICS,
    CONCLUSION,
    METHOD,
    _correlation_security_view,
    _eligible_source,
    compact_gid,
    publisher_gids_from_topic_info,
    sha256_file,
)
from correlate_rtps_writer_locator import correlate  # noqa: E402


def _mapping(value: str) -> tuple[str, Path]:
    if "=" not in value:
        raise argparse.ArgumentTypeError("expected TOPIC=CORRELATION_JSON")
    topic, path = value.split("=", 1)
    if topic not in ALLOWED_TOPICS:
        raise argparse.ArgumentTypeError(f"unsupported clock topic: {topic}")
    if not path:
        raise argparse.ArgumentTypeError("correlation path is empty")
    return topic, Path(path)


def approved_writer(
    topic: str, topic_info_path: Path, correlation_path: Path, pcap_path: Path
) -> dict[str, Any]:
    topic_info_path = topic_info_path.expanduser().resolve()
    correlation_path = correlation_path.expanduser().resolve()
    pcap_path = pcap_path.expanduser().resolve()
    gids = publisher_gids_from_topic_info(
        topic_info_path.read_text(encoding="utf-8")
    )
    if len(gids) != 1:
        raise ValueError(
            f"topic-info must contain exactly one publisher GID for {topic}"
        )
    gid = compact_gid(gids[0])
    try:
        correlation = json.loads(correlation_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise ValueError(
            f"invalid correlation JSON {correlation_path}: {error}"
        ) from error
    recomputed = correlate(pcap_path, gids[0])
    if _correlation_security_view(correlation) != _correlation_security_view(recomputed):
        raise ValueError(f"correlation is not reproducible from PCAP for {topic}")
    if recomputed.get("method") != METHOD or recomputed.get("conclusion") != CONCLUSION:
        raise ValueError(f"raw evidence does not prove one writer source for {topic}")
    sources = recomputed.get("proven_source_ips")
    if not isinstance(sources, list) or len(sources) != 1:
        raise ValueError(f"raw evidence must contain one proven source for {topic}")
    source_ip = _eligible_source(sources[0])
    return {
        "topic": topic,
        "writer_gid": gid,
        "rtps_participant_guid_prefix": recomputed["rtps_participant_guid_prefix"],
        "source_ip": source_ip,
        "correlation_method": METHOD,
        "correlation_conclusion": CONCLUSION,
        "pcap_sha256": sha256_file(pcap_path),
        "topic_info_file": topic_info_path.name,
        "topic_info_sha256": sha256_file(topic_info_path),
        "correlation_file": correlation_path.name,
        "correlation_sha256": sha256_file(correlation_path),
    }


def write_private_json(path: Path, payload: dict[str, Any]) -> None:
    resolved = path.expanduser().resolve()
    resolved.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    if resolved.exists():
        raise FileExistsError(f"refusing to overwrite: {resolved}")
    with resolved.open("x", encoding="utf-8") as stream:
        json.dump(payload, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
    os.chmod(resolved, 0o600)


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Prepare, but do not activate, Go2 clock-source approval"
    )
    parser.add_argument(
        "--topic-correlation",
        action="append",
        required=True,
        type=_mapping,
        metavar="TOPIC=JSON",
    )
    parser.add_argument(
        "--topic-info",
        action="append",
        required=True,
        type=_mapping,
        metavar="TOPIC=TOPIC_INFO_TXT",
    )
    parser.add_argument("--pcap", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    return parser


def main(argv: list[str] | None = None) -> int:
    arguments = build_argument_parser().parse_args(argv)
    output_parent = arguments.output.expanduser().resolve().parent
    pcap_path = arguments.pcap.expanduser().resolve()
    correlations = dict(arguments.topic_correlation)
    topic_infos = dict(arguments.topic_info)
    if len(correlations) != len(arguments.topic_correlation):
        print("approval preparation failed: duplicate correlation topic", file=sys.stderr)
        return 2
    if len(topic_infos) != len(arguments.topic_info):
        print("approval preparation failed: duplicate topic-info topic", file=sys.stderr)
        return 2
    if set(correlations) != set(topic_infos):
        print(
            "approval preparation failed: topic-info/correlation topics differ",
            file=sys.stderr,
        )
        return 2
    evidence_paths = [pcap_path]
    evidence_paths.extend(path.expanduser().resolve() for path in correlations.values())
    evidence_paths.extend(path.expanduser().resolve() for path in topic_infos.values())
    if any(path.parent != output_parent for path in evidence_paths):
        print(
            "approval preparation failed: all evidence and output must be direct files in one bundle directory",
            file=sys.stderr,
        )
        return 2
    writers: list[dict[str, Any]] = []
    try:
        for topic in sorted(correlations):
            writers.append(
                approved_writer(
                    topic, topic_infos[topic], correlations[topic], pcap_path
                )
            )
        payload = {
            "schema_version": 2,
            "purpose": "go2-source-clock-to-chrony-refclock",
            "activation_authorized": False,
            "evidence_bundle": {
                "pcap_file": pcap_path.name,
                "pcap_sha256": sha256_file(pcap_path),
            },
            "approved_writers": writers,
            "note": (
                "This non-authorizing file binds original evidence; it does not grant "
                "SYS_TIME or activate clock discipline."
            ),
        }
        write_private_json(arguments.output, payload)
    except (OSError, ValueError) as error:
        print(f"approval preparation failed: {error}", file=sys.stderr)
        return 2
    print(f"inactive schema-v2 approval prepared at {arguments.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
