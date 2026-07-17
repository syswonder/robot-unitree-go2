#!/usr/bin/env python3
"""Fail-closed validation for a Go2 clock evidence bundle.

An approval is deliberately not self-authenticating.  This module starts from
the original topic-info text and classic PCAP, recomputes their hashes and the
RTPS writer correlation, and only then returns the canonical approval payload
that may be handed to the unprivileged feeder.

The code is offline: it opens regular files only and never initializes ROS,
captures packets, changes a clock, or invokes a subprocess.
"""

from __future__ import annotations

import hashlib
import ipaddress
import json
from pathlib import Path
import re
import stat
import sys
from typing import Any


THIS_DIR = Path(__file__).resolve().parent
ROOT = THIS_DIR.parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from correlate_rtps_writer_locator import correlate  # noqa: E402


SCHEMA_VERSION = 2
METHOD = "pcap_rtps_guid_prefix_and_data_writer_entity_correlation"
CONCLUSION = "single_source_proven_by_rtps_data_writer"
ALLOWED_TOPICS = {"/sportmodestate", "/lf/sportmodestate"}
GO2_NETWORK = ipaddress.ip_network("192.168.123.0/24")
DISALLOWED_SOURCE_IPS = {
    ipaddress.ip_address("192.168.123.18"),
    ipaddress.ip_address("192.168.123.99"),
    GO2_NETWORK.network_address,
    GO2_NETWORK.broadcast_address,
}
MAX_PCAP_BYTES = 512 * 1024 * 1024
MAX_TEXT_BYTES = 2 * 1024 * 1024


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def compact_gid(value: str) -> str:
    compact = re.sub(r"[.\-:\s]", "", value).lower()
    if len(compact) < 32 or len(compact) % 2:
        raise ValueError("writer GID must contain at least 16 hexadecimal bytes")
    try:
        bytes.fromhex(compact)
    except ValueError as error:
        raise ValueError("writer GID contains non-hexadecimal characters") from error
    return compact


def publisher_gids_from_topic_info(text: str) -> list[str]:
    """Extract publisher endpoint GIDs from ``ros2 topic info --verbose``."""

    result: list[str] = []
    publisher = False
    for raw_line in text.splitlines():
        line = raw_line.strip()
        if line.lower().startswith("endpoint type:"):
            publisher = line.split(":", 1)[1].strip().upper() == "PUBLISHER"
            continue
        if publisher and line.lower().startswith("gid:"):
            result.append(line.split(":", 1)[1].strip())
            publisher = False
    return result


def _regular_file(bundle: Path, relative: Any, label: str, limit: int) -> Path:
    if not isinstance(relative, str) or not relative or Path(relative).name != relative:
        raise ValueError(f"{label} must be one plain filename inside the bundle")
    path = bundle / relative
    file_stat = path.lstat()
    if not stat.S_ISREG(file_stat.st_mode):
        raise ValueError(f"{label} is not a regular file: {relative}")
    if file_stat.st_mode & 0o022:
        raise ValueError(f"{label} must not be group/world writable: {relative}")
    if file_stat.st_size <= 0 or file_stat.st_size > limit:
        raise ValueError(f"{label} size is outside the accepted bound: {relative}")
    return path


def _load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise ValueError(f"invalid {label} JSON: {error}") from error
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a JSON object")
    return value


def _correlation_security_view(value: dict[str, Any]) -> dict[str, Any]:
    """Fields that are independently derivable from raw capture bytes."""

    return {
        "schema_version": value.get("schema_version"),
        "method": value.get("method"),
        "writer_gid": compact_gid(str(value.get("writer_gid", ""))),
        "rtps_participant_guid_prefix": value.get("rtps_participant_guid_prefix"),
        "rtps_writer_entity_id": value.get("rtps_writer_entity_id"),
        "total_packets": value.get("total_packets"),
        "decoded_ipv4_udp_packets": value.get("decoded_ipv4_udp_packets"),
        "participant_prefix_sources": value.get("participant_prefix_sources"),
        "writer_data_sources": value.get("writer_data_sources"),
        "proven_source_ips": value.get("proven_source_ips"),
        "conclusion": value.get("conclusion"),
    }


def _eligible_source(value: Any) -> str:
    try:
        address = ipaddress.ip_address(str(value))
    except ValueError as error:
        raise ValueError("correlation has an invalid source IP") from error
    if address not in GO2_NETWORK or address in DISALLOWED_SOURCE_IPS:
        raise ValueError(f"source IP is not an eligible Go2 host: {address}")
    return str(address)


def verify_bundle(bundle: Path, approval_filename: str = "go2-clock-ref-approval.json") -> dict[str, Any]:
    """Rebuild all security conclusions and return a verified approval."""

    bundle = bundle.expanduser().resolve()
    bundle_stat = bundle.lstat()
    if not stat.S_ISDIR(bundle_stat.st_mode):
        raise ValueError("evidence bundle is not a directory")
    approval_path = _regular_file(
        bundle, approval_filename, "approval", MAX_TEXT_BYTES
    )
    approval = _load_json(approval_path, "approval")
    if approval.get("schema_version") != SCHEMA_VERSION:
        raise ValueError("legacy/self-asserted approval rejected; schema_version=2 required")
    if approval.get("purpose") != "go2-source-clock-to-chrony-refclock":
        raise ValueError("approval purpose is not exact")

    evidence = approval.get("evidence_bundle")
    if not isinstance(evidence, dict):
        raise ValueError("approval is missing evidence_bundle")
    pcap_path = _regular_file(
        bundle, evidence.get("pcap_file"), "PCAP", MAX_PCAP_BYTES
    )
    pcap_sha256 = sha256_file(pcap_path)
    if evidence.get("pcap_sha256") != pcap_sha256:
        raise ValueError("PCAP SHA-256 mismatch")

    writers = approval.get("approved_writers")
    if not isinstance(writers, list) or not writers:
        raise ValueError("approval requires approved_writers")
    seen_topics: set[str] = set()
    verified_writers: list[dict[str, Any]] = []
    for item in writers:
        if not isinstance(item, dict):
            raise ValueError("approved writer entry must be an object")
        topic = item.get("topic")
        if topic not in ALLOWED_TOPICS or topic in seen_topics:
            raise ValueError(f"unsupported or duplicate topic: {topic!r}")
        seen_topics.add(topic)

        topic_info_path = _regular_file(
            bundle, item.get("topic_info_file"), "topic-info", MAX_TEXT_BYTES
        )
        correlation_path = _regular_file(
            bundle, item.get("correlation_file"), "correlation", MAX_TEXT_BYTES
        )
        if item.get("topic_info_sha256") != sha256_file(topic_info_path):
            raise ValueError(f"topic-info SHA-256 mismatch for {topic}")
        correlation_sha256 = sha256_file(correlation_path)
        if item.get("correlation_sha256") != correlation_sha256:
            raise ValueError(f"correlation SHA-256 mismatch for {topic}")

        gids = publisher_gids_from_topic_info(
            topic_info_path.read_text(encoding="utf-8")
        )
        if len(gids) != 1:
            raise ValueError(
                f"topic-info must contain exactly one publisher GID for {topic}"
            )
        gid = compact_gid(gids[0])
        stored_correlation = _load_json(correlation_path, "correlation")
        recomputed = correlate(pcap_path, gids[0])
        if _correlation_security_view(stored_correlation) != _correlation_security_view(recomputed):
            raise ValueError(f"stored correlation is not reproducible for {topic}")
        if recomputed.get("method") != METHOD or recomputed.get("conclusion") != CONCLUSION:
            raise ValueError(f"raw evidence does not prove one writer source for {topic}")
        sources = recomputed.get("proven_source_ips")
        if not isinstance(sources, list) or len(sources) != 1:
            raise ValueError(f"raw evidence does not have one source IP for {topic}")
        source_ip = _eligible_source(sources[0])
        prefix = str(recomputed["rtps_participant_guid_prefix"]).lower()
        expected = {
            "topic": topic,
            "writer_gid": gid,
            "rtps_participant_guid_prefix": prefix,
            "source_ip": source_ip,
            "correlation_method": METHOD,
            "correlation_conclusion": CONCLUSION,
            "pcap_sha256": pcap_sha256,
            "topic_info_file": topic_info_path.name,
            "topic_info_sha256": sha256_file(topic_info_path),
            "correlation_file": correlation_path.name,
            "correlation_sha256": correlation_sha256,
        }
        for key, expected_value in expected.items():
            actual = item.get(key)
            if key == "writer_gid":
                actual = compact_gid(str(actual))
            if actual != expected_value:
                raise ValueError(f"approval {key} mismatch for {topic}")
        verified_writers.append(expected)

    # Return only fields reconstructed or explicitly non-authorizing metadata.
    return {
        "schema_version": SCHEMA_VERSION,
        "purpose": "go2-source-clock-to-chrony-refclock",
        "activation_authorized": False,
        "evidence_bundle": {
            "pcap_file": pcap_path.name,
            "pcap_sha256": pcap_sha256,
        },
        "approved_writers": verified_writers,
    }


def canonical_json(payload: dict[str, Any]) -> str:
    return json.dumps(payload, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
