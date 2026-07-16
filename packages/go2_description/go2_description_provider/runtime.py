"""Offline-safe validation and parameter materialization helpers."""

from __future__ import annotations

import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET


MAX_URDF_BYTES = 2_000_000


def validate_urdf(urdf_xml: str) -> tuple[str, int, int]:
    encoded = urdf_xml.encode("utf-8")
    if not encoded or len(encoded) > MAX_URDF_BYTES:
        raise ValueError("URDF is empty or exceeds the bounded size")
    lowered = urdf_xml.lower()
    if "<!doctype" in lowered or "<!entity" in lowered:
        raise ValueError("URDF must not contain DTD or entity declarations")
    root = ET.fromstring(urdf_xml)
    if root.tag != "robot":
        raise ValueError("URDF root must be <robot>")
    links = root.findall("link")
    joints = root.findall("joint")
    link_names = [node.attrib.get("name", "").strip() for node in links]
    if not link_names or any(not name for name in link_names):
        raise ValueError("URDF must contain named links")
    if len(set(link_names)) != len(link_names):
        raise ValueError("URDF contains duplicate link names")
    known_links = set(link_names)
    children: list[str] = []
    for joint in joints:
        parent = joint.find("parent")
        child = joint.find("child")
        parent_name = "" if parent is None else parent.attrib.get("link", "")
        child_name = "" if child is None else child.attrib.get("link", "")
        if parent_name not in known_links or child_name not in known_links:
            raise ValueError("URDF joint references an unknown link")
        children.append(child_name)
    roots = sorted(known_links - set(children))
    if roots != ["base_link"]:
        raise ValueError(f"URDF root link must be exactly base_link, found {roots}")
    return roots[0], len(links), len(joints)


def require_pinned_urdf(received: str, pinned_path: Path) -> str:
    pinned = pinned_path.read_text(encoding="utf-8")
    validate_urdf(pinned)
    validate_urdf(received)
    if received.encode("utf-8") != pinned.encode("utf-8"):
        expected = hashlib.sha256(pinned.encode("utf-8")).hexdigest()
        actual = hashlib.sha256(received.encode("utf-8")).hexdigest()
        raise ValueError(
            f"Soma URDF does not match pinned Go2 model: expected {expected}, got {actual}"
        )
    return hashlib.sha256(received.encode("utf-8")).hexdigest()


def write_robot_state_publisher_params(path: Path, urdf_xml: str) -> None:
    validate_urdf(urdf_xml)
    indented = "\n".join(f"      {line}" for line in urdf_xml.splitlines())
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "/**:\n"
        "  ros__parameters:\n"
        "    robot_description: |\n"
        f"{indented}\n",
        encoding="utf-8",
    )
