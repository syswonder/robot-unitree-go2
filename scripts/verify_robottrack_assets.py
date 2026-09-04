#!/usr/bin/env python3
"""Verify pinned, workspace-local MiniCPM-RobotTrack assets without networking."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import sys
from typing import Any, Mapping, Sequence

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = REPO_ROOT.parents[1]
DEFAULT_MANIFEST = REPO_ROOT / "config" / "robottrack_assets.yaml"
HEX_REVISION_RE = re.compile(r"[0-9a-f]{40}")
SHA256_RE = re.compile(r"[0-9a-f]{64}")


class AssetConfigurationError(RuntimeError):
    """Raised when the local asset manifest or root is malformed."""


@dataclass(frozen=True)
class AssetResult:
    """The complete result for one source or model asset."""

    asset_id: str
    name: str
    root: str
    ready: bool
    revision: str | None
    verification: str
    provenance: str | None
    files_checked: int
    bytes_checked: int
    errors: tuple[str, ...]


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _require_mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise AssetConfigurationError(f"{label} must be a mapping")
    return value


def _require_string(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise AssetConfigurationError(f"{label} must be a non-empty string")
    return value.strip()


def _relative_path(value: Any, label: str) -> Path:
    path = Path(_require_string(value, label))
    if path.is_absolute() or ".." in path.parts:
        raise AssetConfigurationError(f"{label} must be a safe relative path")
    return path


def load_manifest(path: Path) -> Mapping[str, Any]:
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise AssetConfigurationError(
            f"cannot read asset manifest {path}: {error}"
        ) from error
    manifest = _require_mapping(document, "manifest")
    if manifest.get("schema_version") != 1:
        raise AssetConfigurationError("manifest schema_version must be 1")
    _require_mapping(manifest.get("upstream_root"), "upstream_root")
    _require_mapping(manifest.get("source"), "source")
    assets = manifest.get("assets")
    if not isinstance(assets, list) or not assets:
        raise AssetConfigurationError("assets must be a non-empty list")
    return manifest


def resolve_upstream_root(
    workspace_root: Path,
    manifest: Mapping[str, Any],
    cli_root: Path | None,
    environ: Mapping[str, str],
) -> Path:
    try:
        workspace = workspace_root.resolve(strict=True)
    except OSError as error:
        raise AssetConfigurationError(f"workspace root is unavailable: {error}") from error
    if not workspace.is_dir():
        raise AssetConfigurationError(f"workspace root is not a directory: {workspace}")

    root_config = _require_mapping(manifest.get("upstream_root"), "upstream_root")
    environment_name = _require_string(
        root_config.get("environment"), "upstream_root.environment"
    )
    default_value = _require_string(root_config.get("default"), "upstream_root.default")
    configured: str | Path = cli_root if cli_root is not None else environ.get(
        environment_name, default_value
    )
    candidate = Path(configured)
    if not candidate.is_absolute():
        candidate = workspace / candidate
    resolved = candidate.resolve(strict=False)
    if not resolved.is_relative_to(workspace):
        raise AssetConfigurationError(
            f"upstream root must remain inside workspace {workspace}: {resolved}"
        )
    return resolved


def _contained_path(root: Path, relative: Path, workspace_root: Path) -> Path:
    candidate = (root / relative).resolve(strict=False)
    if not candidate.is_relative_to(workspace_root):
        raise AssetConfigurationError(f"asset path escapes the workspace: {candidate}")
    return candidate


def _git_head(path: Path) -> str:
    completed = subprocess.run(
        ["git", "-C", str(path), "rev-parse", "--verify", "HEAD"],
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        timeout=10,
    )
    if completed.returncode != 0:
        detail = completed.stderr.strip() or "git rev-parse failed"
        raise OSError(detail)
    return completed.stdout.strip()


def _read_revision_marker(path: Path) -> str:
    try:
        first_line = path.read_text(encoding="utf-8").splitlines()[0].strip()
    except IndexError as error:
        raise OSError("revision marker is empty") from error
    if not HEX_REVISION_RE.fullmatch(first_line):
        raise OSError("revision marker does not begin with a 40-character revision")
    return first_line


def _verify_declared_files(
    root: Path,
    entries: Any,
    workspace_root: Path,
) -> tuple[int, int, list[str]]:
    if not isinstance(entries, list) or not entries:
        raise AssetConfigurationError("asset files must be a non-empty list")
    files_checked = 0
    bytes_checked = 0
    errors: list[str] = []
    for index, value in enumerate(entries):
        entry = _require_mapping(value, f"files[{index}]")
        relative = _relative_path(entry.get("path"), f"files[{index}].path")
        expected_size = entry.get("size")
        if (
            not isinstance(expected_size, int)
            or isinstance(expected_size, bool)
            or expected_size < 0
        ):
            raise AssetConfigurationError(
                f"files[{index}].size must be a nonnegative integer"
            )
        expected_sha = entry.get("sha256")
        if expected_sha is not None:
            expected_sha = _require_string(expected_sha, f"files[{index}].sha256")
            if not SHA256_RE.fullmatch(expected_sha):
                raise AssetConfigurationError(
                    f"files[{index}].sha256 must be 64 lowercase hexadecimal characters"
                )

        path = _contained_path(root, relative, workspace_root)
        try:
            resolved = path.resolve(strict=True)
            if not resolved.is_relative_to(workspace_root):
                raise OSError("resolved file escapes the workspace")
            stat = resolved.stat()
        except OSError as error:
            errors.append(f"missing {relative}: {error}")
            continue
        if not resolved.is_file():
            errors.append(f"not a regular file: {relative}")
            continue

        files_checked += 1
        bytes_checked += stat.st_size
        if stat.st_size != expected_size:
            errors.append(
                f"size mismatch for {relative}: expected {expected_size}, got {stat.st_size}"
            )
        if expected_sha is not None:
            actual_sha = sha256_file(resolved)
            if actual_sha != expected_sha:
                errors.append(
                    f"sha256 mismatch for {relative}: expected {expected_sha}, got {actual_sha}"
                )
    return files_checked, bytes_checked, errors


def _verify_source(
    source: Mapping[str, Any], upstream_root: Path, workspace_root: Path
) -> AssetResult:
    name = _require_string(source.get("name"), "source.name")
    relative_root = _relative_path(source.get("root"), "source.root")
    root = _contained_path(upstream_root, relative_root, workspace_root)
    expected_revision = _require_string(
        source.get("git_revision"), "source.git_revision"
    )
    if not HEX_REVISION_RE.fullmatch(expected_revision):
        raise AssetConfigurationError(
            "source.git_revision must be 40 lowercase hexadecimal characters"
        )
    errors: list[str] = []
    revision: str | None = None
    try:
        revision = _git_head(root)
    except (OSError, subprocess.SubprocessError) as error:
        errors.append(f"cannot read git HEAD: {error}")
    else:
        if revision != expected_revision:
            errors.append(
                f"git revision mismatch: expected {expected_revision}, got {revision}"
            )

    required_files = source.get("required_files")
    if not isinstance(required_files, list) or not required_files:
        raise AssetConfigurationError("source.required_files must be a non-empty list")
    checked = 0
    total_bytes = 0
    for index, value in enumerate(required_files):
        relative = _relative_path(value, f"source.required_files[{index}]")
        path = _contained_path(root, relative, workspace_root)
        try:
            resolved = path.resolve(strict=True)
            if not resolved.is_relative_to(workspace_root) or not resolved.is_file():
                raise OSError("not a workspace-local regular file")
            size = resolved.stat().st_size
        except OSError as error:
            errors.append(f"missing {relative}: {error}")
            continue
        checked += 1
        total_bytes += size
    return AssetResult(
        asset_id="source",
        name=name,
        root=str(root),
        ready=not errors,
        revision=revision,
        verification="git_revision_and_files",
        provenance=None,
        files_checked=checked,
        bytes_checked=total_bytes,
        errors=tuple(errors),
    )


def _verify_asset(
    asset: Mapping[str, Any], upstream_root: Path, workspace_root: Path
) -> AssetResult:
    asset_id = _require_string(asset.get("id"), "asset.id")
    name = _require_string(asset.get("name"), f"asset {asset_id}.name")
    relative_root = _relative_path(asset.get("root"), f"asset {asset_id}.root")
    root = _contained_path(upstream_root, relative_root, workspace_root)
    expected_revision = _require_string(
        asset.get("revision"), f"asset {asset_id}.revision"
    )
    if not HEX_REVISION_RE.fullmatch(expected_revision):
        raise AssetConfigurationError(
            f"asset {asset_id}.revision must be 40 lowercase hexadecimal characters"
        )
    verification = _require_string(
        asset.get("verification", "revision_and_files"),
        f"asset {asset_id}.verification",
    )
    if verification not in {"revision_and_files", "sha256_files"}:
        raise AssetConfigurationError(
            f"asset {asset_id}.verification must be revision_and_files or sha256_files"
        )

    entries = asset.get("files")
    provenance_label: str | None = None
    if verification == "sha256_files":
        if not isinstance(entries, list) or not entries:
            raise AssetConfigurationError("asset files must be a non-empty list")
        for index, value in enumerate(entries):
            entry = _require_mapping(value, f"files[{index}]")
            if entry.get("sha256") is None:
                raise AssetConfigurationError(
                    f"asset {asset_id} sha256_files verification requires "
                    f"files[{index}].sha256"
                )
        provenance = _require_mapping(
            asset.get("provenance"), f"asset {asset_id}.provenance"
        )
        provider = _require_string(
            provenance.get("provider"), f"asset {asset_id}.provenance.provider"
        )
        repository = _require_string(
            provenance.get("repository"),
            f"asset {asset_id}.provenance.repository",
        )
        resolved_revision = _require_string(
            provenance.get("resolved_revision"),
            f"asset {asset_id}.provenance.resolved_revision",
        )
        if not HEX_REVISION_RE.fullmatch(resolved_revision):
            raise AssetConfigurationError(
                f"asset {asset_id}.provenance.resolved_revision must be "
                "40 lowercase hexadecimal characters"
            )
        provenance_label = f"{provider}:{repository}@{resolved_revision}"

    errors: list[str] = []
    revision: str | None = None
    if verification == "revision_and_files":
        marker_relative = _relative_path(
            asset.get("revision_marker"), f"asset {asset_id}.revision_marker"
        )
        marker = _contained_path(root, marker_relative, workspace_root)
        try:
            if marker.is_file():
                revision = _read_revision_marker(marker)
            elif (root / ".git").exists():
                revision = _git_head(root)
            else:
                raise OSError(f"missing revision marker {marker_relative}")
        except (OSError, UnicodeError, subprocess.SubprocessError) as error:
            errors.append(f"cannot verify revision: {error}")
        else:
            if revision != expected_revision:
                errors.append(
                    f"revision mismatch: expected {expected_revision}, got {revision}"
                )

    checked, total_bytes, file_errors = _verify_declared_files(
        root, entries, workspace_root
    )
    errors.extend(file_errors)
    return AssetResult(
        asset_id=asset_id,
        name=name,
        root=str(root),
        ready=not errors,
        revision=revision,
        verification=verification,
        provenance=provenance_label,
        files_checked=checked,
        bytes_checked=total_bytes,
        errors=tuple(errors),
    )


def verify(
    manifest: Mapping[str, Any], workspace_root: Path, upstream_root: Path
) -> list[AssetResult]:
    workspace = workspace_root.resolve(strict=True)
    upstream = upstream_root.resolve(strict=False)
    if not upstream.is_relative_to(workspace):
        raise AssetConfigurationError("upstream root must remain inside the workspace")
    source = _require_mapping(manifest.get("source"), "source")
    results = [_verify_source(source, upstream, workspace)]
    assets = manifest.get("assets")
    if not isinstance(assets, list) or not assets:
        raise AssetConfigurationError("assets must be a non-empty list")
    seen_ids: set[str] = set()
    for index, value in enumerate(assets):
        asset = _require_mapping(value, f"assets[{index}]")
        asset_id = _require_string(asset.get("id"), f"assets[{index}].id")
        if asset_id in seen_ids:
            raise AssetConfigurationError(f"duplicate asset id: {asset_id}")
        seen_ids.add(asset_id)
        results.append(_verify_asset(asset, upstream, workspace))
    return results


def _print_human(results: Sequence[AssetResult]) -> None:
    for result in results:
        status = "READY" if result.ready else "NOT_READY"
        revision = result.revision or "unknown"
        print(
            f"{status:9} {result.asset_id}: {result.name} "
            f"(verification={result.verification}, revision={revision}, "
            f"files={result.files_checked}, bytes={result.bytes_checked})"
        )
        if result.provenance is not None:
            print(f"  source: {result.provenance}")
        for error in result.errors:
            print(f"  - {error}")
    ready_count = sum(result.ready for result in results)
    print(f"SUMMARY ready={ready_count} not_ready={len(results) - ready_count}")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument(
        "--upstream-root",
        type=Path,
        help="Workspace-local MiniCPM-Robot root; overrides ROBOTTRACK_UPSTREAM_ROOT",
    )
    parser.add_argument("--json", action="store_true", help="Emit one JSON document")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        manifest = load_manifest(args.manifest.resolve())
        upstream_root = resolve_upstream_root(
            WORKSPACE_ROOT, manifest, args.upstream_root, os.environ
        )
        results = verify(manifest, WORKSPACE_ROOT, upstream_root)
    except AssetConfigurationError as error:
        if args.json:
            print(json.dumps({"error": str(error)}, sort_keys=True))
        else:
            print(f"RobotTrack asset configuration error: {error}", file=sys.stderr)
        return 2

    if args.json:
        print(
            json.dumps(
                {
                    "ready": all(result.ready for result in results),
                    "upstream_root": str(upstream_root),
                    "results": [asdict(result) for result in results],
                },
                indent=2,
                sort_keys=True,
            )
        )
    else:
        _print_human(results)
    return 0 if all(result.ready for result in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
