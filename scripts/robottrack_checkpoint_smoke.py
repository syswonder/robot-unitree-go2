#!/usr/bin/env python3
"""Offline MiniCPM-RobotTrack checkpoint smoke test with synthetic fused tokens."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import statistics
import sys
import time
from typing import Any, Sequence


REPO_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = REPO_ROOT.parents[1]
DEFAULT_UPSTREAM_ROOT = WORKSPACE_ROOT / "upstream" / "MiniCPM-Robot"
DEFAULT_DEPENDENCY_ROOT = WORKSPACE_ROOT / ".tools" / "robottrack-python"
ASSET_MANIFEST = REPO_ROOT / "config" / "robottrack_assets.yaml"
DEFAULT_SPEECH_PYTHONS = (
    WORKSPACE_ROOT
    / "upstream"
    / "robonix"
    / "services"
    / "speech"
    / "rbnx-build"
    / "venv"
    / "bin"
    / "python",
    WORKSPACE_ROOT
    / "upstream"
    / "robonix-go2-build"
    / "services"
    / "speech"
    / "rbnx-build"
    / "venv"
    / "bin"
    / "python",
)
CHECKPOINT_RELATIVE = Path(
    "MiniCPM-RobotTrack/minicpm_robot_track/checkpoints/MiniCPM-RobotTrack"
)


def _upstream_root(environ: dict[str, str] | os._Environ[str]) -> Path:
    configured = environ.get("ROBOTTRACK_UPSTREAM_ROOT", "").strip()
    root = Path(configured) if configured else DEFAULT_UPSTREAM_ROOT
    if not root.is_absolute():
        root = WORKSPACE_ROOT / root
    return root.resolve()


def _checkpoint_default(environ: dict[str, str] | os._Environ[str]) -> Path:
    return _upstream_root(environ) / CHECKPOINT_RELATIVE


def _parser(environ: dict[str, str] | os._Environ[str]) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--checkpoint",
        type=Path,
        default=_checkpoint_default(environ),
        help="Complete local Hugging Face MiniCPM-RobotTrack checkpoint directory",
    )
    parser.add_argument(
        "--device",
        default="auto",
        help="auto, cpu, cuda, or an explicit CUDA device such as cuda:0",
    )
    parser.add_argument(
        "--prompt",
        default="Follow the person ahead",
        help="Local tokenizer input used by the synthetic policy forward",
    )
    parser.add_argument("--warmup", type=int, default=1)
    parser.add_argument("--iterations", type=int, default=5)
    parser.add_argument("--seed", type=int, default=20260805)
    return parser


def _configure_offline_environment() -> None:
    cache_root = WORKSPACE_ROOT / ".tools" / "robottrack-hf-cache"
    os.environ["TRANSFORMERS_OFFLINE"] = "1"
    os.environ["HF_HUB_OFFLINE"] = "1"
    os.environ["HF_HUB_DISABLE_TELEMETRY"] = "1"
    os.environ["TOKENIZERS_PARALLELISM"] = "false"
    os.environ.setdefault("HF_HOME", str(cache_root))
    os.environ.setdefault("HF_MODULES_CACHE", str(cache_root / "modules"))


def _default_python() -> Path | None:
    for candidate in DEFAULT_SPEECH_PYTHONS:
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return candidate
    return None


def _prepend_pythonpath(environ: dict[str, str]) -> None:
    configured = environ.get("ROBOTTRACK_PYTHON_DEPS", "").strip()
    dependency_root = Path(configured) if configured else DEFAULT_DEPENDENCY_ROOT
    existing = environ.get("PYTHONPATH", "")
    entries = [str(dependency_root)]
    if existing:
        entries.append(existing)
    environ["PYTHONPATH"] = os.pathsep.join(entries)


def _load_dependencies() -> tuple[Any, Any, Any, Any, Any]:
    try:
        import safetensors
        import torch
        import transformers
        import yaml
        from transformers import AutoModel, AutoTokenizer
    except ModuleNotFoundError as error:
        if os.environ.get("ROBOTTRACK_SMOKE_BOOTSTRAPPED") != "1":
            configured = os.environ.get("PYTHON", "").strip()
            candidate = Path(configured) if configured else _default_python()
            if candidate is not None:
                environment = os.environ.copy()
                environment["ROBOTTRACK_SMOKE_BOOTSTRAPPED"] = "1"
                _prepend_pythonpath(environment)
                os.execvpe(
                    str(candidate),
                    [str(candidate), str(Path(__file__).resolve()), *sys.argv[1:]],
                    environment,
                )
        raise RuntimeError(
            "RobotTrack Python dependencies are unavailable; set PYTHON to the "
            "existing speech torch interpreter and PYTHONPATH to include "
            f"{DEFAULT_DEPENDENCY_ROOT}: {error}"
        ) from error
    return torch, transformers, safetensors, yaml, (AutoModel, AutoTokenizer)


def _validate_args(args: argparse.Namespace) -> None:
    if args.warmup < 0:
        raise ValueError("--warmup must be nonnegative")
    if args.iterations <= 0:
        raise ValueError("--iterations must be positive")
    if not args.prompt.strip():
        raise ValueError("--prompt must not be empty")


def _resolve_device(torch: Any, requested: str) -> Any:
    value = requested.strip().lower()
    if value == "auto":
        value = "cuda:0" if torch.cuda.is_available() else "cpu"
    elif value == "cuda":
        value = "cuda:0"
    if value != "cpu" and not value.startswith("cuda:"):
        raise ValueError("--device must be auto, cpu, cuda, or cuda:N")
    if value.startswith("cuda:") and not torch.cuda.is_available():
        raise RuntimeError(f"CUDA was requested ({value}) but is not available")
    device = torch.device(value)
    if device.type == "cuda":
        torch.cuda.set_device(device)
    return device


def _synchronize(torch: Any, device: Any) -> None:
    if device.type == "cuda":
        torch.cuda.synchronize(device)


def _byte_count(parameters: Any) -> int:
    return sum(parameter.numel() * parameter.element_size() for parameter in parameters)


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _checkpoint_provenance(yaml: Any, checkpoint: Path) -> dict[str, Any]:
    manifest = yaml.safe_load(ASSET_MANIFEST.read_text(encoding="utf-8"))
    asset = next(
        entry
        for entry in manifest["assets"]
        if entry["id"] == "minicpm_robottrack_checkpoint"
    )
    weight = next(
        entry for entry in asset["files"] if entry["path"] == "model.safetensors"
    )
    marker = checkpoint / asset["revision_marker"]
    marker_revision = None
    if marker.is_file():
        lines = marker.read_text(encoding="utf-8").splitlines()
        marker_revision = lines[0].strip() if lines else None
    hash_started = time.perf_counter()
    actual_sha256 = _sha256_file(checkpoint / "model.safetensors")
    hash_elapsed_ms = (time.perf_counter() - hash_started) * 1000.0
    expected_revision = str(asset["revision"])
    expected_sha256 = str(weight["sha256"])
    return {
        "manifest": str(ASSET_MANIFEST),
        "asset_id": str(asset["id"]),
        "revision": marker_revision,
        "expected_revision": expected_revision,
        "revision_matches": marker_revision == expected_revision,
        "revision_source": (
            f"{ASSET_MANIFEST}#assets[id=minicpm_robottrack_checkpoint].revision"
        ),
        "model_sha256": actual_sha256,
        "expected_model_sha256": expected_sha256,
        "sha256_matches": actual_sha256 == expected_sha256,
        "sha256_source": (
            f"{ASSET_MANIFEST}#assets[id=minicpm_robottrack_checkpoint]"
            ".files[path=model.safetensors].sha256"
        ),
        "hash_elapsed_ms": hash_elapsed_ms,
    }


def _vram_snapshot(torch: Any, device: Any) -> dict[str, Any]:
    if device.type != "cuda":
        return {
            "available": False,
            "allocated_bytes": None,
            "reserved_bytes": None,
            "peak_allocated_bytes": None,
        }
    return {
        "available": True,
        "allocated_bytes": int(torch.cuda.memory_allocated(device)),
        "reserved_bytes": int(torch.cuda.memory_reserved(device)),
        "peak_allocated_bytes": int(torch.cuda.max_memory_allocated(device)),
    }


def run_smoke(args: argparse.Namespace) -> dict[str, Any]:
    _validate_args(args)
    checkpoint = args.checkpoint.expanduser().resolve()
    required = ("config.json", "model.safetensors", "tokenizer_config.json")
    missing = [name for name in required if not (checkpoint / name).is_file()]
    if missing:
        raise FileNotFoundError(
            f"incomplete local checkpoint {checkpoint}; missing: {', '.join(missing)}"
        )

    _configure_offline_environment()
    torch, transformers, safetensors, yaml, auto_classes = _load_dependencies()
    AutoModel, AutoTokenizer = auto_classes
    checkpoint_provenance = _checkpoint_provenance(yaml, checkpoint)
    device = _resolve_device(torch, args.device)
    torch.manual_seed(args.seed)
    if device.type == "cuda":
        torch.cuda.manual_seed_all(args.seed)
        torch.cuda.empty_cache()
        torch.cuda.reset_peak_memory_stats(device)
    vram_before_load = _vram_snapshot(torch, device)

    load_started = time.perf_counter()
    tokenizer = AutoTokenizer.from_pretrained(
        checkpoint,
        local_files_only=True,
    )
    model = AutoModel.from_pretrained(
        checkpoint,
        trust_remote_code=True,
        local_files_only=True,
    )
    model = model.to(device).eval()
    _synchronize(torch, device)
    load_ms = (time.perf_counter() - load_started) * 1000.0
    vram_after_load = _vram_snapshot(torch, device)

    config = model.config
    history_frames = int(config.history_frames)
    coarse_per_frame = int(config.coarse_tokens_per_frame)
    fine_count = int(config.fine_tokens_current_frame)
    feature_dim = int(config.vision_feature_dim)
    expected_trajectory = (1, int(config.num_waypoints), int(config.action_dim))

    encoded = tokenizer(
        [args.prompt],
        return_tensors="pt",
        padding=True,
        truncation=True,
        max_length=int(config.max_text_tokens),
    )
    input_ids = encoded["input_ids"].to(device)
    attention_mask = encoded["attention_mask"].to(device)
    coarse_tokens = torch.randn(
        1,
        history_frames * coarse_per_frame,
        feature_dim,
        dtype=torch.float32,
        device=device,
    )
    fine_tokens = torch.randn(
        1,
        fine_count,
        feature_dim,
        dtype=torch.float32,
        device=device,
    )
    coarse_time_indices = (
        torch.arange(history_frames, dtype=torch.long, device=device)
        .repeat_interleave(coarse_per_frame)
        .unsqueeze(0)
    )
    fine_time_indices = torch.full(
        (1, fine_count),
        history_frames,
        dtype=torch.long,
        device=device,
    )

    def forward() -> Any:
        output = model(
            input_ids=input_ids,
            attention_mask=attention_mask,
            coarse_tokens=coarse_tokens,
            coarse_time_indices=coarse_time_indices,
            fine_tokens=fine_tokens,
            fine_time_indices=fine_time_indices,
        )
        trajectory = getattr(output, "trajectories", None)
        if trajectory is None:
            raise RuntimeError("checkpoint output does not contain trajectories")
        return trajectory

    with torch.inference_mode():
        trajectory = None
        for _ in range(args.warmup):
            trajectory = forward()
        _synchronize(torch, device)
        if device.type == "cuda":
            torch.cuda.reset_peak_memory_stats(device)

        samples_ms: list[float] = []
        for _ in range(args.iterations):
            started = time.perf_counter()
            trajectory = forward()
            _synchronize(torch, device)
            samples_ms.append((time.perf_counter() - started) * 1000.0)

    if trajectory is None:
        raise RuntimeError("no policy forward was executed")
    actual_shape = tuple(int(value) for value in trajectory.shape)
    finite_mask = torch.isfinite(trajectory)
    finite_count = int(finite_mask.sum().item())
    element_count = int(trajectory.numel())
    all_finite = finite_count == element_count
    minimum = float(trajectory.detach().float().min().item())
    maximum = float(trajectory.detach().float().max().item())
    shape_matches = actual_shape == expected_trajectory
    vram_after_inference = _vram_snapshot(torch, device)

    result = {
        "ok": bool(
            shape_matches
            and all_finite
            and checkpoint_provenance["revision_matches"]
            and checkpoint_provenance["sha256_matches"]
        ),
        "offline": True,
        "checkpoint_provenance": checkpoint_provenance,
        "versions": {
            "python": platform.python_version(),
            "torch": str(torch.__version__),
            "transformers": str(transformers.__version__),
            "safetensors": str(safetensors.__version__),
            "pyyaml": str(yaml.__version__),
            "cuda_runtime": str(torch.version.cuda) if torch.version.cuda else None,
            "cudnn": (
                int(torch.backends.cudnn.version())
                if torch.backends.cudnn.version() is not None
                else None
            ),
        },
        "load": {
            "checkpoint": str(checkpoint),
            "local_files_only": True,
            "device_requested": args.device,
            "device_resolved": str(device),
            "device_name": (
                torch.cuda.get_device_name(device) if device.type == "cuda" else "CPU"
            ),
            "elapsed_ms": load_ms,
            "parameter_count": int(sum(p.numel() for p in model.parameters())),
            "parameter_bytes": int(_byte_count(model.parameters())),
        },
        "shape": {
            "input_ids": list(input_ids.shape),
            "coarse_tokens": list(coarse_tokens.shape),
            "coarse_time_indices": list(coarse_time_indices.shape),
            "fine_tokens": list(fine_tokens.shape),
            "fine_time_indices": list(fine_time_indices.shape),
            "trajectory": list(actual_shape),
            "expected_trajectory": list(expected_trajectory),
            "matches_expected": shape_matches,
        },
        "finite": {
            "all": all_finite,
            "finite_count": finite_count,
            "element_count": element_count,
            "minimum": minimum if math.isfinite(minimum) else None,
            "maximum": maximum if math.isfinite(maximum) else None,
        },
        "warm_latency_ms": {
            "warmup_iterations": args.warmup,
            "measured_iterations": args.iterations,
            "samples": samples_ms,
            "mean": statistics.fmean(samples_ms),
            "median": statistics.median(samples_ms),
            "minimum": min(samples_ms),
            "maximum": max(samples_ms),
        },
        "vram": {
            "before_load": vram_before_load,
            "after_load": vram_after_load,
            "after_inference": vram_after_inference,
            "load_allocated_delta_bytes": (
                vram_after_load["allocated_bytes"]
                - vram_before_load["allocated_bytes"]
                if device.type == "cuda"
                else None
            ),
        },
    }
    return result


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser(os.environ).parse_args(argv)
    try:
        result = run_smoke(args)
    except Exception as error:
        result = {
            "ok": False,
            "offline": True,
            "versions": {"python": platform.python_version()},
            "error": {
                "type": type(error).__name__,
                "message": str(error),
            },
        }
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
