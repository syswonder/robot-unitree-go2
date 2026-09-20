import hashlib
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

import yaml


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "verify_robottrack_assets.py"
SPEC = importlib.util.spec_from_file_location("verify_robottrack_assets", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
module = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = module
SPEC.loader.exec_module(module)


class RobotTrackAssetVerifierTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory(dir=ROOT)
        self.workspace = Path(self.temporary.name)
        self.upstream = self.workspace / "upstream" / "MiniCPM-Robot"
        self.upstream.mkdir(parents=True)

    def tearDown(self) -> None:
        self.temporary.cleanup()

    @staticmethod
    def _write(path: Path, content: bytes) -> dict:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(content)
        return {
            "path": path.name,
            "size": len(content),
            "sha256": hashlib.sha256(content).hexdigest(),
        }

    def _manifest(self, *, omit_dino: bool = False) -> dict:
        source_file = self.upstream / "MiniCPM-RobotTrack" / "go2_runtime.py"
        source_file.parent.mkdir(parents=True, exist_ok=True)
        source_file.write_text("# source\n", encoding="utf-8")

        assets = []
        for asset_id, revision in (
            ("checkpoint", "2" * 40),
            ("siglip", "3" * 40),
            ("dino", "4" * 40),
        ):
            root = self.upstream / "models" / asset_id
            if not (omit_dino and asset_id == "dino"):
                marker = root / ".cache" / "revision.metadata"
                marker.parent.mkdir(parents=True, exist_ok=True)
                marker.write_text(f"{revision}\nignored\n", encoding="utf-8")
                file_entry = self._write(root / "model.bin", asset_id.encode())
            else:
                file_entry = {
                    "path": "model.bin",
                    "size": len(asset_id),
                    "sha256": hashlib.sha256(asset_id.encode()).hexdigest(),
                }
            assets.append(
                {
                    "id": asset_id,
                    "name": asset_id,
                    "root": f"models/{asset_id}",
                    "revision": revision,
                    "revision_marker": ".cache/revision.metadata",
                    "files": [file_entry],
                }
            )
        return {
            "schema_version": 1,
            "upstream_root": {
                "environment": "ROBOTTRACK_UPSTREAM_ROOT",
                "default": "upstream/MiniCPM-Robot",
            },
            "source": {
                "name": "source",
                "root": ".",
                "git_revision": "1" * 40,
                "required_files": ["MiniCPM-RobotTrack/go2_runtime.py"],
            },
            "assets": assets,
        }

    def test_all_assets_ready_with_pinned_revisions_hashes_and_sizes(self) -> None:
        manifest = self._manifest()
        with mock.patch.object(module, "_git_head", return_value="1" * 40):
            results = module.verify(manifest, self.workspace, self.upstream)
        self.assertEqual(
            [result.asset_id for result in results],
            ["source", "checkpoint", "siglip", "dino"],
        )
        self.assertTrue(all(result.ready for result in results))

    def test_missing_dino_is_one_not_ready_item_and_other_assets_still_report(self) -> None:
        manifest = self._manifest(omit_dino=True)
        with mock.patch.object(module, "_git_head", return_value="1" * 40):
            results = module.verify(manifest, self.workspace, self.upstream)
        readiness = {result.asset_id: result.ready for result in results}
        self.assertEqual(
            readiness,
            {"source": True, "checkpoint": True, "siglip": True, "dino": False},
        )
        dino = next(result for result in results if result.asset_id == "dino")
        self.assertTrue(any("missing revision marker" in error for error in dino.errors))
        self.assertTrue(any("missing model.bin" in error for error in dino.errors))

    def test_hash_and_size_mismatches_are_both_reported(self) -> None:
        manifest = self._manifest()
        checkpoint = manifest["assets"][0]
        checkpoint["files"][0]["size"] += 1
        checkpoint["files"][0]["sha256"] = "0" * 64
        with mock.patch.object(module, "_git_head", return_value="1" * 40):
            results = module.verify(manifest, self.workspace, self.upstream)
        errors = next(
            result.errors for result in results if result.asset_id == "checkpoint"
        )
        self.assertTrue(any("size mismatch" in error for error in errors))
        self.assertTrue(any("sha256 mismatch" in error for error in errors))

    def test_content_hash_mode_accepts_exact_mirror_without_revision_marker(self) -> None:
        manifest = self._manifest()
        dino = manifest["assets"][2]
        marker = self.upstream / "models" / "dino" / dino.pop("revision_marker")
        marker.unlink()
        dino["verification"] = "sha256_files"
        dino["provenance"] = {
            "provider": "modelscope",
            "repository": "facebook/dino",
            "resolved_revision": "5" * 40,
        }
        with mock.patch.object(module, "_git_head", return_value="1" * 40):
            results = module.verify(manifest, self.workspace, self.upstream)
        result = next(item for item in results if item.asset_id == "dino")
        self.assertTrue(result.ready)
        self.assertIsNone(result.revision)
        self.assertEqual(result.verification, "sha256_files")
        self.assertEqual(
            result.provenance,
            f"modelscope:facebook/dino@{'5' * 40}",
        )

    def test_content_hash_mode_requires_every_declared_hash(self) -> None:
        manifest = self._manifest()
        dino = manifest["assets"][2]
        dino["verification"] = "sha256_files"
        dino["provenance"] = {
            "provider": "modelscope",
            "repository": "facebook/dino",
            "resolved_revision": "5" * 40,
        }
        dino["files"][0].pop("sha256")
        with mock.patch.object(module, "_git_head", return_value="1" * 40):
            with self.assertRaisesRegex(
                module.AssetConfigurationError,
                "sha256_files verification requires files\\[0\\].sha256",
            ):
                module.verify(manifest, self.workspace, self.upstream)

    def test_cli_root_precedes_environment_and_root_must_stay_in_workspace(self) -> None:
        manifest = self._manifest()
        environment_root = self.workspace / "environment"
        cli_root = self.workspace / "cli"
        self.assertEqual(
            module.resolve_upstream_root(
                self.workspace,
                manifest,
                cli_root,
                {"ROBOTTRACK_UPSTREAM_ROOT": str(environment_root)},
            ),
            cli_root,
        )
        with self.assertRaisesRegex(
            module.AssetConfigurationError, "must remain inside workspace"
        ):
            module.resolve_upstream_root(
                self.workspace, manifest, Path("/tmp/outside"), {}
            )

    def test_repository_manifest_pins_official_revisions_and_weight_hashes(self) -> None:
        manifest = yaml.safe_load(
            (ROOT / "config" / "robottrack_assets.yaml").read_text(encoding="utf-8")
        )
        self.assertEqual(
            manifest["source"]["git_revision"],
            "f7dc15a016b1b2a7e48a559072521e347658de11",
        )
        assets = {asset["id"]: asset for asset in manifest["assets"]}
        self.assertEqual(
            assets["minicpm_robottrack_checkpoint"]["revision"],
            "80d32d0b091c3b340a1e4a6c78d441e8af19590e",
        )
        self.assertEqual(
            assets["siglip_so400m"]["revision"],
            "9fdffc58afc957d1a03a25b10dba0329ab15c2a3",
        )
        self.assertEqual(
            assets["dinov3_vits16"]["revision"],
            "114c1379950215c8b35dfcd4e90a5c251dde0d32",
        )
        self.assertEqual(
            assets["dinov3_vits16"]["verification"],
            "sha256_files",
        )
        self.assertEqual(
            assets["dinov3_vits16"]["provenance"],
            {
                "provider": "modelscope",
                "repository": "facebook/dinov3-vits16-pretrain-lvd1689m",
                "resolved_revision": "2e601320d0545509ab03374e2f8707f303e1de7a",
            },
        )
        self.assertTrue(
            all("sha256" in item for item in assets["dinov3_vits16"]["files"])
        )
        checkpoint_model = assets["minicpm_robottrack_checkpoint"]["files"][0]
        self.assertEqual(
            checkpoint_model["sha256"],
            "27cc7e58fd0797ea2dfa03b250847a7142eeedc7babbadd7bc63401d051ec7f3",
        )
        self.assertEqual(checkpoint_model["size"], 931428452)

    def test_verifier_has_no_network_or_token_access(self) -> None:
        source = SCRIPT.read_text(encoding="utf-8")
        for forbidden in (
            "urllib",
            "requests",
            "huggingface_hub",
            "HF_TOKEN",
            "create_publisher",
            "ros2",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, source)


if __name__ == "__main__":
    unittest.main()
