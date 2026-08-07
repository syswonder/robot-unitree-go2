from __future__ import annotations

from pathlib import Path
import re
import stat
import subprocess
import unittest


ROOT = Path(__file__).resolve().parents[1]
BASE_LAUNCHER = ROOT / "scripts" / "start_workstation_staged_nav2_corrected.sh"
PERSISTENT_LAUNCHER = (
    ROOT / "scripts" / "start_workstation_persistent_voice_nav2.sh"
)
ROBOTTRACK_LAUNCHER = (
    ROOT / "scripts" / "start_workstation_robottrack_follow.sh"
)


class RobotTrackLauncherTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.base = BASE_LAUNCHER.read_text(encoding="utf-8")
        cls.persistent = PERSISTENT_LAUNCHER.read_text(encoding="utf-8")
        cls.wrapper = ROBOTTRACK_LAUNCHER.read_text(encoding="utf-8")

    def test_default_path_keeps_the_existing_renderers(self) -> None:
        self.assertIn(
            'ROBOTTRACK_MODE="${GO2_ROBOTTRACK_MODE:-false}"', self.base
        )
        staged = self.base.index(
            'MANIFEST_RENDERER="$ROOT/deploy/time-sync/'
            'render_workstation_staged_nav2_manifest.py"'
        )
        persistent = self.base.index(
            'MANIFEST_RENDERER="$ROOT/deploy/time-sync/'
            'render_workstation_persistent_nav2_manifest.py"'
        )
        robottrack = self.base.index(
            'MANIFEST_RENDERER="$ROOT/deploy/time-sync/'
            'render_workstation_robottrack_manifest.py"'
        )
        render = self.base.index('"$PYTHON" "$MANIFEST_RENDERER"')
        self.assertLess(staged, persistent)
        self.assertLess(persistent, robottrack)
        self.assertLess(robottrack, render)
        self.assertRegex(
            self.base,
            re.compile(
                r'case "\$ROBOTTRACK_MODE" in\n'
                r"\s+true\|false\) ;;\n"
                r'.*GO2_ROBOTTRACK_MODE must be true or false',
                re.DOTALL,
            ),
        )

    def test_robottrack_is_only_selectable_in_persistent_standard_mode(self) -> None:
        self.assertIn(
            '[[ "$ROBOTTRACK_MODE" == false \\\n'
            '  || ( "$PERSISTENT_MODE" == true '
            '&& "$STANDARD_MODE" == true ) ]]',
            self.base,
        )
        self.assertIn(
            'if [[ "$ROBOTTRACK_MODE" == true ]]; then', self.base
        )
        self.assertIn(
            '--server-url "${ROBOTTRACK_SERVER_URL:-'
            'http://127.0.0.1:5801/eval_dual}"',
            self.base,
        )
        self.assertIn(
            '--instruction "${ROBOTTRACK_INSTRUCTION:-'
            'Follow the person ahead}"',
            self.base,
        )
        self.assertIn('"${MANIFEST_RENDERER_ARGS[@]}"', self.base)

    def test_extended_graph_ready_timeout_is_robottrack_only(self) -> None:
        self.assertIn(
            'GRAPH_READY_TIMEOUT_SECONDS=60\n'
            'if [[ "$ROBOTTRACK_MODE" == true ]]; then\n'
            '  GRAPH_READY_TIMEOUT_SECONDS=90\n'
            'fi',
            self.base,
        )
        self.assertNotIn(
            'if [[ "$PERSISTENT_MODE" == true ]]; then\n'
            '  GRAPH_READY_TIMEOUT_SECONDS=90',
            self.base,
        )

    def test_persistent_launcher_restores_explicit_selection_across_env_load(self) -> None:
        capture = self.persistent.index(
            'INHERITED_ROBOTTRACK_MODE="${GO2_ROBOTTRACK_MODE:-}"'
        )
        env_load = self.persistent.index('source "$ROOT/.env"')
        restore = self.persistent.index(
            'export GO2_ROBOTTRACK_MODE="$INHERITED_ROBOTTRACK_MODE"'
        )
        self.assertLess(capture, env_load)
        self.assertLess(env_load, restore)
        self.assertNotIn(
            'if [[ -n "$INHERITED_ROBOTTRACK_MODE" ]]', self.persistent
        )
        self.assertIn(
            'export ROBOTTRACK_SERVER_URL="$INHERITED_ROBOTTRACK_SERVER_URL"',
            self.persistent,
        )

    def test_env_file_cannot_enable_robottrack_without_explicit_selection(self) -> None:
        # Unset input is captured as an empty string before .env is loaded;
        # the unconditional export after the load overwrites any stale true.
        self.assertIn(
            'INHERITED_ROBOTTRACK_MODE="${GO2_ROBOTTRACK_MODE:-}"',
            self.persistent,
        )
        self.assertIn(
            'export GO2_ROBOTTRACK_MODE="$INHERITED_ROBOTTRACK_MODE"',
            self.persistent,
        )
        default_resolution = self.base.index(
            'ROBOTTRACK_MODE="${GO2_ROBOTTRACK_MODE:-false}"'
        )
        robottrack_branch = self.base.index(
            'if [[ "$ROBOTTRACK_MODE" == true ]]; then'
        )
        self.assertLess(default_resolution, robottrack_branch)
        self.assertIn(
            'export ROBOTTRACK_INSTRUCTION="$INHERITED_ROBOTTRACK_INSTRUCTION"',
            self.persistent,
        )

    def test_follow_wrapper_is_thin_and_has_no_direct_hardware_actions(self) -> None:
        self.assertTrue(
            ROBOTTRACK_LAUNCHER.stat().st_mode & stat.S_IXUSR
        )
        self.assertIn("GO2_ROBOTTRACK_MODE=true", self.wrapper)
        self.assertIn(
            'ROBOTTRACK_SERVER_URL="${ROBOTTRACK_SERVER_URL:-'
            'http://127.0.0.1:5801/eval_dual}"',
            self.wrapper,
        )
        self.assertIn(
            'ROBOTTRACK_INSTRUCTION="${ROBOTTRACK_INSTRUCTION:-'
            'Follow the person ahead}"',
            self.wrapper,
        )
        self.assertEqual(
            self.wrapper.count('exec bash "$PERSISTENT_LAUNCHER"'), 1
        )
        for forbidden in (
            "ros2 ",
            "docker ",
            "nmcli ",
            "ip link",
            "ip addr",
            "curl ",
            "wget ",
            "ssh ",
            "rbnx ",
        ):
            with self.subTest(forbidden=forbidden):
                self.assertNotIn(forbidden, self.wrapper)
        for forbidden_state in ("armed", "disarmed", "permit", "approval"):
            with self.subTest(forbidden_state=forbidden_state):
                self.assertNotIn(forbidden_state, self.wrapper.lower())

    def test_follow_reuses_generation1_classicwalk_and_accepts_100_or_2010(self) -> None:
        self.assertIn(
            'exec bash "$PERSISTENT_LAUNCHER"', self.wrapper
        )
        self.assertIn("GO2_PERSISTENT_NAV2_MODE=true", self.persistent)
        self.assertIn("PASSIVE_SOURCE_MARKERS=100,1002,2010", self.base)
        self.assertIn("CLASSIC_MOTION_STATE_MARKERS=100,2010", self.base)
        self.assertIn(
            'case ",$CLASSIC_MOTION_STATE_MARKERS," in\n'
            '  *,"$observed_marker",*) ;;',
            self.base,
        )
        self.assertNotIn('"$observed_marker" == 100', self.base)
        self.assertNotIn('"$observed_marker" == 2010', self.base)

    def test_shell_syntax_is_valid_without_executing_launchers(self) -> None:
        completed = subprocess.run(
            [
                "bash",
                "-n",
                str(BASE_LAUNCHER),
                str(PERSISTENT_LAUNCHER),
                str(ROBOTTRACK_LAUNCHER),
            ],
            check=False,
            text=True,
            capture_output=True,
        )
        self.assertEqual(completed.returncode, 0, completed.stderr)


if __name__ == "__main__":
    unittest.main()
