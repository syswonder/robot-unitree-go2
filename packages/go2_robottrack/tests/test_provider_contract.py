from __future__ import annotations

import importlib
import sys
import threading
import types
import unittest
from unittest import mock


class Result:
    def __init__(self, kind: str, detail: str = "") -> None:
        self.kind = kind
        self.detail = detail


class FakePrimitive:
    def __init__(self, *, id: str, namespace: str) -> None:
        self.id = id
        self.namespace = namespace
        self.callbacks = {}

    def _decorator(self, name):
        def register(callback):
            self.callbacks[name] = callback
            return callback

        return register

    @property
    def on_init(self):
        return self._decorator("init")

    @property
    def on_activate(self):
        return self._decorator("activate")

    @property
    def on_deactivate(self):
        return self._decorator("deactivate")

    @property
    def on_shutdown(self):
        return self._decorator("shutdown")

    def run(self):
        raise AssertionError("provider.run must not be called by offline tests")


def load_provider_module():
    fake_api = types.ModuleType("robonix_api")
    fake_api.Primitive = FakePrimitive
    fake_api.Ok = lambda: Result("ok")
    fake_api.Err = lambda detail: Result("err", detail)
    fake_api.Deferred = lambda detail: Result("deferred", detail)
    sys.modules["robonix_api"] = fake_api
    sys.modules.pop("go2_robottrack.provider", None)
    return importlib.import_module("go2_robottrack.provider")


class ProviderContractTests(unittest.TestCase):
    def test_live_lifecycle_starts_and_stops_ros_thread(self) -> None:
        module = load_provider_module()
        self.assertEqual(module.provider.id, "go2_robottrack")
        self.assertEqual(module.provider.namespace, "robonix/primitive/follow")

        observed = {}

        def runner(config, stop_event, ready_event, errors):
            del errors
            observed.update(config)
            ready_event.set()
            stop_event.wait(2.0)

        module._ros_runner = runner
        init_result = module.initialize(
            {
                "mode": "live",
                "rgb_topic": "/go2/d435i/color/image_raw",
                "source_mux": {
                    "nav_input_topic": "/go2/robottrack/nav_cmd_vel_raw",
                    "robottrack_input_topic": "/go2/robottrack/cmd_vel_raw",
                    "output_topic": "/cmd_vel_nav",
                    "selected_source": "robottrack",
                },
                "camera_info_topic": "/go2/d435i/color/camera_info",
                "asset_manifest": "/opt/models/assets.json",
                "upstream_root": "/opt/MiniCPM-Robot",
            }
        )
        self.assertEqual(init_result.kind, "ok")
        self.assertEqual(module.activate().kind, "ok")
        self.assertEqual(observed["mode"], "live")
        self.assertEqual(
            observed["robottrack_raw_topic"], "/go2/robottrack/cmd_vel_raw"
        )
        self.assertEqual(module.deactivate().kind, "ok")

    def test_shutdown_stops_active_runtime(self) -> None:
        module = load_provider_module()
        stopped = threading.Event()

        def runner(config, stop_event, ready_event, errors):
            del config, errors
            ready_event.set()
            stop_event.wait(2.0)
            stopped.set()

        module._ros_runner = runner
        self.assertEqual(module.initialize({"mode": "live"}).kind, "ok")
        self.assertEqual(module.activate().kind, "ok")
        self.assertEqual(module.shutdown().kind, "ok")
        self.assertTrue(stopped.is_set())

    def test_stop_timeout_retains_ownership_and_blocks_second_runtime(self) -> None:
        module = load_provider_module()

        class StuckRuntime:
            def __init__(self) -> None:
                self.join_timeouts = []

            def join(self, timeout=None) -> None:
                self.join_timeouts.append(timeout)

            def is_alive(self) -> bool:
                return True

        runtime = StuckRuntime()
        stop_event = threading.Event()
        module._config = module.RuntimeConfig.from_mapping(
            {"mode": "live"}
        ).as_ros_parameters()
        module._runtime_thread = runtime
        module._runtime_stop = stop_event
        module._active = True

        stopped = module.deactivate()

        self.assertEqual(stopped.kind, "err")
        self.assertTrue(stop_event.is_set())
        self.assertEqual(runtime.join_timeouts, [4.0])
        self.assertIs(module._runtime_thread, runtime)
        self.assertIs(module._runtime_stop, stop_event)
        self.assertEqual(module.activate().kind, "deferred")

    def test_thread_start_failure_releases_unstarted_ownership(self) -> None:
        module = load_provider_module()

        class FailingThread:
            def __init__(self, *args, **kwargs) -> None:
                del args, kwargs

            def start(self) -> None:
                raise RuntimeError("synthetic thread exhaustion")

        self.assertEqual(module.initialize({"mode": "live"}).kind, "ok")
        with mock.patch.object(module.threading, "Thread", FailingThread):
            result = module.activate()
        self.assertEqual(result.kind, "err")
        self.assertIn("could not start", result.detail)
        self.assertIsNone(module._runtime_thread)
        self.assertIsNone(module._runtime_stop)

    def test_default_initialization_remains_dry_run(self) -> None:
        module = load_provider_module()
        self.assertEqual(module.initialize({}).kind, "ok")
        self.assertEqual(module._config["mode"], "dry-run")


if __name__ == "__main__":
    unittest.main()
