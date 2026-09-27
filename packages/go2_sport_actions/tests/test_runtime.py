# SPDX-License-Identifier: Apache-2.0
import struct
import threading
import time
import unittest

from go2_sport_actions.ipc import (
    ARM, DISARM, MOVE, PING, SPORT_ACTION, STOP, COMMAND, REPLY, MAGIC, VERSION,
    DaemonError, make_command, parse_reply,
)
from go2_sport_actions.runtime import ActionManager, PLANS, Run, Step
from go2_sport_actions.daemon_owner import ManagedDaemon, MOTION_ACK, PROFILE
from go2_sport_actions.voice import action_from_utterance, is_cancel_utterance


def checksum(data):
    value = 2166136261
    for byte in data:
        value = ((value ^ byte) * 16777619) & 0xFFFFFFFF
    return value


class IpcTests(unittest.TestCase):
    def test_wire_layout_and_action_id(self):
        self.assertEqual(COMMAND.size, 52)
        self.assertEqual(REPLY.size, 28)
        packet = make_command(SPORT_ACTION, 17, action_id=3, now_ns=1000)
        values = COMMAND.unpack(packet)
        self.assertEqual(values[:4], (MAGIC, VERSION, 52, 17))
        self.assertEqual(values[7], 3)
        self.assertEqual(COMMAND.unpack(make_command(
            SPORT_ACTION, 18, action_id=14, now_ns=1000))[7], 14)
        self.assertEqual(COMMAND.unpack(make_command(
            SPORT_ACTION, 19, action_id=17, now_ns=1000))[7], 17)
        self.assertEqual(values[-1], checksum(packet[:-4] + b"\0\0\0\0"))
        with self.assertRaises(ValueError):
            make_command(PING, 1, action_id=3)
        with self.assertRaises(ValueError):
            make_command(SPORT_ACTION, 1, action_id=255)
        for removed in (7, 8, 9, 10, 11, 12, 13, 16):
            with self.assertRaises(ValueError):
                make_command(SPORT_ACTION, 1, action_id=removed)

    def test_reply_validation(self):
        data = REPLY.pack(MAGIC, VERSION, 28, 9, 0, 1, 0, 0, 0)
        data = data[:-4] + struct.pack("<I", checksum(data))
        parsed = parse_reply(data, 9)
        self.assertTrue(parsed.armed)
        with self.assertRaises(DaemonError):
            parse_reply(data, 10)
        with self.assertRaises(DaemonError):
            parse_reply(data[:-1] + b"x", 9)


class FakeDaemon:
    calls = []

    def __init__(self, _path):
        pass

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        pass

    def call(self, operation, **kwargs):
        self.calls.append((operation, kwargs))
        return object()


class FailingActionDaemon(FakeDaemon):
    def call(self, operation, **kwargs):
        super().call(operation, **kwargs)
        if operation == SPORT_ACTION:
            raise DaemonError("SDK rejected action")
        return object()


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        FakeDaemon.calls = []

    def finished(self, manager, run_id):
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            state = manager.status(run_id)["state"]
            if state in ("SDK_ACCEPTED_COMPLETION_UNVERIFIED", "FAILED", "CANCELLED"):
                return manager.status(run_id)
            time.sleep(0.005)
        self.fail("worker did not finish")

    def test_unconfigured_or_unknown_action_never_moves(self):
        manager = ActionManager()
        self.assertFalse(manager.execute("dance_1")["accepted"])
        self.assertFalse(manager.execute("backflip")["accepted"])
        self.assertFalse(FakeDaemon.calls)

    def test_dance_uses_one_daemon_session_and_cleans_up(self):
        self.assertEqual(PLANS["dance_1"][0].duration_s, 45.0)
        self.assertEqual(PLANS["dance_2"][0].duration_s, 55.0)
        self.assertEqual(PLANS["new_year_greeting"][0].duration_s, 30.0)
        self.assertEqual([step.duration_s for step in PLANS["crouch"]], [8.0, 8.0])
        self.assertEqual([step.duration_s for step in PLANS["handstand"]], [8.0, 4.0])
        self.assertEqual([step.duration_s for step in PLANS["handstand_walk"]],
                         [8.0, 5.0, 0.0, 4.0])
        manager = ActionManager("/test/socket", client_factory=FakeDaemon)
        manager._hold = lambda _daemon, _run, _step: None
        result = manager.execute("dance_1", "voice-123")
        self.assertTrue(result["accepted"])
        self.assertEqual(manager.execute("dance_1", "voice-123")["run_id"], result["run_id"])
        status = self.finished(manager, result["run_id"])
        self.assertEqual(status["state"], "SDK_ACCEPTED_COMPLETION_UNVERIFIED")
        self.assertFalse(status["physical_completion_verified"])
        self.assertTrue(status["stop_acknowledged"])
        operations = [call[0] for call in FakeDaemon.calls]
        self.assertEqual(operations, [ARM, SPORT_ACTION, STOP, DISARM])
        self.assertEqual(FakeDaemon.calls[1][1]["action_id"], 3)

    def test_pose_resets_are_explicit(self):
        for name, expected_ids in (("handstand", [5, 6]),
                                   ("crouch", [14, 15])):
            FakeDaemon.calls = []
            manager = ActionManager("/test/socket", client_factory=FakeDaemon)
            manager._hold = lambda _daemon, _run, _step: None
            result = manager.execute(name)
            self.finished(manager, result["run_id"])
            actual = [args["action_id"] for op, args in FakeDaemon.calls
                      if op == SPORT_ACTION]
            self.assertEqual(actual, expected_ids)

    def test_walk_commands_are_refreshed_not_sent_once(self):
        manager = ActionManager("/test/socket", client_factory=FakeDaemon)
        daemon = FakeDaemon("/test/socket")
        manager._hold(daemon, Run(name="handstand_walk", request_id=""),
                      Step(MOVE, 0.18, vx=0.06))
        self.assertGreaterEqual(sum(op == MOVE for op, _ in daemon.calls), 2)
        self.assertTrue(all(args.get("vx") == 0.06 for _, args in daemon.calls))

    def test_handstand_walk_stops_before_exiting_pose(self):
        manager = ActionManager("/test/socket", client_factory=FakeDaemon)
        manager._hold = lambda _daemon, _run, _step: None
        run_id = manager.execute("handstand_walk")["run_id"]
        self.assertTrue(self.finished(manager, run_id)["stop_acknowledged"])
        self.assertEqual([op for op, _ in FakeDaemon.calls],
                         [ARM, SPORT_ACTION, MOVE, STOP, SPORT_ACTION, STOP, DISARM])
        self.assertEqual(FakeDaemon.calls[2][1]["vx"], 0.06)

    def test_plan_does_not_include_backflip_yet(self):
        self.assertNotIn("backflip", PLANS)

    def test_sdk_rejection_attempts_stop_and_disarm(self):
        manager = ActionManager("/test/socket", client_factory=FailingActionDaemon)
        result = manager.execute("hello")
        status = self.finished(manager, result["run_id"])
        self.assertEqual(status["state"], "FAILED")
        self.assertEqual([op for op, _ in FakeDaemon.calls],
                         [ARM, SPORT_ACTION, STOP, DISARM])

    def test_cancelled_job_stops_and_disarms(self):
        manager = ActionManager("/test/socket", client_factory=FakeDaemon)
        result = manager.execute("dance_1")
        deadline = time.monotonic() + 1
        while manager.status(result["run_id"])["state"] != "RUNNING" and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertTrue(manager.cancel(result["run_id"])["ok"])
        status = self.finished(manager, result["run_id"])
        self.assertEqual(status["state"], "CANCELLED")
        self.assertTrue(status["stop_acknowledged"])
        self.assertEqual([op for op, _ in FakeDaemon.calls][-2:], [STOP, DISARM])
        self.assertFalse(manager.cancel_active()["ok"])


class ManagedDaemonTests(unittest.TestCase):
    def test_existing_daemon_envelope_and_operator_ack(self):
        owner = ManagedDaemon.from_config(
            {"daemon_binary": "/workspace/daemon", "network_interface": "wlx123"},
            "/workspace/sport.sock")
        argv = owner.argv()
        self.assertIn(PROFILE, argv)
        self.assertIn(MOTION_ACK, argv)
        self.assertEqual(argv[argv.index("--max-motion-ms") + 1], "0")
        self.assertFalse(owner.ready_to_activate({}))
        self.assertTrue(owner.ready_to_activate({
            "GO2_OPERATOR_PRESENT": "1",
            "GO2_STAGED_NAV2_RUNTIME_ACK": "I_APPROVE_GO2_STAGED_NAV2_MOTION",
        }))

    def test_managed_backend_needs_explicit_interface(self):
        with self.assertRaises(ValueError):
            ManagedDaemon.from_config({}, "/workspace/sport.sock")


class VoiceTests(unittest.TestCase):
    def test_explicit_commands(self):
        cases = {
            "请跳舞": "dance_1", "跳第一支舞": "dance_1",
            "跳第二支舞": "dance_2",
            "拜年": "new_year_greeting", "拜个年": "new_year_greeting",
            "倒立向前走": "handstand_walk",
            "做个倒立": "handstand", "伸展一下": "stretch",
            "打个招呼": "hello", "卧下再起身": "crouch",
            "卧倒再起身": "crouch",
        }
        for utterance, expected in cases.items():
            self.assertEqual(action_from_utterance(utterance), expected)

    def test_negation_and_unrelated_speech_are_not_actions(self):
        for utterance in ("不要跳舞", "别倒立", "停止跳舞", "导航到门口",
                          "有人跳舞吗", "先跳舞再鞠躬", "给大家鞠个躬", "倒立后空翻",
                          "向前跳", "左右摆动", "先跳舞再向前跳"):
            self.assertIsNone(action_from_utterance(utterance))

    def test_cancel_utterance_is_separate_from_execute(self):
        self.assertTrue(is_cancel_utterance("停止动作"))
        self.assertTrue(is_cancel_utterance("停下！"))
        self.assertFalse(is_cancel_utterance("不要跳舞"))


if __name__ == "__main__":
    unittest.main()
