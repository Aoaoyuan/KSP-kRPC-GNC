"""离线测试发射参数及分离后载具交接。"""
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from MyLaunchV2 import (RecoveryManager, ascent_throttle, pitch_for_altitude,
                        split_booster, tagged_part, take_booster_control)


class Vessel:
    def __init__(self):
        self.name = "同名载具"
        self.control = SimpleNamespace(throttle=1)
        self.parts = SimpleNamespace(controlling=None)


class LaunchTests(unittest.TestCase):
    def test_pitch_profile(self):
        self.assertAlmostEqual(pitch_for_altitude(0), 90)
        self.assertAlmostEqual(pitch_for_altitude(30000), 10)
        self.assertAlmostEqual(pitch_for_altitude(80000), 10)
        values = [pitch_for_altitude(h) for h in range(0, 31000, 1000)]
        self.assertEqual(values, sorted(values, reverse=True))

    def test_throttle_cutoff_and_no_thrust(self):
        self.assertEqual(ascent_throttle(75000, 75000, 10000, 250000), 0)
        self.assertLess(ascent_throttle(74900, 75000, 10000, 250000), 0.1)
        with self.assertRaises(RuntimeError):
            ascent_throttle(10000, 75000, 10000, 0)

    def test_payload_becomes_active_then_select_booster(self):
        booster, payload = Vessel(), Vessel()
        a, b = SimpleNamespace(vessel=booster), SimpleNamespace(vessel=payload)
        sc = SimpleNamespace(active_vessel=payload)
        self.assertIs(take_booster_control(sc, a, b), booster)
        self.assertIs(sc.active_vessel, booster)
        self.assertIs(booster.parts.controlling, a)
        self.assertEqual(booster.control.throttle, 0)
        self.assertEqual(payload.control.throttle, 1)  # 未误控载荷

    def test_background_mode_keeps_payload_active(self):
        booster, payload = Vessel(), Vessel()
        a, b = SimpleNamespace(vessel=booster), SimpleNamespace(vessel=payload)
        sc = SimpleNamespace(active_vessel=payload)
        self.assertIs(take_booster_control(sc, a, b, activate=False), booster)
        self.assertIs(sc.active_vessel, payload)
        self.assertIs(booster.parts.controlling, a)

    def test_wait_for_part_ownership_update(self):
        whole, booster, payload = Vessel(), Vessel(), Vessel()
        a, b = SimpleNamespace(vessel=whole), SimpleNamespace(vessel=whole)
        self.assertIsNone(split_booster(a, b))
        sc = SimpleNamespace(active_vessel=payload)
        def separate(_):
            a.vessel, b.vessel = booster, payload
        with patch("MyLaunchV2.time.sleep", side_effect=separate):
            self.assertIs(take_booster_control(sc, a, b), booster)

    def test_unconfirmed_split_times_out(self):
        whole = Vessel()
        a = SimpleNamespace(vessel=whole)
        with patch("MyLaunchV2.time.monotonic", side_effect=[0, 0, 11]), \
                patch("MyLaunchV2.time.sleep"):
            with self.assertRaises(RuntimeError):
                take_booster_control(SimpleNamespace(active_vessel=whole), a, a)

    def test_missing_or_duplicate_tag_rejected(self):
        for result in ([], [object(), object()]):
            vessel = SimpleNamespace(parts=SimpleNamespace(with_tag=lambda _: result))
            with self.assertRaises(RuntimeError):
                tagged_part(vessel, "booster")

    def test_recovery_manager_forwards_confirmed_vessel_and_connection(self):
        calls = []
        vessel = SimpleNamespace(name="old")
        with patch("MyLaunchV2.recover",
                   side_effect=lambda args, **kwargs: calls.append((args, kwargs))):
            manager = RecoveryManager()
            manager.start("core", ["--execute"], selected_vessel=vessel)
            manager.wait()
        self.assertTrue(vessel.name.startswith("Codex core "))
        self.assertEqual(calls, [(["--execute"], {
            "selected_vessel_name": vessel.name})])

    def test_recovery_manager_forwards_shared_ignition_barrier(self):
        calls = []
        left = SimpleNamespace(name="left")
        right = SimpleNamespace(name="right")
        with patch("MyLaunchV2.recover",
                   side_effect=lambda args, **kwargs: calls.append((args, kwargs))):
            manager = RecoveryManager()
            barrier = manager.ignition_barrier(2)
            manager.start("left", ["--execute"], selected_vessel=left,
                          ignition_barrier=barrier)
            manager.start("right", ["--execute"], selected_vessel=right,
                          ignition_barrier=barrier)
            manager.wait()
        self.assertEqual(len(calls), 2)
        self.assertTrue(all(call[1]["ignition_barrier"] is barrier
                            for call in calls))
        self.assertEqual({call[1]["ignition_label"] for call in calls},
                         {"left", "right"})

    def test_ignition_barrier_rejects_single_vessel(self):
        with self.assertRaises(ValueError):
            RecoveryManager.ignition_barrier(1)


if __name__ == "__main__":
    unittest.main()
