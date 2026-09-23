"""三芯结构识别的纯逻辑回归测试。"""
import unittest
from types import SimpleNamespace

from MyHeavyLaunch import (core_push_pitch, discover_heavy, heavy_pitch,
                           post_separation_view, recovery_args)


class HeavyLaunchTests(unittest.TestCase):
    @staticmethod
    def part(stage, x, y=0):
        module = SimpleNamespace(name="ModuleCommand")
        part = SimpleNamespace(decouple_stage=stage, modules=[module])
        part.position = lambda frame, xyz=(x, y, 0): xyz
        return part

    def test_discovers_three_cores_by_stage_and_position(self):
        left = self.part(3, -4)
        right = self.part(3, 4)
        core = self.part(2, 0)
        capsule = self.part(-1, 0, 20)
        payload = self.part(-1, 0, 30)
        vessel = SimpleNamespace(parts=SimpleNamespace(
            all=[right, capsule, core, left, payload]), reference_frame=object())
        self.assertEqual(discover_heavy(vessel), (left, right, core, payload))

    def test_rejects_missing_side_controller(self):
        vessel = SimpleNamespace(parts=SimpleNamespace(
            all=[self.part(3, -4), self.part(2, 0), self.part(-1, 0)]),
            reference_frame=object())
        with self.assertRaises(RuntimeError):
            discover_heavy(vessel)

    def test_gravity_turn_is_smooth_and_actually_turns(self):
        samples = [heavy_pitch(h) for h in (0, 500, 4000, 8000, 12000,
                                             20000, 30000, 40000, 60000)]
        self.assertEqual(samples[:2], [90, 90])
        self.assertGreater(samples[2], samples[3])
        self.assertAlmostEqual(samples[4], 50)
        self.assertLess(samples[5], 45)
        self.assertAlmostEqual(samples[7], 5)
        self.assertTrue(all(a >= b for a, b in zip(samples, samples[1:])))

    def test_core_push_keeps_nose_up_until_apoapsis_clears_atmosphere(self):
        early = core_push_pitch(18000, 330, 24000, 24, 9.5, 100)
        late = core_push_pitch(65000, 500, 80000, 24, 8.0, 20)
        self.assertGreater(early, 20)
        self.assertLess(early, 45)
        self.assertAlmostEqual(late, 5)

    def test_core_push_rejects_invalid_time_guidance(self):
        with self.assertRaises(ValueError):
            core_push_pitch(15000, 380, 25000, 24, 9.5, 0)

    def test_recovery_args_passes_grid_retract_height(self):
        args = recovery_args("booster_core", 16, grid_retract_height=3500,
                             require_land=True, aero_target_tilt=20)
        index = args.index("--grid-retract-height")
        self.assertEqual(args[index + 1], "3500")
        self.assertIn("--require-land", args)
        self.assertEqual(args[args.index("--aero-target-tilt") + 1], "20")

    def test_core_recovery_explicitly_disables_boostback(self):
        args = recovery_args("booster_core", 16, target=(0, -38.5),
                             no_boostback=True)
        self.assertIn("--no-boostback", args)

    def test_post_separation_view_preserves_side_booster(self):
        side, combined, upper = object(), object(), object()
        self.assertIs(post_separation_view(side, combined, upper,
                                           [side, upper]), side)
        self.assertIs(post_separation_view(combined, combined, upper,
                                           [upper]), upper)


if __name__ == "__main__":
    unittest.main()
