"""三芯结构识别的纯逻辑回归测试。"""
import unittest
from types import SimpleNamespace

from MyHeavyLaunch import (core_push_pitch, discover_heavy, heavy_pitch,
                           post_separation_view, recovery_args,
                           validate_heavy_staging, activate_checked_stage)


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
        vessel = self.staged_vessel(core_stage=2)
        vessel.parts.all = [right, capsule, core, left, payload]
        self.assertEqual(discover_heavy(vessel), (left, right, core, payload))

    def test_rejects_missing_side_controller(self):
        vessel = SimpleNamespace(parts=SimpleNamespace(
            all=[self.part(3, -4), self.part(2, 0), self.part(-1, 0)]),
            reference_frame=object())
        with self.assertRaises(RuntimeError):
            discover_heavy(vessel)

    @classmethod
    def staged_vessel(cls, core_stage=1, payload_count=1, root_on_core=False):
        core_group = -1 if root_on_core else core_stage
        payload_group = core_stage if root_on_core else -1
        side_stage = core_stage + 1
        ignition = side_stage + 1
        anchors = [cls.part(side_stage, -4), cls.part(side_stage, 4),
                   cls.part(core_group, 0), cls.part(payload_group, 0, 30)]
        engines = []
        for stage, x, count, prop, action in (
                (side_stage, -4, 7, "LiquidFuel", ignition),
                (side_stage, 4, 7, "LiquidFuel", ignition),
                (core_group, 0, 7, "LiquidFuel", ignition),
                (side_stage, -4, 4, "SolidFuel", side_stage),
                (side_stage, 4, 4, "SolidFuel", side_stage),
                (payload_group, 0, payload_count, "LiquidFuel", 0)):
            for _ in range(count):
                part = cls.part(stage, x)
                part.stage = action
                engines.append(SimpleNamespace(part=part,
                    propellants=[SimpleNamespace(name=prop)]))
        decouplers = [SimpleNamespace(part=SimpleNamespace(stage=stage,
                        decouple_stage=stage))
                      for stage in (side_stage, side_stage, core_stage)]
        decouplers[-1].part.decouple_stage = core_group
        return SimpleNamespace(parts=SimpleNamespace(all=anchors, engines=engines,
            decouplers=decouplers, launch_clamps=[]), reference_frame=object(),
            control=SimpleNamespace(current_stage=ignition + 1))

    def test_old_and_current_stage_numbers_with_replaceable_payload(self):
        for core_stage in (1, 2):
            for payload_count in (0, 1, 4, 6):
                with self.subTest(core_stage=core_stage, payload_count=payload_count):
                    v = self.staged_vessel(core_stage, payload_count)
                    left, right, core, _ = discover_heavy(v)
                    self.assertEqual(validate_heavy_staging(v, left, right, core),
                                     ((7, 7, 7, payload_count), (4, 4)))

    def test_root_can_be_on_core_with_multiple_payload_controllers(self):
        for core_stage in (1, 2):
            for payload_count in (0, 1, 4, 7):
                with self.subTest(core_stage=core_stage, payload_count=payload_count):
                    v = self.staged_vessel(core_stage, payload_count, root_on_core=True)
                    v.parts.all.extend(self.part(core_stage, 0, y) for y in (20, 22, 24))
                    left, right, core, payload = discover_heavy(v)
                    self.assertEqual(core.decouple_stage, -1)
                    self.assertEqual(payload.decouple_stage, core_stage)
                    self.assertEqual(validate_heavy_staging(v, left, right, core),
                                     ((7, 7, 7, payload_count), (4, 4)))

    def test_rerooted_core_still_rejects_payload_ignition_at_separation(self):
        v = self.staged_vessel(core_stage=2, root_on_core=True)
        v.parts.engines[-1].part.stage = 2
        with self.assertRaisesRegex(RuntimeError, "载荷"):
            validate_heavy_staging(v, *discover_heavy(v)[:3])

    def test_rerooted_core_rejects_wrong_separator_action_stage(self):
        v = self.staged_vessel(core_stage=2, root_on_core=True)
        v.parts.decouplers[-1].part.stage = 1
        with self.assertRaisesRegex(RuntimeError, "分离器"):
            validate_heavy_staging(v, *discover_heavy(v)[:3])

    def test_engine_reads_can_return_fresh_proxy_objects(self):
        v = self.staged_vessel()
        original = v.parts

        class EngineProxy:
            def __init__(self, engine):
                self.part = engine.part
                self.propellants = engine.propellants

        class Parts:
            all = original.all
            decouplers = original.decouplers
            launch_clamps = original.launch_clamps

            @property
            def engines(self):
                return [EngineProxy(e) for e in original.engines]

        v.parts = Parts()
        counts, _ = validate_heavy_staging(v, *discover_heavy(v)[:3])
        self.assertEqual(counts, (7, 7, 7, 1))

    def test_rejects_early_payload_ignition(self):
        v = self.staged_vessel()
        v.parts.engines[-1].part.stage = 1
        with self.assertRaisesRegex(RuntimeError, "载荷"):
            validate_heavy_staging(v, *discover_heavy(v)[:3])

    def test_rejects_misstaged_separation_motor(self):
        v = self.staged_vessel()
        v.parts.engines[21].part.stage = 3
        with self.assertRaisesRegex(RuntimeError, "小火箭"):
            validate_heavy_staging(v, *discover_heavy(v)[:3])

    def test_rejects_misstaged_decoupler(self):
        v = self.staged_vessel()
        v.parts.decouplers[0].part.stage = 3
        with self.assertRaisesRegex(RuntimeError, "分离器"):
            validate_heavy_staging(v, *discover_heavy(v)[:3])

    def test_rejects_held_launch_clamp(self):
        v = self.staged_vessel()
        v.parts.launch_clamps = [SimpleNamespace(part=SimpleNamespace(stage=0))]
        with self.assertRaisesRegex(RuntimeError, "支架"):
            validate_heavy_staging(v, *discover_heavy(v)[:3])

    def test_manual_staging_change_never_triggers_another_stage(self):
        from unittest.mock import Mock
        v = self.staged_vessel()
        v.control.activate_next_stage = Mock()
        v.control.current_stage = 3
        with self.assertRaises(RuntimeError):
            activate_checked_stage(v, 3)
        v.control.activate_next_stage.assert_not_called()

    def test_checked_stage_advances_once(self):
        from unittest.mock import Mock
        v = self.staged_vessel()
        v.control.activate_next_stage = Mock(
            side_effect=lambda: setattr(v.control, 'current_stage', 3))
        activate_checked_stage(v, 3)
        v.control.activate_next_stage.assert_called_once_with()

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
