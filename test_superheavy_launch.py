"""先后两对侧芯结构的只读预检测试。"""
import unittest
from types import SimpleNamespace

from MySuperHeavyLaunch import (discover_superheavy, expected_engines,
                                ground_distance, side_parts, validate_superheavy)


class SuperHeavyTests(unittest.TestCase):
    @staticmethod
    def part(stage, xyz, action=0, command=False):
        modules = [SimpleNamespace(name="ModuleCommand")] if command else []
        resource = SimpleNamespace(name="LiquidFuel", amount=10, max=10)
        p = SimpleNamespace(decouple_stage=stage, stage=action,
                            modules=modules,
                            resources=SimpleNamespace(all=[resource]))
        p.position = lambda frame, xyz=xyz: xyz
        return p

    def vessel(self, *, root_on_core=False):
        first, second, core_stage, ignition = 9, 8, 7, 10
        core_group = -1 if root_on_core else core_stage
        payload_group = core_stage if root_on_core else -1
        stages = [(first, (0, 0, 4)), (first, (0, 0, -4)),
                  (second, (4, 0, 0)), (second, (-4, 0, 0))]
        anchors = [self.part(stage, xyz, command=True) for stage, xyz in stages]
        core = self.part(core_group, (0, 0, 0), command=True)
        payload = self.part(payload_group, (0, 20, 0), command=True)
        engines = []
        for stage, xyz, count, propellant, action in (
                *((stage, xyz, 7, "LiquidFuel", ignition) for stage, xyz in stages),
                (core_group, (0, 0, 0), 7, "LiquidFuel", ignition),
                *((stage, xyz, 4, "SolidFuel", stage) for stage, xyz in stages)):
            for _ in range(count):
                p = self.part(stage, xyz, action=action)
                engines.append(SimpleNamespace(part=p,
                    propellants=[SimpleNamespace(name=propellant)]))
        decouplers = [SimpleNamespace(part=self.part(stage, xyz, action=stage))
                      for stage, xyz in stages]
        decouplers.append(SimpleNamespace(part=self.part(
            core_group, (0, 0, 0), action=core_stage)))
        parts = SimpleNamespace(all=anchors+[core, payload], engines=engines,
                                decouplers=decouplers, launch_clamps=[])
        return SimpleNamespace(parts=parts, reference_frame=object(),
                               control=SimpleNamespace(current_stage=11))

    def test_two_opposed_pairs_and_core_detected_with_either_root(self):
        for root_on_core in (False, True):
            with self.subTest(root_on_core=root_on_core):
                vessel = self.vessel(root_on_core=root_on_core)
                anchors, core, payload = discover_superheavy(vessel)
                self.assertEqual(sorted(a.decouple_stage for a in anchors),
                                 [8, 8, 9, 9])
                self.assertEqual(core.decouple_stage,
                                 -1 if root_on_core else 7)
                self.assertEqual(payload.decouple_stage,
                                 7 if root_on_core else -1)
                self.assertEqual(validate_superheavy(
                    vessel, anchors, core, side_engines=7,
                    core_engines=7, separation_motors=4),
                    ((7, 7, 7, 7), 7, (4, 4, 4, 4)))
                self.assertTrue(all(len(side_parts(vessel, anchors, i)) == 1
                                    for i in range(4)))

    def test_missing_engine_or_decoupler_blocks_launch(self):
        vessel = self.vessel()
        anchors, core, _ = discover_superheavy(vessel)
        vessel.parts.engines.pop(0)
        with self.assertRaisesRegex(RuntimeError, "发动机数不符"):
            validate_superheavy(vessel, anchors, core, side_engines=7,
                                core_engines=7, separation_motors=4)
        vessel = self.vessel()
        anchors, core, _ = discover_superheavy(vessel)
        vessel.parts.decouplers.pop(0)
        with self.assertRaisesRegex(RuntimeError, "分离器"):
            validate_superheavy(vessel, anchors, core, side_engines=7,
                                core_engines=7, separation_motors=4)

    def test_early_pair_must_be_opposed(self):
        vessel = self.vessel()
        early = [p for p in vessel.parts.all if p.decouple_stage == 9]
        early[1].position = lambda frame: (1, 0, 4)
        with self.assertRaisesRegex(RuntimeError, "对置"):
            discover_superheavy(vessel)

    def test_actual_payload_controllers_and_handoff_engines_are_allowed(self):
        vessel = self.vessel()
        for height in (21, 22, 23):
            vessel.parts.all.append(self.part(5, (0, height, 0),
                                               command=True))
        for _ in range(4):
            engine_part = self.part(5, (2, 12, 2), action=7)
            vessel.parts.engines.append(SimpleNamespace(
                part=engine_part, active=False,
                propellants=[SimpleNamespace(name="LiquidFuel"),
                             SimpleNamespace(name="Oxidizer")]))
        anchors, core, _ = discover_superheavy(vessel)
        self.assertEqual(validate_superheavy(
            vessel, anchors, core, side_engines=7,
            core_engines=7, separation_motors=4)[1], 7)

    def test_recovery_engine_count_and_distance(self):
        self.assertEqual(expected_engines(
            ["--expected-engines", "7"], 9), ["--expected-engines", "9"])
        body = SimpleNamespace(equatorial_radius=600000)
        self.assertEqual(ground_distance(body, (0, 0), (0, 0)), 0)
        self.assertGreater(ground_distance(body, (0, 0), (0, 1)), 10000)


if __name__ == "__main__":
    unittest.main()
