"""电影导演镜头决策的离线回归测试。"""

import unittest

from MissionCamera import MissionFacts, SHOTS, choose_shot


def facts(**overrides):
    values = dict(
        launched=True,
        side_split=False,
        core_split=False,
        sides_landed=False,
        core_landed=False,
        core_altitude=0,
        side_altitude=0,
        side_split_age=None,
        core_split_age=None,
        sides_landed_age=None,
    )
    values.update(overrides)
    return MissionFacts(**values)


class MissionCameraTests(unittest.TestCase):
    def test_ascent_uses_long_shots_in_altitude_order(self):
        names = [choose_shot("recap", facts(core_altitude=h))
                 for h in (100, 3000, 12000, 30000)]
        self.assertEqual(names, ["pad_departure", "tail_ascent",
                                 "gravity_turn", "curvature"])

    def test_separation_hero_has_priority_for_full_action(self):
        state = facts(side_split=True, side_split_age=3,
                      side_altitude=18000, core_altitude=24000)
        for profile in ("recap", "master", "boosters"):
            self.assertEqual(choose_shot(profile, state), "separation_hero")

    def test_separation_is_followed_by_one_wide_shot(self):
        state = facts(side_split=True, side_split_age=9,
                      side_ignited=True, side_ignition_age=10,
                      side_altitude=18000, core_altitude=25000)
        for profile in ("recap", "master", "boosters"):
            self.assertEqual(choose_shot(profile, state), "separation_wide")

    def test_real_boostback_ignition_gets_dedicated_hero_shot(self):
        state = facts(side_split=True, side_split_age=15,
                      side_ignited=True, side_ignition_age=2,
                      side_altitude=20000, core_altitude=30000)
        for profile in ("recap", "master", "boosters"):
            self.assertEqual(choose_shot(profile, state),
                             "boostback_ignition")

    def test_booster_profile_keeps_dual_landing_continuous(self):
        state = facts(side_split=True, side_split_age=20,
                      side_ignited=True, side_ignition_age=20,
                      core_split=True, core_split_age=2,
                      side_altitude=700, core_altitude=70000)
        self.assertEqual(choose_shot("boosters", state), "dual_landing")

    def test_master_profile_prioritizes_core_separation(self):
        state = facts(side_split=True, side_split_age=20,
                      side_ignited=True, side_ignition_age=20,
                      core_split=True, core_split_age=2,
                      side_altitude=700, core_altitude=70000)
        self.assertEqual(choose_shot("master", state), "core_separation")

    def test_payload_is_final_subject_after_core_landing(self):
        state = facts(side_split=True, side_split_age=100,
                      core_split=True, core_split_age=80,
                      sides_landed=True, sides_landed_age=50,
                      core_landed=True, core_altitude=0,
                      side_altitude=0)
        for profile in ("recap", "master", "boosters"):
            self.assertEqual(choose_shot(profile, state), "payload_final")

    def test_dual_landing_and_final_shots_have_long_holds(self):
        self.assertGreaterEqual(SHOTS["dual_landing"].min_hold, 10)
        self.assertGreaterEqual(SHOTS["payload_final"].min_hold, 9)


if __name__ == "__main__":
    unittest.main()
