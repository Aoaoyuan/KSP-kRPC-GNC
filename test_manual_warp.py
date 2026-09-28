"""后台回收手动加速的边界与降倍率行为。"""
import math
import unittest
from types import SimpleNamespace
from MyRecoverV2 import coast_allows_manual_warp, restrict_manual_warp


class ManualWarpTests(unittest.TestCase):
    def safe(self, **overrides):
        values = dict(state="COAST", height=150000, sea_altitude=150000,
                      vertical_speed=-300, gravity=8, brake_exit_height=20000,
                      reentry_altitude=45000, atmosphere_depth=70000)
        values.update(overrides)
        return coast_allows_manual_warp(**values)

    def test_high_coast_is_allowed(self):
        self.assertTrue(self.safe())

    def test_exits_before_atmosphere_not_only_before_engine_ignition(self):
        # Still above atmosphere, but less than 30 seconds of descent margin.
        self.assertFalse(self.safe(sea_altitude=80000, height=80000))

    def test_fast_descent_needs_more_margin(self):
        self.assertTrue(self.safe(sea_altitude=100000, height=100000))
        self.assertFalse(self.safe(sea_altitude=100000, height=100000,
                                   vertical_speed=-1000))

    def test_burn_and_terminal_states_forbid_warp(self):
        for state in ("BOOSTBACK", "REENTRY", "BRAKE", "TERMINAL"):
            with self.subTest(state=state):
                self.assertFalse(self.safe(state=state))

    def test_braking_margin_and_invalid_data_fail_closed(self):
        self.assertFalse(self.safe(brake_exit_height=160000))
        self.assertFalse(self.safe(vertical_speed=math.nan))
        self.assertFalse(self.safe(sea_altitude=math.inf))

    def test_never_enables_or_increases_warp(self):
        for rails in (0, 1, 2):
            sc = SimpleNamespace(rails_warp_factor=rails, physics_warp_factor=0)
            restrict_manual_warp(sc, True)
            self.assertEqual(sc.rails_warp_factor, rails)
            self.assertEqual(sc.physics_warp_factor, 0)

    def test_excessive_rails_warp_is_limited(self):
        sc = SimpleNamespace(rails_warp_factor=5, physics_warp_factor=0)
        restrict_manual_warp(sc, True)
        self.assertEqual(sc.rails_warp_factor, 2)

    def test_unsafe_segment_returns_both_warp_types_to_one_x(self):
        for rails, physics in ((2, 0), (0, 3)):
            sc = SimpleNamespace(rails_warp_factor=rails, physics_warp_factor=physics)
            restrict_manual_warp(sc, False)
            self.assertEqual((sc.rails_warp_factor, sc.physics_warp_factor), (0, 0))
