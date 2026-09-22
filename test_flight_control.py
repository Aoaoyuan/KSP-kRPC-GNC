import unittest
from types import SimpleNamespace
from unittest.mock import patch

from FlightControl import ThrottleController, find_booster


class Engine:
    def __init__(self):
        self.active = False
        self.throttle_locked = False
        self.independent_throttle = False
        self.throttle = 0


class BrokenControl:
    @property
    def throttle(self):
        return 0

    @throttle.setter
    def throttle(self, _):
        pass


class ControlTests(unittest.TestCase):
    @patch("FlightControl.time.sleep")
    def test_failed_main_throttle_uses_independent_and_cleans_up(self, _):
        engines = [Engine(), Engine()]
        vessel = SimpleNamespace(parts=SimpleNamespace(engines=engines), control=BrokenControl())
        driver = ThrottleController(vessel)
        self.assertTrue(driver.independent)
        driver.set(.4)
        self.assertTrue(all(e.independent_throttle and e.throttle == .4 for e in engines))
        driver.close()
        self.assertTrue(all(not e.independent_throttle and e.throttle == 0 for e in engines))

    @patch("FlightControl.time.sleep")
    def test_healthy_main_throttle_stays_on_main(self, _):
        vessel = SimpleNamespace(parts=SimpleNamespace(engines=[Engine()]),
                                 control=SimpleNamespace(throttle=0))
        driver = ThrottleController(vessel)
        self.assertFalse(driver.independent)
        driver.set(.4)
        self.assertEqual(vessel.control.throttle, .4)
        driver.close()
        self.assertEqual(vessel.control.throttle, 0)

    @patch("FlightControl.time.sleep")
    def test_background_can_force_independent_throttle(self, _):
        engines = [Engine(), Engine()]
        vessel = SimpleNamespace(parts=SimpleNamespace(engines=engines),
                                 control=SimpleNamespace(throttle=0))
        driver = ThrottleController(vessel, force_independent=True)
        self.assertTrue(driver.independent)
        driver.set(.7)
        self.assertTrue(all(e.independent_throttle and e.throttle == .7 for e in engines))

    def test_reloaded_payload_does_not_hide_booster(self):
        payload = SimpleNamespace(loaded=True, parts=SimpleNamespace(with_tag=lambda _: []))
        booster = SimpleNamespace(loaded=True, parts=SimpleNamespace(with_tag=lambda _: [1]))
        sc = SimpleNamespace(active_vessel=payload, vessels=[payload, booster])
        self.assertIs(find_booster(sc), booster)

    def test_ambiguous_boosters_rejected(self):
        a = SimpleNamespace(loaded=True, parts=SimpleNamespace(with_tag=lambda _: [1]))
        with self.assertRaises(RuntimeError):
            find_booster(SimpleNamespace(vessels=[a, a]))


if __name__ == "__main__":
    unittest.main()
