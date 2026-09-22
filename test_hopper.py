"""横移制导与支腿终态回归测试，不连接游戏。"""
import math
import unittest
from types import SimpleNamespace

from HopperTest import transfer_acceleration
from FlightControl import deploy_grid_fins, grid_fins_deployed, retract_grid_fins


class HopperTests(unittest.TestCase):
    def test_brakes_before_crossing_target(self):
        north, east = transfer_acceleration(100, 90, 0, 80)
        self.assertLess(east, 0)  # 仍在目标西侧，但速度过高，必须朝西刹车。
        self.assertLessEqual(math.hypot(north, east), 3.0)

    def test_removes_cross_track_velocity(self):
        north, east = transfer_acceleration(500, 90, 8, 0)
        self.assertLess(north, 0)
        self.assertGreater(east, 0)

    def test_transfer_converges_with_attitude_lag(self):
        # 625 m 横移，推力方向一阶滞后 1.5 s。并非 KSP 实飞替代品，
        # 仅检查“位置减小但速度不减”这一旧错误不会再次出现。
        x, v, accel = -625.0, 0.0, 0.0
        dt = .05
        for _ in range(4400):
            _, command = transfer_acceleration(abs(x), 90 if x < 0 else 270, 0, v)
            accel += (command - accel) * dt / 1.5
            v += accel * dt
            x += v * dt
        self.assertLess(abs(x), 1.0)
        self.assertLess(abs(v), .2)

    def test_grid_deploy_is_idempotent_and_leaves_legs_alone(self):
        calls = []
        event = SimpleNamespace(name='Toggle', active=True, gui_name='Extend Fins')
        def trigger():
            calls.append('extend')
            event.gui_name = 'Retract Fins'
        event.trigger = trigger
        module = SimpleNamespace(name='ModuleAnimateGeneric', event_list=[event],
                                 field_list=[SimpleNamespace(name='animTime', value='1'),
                                             SimpleNamespace(name='aniState', value='LOCKED')])
        vessel = SimpleNamespace(parts=SimpleNamespace(all=[
            SimpleNamespace(title='T-222 Grid Fin Large', modules=[module]),
            SimpleNamespace(title='Falcon Landing Gear Large', modules=[module])]))
        deploy_grid_fins(vessel)
        fins = deploy_grid_fins(vessel)
        self.assertEqual(calls, ['extend'])
        self.assertEqual(len(fins), 1)
        self.assertEqual(grid_fins_deployed(fins), 1)
        module.field_list[0].value = '0'
        self.assertEqual(grid_fins_deployed(fins), 0)

    def test_grid_retract_only_triggers_explicit_retract_action(self):
        calls = []
        retract = SimpleNamespace(name='Toggle', active=True, gui_name='Retract Fins',
                                  trigger=lambda: calls.append('retract'))
        extend = SimpleNamespace(name='Toggle', active=True, gui_name='Extend Fins',
                                 trigger=lambda: calls.append('extend'))
        modules = [SimpleNamespace(event_list=[retract]),
                   SimpleNamespace(event_list=[extend])]
        self.assertEqual(retract_grid_fins(modules), 1)
        self.assertEqual(calls, ['retract'])


if __name__ == "__main__":
    unittest.main()
