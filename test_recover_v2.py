"""离线验证控制数学；不会导入 krpc 或连接游戏。"""
import math
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from MyRecoverV2 import (Config, Guidance, deploy_landing_legs,
                         landing_legs_deployed, dot, norm, slew_direction,
                         recover_landed_vessel)


class FakeLeg:
    def __init__(self, deployed=False, deployable=True):
        self.deployed = deployed
        self.deployable = deployable


class RecoveryTests(unittest.TestCase):
    def sample(self, controller, **changes):
        data = dict(position=(600100.0, 0.0, 0.0), velocity=(-10.0, 0.0, 0.0),
                    direction=(1.0, 0.0, 0.0), altitude=100.0, sea_altitude=100.0,
                    mass=10000.0, thrust=250000.0, mu=3.5316e12, dt=0.05)
        data.update(changes)
        return controller.update(**data)

    def test_gravity_feedforward_at_terminal_speed(self):
        c = Guidance(Config())
        _, throttle, _ = self.sample(c, altitude=16.0, velocity=(-2.5, 0, 0))
        gravity = 3.5316e12 / 600100.0 ** 2
        self.assertAlmostEqual(throttle, gravity / 25.0)

    def test_disabled_boostback_never_enters_boostback(self):
        c = Guidance(Config(enable_boostback=False,
                            boostback_cutoff_height=3000.0))
        _, throttle, _ = self.sample(
            c, altitude=50000, sea_altitude=50000,
            velocity=(0, 250, 0),
            target_position=(600100, 100000, 0))
        self.assertEqual(c.state, "COAST")
        self.assertEqual(throttle, 0.0)

    def test_landing_leg_command_is_idempotent_and_language_independent(self):
        legs = [FakeLeg(), FakeLeg(deployed=True)]
        self.assertEqual(deploy_landing_legs(legs), 1)
        self.assertTrue(landing_legs_deployed(legs))
        self.assertEqual(deploy_landing_legs(legs), 2)

    def test_horizontal_braking_and_tilt_limit(self):
        target, throttle, _ = self.sample(Guidance(Config()), velocity=(-20, 100, 0))
        self.assertLess(target[1], 0)
        self.assertLessEqual(math.degrees(math.acos(target[0])), 20.00001)
        self.assertTrue(0 <= throttle <= 1)

    def test_low_altitude_abandons_unreachable_position_and_brakes_sideways(self):
        c = Guidance(Config())
        # 目标仍在数公里外，但低于 landing_floor 后必须先刹掉向东速度，
        # 不能继续朝目标追点直到触地倾倒。
        target, _, _ = self.sample(
            c, position=(601000, 0, 0), altitude=1000, sea_altitude=1000,
            velocity=(-120, 25, 0), direction=(1, 0, 0),
            target_position=(601000, 5000, 0))
        self.assertLess(target[1], 0)

    def test_low_altitude_horizontal_capture_locks_upright(self):
        c = Guidance(Config())
        target, _, _ = self.sample(
            c, altitude=100, sea_altitude=100,
            velocity=(-20, 2, 0), direction=(0.98, -0.2, 0))
        self.assertTrue(c.terminal_lateral_complete)
        self.assertEqual(target, (1.0, 0.0, 0.0))
        # 捕获后即使残余横速短时回升，也不在触地前反向倾斜追零。
        target, _, _ = self.sample(
            c, altitude=60, sea_altitude=60,
            velocity=(-10, -8, 0), direction=(0.99, 0.1, 0))
        self.assertEqual(target, (1.0, 0.0, 0.0))

    def test_radial_projection_not_global_axis_slice(self):
        _, _, info = self.sample(Guidance(Config()), position=(0, 600100, 0),
                                 direction=(0, 1, 0), velocity=(3, -10, 4))
        self.assertAlmostEqual(info[1], -10)
        self.assertAlmostEqual(info[2], 5)

    def test_zero_speed_is_finite(self):
        target, throttle, _ = self.sample(Guidance(Config()), velocity=(0, 0, 0))
        self.assertAlmostEqual(norm(target), 1)
        self.assertTrue(math.isfinite(throttle))

    def test_core_recovery_can_skip_reentry_burn(self):
        c = Guidance(Config(enable_reentry_burn=False))
        _, throttle, _ = self.sample(
            c, position=(625000, 0, 0), altitude=25000, sea_altitude=25000,
            velocity=(-1300, 0, 0), thrust=2500000)
        self.assertEqual(c.state, "COAST")
        self.assertEqual(throttle, 0)

    def test_reentry_burn_cannot_restart_after_cutoff(self):
        c = Guidance(Config(reentry_altitude=70000,
                            reentry_on_speed=1200,
                            reentry_off_speed=1100,
                            boostback_cutoff_height=100000))
        # 第一次超过开启速度时进入再入点火。
        self.sample(c, position=(650000, 0, 0), altitude=50000,
                    sea_altitude=50000, velocity=(-200, 1285, 0),
                    direction=(0.154, -0.988, 0))
        self.assertEqual(c.state, "REENTRY")
        # 速度降到关机门槛后锁存完成。
        self.sample(c, position=(645000, 0, 0), altitude=45000,
                    sea_altitude=45000, velocity=(-200, 1000, 0),
                    direction=(0.196, -0.981, 0))
        self.assertEqual(c.state, "COAST")
        self.assertTrue(c.reentry_complete)
        # 继续下落时即使再次加速越过开启门槛，也不能第二次点火。
        _, throttle, _ = self.sample(
            c, position=(640000, 0, 0), altitude=40000,
            sea_altitude=40000, velocity=(-200, 1285, 0),
            direction=(0.154, -0.988, 0))
        self.assertEqual(c.state, "COAST")
        self.assertEqual(throttle, 0)

    def test_aero_targeting_uses_limited_velocity_error_bias(self):
        c = Guidance(Config(aero_target_tilt=10, enable_reentry_burn=False,
                            boostback_cutoff_height=100000))
        position = (650000.0, 0.0, 0.0)
        velocity = (-200.0, 100.0, 0.0)
        pure_retrograde = tuple(-x / norm(velocity) for x in velocity)
        target, throttle, _ = self.sample(
            c, position=position, altitude=50000, sea_altitude=50000,
            velocity=velocity, direction=pure_retrograde,
            target_position=(650000.0, 5000.0, 5000.0))
        turn = math.degrees(math.acos(max(-1, min(1, dot(pure_retrograde, target)))))
        self.assertLessEqual(turn, 10.0001)
        # 发动机朝前的逆行姿态需要把机头向期望气动力的反方向偏转。
        self.assertLess(target[2], 0)
        self.assertEqual(throttle, 0)

    def test_warp_sized_game_time_step_is_accepted(self):
        _, throttle, _ = self.sample(Guidance(Config()), dt=4)
        self.assertTrue(math.isfinite(throttle))

    def test_bad_thrust_and_time_fail(self):
        for changes in (dict(thrust=0), dict(thrust=50000), dict(dt=-1), dict(dt=61)):
            with self.assertRaises((ValueError, RuntimeError)):
                self.sample(Guidance(Config()), **changes)

    def test_early_braking_and_latched_landing(self):
        c = Guidance(Config())
        self.sample(c, altitude=4000, sea_altitude=4000, velocity=(-350, 0, 0))
        self.assertEqual(c.state, "BURN")
        self.sample(c, altitude=5000, sea_altitude=5000, velocity=(10, 0, 0))
        self.assertEqual(c.state, "TERMINAL")

    def test_coasts_until_suicide_burn_distance(self):
        c = Guidance(Config())
        _, throttle, _ = self.sample(c, altitude=50000, sea_altitude=50000,
                                     velocity=(-300, 0, 0))
        self.assertEqual(c.state, "COAST")
        self.assertEqual(throttle, 0)
        _, throttle, _ = self.sample(c, altitude=4000, sea_altitude=4000,
                                     velocity=(-300, 0, 0))
        self.assertEqual(c.state, "BURN")
        self.assertEqual(throttle, 1)

    def test_high_altitude_boostback_targets_return_velocity(self):
        c = Guidance(Config())
        target, throttle, info = self.sample(
            c, position=(650000, 0, 0), altitude=50000, sea_altitude=50000,
            velocity=(-200, 0, 0), direction=(1, 0, 0),
            target_position=(650000, -5000, 0))
        self.assertEqual(c.state, "BOOSTBACK")
        self.assertEqual(throttle, 0)  # 先横滚/俯仰到横向，不在未对准时点火。
        self.assertLess(target[1], 0)  # 目标在西侧，推力应产生向西的水平加速度。
        self.assertLess(info[3], 0)    # 遥测中的目标水平速度也指向西侧。
        _, throttle, _ = self.sample(
            c, position=(650000, 0, 0), altitude=50000, sea_altitude=50000,
            velocity=(-200, 0, 0), direction=(0, -1, 0),
            target_position=(650000, -5000, 0))
        self.assertEqual(throttle, Config().boostback_throttle)

    def test_falling_time_prediction_uses_future_quadratic_root(self):
        c = Guidance(Config(boostback_return_gain=1.0,
                            boostback_speed_cap=1000.0,
                            boostback_cutoff_height=3000.0))
        altitude = 10000.0
        height = altitude - c.cfg.leg_offset
        down = 300.0
        radius = 600000.0 + height
        gravity = 3.5316e12 / radius ** 2
        distance = 5000.0
        _, _, info = self.sample(
            c, position=(radius, 0, 0), altitude=altitude, sea_altitude=altitude,
            velocity=(-down, 0, 0), direction=(0, 1, 0),
            target_position=(radius, distance, 0))
        expected_time = (-down + math.sqrt(down * down + 2 * gravity * height)) / gravity
        self.assertAlmostEqual(-info[3], distance / expected_time, places=6)

    def test_low_trajectory_can_use_lower_boostback_gate(self):
        c = Guidance(Config(boostback_cutoff_height=8000))
        target, _, _ = self.sample(
            c, position=(613000, 0, 0), altitude=13000, sea_altitude=13000,
            velocity=(200, 100, 0), direction=(0, 1, 0),
            target_position=(600000, 5000, 0))
        self.assertEqual(c.state, "BOOSTBACK")

    def test_boostback_waits_for_payload_clearance(self):
        c = Guidance(Config())
        _, throttle, _ = self.sample(
            c, position=(650000, 0, 0), altitude=50000, sea_altitude=50000,
            velocity=(-200, 0, 0), direction=(1, 0, 0),
            target_position=(650000, -5000, 0), boostback_allowed=False)
        self.assertEqual(c.state, "COAST")
        self.assertEqual(throttle, 0)

    def test_boostback_uses_hysteresis_instead_of_pulsing(self):
        c = Guidance(Config())
        # 首帧误差足够大，进入连续返推。
        self.sample(c, position=(650000, 0, 0), altitude=50000, sea_altitude=50000,
                    velocity=(-200, 0, 0), direction=(1, 0, 0),
                    target_position=(650000, -5000, 0))
        self.assertEqual(c.state, "BOOSTBACK")
        # 误差缩小到进入阈值以下、但尚未达到退出阈值时仍保持返推状态。
        self.sample(c, position=(650000, 0, 0), altitude=50000, sea_altitude=50000,
                    velocity=(-200, -6, 0), direction=(1, 0, 0),
                    target_position=(650000, -5000, 0))
        self.assertEqual(c.state, "BOOSTBACK")

    def test_completed_boostback_returns_to_coast(self):
        c = Guidance(Config())
        # 先进入返场状态。
        _, _, first_info = self.sample(
            c, position=(650000, 0, 0), altitude=50000, sea_altitude=50000,
            velocity=(-200, 0, 0), direction=(1, 0, 0),
            target_position=(650000, -5000, 0))
        self.assertEqual(c.state, "BOOSTBACK")
        # 将水平速度放到预测目标附近，返场完成后不能停留在旧状态。
        desired_speed = first_info[3]
        self.sample(c, position=(650000, 0, 0), altitude=49000, sea_altitude=49000,
                    velocity=(-200, desired_speed, 0), direction=(0, -1, 0),
                    target_position=(650000, -5000, 0))
        self.assertEqual(c.state, "COAST")

    def test_boostback_target_tracks_new_velocity_error_with_rate_limit(self):
        c = Guidance(Config(boostback_turn_rate=20))
        # 首帧需要向西修正。
        first, _, _ = self.sample(
            c, position=(650000, 0, 0), altitude=50000, sea_altitude=50000,
            velocity=(-200, 0, 0), direction=(0, -1, 0),
            target_position=(650000, -5000, 0), dt=1.0)
        self.assertLess(first[1], 0)
        # 下一帧实际水平速度已经返推过量，速度误差改为向东。指令应开始
        # 朝新方向转，但每秒不得超过配置的 20°，且未对准时必须收油。
        second, throttle, _ = self.sample(
            c, position=(650000, 0, 0), altitude=49000, sea_altitude=49000,
            velocity=(-200, -100, 0), direction=(0, -1, 0),
            target_position=(650000, -5000, 0), dt=1.0)
        turn = math.degrees(math.acos(max(-1, min(1, dot(first, second)))))
        self.assertLessEqual(turn, 20.0001)
        self.assertEqual(throttle, 0)

    def test_slew_direction_handles_exact_reversal(self):
        result = slew_direction((0, -1, 0), (0, 1, 0), math.radians(10))
        self.assertAlmostEqual(norm(result), 1)
        self.assertAlmostEqual(math.degrees(math.acos(dot((0, -1, 0), result))), 10)

    def test_separation_holds_initial_attitude_until_clear(self):
        c = Guidance(Config())
        target, throttle, _ = self.sample(
            c, position=(650000, 100, 0), altitude=50000, sea_altitude=50000,
            velocity=(-200, 0, 0), direction=(0, 1, 0),
            target_position=(650000, -5000, 0), boostback_allowed=False,
            payload_position=(650000, 0, 0), payload_velocity=(-200, 0, 0))
        self.assertEqual(c.state, "SEPARATION")
        self.assertEqual(target, (0, 1, 0))
        self.assertEqual(throttle, 0)  # 只靠分离器拉开，不转向也不额外点火。

    def test_warp_exit_forecast_precedes_brake_point(self):
        c = Guidance(Config())
        self.sample(c, altitude=50000, sea_altitude=50000,
                    velocity=(-600, 0, 0))
        self.assertEqual(c.state, "COAST")
        self.assertGreater(c.warp_exit_height, c.brake_height)
        # 安全加速会在预计自由下落 30 秒之前结束，而不是等于点火高度。
        self.assertGreater(c.warp_exit_height, c.brake_height + 15000)

    def test_upside_down_inhibits_burn(self):
        _, throttle, _ = self.sample(Guidance(Config()), direction=(-1, 0, 0))
        self.assertEqual(throttle, 0)

    def test_ideal_vertical_descent(self):
        # 简化点质量积分：无阻力、无推力延迟、姿态即时跟踪。
        # 检查下降包线能否在多种初始条件下收敛，不能替代 KSP 飞行验证。
        for initial_height, initial_speed in ((1500, -100), (4000, -250), (100, 0)):
            c = Guidance(Config())
            altitude, speed = float(initial_height), float(initial_speed)
            for _ in range(20000):
                radius = 600000 + altitude
                target, throttle, _ = self.sample(c, position=(radius, 0, 0),
                    altitude=altitude, sea_altitude=altitude, velocity=(speed, 0, 0))
                acceleration = throttle * 25 * dot(target, (1, 0, 0)) - 3.5316e12 / radius ** 2
                speed += acceleration * 0.05
                altitude += speed * 0.05
                if altitude <= 16:
                    break
            self.assertLessEqual(altitude, 16, (initial_height, speed))
            # 离散积分在接触瞬间可能略高于指令值；仍限制在支腿可承受的范围。
            self.assertLess(abs(speed), 5.0, (initial_height, speed))

    def test_auto_recover_removes_stable_background_vessel(self):
        situation = SimpleNamespace(landed="landed", splashed="splashed")
        active = object()
        booster = SimpleNamespace(situation="landed", name="Codex booster_left")
        sc = SimpleNamespace(VesselSituation=situation, active_vessel=active,
                             vessels=[booster])
        booster.recover = Mock(side_effect=lambda: sc.vessels.clear())
        recover_landed_vessel(sc, booster, "payload", True)
        booster.recover.assert_called_once_with()
        self.assertEqual(sc.vessels, [])
        self.assertIs(sc.active_vessel, active)

    def test_auto_recover_refuses_a_flying_vessel(self):
        situation = SimpleNamespace(landed="landed", splashed="splashed")
        booster = SimpleNamespace(situation="flying", name="Codex booster_left",
                                  recover=Mock())
        sc = SimpleNamespace(VesselSituation=situation, active_vessel=object(),
                             vessels=[booster])
        with self.assertRaises(RuntimeError):
            recover_landed_vessel(sc, booster, "payload", True)
        booster.recover.assert_not_called()

    def test_auto_recover_switches_view_from_landed_booster(self):
        situation = SimpleNamespace(landed="landed", splashed="splashed")
        payload = object()
        booster = SimpleNamespace(situation="landed", name="Codex booster_left")
        sc = SimpleNamespace(VesselSituation=situation, active_vessel=booster,
                             vessels=[booster])
        booster.recover = Mock(side_effect=lambda: sc.vessels.clear())
        with patch("MyRecoverV2.find_booster", return_value=payload):
            recover_landed_vessel(sc, booster, "payload", True)
        self.assertIs(sc.active_vessel, payload)
        booster.recover.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
