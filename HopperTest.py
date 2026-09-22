"""低空“蚱蜢跳”着陆验证。

不进行载荷分离或亚轨道飞行：一级从发射架短暂升至 --hop-altitude，
关闭发动机后交给 MyRecoverV2 的末端导航落到指定点。屋顶测试必须传入
CalibrateLandingSite.py 标定出的经纬度和楼顶高度。
"""
import argparse
import math
import time

from FlightControl import ThrottleController, ready_vessel, deploy_grid_fins, grid_fins_deployed
from MyLaunchV2 import KSC_LATITUDE, KSC_LONGITUDE
from MyRecoverV2 import main as recover


def transfer_acceleration(distance, bearing, north_speed, east_speed,
                          horizontal_accel=2.0, response_time=8.0):
    """位置外环 -> 速度内环，返回北/东加速度（m/s²）。

    v²/(2a) 是横向停车距离；额外预留 response_time*v 给箭体转向。
    反解 distance = v²/(2a) + response_time*v 得到允许接近速度，
    因此高速接近时会提前向后倾斜刹车，而不是到点才抬头。
    近目标再限制为 0.04*distance，使目标速度连续归零，避免反复穿越。
    """
    a = horizontal_accel
    desired_speed = min(18.0, .04 * distance,
                        math.sqrt((a * response_time) ** 2 + 2 * a * distance)
                        - a * response_time)
    angle = math.radians(bearing)
    north = .2 * (desired_speed * math.cos(angle) - north_speed)
    east = .2 * (desired_speed * math.sin(angle) - east_speed)
    scale = min(1.0, a / max(math.hypot(north, east), 1e-9))
    return north * scale, east * scale


def surface_distance_and_bearing(body_radius, lat1, lon1, lat2, lon2):
    """返回球面距离和从当前位置指向目标的航向角（度）。"""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    a = (math.sin((p2 - p1) / 2) ** 2 + math.cos(p1) * math.cos(p2) *
         math.sin(dl / 2) ** 2)
    distance = 2 * body_radius * math.asin(min(1.0, math.sqrt(a)))
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    bearing = math.degrees(math.atan2(y, x)) % 360
    return distance, bearing


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--hop-altitude", type=float, default=800.0)
    parser.add_argument("--landing-latitude", type=float, default=KSC_LATITUDE)
    parser.add_argument("--landing-longitude", type=float, default=KSC_LONGITUDE)
    parser.add_argument("--landing-surface-offset", type=float, default=0.0)
    parser.add_argument("--leg-offset", type=float, default=16.0)
    parser.add_argument("--grid-group", type=int, default=8)
    args = parser.parse_args()
    if args.hop_altitude < 150 or not args.execute:
        parser.error("--execute 且 --hop-altitude 至少 150 m 后才会接管")
    import krpc
    conn = krpc.connect(name="Hopper landing test")
    throttle = None
    ap = None
    try:
        sc = conn.space_center
        vessel = ready_vessel(sc)
        if vessel.situation != sc.VesselSituation.pre_launch:
            raise RuntimeError("蚱蜢跳只能从发射架 pre_launch 状态开始")
        expected_engines = len(vessel.parts.engines)
        # 低空实验在点火前展开并核实，单独验证气动控制配置。
        # 正式轨道发射仍由回收阶段展开，不能照搬本试验时序。
        fins = deploy_grid_fins(vessel)
        fin_deadline = time.monotonic() + 8
        while fins and grid_fins_deployed(fins) != len(fins) and time.monotonic() < fin_deadline:
            time.sleep(.2)
        if len(fins) != 4 or grid_fins_deployed(fins) != 4:
            raise RuntimeError("屋顶实验要求四片栅格舵完整展开，起飞已取消")
        print("起飞前栅格舵展开并锁定：4/4")
        legs = list(vessel.parts.legs)
        if not legs or any(leg.deployed or not leg.deployable for leg in legs):
            raise RuntimeError("起飞前支腿没有全部处于可展开的收回状态")
        flight = vessel.flight(vessel.surface_reference_frame)
        launch_altitude = flight.mean_altitude
        throttle = ThrottleController(vessel)
        ap = vessel.auto_pilot
        ap.reference_frame = vessel.surface_reference_frame
        ap.target_pitch_and_heading(90, 90)
        ap.engaged = True
        vessel.control.activate_next_stage()
        # 本箭体满载时 35% 油门低于 1g，无法脱离发射卡箍。点火后按实际
        # 推重比取至少 1.25g 的起飞加速度，优先迅速越过发射塔高度。
        deadline = time.monotonic() + 3
        while vessel.available_thrust <= 0 and time.monotonic() < deadline:
            time.sleep(.05)
        radius = vessel.orbit.body.equatorial_radius + flight.mean_altitude
        gravity = vessel.orbit.body.gravitational_parameter / radius ** 2
        required = 1.25 * vessel.mass * gravity / max(vessel.available_thrust, 1)
        lift_throttle = min(1.0, max(.75, required))
        throttle.set(lift_throttle)
        print(f"HOP：起飞，目标 {args.hop_altitude:.0f} m，起飞油门 {lift_throttle:.2f}")
        last_log = -1e9
        start_ut = sc.ut
        settled_since = None
        while True:
            if sc.ut - start_ut > 180:
                raise RuntimeError("上升横移超时，未满足着陆接管条件")
            if sc.rails_warp_factor or sc.physics_warp_factor:
                sc.rails_warp_factor = 0
                sc.physics_warp_factor = 0
            if vessel.available_thrust <= 0:
                raise RuntimeError("蚱蜢跳阶段失去推力")
            distance, bearing = surface_distance_and_bearing(
                vessel.orbit.body.equatorial_radius,
                flight.latitude, flight.longitude,
                args.landing_latitude, args.landing_longitude)
            # 海拔差不会因飞过屋顶发生跳变；先竖直越过发射塔再横移。
            height = flight.mean_altitude - launch_altitude
            # surface_reference_frame 的原点随箭体平移，自身 velocity 为零！
            # 在天体固连系测地速，再用 transform_direction 仅旋转向量分量。
            up_speed, north_speed, east_speed = sc.transform_direction(
                vessel.velocity(vessel.orbit.body.reference_frame),
                vessel.orbit.body.reference_frame, vessel.surface_reference_frame)
            north, east = transfer_acceleration(distance, bearing, north_speed, east_speed)
            if height < 140:
                north, east = 0.0, 0.0
            # 接近目标高度逐渐降低上升速度，横移未收敛时短暂保持高度。
            # 这只是定点能力验证剖面，并不是最终追求最小 Δv 的发射剖面。
            goal_vs = max(0.0, min(35.0, .25 * (args.hop_altitude - height)))
            radius = vessel.orbit.body.equatorial_radius + flight.mean_altitude
            gravity = vessel.orbit.body.gravitational_parameter / radius ** 2
            vertical = max(1.0, gravity + .6 * (goal_vs - up_speed))
            lateral = math.hypot(north, east)
            scale = min(1.0, vertical * math.tan(math.radians(20)) / max(lateral, 1e-9))
            north, east = north * scale, east * scale
            ap.target_direction = (vertical, north, east)
            alignment = max(.5, vessel.direction(vessel.surface_reference_frame)[0])
            throttle.set(vertical * vessel.mass / (vessel.available_thrust * alignment))
            ready = (height > args.hop_altitude - 20 and distance < 10 and
                     math.hypot(north_speed, east_speed) < 1.0 and abs(up_speed) < 3)
            settled_since = (settled_since if settled_since is not None else sc.ut) if ready else None
            if settled_since is not None and sc.ut - settled_since >= 2:
                break
            if sc.ut - last_log >= 1:
                print(f"HOP-TRANSFER 相对发射点高度={height:.0f}m "
                      f"目标距离={distance:.1f}m 水平速度={math.hypot(north_speed,east_speed):.1f}m/s "
                      f"垂速={up_speed:.1f}m/s 横向加速度={math.hypot(north,east):.1f}m/s²")
                last_log = sc.ut
            time.sleep(.05)
        throttle.set(0)
        ap.engaged = False
        throttle.close()
        print("HOP：达到高度，交给单次着陆点火控制")
        # 起飞后 KSP 的部件列表可能在物理加载间短暂不完整；把起飞前
        # 已知发动机数传给回收器，残骸不能绕过完整性验收。
        recover(["--execute", "--leg-offset", str(args.leg_offset),
                 "--landing-surface-offset", str(args.landing_surface_offset),
                 "--target-latitude", str(args.landing_latitude),
                 "--target-longitude", str(args.landing_longitude),
                 "--require-land", "--grid-group", str(args.grid_group),
                 "--expected-engines", str(expected_engines)],
                connection=conn, selected_vessel=vessel)
    finally:
        if throttle is not None:
            throttle.close()
        if ap is not None:
            ap.engaged = False
        conn.close()


if __name__ == "__main__":
    main()
