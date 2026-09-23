"""三芯重型火箭发射、载荷交接与三路后台回收。

脚本在中央芯与上面级分离后立即把活动载具交给玩家，不控制载荷入轨。
任务优先级：节省载荷推进剂 > 两侧助推器返回 KSC > 中央芯级安全回收。
默认只做结构检查；加 --execute 才会分级、点火和控制载具。
"""
import argparse
import math
import time
from datetime import datetime

from FlightControl import ThrottleController, ready_vessel
from MyLaunchV2 import RecoveryManager, clamp


# 两个目标只沿跑道方向错开约 52 m。这样既避免两枚箭体争抢同一点，
# 编队看起来也不会过分分散。目标都在跑道附近的平地，远离发射塔。
RUNWAY_LEFT = (-0.04855, -74.72200)
RUNWAY_RIGHT = (-0.04855, -74.72700)
# 第 21 次正东弹道完整软着陆在 (约 -0.089°, -33.012°) 的海面。沿同一条
# 赤道航线向西约 57 km 是一片宽阔大陆，地形扫描确认目标点海拔约 277 m。
# 发射与载荷交接始终保持 90°正东；中央芯只靠已有再入点火更早减速来缩短
# 下程距离，不让回收落区改变载荷的轨道方向，也不增加发动机开机次数。
CORE_CONTINENT = (0.0, -38.5)
CORE_HANDOFF_ALTITUDE = 82500.0
CORE_HANDOFF_VERTICAL_SPEED = 120.0


def heavy_pitch(altitude):
    """适合当前三芯长箭体的连续重力转弯。

    500 m 前保持竖直越塔；12 km 时降到 50°；40 km 时接近 5°。
    每段用 smoothstep，使俯仰角和变化率都连续，减少大箭体弯曲振荡。
    """
    def smooth(value):
        value = clamp(value, 0.0, 1.0)
        return value * value * (3 - 2 * value)

    if altitude <= 500:
        return 90.0
    if altitude <= 12000:
        return 90.0 - 40.0 * smooth((altitude - 500) / 11500)
    return 50.0 - 45.0 * smooth((altitude - 12000) / 28000)


def core_push_pitch(altitude, vertical_speed, apoapsis_altitude,
                    thrust_accel, gravity, time_remaining,
                    target_altitude=CORE_HANDOFF_ALTITUDE,
                    target_vertical_speed=CORE_HANDOFF_VERTICAL_SPEED):
    """按远地点余量和剩余燃烧时间分配中央芯的竖直推力。

    直接强迫火箭在燃料用完时同时命中“75 km 高度”和“120 m/s 垂速”，
    会在目标已不可达时仍命令大俯仰角，实飞表现就是把远地点推到 120 km，
    却没有留下足够水平速度。这里改用更稳健的能量外环：

    1. 远地点尚未越过目标时，按远地点误差给出期望垂速；
    2. 远地点已经足够高时，把期望垂速收敛到 120 m/s；
    3. 用剩余燃烧时间把垂速误差换成净竖直加速度，其余推力用于正东加速。

    这样仍保证载荷飞出大气层，同时自然形成“先抬远地点、后压平”的重力
    转弯。函数只决定俯仰角；90°正东航向由外层保持。
    """
    values = (altitude, vertical_speed, apoapsis_altitude, thrust_accel,
              gravity, time_remaining, target_altitude, target_vertical_speed)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("中央芯时间制导输入必须是有限数")
    if thrust_accel <= 0 or gravity <= 0 or time_remaining <= 0:
        raise ValueError("推力加速度、重力和剩余时间必须为正")
    time_remaining = clamp(time_remaining, 2.0, 180.0)
    apoapsis_error = target_altitude - apoapsis_altitude
    desired_vertical_speed = clamp(
        target_vertical_speed + apoapsis_error / 80.0,
        target_vertical_speed, 550.0)
    # 最后数秒不再试图用不可能的大加速度纠正全部误差；8 秒下限使
    # 俯仰指令平滑收敛，也给长箭体姿态响应留出时间。
    correction_time = max(time_remaining, 8.0)
    net_vertical_accel = ((desired_vertical_speed - vertical_speed) /
                          correction_time)
    vertical_thrust_accel = gravity + net_vertical_accel
    sine_pitch = clamp(vertical_thrust_accel / thrust_accel, -1.0, 1.0)
    return clamp(math.degrees(math.asin(sine_pitch)), 5.0, 65.0)


def command_parts(vessel):
    return [p for p in vessel.parts.all
            if any(m.name == "ModuleCommand" for m in p.modules)]


def discover_heavy(vessel):
    """按连接分级和横向位置识别三芯，不依赖载具名称或列表顺序。"""
    commands = command_parts(vessel)
    sides = [p for p in commands if p.decouple_stage == 3]
    cores = [p for p in commands if p.decouple_stage == 2]
    payloads = [p for p in commands if p.decouple_stage == -1]
    if len(sides) != 2 or len(cores) != 1 or not payloads:
        raise RuntimeError(
            f"重型箭结构不匹配：侧芯控制器 {len(sides)}，中央芯控制器 {len(cores)}，"
            f"不随芯级分离的控制器 {len(payloads)}")
    sides.sort(key=lambda p: p.position(vessel.reference_frame)[0])
    payload = max(payloads, key=lambda p: p.position(vessel.reference_frame)[1])
    return sides[0], sides[1], cores[0], payload


def branch_parts(vessel, decouple_stage, side=0):
    parts = [p for p in vessel.parts.all if p.decouple_stage == decouple_stage]
    if side:
        parts = [p for p in parts
                 if p.position(vessel.reference_frame)[0] * side > 0]
    return parts


def liquid_fraction(parts):
    amount = capacity = 0.0
    for part in parts:
        for resource in part.resources.all:
            if resource.name == "LiquidFuel":
                amount += resource.amount
                capacity += resource.max
    if capacity <= 0:
        raise RuntimeError("芯级没有可读的液体燃料容量")
    return amount / capacity


def stage_liquid_fraction(vessel, decouple_stage):
    """从当前 Vessel 重新读取某一分离级的燃料比例。

    KSP 在分离、切换活动载具或改变物理加载状态时会重建 Vessel/Part。
    因此分离后的长时间闭环不能缓存旧 Part 对象；每次都从仍然有效的
    Vessel 查询该分离级资源，代价很小，却能避免半程出现空对象错误。
    """
    resources = vessel.resources_in_decouple_stage(decouple_stage, False)
    capacity = resources.max("LiquidFuel")
    if capacity <= 0:
        raise RuntimeError("当前芯级没有可读的液体燃料容量")
    return resources.amount("LiquidFuel") / capacity


def tag_heavy(left, right, core, payload):
    assignments = ((left, "booster_left"), (right, "booster_right"),
                   (core, "booster_core"), (payload, "payload"))
    for part, tag in assignments:
        part.tag = tag
        if part.tag != tag:
            raise RuntimeError(f"无法写入并回读部件标签 {tag}")


def engines_for(vessel, decouple_stage, side=0, propellant=None):
    result = [e for e in vessel.parts.engines if e.part.decouple_stage == decouple_stage]
    if side:
        result = [e for e in result
                  if e.part.position(vessel.reference_frame)[0] * side > 0]
    if propellant is not None:
        result = [e for e in result
                  if any(p.name == propellant for p in e.propellants)]
    return result


def wait_split(anchors, parent_anchor, timeout=12.0):
    """等待若干部件脱离父级，并返回各自的新 Vessel。"""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        parent = parent_anchor.vessel
        vessels = [part.vessel for part in anchors]
        if all(v != parent for v in vessels) and len(set(vessels)) == len(vessels):
            return vessels
        time.sleep(.05)
    raise RuntimeError("分级已触发，但未确认各芯级成为独立载具；不会重复分级")


def post_separation_view(viewed_vessel, combined_vessel, upper_vessel,
                         existing_vessels):
    """玩家原先在看侧芯时保留原镜头；原先看主箭时把控制交给载荷。"""
    if (viewed_vessel != combined_vessel and
            viewed_vessel in existing_vessels):
        return viewed_vessel
    return upper_vessel


def recovery_args(tag, leg_offset, *, target=None, allow_warp=False,
                   no_reentry_burn=False, no_boostback=False,
                   boostback_cutoff_height=22000,
                   physics_range=2000000, reentry_off_speed=None,
                   reentry_altitude=None,
                   boostback_return_gain=None,
                   max_tilt=None, terminal_tilt=None,
                   gear_lead_seconds=None, gear_max_height=None,
                   grid_retract_height=None, require_land=False,
                   aero_target_tilt=None):
    args = ["--execute", "--background", "--booster-tag", tag,
            "--payload-tag", "payload", "--leg-offset", str(leg_offset),
            "--landing-surface-offset", "0", "--expected-engines", "7"]
    args += ["--boostback-cutoff-height", str(boostback_cutoff_height)]
    # 当前东向弹道在载荷交接后会迅速离 KSC 超过 1600 km。400 km 已实飞
    # 证明会在切到载荷/中央芯后删除落地侧芯，因此四个分离载具统一保持
    # 2000 km 物理范围，直到玩家手动回收。脚本仍不会自动回收或删除载具。
    args += ["--physics-range", str(physics_range)]
    if reentry_off_speed is not None:
        args += ["--reentry-off-speed", str(reentry_off_speed)]
    if reentry_altitude is not None:
        args += ["--reentry-altitude", str(reentry_altitude)]
    if aero_target_tilt is not None:
        args += ["--aero-target-tilt", str(aero_target_tilt)]
    if boostback_return_gain is not None:
        args += ["--boostback-return-gain", str(boostback_return_gain)]
    if max_tilt is not None:
        args += ["--max-tilt", str(max_tilt)]
    if terminal_tilt is not None:
        args += ["--terminal-tilt", str(terminal_tilt)]
    if gear_lead_seconds is not None:
        args += ["--gear-deploy-lead-seconds", str(gear_lead_seconds)]
    if gear_max_height is not None:
        args += ["--gear-deploy-max-height", str(gear_max_height)]
    if grid_retract_height is not None:
        args += ["--grid-retract-height", str(grid_retract_height)]
    if target is not None:
        args += ["--target-latitude", str(target[0]),
                  "--target-longitude", str(target[1]), "--require-land"]
    elif require_land:
        # 中央芯采用自然下程落区，不做耗油的返场闭环；仍要求最终状态必须
        # 是 landed，使落水、平台未加载或弹道偏离都明确判为失败。
        args.append("--require-land")
    if allow_warp:
        args.append("--allow-warp")
    if no_reentry_burn:
        args.append("--no-reentry-burn")
    if no_boostback:
        args.append("--no-boostback")
    return args


def restore_one_x(sc):
    if sc.rails_warp_factor or sc.physics_warp_factor:
        sc.rails_warp_factor = 0
        sc.physics_warp_factor = 0


def fly(args, conn, vessel, left, right, core, payload):
    sc = conn.space_center
    body = vessel.orbit.body
    surface = vessel.flight(body.reference_frame)
    inertial = body.non_rotating_reference_frame
    ap = vessel.auto_pilot
    throttle = None
    manager = RecoveryManager()
    separated_side = separated_core = False
    left_parts = branch_parts(vessel, 3, -1)
    right_parts = branch_parts(vessel, 3, 1)
    core_parts = branch_parts(vessel, 2)
    start_ut = sc.ut
    # 用户要求载荷保持正东轨道，因此上升和中央芯推送全程固定 90°航向。
    # 中央芯落区只由分离后的回收制导调整，绝不借发射航向改变载荷轨道面。
    ascent_heading = 90.0
    try:
        restore_one_x(sc)
        ap.reference_frame = vessel.surface_reference_frame
        ap.target_pitch_and_heading(90, ascent_heading)
        ap.engaged = True
        vessel.control.sas = False
        throttle = ThrottleController(vessel)
        throttle.set(1)
        vessel.control.activate_next_stage()  # 5 -> 4：执行界面中的第 4 级，21 台一级发动机点火
        if vessel.control.current_stage != 4:
            raise RuntimeError("起飞后 current_stage 不是 4，停止任务")
        print("HEAVY ASCENT：三芯点火；侧芯按燃料储备分离；"
              "载荷保持 90°正东轨道")
        last_log = -1e9
        while min(liquid_fraction(left_parts), liquid_fraction(right_parts)) > args.side_reserve:
            # 正式运行默认禁止加速；测试或用户显式传入 --allow-warp 时，
            # 只允许玩家手动选择倍率，脚本本身从不主动加速。
            if not args.allow_warp:
                restore_one_x(sc)
            altitude = surface.mean_altitude
            pitch = heavy_pitch(altitude)
            ap.target_pitch_and_heading(pitch, ascent_heading)
            available = vessel.available_thrust
            if available <= 0 and sc.ut - start_ut > 4:
                raise RuntimeError("一级发动机没有可用推力")
            # 限制加速度，同时在高动压区温和收油；避免大箭体结构过载。
            accel_cap = min(1.0, 24 * vessel.mass / max(available, 1))
            q_cap = clamp(45000 / max(surface.dynamic_pressure, 1), .55, 1)
            throttle.set(min(accel_cap, q_cap))
            if sc.ut - last_log >= 1:
                print(f"ASCENT h={altitude:.0f}m pitch={pitch:.1f}° "
                      f"apo={vessel.orbit.apoapsis_altitude:.0f}m "
                      f"side={100*liquid_fraction(left_parts):.1f}% "
                      f"core={100*liquid_fraction(core_parts):.1f}%")
                last_log = sc.ut
            time.sleep(.05)

        # 短暂关机分离，避免 14 台侧芯发动机推力把它们压回中央芯级。
        # 在任何 Part 代理失效前保存分离瞬间的读数。KSP 分级会重建 Vessel，
        # 原 Vessel 下取得的 Part/Resources 代理此后不能继续使用。
        left_reserve = liquid_fraction(left_parts)
        right_reserve = liquid_fraction(right_parts)
        throttle.set(0)
        time.sleep(.4)
        vessel.control.activate_next_stage()  # 4 -> 3：执行第 3 级，左右侧芯同时分离
        side_vessels = wait_split((left, right), core)
        separated_side = True
        print(f"SIDE SEP：左右助推器已分离，剩余燃料 "
              f"{100*left_reserve:.1f}% / {100*right_reserve:.1f}%")
        # 两个回收线程仍独立判断安全间距、转向和制导；共享栅栏只卡在
        # 第一次非零返场油门之前。两边都对准后才一起亮火，既保留分离
        # 防撞，也让三芯同框的返场点火画面严格同步。
        side_ignition_barrier = manager.ignition_barrier(2)
        manager.start("booster_left", recovery_args(
            "booster_left", args.leg_offset, target=RUNWAY_LEFT,
            allow_warp=args.allow_warp, boostback_cutoff_height=8000,
            # 返场点火只负责把下程速度大致送回KSC；此前关闭气动瞄准后，
            # 返推误差会原样保留到低空，连续实飞出现 0.3--2.5 km 离散。
            # 栅格舵在下降段按实时预测落点修正，不增加发动机开机次数。
            aero_target_tilt=5,
            # 第 24 次 1.05 落到目标以东约 1.15 km；第 25 次 1.12 又落到
            # 目标以西约 0.79 km。线性插值取 1.08，目标是落到跑道本体。
            boostback_return_gain=1.08),
            selected_vessel=side_vessels[0],
            ignition_barrier=side_ignition_barrier)
        manager.start("booster_right", recovery_args(
            "booster_right", args.leg_offset, target=RUNWAY_RIGHT,
            allow_warp=args.allow_warp, boostback_cutoff_height=8000,
            aero_target_tilt=5,
            # 第 24 次 1.06 落到目标以东约 0.82 km；第 25 次 1.13 落到
            # 目标以西约 0.65 km。插值取 1.09，并继续独立标定非对称误差。
            boostback_return_gain=1.09),
            selected_vessel=side_vessels[1],
            ignition_barrier=side_ignition_barrier)

        # 中央芯继续工作，把其余推进剂（扣除最低着陆储备）全部交给载荷。
        core_vessel = core.vessel
        # 分离后必须从中央芯所在的新 Vessel 重新解析部件。复用起飞前的
        # core_parts 会触发 kRPC 的空对象错误，并让中央芯和载荷同时失控。
        core_parts = branch_parts(core_vessel, 2)
        if not core_parts:
            raise RuntimeError("侧芯分离后无法重新识别中央芯燃料部件")
        # 侧芯分离后玩家可以切过去观察；中央芯与载荷组合仍必须继续进行
        # 物理模拟和主发动机推力，直到中央芯分离完成。
        core_vessel.physics_range = 2000000.0
        if core_vessel.physics_range < 1990000.0:
            raise RuntimeError("中央芯推送阶段的临时物理范围设置未生效")
        sc.active_vessel = core_vessel
        core_vessel.parts.controlling = core
        ap = core_vessel.auto_pilot
        ap.reference_frame = core_vessel.surface_reference_frame
        ap.target_pitch_and_heading(
            heavy_pitch(core_vessel.flight(body.reference_frame).mean_altitude),
            ascent_heading)
        ap.engaged = True
        throttle.close()
        throttle = ThrottleController(core_vessel, force_independent=True)
        throttle.set(1)
        print("CORE PUSH：中央芯继续推载荷，直到最低着陆储备")
        last_log = -1e9
        # 用燃料比例的实测下降率估算剩余燃烧时间。0.0088/s 是此前实飞的
        # 保守初值；几帧后即由本次任务的实际流量低通更新，不依赖发动机型号。
        fuel_rate = 0.0088
        last_fuel = stage_liquid_fraction(core_vessel, 2)
        last_fuel_ut = sc.ut
        while last_fuel > args.core_reserve:
            if not args.allow_warp:
                restore_one_x(sc)
            now_ut = sc.ut
            current_fuel = stage_liquid_fraction(core_vessel, 2)
            sample_dt = now_ut - last_fuel_ut
            if sample_dt > 0.02:
                sample_rate = (last_fuel - current_fuel) / sample_dt
                if 0 < sample_rate < 0.05:
                    fuel_rate = 0.9 * fuel_rate + 0.1 * sample_rate
                last_fuel = current_fuel
                last_fuel_ut = now_ut
            time_remaining = max(
                2.0, (current_fuel - args.core_reserve) /
                max(fuel_rate, 1e-5))
            core_flight = core_vessel.flight(body.reference_frame)
            core_altitude = core_flight.mean_altitude
            radius = body.equatorial_radius + core_altitude
            gravity = body.gravitational_parameter / (radius * radius)
            available = core_vessel.available_thrust
            commanded_accel = min(
                24.0, available / max(core_vessel.mass, 1.0))
            push_pitch = core_push_pitch(
                core_altitude, core_flight.vertical_speed,
                core_vessel.orbit.apoapsis_altitude, commanded_accel,
                gravity, time_remaining)
            ap.target_pitch_and_heading(push_pitch, ascent_heading)
            throttle.set(min(1.0, 24 * core_vessel.mass / max(available, 1)))
            if sc.ut - last_log >= 1:
                print(f"CORE h={core_altitude:.0f}m "
                      f"apo={core_vessel.orbit.apoapsis_altitude:.0f}m "
                      f"vs={core_flight.vertical_speed:.0f}m/s "
                      f"pitch={push_pitch:.1f}° tgo={time_remaining:.1f}s "
                      f"peri={core_vessel.orbit.periapsis_altitude:.0f}m "
                      f"fuel={100*current_fuel:.1f}%")
                last_log = sc.ut
            time.sleep(.05)

        core_remaining = stage_liquid_fraction(core_vessel, 2)
        throttle.set(0)
        ap.engaged = False
        throttle.close()
        throttle = None
        # 先记住玩家正在看的载具，再短暂把中央芯设为活动载具并执行正常分级。
        # KSP 的分级器会同时完成分离器、分离小火箭和控制点的状态迁移；直接调用
        # Decoupler.decouple() 会绕过其中一部分流程，并可能立即销毁缓存的 Part
        # 代理，实飞中曾使总控在线程启动前崩溃。这里接受一次极短的镜头闪切，
        # 等两枚新 Vessel 稳定后马上恢复玩家原先观察的侧芯；若原先看主箭，
        # 则把活动载具交给载荷。
        viewed_vessel = sc.active_vessel
        if sc.active_vessel != core_vessel:
            sc.active_vessel = core_vessel
        core_vessel.control.activate_next_stage()
        (core_only,) = wait_split((core,), payload)
        separated_core = True
        upper = payload.vessel
        restored_view = post_separation_view(
            viewed_vessel, core_vessel, upper, sc.vessels)
        if sc.active_vessel != restored_view:
            sc.active_vessel = restored_view

        # 分离后用载荷所在的新 Vessel 读取交接状态。旧的组合体 Vessel 在不同
        # KSP/kRPC版本中可能继续指向中央芯，也可能指向载荷，不能再依赖它。
        handoff_apoapsis = upper.orbit.apoapsis_altitude
        handoff_flight = upper.flight(body.reference_frame)
        handoff_vertical_speed = handoff_flight.vertical_speed
        handoff_horizontal_speed = handoff_flight.horizontal_speed
        handoff_path_angle = math.degrees(math.atan2(
            handoff_vertical_speed, max(handoff_horizontal_speed, 1e-6)))
        handoff_heading = upper.flight(upper.surface_reference_frame).heading
        print(f"CORE SEP：中央芯保留 {100*core_remaining:.1f}% 燃料，就近回收；"
              f"载荷远地点 {handoff_apoapsis:.0f}m，垂直速度 "
              f"{handoff_vertical_speed:.0f}m/s，水平速度 "
              f"{handoff_horizontal_speed:.0f}m/s，"
              f"飞行路径角 {handoff_path_angle:.1f}°，航向 {handoff_heading:.1f}°")
        # 中央芯不做返场。它的横向速度远高于侧芯，因此仍用一次再入点火
        # 防热，但在 1100 m/s 关机，少消耗一点着陆燃料。第 19 次在 1.8 km
        # 才收栅格舵，触水后仍损失 1 片；中央芯改在 3.5 km 收舵，给动画和
        # 结构卸载留出更充分的时间。此次必须落地，溅落不会再被接受。
        manager.start("booster_core", recovery_args(
            "booster_core", args.leg_offset, target=CORE_CONTINENT,
            allow_warp=args.allow_warp,
            # 第 22 次提前到 65 km 会延长动力减速，并在旧控制律下重复点火。
            # 新弹道本身负责缩短下程；再入点火恢复到 45 km 才启动，速度降到
            # 1180 m/s 就关机，其余交给大气和栅格舵。控制器另有一次性锁存，
            # 因而整个再入段最多只启动一次。
            reentry_off_speed=1220, reentry_altitude=45000,
            # 中央芯依靠发射弹道自然下程，显式禁止返场点火。第 24 次把高度
            # 阈值设为 100 km，反而在越过 100 km 后触发返推，白白消耗约 6%
            # 燃料并把水平速度从约 843 m/s 压到 220 m/s；本开关消除歧义。
            no_boostback=True, aero_target_tilt=25,
            max_tilt=28, terminal_tilt=10,
            gear_lead_seconds=15, gear_max_height=2500,
            grid_retract_height=3500),
            selected_vessel=core_only)

        # 到这里自动发射任务结束。确保载荷油门为零，但不改变控制点、
        # 不点火上面级、不接管姿态或规划入轨。用户可以
        # 更换任意载荷，只要仍有一个不随芯级分离的 ModuleCommand 供识别。
        upper.control.throttle = 0.0
        # 当前载荷已经由 KSP 保持物理模拟，额外扩大物理范围会增加结构抖动。
        # 切去看芯级时，PRE 的载具切换事件会给后台载荷恢复远距范围。
        if sc.active_vessel == upper:
            upper.physics_range = 2500.0
        else:
            upper.physics_range = 2000000.0
            if upper.physics_range < 1990000.0:
                raise RuntimeError("后台载荷临时物理范围设置未生效，禁止交接")
        try:
            upper.auto_pilot.engaged = False
        except Exception:
            pass
        view_note = ("保持玩家原先观察的载具" if restored_view != upper
                     else "活动载具为载荷")
        print(f"MANUAL HANDOFF：已与中央芯分离；{view_note}。"
              f"当前远地点 {upper.orbit.apoapsis_altitude:.0f}m，"
              f"上面级尚未点火，后续入轨完全由玩家操作。"
              f"三枚芯级后台回收继续运行。")
        manager.wait()
        # 依照任务约定不自动缩小任何载具的物理范围。这样落地芯级不会因
        # 玩家继续操纵远处载荷而被 KSP 卸载；由玩家手动回收载具结束其生命周期。
        print("全部后台回收线程已结束；所有现存载具继续保持 2000 km 物理范围，"
              "直到玩家手动回收。")
    finally:
        if throttle is not None:
            throttle.close()
        # 分离后的回收线程拥有各自芯级，不能在这里清空它们的油门。
        if not separated_core:
            try:
                vessel.control.throttle = 0
                vessel.auto_pilot.engaged = False
            except Exception:
                pass


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--side-reserve", type=float, default=.16,
                        help="左右助推器分离时的燃料比例，默认 16%%")
    parser.add_argument("--core-reserve", type=float, default=.12,
                        help="中央芯再入防热与着陆燃料比例，默认 12%%")
    parser.add_argument("--leg-offset", type=float, default=16)
    parser.add_argument("--allow-warp", action="store_true",
                        help="允许玩家在滑行段手动加速；脚本从不主动加速")
    args = parser.parse_args(argv)
    if not (0.12 <= args.side_reserve <= .35):
        parser.error("侧助推器储备须在 12% 到 35% 之间")
    if not (.08 <= args.core_reserve < args.side_reserve):
        parser.error("中央芯储备须低于侧芯，且至少 8%")

    import krpc
    conn = krpc.connect(name="Heavy launch and triple recovery")
    try:
        sc = conn.space_center
        vessel = ready_vessel(sc)
        left, right, core, payload = discover_heavy(vessel)
        counts = (len(engines_for(vessel, 3, -1, "LiquidFuel")),
                  len(engines_for(vessel, 3, 1, "LiquidFuel")),
                  len(engines_for(vessel, 2, propellant="LiquidFuel")),
                  len(engines_for(vessel, -1, propellant="LiquidFuel")))
        separators = (len(engines_for(vessel, 3, -1, "SolidFuel")),
                      len(engines_for(vessel, 3, 1, "SolidFuel")))
        print(f"识别结果：左/右/中央/上面级液体发动机 = {counts}；"
              f"左右分离小火箭 = {separators}")
        if counts != (7, 7, 7, 4):
            raise RuntimeError("液体发动机分组不是预期的 7/7/7/4，禁止自动分级")
        if separators != (4, 4):
            raise RuntimeError("分离小火箭不是左右各 4 台，禁止自动分级")
        print(f"燃料：左 {100*liquid_fraction(branch_parts(vessel,3,-1)):.1f}% / "
              f"右 {100*liquid_fraction(branch_parts(vessel,3,1)):.1f}% / "
              f"中央 {100*liquid_fraction(branch_parts(vessel,2)):.1f}%")
        for name, point in (("左侧回收点", RUNWAY_LEFT), ("右侧回收点", RUNWAY_RIGHT)):
            terrain = vessel.orbit.body.surface_height(*point)
            print(f"{name}: {point[0]}, {point[1]}，地形海拔 {terrain:.1f}m")
        core_terrain = vessel.orbit.body.surface_height(*CORE_CONTINENT)
        print(f"中央芯正东下程大陆目标: {CORE_CONTINENT[0]}, {CORE_CONTINENT[1]}，"
              f"地形海拔 {core_terrain:.1f}m")
        if not args.execute:
            print("只读检查通过；加 --execute 才会写标签、保存备份并发射。")
            return
        if vessel.situation != sc.VesselSituation.pre_launch or vessel.control.current_stage != 5:
            raise RuntimeError("必须处于发射前状态且 current_stage 为 5（界面下一次执行第 4 级）")
        tag_heavy(left, right, core, payload)
        backup = "codex-heavy-prelaunch-" + datetime.now().strftime("%Y%m%d-%H%M%S-%f")
        sc.save(backup)
        print(f"已保存重型箭发射前备份：{backup}")
        fly(args, conn, vessel, left, right, core, payload)
    finally:
        conn.close()


if __name__ == "__main__":
    main()
