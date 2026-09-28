"""两对侧芯加中央芯的东向发射：默认只读预检，--execute 才发射。

第 10 级共同点火，两对分别在第 9、8 级分离；中央芯继续尽量推送载荷，
第 7 级交给玩家。仅四枚侧芯后台回收，中央芯不预留着陆燃料。
实际级号由载具预检确定，不硬编码 10/9/8/7。
"""
import argparse
import math
import time
from datetime import datetime

from FlightControl import ThrottleController, ready_vessel
from MyLaunchV2 import RecoveryManager, clamp
from MyHeavyLaunch import (activate_checked_stage, branch_parts, command_parts,
                           core_push_pitch, heavy_pitch,
                           liquid_fraction, post_separation_view, recovery_args,
                           restore_one_x, stage_liquid_fraction, wait_split)


# 四个目标沿跑道方向排开。点位只作为返场初值，实际落点须实飞标定。
RUNWAY_TARGETS = ((-0.04855, -74.7130), (-0.04855, -74.7205),
                  (-0.04855, -74.7280), (-0.04855, -74.7355))
SIDE_TAGS = tuple(f"booster_side_{i+1}" for i in range(4))


def upper_engine_at_core_split(engine, core_stage):
    """中央芯分离同级激活的上面级发动机只能在交接后由玩家给油门。"""
    props = {p.name for p in engine.propellants}
    return (engine.part.stage == core_stage and
            engine.part.decouple_stage < core_stage and
            props == {"LiquidFuel", "Oxidizer"})


def side_layout(vessel, anchors):
    """按横截面方位角给四枚侧芯编号，不依赖两对安装在 X 还是 Z 轴。

    Y 是箭体纵向，因此只比较 X/Z。两对必须在同一级内对置，
    避免先分离一侧造成失衡。
    """
    if len(anchors) != 4:
        raise RuntimeError("必须有四枚独立侧芯控制器")
    positions = [p.position(vessel.reference_frame) for p in anchors]
    cx = sum(x for x, _, _ in positions) / 4
    cz = sum(z for _, _, z in positions) / 4
    radii = [math.hypot(x-cx, z-cz) for x, _, z in positions]
    if min(radii) < 1:
        raise RuntimeError("侧芯控制器离中心太近，无法可靠分组")
    ordered = sorted(zip(anchors, positions),
                     key=lambda item: math.atan2(item[1][2]-cz,
                                                  item[1][0]-cx))
    angles = [math.atan2(pos[2]-cz, pos[0]-cx) for _, pos in ordered]
    gaps = [(angles[(i+1)%4]-angles[i]) % (2*math.pi) for i in range(4)]
    if min(gaps) < math.radians(20):
        raise RuntimeError("侧芯方位过近，无法区分四路回收")
    result = tuple(part for part, _ in ordered)
    stages = sorted({p.decouple_stage for p in result}, reverse=True)
    if len(stages) != 2 or any(
            sum(p.decouple_stage == stage for p in result) != 2
            for stage in stages):
        raise RuntimeError("侧芯必须分为连续两级，每级对称分离两枚")
    if stages[0] != stages[1]+1:
        raise RuntimeError("两对侧芯的分离级必须连续")
    for stage in stages:
        pair = [p for p in result if p.decouple_stage == stage]
        a, b = (p.position(vessel.reference_frame) for p in pair)
        va, vb = (a[0]-cx, a[2]-cz), (b[0]-cx, b[2]-cz)
        cosine = (va[0]*vb[0]+va[1]*vb[1]) / (
            math.hypot(*va)*math.hypot(*vb))
        if cosine > -.5:
            raise RuntimeError(f"第 {stage} 级的两枚侧芯不是对置布局")
    return result


def nearest_side(vessel, part, anchors):
    """只按横截面距离归属部件，忽略发动机与控制器的纵向高度差。"""
    x, _, z = part.position(vessel.reference_frame)
    candidates = [i for i,a in enumerate(anchors)
                  if a.decouple_stage == part.decouple_stage]
    if not candidates:
        raise RuntimeError("部件不属于任何侧芯分离级")
    return min(candidates, key=lambda i: (
        (x - anchors[i].position(vessel.reference_frame)[0]) ** 2 +
        (z - anchors[i].position(vessel.reference_frame)[2]) ** 2))


def side_parts(vessel, anchors, index):
    """只读取该侧芯自己的分离归属和横向位置。"""
    stage = anchors[index].decouple_stage
    return [p for p in vessel.parts.all
            if p.decouple_stage == stage and
            nearest_side(vessel, p, anchors) == index]


def discover_superheavy(vessel):
    """识别先后两对侧芯、中央芯及载荷，不按根部件猜中央芯。"""
    groups = {}
    for part in command_parts(vessel):
        groups.setdefault(part.decouple_stage, []).append(part)
    matches = []
    for first_stage, first_pair in groups.items():
        second_stage, core_stage = first_stage-1, first_stage-2
        if first_stage < 2 or len(first_pair) != 2:
            continue
        if len(groups.get(second_stage, [])) != 2:
            continue
        if not {core_stage, -1}.issubset(groups):
            continue
        for core_group, payload_group in ((core_stage, -1), (-1, core_stage)):
            cores, payloads = groups[core_group], groups[payload_group]
            core_engines = [e for e in vessel.parts.engines
                            if e.part.decouple_stage == core_group and
                            any(p.name == "LiquidFuel" for p in e.propellants) and
                            e.part.stage == first_stage+1]
            if len(cores) == 1 and core_engines and payloads:
                anchors = side_layout(vessel, first_pair+groups[second_stage])
                payload = max(payloads, key=lambda p: p.position(
                    vessel.reference_frame)[1])
                matches.append((anchors, cores[0], payload))
    if len(matches) != 1:
        detail = ", ".join(f"级 {stage}: {len(parts)} 控制器"
                           for stage, parts in sorted(groups.items()))
        raise RuntimeError("无法唯一识别先后两对侧芯、中央芯及载荷；"+detail)
    return matches[0]


def validate_superheavy(vessel, anchors, core, *, side_engines,
                        core_engines, separation_motors):
    """只读验证点火→首对→后对→中央四次分级，不通过则不点火。"""
    first_stage = max(p.decouple_stage for p in anchors)
    second_stage, core_stage = first_stage-1, first_stage-2
    core_group = core.decouple_stage
    ignition_stage = first_stage+1
    if ([p.decouple_stage for p in anchors].count(first_stage) != 2 or
            [p.decouple_stage for p in anchors].count(second_stage) != 2 or
            core_group not in (-1, core_stage)):
        raise RuntimeError("两对侧芯或中央芯分离级不匹配")
    if vessel.control.current_stage != ignition_stage+1:
        raise RuntimeError(f"下一次必须是点火级 {ignition_stage}；"
                           f"当前 current_stage={vessel.control.current_stage}")
    liquid, solids = [0]*4, [0]*4
    core_liquid = 0
    for engine in vessel.parts.engines:
        part = engine.part
        props = {p.name for p in engine.propellants}
        if part.decouple_stage in (first_stage, second_stage):
            i = nearest_side(vessel, part, anchors)
            if anchors[i].decouple_stage != part.decouple_stage:
                raise RuntimeError("侧芯发动机横向位置与分离级不一致")
            if "LiquidFuel" in props:
                liquid[i] += 1
                if part.stage != ignition_stage:
                    raise RuntimeError(f"侧芯 {i+1} 液体发动机不在共同点火级")
            elif "SolidFuel" in props:
                solids[i] += 1
                if part.stage != part.decouple_stage:
                    raise RuntimeError(f"侧芯 {i+1} 分离小火箭不在本芯分离级")
            else:
                raise RuntimeError(f"侧芯 {i+1} 存在未知发动机")
        elif part.decouple_stage == core_group and "LiquidFuel" in props:
            core_liquid += 1
            if part.stage != ignition_stage:
                raise RuntimeError("中央芯液体发动机不在共同点火级")
        elif upper_engine_at_core_split(engine, core_stage):
            if getattr(engine, "active", False):
                raise RuntimeError("上面级发动机已提前启动")
        elif part.stage >= core_stage:
            raise RuntimeError("载荷或未知发动机排在自动分级范围内")
    if liquid != [side_engines]*4 or core_liquid != core_engines:
        raise RuntimeError(f"发动机数不符：四侧芯 {liquid}，中央芯 {core_liquid}；"
                           f"预期每侧 {side_engines}、中央 {core_engines}")
    if solids != [separation_motors]*4:
        raise RuntimeError(f"分离小火箭数不符：四侧芯 {solids}；"
                           f"预期每侧 {separation_motors}")
    decouplers = vessel.parts.decouplers
    for stage in (first_stage, second_stage):
        pair_indices = {i for i,p in enumerate(anchors) if p.decouple_stage == stage}
        actual = [d for d in decouplers
                  if d.part.stage == stage and d.part.decouple_stage == stage]
        if len(actual) != 2 or {nearest_side(vessel,d.part,anchors)
                                for d in actual} != pair_indices:
            raise RuntimeError(f"第 {stage} 级必须恰有一对侧芯分离器")
    core_decouplers = [d for d in decouplers
                       if d.part.stage == core_stage and
                       d.part.decouple_stage == core_group]
    if len(core_decouplers) != 1 or sum(
            d.part.stage >= core_stage for d in decouplers) != 5:
        raise RuntimeError("自动分级只能有四个侧芯分离器和一个中央芯分离器")
    if any(c.part.stage != ignition_stage for c in vessel.parts.launch_clamps):
        raise RuntimeError("发射支架必须与共同点火同级释放")
    for i in range(4):
        liquid_fraction(side_parts(vessel, anchors, i))
    liquid_fraction(branch_parts(vessel, core_group))
    return tuple(liquid), core_liquid, tuple(solids)


def tag_superheavy(anchors, core, payload):
    for part, tag in (*zip(anchors, SIDE_TAGS), (core, "booster_core"),
                      (payload, "payload")):
        part.tag = tag
        if part.tag != tag:
            raise RuntimeError(f"无法写入并回读控制器标签 {tag}")

def expected_engines(args, count):
    """把通用回收器的发动机数量门槛替换为本次预检确认的数量。"""
    result = list(args)
    result[result.index("--expected-engines") + 1] = str(count)
    return result


def ground_distance(body, start, end):
    """两经纬度点的近似大圆距离，作为是否尝试返场的保守门槛。"""
    lat1, lon1 = map(math.radians, start)
    lat2, lon2 = map(math.radians, end)
    dlat, dlon = lat2-lat1, lon2-lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
    return 2*body.equatorial_radius*math.asin(min(1, math.sqrt(a)))


def ascent_until(sc, vessel, ap, throttle, fuel_parts, reserve, start_ut,
                 label, allow_warp):
    """用当前组合体推进到这一对侧芯的储备门槛。分离后须重新调用。"""
    body = vessel.orbit.body
    surface = vessel.flight(body.reference_frame)
    last_log = -1e9
    while min(liquid_fraction(p) for p in fuel_parts) > reserve:
        if not allow_warp:
            restore_one_x(sc)
        altitude = surface.mean_altitude
        pitch = heavy_pitch(altitude)
        ap.target_pitch_and_heading(pitch, 90)
        available = vessel.available_thrust
        if available <= 0 and sc.ut-start_ut > 4:
            raise RuntimeError(f"{label} 推进阶段没有可用推力")
        accel_cap = min(1, 24*vessel.mass/max(available, 1))
        q_cap = clamp(45000/max(surface.dynamic_pressure, 1), .55, 1)
        throttle.set(min(accel_cap, q_cap))
        if sc.ut-last_log >= 1:
            fuels = [round(100*liquid_fraction(p), 1) for p in fuel_parts]
            print(f"{label} h={altitude:.0f}m pitch={pitch:.1f}° "
                  f"apo={vessel.orbit.apoapsis_altitude:.0f}m side={fuels}%")
            last_log = sc.ut
        time.sleep(.05)


def start_pair_recovery(manager, anchors, pair_indices, side_vessels, args,
                        target_runway):
    """同步启动同一对侧芯；不可返场时让后分离的一对就近软着陆/溅落。"""
    barrier = manager.ignition_barrier(2) if target_runway else None
    for i, booster in zip(pair_indices, side_vessels):
        tag = SIDE_TAGS[i]
        if target_runway:
            rec = recovery_args(
                tag, args.leg_offset, target=RUNWAY_TARGETS[i],
                allow_warp=args.allow_warp, boostback_cutoff_height=8000,
                aero_target_tilt=5, boostback_return_gain=args.return_gain)
        else:
            rec = recovery_args(
                tag, args.leg_offset, allow_warp=args.allow_warp,
                no_boostback=True, reentry_off_speed=1220,
                reentry_altitude=45000, max_tilt=28, terminal_tilt=10,
                gear_lead_seconds=15, gear_max_height=2500,
                grid_retract_height=3500)
        manager.start(tag, expected_engines(rec, args.side_engines),
                      selected_vessel=booster, ignition_barrier=barrier)


def fly(args, conn, vessel, anchors, core, payload):
    sc = conn.space_center
    body = vessel.orbit.body
    manager = RecoveryManager()
    first_stage = max(a.decouple_stage for a in anchors)
    second_stage, core_stage = first_stage-1, first_stage-2
    core_group, ignition_stage = core.decouple_stage, first_stage+1
    first = [i for i,a in enumerate(anchors) if a.decouple_stage == first_stage]
    second = [i for i,a in enumerate(anchors) if a.decouple_stage == second_stage]
    throttle = None
    separated_core = False
    start_ut = sc.ut
    try:
        restore_one_x(sc)
        ap = vessel.auto_pilot
        ap.reference_frame = vessel.surface_reference_frame
        ap.target_pitch_and_heading(90, 90)
        ap.engaged = True
        vessel.control.sas = False
        throttle = ThrottleController(vessel)
        throttle.set(1)
        activate_checked_stage(vessel, ignition_stage)
        print("SUPERHEAVY ASCENT：五芯共 35 台发动机点火，正东重力转弯")

        first_parts = [side_parts(vessel, anchors, i) for i in first]
        ascent_until(sc, vessel, ap, throttle, first_parts,
                     args.first_reserve, start_ut, "FIRST PAIR",
                     args.allow_warp)
        first_fuel = [liquid_fraction(p) for p in first_parts]
        throttle.set(0)
        ap.engaged = False
        time.sleep(.4)
        activate_checked_stage(vessel, first_stage)
        first_vessels = wait_split(tuple(anchors[i] for i in first), core)
        print(f"FIRST SEP：第一对已分离，储备 {[round(100*x,1) for x in first_fuel]}%")
        start_pair_recovery(manager, anchors, first, first_vessels, args, True)

        # 第一次分离会重建 Vessel/Part；重新取得载荷+中央芯+后对侧芯。
        combined = core.vessel
        combined.physics_range = 2000000.0
        if combined.physics_range < 1990000:
            raise RuntimeError("组合体远距物理范围设置未生效")
        sc.active_vessel = combined
        combined.parts.controlling = core
        throttle.close()
        throttle = ThrottleController(combined, force_independent=True)
        ap = combined.auto_pilot
        ap.reference_frame = combined.surface_reference_frame
        ap.target_pitch_and_heading(
            heavy_pitch(combined.flight(body.reference_frame).mean_altitude), 90)
        ap.engaged = True
        throttle.set(1)
        second_parts = [side_parts(combined, anchors, i) for i in second]
        ascent_until(sc, combined, ap, throttle, second_parts,
                     args.second_reserve, start_ut, "SECOND PAIR",
                     args.allow_warp)

        second_fuel = [liquid_fraction(p) for p in second_parts]
        flight = combined.flight(body.reference_frame)
        location = (flight.latitude, flight.longitude)
        distances = [ground_distance(body, location, RUNWAY_TARGETS[i])
                     for i in second]
        horizontal_speed = flight.horizontal_speed
        return_possible = (min(second_fuel) >= args.late_return_min_fuel and
                           max(distances) <= args.late_return_max_distance and
                           horizontal_speed <= args.late_return_max_speed)
        throttle.set(0)
        ap.engaged = False
        time.sleep(.4)
        activate_checked_stage(combined, second_stage)
        second_vessels = wait_split(tuple(anchors[i] for i in second), core)
        print(f"SECOND SEP：第二对已分离，储备 "
              f"{[round(100*x,1) for x in second_fuel]}%，"
              f"距跑道最大 {max(distances)/1000:.1f} km，"
              f"水平速度 {horizontal_speed:.0f} m/s；"
              + ("尝试同步返场" if return_possible else "就近软着陆或溅落"))
        start_pair_recovery(manager, anchors, second, second_vessels,
                            args, return_possible)

        core_vessel = core.vessel
        core_parts = branch_parts(core_vessel, core_group)
        if not core_parts:
            raise RuntimeError("第二对分离后无法读取中央芯燃料")
        core_vessel.physics_range = 2000000.0
        if core_vessel.physics_range < 1990000:
            raise RuntimeError("中央芯推送阶段的物理范围设置未生效")
        sc.active_vessel = core_vessel
        core_vessel.parts.controlling = core
        throttle.close()
        throttle = ThrottleController(core_vessel, force_independent=True)
        ap = core_vessel.auto_pilot
        ap.reference_frame = core_vessel.surface_reference_frame
        ap.target_pitch_and_heading(
            heavy_pitch(core_vessel.flight(body.reference_frame).mean_altitude), 90)
        ap.engaged = True
        throttle.set(1)
        print("CORE PUSH：中央芯推进剂优先供载荷，不预留回收燃料")
        fuel_rate = .0088
        last_fuel = stage_liquid_fraction(core_vessel, core_group)
        last_fuel_ut = sc.ut
        last_log = -1e9
        while last_fuel > args.core_reserve:
            if not args.allow_warp:
                restore_one_x(sc)
            now = sc.ut
            fuel = stage_liquid_fraction(core_vessel, core_group)
            dt = now-last_fuel_ut
            if dt > .02:
                rate = (last_fuel-fuel)/dt
                if 0 < rate < .05:
                    fuel_rate = .9*fuel_rate + .1*rate
                last_fuel, last_fuel_ut = fuel, now
            tgo = max(2, (fuel-args.core_reserve)/max(fuel_rate, 1e-5))
            core_flight = core_vessel.flight(body.reference_frame)
            altitude = core_flight.mean_altitude
            radius = body.equatorial_radius+altitude
            gravity = body.gravitational_parameter/radius**2
            available = core_vessel.available_thrust
            if available <= 0:
                if fuel <= .05:
                    break
                raise RuntimeError("中央芯提前失去推力，不能安全继续推送")
            accel = min(24, available/max(core_vessel.mass, 1))
            pitch = core_push_pitch(
                altitude, core_flight.vertical_speed,
                core_vessel.orbit.apoapsis_altitude, accel, gravity, tgo)
            ap.target_pitch_and_heading(pitch, 90)
            throttle.set(min(1, 24*core_vessel.mass/max(available, 1)))
            if sc.ut-last_log >= 1:
                print(f"CORE h={altitude:.0f}m "
                      f"apo={core_vessel.orbit.apoapsis_altitude:.0f}m "
                      f"vs={core_flight.vertical_speed:.0f}m/s "
                      f"pitch={pitch:.1f}° fuel={100*fuel:.1f}%")
                last_log = sc.ut
            time.sleep(.05)

        throttle.set(0)
        ap.engaged = False
        throttle.close()
        throttle = None
        if core_vessel.control.throttle > .001:
            raise RuntimeError("中央芯分离前油门未归零")
        viewed = sc.active_vessel
        if sc.active_vessel != core_vessel:
            sc.active_vessel = core_vessel
        activate_checked_stage(core_vessel, core_stage)
        (core_only,) = wait_split((core,), payload)
        separated_core = True
        upper = payload.vessel
        upper.control.throttle = 0
        restored = post_separation_view(viewed, core_vessel, upper, sc.vessels)
        if sc.active_vessel != restored:
            sc.active_vessel = restored
        upper.control.throttle = 0
        upper.physics_range = 2000000.0
        try:
            upper.auto_pilot.engaged = False
        except Exception:
            pass
        print(f"MANUAL HANDOFF：载荷远地点 {upper.orbit.apoapsis_altitude:.0f}m，"
              "油门为零；玩家负责入轨。中央芯不回收，四枚侧芯继续后台回收。")
        manager.wait()
        print("四路侧芯回收线程已结束；已落地载具仍需玩家手动回收。")
    finally:
        if throttle is not None:
            throttle.close()
        if not separated_core:
            try:
                core.vessel.control.throttle = 0
                core.vessel.auto_pilot.engaged = False
            except Exception:
                pass


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--side-engines", type=int, default=7)
    parser.add_argument("--core-engines", type=int, default=7)
    parser.add_argument("--separation-motors", type=int, default=4)
    parser.add_argument("--first-reserve", type=float, default=.27)
    parser.add_argument("--second-reserve", type=float, default=.20)
    parser.add_argument("--core-reserve", type=float, default=.015)
    parser.add_argument("--return-gain", type=float, default=1.08)
    parser.add_argument("--late-return-min-fuel", type=float, default=.18)
    parser.add_argument("--late-return-max-distance", type=float, default=160000)
    parser.add_argument("--late-return-max-speed", type=float, default=1200)
    parser.add_argument("--leg-offset", type=float, default=16)
    parser.add_argument("--allow-warp", action="store_true",
                        help="仅允许手动加速，程序不会主动加速")
    args = parser.parse_args(argv)
    if min(args.side_engines, args.core_engines, args.separation_motors) < 1:
        parser.error("发动机及分离小火箭数量必须为正")
    if not .15 <= args.second_reserve < args.first_reserve <= .4:
        parser.error("第一对储备须高于第二对，第二对至少 15%")
    if not 0 <= args.core_reserve < .05:
        parser.error("中央芯不回收，分离储备须低于 5%")
    if not .7 <= args.return_gain <= 1.4:
        parser.error("返场增益须在 0.7 到 1.4")
    if not 0 < args.late_return_min_fuel <= args.second_reserve:
        parser.error("后对返场最低燃料须不高于分离储备")
    if args.late_return_max_distance <= 0 or args.late_return_max_speed <= 0:
        parser.error("后对返场距离和速度上限必须为正")

    import krpc
    conn = krpc.connect(name="Paired four booster launch")
    try:
        sc = conn.space_center
        vessel = ready_vessel(sc)
        anchors, core, payload = discover_superheavy(vessel)
        liquid, core_count, solids = validate_superheavy(
            vessel, anchors, core, side_engines=args.side_engines,
            core_engines=args.core_engines,
            separation_motors=args.separation_motors)
        first_stage = max(a.decouple_stage for a in anchors)
        print(f"分级映射：点火 {first_stage+1} → 首对 {first_stage} → "
              f"后对 {first_stage-1} → 中央芯 {first_stage-2}")
        print(f"液体发动机：四侧芯 {liquid}，中央 {core_count}；"
              f"分离小火箭 {solids}")
        print(f"燃料门槛：首对 {100*args.first_reserve:.1f}%，"
              f"后对 {100*args.second_reserve:.1f}%，"
              f"中央芯 {100*args.core_reserve:.1f}%")
        for i, anchor in enumerate(anchors):
            xyz = anchor.position(vessel.reference_frame)
            fraction = liquid_fraction(side_parts(vessel, anchors, i))
            target = RUNWAY_TARGETS[i]
            height = vessel.orbit.body.surface_height(*target)
            print(f"侧芯 {i+1}: 分离级 {anchor.decouple_stage}，"
                  f"X/Z=({xyz[0]:.1f}, {xyz[2]:.1f})m，"
                  f"燃料 {100*fraction:.1f}%，跑道目标 {target}，"
                  f"地形 {height:.1f}m")
        print("后对优先返场；若分离时燃料、下程距离或水平速度超限，"
              "则改为就近软着陆/溅落。中央芯不回收。")
        if not args.execute:
            print("只读预检通过；加 --execute 才会写标签、存档、发射。")
            return
        if vessel.situation != sc.VesselSituation.pre_launch:
            raise RuntimeError("必须处于发射前状态")
        tag_superheavy(anchors, core, payload)
        backup = "codex-superheavy-prelaunch-" + datetime.now().strftime(
            "%Y%m%d-%H%M%S-%f")
        sc.save(backup)
        print(f"已保存发射前备份：{backup}")
        fly(args, conn, vessel, anchors, core, payload)
    finally:
        conn.close()


if __name__ == "__main__":
    main()
