"""发射、载荷分离、自动切回助推器并调用 MyRecoverV2。

默认只读检查。需为助推器正向控制舱设置唯一标签 booster，
为载荷上保留的部件设置唯一标签 payload。用 --separation-stage 指定
分离动作所在的游戏分级编号。本版仅执行一次起飞分级和一次分离分级。
"""
import argparse
import math
import time
import threading
import uuid
from datetime import datetime

from MyRecoverV2 import clamp, main as recover
from FlightControl import ready_vessel, ThrottleController

# 由发射台 pre_launch 遥测得到，而非使用网络上的近似 KSC 坐标。
# 后续定点返回以此为默认落点；命令行参数仍可覆盖它用于其它着陆场。
KSC_LATITUDE = -0.09721373655153352
KSC_LONGITUDE = -74.5577079610145


def tagged_part(vessel, tag):
    parts = vessel.parts.with_tag(tag)
    if len(parts) != 1:
        raise RuntimeError(f"标签 {tag!r} 必须对应唯一部件，当前找到 {len(parts)} 个")
    return parts[0]


def pitch_for_altitude(altitude, start=250.0, end=30000.0, final=10.0):
    """给下程发射用的重力转弯指令，返回相对地平线的俯仰角（度）。

    inland 剖面保持近垂直，使助推器可返回 KSC；downrange 才逐步压低
    俯仰以积累水平速度。指数曲线让初期保持竖直、末段平滑靠近 final。
    """
    progress = clamp((altitude - start) / (end - start), 0, 1)
    return 91 - math.exp(math.log(91 - final) * progress)


def ascent_throttle(apoapsis, target, mass, available_thrust):
    """远地点闭环油门：接近目标时收油，并限制最大加速度为约 20 m/s²。"""
    if available_thrust <= 0 or mass <= 0:
        raise RuntimeError("上升阶段无可用推力或质量无效")
    if apoapsis >= target:
        return 0.0
    acceleration_limit = min(1.0, 20.0 * mass / available_thrust)
    taper = clamp((target - apoapsis) / 3000.0, 0.05, 1.0)
    return min(acceleration_limit, taper)


def split_booster(booster_anchor, payload_anchor):
    """重新从部件归属解析两个载具，绝不按名称或分级返回列表顺序猜测。"""
    booster = booster_anchor.vessel
    payload = payload_anchor.vessel
    return booster if booster != payload else None


def take_booster_control(sc, booster_anchor, payload_anchor, timeout=10.0, activate=True):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        booster = split_booster(booster_anchor, payload_anchor)
        if booster is not None:
            if activate:
                sc.active_vessel = booster
            if not activate or sc.active_vessel == booster:
                booster.control.throttle = 0.0
                booster.parts.controlling = booster_anchor
                if booster.parts.controlling != booster_anchor:
                    raise RuntimeError("助推器控制基准部件未切换成功")
                return booster
        time.sleep(0.05)
    raise RuntimeError("未确认载荷与助推器分离或未能切换活动载具；不会重复触发分级")


class RecoveryManager:
    """管理多个后台助推器回收任务。

    每个助推器都必须有唯一标签，并由自己的 kRPC 连接运行 `recover`；这样
    玩家可继续操作载荷，后续的并联助推器也不会互相抢活动载具或油门通道。
    当前发射器只创建一个任务，接口已保留给多助推分离事件复用。
    """
    def __init__(self):
        self.threads = {}

    @staticmethod
    def ignition_barrier(parties):
        """创建一组返场点火同步栅栏。

        每枚芯级仍用自己的连接完成安全间距检查和姿态转向；只有在即将
        写入第一次非零返场油门时才等待同组其它芯级。这样同步的是画面中
        真正亮火的瞬间，而不是线程启动或开始转向的时刻。
        """
        if parties < 2:
            raise ValueError("同步点火至少需要两枚芯级")
        return threading.Barrier(parties)

    def start(self, booster_tag, recovery_args, *, selected_vessel=None,
              ignition_barrier=None):
        """启动一个回收线程，可直接绑定分离后已经确认的新 Vessel。

        单芯发射器仍可只传标签，让回收器自行寻找。多芯任务应传入
        ``wait_split`` 得到的 Vessel；管理器只保存其服务端对象编号，回收线程
        再用自己的kRPC连接重建代理。这样既不依赖标签迁移，也不会让三个
        回收器共享同一套数据流。第30次共享连接造成 ``Stream does not exist``，
        因此这里明确保持“一枚芯级一条连接”。
        """
        if booster_tag in self.threads and self.threads[booster_tag].is_alive():
            raise RuntimeError(f"助推器 {booster_tag!r} 已有后台回收任务")

        vessel_name = None
        if selected_vessel is not None:
            # 必须在启动新连接之前扩大物理范围。KSP可能在分离后的下一帧就把
            # 非活动芯级打包成0部件；若等回收线程解析参数后再设置，就会输掉
            # 这场竞态。主线程手里已有可用代理，因此同步写入并回读最可靠。
            if "--physics-range" in recovery_args:
                index = recovery_args.index("--physics-range")
                requested_range = float(recovery_args[index + 1])
                selected_vessel.physics_range = requested_range
                if selected_vessel.physics_range < requested_range * .99:
                    raise RuntimeError(
                        f"{booster_tag} 分离后物理范围设置未生效")
            # Part.tag 在分离重建时偶尔丢失，对象编号又只在创建它的kRPC连接
            # 内有效。KSP载具名称属于存档本体，可被其它连接稳定读取；写入带
            # 随机后缀的唯一名称并回读确认，避免匹配到旧任务的同名残骸。
            vessel_name = f"Codex {booster_tag} {uuid.uuid4().hex[:10]}"
            selected_vessel.name = vessel_name
            if selected_vessel.name != vessel_name:
                raise RuntimeError(f"无法为 {booster_tag} 写入唯一载具身份")

        def run():
            try:
                kwargs = {"selected_vessel_name": vessel_name}
                if ignition_barrier is not None:
                    kwargs.update(ignition_barrier=ignition_barrier,
                                  ignition_label=booster_tag)
                recover(recovery_args, **kwargs)
            except Exception as exc:
                print(f"后台助推器 {booster_tag} 回收停止：{exc}")

        thread = threading.Thread(target=run, name=f"recovery-{booster_tag}", daemon=False)
        self.threads[booster_tag] = thread
        thread.start()
        return thread

    def wait(self):
        """等待全部非活动载具回收结束，不改变玩家当前活动载具。"""
        for thread in self.threads.values():
            thread.join()


class FlightClock:
    def __init__(self, sc, vessel, timeout=600.0, allow_warp=False):
        self.sc, self.vessel = sc, vessel
        self.start = self.last = sc.ut
        self.wall = time.monotonic()
        self.timeout = timeout
        self.allow_warp = allow_warp

    def tick(self):
        while True:
            if self.sc.active_vessel != self.vessel:
                raise RuntimeError("发射过程中活动载具发生变化")
            if (self.sc.rails_warp_factor or self.sc.physics_warp_factor) and not self.allow_warp:
                # 用户误触加速时保持任务存活，立即回到物理帧后继续闭环。
                self.sc.rails_warp_factor = 0
                self.sc.physics_warp_factor = 0
                print("发射控制已取消时间加速并继续执行")
            now = self.sc.ut
            dt = now - self.last
            if dt < 0:
                raise RuntimeError("游戏时间回退，可能发生读档")
            if dt > 60:
                # 已错过的高倍率时间不能追回；从当前状态恢复控制比直接退出更安全。
                print(f"检测到 {dt:.1f} 秒时间跳跃，从当前状态继续控制")
            if now - self.start > self.timeout:
                raise RuntimeError("发射阶段超时")
            if dt >= 0.05:
                self.last = now
                self.wall = time.monotonic()
                return now
            if time.monotonic() - self.wall > 60:
                raise RuntimeError("游戏时间超过 60 秒未推进")
            time.sleep(0.01)


def launch(conn, vessel, booster_anchor, payload_anchor, args):
    sc = conn.space_center
    flight = vessel.flight(vessel.orbit.body.reference_frame)
    orbital = vessel.flight(vessel.orbit.body.non_rotating_reference_frame)
    control, ap = vessel.control, vessel.auto_pilot
    streams = []
    separated = False
    recovery_manager = RecoveryManager()
    throttle = None
    try:
        def telemetry(obj, prop):
            s = conn.add_stream(getattr, obj, prop)
            s.rate = 20
            s.start()
            streams.append(s)
            return s

        altitude = telemetry(flight, "mean_altitude")
        vspeed = telemetry(flight, "vertical_speed")
        apoapsis = telemetry(vessel.orbit, "apoapsis_altitude")
        tta = telemetry(vessel.orbit, "time_to_apoapsis")
        speed = telemetry(orbital, "speed")
        thrust = telemetry(vessel, "available_thrust")
        mass = telemetry(vessel, "mass")
        throttle = ThrottleController(vessel)
        control.sas = False
        control.rcs = False
        # 上升阶段采用 surface_reference_frame：pitch=90 表示当地竖直向上，
        # heading=90 指向正东。回收则改用天体固连参考系的三维 direction。
        ap.reference_frame = vessel.surface_reference_frame
        ap.target_pitch_and_heading(90, 90)
        ap.engaged = True
        throttle.set(1.0)
        control.activate_next_stage()
        # 上升点火阶段不允许加速，避免跳过姿态和油门闭环。
        # 闭环发射不能跳过物理帧；滑行后才允许玩家自行时间加速。
        clock = FlightClock(sc, vessel, allow_warp=False)
        if control.current_stage != args.separation_stage + 1:
            raise RuntimeError("起飞后分级编号与预期不符")
        print("ASCENT：起飞，目标远地点", args.target_altitude)
        last_log = -math.inf
        ignition_ut = sc.ut
        while True:
            now = clock.tick()
            if thrust() <= 0:
                if now - ignition_ut < 3:
                    continue  # 等待发动机启动和数据流刷新
                raise RuntimeError("起飞后未检测到可用推力")
            throttle.set(ascent_throttle(apoapsis(), args.target_altitude, mass(), thrust()))
            ap.target_pitch_and_heading(90 if args.profile == "inland" else pitch_for_altitude(altitude()), 90)
            if apoapsis() >= args.target_altitude:
                break
            if now - ignition_ut > 10 and vspeed() < -5:
                raise RuntimeError("未达到目标远地点便开始下降")
            if now - last_log >= 1:
                print(f"ASCENT 高度={altitude():.0f} 远地点={apoapsis():.0f} "
                      f"油门={throttle.value:.2f}")
                last_log = now
        throttle.set(0.0)
        ap.target_pitch_and_heading(90 if args.profile == "inland" else 0, 90)
        print("COAST：等待接近远地点")
        clock.allow_warp = args.allow_warp
        while tta() > 30:
            clock.tick()
            if vspeed() < -5:
                raise RuntimeError("滑行阶段已越过远地点")

        print("BOOST：补充水平速度（非旋转地心参考系）")
        clock.allow_warp = False
        boost_start = sc.ut
        while args.profile == "downrange" and speed() < args.separation_speed:
            clock.tick()
            if sc.ut - boost_start > 45 or vspeed() < -5:
                raise RuntimeError("速度目标不可达或已开始下降，取消分离")
            if thrust() <= 0:
                raise RuntimeError("补速阶段推力丢失")
            throttle.set(min(1.0, 20 * mass() / thrust()))
        throttle.set(0.0)
        while altitude() < args.target_altitude - 500:
            clock.tick()
            if vspeed() < 0:
                raise RuntimeError("无法到达分离高度，取消分离")
        if control.current_stage != args.separation_stage + 1:
            raise RuntimeError("分离前分级编号发生变化")
        if booster_anchor.vessel != payload_anchor.vessel:
            raise RuntimeError("计划分离前两个标记部件已不属于同一载具")
        # 必须在分离前释放旧自动驾驶，避免旧 Vessel 对象随后指向载荷。
        # 分离前先释放旧载具的 AutoPilot。分级后原 vessel 对象可能代表载荷，
        # 继续用它写控制量会把助推器指令误送到载荷。
        ap.engaged = False
        for s in streams:
            s.remove()
        streams.clear()
        throttle.close()
        throttle = None
        separated = True  # 从此清理也只根据助推器部件归属，不能使用旧 vessel
        control.activate_next_stage()
        booster = take_booster_control(sc, booster_anchor, payload_anchor,
                                       activate=args.foreground_recovery)
        print(f"分离确认，助推器已识别：{booster.name}")
        recovery_args = ["--execute", "--leg-offset", str(args.leg_offset),
                         "--landing-surface-offset", str(args.landing_surface_offset)]
        if args.grid_group is not None:
            recovery_args += ["--grid-group", str(args.grid_group)]
        if args.profile == "inland":
            recovery_args += ["--target-latitude", str(args.landing_latitude),
                              "--target-longitude", str(args.landing_longitude), "--require-land"]
        if args.allow_warp:
            recovery_args.append("--allow-warp")
        if args.foreground_recovery:
            # 调试模式：同步回收，会把控制权切到助推器。
            recover(recovery_args, connection=conn, selected_vessel=booster)
        else:
            # 正常模式：后台回收使用独立连接和发动机独立油门，不抢走载荷。
            recovery_args += ["--background", "--booster-tag", args.booster_tag,
                              "--payload-tag", args.payload_tag]
            recovery_manager.start(args.booster_tag, recovery_args)
            sc.active_vessel = payload_anchor.vessel
            print(f"后台回收已启动；活动载具保持为：{sc.active_vessel.name}")
    finally:
        if throttle is not None:
            throttle.close()
        if not separated or args.foreground_recovery:
            try:
                owned = booster_anchor.vessel if separated else vessel
                owned.control.throttle = 0.0
                owned.auto_pilot.engaged = False
            except Exception as exc:
                print(f"控制清理失败，请检查游戏：{exc}")
        for s in streams:
            try:
                s.remove()
            except Exception:
                pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--profile", choices=("inland", "downrange"), default="inland")
    parser.add_argument("--landing-latitude", type=float, default=KSC_LATITUDE,
                        help="目标纬度；默认是本存档发射台/KSC 的实测值")
    parser.add_argument("--landing-longitude", type=float, default=KSC_LONGITUDE,
                        help="目标经度；默认是本存档发射台/KSC 的实测值")
    parser.add_argument("--booster-tag", default="booster")
    parser.add_argument("--payload-tag", default="payload")
    parser.add_argument("--separation-stage", type=int, help="游戏中分离动作所在的分级编号")
    parser.add_argument("--target-altitude", type=float, default=75000)
    parser.add_argument("--separation-speed", type=float, default=1580)
    parser.add_argument("--leg-offset", type=float, default=16)
    parser.add_argument("--landing-surface-offset", type=float, default=0,
                        help="目标面高于地形的高度；屋顶停机坪需用标定值")
    parser.add_argument("--grid-group", type=int)
    parser.add_argument("--allow-warp", action="store_true",
                        help="滑行阶段允许时间加速；动力阶段会自动恢复 1×")
    parser.add_argument("--foreground-recovery", action="store_true",
                        help="调试用：分离后切到助推器并同步执行回收")
    args = parser.parse_args()
    if args.execute and args.separation_stage is None:
        parser.error("执行前必须指定 --separation-stage")
    if args.separation_stage is not None and args.separation_stage < 0:
        parser.error("分级编号必须非负")
    if not all(math.isfinite(x) for x in (args.target_altitude, args.separation_speed,
                                          args.leg_offset, args.landing_surface_offset)):
        parser.error("高度与速度必须是有限数")
    if (args.target_altitude <= 30000 or args.separation_speed <= 0 or
            args.leg_offset < 0 or args.landing_surface_offset < 0):
        parser.error("目标海拔须高于 30000 m，速度须为正，着陆面与支腿高度须非负")
    if args.grid_group is not None and not 0 <= args.grid_group <= 9:
        parser.error("动作组编号必须在 0 到 9 之间")
    import krpc
    conn = krpc.connect(name="Launch and recovery V2")
    try:
        sc = conn.space_center
        vessel = ready_vessel(sc)
        print(f"当前载具：{vessel.name}，当前分级：{vessel.control.current_stage}")
        booster = tagged_part(vessel, args.booster_tag)
        payload = tagged_part(vessel, args.payload_tag)
        if booster == payload:
            raise RuntimeError("两个标记必须是不同部件")
        if not any(m.name == "ModuleCommand" for m in booster.modules):
            raise RuntimeError("booster 标签必须放在助推器控制舱/探测核心上，并朝向箭头方向")
        print(f"助推器标记：{booster.title}；载荷标记：{payload.title}")
        if not args.execute:
            print("只读检查完成。请确认起飞下一次分级即为载荷分离，再执行。")
            return
        if vessel.situation != sc.VesselSituation.pre_launch:
            raise RuntimeError("联动入口要求从发射台 pre_launch 状态开始")
        if (sc.rails_warp_factor or sc.physics_warp_factor) and not args.allow_warp:
            sc.rails_warp_factor = 0
            sc.physics_warp_factor = 0
            print("起飞前已取消时间加速并继续执行")
        if vessel.control.current_stage != args.separation_stage + 2:
            raise RuntimeError("本版要求起飞和分离为相邻两次分级，请检查分级配置")
        if args.profile == "inland":
            terrain = vessel.orbit.body.surface_height(args.landing_latitude, args.landing_longitude)
            if terrain < 10:
                raise RuntimeError("目标点没有足够的陆地高度裕量")
            print(f"陆地测试目标：{args.landing_latitude}, {args.landing_longitude}，海拔 {terrain:.1f} m")
        backup = "codex-before-launch-" + datetime.now().strftime("%Y%m%d-%H%M%S-%f")
        sc.save(backup)
        print(f"已保存测试备份：{backup}")
        launch(conn, vessel, booster, payload, args)
    finally:
        conn.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("用户停止任务")
    except Exception as exc:
        print(f"任务停止：{exc}")
        raise SystemExit(1)
