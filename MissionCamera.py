"""三芯重型任务的独立事件摄影导演。

这个模块只读取载具状态并控制 KSP 相机。它不会写油门、姿态、SAS、分级、
动作组或发动机状态，因此可以和 ``MyHeavyLaunch.py`` 的飞控并行运行。

Stock kRPC 相机只能围绕活动载具构图，不能放置在任意地面坐标。因此这里用
远距离、小视场角和低俯仰模拟地面长焦机位；真正固定在跑道旁的相机仍需要
CameraTools 一类相机模组。HUD 也不在 kRPC 相机 API 内，开拍前请手动按 F2。
"""

from __future__ import annotations

import csv
import argparse
import math
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import MyHeavyLaunch


TAGS = ("booster_left", "booster_right", "booster_core", "payload")
LANDED_NAMES = {"landed", "splashed"}
CRITICAL_SHOTS = {
    "pad_departure", "separation_hero", "boostback_ignition", "dual_landing",
    "core_separation", "core_landing", "payload_final",
}


@dataclass(frozen=True)
class Shot:
    """一个可重复的相机构图预设。角度单位均为度，距离单位为米。"""

    name: str
    subject: str
    distance: float
    pitch: float
    heading: float
    fov: float
    mode: str = "free"
    min_hold: float = 4.0
    note: str = ""


@dataclass(frozen=True)
class MissionFacts:
    """从游戏状态提取的少量事实，独立出来便于离线测试镜头决策。"""

    launched: bool
    side_split: bool
    core_split: bool
    sides_landed: bool
    core_landed: bool
    core_altitude: float
    side_altitude: float
    side_split_age: float | None = None
    side_ignited: bool = False
    side_ignition_age: float | None = None
    core_split_age: float | None = None
    sides_landed_age: float | None = None


SHOTS = {
    # 开场低机位和箭底近景由导演在自动发射倒计时之前顺序执行。
    "opening_low": Shot("opening_low", "booster_core", 115, 8, 225, 38,
                        note="低机位静态整箭；保持六秒"),
    "engine_close": Shot("engine_close", "booster_core", 36, -18, 200, 46,
                         note="箭底和发动机特写；保持到点火"),
    "pad_departure": Shot("pad_departure", "booster_core", 125, 5, 225, 50,
                          note="离塔广角；不要提前切镜"),
    "tail_ascent": Shot("tail_ascent", "booster_core", 82, -16, 200, 48,
                        "chase", note="箭体后下方尾焰；速度方向平滑跟拍"),
    "gravity_turn": Shot("gravity_turn", "booster_core", 245, -5, 220, 38,
                         "free", note="侧面长焦重力转弯；稳定地平线"),
    "curvature": Shot("curvature", "booster_core", 620, -12, 225, 35,
                      "orbital", note="远景和 Kerbin 曲率"),
    "separation_hero": Shot("separation_hero", "booster_core", 470, -9,
                            270, 54, "free", 7,
                            "中央芯斜后下方；保持到两侧真实点火"),
    "boostback_ignition": Shot("boostback_ignition", "booster_core", 520,
                               -10, 270, 58, "free", 9,
                               "两侧液体发动机同步点火；三芯同框"),
    "separation_wide": Shot("separation_wide", "booster_core", 850, -8,
                            270, 42, "free", 4,
                            "分离后的三芯远景"),
    "boosters_return": Shot("boosters_return", "booster_right", 235, -10,
                            90, 48, "free", 7,
                            "固定以右侧芯为摄影主体；双箭近距离稳定跟拍"),
    "dual_landing": Shot("dual_landing", "current_side", 210, 2, 90, 58,
                         "free", 10,
                         "双箭着陆；稳定前不切换活动载具"),
    "landing_hold": Shot("landing_hold", "booster_left", 310, 4, 90, 46,
                         "free", 5, "落地后安静停留"),
    "core_push": Shot("core_push", "booster_core", 500, -11, 270, 40,
                      "chase", 7, "中央芯继续推送载荷"),
    "core_separation": Shot("core_separation", "payload", 145, -5, 270,
                            55, "free", 8,
                            "从载荷方向回望中央芯远离"),
    "core_reentry": Shot("core_reentry", "booster_core", 680, -10, 270,
                         32, "free", 7, "中央芯下程再入长焦"),
    "core_landing": Shot("core_landing", "booster_core", 300, 3, 270, 48,
                         "free", 10, "中央芯着陆点火至接地"),
    "payload_final": Shot("payload_final", "payload", 850, -12, 250, 32,
                          "orbital", 9,
                          "载荷与 Kerbin 的安静结尾"),
}


def situation_name(vessel) -> str:
    value = str(vessel.situation).lower()
    return value.rsplit(".", 1)[-1]


def landed(vessel) -> bool:
    return situation_name(vessel) in LANDED_NAMES


def altitude(vessel) -> float:
    try:
        return vessel.flight(vessel.orbit.body.reference_frame).surface_altitude
    except Exception:
        return float("inf")


def thrust_fraction(vessel) -> float:
    """返回整船实际推力比例；用于发现独立油门驱动的返场点火。"""
    try:
        maximum = vessel.max_thrust
        return vessel.thrust / maximum if maximum > 0 else 0.0
    except Exception:
        return 0.0


def stable_on_ground(vessel) -> bool:
    """摄影层只在支腿、姿态和速度都稳定后承认着陆完成。"""
    if not landed(vessel):
        return False
    try:
        frame = vessel.orbit.body.reference_frame
        position = vessel.position(frame)
        direction = vessel.direction(frame)
        p_norm = math.sqrt(sum(value * value for value in position))
        d_norm = math.sqrt(sum(value * value for value in direction))
        upright = (sum(a * b for a, b in zip(position, direction)) /
                   (p_norm * d_norm))
        flight = vessel.flight(frame)
        speed = math.hypot(flight.vertical_speed, flight.horizontal_speed)
        legs = list(vessel.parts.legs)
        return (upright >= math.cos(math.radians(10)) and speed <= 1.0 and
                bool(legs) and all(leg.deployed for leg in legs))
    except Exception:
        return False


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def choose_shot(profile: str, facts: MissionFacts) -> str:
    """按任务事件选择镜头；配置只改变摄影优先级，不影响飞控。"""
    if not facts.launched:
        return "engine_close"
    if not facts.side_split:
        if facts.core_altitude < 1200:
            return "pad_departure"
        if facts.core_altitude < 7500:
            return "tail_ascent"
        if facts.core_altitude < 23000:
            return "gravity_turn"
        return "curvature"

    if facts.core_landed:
        return "payload_final"

    # 分离后不按固定秒数猜返场点火。一直从中央芯后下方保留三芯，直到
    # 两侧液体主发动机都真实产生推力，确保全片最重要的同步点火入镜。
    if not facts.side_ignited:
        return "separation_hero"
    if (facts.side_ignition_age is not None and
            facts.side_ignition_age < SHOTS["boostback_ignition"].min_hold):
        return "boostback_ignition"
    if (facts.side_ignition_age is not None and
            facts.side_ignition_age < (SHOTS["boostback_ignition"].min_hold +
                                       SHOTS["separation_wide"].min_hold)):
        return "separation_wide"

    if profile == "master":
        if not facts.core_split:
            return "core_push"
        if (facts.core_split_age is not None and
                facts.core_split_age < SHOTS["core_separation"].min_hold):
            return "core_separation"
    else:
        # recap 和 boosters 都优先保留双助推返场的连续动作。
        if not facts.sides_landed:
            if facts.side_altitude > 5200:
                return "boosters_return"
            return "dual_landing"
        if (facts.sides_landed_age is not None and
                facts.sides_landed_age < 5.0):
            # 保持同一个双箭机位，不在支腿回弹阶段另切活动载具。
            return "dual_landing"
        if not facts.core_split:
            return "core_push"
        # boosters 配置在双箭落地后仍把余下任务拍完，避免中途暂停飞控。
        if (profile == "recap" and facts.core_split_age is not None and
                facts.core_split_age < SHOTS["core_separation"].min_hold):
            return "core_separation"

    if facts.core_altitude > 900:
        return "core_reentry"
    return "core_landing"


def parts_by_tag(space_center):
    """只在初始化时扫描标签，避免每帧遍历存档中所有载具。"""
    result = {}
    for vessel in space_center.vessels:
        try:
            for part in vessel.parts.all:
                if part.tag in TAGS:
                    result[part.tag] = part
        except Exception:
            continue
    return result


def prelaunch_parts(space_center):
    """首次拍摄尚未写标签时，按三芯结构只读识别四个控制部件。"""
    vessel = space_center.active_vessel
    if situation_name(vessel) != "pre_launch":
        return None
    try:
        left, right, core, payload = MyHeavyLaunch.discover_heavy(vessel)
    except Exception:
        return None
    return {
        "booster_left": left,
        "booster_right": right,
        "booster_core": core,
        "payload": payload,
    }


def resolve_subject(name, tracked, current=None):
    if name in {"lower_side", "current_side"}:
        left = tracked["booster_left"]
        right = tracked["booster_right"]
        if name == "current_side" and (current == left or current == right):
            return current
        return left if altitude(left) <= altitude(right) else right
    return tracked[name]


def frame(space_center, vessel, shot: Shot):
    """应用一个构图预设；只触碰活动载具和相机属性。"""
    camera = space_center.camera
    switched = space_center.active_vessel != vessel
    if space_center.active_vessel != vessel:
        space_center.active_vessel = vessel
        # 切换活动载具会让 KSP 在随后几帧重建并重置相机。先等它完成，
        # 再应用构图，避免切到助推器时突然缩成远处的小点。
        time.sleep(0.28)

    def apply_values():
        camera.mode = getattr(space_center.CameraMode, shot.mode)
        camera.distance = clamp(shot.distance, camera.min_distance,
                                camera.max_distance)
        camera.pitch = clamp(shot.pitch, camera.min_pitch, camera.max_pitch)
        camera.heading = shot.heading % 360.0
        camera.fo_v = clamp(shot.fov, camera.min_fo_v, camera.max_fo_v)

    apply_values()
    if switched:
        # 有些相机模式会在激活载具后的下一帧再覆盖一次缩放；第二次写入只
        # 发生在切镜瞬间，不会在镜头内反复拉焦。
        time.sleep(0.12)
        apply_values()


class CueLog:
    """记录现实录制时间和游戏 UT，便于剪辑时快速定位关键动作。"""

    def __init__(self, path=None):
        if path is None:
            stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
            path = Path(__file__).with_name(f"cinematic-cues-{stamp}.csv")
        self.path = Path(path)
        self.start = time.monotonic()
        self.file = self.path.open("w", newline="", encoding="utf-8-sig")
        self.writer = csv.writer(self.file)
        self.writer.writerow(("record_seconds", "game_ut", "kind", "name",
                              "subject", "note"))
        self.file.flush()

    def write(self, game_ut, kind, name, subject="", note=""):
        elapsed = time.monotonic() - self.start
        self.writer.writerow((f"{elapsed:.3f}", f"{game_ut:.3f}", kind,
                              name, subject, note))
        self.file.flush()
        print(f"{kind} +{elapsed:06.1f}s: {name}"
              + (f" | {note}" if note else ""), flush=True)

    def close(self):
        self.file.close()


class CameraDirector:
    def __init__(self, stop_event, *, profile="recap", pause_when_done=True,
                 ready_event=None, cue_path=None):
        if profile not in {"recap", "master", "boosters"}:
            raise ValueError("摄影配置必须是 recap、master 或 boosters")
        self.stop_event = stop_event
        self.profile = profile
        self.pause_when_done = pause_when_done
        self.ready_event = ready_event or threading.Event()
        self.cues = CueLog(cue_path)
        self.last_shot = None
        self.last_change = 0.0
        self.side_split_ut = None
        self.side_ignition_ut = None
        self.core_split_ut = None
        self.sides_landed_ut = None
        self.sides_stable_since = None
        self.final_started = None

    def apply(self, sc, tracked, shot_name, *, force=False):
        if shot_name == self.last_shot and not force:
            return
        shot = SHOTS[shot_name]
        now = time.monotonic()
        # 非关键镜头至少留四秒，避免自动导演产生游戏实况式频繁切镜。
        if (not force and self.last_shot is not None and
                shot_name not in CRITICAL_SHOTS and
                now - self.last_change < SHOTS[self.last_shot].min_hold):
            return
        subject = resolve_subject(shot.subject, tracked, sc.active_vessel)
        frame(sc, subject, shot)
        self.last_shot = shot_name
        self.last_change = now
        self.cues.write(sc.ut, "SHOT", shot_name, shot.subject, shot.note)

    def preflight(self, sc, tracked):
        self.cues.write(sc.ut, "CUE", "hide_hud", note="现在按 F2 隐藏 HUD")
        self.apply(sc, tracked, "opening_low", force=True)
        if self.stop_event.wait(6.0):
            return
        self.apply(sc, tracked, "engine_close", force=True)
        if self.stop_event.wait(4.0):
            return
        self.cues.write(sc.ut, "CUE", "start_recording",
                        note="摄影预备完成；保持发动机近景进入倒计时")
        self.ready_event.set()

    def run(self):
        import krpc

        conn = krpc.connect(name=f"Heavy mission camera ({self.profile})")
        sc = conn.space_center
        anchors = None
        tracked = None
        preflight_done = False
        tracking_complete = False
        try:
            while not self.stop_event.is_set():
                if anchors is None:
                    discovered = prelaunch_parts(sc) or parts_by_tag(sc)
                    if all(tag in discovered for tag in TAGS):
                        anchors = discovered
                    else:
                        time.sleep(0.5)
                        continue

                if not tracking_complete:
                    try:
                        tracked = {tag: anchors[tag].vessel for tag in TAGS}
                    except Exception:
                        anchors = None
                        time.sleep(0.5)
                        continue

                left = tracked["booster_left"]
                right = tracked["booster_right"]
                core = tracked["booster_core"]
                payload = tracked["payload"]

                if not preflight_done and situation_name(core) == "pre_launch":
                    self.preflight(sc, tracked)
                    preflight_done = True
                    continue
                if not self.ready_event.is_set():
                    # 连接到已经起飞的任务时不等待开场镜头。
                    self.ready_event.set()

                side_split = left != right or left != core
                core_split = core != payload
                if core_split:
                    tracking_complete = True
                now_ut = sc.ut
                if side_split and self.side_split_ut is None:
                    self.side_split_ut = now_ut
                    self.cues.write(now_ut, "EVENT", "side_separation")
                side_split_age = (None if self.side_split_ut is None else
                                  now_ut - self.side_split_ut)
                # 分离小火箭也会短暂产生 Vessel.thrust，因此至少等待五秒，
                # 再要求两侧实际推力同时超过最大推力的 3%。后台回收使用发动机
                # 独立油门，这个实际推力读数仍然有效。
                if (side_split and self.side_ignition_ut is None and
                        side_split_age is not None and side_split_age >= 5.0 and
                        thrust_fraction(left) >= 0.03 and
                        thrust_fraction(right) >= 0.03):
                    self.side_ignition_ut = now_ut
                    self.cues.write(now_ut, "EVENT", "side_boostback_ignition",
                                    note="两侧液体主发动机已同时产生推力")
                if core_split and self.core_split_ut is None:
                    self.core_split_ut = now_ut
                    self.cues.write(now_ut, "EVENT", "core_separation")
                stable_now = (side_split and stable_on_ground(left) and
                              stable_on_ground(right))
                if stable_now:
                    if self.sides_stable_since is None:
                        self.sides_stable_since = now_ut
                else:
                    self.sides_stable_since = None
                sides_down = (self.sides_stable_since is not None and
                              now_ut - self.sides_stable_since >= 8.0)
                if sides_down and self.sides_landed_ut is None:
                    self.sides_landed_ut = now_ut
                    self.cues.write(now_ut, "EVENT", "dual_landing_stable",
                                    note="双箭连续直立稳定八秒")

                core_h = altitude(core)
                side_h = min(altitude(left), altitude(right))
                facts = MissionFacts(
                    launched=situation_name(core) != "pre_launch",
                    side_split=side_split,
                    core_split=core_split,
                    sides_landed=sides_down,
                    core_landed=core_split and landed(core),
                    core_altitude=core_h,
                    side_altitude=side_h,
                    side_split_age=side_split_age,
                    side_ignited=self.side_ignition_ut is not None,
                    side_ignition_age=(None if self.side_ignition_ut is None else
                                       now_ut - self.side_ignition_ut),
                    core_split_age=(None if self.core_split_ut is None else
                                    now_ut - self.core_split_ut),
                    sides_landed_age=(None if self.sides_landed_ut is None else
                                     now_ut - self.sides_landed_ut),
                )
                shot_name = choose_shot(self.profile, facts)
                try:
                    self.apply(sc, tracked, shot_name)
                except Exception as exc:
                    # 摄影失败只能损失镜头，不能终止自动任务。
                    self.cues.write(now_ut, "CAMERA_ERROR", shot_name,
                                    note=str(exc))

                if shot_name == "payload_final":
                    if self.final_started is None:
                        self.final_started = time.monotonic()
                    elif time.monotonic() - self.final_started >= 9.0:
                        sc.rails_warp_factor = 0
                        sc.physics_warp_factor = 0
                        if self.pause_when_done:
                            conn.krpc.paused = True
                        self.cues.write(now_ut, "EVENT", "recording_complete",
                                        note="插入黑屏任务标题")
                        self.stop_event.set()
                        return
                # 摄影事件都持续数秒，约 1 Hz 已足够准确。PRE 同时模拟三枚
                # 远距离载具时，降低同步 RPC 查询量可给 20 Hz 回收闭环留出
                # 更多 KSP 主线程时间，避免摄影模式改变返场落点。
                time.sleep(0.9)
        finally:
            self.ready_event.set()
            self.cues.close()
            conn.close()


def run_director(stop_event, **kwargs):
    """线程入口；保留函数形式以便发射包装脚本直接调用。"""
    ready_event = kwargs.get("ready_event")
    try:
        CameraDirector(stop_event, **kwargs).run()
    except Exception as exc:
        # 导演初始化失败时通知一键入口取消发射，绝不让任务在没有摄影的
        # 情况下误点火。飞行开始后的单个镜头异常已在内部降级处理。
        print(f"摄影导演停止：{exc}", flush=True)
        stop_event.set()
        if ready_event is not None:
            ready_event.set()


def main(argv=None):
    """独立附着模式：只运行摄影导演，不启动或控制火箭。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=("recap", "master", "boosters"),
                        default="recap")
    parser.add_argument("--no-final-pause", action="store_true")
    parser.add_argument("--cue-file")
    args = parser.parse_args(argv)
    stop_event = threading.Event()
    print("独立摄影导演已启动；它不会点火、分级或控制任何载具。", flush=True)
    try:
        CameraDirector(
            stop_event,
            profile=args.profile,
            pause_when_done=not args.no_final_pause,
            cue_path=args.cue_file,
        ).run()
    except KeyboardInterrupt:
        stop_event.set()
        print("摄影导演已由用户停止。", flush=True)


if __name__ == "__main__":
    main()
