"""kRPC 助推器动力回收（不包含返航或定点导航）。

依赖：pip install krpc
只读检查：python MyRecoverV2.py
开始控制：python MyRecoverV2.py --execute --vessel "助推器名称"
前提：已切到分离后的助推器、着陆发动机已激活且可连续节流/重启。
不会自动点火分级；--leg-offset 必须按支腿展开后的质心高度校准。
Ctrl+C 或异常时尽力关油门、退出自动驾驶，再断开连接。
"""

import argparse
import math
import threading
import time
from dataclasses import dataclass
from FlightControl import (ready_vessel, ThrottleController, find_booster,
                           deploy_grid_fins, grid_fins_deployed,
                           retract_grid_fins)


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def norm(a):
    return math.sqrt(dot(a, a))


def unit(a):
    length = norm(a)
    if length < 1e-9:
        raise ValueError("无法归一化零向量")
    return tuple(x / length for x in a)


def slew_direction(previous, desired, max_angle):
    """把单位方向限制在每次最多转过 ``max_angle`` 弧度。

    返推目标会随预测落点持续变化。直接跳变会让长箭体来不及跟随；永久锁定
    首帧方向又会在目标越过后继续加速。这里用球面插值在两者之间取折中。
    """
    desired = unit(desired)
    if previous is None:
        return desired
    previous = unit(previous)
    cosine = clamp(dot(previous, desired), -1.0, 1.0)
    angle = math.acos(cosine)
    if angle <= max_angle:
        return desired
    # 近 180° 时普通线性插值可能经过零向量；先选一条稳定的正交方向。
    if cosine < -0.9999:
        axis = (0.0, -previous[2], previous[1])
        if norm(axis) < 1e-6:
            axis = (-previous[2], 0.0, previous[0])
        axis = unit(axis)
        return unit(tuple(math.cos(max_angle) * p + math.sin(max_angle) * a
                          for p, a in zip(previous, axis)))
    fraction = max_angle / angle
    sin_angle = math.sin(angle)
    a = math.sin((1.0 - fraction) * angle) / sin_angle
    b = math.sin(fraction * angle) / sin_angle
    return unit(tuple(a * p + b * d for p, d in zip(previous, desired)))


def deploy_landing_legs(legs):
    """幂等地展开每条 kRPC ``Leg``，返回调用前已经展开的数量。

    早期版本读取模组的本地化 ``stateDisplayString`` 并触发 Toggle。Falcon
    支腿在中文环境会把该字段显示成动作名“展开”，脚本因此把真实状态判为
    未知。kRPC 的标准 ``Leg.deployed`` 是与语言无关的布尔量；向它重复写
    ``True`` 不会像 Toggle 那样误把正在展开的支腿收回。
    """
    already_deployed = 0
    for leg in legs:
        if leg.deployed:
            already_deployed += 1
        elif leg.deployable:
            leg.deployed = True
    return already_deployed


def landing_legs_deployed(legs):
    """只有存在支腿且每条都完成展开时才通过终态验收。"""
    return bool(legs) and all(leg.deployed for leg in legs)


@dataclass(frozen=True)
class Config:
    """回收律的可调参数。

    位置和速度均在 Kerbin 固连、原点在天体中心的参考系中表示；因而
    `position` 的单位向量就是当地“向上”方向，投影后可得到垂直/水平速度。
    """
    # 支腿接触着陆面时质心高于着陆面的距离。
    leg_offset: float = 16.0
    # 着陆面相对 KSP 地形的高度。地面着陆为 0；屋顶停机坪应填标定后的楼顶高度。
    landing_surface_offset: float = 0.0
    period: float = 0.05     # 游戏秒；实际频率受物理帧与网络限制
    max_tilt: float = 20.0
    terminal_tilt: float = 5.0
    terminal_height: float = 120.0  # 低空只允许小倾角，优先扶正而不是追点
    terminal_lateral_capture_speed: float = 3.0  # 低空达到此横速后锁定直立，防止反向摆动
    terminal_transition_height: float = 300.0
    # 不追求零速悬停：2.5 m/s 对支腿足够温和，同时避免最后几米耗时过长。
    touchdown_speed: float = 2.5
    max_descent_speed: float = 300.0
    velocity_kp: float = 0.8
    velocity_ki: float = 0.08
    horizontal_kp: float = 0.5
    # 高空返推：把预计落地位置误差换成水平速度目标，避免等到终端段才横移数公里。
    boostback_horizontal_kp: float = 0.5
    boostback_max_tilt: float = 45.0
    boostback_min_height: float = 5000.0
    # 高空返推必须在再入前结束。剩余的数百米误差交给大气段/末段横移，
    # 避免把横向控制和自杀式制动耦合到同一高度区间。
    boostback_cutoff_height: float = 22000.0
    boostback_speed_cap: float = 230.0
    # 自由落体时间会忽略大气阻力、姿态转向耗时和末段倾角限制，因此保留
    # 一个由“完整着陆”实测数据标定的系数。未通过完整性验收的飞行不能用于
    # 改它；默认 1.0 表示尚未采用任何经验修正。
    boostback_return_gain: float = 1.0
    # 中央芯若采用自然下程落区，就不应为了追一个遥远目标先做返场点火。
    # 显式开关比把高度阈值设成极大值更容易读懂，也避免阈值含义写反。
    enable_boostback: bool = True
    boostback_deadband: float = 20.0
    # 点火与关机使用不同阈值（迟滞）。没有迟滞时，速度误差恰好在一个
    # 阈值附近会每帧在 COAST/BOOSTBACK 之间跳变，造成脉冲式返推。
    boostback_enter_speed_error: float = 4.0
    boostback_exit_speed_error: float = 1.5
    # 单次横向 Δv 脉冲的油门上限。它不承担托举，因此即使是 0.55 也只会
    # 持续数秒；数值留出余度，避免大推重比箭体因姿态/柔性结构受冲击。
    boostback_throttle: float = 0.55
    boostback_min_alignment: float = 0.92
    boostback_turn_rate: float = 20.0  # deg/s；长箭体目标方向的最大变化率
    landing_turn_rate: float = 12.0  # deg/s；末段防止制导方向跳变激发箭体摆动
    # 侧芯只需让分离器先建立明确的横向间距，就可开始返场转向。4 秒延迟和
    # 30 m 门槛让中央芯仍在主升段点火时两侧芯开始返推，兼顾画面与防碰撞。
    boostback_clearance: float = 30.0
    separation_wait_seconds: float = 4.0
    separation_speed: float = 25.0       # 避让机动的目标相对速度，沿“远离载荷”方向
    separation_kp: float = 0.5
    separation_throttle_cap: float = 0.25
    reentry_altitude: float = 26000.0  # 海拔，沿用旧脚本
    reentry_on_speed: float = 1200.0
    reentry_off_speed: float = 1050.0
    reentry_throttle: float = 0.33
    enable_reentry_burn: bool = True
    # 有目标落区时，在大气层内让箭体从纯逆行方向朝“所需水平速度修正”
    # 偏转少量角度。栅格舵由此提供无推进剂的横向升力；再入点火也只把同一
    # 方向误差顺带修正，不增加一次独立发动机点火。0 表示关闭该功能。
    aero_target_tilt: float = 0.0
    landing_floor: float = 1500.0  # 提前进入着陆控制的最低离地高度
    brake_margin: float = 1.12  # 姿态建立、推力响应和地形误差的制动裕量
    brake_reaction_seconds: float = 1.5
    # 支腿按预计触地时间展开，比固定高度同时适应高速亚轨道回收和低空跳跃。
    # h/下降速度是匀速外推，并非真实剩余飞行时间：减速时会明显偏短。
    # 先保留 8 秒响应裕量，但加上 150 m 高度门限，避免在数百米处开腿。
    # 60 m 兜底适用于慢速下降；这些值仍需实测展开耗时验证。
    gear_deploy_lead_seconds: float = 8.0
    gear_deploy_max_height: float = 1500.0
    gear_emergency_height: float = 60.0
    grid_retract_height: float = 300.0  # 末段由发动机接管横移后收舵，避免触地/入水折断
    warp_exit_lead_seconds: float = 30.0  # 退出加速后留给姿态、遥测和制动的游戏时间
    timeout: float = 1800.0  # 游戏秒


class Guidance:
    """纯数学控制器，输入均为同一地固参考系中的 SI 数据。"""

    def __init__(self, config):
        self.cfg = config
        self.integral = 0.0
        self.state = "COAST"
        self.brake_height = 0.0
        self.warp_exit_height = 0.0
        self.target_distance = 0.0
        self.boostback_active = False
        self.boostback_complete = False
        self.boostback_ignited = False
        self.boostback_direction = None
        # 再入点火与返场点火一样必须有“已经完成”的记忆。若只看瞬时速度，
        # 第一次关机后重力会让总速度再次越过开启门槛，造成无意义的二次点火。
        self.reentry_complete = False
        self.separation_direction = None
        self.landing_direction = None
        # 末段水平闭环包含箭体转动惯量和自动驾驶姿态滞后。速度刚过零时
        # 若立即反向倾斜，长箭体会在最后几十米左右摆动，实飞中曾以
        # 10 m/s 横移接地并倾倒。一旦低空横移已降到支腿可接受范围，
        # 就锁定直立，不再为了追求数学上的零横速而反向修正。
        self.terminal_lateral_complete = False

    def update(self, *, position, velocity, direction, altitude, sea_altitude,
               mass, thrust, mu, dt, target_position=None, boostback_allowed=True,
               payload_position=None, payload_velocity=None):
        c = self.cfg
        values = (*position, *velocity, *direction, altitude, sea_altitude,
                  mass, thrust, mu, dt)
        if not all(math.isfinite(x) for x in values):
            raise ValueError("遥测包含非有限值")
        if mass <= 0 or thrust <= 0 or mu <= 0 or not 0 < dt <= 60.0:
            raise ValueError(
                "质量、推力或游戏时间步无效："
                f"mass={mass:.3f}, thrust={thrust:.3f}, mu={mu:.3f}, dt={dt:.6f}")
        # 球形天体上“向上”随位置变化，不能假设世界坐标的某一轴恒为竖直。
        # 用位置单位向量投影，可把三维惯性速度拆成垂直速度 vs 和水平速度。
        up = unit(position)
        gravity = mu / dot(position, position)
        vs = dot(velocity, up)
        horizontal = tuple(v - vs * u for v, u in zip(velocity, up))
        hs = norm(horizontal)
        # flight.surface_altitude 通常是相对地形的高度。对于楼顶，真正可用
        # 的制动高度必须扣除“楼顶相对地形高度”和“质心到支腿的高度”。
        height = max(0.0, altitude - c.landing_surface_offset - c.leg_offset)
        max_accel = thrust / mass
        net_accel = max_accel * math.cos(math.radians(c.max_tilt)) - gravity
        if net_accel <= 0:
            raise RuntimeError("可用推力不足以按设定倾角完成动力着陆")

        # 自杀式制动的基础公式是 s = v² / (2a)。其中 net_accel 是推力竖直
        # 分量减重力后的净减速度。margin、reaction_seconds 和 80 m 用于覆盖
        # 姿态建立、发动机响应及地形误差；巡航时完全关油门，避免重力损失。
        down = max(0.0, -vs)
        brake_height = (down * down / (2 * net_accel) * c.brake_margin
                        + down * c.brake_reaction_seconds + 80)
        self.brake_height = brake_height
        # 加速期间仍会按游戏时间积分。提前预测这段时间的自由下落，
        # 在制动点之前恢复 1×，而不是等高度固定低于某个阈值才退出。
        lead = c.warp_exit_lead_seconds
        self.warp_exit_height = (brake_height + down * lead
                                 + 0.5 * gravity * lead * lead + 500.0)

        # `boostback_allowed` 由主循环的载荷间距判定给出。安全间距一旦达成，
        # 必须显式释放 SEPARATION，不能让前一帧的避让状态滞留。
        if self.state == "SEPARATION" and boostback_allowed:
            self.state = "COAST"

        # 返推导航：预计在无动力下降时还剩多少时间，并把目标平面距离换成
        # 所需的平均水平速度。这样可补偿 Kerbin 自转和正常重力转弯带来的下程位移。
        planar_target = None
        target_distance = 0.0
        desired_return_speed = 0.0
        horizontal_error = 0.0
        if target_position is not None and height > c.landing_floor:
            delta = tuple(t - p for t, p in zip(target_position, position))
            radial = dot(delta, up)
            planar = tuple(d - radial * u for d, u in zip(delta, up))
            target_distance = norm(planar)
            self.target_distance = target_distance
            if target_distance > 1e-6:
                planar_target = tuple(x / target_distance for x in planar)
                # 解 h = v_down*t + 1/2*g*t²，取未来的正根：
                # t = (-v_down + sqrt(v_down² + 2gh)) / g。
                # 注意根号前必须是减号；写成加号会让下降越快时反而预测剩余
                # 时间越长，返推会把水平速度压得过头，最终越过跑道目标。
                time_to_ground = (-down + math.sqrt(
                    down * down + 2 * gravity * height)) / gravity
                desired_return_speed = min(c.boostback_speed_cap,
                                           c.boostback_return_gain * target_distance /
                                           max(time_to_ground, 1.0))
                desired_horizontal = tuple(desired_return_speed * x for x in planar_target)
                horizontal_error = norm(tuple(goal - actual
                                               for goal, actual in zip(desired_horizontal, horizontal)))

        # 返推是一个有状态的速度闭环。进入需要较大的误差，退出则要求误差
        # 真正收敛；这段记忆就是迟滞，能把发动机从“每帧点一下”的模式变成
        # 一次连续、可预期的返推。到截止高度时明确释放，随后只由 BURN/TERMINAL
        # 处理着陆，不再在低空尝试长距离返场。
        boostback_possible = (c.enable_boostback and boostback_allowed and
                              planar_target is not None and
                              height > c.boostback_cutoff_height and
                              target_distance > c.boostback_deadband)
        if self.boostback_active:
            if (not boostback_possible or
                    horizontal_error <= c.boostback_exit_speed_error):
                self.boostback_active = False
                self.boostback_complete = True
        elif (not self.boostback_complete and boostback_possible and
              horizontal_error >= c.boostback_enter_speed_error):
            self.boostback_active = True

        # 返场速度已达到目标后必须离开 BOOSTBACK。仅把油门置零而保留该
        # 状态，会继续向过期的横向目标转向，并可能在后续误差变化时造成
        # 额外点火；猎鹰式剖面在这一点应立即进入纯弹道滑行。
        if self.state == "BOOSTBACK" and not self.boostback_active:
            self.state = "COAST"
        if self.state not in ("BURN", "TERMINAL"):
            if height <= max(c.landing_floor, brake_height):
                self.state = "BURN"
            elif (payload_position is not None and payload_velocity is not None and
                  not boostback_allowed and height > c.boostback_min_height):
                self.state = "SEPARATION"
            elif self.boostback_active:
                # 这是完整速度闭环：若先前返推过量，误差向量会自动反向，
                # 控制器会点火刹掉多余的横向速度，而不是单向加速后永久滑行。
                self.state = "BOOSTBACK"
            elif (c.enable_reentry_burn and not self.reentry_complete and
                  sea_altitude < c.reentry_altitude and
                  norm(velocity) > c.reentry_on_speed and vs < 0):
                self.state = "REENTRY"
            elif self.state == "REENTRY" and (norm(velocity) < c.reentry_off_speed or vs >= 0):
                self.state = "COAST"
                self.reentry_complete = True

        if self.state == "SEPARATION":
            # 两枚侧芯刚离开中央芯时，箭体仍彼此重叠在很长的纵向包络内。
            # 此时即使主发动机关闭，立刻横向翻转也会让箭头/箭尾扫过中央芯。
            # 锁住分离瞬间姿态，完全依靠分离器建立净间距；间距确认后才允许
            # BOOSTBACK 开始转向和点火。这样也不会凭空增加一次避让点火。
            if self.separation_direction is None:
                self.separation_direction = unit(direction)
            target = self.separation_direction
            alignment = dot(unit(direction), target)
            return target, 0.0, (height, vs, hs, c.separation_speed, alignment)

        if self.state == "BOOSTBACK":
            # 速度外环 -> 水平加速度需求。返推不是悬停：它只修改预测落点
            # 所需的水平速度，随后立刻回到无动力弹道。不能让比例控制以小
            # 油门在高空长时间抵消重力；那样既不省 Δv，也会过早耗尽余量。
            desired_horizontal = tuple(desired_return_speed * x for x in planar_target)
            lateral = tuple(c.boostback_horizontal_kp * (goal - actual)
                            for goal, actual in zip(desired_horizontal, horizontal))
            # 纯横向的 Δv 脉冲：先转到所需横向方向，再以固定、中等油门点火
            # 直到速度误差进入死区。没有向上托举分量，所以它只会消耗完成
            # 回正所需的 Δv，而不会把下降段重新推成上升段。
            commanded_direction = unit(lateral)
            # 每帧跟踪最新速度误差，而不是永久锁定第一次返推方向。限速后的
            # 目标既能在预测落点改变时平滑修正，也不会突然命令长箭体翻转。
            self.boostback_direction = slew_direction(
                self.boostback_direction, commanded_direction,
                math.radians(c.boostback_turn_rate) * min(dt, 1.0))
            target = self.boostback_direction
            alignment = dot(unit(direction), target)
            command_alignment = dot(unit(direction), commanded_direction)
            # 发动机在 KSP 中保持激活，仅调总推力。姿态偏差大时临时收油，
            # 防止像上次测试那样在箭体已经转反后仍以 55% 推力加速错误方向。
            if not self.boostback_ignited and command_alignment >= c.boostback_min_alignment:
                self.boostback_ignited = True
            alignment_gain = clamp(
                (command_alignment - c.boostback_min_alignment) /
                max(1.0 - c.boostback_min_alignment, 1e-6), 0.0, 1.0)
            throttle = (c.boostback_throttle * alignment_gain
                        if self.boostback_ignited else 0.0)
            return target, throttle, (height, vs, hs, -desired_return_speed, alignment)

        if self.state not in ("BURN", "TERMINAL"):
            # 低速/上升阶段保持朝上，避免速度方向接近零时 `-velocity` 无法
            # 定义可靠姿态。下降后机头对准相对速度反向，先控制阻力和水平漂移。
            target = unit(tuple(-v for v in velocity)) if vs < -5 else up
            if (vs < -5 and target_position is not None and planar_target is not None and
                    c.aero_target_tilt > 0 and horizontal_error > 1.0):
                desired_horizontal = tuple(desired_return_speed * x for x in planar_target)
                velocity_correction = tuple(goal - actual for goal, actual in
                                            zip(desired_horizontal, horizontal))
                if norm(velocity_correction) > 1e-6:
                    # 箭体以发动机朝前、机头逆行的姿态下降。实飞证明把机头
                    # 朝速度修正方向偏会产生反向气动力；第30次20°偏置把误差
                    # 从数百米放大到约2 km。因此机头朝修正方向的反向偏转，
                    # 并只使用有限角度，不牺牲主要减速分量。
                    aero_command = tuple(-x for x in velocity_correction)
                    target = slew_direction(
                        target, unit(aero_command),
                        math.radians(c.aero_target_tilt))
            alignment = dot(unit(direction), target)
            throttle = c.reentry_throttle if self.state == "REENTRY" and alignment > 0.9 else 0.0
            return target, throttle, (height, vs, hs, 0.0, alignment)

        # 终端下降包线 v² = v_touch² + 2 a s：允许高空较快下落，
        # 随剩余高度平方根式收敛到 2.5 m/s 左右的连续接地速度，而不是悬停。
        speed = min(c.max_descent_speed,
                    math.sqrt(c.touchdown_speed ** 2 + 2 * net_accel * 0.35 * height))
        speed = min(speed, c.touchdown_speed + 0.25 * height)
        target_vs = -speed
        error = target_vs - vs
        candidate = clamp(self.integral + error * min(dt, 1.0), -20.0, 20.0)
        vertical_accel = max(0.0, gravity + c.velocity_kp * error + c.velocity_ki * candidate)
        horizontal_target = (0.0, 0.0, 0.0)
        if target_position is not None and height > c.landing_floor:
            delta = tuple(t - p for t, p in zip(target_position, position))
            radial = dot(delta, up)
            planar = tuple(d - radial * u for d, u in zip(delta, up))
            distance = norm(planar)
            # 位置外环只在高于 landing_floor 时追点。进入最后约 1.5 km 后，
            # 剩余数公里已经不可能安全修正，必须把目标水平速度直接归零；
            # 否则箭体会在垂直速度接近零时仍横移十几米每秒并倾倒。
            limit = min(50.0, max(0.0, height - 5) * 0.3)
            desired = min(limit, distance * .035)
            horizontal_target = tuple(x * desired / max(distance, 1e-9) for x in planar)
        if (height <= c.terminal_height and
                hs <= c.terminal_lateral_capture_speed):
            self.terminal_lateral_complete = True
        lateral = tuple(c.horizontal_kp * (goal - x)
                        for goal, x in zip(horizontal_target, horizontal))
        if self.terminal_lateral_complete:
            lateral = (0.0, 0.0, 0.0)
        tilt = c.terminal_tilt if height < c.terminal_height else c.max_tilt
        lateral_limit = vertical_accel * math.tan(math.radians(tilt))
        scale = min(1.0, lateral_limit / max(norm(lateral), 1e-9))
        commanded_target = tuple(vertical_accel * u + scale * a for u, a in zip(up, lateral))
        commanded_target = unit(commanded_target) if norm(commanded_target) > 1e-9 else up
        # 长箭体不能跟随每帧突然翻转的横向速度误差。以当前实际方向作为
        # 初值，再限制目标变化率；这会抑制上一趟右侧芯在 1 km 以下的往复摆动。
        if self.terminal_lateral_complete:
            # 直接把目标设为当地竖直，让自动驾驶尽早卸掉剩余倾角。这里
            # 只是跳过“指令方向”的限速；真实箭体仍受其转动惯量约束。
            self.landing_direction = up
        else:
            if self.landing_direction is None:
                self.landing_direction = unit(direction)
            self.landing_direction = slew_direction(
                self.landing_direction, commanded_target,
                math.radians(c.landing_turn_rate) * min(dt, 1.0))
        target = self.landing_direction

        # 输出油门先由竖直加速度需求得到。箭体倾斜时同样的发动机推力只有
        # cos(tilt) 分量向上，故要按实际 vertical_alignment 补偿；严重未
        # 对准时禁止点火，避免把全推力用在横向或朝下方向。
        vertical_alignment = dot(unit(direction), up)
        raw = vertical_accel / (max_accel * max(vertical_alignment, 0.1))
        aligned = vertical_alignment > 0.5
        if self.state == "BURN":
            # 最大推力把重力损失压到最低；接近末段包线才改为精细调节。
            if height <= c.terminal_transition_height or down <= speed * 1.12:
                self.state = "TERMINAL"
                throttle = clamp(raw, 0.0, 1.0) if aligned else 0.0
            else:
                throttle = 1.0 if aligned else 0.0
        else:
            throttle = clamp(raw, 0.0, 1.0) if aligned else 0.0
        # 饱和或姿态未就绪时冻结积分；允许反向误差解除饱和。
        if aligned and (0 < raw < 1 or (raw >= 1 and error < 0) or (raw <= 0 and error > 0)):
            self.integral = candidate
        return target, throttle, (height, vs, hs, target_vs, vertical_alignment)


def main(argv=None, *, connection=None, selected_vessel=None,
         selected_vessel_name=None, ignition_barrier=None,
         ignition_label=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target-latitude", type=float)
    parser.add_argument("--booster-tag", default="booster")
    parser.add_argument("--payload-tag", default="payload",
                        help="后台返推的安全间隔检查所用载荷标签")
    parser.add_argument("--target-longitude", type=float)
    parser.add_argument("--require-land", action="store_true")
    parser.add_argument("--activate-engines", action="store_true", help="读档后明确激活当前载具上的可用发动机，不分级")
    parser.add_argument("--background", action="store_true",
                        help="回收非活动载具；用于让玩家继续控制载荷")
    parser.add_argument("--allow-warp", action="store_true",
                        help="允许滑行阶段使用时间加速；进入制动自动取消加速")
    parser.add_argument("--execute", action="store_true", help="接管当前活动载具")
    parser.add_argument("--vessel", help="核对活动载具名称，不自动切换载具")
    parser.add_argument("--leg-offset", type=float, default=16.0)
    parser.add_argument("--landing-surface-offset", type=float, default=0.0,
                        help="目标着陆面高出地形的米数；屋顶停机坪需先标定")
    parser.add_argument("--grid-group", type=int, default=None,
                        help="可选：将指定动作组设为开启；仅用于绑定了展开动作的动作组")
    parser.add_argument("--expected-engines", type=int, default=0,
                        help="由起飞器传入的一级发动机基线；用于触地完整性验收")
    parser.add_argument("--no-reentry-burn", action="store_true",
                        help="跳过再入减速点火，把燃料全部留给一次最终着陆点火")
    parser.add_argument("--no-boostback", action="store_true",
                        help="禁用返场点火；用于依靠自然下程落区的中央芯")
    parser.add_argument("--boostback-cutoff-height", type=float, default=22000.0,
                        help="允许开始返推的最低支腿离地高度；低弹道侧芯可下调")
    parser.add_argument("--boostback-return-gain", type=float, default=1.0,
                        help="按实飞落点标定的返场水平速度倍率")
    parser.add_argument("--physics-range", type=float, default=0.0,
                        help="保持本载具离轨物理模拟的距离；0 使用 KSP 默认值")
    parser.add_argument("--reentry-off-speed", type=float, default=1050.0,
                        help="再入点火结束的总速度，单位 m/s")
    parser.add_argument("--reentry-altitude", type=float, default=26000.0,
                        help="下降时允许开始再入点火的海拔，单位 m")
    parser.add_argument("--aero-target-tilt", type=float, default=0.0,
                        help="再入滑翔朝目标速度误差偏转的最大角度；0 表示纯逆行")
    parser.add_argument("--max-tilt", type=float, default=20.0,
                        help="高空动力下降允许的最大倾角，单位度")
    parser.add_argument("--terminal-tilt", type=float, default=5.0,
                        help="低空动力下降允许的最大倾角，单位度")
    parser.add_argument("--gear-deploy-lead-seconds", type=float, default=8.0,
                        help="按匀速外推计算的支腿展开提前量")
    parser.add_argument("--gear-deploy-max-height", type=float, default=1500.0,
                        help="允许发出支腿展开指令的最大离地高度")
    parser.add_argument("--grid-retract-height", type=float, default=300.0,
                        help="栅格舵开始收回的支腿离地高度；高速芯级需给动画留足时间")
    args = parser.parse_args(argv)
    if (not math.isfinite(args.leg_offset) or args.leg_offset < 0 or
            not math.isfinite(args.landing_surface_offset) or args.landing_surface_offset < 0):
        parser.error("着陆面和支腿高度必须是非负有限数")
    if args.grid_group is not None and not 0 <= args.grid_group <= 9:
        parser.error("动作组编号必须在 0 到 9 之间（0 表示第十组）")
    if not math.isfinite(args.boostback_cutoff_height) or args.boostback_cutoff_height < 3000:
        parser.error("返推许可高度必须是至少 3000 m 的有限数")
    if not math.isfinite(args.boostback_return_gain) or not .5 <= args.boostback_return_gain <= 2:
        parser.error("返场速度倍率必须在 0.5 到 2.0 之间")
    if not math.isfinite(args.physics_range) or not 0 <= args.physics_range <= 2500000:
        parser.error("物理范围必须在 0 到 2500 km 之间")
    if not math.isfinite(args.reentry_off_speed) or args.reentry_off_speed <= 0:
        parser.error("再入关机速度必须是正有限数")
    if not math.isfinite(args.reentry_altitude) or args.reentry_altitude <= 0:
        parser.error("再入点火高度必须是正有限数")
    if not math.isfinite(args.aero_target_tilt) or not 0 <= args.aero_target_tilt <= 30:
        parser.error("气动落区修正角必须在 0 到 30 度之间")
    if (not math.isfinite(args.max_tilt) or not 0 < args.max_tilt < 45 or
            not math.isfinite(args.terminal_tilt) or
            not 0 < args.terminal_tilt <= args.max_tilt):
        parser.error("倾角必须满足 0 < 低空倾角 <= 最大倾角 < 45 度")
    if (not math.isfinite(args.gear_deploy_lead_seconds) or
            args.gear_deploy_lead_seconds <= 0 or
            not math.isfinite(args.gear_deploy_max_height) or
            args.gear_deploy_max_height <= 0):
        parser.error("支腿展开提前量和最大高度必须是正有限数")
    if (not math.isfinite(args.grid_retract_height) or
            args.grid_retract_height <= 0):
        parser.error("栅格舵收回高度必须是正有限数")

    import krpc  # 延迟导入，数学测试和 --help 不依赖游戏或 krpc 安装

    cfg = Config(leg_offset=args.leg_offset,
                 landing_surface_offset=args.landing_surface_offset,
                 enable_reentry_burn=not args.no_reentry_burn,
                 enable_boostback=not args.no_boostback,
                  reentry_altitude=args.reentry_altitude,
                  aero_target_tilt=args.aero_target_tilt,
                 boostback_cutoff_height=args.boostback_cutoff_height,
                 boostback_return_gain=args.boostback_return_gain,
                 reentry_off_speed=args.reentry_off_speed,
                 max_tilt=args.max_tilt,
                 terminal_tilt=args.terminal_tilt,
                 gear_deploy_lead_seconds=args.gear_deploy_lead_seconds,
                 gear_deploy_max_height=args.gear_deploy_max_height,
                 grid_retract_height=args.grid_retract_height)
    conn = connection if connection is not None else krpc.connect(
        name="Recovery V2", address="127.0.0.1", rpc_port=50000, stream_port=50001)
    streams = []
    controlling = False
    throttle_driver = None
    vessel = None
    try:
        sc = conn.space_center
        if selected_vessel is not None and selected_vessel_name is not None:
            raise ValueError("selected_vessel 与 selected_vessel_name 只能指定一个")
        if selected_vessel is None and selected_vessel_name is not None:
            # 唯一名称在KSP存档中跨连接稳定。等待新Vessel完成重建，再把当前
            # 线程的独立连接所返回的代理交给制导。
            deadline = time.monotonic() + 15.0
            stable = 0
            while time.monotonic() < deadline:
                try:
                    matches = [v for v in sc.vessels
                               if v.name == selected_vessel_name]
                    if len(matches) != 1:
                        raise RuntimeError(
                            f"唯一载具名称匹配数为 {len(matches)}")
                    candidate = matches[0]
                    signature = (candidate in sc.vessels,
                                 candidate.loaded, candidate.packed,
                                 len(candidate.parts.all),
                                 len(candidate.parts.engines),
                                 candidate.mass, candidate.max_thrust)
                    valid = (signature[0] and signature[1] and not signature[2] and
                             signature[3] > 0 and signature[4] > 0 and
                             signature[5] > 0 and signature[6] > 0)
                except Exception:
                    signature = None
                    valid = False
                # 分离小火箭的余焰会让质量和最大推力逐帧微变；这里只要求
                # 连续健康，不能把浮点读数完全相等误当成稳定条件。
                stable = stable + 1 if valid else 0
                if stable >= 2:
                    selected_vessel = candidate
                    break
                time.sleep(.05)
            else:
                raise RuntimeError(
                    f"分离后指定芯级 {selected_vessel_name!r} 的物理状态未稳定")
        if selected_vessel is None and args.execute:
            identified = find_booster(sc, args.booster_tag)
            if args.background:
                # 后台线程绝不能在标签尚未迁移到新 Vessel 时退回活动载具。
                # 分离后的几帧里 KSP 会重建部件归属；此时 ready_vessel(sc)
                # 往往返回玩家正在看的另一枚侧芯。第 27 次中央芯线程因此误绑
                # 到 32 部件/11 发动机的侧芯并退出。后台模式只认自己的唯一
                # 标签，短暂等待标签出现，超时则明确失败而不接管别的载具。
                deadline = time.monotonic() + 15.0
                while identified is None and time.monotonic() < deadline:
                    time.sleep(.05)
                    identified = find_booster(sc, args.booster_tag)
                if identified is None:
                    raise RuntimeError(
                        f"后台回收未找到标签 {args.booster_tag!r} 对应的载具")
                selected_vessel = identified
                print(f"后台回收已锁定助推器：{identified.name}")
            elif identified is not None and identified != sc.active_vessel:
                print(f"读档当前选中载荷，按标记切回助推器：{identified.name}")
                sc.active_vessel = identified
        vessel = selected_vessel if selected_vessel is not None else ready_vessel(sc)
        if vessel != sc.active_vessel and not args.background:
            raise RuntimeError("指定回收载具不是活动载具，拒绝接管")
        if args.physics_range > 0:
            vessel.physics_range = args.physics_range
            actual_range = vessel.physics_range
            if actual_range < args.physics_range * .99:
                raise RuntimeError(
                    f"物理范围设置未生效：请求 {args.physics_range:.0f}m，"
                    f"回读 {actual_range:.0f}m")
            print(f"[{args.booster_tag}] 临时物理范围已设为 {actual_range/1000:.0f} km")
        body = vessel.orbit.body
        frame = body.reference_frame
        flight = vessel.flight(frame)
        print(f"服务版本: {conn.krpc.get_status().version}")
        print(f"载具: {vessel.name} | 天体: {body.name} | 状态: {vessel.situation}")
        print(f"对地高度: {flight.surface_altitude:.1f} m | 最大推力: {vessel.max_thrust:.0f} N")
        print(f"支腿质心高度估计: {cfg.leg_offset:.1f} m（请校准）")
        if args.vessel and vessel.name != args.vessel:
            raise RuntimeError("活动载具名称不匹配，请在游戏中选中助推器")
        if not args.execute:
            print("只读检查完成。加 --execute 才会控制载具。")
            return
        finished = (sc.VesselSituation.landed, sc.VesselSituation.splashed)
        if vessel.situation in finished:
            print("载具已经着陆或溅落，不接管。")
            return
        if (sc.rails_warp_factor or sc.physics_warp_factor) and not args.allow_warp:
            sc.rails_warp_factor = 0
            sc.physics_warp_factor = 0
            print("回收启动时已取消时间加速并继续执行")
        if (args.target_latitude is None) != (args.target_longitude is None):
            raise ValueError("目标经纬度必须同时指定")
        target_position = None
        if args.target_latitude is not None:
            terrain = body.surface_height(args.target_latitude, args.target_longitude)
            if args.require_land and terrain < 10:
                raise RuntimeError("目标在海面或海岸，拒绝陆地任务")
            target_position = body.surface_position(args.target_latitude, args.target_longitude, frame)
        payload = None
        if args.background:
            # 后台返推不能假设分级已经把两个载具拉开。找不到载荷时同样禁止返推，
            # 因为此时无法证明它已在安全距离之外。
            payload = find_booster(sc, args.payload_tag)
        if args.activate_engines:
            for engine in vessel.parts.engines:
                if engine.has_fuel:
                    engine.active = True
        mu = body.gravitational_parameter
        if vessel.max_thrust / vessel.mass <= mu / norm(vessel.position(frame)) ** 2:
            raise RuntimeError("已激活发动机的可用推重比不足；请检查发动机和燃料")
        # 记录分离后的完整一级，用于区分“完整着陆”和仅剩一个控制核心的残骸。
        # 不把残骸的位置当作落点数据，避免把错误反馈写进导航标定。
        initial_part_count = len(vessel.parts.all)
        initial_engine_count = max(len(vessel.parts.engines), args.expected_engines)
        log_tag = args.booster_tag or vessel.name
        fuel_store = vessel.resources
        fuel_capacity = fuel_store.max("LiquidFuel")
        print(f"[{log_tag}] 完整性基线：部件 {initial_part_count}，"
              f"发动机 {initial_engine_count}，液体燃料容量 {fuel_capacity:.1f}")

        def stream(func, *arguments):
            s = conn.add_stream(func, *arguments)
            s.rate = 20
            s.start()
            streams.append(s)
            return s

        ut = stream(getattr, sc, "ut")
        position = stream(vessel.position, frame)
        velocity = stream(vessel.velocity, frame)
        direction = stream(vessel.direction, frame)
        altitude = stream(getattr, flight, "surface_altitude")
        sea_altitude = stream(getattr, flight, "mean_altitude")
        mass = stream(getattr, vessel, "mass")
        # available_thrust 会在独立油门写为 0 的某些发动机模组上瞬间回报 0，
        # 即使发动机仍然激活且可以再次加油门。制导需要的是推力上限，因此
        # 使用不受当前油门指令影响的 max_thrust 计算制动距离和推重比。
        thrust = stream(getattr, vessel, "max_thrust")
        situation = stream(getattr, vessel, "situation")
        active = stream(getattr, sc, "active_vessel")
        rails = stream(getattr, sc, "rails_warp_factor")
        physics = stream(getattr, sc, "physics_warp_factor")
        payload_position = None
        payload_velocity = None
        if payload is not None and payload != vessel:
            payload_position = stream(payload.position, frame)
            payload_velocity = stream(payload.velocity, frame)

        # kRPC 对非活动 Vessel 的总油门可能只更新属性而不实际点火；
        # 后台回收一律逐发动机写入油门，避免抢走载荷活动控制权。
        throttle_driver = ThrottleController(vessel, force_independent=args.background)
        control = vessel.control
        ap = vessel.auto_pilot
        # 这组模组支腿虽然内部使用 ModuleWheelDeployment，但 kRPC 已将其正确
        # 暴露为标准 Leg。直接使用 Leg API，可避开本地化字符串和 Toggle 反向。
        landing_legs = list(vessel.parts.legs)
        print("接管时支腿状态：", [str(leg.state) for leg in landing_legs])

        controlling = True
        throttle_driver.set(0.0)
        control.sas = False
        control.rcs = True
        ap.reference_frame = frame
        ap.target_direction = unit(position())
        ap.engaged = True
        grid_modules = deploy_grid_fins(vessel)
        grids_retracted = False
        if grid_modules:
            print(f"已检查 {len(grid_modules)} 片栅格舵的独立展开事件")
        elif args.grid_group is not None:
            control.set_action_group(args.grid_group, True)
            print("未识别 T-222 栅格舵，已发送指定动作组；尚未验证实际展开")
        guidance = Guidance(cfg)
        start = previous = ut()
        last_log = start - 1
        contact_start = None
        stable_start = None
        wall_update = time.monotonic()
        gear_set = False
        gear_command_ut = None
        gear_retry_ut = None
        gear_report_ut = None
        gear_verified = False
        grids_verified = False
        separation_cleared = not args.background
        # 多芯任务可共享一个线程栅栏。单芯回收或命令行直接运行时没有
        # 栅栏，行为与原来完全一致。
        ignition_synchronized = ignition_barrier is None
        print("开始回收；Ctrl+C 终止控制。")
        while True:
            now = ut()
            dt = now - previous
            if dt < 0:
                raise RuntimeError("游戏时间回退，可能发生读档")
            # 高空纯滑行阶段不需要三枚芯级都以 20 Hz 通过 kRPC 刷新。
            # 降到 5 Hz 可减少主线程和网络调用；返场、再入、制动及末端
            # 着陆仍保持 20 Hz，制动点和触地控制精度不受影响。
            high_coast = guidance.state == "COAST"
            loop_period = 0.20 if high_coast else cfg.period
            if dt < loop_period:
                if time.monotonic() - wall_update > 60:
                    raise RuntimeError("游戏时间超过 60 秒未推进，停止控制")
                time.sleep(0.01)
                continue
            previous = now
            if grid_modules and not grids_verified and now - start >= 3:
                grid_count = grid_fins_deployed(grid_modules)
                print(f"栅格舵展开锁定回读：{grid_count}/{len(grid_modules)}")
                grids_verified = True
            wall_update = time.monotonic()
            if active() != vessel and not args.background:
                raise RuntimeError("活动载具已改变，停止控制原载具")
            if (rails() or physics()) and not args.allow_warp:
                # 不因误触加速丢弃正在下降的助推器；恢复 1×并继续控制。
                sc.rails_warp_factor = 0
                sc.physics_warp_factor = 0
                print("回收控制已取消时间加速并继续执行")
            if now - start > cfg.timeout:
                raise RuntimeError("回收超时")
            current_situation = situation()
            if current_situation in finished:
                throttle_driver.set(0.0)
                if contact_start is None:
                    contact_start = now
                    stable_start = None
                    print(f"[{log_tag}] 已接触着陆面，开始连续落稳验收")
                remaining_parts = len(vessel.parts.all)
                remaining_engines = len(vessel.parts.engines)
                intact = (remaining_parts == initial_part_count and
                          remaining_engines >= initial_engine_count)
                if not intact:
                    raise RuntimeError(
                        "载具触地但结构已损失，判定为炸毁："
                        f"部件 {remaining_parts}/{initial_part_count}，"
                        f"发动机 {remaining_engines}/{initial_engine_count}")
                legs_ok = landing_legs_deployed(landing_legs)
                upright_dot = dot(unit(direction()), unit(position()))
                upright = upright_dot > math.cos(math.radians(10))
                settle_speed = norm(velocity())
                stable = legs_ok and upright and settle_speed <= 1.0
                if stable:
                    if stable_start is None:
                        stable_start = now
                else:
                    stable_start = None
                # 不在接触后的第 8 秒只抽样一次并立刻退出。水面晃动或支腿
                # 动画可能短暂超过阈值；继续保持姿态，直到真正连续稳定 8 秒。
                # 30 秒仍不稳定才失败，并把每个判据写入日志供下一次标定。
                if stable_start is not None and now - stable_start >= 8.0:
                    if current_situation == sc.VesselSituation.splashed and args.require_land:
                        miss = None
                        if target_position is not None:
                            miss = norm(tuple(t-p for t, p in zip(target_position, position())))
                        raise RuntimeError(
                            f"载具完整落稳但位于水面，陆地回收任务失败；距目标 {miss:.0f}m"
                            if miss is not None else
                            "载具完整落稳但位于水面，陆地回收任务失败")
                    delta = tuple(t-p for t, p in zip(target_position, position())) if target_position else None
                    miss = norm(delta) if delta else None
                    print(f"连续落稳 8 秒，部件 {remaining_parts}/{initial_part_count}，"
                          f"发动机 {remaining_engines}/{initial_engine_count}，"
                          f"纬度={flight.latitude:.8f}，经度={flight.longitude:.8f}，"
                          f"目标三维距离={miss} m；定点成功仍需确认跑道范围。")
                    if args.physics_range > 0:
                        # 不在单枚芯级落稳时恢复默认范围。physics_range 会影响
                        # KSP 对远距离载具的装载；此时载荷和中央芯通常还在数十
                        # 千米外，过早缩小范围会让刚落地的侧芯被卸载，甚至使仍在
                        # 末端制导的另一枚侧芯丢失 Vessel 对象。范围由发射总控在
                        # 脚本结束后也保持 400 km，直到玩家在游戏中手动回收。
                        print(f"[{log_tag}] 保持远距离物理范围，等待玩家手动回收")
                    break
                if now - contact_start >= 30.0:
                    tilt = math.degrees(math.acos(clamp(upright_dot, -1.0, 1.0)))
                    raise RuntimeError(
                        "接触 30 秒后仍未连续落稳："
                        f"支腿={sum(leg.deployed for leg in landing_legs)}/{len(landing_legs)}，"
                        f"倾角={tilt:.1f}°，总速度={settle_speed:.2f}m/s，"
                        f"部件={remaining_parts}/{initial_part_count}，"
                        f"发动机={remaining_engines}/{initial_engine_count}")
                continue
            contact_start = None
            stable_start = None
            if dt > 60:
                # 时间加速可能让 UT 跨大步。控制器的积分只使用保守步长，
                # 但仍依据最新遥测立即重算制动状态，而不是终止任务。
                print(f"检测到 {dt:.1f} 秒时间跳跃，从当前高度重新计算回收")
                dt = 1.0
            current_position = position()
            current_velocity = velocity()
            current_direction = direction()
            current_mass = mass()
            current_thrust = thrust()
            # Vessel刚重建或从打包状态恢复时，kRPC流偶尔先送出一帧全零值。
            # 这不是飞行状态，跳过该帧即可；绝不能把零质量或零方向送入制导。
            if (norm(current_position) < 1.0 or
                    norm(current_direction) < 0.5 or
                    current_mass <= 0 or current_thrust <= 0):
                time.sleep(.01)
                continue
            boostback_allowed = not args.background or separation_cleared
            if (args.background and not separation_cleared and
                    payload_position is not None):
                separation = norm(tuple(
                    a - b for a, b in zip(current_position, payload_position())))
                # 等待分离弹簧先建立相对速度，再以较小间距开始返推；相比 1 km
                # 的硬等待能保留高空返推窗口，同时避免刚分离便横向撞回载荷。
                boostback_allowed = (separation >= cfg.boostback_clearance and
                                     now - start >= cfg.separation_wait_seconds)
                if boostback_allowed:
                    # 安全间隔只需确认一次。此后即使玩家切换载具，或载荷在
                    # 后续任务中被分级/回收，也不再读取它的旧 Vessel 代理。
                    separation_cleared = True
            target, throttle, telemetry = guidance.update(
                position=current_position, velocity=current_velocity,
                direction=current_direction, altitude=altitude(),
                sea_altitude=sea_altitude(), mass=current_mass,
                thrust=current_thrust, mu=mu, dt=dt,
                target_position=target_position,
                boostback_allowed=boostback_allowed,
                payload_position=(payload_position()
                                  if payload_position is not None and not separation_cleared
                                  else None),
                payload_velocity=(payload_velocity()
                                  if payload_velocity is not None and not separation_cleared
                                  else None))
            # 加速由玩家手动控制。高倍率会让单次游戏时间跳跃跨过制动点，
            # 所以基于当前速度预测 30 秒自由下落后自动退回 1×。
            warp_safe = (guidance.state == "COAST" and
                         altitude() > guidance.warp_exit_height and
                         sea_altitude() > cfg.reentry_altitude)
            if (not args.allow_warp or not warp_safe) and (rails() or physics()):
                # 动力制动、大气再入和制动点预测区都需要密集物理帧。
                sc.rails_warp_factor = 0
                sc.physics_warp_factor = 0
                print("接近制动区，已取消时间加速")
            ap.target_direction = target
            # 两侧芯各自完成安全间距判断和返场姿态对准后，在第一次非零
            # BOOSTBACK 油门前会合。先到的一侧保持零油门和当前姿态，等另
            # 一侧也就绪后一起放行；超时则打破栅栏并继续独立回收，避免
            # 单侧故障拖死另一侧。
            if (not ignition_synchronized and
                    guidance.state == "BOOSTBACK" and throttle > 0):
                label = ignition_label or log_tag
                print(f"[{label}] 返场姿态已就绪，等待同步点火")
                try:
                    ignition_barrier.wait(timeout=12.0)
                    print(f"[{label}] 同步返场点火放行，UT={ut():.3f}")
                except threading.BrokenBarrierError:
                    print(f"[{label}] 同步点火等待超时，改为独立点火以保住回收")
                ignition_synchronized = True
            throttle_driver.set(throttle)
            height, vs, hs, desired, alignment = telemetry
            time_to_contact = height / max(-vs, 1.0) if vs < 0 else math.inf
            if (not gear_set and vs < 0 and
                    ((time_to_contact <= cfg.gear_deploy_lead_seconds and
                      height <= cfg.gear_deploy_max_height) or
                     height <= cfg.gear_emergency_height)):
                already_deployed = deploy_landing_legs(landing_legs)
                gear_set = True
                gear_command_ut = now
                gear_retry_ut = now
                gear_report_ut = now
                print(f"已向 {len(landing_legs)} 条支腿发送展开指令"
                      f"（此前明确展开 {already_deployed} 条，匀速外推 {time_to_contact:.1f}s）")
            # 某些可展开支腿会在动压过高时暂时拒绝第一次命令。只发一次并
            # 在三秒后停止检查，会让箭体在支腿仍收起时耗尽燃料。保持幂等
            # 重发；一旦四条支腿都回读为展开便永久停止重发。
            if (gear_set and not gear_verified and gear_retry_ut is not None and
                    now - gear_retry_ut >= 1.0):
                deploy_landing_legs(landing_legs)
                gear_retry_ut = now
            if (gear_set and not gear_verified and gear_report_ut is not None and
                    now - gear_report_ut >= 3.0):
                deployed = sum(leg.deployed for leg in landing_legs)
                print(f"支腿展开回读：{deployed}/{len(landing_legs)}")
                gear_report_ut = now
                gear_verified = deployed == len(landing_legs)
            if (not grids_retracted and grid_modules and
                    height <= cfg.grid_retract_height):
                commanded = retract_grid_fins(grid_modules)
                grids_retracted = True
                print(f"末段收回栅格舵：已向 {commanded}/{len(grid_modules)} 片发送指令")
            if now - last_log >= 1:
                fuel_percent = (100 * fuel_store.amount("LiquidFuel") / fuel_capacity
                                if fuel_capacity > 0 else 0.0)
                location = (f" 纬度={flight.latitude:.7f} 经度={flight.longitude:.7f}"
                            if height <= 500 else "")
                print(f"[{log_tag}] {guidance.state:7} 支腿离地≈{height:7.1f}m "
                      f"垂速={vs:7.1f} 水平={hs:6.1f} 目标垂速={desired:6.1f} "
                      f"燃料={fuel_percent:5.1f}% 油门={throttle:.2f} "
                      f"姿态对齐={alignment:.2f} "
                      f"目标距离={guidance.target_distance:.0f}m{location}")
                last_log = now
    finally:
        if throttle_driver is not None:
            throttle_driver.close()
        if controlling and vessel is not None:
            # 每项独立尝试，连接异常时仍尽量完成其它清理。
            for cleanup in (
                lambda: setattr(vessel.control, "throttle", 0.0),
                lambda: setattr(vessel.auto_pilot, "engaged", False),
                lambda: setattr(vessel.control, "rcs", False),
            ):
                try:
                    cleanup()
                except Exception as exc:
                    print(f"清理未完成（请检查游戏控制状态）: {exc}")
        for s in streams:
            try:
                s.remove()
            except Exception:
                pass
        if connection is None:
            conn.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("用户终止回收。")
    except Exception as exc:
        print(f"回收停止: {exc}")
        raise SystemExit(1)
