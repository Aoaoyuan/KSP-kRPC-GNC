"""读档后重新解析载具，并验证实际油门输入通道。"""
import time


def deploy_grid_fins(vessel):
    """展开本工程 T-222 栅格舵；重复调用不会把已展开的舵面收回。

    ModuleControlSurface.deploy 是舵面的固定偏转开关，并不是折叠展开。
    真正的折叠由 ModuleAnimateGeneric 管理。已实测此型号 animTime=0
    为收起、1 为展开；仅在事件明确显示 Extend Fins 时触发 Toggle。
    返回动画模块，调用方可用 grid_fins_deployed 做终态回读。
    """
    modules = [m for p in vessel.parts.all if 'Grid Fin' in p.title
               for m in p.modules if m.name == 'ModuleAnimateGeneric']
    for module in modules:
        for event in module.event_list:
            if event.name == 'Toggle' and event.active and event.gui_name == 'Extend Fins':
                event.trigger()
    return modules


def grid_fins_deployed(modules):
    """只有动画到达展开终点并锁定才计数；LOCKED 本身不区分收起/展开。"""
    count = 0
    for module in modules:
        fields = {f.name: f.value for f in module.field_list
                  if f.name in ('animTime', 'aniState')}
        if float(fields.get('animTime', '0')) >= .99 and fields.get('aniState') == 'LOCKED':
            count += 1
    return count


def retract_grid_fins(modules):
    """收回已经展开的 T-222 栅格舵，返回发出指令的舵面数量。

    末段发动机已经能够独立消除水平速度，栅格舵继续伸展只会增加触地或
    入水时折断的风险。动画事件的显示名会随状态在 Extend/Retract Fins
    之间切换，因此只触发明确的 ``Retract Fins``，重复调用也不会反向展开。
    """
    commanded = 0
    for module in modules:
        for event in module.event_list:
            if (event.name == 'Toggle' and event.active and
                    event.gui_name == 'Retract Fins'):
                event.trigger()
                commanded += 1
                break
    return commanded


def find_booster(sc, tag="booster"):
    matches = []
    for vessel in sc.vessels:
        if vessel.loaded and vessel.parts.with_tag(tag):
            matches.append(vessel)
    if len(matches) > 1:
        raise RuntimeError("附近有多个助推器标记，不能自动选择，请保留唯一标签")
    return matches[0] if matches else None


def ready_vessel(sc, timeout=15):
    deadline = time.monotonic() + timeout
    previous = None
    stable = 0
    while time.monotonic() < deadline:
        vessel = sc.active_vessel
        if vessel is not None and vessel.loaded and not vessel.packed:
            engines = vessel.parts.engines
            signature = (vessel, len(vessel.parts.all), len(engines))
            stable = stable + 1 if signature == previous else 0
            previous = signature
            if engines and stable >= 2:
                return vessel
        time.sleep(.15)
    raise RuntimeError("载具物理或发动机列表尚未就绪，请完成读档后重试")


class ThrottleController:
    """总油门回读失败时使用已验证的发动机独立油门；不触发分级。"""
    def __init__(self, vessel, *, force_independent=False):
        self.vessel = vessel
        # Sepratron 等固体发动机与液体主发动机同属一个 Vessel，但它们的
        # throttle_locked=True，既不能连续调节也不能重新点火。后台回收只
        # 接管可节流发动机；否则独立油门自检会把正常的固体发动机误判成故障。
        self.engines = [e for e in vessel.parts.engines if not e.throttle_locked]
        self.original = [(e, e.independent_throttle, e.throttle) for e in self.engines]
        self.independent = bool(force_independent)
        self.value = 0.0
        if not self.engines:
            raise RuntimeError("没有识别到发动机；这与发动机未激活不同")
        try:
            # 仅 --execute 路径调用。短暂低油门探测，不释放发射架。
            if not self.independent:
                vessel.control.throttle = .03
                time.sleep(.25)
                actual = vessel.control.throttle
                vessel.control.throttle = 0
                self.independent = (abs(actual - .03) > .015 or
                                    any(flag for _, flag, _ in self.original))
            if self.independent:
                for e in self.engines:
                    e.throttle = 0
                    e.independent_throttle = True
                for e in self.engines:
                    e.throttle = .03
                    time.sleep(.06)
                    if not e.independent_throttle or (e.active and abs(e.throttle - .03) > .01):
                        raise RuntimeError("发动机独立油门回读也失败，禁止继续飞行")
                    e.throttle = 0
            print("油门通道：" + ("发动机独立油门（总油门回读异常或已启用独立油门）"
                              if self.independent else "载具总油门，自检通过"))
        except BaseException:
            self.close()
            raise

    def set(self, value):
        value = max(0.0, min(1.0, float(value)))
        self.value = value
        if self.independent:
            for engine in self.engines:
                engine.throttle = value
        else:
            self.vessel.control.throttle = value

    def close(self):
        failures = []
        try:
            self.vessel.control.throttle = 0.0
        except Exception as exc:
            failures.append(str(exc))
        for e, was_independent, _ in self.original:
            try:
                e.throttle = 0
                e.independent_throttle = was_independent
            except Exception as exc:
                failures.append(str(exc))
        if failures:
            print("部分油门清理失败：", "; ".join(failures))
