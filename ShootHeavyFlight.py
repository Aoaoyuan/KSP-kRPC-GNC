"""三芯重型任务的一键摄影发射入口。

飞行仍完全由经过实飞验证的 ``MyHeavyLaunch.py`` 执行；相机由独立的
``MissionCamera.py`` 线程控制。摄影线程只读任务状态并改相机，不写任何
GNC、分级或回收命令。

推荐正式成片录两遍：

* ``--profile master``：中央芯推送、中央芯分离、再入与载荷结尾；
* ``--profile boosters``：侧芯分离、翻转、同步返场和双箭着陆；

``recap`` 是方便预览的一遍式配置，默认使用它。
"""

from __future__ import annotations

import argparse
import threading
import time

import MissionCamera
import MyHeavyLaunch


def require_prelaunch_state():
    """摄影初始化前确认游戏确实处于可发射状态。"""
    import krpc

    conn = krpc.connect(name="Cinematic launch readiness check")
    try:
        sc = conn.space_center
        vessel = MyHeavyLaunch.ready_vessel(sc)
        if (vessel.situation != sc.VesselSituation.pre_launch or
                vessel.control.current_stage != 5):
            raise RuntimeError(
                "摄影任务尚未启动：请读取三芯火箭发射前存档，确认火箭仍在"
                "发射架上、界面 current stage 为 5，并解除游戏暂停后重试。"
                f"当前状态={vessel.situation}，current stage="
                f"{vessel.control.current_stage}")
    finally:
        conn.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=("recap", "master", "boosters"),
                        default="recap", help="本轮摄影优先级")
    parser.add_argument("--allow-warp", action="store_true",
                        help="允许拍摄者手动加速；脚本本身仍不会主动加速")
    parser.add_argument("--no-final-pause", action="store_true",
                        help="载荷结尾镜头后不自动暂停游戏")
    parser.add_argument("--cue-file",
                        help="可选的剪辑标记 CSV 路径；默认使用时间戳文件名")
    args = parser.parse_args(argv)

    # 先只读检查结构。检查失败时不会运行导演，也不会点火。
    MyHeavyLaunch.main([])
    require_prelaunch_state()

    print("摄影提示：请让游戏保持 1× 运行并开始录屏。")
    print("导演会先拍六秒低机位和四秒发动机特写；现在按 F2 隐藏 HUD。")

    stop_event = threading.Event()
    ready_event = threading.Event()
    camera_thread = threading.Thread(
        target=MissionCamera.run_director,
        args=(stop_event,),
        kwargs={
            "profile": args.profile,
            "pause_when_done": not args.no_final_pause,
            "ready_event": ready_event,
            "cue_path": args.cue_file,
        },
        name="mission-camera",
        daemon=True,
    )
    camera_thread.start()

    # 等低机位和发动机特写拍完后再进入倒计时。镜头初始化失败时禁止发射，
    # 以免用户以为已经得到开场素材。
    if not ready_event.wait(timeout=35.0) or stop_event.is_set():
        stop_event.set()
        camera_thread.join(timeout=3.0)
        raise RuntimeError("摄影导演未能完成开场镜头，已取消发射")

    for number in (3, 2, 1):
        print(f"电影模式发射倒计时：{number}", flush=True)
        time.sleep(1.0)

    launch_finished = False
    try:
        launch_args = ["--execute"]
        if args.allow_warp:
            launch_args.append("--allow-warp")
        MyHeavyLaunch.main(launch_args)
        launch_finished = True
    finally:
        # 正常任务结束后，导演还需要九秒载荷结尾镜头再暂停。
        if launch_finished:
            camera_thread.join(timeout=14.0)
        stop_event.set()
        camera_thread.join(timeout=3.0)


if __name__ == "__main__":
    main()
