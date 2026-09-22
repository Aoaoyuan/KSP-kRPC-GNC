"""记录当前载具所在停机坪的经纬度和相对地形高度。

用法：把一个小型标记载具停在屋顶停机坪中心，切换为活动载具后运行：

    python CalibrateLandingSite.py --name vab-pad-left

输出的 --landing-latitude、--landing-longitude 和 --landing-surface-offset
可直接传给 MyLaunchV2.py。脚本只读取遥测，不会写入游戏状态。
"""
import argparse


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", required=True, help="给此停机坪取的稳定名称")
    parser.add_argument("--leg-offset", type=float, default=0.0,
                        help="标记载具质心到其接触面的高度；未知时保持 0")
    args = parser.parse_args()
    if args.leg_offset < 0:
        parser.error("--leg-offset 必须非负")

    import krpc
    conn = krpc.connect(name="Landing site calibration")
    try:
        vessel = conn.space_center.active_vessel
        frame = vessel.orbit.body.reference_frame
        flight = vessel.flight(frame)
        if vessel.situation not in (conn.space_center.VesselSituation.landed,
                                    conn.space_center.VesselSituation.pre_launch):
            raise RuntimeError("请先让标记载具稳定停在目标停机坪上")
        # surface_altitude 是标记载具质心相对地形的高度；减去标记本身的
        # 接触偏移后，得到楼顶或停机坪相对地形的标定高度。
        surface_offset = max(0.0, flight.surface_altitude - args.leg_offset)
        print(f"[{args.name}]")
        print(f"--landing-latitude {flight.latitude:.10f}")
        print(f"--landing-longitude {flight.longitude:.10f}")
        print(f"--landing-surface-offset {surface_offset:.2f}")
        print(f"# 地形海拔 {vessel.orbit.body.surface_height(flight.latitude, flight.longitude):.2f} m")
    finally:
        conn.close()


if __name__ == "__main__":
    main()
