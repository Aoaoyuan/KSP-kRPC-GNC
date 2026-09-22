# 蚱蜢跳与屋顶停机坪验证

## 当前验证状态（2026-09-21）

屋顶落稳尚未完成。`hopper-roof-02.log` 的着陆提示不代表成功：后续读取到的
载具仅剩 13 个部件、0 台发动机。现改为连续接地 8 秒后再验收部件、发动机、
支腿终态、竖直姿态和速度；这仍不能单凭 landed 状态证明落在停机坪范围内。

本轮低空试验使用 T-222 栅格舵的 `ModuleAnimateGeneric` 折叠事件。
地面已验证：`animTime=1` 且 `aniState=LOCKED` 为完全展开，四片均通过，
支腿仍为收起。`ModuleControlSurface.deploy` 是固定偏转，不能用来代替折叠。

横移采用位置外环和速度内环，并预留转向耗时。自身在
`vessel.surface_reference_frame` 中的速度为零：必须先在天体固连参考系读取
地速，再通过 `transform_direction` 转成当地上/北/东分量。第三次测试暴露并
修正了这一错误；第四次暴露横向振荡，因此第五次降低了增益。

本试验允许在目标高度短暂保持，等水平位置与速度收敛后再下降，目的是验证
定点能力；当前不能称为最省 Δv。正式脚本不主动提高游戏时间倍率。

支腿暂用“下降且匀速外推不超过 8 秒、同时低于 150 m”触发，另有 60 m
兜底。匀速外推不是考虑制动后的实际触地时间；该门限仍待实飞验证。

游戏暂停须通过 Escape 菜单确认。两种 warp factor 为 0 仅表示 1 倍速。

`HopperTest.py` 只验证一级的低空起飞、横移和着陆。它不进行载荷分离，也不会主动改变
游戏时间倍率；测试时可由操作者在无推力滑行段手动加速。

## 1. 标定屋顶中心

将一个小探测器或小车稳定停在某个屋顶停机坪的中心，切换为活动载具，然后运行：

```powershell
python CalibrateLandingSite.py --name vab-roof-left
```

记录输出的 `--landing-latitude`、`--landing-longitude` 和
`--landing-surface-offset`。第三项是楼顶高出地形的实际高度，缺失它会导致控制器误判
支腿离地高度。

## 2. 低空跳跃

从完整火箭的 `pre_launch` 状态运行。脚本按实际推重比计算离架油门，避免低油门被发射
卡箍困住；到达目标高度后关闭发动机，交给一次连续的着陆点火窗口。

```powershell
python HopperTest.py --execute --hop-altitude 800 `
  --landing-latitude <纬度> --landing-longitude <经度> `
  --landing-surface-offset <楼顶高度> --leg-offset 16 --grid-group 8
```

验收条件：四条支腿回读为展开、触地后发动机数量未减少、结构部件数量至少保留分离时的
80%。任一条件不满足都应判定为炸毁，不能拿该次落点修正导航。
