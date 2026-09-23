# PRE 载荷抖动修复

本项目的三芯任务需要 PRE 让远离活动镜头的三枚芯级继续接受物理模拟。原先的白名单修正版还会给当前载荷应用 2000 km 物理范围；PRE 官方说明指出，超过 100 km 可能导致载具抖动和异常力。载荷分离后短暂关闭再开启 PRE 能消除抖动，是物理范围重新设置的线索，但仍需要实飞验证。

本次修复分两处：

- MyHeavyLaunch.py 在载荷为当前活动载具时，把其物理范围设回 2500 m；若玩家仍在观看侧芯，则给后台载荷保留 2000 km，避免它尚在大气内时被 KSP 卸载。
- patches/PhysicsRangeExtender-managed-payload.patch 修改 PRE 的白名单逻辑：活动载荷使用游戏默认范围；玩家切到别的芯级时，切换事件立即恢复载荷的扩展范围。三枚带标签的芯级仍使用扩展范围；空间站等无标签载具保持默认范围。

补丁基于 jrodrigv/PhysicsRangeExtender 的提交 4b726aeba4bb202dcd4b9776c62ad0e6ca678ad1。在该源码仓库根目录应用本项目的补丁，按本机 KSP 路径配置 C# 项目引用后编译 Release。编译得到的 PhysicsRangeExtender.dll 需在**退出 KSP 后**备份并替换 GameData/PhysicsRangeExtender/Plugins/PhysicsRangeExtender.dll，下次启动才会生效。

当前修复已通过 C# Release 编译和 Python 离线测试。尚未在游戏内完成“载荷活动状态平稳、切到双侧芯时载荷仍存活”的实飞验收。验收前应使用存档副本；之前的原版 PRE 曾因给空间站扩展物理范围造成掉轨。