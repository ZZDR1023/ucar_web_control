# ros1小车

ROS 1 Melodic 小车控制与 Web 面板项目。

本仓库的项目指引以 `CLAUDE.md` 为准；Codex/OpenCode 入口 `AGENTS.md` 会指回该文件。

## Web 面板

浏览器访问：

```text
http://10.90.122.179:8080
```

常用能力：

- 手动方向控制发布 `/cmd_vel`；
- `STOP` 取消导航目标并停止底盘；
- `GO` 发布 `/move_base_simple/goal`；
- Map 页显示封门地图预览和导航节点状态。

## 小车端启动

```bash
source /opt/ros/melodic/setup.bash
source /home/ucar/nav_clean_ws/devel/setup.bash
export ROS_MASTER_URI=http://10.90.122.179:11311
export ROS_IP=10.90.122.179
cd /home/ucar/web_panel
python3 server.py
```

