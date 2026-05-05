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
- 巡逻功能支持“点击地图 -> 使用当前目标 -> 添加巡逻点 -> 开始巡逻”，到点后会保存相机照片。

## 小车端启动

推荐使用 systemd 管理底盘/雷达和 Web 服务，这样不用每次手动启动 `start_sensors.sh` 和 `python3 server.py`。

首次安装或更新服务文件：

```bash
sudo cp /home/ucar/systemd/ucar_sensors.service /etc/systemd/system/ucar_sensors.service
sudo cp /home/ucar/systemd/ucar_web.service /etc/systemd/system/ucar_web.service
sudo systemctl daemon-reload
sudo systemctl enable ucar_sensors.service ucar_web.service
sudo systemctl restart ucar_sensors.service ucar_web.service
```

日常检查：

```bash
systemctl status ucar_sensors.service --no-pager
systemctl status ucar_web.service --no-pager
```

如果 Web 已由 `ucar_web.service` 托管，不要再手动运行 `python3 server.py`，否则会因为 8080 端口已被占用而报 `Address already in use`。

手动启动方式仅用于临时调试：

```bash
source /opt/ros/melodic/setup.bash
source /home/ucar/nav_clean_ws/devel/setup.bash
export ROS_MASTER_URI=http://10.90.122.179:11311
export ROS_IP=10.90.122.179
cd /home/ucar/web_panel
python3 server.py
```
