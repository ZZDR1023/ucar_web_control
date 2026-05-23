# XF-ROBOT-UCAR01 小车说明书

本文档面向接手、部署和演示 XF-ROBOT-UCAR01 小车的人，集中说明硬件、系统、设备路径、软件服务和安全边界。更细的故障记录见 `NOTES/bug_notes.md`，命令速查见 `NOTES/command_reference.md`。

## 1. 基本信息

| 项目 | 配置 |
|---|---|
| 系统 | Ubuntu 18.04 |
| ROS | ROS 1 Melodic |
| 开发板/主控 | UCAR 小车随车 Linux 开发板；当前仓库可确认系统与接口，具体 SoC 型号以后以 `lscpu`/厂商资料补充 |
| 网络 | 无线网卡 `RTL8821AE`，接口通常为 `wlan0`，IP 会随网络变化 |
| Web 面板 | `http://<小车当前 IP>:8080/` |
| ROS Master | `http://<小车当前 IP>:11311` |
| 主要用户 | `ucar` |
| 常用工作目录 | `/home/ucar`、`/home/ucar/web_panel`、`/home/ucar/ucar_ws`、`/home/ucar/nav_clean_ws` |

> 操作前必须实时确认当前 IP。不要把历史 IP 写死到脚本、systemd 或文档命令里。

## 2. 硬件与设备路径

| 硬件 | 系统识别/路径 | 说明 |
|---|---|---|
| 底盘控制板串口 | `/dev/base_serial_port -> /dev/ttyUSB0` | 底盘驱动 `base_driver` 使用，不能拿来启动雷达 |
| 激光雷达 | `/dev/ttyTHS1` | YDLidar G4，启动雷达必须使用这个串口 |
| USB 摄像头 | `/dev/ucar_video -> /dev/video0` | Web 预览、人体跟随、巡线实验共用，不能被多个进程同时独占 |
| 麦克风阵列 | USB `10d6:b003 Actions Semiconductor` | 当前 Web 语音优先使用浏览器 Web Speech API，小车端麦克风 ASR 未作为主链路 |
| 底盘 USB 串口芯片 | USB `10c4:ea60 CP210x UART Bridge` | 对应底盘串口 |
| 摄像头 USB 设备 | USB `0edc:2050 USB Camera` | 对应 `/dev/video0` |

设备检查：

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/ttyTHS1 /dev/ydlidar /dev/base_serial_port /dev/ucar_video /dev/video*
lsusb
```

## 3. 软件工作空间

| 路径 | 用途 |
|---|---|
| `/home/ucar/ucar_ws` | 厂商原生 ROS 工作空间，负责底盘、雷达、建图和原厂包 |
| `/home/ucar/nav_clean_ws` | 干净导航工作空间，用于绕开厂商覆盖版 `move_base` 的不稳定问题 |
| `/home/ucar/web_panel` | Flask Web 控制面板部署目录 |
| `/home/ucar/ucar_ws/maps` | 地图文件目录，导航优先使用封门地图 |

常用地图：

```text
/home/ucar/ucar_ws/maps/ucar_map_20260501_202713_sealed.yaml
```

## 4. 仓库文件与小车部署关系

| 仓库路径 | 小车端位置/用途 |
|---|---|
| `web_panel/server.py` | `/home/ucar/web_panel/server.py`，Flask 后端入口 |
| `web_panel/templates/index.html` | Web 前端页面 |
| `web_panel/patrol.py` | 巡逻路线、巡逻状态和执行记录管理 |
| `web_panel/voice_control.py` | 中文语音命令解析 |
| `web_panel/person_follow_adapter.py` | 人体跟随适配器，统一走 Web STOP 和控制互斥 |
| `web_panel/ucar_camera_bridge.py` | 直接读 `/dev/video0` 并发布 `/ucar_camera/image_raw` |
| `web_panel/line_follow.py` | Web 视觉巡线实验算法，当前不作为推荐实车方案 |
| `web_panel/line_record.py` | 巡线人工样本记录器 |
| `start_*.sh` | 小车 `/home/ucar/` 下的一键启动脚本，已改为动态 IP |
| `systemd/*.service` | 安装到 `/etc/systemd/system/` 后守护 sensors 和 Web |
| `src/line_follower.cpp` | 需要 catkin 编译的 C++ 视觉巡线实验节点 |
| `simple_line_follower.py` | 极简 Python 巡线实验脚本，不经过 Web 安全层 |
| `maps/` | 本地保存的地图与预览文件 |
| `NOTES/bug_notes.md` | Bug、根因和解决方法记录 |
| `NOTES/command_reference.md` | 命令速查表 |

## 5. ROS 环境变量

每个新的 SSH 终端运行 ROS 命令前先执行：

```bash
source /opt/ros/melodic/setup.bash
source /home/ucar/ucar_ws/devel/setup.bash
export ROS_MASTER_URI=http://<小车当前 IP>:11311
export ROS_IP=<小车当前 IP>
```

`systemd/ucar_sensors.service` 和 `systemd/ucar_web.service` 支持可选环境文件：

```text
/home/ucar/roscar.env
```

示例：

```bash
ROBOT_IP=10.x.x.x
ROS_MASTER_URI=http://10.x.x.x:11311
ROS_IP=10.x.x.x
```

如果不提供该文件，服务会尝试用 `hostname -I` 的第一个 IPv4 地址作为小车 IP。

## 6. 启动方式

### 推荐：systemd 守护

```bash
sudo cp /home/ucar/systemd/ucar_sensors.service /etc/systemd/system/ucar_sensors.service
sudo cp /home/ucar/systemd/ucar_web.service /etc/systemd/system/ucar_web.service
sudo systemctl daemon-reload
sudo systemctl enable ucar_sensors.service ucar_web.service
sudo systemctl restart ucar_sensors.service ucar_web.service
```

检查状态：

```bash
systemctl status ucar_sensors.service --no-pager
systemctl status ucar_web.service --no-pager
```

### 临时手动启动

```bash
./start_sensors.sh      # 底盘 + 雷达
./start_control.sh      # 键盘控制台
./start_nav_clean.sh    # 干净导航
```

如果 Web 已由 systemd 托管，不要再手动执行 `python3 /home/ucar/web_panel/server.py`，否则会抢占 8080 端口。

## 7. Web 面板功能

Web 地址：

```text
http://<小车当前 IP>:8080/
```

主要功能：

- 手动低速方向控制；
- STOP、手动接管、恢复导航；
- Map 页面点击地图发送 `map` 坐标目标；
- AMCL 位姿、雷达状态、电量和前方安全状态显示；
- Camera MJPEG 预览；
- 巡逻路线保存、加载、执行和到点拍照；
- 浏览器语音命令；
- 人体跟随启动/停止。

所有会让小车运动的入口都应经过 Web 后端安全状态机。不要另写绕过 Web STOP 的控制节点。

## 8. 相机使用规则

`/dev/video0` 不能被多个进程稳定同时读取。常见占用者：

- Web Camera 预览；
- `ucar_camera_bridge.py`；
- 原厂 `ucar_camera.py`；
- 人体跟随；
- 视觉巡线样本记录。

检查占用：

```bash
fuser -v /dev/video0 /dev/ucar_video
```

人体跟随启动前，Web 后端会尝试停止预览和样本记录，并清理残留相机进程。若相机桥不发布图像，先看：

```bash
sed -n '1,160p' /tmp/ucar_camera_bridge.log
```

## 9. 人体跟随

人体跟随必须通过 Web 面板或 Web API 启动：

```bash
curl -s -X POST http://<小车当前 IP>:8080/api/person_follow/start
curl -s -X POST http://<小车当前 IP>:8080/api/person_follow/stop
curl -s http://<小车当前 IP>:8080/api/person_follow/status
```

当前后端启动的是：

```bash
python3 -u /home/ucar/web_panel/person_follow_adapter.py
```

它保留 `/person_follow/*` 服务名，但外层由 Web 控制 STOP、相机释放、导航/巡逻/手动互斥和限速逻辑。

## 10. 视觉巡线实验

视觉巡线代码保留，但当前不是推荐实车主方案。原因是地图/赛道语义工具不足，继续靠颜色规则叠加容易不稳定。

保留入口：

- Web 算法：`web_panel/line_follow.py`；
- 极简 Python 实验：`simple_line_follower.py`；
- C++ 实验节点：`src/line_follower.cpp`。

这些实验可能直接发布 `/cmd_vel`。运行前必须关闭 Web 巡线、导航、巡逻和人体跟随，并有人在车旁看护。

## 11. 安全边界

- `/cmd_vel` 测试必须低速：线速度建议不超过 `0.10m/s`，角速度不超过 `0.50rad/s`。
- 雷达 `/scan` 正常频率约 `10Hz`；里程计 `/odom` 正常频率约 `20Hz`。
- 导航必须使用封门地图，避免小车驶出门外。
- 不要重复启动同名节点；`start_sensors.sh` 已启动底盘和雷达后，导航应使用不重复拉起底盘/雷达的运行版。
- Web STOP 无响应时，立即 SSH 发布 20Hz 零速度。

应急停止：

```bash
pkill -f 'person_follow.py|person_follow_node|person_follow_adapter.py|line_follower_node|simple_line_follower.py'
rostopic pub -r 20 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

## 12. 交接检查清单

1. 确认当前小车 IP；
2. 确认 `/dev/ttyTHS1`、`/dev/base_serial_port`、`/dev/ucar_video` 存在；
3. 确认 `ucar_sensors.service` 和 `ucar_web.service` 状态；
4. 确认 `/scan`、`/odom`、`/cmd_vel` 话题存在；
5. 打开 Web 面板并检查 Safety 卡片；
6. 只在空旷场地低速测试手动控制、导航、巡逻或人体跟随。
