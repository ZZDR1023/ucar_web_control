# ROSCAR1

基于 ROS 1 Melodic 的实体小车控制、Web 驾驶舱、地图导航、巡逻、语音控制、人体跟随和视觉巡线实验项目。

> 实车项目有物理风险。运行任何会发布 `/cmd_vel` 的功能前，必须确认现场安全、底盘/雷达状态正常，并保证有人可以立即 STOP。

## 当前推荐入口

### Web 面板

小车在线后，在浏览器访问：

```text
http://<小车当前 IP>:8080/
```

当前 IP 会随网络变化，先实时确认小车地址，不要只依赖历史文档中的固定 IP。systemd 可通过 `/home/ucar/roscar.env` 固定 `ROBOT_IP`、`ROS_MASTER_URI` 和 `ROS_IP`；不配置时会尝试使用当前主机首个 IP。

Web 面板主要能力：

- 手动低速控制 `/cmd_vel`；
- STOP / 接管 / 恢复导航；
- Map 页面点击地图发送 `map` 坐标系导航目标；
- 显示 AMCL 位姿、导航节点状态、雷达状态、电量和前方安全状态；
- Camera MJPEG 预览；
- 巡逻路线保存、加载、执行和到点拍照记录；
- 浏览器语音命令解析与安全执行；
- 人体跟随 Web 封装；
- 视觉巡线代码保留为实验资产，当前不作为推荐实车路线方案。

### Systemd 服务

推荐在小车端用 systemd 托管底盘/雷达和 Web 后端，避免手动 SSH 后前台运行：

```bash
sudo cp /home/ucar/systemd/ucar_sensors.service /etc/systemd/system/ucar_sensors.service
sudo cp /home/ucar/systemd/ucar_web.service /etc/systemd/system/ucar_web.service
sudo systemctl daemon-reload
sudo systemctl enable ucar_sensors.service ucar_web.service
sudo systemctl restart ucar_sensors.service ucar_web.service
```

查看状态：

```bash
systemctl status ucar_sensors.service --no-pager
systemctl status ucar_web.service --no-pager
```

如果 Web 已由 `ucar_web.service` 托管，不要再手动运行 `python3 /home/ucar/web_panel/server.py`，否则会和 systemd 进程抢占 8080 端口。

## 项目结构

```text
.
├── web_panel/                  # Flask Web 后端、页面模板、视觉/语音/跟随适配器
│   ├── server.py               # Web 控制面板后端入口
│   ├── templates/index.html    # 单页驾驶舱界面
│   ├── patrol.py               # 巡逻路线和执行记录
│   ├── voice_control.py        # 中文语音命令解析
│   ├── person_follow_adapter.py# 人体跟随适配器
│   ├── ucar_camera_bridge.py   # USB 相机到 ROS Image 的桥接脚本
│   ├── line_follow.py          # Web 视觉巡线实验逻辑
│   └── line_record.py          # 巡线样本记录
├── systemd/                    # 小车端服务单元
├── maps/                       # 原始地图和封门地图
├── src/line_follower.cpp       # C++ 视觉巡线实验节点
├── simple_line_follower.py     # 极简 Python 视觉巡线实验脚本
├── tests/                      # Python/C++ 静态与行为回归测试
├── NOTES/                      # 调试记录、Bug 记录、命令速查
├── CMakeLists.txt              # catkin C++ 节点构建入口
└── package.xml                 # catkin 包描述
```

### 为什么 `src/` 里只有一个 C++ 文件

这个仓库不是传统“所有源码都放 `src/`”的单一 C++ 项目，而是按小车实际部署方式组织：

- `src/line_follower.cpp`：唯一需要 catkin 编译的 C++ ROS 实验节点，所以放在 `src/`。
- `web_panel/`：小车 Web 控制台的主要代码，部署到 `/home/ucar/web_panel/` 后由 Flask 运行。
- 根目录 `start_*.sh`：小车 SSH 登录后直接执行的一键启动脚本，也会被 systemd 间接调用。
- 根目录 `simple_line_follower.py`：不经过 Web 面板的极简 Python 巡线实验脚本，保留作对照实验。
- `systemd/`：安装到 `/etc/systemd/system/` 的服务文件，用于守护底盘/雷达和 Web 后端。
- `maps/`：导航用原始地图、封门地图和预览图。
- `NOTES/`：调试记录、Bug 记录和命令速查，不是运行时代码。
- `tests/`：本地回归测试和静态检查。

因此 GitHub 上看起来“代码分散”是有意的：Python/Flask 文件按小车运行路径放，只有 C++ catkin 节点放 `src/`。

## 小车 ROS 环境

小车系统：Ubuntu 18.04 + ROS 1 Melodic。

每个新的 SSH 终端运行 ROS 命令前先加载环境，并把 IP 改成小车当前地址：

```bash
source /opt/ros/melodic/setup.bash
source /home/ucar/ucar_ws/devel/setup.bash
export ROS_MASTER_URI=http://<小车当前 IP>:11311
export ROS_IP=<小车当前 IP>
```

常用硬件关系：

```text
底盘串口：/dev/base_serial_port -> /dev/ttyUSB0
雷达串口：/dev/ttyTHS1
摄像头：/dev/ucar_video -> /dev/video0
```

雷达必须使用 `/dev/ttyTHS1`，不要把 `/dev/ttyUSB0` 当雷达串口；`/dev/ttyUSB0` 是底盘串口。

## 日常验证命令

```bash
rostopic list
rostopic hz /scan
rostopic hz /odom
rostopic info /cmd_vel
```

预期：

- `/scan` 约 10Hz；
- `/odom` 约 20Hz；
- `/cmd_vel` 有底盘订阅者 `/base_driver`；
- Web 首页 `http://<小车当前 IP>:8080/` 返回 200。

## 导航与地图

导航使用封门地图，避免小车规划到门外区域：

```text
/home/ucar/ucar_ws/maps/ucar_map_20260501_202713_sealed.yaml
```

Web Map 页面使用 `map` 坐标，不要把 `/odom` 坐标直接当导航目标。首次启动导航或 AMCL 未收敛时，先在 Web Map 上点击小车真实位置，再使用 `Set AMCL Pose Here` 设置初始位姿。

如果已经启动了 `ucar_sensors.service` 或 `./start_sensors.sh`，后续导航入口必须避免重复启动底盘和雷达，防止同名节点互相挤掉。

## 巡逻与语音

巡逻推荐流程：

1. 在 Map 上点击目标点；
2. 点击“使用当前地图目标”或“使用小车位置”；
3. 填写点位名称并添加巡逻点；
4. 填写路线名称并保存；
5. 从下拉框加载路线后开始巡逻。

语音控制先使用浏览器 Web Speech API。浏览器不支持或识别中断时，可以在同一输入框手动输入命令。示例：

```text
停止
前进1米再左转
开始巡逻
去t1中的1号巡逻点
开始人体跟随
停止跟随
```

所有语音命令都经过 Web 后端安全状态机，不应另写直接发布 `/cmd_vel` 的语音节点。

## 人体跟随

人体跟随必须通过 Web 后端启动，当前后端实际启动：

```bash
python3 -u /home/ucar/web_panel/person_follow_adapter.py
```

不要直接裸跑原厂人体跟随节点，因为它会控制 `/cmd_vel`，可能绕开 Web STOP、手动接管、巡逻/导航互斥和相机释放逻辑。

常用状态接口：

```bash
curl -s http://<小车当前 IP>:8080/api/person_follow/status
curl -s -X POST http://<小车当前 IP>:8080/api/person_follow/start
curl -s -X POST http://<小车当前 IP>:8080/api/person_follow/stop
```

实车测试要求：空间足够、有人看护、STOP 可用。若检测不稳定或目标丢失，优先查看 `detected`、`lost_frames`、`follow_enabled` 和相机桥日志。

## 视觉巡线实验状态

视觉巡线相关代码仍保留：

- `web_panel/line_follow.py`：Web 版算法；
- `simple_line_follower.py`：极简 Python ROS 实验脚本；
- `src/line_follower.cpp`：C++ 实验节点。

但当前结论是：地图/赛道语义工具缺失前，不继续把视觉巡线作为推荐实车方案。实验节点会直接或间接发布 `/cmd_vel`，运行前必须关闭 Web 巡线、导航、巡逻和人体跟随，并由人在车旁看护。

C++ 节点构建示例：

```bash
source /opt/ros/melodic/setup.bash
source /home/ucar/ucar_ws/devel/setup.bash
cd /home/ucar/ucar_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES=roscar1
source devel/setup.bash
```

## 本地测试

本仓库以 `pytest` 为主：

```bash
python3 -m pytest tests
```

常用轻量验证：

```bash
python3 -m py_compile web_panel/server.py web_panel/person_follow_adapter.py web_panel/voice_control.py web_panel/line_follow.py
python3 -m pytest tests/test_person_follow_adapter.py tests/test_voice_control.py tests/test_web_panel_server_static.py
```

## 重要文档

- `CLAUDE.md`：AI/开发代理项目规则，包含实车安全铁律；
- `NOTES/bug_notes.md`：已踩坑、根因和修复记录；
- `NOTES/command_reference.md`：命令速查表；
- `todo.md`：历史待办和阶段记录。

## 应急停止

优先使用 Web STOP。若 Web 无响应，SSH 到小车后执行：

```bash
pkill -f 'person_follow.py|person_follow_node|person_follow_adapter.py|line_follower_node|simple_line_follower.py'
rostopic pub -r 20 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

只有确认小车已经停稳后，再继续排查 Web、ROS 节点或相机占用问题。
