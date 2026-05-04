# ROSCAR1 命令速查表

本文档记录小车常用命令：每条命令启动什么部分、什么时候使用、预期结果是什么。

---

## 一、常用连接命令

### 1. 测试小车是否在线

```bash
ping -c 3 -W 2 10.90.122.179
```

作用：检查电脑是否能通过 Wi-Fi 连接到小车。

参数含义：

- `ping`：测试网络连通性。
- `-c 3`：发送 3 次测试包。
- `-W 2`：每次最多等待 2 秒。
- `10.90.122.179`：小车当前无线 IP。

什么时候用：

- 小车刚开机后；
- SSH 登录失败时；
- 怀疑小车没连上热点时。

正常结果：

```text
3 packets transmitted, 3 received, 0% packet loss
```

---

### 2. SSH 登录小车

```bash
ssh ucar@10.90.122.179
```

作用：远程登录小车 Linux 系统。

参数含义：

- `ssh`：远程登录命令。
- `ucar`：小车用户名。
- `10.90.122.179`：小车 IP。

密码：

```text
ucar
```

什么时候用：

- 需要在小车上启动 ROS；
- 需要查看小车文件；
- 需要执行 `./start_sensors.sh`、`./start_control.sh` 等脚本。

---

## 二、ROS 环境加载命令

如果你是手动执行 ROS 命令，通常需要先执行下面几条。

### 1. 加载 ROS1 Melodic 环境

```bash
source /opt/ros/melodic/setup.bash
```

作用：让当前终端识别 ROS1 命令，例如 `roslaunch`、`rostopic`、`rosrun`。

什么时候用：

- 手动运行 ROS 命令前；
- 新开 SSH 终端后。

---

### 2. 加载小车工作空间环境

```bash
source ~/ucar_ws/devel/setup.bash
```

作用：让当前终端识别小车自己的 ROS 包，例如：

```text
ucar_controller
ucar_practice
ydlidar_ros_driver
ucar_map
ucar_nav
```

什么时候用：

- 使用小车 ROS 包前；
- 执行 `rosrun ucar_practice ...` 前；
- 执行 `roslaunch ucar_practice ...` 前。

---

### 3. 设置 ROS Master 地址

```bash
export ROS_MASTER_URI=http://10.90.122.179:11311
```

作用：告诉 ROS，ROS Master 在小车自己的 IP 和 11311 端口上。

什么时候用：

- 手动运行 ROS 命令前；
- 新开 SSH 终端后。

---

### 4. 设置小车自身 ROS 通信 IP

```bash
export ROS_IP=10.90.122.179
```

作用：告诉 ROS 当前小车自己的通信 IP。

什么时候用：

- 手动运行 ROS 命令前；
- 新开 SSH 终端后。

---

## 三、一键启动脚本

这些脚本都在小车用户目录：

```text
/home/ucar
```

也就是 SSH 登录小车后默认所在目录。

---

### 1. 一键启动底盘

```bash
./start_base.sh
```

启动内容：

```text
底盘驱动 base_driver
ROS Master
/cmd_vel
/odom
/imu
/battery_state
/tf
```

内部实际执行：

```bash
roslaunch ucar_controller base_driver.launch
```

什么时候用：

- 只想控制小车运动；
- 不需要雷达；
- 调试底盘、遥控、里程计时。

启动后应出现话题：

```text
/cmd_vel
/odom
/imu
/battery_state
/tf
```

注意：

这个命令会占用当前终端，保持运行即可。

---

### 2. 一键启动雷达

```bash
./start_lidar.sh
```

启动内容：

```text
激光雷达驱动 ydlidar_ros_driver
/scan
/point_cloud
laser_frame 相关 tf
```

内部实际执行：

```bash
roslaunch ydlidar_ros_driver ucar_g4.launch
```

什么时候用：

- 只想测试雷达；
- 检查 `/scan`；
- 准备建图或导航。

启动后应出现话题：

```text
/scan
/point_cloud
```

正常频率：

```text
/scan 约 10 Hz
```

注意：

雷达正确串口是：

```text
/dev/ttyTHS1
```

不要使用 `/dev/ttyUSB0` 启动雷达，因为 `/dev/ttyUSB0` 是底盘串口。

---

### 3. 一键启动底盘 + 雷达

```bash
./start_sensors.sh
```

启动内容：

```text
底盘驱动
激光雷达驱动
ROS Master
/cmd_vel
/odom
/imu
/battery_state
/tf
/scan
/point_cloud
```

内部实际执行：

```bash
roslaunch ucar_practice sensors.launch
```

什么时候用：

- 推荐作为日常主启动命令；
- 做遥控 + 状态监控；
- 做雷达可视化；
- 做建图；
- 做导航前准备。

启动后应出现话题：

```text
/cmd_vel
/odom
/imu
/battery_state
/tf
/scan
/point_cloud
```

正常频率：

```text
/odom 约 20 Hz
/scan 约 10 Hz
```

注意：

这个命令会占用当前终端，保持运行即可。

推荐使用方式：

```text
终端 1：./start_sensors.sh
终端 2：./start_control.sh
```

---

### 4. 一键启动遥控 + 状态监控界面

```bash
./start_control.sh
```

启动内容：

```text
增强版联合控制台
键盘遥控
状态显示
/cmd_vel 发布
/odom /imu /battery_state 订阅
```

内部实际执行：

```bash
rosrun ucar_practice safe_control_with_status.py
```

什么时候用：

- 手动遥控小车；
- 查看小车状态；
- 练习前进、后退、转弯、弧线运动；
- 配合 `./start_sensors.sh` 使用。

按键说明：

| 按键 | 功能 |
|---|---|
| `w` | 设置前进 |
| `s` | 设置后退 |
| `a` | 设置左转 |
| `d` | 设置右转 |
| `w` 后按 `a` | 前进 + 左转 |
| `w` 后按 `d` | 前进 + 右转 |
| `z` | 只停止前进/后退 |
| `c` | 只停止旋转 |
| `x` | 急停，全部停止 |
| `1` | 慢速档 |
| `2` | 正常档 |
| `3` | 快速档 |
| `q` | 退出并停止 |

速度档位：

```text
1 档：0.10 m/s，0.50 rad/s
2 档：0.18 m/s，0.90 rad/s
3 档：0.28 m/s，1.30 rad/s
```

注意：

使用前需要先启动底盘，推荐先执行：

```bash
./start_sensors.sh
```

---

## 四、ROS 包运行命令

### 1. 从 ROS 包运行联合控制节点

```bash
rosrun ucar_practice safe_control_with_status.py
```

作用：直接从 `ucar_practice` 包运行增强版联合控制节点。

什么时候用：

- 不想用 `./start_control.sh`；
- 想手动执行 ROS 包里的节点；
- 调试 `ucar_practice` 包。

前提：

需要先执行：

```bash
source /opt/ros/melodic/setup.bash
source ~/ucar_ws/devel/setup.bash
export ROS_MASTER_URI=http://10.90.122.179:11311
export ROS_IP=10.90.122.179
```

---

### 2. 从 ROS 包启动底盘 + 雷达

```bash
roslaunch ucar_practice sensors.launch
```

作用：通过 launch 文件同时启动底盘和雷达。

启动内容：

```text
ucar_controller/base_driver.launch
ydlidar_ros_driver/ucar_g4.launch
```

什么时候用：

- 不想用 `./start_sensors.sh`；
- 调试 launch 文件；
- 学习 ROS launch 用法。

---

## 五、话题检查命令

### 1. 查看当前所有 ROS 话题

```bash
rostopic list
```

作用：查看当前 ROS 系统中正在发布或订阅的话题。

什么时候用：

- 检查底盘是否启动；
- 检查雷达是否启动；
- 检查 `/scan` 是否存在；
- 检查 `/odom` 是否存在。

底盘 + 雷达正常时应看到：

```text
/battery_state
/cmd_vel
/imu
/odom
/point_cloud
/scan
/tf
```

---

### 2. 查看 `/cmd_vel` 信息

```bash
rostopic info /cmd_vel
```

作用：查看速度控制话题的类型、发布者、订阅者。

正常结果应包含：

```text
Type: geometry_msgs/Twist
Subscribers:
 * /base_driver
```

什么时候用：

- 小车不动时；
- 检查底盘是否订阅速度命令；
- 调试遥控脚本时。

---

### 3. 查看雷达频率

```bash
rostopic hz /scan
```

作用：查看激光雷达 `/scan` 话题的发布频率。

正常结果：

```text
average rate: 10.0
```

什么时候用：

- 启动雷达后；
- 建图前；
- 导航前；
- 怀疑雷达数据不稳定时。

---

### 4. 查看里程计频率

```bash
rostopic hz /odom
```

作用：查看 `/odom` 里程计话题发布频率。

正常结果：

```text
average rate: 20.0
```

什么时候用：

- 启动底盘后；
- 小车控制异常时；
- 做闭环控制前。

---

### 5. 查看一帧雷达数据

```bash
rostopic echo -n 1 /scan
```

作用：读取一帧激光雷达数据。

参数含义：

- `echo`：打印话题消息。
- `-n 1`：只打印 1 条消息。
- `/scan`：激光雷达扫描话题。

正常结果应包含：

```text
ranges:
intensities:
```

什么时候用：

- 确认雷达是否真的有数据；
- 检查测距范围是否合理。

---

### 6. 查看一帧里程计数据

```bash
rostopic echo -n 1 /odom
```

作用：读取一帧小车里程计数据。

正常结果应包含：

```text
pose:
twist:
```

什么时候用：

- 检查小车位置估计；
- 检查速度反馈；
- 做定距运动或转角控制前。

---

## 六、设备检查命令

### 1. 查看底盘和雷达设备节点

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/ttyTHS1 /dev/ydlidar /dev/base_serial_port /dev/ucar_video
```

作用：查看小车硬件设备节点。

当前确认关系：

```text
底盘串口：/dev/base_serial_port -> /dev/ttyUSB0
雷达串口：/dev/ttyTHS1
摄像头：/dev/ucar_video -> /dev/video0
```

什么时候用：

- 雷达无法启动时；
- 底盘无法启动时；
- 摄像头无法启动时；
- 重启小车后确认设备是否存在。

---

### 2. 查看 USB 设备

```bash
lsusb
```

作用：列出系统识别到的 USB 硬件。

当前已见设备：

```text
10c4:ea60 CP210x UART Bridge      底盘串口
0edc:2050 USB Camera              摄像头
10d6:b003 Actions USB Misc Gadget 麦克风阵列
```

什么时候用：

- 判断硬件有没有被系统识别；
- 排查 USB 设备是否掉线。

---

## 七、停止和清理命令

### 1. 停止底盘驱动

```bash
pkill -x base_driver
```

作用：停止底盘驱动进程。

什么时候用：

- 需要重启底盘驱动；
- 底盘串口异常；
- 准备重新启动 `./start_sensors.sh`。

---

### 2. 停止雷达驱动

```bash
pkill -f ydlidar_ros_driver_node
```

作用：停止雷达驱动节点。

什么时候用：

- 需要重启雷达；
- `/scan` 异常；
- 准备重新启动 `./start_sensors.sh`。

---

### 3. 停止 roslaunch

```bash
pkill -f roslaunch
```

作用：停止由 `roslaunch` 启动的进程。

什么时候用：

- launch 启动异常；
- 需要清理旧启动进程；
- 准备重新启动系统。

注意：

不要在同一条远程命令里同时写复杂的 `pkill` 和启动命令，容易误杀当前启动流程。更稳妥的做法是先清理，再单独启动。

---

## 八、推荐日常启动流程

### 方式 A：推荐，两个终端

#### 终端 1

```bash
ssh ucar@10.90.122.179
./start_sensors.sh
```

作用：登录小车并启动底盘 + 雷达。

#### 终端 2

```bash
ssh ucar@10.90.122.179
./start_control.sh
```

作用：登录小车并启动遥控 + 状态监控界面。

---

### 方式 B：只练遥控，不需要雷达

#### 终端 1

```bash
ssh ucar@10.90.122.179
./start_base.sh
```

作用：只启动底盘。

#### 终端 2

```bash
ssh ucar@10.90.122.179
./start_control.sh
```

作用：启动遥控 + 状态监控界面。

---

### 方式 C：只测试雷达

```bash
ssh ucar@10.90.122.179
./start_lidar.sh
```

然后另开终端检查：

```bash
rostopic hz /scan
```

作用：确认雷达 `/scan` 是否约 10Hz。

---

## 九、下一阶段可能用到的命令

### 1. 启动建图

推荐使用修正版建图入口：

```bash
roslaunch ucar_map ucar_mapping_fixed.launch
```

作用：启动修正版 Cartographer 建图流程。

启动内容：

```text
底盘驱动：ucar_controller/base_driver.launch
雷达驱动：ydlidar_ros_driver/ucar_g4.launch
建图节点：ucar_map/cartographer_start.launch
```

为什么使用这个修正版：

原官方文件 `ucar_mapping.launch` 引用了旧的雷达包名：

```xml
<include file="$(find ydlidar)/launch/ydlidar.launch"/>
```

但当前小车实际可用的雷达包和配置是：

```text
ydlidar_ros_driver/launch/ucar_g4.launch
```

并且雷达串口是：

```text
/dev/ttyTHS1
```

什么时候用：

- 开始 SLAM 建图；
- 想生成室内地图；
- 需要同时启动底盘、雷达和 Cartographer。

启动后应出现：

```text
/scan
/odom
/tf
/map
/submap_list
```

注意：

建图时应低速移动小车，避免快速旋转或快速前进。

---

### 2. 保存地图

建图完成后推荐在小车上保存地图：

```bash
mkdir -p ~/ucar_ws/maps
cd ~/ucar_ws/maps
rosrun map_server map_saver -f ucar_map_$(date +%Y%m%d_%H%M%S)
```

作用：把当前 `/map` 话题中的地图保存为 `.pgm` 图片和 `.yaml` 配置文件。

命令含义：

- `mkdir -p ~/ucar_ws/maps`：创建地图保存目录，如果已存在则不报错。
- `cd ~/ucar_ws/maps`：进入地图保存目录。
- `rosrun map_server map_saver`：运行 ROS 地图保存工具。
- `-f ucar_map_$(date +%Y%m%d_%H%M%S)`：用当前日期时间作为地图文件名，避免覆盖旧地图。

保存后会生成类似：

```text
ucar_map_20260501_202713.pgm
ucar_map_20260501_202713.yaml
```

什么时候用：

- RViz 中地图已经比较完整后；
- 准备进入导航前；
- 每次建图结束时。

### 3. 使用 VNC 打开小车桌面和 RViz

小车上 VNC 服务地址：

```text
10.90.122.179:5900
```

VNC 密码：

```text
ucar
```

在电脑上可以用 Remmina 连接：

```bash
remmina -c vnc://10.90.122.179:5900
```

作用：用 VNC 打开小车本机桌面，比 SSH X11 转发更适合 RViz。

在小车桌面终端中启动 RViz：

```bash
export DISPLAY=:0
export DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus
source /opt/ros/melodic/setup.bash
source ~/ucar_ws/devel/setup.bash
export ROS_MASTER_URI=http://10.90.122.179:11311
export ROS_IP=10.90.122.179
export LIBGL_ALWAYS_SOFTWARE=1
rviz
```

作用：在小车本机桌面运行 RViz，连接当前小车 ROS Master。

RViz 中常用设置：

```text
Fixed Frame: map
Add → Map        Topic: /map
Add → LaserScan  Topic: /scan
Add → TF
```

注意：

不要优先使用 `ssh -Y rviz` 或 Docker X11 方式运行 RViz，容易出现 GLX/OpenGL 错误。

---

### 5. 启动修正版导航

```bash
roslaunch ucar_nav ucar_navigation_fixed.launch
```

作用：启动修正版导航流程，使用封门地图、正确雷达和导航节点。

启动内容：

```text
底盘驱动：ucar_controller/base_driver.launch
雷达驱动：ydlidar_ros_driver/ucar_g4.launch
地图服务器：/home/ucar/ucar_ws/maps/ucar_map_20260501_202713_sealed.yaml
AMCL：ucar_nav/launch/config/amcl/amcl_omni.launch
move_base：robot navigation
```

启动后应出现：

```text
/map
/amcl_pose
/goal
/move_base/status
/cmd_vel
/scan
/odom
/tf
```

什么时候用：

- 地图已经保存并封门后；
- 需要做定位与目标点导航；
- 进入最终演示阶段前。

---

## 十、RViz 里如何导航到目标点

当你已经在电脑上用 VNC 打开小车桌面的 RViz 后，按下面做：

### 1. 设置固定坐标系

在左下角 `Global Options` 里，把：

```text
Fixed Frame
```

改成：

```text
map
```

作用：让 RViz 以地图坐标系显示所有数据。

---

### 2. 添加必要显示项

点击左下角 `Add`，添加：

- `Map`
  - Topic 选 `/map`
- `TF`
- `LaserScan`
  - Topic 选 `/scan`

---

### 3. 先设置初始位姿

如果刚启动导航，机器人还没定位，要先在 RViz 顶部工具栏点：

```text
2D Pose Estimate
```

然后在地图上小车实际位置点一下并拖出朝向箭头。

作用：告诉 AMCL 机器人现在大概在哪、朝哪个方向。

---

### 4. 发送目标点

定位好以后，点击：

```text
2D Nav Goal
```

然后在地图上你希望小车去的位置点一下并拖出方向箭头。

作用：向 `move_base` 发送导航目标。

---

### 5. 观察小车是否在动

如果一切正常：

- `/move_base/status` 会变化；
- 小车会自动规划路径；
- `/cmd_vel` 会输出速度；
- 实体小车会自己走过去。

---

### 6. 遇到问题时先看这几个话题

```bash
rostopic list | grep -E '/map|/scan|/odom|/tf|/amcl_pose|/move_base/status'
```

如果导航不动，优先检查：

- `/map` 是否存在；
- `/scan` 是否存在；
- `Fixed Frame` 是否为 `map`；
- 初始位姿是否设置；
- 地图是否加载了封门版 `sealed.yaml`。

---

## 十一、当前最重要的设备和命令关系

```text
底盘启动：./start_base.sh
雷达启动：./start_lidar.sh
底盘+雷达：./start_sensors.sh
控制界面：./start_control.sh
建图：roslaunch ucar_map ucar_mapping_fixed.launch
保存地图：rosrun map_server map_saver -f ucar_map_$(date +%Y%m%d_%H%M%S)
导航：roslaunch ucar_nav ucar_navigation_fixed.launch
检查话题：rostopic list
检查雷达：rostopic hz /scan
检查里程计：rostopic hz /odom
VNC 连接：remmina -c vnc://10.90.122.179:5900
```

---

## 十、导航启动补充

### 1. 启动修正版导航

```bash
source /opt/ros/melodic/setup.bash
source ~/ucar_ws/devel/setup.bash
export ROS_MASTER_URI=http://10.90.122.179:11311
export ROS_IP=10.90.122.179
roslaunch ucar_nav ucar_navigation_fixed.launch
```

作用：启动修正版导航流程，使用封门地图、正确雷达和导航节点。

启动内容：

```text
底盘驱动：ucar_controller/base_driver.launch
雷达驱动：ydlidar_ros_driver/ucar_g4.launch
地图服务器：/home/ucar/ucar_ws/maps/ucar_map_20260501_202713_sealed.yaml
AMCL：ucar_nav/launch/config/amcl/amcl_omni.launch
move_base：robot navigation
```

注意：当前导航已把原来不可用的全局规划器插件改成了可用的：

```text
global_planner/GlobalPlanner
```

### 2. RViz 里如何导航到目标点

1. 在 RViz 左下角把 `Fixed Frame` 设为：

```text
map
```

2. 添加显示项：

```text
Map        topic: /map
LaserScan  topic: /scan
TF
```

3. 点击顶部工具栏：

```text
2D Pose Estimate
```

在地图上点击小车当前位置，并拖出一个方向箭头。

作用：给 AMCL 一个初始位姿。

4. 再点击：

```text
2D Nav Goal
```

在地图上点击目标点，并拖出目标朝向箭头。

作用：给 `move_base` 发送导航目标。

5. 如果导航正常：

- 小车会开始动；
- `/cmd_vel` 会有速度输出；
- `/move_base/status` 会更新；
- RViz 中路径会刷新。

6. 如果小车不动，优先检查：

```bash
rostopic list | grep -E '/map|/scan|/odom|/tf|/amcl_pose|/move_base/status'
```

以及是否已经：

- 正确加载封门地图；
- 设置了 `2D Pose Estimate`；
- 雷达 `/scan` 正在持续发布。

---

## 十二、导航推荐启动顺序（避免雷达被挤掉）

### 正确顺序

#### 终端 1：先启动底盘 + 雷达

```bash
ssh ucar@10.90.122.179
./start_sensors.sh
```

作用：启动底盘和雷达，确保 `/odom` 与 `/scan` 稳定发布。

#### 终端 2：再启动导航运行版

```bash
ssh ucar@10.90.122.179
source /opt/ros/melodic/setup.bash
source ~/ucar_ws/devel/setup.bash
export ROS_MASTER_URI=http://10.90.122.179:11311
export ROS_IP=10.90.122.179
roslaunch ucar_nav ucar_navigation_runtime.launch
```

作用：只启动导航需要的：

```text
map_server
AMCL
move_base
```

不会再次启动底盘和雷达，因此不会把已经运行的 `base_driver` 和 `ydlidar_lidar_publisher` 挤掉。

### 不推荐顺序

不要在已经执行了：

```bash
./start_sensors.sh
```

之后再运行：

```bash
roslaunch ucar_nav ucar_navigation_fixed.launch
```

因为 `ucar_navigation_fixed.launch` 也会再次启动：

```text
base_driver
ydlidar_lidar_publisher
```

结果会因为同名节点重复注册，导致：

```text
Reason: new node registered with same name
```

然后雷达停转、`/scan` 中断。

---

## 十三、干净导航工作空间（绕开厂商定制导航）

当原厂 `move_base` 持续 `exit code -11` 段错误时，使用独立工作空间：

```text
/home/ucar/nav_clean_ws
```

### 1. 先启动底盘 + 雷达

```bash
ssh ucar@10.90.122.179
source /opt/ros/melodic/setup.bash
source ~/ucar_ws/devel/setup.bash
export ROS_MASTER_URI=http://10.90.122.179:11311
export ROS_IP=10.90.122.179
./start_sensors.sh
```

### 2. 再启动干净导航

```bash
ssh ucar@10.90.122.179
source /opt/ros/melodic/setup.bash
source /home/ucar/nav_clean_ws/devel/setup.bash
export ROS_MASTER_URI=http://10.90.122.179:11311
export ROS_IP=10.90.122.179
roslaunch nav_clean navigation_runtime.launch
```

作用：

- 使用封门地图 `/home/ucar/ucar_ws/maps/ucar_map_20260501_202713_sealed.yaml`
- 使用标准 `amcl`
- 使用 `navfn/NavfnROS`
- 尽量减少厂商导航 launch 的复杂自定义参数影响

注意：

这套工作空间虽然隔离了 launch 和参数，但底层 `move_base`、`dwa_local_planner`、`costmap_2d` 仍可能被 `~/ucar_ws/src/developer_navigation` 覆盖；如果仍崩溃，则说明需要进一步从 `ROS_PACKAGE_PATH` 层面绕开厂商覆盖包。

### 3. 使用一键脚本启动干净导航

```bash
ssh ucar@10.90.122.179
./start_nav_clean.sh
```

作用：一键启动 `nav_clean` 导航运行版。

---

### 4. VNC 远程桌面连接（x11vnc）

小车上已安装 `x11vnc`。

启动命令（在小车 SSH 终端中执行）：

```bash
x11vnc -display :0 -forever -nopw -rfbport 5900 -auth guess -noxdamage
```

含义：

- `x11vnc`：启动 VNC 服务器
- `-display :0`：共享当前桌面屏幕
- `-forever`：客户端断开后服务不退出
- `-nopw`：临时无密码，方便测试
- `-rfbport 5900`：监听标准 VNC 端口
- `-auth guess`：自动猜测 X 认证文件
- `-noxdamage`：禁用 X damage 扩展，减少兼容问题

电脑端 Remmina 连接：

```text
协议：VNC
地址：10.90.122.179:5900
密码：不需要填（nopw 模式）
```

如需密码保护，启动时改为：

```bash
x11vnc -display :0 -forever -passwd ucar -rfbport 5900 -auth guess -noxdamage
```

含义：设置 VNC 密码为 `ucar`。

---

### 5. SSH 终端里手动启动 RViz

如果在 SSH 终端里启动 RViz，需要手动设置 DISPLAY：

```bash
export DISPLAY=:0
export XAUTHORITY=/run/user/1000/gdm/Xauthority
source /opt/ros/melodic/setup.bash
source ~/ucar_ws/devel/setup.bash
export ROS_MASTER_URI=http://10.90.122.179:11311
export ROS_IP=10.90.122.179
export LIBGL_ALWAYS_SOFTWARE=1
rviz
```

含义：

- `export DISPLAY=:0`：指定要使用的 X 显示屏幕
- `export XAUTHORITY=...`：指定 X 认证文件，否则会报 `Could not connect to display`
- `export LIBGL_ALWAYS_SOFTWARE=1`：强制软件渲染，减少 OpenGL 崩溃

---

### 6. 重启 GDM 桌面

如果需要重新登录桌面：

```bash
sudo service gdm restart
```

含义：重启 GDM 登录管理器，回到登录界面。

注意：重启后原桌面会话会丢失，需要重新登录。



### 7. Web 控制面板（当前稳定用法）

浏览器访问：

```text
http://10.90.122.179:8080
```

当前面板应优先只用于：

- 手动按钮控制前进 / 后退 / 左转 / 右转
- `Takeover` 停止导航并切换到手动控制
- `Resume Nav` 恢复导航
- 输入目标点坐标并发送 `/move_base_simple/goal`
- Map 页查看封门地图预览和导航节点状态
- Map 页点击地图可选择目标点，自动换算为 `map` 坐标并填入 X/Y

当前已知限制：

- 地图页显示的是静态封门地图预览，不是 RViz 那种实时代价地图；
- Camera 页如果相机驱动不稳定，只作为附加显示；
- 如果网页按钮要直接控车，必须先点 `Takeover`，避免 `move_base` 继续占用 `/cmd_vel`。
- 如果 `GO` 后没有反应，先看 Map 页状态是否为 `move_base=true amcl=true map_server=true`。
- 高频移动按钮应走 `/api/cmd_vel` 和 `/api/manual_stop` 这类轻量接口，不要走会同步杀导航/等待 ROS 的慢接口。
- 页面状态里的位置应以 `/amcl_pose` 的 `map` 坐标为准；不要把 `/odom` 坐标直接当导航目标。

### 8. x11vnc 断线后的恢复

如果 Remmina 再次提示无法连接 VNC 服务器，先在小车 SSH 终端执行：

```bash
x11vnc -display :0 -forever -nopw -rfbport 5900 -auth guess -noxdamage
```

然后在电脑端 Remmina 连接：

```text
协议：VNC
地址：10.90.122.179:5900
密码：不需要填
```

注意：如果 `x11vnc` 是前台运行，SSH 终端不要关；如果要后台运行，推荐：

```bash
nohup x11vnc -display :0 -forever -nopw -rfbport 5900 -auth guess -noxdamage >/tmp/x11vnc.log 2>&1 &
```
