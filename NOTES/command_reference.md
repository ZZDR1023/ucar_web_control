# ROSCAR1 命令速查表

本文档记录小车常用命令：每条命令启动什么部分、什么时候使用、预期结果是什么。

---

## 一、常用连接命令

### 1. 测试小车是否在线

```bash
ping -c 3 -W 2 10.68.225.179
```

作用：检查电脑是否能通过 Wi-Fi 连接到小车。

参数含义：

- `ping`：测试网络连通性。
- `-c 3`：发送 3 次测试包。
- `-W 2`：每次最多等待 2 秒。
- `10.68.225.179`：小车当前无线 IP。

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
ssh ucar@10.68.225.179
```

作用：远程登录小车 Linux 系统。

参数含义：

- `ssh`：远程登录命令。
- `ucar`：小车用户名。
- `10.68.225.179`：小车 IP。

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
export ROS_MASTER_URI=http://10.68.225.179:11311
```

作用：告诉 ROS，ROS Master 在小车自己的 IP 和 11311 端口上。

什么时候用：

- 手动运行 ROS 命令前；
- 新开 SSH 终端后。

---

### 4. 设置小车自身 ROS 通信 IP

```bash
export ROS_IP=10.68.225.179
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
export ROS_MASTER_URI=http://10.68.225.179:11311
export ROS_IP=10.68.225.179
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
ssh ucar@10.68.225.179
./start_sensors.sh
```

作用：登录小车并启动底盘 + 雷达。

#### 终端 2

```bash
ssh ucar@10.68.225.179
./start_control.sh
```

作用：登录小车并启动遥控 + 状态监控界面。

---

### 方式 B：只练遥控，不需要雷达

#### 终端 1

```bash
ssh ucar@10.68.225.179
./start_base.sh
```

作用：只启动底盘。

#### 终端 2

```bash
ssh ucar@10.68.225.179
./start_control.sh
```

作用：启动遥控 + 状态监控界面。

---

### 方式 C：只测试雷达

```bash
ssh ucar@10.68.225.179
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
10.68.225.179:5900
```

VNC 密码：

```text
ucar
```

在电脑上可以用 Remmina 连接：

```bash
remmina -c vnc://10.68.225.179:5900
```

作用：用 VNC 打开小车本机桌面，比 SSH X11 转发更适合 RViz。

在小车桌面终端中启动 RViz：

```bash
export DISPLAY=:0
export DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus
source /opt/ros/melodic/setup.bash
source ~/ucar_ws/devel/setup.bash
export ROS_MASTER_URI=http://10.68.225.179:11311
export ROS_IP=10.68.225.179
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
VNC 连接：remmina -c vnc://10.68.225.179:5900
```

---

## 十、导航启动补充

### 1. 启动修正版导航

```bash
source /opt/ros/melodic/setup.bash
source ~/ucar_ws/devel/setup.bash
export ROS_MASTER_URI=http://10.68.225.179:11311
export ROS_IP=10.68.225.179
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
ssh ucar@10.68.225.179
./start_sensors.sh
```

作用：启动底盘和雷达，确保 `/odom` 与 `/scan` 稳定发布。

#### 终端 2：再启动导航运行版

```bash
ssh ucar@10.68.225.179
source /opt/ros/melodic/setup.bash
source ~/ucar_ws/devel/setup.bash
export ROS_MASTER_URI=http://10.68.225.179:11311
export ROS_IP=10.68.225.179
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
ssh ucar@10.68.225.179
source /opt/ros/melodic/setup.bash
source ~/ucar_ws/devel/setup.bash
export ROS_MASTER_URI=http://10.68.225.179:11311
export ROS_IP=10.68.225.179
./start_sensors.sh
```

### 2. 再启动干净导航

```bash
ssh ucar@10.68.225.179
source /opt/ros/melodic/setup.bash
source /home/ucar/nav_clean_ws/devel/setup.bash
export ROS_MASTER_URI=http://10.68.225.179:11311
export ROS_IP=10.68.225.179
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
ssh ucar@10.68.225.179
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
地址：10.68.225.179:5900
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
export ROS_MASTER_URI=http://10.68.225.179:11311
export ROS_IP=10.68.225.179
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
http://10.68.225.179:8080
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
- Map 页会显示 `/scan` 雷达状态；全局导航仍使用封门地图，雷达用于 local costmap 动态避障。
- Map 页已内嵌 Camera，并显示 `goal distance` / `reissues`；如果 goal 被 move_base 提前结束，后端会自动补发最近目标。
- Map 页黄色点是实时 `/scan` 雷达命中点。普通点击设置新目标点，`Shift + 点击` 或右键设置目标朝向。
- Web 层前方急停使用窄扇区：`stop < 0.40m`，`clear > 0.65m`。如果障碍还在正前方，重新发 goal 也会被安全逻辑取消。
- 禁止把启动位置当作 map 原点。Map 页 `Set AMCL Pose Here` 只用于定位校正：先点小车真实位置，再设置 AMCL 初始位姿。
- Map 支持缩放：`Ctrl + 鼠标滚轮` 以鼠标位置为中心缩放，也可以用 `Zoom + / Zoom - / Reset`。
- 控制面板现在是单页布局：手动控制、Map、Camera、Log 同屏显示，Camera 会自动实时刷新。
- Camera 现在默认使用 `/api/camera_stream` MJPEG 长连接，不再每 700ms 轮询 `/api/camera`；`Refresh Camera` 只用于重连流。
- Safety 卡片会直接显示 `CLEAR / CAUTION / BLOCKED`、前方距离、安全盒距离和电量，优先看这里判断为什么导航被 Web 急停层拦住。
- 为减少碰撞，Web 启动时会将 DWA 导航速度限制到 `0.18m/s`，并把 local/global costmap inflation 调到 `0.35m`。
- Web 急停层有前方安全盒：`front=0.50m`、`half_width=0.30m`；如果行李箱等物体不在雷达扫描平面内，仍需要人工降低风险。
- 前方安全状态分两级：`caution=true` 只提示，`blocked=true` 才会取消导航并切回 Manual；当前急停阈值为 `0.35m`。
- 顶部和 Status 卡片会显示 `/battery_state` 电量；低于 20% 变红，10 秒没有新数据会显示 `stale`，完全没有话题则显示 `Battery no data`。
- 巡逻路线需要先填写路线名称再保存；保存后会进入路线下拉框，后续应先 `加载路线` 再开始巡逻。
- 巡逻到点会保存相机照片，并生成巡逻记录；点击记录可查看路线点位和对应照片。
- `恢复` 只用于暂停或阻塞后的当前任务继续执行，不等于从磁盘加载历史路线。
- 语音控制入口在 Web 面板中：浏览器支持 `SpeechRecognition` 时先点 `开始录音`，看到“正在听”后说命令，说完点 `结束录音并执行`。不支持语音识别时直接在输入框输入命令并点 `执行`。后端接口是 `/api/voice_command`，执行链路仍经过 Web 后端安全状态机。

#### Web 语音控制

当前语音控制先采用浏览器 Web Speech API，不在小车端额外安装 ASR 包。原因是小车当前没电，无法确认麦克风阵列、科大讯飞包和 ROS 话题状态；先把“识别文本 -> 安全执行”链路写好，后续可把任意 ASR 输出接到同一个 `/api/voice_command`。

支持的命令：

```text
前进
前进1米
前进1米再左转
后退
后退0.5米
左转
右转45度
右转
停止
开始巡逻
暂停巡逻
继续巡逻
停止巡逻
去<已保存巡逻点名称>，例如：去门口
去<t1中的1号巡逻点>，例如：去t1中的1号巡逻点
去<当前路线中的几号巡逻点>，例如：去3号巡逻点
```

测试接口但不让车运动时，优先只发无法执行运动的文本或在底盘断电时测试解析返回：

```bash
curl -s -X POST http://10.68.225.179:8080/api/voice_command \
  -H 'Content-Type: application/json' \
  -d '{"text":"停止"}'
```

实车上电后再测试“前进/左转”等运动命令。语音手动移动被后端限速到低速范围：线速度不超过 `±0.12m/s`，角速度不超过 `±0.50rad/s`；前方障碍触发时会拒绝语音前进。

注意：不带距离/角度的 `前进`、`左转` 仍是普通低速手动命令，会保持当前速度直到 STOP 或下一条命令；带距离/角度或带 `再/然后/接着` 的命令会被解析成动作序列，例如 `前进1米再左转` 会按顺序低速前进约 1 米、停车、再默认左转 90 度、停车。序列优先使用 `/odom` 反馈判断是否到达距离/角度，里程计异常时才使用超时兜底。为了安全，单步距离会被限制在较短范围内，前方障碍触发时拒绝继续前进。

语音动作序列有两个实车校准参数，可在 Web 服务启动参数或 ROS 私有参数里调整：

```bash
rosparam set /voice_turn_odom_scale 1.0
rosparam set /voice_move_odom_scale 1.0
```

- `voice_turn_odom_scale`：转向 odom 目标倍率。默认 `1.0`，即 `左转90度` 就等待 odom yaw 约 90 度再停；只有现场确认底盘/地面导致长期偏小时才手动增大。
- `voice_move_odom_scale`：前进/后退 odom 位移倍率。实测 1 米走短了就略微增大，走长了就减小。

语音导航到巡逻点时，如果说 `去3号巡逻点` 这种不带路线名的命令，后端默认匹配 Web 当前已加载路线中的第 3 个点；如果要跨路线指定，使用 `去t1中的3号巡逻点`。

如果页面显示“目标已排队，但尚未收到 AMCL 定位”，说明 Web 已保存目标，但不会把目标发给 `move_base` 让车盲走。处理顺序：

1. 在地图上点击小车真实位置；
2. 点击 `Set AMCL Pose Here` 设置定位；
3. 等 Status 中 `pose=amcl` 或 `使用小车位置` 不再报错；
4. 再输入 `去3号巡逻点` 或发送地图目标。

如果 `rosnode list` 能看到 `/amcl`、`/move_base`，但 `/amcl_pose` 或 `/move_base/status` 没有新消息，要按僵尸节点处理：重启 Web 服务会使用脱离 shell 的导航启动方式重新拉起导航；必要时执行 `yes | rosnode cleanup` 清理 ROS Master 旧注册。

手机端如果显示 `语音识别被中断` 或浏览器原始错误 `aborted`，通常不是后端问题，而是浏览器语音识别会话被系统中断、麦克风权限未授予，或页面不是浏览器认为的安全上下文。优先操作顺序：

1. 用 Chrome/Edge 打开 Web 面板；
2. 点 `开始录音`；
3. 等页面显示 `Mic OK`、`正在听` 或 `已接入麦克风` 后再说完整命令；
4. 说完点 `结束录音并执行`；
5. 如果麦克风自检正常但仍 `aborted`，说明 Chrome 语音识别服务没有返回结果，直接在输入框输入同样命令并点 `执行`，确认后端控制链路是否正常。

> 暂时搁置：由于当前地图工具缺失，无法稳定建立和校验赛道/路口语义地图，视觉巡线不再作为当前推荐实车方案。Web Camera 页面里的视觉巡线控件已隐藏，只保留一行搁置提示；以下内容保留为历史实验记录和后续恢复开发的参考。

- 历史版本的 Camera 页视觉巡线支持 `开始记录样本` / `停止记录`。记录期间系统会关闭自动巡线，只保存人工驾驶数据，不接管 `/cmd_vel`。
- 巡线样本默认保存到 `/home/ucar/web_panel/line_follow_records/<时间>_<名称>/`，其中 `frames/` 是相机原图，`debug/` 是算法调试图，`metadata.jsonl` 是人工速度、里程计、位姿、巡线判断等逐帧元数据。
- 视觉巡线默认使用黑线模式，目标是黑/深灰可走路面；颜色识别已改为 HSV 掩码参数 `h_min/h_max/s_min/s_max/v_min/v_max`，默认黑线范围为 `H 0-179, S 0-80, V 0-200`。算法会剔除红/黄禁压线、彩色标记、白色高亮和无边界整片灰地面。调试图里红色是当前 HSV 识别的可走路径，紫红色是红/黄禁压实线候选。
- 黑/灰路线的 H 值通常不重要，主要调 `S 最大` 和 `V 最大`。旧的自动收紧黑线亮度逻辑已移除；只有 HSV 初筛无法形成可走路径时，才会启用暗部对比 fallback，避免直线路面被侧边暗块拉偏。
- 巡线控制现在是保守策略：近场没有连续可走黑/灰区域时只原地转向，不向前走；禁压线接触时清零线速度；未禁压但大偏差急转、分叉搜索锁定或红区明显收缩时也清零线速度，只原地转到近场路径稳定后再恢复前进。
- 当前默认转向：默认转向增益 `0.60`，丢线搜索角速度 `0.24 rad/s`，巡线跟随阶段角速度上限 `±0.42 rad/s`。分叉锁定搜索角速度会跟随 `转向增益`，约为 `angular_gain * 0.80`，并限制在 `0.14-0.42 rad/s`；例如增益 `0.25` 时分叉锁定约 `0.20 rad/s`。如果分叉处转过头，在 Web 面板把 `转向增益` 下调到 `0.25-0.35`。
- 禁压线接触时角速度会优先远离接触侧：右侧红/黄接触则左转脱离，左侧接触则右转脱离，避免分叉路口选错候选后继续朝黄线方向拧。
- 如果当前完全没有识别到黑/灰可走路线，同时底部看到黄/红禁压线，巡线会停车等待，不再原地旋转搜索；蓝色区域在黄线外，不会被当作可走路线。
- 分叉处如果红色可走路径整体偏在画面一侧，算法允许足够宽、连续的侧向分支作为初始路径；同时过滤底部 3px 级别的细灰边缘，避免黄线边缘或蓝布反光把绿色目标点拉到非可走区域。
- 分叉找路状态带锁定：分叉、近场丢路或红区明显减少时会立即锁定一个搜索方向，固定原地同向转；没有最短锁定时间，之后只要连续 3 帧看到居中的近场宽路就解除。若红色可走区域明显减少，会沿上一分支方向提前原地转向，避免继续直行到黄线边界。锁定期间黄/红接触只负责停车，不再把角速度改成相反方向。
- 巡线算法带方向连续性：左右同时出现候选时优先沿用上一帧稳定道路中心，单帧噪声不能立刻把转向翻到相反方向。黄/红边界形成明显弯道时，会给小幅预测转向，但不会覆盖禁压停车。

> 搁置边界：上面的视觉巡线能力目前只作为实验记录保存。地图工具缺失期间，后续调试优先走建图/导航链路修复，不继续在视觉巡线上叠加规则。

Web 服务推荐由 systemd 守护运行，避免手动启动后终端关闭或进程崩溃导致面板离线。底盘和雷达也可以交给独立的 sensors 服务管理，这样不用每次开 Web 前手动执行 `./start_sensors.sh`。

仓库内单元文件：

```text
systemd/ucar_sensors.service
systemd/ucar_web.service
```

首次安装到小车：

```bash
sudo cp /tmp/ucar_sensors.service /etc/systemd/system/ucar_sensors.service
sudo cp /tmp/ucar_web.service /etc/systemd/system/ucar_web.service
sudo systemctl daemon-reload
sudo systemctl enable ucar_sensors.service ucar_web.service
sudo systemctl restart ucar_sensors.service ucar_web.service
```

日常查看和重启：

```bash
systemctl status ucar_sensors.service --no-pager
systemctl status ucar_web.service --no-pager
sudo systemctl restart ucar_sensors.service
sudo systemctl restart ucar_web.service
```

`ucar_web.service` 已配置 `Wants=ucar_sensors.service` 和 `After=ucar_sensors.service`。安装两个服务后，启动 Web 服务会同时拉起 sensors 服务；但它们仍是两个独立进程，排障时应分别看状态。

如果手动运行 `python3 /home/ucar/web_panel/server.py` 报 `Address already in use`，通常说明 systemd 或旧 Flask 进程已经占用 8080。优先查看：

```bash
systemctl status ucar_web.service --no-pager
```

解决方法不是再开一个 `server.py`，而是二选一：

- 使用 systemd：`sudo systemctl restart ucar_web.service`
- 临时手动调试：先停服务 `sudo systemctl stop ucar_web.service`，再手动运行 `python3 server.py`

下次给小车部署本地 Web/巡线改动时，先确认小车有电且网络能通：

```bash
ping -c 3 -W 2 10.68.225.179
```

然后从本地仓库执行：

```bash
scp web_panel/line_record.py web_panel/line_follow.py web_panel/server.py web_panel/templates/index.html bug_notes.md command_reference.md ucar@10.68.225.179:/tmp/
```

登录小车后应用文件、语法检查并重启 Web 服务：

```bash
cp /tmp/line_record.py /home/ucar/web_panel/line_record.py
cp /tmp/line_follow.py /home/ucar/web_panel/line_follow.py
cp /tmp/server.py /home/ucar/web_panel/server.py
cp /tmp/index.html /home/ucar/web_panel/templates/index.html
cp /tmp/bug_notes.md /home/ucar/bug_notes.md
cp /tmp/command_reference.md /home/ucar/command_reference.md
python3 -m py_compile /home/ucar/web_panel/server.py /home/ucar/web_panel/line_follow.py /home/ucar/web_panel/line_record.py
sudo systemctl restart ucar_web.service
systemctl is-active ucar_web.service
```

注意：当前 Web 页面已隐藏视觉巡线入口。部署这些历史文件不会启用巡线控制，也不会在页面上显示 `启用巡线控制` 按钮。

### 8. 极简 ROS 巡线实验脚本

> 暂时搁置：由于当前地图工具缺失，视觉巡线实验脚本和 C++ 巡线节点暂不继续作为实车路线方案推进。本节仅保留历史命令、构建方式和安全注意事项，避免后续忘记曾经的实现细节。

仓库新增 `simple_line_follower.py`，用于抛开 Web 视觉巡线状态机做最小实验。它只做：

- 订阅 ROS 图像话题；
- 截取画面下方 ROI；
- HSV 黑色 mask；
- 先用 `cv2.findContours` 把 mask 切成多个轮廓，按 `branch_choice` 选择一个目标轮廓；
- 只对选中的目标轮廓取质心，避免 Y 型分叉时把左右两条线平均到中间空地；
- 用 P 控制发布 `/cmd_vel`；
- 看不到黑线时先短时沿用上一帧命令，超过 `lost_frame_limit` 后停车，用于跨过虚线间隙。

它会直接发布 `/cmd_vel`，因此运行前必须先确认 Web 视觉巡线关闭、导航/巡逻没有运行，并且人在车旁能随时 STOP。

小车端运行示例：

```bash
source /opt/ros/melodic/setup.bash
source ~/ucar_ws/devel/setup.bash
export ROS_MASTER_URI=http://10.68.225.179:11311
export ROS_IP=10.68.225.179
cd /home/ucar
python3 simple_line_follower.py _image_topic:=/camera/image_raw _linear_speed:=0.06 _kp:=0.003 _max_angular:=0.30 _v_max:=90 _branch_choice:=left _lost_frame_limit:=10
```

如果摄像头实际话题不是 `/camera/image_raw`，先查：

```bash
rostopic list | grep -E 'image|camera|usb'
```

如果没有 ROS 图像话题，可以直接读 USB 摄像头设备：

```bash
python3 simple_line_follower.py _video_device:=/dev/video0 _linear_speed:=0.06 _kp:=0.003 _max_angular:=0.30 _v_max:=90 _branch_choice:=left _lost_frame_limit:=10
```

常调参数：

- `_kp`：转向比例，越大转得越猛；
- `_linear_speed`：前进速度，建议先用 `0.04-0.06`；
- `_max_angular`：角速度上限，建议先用 `0.25-0.30`；
- `_v_max`：黑色亮度上限，反光强时可从 `90` 调到 `120`；
- `_roi_top`：只看画面下方比例，默认 `0.67`。
- `_video_device`：不为空时直接用 OpenCV 读取摄像头设备，不订阅 ROS 图像话题。
- `_branch_choice`：分叉时选择目标轮廓，支持 `left`、`right`、`largest`、`center`；外圈/左分叉通常用 `left`，内圈/右分叉用 `right`。
- `_lost_frame_limit`：虚线短暂丢失时沿用上一帧命令的帧数，默认 `10`，约半秒；太大会增加脱轨后继续盲走风险。

同目录还新增了 C++ 版实验节点源码 `src/line_follower.cpp`。它实现同一类策略：HSV 黑线、竖向闭运算连接虚线、滑窗追踪黑色区域中心线、近场/前瞻点混合控制、`branch_choice` 分叉选择、短时丢线沿用上一帧命令、底部黄线保护和转弯降速。仓库根目录已补齐 `CMakeLists.txt` 和 `package.xml`，catkin 可构建目标名为 `line_follower_node`。

小车端建议放到 `~/ucar_ws/src/roscar1` 后构建：

```bash
source /opt/ros/melodic/setup.bash
source ~/ucar_ws/devel/setup.bash
cd ~/ucar_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES=roscar1
source devel/setup.bash
```

编译后检查 OpenCV 不能同时出现 3.2 和 3.3：

```bash
ldd ~/ucar_ws/devel/lib/roscar1/line_follower_node | grep opencv
```

预期只看到 `/usr/lib/aarch64-linux-gnu/libopencv_*.so.3.2`。如果混入 `/usr/local/lib/libopencv_*.so.3.3`，不要实车运行，先检查 `CMakeLists.txt` 是否仍优先使用系统 OpenCV 3.2 库。

实车运行前先确认底盘、相机和话题状态：

```bash
rostopic list
rostopic hz /odom -w 1
rostopic hz /camera/image_raw -w 1
rostopic echo -n 1 /camera/image_raw/header
```

如果没有 `/camera/image_raw`，先启动相机节点。`ucar_sensors.service` 只负责底盘和雷达，不会自动拉起相机：

```bash
rosrun ucar_camera ucar_camera.py _cam_topic_name:=/camera/image_raw _image_width:=640 _image_height:=480 _rate:=10
```

只做节点连通性验证、不让车动时，用 0 速度启动 C++ 节点：

```bash
rosrun roscar1 line_follower_node _image_topic:=/camera/image_raw _max_linear_speed:=0.0 _max_angular_speed:=0.0 _pid_debug_output:=false _img_debug_output:=true
rostopic echo -n 1 /cmd_vel
rostopic echo -n 1 /img_follow/header
```

这个 smoke test 预期 `/cmd_vel` 全为 0，`/img_follow` 能收到调试图 header。

确认 Web 视觉巡线、导航、巡逻都未运行后，再低速启动 C++ 节点：

```bash
rosrun roscar1 line_follower_node _image_topic:=/camera/image_raw _max_linear_speed:=0.035 _max_angular_speed:=0.16 _Kp:=0.0016 _Kd:=0.0008 _line_threshold:=130 _branch_choice:=left _max_lost_frames:=18 _roi_top_ratio:=0.45 _close_kernel_width:=11 _close_kernel_height:=91 _layer_count:=12 _lookahead_layer:=4 _lookahead_weight:=0.35 _turn_slowdown:=0.45 _curve_slowdown:=0.35 _max_curve_delta:=180 _min_curve_speed:=0.012 _straight_hold_enabled:=true _straight_hold_near_error:=35 _straight_hold_curve_delta:=90 _straight_hold_frames:=4 _straight_hold_speed:=0.025 _branch_lock_enabled:=true _branch_approach_hold:=true _branch_split_delta:=120 _branch_commit_frames:=8 _branch_route_memory_frames:=55 _branch_exit_hold_frames:=10 _branch_target_weight:=0.40 _branch_max_angular_scale:=0.60 _branch_approach_speed:=0.022 _white_gap_enabled:=true _white_gap_frames:=8 _white_gap_speed:=0.020 _white_min_pixels:=500 _yellow_guard_enabled:=true _yellow_guard_center_ratio:=0.32 _yellow_guard_height_ratio:=0.08 _yellow_min_pixels:=350
```

关键参数说明：

- `_roi_top_ratio:=0.50`：从画面中线开始看，比只看底部 1/3 更容易跨过虚线空白段。
- `_close_kernel_width:=13 _close_kernel_height:=61`：用竖向长条闭运算连接前后虚线，尽量不把黑色区域横向糊宽。
- `_layer_count:=12 _lookahead_layer:=4 _lookahead_weight:=0.35`：滑窗从近场逐层找中心线，近场占主导，前瞻点只提供弯道趋势，避免半圆弯直接原地猛拧。
- `_straight_hold_enabled:=true _straight_hold_frames:=4`：当近场仍居中但远处已经看到弯时，保持几帧低速直行，推迟入弯，解决直线进弯过早转向。
- `_straight_hold_near_error:=35 _straight_hold_curve_delta:=90 _straight_hold_speed:=0.025`：近场误差小于 35px 且远处曲率大于 90px 时触发，触发期间按近场点控制并低速前进。
- `_branch_lock_enabled:=true _branch_approach_hold:=true`：远处分叉先不允许前瞻点左右抢目标，近场仍直时保持近场中心；到分叉口后按 `_branch_choice` 提交并锁定。
- `_branch_split_delta:=120 _branch_commit_frames:=8`：远处左右候选中心差超过 120px 视为分叉，提交后锁定 8 帧，避免左右跳。
- `_branch_route_memory_frames:=55`：分叉提交后继续记住所选 `left/right` 一段时间，半圆中途再次看到另一侧半圆时不立刻切过去。
- `_branch_target_weight:=0.40 _branch_max_angular_scale:=0.60`：分叉目标只占 40%，其余看近场中心；分叉半圆阶段角速度再乘 0.60，避免入弯和半圆中途转向过大。
- `_branch_exit_hold_frames:=10`：走完半圆后如果近场重新居中，短暂按近场直行，减少被另一边半圆吸走。
- `_white_gap_enabled:=true _white_gap_frames:=8`：白色斑马线只作为短时遮挡跨越，不进入黑色可走区域 mask；地图外白框不会被当成合法路段。
- `_turn_slowdown:=0.45 _curve_slowdown:=0.35 _min_curve_speed:=0.012`：转弯时降速但保留极低速爬行，让连续半圆弯先向前走一点再慢慢转。
- `_yellow_guard_enabled:=true`：ROI 底部车身中心带看到黄线时，线速度清零并小角速度远离黄线。
- `_yellow_guard_center_ratio:=0.32 _yellow_min_pixels:=350`：黄线保护只看更窄的车身中心带，减少远处/侧边黄线导致原地卡住。
- `_max_angular_speed:=0.16`：先用更小角速度验证，避免麦克纳姆轮横向滑移把 9cm 左右余量吃掉。

另开一个终端观察它是否真的在发速度：

```bash
rostopic echo /cmd_vel
```

停车优先使用 Web STOP 或键盘遥控 STOP；如果节点失控或无法退出，直接 `Ctrl-C` 停掉 `rosrun` 终端，并发布 20Hz 零速度：

```bash
rostopic pub -r 20 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

> 搁置边界：地图工具补齐前，不再继续调视觉巡线参数；不要把本节命令当作当前推荐启动流程。恢复该方向时，应先补地图/赛道语义工具，再重新做离线回放、0 速度 smoke test 和有人看护的低速实车验证。

### 9. x11vnc 断线后的恢复

如果 Remmina 再次提示无法连接 VNC 服务器，先在小车 SSH 终端执行：

```bash
x11vnc -display :0 -forever -nopw -rfbport 5900 -auth guess -noxdamage
```

然后在电脑端 Remmina 连接：

```text
协议：VNC
地址：10.68.225.179:5900
密码：不需要填
```

注意：如果 `x11vnc` 是前台运行，SSH 终端不要关；如果要后台运行，推荐：

```bash
nohup x11vnc -display :0 -forever -nopw -rfbport 5900 -auth guess -noxdamage >/tmp/x11vnc.log 2>&1 &
```

### 10. 当前无线 IP 与 Web 面板恢复

当前 `iQ` 网络下：

- 本机无线地址：`10.68.225.14/24`
- 小车无线地址：`10.68.225.179`
- Web 面板：`http://10.68.225.179:8080/`

先确认小车在线：

```bash
ping -c 3 -W 2 10.68.225.179
nc -v -w 3 10.68.225.179 22
```

如果 Web 打不开，先 SSH：

```bash
ssh ucar@10.68.225.179
```

在小车上检查 ROS Master 和 Web：

```bash
source /opt/ros/melodic/setup.bash
source /home/ucar/ucar_ws/devel/setup.bash
export ROS_MASTER_URI=http://10.68.225.179:11311
export ROS_IP=10.68.225.179
rostopic list
ss -ltnp 'sport = :11311'
ss -ltnp 'sport = :8080'
curl -I http://127.0.0.1:8080/
```

如果 `ucar_web.service` 是 active 但 `8080` 不监听，或者雷达只转一下就停，通常是底层 ROS Master 没起来。先确认当前小车 IP；网线直连时曾是 `10.42.0.159`，Wi-Fi 可能是其他网段。必须同时修 `/etc/systemd/system/ucar_sensors.service`、`/etc/systemd/system/ucar_web.service` 和 `/home/ucar/start_sensors.sh` 中的 `ROS_MASTER_URI/ROS_IP`，再重启：

```bash
sudo systemctl daemon-reload
sudo systemctl restart ucar_sensors.service
sudo systemctl restart ucar_web.service
```

### 11. 人体跟随 Web 封装

人体跟随目前通过 Web 后端启动原厂节点，不建议 SSH 后直接裸跑。原因是跟随节点会控制 `/cmd_vel`，必须让 Web STOP、雷达障碍、导航/巡逻/手动控制互斥统一接管。

当前 Web 面板：

```text
http://10.68.225.179:8080/
```

后端接口：

```bash
curl -s http://10.68.225.179:8080/api/person_follow/status
curl -s -X POST http://10.68.225.179:8080/api/person_follow/start
curl -s -X POST http://10.68.225.179:8080/api/person_follow/stop
```

语音或手动输入支持：

```text
跟着我
开始人体跟随
结束跟随
停止跟随
```

Web 后端实际启动命令：

```bash
python3 -u /home/ucar/web_panel/person_follow_adapter.py
```

注意：当前 Web 后端实际启动的是 `/home/ucar/web_panel/person_follow_adapter.py`，不是原厂 `person_detect/person_follow.py`。适配器优先使用原厂 `person_detect.pth` 的 TensorRT/trt_pose 模型，失败时回退 YOLO/HOG。判断包名/节点名时以 `rospack find` 和 `rosnode list` 的当前输出为准，不要按旧笔记误判。

连接小车前必须重新确认当前在线 IP；本文档里的 `10.68.225.179` 只是历史示例，当前会随网络变化，例如本次实测在线 IP 是 `10.241.30.179`。

如果点击“开始跟随”提示查看 `/tmp/person_follow.log`，先看日志：

```bash
sed -n '1,120p' /tmp/person_follow.log
```

常见错误：

- `setsid: invalid option -- 'f'`：小车系统的 `setsid` 不支持 `-f`，Web 后端应使用 `nohup bash -lc 'exec ...' &`。
- `package 'person_follow' not found`：包名写错了，应使用 `person_detect`。
- `person detect node is started,waiting for client...`：这是旧原厂节点日志；当前适配器不再依赖它。如果还看到这类日志，说明系统里还有旧进程残留，需要先停掉旧的 `person_detect/person_follow.py` 再启动适配器。
- 相机相关报错：原厂跟随脚本订阅 `/ucar_camera/image_raw`，不是 `/camera/image_raw`。
- 页面提示 `相机话题 /ucar_camera/image_raw 未就绪`，但 `rosnode list` 能看到 `ucar_camera`：检查 `/dev/video0` 是否被 Web 预览占用。

```bash
fuser -v /dev/video0 /dev/ucar_video
rostopic hz /ucar_camera/image_raw -w 1
```

如果同时看到 Web 后端和 `ucar_camera.py` 占用 `/dev/video0`，先停止 Web 页面 Camera 预览，或重启 Web 后再点“开始跟随”。当前 Web 已在点击“开始跟随”前自动关闭 Camera 预览。

实车启动前先检查底盘、雷达、相机：

```bash
source /opt/ros/melodic/setup.bash
source /home/ucar/ucar_ws/devel/setup.bash
export ROS_MASTER_URI=http://10.68.225.179:11311
export ROS_IP=10.68.225.179
rostopic list
rostopic hz /scan -w 1
rostopic hz /odom -w 1
rostopic hz /ucar_camera/image_raw -w 1
```

如果没有 `/ucar_camera/image_raw`，启动相机：

```bash
rosrun ucar_camera ucar_camera.py _cam_topic_name:=/ucar_camera/image_raw _image_width:=640 _image_height:=480 _rate:=10
```

注意：实测原厂 `ucar_camera.py` 可能进程存在、话题存在，但 `rostopic hz /ucar_camera/image_raw` 一直没有新消息。若直接 OpenCV `CAP_V4L2` 能读 `/dev/video0`，则使用 Web 面板“开始跟随”让后端启动 `/home/ucar/web_panel/ucar_camera_bridge.py` 发布 `/ucar_camera/image_raw`，不要继续反复重启原厂相机节点。

相机桥后台启动必须直接运行 Python，不要再套内层 `bash -lc`：

```bash
nohup python3 -u /home/ucar/web_panel/ucar_camera_bridge.py _device:=/dev/video0 _topic:=/ucar_camera/image_raw _width:=1280 _height:=720 _rate:=10 >/tmp/ucar_camera_bridge.log 2>&1 < /dev/null &
```

启动人体跟随前释放摄像头的硬流程：

```bash
fuser /dev/video0 /dev/ucar_video
```

后端会强杀占用 `/dev/video0` 的非 Web 后端残留进程，然后等待 1.5 秒，再启动相机桥。不要跳过这个缓冲时间，否则 USB 视频节点可能还没被内核完全释放。
释放前也会停止巡线样本记录器，避免 Web 后端自身因为 recorder 仍 active 而继续占用摄像头。

相机桥日志：

```bash
sed -n '1,160p' /tmp/ucar_camera_bridge.log
```

验证相机桥是否真的发布时，优先确认能收到完整 `sensor_msgs/Image`；`rostopic hz` 对大图像在这台车上可能误报 `no new messages`：

```bash
timeout 10 rostopic echo -n 1 /ucar_camera/image_raw
```

Web 后端启动人体跟随时使用 `rospy.wait_for_message("/ucar_camera/image_raw", Image, timeout=10.0)` 判定相机就绪。`ucar_camera_bridge.py` 如果连续 30 帧读不到图像，会 `logerr` 并退出进程；此时看 `/tmp/ucar_camera_bridge.log`。
如果 10 秒内没有收到完整图像，后端不会启动原厂 `person_follow.py`，避免留下“相机未就绪但人体跟随节点已启动”的残留状态。

原厂人体跟随节点启动后不会自动跟随，会等待服务 client。手动验证流程：

```bash
rosrun person_detect person_follow.py
rosservice list | grep /person_follow
rosservice call /person_follow/start_person_detect '{}'
rosservice call /person_follow/get_detect_info '{}'
# 注意：原厂 start_person_follow 可能进入跟随回调后不返回，手动调试要包 timeout，Web 后端用后台调用。
timeout 10 rosservice call /person_follow/start_person_follow 'target_id: 0'
```

原厂节点冷启动会加载 TensorRT 网络，服务可能十几秒后才出现；Web 后端等待 `/person_follow/start_person_detect` 和 `/person_follow/start_person_follow` 的窗口应保持 60 秒。 如果 `timeout 10 rosservice call /person_follow/start_person_follow 'target_id: 0'` 卡住，但 `/tmp/person_follow.log` 出现 `startpersonFollowCB` 和 `folllow_id`，说明原厂回调已经进入跟随逻辑，不是 service 不存在。不要让 Web 请求同步等待这个 service 返回；`/api/person_follow/start` 应立即返回“启动中”，后台线程继续完成启动，页面靠 `/api/person_follow/status` 或 `/api/status` 轮询状态。若 `start_person_detect` 超时但 `/tmp/person_follow_start_service.log` 出现 `Start person following. Following_id is 0`，按跟随请求已提交处理。

安全规则：

- 点击“开始跟随”会停止视觉巡线、导航和巡逻，并先发布零速度；原厂跟随服务成功提交后，Web 后端必须释放 `MANUAL_MODE`，不能继续 20Hz 发布零速度覆盖 `/cmd_vel`。
- 点击 STOP、导航目标、设置 AMCL、手动接管、恢复导航、巡逻控制、语音运动/导航/巡逻都会停止人体跟随。
- 方向键和松手停止只在后端缓存显示人体跟随仍运行时才清理人体跟随进程；人体跟随已经停止时不能每次方向控制都执行 `pkill`，否则 Web 控制会变成数秒级延迟。
- 2026-05-14 起，人体跟随调试时暂时不再被 Web 前方障碍状态拦截：前方障碍不会拒绝启动人体跟随，也不会自动停止人体跟随。Web STOP、手动停止和方向键接管仍然可用。

如果方向键、松手停止或状态轮询变慢，先测接口耗时：

```bash
curl -s -o /tmp/curl_body -w '/api/manual_stop %{http_code} %{time_total}\n' -X POST http://10.68.225.179:8080/api/manual_stop
curl -s -o /tmp/curl_body -w '/api/cmd_vel %{http_code} %{time_total}\n' -H 'Content-Type: application/json' -d '{"linear_x":0,"angular_z":0}' http://10.68.225.179:8080/api/cmd_vel
```

修复后，在人体跟随已经停止时，`/api/manual_stop` 和 `/api/cmd_vel` 应接近几十毫秒级；如果又回到数秒级，优先检查是否又把普通手动控制接回了无条件 `stop_person_follow()`。
- 不要在无人看护、空间狭窄或 STOP 不可用时测试。

应急停止：

```bash
curl -s -X POST http://10.68.225.179:8080/api/person_follow/stop
ssh ucar@10.68.225.179
pkill -f 'person_follow.py|person_follow_node'
rostopic pub -r 20 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

只做非运动验证时，不要调用 `/api/person_follow/start`，只检查状态接口和 Web 页面是否正常：

```bash
curl -s http://10.68.225.179:8080/api/person_follow/status
curl -I http://10.68.225.179:8080/
```
