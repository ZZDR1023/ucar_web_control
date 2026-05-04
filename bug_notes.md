# ROSCAR1 Bug 记录与解决方法

本文档用于记录调试小车过程中遇到的 bug、原因分析、解决方法，以及以后更好的使用方式。

## 记录格式

每个问题建议按以下结构记录：

```text
问题：
现象：
原因：
解决方法：
以后如何避免/更好使用：
相关命令：
```

---

## 1. 小车不是 ROS2，`ros2` 命令不可用

### 问题

最初尝试执行：

```bash
ros2 topic list
```

### 现象

终端提示：

```text
bash: ros2: command not found
```

### 原因

小车系统安装的是 ROS 1 Melodic，不是 ROS2。

检查 `/opt/ros` 后发现：

```text
/opt/ros/melodic
```

### 解决方法

改用 ROS1 命令，例如：

```bash
rostopic list
```

含义：查看 ROS1 当前所有话题。

```bash
roslaunch ucar_controller base_driver.launch
```

含义：启动 ROS1 的底盘驱动 launch 文件。

### 以后更好的使用

以后在这台小车上默认使用 ROS1 命令：

```text
rostopic
rosnode
roslaunch
rosrun
rosparam
```

不要使用 ROS2 命令：

```text
ros2 topic
ros2 node
ros2 launch
```

除非以后重新安装或迁移到 ROS2。

---

## 2. `rostopic list` 提示无法连接 ROS Master

### 问题

执行：

```bash
rostopic list
```

### 现象

提示：

```text
ERROR: Unable to communicate with master!
```

### 原因

ROS Master 没有启动。也就是没有 `roscore` 或 `roslaunch` 进程在运行。

当时检查发现：

```text
11311 端口没有监听
没有 roscore / rosmaster / roslaunch 进程
```

### 解决方法

启动底盘驱动：

```bash
source /opt/ros/melodic/setup.bash
source ~/ucar_ws/devel/setup.bash
export ROS_MASTER_URI=http://10.90.122.179:11311
export ROS_IP=10.90.122.179
roslaunch ucar_controller base_driver.launch
```

含义：

- `source /opt/ros/melodic/setup.bash`：加载 ROS1 Melodic 环境。
- `source ~/ucar_ws/devel/setup.bash`：加载小车工作空间环境。
- `export ROS_MASTER_URI=...`：指定 ROS Master 地址。
- `export ROS_IP=...`：指定小车自身通信 IP。
- `roslaunch ucar_controller base_driver.launch`：启动底盘驱动，同时启动 ROS Master。

### 以后更好的使用

以后如果看到：

```text
Unable to communicate with master
```

先检查底盘是否启动：

```bash
ps -ef | grep -E 'rosmaster|roslaunch|base_driver'
```

含义：查看是否有 ROS Master、roslaunch 或底盘驱动进程。

或者直接使用一键脚本：

```bash
./start_sensors.sh
```

含义：同时启动底盘和雷达。

---

## 3. 无线控制时小车 IP 变化或连接失败

### 问题

使用旧 IP 连接小车时失败。

### 现象

执行：

```bash
ping -c 3 -W 2 10.90.122.179
```

可能出现：

```text
Destination Host Unreachable
```

或：

```text
100% packet loss
```

### 原因

可能原因包括：

1. 小车没有连接到热点；
2. 手机热点没有打开；
3. 小车 IP 发生变化；
4. 电脑和小车不在同一个网络；
5. 小车还没启动完成。

### 解决方法

先检查网络是否能通：

```bash
ping -c 3 -W 2 10.90.122.179
```

含义：向小车发送 3 个测试包，每个最多等 2 秒。

如果不通，查看本机网络：

```bash
ip route
```

含义：查看电脑当前网络路由和网段。

也可以查看局域网设备：

```bash
arp -a
```

含义：查看电脑已经发现过的局域网设备。

### 以后更好的使用

如果换热点或 IP 变化，需要同步修改本地脚本里的：

```python
ROBOT_IP = "10.90.122.179"
```

以及小车启动脚本里的：

```bash
export ROS_MASTER_URI=http://10.90.122.179:11311
export ROS_IP=10.90.122.179
```

---

## 4. 交互式键盘脚本在 Claude 工具里运行失败

### 问题

尝试通过工具直接运行键盘遥控脚本。

### 现象

报错：

```text
termios.error: (25, 'Inappropriate ioctl for device')
```

### 原因

键盘遥控脚本需要真实的交互式终端 TTY 来读取按键。

Claude 工具环境不是完整交互式终端，所以 `termios` 不能读取键盘输入。

### 解决方法

改成让用户在自己的终端运行，或者让本地脚本通过 SSH 分配交互式 TTY。

后来脚本改为：

1. 先把远程 Python 脚本写入小车临时文件；
2. 再使用交互式 SSH 运行；
3. 避免通过管道 `echo ... | python` 运行交互式程序。

### 以后更好的使用

凡是需要键盘实时输入的程序，优先在真实终端运行：

```bash
python3 /home/zzdr1023/xm/ROSCAR1/safe_control_with_status.py
```

含义：在本机启动包装脚本，由它 SSH 到小车运行交互式控制程序。

或者 SSH 登录小车后运行：

```bash
./start_control.sh
```

含义：在小车上启动联合遥控与状态监控界面。

---

## 5. 远程 Python 脚本 `HELP` 字符串语法错误

### 问题

远程键盘控制脚本启动时报语法错误。

### 现象

```text
File "<stdin>", line 12
    HELP =
          ^
SyntaxError: invalid syntax
```

### 原因

远程脚本中的多行字符串拼接方式错误，导致 `HELP` 变量没有正确生成。

### 解决方法

把错误的字符串写法改成标准三引号字符串：

```python
HELP = """
Safe keyboard control is running.
...
"""
```

### 以后更好的使用

远程生成 Python 脚本时，尽量避免复杂嵌套引号。

如果脚本内容较长，优先使用：

```text
base64 编码传输
写入临时文件
再运行临时文件
```

这样可以减少 SSH 引号转义问题。

---

## 6. 控制界面输出斜着漂移

### 问题

联合控制界面显示成斜线状，每一行都向右偏移。

### 现象

界面类似：

```text
Safe Control With Status
                        ----------------------------------------
                                                                w: forward ...
```

### 原因

脚本使用了：

```python
tty.setraw(fd)
```

进入 raw 终端模式后，普通换行 `\n` 只换到下一行，但不一定回到行首。

### 解决方法

将界面输出改成显式使用：

```text
\r\n
```

含义：

- `\r`：回到行首；
- `\n`：换到下一行。

同时改成一次性输出固定宽度面板，减少刷新错位。

### 以后更好的使用

凡是在 raw 终端模式下刷新界面，不要只用 `print()` 或 `\n`，应使用：

```python
sys.stdout.write('\r\n'.join(lines))
```

并手动 flush：

```python
sys.stdout.flush()
```

---

## 7. 按键控制有延迟，按 `w` 后再按 `a` 仍继续前进一会

### 问题

键盘控制响应有延迟。

### 现象

按 `w` 后再按 `a`，小车仍按旧的 `w` 方向走一会儿，之后才旋转。

### 原因

SSH/终端输入会缓存按键。原脚本每轮只读取一个字符，所以旧按键会排队执行。

### 解决方法

把读取逻辑改成：

```text
每轮读取所有已经排队的按键，只执行最后一个按键
```

这样旧输入会被丢弃，只响应最新按键。

### 以后更好的使用

交互式遥控程序中，应避免逐个处理过期按键。

更适合的逻辑是：

```text
读取输入缓冲区全部内容
保留最后一次有效输入
立即执行最新控制意图
```

---

## 8. 小车不能同时前进和转弯

### 问题

一开始按 `w` 前进，再按 `a` 后只会原地左转，不会前进左转。

### 原因

旧脚本中每次按键都会重新创建一个新的 `Twist()`：

```python
w 只设置 linear.x
a 只设置 angular.z
```

所以按 `a` 时，`linear.x` 又变回 0。

### 解决方法

将控制逻辑改成保存当前控制状态：

```python
control = {
    'linear': 0.0,
    'angular': 0.0,
    'gear': DEFAULT_GEAR,
}
```

按 `w/s` 只改变线速度，按 `a/d` 只改变角速度。

这样可以实现：

```text
w 后按 a = 前进 + 左转
w 后按 d = 前进 + 右转
```

### 以后更好的使用

对于 `/cmd_vel` 控制，应该理解：

```text
linear.x 和 angular.z 可以同时非零
```

这不是硬件限制，而是控制代码逻辑决定的。

---

## 9. `catkin_make` 没有编译新建的 `ucar_practice` 包

### 问题

创建 `ucar_practice` 包后，普通执行：

```bash
catkin_make
```

没有编译新包。

### 原因

小车工作空间设置了编译白名单：

```text
CATKIN_WHITELIST_PACKAGES: ydlidar_ros_driver
```

所以默认只编译白名单里的包。

### 解决方法

临时指定编译我们的包：

```bash
catkin_make -DCATKIN_WHITELIST_PACKAGES=ucar_practice
```

含义：只编译 `ucar_practice` 这个包。

### 以后更好的使用

如果以后新增 ROS 包，但 `catkin_make` 没有处理它，先检查是否有白名单：

```bash
catkin config
```

或看 `catkin_make` 输出中的：

```text
CATKIN_WHITELIST_PACKAGES
```

不要盲目认为包坏了。

---

## 10. 雷达最初无法启动，误把 `/dev/ttyUSB0` 当成雷达

### 问题

启动 YDLidar 时没有 `/scan` 数据。

### 现象

尝试使用 `/dev/ttyUSB0` 启动雷达时，部分配置显示连接，但没有稳定 `/scan`。

也曾出现：

```text
Device Block
```

### 原因

`/dev/ttyUSB0` 实际不是雷达，而是底盘串口。

系统设备关系是：

```text
/dev/base_serial_port -> ttyUSB0
```

底盘驱动 `base_driver` 使用这个串口和下位机通信。

真正雷达串口是官方初始化脚本里标注的：

```text
/dev/ttyTHS1
```

对应脚本：

```text
~/ucar_ws/src/startup_scripts/initdev_mini.sh
```

里面有：

```bash
KERNEL=="ttyTHS1" MODE="0666" # 雷达串口
```

### 解决方法

使用 `/dev/ttyTHS1` 启动 YDLidar G4 配置。

最终创建专用 launch：

```text
~/ucar_ws/src/ydlidar_ros_driver/launch/ucar_g4.launch
```

关键参数：

```text
port: /dev/ttyTHS1
baudrate: 230400
sample_rate: 9
range_min: 0.1
range_max: 16.0
frequency: 10.0
frame_id: laser_frame
```

并创建：

```text
~/start_lidar.sh
```

### 验证结果

成功出现：

```text
/scan
/point_cloud
```

`/scan` 频率：

```text
average rate: 10.0 Hz
```

雷达信息：

```text
Model: G4
Firmware version: 3.2
Hardware version: 3
Sample Rate: 9K
Scan Frequency: 10Hz
```

### 以后更好的使用

不要再用 `/dev/ttyUSB0` 启动雷达。

正确关系是：

```text
底盘：/dev/base_serial_port -> /dev/ttyUSB0
雷达：/dev/ttyTHS1
```

以后启动雷达使用：

```bash
./start_lidar.sh
```

含义：启动已修正端口的 U.CAR G4 雷达配置。

---

## 11. `sensors.launch` 第一次测试时只看到雷达话题，没有底盘话题

### 问题

创建 `sensors.launch` 后，第一次测试只看到：

```text
/scan
/point_cloud
/tf
```

没有：

```text
/cmd_vel
/odom
/imu
/battery_state
```

### 原因

测试时在同一条远程命令里同时执行了多个 `pkill -f ...` 和启动命令，`pkill` 的匹配范围可能影响当前启动命令或相关进程，导致 launch 没有完整启动。

### 解决方法

将流程拆开：

1. 单独清理旧进程；
2. 再单独启动 `sensors.launch`。

最终验证成功：

```text
/battery_state
/cmd_vel
/imu
/odom
/scan
/point_cloud
/tf
```

频率正常：

```text
/scan 约 10 Hz
/odom 约 20 Hz
```

### 以后更好的使用

不要在同一条长命令里同时写复杂的 `pkill` 和启动逻辑。

更稳妥的方式：

```bash
pkill -x base_driver
pkill -f ydlidar_ros_driver_node
```

确认清理后，再启动：

```bash
./start_sensors.sh
```

---

## 12. 设备路径混淆总结

### 问题

调试过程中容易混淆底盘、雷达、摄像头、麦克风设备。

### 当前确认的设备关系

```text
底盘串口：/dev/base_serial_port -> /dev/ttyUSB0
雷达串口：/dev/ttyTHS1
摄像头：/dev/ucar_video -> /dev/video0
麦克风阵列：USB 10d6:b003 Actions Semiconductor
```

### 以后更好的使用

检查设备时可执行：

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/ttyTHS1 /dev/ydlidar /dev/base_serial_port /dev/ucar_video
```

含义：查看底盘、雷达和摄像头相关设备节点。

查看 USB 设备：

```bash
lsusb
```

含义：列出系统识别到的 USB 硬件。

---

## 13. 当前推荐启动方式

### 一键启动底盘 + 雷达

```bash
./start_sensors.sh
```

含义：同时启动底盘驱动和激光雷达驱动。

启动后应有：

```text
/cmd_vel
/odom
/imu
/tf
/scan
/point_cloud
```

### 一键启动控制台

```bash
./start_control.sh
```

含义：启动增强版联合遥控与状态监控界面。

### 检查雷达频率

```bash
rostopic hz /scan
```

含义：查看 `/scan` 激光雷达话题发布频率，正常约 10Hz。

### 检查里程计频率

```bash
rostopic hz /odom
```

含义：查看 `/odom` 里程计话题发布频率，正常约 20Hz。

---

## 14. 官方建图 launch 引用旧雷达包名

### 问题

官方建图入口：

```text
~/ucar_ws/src/ucar_map/launch/ucar_mapping.launch
```

里面引用了：

```xml
<include file="$(find ydlidar)/launch/ydlidar.launch"/>
```

### 现象

如果直接使用原始 `ucar_mapping.launch`，可能会找不到 `ydlidar` 包，或者使用错误雷达配置，导致 `/scan` 无法用于建图。

### 原因

当前小车实际雷达包是：

```text
ydlidar_ros_driver
```

并且实际可用雷达配置已经修正为：

```text
~/ucar_ws/src/ydlidar_ros_driver/launch/ucar_g4.launch
```

该配置使用正确串口：

```text
/dev/ttyTHS1
```

而不是旧配置里的 `/dev/ydlidar` 或错误的 USB 串口。

### 解决方法

不直接修改官方原文件，新增修正版建图入口：

```text
~/ucar_ws/src/ucar_map/launch/ucar_mapping_fixed.launch
```

内容包含：

```text
ucar_controller/base_driver.launch
ydlidar_ros_driver/ucar_g4.launch
ucar_map/cartographer_start.launch
```

使用命令：

```bash
roslaunch ucar_map ucar_mapping_fixed.launch
```

含义：启动修正版建图流程，同时启动底盘、正确雷达配置和 Cartographer。

### 以后如何避免/更好使用

以后建图优先使用：

```bash
roslaunch ucar_map ucar_mapping_fixed.launch
```

不要优先使用原始：

```bash
roslaunch ucar_map ucar_mapping.launch
```

除非已经确认原文件里的雷达 include 被修正。

### 验证方法

先检查参数：

```bash
roslaunch ucar_map ucar_mapping_fixed.launch --dump-params
```

含义：只展开并检查 launch 参数，不真正启动小车运动。

已验证参数中雷达端口为：

```text
/ydlidar_lidar_publisher/port: /dev/ttyTHS1
```

---

## 15. Cartographer 建图时 `base_link` 和 `laser_frame` 不在同一棵 TF 树

### 问题

启动：

```bash
roslaunch ucar_map ucar_mapping_fixed.launch
```

后，Cartographer 日志持续警告：

```text
Could not find a connection between 'base_link' and 'laser_frame'
Tf has two or more unconnected trees.
```

### 现象

虽然 `/scan`、`/odom`、`/submap_list` 存在，但 `/map` 没有正常输出地图信息。

检查 TF：

```bash
rosrun tf tf_echo base_link laser_frame
```

提示：

```text
laser_frame passed to lookupTransform argument source_frame does not exist
```

### 原因

`ucar_g4.launch` 里原来使用 ROS1 `tf/static_transform_publisher`：

```xml
<node pkg="tf" type="static_transform_publisher" name="base_link_to_laser4"
  args="0.0 0.0 0.2 0.0 0.0 0.0 /base_link /laser_frame 40" />
```

该静态 TF 发布节点启动后很快退出，导致 `laser_frame` 没有持续发布，Cartographer 找不到 `base_link -> laser_frame` 变换。

### 解决方法

改用 `tf2_ros/static_transform_publisher`，让静态变换发布到 `/tf_static`：

```xml
<node pkg="tf2_ros" type="static_transform_publisher" name="base_link_to_laser4"
  args="0.0 0.0 0.2 0.0 0.0 0.0 base_link laser_frame" />
```

修改文件：

```text
~/ucar_ws/src/ydlidar_ros_driver/launch/ucar_g4.launch
```

### 验证结果

再次启动建图后：

```bash
rosrun tf tf_echo base_link laser_frame
```

正常输出：

```text
Translation: [0.000, 0.000, 0.200]
Rotation: [0.000, 0.000, 0.000, 1.000]
```

读取地图信息：

```bash
rostopic echo -n 1 /map/info
```

正常输出：

```text
resolution: 0.05
width: 255
height: 172
```

### 以后如何避免/更好使用

建图前检查 TF：

```bash
rosrun tf tf_echo base_link laser_frame
```

含义：确认车体坐标系 `base_link` 到雷达坐标系 `laser_frame` 的变换存在。

建图时如果 `/scan` 有数据但 `/map` 不生成，优先检查 TF，而不是只检查雷达。

---

## 16. RViz 出现 “System program problem detected” 崩溃弹窗

### 问题

通过 VNC/远程桌面查看小车桌面时，Ubuntu 弹出：

```text
System program problem detected
```

左上角疑似 RViz 窗口附近出现系统崩溃报告。

### 现象

小车上存在 RViz 崩溃报告文件：

```text
/var/crash/_opt_ros_melodic_bin_rviz.1000.crash
```

但建图节点仍然正常运行，`/map`、`/scan`、`/odom` 等话题仍可用。

### 原因

RViz 之前通过 SSH X11 转发或 Docker/X11 方式启动时，OpenGL/GLX 渲染失败，产生了崩溃报告。Ubuntu 的 apport 会在桌面弹出系统程序崩溃提示。

典型错误包括：

```text
Unable to create a suitable GLXContext
System program problem detected
```

### 解决方法

不要继续用 SSH X11 或 Docker X11 跑 RViz，改用小车本机桌面/VNC 查看 RViz。

如果弹窗出现，可以关闭该弹窗，不影响 ROS 建图数据。

重新启动 RViz 时使用小车桌面显示：

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

含义：在小车本机桌面上运行 RViz，并尽量使用软件渲染，避免远程 X11 的 GLX 问题。

### 以后如何避免/更好使用

RViz 这类 OpenGL 程序不适合普通 SSH X11 转发。优先使用：

```text
VNC 连接小车桌面 → 在小车桌面运行 RViz
```

不要优先使用：

```text
ssh -Y 后直接 rviz
Docker X11 后直接 rviz
```

建图是否正常应以 ROS 话题为准，例如：

```bash
rostopic echo -n 1 /map/info
rostopic hz /scan
```

---

## 17. 地图包含门外区域，导航可能驶出房间

### 问题

建图时室内门是打开的，激光雷达可能扫到门外区域。以后做自主导航时，如果地图中门口被认为是可通行区域，小车可能规划路径穿过门。

### 现象

原始地图中主房间外侧存在门口延伸区域，特别是左下和右侧方向有可通行白色区域。

### 原因

导航使用静态地图和代价地图规划路径。如果门口在 `.pgm` 地图中是白色或未知可通行区域，导航算法可能认为门外也能走。

### 解决方法

保留原始地图不覆盖，单独创建封门版地图：

```text
本机：/home/zzdr1023/xm/ROSCAR1/maps/ucar_map_20260501_202713_sealed.pgm
本机：/home/zzdr1023/xm/ROSCAR1/maps/ucar_map_20260501_202713_sealed.yaml
小车：/home/ucar/ucar_ws/maps/ucar_map_20260501_202713_sealed.pgm
小车：/home/ucar/ucar_ws/maps/ucar_map_20260501_202713_sealed.yaml
```

处理方式：

- 用黑色障碍线围出主矩形活动区域；
- 左下门口封住；
- 右边界左移约半格；
- 删除右侧小矩形区域；
- 后续导航使用 `sealed.yaml`，不要使用未封门原图。

### 以后如何避免/更好使用

如果希望小车只在室内指定区域活动，有两种方式：

1. 建图时关门，让门自然成为障碍；
2. 建图后编辑 `.pgm`，用黑色障碍封住门口或越界区域。

导航时优先加载：

```text
/home/ucar/ucar_ws/maps/ucar_map_20260501_202713_sealed.yaml
```

不要加载：

```text
/home/ucar/ucar_ws/maps/ucar_map_20260501_202713.yaml
```

除非明确允许小车探索门外区域。

---

## 18. 导航 launch 直接报 `ucar_navigation_fixed.launch` 找不到

### 问题

执行：

```bash
roslaunch ucar_nav ucar_navigation_fixed.launch
```

时，报错：

```text
RLException: [ucar_navigation_fixed.launch] is neither a launch file in package [ucar_nav] nor is [ucar_nav] a launch file name
```

### 现象

明明文件已经创建，但启动时仍然提示找不到该 launch。

### 原因

通常是当前终端没有正确加载工作空间环境，或者不是在小车终端中执行。也可能是没有先执行：

```bash
source /opt/ros/melodic/setup.bash
source ~/ucar_ws/devel/setup.bash
```

### 解决方法

在小车 SSH 终端中先执行：

```bash
source /opt/ros/melodic/setup.bash
source ~/ucar_ws/devel/setup.bash
export ROS_MASTER_URI=http://10.90.122.179:11311
export ROS_IP=10.90.122.179
roslaunch ucar_nav ucar_navigation_fixed.launch
```

或者先确认包路径：

```bash
rospack find ucar_nav
```

如果输出：

```text
/home/ucar/ucar_ws/src/ucar_nav
```

说明环境已正确加载。

### 以后如何避免/更好使用

启动导航前，固定执行：

```bash
source /opt/ros/melodic/setup.bash
source ~/ucar_ws/devel/setup.bash
export ROS_MASTER_URI=http://10.90.122.179:11311
export ROS_IP=10.90.122.179
```

然后再运行：

```bash
roslaunch ucar_nav ucar_navigation_fixed.launch
```

不要直接在未加载环境的终端里执行 launch。

---

## 19. 导航启动后 `move_base` 因全局规划器插件不存在而崩溃

### 问题

执行导航 launch 后，小车不动，雷达也停止转动。

### 现象

日志中出现：

```text
[FATAL] Failed to create the voronoi_jps_ns/VoronoiJpsClass planner
Declared types are ... global_planner/GlobalPlanner ...
[move_base-6] process has died
```

随后雷达节点也退出，AMCL 开始报警：

```text
No laser scan received
```

### 原因

`ucar_navigation_fixed.launch` 中原先使用了：

```text
voronoi_jps_ns/VoronoiJpsClass
```

作为全局规划器，但当前小车系统并没有安装或注册这个插件。

`move_base` 崩溃后，整个 launch 的导航链条不完整，小车自然不会运动；同时雷达节点也会随 launch 状态变化而退出。

### 解决方法

把修正版导航 launch 中的全局规划器改成系统里实际存在的插件：

```text
global_planner/GlobalPlanner
```

修正后文件：

```text
~/ucar_ws/src/ucar_nav/launch/ucar_navigation_fixed.launch
```

验证方法：

```bash
roslaunch ucar_nav ucar_navigation_fixed.launch --dump-params
```

确认输出中有：

```text
/move_base/base_global_planner: global_planner/GlobalPlanner
```

### 以后如何避免/更好使用

导航 launch 中用到的 planner 插件不要想当然照搬旧文件。若 `move_base` 一启动就退出，优先检查：

```text
base_global_planner
base_local_planner
```

是否真的是当前系统已注册的插件。

如果不确定，先查看日志里 `Declared types are ...` 后面列出的可用插件，再选择其中一个。

---

## 20. 先启动 `start_sensors.sh` 再启动 `ucar_navigation_fixed.launch` 会把雷达挤掉

### 问题

用户先执行：

```bash
./start_sensors.sh
```

确认雷达已经开始转动后，再执行：

```bash
roslaunch ucar_nav ucar_navigation_fixed.launch
```

结果雷达停止转动，小车也不动。

### 现象

日志中会出现：

```text
Reason given for shutdown: [[/base_driver] Reason: new node registered with same name]
Reason given for shutdown: [[/ydlidar_lidar_publisher] Reason: new node registered with same name]
```

随后出现：

```text
No laser scan received
```

### 原因

`./start_sensors.sh` 已经启动了：

```text
base_driver
ydlidar_lidar_publisher
```

而 `ucar_navigation_fixed.launch` 里也再次启动了同名的：

```text
base_driver
ydlidar_lidar_publisher
```

同名节点重复注册后，ROS 会让旧节点或新节点退出，结果导致雷达停止、`/scan` 中断，导航链路无法成立。

### 解决方法

不要在 `start_sensors.sh` 之后再启动会重复拉起底盘和雷达的导航 launch。

改为新建运行版导航入口：

```text
~/ucar_ws/src/ucar_nav/launch/ucar_navigation_runtime.launch
```

这个运行版 launch 只启动：

```text
map_server
AMCL
move_base
```

不再重复启动底盘和雷达。

正确顺序：

1. 先执行：

```bash
./start_sensors.sh
```

2. 再执行：

```bash
source /opt/ros/melodic/setup.bash
source ~/ucar_ws/devel/setup.bash
export ROS_MASTER_URI=http://10.90.122.179:11311
export ROS_IP=10.90.122.179
roslaunch ucar_nav ucar_navigation_runtime.launch
```

### 以后如何避免/更好使用

以后要把 launch 文件分成两类：

- `fixed.launch`：完整启动版，适合单独一条命令拉起全部节点；
- `runtime.launch`：运行版，假设底盘和雷达已经启动，不再重复拉起同名节点。

如果已经先执行了：

```bash
./start_sensors.sh
```

后续导航就应优先使用：

```bash
roslaunch ucar_nav ucar_navigation_runtime.launch
```

---

## 21. `header_crc8 error` / `header_crc8 error,sensro` 串口校验告警

### 问题

在传感器/底盘终端中出现告警：

```text
[ WARN] ... header_crc8 error
[ WARN] ... header_crc8 error,sensro
```

### 现象

小车在运行底盘或传感器相关 launch 时，终端偶发出现 CRC8 校验错误提示。

### 原因

这类告警通常来自底盘控制板与上位机之间的串口通信帧校验失败。

结合当前工程，底盘串口是：

```text
/dev/base_serial_port -> /dev/ttyUSB0
```

常见原因包括：

1. 串口数据偶发干扰；
2. 供电波动；
3. USB 串口接触不稳；
4. 启停节点频繁，串口短时间重连；
5. 少量坏帧被驱动丢弃。

### 解决方法

如果只是偶发一两条，而 `/odom`、`/imu`、`/cmd_vel` 仍正常，则先继续使用，不必立即停机。

可以用下面命令确认底盘数据是否还在正常更新：

```bash
rostopic hz /odom
rostopic echo -n 1 /odom
rostopic hz /imu
```

如果告警频繁出现，并伴随以下现象：

- 小车不响应控制；
- `/odom` 频率明显掉下去；
- 底盘节点频繁重连；

则应：

1. 停掉当前底盘相关 launch；
2. 重新插拔底盘串口或检查接线；
3. 重新启动 `./start_sensors.sh` 或底盘 launch；
4. 检查供电是否稳定。

### 以后如何避免/更好使用

- 避免同时重复启动多个会访问底盘的 launch；
- 先清理旧节点，再重新启动；
- 如果只是偶发 CRC 告警但话题正常，不要立刻判断系统坏了，应先看 `/odom`、`/imu` 是否还在更新。

---

## 22. 导航 launch 未加载 `my_navigation.yaml`，导致目标坐标系为空、无法规划

### 问题

导航启动后，`move_base` 能起来，但发送目标点后不动。

### 现象

日志中出现：

```text
The goal pose passed to this planner must be in the  frame. It is instead in the map frame.
```

同时终端里还有大量：

```text
it does not get vehicle_length_half(move_base) !!!
it does not get current_to_goal_distance_threshold(move_base) !!!
it does not get allow_clear_global_costmap !!!
```

### 原因

`ucar_navigation_fixed.launch` / `ucar_navigation_runtime.launch` 一开始没有加载：

```text
~/ucar_ws/src/ucar_nav/launch/config/move_base/my_navigation.yaml
```

而小车这套自定义导航代码依赖该文件中的参数。

因为参数缺失，`move_base` 内部某些自定义逻辑没有拿到必要配置，最终导致“目标应当在哪个 frame 中”的判断为空字符串，于是拒绝接收 `map` 坐标系的目标。

### 解决方法

在两个导航 launch 中补上：

```xml
<rosparam command="load" file="$(find ucar_nav)/launch/config/move_base/my_navigation.yaml" />
```

涉及文件：

```text
~/ucar_ws/src/ucar_nav/launch/ucar_navigation_fixed.launch
~/ucar_ws/src/ucar_nav/launch/ucar_navigation_runtime.launch
```

### 验证结果

重新做参数展开检查：

```bash
roslaunch ucar_nav ucar_navigation_runtime.launch --dump-params
```

已能看到这些关键参数：

```text
/allow_clear_global_costmap: true
/current_to_goal_distance_threshold: 1.0
/vehicle_length_half: 0.171
/wait_time: 20.0
/move_base/base_global_planner: global_planner/GlobalPlanner
```

### 以后如何避免/更好使用

以后遇到大量：

```text
it does not get ...
```

这种提示，不要只盯着单个参数报错，要先检查 launch 是否漏加载了项目自己的 `.yaml` 参数文件。

对于这套车，`my_navigation.yaml` 属于自定义导航逻辑必需参数，不是可有可无的附加配置。

---

## 23. 导航 launch 还缺少 `move_base_params.yaml`，导致 `move_base` 段错误崩溃

### 问题

把全局规划器和 `my_navigation.yaml` 修好后，导航仍然可能在目标下发后直接崩溃：

```text
[move_base-3] process has died ... exit code -11
```

### 现象

`move_base` 不是普通报错退出，而是段错误（Segmentation fault）。

### 原因

对比原厂导航 launch 后发现，修正版 launch 还少加载了：

```text
~/ucar_ws/src/ucar_nav/launch/config/move_base/move_base_params.yaml
```

原厂 `ucar_navigation.launch` 虽然没显式加载，但对应的另一套 `developer_navigation` 标准模板里是会把 `move_base_params.yaml` 一并加载的。缺少这个文件后，`move_base` 的基础运行参数不完整，和厂商定制代码叠加后就可能直接崩溃。

### 解决方法

在两个导航 launch 中补上：

```xml
<rosparam file="$(find ucar_nav)/launch/config/move_base/move_base_params.yaml" command="load" />
```

涉及文件：

```text
~/ucar_ws/src/ucar_nav/launch/ucar_navigation_runtime.launch
~/ucar_ws/src/ucar_nav/launch/ucar_navigation_fixed.launch
```

### 验证结果

重新参数展开后，已经能看到这些 `move_base` 基础参数：

```text
/move_base/controller_frequency: 20.0
/move_base/planner_patience: 5.0
/move_base/oscillation_timeout: 8.0
/move_base/recovery_behavior_enabled: true
```

### 以后如何避免/更好使用

做 launch 修复时，不能只盯着“报错里提到的那个参数文件”，还要和原厂完整导航链对照，确认：

- `my_navigation.yaml`（厂商自定义参数）
- `move_base_params.yaml`（move_base 基础参数）
- `costmap_*` / `planner_*` 参数

是否全部被加载。

---

## 24. 即使修正 launch 与参数，厂商覆盖版 `move_base` 仍持续段错误

### 问题

在修正：

- 雷达 launch
- 地图路径
- `my_navigation.yaml`
- `move_base_params.yaml`
- 全局规划器插件

之后，`move_base` 仍然持续：

```text
exit code -11
```

### 原因

进一步检查发现，小车工作空间覆盖了标准 ROS1 导航核心包：

```text
move_base         -> ~/ucar_ws/src/developer_navigation/move_base
dwa_local_planner -> ~/ucar_ws/src/developer_navigation/local_planner_pkg/dwa_local_planner
costmap_2d        -> ~/ucar_ws/src/developer_navigation/costmap_2d
nav_core          -> ~/ucar_ws/src/developer_navigation/nav_core
```

也就是说，即使 launch 文件看起来是“标准 ROS1 配置”，真正运行时加载到的仍然是厂商魔改版核心导航库。

### 解决方法

新建独立导航工作空间：

```text
/home/ucar/nav_clean_ws
```

并在其中创建 `nav_clean` 包，至少把 launch/参数从原厂工作空间里分离出来，作为后续继续绕开厂商覆盖包的基础。

### 以后如何避免/更好使用

以后判断“到底是不是标准 ROS1 导航栈”时，不能只看 launch 内容，必须检查：

```bash
rospack find move_base
rospack find dwa_local_planner
rospack find costmap_2d
rospack find nav_core
```

如果这些指向的是工作空间里的自定义路径，而不是 `/opt/ros/melodic/share/...`，那就仍然在使用厂商覆盖版。

---

## 25. `nav_clean` 干净导航工作空间验证成功

### 问题

原厂/覆盖版 `move_base` 在多次修正 launch 和参数后仍然可能崩溃，无法稳定完成导航验证。

### 解决方法

新建独立工作空间：

```text
/home/ucar/nav_clean_ws
```

新建包：

```text
nav_clean
```

并使用最小导航入口：

```text
/home/ucar/nav_clean_ws/src/nav_clean/launch/navigation_runtime.launch
```

使用：

- 标准 `amcl`
- `navfn/NavfnROS`
- `DWAPlannerROS`
- 封门地图 `ucar_map_20260501_202713_sealed.yaml`

### 验证结果

在保持 `./start_sensors.sh` 运行的前提下，启动：

```bash
source /opt/ros/melodic/setup.bash
source /home/ucar/nav_clean_ws/devel/setup.bash
export ROS_MASTER_URI=http://10.90.122.179:11311
export ROS_IP=10.90.122.179
roslaunch nav_clean navigation_runtime.launch
```

再命令行发送目标点后，`/move_base/status` 返回：

```text
status: 3
text: "Goal reached."
```

说明这套干净导航链已经能接受目标并完成导航。

### 以后如何避免/更好使用

后续优先使用 `nav_clean` 工作空间做导航验证与演示；厂商工作空间保留给底盘、雷达、建图使用即可。

---

## 26. 小车 apt 源证书过期导致无法安装软件

### 问题

在小车上执行：

```bash
sudo apt update && sudo apt install -y x11vnc
```

### 现象

提示：

```text
Certificate verification failed
The certificate chain uses expired certificate
```

### 原因

小车系统是 Ubuntu 18.04，但使用的清华镜像源证书已过期：

```text
https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/
```

同时 ROS 源也有 GPG 签名过期问题。

### 解决方法

将 Ubuntu 源换成官方 ports 镜像：

```bash
sudo cp /etc/apt/sources.list /etc/apt/sources.list.bak
sudo sed -i 's|https://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports/|http://ports.ubuntu.com/ubuntu-ports/|g' /etc/apt/sources.list
```

将 ROS 源换成官方仓库：

```bash
sudo cp /etc/apt/sources.list.d/ros-fish.list /etc/apt/sources.list.d/ros-fish.list.bak
sudo sed -i 's|http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/|http://packages.ros.org/ros/ubuntu/|g; s|http://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu/|http://packages.ros.org/ros2/ubuntu/|g' /etc/apt/sources.list.d/ros-fish.list
```

然后重新执行：

```bash
sudo apt update && sudo apt install -y x11vnc
```

### 以后更好的使用

以后如果小车需要安装新软件但 apt 报证书错误，优先检查：

```bash
cat /etc/apt/sources.list
cat /etc/apt/sources.list.d/*.list
```

确认源地址是否已过期。

---

## 27. x11vnc 共享桌面时 SSH 终端报 `Could not connect to display`

### 问题

在 SSH 终端里执行：

```bash
rviz
```

### 现象

提示：

```text
qt.qpa.screen: QXcbConnection: Could not connect to display
Could not connect to any X display.
```

### 原因

SSH 终端没有设置 `DISPLAY` 和 `XAUTHORITY` 环境变量，无法连接到小车桌面的 X 显示。

### 解决方法

在 SSH 终端里先执行：

```bash
export DISPLAY=:0
export XAUTHORITY=/run/user/1000/gdm/Xauthority
```

然后再启动 RViz：

```bash
source /opt/ros/melodic/setup.bash
source ~/ucar_ws/devel/setup.bash
export ROS_MASTER_URI=http://10.90.122.179:11311
export ROS_IP=10.90.122.179
export LIBGL_ALWAYS_SOFTWARE=1
rviz
```

### 以后更好的使用

SSH 终端里运行 GUI 程序（RViz 等），必须先设置 DISPLAY 和 XAUTHORITY。

推荐在 Remmina 桌面里直接打开终端运行 GUI 程序，不需要手动设置。

---

## 28. 重启 GDM 后原桌面会话丢失

### 问题

执行：

```bash
sudo service gdm restart
```

### 现象

重启后原桌面会话退出，回到 GDM 登录界面。

### 原因

GDM 重启会杀死所有用户桌面会话，需要重新登录。

### 解决方法

重启 GDM 后，在 Remmina 里看到登录界面时：

1. 选择 `ucar` 用户
2. 输入密码 `ucar`
3. 登录进入桌面

登录后在桌面终端里运行 RViz 即可，不需要手动设置 DISPLAY。

### 以后更好的使用

如果只是想打开 RViz，不要重启 GDM。

重启 GDM 只在以下情况使用：

- 桌面卡死
- x11vnc 无法连接
- 需要重新登录桌面

---

## 29. Web 面板 `Takeover` 报 500：`NameError: name 'time' is not defined`

### 问题

点击网页里的 `Takeover` 按钮时，后端报错：

```text
File "/home/ucar/web_panel/server.py", line 166, in api_takeover
    time.sleep(1)
NameError: name 'time' is not defined
```

### 现象

- 网页按钮点了没完全生效；
- `takeover` 接口返回 500；
- 导航没有被真正停掉；
- `/cmd_vel` 仍可能被 `move_base` 抢占。

### 原因

`server.py` 中 `api_takeover()` 调用了：

```python
time.sleep(1)
```

但文件顶部没有：

```python
import time
```

### 解决方法

在 `server.py` 顶部补上：

```python
import time
```

然后重新上传到小车：

```bash
scp /home/zzdr1023/xm/ROSCAR1/web_panel/server.py ucar@10.90.122.179:/home/ucar/web_panel/server.py
```

### 以后如何避免/更好使用

后端接口新增逻辑后，尤其是加了 `sleep()`、时间控制、线程、超时相关代码时，要先检查对应模块是否已经导入。

---

## 30. 网页控制面板请求返回 200，但前端与后端版本不一致导致“按钮无效”

### 问题

浏览器网络面板能看到：

```text
POST /api/cmd_vel -> 200
POST /api/takeover -> 200
```

但网页按钮最初仍表现为：

- 点了没反应；
- 地图页报 `drawMap` 相关错误；
- 标签页切换不完整。

### 原因

问题不是单一层面：

1. 前端早期仍在用 `onmousedown/onmouseup`，点一下就会马上发 `stop`；
2. 地图渲染逻辑里 `traj.length` 在 `traj` 未定义时抛错，导致整个前端脚本后续交互受影响；
3. 浏览器容易缓存旧版 `index.html` / JS，导致后端已更新但前端仍在执行旧逻辑；
4. 后端 `/cmd_vel` 早期只发单次消息，或者 shell 拼接有问题，导致请求 200 但 ROS 里看不到持续速度命令。

### 解决方法

- 把前端方向按钮改成更稳定的交互模型；
- 给地图/相机相关逻辑加防崩保护，不让它拖死整个页面；
- 后端切换为更稳定的 `/cmd_vel` 发布方式（独立脚本 / 持续发布）；
- 每次修改后用无痕窗口或 `Ctrl+Shift+R` 强制刷新页面。

### 以后如何避免/更好使用

以后网页调试要按这个顺序判断：

1. 浏览器网络里是否真的有 `cmd_vel` / `takeover` 请求；
2. 小车端 `/cmd_vel` 是否真正收到消息；
3. 浏览器是否还在用旧缓存页面；
4. 地图/摄像头错误是否把整个前端脚本拖死。

---

## 31. Web 面板发送 Goal 后小车没有反应

### 问题

网页 `Navigation Goal` 里输入目标点后点击 `GO`，请求可能返回成功，但小车没有开始导航。

### 现象

- 前端能看到 `Goal: x, y` 日志；
- `/api/goal` 没有明显 500；
- 导航目标看似发送了，但小车不动；
- Web 后端仍可能持续向 `/cmd_vel` 发布零速度。

### 原因

`web_panel/server.py` 的后台 `cmd_loop()` 原本 20Hz 持续发布 `/cmd_vel`，即使进入导航模式也会发布当前速度值。当前速度默认为 0 时，会持续覆盖 `move_base` 输出，导致导航目标已接收但底盘速度被 Web 面板压成 0。

另一个问题是 `/api/goal` 只发布一次目标。如果 `move_base` / `amcl` / `map_server` 尚未启动，目标会发给空话题，表现为按钮点了但没有实际效果。

### 解决方法

- `cmd_loop()` 只在 `MANUAL_MODE=True` 时持续发布 `/cmd_vel`；
- `/api/goal` 自动切换到导航模式，清零手动速度，不再抢占 `move_base`；
- 如果导航节点未运行，`/api/goal` 会启动 `nav_clean navigation_runtime.launch` 并等待节点就绪；
- Goal 会重复发布 3 次，减少刚启动导航时目标丢失的概率；
- `/api/stop` 会发布 `/move_base/cancel`、切回手动模式，并连续发布零速度，保证紧急停止优先级高于导航；
- Map 页改为显示封门地图预览和导航节点状态，方便确认 `move_base` / `amcl` / `map_server` 是否真的启动。

### 以后如何避免/更好使用

网页导航模式下，任何持续发布 `/cmd_vel` 的逻辑都必须受 `MANUAL_MODE` 约束。否则会和 `move_base` 抢同一个底盘速度话题。

如果 goal 仍没反应，按顺序检查：

```bash
rostopic list | grep -E '/move_base_simple/goal|/move_base/status|/cmd_vel|/map|/amcl_pose'
rostopic echo -n 1 /move_base/status
rostopic hz /scan
rostopic hz /odom
```

页面 Map 状态行必须至少显示：

```text
move_base=true amcl=true map_server=true
```

---

## 32. Web 面板 `/api/status` 3 秒响应导致移动按钮和 Goal 卡顿

### 问题

浏览器 Network 中持续出现：

```text
status 200 fetch 约 3.0 秒
status fetch 挂起
```

同时前端移动按钮和 `GO` 目标点表现为点击后不及时响应。

### 原因

`/api/status` 每秒调用一次，但后端每次请求都会同步执行 `rosnode list | grep ...` 这类 shell/ROS 查询。ROS 查询慢时，请求持续 3 秒以上，浏览器和 Flask 线程会被堆积的 status 请求占住。

前端方向键也有一个问题：`startHold()` 开始移动前调用 `stopHold()`，而 `stopHold()` 会走重型 `/api/stop`，取消导航目标并连续发布零速度，按移动键第一下就可能被 STOP 抵消。

### 解决方法

- 后端增加 `nav_state_loop()` 后台线程，每 3 秒刷新一次导航节点状态；
- `/api/status` 只读取缓存，不再每次同步查 ROS；
- 新增轻量 `/api/manual_stop`，松开方向键只清零手动速度；
- `/api/cmd_vel` 立即切手动模式、取消导航目标并直接发布一次速度；
- `/api/goal` 改成 pub 思路：导航就绪时直接发布目标；导航未就绪时后台启动导航并缓存目标，不再阻塞 HTTP 请求；
- 前端 status 轮询增加 `statusBusy` 防重入，避免请求堆积。

### 验证结果

修复后实测：

```text
/api/status      约 44 ms
/api/cmd_vel     约 46 ms
/api/manual_stop 约 48 ms
/api/goal        约 199 ms
```

以后 Web 控制接口必须遵守：高频按钮接口只做内存赋值或 ROS publish，不要同步跑慢 shell、sleep、launch 或等待节点。

---

## 33. Web Goal 使用 odom 坐标会导致导航偏差、转圈或不到目标

### 问题

网页 `Status` 里早期显示的是 `/odom` 坐标，但 `GO` 发布到 `/move_base_simple/goal` 的 frame 是 `map`。

### 现象

- 点 `GO` 后 move_base 有反应；
- 小车会转圈、短距离前进，但不能稳定到达预期目标；
- 用户按页面显示的 X/Y 填目标时，目标实际不在 map 坐标系中的预期位置。

### 原因

`/odom` 是局部里程计坐标，`/move_base_simple/goal` 使用的是 `map` 坐标。两者不是同一个坐标系。实测同一时刻：

```text
odom: x=1.952, y=0.249
map/amcl_pose: x=0.482, y=0.102
```

如果把 odom 数值当 map goal 发出去，导航目标会偏移。

### 解决方法

- 后端订阅 `/amcl_pose`，`/api/status` 返回 `pose` 作为 map 坐标；
- 前端状态面板改为优先显示 `pose`；
- 新增 `/api/map_info` 返回地图 origin、resolution、width、height；
- Map 页面支持点击地图选点，自动换算为 ROS map 坐标并填入 X/Y；
- 点击地图时根据当前 AMCL 位姿自动计算 yaw，使目标朝向大致指向目标点；
- `Takeover` / `Resume Nav` 不再 kill/restart 导航节点，只切换控制状态或确保导航节点存在。

### 以后如何避免/更好使用

网页上所有导航目标都必须使用 `map` 坐标，不能用 `/odom` 坐标直接填 goal。

导航节点应该常驻。手动控制和导航切换应通过：

```text
手动控制：cancel 当前 move_base goal + 发布 /cmd_vel
导航控制：停止手动 /cmd_vel 循环 + 发布 /move_base_simple/goal
```

不要为了切换模式频繁 kill/restart `move_base`、`amcl` 或 `map_server`，否则定位和代价地图状态会被破坏。

---

## 34. “不用地图只靠雷达避障”的边界

### 问题

用户希望 Map 页面让小车使用雷达自己判断有没有障碍物，而不是只依赖给定地图。

### 结论

当前正确架构不是“静态地图”和“雷达”二选一，而是：

```text
静态封门地图：提供 map 坐标、AMCL 定位、全局路径规划
激光雷达 /scan：提供 local_costmap 动态障碍物标记与清除
```

也就是说，导航到指定目标点仍需要地图作为全局坐标基准；雷达负责识别临时障碍物、避障和清除局部代价地图。

### 当前验证

当前 `move_base` local costmap 已启用雷达障碍层：

```text
/move_base/local_costmap/obstacle_layer/enabled: true
/move_base/local_costmap/obstacle_layer/observation_sources: laser_scan_sensor
/move_base/local_costmap/obstacle_layer/laser_scan_sensor/topic: scan
/move_base/local_costmap/obstacle_layer/laser_scan_sensor/marking: true
/move_base/local_costmap/obstacle_layer/laser_scan_sensor/clearing: true
```

Web `/api/status` 已增加 `/scan` 状态，实测可收到：

```text
sample_count: 909
range_min: 0.1
range_max: 16.0
```

### 如果真的不使用静态地图

那就不是当前“点地图目标导航”模式，而是另一类功能：

- SLAM 建图中导航：边建图边更新 `/map`；
- 纯局部避障：只根据雷达前方障碍做短距离控制；
- 跟随墙/避障漫游：没有全局目标点，只做局部行为。

这些模式不能直接替代 `move_base + map + AMCL` 的全局目标导航。

---

## 35. Map 目标需要多次 Send Goal 才持续前进

### 问题

Map 页点击目标后，`Send Map Goal` 有时需要点多次。每点一次小车只移动一段距离，有时还会原地转圈。

### 原因

实测 `move_base/status` 可能提前进入：

```text
status: 3
text: "Goal reached."
```

但 AMCL 位姿和前端目标之间仍存在明显距离。也可能因为局部路径或朝向调整导致 move_base 停止推进，需要再次发布同一目标才能继续。

### 解决方法

Web 后端增加 goal watchdog：

- 保存最近一次 goal；
- 持续计算 AMCL `map` 位姿到目标点的距离；
- 如果当前不是手动模式、导航节点存在、目标距离仍大于 `0.12m`，但 move_base 已经不处于活动状态，就自动补发目标；
- `/api/status` 返回 `goal_distance` 和 `auto_reissue_count`，Map 页直接显示自动补发次数。

同时将 DWA 到达参数调为更适合实车：

```text
/move_base/DWAPlannerROS/xy_goal_tolerance = 0.08
/move_base/DWAPlannerROS/yaw_goal_tolerance = 0.35
/move_base/DWAPlannerROS/latch_xy_goal_tolerance = true
```

含义：位置更精确，最终朝向更宽松，减少到点附近反复原地转圈。

### 前端改动

Camera 已合并到 Map 页右侧，不再单独切换 Camera tab。

Map 状态行会显示：

```text
goal distance=...m, reissues=...
```

如果 `reissues` 增长，说明后端正在替你自动补发目标，不需要反复手点。

---

## 36. Web 面板显示小车电量

### 问题

小车没电时，Web 面板原来只能看到连接/导航状态，无法直接判断是否是电量导致底盘无响应。

### 解决方法

Web 后端订阅 ROS 标准话题：

```text
/battery_state
sensor_msgs/BatteryState
```

并在 `/api/status` 返回 `battery` 字段。前端顶部和 Status 卡片显示：

```text
Battery 50% · 12.34V
```

兼容两种常见百分比格式：

- ROS 标准 `0.0~1.0`，例如 `0.5` 显示为 `50%`；
- 部分 UCAR 镜像直接发布 `0~100`，例如 `50.0` 显示为 `50%`。

### 状态规则

- `low=true`：电量低于 20%，页面电量变红；
- `stale=true`：超过 10 秒没有收到新的 `/battery_state`，页面显示 `stale` 并变黄；
- 如果完全没有收到 `/battery_state`，页面显示 `Battery no data`。

注意：如果驱动发布的 `voltage` 为 `0.0`，前端会显示 `--V`，避免把无效电压误认为真实电压。

---

## 37. Map 目标点只能改方向和避障后不再移动

### 问题

Map 页第一次点击后，后续点击不能重新设置目标点，只会让目标箭头跟着鼠标改变方向。

另一个现象是：遇到障碍物触发停止后，再设置目标点小车不移动，状态里仍然认为前方 blocked。

### 原因

前端把“第二次点击”解释为设置已有目标的朝向，所以 `selectedGoal` 一旦存在，普通点击就不再更新 X/Y。

后端安全停止使用较宽前方扇区和单一阈值，容易把旁边障碍或刚经过的障碍继续当成正前方阻塞。

### 修复

Map 交互改为：

- 普通点击：永远设置新的目标点；
- `Shift + 点击` 或右键：设置当前目标点的朝向；
- 鼠标移动只有按住 `Shift` 时才预览目标朝向；
- 黄色点是实时 `/scan` 激光雷达命中点，不是导航路线。

避障安全停止改为窄前方扇区和滞回阈值：

```text
front half angle = 18 deg
stop distance = 0.40 m
clear distance = 0.65 m
```

含义：只有更接近正前方的障碍物才触发 Web 层急停；已经触发后，需要前方距离恢复到 `0.65m` 以上才解除 blocked，避免在阈值附近反复抖动。

---

## 38. 启动位置不能当作地图原点

### 问题

用户观察到小车在 Map 页导航时像是只能走半张地图，怀疑之前把启动位置当成了地图原点，导致地图相对位置错误。

### 结论

Web 当前地图坐标换算使用 `/map` 的真实 `OccupancyGrid.info.origin`，不是启动位置。

实测车端 `/api/map_info`：

```text
origin = [-10.435738, -11.50983, 0.0]
resolution = 0.05
width = 453
height = 430
```

地图有效坐标范围约为：

```text
X: -10.44 ~ 12.21
Y: -11.51 ~ 9.99
```

### 真正风险

旧前端有一个 `Set Origin` 按钮，调用 `/api/set_pose` 时固定发布：

```text
/initialpose
x = 0
y = 0
yaw = 0
```

这不会改变 `/map` 的 origin，但会强行告诉 AMCL “小车当前在 map 的 (0,0)”，如果小车真实位置不是 `(0,0)`，后续所有目标点都会相对错位。

### 修复

前端移除误导性的 `Set Origin` 行为，改为：

```text
Set AMCL Pose Here
```

使用方式：

1. 先在 Map 页点击小车真实所在位置；
2. 如需要，`Shift + 点击` 或右键设置真实朝向；
3. 点击 `Set AMCL Pose Here` 发布 `/initialpose`；
4. 等 AMCL 收敛后再设置导航 goal。

后端 `/api/set_pose` 也改为：

- 接收 `x/y/yaw`，不再固定 `(0,0,0)`；
- 发布前取消当前导航 goal；
- 切回手动模式并发布零速度，避免边导航边重定位。

### 注意

如果仍然只能走地图一部分，不一定是 origin 错，还可能是：

- 目标点落在未知区或障碍区，`move_base` 不会规划；
- 封门地图或 costmap 膨胀层把通道封窄；
- AMCL 粒子还没收敛，小车图标和真实位置不一致；
- 目标点超出可达自由空间，即使在图片上看起来有区域。
