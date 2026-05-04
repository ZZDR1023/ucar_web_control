# ROSCAR1 对话历史与小车状态记录

记录日期：2026-04-30

## 1. 初始网络与登录信息

用户使用网线直连一台科大讯飞实体小车。

本机网络环境：

- 本机通过无线网卡联网。
- 本机有线网卡 `enp4s0` 开启网络共享。
- 小车通过有线连接后扫描到的 IP：`10.42.0.159`。
- 小车 SSH 用户名：`ucar`。
- 小车 SSH 密码：`ucar`。

通过 Python `pexpect` 自动输入密码后，确认可以 SSH 登录小车。

## 2. ROS 版本确认

用户最初要求执行：

```bash
ros2 topic list
```

检查结果：

```text
bash: ros2: command not found
```

进一步检查 `/opt/ros`：

```text
/opt/ros/melodic
```

结论：

- 小车当前不是 ROS 2 环境。
- 小车安装的是 ROS 1 Melodic。
- 应使用 ROS 1 命令，例如 `rostopic list`、`roslaunch`。

## 3. ROS Master 与底盘启动

初次执行：

```bash
source /opt/ros/melodic/setup.bash
source ~/ucar_ws/devel/setup.bash
rostopic list
```

结果：

```text
ERROR: Unable to communicate with master!
```

检查发现：

- `ROS_MASTER_URI=http://localhost:11311`
- 没有 `roscore` / `rosmaster` / `roslaunch` 进程。
- `11311` 端口没有监听。

随后查找小车工作空间 launch 文件，发现关键启动入口：

```text
/home/ucar/ucar_ws/src/ucar_controller/launch/base_driver.launch
```

启动命令：

```bash
source /opt/ros/melodic/setup.bash
source ~/ucar_ws/devel/setup.bash
roslaunch ucar_controller base_driver.launch
```

通过后台方式启动后，ROS Master 正常运行：

```text
roslaunch ucar_controller base_driver.launch
rosmaster --core -p 11311
```

`rostopic list` 输出：

```text
/battery_state
/cmd_vel
/imu
/joy
/mag_pose_2d
/odom
/rosout
/rosout_agg
/tf
```

结论：底盘 ROS 1 通信启动成功。

## 4. 无线网卡确认

用户希望不使用网线连接电脑和小车。

在小车上检查无线网卡：

```bash
ip -br addr
nmcli device status
iw dev
rfkill list
lspci | grep -Ei 'wireless|wifi|802.11|network'
```

结果：

```text
wlan0            DOWN
wlan0    wifi    disconnected
```

无线硬件：

```text
Realtek RTL8821AE 802.11ac PCIe Wireless Network Adapter
```

无线未被禁用：

```text
Soft blocked: no
Hard blocked: no
```

结论：小车有无线网卡，接口名为 `wlan0`。

## 5. 连接手机热点

用户提供手机热点：

- 热点名称：`iQ`
- 密码：`louloulou`

第一次使用普通用户执行 `nmcli` 失败：

```text
Not authorized to control networking.
```

随后使用 `sudo` 执行：

```bash
sudo nmcli radio wifi on
sudo nmcli dev wifi rescan ifname wlan0
sudo nmcli dev wifi connect 'iQ' password 'louloulou' ifname wlan0
```

连接成功：

```text
wlan0    wifi    connected    iQ
```

小车无线 IP：

```text
10.235.133.179
```

本机连接同一手机热点后，测试：

```bash
ping -c 3 -W 2 10.235.133.179
```

结果：

```text
3 packets transmitted, 3 received, 0% packet loss
```

并通过无线 SSH 登录成功：

```bash
ssh ucar@10.235.133.179
```

## 6. 拔掉网线后的 ROS 重启

拔掉网线后，原来通过有线环境启动的 ROS 底盘进程退出，`eth0` 变为 `DOWN`。

通过无线重新启动底盘时，显式设置：

```bash
export ROS_MASTER_URI=http://10.235.133.179:11311
export ROS_IP=10.235.133.179
```

后台启动：

```bash
source /opt/ros/melodic/setup.bash
source ~/ucar_ws/devel/setup.bash
export ROS_MASTER_URI=http://10.235.133.179:11311
export ROS_IP=10.235.133.179
roslaunch ucar_controller base_driver.launch
```

启动后检查：

```text
rosmaster 监听 0.0.0.0:11311
```

话题列表正常：

```text
/battery_state
/cmd_vel
/imu
/joy
/mag_pose_2d
/odom
/rosout
/rosout_agg
/tf
```

`/cmd_vel` 信息：

```text
Type: geometry_msgs/Twist

Subscribers:
 * /base_driver (http://10.235.133.179:42827/)
```

结论：无线环境下可以通过 `/cmd_vel` 控制底盘。

## 7. 控制测试

第一次执行低速前进测试：

```bash
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist "linear:
  x: 0.08
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

持续约 0.3 秒后发送停止命令：

```bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

用户反馈最初没有明显移动。

随后检查：

```bash
rostopic info /cmd_vel
rostopic hz /odom
rostopic echo -n 1 /odom
rostopic echo -n 1 /battery_state
rostopic hz /imu
```

结果：

- `/cmd_vel` 有 `/base_driver` 订阅者。
- `/odom` 约 20Hz。
- `/imu` 约 50Hz。
- `/battery_state` 有数据，但电压显示为 `0.0`、percentage 为 `50.0`。
- 底盘日志出现一次：

```text
check crc16 faild(ahrs)
```

后来使用 ROS Python 发布器连续发布旋转命令：

```python
msg.angular.z = 0.6
# 20Hz，持续约 1.5 秒
```

`rostopic echo /cmd_vel` 能看到：

```text
angular:
  z: 0.6
```

用户最终反馈：

```text
小车确实转动了
```

结论：

- 无线 SSH 可用。
- ROS Master 可用。
- `/cmd_vel` 控制链路可用。
- 小车能够响应控制命令。

## 8. 官方 SDK / 文档优先偏好

用户明确要求：

```text
你以后执行前可以查看官方SDK
```

已记录为长期偏好：

- 以后控制实体机器人前，先查看官方 SDK、README、示例脚本、launch 文件和参数说明。
- 优先使用官方推荐的话题、命令频率和安全参数。
- 执行动作前先说明即将执行的控制命令。

本次已查看的官方/本地文档与源码包括：

```text
~/ucar_ws/src/ucar_controller/launch/base_driver.launch
~/ucar_ws/src/ucar_controller/readme.md
~/ucar_ws/src/ucar_controller/scripts/linear_check.py
~/ucar_ws/src/ucar_controller/scripts/angular_check.py
~/ucar_ws/src/ucar_controller/config/driver_params_mini.yaml
~/ucar_ws/src/ucar_controller/src/base_driver.cpp
```

`ucar_controller/readme.md` 中说明：

- `base_driver` 订阅 `/cmd_vel`。
- 消息类型为 `geometry_msgs/Twist`。
- 推荐示例使用 `rostopic pub -r 20 /cmd_vel ...`。
- 示例中 `angular.z = 1.0` 会让小车持续逆时针自转。

## 9. 小车已有 ROS 包与项目方向

已发现工作空间目录：

```text
~/ucar_ws/src/ucar_controller
~/ucar_ws/src/ucar_map
~/ucar_ws/src/ucar_nav
~/ucar_ws/src/person_follow
~/ucar_ws/src/ucar_camera
~/ucar_ws/src/xf_mic_asr_offline
~/ucar_ws/src/ydlidar_ros_driver
~/ucar_ws/src/developer_navigation
~/ucar_ws/src/road_following
~/ucar_ws/src/teleop_twist_keyboard
```

已发现功能方向：

- 底盘控制：`ucar_controller`
- 激光雷达：`ydlidar_ros_driver`
- 建图：`ucar_map`、`cartographer_start.launch`、`ucar_mapping.launch`
- 导航：`ucar_nav`、`ucar_navigation.launch`
- 人体跟随：`person_follow`
- 摄像头：`ucar_camera`
- 离线语音识别：`xf_mic_asr_offline`
- 道路跟随：`road_following`
- 键盘控制：`teleop_twist_keyboard`

推荐项目主题：

```text
基于 ROS 的多模态智能服务机器人系统设计与实现
```

建议分阶段：

1. 无线遥控小车。
2. 状态监控面板。
3. 激光雷达建图。
4. 自主导航。
5. 语音控制。
6. 人体跟随。
7. 语音 + 导航 + 跟随综合系统。

## 10. 常用命令备忘

无线 SSH：

```bash
ssh ucar@10.235.133.179
```

加载 ROS 环境：

```bash
source /opt/ros/melodic/setup.bash
source ~/ucar_ws/devel/setup.bash
export ROS_MASTER_URI=http://10.235.133.179:11311
export ROS_IP=10.235.133.179
```

启动底盘：

```bash
roslaunch ucar_controller base_driver.launch
```

查看话题：

```bash
rostopic list
rostopic info /cmd_vel
rostopic hz /odom
rostopic echo -n 1 /odom
```

停止小车：

```bash
rostopic pub -1 /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

低速旋转测试，执行前需确认周围安全：

```bash
rostopic pub -r 20 /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.6"
```

低速前进测试，执行前需确认周围安全：

```bash
rostopic pub -r 20 /cmd_vel geometry_msgs/Twist "linear:
  x: 0.08
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```
