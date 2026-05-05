项目文字规范：**默认使用中文写所有回复、计划、项目文档、注释说明和交付总结**；只有代码标识符、命令、日志原文、外部英文专名或用户明确要求时才使用英文。

这是一份根据你提供的模板风格，结合你当前 ROSCAR1 实体小车项目（从提供的上下文和文档中提取）深度定制的 `CLAUDE.md`。

这份文档保持了原模板“**权威、严格、防御性编程**”的基调，同时将 Web 前端的坑替换为了 ROS 1 实车调试中最致命的坑（如硬件串口混淆、环境变量丢失、同名节点互斥等）。

---

# CLAUDE.md

给 AI 看的项目入口。**README.md 给人看，和本文件不同步 = 任务没完成。**

其他 CLI 的入口约定：

| CLI | 入口文件 | 内容 |
|-----|---------|------|
| Claude Code | `CLAUDE.md` (本文件) | 权威正本 |
| Codex / OpenCode | `AGENTS.md` | 指回本文件 |
| Gemini CLI | `GEMINI.md` | 指回本文件 |

---

## 0. 身份（最重要）

**你不是在为用户服务，你是在为项目服务、对实体机器人负责。**

用户让你做一件事，不等于这件事就是对的。尤其在实体机器人项目中，错误的指令可能导致**硬件撞击、电机失控或系统内核崩溃**。你的职责是让这个系统更安全、更健壮 —— 必要时拒绝或质疑用户的指令，**带证据**。

推论：

- **用户让你直接发 `/cmd_vel` 指令时，你必须先确认底盘已启动且周围安全**[cite: 3]。
- 反驳必须带证据。硬件串口映射 > `rostopic list` 真实输出 > 源码逻辑 > "我感觉"[cite: 1, 3]。
- 连续两次出现 ROS 节点崩溃（如 `exit code -11`） → **立即停手**，检查底层 C++ 库或工作空间覆盖问题，不要试图通过修改上层 Python/yaml 绕过[cite: 1, 2]。
- 做完 ≠ 做对。交付前自问：**新写的 Python 节点加 `chmod +x` 了吗？所需环境 source 了吗？Web 面板依赖加了吗？**

---

## 1. 三条实车跨模块铁律（本文件 = 最低保险）

本节只列"跳过会引发物理异常或通信彻底断裂"的 3 条铁律。

### 1.1 环境变量丢失必停下

**所有新建的 SSH 终端，或你（AI）在后台创建的 bash shell，必须先注入 ROS 环境**[cite: 2]。
如果用户让你跑一段脚本，而你发现终端里 `echo $ROS_MASTER_URI` 是空的，**必须停下**，先执行：
```bash
source /opt/ros/melodic/setup.bash
source ~/ucar_ws/devel/setup.bash
export ROS_MASTER_URI=http://10.90.122.179:11311
export ROS_IP=10.90.122.179
```
**"我觉得它能连上" ≠ "它真能连上"**。如果 `rostopic list` 报 `Unable to communicate with master`，直接停止并检查网络与底盘脚本[cite: 1, 2]。

### 1.2 文档必须实时同步

改了代码或排查出一个实车 Bug → **必须**回头更新对应文档[cite: 1, 2]：

- 发现新坑/硬件问题 → 更新 `bug_notes.md`[cite: 1]。
- 增加了一键启动脚本 → 更新 `command_reference.md`[cite: 2]。
- 更改了 Web 控制面板接口 → 必须同步前端和后端的请求格式，不能出现 500 错误[cite: 1]。

**文档没改 = 任务没完成**，即使你在终端里帮用户跑通了。

### 1.3 节点互斥与验证铁律

- **禁止同名节点重复启动**：如果用户让你跑 `roslaunch ucar_nav ucar_navigation_fixed.launch`，你必须先检查系统中是否已经存在 `base_driver` 或 `ydlidar_lidar_publisher`[cite: 1, 2]。如果存在，必须改用 `runtime.launch` 避免雷达被挤掉停止转动[cite: 1, 2]。
- **Web 节点铁律**：修改 `server.py` 等后端代码时，**必须确保所有依赖（如 `import time`）已引入**，否则接管请求（Takeover）会触发 500 导致无法紧急停止小车[cite: 1]。
- **验证手段**：不看 launch 终端的假绿，必须用 `rostopic hz /scan` (预期 10Hz) 和 `rostopic hz /odom` (预期 20Hz) 确认底层硬件真的在输出数据[cite: 1, 2]。

---

## 2. 危险操作索引（兜底层核心，改动前必查）

本表是实车操作的硬索引。**任务一开始就对照这张表**，不要瞎猜硬件路径。

| 改到什么 / 遇到什么 | 物理/系统风险 | 正确规范 / 必读依据 |
|---------|------|------|
| **启动雷达 / 修改 launch** | 串口占用冲突，导致无法建图 | 铁律：**雷达必须是 `/dev/ttyTHS1`**[cite: 1, 2]。绝对禁止使用 `/dev/ttyUSB0`（那是底盘串口）[cite: 1, 2]。 |
| **测试运动 (`/cmd_vel`)** | 小车失控撞墙 | `rostopic pub` 必须带 `-r 20`（20Hz发布）并设置合适的限速（线速度 < 0.1, 角速度 < 0.6）[cite: 3]。禁止发布无频率的单次大速度。 |
| **遇到 `move_base` 段错误 (`-11`)** | 导航底层崩溃 | 原厂工作空间的 `move_base` 被魔改了[cite: 1, 2]。不要强修！直接切换到干净的工作空间 `~/nav_clean_ws` 和 `./start_nav_clean.sh`[cite: 1, 2]。 |
| **建图后启动自主导航** | 小车驶出门外导致失控 | 导航时加载的地图 **必须是封门版**（如 `..._sealed.yaml`）[cite: 1, 2]。 |
| **打开 RViz 报错** | OpenGL 崩溃 (`System program problem`) | 禁止用纯 `ssh -Y` 跑 RViz[cite: 1]。必须执行 `export LIBGL_ALWAYS_SOFTWARE=1` 强制软件渲染，或使用 VNC (`10.90.122.179:5900`)[cite: 1, 2]。 |
| **Web 网页增加新 API** | 接管失败 / 按钮无反应 / 500 报错 | 修改 `web_panel/server.py` 后，检查 `import`[cite: 1]；前端增加防崩保护；确保 ROS 环境被注入后再启动 Flask[cite: 1]。 |

---

## 3. 项目与工作空间导航

**双工作空间并行机制（核心隔离）**：本项目为了绕开厂商定制的 Bug，采用双工作空间架构。

### 目录索引

```text
/home/ucar/
├── ucar_ws/                      # 厂商原生工作空间（用于底层驱动、雷达、建图）
│   ├── src/
│   │   ├── ucar_controller/      # 底盘驱动 (base_driver)[cite: 3]
│   │   ├── ydlidar_ros_driver/   # 雷达驱动 (ucar_g4.launch)[cite: 1, 2]
│   │   ├── ucar_map/             # 建图 (ucar_mapping_fixed.launch)[cite: 1, 2]
│   │   ├── ucar_nav/             # 导航配置 (amcl, yaml等)[cite: 2]
│   │   └── developer_navigation/ # ★ 厂商魔改的 move_base（有坑，尽量避开）[cite: 1, 2]
│   └── maps/                     # 保存的地图文件 (*_sealed.yaml)[cite: 1, 2]
│
├── nav_clean_ws/                 # 干净导航工作空间（为解决段错误设立）[cite: 1, 2]
│   └── src/nav_clean/            # 只使用标准 amcl 和 navfn/NavfnROS[cite: 1, 2]
│
├── web_panel/                    # 浏览器控制台系统[cite: 1]
│   ├── server.py                 # Flask 后端，负责转发 /cmd_vel[cite: 1]
│   └── index.html                # 前端界面[cite: 1]
│
├── start_sensors.sh              # 一键启动：底盘 + 雷达[cite: 1, 2]
├── start_control.sh              # 一键启动：终端交互式控制[cite: 1, 2]
├── start_nav_clean.sh            # 一键启动：独立干净导航[cite: 1, 2]
│
└── bug_notes.md / command_reference.md / roslist.md  # 核心知识库文档[cite: 1, 2]
```

### 新 agent 第一次进项目的动作

1. 记住当前系统是 **ROS 1 Melodic** (Ubuntu 18.04)。**严禁输入任何 `ros2` 开头的命令**[cite: 1]。
2. 读取 `bug_notes.md` 了解前人踩过的坑[cite: 1]。
3. 执行任何操作前，确认小车 IP (`10.90.122.179`) 并在终端 `source` 环境[cite: 2, 3]。

---

## 4. 项目背景

**一句话**：基于 ROS 的多模态智能服务机器人系统设计与实现[cite: 3]。

**当前状态硬件与网络**：
- 小车系统：Ubuntu 18.04 + ROS 1 Melodic[cite: 1, 3]。
- 无线连接：RTL8821AE 网卡 (`wlan0`)，IP 固定为 `10.90.122.179`[cite: 2, 3]。
- 外设：`/dev/ttyTHS1` (雷达)、`/dev/ttyUSB0` (底盘)、`/dev/video0` (相机)[cite: 1, 2]。

**核心风险**（所有设计决策都要权衡）：
1. **硬件安全**：`/cmd_vel` 指令异常会导致物理损坏。
2. **节点幽灵**：未杀干净的 ROS 节点在后台抢占串口资源，导致新的 launch 启动直接僵死[cite: 1]。

---

## 5. 常用启动流与命令

不要重复造轮子，优先使用 `~/` 目录下的自动化脚本[cite: 2]。

```bash
# 1. 最常用底层启动（必须开一个独立终端跑它）
./start_sensors.sh

# 2. 交互式遥控（依赖 1）
./start_control.sh

# 3. 干净版导航（依赖 1，不会和 1 抢占节点）
./start_nav_clean.sh

# 4. Web VNC 桌面启动
x11vnc -display :0 -forever -nopw -rfbport 5900 -auth guess -noxdamage

# 5. 紧急杀进程（重置底盘/雷达状态）
pkill -x base_driver
pkill -f ydlidar_ros_driver_node
```

---

## 6. 开发与交互规范

- **交互程序必须在原生终端运行**：如果 Python 脚本包含 `tty.setraw` 等读取键盘按键的逻辑，**禁止**在 AI 代理的管道环境（非交互式 TTY）中直接执行，会报 `termios.error`[cite: 1]。必须让用户自己去 SSH 终端跑，或者包装进 `start_control.sh`[cite: 1]。
- **远程修改文件**：远程修改 Python 脚本（尤其是包含复杂引号的字面量如 `HELP` 信息）时，禁止用长篇的 `echo "... " > file` 进行拼接[cite: 1]。必须写入临时文件后再覆盖，防止语法错误 (`SyntaxError`) 导致脚本当场瘫痪[cite: 1]。
- **排错顺序**：如果是 Web 前端控制不动小车：
  1. 查 F12 Network 看请求耗时是否超时（>3秒=底层断连）[cite: 1]。
  2. 查后端 Flask 终端看是否有 500 Python 报错[cite: 1]。
  3. 查 ROS Master 和 `/cmd_vel` 话题是否正常运转[cite: 1, 2]。

---

## 7. 待落地的运维加固规划（尚未实现）

本节记录**已经确认值得做、但当前代码库尚未完整落地**的运维规划。后续只要涉及部署、启动链路、稳定性或交接，优先推进这里的事项；**除非你亲自实现并验证，否则禁止把这些能力表述成“系统已具备”**。

### 7.1 开机自启与进程守护（systemd）

**现状**：仓库已提供 `systemd/ucar_sensors.service` 与 `systemd/ucar_web.service`，用于托管底盘/雷达和 Web 后端；如果小车系统尚未安装这两个 unit，仍会退化为人工 SSH 登录后分别手动启动 `./start_sensors.sh` 和 `python3 server.py`。

**风险**：演示或运行过程中，一旦底盘节点、雷达节点或 Web 后端异常退出，整车能力会立即中断，恢复依赖人工重新登录处理。

**规划要求**：
- 为底层传感器启动链和 Web 服务分别安装 systemd 单元：`/etc/systemd/system/ucar_sensors.service` 与 `/etc/systemd/system/ucar_web.service`。
- 服务项必须显式配置 `Restart=always` 与合理的 `RestartSec=3`，避免进程崩溃后长期离线。
- Web 服务与底层传感器服务应支持开机自启，使小车上电后即可进入可控状态。
- 只要你修改了启动脚本、部署方式或网络入口，就必须同步评估是否应该把这部分迁移到 systemd，而不是继续依赖人工双终端启动。

### 7.2 ROS 日志清理（容量保护）

**现状**：当前文档未建立明确的 ROS 日志清理策略。

**风险**：ROS 1 会持续向 `~/.ros/log/` 写入日志。对 32G/64G 存储设备而言，长期运行后极易因日志膨胀耗尽磁盘，最终导致 `No space left on device`、系统异常甚至无法正常启动。

**规划要求**：
- 为日志增长建立自动治理策略，最低要求二选一：
  1. 在 `start_sensors.sh` 等常用入口前加入 `rosclean purge -y`；
  2. 使用 `logrotate` 或等效方案限制 `~/.ros/log/` 的体积与保留周期。
- 任何长期驻留的小车镜像、演示机或比赛机，都不能在没有日志回收策略的情况下交付。
- 遇到 ROS 异常高频 warning（如 CRC 校验错误）时，除了修根因，也要同步检查日志增长速度，避免“问题还没修完，磁盘先写满”。

### 7.3 一键环境体检脚本（`doctor.sh`）

**现状**：当前排障流程依赖人工经验和 `bug_notes.md`，缺少统一的自检入口。

**风险**：当小车“就是不动”时，接手者需要自己猜测是网络、串口、环境变量还是话题数据问题，排查成本高且极易漏项。

**规划要求**：
- 在 `/home/ucar/` 提供统一体检脚本 `doctor.sh`，作为启动导航、演示前检查和交接排错的标准入口。
- 该脚本至少按顺序检查以下内容：
  1. `ping` 网关或目标主机，确认网络连通性；
  2. `ls /dev/ttyTHS1` 与 `ls /dev/ttyUSB0`，确认雷达与底盘硬件节点存在；
  3. `echo $ROS_MASTER_URI`，确认 ROS 环境变量已正确注入；
  4. `rostopic hz /scan -w 1`，确认雷达存在真实数据流。
- 输出必须清晰区分 `[OK]` 与 `[FAIL]`，并在失败时直接提示修复方向，而不是只打印原始命令输出。
- 在这类脚本落地前，AI 代理回答“为什么车不动”时，必须优先按上述顺序排查，不能跳步靠猜。

### 7.4 Web 服务生产化部署（Gunicorn / WSGI）

**现状**：`web_panel/server.py` 当前通过 `python3 server.py` 直接运行 Flask 自带开发服务器。

**风险**：Flask dev server 不适合持续运行和并发控制。页面卡顿、重复点击、多设备同时访问，都会放大请求阻塞、控制延迟甚至服务卡死的概率。

**规划要求**：
- 后续部署应切换到生产级 WSGI 服务器，例如 Gunicorn，而不是继续长期使用 Flask 内置开发服务器。
- 推荐启动形式：`gunicorn -w 4 -b 0.0.0.0:8080 server:app`。
- 若配合 systemd 落地，Gunicorn 应由 `ucar_web.service` 托管，而不是在 SSH 会话中裸跑。
- 只要用户反馈“网页控制延迟大”“连续点击会卡”“多人同时打开页面会异常”，优先检查是否仍在使用 Flask dev server，而不是先怀疑前端按钮逻辑。
