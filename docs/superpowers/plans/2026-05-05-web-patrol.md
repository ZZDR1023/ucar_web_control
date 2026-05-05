# Web 巡逻功能实现计划

> **给执行代理的要求：** 实现本计划时必须使用 `superpowers:subagent-driven-development`（推荐）或 `superpowers:executing-plans`，并逐项执行下面的 checkbox 步骤。

**目标：** 在现有 UCAR Web 控制面板中实现地图点位巡逻、路线保存、巡逻控制和到点拍照。

**架构：** 新增 `web_panel/patrol.py` 作为纯 Python 巡逻状态与路线管理模块，方便单元测试。`web_panel/server.py` 负责把该模块接入 ROS goal、cancel、相机缓存和 Flask API；前端 `index.html` 负责点位候选、列表和控制按钮。

**技术栈：** Python 3、Flask、ROS 1 Melodic `rospy`、浏览器原生 HTML/CSS/JS、`unittest`。

---

### 任务 1：巡逻路线与状态核心模块

**文件：**
- 新建：`web_panel/patrol.py`
- 新建：`tests/test_patrol.py`

- [ ] **步骤 1：先写失败测试**

创建 `tests/test_patrol.py`，覆盖路线持久化、点位校验、启动/暂停/停止状态切换、安全阻塞处理和拍照事件。

运行：

```bash
python3 -m unittest tests.test_patrol -v
```

预期：失败，因为 `web_panel.patrol` 尚不存在。

- [ ] **步骤 2：编写最小实现**

创建 `web_panel/patrol.py`，包含：

- `PatrolPoint`
- `PatrolManager`
- `add_point()`
- `delete_point()`
- `save_route()`
- `load_route()`
- `start()`
- `pause()`
- `resume()`
- `stop()`
- `handle_blocked()`
- `mark_current_reached()`
- `snapshot()`

- [ ] **步骤 3：运行测试**

运行：

```bash
python3 -m unittest tests.test_patrol -v
```

预期：通过。

- [ ] **步骤 4：提交**

```bash
git add web_panel/patrol.py tests/test_patrol.py
git commit -m "feat: add patrol state manager"
```

### 任务 2：后端 Flask/ROS 接入

**文件：**
- 修改：`web_panel/server.py`
- 测试：`tests/test_patrol.py`

- [ ] **步骤 1：为后端需要的安全 helper 行为补失败测试**

扩展 `tests/test_patrol.py`，覆盖后端接入需要的行为：

- 停止巡逻会保留路线，但状态变为 `stopped`
- 阻塞状态必须显式 resume 才能继续
- 最后一个点完成后状态变为 `finished`

运行：

```bash
python3 -m unittest tests.test_patrol -v
```

预期：在 helper 行为补全前失败。

- [ ] **步骤 2：实现后端接入**

修改 `web_panel/server.py`：

- import `PatrolManager`
- create `patrol_manager`
- add `/api/patrol`
- add `/api/patrol/points`
- add `/api/patrol/points/<index>` DELETE
- add `/api/patrol/save`
- add `/api/patrol/start`
- add `/api/patrol/pause`
- add `/api/patrol/resume`
- add `/api/patrol/stop`
- include `patrol` in `/api/status`
- 增加巡逻后台线程：发布现有 map goal、监听 `goal_distance`、监听 `forward_obstacle.blocked`，并保存 `latest_camera_jpeg`

- [ ] **步骤 3：运行测试和语法检查**

运行：

```bash
python3 -m unittest tests.test_patrol -v
python3 -m py_compile web_panel/patrol.py web_panel/server.py
```

预期：测试通过，且没有语法错误。

- [ ] **步骤 4：提交**

```bash
git add web_panel/server.py web_panel/patrol.py tests/test_patrol.py
git commit -m "feat: add patrol web api"
```

### 任务 3：前端 Patrol 面板

**文件：**
- 修改：`web_panel/templates/index.html`

- [ ] **步骤 1：增加 UI 控件**

在 Map 控制附近增加 Patrol 卡片或区域：

- 点位名称输入框
- `Use Selected Goal`
- `Add Patrol Point`
- 路线表格或列表
- `Save Route`
- `Start Patrol`
- `Pause/Resume`
- `Stop Patrol`

- [ ] **步骤 2：增加前端状态和 API 调用**

增加 JavaScript 状态：

- `selectedPatrolCandidate`
- `patrolRoute`
- `patrolState`

增加函数：

- `useSelectedGoalForPatrol()`
- `addPatrolPoint()`
- `deletePatrolPoint(index)`
- `loadPatrol()`
- `savePatrol()`
- `startPatrol()`
- `pausePatrol()`
- `resumePatrol()`
- `stopPatrol()`
- `renderPatrol()`

更新地图绘制：巡逻候选点显示为蓝色，巡逻路线点显示为青色。

- [ ] **步骤 3：验证静态语法**

运行：

```bash
python3 -m py_compile web_panel/server.py web_panel/patrol.py
python3 -m unittest tests.test_patrol -v
```

预期：通过。

- [ ] **步骤 4：提交**

```bash
git add web_panel/templates/index.html web_panel/server.py web_panel/patrol.py tests/test_patrol.py
git commit -m "feat: add patrol controls to web panel"
```

### 任务 4：systemd / sensors / server 运维说明

**文件：**
- 修改：`bug_notes.md`
- 修改：`command_reference.md`
- 修改：`README.md`

- [ ] **步骤 1：记录当前 systemd 行为**

说明：

- `ucar_web.service` 已经解决手动启动 `python3 server.py` 的问题。
- `Address already in use` 表示 systemd 或另一个 Flask 进程已经占用 8080 端口。
- `ucar_web.service` 运行时不要再手动启动 `server.py`。

- [ ] **步骤 2：记录传感器启动缺口**

说明：

- Web 服务本身不会启动底盘和雷达。
- 当前仓库已有 `ucar_web.service`，但尚未提交 `ucar_sensors.service`。
- 如果不想手动启动 `start_sensors.sh`，需要新增并启用单独的 sensors systemd 服务，由它负责底盘和雷达启动。

- [ ] **步骤 3：验证文档**

运行：

```bash
rg -n "ucar_web.service|ucar_sensors.service|Address already in use|8080|start_sensors" bug_notes.md command_reference.md README.md
```

预期：文档中明确区分 Web 服务、传感器服务和端口冲突。

- [ ] **步骤 4：提交**

```bash
git add bug_notes.md command_reference.md README.md
git commit -m "docs: clarify web and sensor services"
```

### 任务 5：最终验证

**文件：**
- 现有代码和文档

- [ ] **步骤 1：运行自动验证**

```bash
python3 -m unittest tests.test_patrol -v
python3 -m py_compile web_panel/patrol.py web_panel/server.py
git status --short
```

预期：测试通过、语法检查通过；除明确保留的运行产物外，没有未提交改动。

- [ ] **步骤 2：报告实车手动验证步骤**

记录命令和检查项：

- 启动或检查 `ucar_web.service`
- 启动或检查 sensors
- 打开 Web 面板
- 使用“点击地图 -> Use Selected Goal -> Add”添加点位
- 启动巡逻
- 确认 `web_panel/patrol_captures/` 下生成照片
