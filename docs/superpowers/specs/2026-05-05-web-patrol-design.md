# Web 巡逻功能设计

## 目标

为 UCAR 小车 Web 控制面板增加巡逻功能。第一版支持操作员从地图上添加巡逻点、保存路线、启动/暂停/停止巡逻，并在每个巡逻点到达后保存一张相机照片。

## 范围

本功能扩展现有 `web_panel/server.py` 和 `web_panel/templates/index.html`。实现时复用当前导航目标发布链路、AMCL 地图坐标、Web 安全急停逻辑、相机流和 `/api/status` 轮询。

第一版包含：

- 从已选地图位置添加巡逻点，必须按显式 Add 按钮后才加入列表。
- 显示巡逻点列表，包括名称、地图坐标、朝向、当前进度和删除控制。
- 在小车主机上保存和加载巡逻路线。
- 从 Web 面板启动、暂停、恢复和停止巡逻。
- 按顺序导航到每个巡逻点。
- 每到达一个点后短暂停留，并保存一张相机照片。
- 当 Web 安全层检测到阻塞障碍时，暂停巡逻进度。

第一版不包含：

- 视觉缺陷识别。
- 语音交互。
- 自动 PDF/HTML 报告。
- 多路线调度。
- 拖拽排序巡逻点。
- 自动发现巡逻点。

## 现有上下文

当前 Web 面板已有：

- 使用 `selectedGoal` 的地图点击导航。
- `/api/goal`，用于在 `map` 坐标系发布 `PoseStamped` 目标。
- `/api/stop`、`/api/takeover`、`/api/resume_nav`。
- `/api/status` 中的 AMCL 位姿。
- `goal_distance` 和 `move_base_status`。
- `forward_obstacle` Web 安全状态。
- MJPEG 相机流和后端缓存的最新 JPEG 帧。

巡逻功能不能破坏现有单点导航。特别是：添加巡逻点本身不得发布导航目标。

## 交互设计

Map 卡片在现有地图工具下方新增 Patrol 区域。

控件：

- `Patrol Point Name` 文本输入框。
- `Use Selected Goal` 按钮。
- `Add Patrol Point` 按钮。
- `Save Route` 按钮。
- `Start Patrol` 按钮。
- `Pause Patrol` / `Resume Patrol` 按钮。
- `Stop Patrol` 按钮。

地图交互：

- 现有地图点击行为保持不变：普通点击仍然只选择单次导航目标。
- 按 `Use Selected Goal` 后，才把当前 `selectedGoal` 复制到 `selectedPatrolCandidate`。
- `selectedPatrolCandidate` 在地图上显示为蓝色候选点。
- 按 `Add Patrol Point` 后，才把候选点追加到巡逻列表。
- 单纯点击地图不会添加巡逻点。
- Add 操作不会发送导航命令。

用户指定的流程保持显式：

1. 点击地图。
2. 确认或编辑巡逻点名称。
3. 点击 Add，把该点加入巡逻列表。

## 后端 API

新增接口：

- `GET /api/patrol`
  返回路线和执行状态。

- `POST /api/patrol/points`
  请求体：`{"name": "door", "x": 1.2, "y": 0.4, "yaw": 0.0}`
  向内存路线添加一个点，并持久化路线。

- `DELETE /api/patrol/points/<index>`
  删除一个点，并持久化路线。

- `POST /api/patrol/save`
  保存当前路线。

- `POST /api/patrol/start`
  从第一个点开始执行；也可支持可选 `start_index`。

- `POST /api/patrol/pause`
  暂停巡逻。取消当前导航目标，但不清空路线。

- `POST /api/patrol/resume`
  从当前点恢复巡逻。

- `POST /api/patrol/stop`
  停止巡逻，取消当前导航目标，并切回手动模式。

`/api/status` 额外返回简化的 `patrol` 字段，让现有轮询逻辑能更新 UI，不额外增加高频请求。

## 数据模型

路线文件：

`web_panel/patrol_route.json`

格式：

```json
{
  "points": [
    {"name": "door", "x": 0.5, "y": 0.0, "yaw": 0.0},
    {"name": "desk", "x": 1.2, "y": -0.4, "yaw": 1.57}
  ]
}
```

运行时状态：

```python
patrol_state = {
    "mode": "idle",
    "current_index": None,
    "current_point": None,
    "last_event": "not started",
    "captures": [],
}
```

合法状态：

- `idle`
- `running`
- `paused`
- `blocked`
- `finished`
- `stopped`
- `error`

## 执行流程

启动巡逻：

1. 校验路线至少包含一个点。
2. 设置 `MANUAL_MODE = False`。
3. 如果导航节点未运行，使用现有 `start_nav_async()` 链路确保导航启动。
4. 使用与 `/api/goal` 相同的目标创建和发布 helper 发布第一个巡逻点。
5. 设置 `current_index = 0`，`mode = running`。

巡逻后台循环：

1. 如果状态是 `paused`，睡眠等待，不改变当前点。
2. 如果 `forward_obstacle.blocked` 为 true，设置状态为 `blocked`，取消当前目标并等待。
3. 如果阻塞解除，保持 `paused`；必须由操作员点击 Resume 后才继续运动。
4. 如果当前点已到达，发布零速度，保存一张相机照片，等待 3 秒，再进入下一个点。
5. 如果最后一个点已到达，设置状态为 `finished`，取消当前目标，并切回手动模式。

到点判定：

- 优先使用 `goal_distance <= 0.15`。
- 也接受 `move_base_status.code == 3`，但要求小车与当前点距离仍在合理范围内。

暂停：

- 设置状态为 `paused`。
- 发布 `/move_base/cancel`。
- 发布零速度。
- 不清空 `current_index`。

停止：

- 设置状态为 `stopped`。
- 发布 `/move_base/cancel`。
- 连续发布几次零速度。
- 切回手动模式。
- 保留巡逻路线列表。

## 拍照

照片目录：

`web_panel/patrol_captures/`

文件名格式：

`YYYYmmdd_HHMMSS_<index>_<safe_name>.jpg`

拍照使用 `latest_camera_jpeg`。如果当前没有相机帧，后端在 `last_event` 记录拍照失败，但继续执行下一个点。

## 安全规则

巡逻功能必须尊重现有 Web 安全层。

- 除零速度停止命令外，不直接发布运动 `/cmd_vel`。
- 只使用普通导航目标，让 `move_base` 和 costmap 负责路径规划。
- 巡逻期间如果 `forward_obstacle.blocked` 变为 true，巡逻暂停并取消当前目标。
- 阻塞解除后不能自动恢复运动，必须由操作员显式点击 Resume。
- `Stop Patrol` 和现有 `STOP` 都必须优先于巡逻执行。

## 测试策略

对巡逻状态和路线行为做单元测试；实际 ROS 发布器尽量通过小 helper 隔离，便于测试。

测试用例：

- 添加点会持久化路线，并且不改变手动/导航模式。
- 删除点会持久化路线。
- 空路线启动巡逻会返回错误，且不会启动导航。
- 有路线时启动巡逻会进入 `running` 并发布第一个目标。
- 暂停会取消当前目标并保留 `current_index`。
- 停止会取消当前目标、切回手动模式，并保留路线。
- 安全阻塞会把巡逻切到 `blocked`，并要求显式恢复。
- 到达一个点后会进入下一个点，并记录拍照事件。

实车手动验证：

1. 启动传感器和 Web 服务。
2. 确认 `/scan`、`/odom`、`/amcl_pose` 和相机可用。
3. 点击地图，按 `Use Selected Goal`，输入名称，按 Add。
4. 至少添加两个巡逻点。
5. 点击 `Start Patrol`。
6. 确认小车按顺序驶向每个点。
7. 在小车前方放置障碍物，确认巡逻暂停。
8. 清除障碍物，点击 Resume，确认巡逻继续。
9. 确认 `web_panel/patrol_captures/` 下生成照片。

## 文档更新

实现完成后：

- 更新 `README.md`，说明 Web 巡逻用法。
- 如果新增运维命令，更新 `command_reference.md`。
- 如果实车验证发现新问题，更新 `bug_notes.md`。
