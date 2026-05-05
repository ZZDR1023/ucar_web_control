# 视觉巡线 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 Web 控制面板中加入基于摄像头的视觉巡线预览和低速安全控制。

**Architecture:** `web_panel/line_follow.py` 封装纯 OpenCV 识别逻辑，`web_panel/server.py` 负责读取相机缓存、管理巡线控制状态并接入 `/cmd_vel`，`web_panel/templates/index.html` 提供中文参数面板和调试图。控制默认关闭，只有用户启用后才写入现有手动控制发布循环。

**Tech Stack:** Python 3、OpenCV、NumPy、Flask、ROS 1 Melodic `rospy`、原生 HTML/CSS/JS、`unittest`。

---

## 文件边界

- 新建 `web_panel/line_follow.py`：巡线参数、检测结果、图像处理和速度建议。
- 新建 `tests/test_line_follow.py`：合成图像算法测试。
- 修改 `web_panel/server.py`：巡线状态、API、后台控制循环、安全停止联动。
- 修改 `web_panel/templates/index.html`：视觉巡线面板、参数控件、调试图刷新。
- 修改 `bug_notes.md` 和 `todo.md`：记录视觉巡线使用限制和下一步。

## Task 1: 核心算法

- [ ] 写失败测试：中心黑线、左侧黑线、黄线、空白丢线、参数限幅。
- [ ] 运行 `python3 -m unittest tests.test_line_follow -v`，确认因 `web_panel.line_follow` 不存在失败。
- [ ] 新建 `web_panel/line_follow.py`，实现 `LineFollowConfig`、`LineFollower`、`LineFollowResult`。
- [ ] 再次运行 `python3 -m unittest tests.test_line_follow -v`，确认通过。

## Task 2: 后端 API 和控制循环

- [ ] 修改 `web_panel/server.py`，导入巡线模块并新增全局状态。
- [ ] 新增 `GET /api/line_follow/status`、`GET /api/line_follow/debug.jpg`、`POST /api/line_follow/config`、`POST /api/line_follow/stop`。
- [ ] 新增 `line_follow_loop()`：只有 `enabled=true` 且无前方障碍时写入 `MANUAL_MODE=True` 与低速命令。
- [ ] 在 `/api/cmd_vel`、`/api/stop`、`/api/goal`、巡逻启动等路径关闭巡线使能，避免控制逻辑冲突。

## Task 3: 前端面板

- [ ] 在 Camera 卡片下增加“视觉巡线”面板。
- [ ] 增加颜色选择、ROI、速度、转向增益、启用控制和停止按钮。
- [ ] 轮询 `/api/line_follow/status` 并刷新 `/api/line_follow/debug.jpg`。
- [ ] 所有可见文字使用中文。

## Task 4: 文档与验证

- [ ] 更新 `bug_notes.md`：记录巡线摄像头角度、默认不控车、雷达障碍停走限制。
- [ ] 更新 `todo.md`：标记视觉巡线第一版已开始落地，后续可做路口和语义识别。
- [ ] 运行 `python3 -m unittest tests.test_line_follow tests.test_patrol -v`。
- [ ] 运行 `python3 -m py_compile web_panel/line_follow.py web_panel/server.py`。
- [ ] 部署到小车并重启 `ucar_web.service`。
