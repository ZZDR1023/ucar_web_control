# 视觉巡线设计说明

## 目标

在现有 Web 控制面板中加入视觉巡线能力，让小车通过摄像头识别地面路线，并在确认识别稳定后低速道路跟随。

第一版以安全和可调试为主：默认只做识别预览，不主动控制底盘。用户必须在 Web 页面明确开启“巡线控制”后，后端才会以低速写入现有 `/cmd_vel` 发布循环。

## 范围

本次实现包含：

- 从现有摄像头缓存读取最新 JPEG 帧。
- OpenCV 识别黑线、白线、黄线三类地面路线。
- 输出线中心偏差、像素中心、检测面积、是否丢线、建议线速度和角速度。
- Web 页面显示识别叠加图、状态和参数；默认颜色为黄线，默认 ROI 为画面高度的 `0.45-0.95`，便于先用当前道路地垫的黄色边线验证。黑线支持 `black_threshold` 参数，用于处理反光后变成深灰的黑色路径。
- 提供巡线参数接口：颜色、ROI 起止比例、线速度、角速度增益、是否启用控制。
- 保留雷达安全停止：前方障碍触发时，巡线控制立即停下并退出使能。

本次不做：

- 路口规则、斑马线语义、红绿灯识别。
- 红/黄实线不可压、虚线允许跨越等道路语义约束。
- 自动选择路线颜色。
- 云台控制。当前硬件只发现普通 USB 摄像头，没有 pan/tilt/servo 控制接口。
- 高速闭环行驶。

## 架构

新增 `web_panel/line_follow.py`，只负责纯视觉算法和参数校验，不依赖 ROS。它接收 BGR 图像，按 ROI 裁剪后根据颜色生成 mask，取最大轮廓计算中心偏差，再生成一张叠加调试图。

`web_panel/server.py` 负责把算法接入现有相机缓存和 ROS 控制状态。巡线控制不新建独立 ROS 节点，避免多个发布者抢 `/cmd_vel`；启用时直接写入 `CURRENT_LINEAR_X` 和 `CURRENT_ANGULAR_Z`，由现有 `cmd_loop()` 继续 20Hz 发布。

前端在 Camera 卡片下新增“视觉巡线”区域，显示叠加图、识别状态和参数控件。用户可以先只看预览，再手动开启控制。

## 安全策略

- 默认 `enabled=false`。
- 最大线速度限制为 `0.10 m/s`。
- 最大角速度限制为 `0.60 rad/s`。
- 丢线时线速度归零，只允许小角速度搜索或直接停止；第一版选择直接停止。
- `latest_forward_obstacle.blocked=true` 时强制关闭巡线控制、清零命令并发布停止。
- 用户点击 STOP、手动方向键、发送导航目标、巡逻启动时会覆盖巡线控制。

## 数据流

1. `camera_loop()` 更新 `latest_camera_jpeg`。
2. `/api/line_follow/status` 解码最新 JPEG，调用 `LineFollower.process()`。
3. 返回 JSON：参数、结果、控制状态和安全状态。
4. `/api/line_follow/debug.jpg` 返回叠加后的 JPEG。
5. `/api/line_follow/config` 保存参数；如果启用控制，后台 `line_follow_loop()` 周期性根据最新帧更新 `/cmd_vel` 状态。

## 测试

核心算法用 `unittest` 构造合成图像：

- 黑线在画面中心时，检测成功且偏差接近 0。
- 黑线在左侧时，偏差为负。
- 黄线可被 HSV 阈值识别。
- 空白图像返回丢线状态和停止命令。
- 参数校验会限制线速度和角速度范围。

集成验证：

- `python3 -m unittest tests.test_line_follow -v`
- `python3 -m unittest tests.test_patrol -v`
- `python3 -m py_compile web_panel/line_follow.py web_panel/server.py`
- 小车端重启 `ucar_web.service` 后访问 `http://10.90.122.179:8080`，先看巡线预览，不立即启用控制。
