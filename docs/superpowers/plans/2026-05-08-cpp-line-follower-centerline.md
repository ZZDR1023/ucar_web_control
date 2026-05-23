# C++ Line Follower Centerline Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the C++ line follower keep the robot centered in the black drivable area, reduce yellow-line contact risk, and handle dashed black-line gaps better.

**Architecture:** Keep the existing single ROS node and add conservative vision/control logic inside `src/line_follower.cpp`. Use a larger ROI, vertical morphological closing, layered center extraction, yellow guard logic, and turn-based speed reduction.

**Tech Stack:** ROS 1 Melodic, catkin, C++11, OpenCV 3.2 via cv_bridge.

---

### Task 1: Static Regression Coverage

**Files:**
- Modify: `tests/test_cpp_line_follower_file.py`

- [ ] **Step 1: Write tests for new C++ safety features**

Add assertions that `src/line_follower.cpp` contains tunable ROI, vertical close kernel, layered center tracking, yellow guard, and adaptive speed terms:

```python
    def test_cpp_file_has_centerline_dash_and_yellow_guard_parameters(self):
        text = CPP_FILE.read_text(encoding="utf-8")

        for token in (
            "roi_top_ratio_",
            "close_kernel_width_",
            "close_kernel_height_",
            "computeLayeredCenter",
            "detectYellowGuard",
            "turn_slowdown_",
            "min_curve_speed_",
            "yellow_guard_enabled_",
            "yellow_h_min_",
            "yellow_h_max_",
        ):
            self.assertIn(token, text)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m unittest tests.test_cpp_line_follower_file`

Expected: `FAIL` because these tokens do not exist yet.

### Task 2: C++ Vision And Control Update

**Files:**
- Modify: `src/line_follower.cpp`
- Modify: `tests/line_follower.cpp`

- [ ] **Step 1: Add parameters and helpers**

Add private fields for ROI ratio, close kernel width/height, layered center settings, yellow HSV guard, and speed reduction.

- [ ] **Step 2: Update `followLine`**

Use `roi_top_ratio_` instead of fixed `height * 2 / 3`; use `close_kernel_width_ x close_kernel_height_`; compute target center using layered bands; detect yellow contact in the lower center guard area; reduce speed as turn rate rises; stop forward motion when yellow contact is detected.

- [ ] **Step 3: Mirror source to test fixture**

Copy the updated production source to `tests/line_follower.cpp`, because existing static tests still use it as a reference fixture for legacy checks.

### Task 3: Documentation And Verification

**Files:**
- Modify: `command_reference.md`
- Modify: `bug_notes.md`

- [ ] **Step 1: Document the new parameters**

Update the C++ line follower command with conservative defaults:

```bash
rosrun roscar1 line_follower_node _image_topic:=/camera/image_raw _max_linear_speed:=0.04 _max_angular_speed:=0.22 _Kp:=0.0025 _Kd:=0.001 _line_threshold:=120 _branch_choice:=left _max_lost_frames:=8 _roi_top_ratio:=0.50 _close_kernel_width:=13 _close_kernel_height:=61 _turn_slowdown:=0.70 _min_curve_speed:=0.0 _yellow_guard_enabled:=true
```

- [ ] **Step 2: Run local tests**

Run:

```bash
python3 -m unittest tests.test_cpp_line_follower_file tests.test_line_follower_cmake
python3 -m unittest discover tests
```

Expected: all tests pass.

- [ ] **Step 3: Deploy and compile on robot**

Run:

```bash
scp CMakeLists.txt package.xml src/line_follower.cpp ucar@10.68.225.179:/home/ucar/ucar_ws/src/roscar1/
scp src/line_follower.cpp ucar@10.68.225.179:/home/ucar/ucar_ws/src/roscar1/src/
ssh ucar@10.68.225.179
source /opt/ros/melodic/setup.bash
source ~/ucar_ws/devel/setup.bash
cd ~/ucar_ws
catkin_make -DCATKIN_WHITELIST_PACKAGES=roscar1
ldd ~/ucar_ws/devel/lib/roscar1/line_follower_node | grep opencv
```

Expected: build succeeds and OpenCV links only `*.so.3.2`.
