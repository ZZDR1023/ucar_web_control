# Web Patrol Design

## Goal

Build a Web-integrated patrol feature for the UCAR robot control panel. The first version lets the operator add patrol points from the map, save a route, start/pause/stop patrol execution, and capture a camera image at each reached point.

## Scope

This feature extends the existing `web_panel/server.py` and `web_panel/templates/index.html` control panel. It reuses the current navigation goal pipeline, AMCL map pose, safety stop logic, camera stream, and `/api/status` polling.

First version includes:

- Add patrol points from a selected map position only after pressing an explicit Add button.
- Show a patrol point list with name, map coordinates, heading, current progress, and delete controls.
- Save and load the patrol route on the robot host.
- Start, pause, resume, and stop a patrol route from the Web panel.
- Navigate through points sequentially.
- Wait briefly and save a camera image after each point is reached.
- Pause patrol progress when Web safety reports a blocking obstacle.

Out of scope for first version:

- Visual defect recognition.
- Voice interaction.
- Automatic PDF/HTML reports.
- Multi-route scheduling.
- Drag-and-drop point reordering.
- Autonomous point discovery.

## Existing Context

The current Web panel already has:

- Map click navigation using `selectedGoal`.
- `/api/goal` for publishing `PoseStamped` goals in `map`.
- `/api/stop`, `/api/takeover`, and `/api/resume_nav`.
- AMCL pose in `/api/status`.
- `goal_distance` and `move_base_status`.
- Web safety state in `forward_obstacle`.
- MJPEG camera stream and cached latest camera JPEG.

The patrol feature must not break existing single-goal navigation. In particular, adding patrol points must not publish a navigation goal by itself.

## UX Design

The Map card gets a new Patrol section below the current map tools.

Controls:

- `Patrol Point Name` text input.
- `Use Selected Goal` button.
- `Add Patrol Point` button.
- `Save Route` button.
- `Start Patrol` button.
- `Pause Patrol` / `Resume Patrol` button.
- `Stop Patrol` button.

Map interaction:

- Existing map click behavior continues to select a normal navigation goal.
- Pressing `Use Selected Goal` copies the current `selectedGoal` into `selectedPatrolCandidate`.
- `selectedPatrolCandidate` is rendered as a blue candidate marker.
- Pressing `Add Patrol Point` appends the candidate to the patrol list.
- No patrol point is added by map click alone.
- No navigation command is sent by Add.

This keeps the user's requested flow explicit:

1. Click the map.
2. Confirm or edit the point name.
3. Press Add to put it in the patrol list.

## Backend API

New endpoints:

- `GET /api/patrol`
  Returns route and execution state.

- `POST /api/patrol/points`
  Body: `{"name": "door", "x": 1.2, "y": 0.4, "yaw": 0.0}`
  Adds one point to the in-memory route and persists the route.

- `DELETE /api/patrol/points/<index>`
  Deletes one point and persists the route.

- `POST /api/patrol/save`
  Persists the current route.

- `POST /api/patrol/start`
  Starts execution from the first point unless an optional `start_index` is provided.

- `POST /api/patrol/pause`
  Pauses the patrol. It cancels the active navigation goal but does not clear the route.

- `POST /api/patrol/resume`
  Resumes from the current point.

- `POST /api/patrol/stop`
  Stops patrol execution, cancels the active goal, and returns the robot to manual mode.

`/api/status` also returns a compact `patrol` field so the existing polling loop can update the UI without extra high-frequency requests.

## Data Model

Route file:

`web_panel/patrol_route.json`

Format:

```json
{
  "points": [
    {"name": "door", "x": 0.5, "y": 0.0, "yaw": 0.0},
    {"name": "desk", "x": 1.2, "y": -0.4, "yaw": 1.57}
  ]
}
```

Runtime state:

```python
patrol_state = {
    "mode": "idle",
    "current_index": None,
    "current_point": None,
    "last_event": "not started",
    "captures": [],
}
```

Valid modes:

- `idle`
- `running`
- `paused`
- `blocked`
- `finished`
- `stopped`
- `error`

## Execution Flow

Start patrol:

1. Validate the route has at least one point.
2. Set `MANUAL_MODE = False`.
3. Ensure navigation nodes are running using the existing `start_nav_async()` flow if needed.
4. Publish the first route point using the same goal creation and publish helpers as `/api/goal`.
5. Set `current_index = 0` and `mode = running`.

Patrol worker loop:

1. If mode is `paused`, sleep and keep state unchanged.
2. If `forward_obstacle.blocked` is true, set mode to `blocked`, cancel the active goal, and wait.
3. If blocked clears, keep mode `paused`; the operator must press Resume before movement continues.
4. If the active point is reached, publish zero velocity, save one camera image, wait 3 seconds, then advance.
5. If the final point is reached, set mode to `finished`, cancel active goal, and return to manual mode.

Reached condition:

- Prefer `goal_distance <= 0.15`.
- Also accept `move_base_status.code == 3` when the robot is within a reasonable distance of the active point.

Pause:

- Set mode to `paused`.
- Cancel `/move_base/cancel`.
- Publish zero velocity.
- Do not clear `current_index`.

Stop:

- Set mode to `stopped`.
- Cancel `/move_base/cancel`.
- Publish zero velocity several times.
- Return to manual mode.
- Keep the route list available.

## Captures

Capture directory:

`web_panel/patrol_captures/`

Filename format:

`YYYYmmdd_HHMMSS_<index>_<safe_name>.jpg`

The capture uses `latest_camera_jpeg` if available. If no camera frame is available, the backend records a capture error in `last_event` but continues to the next point.

## Safety Rules

The patrol feature must respect the existing Web safety layer.

- It must not publish direct `/cmd_vel` movement except zero stop commands.
- It must use normal navigation goals, allowing `move_base` and costmaps to handle path planning.
- If `forward_obstacle.blocked` becomes true during patrol, patrol execution pauses and cancels the active goal.
- Resuming after a block is explicit. The robot must not automatically restart motion just because an obstacle moved away.
- `Stop Patrol` and existing `STOP` must both have priority over patrol execution.

## Testing Strategy

Use unit tests for patrol state and route behavior, with ROS publishers abstracted behind small helper functions where practical.

Test cases:

- Adding a point persists the route and does not change manual/navigation mode.
- Deleting a point persists the route.
- Starting with an empty route returns an error and does not start navigation.
- Starting with a route sets patrol mode to `running` and publishes the first goal.
- Pause cancels the active goal and keeps `current_index`.
- Stop cancels the active goal, returns manual mode, and keeps the route.
- Blocked safety state transitions patrol to `blocked` and requires explicit resume.
- Reaching a point advances to the next point and records a capture event.

Manual verification on the robot:

1. Start sensors and Web service.
2. Confirm `/scan`, `/odom`, `/amcl_pose`, and camera are available.
3. Click the map, press `Use Selected Goal`, enter a name, and press Add.
4. Add at least two points.
5. Press `Start Patrol`.
6. Confirm the robot drives to each point in sequence.
7. Place an obstacle in front of the robot and confirm patrol pauses.
8. Clear the obstacle, press Resume, and confirm patrol continues.
9. Confirm capture files appear in `web_panel/patrol_captures/`.

## Documentation Updates

After implementation:

- Update `README.md` with Web patrol usage.
- Update `command_reference.md` if new operational commands are introduced.
- Update `bug_notes.md` if any new real-robot issue is found during verification.
