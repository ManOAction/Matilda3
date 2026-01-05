# Matilda Robot Roadmap

**Goal:** Build a fully autonomous, ROS 2–based differential-drive rover that can be understood, commanded, and debugged using ordinary human interfaces (Slack, etc.), while keeping the core control surfaces cleanly exposed via ROS topics.

Roadmap is ordered from easiest to implement to most complex, so you can make steady, visible progress.

---

## Legend

| Symbol        | Meaning                                    |
| ------------- | ------------------------------------------ |
| 🎯 Outcome    | What we get when the milestone is complete |
| ⚙️ Changes    | Code / ROS / hardware involved             |
| ✅ Done When  | Concrete acceptance criteria               |
| 🧩 Difficulty | Rough implementation difficulty            |

---

## Milestone 0 – Baseline (Today)

**Status:** In progress / mostly done

### 🎯 Outcome

Robot can be built, deployed, and driven via `/cmd_vel` over ROS 2, with a watchdog protecting against runaway motion.

### ⚙️ Changes (Current State)

- Build & deploy scripts (`build.sh`, `deploy.sh`)
- Motor node subscribes to `/cmd_vel`, sends serial commands to Arduino, and enforces a timeout watchdog
- systemd unit (`matilda-robot.service`) manages runtime on the Pi

### ✅ Done When

- Build succeeds from a clean clone
- Deploy script updates code and restarts service cleanly
- Publishing to `/cmd_vel` moves the robot (wheels off the ground)
- Stopping `/cmd_vel` causes motors to stop automatically

### 🧩 Difficulty

🟢 Easy (already mostly working)

---

## Milestone 1 – Operator Ergonomics & Safety

Make it pleasant and safe to drive the robot around the room.

---

### 1.1 Teleop Tools

#### 🎯 Outcome

Operators can drive the robot via keyboard or gamepad without custom publishers.

#### ⚙️ Changes

- Add standard ROS 2 teleop tools (keyboard + joystick)
- Provide launch files:
  - `teleop_keyboard.launch.py`
  - `teleop_joystick.launch.py`

#### ✅ Done When

- Driving via keyboard/gamepad requires only a single launch command
- There's a documented "Teleop" section for operators

#### 🧩 Difficulty

🟢 Easy

---

### 1.2 E-Stop Topic & Node

#### 🎯 Outcome

Software emergency stop immediately halts motion and keeps it halted until cleared.

#### ⚙️ Changes

- Add `std_msgs/Bool` topic: `/e_stop`
- Add `estop_guard` node:
  - Subscribes to `/e_stop` and `/cmd_vel`
  - If `e_stop == true`, suppresses all `/cmd_vel` before motor node
  - Optionally sends forced STOP serial command to Arduino

#### ✅ Done When

- Setting `/e_stop` to `true` prevents motion even if `/cmd_vel` is active
- Clearing `/e_stop` restores normal motion without restart
- E-stop behavior is documented in the operator manual

#### 🧩 Difficulty

🟢 Easy

---

### 1.3 Health & Status Topic

#### 🎯 Outcome

A single, easy-to-read status topic indicates whether the robot is "OK," "Warn," or "Error."

#### ⚙️ Changes

- Add `matilda_status` node:
  - Aggregates:
    - Motor node alive/dead
    - E-stop state
    - Serial connectivity
  - Publishes something like `std_msgs/String`:
    - `"OK"` | `"WARN: no_cmd_vel"` | `"ERROR: serial"`

#### ✅ Done When

- `ros2 topic echo /matilda_status` gives a useful one-line summary
- Status message meanings are documented for operators

#### 🧩 Difficulty

🟢 Easy

---

## Milestone 2 – Basic Sensor Bring-Up

Give Matilda some awareness of her environment and state.

---

### 2.1 Environmental Sensor (Si7021)

#### 🎯 Outcome

Robot publishes temperature and humidity over ROS.

#### ⚙️ Changes

- Create `matilda_sensors` package with `si7021_node`:
  - `/env/temperature` (`sensor_msgs/Temperature`)
  - `/env/humidity` (`sensor_msgs/RelativeHumidity`)
  - Poll over I2C at ~1 Hz

#### ✅ Done When

- `ros2 topic echo /env/temperature` and `/env/humidity` show plausible values
- Node can be enabled/disabled via launch configuration

#### 🧩 Difficulty

🟢 Easy → 🟡 Medium

---

### 2.2 Ultrasonic Distance Sensors

#### 🎯 Outcome

Robot publishes basic range data so we can avoid obvious obstacles.

#### ⚙️ Changes

- Node for one or more range sensors:
  - `/range/front`, `/range/left`, `/range/right` (`sensor_msgs/Range`)
  - Sample at ~5–10 Hz
  - Handle no-echo cases gracefully

#### ✅ Done When

- Moving an obstacle changes the appropriate `/range/*` topic
- Out-of-range and error conditions don't spam logs

#### 🧩 Difficulty

🟡 Medium

---

### 2.3 IMU / Orientation (FXOS8700 + FXAS21002)

#### 🎯 Outcome

Robot publishes IMU data for orientation and angular rates.

#### ⚙️ Changes

- `imu_node` publishes:
  - `/imu/data` (`sensor_msgs/Imu`)
  - Optional `/mag` (`sensor_msgs/MagneticField`)
- Define TF frame: `base_link` → `imu_link`

#### ✅ Done When

- Tilting/rotating the robot produces reasonable IMU output
- IMU frame is consistent with physical mounting and TF

#### 🧩 Difficulty

🟡 Medium

---

## Milestone 3 – Human Interfaces (Slack-First)

Let humans talk to Matilda using tools they already use, on top of ROS.

---

### 3.1 Slack Bridge (Command & Status)

#### 🎯 Outcome

Basic commands and status queries can be issued via Slack.

#### ⚙️ Changes

- Small external service (FastAPI/Flask/etc.) that:
  - Connects to ROS (rclpy or rosbridge)
  - Integrates with Slack (slash commands / bot)
  - Maps Slack commands to ROS topics:
    - `/matilda status` → query `/matilda_status` and respond
    - `/matilda stop` → set `/e_stop` true or publish idle behavior
    - `/matilda teleop` → (optional) respond with instructions / link

#### ✅ Done When

- A Slack user can request status and get a meaningful reply
- A Slack user can trigger a "stop now" that reliably halts the robot
- Slack outage or ROS outage fails safely and clearly

#### 🧩 Difficulty

🟡 Medium

---

### 3.2 Slack-Controlled Behaviors (Pre-Nav)

#### 🎯 Outcome

Slack can select from a small, safe palette of behaviors that don't require full navigation.

#### ⚙️ Changes

- Define a limited behavior set, e.g.:
  - `idle`
  - `slow_spin`
  - `slow_forward_then_back`
- Bridge maps Slack commands like:
  - `/matilda mode slow_spin` → publish mode/behavior topic for a lightweight behavior node
- Ensure behaviors respect:
  - E-stop
  - Range-based safety stop (once implemented)

#### ✅ Done When

- Operator can trigger a small demo behavior from Slack and watch it execute
- Behavior options are constrained and documented

#### 🧩 Difficulty

🟡 Medium

---

### 3.3 Early "Natural-ish" Commands (Constrained Grammar)

#### 🎯 Outcome

A thin layer translates friendly phrases into the limited behavior set.

#### ⚙️ Changes

- Very small command interpreter:
  - "spin in place" → `slow_spin`
  - "do a little demo" → `slow_forward_then_back`
- No real NLP magic yet; just safe, well-mapped templates.

#### ✅ Done When

- A handful of phrases work reliably and always map to safe behaviors
- Unknown phrases produce a clear "I don't understand" response

#### 🧩 Difficulty

🟡 Medium → 🟠 Hard (depending on how fancy you get)

---

## Milestone 4 – "Nav-Lite" & TF Structure

Give Matilda a coherent internal picture of her pose and frames.

---

### 4.1 TF Tree & Frames

#### 🎯 Outcome

Consistent TF tree: `map` → `odom` → `base_link` → sensors.

#### ⚙️ Changes

- Standard frames:
  - `map`, `odom`, `base_link`
  - `imu_link`, `range_front`, `camera_link`, etc.
- Launch with static transforms using `static_transform_publisher`.

#### ✅ Done When

- `view_frames` shows a connected, loop-free TF tree
- All sensor topics use meaningful `frame_id`s

#### 🧩 Difficulty

🟡 Medium

---

### 4.2 Dead-Reckoning Odometry

#### 🎯 Outcome

Robot publishes approximate odometry for short-term pose tracking.

#### ⚙️ Changes

- `odom_node` that:
  - Uses wheel encoders (ideal) or approximate kinematics
  - Publishes `/odom` (`nav_msgs/Odometry`)
  - Broadcasts TF `odom` → `base_link`

#### ✅ Done When

- Moving forward increases `x` consistently in `/odom`
- Rotations change `yaw` appropriately

#### 🧩 Difficulty

🟡 Medium → 🟠 Hard

---

### 4.3 Simple Obstacle Stop

#### 🎯 Outcome

Robot refuses to drive into obvious obstacles.

#### ⚙️ Changes

- `safety_stop_node`:
  - Subscribes to `/range/front` and `/cmd_vel`
  - If within configurable distance, zero out `linear.x` before motor node

#### ✅ Done When

- Placing an object in front prevents forward motion
- Removing it restores motion without restart

#### 🧩 Difficulty

🟡 Medium

---

## Milestone 5 – Vision & Camera Tower

Let Matilda see and look around.

---

### 5.1 Pi Camera ROS Node

#### 🎯 Outcome

Robot publishes camera images over ROS.

#### ⚙️ Changes

- `camera_node`:
  - `/camera/image_raw` or `/camera/image_compressed`
  - Frame: `camera_link`
- Integrate into TF tree.

#### ✅ Done When

- `rqt_image_view` shows live video feed
- Camera `frame_id` is consistent

#### 🧩 Difficulty

🟡 Medium

---

### 5.2 Servo Pan/Tilt Tower

#### 🎯 Outcome

Camera can be aimed via ROS topics.

#### ⚙️ Changes

- Servo control node:
  - `/camera/pan`, `/camera/tilt` (`std_msgs/Float32`)
- TF chain:
  - `base_link` → `camera_pan` → `camera_tilt` → `camera_link`

#### ✅ Done When

- Publishing pan/tilt angles physically moves the tower
- Optional: TF reflects updated camera orientation

#### 🧩 Difficulty

🟡 Medium

---

### 5.3 Simple Visual Behaviors

#### 🎯 Outcome

Matilda performs basic visually-guided actions (e.g. center an AprilTag).

#### ⚙️ Changes

- `vision_behavior` node:
  - Subscribes to camera + detection output
  - Publishes `/cmd_vel` to:
    - Center target in view
    - Or slowly approach a marker (with safety guards)

#### ✅ Done When

- Demo works in a controlled environment
- Behavior is clearly labeled experimental

#### 🧩 Difficulty

🟠 Hard

---

## Milestone 6 – Autonomy Behaviors

Turn sensing + motion into reusable, higher-level behaviors.

---

### 6.1 Behavior Manager (State Machine / Behavior Tree)

#### 🎯 Outcome

Named behaviors like "patrol," "wander," and "idle" that coordinate sensors and motion.

#### ⚙️ Changes

- `behavior_manager` node:
  - Listens on `/matilda/behavior` (`std_msgs/String`)
  - Behaviors:
    - `idle`
    - `wander`
    - `patrol_short`
  - Uses `/odom`, IMU, range sensors to make decisions

#### ✅ Done When

- `wander` performs slow exploration while avoiding obstacles
- `idle` reliably halts motion and cancels behaviors

#### 🧩 Difficulty

🟠 Hard

---

### 6.2 Waypoint Navigation (Local)

#### 🎯 Outcome

Robot follows a list of waypoints in local coordinates.

#### ⚙️ Changes

- Waypoint node:
  - Accepts a list of `(x, y, yaw)` in map or odom frame
  - Uses basic path-following (e.g. pure pursuit) and safety stop

#### ✅ Done When

- Robot can follow a small "square" path with repeatable results
- Drift and limitations are documented

#### 🧩 Difficulty

🔴 Hard

---

## Suggested Implementation Order (Cheat Sheet)

| Milestone         | Description                                 | Difficulty  |
| ----------------- | ------------------------------------------- | ----------- |
| ✅ Milestone 0    | Baseline system                             | Done        |
| 🟢 Milestone 1    | Teleop, E-stop, status topic                | Easy        |
| 🟢/🟡 Milestone 2 | Environmental, range, IMU sensors           | Easy–Medium |
| 🟡 Milestone 3    | Slack bridge + constrained human commands   | Medium      |
| 🟡 Milestone 4    | TF tree, odometry, simple obstacle stop     | Medium      |
| 🟡/🟠 Milestone 5 | Camera + pan/tilt + simple vision behaviors | Medium–Hard |
| 🟠/🔴 Milestone 6 | Behavior manager + waypoint navigation      | Hard        |
