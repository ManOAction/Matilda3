# Matilda Robot — Operator Manual

> **Matilda** is a ROS 2–based differential-drive rover operated from a Raspberry Pi and controlled through ROS topics.
> This document is the authoritative guide for building, running, monitoring, and troubleshooting the robot.

Design goals, autonomy plans, and future features live in `docs/`.
**This README is strictly about operation.**

---

## TODO / Current Mess

-- I think there is a mess with the UNO serial talking out through the USB. It's getting clogged up and causing some sort of issue that was fixed temporarily when I read the serial output.
-- The movement_behaviors are inconsistent at best right now. Not sure.
-- The UNO is locking up with solid lights and no response. Again, I think we need to troubleshoot the println statements and the sleep that's in it.

## Things I keep forgetting.

journalctl -u matilda-robot.service -n 50 --no-pager

## What This Robot Does (Today)

- Accepts velocity commands on `/cmd_vel`
- Translates those commands into motor control via Arduino over serial
- Enforces motor safety via watchdog timeout
- Runs continuously as a system service
- Logs all runtime behavior via systemd / journalctl

_If something moves, stops, pulses, or fails — this document tells you where to look._

---

## Hardware (Operational)

| Component                   | Role                                                 |
| --------------------------- | ---------------------------------------------------- |
| **Raspberry Pi**            | Runs ROS 2; hosts all control nodes                  |
| **Arduino Uno**             | Dedicated motor controller; receives serial commands |
| **Differential drive base** | Two DC motors via H-bridge                           |
| **Serial connection**       | `/dev/matilda-uno` (symlink or udev rule)            |

---

## Software Architecture (Operational View)

- ROS 2 is the control plane
- `/cmd_vel` is the only motion command interface
- A motor node converts velocity → PWM
- The Arduino never makes decisions
- Safety is enforced on the Pi, not in the cloud

---

## Repository Layout

```
matilda_robot/
├── scripts/
│   ├── build.sh              # Build ROS workspace
│   └── deploy.sh             # Update, build, restart service
├── ws/
│   └── src/
│       └── matilda_motor/
│           └── motor_node.py
├── systemd/
│   └── matilda-robot.service
├── docs/                      # Design & future plans
└── README.md
```

---

## Requirements

- Ubuntu (Raspberry Pi)
- ROS 2 Kilted
- Python 3
- colcon
- rosdep
- systemd

---

## Build (Operator Procedure)

From the repo root:

```bash
./scripts/build.sh
```

**What this does:**

1. Enters the ROS workspace
2. Sources ROS 2 Kilted
3. Installs missing dependencies
4. Builds all packages using colcon

A successful build ends with:

```
BUILD SUCCESS
```

> ⚠️ **If the build fails, do not deploy.**

---

## Deploy (Operator Procedure)

To update code and restart the robot:

```bash
./scripts/deploy.sh
```

This will:

1. Hard reset to `origin/main`
2. Rebuild the workspace
3. Restart the system service (if installed)

**This is the only supported deployment method.**

---

## Runtime Management (systemd)

### Check Status

```bash
sudo systemctl status matilda-robot.service
```

Look for:

- `Active: active (running)`
- No rapid restart loops
- No permission errors

### Restart Robot

```bash
sudo systemctl restart matilda-robot.service
```

---

## Logs & Troubleshooting (Critical Section)

### Live Logs (Primary Debug Tool)

```bash
journalctl -u matilda-robot.service -f
```

**Use this when:**

- Motors don't move
- Motors pulse unexpectedly
- Node crashes
- Serial errors occur
- Robot stops responding

> 🔍 **This is always your first stop.**

### Recent Logs (Last Boot)

```bash
journalctl -u matilda-robot.service -b
```

### Search for Errors Only

```bash
journalctl -u matilda-robot.service | grep -i error
```

### ROS Node Logging

ROS logs appear inside journalctl.
You do not need to hunt through `~/.ros/log` for normal operation.

---

## Motor Control Node (Operational Details)

### Subscriptions

- `/cmd_vel` (`geometry_msgs/Twist`)

### Parameters

| Parameter    | Default            | Purpose           |
| ------------ | ------------------ | ----------------- |
| `port`       | `/dev/matilda-uno` | Serial device     |
| `baud`       | `115200`           | Serial speed      |
| `max_pwm`    | `255`              | Motor power limit |
| `timeout_ms` | `500`              | Watchdog timeout  |

### Safety Behavior (Important)

If `/cmd_vel` stops publishing, **motors are stopped automatically**.

This prevents runaway behavior if:

- A node crashes
- Network drops
- Teleop disconnects

> 💡 **Pulsing motors usually mean the watchdog is working correctly.**

---

## Testing Motion (Safe Procedure)

> ⚠️ **Lift wheels off the ground before first test**

```bash
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

For continuous motion, publish at a steady rate (e.g., 10 Hz).

---

## Common Problems & Fixes

### Motors Pulse Instead of Running

| Cause                                  | Fix                                                     |
| -------------------------------------- | ------------------------------------------------------- |
| `/cmd_vel` not publishing continuously | Use a node or timer-based publisher (10 Hz recommended) |

### Robot Doesn't Move At All

Check:

1. `journalctl -u matilda-robot.service`
2. Serial device exists: `ls -l /dev/matilda-uno`
3. Arduino firmware is running
4. Baud rate matches

### Service Keeps Restarting

Check logs for:

- Python exceptions
- Serial permission errors
- Missing ROS environment

### Build Fails

1. Re-run `rosdep install`
2. Confirm ROS 2 Kilted is sourced
3. **Do not deploy until build succeeds**

---

## Emergency Stop

If needed:

```bash
sudo systemctl stop matilda-robot.service
```

This immediately halts all ROS nodes.

---

## Design & Future Work

Design rationale, autonomy plans, sensors, and human interfaces are documented separately in `docs/`.

_This README intentionally avoids speculation._

---

## Operator Rules of Thumb

| Symptom         | Action             |
| --------------- | ------------------ |
| It moves wrong  | Check logs         |
| It doesn't move | Check logs         |
| It pulses       | Check publish rate |
| If unsure       | Stop the service   |
