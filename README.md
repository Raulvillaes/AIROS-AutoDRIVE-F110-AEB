# AEB F110

- [How it works](#how-it-works)
  - [Speed estimation](#speed-estimation)
  - [iTTC computation](#ittc-computation)
  - [State machine](#state-machine)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [Parameters](#parameters)
- [ROS 2 Interface](#ros-2-interface)

---

Automatic Emergency Braking (AEB) for the F1TENTH platform running on the AutoDRIVE simulator. The node intercepts throttle commands from the teleop keyboard and suppresses them when a collision is imminent, then latches the brake until the operator explicitly requests reverse.

## How it works

### Speed estimation

The node subscribes to both wheel encoders (`left_encoder`, `right_encoder`) and differentiates the angular position over time to obtain a per-wheel linear speed. The longitudinal speed estimate is the average of both wheels.

### iTTC computation

On every LiDAR scan the node:

1. **Filters invalid readings** — discards `inf`, `NaN`, and ranges outside `[range_min_cutoff, range_max]`.
2. **Applies an angular window** — keeps only the beams within ±`angular_window_deg` of the forward direction.
3. **Computes the instantaneous Time-To-Collision (iTTC)** for each remaining beam:

   ```
   range_rate_i = v · cos(θᵢ)
   TTC_i        = rᵢ / range_rate_i   (only where range_rate_i > 0)
   ```

4. **Triggers braking** if `min(TTC) < ttc_threshold`.

### State machine

| State | Behaviour |
|---|---|
| `NORMAL` | Throttle commands pass through unchanged |
| `BRAKING` | Throttle is overridden to `brake_command` (0.0) regardless of operator input |

The latch is released **only** when the operator sends a negative throttle (reverse), ensuring deliberate intent to resume.

---

## Prerequisites

- ROS 2 Humble sourced (`/opt/ros/humble/setup.bash`)
- AutoDRIVE workspace (`~/autodrive_ws`) with the simulator and bridge already set up
- Python virtual environment at `~/autodrive_ws/venv/`

---

## Installation

Clone this package into the AutoDRIVE workspace source directory and build it:

```bash
cd ~/autodrive_ws/src
git clone <repository-url> aeb_f110

cd ~/autodrive_ws
source /opt/ros/humble/setup.bash
source venv/bin/activate
colcon build --packages-select aeb_f110
```

---

## Usage

Open **three terminals** from `~/autodrive_ws`. Run the following in each.

### Terminal 1 — Simulator bridge

```bash
source /opt/ros/humble/setup.bash
source venv/bin/activate
export PYTHONPATH=~/autodrive_ws/venv/lib/python3.10/site-packages:$PYTHONPATH
source install/setup.bash
ros2 launch autodrive_f1tenth simulator_bringup_rviz.launch.py
```

> For a headless run replace the last line with:
> `ros2 launch autodrive_f1tenth simulator_bringup_headless.launch.py`

### Terminal 2 — AEB node

```bash
source /opt/ros/humble/setup.bash
source venv/bin/activate && source install/setup.bash
ros2 launch aeb_f110 aeb.launch.py
```

### Terminal 3 — Teleop keyboard

```bash
source /opt/ros/humble/setup.bash
source venv/bin/activate && source install/setup.bash
ros2 run autodrive_f1tenth teleop_keyboard --ros-args \
  -r /autodrive/f1tenth_1/throttle_command:=/aeb_f110/throttle_request \
  -r /autodrive/f1tenth_1/steering_command:=/aeb_f110/steering_request
```

The remaps redirect the teleop output through the AEB node before it reaches the bridge, allowing the node to intercept and override throttle when necessary. Steering is always passed through.

---

## Parameters

All parameters can be tuned in [`launch/aeb.launch.py`](launch/aeb.launch.py).

| Parameter | Default | Description |
|---|---|---|
| `wheel_radius` | `0.058` m | F1TENTH simulated wheel radius |
| `ttc_threshold` | `0.61` s | Minimum TTC before braking is triggered |
| `range_min_cutoff` | `0.13` m | Minimum valid LiDAR range (filters close-range noise) |
| `angular_window_deg` | `12.0` ° | Half-width of the forward sector used for TTC evaluation |
| `brake_command` | `0.0` | Throttle value applied while the brake latch is active |
| `min_speed` | `0.8` m/s | Minimum speed below which AEB does not engage |

---

## ROS 2 Interface

### Subscriptions

| Topic | Type | Description |
|---|---|---|
| `/autodrive/f1tenth_1/lidar` | `sensor_msgs/LaserScan` | LiDAR scan |
| `/autodrive/f1tenth_1/left_encoder` | `sensor_msgs/JointState` | Left wheel encoder |
| `/autodrive/f1tenth_1/right_encoder` | `sensor_msgs/JointState` | Right wheel encoder |
| `/aeb_f110/throttle_request` | `std_msgs/Float32` | Throttle request from the operator |
| `/aeb_f110/steering_request` | `std_msgs/Float32` | Steering request from the operator |

### Publications

| Topic | Type | Description |
|---|---|---|
| `/autodrive/f1tenth_1/throttle_command` | `std_msgs/Float32` | Throttle command sent to the bridge (may be overridden) |
| `/autodrive/f1tenth_1/steering_command` | `std_msgs/Float32` | Steering command sent to the bridge (always passed through) |
