# AEB F110 🚗💨

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue?logo=ros)
![Python](https://img.shields.io/badge/Python-3.10-blue?logo=python)
![Platform](https://img.shields.io/badge/Platform-Ubuntu_22.04-orange?logo=ubuntu)
![License](https://img.shields.io/badge/License-MIT-green)

Automatic Emergency Braking (AEB) for the F1TENTH platform running on the AutoDRIVE simulator. The node intercepts throttle commands from the teleop keyboard and suppresses them when a collision is imminent, then latches the brake until the operator explicitly requests reverse.

---

![AEB demo](demo.gif)

---

## 📋 Table of Contents

- [Step 1 — Prerequisites](#step-1--prerequisites)
- [Step 2 — Installation](#step-2--installation)
- [Step 3 — Usage](#step-3--usage)
- [How it works](#how-it-works)
- [Project structure](#project-structure)
- [Parameters](#parameters)
- [ROS 2 Interface](#ros-2-interface)
- [Presentation Slides](#presentation-slides)
- [Contact](#contact)

---

## Step 1. Prerequisites 📦

- **[Ubuntu 22.04](https://releases.ubuntu.com/22.04/)**
- **[ROS 2 Humble](https://docs.ros.org/en/humble/)** sourced (`/opt/ros/humble/setup.bash`)
- **AutoDRIVE DevKit** workspace (`~/autodrive_ws`) set up and built — see [AutoDRIVE_DevKit_Starter](https://github.com/hector-la/AutoDRIVE_DevKit_Starter/tree/main)
- Python virtual environment at `~/autodrive_ws/venv/`

---

## Step 2. Installation ⚙️

Clone this package into the AutoDRIVE workspace source directory and build it:

```bash
cd ~/autodrive_ws/src
git clone https://github.com/Raulvillaes/AIROS-AutoDRIVE-F110-AEB aeb_f110

cd ~/autodrive_ws
source /opt/ros/humble/setup.bash
source venv/bin/activate
colcon build --packages-select aeb_f110
```

---

## Step 3. Usage 🚀

First, **open the AutoDRIVE Simulator** application. Then open **three terminals** from `~/autodrive_ws` and run the following in each.

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

## ⚙️ How it works

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

## 📁 Project structure

```
aeb_f110/
├── aeb_f110/
│   ├── __init__.py
│   └── aeb_node.py        # Main AEB node
├── launch/
│   └── aeb.launch.py      # Launch file with tunable parameters
├── resource/
│   └── aeb_f110
├── demo.gif
├── package.xml
├── setup.cfg
└── setup.py
```

---

## 🔧 Parameters

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

## 🔌 ROS 2 Interface

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

---

## 📬 Contact

This project was developed as part of **AIROS – ESPOL**.

**Raúl Villavicencio**  
GitHub: [https://github.com/Raulvillaes](https://github.com/Raulvillaes)

**Micaela Anamise**  
GitHub: [https://github.com/carolinanamise13-hub](https://github.com/carolinanamise13-hub)

---

*AIROS – ESPOL*
