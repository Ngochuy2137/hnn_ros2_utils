# hnn_ros2_utils

Utility nodes and scripts for ROS 2 (HNN Robotics). This package is intended to grow over time; new utilities will be added as needed.

**Target ROS 2 distro:** Jazzy (C++ node tested on Jazzy; Python script works with Humble/Jazzy).

---

## Build and install

From your workspace:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select hnn_ros2_utils
source install/setup.bash
```

---

## Utilities overview

| Utility | Type | Description |
|--------|------|--------------|
| [costmap_click_cost_node](#costmap_click_cost_node) | ROS 2 node (C++) | Print local costmap cost at RViz clicked point |
| [resource_monitor_continuous](#resource_monitor_continuous) | Python script | Log CPU, RAM, iftop, and ROS 2 topic bandwidth/delay |

*(More utilities will be listed here as they are added.)*

---

## costmap_click_cost_node

**File:** `src/costmap_click_cost_node.cpp`  
**Target:** ROS 2 Jazzy (compatible with Humble/Jazzy).

Subscribes to:

- **`/clicked_point`** (`geometry_msgs/PointStamped`) — from RViz “Publish Point” tool.
- **Local costmap** — configurable topic (see parameters).

On each click, transforms the point into the costmap frame, maps it to a grid cell, and prints the cost value to the console (`0` = free, `254` = lethal, `255` = unknown). If the point is outside the costmap bounds, it prints “OUT OF BOUNDS”.

### Run

```bash
ros2 run hnn_ros2_utils costmap_click_cost_node
```

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `local_costmap_topic` | string | `"/local_costmap/costmap"` | Topic of the local costmap (`nav_msgs/OccupancyGrid`). Use e.g. `"/local_costmap/costmap"` |
| `clicked_point_topic` | string | `"/clicked_point"` | Topic for RViz clicked points. |

Example with custom costmap topic:

```bash
ros2 run hnn_ros2_utils costmap_click_cost_node --ros-args -p local_costmap_topic:=/move_base/local_costmap/costmap
```

### Requirements

- Navigation (or costmap publisher) running so the local costmap is published.
- RViz with “Publish Point” tool; Fixed Frame should match your setup (e.g. `map` or `odom`). TF between the clicked-point frame and the costmap frame must be available.

---

## resource_monitor_continuous

**File:** `src/resource_monitor_continuous.py`  
**Type:** Standalone Python script (no `ros2 run`; run with `python3`).

Continuously samples:

- **CPU** — total %, per-thread %, frequency (MHz), temperature (°C) if available.
- **RAM** — used/available/total and percent.
- **iftop** — total send/receive rate for given interfaces (e.g. `eth0`, `wlan0`), updated in background threads.
- **ROS 2 topic** — bandwidth (MB/s) and delay (avg/min/max ms) for `/scan`; uses `rclpy` subscription if available, otherwise `ros2 topic bw` / `ros2 topic delay` subprocesses.

Results are written to a log file (one JSON line per sample) and optionally printed to the console. Heavy work (iftop, ROS 2 bw/delay) runs in background threads so the main loop stays light.

### Install dependencies

```bash
pip install psutil
sudo apt-get install -y iftop   # optional, for network stats
```

### Run

With ROS 2 sourced (needed for `/scan` bandwidth and delay):

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src/hnn_ros2_utils/src
python3 resource_monitor_continuous.py [options]
```

**Options:**

| Option | Default | Description |
|--------|---------|-------------|
| `--interval` | `1.0` | Sampling interval in seconds. |
| `--no-iftop` | — | Disable iftop. |
| `--no-ros2-bw` | — | Disable ROS 2 topic bandwidth/delay. |
| `--interfaces` | `eth0 wlan0` | Interfaces for iftop. |
| `--json` | — | Print each sample as one JSON line to stdout. |
| `--log` | `resource_monitor.log` | Log file path. Use `--log ''` to disable file logging. |
| `--print` | — | Print human-readable summary to console each interval. |

Examples:

```bash
# Log every 2 s, print to console, no iftop
python3 resource_monitor_continuous.py --interval 2 --no-iftop --print

# JSON to stdout, no log file
python3 resource_monitor_continuous.py --log '' --json
```

---

## Adding new utilities

When adding a new utility:

1. Add the source file under `src/` (and extend `CMakeLists.txt` / `package.xml` if it is a built node).
2. Add one row in the [Utilities overview](#utilities-overview) table (name, type, one-line description).
3. Add a new section below with the same heading as the table link (e.g. `## my_new_tool`), with short description, run instructions, parameters/options, and requirements.
