
# RR Assignment — Webots + ROS 2 (Quick Start)

## Build & Launch
```bash
cd ~/webots_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch rr_assignment tilde_two_tb3.launch.py
````

> Make sure the Webots sim is **playing** (▶).

## Move the Robots

Open a new terminal, source your workspace, then publish commands:

**Host forward**

```bash
ros2 topic pub /host/diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {stamp: {sec: 0, nanosec: 0}}, twist: {linear: {x: 0.2}}}" -r 5
```

**Guest spin**

```bash
ros2 topic pub /guest/diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {stamp: {sec: 0, nanosec: 0}}, twist: {angular: {z: 0.5}}}" -r 5
```

Press **Ctrl+C** to stop publishing.

```
::contentReference[oaicite:0]{index=0}
```