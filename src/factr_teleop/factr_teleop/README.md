# FACTR Teleoperation for xArm7 using ZMQ

This project adapts the FACTR teleoperation framework to the UFACTORY xArm7 by replacing the original Franka/libfranka communication layer with a ZeroMQ (ZMQ) interface and the xArm Python SDK.

The architecture closely mirrors the original Franka implementation:



---

# Architecture

Unlike the original Franka implementation, the xArm does **not** provide external joint torques.

Instead:

1. The follower PC connects directly to the xArm via the Python SDK.
2. Joint positions, velocities and estimated torques are read.
3. Pinocchio computes the robot dynamics using RNEA.
4. External torque is estimated as

```
τexternal = τmeasured − τmodel
```

5. Joint state and external torque are streamed to the leader PC using ZMQ.

The leader node then behaves almost identically to the Franka implementation.

---


# Prerequisites

Install:

- ROS2 Humble
- Pinocchio
- xArm Python SDK
- pyzmq
- NumPy

Verify installation

```bash
python3 -m pip list | grep -E "pin|xarm|zmq"
```

Expected packages include

```
pin
libpinocchio
pyzmq
```

---

# Configure ZMQ Addresses

Edit

```
python_utils/python_utils/global_configs.py
```

Example:

```python
# FRANKA Panda has two computers: one for the follower arm and one for the leader. Both run on the same computer for the xArm7
sim_desktop_ip_address = "192.168.1.100"

xarm7_right_ip_address = "192.168.1.101"

xarm7_right_real_zmq_addresses = {

    "joint_state_sub":
        f"tcp://{sim_desktop_ip_address}:3099",

    "joint_torque_sub":
        f"tcp://{sim_desktop_ip_address}:3087",

    "joint_pos_cmd_pub":
        f"tcp://{sim_desktop_ip_address}:2098",
}
```

---

# Build

```bash
cd ~/ros2_ws

colcon build --symlink-install --packages-select factr_teleop

source install/setup.bash
```

---

# Running the Follower Side

The follower owns the robot.

Launch

```bash
ros2 run factr_teleop xarm7_zmq_server \
    --ip 192.168.1.205 \
    --urdf ~/ros2_ws/src/factr_teleop/factr_teleop/urdf/xarm7.urdf \
    --cmd_addr tcp://192.168.1.100:2098 \
    --state_addr tcp://192.168.1.101:3099 \
    --torque_addr tcp://192.168.1.101:3087
```

Expected output

```
pinocchio joint order:
['universe','joint1',...,'joint7']

running at 200 Hz
```

---

# What the Follower Does

At every control cycle

```
Read robot state

↓

Read joint positions

↓

Read joint velocities

↓

Read measured torques

↓

Pinocchio RNEA

↓

τexternal = τmeasured − τmodel

↓

Publish (joint positions, joint torques)

↓

Receive desired joint positions

↓

Clamp maximum joint motion

↓

Send command to xArm
```

---

# Running the Leader Side

Launch

```bash
ros2 run factr_teleop factr_teleop_xarm7_zmq \
    --ros-args \
    -p config_file:=xarm_sim.yaml
```

This node

- Reads the leader arm
- Computes the FACTR controller
- Sends desired joint positions
- Receives external torque
- Publishes ROS topics for visualization and logging

---

# ROS Topics

## Published

```
/xarm7/right/obs_xarm7_state
```

Current follower joint positions

---

```
/xarm7/right/obs_xarm7_torque
```

Estimated external joint torques

---

```
/factr_teleop/right/cmd_xarm7_pos
```

Desired follower joint positions

---

```
/factr_teleop/right/cmd_gripper_pos
```

Desired gripper position

---

# Inspect Topics

List topics

```bash
ros2 topic list
```

See publishers

```bash
ros2 topic info /xarm7/right/obs_xarm7_state
```

Echo messages

```bash
ros2 topic echo /xarm7/right/obs_xarm7_state
```

or

```bash
ros2 topic echo /xarm7/right/obs_xarm7_torque
```

---


---

# Verifying Communication

Follower publishes

```
joint state
```

```
external torque
```

Leader publishes

```
desired joint position
```

You can monitor them with

```bash
ros2 topic echo
```

or by printing

```python
print(self.xarm_joint_state_sub.message)

print(self.xarm_torque_sub.message)
```

inside `update_communication()`.


---

# Shutdown

Stop the leader

```
Ctrl+C
```

Stop the follower

```
Ctrl+C
```

The follower automatically

- stops servo mode
- disables motion
- disconnects from the robot

---

