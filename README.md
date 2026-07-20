# FACTR_xArm7
## Installation

This repository requires **ROS 2**.
If you have not installed ROS 2 yet, follow the official [ROS 2 installation guide](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html).

### Provided ROS 2 Packages

The following ROS 2 packages are included in this repository:

- `factr_teleop`
- `bc`
- `cameras`
- `python_utils`

These packages are located in:

```
<repo_root>/src
```

### ROS 2 Workspace Setup

These packages must reside within a **ROS 2 workspace**. If you do not already have one, create a workspace by following the [ROS 2 workspace tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html).

Then:

1. Copy the four provided packages into your workspace's `src/` directory.
2. Ensure to source the ROS2 setup script in your terminal
   ```bash
   source /opt/ros/<ROS-Distribution>/setup.bash
   ```
   Note that this command should be run every time you open a new terminal.
3. From the root of your workspace, build the workspace via:
   ```bash
   colcon build --symlink-install
   ```
   This should create the following folders in your workspace root
   ```bash
   build  install  log  src
   ```
4. From the root of your workspace, source the overlay via> For more guidance, refer to the [ROS 2 Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html).

### Additional Python Dependencies

Install [ZMQ](https://zeromq.org/):

```bash
pip install zmq
```
Install [Pinocchio](https://stack-of-tasks.github.io/pinocchio/):
```bash
sudo apt install ros-<ROS-Distribution>-pinocchio
```
- For example,
   ```bash
   sudo apt install ros-humble-pinocchio
   ```
Alternatively, try the following via pip.
```bash
python -m pip install pin
```

Finally, navigate to the Dynamixel submodule and install it via:
```bash
cd <repo_root>/src/factr_teleop/factr_teleop/dynamixel
pip install -e python
```


## FACTR Teleop
Instructions for setting up FACTR leader arms and running the provided example demos can be found 
[here](src/factr_teleop/README.md).
   ```bash
   source install/local_setup.bash
   ```
   Note that this command should also be run every time you open a new terminal.



   
