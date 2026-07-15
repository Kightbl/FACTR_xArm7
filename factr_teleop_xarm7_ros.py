#!/usr/bin/env python3
"""
factr_teleop_xarm7_ros.py
--------------------------
FACTRTeleop subclass for xArm7 that talks to the REAL arm through UFACTORY's
official xarm_ros2 driver (package `xarm_api`) instead of a custom ZMQ bridge.

Architecture:
  FACTR (this node)  <-- ROS2 topic/service -->  xarm_api driver node  <--> xArm7

Prerequisites:
  1. Install xarm_ros2 (https://github.com/xArm-Developer/xarm_ros2) and build it
     alongside your factr_teleop workspace.
  2. Launch the vendor driver FIRST:
       ros2 launch xarm_api xarm7_driver.launch.py robot_ip:=192.168.1.XXX \
           report_type:=rich
     (bump report_type/rate so /xarm/joint_states updates fast enough for your
     control loop -- default is 5Hz, your yaml runs controller.frequency: 500)
  3. If running two arms (left/right), the driver needs separate namespaces --
     check xarm_api's launch args (e.g. hw_ns / robot_ip per instance) and set
     `ros_namespace` in your yaml accordingly.

ASSUMPTIONS (verify against your installed xarm_msgs version):
  - /{ns}/set_servo_angle_j uses xarm_msgs/srv/MoveJoint
  - /{ns}/motion_enable uses xarm_msgs/srv/SetInt16ById
  - /{ns}/set_mode, /{ns}/set_state use xarm_msgs/srv/SetInt16
  - /{ns}/joint_states is sensor_msgs/msg/JointState (position/velocity/effort)
"""
import os
import pinocchio as pin
from python_utils.utils import get_workspace_root
import numpy as np
import rclpy
import threading
import time
from sensor_msgs.msg import JointState

from xarm_msgs.srv import MoveJoint, SetInt16, SetInt16ById

from factr_teleop.factr_teleop_xArm7 import FACTRTeleop

NUM_ARM_JOINTS = 7


def create_array_msg(data):
    msg = JointState()
    msg.position = list(map(float, data))
    return msg


class FACTRTeleopXArm7ROS(FACTRTeleop):

    def __init__(self):
        super().__init__()
        #Follower arm model
        follower_urdf_path = os.path.join(
            get_workspace_root(),
            'src/factr_teleop/factr_teleop/urdf/',
            self.config["arm_teleop"]["follower_urdf"]
        )
        follower_urdf_dir = os.path.dirname(follower_urdf_path)
        self.follower_model, _, _ = pin.buildModelsFromUrdf(
            filename=follower_urdf_path, package_dirs=follower_urdf_dir
        )
        self.follower_data = self.follower_model.createData()
        self.get_logger().info(f"Follower pinocchio joint order: {list(self.follower_model.names)}")
        self.torque_est_ema_beta = self.config["controller"]["torque_feedback"].get("torque_est_ema_beta", 0.2)
        self.prev_external_torque_est = np.zeros(NUM_ARM_JOINTS)


        self.gripper_feedback_gain = self.config["controller"]["gripper_feedback"]["gain"]
        self.gripper_torque_ema_beta = self.config["controller"]["gripper_feedback"]["ema_beta"]
        self.gripper_external_torque = 0.0
        self.latest_joint_state = None

        # Rate at which set_servo_angle_j is actually called (Hz). Keep within
        # xarm_api's suggests 100-250Hz streaming range
        #FACTR uses 500Hz but xArm7 is not intended to operate at that speed
        self.command_rate_hz = self.config["arm_teleop"].get("xarm_command_rate_hz", 200.0)

        self._pending_target_lock = threading.Lock()
        self._pending_arm_target = None
        self._command_thread_stop = threading.Event()
        self._command_thread = None


    # ── Abstract method implementations ───────────────────────────────────

    def set_up_communication(self):
        if self.name not in ("left", "right"):
            raise ValueError(f"Invalid robot name '{self.name}'. Expected 'left' or 'right'.")

        # Namespace of the xarm_api driver for this arm -- set this to match
        # however you launched xarm7_driver.launch.py (single arm: usually
        # just "/xarm"; dual arm: whatever hw_ns you assigned per side).
        self.xarm_ns = self.config["arm_teleop"].get("xarm_ros_namespace", "/xarm")

        #Service clients: driver setup + streaming joint commands
        self.motion_enable_client = self.create_client(SetInt16ById, f"{self.xarm_ns}/motion_enable")
        self.set_mode_client = self.create_client(SetInt16, f"{self.xarm_ns}/set_mode")
        self.set_state_client = self.create_client(SetInt16, f"{self.xarm_ns}/set_state")
        self.set_servo_angle_j_client = self.create_client(MoveJoint, f"{self.xarm_ns}/set_servo_angle_j")

        for name, client in [
            ("motion_enable", self.motion_enable_client),
            ("set_mode", self.set_mode_client),
            ("set_state", self.set_state_client),
            ("set_servo_angle_j", self.set_servo_angle_j_client),
        ]:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"Waiting for xarm_api service '{name}'...")

        #Enable arm, put it in joint-servo (servoj) mode
        '''
        self.motion_enable_client.call_async(SetInt16ById.Request(id=8, data=1))
        self.set_mode_client.call_async(SetInt16.Request(data=1))  # 1 = servoj mode
        self.set_state_client.call_async(SetInt16.Request(data=0))
        '''
        #Subscribe to the driver's joint state feedback (position, velocity, effort)
        self.xarm_joint_state_sub = self.create_subscription(
            JointState,
            f"{self.xarm_ns}/joint_states",
            self._xarm_joint_state_callback,
            10
        )

        #Re-publish everything to ROS for logging / data collection
        self.obs_xarm7_state_pub = self.create_publisher(
            JointState,
            f'/xarm7/{self.name}/obs_xarm7_state',
            10
        )
        self.cmd_xarm7_pos_pub = self.create_publisher(
            JointState,
            f'/factr_teleop/{self.name}/cmd_xarm7_pos',
            10
        )
        self.cmd_gripper_pos_pub = self.create_publisher(
            JointState,
            f'/factr_teleop/{self.name}/cmd_gripper_pos',
            10
        )

        if self.enable_gripper_feedback:
            self.obs_gripper_torque_pub = self.create_subscription(
                JointState,
                f'/xarm7/{self.name}/obs_gripper_torque',
                self._gripper_external_torque_callback,
                1,
            )

        #Confirm arm setup: enable and servoj mode
        self._call_service_blocking(
            self.motion_enable_client,
            SetInt16ById.Request(id=8, data=1),
            "motion_enable"
        )
        self._call_service_blocking(
            self.set_mode_client,
            SetInt16.Request(data=1),
            "set_mode(servoj)"
        )
        self._call_service_blocking(
            self.set_state_client,
            SetInt16.Request(data=0),
            "set_state(ready)"
        )

        #Block until real joint state is recieved

        if self.enable_torque_feedback:
            self.obs_xarm7_torque_pub = self.create_publisher(
                JointState,
                f'/xarm7/{self.name}/obs_xarm7_torque',
                10
            )
            self.get_logger().info(f"Waiting for xArm7 {self.name}'s first /joint_states message...")
            while self.latest_joint_state is None and rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
            self.get_logger().info(f"Received xArm7 {self.name}'s joint state. Ready.")

        #Start the throttled command-streaming thread
        self._command_thread = threading.Thread(target=self._command_loop, daemon=True)
        self._command_thread.start()

    def _call_service_blocking(self, client, request, description, timeout_sec=5.0):
        """
        Call a service and actually wait for + validate the result, by
        spinning this node's own executor (safe to call before main() starts
        spinning, since nothing else is spinning yet).
        """
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            raise RuntimeError(f"xArm7 {self.name}: '{description}' call timed out.")
        result = future.result()
        if result is None:
            raise RuntimeError(f"xArm7 {self.name}: '{description}' call failed: {future.exception()}")
        #xarm_msgs responses generally carry a `ret` field; 0 == success.
        ret = getattr(result, "ret", 0)
        if ret != 0:
            raise RuntimeError(f"xArm7 {self.name}: '{description}' returned error code {ret}.")
        self.get_logger().info(f"xArm7 {self.name}: '{description}' succeeded.")
        return result

    def _xarm_joint_state_callback(self, msg):
        self.latest_joint_state = msg

    def _gripper_external_torque_callback(self, data):
        gripper_external_torque = data.position[0]
        self.gripper_external_torque = self.gripper_torque_ema_beta * self.gripper_external_torque + \
            (1 - self.gripper_torque_ema_beta) * gripper_external_torque


    def get_leader_arm_external_joint_torque(self):
        if self.latest_joint_state is None or not self.latest_joint_state.effort:
            external_torque = np.zeros(NUM_ARM_JOINTS)
            self.obs_xarm7_torque_pub.publish(create_array_msg(external_torque))
            return external_torque

        q = np.array(self.latest_joint_state.position[:NUM_ARM_JOINTS])
        qdot = np.array(self.latest_joint_state.velocity[:NUM_ARM_JOINTS]) \
            if self.latest_joint_state.velocity else np.zeros(NUM_ARM_JOINTS)
        tau_measured = np.array(self.latest_joint_state.effort[:NUM_ARM_JOINTS])

        qddot = np.zeros(NUM_ARM_JOINTS)

        tau_model = pin.rnea(self.follower_model, self.follower_data, q, qdot, qddot)

        tau_external_raw = tau_measured - tau_model

        # EMA filter
        tau_external_filtered = (
            self.torque_est_ema_beta * self.prev_external_torque_est
            + (1 - self.torque_est_ema_beta) * tau_external_raw
        )
        self.prev_external_torque_est = tau_external_filtered
        tau_external_corrected = tau_external_filtered * self.joint_signs[:NUM_ARM_JOINTS]
        self.obs_xarm7_torque_pub.publish(create_array_msg(tau_external_corrected))
        return tau_external_corrected

    def get_leader_gripper_feedback(self):
        return self.gripper_external_torque

    def gripper_feedback(self, leader_gripper_pos, leader_gripper_vel, gripper_feedback):
        torque_gripper = -1.0 * gripper_feedback / self.gripper_feedback_gain
        return torque_gripper

    def update_communication(self, leader_arm_pos, leader_gripper_pos):
        #Store latest joint position 
        with self._pending_target_lock:
            self._pending_arm_target = np.array(leader_arm_pos, dtype=float)

        self.cmd_xarm7_pos_pub.publish(create_array_msg(leader_arm_pos))
        self.cmd_gripper_pos_pub.publish(create_array_msg([leader_gripper_pos]))

        if self.latest_joint_state is not None:
            self.obs_xarm7_state_pub.publish(self.latest_joint_state)

    def _command_loop(self):
        period = 1.0 / self.command_rate_hz
        while not self._command_thread_stop.is_set() and rclpy.ok():
            t0 = time.time()
            with self._pending_target_lock:
                target = self._pending_arm_target
            if target is not None:
                req = MoveJoint.Request()
                req.angles = target.tolist()
                self.set_servo_angle_j_client.call_async(req)
            elapsed = time.time() - t0
            time.sleep(max(0.0, period - elapsed))
 
    def shut_down(self):
        self._command_thread_stop.set()
        if self._command_thread is not None:
            self._command_thread.join(timeout=1.0)
        super().shut_down()


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = FACTRTeleopXArm7ROS()
    try:
        while rclpy.ok():
            rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Shutting down...")
        node.shut_down()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
