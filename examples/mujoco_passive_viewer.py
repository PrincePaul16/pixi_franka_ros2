#!/usr/bin/env python3
""" 
pixi run -e jazzy-crisp mujoco-viewer
"""

import os
import time

import mujoco
import mujoco.viewer
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState


class PassiveViewerNode(Node):
    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData):
        super().__init__("mujoco_passive_viewer")
        self.m = model
        self.d = data

        # Map joint name -> qpos address so we can place each published joint,
        # regardless of ordering in the JointState message.
        self.qadr = {}
        for jid in range(model.njnt):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)
            if name:
                self.qadr[name] = int(model.jnt_qposadr[jid])

        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 10)
        self.get_logger().info("Passive viewer subscribed to /joint_states")

    def _on_joint_states(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            adr = self.qadr.get(name)
            if adr is not None:
                self.d.qpos[adr] = pos


def main():
    share = get_package_share_directory("franka_ros2_pixi")
    xml_path = os.path.join(share, "config", "scene.xml")

    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    # Start from the home keyframe (arm home + piece spawn poses).
    if model.nkey > 0:
        mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)

    rclpy.init()
    node = PassiveViewerNode(model, data)

    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            # Match the thesis viewer camera (robot_simulation.py).
            viewer.cam.lookat[:] = [0.0, 0.0, 0.95]
            viewer.cam.distance = 2.2
            viewer.cam.azimuth = 135
            viewer.cam.elevation = -25

            while viewer.is_running() and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.0)
                # Reconstruct body/geom poses from the subscribed qpos (no physics step).
                mujoco.mj_forward(model, data)
                viewer.sync()
                time.sleep(1.0 / 60.0)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
