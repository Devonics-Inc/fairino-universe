#!/usr/bin/env python3
"""Republish the MoveIt planning scene on a latched topic for late subscribers."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import GetPlanningScene


class PlanningSceneRelay(Node):
    def __init__(self):
        super().__init__("planning_scene_relay")

        latched = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.pub = self.create_publisher(
            PlanningScene, "planning_scene_latched", latched
        )

        self.create_subscription(
            PlanningScene, "monitored_planning_scene", self.on_scene, 10
        )

        self.cli = self.create_client(GetPlanningScene, "get_planning_scene")
        self.startup_timer = self.create_timer(1.0, self.fetch_initial)

    def on_scene(self, msg: PlanningScene) -> None:
        self.pub.publish(msg)

    def fetch_initial(self) -> None:
        """Seed the latched topic once, so we're not empty until the first change."""
        if not self.cli.service_is_ready():
            self.get_logger().info("waiting for /get_planning_scene...")
            return

        self.startup_timer.cancel()
        req = GetPlanningScene.Request()
        req.components.components = 0xFFFF  # everything
        future = self.cli.call_async(req)
        future.add_done_callback(self.on_initial)

    def on_initial(self, future) -> None:
        try:
            self.pub.publish(future.result().scene)
            self.get_logger().info("seeded latched scene")
        except Exception as exc:
            self.get_logger().warn(f"initial fetch failed: {exc}")


def main() -> None:
    rclpy.init()
    node = PlanningSceneRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
