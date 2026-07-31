#!/usr/bin/env python3
"""
fairino_8083_status_publisher.py

Listen-only status forwarder for the real Fairino arm, built on the port-8083
push feed (see fairino_8083_listener.py) instead of polling the SDK's
XML-RPC connection. Same role as fairino_status_publisher.py -- the only
source of /joint_states when control_system == 'hardware' and
listen_only_mode == 'true' -- but structurally read-only: it never opens
Robot.RPC() at all, so there is no command-capable connection to this node,
by construction rather than by discipline.

The controller pushes a frame over TCP every 100ms (configurable 8-100ms)
without being asked; this node never sends anything on that socket.

Requires fairino_8083_listener.py importable (same package / same directory
on PYTHONPATH).
"""
import math
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from fairino_8083_listener import Fairino8083Listener, RobotStatus


class Fairino8083StatusPublisher(Node):
    def __init__(self):
        super().__init__('fairino_8083_status_publisher')

        self.declare_parameter('robot_ip', '192.168.58.2')
        self.declare_parameter('robot_port', 8083)
        self.declare_parameter('joint_names', ['j1', 'j2', 'j3', 'j4', 'j5', 'j6'])
        self.declare_parameter('stale_timeout_sec', 2.0)

        self.robot_ip = self.get_parameter('robot_ip').value
        self.robot_port = self.get_parameter('robot_port').value
        self.joint_names = self.get_parameter('joint_names').value
        self.stale_timeout_sec = self.get_parameter('stale_timeout_sec').value

        self.pub = self.create_publisher(JointState, 'joint_states', 10)

        self._last_frame_time = None
        self._lock = threading.Lock()

        self.listener = Fairino8083Listener(
            self.robot_ip, self.robot_port, on_status=self._on_status
        )
        # run() blocks and owns its own reconnect loop, so it lives on a
        # dedicated thread; rclpy publishers are safe to call from a
        # non-executor thread.
        self._listener_thread = threading.Thread(
            target=self.listener.run, name='fairino_8083_listener', daemon=True
        )
        self._listener_thread.start()

        # separate watchdog timer on the executor thread: warns (does not
        # publish stale data) if frames stop arriving, since a hung TCP
        # connection can otherwise go unnoticed for a while.
        self._watchdog_timer = self.create_timer(1.0, self._check_stale)

    def _on_status(self, status: RobotStatus):
        """Called from the listener thread, once per valid frame."""
        with self._lock:
            self._last_frame_time = time.monotonic()

        if status.error_code != 0:
            self.get_logger().warn(
                f"robot reports error_code={status.error_code} "
                f"({status.error_description})",
                throttle_duration_sec=5.0,
            )
        if status.emergency_stop:
            self.get_logger().error("EMERGENCY STOP active", throttle_duration_sec=5.0)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [math.radians(d) for d in status.jt_cur_pos[: len(self.joint_names)]]
        self.pub.publish(msg)

    def _check_stale(self):
        with self._lock:
            last = self._last_frame_time
        if last is None:
            return  # haven't connected yet, _connect() inside the listener is still retrying
        if (time.monotonic() - last) > self.stale_timeout_sec:
            self.get_logger().warn(
                f"no status frame received in {self.stale_timeout_sec:.1f}s "
                f"-- connection may be down (listener handles its own "
                f"reconnect; this is a visibility warning only)",
                throttle_duration_sec=5.0,
            )

    def destroy_node(self):
        self.listener.stop()
        self._listener_thread.join(timeout=2.0)
        super().destroy_node()


def main():
    rclpy.init()
    node = Fairino8083StatusPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()