#!/usr/bin/env python3
# Copyright 2026 2026_IFAC contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Verify stale-perception commitment retention, handoff, and next-lap memory."""

import math
import sys
import time

from f110_msgs.msg import Obstacle, ObstacleArray, OTWpntArray, StateMachine
from f110_msgs.msg import Wpnt, WpntArray
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def latched_qos():
    """Return the transient-local QoS used by global waypoints and state."""
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return qos


class StaleObstacleMemoryProbe(Node):
    """Stop perception after commitment and exercise one complete lap cycle."""

    def __init__(self):
        super().__init__('stale_obstacle_memory_probe')
        self.global_pub = self.create_publisher(
            WpntArray, '/global_waypoints', latched_qos())
        self.obstacle_pub = self.create_publisher(
            ObstacleArray, '/confirmed_static_obs', 10)
        self.odom_pub = self.create_publisher(
            Odometry, '/car_state/frenet/odom', 10)
        self.state_pub = self.create_publisher(
            StateMachine, '/state', latched_qos())
        self.avoid_sub = self.create_subscription(
            OTWpntArray, '/avoid_waypoints', self.on_avoid, 10)

        self.stage = 'initial'
        self.ego_s = 0.0
        self.publish_obstacles = True
        self.sensor_cut_time = None
        self.committed_geometry = None
        self.merge_s = None
        self.stale_outputs = 0
        self.saw_handoff = False
        self.passed = False
        self.failure = ''
        self.timer = self.create_timer(0.025, self.publish_inputs)

    @staticmethod
    def reference():
        """Create an ordered straight race line with finite track widths."""
        message = WpntArray()
        message.header.frame_id = 'map'
        for index in range(300):
            waypoint = Wpnt()
            waypoint.id = index
            waypoint.s_m = 0.1 * index
            waypoint.x_m = waypoint.s_m
            waypoint.y_m = 0.0
            waypoint.psi_rad = 0.0
            waypoint.kappa_radpm = 0.0
            waypoint.vx_mps = 2.0
            waypoint.d_left = 1.5
            waypoint.d_right = 1.5
            message.wpnts.append(waypoint)
        return message

    @staticmethod
    def obstacle():
        """Create one detector-style static obstacle near the race line."""
        obstacle = Obstacle()
        obstacle.id = 41
        obstacle.has_cartesian = True
        obstacle.is_static = True
        obstacle.is_visible = True
        obstacle.x_center = 7.0
        obstacle.y_center = 0.0
        obstacle.x_min = 6.8
        obstacle.x_max = 7.2
        obstacle.y_min = -0.2
        obstacle.y_max = 0.2
        obstacle.s_start = 6.8
        obstacle.s_end = 7.2
        obstacle.s_center = 7.0
        obstacle.d_right = -0.2
        obstacle.d_left = 0.2
        obstacle.d_center = 0.0
        obstacle.radius = 0.5 * math.hypot(0.4, 0.4)
        obstacle.size = 2.0 * obstacle.radius
        return obstacle

    @staticmethod
    def geometry(message):
        """Extract the fields that must remain frozen while perception is stale."""
        return [
            (
                waypoint.s_m,
                waypoint.d_m,
                waypoint.x_m,
                waypoint.y_m,
                waypoint.vx_mps,
            )
            for waypoint in message.wpnts
        ]

    @staticmethod
    def same_geometry(first, second):
        """Compare two serialized paths with a tight floating-point tolerance."""
        if len(first) != len(second):
            return False
        return all(
            all(abs(lhs - rhs) <= 1.0e-9 for lhs, rhs in zip(a, b))
            for a, b in zip(first, second)
        )

    def publish_inputs(self):
        """Keep localization fresh while deliberately stopping obstacle input."""
        stamp = self.get_clock().now().to_msg()
        reference = self.reference()
        reference.header.stamp = stamp
        self.global_pub.publish(reference)

        if self.publish_obstacles:
            obstacles = ObstacleArray()
            obstacles.header.stamp = stamp
            obstacles.header.frame_id = 'map'
            obstacles.obstacles.append(self.obstacle())
            self.obstacle_pub.publish(obstacles)

        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = 'map'
        odometry.pose.pose.position.x = self.ego_s
        odometry.pose.pose.position.y = 0.0
        odometry.twist.twist.linear.x = 1.0
        self.odom_pub.publish(odometry)

        state = StateMachine()
        state.header.stamp = stamp
        state.header.frame_id = 'map'
        state.state = (
            StateMachine.STATE_GLOBAL
            if self.stage in ('confirm_global', 'next_lap')
            else StateMachine.STATE_AVOID
        )
        self.state_pub.publish(state)

    def fail_if_empty(self, message):
        """Reject fail-open empty output before GLOBAL handoff confirmation."""
        if message.wpnts:
            return False
        if self.stage in ('stale_hold', 'merge', 'wait_handoff'):
            self.failure = (
                f'avoid waypoints became empty during stage {self.stage}'
            )
        return True

    def on_avoid(self, message):
        """Advance through stale hold, merge handoff, and next-lap replan."""
        if self.fail_if_empty(message):
            if self.stage == 'confirm_global' and self.saw_handoff:
                self.stage = 'next_lap'
                self.ego_s = 0.0
            return

        if self.stage == 'initial':
            if message.ot_line != 'raceline_local_d_offset_spline':
                return
            if max(abs(waypoint.d_m) for waypoint in message.wpnts) < 0.35:
                self.failure = 'initial obstacle did not produce an avoidance offset'
                return
            self.committed_geometry = self.geometry(message)
            self.merge_s = message.wpnts[-1].s_m - 5.0
            self.publish_obstacles = False
            self.sensor_cut_time = time.monotonic()
            self.stage = 'stale_hold'
            self.get_logger().info(
                'stopped /static_obs after commitment; waiting past stale timeout')
            return

        if self.stage == 'stale_hold':
            if message.ot_line != 'raceline_local_d_offset_spline':
                self.failure = (
                    f'commitment changed to {message.ot_line} before merge')
                return
            if not self.same_geometry(
                    self.committed_geometry, self.geometry(message)):
                self.failure = 'committed geometry changed while perception was stale'
                return
            self.stale_outputs += 1
            if time.monotonic() - self.sensor_cut_time >= 1.0:
                self.ego_s = self.merge_s
                self.stage = 'merge'
            return

        if self.stage in ('merge', 'wait_handoff'):
            if message.ot_line == 'raceline_global_handoff':
                self.saw_handoff = True
                self.stage = 'confirm_global'
                return
            self.stage = 'wait_handoff'
            return

        if self.stage == 'confirm_global':
            if message.ot_line != 'raceline_global_handoff':
                self.failure = (
                    'planner stopped handoff before STATE_GLOBAL confirmation')
            return

        if self.stage == 'next_lap':
            if message.ot_line != 'raceline_local_d_offset_spline':
                self.failure = (
                    f'next lap used {message.ot_line} instead of retained memory')
                return
            if max(abs(waypoint.d_m) for waypoint in message.wpnts) < 0.35:
                self.failure = 'retained obstacle did not produce next-lap avoidance'
                return
            self.passed = True


def main():
    """Run the probe against a fresh local_planner_node."""
    rclpy.init()
    node = StaleObstacleMemoryProbe()
    deadline = time.monotonic() + 12.0
    try:
        while (
                rclpy.ok() and time.monotonic() < deadline and
                not node.passed and not node.failure):
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.passed:
            print(
                'PASS: retained frozen avoidance for '
                f'{node.stale_outputs} stale outputs, completed GLOBAL handoff, '
                'and reused the obstacle snapshot on the next lap')
            return 0
        print(f'FAIL: {node.failure or "stale-memory cycle did not complete"}')
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
