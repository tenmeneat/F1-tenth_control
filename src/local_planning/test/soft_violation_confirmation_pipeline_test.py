#!/usr/bin/env python3
# Copyright 2026 2026_IFAC contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Verify transient soft collisions are ignored and persistent ones replan after confirmation."""

import math
import sys
import time

from f110_msgs.msg import Obstacle, ObstacleArray, OTWpntArray, Wpnt, WpntArray
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def latched_qos():
    """Return the transient-local QoS used by global waypoints."""
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return qos


class SoftViolationProbe(Node):
    """Pulse and then persist a same-ID uncertainty-only path violation."""

    def __init__(self):
        super().__init__('soft_violation_confirmation_probe')
        self.global_pub = self.create_publisher(
            WpntArray, '/global_waypoints', latched_qos())
        self.obstacle_pub = self.create_publisher(
            ObstacleArray, '/confirmed_static_obs', 10)
        self.odom_pub = self.create_publisher(
            Odometry, '/car_state/frenet/odom', 10)
        self.avoid_sub = self.create_subscription(
            OTWpntArray, '/avoid_waypoints', self.on_avoid, 10)

        self.stage = 'initial'
        self.stage_started = time.monotonic()
        self.committed_left = None
        self.baseline_peak = None
        self.persistent_old_outputs = 0
        self.passed = False
        self.failure = ''
        self.timer = self.create_timer(0.025, self.publish_inputs)

    @staticmethod
    def reference():
        """Create a straight ordered reference with finite track widths."""
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

    def expanded_lateral_bounds(self):
        """Expand only the committed side without entering the hard vehicle envelope."""
        if self.stage not in ('soft_pulse', 'persistent'):
            return -0.2, 0.2
        if self.committed_left:
            return -0.2, 0.30
        return -0.30, 0.2

    def publish_inputs(self):
        """Publish one short soft pulse, then a persistent soft violation."""
        if (
                self.stage == 'soft_pulse' and
                time.monotonic() - self.stage_started >= 0.035):
            self.stage = 'pulse_clear'
            self.stage_started = time.monotonic()

        stamp = self.get_clock().now().to_msg()
        reference = self.reference()
        reference.header.stamp = stamp
        self.global_pub.publish(reference)

        d_right, d_left = self.expanded_lateral_bounds()
        obstacle = Obstacle()
        obstacle.id = 41
        obstacle.has_cartesian = True
        obstacle.is_static = True
        obstacle.is_visible = True
        obstacle.x_min = 6.8
        obstacle.x_max = 7.2
        obstacle.y_min = d_right
        obstacle.y_max = d_left
        obstacle.x_center = 7.0
        obstacle.y_center = 0.5 * (d_right + d_left)
        obstacle.s_start = obstacle.x_min
        obstacle.s_end = obstacle.x_max
        obstacle.s_center = obstacle.x_center
        obstacle.d_right = d_right
        obstacle.d_left = d_left
        obstacle.d_center = obstacle.y_center
        obstacle.radius = 0.5 * math.hypot(
            obstacle.x_max - obstacle.x_min,
            obstacle.y_max - obstacle.y_min)
        obstacle.size = 2.0 * obstacle.radius
        obstacles = ObstacleArray()
        obstacles.header.stamp = stamp
        obstacles.header.frame_id = 'map'
        obstacles.obstacles = [obstacle]
        self.obstacle_pub.publish(obstacles)

        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = 'map'
        odometry.pose.pose.position.x = 0.0
        if self.committed_left is None:
            odometry.pose.pose.position.y = 0.0
        else:
            odometry.pose.pose.position.y = 0.15 if self.committed_left else -0.15
        odometry.twist.twist.linear.x = 1.0
        self.odom_pub.publish(odometry)

    def selected_peak(self, message):
        """Return the magnitude on the committed side."""
        if self.committed_left:
            return max(point.d_m for point in message.wpnts)
        return abs(min(point.d_m for point in message.wpnts))

    def on_avoid(self, message):
        """Require two old-path outputs before a persistent soft violation replans."""
        if not message.wpnts:
            return
        if message.ot_line == 'raceline_static_safe_stop':
            self.failure = f'soft-only violation incorrectly entered safe-stop in {self.stage}'
            return
        if message.ot_line != 'raceline_local_d_offset_spline':
            return

        if self.stage == 'initial':
            positive = max(point.d_m for point in message.wpnts)
            negative = abs(min(point.d_m for point in message.wpnts))
            self.committed_left = positive >= negative
            self.baseline_peak = max(positive, negative)
            self.stage = 'soft_pulse'
            self.stage_started = time.monotonic()
            return

        peak = self.selected_peak(message)
        if self.stage in ('soft_pulse', 'pulse_clear'):
            if peak > self.baseline_peak + 0.06:
                self.failure = 'transient soft violation replanned the committed path'
                return
            if (
                    self.stage == 'pulse_clear' and
                    time.monotonic() - self.stage_started >= 0.15):
                self.stage = 'persistent'
                self.stage_started = time.monotonic()
            return

        if self.stage != 'persistent':
            return
        if peak <= self.baseline_peak + 0.06:
            self.persistent_old_outputs += 1
            return
        if self.persistent_old_outputs < 2:
            self.failure = (
                'persistent soft violation replanned before three planning-cycle '
                f'confirmations: {self.persistent_old_outputs} old outputs')
            return
        self.passed = True


def main():
    """Run the probe against a fresh local_planner_node."""
    rclpy.init()
    node = SoftViolationProbe()
    deadline = time.monotonic() + 10.0
    try:
        while (
                rclpy.ok() and time.monotonic() < deadline and
                not node.passed and not node.failure):
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.passed:
            print(
                'PASS: transient soft collision kept the frozen path and persistent '
                f'collision replanned after {node.persistent_old_outputs} old outputs')
            return 0
        print(f'FAIL: {node.failure or "soft-violation scenario did not complete"}')
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
