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

"""Verify that an unsafe commitment may switch sides before lateral engagement."""

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


class PreEngagementSwitchProbe(Node):
    """Invalidate an initial left path while ego remains on d=0."""

    def __init__(self):
        super().__init__('pre_engagement_side_switch_probe')
        self.global_pub = self.create_publisher(
            WpntArray, '/global_waypoints', latched_qos())
        self.obstacle_pub = self.create_publisher(ObstacleArray, '/confirmed_static_obs', 10)
        self.odom_pub = self.create_publisher(
            Odometry, '/car_state/frenet/odom', 10)
        self.path_sub = self.create_subscription(
            OTWpntArray, '/avoid_waypoints', self.on_path, 10)
        self.stage = 'initial'
        self.passed = False
        self.failure = ''
        self.timer = self.create_timer(0.025, self.publish_inputs)

    @staticmethod
    def reference():
        """Create an ordered straight reference."""
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

    def publish_inputs(self):
        """Publish a narrow AABB, then widen its left envelope."""
        stamp = self.get_clock().now().to_msg()
        reference = self.reference()
        reference.header.stamp = stamp
        self.global_pub.publish(reference)

        obstacle = Obstacle()
        obstacle.id = 27
        obstacle.has_cartesian = True
        obstacle.is_static = True
        obstacle.is_visible = True
        obstacle.x_min = 6.8
        obstacle.x_max = 7.2
        obstacle.y_min = -0.2
        obstacle.y_max = 1.0 if self.stage == 'expanded' else 0.2
        obstacle.x_center = 0.5 * (obstacle.x_min + obstacle.x_max)
        obstacle.y_center = 0.5 * (obstacle.y_min + obstacle.y_max)
        obstacle.s_start = obstacle.x_min
        obstacle.s_end = obstacle.x_max
        obstacle.s_center = obstacle.x_center
        obstacle.d_right = obstacle.y_min
        obstacle.d_left = obstacle.y_max
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
        odometry.pose.pose.position.y = 0.0
        odometry.twist.twist.linear.x = 1.0
        self.odom_pub.publish(odometry)

    def on_path(self, message):
        """Wait for left commitment, then require a direct right replacement."""
        if (
                not message.wpnts or
                message.ot_line != 'raceline_local_d_offset_spline'):
            return
        peak = max(message.wpnts, key=lambda point: abs(point.d_m)).d_m
        if self.stage == 'initial':
            if peak <= 0.05:
                self.failure = f'initial symmetric obstacle did not choose left: d={peak:.3f}'
                return
            self.stage = 'expanded'
            return
        if peak >= -0.05:
            return
        self.passed = True


def main():
    """Run the probe against an already running local_planner_node."""
    rclpy.init()
    node = PreEngagementSwitchProbe()
    deadline = time.monotonic() + 8.0
    try:
        while (
                rclpy.ok() and time.monotonic() < deadline and
                not node.passed and not node.failure):
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.passed:
            print(
                'PASS: invalid left commitment switched directly to right '
                'before lateral engagement')
            return 0
        print(f'FAIL: {node.failure or "no pre-engagement side replacement"}')
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
