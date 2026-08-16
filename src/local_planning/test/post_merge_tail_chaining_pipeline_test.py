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

"""Verify chaining for an obstacle near either side of the first maneuver merge."""

import argparse
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


class PostMergeTailProbe(Node):
    """Add a second obstacle just before the merge or inside its controller-only tail."""

    def __init__(self, before_merge):
        super().__init__('post_merge_tail_chaining_probe')
        self.before_merge = before_merge
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

        self.stage = 'first'
        self.ego_s = 0.0
        self.ego_d = 0.0
        self.second_s = None
        self.first_path_end = None
        self.outputs_with_second = 0
        self.passed = False
        self.failure = ''
        self.timer = self.create_timer(0.025, self.publish_inputs)

    @staticmethod
    def reference():
        """Create an ordered straight race line."""
        message = WpntArray()
        message.header.frame_id = 'map'
        for index in range(400):
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
    def obstacle(obstacle_id, center_s, d_right=-1.2, d_left=0.1):
        """Build a right-side box that requires a left maneuver."""
        obstacle = Obstacle()
        obstacle.id = obstacle_id
        obstacle.has_cartesian = True
        obstacle.is_static = True
        obstacle.is_visible = True
        obstacle.x_center = center_s
        obstacle.y_center = 0.5 * (d_right + d_left)
        obstacle.x_min = center_s - 0.2
        obstacle.x_max = center_s + 0.2
        obstacle.y_min = d_right
        obstacle.y_max = d_left
        obstacle.s_start = center_s - 0.2
        obstacle.s_end = center_s + 0.2
        obstacle.s_center = center_s
        obstacle.d_right = d_right
        obstacle.d_left = d_left
        obstacle.d_center = 0.5 * (d_right + d_left)
        obstacle.radius = 0.5 * math.hypot(0.4, d_left - d_right)
        obstacle.size = 2.0 * obstacle.radius
        return obstacle

    @staticmethod
    def path_d_at(message, target_s):
        """Return the d of the path sample closest to target s."""
        return min(message.wpnts, key=lambda waypoint: abs(
            waypoint.s_m - target_s)).d_m

    def publish_inputs(self):
        """Publish fresh localization and obstacle observations."""
        stamp = self.get_clock().now().to_msg()
        reference = self.reference()
        reference.header.stamp = stamp
        self.global_pub.publish(reference)

        obstacles = ObstacleArray()
        obstacles.header.stamp = stamp
        obstacles.header.frame_id = 'map'
        obstacles.obstacles.append(self.obstacle(51, 7.0))
        if self.stage == 'chaining':
            second_d_left = -0.21 if self.before_merge else 0.1
            obstacles.obstacles.append(
                self.obstacle(52, self.second_s, d_left=second_d_left))
        self.obstacle_pub.publish(obstacles)

        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = 'map'
        odometry.pose.pose.position.x = self.ego_s
        odometry.pose.pose.position.y = self.ego_d
        odometry.twist.twist.linear.x = 2.0
        self.odom_pub.publish(odometry)

        state = StateMachine()
        state.header.stamp = stamp
        state.header.frame_id = 'map'
        state.state = StateMachine.STATE_AVOID
        self.state_pub.publish(state)

    def on_avoid(self, message):
        """Require first avoidance to be replaced directly by the second."""
        if not message.wpnts:
            if self.stage == 'chaining':
                self.failure = 'avoid waypoints became empty while chaining'
            return

        if self.stage == 'first':
            if message.ot_line != 'raceline_local_d_offset_spline':
                return
            if max(waypoint.d_m for waypoint in message.wpnts) < 0.35:
                self.failure = 'first obstacle did not produce a left avoidance'
                return
            first_merge = message.wpnts[-1].s_m - 5.0
            self.first_path_end = message.wpnts[-1].s_m
            offset = -1.45 if self.before_merge else 1.0
            self.second_s = first_merge + offset
            self.ego_s = 7.8 if self.before_merge else 8.5
            self.ego_d = self.path_d_at(message, self.ego_s)
            self.stage = 'chaining'
            position = (
                'before the first merge' if self.before_merge
                else 'inside the first controller tail')
            self.get_logger().info(
                f'inserted obstacle 52 at s={self.second_s:.2f} {position}; '
                f'ego d={self.ego_d:.2f}')
            return

        self.outputs_with_second += 1
        if message.ot_line in (
                'raceline_static_safe_stop',
                'raceline_static_prepare',
                'raceline_global_handoff'):
            self.failure = (
                f'unexpected {message.ot_line} instead of direct chaining')
            return
        if message.ot_line != 'raceline_local_d_offset_spline':
            return
        if message.wpnts[-1].s_m <= self.first_path_end + 1.0:
            return
        if abs(message.wpnts[0].d_m - self.ego_d) > 0.10:
            self.failure = 'next maneuver was discontinuous from the current ego d'
            return
        if max(waypoint.d_m for waypoint in message.wpnts) < 0.35:
            self.failure = 'next maneuver did not avoid obstacle 52 on the left'
            return
        self.passed = True


def main():
    """Run the probe against a fresh local_planner_node."""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--before-merge', action='store_true',
        help='place the next obstacle one metre before the old merge')
    arguments = parser.parse_args()
    rclpy.init()
    node = PostMergeTailProbe(arguments.before_merge)
    deadline = time.monotonic() + 10.0
    try:
        while (
                rclpy.ok() and time.monotonic() < deadline and
                not node.passed and not node.failure):
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.passed:
            position = 'pre-merge' if arguments.before_merge else 'post-merge-tail'
            print(
                f'PASS: {position} obstacle replaced the first commitment '
                f'directly after {node.outputs_with_second} outputs')
            return 0
        print(f'FAIL: {node.failure or "post-merge chaining did not complete"}')
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
