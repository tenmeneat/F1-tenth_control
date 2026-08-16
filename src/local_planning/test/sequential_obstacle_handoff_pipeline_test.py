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

"""Verify direct AVOID chaining when a second obstacle appears before or during handoff."""

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


class SequentialObstacleProbe(Node):
    """Force the first maneuver left and its immediate successor right."""

    def __init__(self, during_handoff):
        super().__init__('sequential_obstacle_handoff_probe')
        self.during_handoff = during_handoff
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
        self.first_merge_s = None
        self.second_s = None
        self.second_first_publish_time = None
        self.outputs_after_first = 0
        self.saw_second_prepare = False
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
    def obstacle(obstacle_id, center_s, d_right, d_left):
        """Build a detector-style Frenet obstacle for the straight reference."""
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
        obstacle.radius = 0.5 * math.hypot(
            obstacle.x_max - obstacle.x_min,
            obstacle.y_max - obstacle.y_min)
        obstacle.size = 2.0 * obstacle.radius
        return obstacle

    def publish_inputs(self):
        """Publish a continuous, fresh input stream."""
        stamp = self.get_clock().now().to_msg()
        reference = self.reference()
        reference.header.stamp = stamp
        self.global_pub.publish(reference)

        obstacles = ObstacleArray()
        obstacles.header.stamp = stamp
        obstacles.header.frame_id = 'map'
        # This box blocks the right side, so the first committed maneuver must go left.
        obstacles.obstacles.append(self.obstacle(31, 7.0, -1.2, 0.2))
        if self.stage == 'chaining':
            # This later box blocks the left side. A stale left commitment cannot avoid it.
            obstacles.obstacles.append(
                self.obstacle(32, self.second_s, -0.2, 1.2))
            if self.second_first_publish_time is None:
                self.second_first_publish_time = time.monotonic()
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
        state.state = StateMachine.STATE_AVOID
        self.state_pub.publish(state)

    @staticmethod
    def merge_s(message):
        """Recover merge s from the configured 5 m post-merge controller tail."""
        if not message.wpnts:
            return None
        return message.wpnts[-1].s_m - 5.0

    def on_avoid(self, message):
        """Require left avoidance -> stabilized direct chain -> right avoidance."""
        if not message.wpnts:
            if self.stage != 'first':
                self.failure = 'avoid waypoints became empty while chaining maneuvers'
            return

        if self.stage == 'first':
            if message.ot_line != 'raceline_local_d_offset_spline':
                return
            if max(waypoint.d_m for waypoint in message.wpnts) < 0.4:
                self.failure = 'first obstacle did not produce the required left maneuver'
                return
            self.first_merge_s = self.merge_s(message)
            if self.first_merge_s is None:
                self.failure = 'could not locate the first maneuver merge point'
                return
            self.second_s = self.first_merge_s + 5.0
            self.ego_s = self.first_merge_s
            self.stage = 'waiting_handoff' if self.during_handoff else 'chaining'
            self.get_logger().info(
                f'holding ego at first merge s={self.first_merge_s:.2f}; '
                f'second obstacle s={self.second_s:.2f}')
            return

        self.outputs_after_first += 1
        if self.stage == 'waiting_handoff':
            if message.ot_line == 'raceline_global_handoff':
                self.stage = 'chaining'
            return
        if message.ot_line == 'raceline_global_handoff':
            self.failure = 'global handoff was published before the second obstacle maneuver'
            return
        if message.ot_line == 'raceline_static_prepare':
            self.saw_second_prepare = True
            return
        if message.ot_line != 'raceline_local_d_offset_spline':
            return
        if min(waypoint.d_m for waypoint in message.wpnts) > -0.4:
            # The first commitment remains valid while merge confirmation accumulates.
            return
        if self.second_first_publish_time is None:
            self.failure = 'second maneuver committed before its obstacle was published'
            return
        stabilization_elapsed = time.monotonic() - self.second_first_publish_time
        if stabilization_elapsed < 0.15:
            self.failure = (
                'second maneuver skipped the 0.15 s cluster stabilization '
                f'({stabilization_elapsed:.3f} s)')
            return
        self.passed = True


def main():
    """Run the probe against a fresh local_planner_node."""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--during-handoff', action='store_true',
        help='introduce the second obstacle only after global handoff starts')
    arguments = parser.parse_args()
    rclpy.init()
    node = SequentialObstacleProbe(arguments.during_handoff)
    deadline = time.monotonic() + 12.0
    try:
        while (
                rclpy.ok() and time.monotonic() < deadline and
                not node.passed and not node.failure):
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.passed:
            timing = 'during handoff' if arguments.during_handoff else 'before handoff'
            transition = (
                'through preparation' if node.saw_second_prepare
                else 'directly from the active avoidance')
            print(
                f'PASS: first-left avoidance chained {transition} to '
                f'second-right avoidance {timing} after stabilization over '
                f'{node.outputs_after_first} outputs')
            return 0
        print(f'FAIL: {node.failure or "sequential maneuver did not complete"}')
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
