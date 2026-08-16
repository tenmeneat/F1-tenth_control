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

"""Verify avoidance -> latched safe-stop -> delayed avoidance across ROS nodes."""

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
    """Return the transient-local QoS used by global waypoints."""
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return qos


class SafeStopLatchProbe(Node):
    """Drive one same-ID obstacle through feasible and blocked envelopes."""

    def __init__(self):
        super().__init__('safe_stop_latch_pipeline_probe')
        self.global_pub = self.create_publisher(
            WpntArray, '/global_waypoints', latched_qos())
        self.obstacle_pub = self.create_publisher(
            ObstacleArray, '/confirmed_static_obs', 10)
        self.odom_pub = self.create_publisher(
            Odometry, '/car_state/frenet/odom', 10)
        self.avoid_sub = self.create_subscription(
            OTWpntArray, '/avoid_waypoints', self.on_avoid, 10)
        self.state_sub = self.create_subscription(
            StateMachine, '/state', self.on_state, 10)
        self.local_sub = self.create_subscription(
            WpntArray, '/local_waypoints', self.on_local, 10)

        self.stage = 'initial'
        self.stage_started = time.monotonic()
        self.safe_stop_started = None
        self.safe_outputs_after_clear = 0
        self.state = StateMachine.STATE_GLOBAL
        self.committed_left = None
        self.saw_local_safe_stop = False
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

    def publish_inputs(self):
        """Publish the reference, fixed ego, and current obstacle envelope."""
        now = self.get_clock().now().to_msg()
        reference = self.reference()
        reference.header.stamp = now
        self.global_pub.publish(reference)

        obstacle = Obstacle()
        obstacle.id = 17
        obstacle.has_cartesian = True
        obstacle.is_static = True
        obstacle.is_visible = True
        obstacle.x_center = 7.0
        obstacle.y_center = 0.0
        obstacle.x_min = 6.8
        obstacle.x_max = 7.2
        obstacle.y_min = (
            -1.0 if self.stage == 'blocked' and not self.committed_left else -0.2)
        obstacle.y_max = (
            1.0 if self.stage == 'blocked' and self.committed_left else 0.2)
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
        obstacles.header.stamp = now
        obstacles.header.frame_id = 'map'
        obstacles.obstacles = [obstacle]
        self.obstacle_pub.publish(obstacles)

        odometry = Odometry()
        odometry.header.stamp = now
        odometry.header.frame_id = 'map'
        odometry.child_frame_id = '0'
        odometry.pose.pose.position.x = 0.0
        # Enter the selected maneuver before widening its side of the AABB. This verifies that the
        # planner locks direction after engagement and therefore safe-stops instead of reversing.
        if self.stage == 'initial':
            odometry.pose.pose.position.y = 0.0
        else:
            odometry.pose.pose.position.y = 0.15 if self.committed_left else -0.15
        odometry.twist.twist.linear.x = 1.0
        self.odom_pub.publish(odometry)

    def on_state(self, message):
        """Track the state-machine output."""
        self.state = message.state
        if self.safe_stop_started is not None and self.stage != 'released':
            if self.state != StateMachine.STATE_AVOID:
                self.failure = 'state machine left AVOID while safe-stop was latched'

    def on_local(self, message):
        """Confirm waypoint selection keeps a non-empty local stop path."""
        if (
                self.safe_stop_started is not None and message.wpnts and
                min(point.vx_mps for point in message.wpnts) <= 1.0e-9):
            self.saw_local_safe_stop = True

    def on_avoid(self, message):
        """Advance stages and enforce release hysteresis."""
        now = time.monotonic()
        if not message.wpnts:
            return
        if self.stage == 'initial':
            if (
                    message.ot_line == 'raceline_local_d_offset_spline' and
                    self.state == StateMachine.STATE_AVOID):
                positive = max(point.d_m for point in message.wpnts)
                negative = min(point.d_m for point in message.wpnts)
                self.committed_left = positive >= abs(negative)
                self.stage = 'blocked'
                self.stage_started = now
            return
        if self.stage == 'blocked':
            if message.ot_line == 'raceline_static_safe_stop':
                self.safe_stop_started = now
                self.stage = 'clear'
                self.stage_started = now
            return
        if self.stage != 'clear':
            return
        if message.ot_line == 'raceline_static_safe_stop':
            self.safe_outputs_after_clear += 1
            return
        if message.ot_line != 'raceline_local_d_offset_spline':
            self.failure = f'unexpected release mode: {message.ot_line}'
            return
        elapsed = now - self.safe_stop_started
        if elapsed < 0.14 or self.safe_outputs_after_clear < 5:
            self.failure = (
                'safe-stop released without consecutive-plan hysteresis: '
                f'{elapsed:.3f}s, {self.safe_outputs_after_clear} stop outputs')
            return
        if not self.saw_local_safe_stop:
            self.failure = 'wpnt_publisher did not forward the safe-stop path'
            return
        self.stage = 'released'
        self.passed = True


def main():
    """Run the probe against local_planner, state_machine, and wpnt_publisher."""
    rclpy.init()
    node = SafeStopLatchProbe()
    deadline = time.monotonic() + 12.0
    try:
        while (
                rclpy.ok() and time.monotonic() < deadline and
                not node.passed and not node.failure):
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.passed:
            print(
                'PASS: safe-stop stayed local/AVOID and released only after '
                f'{node.safe_outputs_after_clear} stop outputs')
            return 0
        print(f'FAIL: {node.failure or "pipeline did not complete all stages"}')
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
