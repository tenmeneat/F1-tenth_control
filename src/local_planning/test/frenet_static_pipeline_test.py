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

"""Exercise the detector-provided Frenet-obstacle to Cartesian-path contract."""

import argparse
import csv
import math
import sys
import time

from f110_msgs.msg import Obstacle, ObstacleArray, OTWpntArray, Wpnt, WpntArray
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def load_waypoints(path):
    """Load the standard global_waypoints CSV."""
    result = []
    with open(path, newline='', encoding='utf-8') as stream:
        for row in csv.DictReader(stream):
            waypoint = Wpnt()
            waypoint.id = int(row['id'])
            waypoint.s_m = float(row['s'])
            waypoint.x_m = float(row['x_m'])
            waypoint.y_m = float(row['y_m'])
            waypoint.psi_rad = float(row['psi_rad'])
            waypoint.kappa_radpm = float(row['kappa_radpm'])
            waypoint.vx_mps = float(row['vx_mps'])
            waypoint.ax_mps2 = float(row['ax_mps2'])
            waypoint.d_left = float(row['d_left'])
            waypoint.d_right = float(row['d_right'])
            result.append(waypoint)
    return result


def make_straight_waypoints(count=300, spacing=0.2):
    """Build a self-contained straight reference when no CSV is supplied."""
    result = []
    for index in range(count):
        waypoint = Wpnt()
        waypoint.id = index
        waypoint.s_m = spacing * index
        waypoint.x_m = waypoint.s_m
        waypoint.y_m = 0.0
        waypoint.psi_rad = 0.0
        waypoint.kappa_radpm = 0.0
        waypoint.vx_mps = 2.0
        waypoint.d_left = 1.5
        waypoint.d_right = 1.5
        result.append(waypoint)
    return result


class FrenetPipelineProbe(Node):
    """Publish one detector-style Frenet obstacle and wait for a Cartesian path."""

    def __init__(self, waypoints):
        super().__init__('frenet_static_pipeline_probe')
        self.waypoints = waypoints
        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.global_pub = self.create_publisher(WpntArray, '/global_waypoints', latched)
        self.obstacle_pub = self.create_publisher(
            ObstacleArray, '/confirmed_static_obs', 10)
        self.odom_pub = self.create_publisher(Odometry, '/car_state/frenet/odom', 10)
        self.path_sub = self.create_subscription(
            OTWpntArray, '/avoid_waypoints', self.on_path, 10)
        self.passed = False
        self.failure = ''
        self.publish_count = 0
        self.path_sample_count = 0
        self.committed_peak_d = None
        self.saw_preparation = False

        self.ego_index = max(1, len(waypoints) // 8)
        self.obstacle_index = (self.ego_index + 12) % len(waypoints)
        self.timer = self.create_timer(0.05, self.publish_inputs)

    def publish_inputs(self):
        """Publish synchronized test inputs."""
        now = self.get_clock().now().to_msg()
        global_message = WpntArray()
        global_message.wpnts = self.waypoints
        self.global_pub.publish(global_message)

        reference = self.waypoints[self.obstacle_index]
        jitter = 0.01 if self.publish_count % 2 == 0 else -0.01
        self.publish_count += 1
        obstacle = Obstacle()
        obstacle.id = 1
        # Deliberately omit Cartesian metadata. A valid plan proves that local_planning consumes
        # the detector-owned Frenet footprint instead of reprojecting x/y.
        obstacle.has_cartesian = False
        obstacle.s_center = reference.s_m + jitter
        obstacle.s_start = obstacle.s_center - 0.20
        obstacle.s_end = obstacle.s_center + 0.20
        obstacle.d_center = 0.0
        obstacle.d_right = -0.20
        obstacle.d_left = 0.20
        obstacle.size = math.hypot(0.40, 0.40)
        obstacle.s_var = 0.0004
        obstacle.d_var = 0.0001
        obstacle.is_static = True
        obstacle.is_visible = True
        obstacle_message = ObstacleArray()
        obstacle_message.header.stamp = now
        obstacle_message.header.frame_id = 'map'
        obstacle_message.obstacles = [obstacle]
        self.obstacle_pub.publish(obstacle_message)

        ego = self.waypoints[self.ego_index]
        odometry = Odometry()
        odometry.header.stamp = now
        odometry.pose.pose.position.x = ego.s_m
        odometry.pose.pose.position.y = 0.0
        odometry.twist.twist.linear.x = max(0.5, ego.vx_mps)
        self.odom_pub.publish(odometry)

    def on_path(self, message):
        """Accept only a non-empty, finite Cartesian avoidance path."""
        if not message.wpnts:
            return
        if not all(
                math.isfinite(point.x_m) and math.isfinite(point.y_m) and
                math.isfinite(point.s_m) for point in message.wpnts):
            self.failure = 'path contains non-finite Cartesian coordinates'
            return
        if message.ot_line == 'raceline_static_prepare':
            self.saw_preparation = True
            if max(abs(point.d_m) for point in message.wpnts) > 1.0e-9:
                self.failure = 'preparation path unexpectedly moved laterally'
            return
        if max(abs(point.d_m) for point in message.wpnts) < 0.05:
            self.failure = 'path did not move laterally around the obstacle'
            return
        if message.ot_line != 'raceline_local_d_offset_spline':
            self.failure = f'unexpected path mode during Frenet jitter: {message.ot_line}'
            return
        peak_d = max(message.wpnts, key=lambda point: abs(point.d_m)).d_m
        if self.committed_peak_d is None:
            self.committed_peak_d = peak_d
        elif not math.isclose(peak_d, self.committed_peak_d, abs_tol=1.0e-9):
            self.failure = (
                'committed path changed under 1 cm same-ID Frenet jitter: '
                f'{self.committed_peak_d:.6f} -> {peak_d:.6f}')
            return
        self.path_sample_count += 1
        self.passed = self.path_sample_count >= 10


def main():
    """Run the probe against an already running local_planner_node."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--waypoints-csv')
    parser.add_argument('--timeout', type=float, default=8.0)
    arguments = parser.parse_args()

    waypoints = (
        load_waypoints(arguments.waypoints_csv)
        if arguments.waypoints_csv else make_straight_waypoints())
    if len(waypoints) < 30:
        print('FAIL: at least 30 waypoints are required')
        return 2

    rclpy.init()
    node = FrenetPipelineProbe(waypoints)
    deadline = time.monotonic() + arguments.timeout
    try:
        while rclpy.ok() and time.monotonic() < deadline and not node.passed:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.passed:
            print(
                'PASS: preparation preceded a same-ID detector-Frenet commitment '
                'and the uncertainty Guard kept it fixed for 10 output cycles')
            return 0
        print(f'FAIL: {node.failure or "no non-empty avoidance path received"}')
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
