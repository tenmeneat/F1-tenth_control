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

"""Verify the live /scan -> /static_obs -> /avoid_waypoints pipeline."""

import math
import sys
import time

from f110_msgs.msg import ObstacleArray, OTWpntArray, Wpnt, WpntArray
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


TRACK_RADIUS = 8.0
OBSTACLE_ANGLE = 0.50
BEAM_COUNT = 1080
FIELD_OF_VIEW = 4.70
ANGLE_MIN = -0.5 * FIELD_OF_VIEW
ANGLE_INCREMENT = FIELD_OF_VIEW / BEAM_COUNT


def latched_qos():
    """Return the QoS used by the global path and map."""
    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return qos


class StaticObstaclePipelineProbe(Node):
    """Publish one physical static obstacle and validate both pipeline stages."""

    def __init__(self):
        super().__init__('static_obs_pipeline_probe')
        self.global_pub = self.create_publisher(
            WpntArray, '/global_waypoints', latched_qos())
        self.map_pub = self.create_publisher(
            OccupancyGrid, '/map', latched_qos())
        self.pose_pub = self.create_publisher(Odometry, '/pf/pose/odom', 10)
        self.frenet_pub = self.create_publisher(
            Odometry, '/car_state/frenet/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.static_tf = StaticTransformBroadcaster(self)

        self.static_sub = self.create_subscription(
            ObstacleArray, '/static_obs', self.on_static_obstacles, 10)
        self.path_sub = self.create_subscription(
            OTWpntArray, '/avoid_waypoints', self.on_avoidance_path, 10)
        self.static_marker_sub = self.create_subscription(
            MarkerArray, '/static_obs/markers', self.on_static_markers, 10)
        self.saw_valid_static = False
        self.saw_valid_static_marker = False
        self.saw_valid_path = False
        self.failure = ''

        self.publish_reference_inputs()
        self.timer = self.create_timer(0.025, self.publish_sensor_inputs)

    def publish_reference_inputs(self):
        """Publish a closed circular reference and an obstacle-free map."""
        waypoints = WpntArray()
        waypoints.header.frame_id = 'map'
        waypoint_count = 360
        for index in range(waypoint_count):
            angle = 2.0 * math.pi * index / waypoint_count
            waypoint = Wpnt()
            waypoint.id = index
            waypoint.s_m = TRACK_RADIUS * angle
            waypoint.x_m = TRACK_RADIUS * math.cos(angle)
            waypoint.y_m = TRACK_RADIUS * math.sin(angle)
            waypoint.psi_rad = angle + 0.5 * math.pi
            waypoint.kappa_radpm = 1.0 / TRACK_RADIUS
            waypoint.d_left = 2.0
            waypoint.d_right = 2.0
            waypoint.vx_mps = 2.0
            waypoints.wpnts.append(waypoint)
        self.global_pub.publish(waypoints)

        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.resolution = 0.05
        grid.info.width = 600
        grid.info.height = 600
        grid.info.origin.position.x = -15.0
        grid.info.origin.position.y = -15.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * (grid.info.width * grid.info.height)
        self.map_pub.publish(grid)

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'laser'
        transform.transform.translation.x = TRACK_RADIUS
        transform.transform.rotation.z = math.sin(0.25 * math.pi)
        transform.transform.rotation.w = math.cos(0.25 * math.pi)
        self.static_tf.sendTransform(transform)

    @staticmethod
    def obstacle_points():
        """Return the visible faces of a 0.5 m square in the laser frame."""
        map_x = TRACK_RADIUS * math.cos(OBSTACLE_ANGLE)
        map_y = TRACK_RADIUS * math.sin(OBSTACLE_ANGLE)
        delta_x = map_x - TRACK_RADIUS
        delta_y = map_y
        center_x = delta_y
        center_y = -delta_x
        half = 0.25
        points = []
        for index in range(80):
            ratio = index / 79.0
            points.append((center_x - half, center_y - half + 2.0 * half * ratio))
            points.append((center_x - half + 2.0 * half * ratio, center_y - half))
        return points

    def publish_sensor_inputs(self):
        """Publish ego states and a scan containing the static obstacle."""
        stamp = self.get_clock().now().to_msg()

        pose = Odometry()
        pose.header.stamp = stamp
        pose.header.frame_id = 'map'
        pose.child_frame_id = 'base_link'
        pose.pose.pose.position.x = TRACK_RADIUS
        pose.pose.pose.orientation.z = math.sin(0.25 * math.pi)
        pose.pose.pose.orientation.w = math.cos(0.25 * math.pi)
        self.pose_pub.publish(pose)

        frenet = Odometry()
        frenet.header.stamp = stamp
        frenet.header.frame_id = 'map'
        frenet.pose.pose.position.x = 0.0
        frenet.pose.pose.position.y = 0.0
        frenet.twist.twist.linear.x = 1.0
        self.frenet_pub.publish(frenet)

        ranges = [float('inf')] * BEAM_COUNT
        for point_x, point_y in self.obstacle_points():
            angle = math.atan2(point_y, point_x)
            index = int(round((angle - ANGLE_MIN) / ANGLE_INCREMENT))
            if 0 <= index < BEAM_COUNT:
                distance = math.hypot(point_x, point_y)
                ranges[index] = min(ranges[index], distance)

        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = 'laser'
        scan.angle_min = ANGLE_MIN
        scan.angle_max = ANGLE_MIN + FIELD_OF_VIEW
        scan.angle_increment = ANGLE_INCREMENT
        scan.range_min = 0.1
        scan.range_max = 30.0
        scan.ranges = ranges
        self.scan_pub.publish(scan)

    def on_static_obstacles(self, message):
        """Require detector output with valid Cartesian metadata and Frenet planning bounds."""
        for obstacle in message.obstacles:
            cartesian_values = (
                obstacle.x_center,
                obstacle.y_center,
                obstacle.radius,
                obstacle.x_min,
                obstacle.x_max,
                obstacle.y_min,
                obstacle.y_max,
            )
            frenet_values = (
                obstacle.s_center,
                obstacle.s_start,
                obstacle.s_end,
                obstacle.d_center,
                obstacle.d_right,
                obstacle.d_left,
            )
            if (
                    obstacle.is_static and obstacle.is_visible and
                    obstacle.has_cartesian and obstacle.radius > 0.0 and
                    obstacle.x_min <= obstacle.x_max and
                    obstacle.y_min <= obstacle.y_max and
                    obstacle.d_right <= obstacle.d_left and
                    math.hypot(
                        obstacle.x_max - obstacle.x_min,
                        obstacle.y_max - obstacle.y_min) > 0.0 and
                    all(math.isfinite(value) for value in cartesian_values) and
                    all(math.isfinite(value) for value in frenet_values)):
                self.saw_valid_static = True

    def on_static_markers(self, message):
        """Require a map-frame Frenet boundary mirroring the static obstacle input."""
        self.saw_valid_static_marker = (
            self.saw_valid_static_marker
            or any(
                marker.action == Marker.ADD
                and marker.type == Marker.LINE_STRIP
                and marker.header.frame_id == 'map'
                and len(marker.points) >= 5
                and all(
                    math.isfinite(point.x)
                    and math.isfinite(point.y)
                    and math.isfinite(point.z)
                    for point in marker.points)
                for marker in message.markers)
        )

    def on_avoidance_path(self, message):
        """Require a finite path that actually moves off the global race line."""
        if not self.saw_valid_static or not message.wpnts:
            return
        if not all(
                math.isfinite(point.x_m) and math.isfinite(point.y_m) and
                math.isfinite(point.s_m) and math.isfinite(point.d_m)
                for point in message.wpnts):
            self.failure = 'avoidance path contains a non-finite value'
            return
        if message.ot_line == 'raceline_static_prepare':
            return
        if max(abs(point.d_m) for point in message.wpnts) <= 0.05:
            self.failure = 'local planner did not move laterally around /static_obs'
            return
        self.saw_valid_path = True


def main():
    """Run the probe against active detector and local-planner nodes."""
    rclpy.init()
    node = StaticObstaclePipelineProbe()
    deadline = time.monotonic() + 10.0
    try:
        while (
                rclpy.ok() and time.monotonic() < deadline and
                not (node.saw_valid_path and node.saw_valid_static_marker)):
            rclpy.spin_once(node, timeout_sec=0.1)
        if (
                node.saw_valid_static and node.saw_valid_static_marker
                and node.saw_valid_path):
            print(
                'PASS: /scan -> /static_obs + matching Frenet marker '
                '-> /avoid_waypoints is valid')
            return 0
        if not node.saw_valid_static:
            print('FAIL: detector did not publish a valid Cartesian /static_obs')
        elif not node.saw_valid_static_marker:
            print('FAIL: detector did not publish a matching /static_obs/markers boundary')
        else:
            print(f'FAIL: {node.failure or "no lateral /avoid_waypoints received"}')
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
