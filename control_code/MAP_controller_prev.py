import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from f110_msgs.msg import WpntArray, Wpnt
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import Point, PointStamped, Pose, PoseWithCovarianceStamped
from std_msgs.msg import String, Float64
from visualization_msgs.msg import Marker, MarkerArray
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterValue, ParameterType, ParameterDescriptor, FloatingPointRange, IntegerRange
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from pandas import read_csv
try:
    import tf2_ros
except Exception:
    tf2_ros = None


import threading
import time
import logging
import numpy as np
from scipy.spatial.transform import Rotation
from steering_lookup.lookup_steer_angle import LookupSteerAngle
from frenet_conversion.frenet_converter import FrenetConverter


class MAPController(Node):
    def __init__(self):
        super().__init__('map_controller')

        # use proper logging functions (avoid trailing comma creating tuple)
        self.logger_info = self.get_logger().info
        self.logger_warn = self.get_logger().warning

        # TF / transform listener (optional - guard when tf2_ros not available)
        self.tf_buffer = None
        self.tf_listener = None
        if tf2_ros is not None:
            try:
                self.tf_buffer = tf2_ros.Buffer()
                self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            except Exception:
                # leave buffer as None if tf2_ros cannot be initialized in this env
                self.tf_buffer = None

        # EKF odometry placeholder (may be set by another subscriber)
        self.ekf_odom = None

        # state machine rate parameter (used by map_cycle safety logic)
        self.state_machine_rate = self.declare_parameter("state_machine_rate", 1.0).get_parameter_value().double_value

        # control timer handle (create once when ready)
        self.control_timer = None

        #declare parameters(*launch files can override these)
        self.rate = 40 # rate in hertz
        self.state = "GB_TRACK"
        self.sim = self.declare_parameter("sim", True).get_parameter_value().bool_value
        self.scan = None
        self.LUT_name = self.declare_parameter("LU_table", "NUC6_glc_pacejka").get_parameter_value().string_value
        self.steer_lookup = LookupSteerAngle(self.LUT_name, self.logger_info)
        self.t_clip_min = self.declare_parameter("t_clip_min", 1.5).get_parameter_value().double_value
        self.t_clip_max = self.declare_parameter("t_clip_max", 5.0).get_parameter_value().double_value
        self.m_l1 = self.declare_parameter("l1_distance", 0.3).get_parameter_value().double_value
        self.q_l1 = self.declare_parameter("l1_gain", 0.5).get_parameter_value().double_value
        self.speed_lookahead = self.declare_parameter("speed_lookahead", 0.15).get_parameter_value().double_value
        self.lat_err_coeff = self.declare_parameter("lateral_error_coeff", 1.0).get_parameter_value().double_value
        self.acc_scaler_for_steer = self.declare_parameter("acceleration_scaler_for_steering", 1.2).get_parameter_value().double_value
        self.dec_scaler_for_steer = self.declare_parameter("deceleration_scaler_for_steering", 0.9).get_parameter_value().double_value
        self.start_scale_speed = self.declare_parameter("start_scale_speed", 7.0).get_parameter_value().double_value
        self.end_scale_speed = self.declare_parameter("end_scale_speed", 8.0).get_parameter_value().double_value
        self.downscale_factor = self.declare_parameter("downscale_factor", 0.10).get_parameter_value().integer_value
        self.speed_lookahead_for_steer = self.declare_parameter("speed_lookahead_for_steering", 0.0).get_parameter_value().double_value
        self.lateral_error_list = [] # list of squared lateral error 
        self.curr_steering_angle = 0
        self.idx_nearest_waypoint = None # index of nearest waypoint to car
        self.track_length = None
        self.waypoint_array_in_map = None
        self.speed_now = None
        self.position_in_map = None
        self.position_in_map_frenet = None
        self.waypoint_safety_counter = 0
        self.gap = None
        self.gap_should = None
        self.gap_error = None
        self.gap_actual = None
        self.v_diff = None
        self.i_gap = 0
        self.speed_command = None
        self.curvature_waypoints = 0
        self.d_vs = np.zeros(10)
        self.acceleration_command = 0

        self.get_logger().info(f"Using LUT: {self.LUT_name}")

        #declare publishers
        self.publish_topic = '/drive'
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.publish_topic, 10)
        self.steering_pub = self.create_publisher(Marker, 'steering', 10)
        self.lookahead_pub = self.create_publisher(Marker, 'lookahead_point', 10)
        self.waypoint_pub = self.create_publisher(MarkerArray, 'my_waypoints', 10)
        self.l1_pub = self.create_publisher(Point, 'l1_distance', 10)
        
        # buffers for improved computation
        self.waypoint_array_buf = MarkerArray()
        self.markers_buf = [Marker() for _ in range(1000)]

        #declare subscribers
        self.create_subscription(String,'/state',  self.state_cb, 10)
        qos_gl = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,)
        self.create_subscription(WpntArray,'/global_waypoints',  self.track_length_cb, qos_gl)
        self.create_subscription(WpntArray,'/local_waypoints', self.local_waypoint_cb, 10) # waypoints (x, y, v, norm trackbound, s, kappa)
        self.create_subscription(Odometry,'/odom',  self.odom_cb, 10) # car speed
        self.create_subscription(Odometry,'/pf/pose/odom',  self.car_state_cb, 10) # car position (x, y, theta)
        self.create_subscription(Odometry, '/car_state/frenet/odom',self.car_state_frenet_cb, 10) # car frenet coordinates
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10) # lidar scan
        self.create_subscription(Imu,'/sensors/imu/raw', self.imu_cb, 10) # acceleration subscriber for steer change
        self.acc_now = np.zeros(10) # rolling buffer for acceleration


        self.get_logger().info("Controller ready")

    def car_state_cb(self, data: Odometry):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        rot = Rotation.from_quat([data.pose.pose.orientation.x, data.pose.pose.orientation.y,
                                    data.pose.pose.orientation.z, data.pose.pose.orientation.w])
        rot_euler = rot.as_euler('xyz', degrees=False)
        theta = rot_euler[2]
        self.position_in_map = np.array([x, y, theta])[np.newaxis]
        
    def scan_cb(self, data: LaserScan):
        
        self.scan = data

    def track_length_cb(self, data:WpntArray):
        self.track_length = data.wpnts[-1].s_m
        self.waypoints = np.array([[wpnt.x_m, wpnt.y_m, wpnt.psi_rad] for wpnt in data.wpnts])

    def odom_cb(self, raw: Odometry):
        # Use TF buffer if available to check transforms, but don't fail if it's missing
        carstate_odom_msg = Odometry()
        if getattr(self, 'tf_buffer', None) is not None:
            try:
                # short timeout; if lookup fails we continue using incoming raw odom
                self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time(), rclpy.duration.Duration(seconds=0.5))
            except Exception as e:
                self.get_logger().warning(f"tf lookup failed: {e}")

        # Prefer EKF odom if available, otherwise use raw odom provided by topic
        if getattr(self, 'ekf_odom', None) is not None:
            carstate_odom_msg.twist.twist = self.ekf_odom.twist.twist
        else:
            carstate_odom_msg.twist.twist = raw.twist.twist

        self.speed_now = carstate_odom_msg.twist.twist.linear.x


    def local_waypoint_cb(self, data: WpntArray):
        self.waypoint_list_in_map = []
        for waypoint in data.wpnts:
            waypoint_in_map = [waypoint.x_m, waypoint.y_m]
            speed = waypoint.vx_mps
            if waypoint.d_right + waypoint.d_left != 0:
                self.waypoint_list_in_map.append([waypoint_in_map[0],
                                                waypoint_in_map[1],
                                                speed,
                                                min(waypoint.d_left, waypoint.d_right)/(waypoint.d_right + waypoint.d_left),
                                                waypoint.s_m, waypoint.kappa_radpm, waypoint.psi_rad, waypoint.ax_mps2]
                                                )
            else:
                self.waypoint_list_in_map.append([waypoint_in_map[0], waypoint_in_map[1], speed, 0, waypoint.s_m, waypoint.kappa_radpm, waypoint.psi_rad, waypoint.ax_mps2])
        self.waypoint_array_in_map = np.array(self.waypoint_list_in_map)
        self.waypoint_safety_counter = 0

    def imu_cb(self, data):
        # save acceleration in a rolling buffer
        self.acc_now[1:] = self.acc_now[:-1]
        self.acc_now[0] = -data.linear_acceleration.y # vesc is rotated 90 deg, so (-acc_y) == (long_acc)

    def car_state_frenet_cb(self, raw: Odometry):
        # Defensive handling: ensure converter and timer created only once and only when waypoints exist
        odom_pos = raw.pose.pose.position
        odom_quat = raw.pose.pose.orientation
        odom_vel = raw.twist.twist.linear
        theta = euler_from_quaternion([odom_quat.x, odom_quat.y, odom_quat.z, odom_quat.w])[2]
        self.get_logger().debug(f"odom_pos: ({odom_pos.x}, {odom_pos.y}), theta: {theta}")

        # Ensure converter exists; waypoints come from track_length_cb
        if getattr(self, 'converter', None) is None:
            if getattr(self, 'waypoints', None) is None:
                self.get_logger().warning('FrenetConverter: waypoints not available yet')
                return
            try:
                self.converter = FrenetConverter(self.waypoints[:, 0], self.waypoints[:, 1], self.waypoints[:, 2])
            except Exception as e:
                self.get_logger().error(f'Failed to create FrenetConverter: {e}')
                return

        # create control timer only once
        if self.control_timer is None:
            try:
                self.control_timer = self.create_timer(1.0 / float(self.rate), self.control_loop)
            except Exception as e:
                self.get_logger().warning(f'Failed to create control timer: {e}')

        try:
            frenet_pos = self.converter.get_frenet([odom_pos.x], [odom_pos.y])
            frenet_vel = self.converter.get_frenet_velocities(odom_vel.x, odom_vel.y, theta)

            frenet_msg = raw
            idx_arr = self.converter.get_closest_index([odom_pos.x], [odom_pos.y])
            idx = str(int(idx_arr[0])) if hasattr(idx_arr, '__len__') else str(int(idx_arr))
            frenet_msg.child_frame_id = idx
            s = odom_pos.x
            d = odom_pos.y
            vs = odom_vel.x
            vd = odom_vel.y
            self.position_in_map_frenet = np.array([s, d, vs, vd])
        except Exception as e:
            self.get_logger().warning(f'Error computing frenet state: {e}')
            return

    def state_cb(self, data):
        self.state = data.data

    def map_cycle(self):
        if self.position_in_map is None or self.waypoint_array_in_map is None or self.speed_now is None:
            self.get_logger().debug("Waiting for initialization data (position_in_map, waypoint_array_in_map, or speed_now)")
            return 0.0, 0.0  # return safe defaults: (speed, steer)
  
        # Call local main_loop (self.map_controller was not defined previously)
        speed, acceleration, jerk, steering_angle, L1_point, L1_distance, idx_nearest_waypoint = self.main_loop(
            self.state,
            self.position_in_map,
            self.waypoint_array_in_map,
            self.speed_now,
            "",
            self.position_in_map_frenet,
            self.acc_now,
            self.track_length,
        )
        self.set_lookahead_marker(L1_point, 100)
        self.visualize_steering(steering_angle)
        self.l1_pub.publish(Point(x=float(idx_nearest_waypoint), y=L1_distance))
        
        self.waypoint_safety_counter += 1
        if self.state_machine_rate is None:
            self.state_machine_rate = 1.0
        if self.waypoint_safety_counter >= self.rate / self.state_machine_rate * 10:  # we can use the same waypoints for 5 cycles
            self.get_logger().warning("[controller_manager] Received no local wpnts. STOPPING!!")
            speed = 0
            steering_angle = 0
        # If there is an external controller object with flag1, try to clear it; otherwise ignore
        if getattr(self, 'map_controller', None) is not None:
            try:
                self.map_controller.flag1 = False
            except Exception:
                pass
        self.get_logger().info(f"[drive out] v={speed:.2f} steer={steering_angle:.3f}, l1_dis={L1_distance}")
        return speed, steering_angle


    def control_loop(self):
        speed, steer = self.map_cycle()
        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = self.get_clock().now().to_msg()
        ack_msg.header.frame_id = 'base_link'
        ack_msg.drive.steering_angle = steer
        ack_msg.drive.speed = speed
        self.drive_pub.publish(ack_msg)

###########################################MAP CONTROLLER###########################################
    def main_loop(self, state, position_in_map, waypoint_array_in_map, speed_now, Opp , position_in_map_frenet, acc_now, track_length):
        # Updating parameters from manager
        self.state = state
        self.position_in_map = position_in_map
        self.waypoint_array_in_map = waypoint_array_in_map
        self.speed_now = speed_now
        self.position_in_map_frenet = position_in_map_frenet
        self.acc_now = acc_now
        self.track_length = track_length
        ## PREPROCESS ##
        # speed vector
        yaw = self.position_in_map[0, 2]
        v = [np.cos(yaw)*self.speed_now, np.sin(yaw)*self.speed_now] 

        # calculate lateral error and lateral error norm (lateral_error, self.lateral_error_list, self.lat_e_norm)
        lat_e_norm, lateral_error = self.calc_lateral_error_norm()

        ### LONGITUDINAL CONTROL ###
        self.speed_command = self.calc_speed_command(v, lat_e_norm)
        
        # POSTPROCESS for acceleration/speed decision
        if self.speed_command is not None:
            speed = np.max(self.speed_command, 0)
            acceleration = 0
            jerk = 0
        else:
            speed = 0
            jerk = 0
            acceleration = 0
            self.logger_warn("[Controller] speed was none")

        ### LATERAL CONTROL ###
        steering_angle = None
        L1_point, L1_distance = self.calc_L1_point(lateral_error)
        
        if L1_point.any() is not None: 
            steering_angle = self.calc_steering_angle(L1_point, L1_distance, yaw, lat_e_norm, v)
        else: 
            raise Exception("L1_point is None")
        
        return speed, acceleration, jerk, steering_angle, L1_point, L1_distance, self.idx_nearest_waypoint

    def calc_steering_angle(self, L1_point, L1_distance, yaw, lat_e_norm, v):
            """
            The purpose of this function is to calculate the steering angle based on the L1 point, desired lateral acceleration and velocity

            Inputs:
                L1_point: point in frenet coordinates at L1 distance in front of the car
                L1_distance: distance of the L1 point to the car
                yaw: yaw angle of the car
                lat_e_norm: normed lateral error
                v : speed vector

            Returns:
                steering_angle: calculated steering angle

            """
            adv_ts_st = self.speed_lookahead_for_steer
            la_position_steer = [self.position_in_map[0, 0] + v[0]*adv_ts_st, self.position_in_map[0, 1] + v[1]*adv_ts_st]
            idx_la_steer = self.nearest_waypoint(la_position_steer, self.waypoint_array_in_map[:, :2])
            speed_la_for_lu = self.waypoint_array_in_map[idx_la_steer, 2]
            speed_for_lu = self.speed_adjust_lat_err(speed_la_for_lu, lat_e_norm)

            L1_vector = np.array([L1_point[0] - self.position_in_map[0, 0], L1_point[1] - self.position_in_map[0, 1]])
            if np.linalg.norm(L1_vector) == 0:
                self.logger_warn("[Controller] norm of L1 vector was 0, eta is set to 0")
                eta = 0
            else:
                eta = np.arcsin(np.dot([-np.sin(yaw), np.cos(yaw)], L1_vector)/np.linalg.norm(L1_vector))
            
            if L1_distance == 0 or np.sin(eta) == 0:
                lat_acc = 0
                self.logger_warn("[Controller] L1 * np.sin(eta), lat_acc is set to 0")
            else:
                lat_acc = 2*speed_for_lu**2 / L1_distance * np.sin(eta)

            steering_angle = self.steer_lookup.lookup_steer_angle(lat_acc, speed_for_lu)

            # modifying steer based on acceleration
            steering_angle = self.acc_scaling(steering_angle)
            # modifying steer based on speed
            steering_angle = self.speed_steer_scaling(steering_angle, speed_for_lu)

            # modifying steer based on velocity
            steering_angle *= np.clip(1 + (self.speed_now/10), 1, 1.25)
            
            # limit change of steering angle
            threshold = 0.4
            if abs(steering_angle - self.curr_steering_angle) > threshold:
                self.logger_info(f"[MAP Controller] steering angle clipped")
            steering_angle = np.clip(steering_angle, self.curr_steering_angle - threshold, self.curr_steering_angle + threshold) 
            self.curr_steering_angle = steering_angle
            return steering_angle

    def calc_L1_point(self, lateral_error):
        """
        The purpose of this function is to calculate the L1 point and distance
        
        Inputs:
            lateral_error: frenet d distance from car's position to nearest waypoint
        Returns:
            L1_point: point in frenet coordinates at L1 distance in front of the car
            L1_distance: distance of the L1 point to the car
        """
        
        self.idx_nearest_waypoint = self.nearest_waypoint(self.position_in_map[0, :2], self.waypoint_array_in_map[:, :2]) 
        
        # if all waypoints are equal set self.idx_nearest_waypoint to 0
        if np.isnan(self.idx_nearest_waypoint): 
            self.idx_nearest_waypoint = 0
        
        if len(self.waypoint_array_in_map[self.idx_nearest_waypoint:]) > 2:
            # calculate curvature of global optimizer waypoints
            self.curvature_waypoints = np.mean(abs(self.waypoint_array_in_map[self.idx_nearest_waypoint:,5]))

        # calculate L1 guidance
        L1_distance = self.q_l1 + self.speed_now *self.m_l1

        # clip lower bound to avoid ultraswerve when far away from mincurv
        lower_bound = max(self.t_clip_min, np.sqrt(2)*lateral_error)
        self.get_logger().info(f"Lower bound={lower_bound}")
        L1_distance = np.clip(L1_distance, lower_bound, self.t_clip_max)

        L1_point = self.waypoint_at_distance_before_car(L1_distance, self.waypoint_array_in_map[:,:2], self.idx_nearest_waypoint)
        return L1_point, L1_distance
    
    
    def calc_speed_command(self, v, lat_e_norm):
        """
        The purpose of this function is to isolate the speed calculation from the main control_loop
        
        Inputs:
            v: speed vector
            lat_e_norm: normed lateral error
            curvature_waypoints: -
        Returns:
            speed_command: calculated and adjusted speed, which can be sent to mux
        """
        
        # lookahead for speed (speed delay incorporation by propagating position)
        adv_ts_sp = self.speed_lookahead
        la_position = [self.position_in_map[0, 0] + v[0]*adv_ts_sp, self.position_in_map[0, 1] + v[1]*adv_ts_sp]
        idx_la_position = self.nearest_waypoint(la_position, self.waypoint_array_in_map[:, :2])
        global_speed = self.waypoint_array_in_map[idx_la_position, 2]
        self.trailing_speed = global_speed
        self.i_gap = 0
        speed_command = global_speed
        speed_command = self.speed_adjust_lat_err(speed_command, lat_e_norm)
        return speed_command
    
    def distance(self, point1, point2):
        return np.linalg.norm(point2 - point1)

    def acc_scaling(self, steer):
        """
        Steer scaling based on acceleration
        increase steer when accelerating
        decrease steer when decelerating

        Returns:
            steer: scaled steering angle based on acceleration
        """
        if np.mean(self.acc_now) >= 1:
            steer *= self.acc_scaler_for_steer
        elif np.mean(self.acc_now) <= -1:
            steer *= self.dec_scaler_for_steer
        return steer

    def speed_steer_scaling(self, steer, speed):
        """
        Steer scaling based on speed
        decrease steer when driving fast

        Returns:
            steer: scaled steering angle based on speed
        """
        speed_diff = max(0.1,self.end_scale_speed-self.start_scale_speed) # to prevent division by zero
        factor = 1 - np.clip((speed - self.start_scale_speed)/(speed_diff), 0.0, 1.0) * self.downscale_factor
        steer *= factor
        return steer

    def calc_lateral_error_norm(self):
        """
        Calculates lateral error

        Returns:
            lat_e_norm: normalization of the lateral error
            lateral_error: distance from car's position to nearest waypoint
        """
        # DONE rename function and adapt
        lateral_error = abs(self.position_in_map_frenet[1]) # frenet coordinates d

        max_lat_e = 0.5
        min_lat_e = 0.01
        lat_e_clip = np.clip(lateral_error, a_min=min_lat_e, a_max=max_lat_e)
        lat_e_norm = 0.5 * ((lat_e_clip - min_lat_e) / (max_lat_e - min_lat_e))
        return lat_e_norm, lateral_error

    def speed_adjust_lat_err(self, global_speed, lat_e_norm):
        """
        Reduce speed from the global_speed based on the lateral error 
        and curvature of the track. lat_e_coeff scales the speed reduction:
        lat_e_coeff = 0: no account for lateral error
        lat_e_coaff = 1: maximum accounting

        Returns:
            global_speed: the speed we want to follow
        """
        # scaling down global speed with lateral error and curvature
        lat_e_coeff = self.lat_err_coeff # must be in [0, 1]
        lat_e_norm *= 2 
        curv = np.clip(2*(np.mean(self.curvature_waypoints)/0.8) - 2, a_min = 0, a_max = 1) # 0.8 ca. max curvature mean
        
        global_speed *= (1 - lat_e_coeff + lat_e_coeff*np.exp(-lat_e_norm*curv))
        return global_speed
    
    def speed_adjust_heading(self, speed_command):
        """
        Reduce speed from the global_speed based on the heading error.
        If the difference between the map heading and the actual heading
        is larger than 20 degrees, the speed gets scaled down linearly up to 0.5x
        
        Returns:
            global_speed: the speed we want to follow
        """

        heading = self.position_in_map[0,2]
        map_heading = self.waypoint_array_in_map[self.idx_nearest_waypoint, 6]
        if abs(heading - map_heading) > np.pi: # resolves wrapping issues
            heading_error = 2*np.pi - abs(heading- map_heading)
        else:
            heading_error = abs(heading - map_heading)

        if heading_error < np.pi/9: # 20 degrees error is okay
            return speed_command
        elif heading_error < np.pi/2: 
            scaler = 1 - 0.5* heading_error/(np.pi/2) # scale linearly to 0.5x
        else:
            scaler = 0.5
        self.logger_info(f"[MAP Controller] heading error decreasing velocity by {scaler}")
        return speed_command * scaler
        
    def nearest_waypoint(self, position, waypoints):
        """
        Calculates index of nearest waypoint to the car

        Returns:
            index of nearest waypoint to the car
        """        
        position_array = np.array([position]*len(waypoints))
        distances_to_position = np.linalg.norm(abs(position_array - waypoints), axis=1)
        return np.argmin(distances_to_position)

    def waypoint_at_distance_before_car(self, distance, waypoints, idx_waypoint_behind_car):
        """
        Calculates the waypoint at a certain frenet distance in front of the car

        Returns:
            waypoint as numpy array at a ceratin distance in front of the car
        """
        if distance is None:
            distance = self.t_clip_min
        d_distance = distance
        waypoints_distance = 0.1
        d_index= int(d_distance/waypoints_distance + 0.5)

        return np.array(waypoints[min(len(waypoints) -1, idx_waypoint_behind_car + d_index)]) 






############################################MSG CREATION############################################
    # visualization utilities
    def visualize_steering(self, theta):

        quaternions = quaternion_from_euler(0, 0, theta)

        lookahead_marker = Marker()
        lookahead_marker.header.frame_id = "car_state/base_link"
        lookahead_marker.header.stamp = self.get_clock().now().to_msg()
        lookahead_marker.type = 0
        lookahead_marker.id = 50
        lookahead_marker.scale.x = 0.6
        lookahead_marker.scale.y = 0.05
        lookahead_marker.scale.z = 0.01
        lookahead_marker.color.r = 1.0
        lookahead_marker.color.g = 0.0
        lookahead_marker.color.b = 0.0
        lookahead_marker.color.a = 1.0
        lookahead_marker.pose.position.x = 0.0
        lookahead_marker.pose.position.y = 0.0
        lookahead_marker.pose.position.z = 0.0
        lookahead_marker.pose.orientation.x = quaternions[0]
        lookahead_marker.pose.orientation.y = quaternions[1]
        lookahead_marker.pose.orientation.z = quaternions[2]
        lookahead_marker.pose.orientation.w = quaternions[3]
        self.steering_pub.publish(lookahead_marker)

    def set_waypoint_markers(self, waypoints):
        wpnt_id = 0

        for waypoint in waypoints:
            waypoint_marker = self.markers_buf[wpnt_id]
            waypoint_marker.header.frame_id = "map"
            waypoint_marker.header.stamp = self.get_clock().now().to_msg()
            waypoint_marker.type = 2
            waypoint_marker.scale.x = 0.1
            waypoint_marker.scale.y = 0.1
            waypoint_marker.scale.z = 0.1
            waypoint_marker.color.r = 0.0
            waypoint_marker.color.g = 0.0
            waypoint_marker.color.b = 1.0
            waypoint_marker.color.a = 1.0
            waypoint_marker.pose.position.x = waypoint[0]
            waypoint_marker.pose.position.y = waypoint[1]
            waypoint_marker.pose.position.z = 0.0
            waypoint_marker.pose.orientation.x = 0.0
            waypoint_marker.pose.orientation.y = 0.0
            waypoint_marker.pose.orientation.z = 0.0
            waypoint_marker.pose.orientation.w = 1.0
            waypoint_marker.id = wpnt_id + 1
            wpnt_id += 1
        self.waypoint_array_buf.markers = self.markers_buf[:wpnt_id]
        self.waypoint_pub.publish(self.waypoint_array_buf)

    def set_lookahead_marker(self, lookahead_point, id):
        lookahead_marker = Marker()
        lookahead_marker.header.frame_id = "map"
        lookahead_marker.header.stamp = self.get_clock().now().to_msg()
        lookahead_marker.type = 2
        lookahead_marker.id = id
        lookahead_marker.scale.x = 0.15
        lookahead_marker.scale.y = 0.15
        lookahead_marker.scale.z = 0.15
        lookahead_marker.color.r = 1.0
        lookahead_marker.color.g = 0.0
        lookahead_marker.color.b = 0.0
        lookahead_marker.color.a = 1.0
        lookahead_marker.pose.position.x = lookahead_point[0]
        lookahead_marker.pose.position.y = lookahead_point[1]
        lookahead_marker.pose.position.z = 0.0
        lookahead_marker.pose.orientation.x = 0.0
        lookahead_marker.pose.orientation.y = 0.0
        lookahead_marker.pose.orientation.z = 0.0
        lookahead_marker.pose.orientation.w = 1.0
        self.lookahead_pub.publish(lookahead_marker)


def main():
    rclpy.init()
    node = MAPController()
    rclpy.spin(node)
    rclpy.shutdown()
