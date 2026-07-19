from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# ============================================================================
# 시뮬/실차 런치파일 공용 헬퍼 (control_sim.launch.py / control_real.launch.py)
# ============================================================================
# 런치파일이 아니라 순수 헬퍼 모듈 — ros2 launch 진입점으로 직접 실행되지 않음.
# 두 환경에서 100% 동일했던 파라미터/노드 정의를 여기 한 곳에만 두어, 파라미터
# 추가/변경 시 두 파일에 수동으로 미러링해야 하는 드리프트 위험을 없앤다.
# ============================================================================
# IMU 각속도 단위 보정 계수 — 하드웨어 상수 (여기가 유일한 정의 위치)
# ============================================================================
#   ✅ 2026-07-19 실차 확인 완료 — **deg/s로 발행되는 것이 확정됐다.** 팀원이 젯슨에서 측정:
#      차를 손으로 좌우 왕복 회전시켰을 때 |angular_velocity.z|가 60을 넘었다. rad/s였다면
#      60 rad/s = 초당 9.5바퀴라 손으로 불가능한 값이다(60 deg/s ≈ 1.05 rad/s가 실제 회전).
#      부호는 정상 — 반시계(좌회전)에서 양수로 REP-103과 일치하므로 부호 보정은 불필요.
#      따라서 이 상수는 1.0이 아니라 pi/180이어야 한다.
#
#   ⚠️ 이 값은 "튜닝 노브"가 아니라 **하드웨어의 물리적 성질**이다. 주행마다 바꿀 값이
#      아니므로 런치 인자로 매번 넘기지 말고 **여기 기본값을 고칠 것.** 인자로 넘기는
#      방식은 빠뜨렸을 때 57배 틀린 값으로 조용히 주행하게 되는데, 특히
#      lut_calibrator_node는 /drive를 발행하지 않아 **주행 중 아무 증상이 없어서**
#      LUT가 통째로 오염된 걸 나중에야 알게 된다.
#
#   재확인 절차(드라이버/펌웨어를 건드린 뒤에는 반드시 다시 볼 것):
#     ros2 topic hz /imu/data
#     timeout 8 ros2 topic echo /imu/data --field angular_velocity.z \
#       | awk '$1+0==$1 {v=($1<0?-$1:$1); if(v>m){m=v}} END{printf "최댓값 = %.3f\n", m}'
#     → 차를 손으로 들고 좌우 왕복 회전(1초에 반 바퀴 정도). 최댓값이
#       1~5 이면 rad/s(→ 1.0) / 60~300 이면 deg/s(→ 0.0174533)
#
#   ⚠️ 근본 해결은 젯슨 vesc_driver 수정이다(sensor_msgs/Imu의 rad/s 규약 위반을 우리 쪽에서
#      상쇄하고 있는 상태). 거기서 고쳐지면 **반드시 1.0으로 되돌릴 것** — 양쪽에 걸면 이중
#      보정되어 요레이트가 1/57로 죽는다. 팀에 넘길 때 "임시로 막아둔 것"임을 명시할 것.
#   ⚠️ rosbag에는 보정 전 원시값(deg/s)이 기록된다. 오프라인 분석 시 직접 pi/180을 곱할 것.
#
#   이 상수는 control_map_node(요레이트 카운터스티어)와 lut_calibrator_node(실측 횡가속도
#   a_lat = v*yaw_rate) 양쪽이 공유한다. lut_calibration.launch.py도 이 값을 import 해서
#   쓰므로, 두 곳이 어긋날 일이 구조적으로 없다.
IMU_ANGULAR_SCALE = 0.0174533   

# ⚠️ 조이스틱 드라이버·sim_imu_bridge_node 포함 여부 등 안전 관련 구조 차이는
# 일부러 여기로 옮기지 않고 각 진입점 파일에 그대로 둔다(환경을 잘못 골라 안전
# 기능이 빠진 채 기동되는 실수를 구조적으로 차단하기 위함).


def declare_common_args():
    """두 런치파일에서 동일하게 쓰는 인자 선언 목록."""
    return [
        DeclareLaunchArgument(
            'force_autonomous',
            default_value='false',
            description='true 시 조이스틱 없이 자율주행 모드 즉시 기동'
        ),
        # 요레이트 피드백 카운터스티어 게인. 2026-07-11 시뮬 스윕(fuck_f1, 0.0/0.08/0.15) 결과
        # 랩타임/속도는 게인 무관, 0.15부터 조향 채터링이 뚜렷(부호전환 0→3.32/s) → 보수값 0.08.
        DeclareLaunchArgument(
            'yaw_rate_gain',
            default_value='0.08',
            description='요레이트 카운터스티어 게인 (낮게 시작해 채터링 보며 상향)'
        ),

        # ── 조향 스케일러 (가감속/속도 구간별 조향 게인 완화) ──
        DeclareLaunchArgument(
            'acceleration_scaler_for_steering', default_value='1.0',
            description='가속 중(acc_mean>=1.0) 조향각에 곱하는 스케일러'
        ),
        DeclareLaunchArgument(
            'deceleration_scaler_for_steering', default_value='0.95',
            description='감속 중(acc_mean<=-1.0) 조향각에 곱하는 스케일러'
        ),
        DeclareLaunchArgument(
            'start_scale_speed', default_value='7.0',
            description='속도 비례 조향 다운스케일 시작 속도 [m/s]'
        ),
        DeclareLaunchArgument(
            'end_scale_speed', default_value='8.0',
            description='속도 비례 조향 다운스케일 종료 속도 [m/s] (이후 downscale_factor 최대 적용)'
        ),
        DeclareLaunchArgument(
            'downscale_factor', default_value='0.10',
            description='고속 구간 조향각 다운스케일 최대 비율'
        ),
        DeclareLaunchArgument(
            'speed_lookahead', default_value='0.15',
            description='종방향 목표속도용 예측 룩어헤드 시간 [s]'
        ),
        DeclareLaunchArgument(
            'speed_lookahead_for_steering', default_value='0.0',
            description='조향 계산용 속도 예측 룩어헤드 시간 [s]'
        ),

        # ── 롤 인지형 ESC ──
        DeclareLaunchArgument(
            'max_roll_limit', default_value='0.15',
            description='롤 각도 전복 위험 임계치 [rad] (약 8.6도)'
        ),
        DeclareLaunchArgument(
            'decel_attenuation', default_value='0.6',
            description='롤 비율에 따른 가감속 한계 축소 비율'
        ),

        # ── 경로소스 신선도 / 장애물 회피 폴백(GapFollower) ──
        DeclareLaunchArgument(
            'local_fresh_timeout', default_value='0.3',
            description='이 시간(s) 넘게 /local_waypoints 미수신 시 글로벌 경로로 폴백'
        ),
        DeclareLaunchArgument(
            'obstacle_avoid_enable', default_value='false',
            description='글로벌 추종 중 장애물 차단 감지 시 GapFollower 회피 폴백 활성화. '
                        '기본 false — overtake 방해 방지(앞차를 장애물로 오인해 회피 전환하는 것 차단). '
                        'obstacle_avoid_enable:=true로 되살릴 수 있음.'
        ),
        DeclareLaunchArgument(
            'obstacle_cone_halfangle', default_value='0.14',
            description='장애물 차단 판정용 L1 방향 콘 반각 [rad] (~8도)'
        ),
        DeclareLaunchArgument(
            'obstacle_trigger_dist', default_value='1.5',
            description='이 거리[m] 이내 근접 장애물 감지 시 회피 폴백 트리거'
        ),
        DeclareLaunchArgument(
            'obstacle_margin', default_value='0.3',
            description='장애물 차단 판정 시 목표점 거리 대비 최소 여유 [m]'
        ),
        DeclareLaunchArgument(
            'obstacle_avoid_hold_cycles', default_value='15',
            description='회피 폴백 유지 사이클 수(50Hz 기준, 채터링 방지)'
        ),

        # ── L1 Guidance 룩어헤드 거리 ──
        # 공식: L1 = clamp(l1_gain + v*l1_distance, max(t_clip_min, sqrt2*lat_err), t_clip_max)
        DeclareLaunchArgument(
            'l1_gain', default_value='0.5',
            description='L1 룩어헤드 거리 베이스 오프셋 [m] (공식: l1_gain + v*l1_distance)'
        ),
        DeclareLaunchArgument(
            'l1_distance', default_value='0.3',
            description='L1 룩어헤드 거리 속도 게인 [s] (공식: l1_gain + v*l1_distance)'
        ),
        DeclareLaunchArgument(
            't_clip_min', default_value='0.8',
            description='L1 룩어헤드 거리 하한 [m] (낮을수록 저속/시케인 구간에서 국소 지그재그를 '
                        '쫓아 고주파 조향 유발 가능)'
        ),
        DeclareLaunchArgument(
            't_clip_max', default_value='5.0',
            description='L1 룩어헤드 거리 상한 [m]'
        ),

        # ── 종방향 감속 한계 (곡률 사전감속 제동거리 계산에도 직접 쓰임) ──
        # sim/real 둘 다 동일값이라 base_max_accel과 달리 여기서 공용 선언. 8.0은 실측 검증
        # 전 추정값 — max_lateral_accel 마찰피크(~6.7) 대비 낙관적일 수 있어 실차에서 급제동
        # IMU 실측(acc_mean) 후 재조정 권장.
        DeclareLaunchArgument(
            'base_max_decel', default_value='8.0',
            description='종방향 최대 감속도 한계 [m/s^2] (곡률 사전감속 제동거리 계산에 사용, 실측 전 추정값)'
        ),

        # ── IMU 기반 보정 전체 on/off (요레이트 카운터스티어 + 롤 인지 ESC) ──
        # 실차에서 조향 채터링이 보이면 즉시 끌 수 있도록 런치 인자로 노출. 끄면 순수
        # L1+LUT(시뮬 검증 상태)로 돌아간다. 단위 문제는 IMU_ANGULAR_SCALE로 해결됐으므로
        # 평상시엔 true로 둘 것.
        DeclareLaunchArgument(
            'use_imu', default_value='true',
            description='IMU 보정 사용 여부(요레이트 카운터스티어+롤 ESC). '
                        '조향 채터링 시 false로 순수 L1+LUT 주행'
        ),

        # ── IMU 각속도 단위 보정 계수 ──
        DeclareLaunchArgument(
            'imu_angular_scale', default_value=str(IMU_ANGULAR_SCALE),
            description=f'IMU 각속도 단위 보정 계수 (하드웨어 상수, 기본 {IMU_ANGULAR_SCALE}). '
                        '평상시 넘기지 말 것 — _control_common.py의 IMU_ANGULAR_SCALE을 고칠 것'
        ),

        # ── MPPI 컨트롤러 튜너블 (control_mppi_node 전용) ──
        # 나머지 MPPI 파라미터(N/K/차량/타이어/비용가중)는 노드 코드 기본값 사용.
        DeclareLaunchArgument(
            'mppi_lambda', default_value='1.0',
            description='MPPI 역온도 λ (작을수록 저비용 롤아웃에 집중)'
        ),
        DeclareLaunchArgument(
            'mppi_sigma_steer', default_value='0.15',
            description='MPPI 조향 탐색 노이즈 σ [rad]'
        ),
        DeclareLaunchArgument(
            'mppi_sigma_accel', default_value='1.5',
            description='MPPI 종가속 탐색 노이즈 σ [m/s^2]'
        ),

        # ── VESC 속도→ERPM 변환 게인 (시뮬 대시보드 RPM 표시 전용) ──
        # ⚠️ 이 저장소는 더 이상 ackermann_to_vesc_node를 띄우지 않는다(f1tenth_stack이 담당).
        #   따라서 이 값은 표시용일 뿐이고, 실제 VESC 변환 게인은 젯슨 f1tenth_stack의
        #   vesc.yaml에 있다. 표시가 실제와 맞으려면 그쪽 값과 같아야 한다.
        DeclareLaunchArgument(
            'speed_to_erpm_gain', default_value='4614.0',
            description='속도[m/s]→VESC ERPM 변환 게인 (표시 전용 — 젯슨 vesc.yaml과 같은 값이어야 함)'
        ),
    ]


def build_control_map_node(*, odom_topic, max_speed, max_lateral_accel, base_max_accel,
                            lookup_table_file='', remappings=None):
    """control_map_node — 환경별로 다른 값만 인자로 받고 나머지는 공용 정의.
    remappings: 실차에서만 필요한 토픽 리매핑(예: vesc_driver의 sensors/imu/raw →
    코드에 하드코딩된 /imu/data). 시뮬은 sim_imu_bridge_node가 /imu/data로 바로 발행하므로 불필요."""
    return Node(
        package='f1tenth_control',
        executable='control_map_node',
        name='control_map_node',
        output='screen',
        remappings=remappings,
        parameters=[{
            'odom_topic': odom_topic,
            'wheelbase': 0.33,
            'l1_gain': LaunchConfiguration('l1_gain'),
            'l1_distance': LaunchConfiguration('l1_distance'),
            't_clip_min': LaunchConfiguration('t_clip_min'),
            't_clip_max': LaunchConfiguration('t_clip_max'),
            'lateral_error_coeff': 1.0,
            'max_speed': max_speed,
            'min_speed': 2.0,
            'max_lateral_accel': max_lateral_accel,
            'curvature_lookahead_count': 20,
            'base_max_accel': base_max_accel,
            'base_max_decel': LaunchConfiguration('base_max_decel'),
            'wall_safety_margin': 0.6,
            'lookup_table_file': lookup_table_file,
            'use_imu': ParameterValue(LaunchConfiguration('use_imu'), value_type=bool),
            'imu_angular_scale': LaunchConfiguration('imu_angular_scale'),
            'yaw_rate_gain': LaunchConfiguration('yaw_rate_gain'),
            'curvature_ff_blend': 0.0,
            'heading_damping_gain': 0.0,
            'acceleration_scaler_for_steering': LaunchConfiguration('acceleration_scaler_for_steering'),
            'deceleration_scaler_for_steering': LaunchConfiguration('deceleration_scaler_for_steering'),
            'start_scale_speed': LaunchConfiguration('start_scale_speed'),
            'end_scale_speed': LaunchConfiguration('end_scale_speed'),
            'downscale_factor': LaunchConfiguration('downscale_factor'),
            'speed_lookahead': LaunchConfiguration('speed_lookahead'),
            'speed_lookahead_for_steering': LaunchConfiguration('speed_lookahead_for_steering'),
            'max_roll_limit': LaunchConfiguration('max_roll_limit'),
            'decel_attenuation': LaunchConfiguration('decel_attenuation'),
            'local_fresh_timeout': LaunchConfiguration('local_fresh_timeout'),
            'obstacle_avoid_enable': LaunchConfiguration('obstacle_avoid_enable'),
            'obstacle_cone_halfangle': LaunchConfiguration('obstacle_cone_halfangle'),
            'obstacle_trigger_dist': LaunchConfiguration('obstacle_trigger_dist'),
            'obstacle_margin': LaunchConfiguration('obstacle_margin'),
            'obstacle_avoid_hold_cycles': ParameterValue(
                LaunchConfiguration('obstacle_avoid_hold_cycles'), value_type=int),
        }]
    )


def build_control_mppi_node(*, odom_topic, max_speed, remappings=None):
    """control_mppi_node — control_map_node와 나란히 상시 구동되는 MPPI 컨트롤러.
    /drive_mppi로 발행하며, Mux가 RB 상태에 따라 /drive_autonomous(MAP)와 라우팅한다.
    솔버(CPU/GPU)는 빌드타임 자동선택 — 런치는 무관.
    max_speed는 노드의 v_max(직선 최고속도 캡)로 매핑. Pacejka/차량 파라미터는 노드
    기본값(gym) — 실차 보정 전까지 노출 최소화. remappings: 실차 /imu/data→sensors/imu/raw."""
    return Node(
        package='f1tenth_control',
        executable='control_mppi_node',
        name='control_mppi_node',
        output='screen',
        remappings=remappings,
        parameters=[{
            'odom_topic': odom_topic,
            'v_max': max_speed,
            'lambda': LaunchConfiguration('mppi_lambda'),
            'sigma_steer': LaunchConfiguration('mppi_sigma_steer'),
            'sigma_accel': LaunchConfiguration('mppi_sigma_accel'),
        }]
    )


def build_joy_teleop_monitor():
    """joy_teleop_monitor — 2026-07-17부터 시뮬 전용(실차 런치에서는 제외).
    실차는 f1tenth_stack의 drive_mode_manager + ackermann_mux가 수동/자율/E-stop Mux를 담당하므로
    이 노드를 띄우면 /drive가 이중 발행되어 충돌한다. 시뮬에는 f1tenth_stack이 없으므로 이 노드가
    여전히 전체 Mux(수동/자율/MAP·MPPI/E-stop) 역할을 한다.

    버튼/축 매핑은 실차 drive_mode_manager와 일치시킨다(A=자율/B=정지/X=수동, 좌스틱세로=속도/
    우스틱가로=조향) — 시뮬↔실차 조작감을 같게 해 근육기억을 전이시키기 위함. 스케일도 정렬:
    speed_scale 5.0(max_speed), steering_scale 0.34(max_steering_angle). RB(5)=MAP/MPPI 전환은
    drive_mode_manager가 안 쓰는 버튼이라 유지.

    'is_simulation': True는 의도적으로 고정한 값이다(2026-07-12 사용자 확정) — 항상 MANUAL(조이스틱
    수동 대기)로 시작. 실차 안전 차단이 필요해졌다고 임의로 false로 되돌리지 말 것."""
    return Node(
        package='f1tenth_control',
        executable='joy_teleop_monitor',
        name='joy_teleop_monitor',
        output='screen',
        parameters=[{
            'is_simulation': True,
            'force_autonomous': LaunchConfiguration('force_autonomous'),
            'max_speed': 5.0,             # drive_mode_manager speed_scale와 정렬
            'max_steering_angle': 0.34,   # drive_mode_manager steering_scale와 정렬
            'use_trigger_throttle': False,
            'steering_axis': 3,           # 우스틱 가로
            'throttle_axis': 1,           # 좌스틱 세로
            'autonomous_button': 0,       # A
            'emergency_button': 1,        # B
            'manual_button': 2,           # X
            'algorithm_button': 5,        # RB (MAP/MPPI)
            'speed_to_erpm_gain': LaunchConfiguration('speed_to_erpm_gain'),
        }]
    )
