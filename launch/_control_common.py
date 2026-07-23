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
#
#   이 상수는 control_map_node(요레이트 카운터스티어)와 lut_calibrator_node(실측 횡가속도
#   a_lat = v*yaw_rate) 양쪽이 공유한다. lut_calibration.launch.py도 이 값을 import 해서
#   쓰므로, 두 곳이 어긋날 일이 구조적으로 없다.
IMU_ANGULAR_SCALE_REAL = 0.0174533   # = pi/180. VESC가 deg/s로 발행(2026-07-19 확인)
IMU_ANGULAR_SCALE_SIM  = 1.0         # sim_imu_bridge_node는 이미 rad/s로 중계 → 보정 불필요

# ============================================================================
# IMU 선형가속도 단위 보정 계수 — 하드웨어 상수 (여기가 유일한 정의 위치)
# ============================================================================

IMU_LINEAR_SCALE_REAL = 9.80665      # g → m/s². VESC가 g로 발행(2026-07-19 소스 확인)
IMU_LINEAR_SCALE_SIM  = 1.0          # sim_imu_bridge_node는 0 고정(향후 싣더라도 m/s²) → 보정 불필요

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

        # ── 경로 이탈 복구 가드 (2026-07-21) ──
        # 횡오차가 recovery_lat_error를 넘으면 L1 목표점을 차량 기준 직선거리로 재선정하고
        # 속도를 recovery_speed로 낮춰 라인 복귀를 우선한다. 0이면 비활성(기존 거동).
        # ⚠️ 기본 1.0m는 ifac_track 반폭(0.55~0.8m) 기준으로 "정상 추종 중엔 절대 안 걸리게"
        #    잡은 값이다. **트랙 폭이 다른 맵에서는 반드시 재검토할 것** — 넓은 트랙에서
        #    회피/추월 라인이 글로벌 대비 1m 넘게 벌어지면 정상 주행 중에 가드가 걸려
        #    불필요하게 recovery_speed로 감속한다. 대략 트랙 반폭보다 조금 크게 잡으면 된다.
        DeclareLaunchArgument(
            'recovery_lat_error', default_value='0.0',
            description='경로 이탈 복구 가드 발동 횡오차 [m] (0=비활성). 트랙 반폭보다 크게 잡을 것'
        ),
        DeclareLaunchArgument(
            'recovery_speed', default_value='2.0',
            description='이탈 복구 중 속도 상한 [m/s] (선회반경을 줄여 라인 복귀를 돕는다)'
        ),

        # ── 경로소스 신선도 / 장애물 회피 폴백(GapFollower) ──
        DeclareLaunchArgument(
            'local_fresh_timeout', default_value='0.3',
            description='이 시간(s) 넘게 /local_waypoints 미수신 시 글로벌 경로로 폴백'
        ),
        DeclareLaunchArgument(
            'gap_follower_failsafe', default_value='false',
            description='글로벌·로컬 웨이포인트가 둘 다 없을 때 GapFollower로 자율주행할지. '
                        '기본 false=안전 정지 발행(control_mppi_node와 동일). '
                        '⚠️ true면 플래닝이 안 떠 있거나 죽었을 때 컨트롤러가 라이다 갭만 보고 '
                        '차를 스스로 몰기 시작한다(1.2~3.5 m/s) — 실차에서는 켜지 말 것'
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

        # ── 장애물 종방향 감속 (opponent_detector raw 장애물 → 속도 캡) ──
        DeclareLaunchArgument(
            'obstacle_brake_enable', default_value='true',
            description='통로 전방 장애물(opponent_detector raw)에 대해 정지 가능 속도로 감속. '
                        '조향 미개입 종방향 soft 감속 — 최종 e-stop은 planning 소관. false로 비활성.'
        ),
        DeclareLaunchArgument(
            'obstacle_raw_topic', default_value='/perception/detection/raw_obstacles',
            description='raw 장애물(추적 확정 전, 벽 제거+Frenet) 토픽 (f110_msgs/ObstacleArray)'
        ),
        DeclareLaunchArgument(
            'obstacle_brake_decel', default_value='6.0',
            description='감속 캡 v=√(2·a·d) 산출용 감속도 [m/s²]. base_max_decel보다 낮게 잡아 보수적으로.'
        ),
        DeclareLaunchArgument(
            'obstacle_stop_gap', default_value='1.0',
            description='장애물 앞 정지 여유 거리 [m]'
        ),
        DeclareLaunchArgument(
            'obstacle_corridor_halfwidth', default_value='0.35',
            description='통로 반폭(차폭/2+여유) [m] — 장애물이 이 밴드와 겹칠 때만 감속'
        ),
        DeclareLaunchArgument(
            'obstacle_max_range', default_value='9.0',
            description='이 전방거리[m] 밖 장애물은 무시(라이다 유효거리)'
        ),
        DeclareLaunchArgument(
            'obstacle_brake_hold_cycles', default_value='10',
            description='장애물 소실 후 캡 유지 사이클 수(50Hz, 채터링 방지)'
        ),
        DeclareLaunchArgument(
            'obstacle_brake_timeout', default_value='0.3',
            description='raw 장애물 토픽 신선도 타임아웃 [s]'
        ),
        DeclareLaunchArgument(
            'obstacle_avoid_min_speed', default_value='1.5',
            description='로컬 회피경로 추종 중 감속캡 하한 [m/s] — 정지 대신 최소속도로 회피 관통. '
                        '글로벌 대기(회피경로 없음) 중엔 미적용(정지까지 허용)'
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

        # ── 기동 실패(VESC 센서리스 탈조) 가드 ──
        # 2026-07-22 실차: 출발 시 4초간 덜그럭거리다 출발하는 증상. 그동안 컨트롤러의 속도
        # 램프는 실측과 무관하게 프로파일 속도까지 감겨 올라가, 모터가 물리는 순간 풀 명령이
        # 걸린 채 튀어나간다. 근본 원인은 VESC mcconf(오픈루프 800 vs 옵저버 인수 2500 ERPM
        # 갭)라 그쪽에서 고쳐야 하지만, 이 가드는 그와 무관하게 급발진만 막는 안전망이다.
        # 시뮬에선 차가 명령을 즉시 따라가므로 발동하지 않는다(무회귀).
        DeclareLaunchArgument(
            'stall_guard_enable', default_value='true',
            description='기동 실패(탈조) 시 속도 명령 와인드업 차단 가드 on/off'
        ),
        DeclareLaunchArgument(
            'stall_speed_threshold', default_value='0.7',
            description='이 속도[m/s] 미만이면 "안 움직인다"로 판정. 센서리스 데드존 상단(0.59)보다 위에 둘 것'
        ),
        DeclareLaunchArgument(
            'stall_hold_speed', default_value='1.5',
            description='탈조 판정 시 속도 명령을 묶어둘 값 [m/s] (데드존 위 + 완만한 출발)'
        ),
        DeclareLaunchArgument(
            'stall_hold_delay', default_value='1.0',
            description='이 시간[s] 이상 안 움직이면 가드 발동. 4초 탈조는 잡고 정상 기동 지연(~0.3s)은 안 잡히게'
        ),

        # ── IMU 기반 보정 전체 on/off (요레이트 카운터스티어 + 롤 인지 ESC) ──
        # 실차에서 조향 채터링이 보이면 즉시 끌 수 있도록 런치 인자로 노출. 끄면 순수
        # L1+LUT(시뮬 검증 상태)로 돌아간다. 단위 문제는 imu_angular_scale로 해결됐으므로
        # 평상시엔 true로 둘 것.
        DeclareLaunchArgument(
            'use_imu', default_value='true',
            description='IMU 보정 사용 여부(요레이트 카운터스티어+롤 ESC). '
                        '조향 채터링 시 false로 순수 L1+LUT 주행'
        ),



        # ── MPPI 컨트롤러 튜너블 (control_mppi_node 전용) ──
        # 나머지 MPPI 파라미터(차량/타이어)는 노드 코드 기본값 사용.
        # 2026-07-22: 시뮬 튜닝 스윕을 위해 수평/샘플수/비용가중/평활화/가속한계를 전부 인자화
        #   (control_map_node가 07-11에 밟은 것과 같은 경로 — 코드를 안 건드리고 터미널에서 스윕).
        DeclareLaunchArgument(
            'mppi_lambda_rel', default_value='0.02',
            description='MPPI 적응 역온도 비율: λ_eff = mppi_lambda_rel·(J_mean − J_min). '
                        'λ를 비용 스케일에 불변으로 만든다(w_*를 바꿔도 재조정 불필요). '
                        '작을수록 저비용 샘플에 집중(ESS↓, 반응 빠르고 거칠다) — 0.02가 ESS≈K의 10%'
        ),
        DeclareLaunchArgument(
            'mppi_lambda', default_value='1.0',
            description='MPPI 고정 역온도 λ (mppi_lambda_rel:=0 으로 둘 때만 사용)'
        ),
        DeclareLaunchArgument(
            'mppi_noise_beta', default_value='0.7',
            description='MPPI 잡음 시간상관 AR(1) 계수 [0,1). 0=백색잡음(고주파 해가 뽑혀 채터링), '
                        '0.6~0.8이 매끈한 기동'
        ),
        DeclareLaunchArgument(
            'mppi_sigma_steer', default_value='0.15',
            description='MPPI 조향 탐색 노이즈 σ [rad]'
        ),
        DeclareLaunchArgument(
            'mppi_sigma_accel', default_value='1.5',
            description='MPPI 종가속 탐색 노이즈 σ [m/s^2]'
        ),
        DeclareLaunchArgument(
            'mppi_N', default_value='25',
            description='MPPI 예측 수평 스텝 수 (수평시간 = N·dt, 기본 25×0.05=1.25s). '
                        '수평이 코너 하나보다 짧으면 시케인에서 모드가 매 사이클 바뀐다'
        ),
        DeclareLaunchArgument(
            'mppi_K', default_value='0',
            description='MPPI 롤아웃 샘플 수. 0=솔버별 자동(GPU 2048 / CPU 512). '
                        '키울수록 직선 미세 사행이 줄어든다(분산 ~1/ESS) — GPU에서는 사실상 공짜'
        ),
        DeclareLaunchArgument(
            'mppi_u_smooth', default_value='0.3',
            description='MPPI 출력 저역통과 계수 [0,1) — 클수록 부드럽지만 지연 증가'
        ),
        DeclareLaunchArgument(
            'mppi_w_lat', default_value='150.0',
            description='MPPI 경로 **횡**오차(컨투어링) 비용 가중 — 경로 추종의 주력'
        ),
        DeclareLaunchArgument(
            'mppi_w_lon', default_value='1.0',
            description='MPPI 경로 **진행방향**(lag) 오차 비용 가중 — 시간정합용, 작게 둘 것'
        ),
        DeclareLaunchArgument(
            'mppi_w_dsteer', default_value='100.0',
            description='MPPI 조향 변화율(Δδ) 비용 — 채터링 억제의 본체'
        ),
        DeclareLaunchArgument(
            'mppi_w_daccel', default_value='0.5',
            description='MPPI 종가속 변화율(Δa) 비용'
        ),
        DeclareLaunchArgument(
            'mppi_w_yaw', default_value='5.0',
            description='MPPI 헤딩 추종 비용 가중'
        ),
        DeclareLaunchArgument(
            'mppi_w_v', default_value='0.5',
            description='MPPI 속도 추종 비용 가중'
        ),
        DeclareLaunchArgument(
            'mppi_ref_max_lat_accel', default_value='8.0',
            description='MPPI 기준속도 곡률 클램프 a_lat [m/s²] (0=플래너 프로파일 그대로). '
                        '플래너 프로파일은 이상적 라인 기준이라, 라인에서 조금만 벗어나면 같은 '
                        '속도로 마찰한계를 넘는다. ⚠️ 모델 마찰한계(μ·g≈10.3)보다 낮게 잡을 것'
        ),
        DeclareLaunchArgument(
            'mppi_w_terminal', default_value='20.0',
            description='MPPI 종단(마지막 스테이지) 위치·헤딩 가중 배수. 수평은 속도에 비례해 '
                        '길어지므로, 이 값이 크면 먼 종단점이 비용을 지배해 **직선에서 속도가 '
                        '스스로 눌린다**(가속할수록 더 먼 코너를 끌어옴)'
        ),
        DeclareLaunchArgument(
            'mppi_margin', default_value='0.15',
            description='MPPI 트랙 경계 여유 [m] (차 반폭 + 안전여유)'
        ),
        DeclareLaunchArgument(
            'mppi_w_boundary', default_value='500.0',
            description='MPPI 트랙 경계 소프트 페널티 가중'
        ),
        DeclareLaunchArgument(
            'mppi_speed_cmd_horizon', default_value='0.21',
            description='MPPI (종가속→속도명령) 변환 지평 [s]. 하위 속도루프 P게인의 역수로 '
                        '두어야 계획한 가속이 실제로 전달된다(gym kp≈4.75 → 약 0.21)'
        ),
        DeclareLaunchArgument(
            'mppi_accel_max', default_value='9.0',
            description='MPPI 종가속 상한 [m/s^2] '
                        '(control_map_node의 base_max_accel과 정렬 — 기준궤적 램프 속도도 이 값을 쓴다)'
        ),

        # ── VESC 속도→ERPM 변환 게인 (시뮬 대시보드 RPM 표시 전용) ──
        # ⚠️ 이 저장소는 더 이상 ackermann_to_vesc_node를 띄우지 않는다(f1tenth_stack이 담당).
        #   따라서 이 값은 표시용일 뿐이고, 실제 VESC 변환 게인은 젯슨 f1tenth_stack의
        #   vesc.yaml에 있다. 표시가 실제와 맞으려면 그쪽 값과 같아야 한다.
        # 2026-07-20 실측 보정: 4614.0 → 4232.0. 복도 직선 5회(전진4·후진1) 줄자 대조로
        #   휠 오도메트리가 8.3% 과소보고하는 것이 확인됨(잔차 ±4cm, 정/역 대칭).
        #   즉 그동안 차가 명령보다 9% 빠르게 달리고 있었다.
        DeclareLaunchArgument(
            'speed_to_erpm_gain', default_value='4232.0',
            description='속도[m/s]→VESC ERPM 변환 게인 (표시 전용 — 젯슨 vesc.yaml과 같은 값이어야 함)'
        ),
    ]


def build_control_map_node(*, odom_topic, max_speed, max_lateral_accel, base_max_accel,
                            imu_angular_scale, imu_linear_scale,
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
            'min_speed': 2.5,
            'max_lateral_accel': max_lateral_accel,
            'curvature_lookahead_count': 20,
            'base_max_accel': base_max_accel,
            'base_max_decel': LaunchConfiguration('base_max_decel'),
            'stall_guard_enable': LaunchConfiguration('stall_guard_enable'),
            'stall_speed_threshold': LaunchConfiguration('stall_speed_threshold'),
            'stall_hold_speed': LaunchConfiguration('stall_hold_speed'),
            'stall_hold_delay': LaunchConfiguration('stall_hold_delay'),
            'wall_safety_margin': 0.6,
            'recovery_lat_error': LaunchConfiguration('recovery_lat_error'),
            'recovery_speed': LaunchConfiguration('recovery_speed'),
            'lookup_table_file': lookup_table_file,
            'use_imu': ParameterValue(LaunchConfiguration('use_imu'), value_type=bool),
            'imu_angular_scale': imu_angular_scale,
            'imu_linear_scale': imu_linear_scale,
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
            'gap_follower_failsafe': LaunchConfiguration('gap_follower_failsafe'),
            'obstacle_avoid_enable': LaunchConfiguration('obstacle_avoid_enable'),
            'obstacle_cone_halfangle': LaunchConfiguration('obstacle_cone_halfangle'),
            'obstacle_trigger_dist': LaunchConfiguration('obstacle_trigger_dist'),
            'obstacle_margin': LaunchConfiguration('obstacle_margin'),
            'obstacle_avoid_hold_cycles': ParameterValue(
                LaunchConfiguration('obstacle_avoid_hold_cycles'), value_type=int),
            'obstacle_brake_enable': LaunchConfiguration('obstacle_brake_enable'),
            'obstacle_raw_topic': LaunchConfiguration('obstacle_raw_topic'),
            'obstacle_brake_decel': LaunchConfiguration('obstacle_brake_decel'),
            'obstacle_stop_gap': LaunchConfiguration('obstacle_stop_gap'),
            'obstacle_corridor_halfwidth': LaunchConfiguration('obstacle_corridor_halfwidth'),
            'obstacle_max_range': LaunchConfiguration('obstacle_max_range'),
            'obstacle_brake_hold_cycles': ParameterValue(
                LaunchConfiguration('obstacle_brake_hold_cycles'), value_type=int),
            'obstacle_brake_timeout': LaunchConfiguration('obstacle_brake_timeout'),
            'obstacle_avoid_min_speed': LaunchConfiguration('obstacle_avoid_min_speed'),
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
            'lambda_rel': LaunchConfiguration('mppi_lambda_rel'),
            'noise_beta': LaunchConfiguration('mppi_noise_beta'),
            'sigma_steer': LaunchConfiguration('mppi_sigma_steer'),
            'sigma_accel': LaunchConfiguration('mppi_sigma_accel'),
            'N': LaunchConfiguration('mppi_N'),
            'K': LaunchConfiguration('mppi_K'),
            'u_smooth': LaunchConfiguration('mppi_u_smooth'),
            'w_lat': LaunchConfiguration('mppi_w_lat'),
            'w_lon': LaunchConfiguration('mppi_w_lon'),
            'w_dsteer': LaunchConfiguration('mppi_w_dsteer'),
            'w_daccel': LaunchConfiguration('mppi_w_daccel'),
            'w_yaw': LaunchConfiguration('mppi_w_yaw'),
            'w_v': LaunchConfiguration('mppi_w_v'),
            'w_boundary': LaunchConfiguration('mppi_w_boundary'),
            'w_terminal': LaunchConfiguration('mppi_w_terminal'),
            'ref_max_lateral_accel': LaunchConfiguration('mppi_ref_max_lat_accel'),
            'margin': LaunchConfiguration('mppi_margin'),
            'accel_max': LaunchConfiguration('mppi_accel_max'),
            'speed_cmd_horizon': LaunchConfiguration('mppi_speed_cmd_horizon'),
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
