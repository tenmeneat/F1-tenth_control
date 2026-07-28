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
            default_value='0.00',
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
            't_clip_min', default_value='0.6',
            description='L1 룩어헤드 거리 하한 [m] (낮을수록 저속/시케인 구간에서 국소 지그재그를 '
                        '쫓아 고주파 조향 유발 가능)'
        ),
        DeclareLaunchArgument(
            't_clip_max', default_value='5.0',
            description='L1 룩어헤드 거리 상한 [m]'
        ),

        # ── 종방향 감속: 두 개의 서로 다른 감속도 ──
        # 2026-07-25 분리(그 전엔 base_max_decel 하나가 두 역할을 겸했다). 두 값은 튜닝 방향이
        # 정반대라 한 노브로 묶으면 반드시 한쪽이 손해를 본다:
        #   base_max_decel = "명령 속도를 초당 얼마나 빨리 떨어뜨릴 수 있나"(램프 rate limit).
        #                    높을수록 감속 명령이 빨리 도달 → 높게 유지.
        #   prebrake_decel = "차가 실제로 낼 수 있는 감속도"(곡률 사전감속 제동거리 v²/2a).
        #                    낮을수록 코너를 더 멀리서 보고 일찍 감속 → 실측값에 맞춰야 함.
        DeclareLaunchArgument(
            'base_max_decel', default_value='8.0',
            description='명령 속도 하강 rate limit [m/s^2]. 낮추면 감속 명령이 늦게 도달하므로 높게 유지'
        ),
        # ⚠️ prebrake_decel은 반드시 **실측 감속 권한**이어야 한다. 2026-07-25 실차 bag
        # (rosbag2_2026_07_25-22_08_50): 명령을 4.00→3.11로 내렸는데 실속은 4.03→3.80(-0.4 m/s²).
        # VESC 속도모드는 회생제동이 거의 없어 사실상 coast고, 주행 중 /commands/motor/brake는
        # 0건이었다. 예전처럼 8.0을 쓰면 4 m/s에서 제동거리를 1.0m로 착각(실제 필요 ~8m)해
        # 사전감속이 0.5초 앞만 보고 시작 → 시케인 언더스티어 크래시. 0.6은 실측 타력감속 반영.
        DeclareLaunchArgument(
            'prebrake_decel', default_value='1.0',
            description='곡률 사전감속 제동거리 산출용 실측 감속 권한 [m/s^2]. 낮을수록 코너를 일찍 봄'
        ),
        # ── 조향 권한 속도 캡 (2026-07-26 신설) ──
        # 곡률 캡이 그동안 **그립만** 봤다(√(a_lat/κ)). 그립과 조향은 다른 물리다:
        #   그립 = 타이어가 그 횡가속을 낼 수 있나 / 조향 = 바퀴가 그만큼 꺾일 수 있나.
        # 정상상태 자전거 모델 δ = L·κ + K_us·κ·v² 을 δ_avail로 풀면
        #   v ≤ √( (ratio·0.41 − L·κ) / (K_us·κ) )
        # 2026-07-26 실차 bag(run_0726_181747)의 κ=1.190(R=0.84m) 헤어핀에서
        #   그립 한계 2.11 m/s  vs  조향 한계 0.87 m/s  ← 조향이 먼저 걸린다.
        # 그립만 보고 2배 빠르게 진입한 결과 풀락(0.410)에도 안 돌아가고 크로스트랙이
        # 0.11 → 2.07m로 발산했다. understeer_gradient=0 이면 이 항 전체 비활성(구 거동).
        DeclareLaunchArgument(
            'understeer_gradient', default_value='0.0',
            description='언더스티어 그래디언트 K_us [rad/(m/s^2)]. 0이면 조향 권한 캡 비활성'
        ),
        # ⚠️ 1.0으로 두면 곡률 추종에 δ_max를 다 써버려 횡오차 보정·요레이트 피드백 여력이 0이 된다.
        DeclareLaunchArgument(
            'steer_authority_ratio', default_value='0.85',
            description='조향 한계(0.41rad) 중 곡률 추종에 배정할 비율. 나머지는 보정 여유'
        ),
        # ── 곡률 사전감속 스캔 거리 하한 ──
        # 전방 곡률 스캔 거리 = max(count*0.1, v²/(2·prebrake_decel)). count는 저속에서 제동거리가
        # 짧아질 때의 하한. 80 = 8m 스캔.
        DeclareLaunchArgument(
            'curvature_lookahead_count', default_value='80',
            description='곡률 룩어헤드 스캔 거리 하한 (×0.1m). 80 = 8m'
        ),
        # ── 최저 순항 속도 하한 ──
        # 곡률 사전감속·헤어핀에서 목표속도가 이 값 밑으로 안 내려가게 하는 하한(sim/real 공용).
        # ⚠️ 장애물 정지 경로는 이 하한을 무시하고 0까지 내려간다(안전 우선) — 순수 순항 프로파일에만 적용.
        DeclareLaunchArgument(
            'min_speed', default_value='1.0',
            description='최저 순항 속도 [m/s] (곡률 감속 하한). 장애물 정지엔 미적용(0까지 허용)'
        ),

        # ── L1 횡가속 분모: 목표점까지의 실제 거리 (2026-07-28) ──
        # pure pursuit 법칙은 a_lat = 2·v²·sin(eta)/L_실제 인데, L1 목표점은 경로 **호 길이**
        # L1_distance만큼 전진해 고르므로 |목표점−차량| != L1_distance다. 차량이 경로 뒤/옆에
        # 있으면 실제 거리가 더 길고, 07-27 실차 bag 실측 비율(|목표점−차량|/L1_distance)은
        # 중앙 1.06~1.31 · p95 최대 1.72였다. 명목값을 분모로 쓰면 횡가속 명령이 그만큼
        # 과대(최대 +70%)해지고 **경로에서 벗어날수록 = 복귀가 필요한 순간에 더 심해져**
        # 오버슈트·조향 발진을 만든다. false면 구 거동(명목 L1_distance) — 즉시 롤백용.
        DeclareLaunchArgument(
            'l1_use_actual_distance', default_value='true',
            description='L1 횡가속 분모로 목표점까지의 실제 직선거리 사용. false면 구 거동(명목 L1 거리)'
        ),

        # ── 최근접 인덱스 견고화 (MCL pose 붕괴 대응, 2026-07-28) ──
        # 07-27 실차 bag(run_0727_195937)에서 MCL pose가 깨진 직후 closest_idx가
        # 86→27→31→89로 트랙 반대편을 오갔고, 경로 접선과 차량 헤딩의 오차가 +146.7°/-173.0°
        # 까지 벌어졌다(주행 샘플의 9.5%가 |오차|>90°). 그 인덱스로 만든 L1 목표점이 차 뒤에
        # 찍혀 조향 명령이 0.2초마다 부호를 뒤집었다 — "명령은 왼쪽인데 차는 오른쪽"의 정체.
        #
        # closest_idx_max_heading_err: 전역 재탐색에서 경로 접선이 차량 헤딩과 이 각도 이내인
        #   후보만 고려한다. 0이면 게이트 비활성(구 거동). 기본 1.75rad(=100°)는 정상 추종에서
        #   절대 안 걸리고 역주행 웨이포인트만 배제하는 값. 조건을 만족하는 후보가 하나도
        #   없으면 게이트를 포기하고 무제한 스캔으로 폴백한다(재획득 불능 상황을 만들지 않음).
        DeclareLaunchArgument(
            'closest_idx_max_heading_err', default_value='1.75',
            description='최근접 전역 재탐색 시 경로접선-차량헤딩 허용오차 [rad]. 0이면 비활성'
        ),
        # 한 사이클(20ms)에 물리적으로 가능한 경로 이동은 v·dt(8m/s에서 16cm)뿐이다. 그보다 먼
        # 점프는 즉시 채택하지 않고 confirm_cycles 동안 연속 유지될 때만 받아들인다(1회성 pose
        # 튐 무시). 보류 중에는 조향을 직전값으로 홀드하고 pose_suspect_speed로 감속한다.
        # ⚠️ cycles를 너무 키우면 진짜 이탈 후 재획득이 늦어진다. 5 = 100ms.
        DeclareLaunchArgument(
            'idx_jump_confirm_dist', default_value='2.0',
            description='이 거리[m]를 넘는 최근접 인덱스 점프는 확인 대기'
        ),
        DeclareLaunchArgument(
            'idx_jump_confirm_cycles', default_value='5',
            description='점프가 연속 이 사이클(50Hz) 유지되면 채택. 0이면 게이트 비활성'
        ),
        DeclareLaunchArgument(
            'pose_suspect_speed', default_value='5.0',
            description='pose 튐 보류 중 속도 상한 [m/s]'
        ),

        # ── 자율 미체결 중 속도 램프 고정 (bumpless transfer, 2026-07-28) ──
        # control_map_node는 /drive_mode와 무관하게 상시 50Hz로 돌기 때문에, MANUAL/E-stop으로
        # 서 있는 동안에도 속도 램프가 계속 감겨 올라간다. A를 눌러 ackermann_mux가 열리는 순간
        # 그 값이 **계단으로** VESC에 꽂힌다. 07-27 실차 bag 8개 전부에서 확인:
        #   정차 중 /drive_autonomous 1.50(최대 3.98) → engage 순간 commands/motor/speed가
        #   0 → 6348 ERPM 한 스텝 → 모터전류 60~62A(l_current_max 포화) → 급발진
        # 램프를 실측 속도로 눌러두면 engage 시점의 명령이 실측과 같아져 계단이 사라진다.
        #
        # ⚠️ 체결(autonomous) 중에는 아무 일도 하지 않는다 — 07-22에 금지한 일반 lead-clamp
        #    ("명령이 실측보다 앞서지 못하게")와 다르다. VESC 속도 PID가 60A를 뽑는 데 필요한
        #    명령 선행(~4.7 m/s)은 주행 중 그대로 보존된다.
        # ⚠️ /drive_mode 미수신·끊김(timeout 초과) 시 게이트는 **자동 비활성**이다. 시뮬은
        #    joy_teleop_monitor가 /drive_mode를 발행하지 않으므로 기존 거동이 그대로 유지된다.
        DeclareLaunchArgument(
            'engage_gate_enable', default_value='true',
            description='자율 미체결(/drive_mode != autonomous) 중 속도 램프를 실측에 고정'
        ),
        DeclareLaunchArgument(
            'drive_mode_topic', default_value='/drive_mode',
            description='자율 체결 상태 토픽 (f1tenth_stack drive_mode_manager 발행)'
        ),
        DeclareLaunchArgument(
            'engaged_mode_value', default_value='autonomous',
            description='체결로 판정할 /drive_mode 문자열'
        ),
        DeclareLaunchArgument(
            'drive_mode_timeout', default_value='1.0',
            description='이 시간[s] 넘게 /drive_mode 미수신이면 게이트 자동 비활성(시뮬 호환)'
        ),

        # ── 기동 실패(VESC 센서리스 탈조) 가드 ──
        # 2026-07-22 실차: 출발 시 4초간 덜그럭거리다 출발하는 증상. 그동안 컨트롤러의 속도
        # 램프는 실측과 무관하게 프로파일 속도까지 감겨 올라가, 모터가 물리는 순간 풀 명령이
        # 걸린 채 튀어나간다. 근본 원인은 VESC mcconf(오픈루프 800 vs 옵저버 인수 2500 ERPM
        # 갭)라 그쪽에서 고쳐야 하지만, 이 가드는 그와 무관하게 급발진만 막는 안전망이다.
        # 시뮬에선 차가 명령을 즉시 따라가므로 발동하지 않는다(무회귀).
        #
        # ⚠️ 2026-07-27 기본값 true→false. 07-27 21:31 bag에서 자율 명령이 3.5초 내내
        # **정확히 1.50**(= stall_hold_speed)에 묶였다. 그 지점(idx 48~51)의 계산 캡은
        # 3.13 m/s이고, min_speed(1.0)·로컬경로(2.42~4.50, 글로벌 대비 0.997)·글로벌경로
        # (2.40~4.50) 어느 것도 1.5를 설명하지 못한다 — 값이 일치하는 건 이 가드뿐이다.
        # 컨트롤러는 수동/E-stop 중에도 돌기 때문에, 자율 진입 전 정차 동안 가드가 이미
        # 발동해 램프를 되감아 둔 것으로 보인다(자율 첫 샘플부터 1.50).
        #
        # 이 가드는 램프 2000 시절(명령만 감겨 올라가다 모터가 물리는 순간 급발진)의
        # 안전망이었고, 데드존은 그 뒤 VESC 오픈루프 전류 상향으로 근본 해결됐다.
        # ⚠️ 다만 끄면 와인드업 급발진 보호가 사라진다. 출발이 여전히 더듬거리면
        # `stall_guard_enable:=true`로 즉시 되돌릴 것. base_max_accel이 9.0→2.5로
        # 내려가 있어 와인드업 속도 자체는 예전보다 3.6배 느리다.
        DeclareLaunchArgument(
            'stall_guard_enable', default_value='false',
            description='기동 실패(탈조) 시 속도 명령 와인드업 차단 가드 on/off. 07-27부터 기본 꺼짐 — 명령이 stall_hold_speed(1.5)에 묶이는 증상 때문'
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

        # ── 런치 킥(자율 정지출발 시 VESC 센서리스 데드존 관통) — 2026-07-27부터 기본 꺼짐 ──
        # 매뉴얼은 초반 스로틀 펀치로 데드존을 때려 관통하는데 자율은 살살 램프해 걸터앉아 탈조한다.
        # 정지 상태에서 짧게 높은 속도를 명령해 VESC 속도 PID가 큰 전류를 뽑게 만든다(매뉴얼 펀치 재현).
        # 오픈루프 전류↑·HFI·Coupled HFI 모두 저돌극성 때문에 부하서 실패 확인(2026-07-25).
        #
        # ⚠️ 2026-07-27 기본값 true→false. 이유 둘:
        #   ① 데드존을 **VESC 오픈루프 전류 상향으로 근본 해결**했다. 컨트롤 측 우회책은 이제 불필요.
        #   ② `s_pid_ramp_erpms_s` 2000→21160 이후로는 이 킥이 훨씬 사납다. 램프 2000일 땐 3.0을
        #      명령해도 VESC setpoint가 0.47 m/s²로 기어갔지만, 21160이면 즉시 큰 ERPM 오차 →
        #      큰 전류 → 예측 불가능한 펀치가 된다.
        # 출발 불능이 재발하면 `launch_boost_enable:=true`로 되살릴 것(파라미터는 그대로 보존).
        DeclareLaunchArgument(
            'launch_boost_enable', default_value='false',
            description='런치 킥 on/off (자율 정지출발 데드존 관통 펀치). 07-27부터 기본 꺼짐 — 데드존은 VESC 오픈루프 전류로 해결됨'
        ),
        DeclareLaunchArgument(
            'launch_boost_speed', default_value='3.0',
            description='데드존 관통용 펀치 속도 명령 [m/s]. 매뉴얼 스로틀 세기 감각으로 조정(세게=5.0/약하게=2.0)'
        ),
        DeclareLaunchArgument(
            'launch_boost_time', default_value='0.6',
            description='관통 실패 시 포기까지 최대 펀치 시간 [s]. stall_hold_delay(1.0)보다 작아야 stall_guard와 안 싸움'
        ),
        DeclareLaunchArgument(
            'launch_exit_speed', default_value='0.8',
            description='실측이 이 속도[m/s] 넘으면 관통 성공 판정 → 킥 종료(데드존 상단 0.59보다 위)'
        ),
        DeclareLaunchArgument(
            'launch_standstill_speed', default_value='0.3',
            description='실측이 이 속도[m/s] 미만이면 정지 판정 → 킥 시작(exit보다 낮아 히스테리시스)'
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
                            max_steering_left, max_steering_right,
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
            'min_speed': LaunchConfiguration('min_speed'),
            'max_lateral_accel': max_lateral_accel,
            'understeer_gradient': LaunchConfiguration('understeer_gradient'),
            'steer_authority_ratio': LaunchConfiguration('steer_authority_ratio'),
            'curvature_lookahead_count': ParameterValue(
                LaunchConfiguration('curvature_lookahead_count'), value_type=int),
            'base_max_accel': base_max_accel,
            'base_max_decel': LaunchConfiguration('base_max_decel'),
            'prebrake_decel': LaunchConfiguration('prebrake_decel'),
            'stall_guard_enable': LaunchConfiguration('stall_guard_enable'),
            'stall_speed_threshold': LaunchConfiguration('stall_speed_threshold'),
            'stall_hold_speed': LaunchConfiguration('stall_hold_speed'),
            'stall_hold_delay': LaunchConfiguration('stall_hold_delay'),
            'launch_boost_enable': ParameterValue(LaunchConfiguration('launch_boost_enable'), value_type=bool),
            'launch_boost_speed': LaunchConfiguration('launch_boost_speed'),
            'launch_boost_time': LaunchConfiguration('launch_boost_time'),
            'launch_exit_speed': LaunchConfiguration('launch_exit_speed'),
            'launch_standstill_speed': LaunchConfiguration('launch_standstill_speed'),
            'wall_safety_margin': 0.6,
            'recovery_lat_error': LaunchConfiguration('recovery_lat_error'),
            'recovery_speed': LaunchConfiguration('recovery_speed'),
            # L1 횡가속 분모 (2026-07-28)
            'l1_use_actual_distance': ParameterValue(
                LaunchConfiguration('l1_use_actual_distance'), value_type=bool),
            # ── 좌우 조향 한계 (2026-07-28) — 진입점 런치가 환경별로 넘긴다 ──
            # 실차는 vesc.yaml의 servo_min/max와 **반드시 한 쌍**으로 움직인다. 컨트롤러만
            # 올리면 vesc_driver가 조용히 자르고 컨트롤러는 꺾었다고 착각한다. 경위와 값은
            # control_real.launch.py 주석 참고(트림 0.4672 기준 ±0.42 → [0.2798, 0.6546]).
            # ⚠️ 시뮬은 차량 모델이 대칭이라 ±0.41 — 실차 하드웨어 특성을 시뮬에 옮기지 않는다.
            'max_steering_left': max_steering_left,
            'max_steering_right': max_steering_right,
            # 최근접 인덱스 견고화 (2026-07-28)
            'closest_idx_max_heading_err': LaunchConfiguration('closest_idx_max_heading_err'),
            'idx_jump_confirm_dist': LaunchConfiguration('idx_jump_confirm_dist'),
            'idx_jump_confirm_cycles': ParameterValue(
                LaunchConfiguration('idx_jump_confirm_cycles'), value_type=int),
            'pose_suspect_speed': LaunchConfiguration('pose_suspect_speed'),
            # 자율 미체결 중 램프 고정 (2026-07-28)
            'engage_gate_enable': ParameterValue(
                LaunchConfiguration('engage_gate_enable'), value_type=bool),
            'drive_mode_topic': LaunchConfiguration('drive_mode_topic'),
            'engaged_mode_value': LaunchConfiguration('engaged_mode_value'),
            'drive_mode_timeout': LaunchConfiguration('drive_mode_timeout'),
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
