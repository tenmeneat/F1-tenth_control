from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# ============================================================================
# 시뮬/실차 런치파일 공용 헬퍼 (control_sim.launch.py / control_real.launch.py)
# ============================================================================
# 런치파일이 아니라 순수 헬퍼 모듈 — ros2 launch 진입점으로 직접 실행되지 않는다.
# 두 환경에서 100% 동일한 파라미터/노드 정의를 여기 한 곳에만 두어 드리프트를 막는다.
#
# IMU 단위 보정 계수 — 하드웨어 상수, 여기가 유일한 정의 위치.
# control_map_node의 조향 가감속 스케일러(acc_mean)가 소비한다.
# ⚠️ 2026-08-06: 각속도 계수(IMU_ANGULAR_SCALE_*)는 요레이트 카운터스티어 제거로 소비처가
#    없어져 함께 삭제했다. 자이로를 다시 쓰게 되면 **VESC는 deg/s로 발행**하므로
#    real = pi/180 = 0.0174533, sim = 1.0(sim_imu_bridge_node는 이미 rad/s 중계)로 되살릴 것.
IMU_LINEAR_SCALE_REAL = 9.80665      # g → m/s². VESC가 g로 발행(2026-07-19 소스 확인)
IMU_LINEAR_SCALE_SIM  = 1.0          # sim_imu_bridge_node는 0 고정

# ⚠️ 조이스틱 드라이버·sim_imu_bridge_node 포함 여부 등 안전 관련 구조 차이는
# 일부러 여기로 옮기지 않고 각 진입점 파일에 그대로 둔다(환경을 잘못 골라 안전
# 기능이 빠진 채 기동되는 실수를 구조적으로 차단하기 위함).


def declare_common_args():
    """두 런치파일에서 동일하게 쓰는 인자 선언 목록."""
    return [
        # ⚠️ force_autonomous·speed_to_erpm_gain 인자는 teleop 제거(2026-07-29)와 함께 폐지됐다 —
        #    유일한 소비처가 joy_teleop_monitor였다. 시뮬은 drive_source_selector가 자율 명령을
        #    /drive로 직결하므로 기동 즉시 자율주행이다.
        # ❌ 2026-08-06: yaw_rate_gain(요레이트 카운터스티어) 인자 제거 — 아래 "제거된 인자" 참고.

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

        # ── 경로소스 신선도 ──
        DeclareLaunchArgument(
            'local_fresh_timeout', default_value='0.3',
            description='이 시간(s) 넘게 /local_waypoints 미수신 시 글로벌 경로로 폴백'
        ),
        # 경로 진행방향 게이트 — 2026-08-07 run_0807_174227 크래시 방어.
        # state_machine이 Frenet s로 잘라 발행하는 /local_waypoints가 반대 브랜치로 튀면
        # 경로 전체가 뒤집힌 채 들어온다. 접선-헤딩 오차가 이 값을 넘는 점은 최근접 후보에서
        # 빼고, 로컬에 정합 후보가 하나도 없으면 로컬을 버리고 글로벌로 폴백한다.
        # 실측(run_0807_174227 50Hz 전수): 정상 주행 최소헤딩오차 최대 24.3°(p95 18.9°),
        # 경로 반전 구간 88~107°. 1.40 rad(80°)에서 오탐 0/159·검출 10/10.
        # ⚠️ 구 기본값 1.75(100°)는 3/10만 잡고 중간에 풀린다 — 되돌리지 말 것.
        DeclareLaunchArgument(
            'closest_idx_max_heading_err', default_value='1.40',
            description='경로 접선과 차량 헤딩의 허용 오차 [rad]. 0이면 게이트 비활성(구 거동)'
        ),
        # ── L1 Guidance 룩어헤드 거리 ──
        # 공식: L1 = clamp(l1_offset + v*l1_speed_gain, max(t_clip_min, sqrt2*lat_err), t_clip_max)
        # ⚠️ 2026-07-30 개명: l1_gain → l1_offset, l1_distance → l1_speed_gain.
        #    구 이름이 역할과 정반대였다(gain이 절편, distance가 기울기). 구 이름을 명령줄에
        #    넘기면 노드가 경고와 함께 여전히 받아주지만(호환 shim), 새 이름을 쓸 것.
        DeclareLaunchArgument(
            'l1_offset', default_value='0.70',
            description='L1 룩어헤드 거리의 **절편** [m] (공식: l1_offset + v*l1_speed_gain). '
                        '구 이름 l1_gain'
        ),
        DeclareLaunchArgument(
            'l1_speed_gain', default_value='0.3',
            description='L1 룩어헤드 거리의 **속도 계수** [s] (공식: l1_offset + v*l1_speed_gain). '
                        '구 이름 l1_distance'
        ),
        DeclareLaunchArgument(
            't_clip_min', default_value='0.9',
            description='L1 룩어헤드 거리 하한 [m] (낮을수록 저속/시케인 구간에서 국소 지그재그를 '
                        '쫓아 고주파 조향 유발 가능)'
        ),
        DeclareLaunchArgument(
            't_clip_max', default_value='5.0',
            description='L1 룩어헤드 거리 상한 [m]'
        ),
        # ⚠️ 예전엔 t_clip_min을 횡가속 분모 하한으로 재사용했다 — t_clip_min은 룩어헤드
        #    튜닝 노브라, 낮추면 조용히 횡가속 명령 상한이 올라갔다(0.6이면 6 m/s에서
        #    최대 lat_acc 120 m/s²). 수치 발산 방지용 하한은 별도 파라미터로 분리했다.
        DeclareLaunchArgument(
            'l1_min_denom', default_value='0.9',
            description='L1 횡가속 분모 하한 [m] (목표점이 차량에 붙었을 때 발산 방지). '
                        't_clip_min과 무관하게 튜닝'
        ),
        # 🔴 2026-08-07 신설. L1_distance는 실측 속도로 계산하는데 횡가속 게인은 프로파일
        #    속도를 써서, 둘이 갈라지면 게인이 (v_prof/v_meas)²배로 뛰던 것을 막는다.
        #    0807 실차 로그: 정지(실측 0.00) 상태에서 프로파일 4.01로 a_lat 13.93을 요구해
        #    LUT 포화 → 조향 −0.387(풀락 94%). 상세는 control_map_node.cpp 선언부 주석.
        DeclareLaunchArgument(
            'steering_speed_cap_measured', default_value='true',
            description='조향용 속도(횡가속 게인+LUT 조회)를 실측 속도로 상한. '
                        'false면 구 거동(프로파일 속도만 사용) — 롤백용'
        ),
        # 상태 한 줄(Pose/Target WP/Idx/Steer/Speed/L1_dist) 로그 주기. 구 500ms 고정에서
        # 2000ms로 완화 — 0807 실차 로그 1007줄 중 617줄이 이 줄이었다.
        # ⚠️ 이 줄은 rosbag에 없는 closest_idx/idx_a/L1_dist를 유일하게 보여준다(②-f 규명에 사용).
        #    디버깅 땐 500으로 낮추고, 정말 조용히 하려면 0으로 끌 것.
        DeclareLaunchArgument(
            'status_log_period_ms', default_value='2000',
            description='컨트롤러 상태 한 줄 로그 주기 [ms]. 0 = 끔 (디버깅 시 500 권장)'
        ),
        # 🔵 2026-08-07 신설. L1 목표점 점프 **검출**(주행 개입 없음, 순수 관측).
        #    08-04에 인덱스 점프 가드가 제거된 뒤 "룩어헤드가 튄다"는 보고가 있는데, 500ms
        #    상태로그로는 50Hz에서 자기수복되는 점프를 못 본다. 이 카운터가 그 창구다.
        DeclareLaunchArgument(
            'l1_jump_warn_m', default_value='1.0',
            description='L1 목표점이 차량보다 이만큼[m] 더 튀면 경고+카운트. 0 = 검출 끔'
        ),

        # ── 조향 체인 (2026-07-30 신설) ──
        # 명령각 중 바퀴가 실제로 내는 비율. 0.41 명령 → 실측 ~0.30(74%, 07-28 3회 재현,
        # 횡가속 1.09 m/s²라 슬립으론 설명 불가 = 기계적).
        # ⚠️ 이 값 하나가 두 곳을 지배한다: 조향 명령 보상(×1/ratio)과 조향 권한 속도 캡의
        #    δ_avail(×ratio). 예전엔 전자가 `clamp(1+v/10,1,1.4)` 하드코딩(≈1/0.74지만 속도
        #    램프 모양), 후자는 보상을 아예 모르는 상태로 어긋나 있었다.
        # 1.0 = 보상·캡 모두 구 낙관 거동. 각도기 실측 후 조정할 값.
        # 🔴 2026-07-31: 0.74 → 1.0. 74% 결손의 원인이 **서보 암 풀림**이었다.
        #    07-30에 팀원 주행 중 서보 암이 스플라인에서 빠졌고, 재장착(한 톱니 보정) 후
        #    각도기 실측: 11.46° 명령 → 좌 11° / 우 12° (게인 1.003, 중심 −0.5°).
        #    즉 링키지는 명령대로 도달한다. 0.74를 그대로 두면 1.35배 **과조향**이 되어
        #    그동안의 언더스티어와 반대 방향으로 스핀 위험이다.
        #    ⚠️ 풀락 부근은 링키지 기하가 비선형이라 좌 0.871(25.3° 명령 → 22°)로 떨어지지만,
        #       상수 보상은 중간각 기준이 맞다(끝단은 어차피 클리핑되고, 과보상이 더 위험).
        DeclareLaunchArgument(
            'steering_reach_ratio', default_value='0.85',
            description='명령 조향각 중 바퀴가 실제 도달하는 비율. 보상(1/ratio)과 조향권한 캡을 '
                        '동시 지배. 1.0 = 보상 없음(2026-07-31 실측: 링키지 정상)'
        ),
        # 50Hz에서 20 rad/s = 사이클당 0.4 rad = 풀락까지 2 사이클 = 구 하드코딩과 동일(무제한).
        # 서보 물리 속도(~7 rad/s 추정)로 낮추면 고주파 채터링을 막지만 실측 전이라 중립 유지.
        DeclareLaunchArgument(
            'max_steering_rate', default_value='20.0',
            description='조향 rate limit [rad/s] (dt 비례). 20.0 = 구 거동(사이클당 0.4rad)'
        ),
        # 가감속 조향 스케일러가 완전히 적용되는 기준 |종가속| [m/s²]. 예전엔 ±1.0 하드 임계라
        # 넘는 순간 조향이 5% 계단 점프했고, 실측 coast 감속 −0.4에선 감속측이 급제동
        # 스파이크에서만 드물게 튀었다. 0~ref 선형 블렌딩으로 바꿨다(ref 이상은 구 거동).
        DeclareLaunchArgument(
            'steering_scaler_accel_ref', default_value='1.0',
            description='가감속 조향 스케일러 완전 적용 기준 |a_x| [m/s²] (0~이 값 선형 블렌딩)'
        ),

        # ── 종방향 감속: 두 개의 서로 다른 감속도 (튜닝 방향이 정반대라 분리했다) ──
        #   base_max_decel = 명령 속도를 초당 얼마나 빨리 떨어뜨릴 수 있나(램프 rate limit) → 높게
        #   prebrake_decel = 차가 **실제로** 낼 수 있는 감속도(제동거리 v²/2a) → 실측값에 맞춤
        
        DeclareLaunchArgument(
            'base_max_decel', default_value='8.0',
            description='명령 속도 하강 rate limit [m/s^2]. 낮추면 감속 명령이 늦게 도달하므로 높게 유지'
        ),
        # ⚠️ 2026-07-30 1.0→2.5 상향(사용자 결정, 고속 주행 세팅). 실측 coast(-0.4)보다 제동거리를
        #    낙관적으로 보므로, 코너 진입이 늦게 느껴지면(언더스티어) 가장 먼저 되돌릴 값이다.
        DeclareLaunchArgument(
            'prebrake_decel', default_value='2.6',
            description='곡률 사전감속 제동거리 산출용 감속 권한 [m/s^2]. 낮을수록 코너를 일찍 봄'
        ),
        # ── 램프 안티와인드업 (2026-08-08 신설) ──
        # 램프는 차가 서 있어도 base_max_accel로 계속 올라간다. VESC 센서리스 탈조로 출발이
        # 지연되면 그동안 램프가 통째로 감겨, 바퀴가 물리는 순간 rate limit이 이미 소진된 채
        # 물리 한계로 튀어나간다 → 그 속도로 첫 코너 진입 = 언더스티어.
        # 0807 bag 10회: 관통 순간 명령 2.85~5.81 m/s, 관통 후 실가속 3.6~6.4 m/s²(의도 3.5).
        # 🔑 기본값 2.4는 VESC에서 유도된다 — s_pid_kp(0.006)로 l_current_max(60A)를 뽑는 데
        #    필요한 선행이 60/0.006 = 10000 ERPM ÷ 4336 = 2.31 m/s. 그 위 선행은 전류를 더
        #    만들지 못하는 순수 와인드업이라, 자르는 데 가속 손실이 없다.
        # ⚠️ 젯슨 s_pid_kp·l_current_max·speed_to_erpm_gain이 바뀌면 같이 재계산할 것.
        DeclareLaunchArgument(
            'ramp_lead_max', default_value='2.4',
            description='명령 속도 램프가 실측보다 앞설 수 있는 최대폭 [m/s]. 0이면 비활성(구 거동)'
        ),
        # ── 조향 권한 속도 캡 ──
        # 곡률 캡이 그립만 보던 구멍을 메운다. 그립("타이어가 그 횡가속을 낼 수 있나")과
        # 조향("바퀴가 그만큼 꺾일 수 있나")은 다른 물리다: δ = L·κ + K_us·κ·v² ≤ δ_avail 이면
        #   v ≤ √((ratio·δ_max − L·κ) / (K_us·κ))
        # 07-26 실차 κ=1.190(R=0.84m) 헤어핀에서 그립 2.11 m/s vs 조향 0.87 m/s — 조향이 먼저 걸린다.
        DeclareLaunchArgument(
            'understeer_gradient', default_value='0.014',
            description='언더스티어 그래디언트 K_us [rad/(m/s^2)]. 0이면 조향 권한 캡 비활성'
        ),
        DeclareLaunchArgument(
            'steer_authority_ratio', default_value='0.95',
            description='조향 한계 중 곡률 추종에 배정할 비율. 나머지는 횡오차·요레이트 보정 여유 '
                        '(1.0이면 보정 여력이 0)'
        ),
        # 전방 곡률 스캔 거리 = max(count*0.1, v²/(2·prebrake_decel)). count는 저속 하한.
        DeclareLaunchArgument(
            'curvature_lookahead_count', default_value='80',
            description='곡률 룩어헤드 스캔 거리 하한 (×0.1m). 80 = 8m'
        ),
        DeclareLaunchArgument(
            'min_speed', default_value='2.0',
            description='최저 순항 속도 [m/s] (곡률 감속 하한). 장애물 정지엔 미적용(0까지 허용)'
        ),

        # ── L1 횡가속 분모: 목표점까지의 실제 거리 ──
        # pure pursuit는 a_lat = 2·v²·sin(eta)/L_실제 인데, 목표점은 **호 길이**로 고르므로
        # |목표점−차량| != L1_distance다(07-27 bag 실측 비율 중앙 1.06~1.31 · p95 1.72).
        # 명목값을 분모로 쓰면 횡가속이 최대 +70% 과대해지고, 경로에서 벗어날수록 = 복귀가
        # 필요한 바로 그 순간에 더 심해진다. false면 구 거동(즉시 롤백용).
        DeclareLaunchArgument(
            'l1_use_actual_distance', default_value='true',
            description='L1 횡가속 분모로 목표점까지의 실제 직선거리 사용. false면 구 거동(명목 L1 거리)'
        ),

        # ── 자율 미체결 중 속도 램프 고정 (bumpless transfer) ──
        # 이 노드는 /drive_mode와 무관하게 상시 돌기 때문에 MANUAL/E-stop으로 서 있는 동안에도
        # 램프가 감겨 올라가고, engage 순간 그 값이 계단으로 VESC에 꽂힌다(07-27 bag 8개 전부:
        # 정차 중 명령 1.50~3.98 → 0→6348 ERPM 한 스텝 → 모터전류 60~62A 포화 → 급발진).
        # ⚠️ 체결 중에는 아무 일도 하지 않는다 — 07-22에 금지한 일반 lead-clamp와 다르다.
        #    VESC 속도 PID가 60A를 뽑는 데 필요한 명령 선행(~4.7 m/s)은 그대로 보존된다.
        # ⚠️ /drive_mode 미수신·끊김 시 게이트 자동 비활성(시뮬 호환).
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

        # ── 런치 킥 (자율 정지출발 시 VESC 센서리스 데드존 관통) ──
        # 매뉴얼은 초반 스로틀 펀치로 데드존을 때려 관통하는데, 자율은 살살 램프해 걸터앉아
        # 탈조한다. 정지 상태에서 짧게 높은 속도를 명령해 속도 PID가 큰 전류를 뽑게 만든다.
        # ⚠️ s_pid_ramp_erpms_s가 2000→21160으로 오른 뒤로는 이 킥이 훨씬 사납다(즉시 큰 ERPM
        #    오차 → 큰 전류). 부스트 속도를 올릴 땐 반드시 잭업 상태에서 먼저 볼 것.
        DeclareLaunchArgument(
            'launch_boost_enable', default_value='false',
            description='런치 킥 on/off (자율 정지출발 데드존 관통 펀치)'
        ),
        DeclareLaunchArgument(
            'launch_boost_speed', default_value='2.0',
            description='데드존 관통용 펀치 속도 명령 [m/s]'
        ),
        DeclareLaunchArgument(
            'launch_boost_time', default_value='0.6',
            description='관통 실패 시 포기까지 최대 펀치 시간 [s]'
        ),
        DeclareLaunchArgument(
            'launch_exit_speed', default_value='0.8',
            description='실측이 이 속도[m/s] 넘으면 관통 성공 판정 → 킥 종료(데드존 상단 0.59보다 위)'
        ),
        DeclareLaunchArgument(
            'launch_standstill_speed', default_value='0.3',
            description='실측이 이 속도[m/s] 미만이면 정지 판정 → 킥 시작(exit보다 낮아 히스테리시스)'
        ),

        # IMU 보정 on/off. 끄면 조향 가감속 스케일러가 중립(acc_mean=0)으로 떨어져
        # 순수 L1+LUT(시뮬 검증 상태)가 된다.
        DeclareLaunchArgument(
            'use_imu', default_value='true',
            description='IMU 종가속(조향 가감속 스케일러) 사용 여부. false면 순수 L1+LUT 주행'
        ),

        # ❌ 2026-08-06 제거된 인자 — 되살리기 전에 control_map_node.cpp의 해당 ❌ 주석을 읽을 것.
        #    전부 **기본값이 비활성**이라 제거로 인한 실거동 변화는 없다.
        #    · yaw_rate_gain(0.00)            — 요레이트 카운터스티어. 컨트롤러가 언더스티어를
        #                                       **판정**하는 항이라 planning 전담 방침과 충돌
        #    · deadzone_floor_speed(0.0)      — 플래닝 요구보다 빠르게 가는 권한 침범
        #    · launch_align_{enable,speed,time} — 정상 출발 효과 0~25%. 출발 덜그럭의 원인은
        #                                       조향이 아니라 VESC 오픈루프 기동 시퀀스였다
    ]


def build_control_map_node(*, odom_topic, max_speed, max_lateral_accel, base_max_accel,
                            imu_linear_scale,
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
            'l1_offset': LaunchConfiguration('l1_offset'),
            'l1_speed_gain': LaunchConfiguration('l1_speed_gain'),
            't_clip_min': LaunchConfiguration('t_clip_min'),
            't_clip_max': LaunchConfiguration('t_clip_max'),
            'l1_min_denom': LaunchConfiguration('l1_min_denom'),
            'steering_speed_cap_measured': LaunchConfiguration('steering_speed_cap_measured'),
            'status_log_period_ms': ParameterValue(
                LaunchConfiguration('status_log_period_ms'), value_type=int),
            'l1_jump_warn_m': LaunchConfiguration('l1_jump_warn_m'),
            # ⚠️ lateral_error_coeff는 2026-07-30에 폐지됐다 — 소비처인 lat_err_scale이
            #    항상 1.0인 죽은 코드였다(control_map_node.cpp control_loop 4 주석 참고).
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
            'ramp_lead_max': LaunchConfiguration('ramp_lead_max'),
            'launch_boost_enable': ParameterValue(LaunchConfiguration('launch_boost_enable'), value_type=bool),
            'launch_boost_speed': LaunchConfiguration('launch_boost_speed'),
            'launch_boost_time': LaunchConfiguration('launch_boost_time'),
            'launch_exit_speed': LaunchConfiguration('launch_exit_speed'),
            'launch_standstill_speed': LaunchConfiguration('launch_standstill_speed'),
            'l1_use_actual_distance': ParameterValue(
                LaunchConfiguration('l1_use_actual_distance'), value_type=bool),
            # ⚠️ 좌우 조향 한계는 진입점 런치가 환경별로 넘긴다. 실차는 젯슨 vesc.yaml의
            #    servo_min/max와 **반드시 한 쌍** — 컨트롤러만 올리면 vesc_driver가 조용히
            #    자르고 컨트롤러는 꺾었다고 착각한다. 시뮬은 차량 모델이 대칭이라 ±0.41.
            'max_steering_left': max_steering_left,
            'max_steering_right': max_steering_right,
            # 조향 체인 (2026-07-30)
            'steering_reach_ratio': LaunchConfiguration('steering_reach_ratio'),
            'max_steering_rate': LaunchConfiguration('max_steering_rate'),
            'steering_scaler_accel_ref': LaunchConfiguration('steering_scaler_accel_ref'),
            # 자율 미체결 중 램프 고정 (2026-07-28)
            'engage_gate_enable': ParameterValue(
                LaunchConfiguration('engage_gate_enable'), value_type=bool),
            'drive_mode_topic': LaunchConfiguration('drive_mode_topic'),
            'engaged_mode_value': LaunchConfiguration('engaged_mode_value'),
            'drive_mode_timeout': LaunchConfiguration('drive_mode_timeout'),
            'lookup_table_file': lookup_table_file,
            'use_imu': ParameterValue(LaunchConfiguration('use_imu'), value_type=bool),
            'imu_linear_scale': imu_linear_scale,
            'curvature_ff_blend': 0.0,
            'heading_damping_gain': 0.2,
            'acceleration_scaler_for_steering': LaunchConfiguration('acceleration_scaler_for_steering'),
            'deceleration_scaler_for_steering': LaunchConfiguration('deceleration_scaler_for_steering'),
            'start_scale_speed': LaunchConfiguration('start_scale_speed'),
            'end_scale_speed': LaunchConfiguration('end_scale_speed'),
            'downscale_factor': LaunchConfiguration('downscale_factor'),
            'speed_lookahead': LaunchConfiguration('speed_lookahead'),
            'speed_lookahead_for_steering': LaunchConfiguration('speed_lookahead_for_steering'),
            'local_fresh_timeout': LaunchConfiguration('local_fresh_timeout'),
            'closest_idx_max_heading_err': LaunchConfiguration('closest_idx_max_heading_err'),
        }]
    )

# ⚠️ build_control_mppi_node()는 2026-08-01 MPPI 노드/솔버 전체 제거와 함께 삭제됐다 —
#    대회 준비 기간 동안 MAP(control_map_node) 하나에만 집중하기로 함. 되살리려면 git
#    이력에서 control_code/control_mppi_{node,solver_cpu.cpp,solver_gpu.cu}와
#    include/f1tenth_control/mppi_{gpu,types_gpu}.hpp를 함께 복원할 것.

# ⚠️ build_joy_teleop_monitor()는 teleop 제거(2026-07-29)와 함께 삭제됐다. 수동/자율/E-stop
#    Mux는 이 저장소 담당이 아니다 — 실차는 f1tenth_stack(drive_mode_manager + ackermann_mux),
#    시뮬은 Mux 없이 drive_source_selector가 자율 명령을 /drive로 직결한다.
