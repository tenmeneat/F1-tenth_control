import os
import sys
sys.path.insert(0, os.path.dirname(__file__))

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import _control_common as common


def generate_launch_description():
    # ==========================================================================
    # 실차용 런치 (첫 실주행 셰이크다운 보수 프리셋)
    # ==========================================================================
    # 시뮬 런치(control_sim.launch.py)와의 차이 = 실차 안전/환경 정합:
    #   - odom_topic: /pf/pose/odom (파티클필터) — 시뮬의 /ego_racecar/odom 대체
    #   - use_imu=True: VESC 내장 IMU 영점/세팅 완료 → 롤 인지 ESC 활성
    #   - 비상제동(AEB)은 제어 파트에서 제거됨 — 실제 비상정지는 planning 파트가 판단/발행
    #   - 수동/자율/E-stop Mux는 f1tenth_stack(drive_mode_manager + ackermann_mux)이 담당 →
    #     우리 joy_teleop_monitor는 이 런치에서 제외(2026-07-17). 대신 drive_source_selector가
    #     control_map_node의 /drive_autonomous를 그대로 /drive(navigation 채널)로 포워딩한다
    #     (2026-08-01 MPPI 제거 후 순수 포워더, 아래 참고).
    #   - joy_node(조이스틱 드라이버)도 이 런치에 없음 — f1tenth_stack이 라이다/조이스틱/vesc
    #     드라이버를 함께 기동하므로 중복 방지 위해 제거(2026-07-14).
    #   - ackermann_to_vesc_node도 없음(f1tenth_stack이 자체 기동, 아래 참고)
    # 전제: 하드웨어 브링업(f1tenth_stack의 drive_mode_manager, ackermann_mux, vesc_driver,
    #       ackermann_to_vesc, LiDAR, joy_node)과 particle_filter, planning이 /scan, /joy,
    #       /pf/pose/odom, /global_waypoints 를 발행 중이어야 함.
    # 공통 파라미터(wheelbase, l1_offset 등)는 _control_common.py 참고 — 시뮬과 겹치는 부분은
    # 거기 한 곳만 고치면 됨. 아래는 실차 전용 인자/노드만.

    # 파티클필터 odom 토픽 (로컬라이제이션 스택에 맞춰 변경 가능)
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/pf/pose/odom',
        description='실차 위치추정 odom 토픽 (파티클필터/EKF)'
    )

    # 직선 최대 속도 [m/s] — 2026-07-30 5.0→8.0 상향(사용자 결정, 고속 주행 세팅).
    # ⚠️ 하드웨어 ERPM(40000) 상한 = 바퀴 ~9 m/s. 8.0은 그 약 89% 수준 — 여유가 작으므로
    #    직선 끝 제동 여력과 프로파일 vx가 실제 상한을 결정한다.
    # ⚠️ 실제 도달 최고속은 7.4 m/s다(전방-후방 패스 시뮬). 트랙 최장 직선이 13.3 m라
    #    8.0에 닿기 전에 다음 코너 제동이 시작된다 — max_speed를 올려도 이득이 없고,
    #    최고속 레버는 base_max_accel(아래)과 제동 권한이다.
    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='8.0',
        description='control_map_node 직선 최대 속도 [m/s]'
    )

    # max_lateral_accel = backward-pass 사전감속 그립 클램프 min(프로파일 vx, sqrt(a_lat/kappa)).
    # 실차 기본값을 6.5로 보수화(2026-07-23) — LUT 실 그립 마찰피크(~6.7) 이내로 잡아 코너에서
    # 실 타이어 그립을 초과하지 않도록. 프로파일을 공격적으로 재생성해 코너속도가 sqrt(6.5/kappa)를
    # 넘으면 backward-pass가 여기서 클램프한다. 실그립 매칭 프로파일+저속 셰이크다운 후, 여유가
    # 확인되면 상향 가능(sim 런치는 여전히 낙관치 10.0 — 랩타임 튜닝 기준 유지).
    # 2026-07-30 5.1→6.0 상향(사용자 결정) — LUT 실 그립 피크(~6.7) 이내이나 구 실측
    # 마찰한계(~3.1 m/s²)보다는 낙관치. 코너 슬라이드/언더스티어 시 다시 낮출 것.
    max_lateral_accel_arg = DeclareLaunchArgument(
        'max_lateral_accel', default_value='6.0',
        description='코너 그립 클램프 a_lat [m/s^2] (LUT 그립 피크 ~6.7 이내)'
    )

    # ── 좌우 조향 한계 (2026-07-28, 실차 전용) ──
    # 젯슨 vesc.yaml의 servo_min 0.2703 / servo_max 0.6363 은 servo **0.4533 중심 ±0.1830**
    # (= ±0.41 rad)으로 계산된 낡은 값이다. 이후 기계적 센터를 맞추며 트림 offset이
    # **0.4672로 의도적으로 이동**됐는데 servo_min/max는 갱신되지 않아, 실제 가동각이
    # 좌 +0.441 / 우 **-0.379(잘림)**로 갈려 있었다. 07-27 bag(run_0727_203040)에서
    # servo 0.6502가 발행돼 vesc_driver의 servo_limit에 잘린 것이 실제로 관측된다.
    #
    # 🔧 대칭화 시도(2026-07-28)는 롤백됐었다 — **그 롤백 사유는 2026-07-31 기준 해소됐다.**
    #    당시엔 odom이 조향 **명령**에서 요레이트를 합성해서(`use_servo_cmd_to_calc_angular_
    #    velocity: true`), servo 범위를 넓히면 클립이 사라지고 odom이 링키지가 못 가는 각을
    #    믿어 하드 우회전마다 요레이트를 8% 과적분했다(SLAM 헤딩 뭉개짐).
    #    → 07-28에 `use_imu_for_angular_velocity: true`로 **실측 자이로** 기반이 되면서
    #      이 결합이 끊겼다. 그래서 아래 07-31 재설정에서는 범위를 넓혀도 odom이 안 깨진다.
    #    ⚠️ 단 IMU가 imu_timeout(0.2s) 넘게 끊기면 **구 방식으로 자동 폴백**하므로 그 위험이
    #      완전히 사라진 건 아니다 — 아래 "클리핑 로그 → 시리얼 기아" 항목이 그 트리거다.
    # ── 2026-07-31 각도기 실측으로 재설정 (서보 암 재장착 후) ──────────────────────
    # 07-30에 서보 암이 스플라인에서 빠진 걸 재장착(한 톱니 보정)한 뒤 전 구간 실측했다.

    #       좌 (0.23−0.4672)/−0.4463 = 0.5315,  우 (0.66−0.4672)/−0.4463 = 0.4320
    #
    # ⚠️ 명령각이 비대칭인 것이 **맞다.** 좌측 링키지가 우측보다 servo를 21.9% 더 써야 같은
    #    각이 나와서(좌 도달비 0.756 / 우 0.922), 명령을 비대칭으로 줘야 **실제 바퀴 각이
    #    좌우 23.5°로 대칭**이 된다. 조향 권한 캡은 min(좌,우)=0.4320(더 선형인 우측)을 쓰므로
    #    정확한 쪽이 기준이 된다 → δ_avail = 0.85 × 0.4320 × 1.0 = 0.367 rad.
    #    ℹ️ 더 근본적으로는, 서보 중립 트림(0.4672)이 가동범위 [0.23, 0.66]의 정중앙(0.445)이
    #       아니라서 중립 기준 가용 스트로크가 좌 0.2372 / 우 0.1928로 애초에 다르다.
    #       비대칭 명령 한계는 그 스트로크를 정확히 다 쓰기 위한 계산 결과다.
    #
    # ❌ 2026-08-03 기각: "좌우 공용 게인이라 비대칭 한계가 무의미하다 → 젯슨에 좌/우 개별
    #    게인(-0.5785 / -0.4702)을 두고 명령 한계를 ±0.410 대칭으로" 안을 검토했다가 **취소**.
    #    발단은 0803 bag에서 좌회전 달성 곡률이 우회전 대비 17% 낮게(좌 κ p99 0.992 /
    #    우 1.200) 나온 것이었는데, **조향각과 속도를 함께 맞춰 재측정하니 좌/우 비가
    #    0.50~1.95로 방향 없이 흩어졌다 — 비대칭은 실재하지 않는다.** 이 트랙은 좌회전
    #    표본이 압도적이고(4875 vs 2712) 가장 급한 코너가 전부 좌회전이라, 좌측 표본에
    #    어려운 기동이 몰린 교란이었다.
    #    → 게다가 각도기 실측상 **중간각은 이미 1:1**(11.46° 명령 → 좌 11°/우 12°)이고
    #       비선형은 풀락 부근에만 있다. 풀락 기준으로 뽑은 선형 게인을 넣었다면 중간각에서
    #       좌측을 ~30% **과조향**시킬 뻔했다(스핀 방향).
    #    젯슨 ackermann_to_vesc에 좌/우 개별 게인 파라미터 자체는 남겨뒀으나
    #    (steering_angle_to_servo_gain_left/right) **0.0 = 비활성**이라 거동은 종전과 같다.
    #    되살리려면 각도기로 servo↔바퀴각 곡선을 여러 점 실측해 비선형까지 잡을 것.
    #
    # 🔴 이 값과 젯슨 vesc.yaml의 servo_min/servo_max는 **반드시 함께** 움직인다.
    #    컨트롤러만 넓히면 vesc_driver가 자르는데, 그때 **매 메시지마다 클리핑 로그가 찍히면서
    #    VESC 시리얼을 굶겨 /sensors/imu가 50Hz → 39Hz(최대 0.33s 공백)로 떨어지고,
    #    imu_timeout(0.2s)를 넘겨 odom이 "조향명령 기반" 구식 경로로 폴백한다**
    #    (2026-07-31 실측 재현). 그 경로는 선회 중 헤딩을 +36~39% 과적분한다.
    #    반대로 vesc.yaml만 넓히면 컨트롤러가 못 내는 각을 안 쓰게 되어 손해만 본다.
    #    → 08-03에 ackermann_to_vesc가 발행 전에 스스로 [servo_min, servo_max]로 클램프하도록
    #      고쳐서, 혹시 어긋나도 driver의 클리핑 로그 폭주 경로는 타지 않는다(안전망).
    # 🔴 2026-08-05: 좌우 대칭 0.410 으로 변경. 젯슨 vesc.yaml 의 **좌우 개별 조향 게인**
    #    (steering_angle_to_servo_gain_left -0.5785 / _right -0.4702) 활성화와 **한 쌍**이다.
    #    개별 게인을 켜면 steering_angle 의 의미가 "단일 게인용 명령값"에서
    #    **실제 바퀴 각(rad)** 으로 바뀌므로, 한계도 실측 풀락각인 좌우 대칭 0.410 이 된다.
    #    ⚠️ 둘 중 하나만 배포하면 안 된다 — 젯슨이 구 방식(개별게인 0.0=폴백)인데 여기만
    #       0.410 이면 좌조향이 23.5°가 아니라 19.3°까지밖에 안 나온다(손해만 봄).
    #
    #    왜 바꿨나 (0805 bag `run_0805_150603` 실측):
    #      정상상태 자전거 모델로 역산한 K_us 가 **좌 0.0338 / 우 0.0175 = 좌회전
    #      언더스티어가 우회전의 약 2배**. 이 트랙은 좌선회 39% / 우선회 8% 라 좌코너마다
    #      바깥으로 밀려 **직선 진입 크로스트랙이 매 랩 −0.46 m(σ 1.6 cm)** 로 반복됐다.
    #      단일 게인 -0.4463 은 풀락에서만 좌우가 맞고 중간각이 전부 어긋난 게 원인
    #      (좌측 링키지가 같은 바퀴각에 servo 를 21.9% 더 씀).
    #    검산: 개별게인+0.410 이면 풀락 servo 가 좌 0.230015 / 우 0.659982 로
    #      [servo_min 0.23, servo_max 0.66] **안쪽**에 들어온다(클리핑 없음).
    # 🔴 2026-08-06: 우편향 보정으로 젯슨 offset 을 0.4672 → 0.4591 로 0.8° 좌 트림했다.
    #    중립이 좌로 간 만큼 **좌측 가동범위가 줄어든다** — servo_min 까지 남은 각이
    #    (0.4591−0.23)/0.5785 = 0.3959 rad 뿐이다. 그래서 좌만 0.410 → 0.395 로 낮췄다.
    #    (우는 (0.66−0.4591)/0.4702 = 0.4273 까지 여유가 생겼으므로 0.410 유지)
    #    ⚠️ 0.410 을 그대로 두면 풀락 좌조향에서 servo 0.2219 → servo_min 클리핑 →
    #       클리핑 로그가 VESC 시리얼을 굶겨 /sensors/imu 50→39 Hz → imu_timeout 초과 →
    #       vesc_to_odom 이 구식 조향명령 방식으로 폴백(선회 헤딩 +36~39% 과적분).
    #    ⚠️ 젯슨 offset 을 되돌리면 이 값도 같이 0.410 으로 되돌릴 것.
    # 🔴 2026-08-07: 서보암이 다시 빠져(08-06 야간) 재장착 → 08-06 의 0.8° 좌 트림
    #    (offset 0.4591 / 좌 0.395)은 기계 기준점이 사라져 무의미해졌다. 젯슨 offset 을
    #    대칭 기준값 0.4672 로 되돌렸으므로 좌우 모두 0.410 이 맞다:
    #      좌 (0.4672-0.23)/0.5785 = 0.4100  /  우 (0.66-0.4672)/0.4702 = 0.4100
    #    ⚠️ 이 값은 "servo 0.4672 에서 바퀴가 직진"이라는 전제 위에서만 맞다. 암 재장착
    #       실측 트림이 끝나기 전에는 주행 금지 — 중립이 어긋난 채면 한쪽이 스톱에 물린다.
    max_steering_left_arg = DeclareLaunchArgument(
        'max_steering_left', default_value='0.410',
        description='좌조향(δ>0) 명령 한계 [rad] = 실제 바퀴 각. 젯슨 vesc.yaml 의 '
                    'steering_angle_to_servo_gain_left(-0.5785) + offset(0.4672) 과 한 쌍'
    )
    max_steering_right_arg = DeclareLaunchArgument(
        'max_steering_right', default_value='0.410',
        description='우조향(δ<0) 명령 한계 [rad] = 실제 바퀴 각. 젯슨 vesc.yaml 의 '
                    'steering_angle_to_servo_gain_right(-0.4702) + offset(0.4672) 과 한 쌍'
    )

    # 비워두면 기존 폴백 순서(f1tenth_control share → steering_lookup share)로 로드.
    # tools/lut_calibrator가 만든 보정 LUT를 임시로 켜보려면 그 출력 경로를 지정할 것
    # (예: $HOME/f1tenth_lut_calibration/LUT_calibrated.csv). 검증 끝나면 그 파일을
    # control_code/LUT_calibrated.csv에 덮어써서 디폴트로 확정하는 편이 낫다(CLAUDE.md 참고).
    lookup_table_file_arg = DeclareLaunchArgument(
        'lookup_table_file', default_value='',
        description='Steering LUT CSV 경로 (비워두면 기본 폴백 사용, 캘리브레이션 결과 적용 시 지정)'
    )

    # 종방향 최대 가속도 한계 [m/s^2]. base_max_decel은 sim/real 동일이라 _control_common.py에 공용.
    #
    # 🔑 **천장은 컨트롤러가 아니라 VESC다.** mcconf의 `s_pid_ramp_erpms_s`가 속도 setpoint의
    #    상승률을 하드 제한한다: 21160 ERPM/s ÷ 4336 = **4.88 m/s²**(2026-08-05 상향).
    #    이보다 크게 주면 VESC가 깎을 뿐이고, 명령만 실측보다 더 앞서 나가 와인드업 위험만
    #    커진다(07-27 급발진과 같은 구조).
    #    ⚠️ **3.5는 이제 천장(4.88)의 72%다** — 2026-08-05에 램프를 15600(3.60)에서 21160으로
    #       올려 여유를 만들어 뒀다. 값을 올리는 건 `base_max_accel:=X` 인자로 즉시 되지만,
    #       **컨트롤러가 제한을 쥐고 있어야** 곡률 사전감속의 전방-후방 패스 예측이 실제와
    #       맞으므로 천장까지 다 열지는 말 것(천장 = VESC가 깎기 시작하는 지점).
    #    ⚠️ 게인 4336은 2026-08-04 세미슬릭 타이어 교체 후 재보정값이다(구 4232). VESC에서
    #       s_pid_ramp_erpms_s나 젯슨에서 speed_to_erpm_gain을 바꾸면 이 값도 같이 재검토할 것.
    #
    # 2026-07-31 2.5 → 3.5. 전방-후방 패스 시뮬(ifac_track_v2, a_lat 6.0 / decel 2.5 / v_max 8.0):
    #   랩타임 11.97 → 11.63 s (−0.34 s, −2.8%), 최고 도달속 6.93 → 7.44 m/s.
    #   ℹ️ `max_speed`는 8.0이지만 실제 도달은 7.4다 — 트랙이 46.9 m(최장 직선 13.3 m)라
    #      8.0에 닿기 전에 다음 코너 제동이 시작된다. **max_speed를 더 올려도 무의미하고**,
    #      최고속을 올리는 유일한 레버가 이 가속도다.
    #   ℹ️ 더 큰 레버는 제동 쪽에 있다 — 젯슨 brake_max_current 8.0A ≈ 4.8 m/s²가 하드웨어
    #      한계인데 brake_gain은 −2.5 목표로만 튜닝돼 있다. 제동을 4.8까지 쓰면 −0.62 s,
    #      가속·제동 둘 다 올리면 −1.32 s(−11%)다. 단 brake_gain과 prebrake_decel은 한 쌍.
    base_max_accel_arg = DeclareLaunchArgument(
        'base_max_accel', default_value='3.5',
        description='종방향 최대 가속도 한계 [m/s^2]. VESC 천장은 s_pid_ramp_erpms_s(21160 ÷ 4336 '
                    '= 4.88) — 3.5는 그 72%라 여유가 있다. 4.88을 넘겨 주면 VESC가 깎고 '
                    '와인드업 위험만 커진다'
    )

    steering_control = common.build_control_map_node(
        odom_topic=LaunchConfiguration('odom_topic'),
        max_speed=LaunchConfiguration('max_speed'),
        max_lateral_accel=LaunchConfiguration('max_lateral_accel'),
        base_max_accel=LaunchConfiguration('base_max_accel'),
        max_steering_left=LaunchConfiguration('max_steering_left'),
        max_steering_right=LaunchConfiguration('max_steering_right'),
        lookup_table_file=LaunchConfiguration('lookup_table_file'),
        # VESC 가속도계가 g로 발행(2026-07-19 소스 확인) → m/s² 환산.
        imu_linear_scale=common.IMU_LINEAR_SCALE_REAL,
        # vesc_driver_node는 IMU를 sensors/imu/raw로 발행하지만 control_map_node.cpp는
        # /imu/data를 구독하도록 하드코딩돼있어 리매핑 필요(안 하면 IMU 미수신 → 롤 ESC/
        # 요레이트 카운터스티어가 조용히 무효화됨).
        remappings=[('/imu/data', 'sensors/imu/raw')],
    )

    # ── MPPI 배선 완전 제거 (2026-08-01) ─────────────────────────────────────
    # 실차 bag 3건(07-26 20:41, 07-27 19:44/20:25) 모두에서 /drive_mppi가 50Hz 목표 대비
    # 10Hz밖에 못 나왔고(solve ~100ms, 실시간 예산 20ms의 5배), 실제로는 MAP만 쓰고 있어
    # 출력은 버려지는 순수 낭비였다(07-27부터 이 런치에서 배선 제외). 대회가 한 달 앞으로
    # 다가와 MAP 하나에만 집중하기로 하고 2026-08-01에 MPPI 노드/솔버/셀렉터 로직 전체를
    # 코드베이스에서 제거했다 — 되살리려면 git 이력에서
    # control_code/control_mppi_{node,solver_cpu.cpp,solver_gpu.cu},
    # include/f1tenth_control/mppi_{gpu,types_gpu}.hpp,
    # _control_common.py의 build_control_mppi_node()를 함께 복원할 것.
    #
    # 실차 수동/자율/E-stop Mux는 팀 공용 f1tenth_stack이 담당한다(drive_mode_manager +
    # ackermann_mux). 따라서 우리 joy_teleop_monitor는 이 런치에서 제외한다(2026-07-17) —
    # 띄우면 /drive를 이중 발행해 f1tenth_stack의 navigation 입력과 충돌한다.
    #
    # drive_source_selector는 이제 MAP 출력(/drive_autonomous)을 그대로 /drive로 포워딩만
    # 한다. /drive는 f1tenth_stack ackermann_mux의 navigation 입력(우선순위10)과 토픽명이
    # 일치해 자동으로 흘러들어간다. E-stop은 drive_mode_manager가 estop_lock으로 mux 전체를
    # 마스킹하므로 이 노드가 계속 /drive를 내보내도 제동 중엔 차단된다(셀렉터는 E-stop을
    # 몰라도 됨).
    drive_source_selector = Node(
        package='f1tenth_control',
        executable='drive_source_selector',
        name='drive_source_selector',
        output='screen',
    )

    # ackermann_to_vesc_node도 이 launch에 없다(2026-07-17 제거). f110(f1tenth_stack)이 이미
    # 자체 ackermann_to_vesc_node를 띄우고, 그 입력('ackermann_drive')은 자체 ackermann_mux가
    # 'teleop'(drive_mode_manager 수동, 우선순위100)과 'drive'(navigation, 우선순위10 — 위
    # 셀렉터 출력)를 중재해 만든다. 우리가 또 띄우면 같은 VESC 명령 토픽에 중복 발행되어
    # 경합(조향 덜컹거림)한다.
    # ⚠️ 전제조건(f1tenth_stack 쪽, 이 repo 밖):
    #   1. drive_mode_manager가 AUTONOMOUS 모드에서 teleop을 침묵시켜 navigation('drive')이
    #      mux에서 이길 수 있어야 함(구 joy_teleop.yaml default 섹션 상시발행 이슈는 해소됨).
    #   2. vesc.yaml의 조향 캘리브레이션이 2026-07-21 확정값과 동기화돼 있어야 함:
    #        steering_angle_to_servo_offset: 0.4672   # 조향 중립(07-20 직진편향 실측 확정)
    #        steering_angle_to_servo_gain:   -0.4463  # 실측 민감도 0.00779 서보/도 기반
    #        servo_min / servo_max: 0.2703 / 0.6363   # 명령범위를 ±0.41 rad에 1:1로 맞춤
    #      ⚠️ offset은 0.4633(07-17)도 0.4533(07-21 일시적용)도 아니다. 0.4533은 "서보 0.5가
    #      조향 중립"이라는 오해로 6°를 한 번 더 뺀 값이라 좌측 편향이 생겨 원복했다 —
    #      서보의 기계 중앙(0.5)과 조향 중립(0.4672)은 원래 다르다. 경위는 WORKLOG 07-21 참고.
    #      ⚠️ gain·servo_min/max는 서보가 스토퍼를 상시 밀던 결함을 고친 것이라 offset과 달리
    #      07-21 값이 유효하다. 이 게인은 `/**:` 공유 파라미터라 vesc_to_odom에도 함께 걸린다.

    return LaunchDescription([
        *common.declare_common_args(),
        odom_topic_arg,
        max_speed_arg,
        max_lateral_accel_arg,
        max_steering_left_arg,
        max_steering_right_arg,
        lookup_table_file_arg,
        base_max_accel_arg,
        steering_control,
        drive_source_selector,
    ])
