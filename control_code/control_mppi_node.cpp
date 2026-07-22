// ============================================================================
// control_mppi_node.cpp — MPPI 자율주행 제어 ROS 2 노드
// ----------------------------------------------------------------------------
// control_map_node.cpp(L1+LUT)와 동급의 상시 구동 컨트롤러 노드. 글로벌 경로를 추종하되
// L1 대신 샘플링 기반 MPPI로 조향+종가속을 동시 최적화한다. 결과를 `/drive_mppi`로
// 발행하고, RB 버튼 상태에 따라 `/drive_autonomous`(MAP) ↔ `/drive_mppi`(MPPI) 중 하나가
// 최종 `/drive`로 라우팅된다. 라우터는 환경별로 다르다 — 시뮬은 joy_teleop_monitor,
// 실차는 drive_source_selector(실차엔 joy_teleop_monitor가 없음).
//
// 솔버 = 컴파일 타임 자동선택:
//   - CUDA 있으면(USE_MPPI_GPU) GPU 솔버(control_mppi_solver_gpu.cu, float32, 병렬 롤아웃)
//   - 없으면 CPU 솔버(control_mppi_solver_cpu.cpp, double, 순차)
//   두 솔버는 구조체 필드명이 동일해 using 별칭 한 벌로 본문을 공유한다.
//
// ⚠️ MPPI 전방 롤아웃은 자체 Pacejka 모델(gym 기본값)을 쓴다 — NUC6 LUT(역방향 맵)는
//    못 씀. 실차 Pacejka 보정은 별도 작업(범위 밖).
// ⚠️ 갭팔로워는 이 노드에 없음. 비상제동은 제어 파트에서 제거됨 — planning 파트가 판단/발행.
// ============================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "f110_msgs/msg/wpnt_array.hpp"
#include "std_msgs/msg/bool.hpp"

// --- 솔버 선택 (CMake가 CUDA 감지 시 USE_MPPI_GPU 정의 + GPU 솔버 링크) ---
#ifdef USE_MPPI_GPU
#include "f1tenth_control/mppi_gpu.hpp"
using Solver   = f1tenth_control::MPPIControllerGPU;
using SState   = f1tenth_control::MppiStateF;
using SRef     = f1tenth_control::MppiRefF;
using SControl = f1tenth_control::MppiControlF;
using SParams  = f1tenth_control::MppiParamsF;
#else
#include "control_mppi_solver_cpu.cpp"  // 헤더 없는 CPU 클래스를 직접 include(가드된 main은 안 딸려옴)
using Solver   = f1tenth_control::MPPIController;
using SState   = f1tenth_control::MppiState;
using SRef     = f1tenth_control::MppiRef;
using SControl = f1tenth_control::MppiControl;
using SParams  = f1tenth_control::MppiParams;
#endif

class ControlMppiNode : public rclcpp::Node {
 public:
    ControlMppiNode() : Node("control_mppi_node") {
        // 1. 파라미터 선언·1회 읽기 (control_map_node 패턴: 콜백 없음, 재시작 시 반영)
        // double로 선언·읽기 → float/double 솔버 파라미터에 대입(narrowing 허용).
        auto dparam = [&](const std::string& name, double def) {
            return this->declare_parameter<double>(name, def);
        };
        auto iparam = [&](const std::string& name, int def) {
            return this->declare_parameter<int>(name, def);
        };

        odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/ego_racecar/odom");

        // --- MPPI 코어 ---
        p_.N           = iparam("N", p_.N);
        // K(롤아웃 수)는 0이면 **솔버별 자동**. 샘플링 노이즈(직선 미세 사행)의 분산이
        // 대략 1/ESS라 K를 키우면 그대로 줄어드는데(07-22 실측: 512→2048에서 지터 0.034→0.020),
        // GPU는 K=2048도 0.4ms라 공짜인 반면 CPU 순차 솔버는 20ms 예산을 넘긴다.
        {
            const int k_arg = iparam("K", 0);
#ifdef USE_MPPI_GPU
            const int k_auto = 2048;
#else
            const int k_auto = 512;
#endif
            p_.K = (k_arg > 0) ? k_arg : k_auto;
        }
        p_.dt          = dparam("dt", p_.dt);
        p_.lambda      = dparam("lambda", p_.lambda);
        p_.lambda_rel  = dparam("lambda_rel", p_.lambda_rel);
        p_.sigma_steer = dparam("sigma_steer", p_.sigma_steer);
        p_.sigma_accel = dparam("sigma_accel", p_.sigma_accel);
        p_.noise_beta  = dparam("noise_beta", p_.noise_beta);
        p_.substeps    = iparam("substeps", p_.substeps);
        // --- 차량/타이어 (Pacejka는 gym 기본값 유지 — 실차 보정 전까지 노출 최소화) ---
        p_.wheelbase   = dparam("wheelbase", p_.wheelbase);
        p_.v_switch    = dparam("v_switch", p_.v_switch);
        // --- 비용 가중 ---
        p_.w_lat       = dparam("w_lat", p_.w_lat);
        p_.w_lon       = dparam("w_lon", p_.w_lon);
        p_.w_yaw       = dparam("w_yaw", p_.w_yaw);
        p_.w_v         = dparam("w_v", p_.w_v);
        p_.w_boundary  = dparam("w_boundary", p_.w_boundary);
        p_.margin      = dparam("margin", p_.margin);
        p_.w_terminal  = dparam("w_terminal", p_.w_terminal);
        p_.w_dsteer    = dparam("w_dsteer", p_.w_dsteer);
        p_.w_daccel    = dparam("w_daccel", p_.w_daccel);
        // --- 한계 (v_max = 직선 최고속도 캡, control_map_node의 max_speed에 대응) ---
        p_.steer_max   = dparam("steer_max", p_.steer_max);
        p_.accel_max   = dparam("accel_max", p_.accel_max);
        p_.accel_min   = dparam("accel_min", p_.accel_min);
        p_.v_min       = dparam("v_min", p_.v_min);
        p_.v_max       = dparam("v_max", p_.v_max);
        // --- 출력 평활화 ---
        p_.u_smooth    = dparam("u_smooth", p_.u_smooth);
        // 진단 계측(ESS·비용구성) on/off. 젯슨 실시간 예산이 빡빡하면 false — 제어는 동일.
        p_.diag_enable = this->declare_parameter<bool>("diag_enable", true);

        // 기준속도 곡률 클램프 [m/s²]. 0이면 비활성(플래너 프로파일 그대로).
        // 플래너 프로파일은 **이상적 레이싱라인** 기준으로 생성된다. MPPI가 그 라인에서
        // 0.2m만 벗어나도 유효 선회반경이 줄어 같은 속도에서 마찰한계를 넘고, 그대로
        // 언더스티어로 벽에 간다(07-22 시뮬: 진입 3.4m/s·요레이트 3.05 ⇒ a_lat=10.4 ≈ μg=10.3).
        // 기준속도 자체를 v ≤ √(a_lat_max/κ)로 눌러 두면 수평 안의 앞쪽 스테이지가 미리 낮은
        // 목표속도를 주므로, 사전감속이 자연스럽게 나온다(control_map_node의 곡률 캡과 같은 원리).
        // ⚠️ 모델 마찰한계(μ·g ≈ 10.3)보다 낮게 잡을 것 — 여유가 곧 안정성이다.
        ref_max_lateral_accel_ = dparam("ref_max_lateral_accel", 8.0);

        // --- 비활성(Mux가 MAP 라우팅 중) 시 아이들 워밍업 주기 ---
        // Mux가 RB 상태를 /mppi_active로 알려주기 전까지는 비활성으로 가정(기본 MAP과 동일).
        // 비활성 동안은 idle_solve_decimation_ 사이클(50Hz 기준)마다 한 번만 solve해
        // warm-start(U_)만 유지하고 나머지는 완전히 스킵 — Jetson CPU/GPU 사이클 절약이 목적.
        // RB로 활성화되면 다음 사이클부터 즉시 매 틱 풀 연산으로 복귀(전환 지연 없음).
        idle_solve_decimation_ = iparam("idle_solve_decimation", 5);
        // (조향, 종가속) → 속도명령 변환 지평 [s]. 위 control_loop 주석 참고.
        speed_cmd_horizon_ = dparam("speed_cmd_horizon", static_cast<double>(p_.dt));
        // 텔레메트리 주기(사이클). 기본 50 = 1초. 튜닝 중에는 5(0.1초)로 낮춰 채터링을 본다.
        telemetry_decimation_ = std::max(1, static_cast<int>(iparam("telemetry_decimation", 50)));

        // 2. 구독 / 발행 / 타이머 (control_map_node와 동일 QoS)
        auto qos_gl = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        global_path_sub_ = this->create_subscription<f110_msgs::msg::WpntArray>(
            "/global_waypoints", qos_gl,
            std::bind(&ControlMppiNode::global_path_callback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10,
            std::bind(&ControlMppiNode::odom_callback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            std::bind(&ControlMppiNode::imu_callback, this, std::placeholders::_1));

        // 라우터(시뮬 joy_teleop_monitor / 실차 drive_source_selector)가 RB 상태를
        // latched로 발행 — 이 노드가 나중에 떠도
        // 최신 활성/비활성 상태를 즉시 수신한다.
        mppi_active_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/mppi_active", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
            [this](const std_msgs::msg::Bool::ConstSharedPtr msg) { mppi_active_ = msg->data; });

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive_mppi", 10);

        solver_ = std::make_unique<Solver>(p_);
        last_time_ = this->now();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),  // 50 Hz
            std::bind(&ControlMppiNode::control_loop, this));

        RCLCPP_INFO(this->get_logger(),
                    "control_mppi_node 시작 (%s 솔버, N=%d K=%d, v_max=%.1f)",
#ifdef USE_MPPI_GPU
                    "GPU",
#else
                    "CPU",
#endif
                    p_.N, p_.K, static_cast<double>(p_.v_max));
    }

 private:
    // ------------------------------------------------------------------
    // 콜백
    // ------------------------------------------------------------------
    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        const auto q = msg->pose.pose.orientation;
        const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        current_yaw_ = std::atan2(siny_cosp, cosy_cosp);

        // MPPI는 전체 상태 필요 → odom twist(차체프레임)에서 vx/vy/yaw_rate 직접 추출
        current_vx_       = msg->twist.twist.linear.x;
        current_vy_       = msg->twist.twist.linear.y;
        current_yaw_rate_ = msg->twist.twist.angular.z;
        odom_received_ = true;
    }

    void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        // odom twist가 yaw_rate를 이미 제공하므로 IMU는 보조(향후 융합 여지). 현재는
        // 실측 yaw_rate가 더 신뢰되는 소스면 여기서 덮어쓸 수 있도록 값만 캐시.
        last_imu_yaw_rate_ = msg->angular_velocity.z;
        (void)last_imu_yaw_rate_;  // 현재 미사용(odom twist 우선)
    }

    void global_path_callback(const f110_msgs::msg::WpntArray::ConstSharedPtr msg) {
        if (msg->wpnts.empty()) {
            RCLCPP_WARN(this->get_logger(), "빈 글로벌 웨이포인트 수신.");
            return;
        }
        waypoints_.clear();
        waypoints_.reserve(msg->wpnts.size());
        for (const auto& wp : msg->wpnts) {
            MppiWaypoint w;
            w.x = wp.x_m;
            w.y = wp.y_m;
            w.yaw = wp.psi_rad;
            w.v = wp.vx_mps;
            // 경계비용용 좌/우 벽 거리를 **따로** 넘긴다(2026-07-22). min()으로 뭉치면
            // 최적라인이 한쪽 벽에 붙는 구간에서 넓은 쪽 여유까지 같이 깎여, MPPI가
            // ±(좁은쪽−margin) 튜브에 갇힌다. 최소 0.2m 하한(0/음수 방어).
            w.d_left  = std::max(0.2, static_cast<double>(wp.d_left));
            w.d_right = std::max(0.2, static_cast<double>(wp.d_right));
            w.kappa   = std::abs(static_cast<double>(wp.kappa_radpm));
            waypoints_.push_back(w);
        }
        // 새 경로 수신 시 최근접 인덱스 전역 재초기화
        double best = std::numeric_limits<double>::max();
        for (size_t i = 0; i < waypoints_.size(); ++i) {
            const double d = std::hypot(waypoints_[i].x - current_x_, waypoints_[i].y - current_y_);
            if (d < best) { best = d; last_target_idx_ = i; }
        }
    }

    // ------------------------------------------------------------------
    // 50Hz 제어 루프
    // ------------------------------------------------------------------
    void control_loop() {
        const rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        if (dt <= 0.0 || dt > 0.5) dt = 0.02;  // 클럭 점프/초기화 방어
        last_time_ = now;

        // MPPI 비활성(Mux가 MAP 라우팅 중)이면 idle_solve_decimation_ 사이클마다 한 번만
        // 이번 함수를 계속 진행(경로 탐색+solve)하고, 나머지 사이클은 완전히 스킵한다.
        // 활성화되는 즉시(다음 사이클부터) 매 틱 정상 주기로 복귀.
        if (mppi_active_) {
            idle_skip_counter_ = 0;
        } else if ((++idle_skip_counter_ % idle_solve_decimation_) != 0) {
            return;
        }

        // 경로 미수신 → 안전 정지 명령(coasting 방지). odom 없어도 정지 발행.
        if (waypoints_.empty() || !odom_received_) {
            publish_stop();
            return;
        }

        // 현재 상태 조립
        SState cur;
        cur.x = current_x_;   cur.y = current_y_;   cur.yaw = current_yaw_;
        cur.vx = current_vx_; cur.vy = current_vy_; cur.yaw_rate = current_yaw_rate_;

        // 기준궤적 N+1개 빌드 후 solve
        std::vector<SRef> ref;
        build_reference(ref);

        SControl u;
        const bool ok = solver_->solve(cur, ref, u);
        if (!ok) {
            // 갱신 실패(경로 부족/전 샘플 발산) → 직전 명령 홀드, 없으면 정지
            if (!have_last_cmd_) { publish_stop(); return; }
            u = last_cmd_;
        }
        last_cmd_ = u;
        have_last_cmd_ = true;

        // MPPI는 (조향, 종가속)을 출력하는데 하위 인터페이스(gym/VESC)는 **속도** 명령을 받는다.
        // 이전에는 `vx + a·dt`(dt=0.05)로 변환했는데, 이러면 a=9를 계획해도 명령이 실측속도
        // +0.45 m/s에 불과하다. 하위 속도루프가 P제어(gym: accel = kp·(목표−현재), kp≈4.75)라
        // 실제 전달되는 가속은 요청의 1/4로 깎이고, 차가 가속을 못 해 코너 전에 무너진다.
        // → 변환 지평 speed_cmd_horizon을 파라미터로 뺀다. 하위 루프 게인의 역수(sim ≈ 1/4.75
        //   ≈ 0.21s)로 두면 계획한 가속이 그대로 전달된다. 실차 VESC는 속도루프가 빨라 값이
        //   다를 수 있으므로 환경별 튜닝 대상(기본값은 기존 동작 보존용 dt).
        const double accel = static_cast<double>(u.accel);
        const double steer = static_cast<double>(u.steer);
        double speed_cmd = current_vx_ + accel * speed_cmd_horizon_;
        speed_cmd = std::max(static_cast<double>(p_.v_min),
                             std::min(speed_cmd, static_cast<double>(p_.v_max)));

        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.header.stamp = now;
        drive_msg.header.frame_id = "base_link";
        drive_msg.drive.steering_angle = steer;
        drive_msg.drive.speed = speed_cmd;
        drive_msg.drive.acceleration = accel;
        drive_pub_->publish(drive_msg);

        // 텔레메트리 — solve 시간/상태. CPU 솔버 빌드에서는 튜닝 진단값(ESS·비용 구성)까지
        // 함께 찍는다(GPU 솔버에는 아직 진단 계측이 없어 조건부 컴파일).
        if ((++loop_count_ % telemetry_decimation_) == 0) {
            const auto& d = solver_->last_diag();
            RCLCPP_INFO(this->get_logger(),
                        "MPPI %.2fms | steer=%.3f v=%.2f(vx=%.2f) | ESS=%.1f/%d fin=%d lam=%.1f "
                        "C[lat=%.1f lon=%.1f yaw=%.1f v=%.1f bnd=%.1f du=%.1f] maxlat=%.2f",
                        solver_->last_solve_ms(), steer, speed_cmd, current_vx_,
                        d.ess, p_.K, d.n_finite, d.lambda_eff,
                        d.c_lat, d.c_lon, d.c_yaw, d.c_v, d.c_bnd, d.c_du, d.nominal_max_lat);
        }
    }

    // 현재 위치 앞으로 N+1개의 기준점을 호 길이 기준으로 샘플링해 채운다.
    // ref[0]은 최근접점(솔버 비용은 ref[1..N]만 사용). 스텝 간격 ds = v_target·dt로
    // 시간 정합된 수평을 만들되, 정지 시에도 앞을 보도록 속도 하한(1.0m/s)을 둔다.
    void build_reference(std::vector<SRef>& ref) {
        const size_t n = waypoints_.size();
        ref.resize(static_cast<size_t>(p_.N) + 1);

        // 최근접 웨이포인트: 직전 인덱스 주변 윈도우 스캔 + 2.5m 초과 이탈 시 전역 재탐색
        size_t closest = last_target_idx_;
        double min_dist = std::numeric_limits<double>::max();
        for (int i = -2; i <= 8; ++i) {
            const size_t idx = (last_target_idx_ + i + n) % n;
            const double d = std::hypot(waypoints_[idx].x - current_x_, waypoints_[idx].y - current_y_);
            if (d < min_dist) { min_dist = d; closest = idx; }
        }
        if (min_dist > 2.5) {
            min_dist = std::numeric_limits<double>::max();
            for (size_t i = 0; i < n; ++i) {
                const double d = std::hypot(waypoints_[i].x - current_x_, waypoints_[i].y - current_y_);
                if (d < min_dist) { min_dist = d; closest = i; }
            }
        }
        last_target_idx_ = closest;

        const double dt = static_cast<double>(p_.dt);
        const double v_floor = 1.0;  // 정지 시 수평 붕괴 방지용 속도 하한

        // 수평 간격 = **현재 속도에서 시작해 도달가능 가속으로 프로파일 속도까지 램프**
        // (2026-07-22 수정). 두 극단이 모두 실패하기 때문에 나온 중간값이다:
        //   ① 프로파일 속도로 고정(구버전) → 차가 못 따라가면 기준점이 달아나고, 추종 비용이
        //      "저 앞 점으로 최단거리로 가라" = 코너 가로지르기가 된다(헤어핀 탈출 벽충돌).
        //   ② 현재 속도로 고정 → **가속 자체에 벌점**이 걸린다. 곡선에서 기준점보다 Δs 앞서면
        //      그 lag가 횡오차로 둔갑하기 때문(원호에서 약 Δs²/2R). 실측으로 속도가 0.5m/s까지
        //      붕괴했다.
        // 램프는 "이 계획대로 가속하면 실제로 지나갈 자리"를 기준으로 삼아 둘 다 피한다.
        const double a_ref = std::max(0.5, static_cast<double>(p_.accel_max));

        auto fill = [&](size_t stage, size_t widx) {
            SRef r;
            r.x = waypoints_[widx].x;
            r.y = waypoints_[widx].y;
            r.yaw = waypoints_[widx].yaw;
            // 곡률 기반 기준속도 클램프 (위 ref_max_lateral_accel_ 주석 참고)
            r.v = waypoints_[widx].v;
            if (ref_max_lateral_accel_ > 1e-3) {
                const double k = waypoints_[widx].kappa;
                if (k > 1e-4) {
                    r.v = std::min(static_cast<double>(r.v),
                                   std::sqrt(ref_max_lateral_accel_ / k));
                }
            }
            r.d_left  = waypoints_[widx].d_left;
            r.d_right = waypoints_[widx].d_right;
            ref[stage] = r;
        };
        fill(0, closest);

        size_t cursor = closest;
        double accum = 0.0;     // 커서까지 누적 호 길이
        double target_s = 0.0;  // 다음 스테이지 목표 호 길이
        double v_h = std::max(v_floor, current_vx_);   // 수평 속도(스테이지마다 램프)
        for (int t = 1; t <= p_.N; ++t) {
            // 프로파일 속도를 상한으로, 도달가능 가속만큼만 올린다
            double v_prof = static_cast<double>(waypoints_[cursor].v);
            if (ref_max_lateral_accel_ > 1e-3 && waypoints_[cursor].kappa > 1e-4) {
                v_prof = std::min(v_prof, std::sqrt(ref_max_lateral_accel_ / waypoints_[cursor].kappa));
            }
            v_prof = std::max(v_floor, v_prof);
            v_h = std::min(v_prof, v_h + a_ref * dt);
            target_s += v_h * dt;
            // 커서를 target_s 도달할 때까지 전진(폐루프 wrap, 한바퀴 방지)
            size_t steps = 0;
            while (accum < target_s && steps < n) {
                const size_t nxt = (cursor + 1) % n;
                accum += std::hypot(waypoints_[nxt].x - waypoints_[cursor].x,
                                    waypoints_[nxt].y - waypoints_[cursor].y);
                cursor = nxt;
                ++steps;
            }
            fill(static_cast<size_t>(t), cursor);
        }
    }

    void publish_stop() {
        auto msg = ackermann_msgs::msg::AckermannDriveStamped();
        msg.header.stamp = this->now();
        msg.header.frame_id = "base_link";
        msg.drive.steering_angle = 0.0;
        msg.drive.speed = 0.0;
        msg.drive.acceleration = 0.0;
        drive_pub_->publish(msg);
    }

    // ------------------------------------------------------------------
    // 내부 자료구조 / 멤버
    // ------------------------------------------------------------------
    struct MppiWaypoint { double x, y, yaw, v, d_left, d_right, kappa; };

    SParams p_;
    std::unique_ptr<Solver> solver_;

    // 구독/발행/타이머
    rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr global_path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mppi_active_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string odom_topic_;
    bool mppi_active_ = false;           // Mux(RB)가 알려주는 현재 활성 알고리즘 여부
    int idle_solve_decimation_ = 5;      // 비활성 시 이 사이클마다 한 번만 solve
    int telemetry_decimation_ = 50;      // 텔레메트리 출력 주기(사이클)
    double speed_cmd_horizon_ = 0.05;    // 가속→속도명령 변환 지평 [s]
    double ref_max_lateral_accel_ = 8.0; // 기준속도 곡률 클램프 a_lat [m/s²] (0=비활성)
    unsigned long idle_skip_counter_ = 0;

    // 상태
    std::vector<MppiWaypoint> waypoints_;
    size_t last_target_idx_ = 0;
    double current_x_ = 0.0, current_y_ = 0.0, current_yaw_ = 0.0;
    double current_vx_ = 0.0, current_vy_ = 0.0, current_yaw_rate_ = 0.0;
    double last_imu_yaw_rate_ = 0.0;
    bool odom_received_ = false;
    rclcpp::Time last_time_;

    SControl last_cmd_{};
    bool have_last_cmd_ = false;
    unsigned long loop_count_ = 0;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlMppiNode>());
    rclcpp::shutdown();
    return 0;
}
