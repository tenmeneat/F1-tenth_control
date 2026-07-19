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
        p_.K           = iparam("K", p_.K);
        p_.dt          = dparam("dt", p_.dt);
        p_.lambda      = dparam("lambda", p_.lambda);
        p_.sigma_steer = dparam("sigma_steer", p_.sigma_steer);
        p_.sigma_accel = dparam("sigma_accel", p_.sigma_accel);
        p_.substeps    = iparam("substeps", p_.substeps);
        // --- 차량/타이어 (Pacejka는 gym 기본값 유지 — 실차 보정 전까지 노출 최소화) ---
        p_.wheelbase   = dparam("wheelbase", p_.wheelbase);
        p_.v_switch    = dparam("v_switch", p_.v_switch);
        // --- 비용 가중 ---
        p_.w_pos       = dparam("w_pos", p_.w_pos);
        p_.w_yaw       = dparam("w_yaw", p_.w_yaw);
        p_.w_v         = dparam("w_v", p_.w_v);
        p_.w_boundary  = dparam("w_boundary", p_.w_boundary);
        p_.margin      = dparam("margin", p_.margin);
        p_.w_terminal  = dparam("w_terminal", p_.w_terminal);
        // --- 한계 (v_max = 직선 최고속도 캡, control_map_node의 max_speed에 대응) ---
        p_.steer_max   = dparam("steer_max", p_.steer_max);
        p_.accel_max   = dparam("accel_max", p_.accel_max);
        p_.accel_min   = dparam("accel_min", p_.accel_min);
        p_.v_min       = dparam("v_min", p_.v_min);
        p_.v_max       = dparam("v_max", p_.v_max);
        // --- 출력 평활화 ---
        p_.u_smooth    = dparam("u_smooth", p_.u_smooth);

        // --- 비활성(Mux가 MAP 라우팅 중) 시 아이들 워밍업 주기 ---
        // Mux가 RB 상태를 /mppi_active로 알려주기 전까지는 비활성으로 가정(기본 MAP과 동일).
        // 비활성 동안은 idle_solve_decimation_ 사이클(50Hz 기준)마다 한 번만 solve해
        // warm-start(U_)만 유지하고 나머지는 완전히 스킵 — Jetson CPU/GPU 사이클 절약이 목적.
        // RB로 활성화되면 다음 사이클부터 즉시 매 틱 풀 연산으로 복귀(전환 지연 없음).
        idle_solve_decimation_ = iparam("idle_solve_decimation", 5);

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
            // 경계비용용 트랙 반폭 ≈ min(좌,우 벽까지 거리). 최소 0.2m 하한(0/음수 방어).
            w.half_width = std::max(0.2, std::min(wp.d_left, wp.d_right));
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

        // MPPI는 (조향, 종가속) 출력 → VESC/시뮬용 speed는 다음스텝 속도로 적분
        const double accel = static_cast<double>(u.accel);
        const double steer = static_cast<double>(u.steer);
        double speed_cmd = current_vx_ + accel * static_cast<double>(p_.dt);
        speed_cmd = std::max(static_cast<double>(p_.v_min),
                             std::min(speed_cmd, static_cast<double>(p_.v_max)));

        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.header.stamp = now;
        drive_msg.header.frame_id = "base_link";
        drive_msg.drive.steering_angle = steer;
        drive_msg.drive.speed = speed_cmd;
        drive_msg.drive.acceleration = accel;
        drive_pub_->publish(drive_msg);

        // 텔레메트리(1초에 한 번 정도) — solve 시간/상태
        if ((++loop_count_ % 50) == 0) {
            RCLCPP_INFO(this->get_logger(),
                        "MPPI solve=%.2fms | steer=%.3f speed=%.2f (vx=%.2f)",
                        solver_->last_solve_ms(), steer, speed_cmd, current_vx_);
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

        auto fill = [&](size_t stage, size_t widx) {
            SRef r;
            r.x = waypoints_[widx].x;
            r.y = waypoints_[widx].y;
            r.yaw = waypoints_[widx].yaw;
            r.v = waypoints_[widx].v;
            r.half_width = waypoints_[widx].half_width;
            ref[stage] = r;
        };
        fill(0, closest);

        size_t cursor = closest;
        double accum = 0.0;     // 커서까지 누적 호 길이
        double target_s = 0.0;  // 다음 스테이지 목표 호 길이
        for (int t = 1; t <= p_.N; ++t) {
            const double v_target = std::max(v_floor, static_cast<double>(waypoints_[cursor].v));
            target_s += v_target * dt;
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
    struct MppiWaypoint { double x, y, yaw, v, half_width; };

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
