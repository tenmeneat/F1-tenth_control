#include <iostream>
#include <iomanip>
#include <sstream>
#include <memory>
#include <cstdlib>
#include <cmath>
#include <deque>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

// ============================================================================
// odom_calib_node — 오도메트리 거리 스케일 실측 보정 (우리 컴에서 실행, 표시 전용)
// ============================================================================
// "1m 명령 주고 자로 재기" 테스트를 우리 차에 맞게 자동화한 것. 젯슨은 원시 토픽만
// 내보내고 이 노드가 우리 컴 터미널에서 조립·렌더링한다(realcar_dashboard_node와 동일 구조,
// 젯슨 연산 0). `/drive`를 발행하지 않는 순수 관찰자라 주행 중 켜둬도 제어에 영향 없음.
//
// ── 왜 세 개를 동시에 재는가 ────────────────────────────────────────────────
// 명령→모터와 모터→odom은 서로 다른 게인을 쓰는 별개 경로라, "명령 1m vs 자 0.9m"
// 하나만 보면 어느 쪽이 틀렸는지 구분이 안 된다:
//   컨트롤러 speed[m/s] ──(speed_to_erpm_gain 4614)──> ERPM ──> 모터   [명령 방향]
//   모터 실측 ERPM ──(erpm_to_speed, 젯슨 vesc_to_odom 설정)──> odom vx [보고 방향]
// 그래서 한 번의 주행에서 독립적인 거리 3개를 동시에 적분한다:
//   1) 명령 = ∫ /drive.speed dt            — 우리가 요구한 거리
//   2) 휠   = ∫ |odom twist.linear.x| dt    — VESC 휠 오도메트리 (erpm_to_speed 경로)
//   3) 맵   = |끝 위치 - 시작 위치|          — MCL 스캔매칭 기반 직선변위
// `/pf/pose/odom` 한 토픽 안에 twist(VESC 패스스루)와 pose(MCL 추정)라는 출처가 다른
// 두 신호가 들어있다는 점이 핵심 — 2)와 3)이 어긋나면 **자를 대기 전에도** 스케일
// 오차가 드러난다. 자는 3)까지 검증(맵 스케일 자체가 틀렸을 경우)하는 데 쓴다.
//
// ⚠️ 3)에 **직선변위(map_disp)를 쓰고 경로길이(map_path)는 쓰지 않는다.** 이유:
//   `/pf/pose/odom`의 pose는 "스캔매칭 앵커 + 마지막 앵커 이후의 휠 변위"다
//   (particle_filter.cpp:1099-1101에서 LiDAR 프레임마다 앵커를 스캔매칭 값으로 리셋,
//    :1412에서 그 사이를 휠로 보간). 총 이동거리는 앵커들이 결정하므로 휠 스케일 오차가
//   누적되지 않아 캘리브레이션에 유효하지만, **경로길이 Σ|Δpos|는 절댓값 합이라 앵커에서
//   뒤로 당겨지는 보정 점프까지 길이로 더해져 항상 과대평가된다.** 직선변위는 이 지터에
//   면역이다. 경로길이는 아래 "직진도" 판정(곡선이 섞였는지)에만 쓴다.
//
// 판정:
//   휠 ≠ 맵          → 오도메트리 스케일 오차 (젯슨 vesc_to_odom의 erpm_to_speed/휠반지름)
//   명령 ≠ 휠        → 명령 스케일 또는 속도 추종 부족 (speed_to_erpm_gain)
//   맵 ≠ 자          → 맵 스케일(resolution) 또는 MCL 문제
//
// ── 사용법 ──────────────────────────────────────────────────────────────────
//   우리 컴:  ros2 launch f1tenth_control dashboard.launch.py mode:=calib
//   (mode:=real 대시보드와 동일하게 무선이면 ROS_DISCOVERY_SERVER 필요)
//   주행은 수동(조이스틱)으로 **직선만**. 노드가 출발/정지를 자동 감지해 구간을 끊고,
//   멈추면 그 주행의 결과를 요약해 이력에 쌓는다. 자로 잰 값은 손으로 비교.
//   `/odom_calib/reset`(std_msgs/Empty) 발행 시 이력 초기화.
//
// ⚠️ 주의사항(측정 품질 직결):
//   - **직선 주행만.** 조향이 들어가면 경로 길이가 달라져 세 값의 비교가 무의미해진다.
//     화면의 "직진도"(직선변위/경로길이)가 0.99 미만이면 그 주행은 버릴 것.
//   - **1m는 너무 짧다.** 자 대는 위치오차 ±2cm가 그대로 2% 불확실성이 된다. 5~10m로
//     늘리면 같은 오차가 0.2~0.4%로 줄어든다(스케일 오차는 거리에 비례).
//   - **저속 금지.** VESC FOC 센서리스 데드존이 800~2250 ERPM(≈0.17~0.49 m/s)이라
//     기어가듯 천천히 하면 그 구간에 머물러 크래시난다. 1.0~2.0 m/s 정속으로 할 것.
//   - **양방향으로.** 정/역 결과가 비대칭이면 게인이 아니라 오프셋(speed_to_erpm_offset) 문제.
//   - **여러 속도에서.** 비율이 속도에 따라 변하면 순수 스케일 오차가 아니라 슬립/오프셋 항이라
//     게인 하나로 못 고친다.
//   - dt는 **헤더 스탬프가 아니라 우리 컴 수신시각**으로 적분한다. 원격 뷰라 젯슨 시계가
//     우리와 동기화돼 있다는 보장이 없기 때문(무선 지연은 수 m 주행에서 평균으로 상쇄됨).
// ============================================================================

class OdomCalib : public rclcpp::Node {
public:
    OdomCalib() : Node("odom_calib_node") {
        this->declare_parameter<std::string>("odom_topic", "/pf/pose/odom");
        // 주행 구간 자동 분할 임계치. 출발은 확실히(노이즈 무시), 정지는 민감하게 잡는다.
        this->declare_parameter<double>("start_speed", 0.30);   // 이 속도 넘으면 주행 시작 [m/s]
        this->declare_parameter<double>("stop_speed", 0.05);    // 이 속도 밑이면 정지 후보 [m/s]
        this->declare_parameter<double>("stop_hold", 0.70);     // 정지 후보가 이만큼 지속되면 종료 [s]
        this->declare_parameter<double>("min_distance", 0.50);  // 이보다 짧은 구간은 이력에 안 넣음 [m]

        odom_topic_ = this->get_parameter("odom_topic").as_string();
        start_speed_ = this->get_parameter("start_speed").as_double();
        stop_speed_ = this->get_parameter("stop_speed").as_double();
        stop_hold_ = this->get_parameter("stop_hold").as_double();
        min_distance_ = this->get_parameter("min_distance").as_double();

        // rclcpp::Time은 clock type이 다르면 비교 시 예외를 던지므로 전부 노드 시계로 초기화.
        never_ = this->now();
        last_odom_t_ = last_drive_t_ = run_start_t_ = below_since_ = never_;

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 50,
            std::bind(&OdomCalib::odom_callback, this, std::placeholders::_1));

        drive_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive", 50,
            std::bind(&OdomCalib::drive_callback, this, std::placeholders::_1));

        reset_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "/odom_calib/reset", 10,
            [this](const std_msgs::msg::Empty::ConstSharedPtr) {
                history_.clear();
                RCLCPP_INFO(this->get_logger(), "이력 초기화됨");
            });

        render_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&OdomCalib::render, this));

        RCLCPP_INFO(this->get_logger(),
            "odom_calib_node 시작 — odom=%s. 직선으로 5~10m 주행하면 자동으로 구간을 끊습니다.",
            odom_topic_.c_str());
    }

private:
    // 한 번의 주행 구간 결과
    struct Run {
        double cmd = 0.0;        // ∫ /drive.speed dt        [m]
        double wheel = 0.0;      // ∫ |odom vx| dt           [m]
        double map_path = 0.0;   // Σ |Δposition| (경로길이)  [m]
        double map_disp = 0.0;   // |끝 - 시작| (직선변위)    [m]
        double duration = 0.0;   // 소요 시간                 [s]
        double peak_speed = 0.0; // 최고 속도                 [m/s]
    };

    void drive_callback(const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr m) {
        rclcpp::Time now_t = this->now();
        // 명령 거리는 주행 중일 때만 적분. dt는 수신시각 기준(원격 시계 비동기 회피).
        // ⚠️ 적분 규칙을 odom 쪽(휠/맵)과 반드시 일치시킬 것 — 둘 다 "현재 샘플값 × 직전과의 dt"
        //    (우단점 규칙)이고, 주행 시작 직후 첫 샘플은 양쪽 다 건너뛴다. 규칙이 어긋나면
        //    구간 경계에서 한 샘플씩 밀려 명령/휠 비율에 0.5%가량 계통오차가 생긴다
        //    (실측 1.0304 vs 참값 1.025 — 찾으려는 오차와 같은 크기라 무시 못 함).
        if (running_) {
            if (!cmd_primed_) {
                cmd_primed_ = true;          // 시작 샘플: 기준시각만 잡고 적분 안 함
            } else {
                double dt = (now_t - last_drive_t_).seconds();
                if (dt > 1e-4 && dt < 0.5) {
                    cur_.cmd += std::fabs(m->drive.speed) * dt;
                }
            }
        }
        got_drive_ = true;
        last_drive_t_ = now_t;
    }

    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr m) {
        rclcpp::Time now_t = this->now();
        double vx = m->twist.twist.linear.x;
        double px = m->pose.pose.position.x;
        double py = m->pose.pose.position.y;
        speed_ = vx;

        if (got_odom_) {
            double dt = (now_t - last_odom_t_).seconds();
            if (dt > 1e-4 && dt < 0.5) {
                // ── 주행 시작 판정 ──
                // ⚠️ 시작 샘플은 적분에 넣지 않고 기준점만 잡고 빠져나간다. 이 샘플의 dt는
                //    직전 '정지 상태' 샘플까지 걸쳐 있어서, 그대로 적분하면 정지해 있던
                //    한 주기를 주행 속도로 계산해 휠 거리가 한 샘플만큼 과대평가된다
                //    (50Hz·10m 주행에서 약 0.4% — 찾으려는 스케일 오차와 같은 크기라 무시 못 함).
                if (!running_ && std::fabs(vx) > start_speed_) {
                    cur_ = Run{};
                    start_x_ = px; start_y_ = py;
                    run_start_t_ = now_t;
                    below_since_ = never_;
                    running_ = true;
                    cmd_primed_ = false;     // 명령 적분도 같은 규칙으로 첫 샘플을 건너뛴다
                    prev_px_ = px; prev_py_ = py;
                    got_odom_ = true; last_odom_t_ = now_t;
                    return;
                }

                if (running_) {
                    cur_.wheel += std::fabs(vx) * dt;

                    // 맵 경로길이. MCL 재수렴/텔레포트로 좌표가 튀면 그 구간은 버린다
                    // (0.5m 이상 점프는 한 스텝의 실제 이동일 수 없음 — 50Hz에서 25m/s).
                    double step = std::hypot(px - prev_px_, py - prev_py_);
                    if (step < 0.5) cur_.map_path += step;

                    cur_.map_disp = std::hypot(px - start_x_, py - start_y_);
                    cur_.duration = (now_t - run_start_t_).seconds();
                    cur_.peak_speed = std::max(cur_.peak_speed, std::fabs(vx));

                    // ── 주행 종료 판정: 저속이 stop_hold_ 이상 지속 ──
                    if (std::fabs(vx) < stop_speed_) {
                        if (below_since_ == never_) below_since_ = now_t;
                        else if ((now_t - below_since_).seconds() > stop_hold_) {
                            running_ = false;
                            below_since_ = never_;
                            if (cur_.wheel >= min_distance_) {
                                history_.push_back(cur_);
                                if (history_.size() > kMaxHistory) history_.pop_front();
                                RCLCPP_INFO(this->get_logger(),
                                    "구간 종료 — 명령 %.3f / 휠 %.3f / 맵(직선변위) %.3f m "
                                    "(경로길이 %.3f)",
                                    cur_.cmd, cur_.wheel, cur_.map_disp, cur_.map_path);
                            }
                        }
                    } else {
                        below_since_ = never_;
                    }
                }
            }
        }

        prev_px_ = px; prev_py_ = py;
        got_odom_ = true;
        last_odom_t_ = now_t;
    }

    std::string age(bool ever, const rclcpp::Time& t) {
        if (!ever) return "\033[1;31m --  \033[0m";
        double a = (this->now() - t).seconds();
        const char* col = (a < 0.5) ? "\033[1;32m" : (a < 1.5 ? "\033[1;33m" : "\033[1;31m");
        std::ostringstream o;
        o << col << std::fixed << std::setprecision(1) << a << "s\033[0m";
        return o.str();
    }

    // 비율 표시. 1.000에서 벗어난 정도로 색을 준다(±1% 이내 초록, ±3% 노랑, 그 이상 빨강).
    std::string ratio(double num, double den) {
        std::ostringstream o;
        if (den < 1e-6) return "   --  ";
        double r = num / den;
        double err = std::fabs(r - 1.0);
        const char* col = (err < 0.01) ? "\033[1;32m" : (err < 0.03 ? "\033[1;33m" : "\033[1;31m");
        o << col << std::fixed << std::setprecision(4) << r << "\033[0m";
        return o.str();
    }

    void render() {
        std::ostringstream oss;
        oss << "=========================================================\n";
        oss << "   ODOM DISTANCE CALIBRATION   (remote @ laptop)          \n";
        oss << "=========================================================\n";
        const char* dom = std::getenv("ROS_DOMAIN_ID");
        oss << " [Link] ROS_DOMAIN_ID=" << (dom ? dom : "0")
            << "   odom " << age(got_odom_, last_odom_t_)
            << "   /drive " << age(got_drive_, last_drive_t_) << "\n";
        oss << " 직선으로 5~10m, 1.0~2.0 m/s 정속. 출발/정지는 자동 감지.\n\n";

        oss << std::fixed;
        oss << " [Status] "
            << (running_ ? "\033[1;36m[ 측정 중 ]\033[0m" : "\033[1;32m[ 대기 ]\033[0m")
            << "   현재 속도 " << std::setprecision(2) << speed_ << " m/s\n\n";

        // ── 현재/직전 구간 ──
        const Run* r = running_ ? &cur_ : (history_.empty() ? nullptr : &history_.back());
        oss << " [" << (running_ ? "현재 구간" : "직전 구간") << "]\n";
        if (!r) {
            oss << "   \033[1;33m(아직 측정된 구간 없음 — 주행하세요)\033[0m\n\n";
        } else {
            oss << std::setprecision(3);
            oss << "   1) 명령 (∫/drive.speed)   : " << r->cmd      << " m\n";
            oss << "   2) 휠   (∫odom vx, VESC)  : " << r->wheel    << " m\n";
            oss << "   3) 맵   (직선변위, MCL)   : " << r->map_disp << " m"
                << "   \033[1;36m← 자와 비교할 값\033[0m\n";
            oss << "      (참고) 경로길이        : " << r->map_path << " m"
                << "   (소요 " << std::setprecision(1) << r->duration
                << "s, 최고 " << std::setprecision(2) << r->peak_speed << " m/s)\n";

            // 직진도 — 조향이 섞였는지 검사. 낮으면 그 구간은 버려야 한다.
            double straight = (r->map_path > 1e-6) ? r->map_disp / r->map_path : 0.0;
            const char* scol = (straight > 0.99) ? "\033[1;32m"
                             : (straight > 0.97 ? "\033[1;33m" : "\033[1;31m");
            oss << "      직진도(변위/경로)      : " << scol
                << std::setprecision(3) << straight << "\033[0m"
                << (straight > 0.99 ? "  (직선 OK)" : "  \033[1;31m← 곡선 섞임, 이 구간 버릴 것\033[0m")
                << "\n\n";

            oss << "   [비율]  휠/맵 = " << ratio(r->wheel, r->map_disp)
                << "   ← 어긋나면 오도메트리 스케일(젯슨 erpm_to_speed)\n";
            oss << "           명령/휠 = " << ratio(r->cmd, r->wheel)
                << "   ← 어긋나면 명령 스케일(speed_to_erpm_gain) 또는 추종 부족\n\n";
        }

        // ── 이력 + 평균 ──
        oss << " [이력]  (최근 " << history_.size() << "건, /odom_calib/reset 로 초기화)\n";
        if (history_.empty()) {
            oss << "   -\n";
        } else {
            oss << "    #   명령[m]   휠[m]   맵[m]   휠/맵    명령/휠\n";
            int i = 1;
            double sum_wm = 0.0, sum_cw = 0.0;
            int n_wm = 0, n_cw = 0;
            for (const auto& h : history_) {
                oss << "   " << std::setw(2) << i++ << "  "
                    << std::setprecision(3) << std::setw(7) << h.cmd << "  "
                    << std::setw(7) << h.wheel << "  "
                    << std::setw(7) << h.map_disp << "   "
                    << ratio(h.wheel, h.map_disp) << "   "
                    << ratio(h.cmd, h.wheel) << "\n";
                if (h.map_disp > 1e-6) { sum_wm += h.wheel / h.map_disp; n_wm++; }
                if (h.wheel > 1e-6)    { sum_cw += h.cmd / h.wheel;      n_cw++; }
            }
            oss << "   ── 평균:  휠/맵 = "
                << (n_wm ? ratio(sum_wm / n_wm, 1.0) : "  --  ")
                << "    명령/휠 = "
                << (n_cw ? ratio(sum_cw / n_cw, 1.0) : "  --  ") << "\n";
            oss << "\n   자로 잰 실제 거리와 위 3)맵 값을 비교하세요.\n";
            oss << "   맵 ≠ 자 이면 맵 스케일(resolution) 또는 MCL 문제입니다.\n";
        }
        oss << "=========================================================\n";

        std::cout << "\033[2J\033[H" << oss.str() << std::flush;
    }

    // 파라미터
    std::string odom_topic_;
    double start_speed_ = 0.30, stop_speed_ = 0.05, stop_hold_ = 0.70, min_distance_ = 0.50;

    // 상태
    bool running_ = false;
    bool cmd_primed_ = false;   // 주행 시작 후 명령 적분의 첫 샘플을 건너뛰었는지
    Run cur_;
    std::deque<Run> history_;
    static constexpr size_t kMaxHistory = 10;

    double speed_ = 0.0;
    double prev_px_ = 0.0, prev_py_ = 0.0, start_x_ = 0.0, start_y_ = 0.0;

    bool got_odom_ = false, got_drive_ = false;
    rclcpp::Time never_, last_odom_t_, last_drive_t_, run_start_t_, below_since_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub_;
    rclcpp::TimerBase::SharedPtr render_timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomCalib>());
    rclcpp::shutdown();
    return 0;
}
