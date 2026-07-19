#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <sstream>
#include <sys/stat.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "f1tenth_control/steering_lookup_table.hpp"

// lut_calibrator_node — 실차 주행 기반 Steering LUT 실측 보정 (관찰 전용)
// 이 노드는 /drive를 절대 발행하지 않는다(순수 구독·기록 노드) — 즉 제어 루프·
// 안전 로직에 어떤 영향도 주지 않으며, control_real.launch.py와 함께 켜둬도 위험 없음.
//
// 방법: 실제 주행 중 (조향각, 속도, 실측 횡가속도)를 계속 관측해 LUT와 같은
// (steer_axis x velocity_axis) 그리드에 비닝(binning)하여 셀별 평균을 누적한다.
//   - 조향각: /drive(최종 송출 명령)의 steering_angle. 서보 인코더가 없어 "실제
//     서보가 도달한 각도"의 최선의 근사치로 사용(명령=실제 각도라는 가정).
//   - 속도: odom_topic의 twist.linear.x (실측 차량 속도).
//   - 실제 횡가속도: v * yaw_rate (IMU angular_velocity.z). 가속도계 대신 이걸
//     쓰는 이유는 트랙이 평평하면 롤 성분에 오염되지 않는 표준적인 방법이기 때문.
//
// 여러 번 주행에 걸친 평균: 누적치(합/카운트)를 state_file에 저장해 놓고, 다음
// 실행 시 자동으로 이어서 누적한다(런치를 여러 번 다시 켜도 계속 평균이 쌓임).
//
// 산출물: output_lut_file에 "블렌딩된" LUT CSV를 주기적으로 기록한다.
//   blended = (base_value * prior_weight + sum_samples) / (prior_weight + count)
// 즉 샘플이 적은 셀은 원본 LUT 값에 가깝게 유지되고(prior_weight로 과신 방지),
// 샘플이 쌓일수록 실측 평균으로 수렴한다. 샘플이 0인 셀은 원본 값 그대로 둔다.
//
// 주의: 이 노드가 새 CSV를 써도 control_map_node가 자동으로 다시 읽지는
// 않는다(LUT는 생성자에서 1회 로드). 캘리브레이션 결과를 실제로 쓰려면 다음 실행 때
// control_map_node의 lookup_table_file 파라미터를 이 output 경로로 지정할 것.

class LutCalibratorNode : public rclcpp::Node {
public:
    LutCalibratorNode() : Node("lut_calibrator_node") {
        this->declare_parameter<std::string>("odom_topic", "/pf/pose/odom");
        this->declare_parameter<std::string>("imu_topic", "/imu/data");
        this->declare_parameter<std::string>("drive_topic", "/drive");
        this->declare_parameter<std::string>("base_lut_file", "");
        this->declare_parameter<std::string>("output_dir", "");
        this->declare_parameter<double>("min_speed_for_sample", 1.0);
        this->declare_parameter<double>("prior_weight", 3.0);
        this->declare_parameter<double>("yaw_rate_filter_alpha", 0.3);
        // IMU 각속도 단위 보정(VESC가 deg/s로 발행 — 2026-07-19 확인). 보정 안 하면
        // a_lat = v*yaw_rate가 57.3배가 되어 **LUT 보정 데이터가 조용히 전부 오염된다**
        // — /drive를 발행하지 않는 관찰 노드라 주행 중엔 아무 증상이 없다.
        // 실제 값은 런치가 넘긴다(_control_common.py IMU_ANGULAR_SCALE, control_map_node와 공유).
        this->declare_parameter<double>("imu_angular_scale", 1.0);
        this->declare_parameter<double>("save_interval_sec", 15.0);

        this->get_parameter("odom_topic", odom_topic_);
        this->get_parameter("imu_topic", imu_topic_);
        this->get_parameter("drive_topic", drive_topic_);
        this->get_parameter("min_speed_for_sample", min_speed_for_sample_);
        this->get_parameter("prior_weight", prior_weight_);
        this->get_parameter("yaw_rate_filter_alpha", yaw_rate_alpha_);
        this->get_parameter("imu_angular_scale", imu_angular_scale_);
        this->get_parameter("save_interval_sec", save_interval_sec_);

        std::string output_dir;
        this->get_parameter("output_dir", output_dir);
        if (output_dir.empty()) {
            const char* home = std::getenv("HOME");
            output_dir = std::string(home ? home : ".") + "/f1tenth_lut_calibration";
        }
        mkdir_p(output_dir);
        state_file_ = output_dir + "/calibration_state.csv";
        output_lut_file_ = output_dir + "/NUC6_glc_pacejka_lookup_table_calibrated.csv";

        // ===== 베이스 LUT 로드 (grid 축/기준값 확보) — control_map_node와 동일한 폴백 순서 =====
        std::string base_lut_file;
        this->get_parameter("base_lut_file", base_lut_file);
        bool loaded = false;
        if (!base_lut_file.empty()) {
            loaded = base_lut_.load(base_lut_file);
        }
        if (!loaded) {
            try {
                std::string share_dir = ament_index_cpp::get_package_share_directory("steering_lookup");
                loaded = base_lut_.load(share_dir + "/cfg/NUC6_glc_pacejka_lookup_table.csv");
            } catch (...) {}
        }
        if (!loaded) {
            try {
                std::string share_dir = ament_index_cpp::get_package_share_directory("f1tenth_control");
                loaded = base_lut_.load(share_dir + "/cfg/NUC6_glc_pacejka_lookup_table.csv");
            } catch (...) {}
        }
        if (!loaded) {
            RCLCPP_ERROR(this->get_logger(),
                "❌ [LutCalibrator] 베이스 LUT 로드 실패 — 캘리브레이션을 진행할 수 없습니다.");
            return;
        }

        size_t n_steer = base_lut_.steer_axis().size();
        size_t n_vel = base_lut_.velocity_axis().size();
        sum_grid_.assign(n_steer, std::vector<double>(n_vel, 0.0));
        count_grid_.assign(n_steer, std::vector<int>(n_vel, 0));

        load_state(); // 이전 실행에서 누적된 상태가 있으면 이어서 사용

        // ===== 통신 =====
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, 10, std::bind(&LutCalibratorNode::imu_callback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10, std::bind(&LutCalibratorNode::odom_callback, this, std::placeholders::_1));
        drive_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            drive_topic_, 10, std::bind(&LutCalibratorNode::drive_callback, this, std::placeholders::_1));
        save_trigger_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "/lut_calibration/save", 10,
            std::bind(&LutCalibratorNode::save_trigger_callback, this, std::placeholders::_1));

        status_pub_ = this->create_publisher<std_msgs::msg::String>("/lut_calibration/status", 10);

        save_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(save_interval_sec_),
            std::bind(&LutCalibratorNode::save_all, this));
        status_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&LutCalibratorNode::publish_status, this));

        // Ctrl+C(SIGINT) 등으로 종료될 때 확실히 저장되도록 shutdown 훅에도 등록.
        // (주기 타이머·소멸자와 별개로 rclcpp::shutdown() 시점에 반드시 실행됨 — 이중 안전장치)
        // 이 콜백은 main()의 rclcpp::spin(node) 블로킹이 shutdown으로 풀리는 시점에 동기 실행되며,
        // 이때 node(및 this)는 main() 스택에서 아직 살아있음이 보장되므로 raw this 캡처가 안전함.
        rclcpp::on_shutdown([this]() { save_all(); });

        RCLCPP_INFO(this->get_logger(), "=================================================");
        RCLCPP_INFO(this->get_logger(), "LUT 캘리브레이션 노드 시작 (관찰 전용, /drive 미발행)");
        RCLCPP_INFO(this->get_logger(), " - Grid: %zu(steer) x %zu(vel)", n_steer, n_vel);
        RCLCPP_INFO(this->get_logger(), " - 상태 파일: %s", state_file_.c_str());
        RCLCPP_INFO(this->get_logger(), " - 출력 LUT : %s", output_lut_file_.c_str());
        RCLCPP_INFO(this->get_logger(), "=================================================");
    }

    ~LutCalibratorNode() override {
        save_all();
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
        double raw_yaw_rate = msg->angular_velocity.z * imu_angular_scale_;
        if (!yaw_rate_initialized_) {
            filtered_yaw_rate_ = raw_yaw_rate;
            yaw_rate_initialized_ = true;
        } else {
            filtered_yaw_rate_ = yaw_rate_alpha_ * raw_yaw_rate + (1.0 - yaw_rate_alpha_) * filtered_yaw_rate_;
        }
        try_record_sample();
    }

    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        current_speed_ = msg->twist.twist.linear.x;
    }

    void drive_callback(const ackermann_msgs::msg::AckermannDriveStamped::ConstSharedPtr msg) {
        current_steering_ = msg->drive.steering_angle;
    }

    void save_trigger_callback(const std_msgs::msg::Empty::ConstSharedPtr) {
        RCLCPP_INFO(this->get_logger(), "🔄 수동 저장 트리거 수신 — 즉시 저장합니다.");
        save_all();
    }

    // IMU 콜백(가장 높은 갱신 빈도) 시점마다 현재 캐시된 speed/steering과 묶어 샘플로 기록.
    void try_record_sample() {
        if (current_speed_ < min_speed_for_sample_) return; // 저속/정지/후진 구간 배제(노이즈)

        double lat_accel = current_speed_ * filtered_yaw_rate_; // a_lat = v * yaw_rate
        double steer_mag = std::abs(current_steering_);
        double accel_mag = std::abs(lat_accel);

        size_t steer_idx = nearest_index(base_lut_.steer_axis(), steer_mag);
        size_t vel_idx = nearest_index(base_lut_.velocity_axis(), current_speed_);

        sum_grid_[steer_idx][vel_idx] += accel_mag;
        count_grid_[steer_idx][vel_idx] += 1;
        total_samples_++;
    }

    static size_t nearest_index(const std::vector<double>& axis, double val) {
        size_t best = 0;
        double best_diff = std::numeric_limits<double>::max();
        for (size_t i = 0; i < axis.size(); ++i) {
            double d = std::abs(axis[i] - val);
            if (d < best_diff) { best_diff = d; best = i; }
        }
        return best;
    }

    void publish_status() {
        size_t n_steer = count_grid_.size();
        size_t n_vel = n_steer > 0 ? count_grid_[0].size() : 0;
        size_t filled = 0, total_cells = n_steer * n_vel;
        for (const auto& row : count_grid_)
            for (int c : row) if (c > 0) filled++;

        std::ostringstream oss;
        oss << "[LUT Calibration] samples=" << total_samples_
            << " coverage=" << filled << "/" << total_cells
            << " speed=" << current_speed_ << "m/s"
            << " steer=" << current_steering_ << "rad"
            << " yaw_rate=" << filtered_yaw_rate_ << "rad/s";
        auto msg = std_msgs::msg::String();
        msg.data = oss.str();
        status_pub_->publish(msg);
    }

    // ===== 상태(누적치) 저장/로드 — 여러 번의 주행(런치 재실행)에 걸쳐 이어붙이기 위함 =====
    void save_state() {
        std::ofstream out(state_file_);
        if (!out.is_open()) {
            RCLCPP_WARN(this->get_logger(), "상태 파일 저장 실패: %s", state_file_.c_str());
            return;
        }
        out << std::scientific << std::setprecision(15);
        out << "# lut_calibration_state v1\n";
        for (const auto& row : sum_grid_) {
            for (size_t j = 0; j < row.size(); ++j) out << (j ? "," : "") << row[j];
            out << "\n";
        }
        out << "#COUNT\n";
        for (const auto& row : count_grid_) {
            for (size_t j = 0; j < row.size(); ++j) out << (j ? "," : "") << row[j];
            out << "\n";
        }
    }

    void load_state() {
        std::ifstream in(state_file_);
        if (!in.is_open()) return; // 첫 실행이면 그냥 0으로 시작

        std::vector<std::vector<double>> sums;
        std::vector<std::vector<int>> counts;
        std::string line;
        bool in_count_section = false;
        while (std::getline(in, line)) {
            if (line.empty()) continue;
            if (line[0] == '#') {
                if (line.find("COUNT") != std::string::npos) in_count_section = true;
                continue;
            }
            std::stringstream ss(line);
            std::string cell;
            if (!in_count_section) {
                std::vector<double> row;
                while (std::getline(ss, cell, ',')) row.push_back(std::stod(cell));
                sums.push_back(row);
            } else {
                std::vector<int> row;
                while (std::getline(ss, cell, ',')) row.push_back(std::stoi(cell));
                counts.push_back(row);
            }
        }

        if (sums.size() == sum_grid_.size() && counts.size() == count_grid_.size()) {
            sum_grid_ = sums;
            count_grid_ = counts;
            int loaded_total = 0;
            for (const auto& row : count_grid_) for (int c : row) loaded_total += c;
            total_samples_ = loaded_total;
            RCLCPP_INFO(this->get_logger(),
                "🟢 이전 캘리브레이션 상태 로드 완료 (누적 샘플 %d개) — 이번 주행부터 이어서 누적합니다.",
                loaded_total);
        } else {
            RCLCPP_WARN(this->get_logger(),
                "상태 파일 grid 크기가 현재 베이스 LUT와 달라 무시합니다(초기화). 파일: %s",
                state_file_.c_str());
        }
    }

    // ===== 블렌딩된 LUT CSV 출력 =====
    void save_calibrated_lut() {
        size_t n_steer = base_lut_.steer_axis().size();
        size_t n_vel = base_lut_.velocity_axis().size();
        std::vector<std::vector<double>> blended(n_steer, std::vector<double>(n_vel, 0.0));

        for (size_t i = 0; i < n_steer; ++i) {
            for (size_t j = 0; j < n_vel; ++j) {
                double base_val = base_lut_.raw_value(i, j);
                int count = count_grid_[i][j];
                if (count <= 0 || std::isnan(base_val)) {
                    blended[i][j] = base_val; // 샘플 없으면 원본값 그대로 보존
                } else {
                    double measured_avg = sum_grid_[i][j] / static_cast<double>(count);
                    blended[i][j] = (base_val * prior_weight_ + sum_grid_[i][j])
                                     / (prior_weight_ + static_cast<double>(count));
                    (void)measured_avg; // (참고용) 순수 실측 평균은 measured_avg
                }
            }
        }

        if (base_lut_.save_csv(output_lut_file_, blended)) {
            RCLCPP_INFO(this->get_logger(), "💾 캘리브레이션 LUT 저장: %s (누적 샘플 %d개)",
                output_lut_file_.c_str(), total_samples_);
        } else {
            RCLCPP_WARN(this->get_logger(), "캘리브레이션 LUT 저장 실패: %s", output_lut_file_.c_str());
        }
    }

    void save_all() {
        save_state();
        save_calibrated_lut();
    }

    static void mkdir_p(const std::string& path) {
        mkdir(path.c_str(), 0775); // 이미 존재하면 그냥 무시(에러 체크 불필요 — best-effort)
    }

    // 파라미터
    std::string odom_topic_, imu_topic_, drive_topic_;
    std::string state_file_, output_lut_file_;
    double min_speed_for_sample_;
    double prior_weight_;
    double yaw_rate_alpha_;
    double imu_angular_scale_;
    double save_interval_sec_;

    // LUT / 누적 grid
    f1tenth_control::SteeringLookupTable base_lut_;
    std::vector<std::vector<double>> sum_grid_;
    std::vector<std::vector<int>> count_grid_;
    int total_samples_ = 0;

    // 최신 센서 캐시
    double current_speed_ = 0.0;
    double current_steering_ = 0.0;
    double filtered_yaw_rate_ = 0.0;
    bool yaw_rate_initialized_ = false;

    // ROS 2 통신 개체
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr save_trigger_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr save_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LutCalibratorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
