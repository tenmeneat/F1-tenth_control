#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <limits>
#include <utility>

namespace f1tenth_control {

class SteeringLookupTable {
public:
    SteeringLookupTable() : is_loaded_(false) {}

    bool load(const std::string& file_path) {
        std::ifstream file(file_path);
        if (!file.is_open()) {
            std::cerr << "[SteeringLookupTable] Failed to open CSV file: " << file_path << std::endl;
            is_loaded_ = false;
            return false;
        }

        lu_.clear();
        std::string line;
        while (std::getline(file, line)) {
            if (line.empty()) continue;
            std::vector<double> row;
            std::stringstream ss(line);
            std::string cell;
            while (std::getline(ss, cell, ',')) {
                try {
                    row.push_back(std::stod(cell));
                } catch (...) {
                    row.push_back(std::numeric_limits<double>::quiet_NaN());
                }
            }
            lu_.push_back(row);
        }

        if (lu_.empty() || lu_[0].empty()) {
            std::cerr << "[SteeringLookupTable] CSV file is empty or invalid: " << file_path << std::endl;
            is_loaded_ = false;
            return false;
        }

        // Cache velocity axis (row 0, skip first cell) and steer axis (col 0, skip first row)
        lu_vs_.assign(lu_[0].begin() + 1, lu_[0].end());
        lu_steers_.clear();
        lu_steers_.reserve(lu_.size() - 1);
        for (size_t i = 1; i < lu_.size(); ++i) {
            lu_steers_.push_back(lu_[i][0]);
        }

        // 속도 열별 "그립 내 단조 구간" 사전 계산. 조회 때마다 열을 추출하고 피크를
        // 찾던 것을 로드 시 1회로 옮긴 것 — 50Hz 조향 경로에서 힙 할당이 사라진다.
        build_columns();

        is_loaded_ = true;
        std::cout << "[SteeringLookupTable] Loaded LUT with size: "
                  << lu_.size() << "x" << lu_[0].size() << " from " << file_path << std::endl;
        return true;
    }

    // lut_calibrator_node가 실측 보정 LUT를 만들 때 쓰는 그리드 접근/저장 API.
    const std::vector<double>& steer_axis() const { return lu_steers_; }
    const std::vector<double>& velocity_axis() const { return lu_vs_; }

    // (steer_idx, vel_idx) 그리드 셀의 원본 LUT 값(횡가속도). load()가 채운 lu_는
    // [0]행=속도축 헤더, [i+1]행의 [0]열=조향축 헤더라 +1 오프셋으로 조회한다.
    double raw_value(size_t steer_idx, size_t vel_idx) const {
        size_t row = steer_idx + 1;
        size_t col = vel_idx + 1;
        if (row < lu_.size() && col < lu_[row].size()) return lu_[row][col];
        return std::numeric_limits<double>::quiet_NaN();
    }

    // 블렌딩된 grid(data[steer_idx][vel_idx])를 원본과 동일한 CSV 포맷(행=조향각축,
    // 열=속도축, 좌상단 코너 셀 보존)으로 저장.
    bool save_csv(const std::string& file_path, const std::vector<std::vector<double>>& data) const {
        std::ofstream out(file_path);
        if (!out.is_open()) return false;

        out << lu_[0][0];
        for (double v : lu_vs_) out << "," << v;
        out << "\n";

        for (size_t i = 0; i < lu_steers_.size(); ++i) {
            out << lu_steers_[i];
            for (size_t j = 0; j < lu_vs_.size(); ++j) {
                double val = (i < data.size() && j < data[i].size())
                                 ? data[i][j] : std::numeric_limits<double>::quiet_NaN();
                out << "," << val;
            }
            out << "\n";
        }
        return true;
    }

    // (횡가속도, 속도) → 조향각.
    //
    // ⚠️ 2026-07-30: 속도축을 **최근접 열 하나**로 고르던 것을 인접 두 열 선형보간으로 바꿨다.
    //    속도축 간격이 0.1016 m/s라 최근접 방식은 순항 중 속도가 열 경계를 넘을 때마다 조향각이
    //    계단으로 튄다(50Hz에서 경계 근처를 오가면 그대로 채터링). 그립 포화 조향각이 속도에
    //    민감한 고속(v=6.19 → 0.121 rad, v=7.0 → 0.100 rad)에서 특히 크다.
    //
    // saturated(옵션): 요청 횡가속도가 그 속도의 **그립 피크를 넘어** 조향각이 saturate됐는지.
    //   포화 중에는 입력이 얼마나 커도 출력이 같아서 조향 피드백이 사실상 개루프가 되므로,
    //   호출부에서 진단 로그를 띄우는 데 쓴다(제어 개입은 아님).
    double lookup_steer_angle(double accel, double vel, bool* saturated = nullptr) {
        if (saturated) *saturated = false;
        if (!is_loaded_ || lu_vs_.empty()) return 0.0;

        double sign_accel = (accel > 0.0) ? 1.0 : -1.0;
        accel = std::abs(accel);

        // 속도축 범위 밖은 끝 열로 클램프한다. ⚠️ 축 상한은 7.0 m/s이므로 그보다 빠르면
        //    이 클램프가 걸린다 — 호출부(control_map_node)가 자전거모델로 초과분을 보정한다.
        const double v_lo = lu_vs_.front(), v_hi = lu_vs_.back();
        const double vc = std::clamp(vel, std::min(v_lo, v_hi), std::max(v_lo, v_hi));

        // 브래키팅 두 열 찾기(축은 오름차순 균일이지만 방어적으로 탐색).
        size_t j1 = lu_vs_.size() - 1;
        for (size_t j = 0; j < lu_vs_.size(); ++j) {
            if (lu_vs_[j] >= vc) { j1 = j; break; }
        }
        const size_t j0 = (j1 == 0) ? 0 : (j1 - 1);

        bool sat0 = false, sat1 = false;
        const double s0 = column_lookup(j0, accel, sat0);
        const double s1 = column_lookup(j1, accel, sat1);
        if (saturated) *saturated = (sat0 || sat1);

        double steer_angle = s0;
        const double v_span = lu_vs_[j1] - lu_vs_[j0];
        if (std::abs(v_span) > 1e-9) {
            const double t = std::clamp((vc - lu_vs_[j0]) / v_span, 0.0, 1.0);
            steer_angle = s0 + t * (s1 - s0);
        }

        return steer_angle * sign_accel;
    }

    // 속도축 상한 [m/s] — 호출부가 축 범위 초과를 알고 보정할 수 있게 노출.
    double max_velocity() const { return lu_vs_.empty() ? 0.0 : lu_vs_.back(); }

private:
    // 단일 속도 열(그립 내 단조 구간) 안에서 |accel| → 조향각 보간. 요청이 그 열의 그립
    // 피크를 넘으면 피크 조향각으로 saturate하고 sat=true를 돌려준다.
    double column_lookup(size_t j, double accel, bool& sat) const {
        sat = false;
        // 유효한 조향 해가 없는 열이면 조향축 첫 값으로 떨어진다(기존 동작 유지).
        const std::vector<double>& col_accel = cols_[j];
        if (col_accel.empty()) return lu_steers_.empty() ? 0.0 : lu_steers_[0];

        if (accel >= col_accel.back()) sat = true;

        auto neighbors = find_closest_neighbors(col_accel, accel);
        if (neighbors.c_a_idx == neighbors.s_a_idx) return lu_steers_[neighbors.c_a_idx];

        // 두 최근접 횡가속도 사이 선형보간
        const double c_steer = lu_steers_[neighbors.c_a_idx];
        const double s_steer = lu_steers_[neighbors.s_a_idx];
        const double denom = neighbors.s_a - neighbors.c_a;
        if (std::abs(denom) <= 1e-6) return c_steer;
        return c_steer + (accel - neighbors.c_a) / denom * (s_steer - c_steer);
    }

    struct NeighborResult {
        double c_a; size_t c_a_idx;
        double s_a; size_t s_a_idx;
    };

    std::pair<double, size_t> find_nearest(const std::vector<double>& arr, double val) const {
        double min_diff = std::numeric_limits<double>::max();
        size_t best_idx = 0;
        for (size_t i = 0; i < arr.size(); ++i) {
            if (std::isnan(arr[i])) continue;
            double diff = std::abs(arr[i] - val);
            if (diff < min_diff) {
                min_diff = diff;
                best_idx = i;
            }
        }
        return {arr[best_idx], best_idx};
    }

    // arr는 build_columns()가 NaN을 걷어낸 배열이라 인덱스가 곧 조향축 인덱스다.
    NeighborResult find_closest_neighbors(const std::vector<double>& arr, double val) const {
        if (arr.empty()) return {0.0, 0, 0.0, 0};

        auto [closest, closest_idx] = find_nearest(arr, val);

        if (closest_idx == 0) {
            return {arr[0], 0, arr[0], 0};
        }
        if (closest_idx == arr.size() - 1) {
            size_t last = arr.size() - 1;
            return {arr[last], last, arr[last], last};
        }

        size_t prev_idx = closest_idx - 1;
        size_t next_idx = closest_idx + 1;
        size_t second_idx = (std::abs(arr[prev_idx] - val) < std::abs(arr[next_idx] - val))
                                ? prev_idx : next_idx;

        return {closest, closest_idx, arr[second_idx], second_idx};
    }

    // 속도 열마다 조향축 방향 "피크 이전(그립 내) 단조 구간"만 잘라 캐시한다.
    //
    // 각 속도 열은 조향각(row)이 커질수록 lat_acc가 단조 증가하다가 타이어가 슬립각
    // 한계(Pacejka 피크)를 넘으면 다시 감소하는 "봉우리형" 곡선이다(전 속도축 실측 확인,
    // 단일 피크). find_closest_neighbors는 target에 가장 가까운 값과 그 이웃으로
    // 선형보간하는데, 피크를 넘는 목표 lat_acc가 들어오면 피크 양쪽(저조향/고조향)의
    // 서로 다른 두 조향각이 "같은 정도로 가까운 값"이 되어 매 사이클 어느 쪽이 선택되는지
    // 진동해 조향 채터링/포화가 반복된다. 피크 이후를 잘라 검색을 단조 구간으로 제한하면,
    // 피크를 넘는 요청은 그 속도의 최대 그립 조향각으로 자연히 saturate되어 항상 하나의
    // 안정적인 해로 수렴한다.
    void build_columns() {
        cols_.assign(lu_vs_.size(), {});

        for (size_t j = 0; j < lu_vs_.size(); ++j) {
            const size_t target_col = j + 1;  // +1 to offset steering column

            // 피크 인덱스 탐색(결측 셀은 건너뜀)
            size_t peak_idx = 0;
            double peak_val = -std::numeric_limits<double>::infinity();
            for (size_t i = 1; i < lu_.size(); ++i) {
                if (target_col >= lu_[i].size()) continue;
                const double v = lu_[i][target_col];
                if (!std::isnan(v) && v > peak_val) {
                    peak_val = v;
                    peak_idx = i - 1;
                }
            }

            // 선두부터 피크까지, 단 결측/NaN을 만나면 거기서 중단(numpy의 nan-slice 동작)
            std::vector<double>& col = cols_[j];
            col.reserve(peak_idx + 1);
            for (size_t i = 0; i <= peak_idx && i + 1 < lu_.size(); ++i) {
                const std::vector<double>& row = lu_[i + 1];
                if (target_col >= row.size() || std::isnan(row[target_col])) break;
                col.push_back(row[target_col]);
            }
        }
    }

    std::vector<std::vector<double>> lu_;
    std::vector<double> lu_vs_;     // cached velocity axis (row 0, cols 1..)
    std::vector<double> lu_steers_; // cached steer angle axis (col 0, rows 1..)
    std::vector<std::vector<double>> cols_; // 속도 열별 피크 이전 단조 구간 (조회 시 무할당)
    bool is_loaded_;
};

} // namespace f1tenth_control
