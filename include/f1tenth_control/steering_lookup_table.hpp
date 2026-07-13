#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
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

        is_loaded_ = true;
        std::cout << "[SteeringLookupTable] Loaded LUT with size: "
                  << lu_.size() << "x" << lu_[0].size() << " from " << file_path << std::endl;
        return true;
    }

    bool is_loaded() const { return is_loaded_; }

    // ===== 실차 LUT 캘리브레이션(lut_calibrator_node)용 조회/저장 헬퍼 =====
    // 기존 lookup_steer_angle() 동작에는 전혀 영향 없는 추가 전용 메서드들.
    const std::vector<double>& velocity_axis() const { return lu_vs_; }
    const std::vector<double>& steer_axis() const { return lu_steers_; }

    // steer_idx/vel_idx는 steer_axis()/velocity_axis() 상의 0-based 인덱스(헤더 행/열 제외).
    double raw_value(size_t steer_idx, size_t vel_idx) const {
        size_t row = steer_idx + 1, col = vel_idx + 1;
        if (row >= lu_.size() || col >= lu_[row].size()) return std::numeric_limits<double>::quiet_NaN();
        return lu_[row][col];
    }

    // steer_axis() x velocity_axis() 크기의 grid를 원본과 동일한 CSV 포맷(행0=속도축, 열0=조향축)으로 저장.
    bool save_csv(const std::string& path, const std::vector<std::vector<double>>& accel_grid) const {
        if (!is_loaded_) return false;
        if (accel_grid.size() != lu_steers_.size()) return false;

        std::ofstream out(path);
        if (!out.is_open()) return false;
        out << std::scientific << std::setprecision(15);

        // 행 0: 첫 칸(throwaway 0.0) + 속도축
        out << 0.0;
        for (double v : lu_vs_) out << "," << v;
        out << "\n";

        for (size_t i = 0; i < lu_steers_.size(); ++i) {
            out << lu_steers_[i];
            for (size_t j = 0; j < lu_vs_.size(); ++j) {
                double val = (j < accel_grid[i].size()) ? accel_grid[i][j] : std::numeric_limits<double>::quiet_NaN();
                out << "," << val;
            }
            out << "\n";
        }
        return true;
    }

    double lookup_steer_angle(double accel, double vel) {
        if (!is_loaded_) return 0.0;

        double sign_accel = (accel > 0.0) ? 1.0 : -1.0;
        accel = std::abs(accel);

        // find closest velocity column index c_v_idx
        auto [c_v, c_v_idx] = find_nearest(lu_vs_, vel);
        size_t target_col = c_v_idx + 1; // +1 to offset steering column

        // Extract acceleration column corresponding to c_v_idx
        std::vector<double> col_accel;
        col_accel.reserve(lu_.size() - 1);
        for (size_t i = 1; i < lu_.size(); ++i) {
            if (target_col < lu_[i].size()) {
                col_accel.push_back(lu_[i][target_col]);
            } else {
                col_accel.push_back(std::numeric_limits<double>::quiet_NaN());
            }
        }

        // 각 속도 열은 조향각(row)이 커질수록 lat_acc가 단조 증가하다가 타이어가 슬립각
        // 한계(Pacejka 피크)를 넘으면 다시 감소하는 "봉우리형" 곡선이다(전 속도축 실측 확인,
        // 단일 피크). find_closest_neighbors는 target에 가장 가까운 값과 그 이웃으로
        // 선형보간하는데, 피크를 넘는 목표 lat_acc가 들어오면 피크 양쪽(저조향/고조향)의
        // 서로 다른 두 조향각이 "같은 정도로 가까운 값"이 되어 매 사이클 어느 쪽이 선택되는지
        // 진동해 조향 채터링/포화가 반복된다. 피크 이후 구간을 NaN 처리해 검색을 "피크
        // 이전(그립 내)" 단조 구간으로 제한하면, 피크를 넘는 요청은 그 속도의 최대 그립
        // 조향각으로 자연히 saturate되어 항상 하나의 안정적인 해로 수렴한다
        // (find_closest_neighbors는 첫 NaN에서 순회를 멈추므로 피크 이후를 NaN으로 채우기만
        // 하면 됨).
        {
            size_t peak_idx = 0;
            double peak_val = -std::numeric_limits<double>::infinity();
            for (size_t i = 0; i < col_accel.size(); ++i) {
                if (!std::isnan(col_accel[i]) && col_accel[i] > peak_val) {
                    peak_val = col_accel[i];
                    peak_idx = i;
                }
            }
            for (size_t i = peak_idx + 1; i < col_accel.size(); ++i) {
                col_accel[i] = std::numeric_limits<double>::quiet_NaN();
            }
        }

        // Find two closest accelerations to target accel
        auto neighbors = find_closest_neighbors(col_accel, accel);
        double steer_angle = 0.0;

        if (neighbors.c_a_idx == neighbors.s_a_idx) {
            steer_angle = lu_steers_[neighbors.c_a_idx];
        } else {
            // Linear interpolation between the two closest accelerations
            double c_steer = lu_steers_[neighbors.c_a_idx];
            double s_steer = lu_steers_[neighbors.s_a_idx];
            double denom = neighbors.s_a - neighbors.c_a;
            if (std::abs(denom) > 1e-6) {
                steer_angle = c_steer + (accel - neighbors.c_a) / denom * (s_steer - c_steer);
            } else {
                steer_angle = c_steer;
            }
        }

        return steer_angle * sign_accel;
    }

private:
    struct NeighborResult {
        double c_a; size_t c_a_idx;
        double s_a; size_t s_a_idx;
    };

    std::pair<double, size_t> find_nearest(const std::vector<double>& arr, double val) {
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

    NeighborResult find_closest_neighbors(const std::vector<double>& arr, double val) {
        std::vector<double> clean_arr;
        std::vector<size_t> orig_indices;
        for (size_t i = 0; i < arr.size(); ++i) {
            if (std::isnan(arr[i])) break; // nan array termination matching numpy's nan slice behavior
            clean_arr.push_back(arr[i]);
            orig_indices.push_back(i);
        }

        if (clean_arr.empty()) return {0.0, 0, 0.0, 0};

        auto [closest, closest_idx] = find_nearest(clean_arr, val);
        size_t orig_closest_idx = orig_indices[closest_idx];

        if (closest_idx == 0) {
            return {clean_arr[0], orig_indices[0], clean_arr[0], orig_indices[0]};
        }
        if (closest_idx == clean_arr.size() - 1) {
            size_t last = clean_arr.size() - 1;
            return {clean_arr[last], orig_indices[last], clean_arr[last], orig_indices[last]};
        }

        size_t prev_idx = closest_idx - 1;
        size_t next_idx = closest_idx + 1;
        double prev_val = clean_arr[prev_idx];
        double next_val = clean_arr[next_idx];

        size_t second_idx = (std::abs(prev_val - val) < std::abs(next_val - val)) ? prev_idx : next_idx;
        size_t orig_second_idx = orig_indices[second_idx];

        return {closest, orig_closest_idx, clean_arr[second_idx], orig_second_idx};
    }

    std::vector<std::vector<double>> lu_;
    std::vector<double> lu_vs_;     // cached velocity axis (row 0, cols 1..)
    std::vector<double> lu_steers_; // cached steer angle axis (col 0, rows 1..)
    bool is_loaded_;
};

} // namespace f1tenth_control
