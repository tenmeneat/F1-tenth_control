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

        is_loaded_ = true;
        std::cout << "[SteeringLookupTable] Loaded LUT with size: "
                  << lu_.size() << "x" << lu_[0].size() << " from " << file_path << std::endl;
        return true;
    }

    bool is_loaded() const { return is_loaded_; }

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
