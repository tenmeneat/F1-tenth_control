// Copyright 2026 2026_IFAC contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Audit-only frozen-stream parity harness. It parses immutable PATH_FAMILY_STREAM_V1 inputs,
// invokes the production-owned P3/M1 core, and emits records for an external oracle comparison.
// It never embeds, links, or invokes the external evaluator.

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include "local_planning/raceline_spline_planner.hpp"

namespace
{

using local_planning::EgoFrenetState;
using local_planning::RacelineSplineParameters;
using local_planning::RacelineSplinePlanner;

struct Frame
{
  std::int64_t source_stamp_ns{0};
  EgoFrenetState ego;
  std::vector<f110_msgs::msg::Obstacle> obstacles;
};

struct Stream
{
  std::string scenario;
  std::map<std::string, double> scalars;
  std::map<std::string, std::vector<double>> vectors;
  f110_msgs::msg::WpntArray reference;
  std::vector<Frame> frames;
};

std::vector<std::string> splitTabs(const std::string & line)
{
  std::vector<std::string> tokens;
  std::size_t begin = 0U;
  while (true) {
    const std::size_t end = line.find('\t', begin);
    tokens.push_back(line.substr(begin, end == std::string::npos ? end : end - begin));
    if (end == std::string::npos) {
      return tokens;
    }
    begin = end + 1U;
  }
}

double finiteDouble(const std::string & token)
{
  std::size_t consumed = 0U;
  const double value = std::stod(token, &consumed);
  if (consumed != token.size() || !std::isfinite(value)) {
    throw std::runtime_error("invalid finite token: " + token);
  }
  return value;
}

std::int64_t integer(const std::string & token)
{
  std::size_t consumed = 0U;
  const auto value = std::stoll(token, &consumed);
  if (consumed != token.size()) {
    throw std::runtime_error("invalid integer token: " + token);
  }
  return value;
}

std::size_t count(const std::string & token)
{
  const auto value = integer(token);
  if (value < 0) {
    throw std::runtime_error("negative count");
  }
  return static_cast<std::size_t>(value);
}

Stream readStream(const std::filesystem::path & path)
{
  std::ifstream input(path);
  std::string line;
  if (!input || !std::getline(input, line) || line != "PATH_FAMILY_STREAM_V1") {
    throw std::runtime_error("unsupported frozen stream: " + path.string());
  }
  Stream stream;
  stream.reference.header.frame_id = "map";
  while (std::getline(input, line)) {
    const auto tokens = splitTabs(line);
    const bool metadata_record = tokens[0] == "SOURCE_BAG" ||
      tokens[0] == "CONFIG_PATH" || tokens[0] == "CONFIG_SHA256";
    if (tokens[0] == "SCENARIO" && tokens.size() == 2U) {
      stream.scenario = tokens[1];
    } else if (metadata_record) {
      continue;
    } else if (tokens[0] == "PARAM" && tokens.size() == 3U) {
      stream.scalars[tokens[1]] = finiteDouble(tokens[2]);
    } else if (tokens[0] == "PARAMV" && tokens.size() >= 3U) {
      const std::size_t values = count(tokens[2]);
      if (tokens.size() != values + 3U) {
        throw std::runtime_error("parameter-vector count mismatch");
      }
      auto & output = stream.vectors[tokens[1]];
      for (std::size_t index = 0U; index < values; ++index) {
        output.push_back(finiteDouble(tokens[index + 3U]));
      }
    } else if (tokens[0] == "REFERENCE" && tokens.size() == 2U) {
      stream.reference.wpnts.reserve(count(tokens[1]));
    } else if (tokens[0] == "W" && tokens.size() == 12U) {
      f110_msgs::msg::Wpnt waypoint;
      waypoint.id = static_cast<std::int32_t>(integer(tokens[1]));
      waypoint.s_m = finiteDouble(tokens[2]);
      waypoint.d_m = finiteDouble(tokens[3]);
      waypoint.x_m = finiteDouble(tokens[4]);
      waypoint.y_m = finiteDouble(tokens[5]);
      waypoint.d_right = finiteDouble(tokens[6]);
      waypoint.d_left = finiteDouble(tokens[7]);
      waypoint.psi_rad = finiteDouble(tokens[8]);
      waypoint.kappa_radpm = finiteDouble(tokens[9]);
      waypoint.vx_mps = finiteDouble(tokens[10]);
      waypoint.ax_mps2 = finiteDouble(tokens[11]);
      stream.reference.wpnts.push_back(waypoint);
    } else if (tokens[0] == "FRAME" && tokens.size() == 7U) {
      Frame frame;
      frame.source_stamp_ns = integer(tokens[1]);
      frame.ego.s = finiteDouble(tokens[3]);
      frame.ego.d = finiteDouble(tokens[4]);
      frame.ego.speed = finiteDouble(tokens[5]);
      const std::size_t obstacles = count(tokens[6]);
      for (std::size_t index = 0U; index < obstacles; ++index) {
        if (!std::getline(input, line)) {
          throw std::runtime_error("truncated obstacle frame");
        }
        const auto obstacle_tokens = splitTabs(line);
        if (obstacle_tokens.size() != 10U || obstacle_tokens[0] != "O") {
          throw std::runtime_error("malformed obstacle record");
        }
        f110_msgs::msg::Obstacle obstacle;
        obstacle.id = static_cast<std::int32_t>(integer(obstacle_tokens[1]));
        obstacle.s_center = finiteDouble(obstacle_tokens[2]);
        obstacle.s_start = finiteDouble(obstacle_tokens[3]);
        obstacle.s_end = finiteDouble(obstacle_tokens[4]);
        obstacle.d_right = finiteDouble(obstacle_tokens[5]);
        obstacle.d_left = finiteDouble(obstacle_tokens[6]);
        obstacle.size = finiteDouble(obstacle_tokens[7]);
        obstacle.s_var = finiteDouble(obstacle_tokens[8]);
        obstacle.d_var = finiteDouble(obstacle_tokens[9]);
        obstacle.d_center = 0.5 * (obstacle.d_right + obstacle.d_left);
        obstacle.is_static = true;
        obstacle.is_visible = true;
        frame.obstacles.push_back(obstacle);
      }
      if (!std::getline(input, line) || line != "END_FRAME") {
        throw std::runtime_error("missing END_FRAME");
      }
      stream.frames.push_back(std::move(frame));
    } else if (tokens[0] == "END_STREAM") {
      break;
    } else if (!line.empty()) {
      throw std::runtime_error("unknown stream record: " + line);
    }
  }
  if (stream.scenario.empty() || stream.reference.wpnts.empty() || stream.frames.empty()) {
    throw std::runtime_error("incomplete frozen stream");
  }
  return stream;
}

RacelineSplineParameters parameters(const Stream & stream)
{
  RacelineSplineParameters value;
  const auto scalar = [&](const char * name) {return stream.scalars.at(name);};
  const auto vector = [&](const char * name) {return stream.vectors.at(name);};
  value.detection_lookahead_m = scalar("detection_lookahead_m");
  value.obstacle_cluster_gap_m = scalar("obstacle_cluster_gap_m");
  value.obstacle_longitudinal_padding_m = scalar("obstacle_longitudinal_padding_m");
  value.vehicle_length_m = scalar("vehicle_length_m");
  value.vehicle_half_width_m = scalar("vehicle_half_width_m");
  value.safety_margin_m = scalar("safety_margin_m");
  value.tracking_error_reserve_m = scalar("tracking_error_reserve_m");
  value.tracking_error_lut_speed_bins_mps = vector("tracking_error_lut_speed_bins_mps");
  value.tracking_error_lut_curvature_bins_radpm =
    vector("tracking_error_lut_curvature_bins_radpm");
  value.tracking_error_lut_values_m = vector("tracking_error_lut_values_m");
  value.avoidance_velocity_limit_speed_bins_mps =
    vector("avoidance_velocity_limit_speed_bins_mps");
  value.avoidance_velocity_limit_lateral_accel_mps2 =
    vector("avoidance_velocity_limit_lateral_accel_mps2");
  value.wall_safety_margin_m = scalar("wall_safety_margin_m");
  value.fallback_track_half_width_m = scalar("fallback_track_half_width_m");
  value.pre_apex_distances_m = vector("pre_apex_distances_m");
  value.post_apex_distances_m = vector("post_apex_distances_m");
  value.entry_transition_fractions = vector("entry_transition_fractions");
  value.transition_distance_scales = vector("transition_distance_scales");
  value.outside_line_transition_scale = scalar("outside_line_transition_scale");
  value.post_merge_lookahead_m = scalar("post_merge_lookahead_m");
  value.post_merge_min_time_sec = scalar("post_merge_min_time_sec");
  value.minimum_target_offset_m = scalar("minimum_target_offset_m");
  value.maximum_target_offset_m = scalar("maximum_target_offset_m");
  value.target_d_candidate_count =
    static_cast<int>(std::llround(scalar("target_d_candidate_count")));
  value.maximum_lateral_slope = scalar("maximum_lateral_slope");
  value.maximum_curvature_radpm = scalar("maximum_curvature_radpm");
  value.maximum_curvature_rate_radpm2 = scalar("maximum_curvature_rate_radpm2");
  value.safe_stop_buffer_m = scalar("safe_stop_buffer_m");
  value.safe_stop_deceleration_mps2 = scalar("safe_stop_deceleration_mps2");
  value.minimum_path_points = static_cast<int>(std::llround(scalar("minimum_path_points")));
  return value;
}

std::string clean(std::string value)
{
  for (char & character : value) {
    if (character == '\t' || character == '\n' || character == '\r') {
      character = ' ';
    }
  }
  return value;
}

void emit(const Stream & stream)
{
  RacelineSplinePlanner planner(parameters(stream));
  std::string error;
  if (!planner.setReference(stream.reference, &error)) {
    throw std::runtime_error("invalid frozen reference: " + error);
  }
  for (std::size_t frame_index = 0U; frame_index < stream.frames.size(); ++frame_index) {
    const auto & frame = stream.frames[frame_index];
    const auto result = planner.evaluateP3Shadow(
      frame.ego, frame.obstacles, frame.source_stamp_ns, 0U, 1U, "PARITY_ORACLE");
    std::cout << stream.scenario << '\t' << frame_index << '\t' << "SUMMARY" << '\t' << -1 <<
      '\t' << result.selected_candidate_identity << '\t' << result.selected_path_digest << '\t' <<
      result.would_recover << '\t' << clean(result.failure_classification) << '\t' <<
      result.selected_candidate_identity << '\t' << result.selected_path_digest << '\t' <<
      result.m1_invoked << '\t' << result.candidate_count << '\t' <<
      result.hard_validator_call_count << '\t' << result.m0_candidate_count << '\t' <<
      result.m1_candidate_count << '\n';
    for (std::size_t index = 0U; index < result.candidates.size(); ++index) {
      const auto & candidate = result.candidates[index];
      std::cout << stream.scenario << '\t' << frame_index << '\t' << "CANDIDATE" << '\t' <<
        index << '\t' << candidate.candidate_identity << '\t' << candidate.path_digest << '\t' <<
        candidate.hard_valid << '\t' << clean(candidate.rejection_reason) << '\t' <<
        result.selected_candidate_identity << '\t' << result.selected_path_digest << '\t' <<
        result.m1_invoked << '\t' << result.candidate_count << '\t' <<
        result.hard_validator_call_count << '\t' << result.m0_candidate_count << '\t' <<
        result.m1_candidate_count << '\n';
    }
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cerr << "usage: p3_production_parity_harness STREAM [STREAM ...]\n";
    return 2;
  }
  try {
    std::cout << "scenario\tframe_index\trecord\tcandidate_index\tcandidate_identity\t"
      "candidate_path_digest\thard_valid\tvalidator_result\tselected_identity\t"
      "selected_path_digest\tm1_invoked\ttotal_candidates\ttotal_validator_calls\t"
      "m0_candidates\tm1_candidates\n";
    for (int index = 1; index < argc; ++index) {
      emit(readStream(argv[index]));
    }
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "production P3 parity harness failed: " << error.what() << '\n';
    return 1;
  }
}
