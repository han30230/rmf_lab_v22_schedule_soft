#!/usr/bin/env python3
"""Prepare an isolated rmf_traffic workspace with corridor-aware A* policy.

The experiment is implemented as a header-only internal policy provider plus a
small hook in the real DifferentialDrivePlanner expansion. It does not change
public RMF headers or ABI and is bypassed when no policy input is configured.
"""

from __future__ import annotations

import argparse
import json
import re
import shutil
from pathlib import Path


PATCH_MARKER = "RMF_TRAFFIC_LAB_SCHEDULE_CORRIDOR_POLICY_V5"
OLD_PATCH_MARKER = "RMF_TRAFFIC_LAB_SCHEDULE_CORRIDOR_POLICY_V4"
PLANNER_RELATIVE_PATHS = (
    Path("rmf_traffic/src/rmf_traffic/agv/planning/DifferentialDrivePlanner.cpp"),
    Path("src/rmf_traffic/agv/planning/DifferentialDrivePlanner.cpp"),
)
NEGOTIATOR_RELATIVE_PATHS = (
    Path("rmf_traffic/src/rmf_traffic/agv/SimpleNegotiator.cpp"),
    Path("src/rmf_traffic/agv/SimpleNegotiator.cpp"),
)
POLICY_HEADER_NAME = "RmfLabCorridorPolicy.hpp"


POLICY_HEADER_CODE = r'''/* RMF_TRAFFIC_LAB_SCHEDULE_CORRIDOR_POLICY_V5
 * Internal experiment only: no public rmf_traffic ABI is changed. */
#ifndef RMF_TRAFFIC__AGV__PLANNING__RMF_LAB_CORRIDOR_POLICY_HPP
#define RMF_TRAFFIC__AGV__PLANNING__RMF_LAB_CORRIDOR_POLICY_HPP

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace rmf_traffic { namespace agv { namespace planning {
namespace rmf_lab_policy {

enum class Direction { A_TO_B, B_TO_A, Unknown };
enum class State { Free, Reserved, Occupied, UnknownHold };

inline Direction parse_direction(const std::string& value)
{
  if (value == "A_TO_B") return Direction::A_TO_B;
  if (value == "B_TO_A") return Direction::B_TO_A;
  return Direction::Unknown;
}
inline State parse_state(const std::string& value)
{
  if (value == "RESERVED") return State::Reserved;
  if (value == "OCCUPIED") return State::Occupied;
  if (value == "UNKNOWN_HOLD") return State::UnknownHold;
  return State::Free;
}
inline const char* direction_name(const Direction value)
{
  if (value == Direction::A_TO_B) return "A_TO_B";
  if (value == Direction::B_TO_A) return "B_TO_A";
  return "UNKNOWN";
}
inline const char* state_name(const State value)
{
  if (value == State::Reserved) return "RESERVED";
  if (value == State::Occupied) return "OCCUPIED";
  if (value == State::UnknownHold) return "UNKNOWN_HOLD";
  return "FREE";
}
inline bool opposite(const Direction lhs, const Direction rhs)
{
  return lhs != Direction::Unknown && rhs != Direction::Unknown && lhs != rhs;
}

struct Weights
{
  double same_per_second = 0.25;
  double opposite_per_second = 8.0;
  double occupied_per_second = 1.5;
  double future_per_second = 0.6;
  double no_escape = 25.0;
  double static_penalty = 0.0;
  double overlap_margin = 0.25;
  double schedule_soft_lambda = 0.25;
  double schedule_soft_max_penalty = 10.0;
  double schedule_soft_same_weight = 0.5;
  double schedule_soft_opposite_weight = 1.5;
};
struct Corridor
{
  std::string id;
  std::size_t capacity = 1;
  bool passing_allowed = false;
  bool hard_opposite_block = true;
  double base_penalty = 0.0;
};
struct LaneBinding
{
  std::string corridor;
  Direction direction = Direction::Unknown;
};
struct Interval
{
  std::string corridor;
  std::size_t participant = 0;
  std::size_t plan = 0;
  std::size_t route = 0;
  Direction direction = Direction::Unknown;
  double enter = 0.0;
  double exit = 0.0;
  State state = State::Reserved;
  bool owner = false;
  bool responsive = true;
  std::size_t itinerary_version = 0;
  std::string source = "SCHEDULE";
};
struct Snapshot
{
  std::string signature;
  std::string mode = "baseline";
  std::size_t schedule_version = 0;
  std::size_t participant = 0;
  Weights weights;
  std::unordered_map<std::string, Corridor> corridors;
  std::unordered_map<std::size_t, LaneBinding> lanes;
  std::unordered_map<std::string, std::vector<Interval>> intervals;
  std::unordered_map<std::size_t, double> legacy_lane_penalties;
};
struct Overlap
{
  Interval interval;
  double enter = 0.0;
  double exit = 0.0;
  double duration = 0.0;
  double admission_duration = 0.0;
  bool is_opposite = false;
};
struct CorridorDecision
{
  std::size_t candidate_id = 0;
  std::size_t parent_search_node_id = 0;
  std::size_t current_waypoint = 0;
  std::size_t target_waypoint = 0;
  std::vector<std::size_t> lane_ids;
  std::string corridor;
  Direction direction = Direction::Unknown;
  double enter = 0.0;
  double exit = 0.0;
  bool is_entry = true;
  bool hard_block = false;
  double static_penalty = 0.0;
  double same_penalty = 0.0;
  double opposite_penalty = 0.0;
  double occupancy_penalty = 0.0;
  double no_escape_penalty = 0.0;
  double total = 0.0;
  double parent_g = 0.0;
  double approach_cost = 0.0;
  double rmf_core_alt_cost = 0.0;
  double base_move_cost = 0.0;
  double rotation_cost = 0.0;
  double event_cost = 0.0;
  double wait_cost = 0.0;
  double final_g = 0.0;
  double h = 0.0;
  double f = 0.0;
  std::string reason = "NO_POLICY_COST";
  std::size_t queried_interval_count = 0;
  std::size_t overlap_check_count = 0;
  std::vector<Overlap> overlaps;
};
struct MotionBreakdown
{
  double movement_time = 0.0;
  double rotation_time = 0.0;
  double waiting_time = 0.0;
};
struct Decision
{
  bool hard_block = false;
  double total_penalty = 0.0;
  std::vector<CorridorDecision> corridors;
};

template<typename Routes>
inline MotionBreakdown analyze_routes(const Routes& routes)
{
  MotionBreakdown output;
  for (const auto& route : routes)
  {
    auto previous = route.trajectory().begin();
    if (previous == route.trajectory().end()) continue;
    auto current = previous; ++current;
    for (; current != route.trajectory().end(); ++current, ++previous)
    {
      const double dt = std::chrono::duration<double>(
        current->time() - previous->time()).count();
      const auto delta = current->position() - previous->position();
      const double translation = std::hypot(delta[0], delta[1]);
      const double rotation = std::abs(delta[2]);
      if (translation > 1e-6) output.movement_time += dt;
      else if (rotation > 1e-6) output.rotation_time += dt;
      else output.waiting_time += dt;
    }
  }
  return output;
}

inline std::vector<std::string> split(const std::string& input, const char token)
{
  std::vector<std::string> output;
  std::stringstream stream(input); std::string value;
  while (std::getline(stream, value, token)) output.push_back(value);
  return output;
}
inline bool parse_bool(const std::string& value)
{
  return value == "1" || value == "true" || value == "TRUE";
}
inline std::string environment(const char* name)
{
  const char* value = std::getenv(name);
  return value ? std::string(value) : std::string();
}
inline void parse_legacy_lane_penalties(Snapshot& snapshot)
{
  for (const auto& item : split(environment("RMF_TRAFFIC_LAB_LANE_PENALTIES"), ','))
  {
    const auto fields = split(item, ':'); if (fields.size() != 2) continue;
    try
    {
      const auto lane = static_cast<std::size_t>(std::stoull(fields[0]));
      const double value = std::stod(fields[1]);
      if (std::isfinite(value) && value > 0.0)
        snapshot.legacy_lane_penalties[lane] = value;
    }
    catch (const std::exception&) {}
  }
}
inline Snapshot read_snapshot(const std::string& signature)
{
  Snapshot output; output.signature = signature; parse_legacy_lane_penalties(output);
  const std::string path = environment("RMF_TRAFFIC_LAB_POLICY_SNAPSHOT");
  if (path.empty()) return output;
  std::ifstream stream(path); std::string line;
  while (std::getline(stream, line))
  {
    const auto f = split(line, '\t'); if (f.empty()) continue;
    try
    {
      if (f[0] == "META" && f.size() >= 5)
      {
        output.mode = f[2]; output.schedule_version = std::stoull(f[3]);
        output.participant = std::stoull(f[4]);
      }
      else if (f[0] == "WEIGHTS" && f.size() >= 8)
      {
        output.weights.same_per_second = std::stod(f[1]);
        output.weights.opposite_per_second = std::stod(f[2]);
        output.weights.occupied_per_second = std::stod(f[3]);
        output.weights.future_per_second = std::stod(f[4]);
        output.weights.no_escape = std::stod(f[5]);
        output.weights.static_penalty = std::stod(f[6]);
        output.weights.overlap_margin = std::stod(f[7]);
        if (f.size() >= 10)
        {
          output.weights.schedule_soft_lambda = std::stod(f[8]);
          output.weights.schedule_soft_max_penalty = std::stod(f[9]);
        }
        if (f.size() >= 12)
        {
          output.weights.schedule_soft_same_weight = std::stod(f[10]);
          output.weights.schedule_soft_opposite_weight = std::stod(f[11]);
        }
      }
      else if (f[0] == "CORRIDOR" && f.size() >= 6)
      {
        Corridor c; c.id = f[1]; c.capacity = std::stoull(f[2]);
        c.passing_allowed = parse_bool(f[3]); c.hard_opposite_block = parse_bool(f[4]);
        c.base_penalty = std::stod(f[5]); output.corridors[c.id] = c;
      }
      else if (f[0] == "LANE" && f.size() >= 4)
        output.lanes[std::stoull(f[1])] = LaneBinding{f[2], parse_direction(f[3])};
      else if (f[0] == "INTERVAL" && f.size() >= 13)
      {
        Interval i; i.corridor = f[1]; i.participant = std::stoull(f[2]);
        i.plan = std::stoull(f[3]); i.route = std::stoull(f[4]);
        i.direction = parse_direction(f[5]); i.enter = std::stod(f[6]);
        i.exit = std::stod(f[7]); i.state = parse_state(f[8]);
        i.owner = parse_bool(f[9]); i.responsive = parse_bool(f[10]);
        i.itinerary_version = std::stoull(f[11]); i.source = f[12];
        output.intervals[i.corridor].push_back(i);
      }
    }
    catch (const std::exception&) {}
  }
  for (auto& item : output.intervals)
  {
    auto& values = item.second;
    std::sort(values.begin(), values.end(), [](const Interval& a, const Interval& b)
      {
        return std::tie(a.enter, a.exit, a.participant, a.plan, a.route)
          < std::tie(b.enter, b.exit, b.participant, b.plan, b.route);
      });
  }
  return output;
}
inline const Snapshot& snapshot()
{
  const std::string signature = environment("RMF_TRAFFIC_LAB_POLICY_SNAPSHOT")
    + "|" + environment("RMF_TRAFFIC_LAB_POLICY_GENERATION")
    + "|" + environment("RMF_TRAFFIC_LAB_LANE_PENALTIES");
  thread_local Snapshot cached;
  if (cached.signature != signature) cached = read_snapshot(signature);
  return cached;
}
inline std::optional<std::size_t>& participant_override()
{
  thread_local std::optional<std::size_t> value;
  return value;
}
class ParticipantScope
{
public:
  explicit ParticipantScope(const std::size_t participant)
  : _previous(participant_override())
  {
    participant_override() = participant;
  }
  ~ParticipantScope()
  {
    participant_override() = _previous;
  }
  ParticipantScope(const ParticipantScope&) = delete;
  ParticipantScope& operator=(const ParticipantScope&) = delete;
private:
  std::optional<std::size_t> _previous;
};
inline std::size_t current_participant(const Snapshot& value)
{
  return participant_override().value_or(value.participant);
}
inline bool enabled()
{
  const auto& value = snapshot();
  if (value.mode == "schedule_soft")
  {
    return value.weights.schedule_soft_lambda > 0.0
      && value.weights.schedule_soft_max_penalty > 0.0;
  }
  return value.mode != "baseline" || !value.legacy_lane_penalties.empty();
}
inline std::string json_escape(const std::string& input)
{
  std::ostringstream out;
  for (const char c : input)
  {
    if (c == '\\' || c == '"') out << '\\' << c;
    else if (c == '\n') out << "\\n";
    else if (c == '\r') out << "\\r";
    else if (c == '\t') out << "\\t";
    else out << c;
  }
  return out.str();
}
inline void trace(const CorridorDecision& value, const Snapshot& data)
{
  const std::string path = environment("RMF_TRAFFIC_LAB_POLICY_TRACE");
  if (path.empty()) return;
  std::ofstream out(path, std::ios::app); if (!out) return;
  out << std::setprecision(15)
      << "{\"event\":\"corridor_policy_expansion\","
      << "\"source\":\"RMF_CORE\",\"schedule_source\":\"SCHEDULE\","
      << "\"analysis_source\":\"POLICY_DERIVED\","
      << "\"mode\":\"" << json_escape(data.mode) << "\","
      << "\"schedule_version\":" << data.schedule_version << ","
      << "\"participant_id\":" << current_participant(data) << ","
      << "\"participant_context_source\":\""
      << (participant_override() ? "SIMPLE_NEGOTIATOR_SCOPE" : "SNAPSHOT_FALLBACK")
      << "\","
      << "\"candidate_id\":" << value.candidate_id << ","
      << "\"parent_id\":" << value.parent_search_node_id << ","
      << "\"current_waypoint\":" << value.current_waypoint << ","
      << "\"target_waypoint\":" << value.target_waypoint << ","
      << "\"lane_ids\":[";
  for (std::size_t n = 0; n < value.lane_ids.size(); ++n)
  {
    if (n) out << ',';
    out << value.lane_ids[n];
  }
  out << "],"
      << "\"corridor_id\":\"" << json_escape(value.corridor) << "\","
      << "\"direction\":\"" << direction_name(value.direction) << "\","
      << "\"predicted_enter_time\":" << value.enter << ","
      << "\"predicted_exit_time\":" << value.exit << ","
      << "\"interval_basis\":\"RMF_CORE_ROUTE_TRAJECTORY_ENVELOPE\","
      << "\"is_entry\":" << (value.is_entry ? "true" : "false") << ","
      << "\"decision\":\"" << (value.hard_block ? "HARD_CORRIDOR_BLOCK" :
          value.total > 0.0 ? "SOFT_PENALIZED" : "ACCEPT") << "\","
      << "\"reason_code\":\"" << value.reason << "\","
      << "\"static_penalty\":" << value.static_penalty << ","
      << "\"same_direction_penalty\":" << value.same_penalty << ","
      << "\"opposite_direction_penalty\":" << value.opposite_penalty << ","
      << "\"corridor_occupancy_penalty\":" << value.occupancy_penalty << ","
      << "\"no_escape_penalty\":" << value.no_escape_penalty << ","
      << "\"total_policy_penalty\":" << value.total
      << ",\"raw_schedule_penalty\":" << (value.same_penalty + value.opposite_penalty)
      << ",\"capped_schedule_penalty\":" << value.total
      << ",\"parent_g\":" << value.parent_g
      << ",\"approach_cost\":" << value.approach_cost
      << ",\"rmf_core_alt_cost\":" << value.rmf_core_alt_cost
      << ",\"base_move_cost\":" << value.base_move_cost
      << ",\"rotation_cost\":" << value.rotation_cost
      << ",\"event_cost\":" << value.event_cost
      << ",\"wait_cost\":" << value.wait_cost
      << ",\"final_g\":" << value.final_g
      << ",\"h\":" << value.h << ",\"f\":" << value.f
      << ",\"queried_interval_count\":" << value.queried_interval_count
      << ",\"overlap_check_count\":" << value.overlap_check_count
      << ",\"predicted_timestamp_source\":\"RMF_CORE\""
      << ",\"exact_cost_source\":\"RMF_CORE\""
      << ",\"motion_breakdown_source\":\"POLICY_DERIVED_FROM_RMF_CORE_TRAJECTORY\""
      << ",\"cost_component_note\":\"movement/rotation/wait are diagnostic trajectory timestamp classes; parent_g, approach, event, alt, h and final_g are captured at the real RMF child creation point\""
      << ",\"overlaps\":[";
  for (std::size_t n = 0; n < value.overlaps.size(); ++n)
  {
    if (n) out << ','; const auto& overlap = value.overlaps[n];
    out << "{\"participant_id\":" << overlap.interval.participant
        << ",\"plan_id\":" << overlap.interval.plan
        << ",\"route_id\":" << overlap.interval.route
        << ",\"direction\":\"" << direction_name(overlap.interval.direction)
        << "\",\"state\":\"" << state_name(overlap.interval.state)
        << "\",\"occupancy_enter\":" << overlap.interval.enter
        << ",\"occupancy_exit\":" << overlap.interval.exit
        << ",\"overlap_enter\":" << overlap.enter
        << ",\"overlap_exit\":" << overlap.exit
        << ",\"overlap_duration\":" << overlap.duration
        << ",\"admission_overlap_duration\":" << overlap.admission_duration
        << ",\"relation\":\"" << (overlap.is_opposite ? "OPPOSITE" : "SAME")
        << "\",\"source\":\"" << json_escape(overlap.interval.source) << "\"}";
  }
  out << "]}\n";
}

template<typename TimeType>
inline Decision evaluate(
  const std::size_t initial_lane,
  const std::vector<std::size_t>& traversed_lanes,
  const std::optional<std::size_t> previous_lane,
  const std::size_t current_waypoint,
  const std::size_t target_waypoint,
  const void* parent_node,
  const TimeType& enter_time,
  const TimeType& exit_time,
  const double parent_g,
  const double approach_cost,
  const double rmf_core_alt_cost,
  const double event_cost,
  const MotionBreakdown& breakdown,
  const double h)
{
  const auto& data = snapshot(); Decision result;
  std::set<std::size_t> lanes(traversed_lanes.begin(), traversed_lanes.end());
  lanes.insert(initial_lane);
  for (const auto lane : lanes)
  {
    const auto legacy = data.legacy_lane_penalties.find(lane);
    if (legacy != data.legacy_lane_penalties.end()) result.total_penalty += legacy->second;
  }
  if (data.mode == "baseline") return result;
  const double enter = std::chrono::duration<double>(enter_time.time_since_epoch()).count();
  const double exit = std::chrono::duration<double>(exit_time.time_since_epoch()).count();
  std::map<std::string, Direction> candidate_corridors;
  for (const auto lane : lanes)
  {
    const auto binding = data.lanes.find(lane);
    if (binding != data.lanes.end())
      candidate_corridors.emplace(binding->second.corridor, binding->second.direction);
  }
  std::optional<std::string> previous_corridor;
  if (previous_lane)
  {
    const auto previous = data.lanes.find(*previous_lane);
    if (previous != data.lanes.end()) previous_corridor = previous->second.corridor;
  }
  thread_local std::unordered_map<const void*, std::size_t> parent_ids;
  thread_local std::size_t next_search_id = 1;
  const auto parent_insertion = parent_ids.emplace(parent_node, next_search_id);
  if (parent_insertion.second) ++next_search_id;
  for (const auto& item : candidate_corridors)
  {
    const auto& corridor_id = item.first; const auto direction = item.second;
    const auto configured = data.corridors.find(corridor_id);
    if (configured == data.corridors.end()) continue;
    const auto& corridor = configured->second; CorridorDecision decision;
    decision.candidate_id = next_search_id++;
    decision.parent_search_node_id = parent_insertion.first->second;
    decision.current_waypoint = current_waypoint;
    decision.target_waypoint = target_waypoint;
    for (const auto lane : lanes)
    {
      const auto lane_binding = data.lanes.find(lane);
      if (lane_binding != data.lanes.end()
        && lane_binding->second.corridor == corridor_id)
      {
        decision.lane_ids.push_back(lane);
      }
    }
    decision.corridor = corridor_id; decision.direction = direction;
    decision.enter = enter; decision.exit = exit;
    decision.parent_g = parent_g; decision.approach_cost = approach_cost;
    decision.rmf_core_alt_cost = rmf_core_alt_cost;
    decision.base_move_cost = breakdown.movement_time;
    decision.rotation_cost = breakdown.rotation_time;
    decision.event_cost = event_cost;
    decision.wait_cost = breakdown.waiting_time; decision.h = h;
    decision.final_g = parent_g + approach_cost + event_cost
      + rmf_core_alt_cost;
    decision.f = decision.final_g + h;
    decision.is_entry = !previous_corridor || *previous_corridor != corridor_id;
    bool current_participant_is_occupant = false;
    const auto participant_bucket = data.intervals.find(corridor_id);
    if (participant_bucket != data.intervals.end())
    {
      current_participant_is_occupant = std::any_of(
        participant_bucket->second.begin(), participant_bucket->second.end(),
        [&](const Interval& interval)
        {
          return interval.participant == current_participant(data)
            && (interval.state == State::Occupied
              || interval.state == State::UnknownHold);
        });
    }
    // A participant already inside the physical corridor must be able to
    // continue toward an exit. This also covers a replan whose Start has no
    // approach_lane even though the Schedule snapshot still shows occupancy.
    if (current_participant_is_occupant)
      decision.is_entry = false;
    if (!decision.is_entry)
    {
      decision.reason = current_participant_is_occupant
        ? "EXISTING_OCCUPANT_EXIT_ALWAYS_ALLOWED"
        : "CORRIDOR_CONTINUATION_NOT_DOUBLE_CHARGED";
      trace(decision, data); result.corridors.push_back(decision); continue;
    }
    if (data.mode == "schedule_soft")
    {
      const auto bucket = data.intervals.find(corridor_id);
      double raw_schedule_penalty = 0.0;
      if (bucket != data.intervals.end())
      {
        decision.queried_interval_count = bucket->second.size();
        for (const auto& interval : bucket->second)
        {
          ++decision.overlap_check_count;
          // SCHEDULE_SOFT intentionally ignores every derived/free-flow interval
          // and the planning participant's own committed itinerary.
          if (interval.source != "SCHEDULE") continue;
          if (interval.participant == current_participant(data)) continue;
          const double overlap_enter = std::max(enter, interval.enter);
          const double overlap_exit = std::min(exit, interval.exit);
          const double actual_overlap = std::max(0.0, overlap_exit - overlap_enter);
          if (actual_overlap <= 0.0) continue;
          const bool is_opposite = opposite(direction, interval.direction);
          const double direction_weight = is_opposite
            ? data.weights.schedule_soft_opposite_weight
            : data.weights.schedule_soft_same_weight;
          const double component = data.weights.schedule_soft_lambda
            * actual_overlap * direction_weight;
          if (is_opposite)
            decision.opposite_penalty += component;
          else
            decision.same_penalty += component;
          raw_schedule_penalty += component;
          decision.overlaps.push_back(Overlap{
            interval, overlap_enter, overlap_exit, actual_overlap,
            actual_overlap, is_opposite});
        }
      }
      decision.total = std::min(
        raw_schedule_penalty, data.weights.schedule_soft_max_penalty);
      decision.reason = decision.overlaps.empty()
        ? "NO_SCHEDULE_SPACETIME_OVERLAP"
        : (decision.total + 1e-12 < raw_schedule_penalty
          ? "SCHEDULE_SOFT_COST_CAPPED"
          : "SCHEDULE_SOFT_SPACETIME_COST");
      decision.final_g = parent_g + approach_cost + event_cost
        + rmf_core_alt_cost + decision.total;
      decision.f = decision.final_g + h;
      result.total_penalty += decision.total;
      trace(decision, data); result.corridors.push_back(std::move(decision));
      continue;
    }

    decision.static_penalty = data.weights.static_penalty + corridor.base_penalty;
    bool has_opposite_owner = false;
    bool has_opposite_overlap = false;
    const auto bucket = data.intervals.find(corridor_id);
    if (bucket != data.intervals.end())
    {
      for (const auto& interval : bucket->second)
      {
        if (interval.participant == current_participant(data)) continue;
        const double overlap_enter = std::max(enter, interval.enter);
        const double overlap_exit = std::min(exit, interval.exit);
        const double actual_overlap = std::max(0.0, overlap_exit - overlap_enter);
        const double admission_overlap = std::max(
          0.0, std::min(exit + data.weights.overlap_margin,
            interval.exit + data.weights.overlap_margin)
          - std::max(enter - data.weights.overlap_margin,
            interval.enter - data.weights.overlap_margin));
        if (admission_overlap <= 0.0) continue;
        const bool is_opposite = opposite(direction, interval.direction);
        has_opposite_overlap = has_opposite_overlap || is_opposite;
        decision.overlaps.push_back(Overlap{
          interval, overlap_enter, overlap_exit, actual_overlap,
          admission_overlap, is_opposite});
        if (is_opposite)
          decision.opposite_penalty += actual_overlap * data.weights.opposite_per_second;
        else
          decision.same_penalty += actual_overlap * data.weights.same_per_second;
        if (interval.state == State::Occupied || interval.state == State::UnknownHold)
          decision.occupancy_penalty += actual_overlap * data.weights.occupied_per_second;
        else
          decision.occupancy_penalty += actual_overlap * data.weights.future_per_second;
        if (is_opposite && (interval.owner || interval.state == State::Occupied
          || interval.state == State::UnknownHold)) has_opposite_owner = true;
      }
    }
    const bool hard_mode = data.mode == "hybrid" || data.mode == "hybrid_nego";
    if (has_opposite_overlap && !corridor.passing_allowed
      && corridor.capacity <= 1)
    {
      decision.no_escape_penalty = data.weights.no_escape;
    }
    decision.hard_block = hard_mode && corridor.hard_opposite_block
      && !corridor.passing_allowed && has_opposite_owner;
    decision.reason = decision.hard_block ? "OPPOSITE_DIRECTION_CORRIDOR_BLOCK"
      : decision.overlaps.empty() ? "NO_SCHEDULE_OVERLAP" : "SCHEDULE_AWARE_CORRIDOR_COST";
    decision.total = decision.static_penalty + decision.same_penalty
      + decision.opposite_penalty + decision.occupancy_penalty + decision.no_escape_penalty;
    decision.final_g = parent_g + approach_cost + event_cost
      + rmf_core_alt_cost + decision.total;
    decision.f = decision.final_g + h;
    result.hard_block = result.hard_block || decision.hard_block;
    result.total_penalty += decision.total;
    trace(decision, data); result.corridors.push_back(std::move(decision));
  }
  return result;
}

} } } } // rmf_traffic::agv::planning::rmf_lab_policy
#endif
'''.strip() + "\n"


def _planner_file(source: Path) -> Path:
    for relative in PLANNER_RELATIVE_PATHS:
        candidate = source / relative
        if candidate.is_file():
            return candidate
    expected = ", ".join(str(source / path) for path in PLANNER_RELATIVE_PATHS)
    raise FileNotFoundError(
        "DifferentialDrivePlanner.cpp를 찾지 못했습니다. 확인 경로: " + expected)


def _negotiator_file(source: Path) -> Path:
    for relative in NEGOTIATOR_RELATIVE_PATHS:
        candidate = source / relative
        if candidate.is_file():
            return candidate
    expected = ", ".join(str(source / path) for path in NEGOTIATOR_RELATIVE_PATHS)
    raise FileNotFoundError(
        "SimpleNegotiator.cpp를 찾지 못했습니다. 확인 경로: " + expected)


def _restore_legacy_patch(text: str) -> str:
    for begin_marker, end_marker in (
        ("// RMF_TRAFFIC_LAB_OCCUPANCY_PENALTY_V2_BEGIN",
         "// RMF_TRAFFIC_LAB_OCCUPANCY_PENALTY_V2_END"),
        ("// RMF_TRAFFIC_LAB_LANE_PENALTY_V1_BEGIN",
         "// RMF_TRAFFIC_LAB_LANE_PENALTY_V1_END"),
    ):
        begin = text.find(begin_marker)
        if begin >= 0:
            end = text.find(end_marker, begin)
            if end < 0:
                raise RuntimeError(f"legacy patch end marker not found: {end_marker}")
            text = text[:begin] + text[end + len(end_marker):]
    text = text.replace(
        "            + entry_event_cost + alt->cost + exit_event_cost\n"
        "            + rmf_lab_detour_penalty,",
        "            + entry_event_cost + alt->cost + exit_event_cost,",
    )
    text = text.replace(
        "          node->current_cost + entry_event_cost + alt->cost\n"
        "          + rmf_lab_detour_penalty,",
        "          node->current_cost + entry_event_cost + alt->cost,",
    )
    text = text.replace(
        "    if (!_validator && !rmf_lab_lane_penalty_enabled())\n"
        "    {\n      // If we don't have a validator",
        "    if (!_validator)\n    {\n      // If we don't have a validator",
    )
    return text.replace(
        "    const double rmf_lab_detour_penalty = rmf_lab_lane_penalty(\n"
        "      traversal.initial_lane_index, traversal.traversed_lanes);\n\n", "")


def after_core_patch_status(source: Path) -> tuple[bool, Path | None]:
    try:
        planner = _planner_file(source.expanduser().resolve())
        header = planner.with_name(POLICY_HEADER_NAME)
        negotiator = _negotiator_file(source.expanduser().resolve())
        return (
            PATCH_MARKER in planner.read_text(encoding="utf-8")
            and header.is_file()
            and PATCH_MARKER in header.read_text(encoding="utf-8")
            and PATCH_MARKER in negotiator.read_text(encoding="utf-8"), planner)
    except (OSError, UnicodeError):
        return False, None


def patch_planner(planner: Path) -> bool:
    text = planner.read_text(encoding="utf-8")
    header = planner.with_name(POLICY_HEADER_NAME)
    if PATCH_MARKER in text and header.is_file():
        if header.read_text(encoding="utf-8") != POLICY_HEADER_CODE:
            header.write_text(POLICY_HEADER_CODE, encoding="utf-8")
            return True
        return False
    backup = planner.with_suffix(planner.suffix + ".before_rmf_lab_corridor_policy")
    if OLD_PATCH_MARKER in text:
        if not backup.is_file():
            raise RuntimeError(
                "기존 V3 코어 패치를 복구할 원본 백업이 없습니다: " + str(backup))
        text = backup.read_text(encoding="utf-8")
    text = _restore_legacy_patch(text); original = text
    include_anchor = '#include "a_star.hpp"'
    if include_anchor not in text:
        raise RuntimeError("a_star.hpp include 위치를 찾지 못했습니다")
    text = text.replace(
        include_anchor,
        include_anchor + f'\n#include "{POLICY_HEADER_NAME}"\n// {PATCH_MARKER}', 1)
    route_anchor = (
        "      auto traversal_result = alt->routes(std::nullopt)(ready_time, ready_yaw);\n\n"
        "      bool all_valid = true;")
    route_replacement = (
        "      auto traversal_result = alt->routes(std::nullopt)(ready_time, ready_yaw);\n\n"
        "      bool all_valid = true;")
    if route_anchor not in text:
        raise RuntimeError("traversal_result policy hook 위치를 찾지 못했습니다")
    text = text.replace(route_anchor, route_replacement, 1)
    current_anchor = "          node->current_cost + entry_event_cost + alt->cost,"
    if current_anchor not in text:
        raise RuntimeError("A* child g-cost 위치를 찾지 못했습니다")
    policy_hook = (
        "      const auto rmf_lab_previous_lane = node->approach_lanes.empty()\n"
        "        ? std::optional<std::size_t>{}\n"
        "        : std::make_optional(node->approach_lanes.back());\n"
        "      const auto rmf_lab_policy_decision = rmf_lab_policy::evaluate(\n"
        "        traversal.initial_lane_index, traversal.traversed_lanes,\n"
        "        rmf_lab_previous_lane, initial_waypoint_index,\n"
        "        next_waypoint_index, node.get(), ready_time,\n"
        "        traversal_result.finish_time, node->current_cost,\n"
        "        0.0, alt->cost, entry_event_cost,\n"
        "        rmf_lab_policy::analyze_routes(traversal_result.routes),\n"
        "        *remaining_cost_estimate);\n"
        "      if (rmf_lab_policy_decision.hard_block)\n"
        "        continue;\n\n")
    cost_offset = text.index(current_anchor)
    construction_matches = list(re.finditer(
        r"(?m)^      [^\n]+\(\s*$", text[:cost_offset]))
    if not construction_matches:
        raise RuntimeError("A* child SearchNode 생성 시작 위치를 찾지 못했습니다")
    construction_offset = construction_matches[-1].start()
    text = text[:construction_offset] + policy_hook + text[construction_offset:]
    text = text.replace(
        current_anchor,
        "          node->current_cost + entry_event_cost + alt->cost\n"
        "          + rmf_lab_policy_decision.total_penalty,", 1)
    free_anchor = "    if (!_validator)\n    {\n      // If we don't have a validator"
    if free_anchor not in text:
        raise RuntimeError("free-flow shortcut 조건을 찾지 못했습니다")
    text = text.replace(
        free_anchor,
        "    if (!_validator && !rmf_lab_policy::enabled())\n"
        "    {\n      // If we don't have a validator", 1)
    if text == original:
        raise RuntimeError("패치 결과가 원본과 같습니다")
    if not backup.exists(): shutil.copy2(planner, backup)
    planner.write_text(text, encoding="utf-8")
    header.write_text(POLICY_HEADER_CODE, encoding="utf-8")
    return True


def patch_simple_negotiator(negotiator: Path) -> bool:
    text = negotiator.read_text(encoding="utf-8")
    if PATCH_MARKER in text:
        return False
    backup = negotiator.with_suffix(
        negotiator.suffix + ".before_rmf_lab_corridor_policy")
    if OLD_PATCH_MARKER in text:
        if not backup.is_file():
            raise RuntimeError(
                "기존 V3 negotiator 패치를 복구할 원본 백업이 없습니다: "
                + str(backup))
        text = backup.read_text(encoding="utf-8")
    include_anchor = "#include <rmf_traffic/agv/SimpleNegotiator.hpp>"
    if include_anchor not in text:
        raise RuntimeError("SimpleNegotiator include 위치를 찾지 못했습니다")
    text = text.replace(
        include_anchor,
        include_anchor
        + f'\n#include "planning/{POLICY_HEADER_NAME}"\n// {PATCH_MARKER}',
        1,
    )
    respond_anchor = (
        "void SimpleNegotiator::respond(\n"
        "  const schedule::Negotiation::Table::ViewerPtr& table_viewer,\n"
        "  const ResponderPtr& responder)\n"
        "{\n")
    if respond_anchor not in text:
        raise RuntimeError("SimpleNegotiator::respond 위치를 찾지 못했습니다")
    text = text.replace(
        respond_anchor,
        respond_anchor
        + "  const planning::rmf_lab_policy::ParticipantScope\n"
        + "    rmf_lab_participant_scope(\n"
        + "      table_viewer->sequence().back().participant);\n",
        1,
    )
    if not backup.exists():
        shutil.copy2(negotiator, backup)
    negotiator.write_text(text, encoding="utf-8")
    return True


def prepare_after_core(before_source: Path, after_workspace: Path) -> dict:
    before_source = before_source.expanduser().resolve()
    after_workspace = after_workspace.expanduser().resolve()
    after_source = after_workspace / "src" / "rmf_traffic"
    copied = False
    if not after_source.exists():
        if not before_source.is_dir():
            raise FileNotFoundError(f"Before rmf_traffic source가 없습니다: {before_source}")
        after_source.parent.mkdir(parents=True, exist_ok=True)
        shutil.copytree(before_source, after_source, symlinks=True); copied = True
    planner = _planner_file(after_source)
    negotiator = _negotiator_file(after_source)
    planner_patched = patch_planner(planner)
    negotiator_patched = patch_simple_negotiator(negotiator)
    metadata = {
        "patch": PATCH_MARKER, "before_source": str(before_source),
        "after_workspace": str(after_workspace), "after_source": str(after_source),
        "planner": str(planner), "negotiator": str(negotiator),
        "policy_header": str(planner.with_name(POLICY_HEADER_NAME)),
        "copied": copied,
        "patched": planner_patched or negotiator_patched,
        "planner_patched": planner_patched,
        "negotiator_patched": negotiator_patched,
        "environment": {
            "snapshot": "RMF_TRAFFIC_LAB_POLICY_SNAPSHOT",
            "generation": "RMF_TRAFFIC_LAB_POLICY_GENERATION",
            "trace": "RMF_TRAFFIC_LAB_POLICY_TRACE"},
        "abi": "internal header only; no public header or symbol change",
    }
    (after_workspace / ".rmf_traffic_lab_after.json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return metadata


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Copy rmf_traffic and patch schedule-aware corridor A* g-cost")
    parser.add_argument("--before-source", type=Path, default=Path("~/rmf_ws/src/rmf_traffic"))
    parser.add_argument("--after-workspace", type=Path, default=Path("~/rmf_ws_modified"))
    args = parser.parse_args()
    print(json.dumps(prepare_after_core(args.before_source, args.after_workspace),
                     ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
