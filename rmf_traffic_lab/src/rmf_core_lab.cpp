#include <rmf_traffic/Time.hpp>
#include <rmf_traffic/DetectConflict.hpp>
#include <rmf_traffic/agv/CentralizedNegotiation.hpp>
#include <rmf_traffic/agv/LaneClosure.hpp>
#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/agv/RouteValidator.hpp>
#include <rmf_traffic/agv/VehicleTraits.hpp>
#include <rmf_traffic/agv/debug/debug_Planner.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/schedule/Database.hpp>
#include <rmf_traffic/schedule/Participant.hpp>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <queue>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

using Graph = rmf_traffic::agv::Graph;
using Plan = rmf_traffic::agv::Plan;
using Planner = rmf_traffic::agv::Planner;
using Start = Plan::Start;
using Goal = Plan::Goal;

constexpr double Pi = 3.14159265358979323846;
constexpr double RobotRadius = 0.30;
constexpr double LinearVelocity = 0.70;
constexpr double LinearAcceleration = 0.75;
constexpr double AngularVelocity = 0.60;
constexpr double AngularAcceleration = 2.00;
constexpr std::size_t SaturationLimit = 10000;
constexpr std::size_t TraceStepLimit = 5000;
const std::string MapName = "L1";

struct NodeDef
{
  std::size_t id;
  std::string name;
  double x;
  double y;
};

struct LaneDef
{
  std::size_t id;
  std::size_t entry;
  std::size_t exit;
  std::string mutex_group;
};

struct LabGraph
{
  Graph graph;
  std::vector<NodeDef> nodes;
  std::vector<LaneDef> lanes;
  std::string map_name = MapName;
};

struct RobotRequest
{
  std::string name;
  std::size_t start;
  std::size_t goal;
  double yaw;
  double start_time_s = 0.0;
  double insertion_time_s = 0.0;
};

struct CorridorDef
{
  std::string id;
  std::size_t capacity = 1;
  bool passing_allowed = false;
  bool hard_opposite_direction_block = true;
  std::optional<std::size_t> holding_entry_a;
  std::optional<std::size_t> holding_entry_b;
  double base_penalty = 0.0;
  std::vector<std::size_t> lanes_forward;
  std::vector<std::size_t> lanes_reverse;
};

struct RuntimeEvent
{
  std::string type;
  std::string robot;
  double at_s = 0.0;
  double value_s = 0.0;
  bool flag = false;
  std::string detail;
};

std::vector<CorridorDef> ActiveCorridors;
std::vector<RuntimeEvent> ActiveRuntimeEvents;
std::map<rmf_traffic::schedule::ParticipantId, double> ActiveCumulativeDelay;

rmf_traffic::Time request_start_time(const RobotRequest& request)
{
  return rmf_traffic::Time(
    std::chrono::duration_cast<rmf_traffic::Duration>(
      std::chrono::duration<double>(
        std::max(request.start_time_s, request.insertion_time_s))));
}

std::string json_string(const std::string& value)
{
  std::ostringstream out;
  out << '"';
  for (const unsigned char c : value)
  {
    switch (c)
    {
      case '"': out << "\\\""; break;
      case '\\': out << "\\\\"; break;
      case '\b': out << "\\b"; break;
      case '\f': out << "\\f"; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default:
        if (c < 0x20)
        {
          out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
              << static_cast<int>(c) << std::dec;
        }
        else
        {
          out << static_cast<char>(c);
        }
    }
  }
  out << '"';
  return out.str();
}

template<typename T>
std::string json_number_array(const std::vector<T>& values)
{
  std::ostringstream out;
  out << '[';
  for (std::size_t i = 0; i < values.size(); ++i)
  {
    if (i > 0)
      out << ',';
    out << values[i];
  }
  out << ']';
  return out.str();
}

std::string json_string_array(const std::vector<std::string>& values)
{
  std::ostringstream out;
  out << '[';
  for (std::size_t i = 0; i < values.size(); ++i)
  {
    if (i > 0)
      out << ',';
    out << json_string(values[i]);
  }
  out << ']';
  return out.str();
}

std::string json_optional_number(const std::optional<double>& value)
{
  if (!value.has_value())
    return "null";

  std::ostringstream out;
  out << std::setprecision(12) << *value;
  return out.str();
}

class JsonlWriter
{
public:
  explicit JsonlWriter(const std::string& path)
  : _stream(path)
  {
    if (!_stream)
      throw std::runtime_error("Could not open output file: " + path);
  }

  void write(const std::string& object)
  {
    if (object.size() < 2 || object.front() != '{')
      throw std::runtime_error("JSONL event must be a JSON object");

    _stream << "{\"seq\":" << _sequence++ << ',' << object.substr(1) << '\n';
    _stream.flush();
  }

private:
  std::ofstream _stream;
  std::size_t _sequence = 0;
};

std::size_t add_node(
  LabGraph& result,
  std::string name,
  const double x,
  const double y,
  const bool holding = false,
  const bool parking = false,
  const bool passthrough = false,
  const std::string& mutex_group = "")
{
  const std::size_t id = result.nodes.size();
  auto& waypoint = result.graph.add_waypoint(result.map_name, {x, y});
  waypoint.set_holding_point(holding)
    .set_parking_spot(parking)
    .set_passthrough_point(passthrough)
    .set_in_mutex_group(mutex_group);
  result.graph.add_key(name, id);
  result.nodes.push_back(NodeDef{id, std::move(name), x, y});
  return id;
}

std::size_t add_directed_lane(
  LabGraph& result,
  const std::size_t entry,
  const std::size_t exit,
  const std::optional<double> speed_limit = std::nullopt,
  const std::string& mutex_group = "")
{
  const std::size_t id = result.lanes.size();
  auto& lane = result.graph.add_lane(entry, exit);
  lane.properties().speed_limit(speed_limit).set_in_mutex_group(mutex_group);
  result.lanes.push_back(LaneDef{id, entry, exit, mutex_group});
  return id;
}

void add_bidirectional_lane(
  LabGraph& result,
  const std::size_t a,
  const std::size_t b,
  const std::optional<double> speed_limit = std::nullopt,
  const std::string& mutex_group = "")
{
  add_directed_lane(result, a, b, speed_limit, mutex_group);
  add_directed_lane(result, b, a, speed_limit, mutex_group);
}

LabGraph make_single_path_graph(const bool speed_choice = false)
{
  //                   5 -------- 6
  //                 detour       detour
  // START 0 -- 1 -- 2 -- 3 -- 4 GOAL
  // The lower route is geometrically shorter. In speed_choice it is slower.
  LabGraph result;
  add_node(result, "START", -4.0, 0.0, true, true);        // 0
  add_node(result, "LEFT_GATE", -2.0, 0.0, true);         // 1
  add_node(result, "CENTER", 0.0, 0.0, false, false, true);// 2
  add_node(result, "RIGHT_GATE", 2.0, 0.0, true);         // 3
  add_node(result, "GOAL", 4.0, 0.0, true, true);         // 4
  add_node(result, "DETOUR_LEFT", -2.0, 2.5, true);       // 5
  add_node(result, "DETOUR_RIGHT", 2.0, 2.5, true);       // 6

  const auto center_speed = speed_choice
    ? std::optional<double>(0.22)
    : std::nullopt;
  const auto detour_speed = speed_choice
    ? std::optional<double>(0.70)
    : std::nullopt;

  add_bidirectional_lane(result, 0, 1);                    // 0,1
  add_bidirectional_lane(result, 1, 2, center_speed);      // 2,3
  add_bidirectional_lane(result, 2, 3, center_speed);      // 4,5
  add_bidirectional_lane(result, 3, 4);                    // 6,7
  add_bidirectional_lane(result, 1, 5, detour_speed);      // 8,9
  add_bidirectional_lane(result, 5, 6, detour_speed);      // 10,11
  add_bidirectional_lane(result, 6, 3, detour_speed);      // 12,13
  return result;
}

LabGraph make_head_on_graph()
{
  LabGraph result;
  add_node(result, "LEFT", -4.0, 0.0, true, true);
  add_node(result, "C1", -2.0, 0.0, false, false, true);
  add_node(result, "C2", 0.0, 0.0, false, false, true);
  add_node(result, "C3", 2.0, 0.0, false, false, true);
  add_node(result, "RIGHT", 4.0, 0.0, true, true);
  add_bidirectional_lane(result, 0, 1, std::nullopt, "corridor");
  add_bidirectional_lane(result, 1, 2, std::nullopt, "corridor");
  add_bidirectional_lane(result, 2, 3, std::nullopt, "corridor");
  add_bidirectional_lane(result, 3, 4, std::nullopt, "corridor");
  return result;
}

LabGraph make_single_lane_bidirectional_graph()
{
  // Two separated staging spots exist on each side of one physical corridor.
  // The robots must use the same centerline in opposite directions, but they
  // do not need to exchange the exact same endpoint. This makes sequential
  // passage physically possible: one waits outside while the other exits.
  //
  // W_START 0 --\                         /-- 8 E_GOAL
  //              2 -- 3 -- 4 -- 5 -- 6
  // W_GOAL  1 --/                         \-- 7 E_START
  LabGraph result;
  add_node(result, "W_START", -6.0, 1.6, true, true);   // 0
  add_node(result, "W_GOAL", -6.0, -1.6, true, true);  // 1
  add_node(result, "W_GATE", -4.0, 0.0, false, false, true); // 2
  add_node(result, "CORRIDOR_W", -2.0, 0.0, false, false, true); // 3
  add_node(result, "CORRIDOR_C", 0.0, 0.0, false, false, true);  // 4
  add_node(result, "CORRIDOR_E", 2.0, 0.0, false, false, true);  // 5
  add_node(result, "E_GATE", 4.0, 0.0, false, false, true); // 6
  add_node(result, "E_START", 6.0, -1.6, true, true);  // 7
  add_node(result, "E_GOAL", 6.0, 1.6, true, true);    // 8

  add_bidirectional_lane(result, 0, 2); // west staging approach
  add_bidirectional_lane(result, 1, 2);
  add_bidirectional_lane(result, 2, 3, std::nullopt, "one_lane_corridor");
  add_bidirectional_lane(result, 3, 4, std::nullopt, "one_lane_corridor");
  add_bidirectional_lane(result, 4, 5, std::nullopt, "one_lane_corridor");
  add_bidirectional_lane(result, 5, 6, std::nullopt, "one_lane_corridor");
  add_bidirectional_lane(result, 6, 7); // east staging approach
  add_bidirectional_lane(result, 6, 8);
  return result;
}

LabGraph make_t_junction_graph()
{
  LabGraph result;
  add_node(result, "WEST", -4.0, 0.0, true, true);  // 0
  add_node(result, "CENTER", 0.0, 0.0, true);       // 1
  add_node(result, "EAST", 4.0, 0.0, true, true);   // 2
  add_node(result, "NORTH", 0.0, 4.0, true, true);  // 3
  add_node(result, "WAIT_W", -2.0, 0.0, true);      // 4
  add_node(result, "WAIT_E", 2.0, 0.0, true);       // 5
  add_node(result, "WAIT_N", 0.0, 2.0, true);       // 6
  add_bidirectional_lane(result, 0, 4);
  add_bidirectional_lane(result, 4, 1, std::nullopt, "junction");
  add_bidirectional_lane(result, 1, 5, std::nullopt, "junction");
  add_bidirectional_lane(result, 5, 2);
  add_bidirectional_lane(result, 1, 6, std::nullopt, "junction");
  add_bidirectional_lane(result, 6, 3);
  return result;
}

LabGraph make_cross_graph()
{
  LabGraph result;
  add_node(result, "WEST", -4.0, 0.0, true, true);   // 0
  add_node(result, "CENTER", 0.0, 0.0, true);        // 1
  add_node(result, "EAST", 4.0, 0.0, true, true);    // 2
  add_node(result, "NORTH", 0.0, 4.0, true, true);   // 3
  add_node(result, "SOUTH", 0.0, -4.0, true, true);  // 4
  add_node(result, "W_GATE", -2.0, 0.0, true);       // 5
  add_node(result, "E_GATE", 2.0, 0.0, true);        // 6
  add_node(result, "N_GATE", 0.0, 2.0, true);        // 7
  add_node(result, "S_GATE", 0.0, -2.0, true);       // 8
  add_bidirectional_lane(result, 0, 5);
  add_bidirectional_lane(result, 5, 1, std::nullopt, "intersection");
  add_bidirectional_lane(result, 1, 6, std::nullopt, "intersection");
  add_bidirectional_lane(result, 6, 2);
  add_bidirectional_lane(result, 3, 7);
  add_bidirectional_lane(result, 7, 1, std::nullopt, "intersection");
  add_bidirectional_lane(result, 1, 8, std::nullopt, "intersection");
  add_bidirectional_lane(result, 8, 4);
  return result;
}

LabGraph make_passing_bay_graph()
{
  // LEFT--A--B--C--RIGHT is the narrow main corridor. A-BAY-C provides
  // enough graph topology for one robot to yield while the other passes.
  LabGraph result;
  add_node(result, "LEFT", -5.0, 0.0, true, true);  // 0
  add_node(result, "A", -2.5, 0.0, true);           // 1
  add_node(result, "B", 0.0, 0.0, true);            // 2
  add_node(result, "C", 2.5, 0.0, true);            // 3
  add_node(result, "RIGHT", 5.0, 0.0, true, true);  // 4
  add_node(result, "BAY", 0.0, 2.2, true, true);    // 5
  add_bidirectional_lane(result, 0, 1);
  add_bidirectional_lane(result, 1, 2, std::nullopt, "bottleneck");
  add_bidirectional_lane(result, 2, 3, std::nullopt, "bottleneck");
  add_bidirectional_lane(result, 3, 4);
  add_bidirectional_lane(result, 1, 5);
  add_bidirectional_lane(result, 5, 3);
  return result;
}

LabGraph make_disconnected_graph()
{
  LabGraph result;
  add_node(result, "START", -3.0, 0.0, true, true);
  add_node(result, "ISLAND_A", -1.0, 0.0, true);
  add_node(result, "ISLAND_B", 1.0, 0.0, true);
  add_node(result, "GOAL", 3.0, 0.0, true, true);
  add_bidirectional_lane(result, 0, 1);
  add_bidirectional_lane(result, 2, 3);
  return result;
}

struct CustomScenario
{
  std::string name = "custom";
  std::string description = "User-defined RMF Traffic scenario";
  std::string source_json;
  std::string mode = "auto";
  bool dynamic_insertion = false;
  LabGraph graph;
  std::vector<RobotRequest> robots;
  std::vector<std::size_t> closed_lanes;
  std::vector<CorridorDef> corridors;
  std::vector<RuntimeEvent> runtime_events;
  std::vector<std::string> validation_warnings;
};

std::vector<std::string> split_tab(const std::string& line)
{
  std::vector<std::string> fields;
  std::size_t begin = 0;
  while (true)
  {
    const auto end = line.find('\t', begin);
    fields.push_back(line.substr(begin, end - begin));
    if (end == std::string::npos)
      break;
    begin = end + 1;
  }
  return fields;
}

bool parse_compiled_bool(const std::string& value)
{
  if (value == "1" || value == "true")
    return true;
  if (value == "0" || value == "false")
    return false;
  throw std::runtime_error("Invalid compiled boolean: " + value);
}

CustomScenario load_custom_scenario(const std::string& path)
{
  std::ifstream input(path);
  if (!input)
    throw std::runtime_error("Could not open compiled custom scenario: " + path);

  CustomScenario result;
  std::string line;
  std::size_t line_number = 0;
  bool format_seen = false;
  while (std::getline(input, line))
  {
    ++line_number;
    if (line.empty())
      continue;
    const auto fields = split_tab(line);
    const auto require = [&](const std::size_t count)
    {
      if (fields.size() != count)
      {
        throw std::runtime_error(
          "Invalid custom scenario record at line "
          + std::to_string(line_number));
      }
    };

    if (fields[0] == "FORMAT")
    {
      require(2);
      if (fields[1] != "rmf_custom_v1")
        throw std::runtime_error("Unsupported custom scenario format: " + fields[1]);
      format_seen = true;
    }
    else if (fields[0] == "META")
    {
      require(3);
      result.name = fields[1];
      result.description = fields[2];
    }
    else if (fields[0] == "MAP")
    {
      require(2);
      if (!result.graph.nodes.empty())
        throw std::runtime_error("MAP must appear before NODE records");
      result.graph.map_name = fields[1];
    }
    else if (fields[0] == "SOURCE_JSON")
    {
      require(2);
      result.source_json = fields[1];
    }
    else if (fields[0] == "MODE")
    {
      require(2);
      result.mode = fields[1];
    }
    else if (fields[0] == "DYNAMIC")
    {
      require(2);
      result.dynamic_insertion = parse_compiled_bool(fields[1]);
    }
    else if (fields[0] == "NODE")
    {
      require(8);
      add_node(
        result.graph,
        fields[1],
        std::stod(fields[2]),
        std::stod(fields[3]),
        parse_compiled_bool(fields[4]),
        parse_compiled_bool(fields[5]),
        parse_compiled_bool(fields[6]),
        fields[7]);
    }
    else if (fields[0] == "LANE")
    {
      require(6);
      const auto entry = static_cast<std::size_t>(std::stoull(fields[1]));
      const auto exit = static_cast<std::size_t>(std::stoull(fields[2]));
      if (entry >= result.graph.nodes.size() || exit >= result.graph.nodes.size())
        throw std::runtime_error("Custom lane endpoint is outside the node array");
      const auto speed = fields[3] == "-"
        ? std::optional<double>()
        : std::optional<double>(std::stod(fields[3]));
      const auto lane_id = add_directed_lane(
        result.graph, entry, exit, speed, fields[4]);
      if (parse_compiled_bool(fields[5]))
        result.closed_lanes.push_back(lane_id);
    }
    else if (fields[0] == "ROBOT")
    {
      if (fields.size() != 5 && fields.size() != 6 && fields.size() != 7)
      {
        throw std::runtime_error(
          "ROBOT record needs name, start, goal, yaw and optional start/insertion times at line "
          + std::to_string(line_number));
      }
      const double start_time_s = fields.size() >= 6
        ? std::stod(fields[5])
        : 0.0;
      if (!std::isfinite(start_time_s) || start_time_s < 0.0)
        throw std::runtime_error("Robot start_time_s must be finite and non-negative");
      const double insertion_time_s = fields.size() == 7
        ? std::stod(fields[6])
        : 0.0;
      if (!std::isfinite(insertion_time_s) || insertion_time_s < 0.0)
        throw std::runtime_error("Robot insertion_time_s must be finite and non-negative");
      result.robots.push_back({
        fields[1],
        static_cast<std::size_t>(std::stoull(fields[2])),
        static_cast<std::size_t>(std::stoull(fields[3])),
        std::stod(fields[4]),
        start_time_s,
        insertion_time_s});
    }
    else if (fields[0] == "CORRIDOR")
    {
      require(8);
      CorridorDef corridor;
      corridor.id = fields[1];
      corridor.capacity = static_cast<std::size_t>(std::stoull(fields[2]));
      corridor.passing_allowed = parse_compiled_bool(fields[3]);
      corridor.hard_opposite_direction_block = parse_compiled_bool(fields[4]);
      if (fields[5] != "-")
        corridor.holding_entry_a = static_cast<std::size_t>(std::stoull(fields[5]));
      if (fields[6] != "-")
        corridor.holding_entry_b = static_cast<std::size_t>(std::stoull(fields[6]));
      corridor.base_penalty = std::stod(fields[7]);
      result.corridors.push_back(std::move(corridor));
    }
    else if (fields[0] == "CORRIDOR_LANE")
    {
      require(4);
      const auto corridor = std::find_if(
        result.corridors.begin(), result.corridors.end(),
        [&](const CorridorDef& item) { return item.id == fields[1]; });
      if (corridor == result.corridors.end())
        throw std::runtime_error("CORRIDOR_LANE refers to unknown corridor");
      const auto lane = static_cast<std::size_t>(std::stoull(fields[3]));
      if (lane >= result.graph.lanes.size())
        throw std::runtime_error("CORRIDOR_LANE refers to unknown directed lane");
      if (fields[2] == "A_TO_B")
        corridor->lanes_forward.push_back(lane);
      else if (fields[2] == "B_TO_A")
        corridor->lanes_reverse.push_back(lane);
      else
        throw std::runtime_error("CORRIDOR_LANE direction is invalid");
    }
    else if (fields[0] == "DELAY")
    {
      require(6);
      result.runtime_events.push_back(RuntimeEvent{
        fields[0], fields[1], std::stod(fields[2]), std::stod(fields[3]),
        parse_compiled_bool(fields[4]), fields[5]});
    }
    else if (fields[0] == "COMM_LOSS")
    {
      require(5);
      result.runtime_events.push_back(RuntimeEvent{
        fields[0], fields[1], std::stod(fields[2]), std::stod(fields[3]),
        parse_compiled_bool(fields[4]), "hold_ownership_when_unknown"});
    }
    else if (fields[0] == "CHECKPOINT_RELEASE")
    {
      require(5);
      result.runtime_events.push_back(RuntimeEvent{
        fields[0], fields[1], std::stod(fields[2]), 0.0,
        parse_compiled_bool(fields[4]), fields[3]});
    }
    else if (fields[0] == "WARNING")
    {
      require(2);
      result.validation_warnings.push_back(fields[1]);
    }
    else
    {
      throw std::runtime_error(
        "Unknown custom scenario record at line "
        + std::to_string(line_number) + ": " + fields[0]);
    }
  }

  if (!format_seen)
    throw std::runtime_error("Custom scenario FORMAT record is missing");
  if (result.graph.nodes.empty() || result.graph.lanes.empty())
    throw std::runtime_error("Custom scenario needs at least one node and lane");
  if (result.robots.empty())
    throw std::runtime_error("Custom scenario needs at least one robot");
  for (const auto& robot : result.robots)
  {
    if (robot.start >= result.graph.nodes.size()
      || robot.goal >= result.graph.nodes.size())
    {
      throw std::runtime_error("Custom robot start/goal is outside the node array");
    }
  }
  if (result.mode == "auto")
    result.mode = result.robots.size() == 1 ? "free_flow" : "negotiation";
  if (result.mode != "free_flow" && result.mode != "negotiation")
    throw std::runtime_error("Custom scenario mode must be auto, free_flow or negotiation");
  if (result.mode == "free_flow" && result.robots.size() != 1)
    throw std::runtime_error("free_flow custom mode requires exactly one robot");
  if (result.mode == "negotiation" && result.robots.size() < 2)
    throw std::runtime_error("negotiation custom mode requires at least two robots");
  return result;
}

rmf_traffic::Profile make_profile()
{
  return rmf_traffic::Profile{
    rmf_traffic::geometry::make_final_convex<rmf_traffic::geometry::Circle>(
      RobotRadius)};
}

rmf_traffic::agv::VehicleTraits make_traits(
  const rmf_traffic::Profile& profile)
{
  return rmf_traffic::agv::VehicleTraits{
    rmf_traffic::agv::VehicleTraits::Limits(
      LinearVelocity, LinearAcceleration),
    rmf_traffic::agv::VehicleTraits::Limits(
      AngularVelocity, AngularAcceleration),
    profile,
    rmf_traffic::agv::VehicleTraits::Differential(
      Eigen::Vector2d::UnitX(), false)};
}

Planner::Options make_planner_options()
{
  Planner::Options options{nullptr};
  options.saturation_limit(SaturationLimit);
  return options;
}

bool experimental_lane_penalty_active()
{
  const char* direct = std::getenv("RMF_TRAFFIC_LAB_LANE_PENALTIES");
  const char* occupancy = std::getenv("RMF_TRAFFIC_LAB_LANE_OCCUPANCY");
  const char* mode = std::getenv("RMF_TRAFFIC_LAB_PENALTY_MODE");
  return (direct != nullptr && direct[0] != '\0')
    || (occupancy != nullptr && occupancy[0] != '\0')
    || (mode != nullptr && std::string(mode) == "shared_corridor");
}

double positive_environment_value(
  const char* name, const double fallback)
{
  const char* raw = std::getenv(name);
  if (!raw)
    return fallback;

  try
  {
    const double value = std::stod(raw);
    return std::isfinite(value) && value > 0.0 ? value : fallback;
  }
  catch (const std::exception&)
  {
    return fallback;
  }
}

double nonnegative_environment_value(
  const char* name, const double fallback)
{
  const char* raw = std::getenv(name);
  if (!raw)
    return fallback;
  try
  {
    const double value = std::stod(raw);
    return std::isfinite(value) && value >= 0.0 ? value : fallback;
  }
  catch (const std::exception&)
  {
    return fallback;
  }
}

struct PolicyScheduleInterval
{
  std::string corridor;
  rmf_traffic::schedule::ParticipantId participant = 0;
  std::size_t plan = 0;
  std::size_t route = 0;
  std::string direction;
  double enter_s = 0.0;
  double exit_s = 0.0;
  std::string state = "RESERVED";
  bool owner = false;
  bool responsive = true;
  std::size_t itinerary_version = 0;
  std::string source = "SCHEDULE";
};

std::vector<double> trajectory_times_at_node(
  const rmf_traffic::Trajectory& trajectory,
  const NodeDef& node)
{
  std::vector<double> output;
  for (const auto& point : trajectory)
  {
    const auto position = point.position();
    if (std::hypot(position[0] - node.x, position[1] - node.y) <= 1e-3)
    {
      output.push_back(rmf_traffic::time::to_seconds(
        point.time().time_since_epoch()));
    }
  }
  return output;
}

std::optional<std::pair<double, double>> route_interval_for_lanes(
  const rmf_traffic::Route& route,
  const LabGraph& graph,
  const std::vector<std::size_t>& lane_ids)
{
  std::optional<double> earliest;
  std::optional<double> latest;
  for (const auto lane_id : lane_ids)
  {
    if (lane_id >= graph.lanes.size())
      continue;
    const auto& lane = graph.lanes[lane_id];
    const auto entry_times = trajectory_times_at_node(
      route.trajectory(), graph.nodes.at(lane.entry));
    const auto exit_times = trajectory_times_at_node(
      route.trajectory(), graph.nodes.at(lane.exit));
    for (const double entry : entry_times)
    {
      const auto exit = std::find_if(
        exit_times.begin(), exit_times.end(),
        [entry](const double value) { return value + 1e-9 >= entry; });
      if (exit == exit_times.end())
        continue;
      earliest = earliest.has_value() ? std::min(*earliest, entry) : entry;
      latest = latest.has_value() ? std::max(*latest, *exit) : *exit;
      break;
    }
  }
  if (!earliest.has_value() || !latest.has_value())
    return std::nullopt;
  return std::make_pair(*earliest, *latest);
}

bool robot_unknown_at(const std::string& robot, const double planning_time_s)
{
  for (const auto& event : ActiveRuntimeEvents)
  {
    if (event.type != "COMM_LOSS" || event.robot != robot)
      continue;
    if (event.at_s <= planning_time_s
      && planning_time_s < event.at_s + event.value_s)
    {
      return true;
    }
  }
  return false;
}

bool corridor_release_confirmed(
  const std::string& robot,
  const std::string& corridor,
  const double planning_time_s)
{
  return std::any_of(
    ActiveRuntimeEvents.begin(), ActiveRuntimeEvents.end(),
    [&](const RuntimeEvent& event)
    {
      return event.type == "CHECKPOINT_RELEASE"
        && event.robot == robot
        && event.detail == corridor
        && event.flag
        && event.at_s <= planning_time_s;
    });
}

void configure_policy_snapshot(
  JsonlWriter& writer,
  const LabGraph& graph,
  const std::shared_ptr<rmf_traffic::schedule::Database>& database,
  const std::size_t participant_id,
  const double planning_time_s,
  const std::string& invocation_reason,
  const std::vector<PolicyScheduleInterval>& admission_reservations = {})
{
  const char* raw_mode = std::getenv("RMF_TRAFFIC_LAB_POLICY_MODE");
  const std::string mode = raw_mode ? raw_mode : "baseline";
  const char* raw_path = std::getenv("RMF_TRAFFIC_LAB_POLICY_SNAPSHOT");
  if (!raw_path || std::string(raw_path).empty())
    return;

  const bool schedule_soft_mode = mode == "schedule_soft";
  std::vector<PolicyScheduleInterval> intervals =
    schedule_soft_mode ? std::vector<PolicyScheduleInterval>{} : admission_reservations;
  const auto schedule_version = database ? database->latest_version() : 0;
  std::size_t queried_route_count = 0;
  std::size_t queried_participant_count = 0;
  std::size_t self_filtered_route_count = 0;
  std::set<rmf_traffic::schedule::ParticipantId> queried_participants;
  if (database)
  {
    // This is one real Viewer query per planning invocation. The resulting
    // corridor index is immutable while that Planner search runs.
    const auto view = database->query(rmf_traffic::schedule::query_all());
    for (const auto& element : view)
    {
      ++queried_route_count;
      queried_participants.insert(element.participant);
      if (schedule_soft_mode && element.participant == participant_id)
      {
        ++self_filtered_route_count;
        continue;
      }
      const auto& route = *element.route;
      for (const auto& corridor : ActiveCorridors)
      {
        const auto forward = route_interval_for_lanes(
          route, graph, corridor.lanes_forward);
        const auto reverse = route_interval_for_lanes(
          route, graph, corridor.lanes_reverse);
        const bool use_forward = forward.has_value()
          && (!reverse.has_value()
          || (forward->second - forward->first)
            >= (reverse->second - reverse->first));
        const auto selected = use_forward ? forward : reverse;
        if (!selected.has_value())
          continue;
        const auto& robot_name = element.description.name();
        if (corridor_release_confirmed(
            robot_name, corridor.id, planning_time_s))
        {
          continue;
        }
        const bool unknown = robot_unknown_at(robot_name, planning_time_s);
        std::string state = "RESERVED";
        double effective_exit = selected->second;
        std::string interval_source = "SCHEDULE";
        if (selected->first <= planning_time_s
          && planning_time_s <= selected->second)
        {
          state = unknown ? "UNKNOWN_HOLD" : "OCCUPIED";
        }
        else if (!schedule_soft_mode && selected->first <= planning_time_s)
        {
          // OLD_SOFT/HYBRID keeps historical ownership until an explicit
          // checkpoint release. SCHEDULE_SOFT intentionally does not extend
          // Schedule intervals beyond the actual registered trajectory.
          state = "UNKNOWN_HOLD";
          effective_exit = planning_time_s + 86400.0;
          interval_source = "POLICY_DERIVED";
        }
        else if (!schedule_soft_mode && unknown)
        {
          state = "UNKNOWN_HOLD";
          effective_exit = std::max(effective_exit, planning_time_s + 86400.0);
          interval_source = "POLICY_DERIVED";
        }
        intervals.push_back(PolicyScheduleInterval{
          corridor.id,
          element.participant,
          element.plan_id,
          element.route_id,
          use_forward ? "A_TO_B" : "B_TO_A",
          selected->first,
          effective_exit,
          state,
          false,
          element.description.responsiveness()
            == rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
          database->itinerary_version(element.participant),
          interval_source});
      }
    }
    queried_participant_count = queried_participants.size();
  }

  // One deterministic owner per corridor prevents symmetric admission block.
  static std::map<std::string, std::string> previous_corridor_state;
  for (const auto& corridor : ActiveCorridors)
  {
    auto winner = intervals.end();
    for (auto it = intervals.begin(); it != intervals.end(); ++it)
    {
      if (it->corridor != corridor.id)
        continue;
      if (winner == intervals.end()
        || std::make_tuple(
          it->state == "OCCUPIED" || it->state == "UNKNOWN_HOLD" ? 0 : 1,
          it->enter_s, it->participant)
        < std::make_tuple(
          winner->state == "OCCUPIED" || winner->state == "UNKNOWN_HOLD" ? 0 : 1,
          winner->enter_s, winner->participant))
      {
        winner = it;
      }
    }
    if (winner != intervals.end())
      winner->owner = true;
  }

  std::ofstream snapshot(raw_path);
  if (!snapshot)
    throw std::runtime_error("Could not write corridor policy snapshot");
  snapshot << "META\t3\t" << mode << '\t' << schedule_version
           << '\t' << participant_id << '\n';
  snapshot << std::setprecision(15)
           << "WEIGHTS\t"
           << nonnegative_environment_value("RMF_TRAFFIC_LAB_SAME_WEIGHT", 0.25) << '\t'
           << nonnegative_environment_value("RMF_TRAFFIC_LAB_OPPOSITE_WEIGHT", 8.0) << '\t'
           << nonnegative_environment_value("RMF_TRAFFIC_LAB_OCCUPIED_WEIGHT", 1.5) << '\t'
           << nonnegative_environment_value("RMF_TRAFFIC_LAB_FUTURE_WEIGHT", 0.6) << '\t'
           << nonnegative_environment_value("RMF_TRAFFIC_LAB_NO_ESCAPE_WEIGHT", 25.0) << '\t'
           << nonnegative_environment_value("RMF_TRAFFIC_LAB_STATIC_WEIGHT", 0.0) << '\t'
           << nonnegative_environment_value("RMF_TRAFFIC_LAB_OVERLAP_MARGIN", 0.25) << '\t'
           << nonnegative_environment_value("RMF_TRAFFIC_LAB_SCHEDULE_SOFT_LAMBDA", 0.25) << '\t'
           << nonnegative_environment_value("RMF_TRAFFIC_LAB_SCHEDULE_SOFT_MAX_PENALTY", 10.0) << '\t'
           << nonnegative_environment_value("RMF_TRAFFIC_LAB_SCHEDULE_SOFT_SAME_WEIGHT", 0.5) << '\t'
           << nonnegative_environment_value("RMF_TRAFFIC_LAB_SCHEDULE_SOFT_OPPOSITE_WEIGHT", 1.5)
           << '\n';
  for (const auto& corridor : ActiveCorridors)
  {
    snapshot << "CORRIDOR\t" << corridor.id << '\t' << corridor.capacity
             << '\t' << (corridor.passing_allowed ? 1 : 0)
             << '\t' << (corridor.hard_opposite_direction_block ? 1 : 0)
             << '\t' << corridor.base_penalty << '\n';
    for (const auto lane : corridor.lanes_forward)
      snapshot << "LANE\t" << lane << '\t' << corridor.id << "\tA_TO_B\n";
    for (const auto lane : corridor.lanes_reverse)
      snapshot << "LANE\t" << lane << '\t' << corridor.id << "\tB_TO_A\n";
  }
  for (const auto& interval : intervals)
  {
    snapshot << "INTERVAL\t" << interval.corridor
             << '\t' << interval.participant << '\t' << interval.plan
             << '\t' << interval.route << '\t' << interval.direction
             << '\t' << interval.enter_s << '\t' << interval.exit_s
             << '\t' << interval.state << '\t' << (interval.owner ? 1 : 0)
             << '\t' << (interval.responsive ? 1 : 0)
             << '\t' << interval.itinerary_version << '\t' << interval.source << '\n';
  }
  snapshot.close();
  static std::size_t generation = 0;
  const std::string generation_text = std::to_string(++generation);
  ::setenv("RMF_TRAFFIC_LAB_POLICY_GENERATION", generation_text.c_str(), 1);

  for (const auto& interval : intervals)
  {
    const bool from_schedule = interval.source == "SCHEDULE";
    writer.write(
      "{\"event\":\"corridor_schedule_interval\"," 
      "\"source\":" + json_string(
        from_schedule ? "SCHEDULE" : "POLICY_DERIVED")
      + ",\"analysis_source\":\"POLICY_DERIVED\"," 
      "\"snapshot_generation\":" + generation_text
      + ",\"schedule_version\":" + std::to_string(schedule_version)
      + ",\"corridor_id\":" + json_string(interval.corridor)
      + ",\"participant_id\":" + std::to_string(interval.participant)
      + ",\"plan_id\":" + std::to_string(interval.plan)
      + ",\"route_id\":" + std::to_string(interval.route)
      + ",\"direction\":" + json_string(interval.direction)
      + ",\"corridor_enter_s\":" + std::to_string(interval.enter_s)
      + ",\"corridor_exit_s\":" + std::to_string(interval.exit_s)
      + ",\"state\":" + json_string(interval.state)
      + ",\"owner\":" + (interval.owner ? "true" : "false")
      + ",\"responsive\":" + (interval.responsive ? "true" : "false")
      + ",\"itinerary_version\":"
      + std::to_string(interval.itinerary_version)
      + ",\"trajectory_source\":" + json_string(
        from_schedule
          ? "Database::query(query_all) Route::trajectory"
          : "RMF_CORE Plan::get_itinerary free-flow reservation")
      + ",\"state_source\":" + json_string(interval.source)
      + "}");
  }

  for (const auto& corridor : ActiveCorridors)
  {
    const auto owner = std::find_if(
      intervals.begin(), intervals.end(),
      [&](const PolicyScheduleInterval& interval)
      {
        return interval.corridor == corridor.id && interval.owner;
      });
    std::vector<std::size_t> occupants;
    std::vector<std::size_t> reserved;
    std::size_t waiting_same = 0;
    std::size_t waiting_opposite = 0;
    for (const auto& interval : intervals)
    {
      if (interval.corridor != corridor.id)
        continue;
      if (interval.state == "OCCUPIED" || interval.state == "UNKNOWN_HOLD")
        occupants.push_back(interval.participant);
      else
        reserved.push_back(interval.participant);
      if (owner != intervals.end() && interval.participant != owner->participant)
      {
        if (interval.direction == owner->direction)
          ++waiting_same;
        else
          ++waiting_opposite;
      }
    }
    const std::string state = owner == intervals.end()
      ? "FREE" : owner->state;
    writer.write(
      "{\"event\":\"corridor_runtime_state\","
      "\"source\":\"POLICY_DERIVED\",\"schedule_source\":\"SCHEDULE\","
      "\"snapshot_generation\":" + generation_text
      + ",\"schedule_version\":" + std::to_string(schedule_version)
      + ",\"planning_time_s\":" + std::to_string(planning_time_s)
      + ",\"corridor_id\":" + json_string(corridor.id)
      + ",\"state\":" + json_string(state)
      + ",\"direction\":"
      + json_string(owner == intervals.end() ? "NONE" : owner->direction)
      + ",\"owner\":"
      + (owner == intervals.end()
        ? std::string("null") : std::to_string(owner->participant))
      + ",\"occupants\":" + json_number_array(occupants)
      + ",\"reserved_participants\":" + json_number_array(reserved)
      + ",\"waiting_same_direction\":" + std::to_string(waiting_same)
      + ",\"waiting_opposite_direction\":" + std::to_string(waiting_opposite)
      + ",\"reserved_enter_s\":"
      + (owner == intervals.end()
        ? std::string("null") : std::to_string(owner->enter_s))
      + ",\"reserved_exit_s\":"
      + (owner == intervals.end()
        ? std::string("null") : std::to_string(owner->exit_s))
      + ",\"last_update_s\":" + std::to_string(planning_time_s)
      + ",\"release_condition\":\"confirmed corridor exit checkpoint; expected ETA alone does not release\""
      + ",\"passing_allowed\":"
      + (corridor.passing_allowed ? "true" : "false")
      + ",\"capacity\":" + std::to_string(corridor.capacity)
      + "}");
    const auto previous = previous_corridor_state.find(corridor.id);
    const std::string from_state = previous == previous_corridor_state.end()
      ? "FREE" : previous->second;
    if (from_state != state)
    {
      writer.write(
        "{\"event\":\"corridor_state_transition\","
        "\"source\":\"POLICY_DERIVED\",\"corridor_id\":"
        + json_string(corridor.id)
        + ",\"from_state\":" + json_string(from_state)
        + ",\"to_state\":" + json_string(state)
        + ",\"at_s\":" + std::to_string(planning_time_s)
        + ",\"owner\":"
        + (owner == intervals.end()
          ? std::string("null") : std::to_string(owner->participant))
        + ",\"reason\":\"planning snapshot admission state changed\"}");
    }
    previous_corridor_state[corridor.id] = state;
  }

  writer.write(
    "{\"event\":\"corridor_policy_snapshot\",\"source\":\"SCHEDULE\","
    "\"derived_source\":\"POLICY_DERIVED\",\"mode\":" + json_string(mode)
    + ",\"schedule_version\":" + std::to_string(schedule_version)
    + ",\"participant_id\":" + std::to_string(participant_id)
    + ",\"planning_time_s\":" + std::to_string(planning_time_s)
    + ",\"corridor_count\":" + std::to_string(ActiveCorridors.size())
    + ",\"interval_count\":" + std::to_string(intervals.size())
    + ",\"invocation_reason\":" + json_string(invocation_reason)
    + ",\"query_api\":\"Database::query(schedule::query_all) once per planning invocation\""
    + ",\"query_strategy\":\"single DB snapshot query; indexed by physical corridor; no per-expansion DB scan\""
    + ",\"schedule_soft_mode\":" + (schedule_soft_mode ? "true" : "false")
    + ",\"queried_participant_count\":" + std::to_string(queried_participant_count)
    + ",\"schedule_query_count\":" + std::to_string(database ? 1 : 0)
    + ",\"queried_route_count\":" + std::to_string(queried_route_count)
    + ",\"self_filtered_route_count\":" + std::to_string(self_filtered_route_count)
    + ",\"snapshot_generation\":" + generation_text
    + ",\"snapshot_consistency\":\"fixed for one Planner invocation\"}");
}

std::vector<PolicyScheduleInterval> make_deterministic_admission_reservations(
  const LabGraph& graph,
  const std::vector<std::optional<Plan>>& baseline_plans,
  const std::vector<RobotRequest>& requests,
  const std::vector<rmf_traffic::schedule::ParticipantId>& participant_ids)
{
  std::vector<PolicyScheduleInterval> candidates;
  for (std::size_t robot = 0; robot < baseline_plans.size(); ++robot)
  {
    if (!baseline_plans[robot].has_value())
      continue;
    const auto& itinerary = baseline_plans[robot]->get_itinerary();
    for (const auto& corridor : ActiveCorridors)
    {
      std::optional<std::pair<double, double>> best_forward;
      std::optional<std::pair<double, double>> best_reverse;
      std::size_t route_id = 0;
      std::size_t best_route_id = 0;
      for (const auto& route : itinerary)
      {
        const auto forward = route_interval_for_lanes(
          route, graph, corridor.lanes_forward);
        const auto reverse = route_interval_for_lanes(
          route, graph, corridor.lanes_reverse);
        if (forward.has_value()
          && (!best_forward.has_value() || forward->first < best_forward->first))
        {
          best_forward = forward;
          best_route_id = route_id;
        }
        if (reverse.has_value()
          && (!best_reverse.has_value() || reverse->first < best_reverse->first))
        {
          best_reverse = reverse;
          best_route_id = route_id;
        }
        ++route_id;
      }
      const bool forward = best_forward.has_value()
        && (!best_reverse.has_value()
        || best_forward->first <= best_reverse->first);
      const auto selected = forward ? best_forward : best_reverse;
      if (!selected.has_value())
        continue;
      candidates.push_back(PolicyScheduleInterval{
        corridor.id,
        participant_ids.at(robot),
        0,
        best_route_id,
        forward ? "A_TO_B" : "B_TO_A",
        selected->first,
        selected->second,
        "RESERVED",
        false,
        true,
        0,
        "POLICY_DERIVED"});
    }
  }

  std::vector<PolicyScheduleInterval> output;
  for (const auto& corridor : ActiveCorridors)
  {
    auto winner = candidates.end();
    for (auto it = candidates.begin(); it != candidates.end(); ++it)
    {
      if (it->corridor != corridor.id)
        continue;
      const auto robot = std::find(
        participant_ids.begin(), participant_ids.end(), it->participant);
      const std::size_t index = robot == participant_ids.end()
        ? 0 : static_cast<std::size_t>(std::distance(participant_ids.begin(), robot));
      const double requested = index < requests.size()
        ? std::max(requests[index].start_time_s, requests[index].insertion_time_s)
        : it->enter_s;
      if (winner == candidates.end())
      {
        winner = it;
        continue;
      }
      const auto current_robot = std::find(
        participant_ids.begin(), participant_ids.end(), winner->participant);
      const std::size_t current_index = current_robot == participant_ids.end()
        ? 0 : static_cast<std::size_t>(
          std::distance(participant_ids.begin(), current_robot));
      const double current_requested = current_index < requests.size()
        ? std::max(
          requests[current_index].start_time_s,
          requests[current_index].insertion_time_s)
        : winner->enter_s;
      if (std::make_pair(requested, it->participant)
        < std::make_pair(current_requested, winner->participant))
      {
        winner = it;
      }
    }
    if (winner == candidates.end())
      continue;
    for (auto candidate : candidates)
    {
      if (candidate.corridor != corridor.id
        || candidate.direction != winner->direction)
      {
        continue;
      }
      candidate.owner = candidate.participant == winner->participant;
      output.push_back(std::move(candidate));
    }
  }
  return output;
}

void configure_shared_corridor_penalty(
  JsonlWriter& writer,
  const LabGraph& graph,
  const std::vector<RobotRequest>& requests,
  const std::vector<std::vector<std::size_t>>& baseline_lanes)
{
  const char* raw_mode = std::getenv("RMF_TRAFFIC_LAB_PENALTY_MODE");
  if (!raw_mode || std::string(raw_mode) != "shared_corridor")
    return;

  using Corridor = std::pair<std::size_t, std::size_t>;
  std::map<Corridor, std::set<std::size_t>> corridor_users;
  for (std::size_t robot = 0; robot < baseline_lanes.size(); ++robot)
  {
    std::set<Corridor> unique_corridors;
    for (const auto lane_id : baseline_lanes[robot])
    {
      if (lane_id >= graph.lanes.size())
        continue;
      const auto& lane = graph.lanes[lane_id];
      unique_corridors.insert(Corridor{
        std::min(lane.entry, lane.exit), std::max(lane.entry, lane.exit)});
    }
    for (const auto& corridor : unique_corridors)
      corridor_users[corridor].insert(robot);
  }

  const double weight = positive_environment_value(
    "RMF_TRAFFIC_LAB_OCCUPANCY_WEIGHT", 60.0);
  const double free_capacity = positive_environment_value(
    "RMF_TRAFFIC_LAB_OCCUPANCY_FREE_CAPACITY", 1.0);
  std::map<std::size_t, double> occupancy;
  std::map<std::size_t, double> penalties;
  for (const auto& lane : graph.lanes)
  {
    const Corridor corridor{
      std::min(lane.entry, lane.exit), std::max(lane.entry, lane.exit)};
    const auto users = corridor_users.find(corridor);
    if (users == corridor_users.end()
      || static_cast<double>(users->second.size()) <= free_capacity)
    {
      continue;
    }
    const double demand = static_cast<double>(users->second.size());
    occupancy[lane.id] = demand;
    penalties[lane.id] = weight * (demand - free_capacity);
  }

  std::ostringstream specification;
  bool first_specification = true;
  for (const auto& [lane, demand] : occupancy)
  {
    if (!first_specification)
      specification << ',';
    first_specification = false;
    specification << lane << ':' << demand;
  }
  if (specification.str().empty())
    ::unsetenv("RMF_TRAFFIC_LAB_LANE_OCCUPANCY");
  else
    ::setenv(
      "RMF_TRAFFIC_LAB_LANE_OCCUPANCY",
      specification.str().c_str(), true);

  std::ostringstream line;
  line << std::setprecision(12)
       << "{\"event\":\"occupancy_penalty_configuration\","
       << "\"source\":\"real_rmf_free_flow_baseline_plan_overlap\","
       << "\"mode\":\"shared_corridor\","
       << "\"algorithm\":\"penalty=weight*max(0,predicted_robots-free_capacity)\","
       << "\"weight\":" << weight
       << ",\"free_capacity\":" << free_capacity
       << ",\"active\":" << (penalties.empty() ? "false" : "true")
       << ",\"baseline_lanes_by_robot\":{";
  for (std::size_t i = 0; i < requests.size(); ++i)
  {
    if (i > 0)
      line << ',';
    const std::vector<std::size_t> empty;
    const auto& lanes = i < baseline_lanes.size() ? baseline_lanes[i] : empty;
    line << json_string(requests[i].name) << ':' << json_number_array(lanes);
  }
  line << "},\"shared_corridor_users\":{";
  bool first_corridor = true;
  for (const auto& [corridor, users] : corridor_users)
  {
    if (static_cast<double>(users.size()) <= free_capacity)
      continue;
    if (!first_corridor)
      line << ',';
    first_corridor = false;
    line << json_string(
      std::to_string(corridor.first) + "-" + std::to_string(corridor.second))
         << ":[";
    bool first_user = true;
    for (const auto robot : users)
    {
      if (!first_user)
        line << ',';
      first_user = false;
      line << json_string(requests.at(robot).name);
    }
    line << ']';
  }
  line << "},\"directed_lane_occupancy\":{";
  bool first_value = true;
  for (const auto& [lane, demand] : occupancy)
  {
    if (!first_value)
      line << ',';
    first_value = false;
    line << json_string(std::to_string(lane)) << ':' << demand;
  }
  line << "},\"directed_lane_penalties\":{";
  first_value = true;
  for (const auto& [lane, penalty] : penalties)
  {
    if (!first_value)
      line << ',';
    first_value = false;
    line << json_string(std::to_string(lane)) << ':' << penalty;
  }
  line << "},\"environment_spec\":"
       << json_string(specification.str()) << '}';
  writer.write(line.str());
}

std::set<std::size_t> make_closed_set(
  const std::vector<std::size_t>& closed_lanes)
{
  return {closed_lanes.begin(), closed_lanes.end()};
}

double lane_length(const LabGraph& graph, const LaneDef& lane)
{
  const auto& a = graph.nodes.at(lane.entry);
  const auto& b = graph.nodes.at(lane.exit);
  return std::hypot(b.x - a.x, b.y - a.y);
}

void write_graph_and_configuration(
  JsonlWriter& writer,
  const std::string& scenario,
  const std::string& description,
  const LabGraph& lab_graph,
  const std::vector<std::size_t>& closed_lanes,
  const std::size_t robot_count)
{
  writer.write(
    "{\"event\":\"run_started\",\"schema\":\"rmf_core_lab.v16\","
    "\"scenario\":" + json_string(scenario)
    + ",\"description\":" + json_string(description)
    + ",\"robot_count\":" + std::to_string(robot_count) + "}");

  writer.write(
    "{\"event\":\"rmf_runtime_proof\","
    "\"language\":\"C++17\","
    "\"linked_target\":\"rmf_traffic::rmf_traffic\","
    "\"planner_api\":\"rmf_traffic::agv::Planner\","
    "\"search_api\":\"rmf_traffic::agv::Planner::Debug\","
    "\"negotiation_api\":\"rmf_traffic::agv::CentralizedNegotiation\","
    "\"schedule_api\":\"rmf_traffic::schedule::Database\","
    "\"conflict_api\":\"rmf_traffic::DetectConflict::between\","
    "\"python_role\":\"build/run orchestration and JSONL-to-HTML rendering only\","
    "\"mock_planner\":false}");

  writer.write(
    "{\"event\":\"process_phase\",\"phase\":\"graph_loaded\","
    "\"order\":1,\"label\":\"Navigation Graph and vehicle traits loaded\"}");

  writer.write(
    "{\"event\":\"data_model\",\"navigation_db\":"
    + json_string(
      "RMF Graph is an in-memory navigation graph: waypoint properties plus directed lanes")
    + ",\"schedule_db\":"
    + json_string(
      "RMF schedule::Database stores participant descriptions and time-parameterized itineraries")
    + ",\"search_debug\":"
    + json_string(
      "Planner::Debug exposes real A* frontier nodes, g, h, f and expansion order; API is unstable/debug-only")
    + ",\"limitation\":"
    + json_string(
      "The public debug API does not expose a complete reason code for every rejected child branch")
    + "}");

  writer.write(
    "{\"event\":\"schedule_model_schema\","
    "\"database_class\":\"rmf_traffic::schedule::Database\","
    "\"write_path\":\"Database::register_participant -> Participant::set\","
    "\"read_path\":\"Database::query(query_all) -> Viewer::View::Element -> Route -> Trajectory -> Waypoint\","
    "\"participant_read_path\":\"get_participant/get_itinerary/get_current_plan_id/itinerary_version/get_current_progress_version\","
    "\"hierarchy\":[\"Database\",\"ParticipantDescription\",\"Itinerary\",\"Route\",\"Trajectory\",\"Waypoint\"],"
    "\"jsonl_representation\":\"A flattened observation of the real in-memory RMF objects, not an independent mock database or RMF binary serialization\","
    "\"not_exported\":[\"internal storage indexes\",\"patch cull history\",\"dependency graph internals\",\"inconsistency ranges not exercised by this lab\"]}");

  writer.write(
    "{\"event\":\"graph_summary\",\"map\":" + json_string(lab_graph.map_name)
    + ",\"waypoint_count\":" + std::to_string(lab_graph.graph.num_waypoints())
    + ",\"lane_count\":" + std::to_string(lab_graph.graph.num_lanes())
    + ",\"directed\":true}");

  writer.write(
    "{\"event\":\"planner_graph_context\","
    "\"graph_object\":\"rmf_traffic::agv::Graph\","
    "\"graph_read_api\":\"Planner::Configuration::graph + Graph::get_waypoint/get_lane\","
    "\"waypoint_count\":" + std::to_string(lab_graph.graph.num_waypoints())
    + ",\"directed_lane_count\":" + std::to_string(lab_graph.graph.num_lanes())
    + ",\"supergraph_object\":\"rmf_traffic::agv::planning::Supergraph (internal header)\"," 
    "\"supergraph_public_api_available\":false,"
    "\"supergraph_observation\":\"Planner and Planner::Debug consume the internally constructed planning graph, but the public/debug API does not expose the Supergraph nodes, keys or cached heuristic tables\","
    "\"ui_representation\":\"Graph rows are exact public API values; Supergraph is shown only as an explicitly limited internal layer\"}");

  for (const auto& node : lab_graph.nodes)
  {
    const auto& waypoint = lab_graph.graph.get_waypoint(node.id);
    std::ostringstream line;
    line << std::setprecision(12)
         << "{\"event\":\"graph_node\",\"id\":" << node.id
         << ",\"name\":" << json_string(node.name)
         << ",\"map\":" << json_string(waypoint.get_map_name())
         << ",\"x\":" << node.x
         << ",\"y\":" << node.y
         << ",\"holding\":" << (waypoint.is_holding_point() ? "true" : "false")
         << ",\"passthrough\":" << (waypoint.is_passthrough_point() ? "true" : "false")
         << ",\"parking\":" << (waypoint.is_parking_spot() ? "true" : "false")
         << ",\"charger\":" << (waypoint.is_charger() ? "true" : "false")
         << ",\"mutex_group\":" << json_string(waypoint.in_mutex_group())
         << ",\"merge_radius_m\":";
    if (waypoint.merge_radius().has_value())
      line << *waypoint.merge_radius();
    else
      line << "null";
    line << ",\"outgoing_lanes\":"
         << json_number_array(lab_graph.graph.lanes_from(node.id))
         << ",\"incoming_lanes\":"
         << json_number_array(lab_graph.graph.lanes_into(node.id)) << '}';
    writer.write(line.str());
  }

  const auto closed = make_closed_set(closed_lanes);
  for (const auto& lane_def : lab_graph.lanes)
  {
    const auto& lane = lab_graph.graph.get_lane(lane_def.id);
    const auto speed = lane.properties().speed_limit();
    std::ostringstream line;
    line << std::setprecision(12)
         << "{\"event\":\"graph_lane\",\"id\":" << lane_def.id
         << ",\"entry\":" << lane_def.entry
         << ",\"exit\":" << lane_def.exit
         << ",\"length_m\":" << lane_length(lab_graph, lane_def)
         << ",\"speed_limit_mps\":" << json_optional_number(speed)
         << ",\"effective_speed_mps\":"
         << (speed.has_value() ? std::min(*speed, LinearVelocity) : LinearVelocity)
         << ",\"mutex_group\":"
         << json_string(lane.properties().in_mutex_group())
         << ",\"closed\":" << (closed.count(lane_def.id) ? "true" : "false")
         << '}';
    writer.write(line.str());
  }

  writer.write(
    "{\"event\":\"vehicle_traits\",\"profile_radius_m\":"
    + std::to_string(RobotRadius)
    + ",\"linear_velocity_mps\":" + std::to_string(LinearVelocity)
    + ",\"linear_acceleration_mps2\":" + std::to_string(LinearAcceleration)
    + ",\"angular_velocity_radps\":" + std::to_string(AngularVelocity)
    + ",\"angular_acceleration_radps2\":" + std::to_string(AngularAcceleration)
    + ",\"steering\":\"differential\",\"reversible\":false,"
    "\"motion_policy\":\"forward-only; rotate in place before changing travel direction\"}");

  writer.write(
    "{\"event\":\"planner_configuration\",\"cost_formula\":"
    + json_string(
      "RMF cost combines elapsed time, traversal_cost_per_meter*distance, rotations, events, waiting and validator constraints")
    + ",\"traversal_cost_per_meter\":0.0,\"minimum_holding_time_s\":1.0"
    + ",\"saturation_limit\":" + std::to_string(SaturationLimit)
    + ",\"closed_lanes\":" + json_number_array(closed_lanes)
    + ",\"reverse_motion_allowed\":false"
    + ",\"heuristic\":"
    + json_string(
      "h(n) uses RMF QuickestPath-style remaining-cost estimates; f(n)=g(n)+h(n)")
    + "}");

  writer.write(
    "{\"event\":\"validator_configuration\"," 
    "\"source\":\"RMF_CORE\"," 
    "\"phase\":\"base_planner\"," 
    "\"planner_options_validator\":null,"
    "\"planner_options_constructor\":\"Planner::Options(nullptr)\","
    "\"schedule_aware\":false,"
    "\"purpose\":\"free-flow and baseline searches intentionally have no RouteValidator\","
    "\"post_proposal_validator\":\"rmf_traffic::DetectConflict::between\"}");

  if (!closed_lanes.empty())
  {
    writer.write(
      "{\"event\":\"lane_closure\",\"closed_lanes\":"
      + json_number_array(closed_lanes) + "}");
  }
}

Planner::Configuration make_configuration(
  const LabGraph& lab_graph,
  const rmf_traffic::agv::VehicleTraits& traits,
  const std::vector<std::size_t>& closed_lanes)
{
  Planner::Configuration configuration{lab_graph.graph, traits};
  if (!closed_lanes.empty())
  {
    rmf_traffic::agv::LaneClosure closures;
    for (const auto lane : closed_lanes)
      closures.close(lane);
    configuration.lane_closures(std::move(closures));
  }
  return configuration;
}

struct TraceStats
{
  std::size_t expansions = 0;
  std::size_t unique_nodes = 0;
  std::size_t terminal_nodes = 0;
  bool solution_found = false;
  bool step_limit_reached = false;
};

struct SearchCostDiagnostics
{
  double route_elapsed_s = 0.0;
  double translation_time_s = 0.0;
  double rotation_time_s = 0.0;
  double wait_time_s = 0.0;
  double translation_distance_m = 0.0;
  double rotation_angle_rad = 0.0;
  std::optional<double> delta_g;
  std::optional<double> unexposed_g_remainder;
};

struct HeuristicDiagnostics
{
  std::optional<double> euclidean_distance_m;
  std::optional<double> euclidean_cruise_time_s;
  std::optional<double> graph_distance_m;
  std::optional<double> graph_cruise_time_s;
  std::optional<std::size_t> first_lane;
  std::optional<double> first_turn_angle_rad;
  std::optional<double> first_turn_time_s;
  std::optional<double> graph_plus_first_turn_s;
  std::optional<double> rmf_h_minus_graph_cruise_s;
};

double wrapped_angle_difference(const double from, const double to)
{
  return std::abs(std::remainder(to - from, 2.0 * Pi));
}

double rest_to_rest_rotation_time(const double angle)
{
  const double magnitude = std::abs(angle);
  if (magnitude <= 1e-12)
    return 0.0;

  const double triangular_limit =
    AngularVelocity * AngularVelocity / AngularAcceleration;
  if (magnitude <= triangular_limit)
    return 2.0 * std::sqrt(magnitude / AngularAcceleration);

  return 2.0 * AngularVelocity / AngularAcceleration
    + (magnitude - triangular_limit) / AngularVelocity;
}

SearchCostDiagnostics search_cost_diagnostics(
  const Planner::Debug::ConstNodePtr& node)
{
  SearchCostDiagnostics result;
  if (node->parent)
    result.delta_g = node->current_cost - node->parent->current_cost;

  for (const auto& route : node->route_from_parent)
  {
    bool have_previous = false;
    double previous_time_s = 0.0;
    Eigen::Vector3d previous_position = Eigen::Vector3d::Zero();
    for (const auto& point : route.trajectory())
    {
      const double time_s = rmf_traffic::time::to_seconds(
        point.time().time_since_epoch());
      const auto position = point.position();
      if (have_previous)
      {
        const double dt = std::max(0.0, time_s - previous_time_s);
        const double distance = std::hypot(
          position[0] - previous_position[0],
          position[1] - previous_position[1]);
        const double rotation = wrapped_angle_difference(
          previous_position[2], position[2]);
        result.route_elapsed_s += dt;
        result.translation_distance_m += distance;
        result.rotation_angle_rad += rotation;
        if (distance > 1e-6)
          result.translation_time_s += dt;
        else if (rotation > 1e-6)
          result.rotation_time_s += dt;
        else
          result.wait_time_s += dt;
      }
      previous_time_s = time_s;
      previous_position = position;
      have_previous = true;
    }
  }

  if (result.delta_g.has_value())
    result.unexposed_g_remainder = *result.delta_g - result.route_elapsed_s;
  return result;
}

HeuristicDiagnostics heuristic_diagnostics(
  const LabGraph& lab_graph,
  const std::vector<std::size_t>& closed_lanes,
  const Planner::Debug::ConstNodePtr& node,
  const std::size_t goal_waypoint)
{
  HeuristicDiagnostics result;
  if (!node->waypoint.has_value()
    || *node->waypoint >= lab_graph.nodes.size()
    || goal_waypoint >= lab_graph.nodes.size())
  {
    return result;
  }

  const std::size_t start_waypoint = *node->waypoint;
  const auto& start_node = lab_graph.nodes.at(start_waypoint);
  const auto& goal_node = lab_graph.nodes.at(goal_waypoint);
  result.euclidean_distance_m = std::hypot(
    goal_node.x - start_node.x, goal_node.y - start_node.y);
  result.euclidean_cruise_time_s =
    *result.euclidean_distance_m / LinearVelocity;

  struct Candidate
  {
    double time_s;
    double distance_m;
    std::size_t waypoint;
    std::optional<std::size_t> first_lane;
  };
  struct CandidateLater
  {
    bool operator()(const Candidate& a, const Candidate& b) const
    {
      return a.time_s > b.time_s;
    }
  };

  std::vector<double> best(
    lab_graph.nodes.size(), std::numeric_limits<double>::infinity());
  std::priority_queue<Candidate, std::vector<Candidate>, CandidateLater> queue;
  const auto closed = make_closed_set(closed_lanes);
  best[start_waypoint] = 0.0;
  queue.push(Candidate{0.0, 0.0, start_waypoint, std::nullopt});

  while (!queue.empty())
  {
    const auto current = queue.top();
    queue.pop();
    if (current.time_s > best[current.waypoint] + 1e-12)
      continue;
    if (current.waypoint == goal_waypoint)
    {
      result.graph_distance_m = current.distance_m;
      result.graph_cruise_time_s = current.time_s;
      result.first_lane = current.first_lane;
      break;
    }

    for (const auto lane_id : lab_graph.graph.lanes_from(current.waypoint))
    {
      if (closed.count(lane_id) > 0 || lane_id >= lab_graph.lanes.size())
        continue;
      const auto& lane_def = lab_graph.lanes.at(lane_id);
      const auto speed_limit =
        lab_graph.graph.get_lane(lane_id).properties().speed_limit();
      const double speed = speed_limit.has_value()
        ? std::min(*speed_limit, LinearVelocity)
        : LinearVelocity;
      if (speed <= 0.0)
        continue;
      const double distance = lane_length(lab_graph, lane_def);
      const double next_time = current.time_s + distance / speed;
      if (next_time + 1e-12 >= best.at(lane_def.exit))
        continue;
      best[lane_def.exit] = next_time;
      queue.push(Candidate{
        next_time,
        current.distance_m + distance,
        lane_def.exit,
        current.first_lane.has_value()
          ? current.first_lane
          : std::optional<std::size_t>{lane_id}});
    }
  }

  if (result.first_lane.has_value())
  {
    const auto& first = lab_graph.lanes.at(*result.first_lane);
    const auto& entry = lab_graph.nodes.at(first.entry);
    const auto& exit = lab_graph.nodes.at(first.exit);
    const double desired_yaw = std::atan2(exit.y - entry.y, exit.x - entry.x);
    result.first_turn_angle_rad = wrapped_angle_difference(
      node->orientation, desired_yaw);
    result.first_turn_time_s = rest_to_rest_rotation_time(
      *result.first_turn_angle_rad);
  }

  if (result.graph_cruise_time_s.has_value())
  {
    result.graph_plus_first_turn_s = *result.graph_cruise_time_s
      + result.first_turn_time_s.value_or(0.0);
    result.rmf_h_minus_graph_cruise_s =
      node->remaining_cost_estimate - *result.graph_cruise_time_s;
  }
  return result;
}

void append_search_diagnostics(
  std::ostringstream& line,
  const SearchCostDiagnostics& cost,
  const HeuristicDiagnostics& heuristic)
{
  line << std::setprecision(12)
       << ",\"g_breakdown_scope\":\"route_from_parent trajectory-derived; aggregate delta_g is exact RMF\""
       << ",\"g_route_elapsed_s\":" << cost.route_elapsed_s
       << ",\"g_translation_time_s\":" << cost.translation_time_s
       << ",\"g_rotation_time_s\":" << cost.rotation_time_s
       << ",\"g_wait_time_s\":" << cost.wait_time_s
       << ",\"g_translation_distance_m\":" << cost.translation_distance_m
       << ",\"g_rotation_angle_rad\":" << cost.rotation_angle_rad
       << ",\"g_unexposed_remainder\":"
       << json_optional_number(cost.unexposed_g_remainder)
       << ",\"g_component_breakdown_available\":false"
       << ",\"h_breakdown_scope\":\"exact RMF h plus lab-side lower-bound diagnostics; not an internal h serialization\""
       << ",\"h_euclidean_distance_m\":"
       << json_optional_number(heuristic.euclidean_distance_m)
       << ",\"h_euclidean_cruise_time_s\":"
       << json_optional_number(heuristic.euclidean_cruise_time_s)
       << ",\"h_graph_distance_m\":"
       << json_optional_number(heuristic.graph_distance_m)
       << ",\"h_graph_cruise_time_s\":"
       << json_optional_number(heuristic.graph_cruise_time_s)
       << ",\"h_first_lane\":";
  if (heuristic.first_lane.has_value())
    line << *heuristic.first_lane;
  else
    line << "null";
  line << ",\"h_first_turn_angle_rad\":"
       << json_optional_number(heuristic.first_turn_angle_rad)
       << ",\"h_first_turn_time_s\":"
       << json_optional_number(heuristic.first_turn_time_s)
       << ",\"h_graph_plus_first_turn_s\":"
       << json_optional_number(heuristic.graph_plus_first_turn_s)
       << ",\"h_rmf_minus_graph_cruise_s\":"
       << json_optional_number(heuristic.rmf_h_minus_graph_cruise_s)
       << ",\"h_component_breakdown_available\":false";
}

void write_search_node(
  JsonlWriter& writer,
  const std::string& event,
  const std::string& robot,
  const std::size_t step,
  const Planner::Debug::ConstNodePtr& node,
  const std::size_t queue_size,
  const LabGraph& lab_graph,
  const std::vector<std::size_t>& closed_lanes,
  const std::size_t goal_waypoint)
{
  const auto cost = search_cost_diagnostics(node);
  const auto heuristic = heuristic_diagnostics(
    lab_graph, closed_lanes, node, goal_waypoint);
  std::ostringstream line;
  line << std::setprecision(12)
       << "{\"event\":" << json_string(event)
       << ",\"robot\":" << json_string(robot)
       << ",\"step\":" << step
       << ",\"node_id\":" << node->id
       << ",\"parent_id\":";
  if (node->parent)
    line << node->parent->id;
  else
    line << "null";
  line << ",\"waypoint\":";
  if (node->waypoint.has_value())
    line << *node->waypoint;
  else
    line << "null";
  line << ",\"g\":" << node->current_cost
       << ",\"h\":" << node->remaining_cost_estimate
       << ",\"f\":" << node->current_cost + node->remaining_cost_estimate
       << ",\"delta_g_from_parent\":";
  if (node->parent)
    line << node->current_cost - node->parent->current_cost;
  else
    line << "null";
  line << ",\"delta_h_from_parent\":";
  if (node->parent)
  {
    line << node->remaining_cost_estimate
      - node->parent->remaining_cost_estimate;
  }
  else
    line << "null";
  line << ",\"delta_f_from_parent\":";
  if (node->parent)
  {
    line << (node->current_cost + node->remaining_cost_estimate)
      - (node->parent->current_cost
      + node->parent->remaining_cost_estimate);
  }
  else
    line << "null";
  append_search_diagnostics(line, cost, heuristic);
  line << ",\"ordering_rule\":\"Planner::Debug frontier top; lowest aggregate estimated cost\""
       << ",\"orientation_rad\":" << node->orientation
       << ",\"queue_size\":" << queue_size
       << ",\"route_segments_from_parent\":" << node->route_from_parent.size()
       << ",\"has_lane_event\":" << (node->event ? "true" : "false")
       << '}';
  writer.write(line.str());
}

TraceStats trace_real_search(
  JsonlWriter& writer,
  const std::string& robot,
  const Planner& planner,
  const Start& start,
  const Goal& goal,
  const LabGraph& lab_graph,
  const std::vector<std::size_t>& closed_lanes,
  const std::size_t goal_waypoint)
{
  writer.write(
    "{\"event\":\"astar_trace_started\",\"robot\":"
    + json_string(robot)
    + ",\"source\":\"rmf_traffic::agv::Planner::Debug\""
    + ",\"production_api\":false,\"ordering\":\"lowest f=g+h first\"}");

  Planner::Debug debugger(planner);
  auto progress = debugger.begin({start}, goal, make_planner_options());
  TraceStats stats;
  std::set<std::size_t> unique_node_ids;

  for (std::size_t step = 0; progress && step < TraceStepLimit; ++step)
  {
    const auto queue_before = progress.queue();
    const auto selected = queue_before.top();
    auto queue_snapshot = queue_before;
    queue_snapshot.pop();
    const auto next_best = queue_snapshot.empty()
      ? Planner::Debug::ConstNodePtr{nullptr}
      : queue_snapshot.top();

    std::ostringstream decision;
    const double selected_f =
      selected->current_cost + selected->remaining_cost_estimate;
    const auto selected_cost = search_cost_diagnostics(selected);
    const auto selected_heuristic = heuristic_diagnostics(
      lab_graph, closed_lanes, selected, goal_waypoint);
    decision << std::setprecision(12)
             << "{\"event\":\"astar_step_decision\",\"robot\":"
             << json_string(robot)
             << ",\"step\":" << step
             << ",\"selected_node_id\":" << selected->id
             << ",\"selected_waypoint\":";
    if (selected->waypoint.has_value())
      decision << *selected->waypoint;
    else
      decision << "null";
    decision << ",\"selected_g\":" << selected->current_cost
             << ",\"selected_h\":" << selected->remaining_cost_estimate
             << ",\"selected_f\":" << selected_f
             << ",\"delta_g_from_parent\":"
             << json_optional_number(selected_cost.delta_g)
             << ",\"frontier_size_before\":" << queue_before.size()
             << ",\"frontier_alternative_count\":"
             << (queue_before.size() > 0 ? queue_before.size() - 1 : 0)
             << ",\"next_best_node_id\":";
    if (next_best)
      decision << next_best->id;
    else
      decision << "null";
    decision << ",\"next_best_f\":";
    if (next_best)
    {
      decision << next_best->current_cost
        + next_best->remaining_cost_estimate;
    }
    else
      decision << "null";
    decision << ",\"f_margin_to_next\":";
    if (next_best)
    {
      decision << next_best->current_cost
        + next_best->remaining_cost_estimate - selected_f;
    }
    else
      decision << "null";
    decision << ",\"selection_basis\":"
             << json_string(
                  "The selected node was the top element of the real Planner::Debug frontier queue; aggregate g/h/f are exposed, but the comparator tie-break internals and rejected-branch reason codes are not")
             ;
    append_search_diagnostics(decision, selected_cost, selected_heuristic);
    decision << '}';
    writer.write(decision.str());

    queue_snapshot = queue_before;
    while (!queue_snapshot.empty())
    {
      unique_node_ids.insert(queue_snapshot.top()->id);
      queue_snapshot.pop();
    }

    write_search_node(
      writer, "astar_expand", robot, step, selected, queue_before.size(),
      lab_graph, closed_lanes, goal_waypoint);

    const auto plan = progress.step();
    ++stats.expansions;

    auto queue_after = progress.queue();
    std::size_t generated = 0;
    while (!queue_after.empty())
    {
      const auto node = queue_after.top();
      unique_node_ids.insert(node->id);
      if (node->parent && node->parent->id == selected->id)
      {
        write_search_node(
          writer, "astar_generated", robot, step, node,
          progress.queue().size(), lab_graph, closed_lanes, goal_waypoint);
        ++generated;
      }
      queue_after.pop();
    }

    std::ostringstream summary;
    summary << "{\"event\":\"astar_step_summary\",\"robot\":"
            << json_string(robot)
            << ",\"step\":" << step
            << ",\"expanded_node_id\":" << selected->id
            << ",\"selected_g\":" << selected->current_cost
            << ",\"selected_h\":" << selected->remaining_cost_estimate
            << ",\"selected_f\":" << selected_f
            << ",\"generated_children\":" << generated
            << ",\"frontier_size_after\":" << progress.queue().size()
            << ",\"terminal_count\":" << progress.terminal_nodes().size()
            << ",\"solution_found\":" << (plan.has_value() ? "true" : "false")
            << '}';
    writer.write(summary.str());

    if (!progress.queue().empty())
    {
      write_search_node(
        writer, "astar_frontier_best", robot, step,
        progress.queue().top(), progress.queue().size(),
        lab_graph, closed_lanes, goal_waypoint);
    }

    if (plan.has_value())
    {
      stats.solution_found = true;
      break;
    }
  }

  stats.unique_nodes = unique_node_ids.size();
  stats.terminal_nodes = progress.terminal_nodes().size();
  stats.step_limit_reached = !stats.solution_found && static_cast<bool>(progress);

  std::ostringstream line;
  line << "{\"event\":\"astar_trace_summary\",\"robot\":"
       << json_string(robot)
       << ",\"expansions\":" << stats.expansions
       << ",\"unique_nodes_observed\":" << stats.unique_nodes
       << ",\"terminal_nodes\":" << stats.terminal_nodes
       << ",\"solution_found\":" << (stats.solution_found ? "true" : "false")
       << ",\"step_limit_reached\":" << (stats.step_limit_reached ? "true" : "false")
       << '}';
  writer.write(line.str());
  return stats;
}

std::vector<std::size_t> write_plan(
  JsonlWriter& writer,
  const std::string& robot,
  const Plan& plan,
  const std::string& phase,
  const std::optional<double> ideal_cost = std::nullopt,
  const std::optional<std::size_t> result_expansions = std::nullopt,
  const std::optional<std::size_t> result_nodes = std::nullopt)
{
  std::vector<std::size_t> used_lanes;
  const auto& waypoints = plan.get_waypoints();
  for (std::size_t sequence = 0; sequence < waypoints.size(); ++sequence)
  {
    const auto& waypoint = waypoints[sequence];
    const auto& approach = waypoint.approach_lanes();
    used_lanes.insert(used_lanes.end(), approach.begin(), approach.end());

    const double current_time = rmf_traffic::time::to_seconds(
      waypoint.time().time_since_epoch());
    double delta_time = 0.0;
    double delta_distance = 0.0;
    double delta_yaw = 0.0;
    std::string movement_type = "start";
    std::string movement_reason =
      "Initial pose supplied to the RMF Planner";
    if (sequence > 0)
    {
      const auto& previous = waypoints[sequence - 1];
      const double previous_time = rmf_traffic::time::to_seconds(
        previous.time().time_since_epoch());
      delta_time = current_time - previous_time;
      const double dx = waypoint.position()[0] - previous.position()[0];
      const double dy = waypoint.position()[1] - previous.position()[1];
      delta_distance = std::hypot(dx, dy);
      const double raw_delta_yaw =
        waypoint.position()[2] - previous.position()[2];
      delta_yaw = std::atan2(
        std::sin(raw_delta_yaw), std::cos(raw_delta_yaw));

      if (delta_distance < 1e-6 && std::abs(delta_yaw) >= 1e-5)
      {
        movement_type = "rotate_in_place";
        movement_reason =
          "Reverse travel is disabled, so the differential-drive robot rotates in place to align with the next forward segment";
      }
      else if (delta_distance < 1e-6 && delta_time > 1e-6)
      {
        movement_type = "wait";
        movement_reason =
          "RMF kept the same pose while advancing time to satisfy timing, event or negotiation constraints";
      }
      else if (delta_distance >= 1e-6)
      {
        movement_type = "forward_traverse";
        movement_reason =
          "This segment belongs to the final RMF plan and is traversed forward through the recorded approach lanes";
      }
    }

    std::ostringstream line;
    line << std::setprecision(12)
         << "{\"event\":\"plan_waypoint\",\"robot\":"
         << json_string(robot)
         << ",\"phase\":" << json_string(phase)
         << ",\"sequence\":" << sequence
         << ",\"time_s\":"
         << current_time
         << ",\"x\":" << waypoint.position()[0]
         << ",\"y\":" << waypoint.position()[1]
         << ",\"yaw_rad\":" << waypoint.position()[2]
         << ",\"delta_time_s\":" << delta_time
         << ",\"delta_distance_m\":" << delta_distance
         << ",\"delta_yaw_rad\":" << delta_yaw
         << ",\"movement_type\":" << json_string(movement_type)
         << ",\"forward_only\":true"
         << ",\"movement_reason\":" << json_string(movement_reason)
         << ",\"graph_index\":";

    if (waypoint.graph_index().has_value())
      line << *waypoint.graph_index();
    else
      line << "null";

    line << ",\"approach_lanes\":" << json_number_array(approach)
         << ",\"decision_scope\":\"final_rmf_plan_waypoint\"}";
    writer.write(line.str());
  }

  const auto& itinerary = plan.get_itinerary();
  writer.write(
    "{\"event\":\"itinerary_summary\",\"robot\":"
    + json_string(robot)
    + ",\"phase\":" + json_string(phase)
    + ",\"source_api\":\"rmf_traffic::agv::Plan::get_itinerary\""
    + ",\"object_type\":\"rmf_traffic::schedule::Itinerary (std::vector<Route>)\""
    + ",\"route_count\":" + std::to_string(itinerary.size())
    + ",\"schedule_committed\":false"
    + ",\"meaning\":\"This is the itinerary carried by the Planner or proposal Plan; Schedule DB commit is recorded separately\"}");
  std::size_t trajectory_points = 0;
  for (std::size_t route_index = 0; route_index < itinerary.size(); ++route_index)
  {
    const auto& route = itinerary[route_index];
    std::size_t route_point_count = 0;
    std::optional<double> route_start_time;
    std::optional<double> route_finish_time;
    for (const auto& trajectory_waypoint : route.trajectory())
    {
      const double time_s = rmf_traffic::time::to_seconds(
        trajectory_waypoint.time().time_since_epoch());
      if (!route_start_time.has_value())
        route_start_time = time_s;
      route_finish_time = time_s;
      ++route_point_count;
    }
    std::ostringstream route_summary;
    route_summary << std::setprecision(12)
                  << "{\"event\":\"route_summary\",\"robot\":"
                  << json_string(robot)
                  << ",\"phase\":" << json_string(phase)
                  << ",\"route_index\":" << route_index
                  << ",\"object_type\":\"rmf_traffic::Route\""
                  << ",\"source_api\":\"Plan::get_itinerary()[route_index]\""
                  << ",\"map\":" << json_string(route.map())
                  << ",\"trajectory_object\":\"rmf_traffic::Trajectory\""
                  << ",\"trajectory_point_count\":" << route_point_count
                  << ",\"start_time_s\":"
                  << json_optional_number(route_start_time)
                  << ",\"finish_time_s\":"
                  << json_optional_number(route_finish_time)
                  << ",\"duration_s\":";
    if (route_start_time.has_value() && route_finish_time.has_value())
      route_summary << *route_finish_time - *route_start_time;
    else
      route_summary << "null";
    route_summary << '}';
    writer.write(route_summary.str());

    std::size_t sequence = 0;
    for (const auto& trajectory_waypoint : route.trajectory())
    {
      ++trajectory_points;
      const auto position = trajectory_waypoint.position();
      const auto velocity = trajectory_waypoint.velocity();
      std::ostringstream line;
      line << std::setprecision(12)
           << "{\"event\":\"trajectory_point\",\"robot\":"
           << json_string(robot)
           << ",\"phase\":" << json_string(phase)
           << ",\"route_index\":" << route_index
           << ",\"sequence\":" << sequence++
           << ",\"map\":" << json_string(route.map())
           << ",\"time_s\":"
           << rmf_traffic::time::to_seconds(
                trajectory_waypoint.time().time_since_epoch())
           << ",\"x\":" << position[0]
           << ",\"y\":" << position[1]
           << ",\"yaw_rad\":" << position[2]
           << ",\"vx\":" << velocity[0]
           << ",\"vy\":" << velocity[1]
           << ",\"vyaw\":" << velocity[2]
           << ",\"object_type\":\"rmf_traffic::Trajectory::Waypoint\""
           << ",\"source_api\":\"Route::trajectory() const_iterator\""
           << '}';
      writer.write(line.str());
    }
  }

  std::sort(used_lanes.begin(), used_lanes.end());
  used_lanes.erase(
    std::unique(used_lanes.begin(), used_lanes.end()), used_lanes.end());

  const double finish_time = waypoints.empty()
    ? 0.0
    : rmf_traffic::time::to_seconds(waypoints.back().time().time_since_epoch());

  std::ostringstream summary;
  summary << std::setprecision(12)
          << "{\"event\":\"plan_summary\",\"robot\":"
          << json_string(robot)
          << ",\"phase\":" << json_string(phase)
          << ",\"success\":true,\"cost\":" << plan.get_cost()
          << ",\"ideal_cost\":" << json_optional_number(ideal_cost)
          << ",\"finish_time_s\":" << finish_time
          << ",\"plan_waypoint_count\":" << waypoints.size()
          << ",\"itinerary_route_count\":" << itinerary.size()
          << ",\"trajectory_point_count\":" << trajectory_points
          << ",\"planner_result_expansions\":";
  if (result_expansions.has_value())
    summary << *result_expansions;
  else
    summary << "null";
  summary << ",\"planner_result_nodes\":";
  if (result_nodes.has_value())
    summary << *result_nodes;
  else
    summary << "null";
  summary << ",\"used_lanes\":" << json_number_array(used_lanes) << '}';
  writer.write(summary.str());
  return used_lanes;
}

std::string negotiation_log_action(const std::string& message)
{
  std::string lower = message;
  std::transform(
    lower.begin(), lower.end(), lower.begin(),
    [](const unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (lower.find("selected table") != std::string::npos)
    return "select_table";
  if (lower.find("submitted plan") != std::string::npos)
    return "submit_plan";
  if (lower.find("rejected parent") != std::string::npos
    || lower.find("rejected") != std::string::npos)
  {
    return "reject";
  }
  if (lower.find("forfeited") != std::string::npos
    || lower.find("forfeit") != std::string::npos)
  {
    return "forfeit";
  }
  if (lower.find("skipping") != std::string::npos
    || lower.find("skipped") != std::string::npos)
  {
    return "skip";
  }
  if (lower.find("resolved") != std::string::npos
    || lower.find("finished") != std::string::npos)
  {
    return "resolve";
  }
  return "other";
}

void write_negotiation_log_event(
  JsonlWriter& writer,
  const std::string& message,
  const std::optional<std::size_t> stage = std::nullopt)
{
  std::ostringstream line;
  line << "{\"event\":\"negotiation_log\",\"source_api\":"
       << json_string("CentralizedNegotiation::Result::log")
       << ",\"actual_rmf_message\":true"
       << ",\"classification_source\":\"lab string classification; raw message preserved\""
       << ",\"action\":" << json_string(negotiation_log_action(message))
       << ",\"stage\":";
  if (stage.has_value())
    line << *stage;
  else
    line << "null";
  line << ",\"message\":" << json_string(message) << '}';
  writer.write(line.str());
}

void write_proposal_snapshot(
  JsonlWriter& writer,
  const std::string& phase,
  const std::map<rmf_traffic::schedule::ParticipantId, Plan>& proposal,
  const std::vector<rmf_traffic::schedule::Participant>& participants,
  const std::optional<std::size_t> stage = std::nullopt)
{
  std::ostringstream summary;
  summary << "{\"event\":\"proposal_summary\",\"phase\":"
          << json_string(phase) << ",\"stage\":";
  if (stage.has_value())
    summary << *stage;
  else
    summary << "null";
  summary << ",\"present\":true,"
          << "\"source_api\":\"CentralizedNegotiation::Result::proposal\","
          << "\"participant_plan_count\":" << proposal.size()
          << ",\"commit_state\":\"not_yet_committed\"}";
  writer.write(summary.str());

  for (const auto& [participant_id, plan] : proposal)
  {
    const auto participant_it = std::find_if(
      participants.begin(), participants.end(),
      [participant_id](const auto& participant)
      {
        return participant.id() == participant_id;
      });
    const std::string name = participant_it == participants.end()
      ? "unknown"
      : participant_it->description().name();
    std::size_t point_count = 0;
    for (const auto& route : plan.get_itinerary())
      point_count += route.trajectory().size();
    const auto& waypoints = plan.get_waypoints();
    std::optional<double> finish_time;
    if (!waypoints.empty())
    {
      finish_time = rmf_traffic::time::to_seconds(
        waypoints.back().time().time_since_epoch());
    }
    std::ostringstream line;
    line << std::setprecision(12)
         << "{\"event\":\"proposal_plan\",\"phase\":"
         << json_string(phase)
         << ",\"stage\":";
    if (stage.has_value())
      line << *stage;
    else
      line << "null";
    line
         << ",\"participant_id\":" << participant_id
         << ",\"robot\":" << json_string(name)
         << ",\"plan_object\":\"rmf_traffic::agv::Plan\""
         << ",\"source_api\":\"CentralizedNegotiation::Result::proposal\""
         << ",\"cost\":" << plan.get_cost()
         << ",\"waypoint_count\":" << waypoints.size()
         << ",\"itinerary_route_count\":" << plan.get_itinerary().size()
         << ",\"trajectory_point_count\":" << point_count
         << ",\"finish_time_s\":" << json_optional_number(finish_time)
         << ",\"validated\":false,\"committed\":false}";
    writer.write(line.str());
  }
}

struct CandidatePath
{
  std::vector<std::size_t> waypoints;
  std::vector<std::size_t> lanes;
  double distance = 0.0;
  bool feasible = false;
  double cost = std::numeric_limits<double>::infinity();
  double finish_time = 0.0;
};

void enumerate_path_dfs(
  const LabGraph& graph,
  const std::size_t current,
  const std::size_t goal,
  const std::set<std::size_t>& closed,
  std::vector<bool>& visited,
  CandidatePath& working,
  std::vector<CandidatePath>& output,
  const std::size_t max_paths)
{
  if (output.size() >= max_paths)
    return;

  if (current == goal)
  {
    output.push_back(working);
    return;
  }

  visited[current] = true;
  for (const auto lane_id : graph.graph.lanes_from(current))
  {
    if (closed.count(lane_id))
      continue;

    const auto& lane = graph.lanes.at(lane_id);
    if (visited[lane.exit])
      continue;

    working.lanes.push_back(lane_id);
    working.waypoints.push_back(lane.exit);
    working.distance += lane_length(graph, lane);
    enumerate_path_dfs(
      graph, lane.exit, goal, closed, visited, working, output, max_paths);
    working.distance -= lane_length(graph, lane);
    working.waypoints.pop_back();
    working.lanes.pop_back();
  }
  visited[current] = false;
}

std::vector<CandidatePath> enumerate_paths(
  const LabGraph& graph,
  const std::size_t start,
  const std::size_t goal,
  const std::vector<std::size_t>& closed_lanes,
  const std::size_t max_paths = 64)
{
  std::vector<CandidatePath> paths;
  CandidatePath working;
  working.waypoints.push_back(start);
  std::vector<bool> visited(graph.nodes.size(), false);
  enumerate_path_dfs(
    graph, start, goal, make_closed_set(closed_lanes), visited,
    working, paths, max_paths);
  return paths;
}

void write_solution_diagnosis(
  JsonlWriter& writer,
  const std::string& status,
  const std::string& category,
  const std::string& confidence,
  const std::string& basis,
  const std::string& root_cause,
  const std::vector<std::string>& evidence,
  const std::vector<std::string>& actions)
{
  writer.write(
    "{\"event\":\"solution_diagnosis\",\"status\":"
    + json_string(status)
    + ",\"category\":" + json_string(category)
    + ",\"confidence\":" + json_string(confidence)
    + ",\"basis\":" + json_string(basis)
    + ",\"root_cause\":" + json_string(root_cause)
    + ",\"evidence\":" + json_string_array(evidence)
    + ",\"recommended_actions\":" + json_string_array(actions)
    + "}");
}

std::size_t holding_point_count(const LabGraph& graph)
{
  std::size_t count = 0;
  for (std::size_t i = 0; i < graph.nodes.size(); ++i)
  {
    if (graph.graph.get_waypoint(i).is_holding_point())
      ++count;
  }
  return count;
}

std::size_t parking_point_count(const LabGraph& graph)
{
  std::size_t count = 0;
  for (std::size_t i = 0; i < graph.nodes.size(); ++i)
  {
    if (graph.graph.get_waypoint(i).is_parking_spot())
      ++count;
  }
  return count;
}

std::size_t endpoint_swap_count(const std::vector<RobotRequest>& requests)
{
  std::size_t count = 0;
  for (std::size_t a = 0; a < requests.size(); ++a)
  {
    for (std::size_t b = a + 1; b < requests.size(); ++b)
    {
      if (requests[a].start == requests[b].goal
        && requests[a].goal == requests[b].start)
      {
        ++count;
      }
    }
  }
  return count;
}

void diagnose_negotiation_result(
  JsonlWriter& writer,
  const LabGraph& graph,
  const std::vector<RobotRequest>& requests,
  const std::vector<std::size_t>& closed_lanes,
  const std::vector<bool>& baseline_success,
  const bool proposal_found,
  const bool safety_passed)
{
  std::vector<std::size_t> path_counts;
  path_counts.reserve(requests.size());
  std::vector<double> request_start_times;
  request_start_times.reserve(requests.size());
  std::size_t robots_without_path = 0;
  std::size_t robots_with_alternatives = 0;
  for (std::size_t i = 0; i < requests.size(); ++i)
  {
    const auto count = enumerate_paths(
      graph, requests[i].start, requests[i].goal, closed_lanes).size();
    path_counts.push_back(count);
    request_start_times.push_back(requests[i].start_time_s);
    if (count == 0 || (i < baseline_success.size() && !baseline_success[i]))
      ++robots_without_path;
    if (count > 1)
      ++robots_with_alternatives;
  }

  const auto swaps = endpoint_swap_count(requests);
  const auto holding = holding_point_count(graph);
  const auto parking = parking_point_count(graph);
  const std::vector<std::string> common_evidence = {
    "robots=" + std::to_string(requests.size()),
    "requested_start_times_s=" + json_number_array(request_start_times),
    "simple_path_counts=" + json_number_array(path_counts),
    "robots_with_alternate_paths=" + std::to_string(robots_with_alternatives),
    "holding_points=" + std::to_string(holding),
    "parking_points=" + std::to_string(parking),
    "exact_endpoint_swap_pairs=" + std::to_string(swaps),
    "closed_lanes=" + json_number_array(closed_lanes)};

  if (proposal_found && safety_passed)
  {
    write_solution_diagnosis(
      writer, "solved", "executable_time_space_plan", "high",
      "confirmed_by_rmf_and_detect_conflict",
      "CentralizedNegotiation returned a proposal and every route pair passed continuous-time conflict detection",
      common_evidence,
      {"Use this JSONL as the solved baseline for before/after comparison"});
    return;
  }

  if (proposal_found && !safety_passed)
  {
    write_solution_diagnosis(
      writer, "no_solution", "continuous_time_overlap", "high",
      "confirmed_by_rmf_detect_conflict",
      "A proposal was generated, but robot footprints overlap in continuous time, so it is unsafe to execute",
      common_evidence,
      {
        "Increase temporal separation or add a holding point before the shared resource",
        "Add a physically separated alternate lane or passing bay",
        "Do not commit the proposal until DetectConflict passes"});
    return;
  }

  if (robots_without_path > 0)
  {
    write_solution_diagnosis(
      writer, "no_solution", "individual_path_missing", "high",
      "confirmed_by_individual_rmf_planner",
      "At least one robot cannot reach its goal even when planned alone; negotiation cannot repair disconnected or directionally unreachable topology",
      common_evidence,
      {
        "Add or reverse the required directed lane between the disconnected components",
        "Reopen any closed lane that removed the only route",
        "Move the robot start or goal onto the same reachable graph component"});
    return;
  }

  const bool every_robot_has_one_path = std::all_of(
    path_counts.begin(), path_counts.end(),
    [](const std::size_t count) { return count == 1; });
  if (swaps > 0 && every_robot_has_one_path)
  {
    write_solution_diagnosis(
      writer, "no_solution", "endpoint_exchange_without_buffer", "high",
      "topology_inference_from_confirmed_no_proposal",
      "Robots must exchange occupied endpoints through one route, but the graph has no independent buffer or alternate path where one robot can vacate and wait",
      common_evidence,
      {
        "Split each shared endpoint into separate start, goal and corridor-gate nodes",
        "Add a side bay node connected to two corridor nodes so it forms an actual alternate path",
        "Mark staging or bay nodes as holding/parking points outside the bottleneck",
        "Remove one robot or dispatch the requests in separate time windows to verify the resource-capacity cause"});
    return;
  }

  if (every_robot_has_one_path && robots_with_alternatives == 0)
  {
    write_solution_diagnosis(
      writer, "no_solution", "single_route_no_yield_space", "medium_high",
      "topology_inference_from_confirmed_no_proposal",
      "Every robot is individually reachable, but all traffic is forced through one route and the graph offers no alternate topology for yielding",
      common_evidence,
      {
        "Add a passing loop: a side node must connect to two different corridor nodes",
        "Add holding points before entering the shared narrow section",
        "Reduce simultaneous robots to identify the minimum unsatisfiable set"});
    return;
  }

  write_solution_diagnosis(
    writer, "no_solution", "negotiation_no_proposal", "medium",
    "confirmed_no_proposal_with_structural_inference",
    "All robots have individual paths, but CentralizedNegotiation could not combine them into one conflict-free time-space proposal under the current topology and negotiation limits",
    common_evidence,
    {
      "Inspect the unfiltered negotiation log for rejected tables and submitted plans",
      "Add holding points before shared mutex or corridor segments",
      "Add a geometrically separate alternate route or passing bay",
      "Remove robots one at a time to find the minimum conflicting subset",
      "If topology is sufficient, compare negotiator cost leeway, threshold and search saturation before and after code changes"});
}

void write_candidate_paths(
  JsonlWriter& writer,
  const std::string& robot,
  const LabGraph& graph,
  const rmf_traffic::agv::VehicleTraits& traits,
  const RobotRequest& request,
  const std::vector<std::size_t>& global_closed,
  const std::vector<std::size_t>& selected_lanes)
{
  auto candidates = enumerate_paths(
    graph, request.start, request.goal, global_closed);
  const auto initial_time = request_start_time(request);

  for (auto& candidate : candidates)
  {
    std::set<std::size_t> allowed(candidate.lanes.begin(), candidate.lanes.end());
    std::vector<std::size_t> forced_closed = global_closed;
    for (const auto& lane : graph.lanes)
    {
      if (!allowed.count(lane.id))
        forced_closed.push_back(lane.id);
    }

    Planner forced_planner(
      make_configuration(graph, traits, forced_closed), make_planner_options());
    auto result = forced_planner.plan(
      Start(initial_time, request.start, request.yaw), Goal(request.goal));
    if (result)
    {
      candidate.feasible = true;
      candidate.cost = result->get_cost();
      const auto& waypoints = result->get_waypoints();
      candidate.finish_time = waypoints.empty()
        ? 0.0
        : rmf_traffic::time::to_seconds(
            waypoints.back().time().time_since_epoch());
    }
  }

  std::stable_sort(
    candidates.begin(), candidates.end(),
    [](const CandidatePath& a, const CandidatePath& b)
    {
      if (a.feasible != b.feasible)
        return a.feasible > b.feasible;
      return a.cost < b.cost;
    });

  auto selected_sorted = selected_lanes;
  std::sort(selected_sorted.begin(), selected_sorted.end());
  std::optional<std::size_t> selected_rank;
  std::optional<double> selected_cost;
  std::optional<double> next_best_cost;
  for (std::size_t rank = 0; rank < candidates.size(); ++rank)
  {
    auto candidate_sorted = candidates[rank].lanes;
    std::sort(candidate_sorted.begin(), candidate_sorted.end());
    const bool selected = candidate_sorted == selected_sorted;
    if (selected)
    {
      selected_rank = rank + 1;
      if (candidates[rank].feasible)
        selected_cost = candidates[rank].cost;
    }
    else if (candidates[rank].feasible
      && (!next_best_cost.has_value()
      || candidates[rank].cost < *next_best_cost))
    {
      next_best_cost = candidates[rank].cost;
    }
    const double best_cost = candidates.empty() || !candidates.front().feasible
      ? 0.0
      : candidates.front().cost;
    std::ostringstream line;
    line << std::setprecision(12)
         << "{\"event\":\"route_candidate\",\"robot\":"
         << json_string(robot)
         << ",\"rank\":" << rank + 1
         << ",\"waypoints\":" << json_number_array(candidates[rank].waypoints)
         << ",\"lanes\":" << json_number_array(candidates[rank].lanes)
         << ",\"distance_m\":" << candidates[rank].distance
         << ",\"feasible\":" << (candidates[rank].feasible ? "true" : "false")
         << ",\"rmf_cost\":";
    if (candidates[rank].feasible)
      line << candidates[rank].cost;
    else
      line << "null";
    line << ",\"delta_from_best\":";
    if (candidates[rank].feasible)
      line << candidates[rank].cost - best_cost;
    else
      line << "null";
    line << ",\"finish_time_s\":";
    if (candidates[rank].feasible)
      line << candidates[rank].finish_time;
    else
      line << "null";
    line << ",\"selected_by_plan\":" << (selected ? "true" : "false")
         << ",\"evaluation_method\":"
         << json_string(
              "Each simple graph path is forced by closing all other lanes, then evaluated by the real RMF Planner")
         << '}';
    writer.write(line.str());
  }

  std::ostringstream explanation;
  explanation << std::setprecision(12)
              << "{\"event\":\"route_choice_explanation\",\"robot\":"
              << json_string(robot)
              << ",\"candidate_count\":" << candidates.size()
              << ",\"selected_rank\":";
  if (selected_rank.has_value())
    explanation << *selected_rank;
  else
    explanation << "null";
  explanation << ",\"selected_cost\":";
  if (selected_cost.has_value())
    explanation << *selected_cost;
  else
    explanation << "null";
  explanation << ",\"next_best_cost\":";
  if (next_best_cost.has_value())
    explanation << *next_best_cost;
  else
    explanation << "null";
  explanation << ",\"cost_margin\":";
  if (selected_cost.has_value() && next_best_cost.has_value())
    explanation << *next_best_cost - *selected_cost;
  else
    explanation << "null";
  explanation << ",\"reason\":"
              << json_string(
                   "The selected free-flow route is the first optimal A* solution; candidate rows make the cost difference independently visible")
              << ",\"caveat\":"
              << json_string(
                   "Candidate enumeration is a lab diagnostic for simple paths, not RMF Planner's internal branching algorithm")
              << '}';
  writer.write(explanation.str());
}

void write_planning_request(
  JsonlWriter& writer,
  const RobotRequest& request,
  const std::string& mode)
{
  std::ostringstream line;
  line << std::setprecision(12)
       << "{\"event\":\"planning_request\",\"robot\":"
       << json_string(request.name)
       << ",\"mode\":" << json_string(mode)
       << ",\"start\":" << request.start
       << ",\"goal\":" << request.goal
       << ",\"start_object_type\":\"rmf_traffic::agv::Plan::Start\""
       << ",\"goal_object_type\":\"rmf_traffic::agv::Plan::Goal\""
       << ",\"start_yaw_rad\":" << request.yaw
       << ",\"start_time_s\":" << request.start_time_s
       << ",\"insertion_time_s\":" << request.insertion_time_s
       << ",\"effective_plan_time_s\":"
       << std::max(request.start_time_s, request.insertion_time_s)
       << ",\"goal_orientation_constraint\":null"
       << ",\"goal_any_orientation\":true"
       << ",\"start_source\":\"scenario request converted directly into Plan::Start\""
       << ",\"goal_source\":\"scenario request converted directly into Plan::Goal\""
       << '}';
  writer.write(line.str());
}

void write_schedule_state(
  JsonlWriter& writer,
  const std::string& phase,
  const std::shared_ptr<rmf_traffic::schedule::Database>& database,
  const std::vector<rmf_traffic::schedule::Participant>& participants);

int run_free_flow(
  const std::string& scenario,
  const std::string& description,
  LabGraph lab_graph,
  const RobotRequest& request,
  const std::vector<std::size_t>& closed_lanes,
  const std::vector<std::size_t>& expected_any_lane,
  const bool expect_success,
  JsonlWriter& writer,
  const bool enforce_expectation = true)
{
  write_graph_and_configuration(
    writer, scenario, description, lab_graph, closed_lanes, 1);
  const auto profile = make_profile();
  const auto traits = make_traits(profile);
  Planner planner(
    make_configuration(lab_graph, traits, closed_lanes), make_planner_options());
  const auto initial_time = request_start_time(request);
  const Start start(initial_time, request.start, request.yaw);
  const Goal goal(request.goal);
  write_planning_request(writer, request, "free_flow");
  configure_policy_snapshot(
    writer, lab_graph, nullptr, 0,
    std::max(request.start_time_s, request.insertion_time_s),
    "free_flow_planner_invocation");
  const auto trace = trace_real_search(
    writer, request.name, planner, start, goal,
    lab_graph, closed_lanes, request.goal);

  const auto begin = std::chrono::steady_clock::now();
  auto result = planner.plan(start, goal);
  const auto elapsed = std::chrono::steady_clock::now() - begin;
  const double planning_ms = 1000.0 * rmf_traffic::time::to_seconds(elapsed);

  if (!result)
  {
    std::ostringstream line;
    line << "{\"event\":\"plan_summary\",\"robot\":"
         << json_string(request.name)
         << ",\"phase\":\"free_flow\",\"success\":false"
         << ",\"disconnected\":" << (result.disconnected() ? "true" : "false")
         << ",\"saturated\":" << (result.saturated() ? "true" : "false")
         << ",\"interrupted\":" << (result.interrupted() ? "true" : "false")
         << ",\"ideal_cost\":" << json_optional_number(result.ideal_cost())
         << ",\"planner_result_expansions\":"
         << Planner::Debug::expansion_count(result)
         << ",\"planner_result_nodes\":" << Planner::Debug::node_count(result)
         << '}';
    writer.write(line.str());
    write_candidate_paths(
      writer, request.name, lab_graph, traits, request, closed_lanes, {});
    const bool passed = enforce_expectation ? !expect_success : true;
    writer.write(
      std::string("{\"event\":\"expectation\",\"passed\":")
      + (passed ? "true" : "false")
      + ",\"rule\":"
      + json_string(
          enforce_expectation
          ? "planner result matches the built-in scenario expectation"
          : "custom exploratory run: solution and no-solution are both valid observations")
      + "}");
    std::string category = "planner_no_solution";
    std::string cause =
      "The RMF Planner did not return a route; inspect connectivity, closures and search status";
    std::vector<std::string> actions = {
      "Inspect the raw A* termination and candidate-path events",
      "Add a directed lane or move start/goal if the graph is unreachable"};
    if (result.disconnected())
    {
      category = "disconnected_topology";
      cause =
        "Start and goal are not connected by any open directed path, so A* cannot generate a route";
      actions = {
        "Add a directed lane that connects the start component to the goal component",
        "Add the reverse direction when bidirectional travel is intended",
        "Reopen a closed lane if it removed the only connection"};
    }
    else if (result.saturated())
    {
      category = "search_saturation";
      cause = "The planner reached its search saturation limit before proving a solution";
      actions = {
        "Reduce graph branching or robot constraints to isolate the explosion",
        "Increase saturation only after checking for repeated or unnecessary states"};
    }
    else if (result.interrupted())
    {
      category = "planner_interrupted";
      cause = "Planning was interrupted before a solution was completed";
      actions = {"Remove the interrupt condition and rerun the same scenario"};
    }
    write_solution_diagnosis(
      writer, "no_solution", category, "high", "confirmed_by_rmf_planner_result",
      cause,
      {
        "disconnected=" + std::string(result.disconnected() ? "true" : "false"),
        "saturated=" + std::string(result.saturated() ? "true" : "false"),
        "interrupted=" + std::string(result.interrupted() ? "true" : "false"),
        "closed_lanes=" + json_number_array(closed_lanes),
        "search_expansions=" + std::to_string(trace.expansions)},
      actions);
    std::cout << "Scenario: " << scenario << '\n'
              << "Plan: NO SOLUTION\n"
              << "Disconnected: " << (result.disconnected() ? "YES" : "NO") << '\n'
              << "Search expansions: " << trace.expansions << '\n';
    return passed ? 0 : 2;
  }

  const auto used_lanes = write_plan(
    writer, request.name, *result, "free_flow", result.ideal_cost(),
    Planner::Debug::expansion_count(result), Planner::Debug::node_count(result));
  write_candidate_paths(
    writer, request.name, lab_graph, traits, request, closed_lanes, used_lanes);
  {
    std::ostringstream line;
    line << std::setprecision(12)
         << "{\"event\":\"planner_timing\",\"robot\":"
         << json_string(request.name)
         << ",\"elapsed_ms\":" << planning_ms << '}';
    writer.write(line.str());
  }

  // A real Fleet Adapter would publish this accepted single-robot itinerary
  // to the schedule as well. Keep planning isolated, then record the exact
  // Database state using the same Participant::set API as multi-robot runs.
  auto database = std::make_shared<rmf_traffic::schedule::Database>();
  writer.write(
    "{\"event\":\"schedule_database_operation\","
    "\"action\":\"construct\","
    "\"api\":\"std::make_shared<rmf_traffic::schedule::Database>\","
    "\"version_before\":null,\"version_after\":"
    + std::to_string(database->latest_version())
    + ",\"result\":\"empty database ready for free-flow itinerary\"}");
  std::vector<rmf_traffic::schedule::Participant> participants;
  const auto register_version_before = database->latest_version();
  participants.push_back(rmf_traffic::schedule::make_participant(
    {
      request.name,
      "lab_fleet",
      rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
      profile
    },
    database));
  auto& participant = participants.back();
  writer.write(
    "{\"event\":\"schedule_database_operation\","
    "\"action\":\"register_participant\","
    "\"api\":\"rmf_traffic::schedule::make_participant\","
    "\"participant_id\":" + std::to_string(participant.id())
    + ",\"name\":" + json_string(request.name)
    + ",\"version_before\":" + std::to_string(register_version_before)
    + ",\"version_after\":" + std::to_string(database->latest_version())
    + ",\"result\":\"participant description registered\"}");
  write_schedule_state(writer, "registered", database, participants);

  const auto database_version_before = database->latest_version();
  const auto itinerary_version_before = participant.version();
  const auto plan_id = participant.assign_plan_id();
  const bool accepted = participant.set(plan_id, result->get_itinerary());
  std::size_t point_count = 0;
  for (const auto& route : result->get_itinerary())
    point_count += route.trajectory().size();
  std::ostringstream schedule_operation;
  schedule_operation
    << "{\"event\":\"schedule_database_operation\","
    << "\"action\":\"set_itinerary\","
    << "\"api\":\"rmf_traffic::schedule::Participant::set\","
    << "\"participant_id\":" << participant.id()
    << ",\"name\":" << json_string(request.name)
    << ",\"plan_id\":" << plan_id
    << ",\"route_count\":" << result->get_itinerary().size()
    << ",\"trajectory_point_count\":" << point_count
    << ",\"itinerary_version_before\":" << itinerary_version_before
    << ",\"itinerary_version_after\":" << participant.version()
    << ",\"version_before\":" << database_version_before
    << ",\"version_after\":" << database->latest_version()
    << ",\"accepted\":" << (accepted ? "true" : "false")
    << ",\"result\":\"free-flow itinerary written to real schedule Database\"}";
  writer.write(schedule_operation.str());
  write_schedule_state(writer, "free_flow_committed", database, participants);

  bool expected_route = expected_any_lane.empty();
  for (const auto lane : expected_any_lane)
  {
    if (std::find(used_lanes.begin(), used_lanes.end(), lane) != used_lanes.end())
      expected_route = true;
  }
  const bool passed = enforce_expectation
    ? expect_success && expected_route
    : true;
  writer.write(
    std::string("{\"event\":\"expectation\",\"passed\":")
    + (passed ? "true" : "false")
    + ",\"rule\":"
    + json_string(
        enforce_expectation
        ? "solution exists and uses one of the expected diagnostic lanes"
        : "custom exploratory run: solution and no-solution are both valid observations")
    + "}");
  write_solution_diagnosis(
    writer, "solved", "free_flow_route_found", "high",
    "confirmed_by_rmf_planner",
    "The real RMF Planner returned a valid time-parameterized route",
    {
      "used_lanes=" + json_number_array(used_lanes),
      "search_expansions=" + std::to_string(trace.expansions),
      "plan_cost=" + std::to_string(result->get_cost())},
    {"Use this result as a baseline when changing the map or RMF planner code"});

  std::cout << "Scenario: " << scenario << '\n'
            << "Plan cost: " << result->get_cost() << '\n'
            << "Used lanes: " << json_number_array(used_lanes) << '\n'
            << "Planning time: " << planning_ms << " ms\n"
            << "Real A* expansions: " << trace.expansions << '\n'
            << "Expected route selected: " << (passed ? "YES" : "NO") << '\n';
  return passed ? 0 : 3;
}

void write_schedule_state(
  JsonlWriter& writer,
  const std::string& phase,
  const std::shared_ptr<rmf_traffic::schedule::Database>& database,
  const std::vector<rmf_traffic::schedule::Participant>& participants)
{
  const auto participant_ids = database->participant_ids();
  const auto database_view = database->query(rmf_traffic::schedule::query_all());
  std::ostringstream state;
  state << "{\"event\":\"schedule_database_state\",\"phase\":"
        << json_string(phase)
        << ",\"latest_version\":" << database->latest_version()
        << ",\"participant_count\":" << participant_ids.size()
        << ",\"client_handle_count\":" << participants.size()
        << ",\"stored_route_count\":" << database_view.size()
        << ",\"participant_ids\":[";
  bool first_id = true;
  for (const auto participant_id : participant_ids)
  {
    if (!first_id)
      state << ',';
    first_id = false;
    state << participant_id;
  }
  state << "],\"storage\":\"in_memory\"," 
        << "\"navigation_graph_stored_here\":false,"
        << "\"database_class\":\"rmf_traffic::schedule::Database\","
        << "\"view_class\":\"rmf_traffic::schedule::Viewer::View\","
        << "\"read_api\":\"Database::query(query_all) + Database::get_itinerary\","
        << "\"snapshot_representation\":\"flattened JSONL projection of actual in-memory objects\","
        << "\"meaning\":"
        << json_string(
             "Exact routes currently returned by the real RMF schedule Database; the navigation Graph is separate")
        << '}';
  writer.write(state.str());

  for (const auto participant_id : participant_ids)
  {
    const auto description = database->get_participant(participant_id);
    const auto itinerary = database->get_itinerary(participant_id);
    const auto current_plan_id = database->get_current_plan_id(participant_id);
    const std::size_t route_count = itinerary.has_value() ? itinerary->size() : 0;
    std::size_t trajectory_points = 0;
    if (itinerary.has_value())
    {
      for (const auto& route : *itinerary)
        trajectory_points += route->trajectory().size();
    }

    std::ostringstream line;
    line << "{\"event\":\"schedule_participant\",\"phase\":"
         << json_string(phase)
         << ",\"participant_id\":" << participant_id
         << ",\"name\":"
         << json_string(description ? description->name() : "unknown")
         << ",\"owner\":"
         << json_string(description ? description->owner() : "unknown")
         << ",\"responsive\":"
         << (description
             && description->responsiveness()
             == rmf_traffic::schedule::ParticipantDescription::Rx::Responsive
           ? "true" : "false")
         << ",\"profile_footprint\":\"Circle\""
         << ",\"profile_radius_m\":" << RobotRadius
         << ",\"itinerary_version\":"
         << database->itinerary_version(participant_id)
         << ",\"progress_version\":"
         << database->get_current_progress_version(participant_id)
         << ",\"cumulative_delay_s\":"
         << ActiveCumulativeDelay[participant_id]
         << ",\"cumulative_delay_source\":\"Participant::delay call recorded by lab; shifted trajectory is read back from SCHEDULE\""
         << ",\"reached_checkpoint\":null"
         << ",\"reached_checkpoint_source\":\"UNAVAILABLE_WITHOUT_FLEET_ADAPTER_PROGRESS_FEEDBACK\""
         << ",\"current_plan_id\":";
    if (current_plan_id.has_value())
      line << *current_plan_id;
    else
      line << "null";
    line << ",\"itinerary_present\":"
         << (itinerary.has_value() ? "true" : "false")
         << ",\"route_count\":" << route_count
         << ",\"trajectory_point_count\":" << trajectory_points
         << ",\"description_read_api\":\"Database::get_participant\""
         << ",\"itinerary_read_api\":\"Database::get_itinerary\""
         << ",\"plan_id_read_api\":\"Database::get_current_plan_id\""
         << ",\"read_from\":\"rmf_traffic::schedule::Database\""
         << '}';
    writer.write(line.str());
  }

  // Read every stored route through the Database Viewer API. These are not
  // reconstructed from Planner output or the local Participant handles.
  for (const auto& element : database_view)
  {
    const auto& route = *element.route;
    std::size_t sequence = 0;
    std::optional<double> start_time;
    std::optional<double> finish_time;
    for (const auto& point : route.trajectory())
    {
      const double time_s = rmf_traffic::time::to_seconds(
        point.time().time_since_epoch());
      if (!start_time.has_value())
        start_time = time_s;
      finish_time = time_s;

      const auto position = point.position();
      const auto velocity = point.velocity();
      std::ostringstream point_line;
      point_line << std::setprecision(12)
                 << "{\"event\":\"schedule_database_trajectory_point\",\"phase\":"
                 << json_string(phase)
                 << ",\"participant_id\":" << element.participant
                 << ",\"name\":" << json_string(element.description.name())
                 << ",\"plan_id\":" << element.plan_id
                 << ",\"route_id\":" << element.route_id
                 << ",\"sequence\":" << sequence++
                 << ",\"map\":" << json_string(route.map())
                 << ",\"time_s\":" << time_s
                 << ",\"x\":" << position[0]
                 << ",\"y\":" << position[1]
                 << ",\"yaw_rad\":" << position[2]
                 << ",\"vx\":" << velocity[0]
                 << ",\"vy\":" << velocity[1]
                 << ",\"vyaw\":" << velocity[2]
                 << ",\"object_path\":\"Viewer::View::Element.route -> Route::trajectory -> Trajectory::const_iterator\""
                 << ",\"read_from\":\"Database::query(query_all)\"}";
      writer.write(point_line.str());
    }

    std::ostringstream route_line;
    route_line << std::setprecision(12)
               << "{\"event\":\"schedule_database_route\",\"phase\":"
               << json_string(phase)
               << ",\"participant_id\":" << element.participant
               << ",\"name\":" << json_string(element.description.name())
               << ",\"plan_id\":" << element.plan_id
               << ",\"route_id\":" << element.route_id
               << ",\"map\":" << json_string(route.map())
               << ",\"trajectory_point_count\":" << sequence
               << ",\"start_time_s\":";
    if (start_time.has_value())
      route_line << *start_time;
    else
      route_line << "null";
    route_line << ",\"finish_time_s\":";
    if (finish_time.has_value())
      route_line << *finish_time;
    else
      route_line << "null";
    route_line << ",\"duration_s\":";
    if (start_time.has_value() && finish_time.has_value())
      route_line << (*finish_time - *start_time);
    else
      route_line << "null";
    route_line << ",\"object_path\":\"Viewer::View::Element -> Route(map, Trajectory)\""
               << ",\"read_from\":\"Database::query(query_all)\"}";
    writer.write(route_line.str());
  }
}

bool validate_plan_with_schedule_route_validator(
  JsonlWriter& writer,
  const std::string& phase,
  const std::shared_ptr<rmf_traffic::schedule::Database>& database,
  const rmf_traffic::schedule::ParticipantId participant_id,
  const Plan& plan,
  const rmf_traffic::Profile& profile)
{
  rmf_traffic::agv::ScheduleRouteValidator validator(
    *database, participant_id, profile);
  bool passed = true;
  std::size_t candidate_route_id = 0;
  for (const auto& route : plan.get_itinerary())
  {
    const auto conflict = validator.find_conflict(route);
    std::ostringstream line;
    line << std::setprecision(12)
         << "{\"event\":\"route_validator_result\",\"source\":\"RMF_CORE\","
         << "\"schedule_source\":\"SCHEDULE\",\"phase\":"
         << json_string(phase)
         << ",\"validator\":\"ScheduleRouteValidator\","
         << "\"actual_call\":true,\"participant_id\":" << participant_id
         << ",\"candidate_route_id\":" << candidate_route_id
         << ",\"decision\":";
    if (conflict.has_value())
    {
      passed = false;
      line << "\"ROUTE_VALIDATOR_CONFLICT\","
           << "\"reason_code\":\"SPACETIME_PROFILE_CONFLICT\","
           << "\"blocker_participant\":"
           << conflict->dependency.on_participant
           << ",\"blocker_plan_id\":" << conflict->dependency.on_plan
           << ",\"blocker_route_id\":" << conflict->dependency.on_route
           << ",\"blocker_checkpoint\":"
           << conflict->dependency.on_checkpoint
           << ",\"conflict_time_s\":"
           << rmf_traffic::time::to_seconds(
             conflict->time.time_since_epoch());
    }
    else
    {
      line << "\"ACCEPT\",\"reason_code\":\"NO_SCHEDULE_CONFLICT\"";
    }
    line << ",\"api\":\"ScheduleRouteValidator::find_conflict(Route)\"}";
    writer.write(line.str());
    ++candidate_route_id;
  }
  return passed;
}

struct SafetyVerification
{
  bool passed = true;
  std::size_t robot_pairs = 0;
  std::size_t route_pairs = 0;
  std::size_t conflicts = 0;
};

SafetyVerification verify_negotiated_plans(
  JsonlWriter& writer,
  const std::map<rmf_traffic::schedule::ParticipantId, Plan>& plans,
  const std::vector<rmf_traffic::schedule::Participant>& participants,
  const rmf_traffic::Profile& profile)
{
  SafetyVerification verification;
  std::vector<std::pair<rmf_traffic::schedule::ParticipantId, const Plan*>> ordered;
  ordered.reserve(plans.size());
  for (const auto& [participant_id, plan] : plans)
    ordered.emplace_back(participant_id, &plan);

  const auto participant_name = [&participants](
    const rmf_traffic::schedule::ParticipantId id)
  {
    const auto it = std::find_if(
      participants.begin(), participants.end(),
      [id](const auto& participant) { return participant.id() == id; });
    return it == participants.end()
      ? std::string("participant_") + std::to_string(id)
      : it->description().name();
  };

  for (std::size_t a = 0; a < ordered.size(); ++a)
  {
    for (std::size_t b = a + 1; b < ordered.size(); ++b)
    {
      ++verification.robot_pairs;
      bool pair_passed = true;
      std::optional<double> earliest_conflict_time;
      std::size_t pair_route_checks = 0;
      for (const auto& route_a : ordered[a].second->get_itinerary())
      {
        for (const auto& route_b : ordered[b].second->get_itinerary())
        {
          if (route_a.map() != route_b.map())
            continue;

          ++pair_route_checks;
          ++verification.route_pairs;
          const auto conflict = rmf_traffic::DetectConflict::between(
            profile, route_a.trajectory(), nullptr,
            profile, route_b.trajectory(), nullptr);
          if (!conflict.has_value())
            continue;

          pair_passed = false;
          ++verification.conflicts;
          const double conflict_time = rmf_traffic::time::to_seconds(
            conflict->time.time_since_epoch());
          if (!earliest_conflict_time.has_value()
            || conflict_time < *earliest_conflict_time)
          {
            earliest_conflict_time = conflict_time;
          }
        }
      }

      verification.passed = verification.passed && pair_passed;
      std::ostringstream line;
      line << std::setprecision(12)
           << "{\"event\":\"pairwise_conflict_check\",\"robot_a\":"
           << json_string(participant_name(ordered[a].first))
           << ",\"robot_b\":"
           << json_string(participant_name(ordered[b].first))
           << ",\"passed\":" << (pair_passed ? "true" : "false")
           << ",\"route_pair_checks\":" << pair_route_checks
           << ",\"earliest_conflict_time_s\":";
      if (earliest_conflict_time.has_value())
        line << *earliest_conflict_time;
      else
        line << "null";
      line << ",\"method\":\"rmf_traffic::DetectConflict::between\"}";
      writer.write(line.str());
    }
  }

  std::ostringstream summary;
  summary << "{\"event\":\"safety_verification\",\"passed\":"
          << (verification.passed ? "true" : "false")
          << ",\"executable_plan\":"
          << (verification.passed ? "true" : "false")
          << ",\"robot_pairs\":" << verification.robot_pairs
          << ",\"route_pair_checks\":" << verification.route_pairs
          << ",\"conflicts\":" << verification.conflicts
          << ",\"method\":\"rmf_traffic::DetectConflict::between\""
          << ",\"required_center_distance_m\":" << 2.0 * RobotRadius
          << '}';
  writer.write(summary.str());
  writer.write(
    std::string("{\"event\":\"process_phase\",\"phase\":\"safety_checked\",\"order\":4,")
    + "\"label\":"
    + json_string(
      verification.passed
        ? "All negotiated route pairs passed RMF continuous-time conflict detection"
        : "Negotiated proposal rejected because RMF conflict detection found an overlap")
    + "}");
  return verification;
}

int run_negotiation(
  const std::string& scenario,
  const std::string& description,
  LabGraph lab_graph,
  const std::vector<RobotRequest>& requests,
  JsonlWriter& writer,
  const std::vector<std::size_t>& closed_lanes = {})
{
  using Negotiation = rmf_traffic::agv::CentralizedNegotiation;
  write_graph_and_configuration(
    writer, scenario, description, lab_graph, closed_lanes, requests.size());

  auto database = std::make_shared<rmf_traffic::schedule::Database>();
  writer.write(
    "{\"event\":\"schedule_database_operation\","
    "\"action\":\"construct\","
    "\"api\":\"std::make_shared<rmf_traffic::schedule::Database>\","
    "\"version_before\":null,\"version_after\":"
    + std::to_string(database->latest_version())
    + ",\"result\":\"empty database ready\"}");
  const auto profile = make_profile();
  const auto traits = make_traits(profile);
  auto planner = std::make_shared<Planner>(
    make_configuration(lab_graph, traits, closed_lanes), make_planner_options());
  std::vector<rmf_traffic::schedule::Participant> participants;
  participants.reserve(requests.size());
  for (const auto& request : requests)
  {
    const auto version_before = database->latest_version();
    participants.push_back(rmf_traffic::schedule::make_participant(
      {
        request.name,
        "lab_fleet",
        rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
        profile
      },
      database));
    const auto& participant = participants.back();
    std::ostringstream operation;
    operation << "{\"event\":\"schedule_database_operation\","
              << "\"action\":\"register_participant\","
              << "\"api\":\"rmf_traffic::schedule::make_participant\","
              << "\"participant_id\":" << participant.id()
              << ",\"name\":" << json_string(request.name)
              << ",\"version_before\":" << version_before
              << ",\"version_after\":" << database->latest_version()
              << ",\"result\":\"participant description registered\"}";
    writer.write(operation.str());
  }
  write_schedule_state(writer, "registered", database, participants);
  writer.write(
    "{\"event\":\"process_phase\",\"phase\":\"participants_registered\","
    "\"order\":2,\"label\":\"Robots registered in the real Schedule Database\"}");

  std::vector<Negotiation::Agent> agents;
  const bool lane_penalty_active = experimental_lane_penalty_active();
  const double maximum_cost_leeway = lane_penalty_active ? 100.0 : 10.0;
  const double minimum_cost_threshold = lane_penalty_active ? 10000.0 : 180.0;
  std::ostringstream request_event;
  request_event << "{\"event\":\"negotiation_request\","
                << "\"experimental_lane_penalty_active\":"
                << (lane_penalty_active ? "true" : "false")
                << ",\"maximum_cost_leeway\":" << maximum_cost_leeway
                << ",\"minimum_cost_threshold\":" << minimum_cost_threshold
                << ",\"robots\":[";
  for (std::size_t i = 0; i < requests.size(); ++i)
  {
    if (i > 0)
      request_event << ',';
    request_event << std::setprecision(12)
                  << "{\"name\":" << json_string(requests[i].name)
                  << ",\"participant_id\":" << participants[i].id()
                  << ",\"start\":" << requests[i].start
                  << ",\"goal\":" << requests[i].goal
                  << ",\"start_yaw_rad\":" << requests[i].yaw
                  << ",\"start_time_s\":" << requests[i].start_time_s
                  << ",\"insertion_time_s\":" << requests[i].insertion_time_s
                  << '}';
    auto negotiator_options = rmf_traffic::agv::SimpleNegotiator::Options();
    negotiator_options
      .maximum_cost_leeway(maximum_cost_leeway)
      .minimum_cost_threshold(minimum_cost_threshold);
    agents.emplace_back(
      participants[i].id(),
      Start(request_start_time(requests[i]), requests[i].start, requests[i].yaw),
      Goal(requests[i].goal),
      planner,
      std::move(negotiator_options));
  }
  request_event << "]}";
  writer.write(request_event.str());
  writer.write(
    "{\"event\":\"baseline_notice\",\"executable\":false,"
    "\"label\":\"Free-flow baselines ignore the other negotiating robots and are diagnostic only\"}");

  // Show what each robot would do if it were alone. Negotiation may alter
  // these plans because the schedule validator adds time-space constraints.
  std::vector<bool> baseline_success;
  baseline_success.reserve(requests.size());
  std::vector<std::vector<std::size_t>> baseline_used_lanes;
  baseline_used_lanes.reserve(requests.size());
  std::vector<std::optional<Plan>> baseline_plans;
  baseline_plans.reserve(requests.size());
  for (const auto& request : requests)
  {
    write_planning_request(writer, request, "free_flow_baseline");
    const Start start(request_start_time(request), request.start, request.yaw);
    trace_real_search(
      writer, request.name, *planner, start, Goal(request.goal),
      lab_graph, closed_lanes, request.goal);
    auto baseline = planner->plan(start, Goal(request.goal));
    baseline_success.push_back(static_cast<bool>(baseline));
    if (baseline)
    {
      baseline_plans.emplace_back(*baseline);
      const auto lanes = write_plan(
        writer, request.name, *baseline, "free_flow_baseline",
        baseline.ideal_cost(), Planner::Debug::expansion_count(baseline),
        Planner::Debug::node_count(baseline));
      baseline_used_lanes.push_back(lanes);
      write_candidate_paths(
        writer, request.name, lab_graph, traits, request, closed_lanes, lanes);
    }
    else
    {
      baseline_plans.emplace_back(std::nullopt);
      baseline_used_lanes.emplace_back();
      std::ostringstream failed;
      failed << "{\"event\":\"plan_summary\",\"robot\":"
             << json_string(request.name)
             << ",\"phase\":\"free_flow_baseline\",\"success\":false"
             << ",\"disconnected\":"
             << (baseline.disconnected() ? "true" : "false")
             << ",\"saturated\":"
             << (baseline.saturated() ? "true" : "false")
             << ",\"interrupted\":"
             << (baseline.interrupted() ? "true" : "false") << '}';
      writer.write(failed.str());
      write_candidate_paths(
        writer, request.name, lab_graph, traits, request, closed_lanes, {});
    }
  }

  // This uses routes returned by the unmodified free-flow RMF Planner, not
  // scenario lane annotations. The modified DifferentialDrivePlanner reads the
  // resulting occupancy demand when negotiation begins.
  configure_shared_corridor_penalty(
    writer, lab_graph, requests, baseline_used_lanes);
  std::vector<rmf_traffic::schedule::ParticipantId> participant_ids;
  participant_ids.reserve(participants.size());
  for (const auto& participant : participants)
    participant_ids.push_back(participant.id());
  const auto admission_reservations =
    make_deterministic_admission_reservations(
      lab_graph, baseline_plans, requests, participant_ids);
  configure_policy_snapshot(
    writer, lab_graph, database,
    participants.empty() ? 0 : participants.front().id(),
    requests.empty() ? 0.0
      : std::max(requests.front().start_time_s, requests.front().insertion_time_s),
    "centralized_negotiation_with_deterministic_admission",
    admission_reservations);

  writer.write(
    "{\"event\":\"process_phase\",\"phase\":\"negotiation_started\","
    "\"order\":3,\"label\":\"CentralizedNegotiation evaluates time-space conflicts and waits\"}");
  writer.write(
    "{\"event\":\"schedule_database_operation\","
    "\"action\":\"read_for_negotiation\","
    "\"api\":\"rmf_traffic::agv::CentralizedNegotiation(database).solve\","
    "\"version_before\":" + std::to_string(database->latest_version())
    + ",\"version_after\":" + std::to_string(database->latest_version())
    + ",\"result\":\"schedule state supplied to conflict-aware planning\"}");
  writer.write(
    "{\"event\":\"validator_configuration\"," 
    "\"source\":\"RMF_CORE\",\"schedule_source\":\"SCHEDULE\"," 
    "\"phase\":\"centralized_negotiation\"," 
    "\"validator\":\"NegotiatingRouteValidator\"," 
    "\"actual_internal_path\":true,\"per_call_result_observable\":false,"
    "\"planner_options_validator\":\"negotiation table constraints supplied internally by SimpleNegotiator\","
    "\"validator_object_publicly_exposed\":false,"
    "\"schedule_aware\":true,"
    "\"schedule_database_version\":" + std::to_string(database->latest_version())
    + ",\"agent_type\":\"rmf_traffic::agv::SimpleNegotiator\","
    "\"solver_type\":\"rmf_traffic::agv::CentralizedNegotiation\","
    "\"post_proposal_validator\":\"rmf_traffic::DetectConflict::between\","
    "\"important\":\"The public Result exposes proposal and raw log, not every internal RouteValidator call\"}");
  const auto begin = std::chrono::steady_clock::now();
  const auto result = Negotiation(database)
    .optimal(requests.size() <= 2)
    .log(true)
    .solve(agents);
  const auto elapsed = std::chrono::steady_clock::now() - begin;
  const double negotiation_ms = 1000.0 * rmf_traffic::time::to_seconds(elapsed);

  for (const auto& message : result.log())
    write_negotiation_log_event(writer, message);

  if (!result.proposal().has_value())
  {
    writer.write(
      "{\"event\":\"proposal_summary\",\"phase\":\"centralized_negotiation\","
      "\"present\":false,\"source_api\":\"CentralizedNegotiation::Result::proposal\","
      "\"participant_plan_count\":0,\"commit_state\":\"no_proposal\"}");
    writer.write(
      "{\"event\":\"proposal_outcome\",\"phase\":\"centralized_negotiation\","
      "\"action\":\"reject_no_proposal\",\"accepted\":false,\"committed\":false,"
      "\"reason\":\"Result::proposal returned nullopt; inspect raw log classifications for rejected, skipped or forfeited branches\"}");
    writer.write(
      "{\"event\":\"schedule_database_operation\","
      "\"action\":\"skip_commit\",\"api\":\"Participant::set\","
      "\"version_before\":" + std::to_string(database->latest_version())
      + ",\"version_after\":" + std::to_string(database->latest_version())
      + ",\"result\":\"no proposal; database itinerary remains empty\"}");
    writer.write(
      "{\"event\":\"safety_verification\",\"passed\":false,"
      "\"executable_plan\":false,\"reason\":\"no_negotiated_proposal\","
      "\"method\":\"rmf_traffic::DetectConflict::between\"}");
    std::ostringstream summary;
    summary << std::setprecision(12)
            << "{\"event\":\"negotiation_summary\",\"success\":false"
            << ",\"elapsed_ms\":" << negotiation_ms
            << ",\"schedule_version\":" << database->latest_version()
            << ",\"interpretation\":"
            << json_string(
                 "No conflict-free time-space proposal was found. Compare topology, free-flow plans, holding points and negotiation log; this alone is not proof of an RMF bug")
            << '}';
    writer.write(summary.str());
    write_schedule_state(writer, "no_proposal", database, participants);
    diagnose_negotiation_result(
      writer, lab_graph, requests, closed_lanes, baseline_success, false, false);
    std::cout << "Scenario: " << scenario << '\n'
              << "Negotiation result: NO PROPOSAL\n"
              << "Calculation time: " << negotiation_ms << " ms\n";
    return 0;
  }

  std::map<rmf_traffic::schedule::ParticipantId, Plan> ordered(
    result.proposal()->begin(), result.proposal()->end());
  write_proposal_snapshot(
    writer, "centralized_negotiation", ordered, participants);
  bool schedule_route_validator_passed = true;
  for (const auto& [participant_id, plan] : ordered)
  {
    schedule_route_validator_passed =
      validate_plan_with_schedule_route_validator(
        writer, "centralized_negotiation_post_proposal", database,
        participant_id, plan, profile)
      && schedule_route_validator_passed;
  }
  const auto safety = verify_negotiated_plans(
    writer, ordered, participants, profile);

  if (!safety.passed || !schedule_route_validator_passed)
  {
    writer.write(
      "{\"event\":\"proposal_outcome\",\"phase\":\"post_proposal_safety\","
      "\"action\":\"reject_after_detect_conflict\",\"accepted\":false,"
      "\"committed\":false,\"reason\":\"ScheduleRouteValidator or continuous-time DetectConflict rejected the proposal\"}");
    for (const auto& [participant_id, plan] : ordered)
    {
      const auto it = std::find_if(
        participants.begin(), participants.end(),
        [participant_id](const auto& p) { return p.id() == participant_id; });
      if (it != participants.end())
        write_plan(writer, it->description().name(), plan, "rejected_negotiated");
    }
    writer.write(
      "{\"event\":\"negotiation_summary\",\"success\":true,"
      "\"safety_verified\":false,\"executable_plan\":false,"
      "\"interpretation\":\"Proposal exists but failed independent RMF DetectConflict verification; it was not committed or shown as safe execution\"}");
    writer.write(
      "{\"event\":\"schedule_database_operation\","
      "\"action\":\"reject_commit\",\"api\":\"Participant::set\","
      "\"version_before\":" + std::to_string(database->latest_version())
      + ",\"version_after\":" + std::to_string(database->latest_version())
      + ",\"result\":\"DetectConflict failed; unsafe proposal not written\"}");
    write_schedule_state(
      writer, "proposal_rejected_by_safety_check", database, participants);
    diagnose_negotiation_result(
      writer, lab_graph, requests, closed_lanes, baseline_success, true, false);
    std::cout << "Scenario: " << scenario << '\n'
              << "Negotiation result: PROPOSAL REJECTED BY SAFETY CHECK\n"
              << "Detected conflicts: " << safety.conflicts << '\n';
    return 0;
  }

  std::size_t commit_index = 0;
  for (const auto& [participant_id, plan] : ordered)
  {
    const auto it = std::find_if(
      participants.begin(), participants.end(),
      [participant_id](const auto& p) { return p.id() == participant_id; });
    if (it == participants.end())
      continue;
    write_plan(writer, it->description().name(), plan, "negotiated");
    const auto database_version_before = database->latest_version();
    const auto itinerary_version_before = it->version();
    const auto plan_id = it->assign_plan_id();
    const bool accepted = it->set(plan_id, plan.get_itinerary());
    std::size_t point_count = 0;
    for (const auto& route : plan.get_itinerary())
      point_count += route.trajectory().size();
    writer.write(
      "{\"event\":\"schedule_commit\",\"participant_id\":"
      + std::to_string(participant_id)
      + ",\"name\":" + json_string(it->description().name())
      + ",\"plan_id\":" + std::to_string(plan_id)
      + ",\"accepted\":" + (accepted ? "true" : "false") + "}");
    std::ostringstream operation;
    operation << "{\"event\":\"schedule_database_operation\","
              << "\"action\":\"set_itinerary\","
              << "\"api\":\"rmf_traffic::schedule::Participant::set\","
              << "\"participant_id\":" << participant_id
              << ",\"name\":" << json_string(it->description().name())
              << ",\"plan_id\":" << plan_id
              << ",\"route_count\":" << plan.get_itinerary().size()
              << ",\"trajectory_point_count\":" << point_count
              << ",\"itinerary_version_before\":" << itinerary_version_before
              << ",\"itinerary_version_after\":" << it->version()
              << ",\"version_before\":" << database_version_before
              << ",\"version_after\":" << database->latest_version()
              << ",\"accepted\":" << (accepted ? "true" : "false")
              << ",\"result\":\"verified time-parameterized itinerary written\"}";
    writer.write(operation.str());
    ++commit_index;
    write_schedule_state(
      writer,
      "commit_" + std::to_string(commit_index)
      + "_of_" + std::to_string(ordered.size()),
      database,
      participants);
  }
  write_schedule_state(writer, "proposal_committed", database, participants);
  writer.write(
    "{\"event\":\"proposal_outcome\",\"phase\":\"schedule_commit\","
    "\"action\":\"accept_and_commit\",\"accepted\":true,\"committed\":true,"
    "\"reason\":\"Proposal passed pairwise DetectConflict verification and each Participant::set accepted its itinerary\"}");
  writer.write(
    "{\"event\":\"process_phase\",\"phase\":\"schedule_committed\","
    "\"order\":5,\"label\":\"Verified itineraries committed; execution may begin\"}");

  std::ostringstream summary;
  summary << std::setprecision(12)
          << "{\"event\":\"negotiation_summary\",\"success\":true"
          << ",\"safety_verified\":true,\"executable_plan\":true"
          << ",\"elapsed_ms\":" << negotiation_ms
          << ",\"proposal_plan_count\":" << ordered.size()
          << ",\"schedule_version\":" << database->latest_version()
          << ",\"interpretation\":"
          << json_string(
               "CentralizedNegotiation produced conflict-aware time-space plans and they were committed to the real in-memory schedule Database")
          << '}';
  writer.write(summary.str());
  diagnose_negotiation_result(
    writer, lab_graph, requests, closed_lanes, baseline_success, true, true);

  std::cout << "Scenario: " << scenario << '\n'
            << "Negotiation result: PROPOSAL FOUND\n"
            << "Plans: " << ordered.size() << '\n'
            << "Schedule DB version: " << database->latest_version() << '\n'
            << "Calculation time: " << negotiation_ms << " ms\n";
  return 0;
}

std::map<std::size_t, double> configure_newcomer_detour_penalty(
  JsonlWriter& writer,
  const LabGraph& graph,
  const std::map<rmf_traffic::schedule::ParticipantId,
    std::vector<std::size_t>>& committed_lanes,
  const std::vector<RobotRequest>& newcomers,
  const std::size_t stage,
  const double insertion_time_s)
{
  const char* raw_policy = std::getenv("RMF_TRAFFIC_LAB_DYNAMIC_POLICY");
  const std::string policy = raw_policy ? raw_policy : "fixed_existing";
  std::map<std::size_t, double> penalties;
  std::map<std::pair<std::size_t, std::size_t>, std::size_t> corridor_demand;
  std::map<std::string, std::size_t> mutex_demand;
  for (const auto& [participant, lanes] : committed_lanes)
  {
    (void)participant;
    std::set<std::pair<std::size_t, std::size_t>> unique_corridors;
    std::set<std::string> unique_mutexes;
    for (const auto lane_id : lanes)
    {
      if (lane_id >= graph.lanes.size())
        continue;
      const auto& lane = graph.lanes[lane_id];
      unique_corridors.insert({
        std::min(lane.entry, lane.exit), std::max(lane.entry, lane.exit)});
      if (!lane.mutex_group.empty())
        unique_mutexes.insert(lane.mutex_group);
    }
    for (const auto& corridor : unique_corridors)
      ++corridor_demand[corridor];
    for (const auto& mutex : unique_mutexes)
      ++mutex_demand[mutex];
  }

  const double weight = positive_environment_value(
    "RMF_TRAFFIC_LAB_NEWCOMER_PENALTY", 120.0);
  if (policy == "after_nego")
  {
    for (const auto& lane : graph.lanes)
    {
      const auto corridor = std::make_pair(
        std::min(lane.entry, lane.exit), std::max(lane.entry, lane.exit));
      const auto corridor_it = corridor_demand.find(corridor);
      const auto mutex_it = mutex_demand.find(lane.mutex_group);
      const std::size_t demand = std::max(
        corridor_it == corridor_demand.end() ? 0 : corridor_it->second,
        lane.mutex_group.empty() || mutex_it == mutex_demand.end()
          ? 0 : mutex_it->second);
      if (demand > 0)
        penalties[lane.id] = weight * static_cast<double>(demand);
    }
  }

  std::ostringstream specification;
  bool first = true;
  for (const auto& [lane, penalty] : penalties)
  {
    if (!first)
      specification << ',';
    first = false;
    specification << lane << ':' << std::setprecision(12) << penalty;
  }
  if (specification.str().empty())
    ::unsetenv("RMF_TRAFFIC_LAB_LANE_PENALTIES");
  else
    ::setenv(
      "RMF_TRAFFIC_LAB_LANE_PENALTIES",
      specification.str().c_str(), true);

  std::ostringstream event;
  event << std::setprecision(12)
        << "{\"event\":\"newcomer_penalty_configuration\","
        << "\"stage\":" << stage
        << ",\"insertion_time_s\":" << insertion_time_s
        << ",\"policy\":" << json_string(policy)
        << ",\"source\":\"committed_real_rmf_plan_lanes_and_mutex_groups\","
        << "\"weight\":" << weight
        << ",\"newcomers\":[";
  for (std::size_t i = 0; i < newcomers.size(); ++i)
  {
    if (i > 0)
      event << ',';
    event << json_string(newcomers[i].name);
  }
  event << "],\"committed_participant_count\":" << committed_lanes.size()
        << ",\"directed_lane_penalties\":{";
  bool first_penalty = true;
  for (const auto& [lane, penalty] : penalties)
  {
    if (!first_penalty)
      event << ',';
    first_penalty = false;
    event << json_string(std::to_string(lane)) << ':' << penalty;
  }
  event << "},\"environment_spec\":"
        << json_string(specification.str())
        << ",\"reason\":"
        << json_string(
             policy == "after_nego"
               ? "Only the newcomer batch is replanned; lanes and mutex groups already used by committed itineraries receive a soft A* g-cost so alternatives are preferred"
               : "Before policy keeps committed itineraries fixed but does not add a detour preference to newcomer A* cost")
        << '}';
  writer.write(event.str());

  std::ostringstream occupancy_event;
  occupancy_event << std::setprecision(12)
                  << "{\"event\":\"occupancy_penalty_configuration\","
                  << "\"source\":\"dynamic_committed_schedule_plan_lanes\","
                  << "\"mode\":" << json_string(policy)
                  << ",\"active\":" << (penalties.empty() ? "false" : "true")
                  << ",\"directed_lane_occupancy\":{},"
                  << "\"shared_corridor_users\":{},"
                  << "\"directed_lane_penalties\":{";
  first_penalty = true;
  for (const auto& [lane, penalty] : penalties)
  {
    if (!first_penalty)
      occupancy_event << ',';
    first_penalty = false;
    occupancy_event << json_string(std::to_string(lane)) << ':' << penalty;
  }
  occupancy_event << "}}";
  writer.write(occupancy_event.str());
  return penalties;
}

template<typename ParticipantT>
auto apply_participant_delay(
  ParticipantT& participant,
  const rmf_traffic::Duration delay,
  int) -> decltype(participant.delay(delay), bool())
{
  participant.delay(delay);
  return true;
}

template<typename ParticipantT>
bool apply_participant_delay(
  ParticipantT&,
  const rmf_traffic::Duration,
  long)
{
  return false;
}

int run_dynamic_negotiation(
  const std::string& scenario,
  const std::string& description,
  LabGraph lab_graph,
  const std::vector<RobotRequest>& requests,
  JsonlWriter& writer,
  const std::vector<std::size_t>& closed_lanes = {})
{
  using Negotiation = rmf_traffic::agv::CentralizedNegotiation;
  write_graph_and_configuration(
    writer, scenario, description, lab_graph, closed_lanes, requests.size());
  auto database = std::make_shared<rmf_traffic::schedule::Database>();
  const auto profile = make_profile();
  const auto traits = make_traits(profile);
  auto planner = std::make_shared<Planner>(
    make_configuration(lab_graph, traits, closed_lanes), make_planner_options());
  std::vector<rmf_traffic::schedule::Participant> participants;
  participants.reserve(requests.size());
  std::map<rmf_traffic::schedule::ParticipantId, Plan> committed_plans;
  std::map<rmf_traffic::schedule::ParticipantId,
    std::vector<std::size_t>> committed_lanes;
  std::set<std::size_t> applied_runtime_events;

  std::map<double, std::vector<std::size_t>> batches;
  for (std::size_t i = 0; i < requests.size(); ++i)
    batches[requests[i].insertion_time_s].push_back(i);
  const char* raw_policy = std::getenv("RMF_TRAFFIC_LAB_DYNAMIC_POLICY");
  const std::string policy = raw_policy ? raw_policy : "fixed_existing";
  writer.write(
    "{\"event\":\"dynamic_run_started\",\"policy\":"
    + json_string(policy)
    + ",\"database_lifetime\":\"one real in-memory Database persists across every insertion stage\","
      "\"existing_robot_policy\":\"committed itineraries remain fixed; only newly inserted participants are agents\","
      "\"batch_count\":" + std::to_string(batches.size()) + "}");

  std::size_t stage = 0;
  for (const auto& [insertion_time_s, indexes] : batches)
  {
    ++stage;
    for (std::size_t event_index = 0;
      event_index < ActiveRuntimeEvents.size(); ++event_index)
    {
      if (applied_runtime_events.count(event_index) > 0)
        continue;
      const auto& event = ActiveRuntimeEvents[event_index];
      if (event.at_s > insertion_time_s)
        continue;
      const auto participant = std::find_if(
        participants.begin(), participants.end(),
        [&](const auto& item) { return item.description().name() == event.robot; });
      const auto version_before = database->latest_version();
      bool schedule_changed = false;
      std::string api = "simulation state only";
      if (event.type == "DELAY" && participant != participants.end())
      {
        const auto duration = std::chrono::duration_cast<rmf_traffic::Duration>(
          std::chrono::duration<double>(event.value_s));
        schedule_changed = apply_participant_delay(*participant, duration, 0);
        if (schedule_changed)
          ActiveCumulativeDelay[participant->id()] += event.value_s;
        api = schedule_changed
          ? "rmf_traffic::schedule::Participant::delay(Duration)"
          : "Participant::delay(Duration) unavailable in this RMF build";
      }
      writer.write(
        "{\"event\":\"runtime_traffic_event\",\"source\":\"SIMULATION_EVENT\","
        "\"type\":" + json_string(event.type)
        + ",\"robot\":" + json_string(event.robot)
        + ",\"at_s\":" + std::to_string(event.at_s)
        + ",\"value_s\":" + std::to_string(event.value_s)
        + ",\"schedule_api\":" + json_string(api)
        + ",\"schedule_changed\":" + (schedule_changed ? "true" : "false")
        + ",\"schedule_version_before\":" + std::to_string(version_before)
        + ",\"schedule_version_after\":"
        + std::to_string(database->latest_version())
        + ",\"replan_trigger\":" + json_string(event.detail)
        + ",\"automatic_periodic_replan\":false}");
      if (event.flag)
      {
        writer.write(
          "{\"event\":\"replan_trigger\",\"source\":\"SIMULATION_EVENT\","
          "\"robot\":" + json_string(event.robot)
          + ",\"at_s\":" + std::to_string(event.at_s)
          + ",\"reason\":" + json_string(event.detail)
          + ",\"schedule_changed\":"
          + (schedule_changed ? "true" : "false")
          + ",\"action\":\"next explicit newcomer planning invocation reads latest fixed snapshot\""
          + ",\"periodic_replan\":false}");
      }
      applied_runtime_events.insert(event_index);
      if (schedule_changed)
      {
        write_schedule_state(
          writer, "runtime_event_" + std::to_string(event_index),
          database, participants);
      }
    }
    std::vector<RobotRequest> newcomers;
    std::vector<std::size_t> participant_indexes;
    std::ostringstream stage_event;
    stage_event << std::setprecision(12)
                << "{\"event\":\"dynamic_insertion_stage\",\"stage\":" << stage
                << ",\"insertion_time_s\":" << insertion_time_s
                << ",\"schedule_version_before\":" << database->latest_version()
                << ",\"existing_committed_count\":" << committed_plans.size()
                << ",\"new_robots\":[";
    for (std::size_t j = 0; j < indexes.size(); ++j)
    {
      const auto& request = requests[indexes[j]];
      if (j > 0)
        stage_event << ',';
      stage_event << json_string(request.name);
      newcomers.push_back(request);
      const auto version_before = database->latest_version();
      participants.push_back(rmf_traffic::schedule::make_participant(
        {request.name, "lab_fleet",
         rmf_traffic::schedule::ParticipantDescription::Rx::Responsive,
         profile}, database));
      participant_indexes.push_back(participants.size() - 1);
      const auto& participant = participants.back();
      writer.write(
        "{\"event\":\"schedule_database_operation\","
        "\"action\":\"register_dynamic_participant\","
        "\"api\":\"rmf_traffic::schedule::make_participant\","
        "\"participant_id\":" + std::to_string(participant.id())
        + ",\"name\":" + json_string(request.name)
        + ",\"insertion_time_s\":" + std::to_string(insertion_time_s)
        + ",\"version_before\":" + std::to_string(version_before)
        + ",\"version_after\":" + std::to_string(database->latest_version())
        + ",\"result\":\"new participant registered without replacing existing itineraries\"}");
    }
    stage_event << "]}";
    writer.write(stage_event.str());
    write_schedule_state(
      writer, "dynamic_stage_" + std::to_string(stage) + "_registered",
      database, participants);

    configure_policy_snapshot(
      writer, lab_graph, database,
      participant_indexes.empty() ? 0 : participants[participant_indexes.front()].id(),
      insertion_time_s,
      "dynamic_newcomer_baseline_against_latest_schedule");

    ::unsetenv("RMF_TRAFFIC_LAB_LANE_PENALTIES");
    std::vector<std::optional<Plan>> newcomer_baseline_plans;
    newcomer_baseline_plans.reserve(newcomers.size());
    for (const auto& request : newcomers)
    {
      write_planning_request(writer, request, "dynamic_newcomer_free_flow_baseline");
      const Start start(request_start_time(request), request.start, request.yaw);
      trace_real_search(
        writer, request.name, *planner, start, Goal(request.goal),
        lab_graph, closed_lanes, request.goal);
      auto baseline = planner->plan(start, Goal(request.goal));
      if (baseline)
      {
        newcomer_baseline_plans.emplace_back(*baseline);
        const auto lanes = write_plan(
          writer, request.name, *baseline, "free_flow_baseline",
          baseline.ideal_cost(), Planner::Debug::expansion_count(baseline),
          Planner::Debug::node_count(baseline));
        write_candidate_paths(
          writer, request.name, lab_graph, traits, request, closed_lanes, lanes);
      }
      else
      {
        newcomer_baseline_plans.emplace_back(std::nullopt);
        writer.write(
          "{\"event\":\"plan_summary\",\"robot\":"
          + json_string(request.name)
          + ",\"phase\":\"free_flow_baseline\",\"success\":false}");
        write_candidate_paths(
          writer, request.name, lab_graph, traits, request, closed_lanes, {});
      }
    }

    const auto penalties = configure_newcomer_detour_penalty(
      writer, lab_graph, committed_lanes, newcomers, stage, insertion_time_s);
    std::vector<rmf_traffic::schedule::ParticipantId> newcomer_ids;
    newcomer_ids.reserve(participant_indexes.size());
    for (const auto index : participant_indexes)
      newcomer_ids.push_back(participants[index].id());
    const auto newcomer_admission_reservations =
      make_deterministic_admission_reservations(
        lab_graph, newcomer_baseline_plans, newcomers, newcomer_ids);
    configure_policy_snapshot(
      writer, lab_graph, database,
      participant_indexes.empty() ? 0 : participants[participant_indexes.front()].id(),
      insertion_time_s,
      "dynamic_newcomer_negotiation_against_latest_schedule",
      newcomer_admission_reservations);
    std::vector<Negotiation::Agent> agents;
    for (std::size_t j = 0; j < newcomers.size(); ++j)
    {
      const auto& request = newcomers[j];
      auto options = rmf_traffic::agv::SimpleNegotiator::Options();
      options.maximum_cost_leeway(penalties.empty() ? 10.0 : 100.0)
        .minimum_cost_threshold(penalties.empty() ? 180.0 : 10000.0);
      agents.emplace_back(
        participants[participant_indexes[j]].id(),
        Start(request_start_time(request), request.start, request.yaw),
        Goal(request.goal), planner, std::move(options));
    }

    writer.write(
      "{\"event\":\"dynamic_negotiation_request\",\"stage\":"
      + std::to_string(stage)
      + ",\"agent_count\":" + std::to_string(agents.size())
      + ",\"fixed_existing_itinerary_count\":"
      + std::to_string(committed_plans.size())
      + ",\"api\":\"CentralizedNegotiation(database).solve(newcomer_agents)\"}");
    writer.write(
      "{\"event\":\"validator_configuration\"," 
      "\"source\":\"RMF_CORE\",\"schedule_source\":\"SCHEDULE\"," 
      "\"phase\":\"dynamic_newcomer_negotiation\",\"stage\":"
      + std::to_string(stage)
      + ",\"validator\":\"NegotiatingRouteValidator\"," 
      "\"actual_internal_path\":true,\"per_call_result_observable\":false,"
      "\"planner_options_validator\":\"negotiation table plus fixed Schedule Database itineraries supplied internally\"," 
      "\"validator_object_publicly_exposed\":false,\"schedule_aware\":true,"
      "\"schedule_database_version\":" + std::to_string(database->latest_version())
      + ",\"fixed_existing_itinerary_count\":"
      + std::to_string(committed_plans.size())
      + ",\"post_proposal_validator\":\"rmf_traffic::DetectConflict::between\"}");
    const auto begin = std::chrono::steady_clock::now();
    const auto result = Negotiation(database)
      .optimal(agents.size() <= 2)
      .log(true)
      .solve(agents);
    const double elapsed_ms = 1000.0 * rmf_traffic::time::to_seconds(
      std::chrono::steady_clock::now() - begin);
    for (const auto& message : result.log())
      write_negotiation_log_event(writer, message, stage);

    if (!result.proposal().has_value())
    {
      writer.write(
        "{\"event\":\"proposal_summary\",\"phase\":\"dynamic_newcomer_negotiation\","
        "\"stage\":" + std::to_string(stage)
        + ",\"present\":false,\"source_api\":\"CentralizedNegotiation::Result::proposal\","
        "\"participant_plan_count\":0,\"commit_state\":\"no_proposal\"}");
      writer.write(
        "{\"event\":\"proposal_outcome\",\"phase\":\"dynamic_newcomer_negotiation\","
        "\"stage\":" + std::to_string(stage)
        + ",\"action\":\"reject_no_proposal\",\"accepted\":false,\"committed\":false,"
        "\"reason\":\"Newcomer negotiation returned nullopt against fixed committed itineraries\"}");
      writer.write(
        "{\"event\":\"dynamic_insertion_result\",\"stage\":"
        + std::to_string(stage)
        + ",\"success\":false,\"reason\":\"newcomer_no_proposal_against_fixed_schedule\","
          "\"existing_itineraries_preserved\":true}");
      writer.write(
        "{\"event\":\"negotiation_summary\",\"success\":false,"
        "\"executable_plan\":false,\"dynamic_stage\":"
        + std::to_string(stage) + ",\"elapsed_ms\":"
        + std::to_string(elapsed_ms) + "}");
      write_schedule_state(
        writer, "dynamic_stage_" + std::to_string(stage) + "_no_proposal",
        database, participants);
      write_solution_diagnosis(
        writer, "no_solution", "dynamic_newcomer_no_proposal", "high",
        "confirmed_by_staged_centralized_negotiation",
        "The newcomer batch could not produce a proposal against the itineraries already committed in the persistent Schedule Database",
        {"failed_stage=" + std::to_string(stage),
         "insertion_time_s=" + std::to_string(insertion_time_s),
         "fixed_existing_itineraries=" + std::to_string(committed_plans.size()),
         "after_nego_penalized_lanes=" + std::to_string(penalties.size())},
        {"Add a connected bypass before the bottleneck",
         "Insert the task earlier or later so the committed resource window changes",
         "Escalate to a controlled replan of selected existing robots if newcomer-only planning is insufficient"});
      return 0;
    }

    std::map<rmf_traffic::schedule::ParticipantId, Plan> new_plans(
      result.proposal()->begin(), result.proposal()->end());
    write_proposal_snapshot(
      writer, "dynamic_newcomer_negotiation", new_plans, participants, stage);
    bool schedule_route_validator_passed = true;
    for (const auto& [participant_id, plan] : new_plans)
    {
      schedule_route_validator_passed =
        validate_plan_with_schedule_route_validator(
          writer, "dynamic_newcomer_post_proposal", database,
          participant_id, plan, profile)
        && schedule_route_validator_passed;
    }
    auto combined = committed_plans;
    combined.insert(new_plans.begin(), new_plans.end());
    const auto safety = verify_negotiated_plans(
      writer, combined, participants, profile);
    if (!safety.passed || !schedule_route_validator_passed)
    {
      writer.write(
        "{\"event\":\"proposal_outcome\",\"phase\":\"dynamic_post_proposal_safety\","
        "\"stage\":" + std::to_string(stage)
        + ",\"action\":\"reject_after_detect_conflict\",\"accepted\":false,"
        "\"committed\":false,\"reason\":\"ScheduleRouteValidator or combined DetectConflict verification rejected the newcomer proposal\"}");
      writer.write(
        "{\"event\":\"dynamic_insertion_result\",\"stage\":"
        + std::to_string(stage)
        + ",\"success\":false,\"reason\":\"combined_plan_conflict\","
          "\"existing_itineraries_preserved\":true}");
      write_schedule_state(
        writer, "dynamic_stage_" + std::to_string(stage) + "_safety_rejected",
        database, participants);
      write_solution_diagnosis(
        writer, "no_solution", "dynamic_combined_plan_conflict", "high",
        "confirmed_by_rmf_detect_conflict",
        "The newcomer proposal conflicted with a committed or same-batch trajectory during continuous-time verification",
        {"failed_stage=" + std::to_string(stage),
         "conflicts=" + std::to_string(safety.conflicts)},
        {"Increase temporal separation", "Add a physically separated bypass"});
      return 0;
    }

    for (const auto& [participant_id, plan] : new_plans)
    {
      const auto it = std::find_if(
        participants.begin(), participants.end(),
        [participant_id](const auto& p) { return p.id() == participant_id; });
      if (it == participants.end())
        continue;
      const auto lanes = write_plan(
        writer, it->description().name(), plan, "negotiated");
      const auto version_before = database->latest_version();
      const auto plan_id = it->assign_plan_id();
      const bool accepted = it->set(plan_id, plan.get_itinerary());
      writer.write(
        "{\"event\":\"schedule_database_operation\","
        "\"action\":\"set_dynamic_newcomer_itinerary\","
        "\"api\":\"rmf_traffic::schedule::Participant::set\","
        "\"participant_id\":" + std::to_string(participant_id)
        + ",\"name\":" + json_string(it->description().name())
        + ",\"plan_id\":" + std::to_string(plan_id)
        + ",\"version_before\":" + std::to_string(version_before)
        + ",\"version_after\":" + std::to_string(database->latest_version())
        + ",\"accepted\":" + (accepted ? "true" : "false") + "}");
      committed_plans.emplace(participant_id, plan);
      committed_lanes[participant_id] = lanes;
    }
    write_schedule_state(
      writer, "dynamic_stage_" + std::to_string(stage) + "_committed",
      database, participants);
    writer.write(
      "{\"event\":\"proposal_outcome\",\"phase\":\"dynamic_schedule_commit\","
      "\"stage\":" + std::to_string(stage)
      + ",\"action\":\"accept_and_commit\",\"accepted\":true,\"committed\":true,"
      "\"reason\":\"Newcomer proposal passed combined DetectConflict verification and was written with Participant::set\"}");
    writer.write(
      "{\"event\":\"dynamic_insertion_result\",\"stage\":"
      + std::to_string(stage)
      + ",\"success\":true,\"existing_itineraries_preserved\":true,"
        "\"new_plan_count\":" + std::to_string(new_plans.size())
      + ",\"penalized_lane_count\":" + std::to_string(penalties.size()) + "}");
    writer.write(
      "{\"event\":\"negotiation_summary\",\"success\":true,"
      "\"safety_verified\":true,\"executable_plan\":true,"
      "\"dynamic_stage\":" + std::to_string(stage)
      + ",\"elapsed_ms\":" + std::to_string(elapsed_ms)
      + ",\"proposal_plan_count\":" + std::to_string(new_plans.size()) + "}");
  }

  write_schedule_state(writer, "dynamic_all_stages_committed", database, participants);
  write_solution_diagnosis(
    writer, "solved", "dynamic_all_insertions_committed", "high",
    "confirmed_by_staged_rmf_negotiation_and_detect_conflict",
    "Every newcomer batch was negotiated against the persistent real Schedule Database, verified with committed plans, and committed without replacing existing itineraries",
    {"stages=" + std::to_string(batches.size()),
     "participants=" + std::to_string(participants.size()),
     "policy=" + policy},
    {"Compare newcomer used_lanes and completion time between Before and After_nego"});
  return 0;
}

const std::vector<std::string> Scenarios = {
  "single_lane_bidirectional",
  "single_path",
  "single_path_closed",
  "speed_limit_choice",
  "single_path_multi",
  "head_on",
  "passing_bay",
  "t_junction",
  "cross_intersection",
  "disconnected"
};

struct Arguments
{
  std::string scenario = "single_lane_bidirectional";
  std::string scenario_file;
  std::string output = "result.jsonl";
};

void print_help(const char* program)
{
  std::cout << "Usage: " << program << " --scenario <";
  for (std::size_t i = 0; i < Scenarios.size(); ++i)
  {
    if (i > 0)
      std::cout << '|';
    std::cout << Scenarios[i];
  }
  std::cout << "> [--scenario-file <compiled.rmf>] --output <result.jsonl>\n";
}

Arguments parse_arguments(const int argc, char* argv[])
{
  Arguments args;
  for (int i = 1; i < argc; ++i)
  {
    const std::string arg = argv[i];
    if (arg == "--scenario" && i + 1 < argc)
      args.scenario = argv[++i];
    else if (arg == "--scenario-file" && i + 1 < argc)
      args.scenario_file = argv[++i];
    else if (arg == "--output" && i + 1 < argc)
      args.output = argv[++i];
    else if (arg == "--help" || arg == "-h")
    {
      print_help(argv[0]);
      std::exit(0);
    }
    else
      throw std::runtime_error("Unknown or incomplete argument: " + arg);
  }

  if (args.scenario_file.empty()
    && std::find(Scenarios.begin(), Scenarios.end(), args.scenario) == Scenarios.end())
  {
    throw std::runtime_error("Unknown scenario: " + args.scenario);
  }
  return args;
}

} // namespace

int main(const int argc, char* argv[])
{
  try
  {
    const auto args = parse_arguments(argc, argv);
    JsonlWriter writer(args.output);

    if (!args.scenario_file.empty())
    {
      auto custom = load_custom_scenario(args.scenario_file);
      ActiveCorridors = custom.corridors;
      ActiveRuntimeEvents = custom.runtime_events;
      ActiveCumulativeDelay.clear();
      writer.write(
        "{\"event\":\"custom_scenario_loaded\",\"format\":\"rmf_custom_v1\","
        "\"source\":" + json_string(args.scenario_file)
        + ",\"source_json\":" + json_string(custom.source_json)
        + ",\"name\":" + json_string(custom.name)
        + ",\"mode\":" + json_string(custom.mode)
        + ",\"dynamic_insertion\":"
        + (custom.dynamic_insertion ? "true" : "false")
        + ",\"node_count\":" + std::to_string(custom.graph.nodes.size())
        + ",\"directed_lane_count\":" + std::to_string(custom.graph.lanes.size())
        + ",\"robot_count\":" + std::to_string(custom.robots.size())
        + ",\"corridor_count\":" + std::to_string(custom.corridors.size())
        + ",\"runtime_event_count\":" + std::to_string(custom.runtime_events.size())
        + ",\"closed_lanes\":" + json_number_array(custom.closed_lanes)
        + ",\"validation_warnings\":"
        + json_string_array(custom.validation_warnings)
        + "}");
      for (const auto& corridor : custom.corridors)
      {
        writer.write(
          "{\"event\":\"corridor_definition\",\"source\":\"POLICY_DERIVED\","
          "\"corridor_id\":" + json_string(corridor.id)
          + ",\"lanes_forward\":" + json_number_array(corridor.lanes_forward)
          + ",\"lanes_reverse\":" + json_number_array(corridor.lanes_reverse)
          + ",\"capacity\":" + std::to_string(corridor.capacity)
          + ",\"passing_allowed\":"
          + (corridor.passing_allowed ? "true" : "false")
          + ",\"hard_opposite_direction_block\":"
          + (corridor.hard_opposite_direction_block ? "true" : "false")
          + ",\"holding_entry_a\":"
          + (corridor.holding_entry_a.has_value()
            ? std::to_string(*corridor.holding_entry_a) : "null")
          + ",\"holding_entry_b\":"
          + (corridor.holding_entry_b.has_value()
            ? std::to_string(*corridor.holding_entry_b) : "null")
          + ",\"base_penalty\":" + std::to_string(corridor.base_penalty)
          + ",\"meaning\":\"Multiple directed RMF Graph lanes mapped to one physical traffic resource\"}");
      }
      for (const auto& event : custom.runtime_events)
      {
        writer.write(
          "{\"event\":\"runtime_event_definition\",\"source\":\"SIMULATION_EVENT\","
          "\"type\":" + json_string(event.type)
          + ",\"robot\":" + json_string(event.robot)
          + ",\"at_s\":" + std::to_string(event.at_s)
          + ",\"value_s\":" + std::to_string(event.value_s)
          + ",\"trigger_enabled\":" + (event.flag ? "true" : "false")
          + ",\"detail\":" + json_string(event.detail) + "}");
      }
      if (custom.mode == "free_flow")
      {
        return run_free_flow(
          custom.name, custom.description, std::move(custom.graph),
          custom.robots.front(), custom.closed_lanes, {}, true, writer, false);
      }
      if (custom.dynamic_insertion)
      {
        return run_dynamic_negotiation(
          custom.name, custom.description, std::move(custom.graph), custom.robots,
          writer, custom.closed_lanes);
      }
      return run_negotiation(
        custom.name, custom.description, std::move(custom.graph), custom.robots,
        writer, custom.closed_lanes);
    }

    if (args.scenario == "single_lane_bidirectional")
    {
      return run_negotiation(
        args.scenario,
        "Two opposing robots share one bidirectional lane; one must wait in a separated staging area until the corridor is clear",
        make_single_lane_bidirectional_graph(),
        {{"R_WEST", 0, 8, 0.0}, {"R_EAST", 7, 1, Pi}}, writer);
    }

    if (args.scenario == "single_path")
    {
      return run_free_flow(
        args.scenario,
        "One robot chooses between a short center route and a longer detour",
        make_single_path_graph(), {"R0", 0, 4, 0.0}, {}, {4}, true, writer);
    }

    if (args.scenario == "single_path_closed")
    {
      return run_free_flow(
        args.scenario,
        "The center connection is closed, forcing the upper detour",
        make_single_path_graph(), {"R0", 0, 4, 0.0}, {4, 5}, {10}, true, writer);
    }

    if (args.scenario == "speed_limit_choice")
    {
      return run_free_flow(
        args.scenario,
        "A geometrically short route is slow, so RMF can prefer the faster detour",
        make_single_path_graph(true), {"R0", 0, 4, 0.0}, {}, {10}, true, writer);
    }

    if (args.scenario == "disconnected")
    {
      return run_free_flow(
        args.scenario,
        "Start and goal lie on disconnected graph islands",
        make_disconnected_graph(), {"R0", 0, 3, 0.0}, {}, {}, false, writer);
    }

    if (args.scenario == "single_path_multi")
    {
      return run_negotiation(
        args.scenario,
        "Two robots exchange ends on the same graph; center and detour provide alternatives",
        make_single_path_graph(),
        {{"R_LEFT", 0, 4, 0.0}, {"R_RIGHT", 4, 0, Pi}}, writer);
    }

    if (args.scenario == "head_on")
    {
      return run_negotiation(
        args.scenario,
        "Two robots exchange ends in a corridor with no passing bay",
        make_head_on_graph(),
        {{"R_LEFT", 0, 4, 0.0}, {"R_RIGHT", 4, 0, Pi}}, writer);
    }

    if (args.scenario == "passing_bay")
    {
      return run_negotiation(
        args.scenario,
        "Head-on exchange with an alternate bay route and holding points",
        make_passing_bay_graph(),
        {{"R_LEFT", 0, 4, 0.0}, {"R_RIGHT", 4, 0, Pi}}, writer);
    }

    if (args.scenario == "t_junction")
    {
      return run_negotiation(
        args.scenario,
        "Three robots compete for a T-junction",
        make_t_junction_graph(),
        {{"R_WEST", 0, 2, 0.0}, {"R_EAST", 2, 0, Pi}, {"R_NORTH", 3, 0, -Pi/2.0}},
        writer);
    }

    return run_negotiation(
      args.scenario,
      "Four robots cross one shared intersection from all directions",
      make_cross_graph(),
      {
        {"R_WEST", 0, 2, 0.0},
        {"R_EAST", 2, 0, Pi},
        {"R_NORTH", 3, 4, -Pi/2.0},
        {"R_SOUTH", 4, 3, Pi/2.0}
      },
      writer);
  }
  catch (const std::exception& e)
  {
    std::cerr << "ERROR: " << e.what() << '\n';
    return 1;
  }
}
