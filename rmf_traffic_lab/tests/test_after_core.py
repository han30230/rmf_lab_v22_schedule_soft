from __future__ import annotations

import json
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

from run import build_lane_penalty_configuration
from tools.scenario_templates import builtin_scenarios
from tools.setup_after_core import (
    PATCH_MARKER,
    POLICY_HEADER_NAME,
    after_core_patch_status,
    patch_planner,
    patch_simple_negotiator,
)
from tools.setup_after_nego_core import (
    AFTER_NEGO_POLICY,
    prepare_after_nego_core,
)
from tools.setup_schedule_soft_core import (
    SCHEDULE_SOFT_POLICY,
    prepare_schedule_soft_core,
)


class AfterCorePenaltyTest(unittest.TestCase):
    SIMPLE_NEGOTIATOR_FIXTURE = """#include <rmf_traffic/agv/SimpleNegotiator.hpp>
void SimpleNegotiator::respond(
  const schedule::Negotiation::Table::ViewerPtr& table_viewer,
  const ResponderPtr& responder)
{
  use(table_viewer, responder);
}
"""

    def _scenario_file(self, root: Path, name: str = "single_path") -> Path:
        source = root / f"{name}.json"
        source.write_text(
            json.dumps(builtin_scenarios()[name], ensure_ascii=False),
            encoding="utf-8",
        )
        return source

    def test_automatic_mode_penalizes_original_shortest_directed_route(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = build_lane_penalty_configuration(
                self._scenario_file(Path(directory)), "shortest_path", 60.0)
        self.assertTrue(config["active"])
        self.assertEqual(config["selected_baseline_lanes_by_robot"]["R0"], [0, 2, 4, 6])
        self.assertEqual(
            config["directed_lane_penalties"], {0: 60.0, 2: 60.0, 4: 60.0, 6: 60.0})
        self.assertEqual(config["environment_spec"], "0:60,2:60,4:60,6:60")

    def test_manual_mode_uses_both_directions_of_marked_source_lanes(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = self._scenario_file(root)
            payload = json.loads(source.read_text(encoding="utf-8"))
            payload["lanes"][1]["after_penalty"] = 60.0
            payload["lanes"][2]["after_penalty"] = 60.0
            source.write_text(json.dumps(payload), encoding="utf-8")
            config = build_lane_penalty_configuration(
                source, "manual", 999.0)
        self.assertEqual(
            config["directed_lane_penalties"],
            {2: 60.0, 3: 60.0, 4: 60.0, 5: 60.0},
        )

    def test_shared_corridor_mode_uses_robot_route_overlap_not_map_penalty(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = build_lane_penalty_configuration(
                self._scenario_file(Path(directory), "single_path_multi"),
                "shared_corridor",
                60.0,
            )
        self.assertEqual(set(config["directed_lane_occupancy"]), set(range(8)))
        self.assertTrue(all(
            demand == 2.0
            for demand in config["directed_lane_occupancy"].values()))
        self.assertTrue(all(
            penalty == 60.0
            for penalty in config["directed_lane_penalties"].values()))
        self.assertEqual(
            config["shared_corridor_users"]["1-2"], ["R_LEFT", "R_RIGHT"])
        self.assertEqual(config["occupancy_environment_spec"], ",".join(
            f"{lane}:2" for lane in range(8)))

    def test_shared_corridor_mode_does_not_invent_congestion_for_one_robot(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            config = build_lane_penalty_configuration(
                self._scenario_file(Path(directory)), "shared_corridor", 60.0)
        self.assertFalse(config["active"])
        self.assertEqual(config["directed_lane_occupancy"], {})

    def test_real_planner_patch_is_idempotent_and_changes_g_cost(self) -> None:
        fixture = """#include "a_star.hpp"
class ScheduledDifferentialDriveExpander
{
  void expand_traversal()
  {
      auto traversal_result = alt->routes(std::nullopt)(ready_time, ready_yaw);

      bool all_valid = true;
      value(
          node->current_cost + entry_event_cost + alt->cost,
      );
  }
  void expand()
  {
    if (!_validator)
    {
      // If we don't have a validator
    }
  }
};
"""
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            planner = root / "rmf_traffic/src/rmf_traffic/agv/planning/DifferentialDrivePlanner.cpp"
            planner.parent.mkdir(parents=True)
            planner.write_text(fixture, encoding="utf-8")
            negotiator = root / "rmf_traffic/src/rmf_traffic/agv/SimpleNegotiator.cpp"
            negotiator.write_text(self.SIMPLE_NEGOTIATOR_FIXTURE, encoding="utf-8")
            self.assertTrue(patch_planner(planner))
            self.assertTrue(patch_simple_negotiator(negotiator))
            self.assertFalse(patch_planner(planner))
            self.assertFalse(patch_simple_negotiator(negotiator))
            text = planner.read_text(encoding="utf-8")
            self.assertIn(PATCH_MARKER, text)
            self.assertIn(
                "node->current_cost + entry_event_cost + alt->cost\n"
                "          + rmf_lab_policy_decision.total_penalty", text)
            self.assertIn("if (!_validator && !rmf_lab_policy::enabled())", text)
            self.assertIn(f'#include "{POLICY_HEADER_NAME}"', text)
            header = planner.with_name(POLICY_HEADER_NAME)
            self.assertIn("SCHEDULE", header.read_text(encoding="utf-8"))
            self.assertIn("opposite_per_second", header.read_text(encoding="utf-8"))
            self.assertIn(
                "POLICY_DERIVED_FROM_RMF_CORE_TRAJECTORY",
                header.read_text(encoding="utf-8"))
            self.assertNotIn(
                "*remaining_cost_estimate\n            + rmf_lab_policy",
                text,
            )
            self.assertEqual(after_core_patch_status(root), (True, planner))
            self.assertTrue(planner.with_suffix(
                planner.suffix + ".before_rmf_lab_corridor_policy").is_file())
            self.assertIn(
                "ParticipantScope", negotiator.read_text(encoding="utf-8"))

    def test_generated_policy_header_compiles_as_cxx17(self) -> None:
        if shutil.which("g++") is None:
            self.skipTest("g++ is unavailable")
        fixture = """#include "a_star.hpp"
class ScheduledDifferentialDriveExpander
{
  void expand_traversal()
  {
      auto traversal_result = alt->routes(std::nullopt)(ready_time, ready_yaw);

      bool all_valid = true;
      value(
          node->current_cost + entry_event_cost + alt->cost,
      );
  }
  void expand()
  {
    if (!_validator)
    {
      // If we don't have a validator
    }
  }
};
"""
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            planner = root / "DifferentialDrivePlanner.cpp"
            planner.write_text(fixture, encoding="utf-8")
            patch_planner(planner)
            harness = root / "policy_compile.cpp"
            harness.write_text(
                '#include "RmfLabCorridorPolicy.hpp"\n'
                "int main() {\n"
                "  using namespace rmf_traffic::agv::planning::rmf_lab_policy;\n"
                "  const auto t = std::chrono::steady_clock::now();\n"
                "  ParticipantScope scope(7);\n"
                "  const auto d = evaluate(0, {}, std::nullopt, 0, 1, "
                "nullptr, t, t, 0.0, 0.0, 0.0, 0.0, MotionBreakdown{}, 0.0);\n"
                "  return d.hard_block ? 1 : 0;\n"
                "}\n",
                encoding="utf-8",
            )
            subprocess.run(
                ["g++", "-std=c++17", "-Wall", "-Wextra", "-pedantic",
                 str(harness), "-o", str(root / "policy_compile")],
                check=True,
                capture_output=True,
                text=True,
            )

    def test_previous_v1_patch_is_upgraded_in_place(self) -> None:
        fixture = """#include "a_star.hpp"
// RMF_TRAFFIC_LAB_LANE_PENALTY_V1_BEGIN
bool rmf_lab_lane_penalty_enabled() { return true; }
double rmf_lab_lane_penalty(std::size_t, const std::vector<std::size_t>&) { return 1.0; }
// RMF_TRAFFIC_LAB_LANE_PENALTY_V1_END
class ScheduledDifferentialDriveExpander
{
  void expand_traversal()
  {
      auto traversal_result = alt->routes(std::nullopt)(ready_time, ready_yaw);

      bool all_valid = true;
      value(
          node->current_cost + entry_event_cost + alt->cost,
      );
  }
  void expand()
  {
    if (!_validator)
    {
      // If we don't have a validator
    }
  }
};
"""
        with tempfile.TemporaryDirectory() as directory:
            planner = Path(directory) / "DifferentialDrivePlanner.cpp"
            planner.write_text(fixture, encoding="utf-8")
            self.assertIn(
                "RMF_TRAFFIC_LAB_LANE_PENALTY_V1",
                planner.read_text(encoding="utf-8"))
            self.assertTrue(patch_planner(planner))
            upgraded = planner.read_text(encoding="utf-8")
            self.assertIn(PATCH_MARKER, upgraded)
            self.assertNotIn("RMF_TRAFFIC_LAB_LANE_PENALTY_V1", upgraded)
            self.assertIn(POLICY_HEADER_NAME, upgraded)

    def test_schedule_soft_prepares_independent_baseline_derived_workspace(self) -> None:
        fixture = """#include "a_star.hpp"
class ScheduledDifferentialDriveExpander
{
  void expand_traversal()
  {
      auto traversal_result = alt->routes(std::nullopt)(ready_time, ready_yaw);

      bool all_valid = true;
      value(
          node->current_cost + entry_event_cost + alt->cost,
      );
  }
  void expand()
  {
    if (!_validator)
    {
      // If we don't have a validator
    }
  }
};
"""
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            baseline = root / "baseline"
            planner = baseline / "rmf_traffic/src/rmf_traffic/agv/planning/DifferentialDrivePlanner.cpp"
            planner.parent.mkdir(parents=True)
            planner.write_text(fixture, encoding="utf-8")
            negotiator = baseline / "rmf_traffic/src/rmf_traffic/agv/SimpleNegotiator.cpp"
            negotiator.write_text(self.SIMPLE_NEGOTIATOR_FIXTURE, encoding="utf-8")
            workspace = root / "schedule_soft"
            result = prepare_schedule_soft_core(baseline, workspace)
            self.assertEqual(result["policy"], SCHEDULE_SOFT_POLICY)
            self.assertEqual(result["variant"], "schedule_soft")
            self.assertIn("BASELINE", result["derivation"])
            self.assertTrue(result["safety_contract"]["one_fixed_snapshot_per_plan"])
            self.assertTrue(result["safety_contract"]["self_itinerary_excluded"])
            self.assertTrue(result["safety_contract"]["no_per_expansion_database_query"])
            self.assertTrue(result["safety_contract"]["lambda_zero_disables_hook"])
            self.assertTrue(after_core_patch_status(Path(result["after_source"]))[0])
            self.assertTrue((workspace / ".rmf_traffic_lab_schedule_soft.json").is_file())
            header = Path(result["policy_header"]).read_text(encoding="utf-8")
            self.assertIn('value.mode == "schedule_soft"', header)
            self.assertIn('interval.source != "SCHEDULE"', header)
            self.assertIn('interval.participant == current_participant(data)', header)
            self.assertIn('schedule_soft_max_penalty', header)
            self.assertNotIn("std::size_t plan = 0;\n  std::size_t plan = 0;", header)

    def test_schedule_soft_refuses_old_soft_source(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "old_soft"
            planner = source / "rmf_traffic/src/rmf_traffic/agv/planning/DifferentialDrivePlanner.cpp"
            planner.parent.mkdir(parents=True)
            planner.write_text(
                "// RMF_TRAFFIC_LAB_SCHEDULE_CORRIDOR_POLICY_V4\n",
                encoding="utf-8",
            )
            with self.assertRaisesRegex(RuntimeError, "BASELINE"):
                prepare_schedule_soft_core(source, root / "schedule_soft")

    def test_after_nego_prepares_separate_real_core_workspace(self) -> None:
        fixture = """#include "a_star.hpp"
class ScheduledDifferentialDriveExpander
{
  void expand_traversal()
  {
      auto traversal_result = alt->routes(std::nullopt)(ready_time, ready_yaw);

      bool all_valid = true;
      value(
          node->current_cost + entry_event_cost + alt->cost,
      );
  }
  void expand()
  {
    if (!_validator)
    {
      // If we don't have a validator
    }
  }
};
"""
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            before = root / "before"
            planner = before / "rmf_traffic/src/rmf_traffic/agv/planning/DifferentialDrivePlanner.cpp"
            planner.parent.mkdir(parents=True)
            planner.write_text(fixture, encoding="utf-8")
            negotiator = before / "rmf_traffic/src/rmf_traffic/agv/SimpleNegotiator.cpp"
            negotiator.write_text(self.SIMPLE_NEGOTIATOR_FIXTURE, encoding="utf-8")
            workspace = root / "after_nego"
            result = prepare_after_nego_core(before, workspace)
            self.assertEqual(result["policy"], AFTER_NEGO_POLICY)
            self.assertTrue(after_core_patch_status(Path(result["after_source"]))[0])
            self.assertTrue(
                (workspace / ".rmf_traffic_lab_after_nego.json").is_file())


if __name__ == "__main__":
    unittest.main()
