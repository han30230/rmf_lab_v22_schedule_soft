import json
import tempfile
import unittest
from pathlib import Path

from regression_runner import PROFILE_ORDER, classify_against_baseline, summarize_jsonl
from tools.stress_scenarios import generate_grid_stress


class RegressionRunnerTest(unittest.TestCase):
    def _write(self, events: list[dict]) -> Path:
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / "result.jsonl"
        path.write_text("".join(json.dumps(e) + "\n" for e in events), encoding="utf-8")
        return path

    def test_summary_uses_native_jsonl_metrics(self) -> None:
        path = self._write([
            {"event": "planning_request", "robot": "R1", "start_time_s": 0},
            {"event": "plan_waypoint", "robot": "R1", "phase": "negotiated",
             "delta_distance_m": 2.5, "delta_time_s": 4.0, "movement_type": "forward_traverse"},
            {"event": "plan_waypoint", "robot": "R1", "phase": "negotiated",
             "delta_distance_m": 0.0, "delta_time_s": 3.0, "movement_type": "wait"},
            {"event": "plan_summary", "robot": "R1", "phase": "negotiated",
             "success": True, "finish_time_s": 7.0, "used_lanes": [1, 2]},
            {"event": "planner_timing", "elapsed_ms": 4.2},
            {"event": "astar_trace_summary", "expansions": 12},
            {"event": "safety_verification", "passed": True, "conflicts": 0},
            {"event": "solution_diagnosis", "status": "solved",
             "category": "executable_time_space_plan"},
            {"event": "runner_core_profile", "scenario_sha256": "abc", "random_seed": 7},
        ])
        summary = summarize_jsonl(path, "baseline")
        self.assertTrue(summary["success"])
        self.assertEqual(summary["distance_m"], 2.5)
        self.assertEqual(summary["wait_time_s"], 3.0)
        self.assertEqual(summary["expanded_nodes"], 12)
        self.assertEqual(summary["random_seed"], 7)


    def test_profile_order_includes_independent_schedule_soft(self) -> None:
        self.assertEqual(
            PROFILE_ORDER,
            ("baseline", "soft", "schedule_soft", "hybrid", "hybrid_nego"),
        )

    def test_summary_reports_schedule_query_and_self_filter_metrics(self) -> None:
        path = self._write([
            {"event": "runner_core_profile", "scenario_sha256": "abc",
             "traffic_policy_mode": "schedule_soft"},
            {"event": "corridor_policy_snapshot", "schedule_version": 4,
             "schedule_query_count": 1, "queried_participant_count": 3,
             "queried_route_count": 5, "self_filtered_route_count": 1},
            {"event": "corridor_policy_expansion", "overlap_check_count": 2,
             "total_policy_penalty": 0.5},
            {"event": "solution_diagnosis", "status": "solved",
             "category": "executable_time_space_plan"},
        ])
        summary = summarize_jsonl(path, "schedule_soft")
        self.assertEqual(summary["schedule_snapshot_count"], 1)
        self.assertEqual(summary["schedule_snapshot_version"], 4)
        self.assertEqual(summary["schedule_query_count"], 1)
        self.assertEqual(summary["queried_participant_count"], 3)
        self.assertEqual(summary["queried_route_count"], 5)
        self.assertEqual(summary["self_filtered_route_count"], 1)
        self.assertEqual(summary["overlap_check_count"], 2)

    def test_critical_regression_and_improvement(self) -> None:
        verdict, _ = classify_against_baseline(
            {"success": True, "deadlock": False, "conflict": False},
            {"success": False, "deadlock": True, "conflict": False})
        self.assertEqual(verdict, "REGRESSION")
        verdict, _ = classify_against_baseline(
            {"success": False, "deadlock": True, "conflict": False},
            {"success": True, "deadlock": False, "conflict": False})
        self.assertEqual(verdict, "IMPROVEMENT")

    def test_summary_totals_robot_time_and_normalizes_native_reason(self) -> None:
        path = self._write([
            {"event": "planning_request", "robot": "R1", "start_time_s": 0},
            {"event": "planning_request", "robot": "R2", "start_time_s": 2},
            {"event": "plan_summary", "robot": "R1", "phase": "negotiated",
             "success": True, "finish_time_s": 10, "used_lanes": [1]},
            {"event": "plan_summary", "robot": "R2", "phase": "negotiated",
             "success": True, "finish_time_s": 12, "used_lanes": [2]},
            {"event": "solution_diagnosis", "status": "no_solution",
             "category": "search_saturation"},
        ])
        summary = summarize_jsonl(path, "hybrid")
        self.assertEqual(summary["travel_time_s"], 20.0)
        self.assertEqual(summary["makespan_s"], 12.0)
        self.assertEqual(summary["termination_reason"], "SATURATION_LIMIT")

    def test_stress_generator_is_seed_reproducible(self) -> None:
        left = generate_grid_stress(5, 5, 42, random_start_time_max_s=10)
        right = generate_grid_stress(5, 5, 42, random_start_time_max_s=10)
        other = generate_grid_stress(5, 5, 43, random_start_time_max_s=10)
        self.assertEqual(left, right)
        self.assertNotEqual(left["robots"], other["robots"])


if __name__ == "__main__":
    unittest.main()
