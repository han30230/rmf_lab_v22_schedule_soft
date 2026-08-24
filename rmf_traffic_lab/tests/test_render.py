from __future__ import annotations

import json
import re
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path

from run import compile_custom_scenario
from tools.render_html import read_jsonl, render


class RenderTest(unittest.TestCase):
    def test_reads_and_renders_minimal_result(self) -> None:
        events = [
            {
                "seq": 0,
                "event": "run_started",
                "schema": "rmf_core_lab.v7",
                "scenario": "test",
                "description": "renderer test",
            },
            {"seq": 1, "event": "graph_summary", "map": "L1"},
            {
                "seq": 2,
                "event": "vehicle_traits",
                "profile_radius_m": 0.3,
                "linear_velocity_mps": 0.7,
                "linear_acceleration_mps2": 0.75,
            },
            {
                "seq": 3,
                "event": "planner_configuration",
                "saturation_limit": 10000,
                "closed_lanes": [],
            },
            {
                "seq": 4,
                "event": "graph_node",
                "id": 0,
                "name": "A",
                "x": 0,
                "y": 0,
                "holding": True,
                "outgoing_lanes": [0],
            },
            {
                "seq": 5,
                "event": "graph_node",
                "id": 1,
                "name": "B",
                "x": 1,
                "y": 0,
                "parking": True,
                "outgoing_lanes": [],
            },
            {
                "seq": 6,
                "event": "graph_lane",
                "id": 0,
                "entry": 0,
                "exit": 1,
                "length_m": 1.0,
                "closed": False,
            },
            {
                "seq": 7,
                "event": "planning_request",
                "robot": "R0",
                "start": 0,
                "goal": 1,
            },
            {
                "seq": 8,
                "event": "astar_expand",
                "robot": "R0",
                "step": 0,
                "node_id": 0,
                "parent_id": None,
                "waypoint": 0,
                "g": 0.0,
                "h": 1.0,
                "f": 1.0,
                "queue_size": 1,
            },
            {
                "seq": 9,
                "event": "plan_waypoint",
                "robot": "R0",
                "phase": "free_flow",
                "sequence": 0,
                "time_s": 0,
                "x": 0,
                "y": 0,
                "approach_lanes": [],
            },
            {
                "seq": 10,
                "event": "plan_waypoint",
                "robot": "R0",
                "phase": "free_flow",
                "sequence": 1,
                "time_s": 1,
                "x": 1,
                "y": 0,
                "approach_lanes": [0],
            },
            {
                "seq": 11,
                "event": "trajectory_point",
                "robot": "R0",
                "phase": "free_flow",
                "route_index": 0,
                "sequence": 0,
                "time_s": 0,
                "x": 0,
                "y": 0,
                "yaw_rad": 0,
                "vx": 0,
                "vy": 0,
            },
            {
                "seq": 12,
                "event": "trajectory_point",
                "robot": "R0",
                "phase": "free_flow",
                "route_index": 0,
                "sequence": 1,
                "time_s": 1,
                "x": 1,
                "y": 0,
                "yaw_rad": 0,
                "vx": 0,
                "vy": 0,
            },
            {
                "seq": 13,
                "event": "plan_summary",
                "robot": "R0",
                "phase": "free_flow",
                "success": True,
                "cost": 1.0,
                "used_lanes": [0],
            },
            {
                "seq": 14,
                "event": "route_candidate",
                "robot": "R0",
                "rank": 1,
                "waypoints": [0, 1],
                "lanes": [0],
                "distance_m": 1.0,
                "rmf_cost": 1.0,
                "delta_from_best": 0.0,
                "finish_time_s": 1.0,
                "selected_by_plan": True,
            },
        ]
        with tempfile.TemporaryDirectory() as directory:
            directory_path = Path(directory)
            jsonl_path = directory_path / "result.jsonl"
            html_path = directory_path / "result.html"
            jsonl_path.write_text(
                "".join(json.dumps(event) + "\n" for event in events),
                encoding="utf-8",
            )

            self.assertEqual(len(read_jsonl(jsonl_path)), len(events))
            render(jsonl_path, html_path)
            rendered = html_path.read_text(encoding="utf-8")
            self.assertIn("RMF Traffic Analyzer", rendered)
            self.assertIn("id=\"play-button\"", rendered)
            self.assertIn("id=\"tab-search\"", rendered)
            self.assertIn("id=\"tab-diagnosis\"", rendered)
            self.assertIn("id=\"tab-raw\"", rendered)
            self.assertIn("id=\"raw-log-output\"", rendered)
            self.assertIn("id=\"tab-schedule\"", rendered)
            self.assertIn("id=\"tab-negotiation\"", rendered)
            self.assertIn("id=\"tab-scenarios\"", rendered)
            self.assertIn("id=\"db-phase-select\"", rendered)
            self.assertIn("실제 DB API operation", rendered)
            self.assertIn("Navigation Graph", rendered)
            self.assertIn("커스텀 JSON 시나리오", rendered)
            self.assertIn("전체 시퀀스", rendered)
            self.assertIn("trajectory_point", jsonl_path.read_text(encoding="utf-8"))
            self.assertIn("R0", rendered)
            node = shutil.which("node")
            scripts = re.findall(r"<script>(.*?)</script>", rendered, re.DOTALL)
            self.assertTrue(scripts)
            if node:
                subprocess.run(
                    [node, "--check", "-"],
                    input="\n".join(scripts),
                    text=True,
                    check=True,
                )

    def test_failed_negotiation_does_not_default_to_unsafe_baseline(self) -> None:
        events = [
            {
                "seq": 0,
                "event": "run_started",
                "schema": "rmf_core_lab.v7",
                "scenario": "unsafe_test",
                "robot_count": 2,
            },
            {"seq": 1, "event": "graph_node", "id": 0, "name": "L", "x": -1, "y": 0},
            {"seq": 2, "event": "graph_node", "id": 1, "name": "R", "x": 1, "y": 0},
            {
                "seq": 3,
                "event": "graph_lane",
                "id": 0,
                "entry": 0,
                "exit": 1,
                "length_m": 2.0,
            },
            {
                "seq": 4,
                "event": "negotiation_request",
                "robots": [
                    {"name": "A", "start": 0, "goal": 1},
                    {"name": "B", "start": 1, "goal": 0},
                ],
            },
            {
                "seq": 5,
                "event": "trajectory_point",
                "robot": "A",
                "phase": "free_flow_baseline",
                "time_s": 0,
                "route_index": 0,
                "sequence": 0,
                "x": -1,
                "y": 0,
                "yaw_rad": 0,
                "vx": 0,
                "vy": 0,
            },
            {
                "seq": 6,
                "event": "trajectory_point",
                "robot": "A",
                "phase": "free_flow_baseline",
                "time_s": 2,
                "route_index": 0,
                "sequence": 1,
                "x": 1,
                "y": 0,
                "yaw_rad": 0,
                "vx": 0,
                "vy": 0,
            },
            {
                "seq": 7,
                "event": "trajectory_point",
                "robot": "B",
                "phase": "free_flow_baseline",
                "time_s": 0,
                "route_index": 0,
                "sequence": 0,
                "x": 1,
                "y": 0,
                "yaw_rad": 3.14159,
                "vx": 0,
                "vy": 0,
            },
            {
                "seq": 8,
                "event": "trajectory_point",
                "robot": "B",
                "phase": "free_flow_baseline",
                "time_s": 2,
                "route_index": 0,
                "sequence": 1,
                "x": -1,
                "y": 0,
                "yaw_rad": 3.14159,
                "vx": 0,
                "vy": 0,
            },
            {
                "seq": 9,
                "event": "safety_verification",
                "passed": False,
                "executable_plan": False,
                "reason": "no_negotiated_proposal",
            },
            {"seq": 10, "event": "negotiation_summary", "success": False},
            {
                "seq": 11,
                "event": "solution_diagnosis",
                "status": "no_solution",
                "category": "endpoint_exchange_without_buffer",
                "confidence": "high",
                "basis": "topology_inference_from_confirmed_no_proposal",
                "root_cause": "No independent buffer exists",
                "evidence": ["simple_path_counts=[1,1]"],
                "recommended_actions": ["Add a passing loop"],
            },
        ]
        with tempfile.TemporaryDirectory() as directory:
            directory_path = Path(directory)
            jsonl_path = directory_path / "result.jsonl"
            html_path = directory_path / "result.html"
            jsonl_path.write_text(
                "".join(json.dumps(event) + "\n" for event in events),
                encoding="utf-8",
            )
            render(jsonl_path, html_path)
            rendered = html_path.read_text(encoding="utf-8")
            self.assertIn('"safePhase":"static"', rendered)
            self.assertIn("실행 금지", rendered)
            self.assertIn("충돌 미검증 free-flow 비교", rendered)
            self.assertIn("No independent buffer exists", rendered)
            self.assertIn("Add a passing loop", rendered)

    def test_compiles_custom_json_to_deterministic_intermediate(self) -> None:
        payload = {
            "name": "custom_test",
            "nodes": [
                {"name": "A", "x": 0, "y": 0, "holding": True},
                {"name": "B", "x": 1, "y": 0, "parking": True},
            ],
            "lanes": [{"from": 0, "to": 1, "bidirectional": True}],
            "robots": [
                {"name": "R0", "start": 0, "goal": 1, "yaw": 0, "start_time_s": 3.5}
            ],
        }
        with tempfile.TemporaryDirectory() as directory:
            directory_path = Path(directory)
            source = directory_path / "scenario.json"
            compiled = directory_path / "scenario.rmf"
            source.write_text(json.dumps(payload), encoding="utf-8")
            name, warnings = compile_custom_scenario(source, compiled)
            text = compiled.read_text(encoding="utf-8")
            self.assertEqual(name, "custom_test")
            self.assertEqual(warnings, [])
            self.assertIn("FORMAT\trmf_custom_v1", text)
            self.assertIn("MODE\tfree_flow", text)
            self.assertEqual(text.count("LANE\t"), 2)
            self.assertIn("ROBOT\tR0\t0\t1\t0\t3.5", text)

    def test_rejects_negative_robot_start_time(self) -> None:
        payload = {
            "name": "negative_start",
            "nodes": [
                {"name": "A", "x": 0, "y": 0},
                {"name": "B", "x": 1, "y": 0},
            ],
            "lanes": [{"from": 0, "to": 1, "bidirectional": True}],
            "robots": [
                {"name": "R0", "start": 0, "goal": 1, "start_time_s": -1}
            ],
        }
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "scenario.json"
            source.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "start_time_s must be non-negative"):
                compile_custom_scenario(source, root / "scenario.rmf")


if __name__ == "__main__":
    unittest.main()
