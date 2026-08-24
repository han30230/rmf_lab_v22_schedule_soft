from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from run import append_core_profile, compile_custom_scenario
from tools.scenario_templates import builtin_scenarios


class ScenarioTemplateTest(unittest.TestCase):
    def test_core_profile_records_scenario_identity(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            jsonl = root / "result.jsonl"
            scenario = root / "scenario.json"
            jsonl.write_text('{"seq":0,"event":"run_started"}\n', encoding="utf-8")
            scenario.write_text('{"name":"same_input"}\n', encoding="utf-8")
            append_core_profile(
                jsonl,
                label="after_soft_penalty",
                setup=None,
                build_dir=root / "build_after",
                rmf_source=None,
                scenario_source=scenario,
                binary=Path("/bin/true"),
                environment={},
            )
            profile = json.loads(jsonl.read_text(encoding="utf-8").splitlines()[-1])
            self.assertEqual(profile["seq"], 1)
            self.assertEqual(profile["label"], "after_soft_penalty")
            self.assertEqual(len(profile["scenario_sha256"]), 64)

    def test_core_profile_prefers_cpp_real_baseline_occupancy_evidence(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            jsonl = root / "result.jsonl"
            scenario = root / "scenario.json"
            jsonl.write_text(
                '{"seq":0,"event":"occupancy_penalty_configuration",'
                '"source":"real_rmf_free_flow_baseline_plan_overlap",'
                '"directed_lane_occupancy":{"2":2},'
                '"directed_lane_penalties":{"2":75},'
                '"shared_corridor_users":{"1-2":["A","B"]}}\n',
                encoding="utf-8",
            )
            scenario.write_text('{"name":"same_input"}\n', encoding="utf-8")
            append_core_profile(
                jsonl,
                label="after_occupancy",
                setup=None,
                build_dir=root / "build_after",
                rmf_source=None,
                scenario_source=scenario,
                binary=Path("/bin/true"),
                environment={},
                lane_penalty_configuration={
                    "active": True,
                    "mode": "shared_corridor",
                    "automatic_penalty": 75.0,
                },
            )
            profile = json.loads(jsonl.read_text(encoding="utf-8").splitlines()[-1])
            self.assertEqual(profile["occupancy_source"],
                             "real_rmf_free_flow_baseline_plan_overlap")
            self.assertEqual(profile["directed_lane_penalties"], {"2": 75})
            self.assertEqual(profile["shared_corridor_users"]["1-2"], ["A", "B"])

    def test_korean_font_is_bundled_with_license(self) -> None:
        root = Path(__file__).resolve().parents[1]
        font = root / "assets" / "fonts" / "NotoSansKR-Regular.woff2"
        license_file = root / "assets" / "fonts" / "OFL.txt"
        self.assertTrue(font.is_file())
        self.assertGreater(font.stat().st_size, 100_000)
        self.assertIn("SIL OPEN FONT LICENSE", license_file.read_text(encoding="utf-8"))

    def test_every_gui_template_compiles_to_custom_rmf_format(self) -> None:
        scenarios = builtin_scenarios()
        self.assertEqual(len(scenarios), 29)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for name, payload in scenarios.items():
                with self.subTest(name=name):
                    source = root / f"{name}.json"
                    target = root / f"{name}.rmf"
                    source.write_text(json.dumps(payload), encoding="utf-8")
                    compiled_name, _warnings = compile_custom_scenario(source, target)
                    self.assertEqual(compiled_name, name)
                    self.assertIn("FORMAT\trmf_custom_v1", target.read_text(encoding="utf-8"))

    def test_grid_scenario_sizes_and_robot_counts(self) -> None:
        scenarios = builtin_scenarios()
        expected = {
            "grid_3x3_multi": (9, 12, 4),
            "grid_5x5_multi": (25, 40, 6),
            "grid_10x10_multi": (100, 180, 8),
        }
        for name, (nodes, lanes, robots) in expected.items():
            payload = scenarios[name]
            self.assertEqual(len(payload["nodes"]), nodes)
            self.assertEqual(len(payload["lanes"]), lanes)
            self.assertEqual(len(payload["robots"]), robots)

    def test_p4_fab_scenario_shape(self) -> None:
        payload = builtin_scenarios()["P4_fab_3aisle_10robots"]
        self.assertEqual(len(payload["nodes"]), 141)
        self.assertEqual(len(payload["lanes"]), 156)
        self.assertEqual(len(payload["robots"]), 10)
        self.assertEqual(
            sum(bool(node.get("parking")) for node in payload["nodes"]), 15)
        self.assertEqual(
            sum(bool(node.get("holding")) for node in payload["nodes"]), 69)
        vertical = [node for node in payload["nodes"] if node["name"].startswith("P4_V")]
        self.assertEqual(len(vertical), 54)
        self.assertEqual(
            len({node["name"].split("_")[1] for node in vertical}), 9)

    def test_p3_fab_scenario_is_larger_and_has_multi_node_verticals(self) -> None:
        payload = builtin_scenarios()["P3_fab_3aisle_12robots"]
        self.assertEqual(len(payload["nodes"]), 179)
        self.assertEqual(len(payload["lanes"]), 196)
        self.assertEqual(len(payload["robots"]), 12)
        vertical = [node for node in payload["nodes"] if node["name"].startswith("P3_V")]
        self.assertEqual(len(vertical), 80)
        self.assertEqual(
            len({node["name"].split("_")[1] for node in vertical}), 10)

    def test_staggered_departure_template_has_per_robot_start_times(self) -> None:
        payload = builtin_scenarios()["staggered_departures"]
        self.assertEqual(len(payload["robots"]), 3)
        self.assertEqual(payload["nodes"][9]["name"], "DELAYED_STAGING")
        self.assertTrue(payload["nodes"][9]["parking"])
        self.assertEqual(payload["robots"][2]["start"], 9)
        self.assertEqual(
            [robot["start_time_s"] for robot in payload["robots"]],
            [0.0, 0.0, 8.0],
        )

    def test_dynamic_bottleneck_has_staged_newcomers_and_bypasses(self) -> None:
        payload = builtin_scenarios()["dynamic_bottleneck_insertion"]
        self.assertTrue(payload["dynamic_insertion"])
        self.assertGreaterEqual(len(payload["nodes"]), 15)
        self.assertGreaterEqual(len(payload["lanes"]), 18)
        self.assertEqual(
            [robot["insertion_time_s"] for robot in payload["robots"]],
            [0.0, 0.0, 8.0, 14.0],
        )

    def test_compiled_dynamic_schema_carries_insertion_times(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            payload = builtin_scenarios()["dynamic_bottleneck_insertion"]
            source = root / "dynamic.json"
            target = root / "dynamic.rmf"
            source.write_text(json.dumps(payload), encoding="utf-8")
            compile_custom_scenario(source, target)
            text = target.read_text(encoding="utf-8")
            self.assertIn("DYNAMIC\ttrue", text)
            self.assertIn("ROBOT\tR_NEW_8S\t7\t0", text)
            self.assertIn("\t8\t8\n", text)

    def test_compiled_policy_schema_carries_corridors_and_runtime_events(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            payload = builtin_scenarios()["S5_delay_inside_corridor"]
            source = root / "policy.json"
            target = root / "policy.rmf"
            source.write_text(json.dumps(payload), encoding="utf-8")
            compile_custom_scenario(source, target)
            text = target.read_text(encoding="utf-8")
            self.assertIn("CORRIDOR\t", text)
            self.assertIn("CORRIDOR_LANE\t", text)
            self.assertIn("DELAY\t", text)
            release_source = root / "release.json"
            release_target = root / "release.rmf"
            release_source.write_text(json.dumps(
                builtin_scenarios()["S7_confirmed_release"]), encoding="utf-8")
            compile_custom_scenario(release_source, release_target)
            self.assertIn(
                "CHECKPOINT_RELEASE\t",
                release_target.read_text(encoding="utf-8"))

    def test_occupied_corridor_scenario_has_fixed_and_flexible_robot(self) -> None:
        payload = builtin_scenarios()["occupied_corridor_detour"]
        self.assertEqual(
            [(robot["name"], robot["start"], robot["goal"])
             for robot in payload["robots"]],
            [("R_OCCUPY", 1, 2), ("R_DETOUR", 0, 4)],
        )
        self.assertFalse(payload["lanes"][2]["bidirectional"])

    def test_templates_are_independent_copies(self) -> None:
        first = builtin_scenarios()
        first["single_path"]["nodes"][0]["x"] = 999
        second = builtin_scenarios()
        self.assertEqual(second["single_path"]["nodes"][0]["x"], -4)

    def test_cpp_runner_enforces_forward_only_motion_and_v16_schema(self) -> None:
        source = (
            Path(__file__).resolve().parents[1] / "src" / "rmf_core_lab.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn("rmf_core_lab.v16", source)
        self.assertIn("Eigen::Vector2d::UnitX(), false", source)
        self.assertIn("movement_type", source)
        self.assertIn("forward_only", source)
        self.assertIn("request_start_time", source)
        self.assertIn("RMF_TRAFFIC_LAB_LANE_PENALTIES", source)
        self.assertIn("maximum_cost_leeway = lane_penalty_active ? 100.0 : 10.0", source)
        self.assertIn("configure_shared_corridor_penalty", source)
        self.assertIn("real_rmf_free_flow_baseline_plan_overlap", source)
        self.assertIn("RMF_TRAFFIC_LAB_LANE_OCCUPANCY", source)
        self.assertIn("run_dynamic_negotiation", source)
        self.assertIn("CentralizedNegotiation(database).solve(newcomer_agents)", source)
        self.assertIn("RMF_TRAFFIC_LAB_DYNAMIC_POLICY", source)
        self.assertIn("existing_itineraries_preserved", source)
        self.assertIn("g_translation_time_s", source)
        self.assertIn("h_graph_cruise_time_s", source)
        self.assertIn("schedule_model_schema", source)
        self.assertIn("planner_graph_context", source)
        self.assertIn("validator_configuration", source)
        self.assertIn("itinerary_summary", source)
        self.assertIn("route_summary", source)
        self.assertIn("proposal_summary", source)
        self.assertIn("proposal_outcome", source)
        self.assertIn("write_negotiation_log_event", source)
        self.assertIn("database->query(rmf_traffic::schedule::query_all())", source)
        self.assertIn("ScheduleRouteValidator validator", source)
        self.assertIn("NegotiatingRouteValidator", source)
        self.assertIn("per_call_result_observable", source)
        self.assertIn("expected ETA alone does not release", source)
        self.assertIn("make_deterministic_admission_reservations", source)

    def test_desktop_window_has_resizable_persistent_large_layout(self) -> None:
        source = (
            Path(__file__).resolve().parents[1] / "simulator.py"
        ).read_text(encoding="utf-8")
        self.assertIn("self.resize(1840, 1120)", source)
        self.assertIn("window.showMaximized()", source)
        self.assertIn("QSettings", source)
        self.assertIn("splitter/map_editor_v2", source)
        self.assertIn("splitter/map_output_v2", source)
        self.assertIn("def focus_map_layout", source)
        self.assertIn("def toggle_output_panel", source)
        self.assertIn("def toggle_window_size", source)
        self.assertIn("실시간 RMF 판단", source)
        self.assertIn("def toggle_live_decision_panel", source)
        self.assertIn("playback_speed_combo", source)
        self.assertIn("HYBRID + NEGO 코어 준비", source)
        self.assertIn('self.output_tabs.addTab(compare_panel, "5-Mode 비교 / Regression")', source)
        self.assertIn("schedule_model_text", source)
        self.assertIn("RMF 객체·협상 원문", source)
        self.assertIn("self.negotiation_timeline_table", source)
        self.assertIn("협상 전체 시퀀스", source)
        self.assertIn("self.reject_forfeit_table", source)
        self.assertIn("self.supergraph_table", source)
        self.assertIn("after_core_patch_status", source)
        self.assertIn("self.vertical_splitter.setHandleWidth(14)", source)
        self.assertIn("QSizePolicy.Policy.Ignored", source)
        self.assertIn('tabs.addTab(props, "노드/Lane")', source)


if __name__ == "__main__":
    unittest.main()
