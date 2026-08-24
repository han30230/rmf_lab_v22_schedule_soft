from __future__ import annotations

import unittest

from tools.event_explainer import (
    classify_negotiation_message, decision_records, diagnosis_text,
    explain_event, explain_runtime_output, failure_analysis,
    failure_summary_text, failure_trace_records, rmf_object_guide_text,
    schedule_guide_text, schedule_model_text, summarize_jsonl,
)


class EventExplainerTest(unittest.TestCase):
    def test_astar_decision_uses_recorded_g_h_f_and_tie_caveat(self) -> None:
        text = explain_event({
            "seq": 10,
            "event": "astar_step_decision",
            "robot": "R1",
            "step": 2,
            "selected_node_id": 7,
            "selected_g": 3.0,
            "selected_h": 4.0,
            "selected_f": 7.0,
            "next_best_f": 7.0,
            "f_margin_to_next": 0.0,
        })
        self.assertIn("g=3.000", text)
        self.assertIn("f=g+h=7.000", text)
        self.assertIn("동률", text)
        self.assertIn("노출되지 않습니다", text)

    def test_no_solution_diagnosis_is_explained_in_korean(self) -> None:
        text = diagnosis_text({
            "status": "no_solution",
            "category": "endpoint_exchange_without_buffer",
            "confidence": "high",
            "basis": "topology_inference_from_confirmed_no_proposal",
            "root_cause": "Robots must exchange endpoints",
            "evidence": ["robots=2", "holding_points=3"],
            "recommended_actions": [
                "Add a side bay node connected to two corridor nodes so it forms an actual alternate path",
            ],
        })
        self.assertIn("끝점 맞교환", text)
        self.assertIn("협상 대상 로봇 수: 2", text)
        self.assertIn("실제 우회 루프", text)

    def test_jsonl_summary_and_decision_timeline(self) -> None:
        events = [
            {"seq": 0, "event": "run_started", "scenario": "sample", "robot_count": 1},
            {"seq": 1, "event": "graph_summary", "map": "L1", "waypoint_count": 2, "lane_count": 2},
            {"seq": 2, "event": "planning_request", "robot": "R0", "start": 0, "goal": 1},
            {"seq": 3, "event": "plan_summary", "robot": "R0", "phase": "free_flow", "success": True,
             "cost": 2.0, "finish_time_s": 2.0, "used_lanes": [0]},
        ]
        summary = summarize_jsonl(events)
        self.assertIn("sample", summary)
        self.assertIn("비용 2.000", summary)
        records = decision_records(events)
        self.assertEqual(records[0]["seq"], 0)
        self.assertTrue(any(record["seq"] == 2 for record in records))

    def test_jsonl_summary_counts_actual_negotiation_actions(self) -> None:
        summary = summarize_jsonl([
            {"seq": 0, "event": "run_started", "scenario": "nego", "robot_count": 2},
            {"seq": 1, "event": "proposal_summary", "present": False,
             "participant_plan_count": 0},
            {"seq": 2, "event": "negotiation_log", "message": "Rejected parent"},
            {"seq": 3, "event": "negotiation_log", "message": "Forfeited"},
            {"seq": 4, "event": "proposal_outcome", "action": "reject_no_proposal",
             "accepted": False, "committed": False},
        ])
        self.assertIn("Proposal 있음 0회, 없음 1회", summary)
        self.assertIn("협상 분기 Reject=1", summary)
        self.assertIn("협상 분기 Forfeit=1", summary)

    def test_runtime_and_schedule_copy_guidance(self) -> None:
        runtime = explain_runtime_output(
            "Scenario: sample\nPlan cost: 3.2\nUsed lanes: [0,2]\n"
        )
        self.assertIn("실행 시나리오", runtime)
        self.assertIn("최종 계획 비용", runtime)
        guide = schedule_guide_text([
            {"event": "schedule_database_state", "phase": "proposal_committed",
             "latest_version": 4, "participant_count": 2, "stored_route_count": 2},
        ])
        self.assertIn("DB version", guide)
        self.assertIn("Ctrl+Shift+C", guide)
        self.assertIn("POLICY_DERIVED", guide)

    def test_planning_request_explains_delayed_departure(self) -> None:
        text = explain_event({
            "seq": 8,
            "event": "planning_request",
            "robot": "R_DELAYED",
            "start": 1,
            "goal": 7,
            "start_yaw_rad": -1.57,
            "start_time_s": 8.0,
        })
        self.assertIn("요청 출발 시각 8.000 s", text)
        self.assertIn("동적으로 들어온 새 작업은 아닙니다", text)

    def test_forward_only_waypoint_explains_rotation_and_evidence(self) -> None:
        event = {
            "seq": 42,
            "event": "plan_waypoint",
            "robot": "R0",
            "phase": "negotiated",
            "sequence": 3,
            "time_s": 5.5,
            "x": 2.0,
            "y": 1.0,
            "yaw_rad": 3.14159,
            "delta_time_s": 1.25,
            "delta_distance_m": 0.0,
            "delta_yaw_rad": 3.14159,
            "movement_type": "rotate_in_place",
            "forward_only": True,
            "graph_index": 4,
            "approach_lanes": [7],
            "movement_reason": "Reverse travel is disabled",
        }
        explanation = explain_event(event)
        self.assertIn("제자리 회전", explanation)
        self.assertIn("후진을 금지", explanation)
        self.assertIn("Δ거리=0.000", explanation)
        record = decision_records([event])[0]
        self.assertIn("movement_type=rotate_in_place", record["evidence"])
        self.assertIn("graph_index=4", record["evidence"])

    def test_occupancy_penalty_explains_real_baseline_overlap(self) -> None:
        event = {
            "seq": 30,
            "event": "occupancy_penalty_configuration",
            "active": True,
            "baseline_lanes_by_robot": {"R0": [0, 2], "R1": [3, 1]},
            "shared_corridor_users": {"0-1": ["R0", "R1"]},
            "directed_lane_occupancy": {"0": 2, "1": 2},
            "directed_lane_penalties": {"0": 60, "1": 60},
            "algorithm": "penalty=weight*max(0,predicted_robots-free_capacity)",
        }
        explanation = explain_event(event)
        self.assertIn("원본 RMF", explanation)
        self.assertIn("R0", explanation)
        self.assertIn("60", explanation)
        self.assertEqual(decision_records([event])[0]["seq"], 30)

    def test_astar_breakdown_distinguishes_exact_and_diagnostic_values(self) -> None:
        text = explain_event({
            "seq": 50,
            "event": "astar_expand",
            "robot": "R0",
            "g": 5.0,
            "h": 7.0,
            "f": 12.0,
            "delta_g_from_parent": 2.2,
            "g_route_elapsed_s": 2.0,
            "g_translation_time_s": 1.0,
            "g_rotation_time_s": 0.7,
            "g_wait_time_s": 0.3,
            "g_translation_distance_m": 0.7,
            "g_rotation_angle_rad": 0.4,
            "g_unexposed_remainder": 0.2,
            "h_graph_distance_m": 4.2,
            "h_graph_cruise_time_s": 6.0,
            "h_first_turn_angle_rad": 0.3,
            "h_first_turn_time_s": 0.8,
            "h_rmf_minus_graph_cruise_s": 1.0,
        })
        self.assertIn("이동 1.000 s", text)
        self.assertIn("회전 0.700 s", text)
        self.assertIn("미노출 잔차", text)
        self.assertIn("검산용 하한", text)

    def test_schedule_model_states_real_object_and_flattening_boundary(self) -> None:
        text = schedule_model_text([
            {"event": "schedule_database_state", "phase": "registered",
             "latest_version": 2},
            {"event": "schedule_participant"},
            {"event": "schedule_database_route"},
            {"event": "schedule_database_trajectory_point"},
        ])
        self.assertIn("rmf_traffic::schedule::Database", text)
        self.assertIn("Viewer::View::Element", text)
        self.assertIn("평탄화", text)
        self.assertIn("별도 목업 DB", text)

    def test_negotiation_reject_and_forfeit_keep_raw_meaning_separate(self) -> None:
        action, label, detail = classify_negotiation_message(
            "Rejected parent [1,2]")
        self.assertEqual(action, "reject")
        self.assertIn("Reject", label)
        self.assertIn("거부", detail)
        action, label, detail = classify_negotiation_message(
            "Forfeited table [3,4]")
        self.assertEqual(action, "forfeit")
        self.assertIn("Forfeit", label)
        self.assertIn("전체 협상 실패", detail)

    def test_object_guide_explains_supergraph_and_proposal_boundary(self) -> None:
        text = rmf_object_guide_text([
            {"event": "negotiation_log", "message": "Submitted plan [1]"},
            {"event": "negotiation_log", "message": "Forfeited table [2]"},
        ])
        self.assertIn("Supergraph", text)
        self.assertIn("public/Planner::Debug API", text)
        self.assertIn("Result::proposal()", text)
        self.assertIn("Forfeit", text)

    def test_v3_astar_guide_separates_actual_overlap_and_admission_margin(self) -> None:
        from tools.event_explainer import astar_guide_text
        text = astar_guide_text()
        self.assertIn("DifferentialDrivePlanner", text)
        self.assertIn("admission_overlap_duration", text)
        self.assertIn("h와 trajectory timestamp는 바뀌지 않습니다", text)

    def test_failure_analysis_uses_actual_no_proposal_evidence(self) -> None:
        events = [
            {"seq": 1, "event": "planning_request", "robot": "R1", "start": 0, "goal": 2},
            {"seq": 2, "event": "plan_summary", "robot": "R1", "phase": "free_flow_baseline",
             "success": True, "cost": 3.0, "used_lanes": [0, 1]},
            {"seq": 3, "event": "negotiation_request", "robot_count": 2},
            {"seq": 4, "event": "proposal_summary", "present": False, "participant_plan_count": 0},
            {"seq": 5, "event": "negotiation_summary", "success": False},
            {"seq": 6, "event": "solution_diagnosis", "status": "no_solution",
             "category": "negotiation_no_proposal", "confidence": "medium",
             "basis": "confirmed_no_proposal_with_structural_inference",
             "root_cause": "No conflict-free proposal"},
        ]
        analysis = failure_analysis(events)
        self.assertEqual(analysis["primary_cause"], "NO_NEGOTIATION_ALTERNATIVE")
        self.assertEqual(analysis["proposal_present"], False)
        self.assertIn("UNKNOWN", analysis["negotiation_internal_alternative_count"])
        text = failure_summary_text(events)
        self.assertIn("Primary Cause: NO_NEGOTIATION_ALTERNATIVE", text)
        self.assertIn("공개 API", text)

    def test_failure_trace_keeps_unknown_location_instead_of_guessing(self) -> None:
        events = [
            {"seq": 1, "event": "pairwise_conflict_check", "robot_a": "R1",
             "robot_b": "R2", "passed": False, "earliest_conflict_time_s": 5.8,
             "route_pair_checks": 1, "method": "rmf_traffic::DetectConflict::between"},
            {"seq": 2, "event": "solution_diagnosis", "status": "no_solution",
             "category": "continuous_time_overlap", "confidence": "high",
             "basis": "confirmed_by_rmf_detect_conflict", "root_cause": "overlap"},
        ]
        rows = failure_trace_records(events)
        self.assertEqual(rows[0]["stage"], "CONFLICT")
        self.assertEqual(rows[0]["location"], "UNKNOWN")
        self.assertEqual(rows[0]["time"], "5.800 s")
        analysis = failure_analysis(events)
        self.assertEqual(analysis["primary_cause"], "SCHEDULE_CONFLICT")
        self.assertEqual(analysis["conflict_pair"], "R1 ↔ R2")

    def test_failure_analysis_marks_physical_escape_issue(self) -> None:
        analysis = failure_analysis([{
            "seq": 1, "event": "solution_diagnosis", "status": "no_solution",
            "category": "endpoint_exchange_without_buffer", "confidence": "high",
            "basis": "topology_inference_from_confirmed_no_proposal",
            "root_cause": "no independent buffer",
        }])
        self.assertEqual(analysis["primary_cause"], "NO_PHYSICAL_ESCAPE")
        self.assertIn("passing bay", analysis["improvement"])



if __name__ == "__main__":
    unittest.main()
