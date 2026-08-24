#!/usr/bin/env python3
"""Run reproducible multi-profile regressions against the native RMF lab runner."""

from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import json
import math
import re
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parent
PROFILE_ORDER = ("baseline", "soft", "schedule_soft", "hybrid", "hybrid_nego")
DEADLOCK_CATEGORIES = {
    "endpoint_exchange_without_buffer",
    "single_route_no_yield_space",
    "runner_timeout",
}


def _safe_name(value: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_-]+", "_", value).strip("_")
    return cleaned or "scenario"


def _events(path: Path) -> list[dict[str, Any]]:
    output: list[dict[str, Any]] = []
    if not path.is_file():
        return output
    for line in path.read_text(encoding="utf-8").splitlines():
        try:
            value = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict):
            output.append(value)
    return output


def _last(events: list[dict[str, Any]], event_name: str) -> dict[str, Any]:
    for event in reversed(events):
        if event.get("event") == event_name:
            return event
    return {}


def _termination_reason(
    diagnosis: str,
    events: list[dict[str, Any]],
    validator_rejects: int,
    hard_blocks: int,
) -> str:
    if not diagnosis:
        return "MISSING_TERMINATION_EVENT"
    if diagnosis == "runner_timeout":
        return "RUNNER_TIMEOUT"
    if diagnosis in {"planner_saturated", "saturation_limit", "search_saturation"}:
        return "SATURATION_LIMIT"
    if diagnosis in {
        "planner_disconnected", "disconnected_topology", "individual_path_missing",
    }:
        return "NO_VALID_ROUTE"
    if diagnosis in {"continuous_time_overlap", "dynamic_combined_plan_conflict"}:
        return "CONFLICT_DETECTED"
    if validator_rejects:
        return "VALIDATOR_REJECT"
    if hard_blocks and diagnosis != "executable_time_space_plan":
        return "POLICY_ADMISSION_EXHAUSTED"
    if diagnosis in DEADLOCK_CATEGORIES or diagnosis in {
        "negotiation_no_proposal", "dynamic_newcomer_no_proposal",
    }:
        return "NEGOTIATION_FAILED"
    if any(
        event.get("event") == "astar_trace_summary"
        and event.get("step_limit_reached")
        for event in events
    ):
        return "SEARCH_EXHAUSTED"
    if diagnosis == "executable_time_space_plan":
        return "SUCCESS"
    if diagnosis in {"free_flow_route_found", "dynamic_all_insertions_committed"}:
        return "SUCCESS"
    if diagnosis in {"planner_no_solution", "planner_interrupted"}:
        return "SEARCH_EXHAUSTED"
    return diagnosis.upper()


def summarize_jsonl(path: Path, profile: str = "") -> dict[str, Any]:
    events = _events(path)
    profile_event = _last(events, "runner_core_profile")
    traits_event = _last(events, "vehicle_traits")
    diagnosis_event = _last(events, "solution_diagnosis")
    safety_events = [e for e in events if e.get("event") == "safety_verification"]
    negotiation_events = [e for e in events if e.get("event") == "negotiation_summary"]
    policy_events = [e for e in events if e.get("event") == "corridor_policy_expansion"]
    snapshot_events = [e for e in events if e.get("event") == "corridor_policy_snapshot"]
    validator_rejects = sum(
        e.get("event") == "route_validator_result"
        and e.get("decision") != "ACCEPT"
        for e in events
    )
    hard_blocks = sum(
        e.get("decision") == "HARD_CORRIDOR_BLOCK" for e in policy_events)
    conflicts = sum(int(e.get("conflicts", 0) or 0) for e in safety_events)
    if not safety_events:
        conflicts = sum(
            e.get("event") == "pairwise_conflict_check"
            and not e.get("passed", True)
            for e in events
        )

    successful_plans = [
        e for e in events if e.get("event") == "plan_summary" and e.get("success")]
    plan_phases = {str(e.get("phase", "")) for e in successful_plans}
    final_phase = (
        "negotiated" if "negotiated" in plan_phases else
        "free_flow" if "free_flow" in plan_phases else "")
    final_plans = [e for e in successful_plans if e.get("phase") == final_phase]
    baseline_plans = [e for e in successful_plans if e.get("phase") == "free_flow_baseline"]
    final_paths = {
        str(e.get("robot")): list(e.get("used_lanes", [])) for e in final_plans}
    baseline_paths = {
        str(e.get("robot")): list(e.get("used_lanes", [])) for e in baseline_plans}

    final_waypoints = [
        e for e in events
        if e.get("event") == "plan_waypoint" and e.get("phase") == final_phase]
    total_distance = sum(float(e.get("delta_distance_m", 0) or 0) for e in final_waypoints)
    total_wait = sum(
        float(e.get("delta_time_s", 0) or 0)
        for e in final_waypoints if e.get("movement_type") == "wait")
    start_times = {
        str(e.get("robot")): float(e.get("start_time_s", 0) or 0)
        for e in events if e.get("event") == "planning_request"}
    finish_times = {
        str(e.get("robot")): float(e.get("finish_time_s", 0) or 0)
        for e in final_plans}
    makespan = max(finish_times.values(), default=0.0) - min(
        start_times.values(), default=0.0)
    makespan = max(0.0, makespan)
    total_travel_time = sum(
        max(0.0, finish - start_times.get(robot, 0.0))
        for robot, finish in finish_times.items())

    planning_ms = sum(
        float(e.get("elapsed_ms", 0) or 0)
        for e in events if e.get("event") == "planner_timing")
    negotiation_ms = sum(float(e.get("elapsed_ms", 0) or 0) for e in negotiation_events)
    expanded = sum(
        int(e.get("expansions", 0) or 0)
        for e in events if e.get("event") == "astar_trace_summary")
    negotiation_rounds = sum(
        e.get("event") == "negotiation_log" and e.get("action") == "select_table"
        for e in events)
    observed_penalty = sum(
        float(e.get("total_policy_penalty", 0) or 0) for e in policy_events)
    diagnosis = str(diagnosis_event.get("category", ""))
    solution = str(diagnosis_event.get("status", "unknown"))
    success = solution == "solved"
    deadlock = diagnosis in DEADLOCK_CATEGORIES
    rerouted = any(
        robot in baseline_paths and lanes != baseline_paths[robot]
        for robot, lanes in final_paths.items())
    baseline_distance = 0.0
    if baseline_paths:
        baseline_waypoints = [
            e for e in events
            if e.get("event") == "plan_waypoint"
            and e.get("phase") == "free_flow_baseline"]
        baseline_distance = sum(
            float(e.get("delta_distance_m", 0) or 0) for e in baseline_waypoints)

    shared_input = {
        "scenario_sha256": profile_event.get("scenario_sha256", ""),
        "random_seed": profile_event.get("random_seed", ""),
        "vehicle_traits": traits_event,
    }
    input_signature = hashlib.sha256(json.dumps(
        shared_input, ensure_ascii=False, sort_keys=True,
        separators=(",", ":")).encode("utf-8")).hexdigest()
    return {
        "profile": profile or str(profile_event.get("traffic_policy_mode", "")),
        "result": "SUCCESS" if success else "NO_SOLUTION",
        "success": success,
        "conflict": conflicts > 0,
        "conflict_count": conflicts,
        "deadlock": deadlock,
        "travel_time_s": round(total_travel_time, 6),
        "makespan_s": round(makespan, 6),
        "wait_time_s": round(total_wait, 6),
        "distance_m": round(total_distance, 6),
        "baseline_distance_m": round(baseline_distance, 6),
        "detour_distance_m": round(max(0.0, total_distance - baseline_distance), 6)
        if baseline_distance > 0 else None,
        "detour": rerouted or (
            baseline_distance > 0 and total_distance > baseline_distance + 1e-6),
        "planning_time_ms": round(planning_ms, 6),
        "negotiation_time_ms": round(negotiation_ms, 6),
        "expanded_nodes": expanded,
        "negotiation_count": len(negotiation_events),
        "negotiation_rounds": negotiation_rounds,
        "validator_rejects": validator_rejects,
        "hard_admission_blocks": hard_blocks,
        "observed_penalty_sum": round(observed_penalty, 6),
        "schedule_snapshot_count": len(snapshot_events),
        "schedule_snapshot_version": (snapshot_events[-1].get("schedule_version", "")
            if snapshot_events else ""),
        "schedule_query_count": sum(int(e.get("schedule_query_count", 0) or 0)
            for e in snapshot_events),
        "queried_participant_count": sum(int(e.get("queried_participant_count", 0) or 0)
            for e in snapshot_events),
        "queried_route_count": sum(int(e.get("queried_route_count", 0) or 0)
            for e in snapshot_events),
        "self_filtered_route_count": sum(int(e.get("self_filtered_route_count", 0) or 0)
            for e in snapshot_events),
        "overlap_check_count": sum(int(e.get("overlap_check_count", 0) or 0)
            for e in policy_events),
        "diagnosis": diagnosis,
        "termination_reason": _termination_reason(
            diagnosis, events, validator_rejects, hard_blocks),
        "final_paths": final_paths,
        "scenario_sha256": profile_event.get("scenario_sha256", ""),
        "random_seed": profile_event.get("random_seed", ""),
        "vehicle_traits": traits_event,
        "shared_input_signature": input_signature,
        "actual_traffic_mode": profile_event.get("traffic_policy_mode", ""),
        "rmf_library": profile_event.get("resolved_rmf_library", ""),
        "rmf_source": profile_event.get("rmf_source", ""),
        "rmf_commit": profile_event.get("rmf_source_commit", ""),
        "rmf_source_dirty": profile_event.get("rmf_source_dirty"),
        "rmf_source_diff_sha256": profile_event.get("rmf_source_diff_sha256", ""),
        "jsonl": str(path),
    }


def classify_against_baseline(
    baseline: dict[str, Any], modified: dict[str, Any]) -> tuple[str, list[str]]:
    reasons: list[str] = []
    if baseline.get("success") and not modified.get("success"):
        reasons.append("Baseline SUCCESS → Modified NO_SOLUTION")
    if baseline.get("success") and modified.get("deadlock"):
        reasons.append("Baseline 정상 → Modified Deadlock")
    if baseline.get("success") and modified.get("conflict"):
        reasons.append("Baseline 정상 → Modified Conflict")
    if reasons:
        return "REGRESSION", reasons
    if (baseline.get("deadlock") or baseline.get("conflict") or not baseline.get("success")) \
            and modified.get("success") and not modified.get("conflict"):
        return "IMPROVEMENT", ["Baseline 실패/위험 → Modified 안전한 SUCCESS"]
    return "NO_CHANGE", []


@dataclass(frozen=True)
class Profile:
    name: str
    setup: str
    source: str
    workspace: str = ""
    rebuild_workspace: bool = False


def _run_command(
    scenario_path: Path,
    profile: Profile,
    config: dict[str, Any],
    result_name: str,
    build_dir: Path,
    *,
    skip_build: bool,
    rebuild_workspace: bool,
    dynamic: bool,
) -> list[str]:
    weights = config.get("weights", {})
    command = [
        sys.executable, str(ROOT / "run.py"),
        "--scenario-file", str(scenario_path),
        "--timeout", str(config.get("timeout_s", 60)),
        "--no-html", "--build-dir", str(build_dir),
        "--result-name", result_name,
        "--core-label", profile.name,
        "--traffic-mode", profile.name,
        "--random-seed", str(config.get("random_seed", 0)),
        "--same-direction-weight", str(weights.get("same", 0.25)),
        "--opposite-direction-weight", str(weights.get("opposite", 8.0)),
        "--occupied-weight", str(weights.get("occupied", 1.5)),
        "--future-reservation-weight", str(weights.get("future", 0.6)),
        "--no-escape-weight", str(weights.get("no_escape", 25.0)),
        "--static-policy-weight", str(weights.get("static", 0.0)),
        "--overlap-margin", str(weights.get("overlap_margin", 0.25)),
        "--schedule-soft-lambda", str(weights.get("schedule_soft_lambda", 0.25)),
        "--schedule-soft-max-penalty", str(weights.get("schedule_soft_max_penalty", 10.0)),
        "--schedule-soft-same-weight", str(weights.get("schedule_soft_same_weight", 0.5)),
        "--schedule-soft-opposite-weight", str(weights.get("schedule_soft_opposite_weight", 1.5)),
        "--lane-penalty-value", str(config.get("newcomer_penalty", 60.0)),
        "--dynamic-insertion-policy",
        "after_nego" if profile.name == "hybrid_nego" and dynamic
        else "fixed_existing",
    ]
    if profile.setup:
        command.extend(["--setup", str(Path(profile.setup).expanduser())])
    if profile.source:
        command.extend(["--rmf-source", str(Path(profile.source).expanduser())])
    if skip_build:
        command.append("--skip-build")
    if rebuild_workspace and profile.workspace:
        command.extend([
            "--rebuild-rmf-workspace", str(Path(profile.workspace).expanduser()),
            "--base-ros-setup", str(Path(config.get(
                "base_ros_setup", "/opt/ros/jazzy/setup.bash")).expanduser()),
        ])
    return command


def run_regression(config_path: Path) -> Path:
    config = json.loads(config_path.read_text(encoding="utf-8"))
    run_id = str(config.get("run_id") or datetime.now(
        timezone.utc).strftime("%Y%m%dT%H%M%SZ"))
    output_root = ROOT / "results" / "regression" / _safe_name(run_id)
    output_root.mkdir(parents=True, exist_ok=True)
    profiles = {
        name: Profile(name=name, **config["profiles"][name])
        for name in PROFILE_ORDER}
    scenarios = config.get("scenarios", [])
    if not scenarios:
        raise ValueError("Regression config has no scenarios")

    built_profiles: set[str] = set()
    rebuilt_workspaces: set[str] = set()
    scenario_summaries: list[dict[str, Any]] = []
    for scenario_index, scenario in enumerate(scenarios):
        document = copy.deepcopy(scenario["document"])
        scenario_name = str(scenario.get("name") or document.get("name") or "scenario")
        scenario_dir = output_root / f"{scenario_index:03d}_{_safe_name(scenario_name)}"
        scenario_dir.mkdir(parents=True, exist_ok=True)
        input_path = scenario_dir / "input.json"
        input_path.write_text(
            json.dumps(document, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        input_sha = hashlib.sha256(input_path.read_bytes()).hexdigest()
        dynamic = any(
            float(robot.get("insertion_time_s", 0) or 0) > 0
            for robot in document.get("robots", []))
        results: dict[str, dict[str, Any]] = {}
        for profile_name in PROFILE_ORDER:
            profile = profiles[profile_name]
            result_name = _safe_name(
                f"reg_{run_id}_{scenario_index:03d}_{profile_name}")
            build_dir = ROOT / "build" / "regression" / profile_name
            workspace_key = str(Path(profile.workspace).expanduser()) if profile.workspace else ""
            rebuild_workspace = (
                profile.rebuild_workspace and workspace_key not in rebuilt_workspaces)
            command = _run_command(
                input_path, profile, config, result_name, build_dir,
                skip_build=profile_name in built_profiles,
                rebuild_workspace=rebuild_workspace,
                dynamic=dynamic)
            print(
                f"REGRESSION {scenario_index + 1}/{len(scenarios)} "
                f"{scenario_name} · {profile_name}", flush=True)
            completed = subprocess.run(
                command, cwd=ROOT, text=True,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=False)
            (scenario_dir / f"{profile_name}.log").write_text(
                completed.stdout, encoding="utf-8")
            global_result = ROOT / "results" / f"{result_name}.jsonl"
            archived_result = scenario_dir / f"{profile_name}.jsonl"
            if global_result.is_file():
                archived_result.write_bytes(global_result.read_bytes())
            summary = summarize_jsonl(archived_result, profile_name)
            summary["process_exit_code"] = completed.returncode
            summary["input_sha256"] = input_sha
            summary["input_identity_ok"] = summary.get("scenario_sha256") == input_sha
            results[profile_name] = summary
            built_profiles.add(profile_name)
            if rebuild_workspace and workspace_key:
                rebuilt_workspaces.add(workspace_key)

        baseline = results["baseline"]
        for profile_name in PROFILE_ORDER:
            if profile_name == "baseline":
                results[profile_name]["comparison"] = "REFERENCE"
                results[profile_name]["comparison_reasons"] = []
            else:
                verdict, reasons = classify_against_baseline(
                    baseline, results[profile_name])
                results[profile_name]["comparison"] = verdict
                results[profile_name]["comparison_reasons"] = reasons
        identity_values = {
            result.get("shared_input_signature") for result in results.values()}
        libraries = {
            name: str(result.get("rmf_library") or "")
            for name, result in results.items()}
        mode_match = all(
            results[name].get("actual_traffic_mode") == name
            for name in PROFILE_ORDER)
        core_provenance = {
            "mode_labels_match": mode_match,
            "baseline_distinct_from_modified": all(
                libraries["baseline"]
                and libraries[name]
                and libraries["baseline"] != libraries[name]
                for name in ("soft", "schedule_soft", "hybrid", "hybrid_nego")),
            "soft_hybrid_same_library": (
                bool(libraries["soft"])
                and libraries["soft"] == libraries["hybrid"]),
            "schedule_soft_distinct_library": (
                bool(libraries["schedule_soft"])
                and libraries["schedule_soft"] != libraries["baseline"]
                and libraries["schedule_soft"] != libraries["soft"]),
            "hybrid_nego_distinct_library": (
                bool(libraries["hybrid_nego"])
                and libraries["hybrid_nego"] != libraries["hybrid"]),
            "libraries": libraries,
        }
        core_provenance["verified"] = all((
            core_provenance["mode_labels_match"],
            core_provenance["baseline_distinct_from_modified"],
            core_provenance["soft_hybrid_same_library"],
            core_provenance["schedule_soft_distinct_library"],
            core_provenance["hybrid_nego_distinct_library"],
        ))
        scenario_summaries.append({
            "scenario": scenario_name,
            "input_sha256": input_sha,
            "identical_input": (
                len(identity_values) == 1
                and all(result.get("input_identity_ok") for result in results.values())),
            "core_provenance": core_provenance,
            "results": results,
        })
        print(
            f"REGRESSION_DONE {scenario_name} · "
            + " · ".join(
                f"{name}={results[name]['result']}"
                for name in PROFILE_ORDER), flush=True)

    totals: dict[str, Any] = {
        "scenario_count": len(scenario_summaries),
        "profiles": {}, "regressions": 0, "improvements": 0,
        "input_identity_failures": sum(
            not scenario["identical_input"] for scenario in scenario_summaries),
        "core_provenance_failures": sum(
            not scenario["core_provenance"]["verified"]
            for scenario in scenario_summaries),
    }
    for profile_name in PROFILE_ORDER:
        profile_results = [s["results"][profile_name] for s in scenario_summaries]
        totals["profiles"][profile_name] = {
            "success": sum(r["success"] for r in profile_results),
            "no_solution": sum(not r["success"] for r in profile_results),
            "deadlock": sum(r["deadlock"] for r in profile_results),
            "conflict": sum(r["conflict"] for r in profile_results),
        }
        if profile_name != "baseline":
            totals["regressions"] += sum(
                r["comparison"] == "REGRESSION" for r in profile_results)
            totals["improvements"] += sum(
                r["comparison"] == "IMPROVEMENT" for r in profile_results)

    report = {
        "schema": "rmf_lab_regression.v1",
        "run_id": run_id,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "random_seed": int(config.get("random_seed", 0)),
        "profile_order": list(PROFILE_ORDER),
        "totals": totals,
        "scenarios": scenario_summaries,
    }
    summary_path = output_root / "summary.json"
    summary_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    with (output_root / "summary.csv").open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow([
            "scenario", "profile", "result", "comparison", "conflict", "deadlock",
            "travel_time_s", "wait_time_s", "distance_m", "detour",
            "planning_time_ms", "expanded_nodes", "negotiation_count",
            "negotiation_rounds", "validator_rejects", "observed_penalty_sum",
            "schedule_query_count", "queried_route_count",
            "self_filtered_route_count", "overlap_check_count",
            "termination_reason", "scenario_sha256"])
        for scenario in scenario_summaries:
            for profile_name in PROFILE_ORDER:
                item = scenario["results"][profile_name]
                writer.writerow([
                    scenario["scenario"], profile_name, item["result"],
                    item["comparison"], item["conflict"], item["deadlock"],
                    item["travel_time_s"], item["wait_time_s"], item["distance_m"],
                    item["detour"], item["planning_time_ms"], item["expanded_nodes"],
                    item["negotiation_count"], item["negotiation_rounds"],
                    item["validator_rejects"], item["observed_penalty_sum"],
                    item["schedule_query_count"], item["queried_route_count"],
                    item["self_filtered_route_count"], item["overlap_check_count"],
                    item["termination_reason"], item["scenario_sha256"]])
    print(f"REGRESSION_SUMMARY {summary_path}", flush=True)
    return summary_path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    args = parser.parse_args()
    run_regression(args.config.expanduser().resolve())
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyError) as exc:
        print(f"REGRESSION_ERROR {exc}", file=sys.stderr)
        raise SystemExit(1)
