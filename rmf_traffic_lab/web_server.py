#!/usr/bin/env python3
"""Browser server for the RMF Traffic Core Analyzer Lab.

The server deliberately uses only Python's standard library. It serves the
single-page editor, serializes RMF build/run jobs, streams stdout and JSONL over
Server-Sent Events, and keeps every run in an isolated evidence directory.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import mimetypes
import os
import queue
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
import traceback
import uuid
from dataclasses import dataclass, field
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, unquote, urlparse

from tools.event_explainer import (
    astar_guide_text,
    decision_records,
    diagnosis_text,
    explain_runtime_output,
    schedule_guide_text,
    summarize_jsonl,
)
from tools.building_map_import import convert_building_map_yaml
from tools.scenario_templates import builtin_scenarios
from tools.setup_after_core import after_core_patch_status, prepare_after_core


ROOT = Path(__file__).resolve().parent
WEB_ROOT = ROOT / "web"
DATA_ROOT = ROOT / "web_data"
RUN_ROOT = DATA_ROOT / "runs"
SAVED_SCENARIO_ROOT = DATA_ROOT / "scenarios"
RESULT_ROOT = ROOT / "results"
MAX_REQUEST_BYTES = 20 * 1024 * 1024
FINAL_STATES = {"completed", "failed", "timeout", "cancelled"}


def _env_bool(name: str, fallback: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return fallback
    return raw.strip().lower() in {"1", "true", "yes", "on"}


SERVER_CONFIG = {
    "before_setup": os.environ.get(
        "RMF_LAB_BEFORE_SETUP", "~/rmf_ws/install/setup.bash"),
    "before_source": os.environ.get(
        "RMF_LAB_BEFORE_SOURCE", "~/rmf_ws/src/rmf_traffic"),
    "after_workspace": os.environ.get(
        "RMF_LAB_AFTER_WORKSPACE", "~/rmf_ws_modified"),
    "after_setup": os.environ.get(
        "RMF_LAB_AFTER_SETUP", "~/rmf_ws_modified/install/setup.bash"),
    "after_source": os.environ.get(
        "RMF_LAB_AFTER_SOURCE", "~/rmf_ws_modified/src/rmf_traffic"),
    "base_ros_setup": os.environ.get(
        "RMF_LAB_BASE_ROS_SETUP", "/opt/ros/jazzy/setup.bash"),
    "after_label": os.environ.get(
        "RMF_LAB_AFTER_LABEL", "after_lane_penalty"),
    "allow_core_patch": _env_bool("RMF_LAB_ALLOW_CORE_PATCH", True),
    "allow_path_overrides": _env_bool("RMF_LAB_ALLOW_PATH_OVERRIDES", True),
    "access_token_required": bool(os.environ.get("RMF_LAB_TOKEN", "")),
}


def _json_dumps(value: Any) -> bytes:
    return json.dumps(value, ensure_ascii=False, separators=(",", ":")).encode("utf-8")


def _safe_text(value: Any, label: str, *, allow_empty: bool = True) -> str:
    text = str(value or "").strip()
    if not allow_empty and not text:
        raise ValueError(f"{label} 값이 비어 있습니다")
    if any(character in text for character in ("\x00", "\r", "\n")):
        raise ValueError(f"{label}에 줄바꿈 또는 NUL 문자를 사용할 수 없습니다")
    return text


def _safe_path(value: Any, label: str, fallback: str) -> str:
    selected = _safe_text(value, label) if SERVER_CONFIG["allow_path_overrides"] else fallback
    selected = selected or fallback
    return str(Path(selected).expanduser())


def _finite_number(value: Any, label: str, minimum: float, maximum: float) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{label}은 숫자여야 합니다")
    number = float(value)
    if not math.isfinite(number) or not minimum <= number <= maximum:
        raise ValueError(f"{label}은 {minimum:g}~{maximum:g} 범위의 유한한 값이어야 합니다")
    return number


def validate_document(document: Any) -> dict[str, Any]:
    if not isinstance(document, dict):
        raise ValueError("시나리오는 JSON 객체여야 합니다")
    payload = copy.deepcopy(document)
    for field_name in ("nodes", "lanes", "robots"):
        if not isinstance(payload.get(field_name), list):
            raise ValueError(f"시나리오의 {field_name}는 배열이어야 합니다")
    if not payload["nodes"]:
        raise ValueError("노드가 한 개 이상 필요합니다")
    node_count = len(payload["nodes"])
    for index, node in enumerate(payload["nodes"]):
        if not isinstance(node, dict):
            raise ValueError(f"nodes[{index}]가 객체가 아닙니다")
        node["name"] = _safe_text(node.get("name", f"N{index}"), f"nodes[{index}].name", allow_empty=False)
        node["x"] = _finite_number(node.get("x"), f"nodes[{index}].x", -100000, 100000)
        node["y"] = _finite_number(node.get("y"), f"nodes[{index}].y", -100000, 100000)
    for index, lane in enumerate(payload["lanes"]):
        if not isinstance(lane, dict):
            raise ValueError(f"lanes[{index}]가 객체가 아닙니다")
        entry = int(lane.get("from", -1))
        exit = int(lane.get("to", -1))
        if not 0 <= entry < node_count or not 0 <= exit < node_count or entry == exit:
            raise ValueError(f"lanes[{index}]의 from/to 노드가 올바르지 않습니다")
        lane["from"], lane["to"] = entry, exit
    names: set[str] = set()
    for index, robot in enumerate(payload["robots"]):
        if not isinstance(robot, dict):
            raise ValueError(f"robots[{index}]가 객체가 아닙니다")
        name = _safe_text(robot.get("name", f"R{index}"), f"robots[{index}].name", allow_empty=False)
        if name in names:
            raise ValueError(f"중복 로봇 이름: {name}")
        names.add(name)
        start, goal = int(robot.get("start", -1)), int(robot.get("goal", -1))
        if not 0 <= start < node_count or not 0 <= goal < node_count:
            raise ValueError(f"robots[{index}]의 start/goal 노드가 올바르지 않습니다")
        robot.update({
            "name": name,
            "start": start,
            "goal": goal,
            "yaw": _finite_number(robot.get("yaw", 0), f"robots[{index}].yaw", -1000, 1000),
            "start_time_s": _finite_number(
                robot.get("start_time_s", robot.get("start_time", 0)),
                f"robots[{index}].start_time_s", 0, 86400),
        })
    payload["name"] = _safe_text(payload.get("name", "web_scenario"), "name", allow_empty=False)
    payload["description"] = _safe_text(payload.get("description", ""), "description")
    payload["map"] = _safe_text(payload.get("map", "L1"), "map", allow_empty=False)
    payload["mode"] = _safe_text(payload.get("mode", "auto"), "mode", allow_empty=False)
    payload.setdefault("closed_lanes", [])
    return payload


def scenario_catalog() -> dict[str, dict[str, Any]]:
    catalog = builtin_scenarios()
    for path in sorted((ROOT / "scenarios").glob("*.json")):
        try:
            catalog[f"example__{path.stem}"] = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
    for path in sorted(SAVED_SCENARIO_ROOT.glob("*.json")):
        try:
            catalog[f"saved__{path.stem}"] = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
    return catalog


def _last(events: list[dict], event_name: str) -> dict:
    matches = [event for event in events if event.get("event") == event_name]
    return matches[-1] if matches else {}


def summarize_result(events: list[dict]) -> dict[str, Any]:
    if not events:
        return {}
    successful = [
        event for event in events
        if event.get("event") == "plan_summary" and event.get("success")
    ]
    phases = {event.get("phase") for event in successful}
    selected_phase = "negotiated" if "negotiated" in phases else "free_flow" if "free_flow" in phases else ""
    plans = [event for event in successful if event.get("phase") == selected_phase]
    baselines = [event for event in successful if event.get("phase") == "free_flow_baseline"]
    route_map = {str(event.get("robot")): list(event.get("used_lanes", [])) for event in plans}
    baseline_map = {str(event.get("robot")): list(event.get("used_lanes", [])) for event in baselines}
    finish_map = {str(event.get("robot")): float(event.get("finish_time_s", 0) or 0) for event in plans}
    baseline_finish = {str(event.get("robot")): float(event.get("finish_time_s", 0) or 0) for event in baselines}
    lane_lengths = {
        int(event["id"]): float(event.get("length_m", 0) or 0)
        for event in events if event.get("event") == "graph_lane" and event.get("id") is not None
    }
    distance = sum(lane_lengths.get(int(lane), 0) for lanes in route_map.values() for lane in lanes)
    baseline_distance = sum(lane_lengths.get(int(lane), 0) for lanes in baseline_map.values() for lane in lanes)
    rerouted = [robot for robot in route_map if robot in baseline_map and route_map[robot] != baseline_map[robot]]
    rescheduled = [
        robot for robot in route_map
        if robot in baseline_map and route_map[robot] == baseline_map[robot]
        and abs(finish_map.get(robot, 0) - baseline_finish.get(robot, 0)) > 1e-6
    ]
    profile = _last(events, "runner_core_profile")
    diagnosis = _last(events, "solution_diagnosis")
    negotiation = _last(events, "negotiation_summary")
    safety = _last(events, "safety_verification")
    schedule = _last(events, "schedule_database_state")
    astar = [event for event in events if event.get("event") == "astar_trace_summary"]
    penalty_configuration = _last(events, "occupancy_penalty_configuration")
    return {
        "core_label": profile.get("label", ""),
        "rmf_commit": profile.get("rmf_source_commit", ""),
        "rmf_library": profile.get("resolved_rmf_library", ""),
        "scenario_sha256": profile.get("scenario_sha256", ""),
        "penalty_active": bool(profile.get("lane_penalty_active", False)),
        "penalty_mode": profile.get("lane_penalty_mode", ""),
        "penalty_lanes": profile.get(
            "directed_lane_penalties", penalty_configuration.get("directed_lane_penalties", {})),
        "occupancy": profile.get(
            "directed_lane_occupancy", penalty_configuration.get("directed_lane_occupancy", {})),
        "solution": diagnosis.get("status", "unknown"),
        "diagnosis": diagnosis.get("category", ""),
        "negotiation_success": negotiation.get("success", ""),
        "executable": negotiation.get("executable_plan", ""),
        "safety_verified": safety.get("passed", negotiation.get("safety_verified", "")),
        "negotiation_ms": negotiation.get("planning_time_ms", negotiation.get("duration_ms", "")),
        "routes": route_map,
        "baseline_routes": baseline_map,
        "rerouted_robots": rerouted,
        "rescheduled_robots": rescheduled,
        "route_distance_m": round(distance, 6),
        "baseline_distance_m": round(baseline_distance, 6),
        "completion_s": round(max(finish_map.values(), default=0.0), 6),
        "plan_cost": round(sum(float(event.get("cost", 0) or 0) for event in plans), 6),
        "astar_expansions": sum(int(event.get("expansions", 0) or 0) for event in astar),
        "schedule_phase": schedule.get("phase", ""),
        "stored_routes": schedule.get("stored_route_count", 0),
        "stored_points": schedule.get("stored_trajectory_point_count", 0),
    }


def comparison_payload(before: dict[str, Any], after: dict[str, Any]) -> dict[str, Any]:
    keys = [
        ("입력 Scenario SHA-256", "scenario_sha256"),
        ("RMF Core label", "core_label"),
        ("실제 RMF 라이브러리", "rmf_library"),
        ("Penalty 활성", "penalty_active"),
        ("Penalty 모드", "penalty_mode"),
        ("최종 해 상태", "solution"),
        ("진단 분류", "diagnosis"),
        ("협상 성공", "negotiation_success"),
        ("실행 가능한 협상안", "executable"),
        ("연속시간 안전검사", "safety_verified"),
        ("최종 로봇별 Lane", "routes"),
        ("자유경로 기준 Lane", "baseline_routes"),
        ("협상 우회 로봇", "rerouted_robots"),
        ("시간축 조정 로봇", "rescheduled_robots"),
        ("최종 경로 총거리(m)", "route_distance_m"),
        ("전체 완료시간(s)", "completion_s"),
        ("최종 계획비용 합", "plan_cost"),
        ("A* 확장 수", "astar_expansions"),
        ("Schedule DB 최종 단계", "schedule_phase"),
        ("DB 저장 Route", "stored_routes"),
        ("DB 저장 Trajectory point", "stored_points"),
    ]
    rows = []
    for label, key in keys:
        left, right = before.get(key, ""), after.get(key, "")
        delta: Any = ""
        if isinstance(left, (int, float)) and not isinstance(left, bool) and isinstance(right, (int, float)) and not isinstance(right, bool):
            delta = round(float(right) - float(left), 6)
        elif key == "scenario_sha256" and left and right:
            delta = "동일 입력" if left == right else "입력 다름"
        elif key == "routes":
            changed = sorted(
                robot for robot in set(left or {}) | set(right or {})
                if (left or {}).get(robot) != (right or {}).get(robot))
            delta = f"{len(changed)}대 경로 변경" if changed else "Lane 동일"
        rows.append({"label": label, "before": left, "after": right, "change": delta})
    same_input = bool(before.get("scenario_sha256")) and before.get("scenario_sha256") == after.get("scenario_sha256")
    changed_robots = sorted(
        robot for robot in set(before.get("routes", {})) | set(after.get("routes", {}))
        if before.get("routes", {}).get(robot) != after.get("routes", {}).get(robot))
    explanation = [
        "Before/After 성과 판정",
        "",
        f"1. 입력 동일성: {'통과 · 같은 시나리오' if same_input else '확인 필요 · Scenario SHA가 다름'}",
        f"2. 경로 재생성: {'확인 · ' + ', '.join(changed_robots) if changed_robots else 'Lane 조합 변화 없음'}",
        f"3. 해 상태: {before.get('solution', '없음')} → {after.get('solution', '없음')}",
        f"4. 우회 로봇: {after.get('rerouted_robots', [])}",
        f"5. 시간축 조정 로봇: {after.get('rescheduled_robots', [])}",
        f"6. 안전검사: {before.get('safety_verified')} → {after.get('safety_verified')}",
        f"7. 총거리: {before.get('route_distance_m', 0)} → {after.get('route_distance_m', 0)} m",
        f"8. 완료시간: {before.get('completion_s', 0)} → {after.get('completion_s', 0)} s",
        "",
        "우회 성공은 Lane 변경만으로 판단하지 않습니다. 실행 가능한 협상안, 연속시간 충돌검사 통과, Schedule DB Route 저장까지 함께 확인해야 합니다.",
    ]
    return {"before": before, "after": after, "rows": rows, "explanation": "\n".join(explanation)}


@dataclass
class RunRecord:
    run_id: str
    profile: str
    label: str
    document: dict[str, Any]
    options: dict[str, Any]
    run_dir: Path
    result_name: str
    status: str = "queued"
    created_at: float = field(default_factory=time.time)
    started_at: float | None = None
    finished_at: float | None = None
    exit_code: int | None = None
    error: str = ""
    log_lines: list[str] = field(default_factory=list)
    events: list[dict] = field(default_factory=list)
    process: subprocess.Popen[str] | None = None
    stop_requested: bool = False
    lock: threading.RLock = field(default_factory=threading.RLock, repr=False)

    @property
    def global_result_path(self) -> Path:
        return RESULT_ROOT / f"{self.result_name}.jsonl"

    @property
    def archived_result_path(self) -> Path:
        return self.run_dir / "result.jsonl"

    def append_log(self, text: str) -> None:
        with self.lock:
            self.log_lines.append(text)
        with (self.run_dir / "runtime.log").open("a", encoding="utf-8") as stream:
            stream.write(text)

    def refresh_events(self) -> None:
        path = self.global_result_path if self.global_result_path.is_file() else self.archived_result_path
        if not path.is_file():
            return
        parsed: list[dict] = []
        try:
            for raw in path.read_text(encoding="utf-8").splitlines():
                try:
                    parsed.append(json.loads(raw))
                except json.JSONDecodeError:
                    continue
        except OSError:
            return
        with self.lock:
            self.events = parsed

    def public_state(self) -> dict[str, Any]:
        with self.lock:
            return {
                "run_id": self.run_id,
                "profile": self.profile,
                "label": self.label,
                "status": self.status,
                "created_at": self.created_at,
                "started_at": self.started_at,
                "finished_at": self.finished_at,
                "exit_code": self.exit_code,
                "error": self.error,
                "log_count": len(self.log_lines),
                "event_count": len(self.events),
                "document_name": self.document.get("name", ""),
                "result_available": self.archived_result_path.is_file() or self.global_result_path.is_file(),
            }

    def save_meta(self) -> None:
        meta = self.public_state() | {"options": self.options}
        (self.run_dir / "meta.json").write_text(
            json.dumps(meta, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


class RunManager:
    def __init__(self) -> None:
        RUN_ROOT.mkdir(parents=True, exist_ok=True)
        SAVED_SCENARIO_ROOT.mkdir(parents=True, exist_ok=True)
        RESULT_ROOT.mkdir(parents=True, exist_ok=True)
        self.records: dict[str, RunRecord] = {}
        self.queue: queue.Queue[str] = queue.Queue()
        self.lock = threading.RLock()
        self.latest_by_profile: dict[str, str] = {}
        self._load_archived_runs()
        self.worker = threading.Thread(target=self._worker_loop, name="rmf-web-runner", daemon=True)
        self.worker.start()

    def _load_archived_runs(self) -> None:
        for run_dir in sorted(RUN_ROOT.glob("*")):
            meta_path = run_dir / "meta.json"
            scenario_path = run_dir / "scenario.json"
            if not meta_path.is_file() or not scenario_path.is_file():
                continue
            try:
                meta = json.loads(meta_path.read_text(encoding="utf-8"))
                document = json.loads(scenario_path.read_text(encoding="utf-8"))
                status = str(meta.get("status", "failed"))
                if status not in FINAL_STATES:
                    status = "failed"
                record = RunRecord(
                    run_id=str(meta["run_id"]),
                    profile=str(meta.get("profile", "before")),
                    label=str(meta.get("label", "")),
                    document=document,
                    options=dict(meta.get("options", {})),
                    run_dir=run_dir,
                    result_name=f"web_{meta.get('profile', 'before')}_{meta['run_id']}",
                    status=status,
                    created_at=float(meta.get("created_at", 0) or 0),
                    started_at=meta.get("started_at"),
                    finished_at=meta.get("finished_at"),
                    exit_code=meta.get("exit_code"),
                    error=str(meta.get("error", "")),
                )
                log_path = run_dir / "runtime.log"
                if log_path.is_file():
                    record.log_lines = log_path.read_text(encoding="utf-8", errors="replace").splitlines(True)
                record.refresh_events()
                self.records[record.run_id] = record
                previous = self.latest_by_profile.get(record.profile)
                if previous is None or self.records[previous].created_at < record.created_at:
                    self.latest_by_profile[record.profile] = record.run_id
            except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
                continue

    def create(self, request: dict[str, Any]) -> RunRecord:
        profile = str(request.get("profile", "before"))
        if profile not in {"before", "after"}:
            raise ValueError("profile은 before 또는 after여야 합니다")
        document = validate_document(request.get("document"))
        timeout = _finite_number(request.get("timeout", 60), "실행 제한", 1, 3600)
        penalty_mode = str(request.get("lane_penalty_mode", "shared_corridor"))
        if penalty_mode not in {"off", "shared_corridor", "shortest_path", "manual"}:
            raise ValueError("지원하지 않는 AFTER penalty 모드입니다")
        penalty_value = _finite_number(request.get("lane_penalty_value", 60), "Penalty 강도", 0.1, 100000)
        options = {
            "timeout": timeout,
            "rebuild_lab": bool(request.get("rebuild_lab", True)),
            "rebuild_after": bool(request.get("rebuild_after", True)),
            "setup": _safe_path(
                request.get("setup"), "setup.bash",
                SERVER_CONFIG["before_setup"] if profile == "before" else SERVER_CONFIG["after_setup"]),
            "before_source": _safe_path(request.get("before_source"), "Before source", SERVER_CONFIG["before_source"]),
            "after_workspace": _safe_path(request.get("after_workspace"), "After workspace", SERVER_CONFIG["after_workspace"]),
            "after_source": _safe_path(request.get("after_source"), "After source", SERVER_CONFIG["after_source"]),
            "base_ros_setup": _safe_path(request.get("base_ros_setup"), "Base ROS setup", SERVER_CONFIG["base_ros_setup"]),
            "after_label": _safe_text(request.get("after_label", SERVER_CONFIG["after_label"]), "After label", allow_empty=False),
            "lane_penalty_mode": penalty_mode,
            "lane_penalty_value": penalty_value,
        }
        if profile == "after" and penalty_mode != "off":
            patched, planner = after_core_patch_status(Path(options["after_source"]))
            if not patched:
                raise ValueError(
                    "AFTER rmf_traffic에 RMF_TRAFFIC_LAB_OCCUPANCY_PENALTY_V2 패치가 없습니다. "
                    f"AFTER 코어 준비를 먼저 실행하세요. 확인 Planner={planner or '찾지 못함'}")
        run_id = uuid.uuid4().hex[:12]
        run_dir = RUN_ROOT / run_id
        run_dir.mkdir(parents=True, exist_ok=False)
        label = "before" if profile == "before" else options["after_label"]
        record = RunRecord(
            run_id=run_id,
            profile=profile,
            label=label,
            document=document,
            options=options,
            run_dir=run_dir,
            result_name=f"web_{profile}_{run_id}",
        )
        (run_dir / "scenario.json").write_text(
            json.dumps(document, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        record.save_meta()
        with self.lock:
            self.records[run_id] = record
        self.queue.put(run_id)
        return record

    def get(self, run_id: str) -> RunRecord:
        with self.lock:
            record = self.records.get(run_id)
        if record is None:
            raise KeyError(run_id)
        return record

    def list_runs(self) -> list[dict[str, Any]]:
        with self.lock:
            records = sorted(self.records.values(), key=lambda record: record.created_at, reverse=True)
        return [record.public_state() for record in records[:100]]

    def stop(self, run_id: str) -> dict[str, Any]:
        record = self.get(run_id)
        with record.lock:
            record.stop_requested = True
            process = record.process
            if record.status == "queued":
                record.status = "cancelled"
                record.finished_at = time.time()
        if process is not None and process.poll() is None:
            self._terminate_process(process)
        record.save_meta()
        return record.public_state()

    @staticmethod
    def _terminate_process(process: subprocess.Popen[str]) -> None:
        try:
            if os.name == "posix":
                os.killpg(process.pid, signal.SIGTERM)
            else:
                process.terminate()
        except (OSError, ProcessLookupError):
            pass

    def _worker_loop(self) -> None:
        while True:
            run_id = self.queue.get()
            try:
                record = self.get(run_id)
                with record.lock:
                    if record.stop_requested or record.status == "cancelled":
                        continue
                self._execute(record)
            except Exception:
                try:
                    record = self.get(run_id)
                    with record.lock:
                        record.status = "failed"
                        record.error = traceback.format_exc()
                        record.finished_at = time.time()
                    record.append_log(record.error)
                    record.save_meta()
                except Exception:
                    pass
            finally:
                self.queue.task_done()

    def _reader(self, record: RunRecord, process: subprocess.Popen[str]) -> None:
        assert process.stdout is not None
        for line in process.stdout:
            record.append_log(line)

    def _execute(self, record: RunRecord) -> None:
        with record.lock:
            if record.stop_requested:
                record.status = "cancelled"
                record.finished_at = time.time()
                record.save_meta()
                return
            record.status = "running"
            record.started_at = time.time()
        record.save_meta()
        options = record.options
        build_dir = ROOT / "build" / f"web_{record.profile}"
        source = options["before_source"] if record.profile == "before" else options["after_source"]
        command = [
            sys.executable,
            str(ROOT / "run.py"),
            "--scenario-file", str(record.run_dir / "scenario.json"),
            "--timeout", str(options["timeout"]),
            "--no-html",
            "--build-dir", str(build_dir),
            "--result-name", record.result_name,
            "--core-label", record.label,
            "--setup", str(options["setup"]),
            "--rmf-source", str(source),
        ]
        if not options["rebuild_lab"]:
            command.append("--skip-build")
        if record.profile == "after":
            if options["lane_penalty_mode"] != "off":
                command.extend([
                    "--lane-penalty-mode", str(options["lane_penalty_mode"]),
                    "--lane-penalty-value", str(options["lane_penalty_value"]),
                ])
            if options["rebuild_after"]:
                command.extend([
                    "--rebuild-rmf-workspace", str(options["after_workspace"]),
                    "--base-ros-setup", str(options["base_ros_setup"]),
                ])
        record.append_log("+ " + " ".join(command) + "\n")
        process = subprocess.Popen(
            command,
            cwd=ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="utf-8",
            errors="replace",
            bufsize=1,
            start_new_session=True,
        )
        with record.lock:
            record.process = process
        reader = threading.Thread(target=self._reader, args=(record, process), daemon=True)
        reader.start()
        while process.poll() is None:
            record.refresh_events()
            with record.lock:
                stop_requested = record.stop_requested
            if stop_requested:
                self._terminate_process(process)
                break
            time.sleep(0.2)
        exit_code = process.wait()
        reader.join(timeout=2.0)
        record.refresh_events()
        if record.global_result_path.is_file():
            shutil.copy2(record.global_result_path, record.archived_result_path)
            record.refresh_events()
        with record.lock:
            record.process = None
            record.exit_code = exit_code
            record.finished_at = time.time()
            if record.stop_requested:
                record.status = "cancelled"
            elif exit_code == 0:
                record.status = "completed"
            elif exit_code == 124:
                record.status = "timeout"
            else:
                record.status = "failed"
        record.save_meta()
        with self.lock:
            self.latest_by_profile[record.profile] = record.run_id

    def compare(self, before_id: str | None = None, after_id: str | None = None) -> dict[str, Any]:
        with self.lock:
            before_id = before_id or self.latest_by_profile.get("before")
            after_id = after_id or self.latest_by_profile.get("after")
        before = summarize_result(self.get(before_id).events) if before_id else {}
        after = summarize_result(self.get(after_id).events) if after_id else {}
        payload = comparison_payload(before, after) if before and after else {
            "before": before,
            "after": after,
            "rows": [],
            "explanation": "Before와 After를 같은 시나리오로 각각 실행하면 비교 결과가 표시됩니다.",
        }
        payload["before_run_id"] = before_id
        payload["after_run_id"] = after_id
        return payload


MANAGER = RunManager()


def analysis_payload(record: RunRecord) -> dict[str, Any]:
    with record.lock:
        events = list(record.events)
        runtime_text = "".join(record.log_lines)
    diagnoses = [event for event in events if event.get("event") == "solution_diagnosis"]
    profile = _last(events, "runner_core_profile")
    return {
        "runtime_summary": explain_runtime_output(runtime_text),
        "jsonl_summary": summarize_jsonl(events),
        "diagnosis_summary": "\n\n".join(diagnosis_text(event) for event in diagnoses)
        or "아직 최종 진단이 없습니다.",
        "diagnosis_raw": diagnoses,
        "schedule_guide": schedule_guide_text(events),
        "astar_guide": astar_guide_text(),
        "decisions": decision_records(events),
        "profile": profile,
        "summary": summarize_result(events),
    }


class RMFWebHandler(BaseHTTPRequestHandler):
    server_version = "RMFTrafficLab/0.15"

    def log_message(self, format_string: str, *args: Any) -> None:
        sys.stderr.write("[%s] %s\n" % (self.log_date_time_string(), format_string % args))

    def _authorized(self, query: dict[str, list[str]] | None = None) -> bool:
        expected = os.environ.get("RMF_LAB_TOKEN", "")
        if not expected:
            return True
        authorization = self.headers.get("Authorization", "")
        supplied = authorization[7:] if authorization.startswith("Bearer ") else ""
        if query and not supplied:
            supplied = query.get("token", [""])[0]
        return supplied == expected

    def _send_json(self, value: Any, status: int = 200) -> None:
        payload = _json_dumps(value)
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def _send_error_json(self, status: int, message: str) -> None:
        self._send_json({"error": message}, status)

    def _read_json(self) -> dict[str, Any]:
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError as exc:
            raise ValueError("잘못된 Content-Length입니다") from exc
        if length <= 0 or length > MAX_REQUEST_BYTES:
            raise ValueError("요청 본문 크기가 올바르지 않습니다")
        try:
            value = json.loads(self.rfile.read(length).decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ValueError("올바른 UTF-8 JSON 요청이 아닙니다") from exc
        if not isinstance(value, dict):
            raise ValueError("JSON 객체가 필요합니다")
        return value

    def _serve_file(self, path: Path, content_type: str | None = None) -> None:
        if not path.is_file():
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        payload = path.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type or mimetypes.guess_type(path.name)[0] or "application/octet-stream")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)

    def do_GET(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        query = parse_qs(parsed.query)
        if parsed.path.startswith("/api/") and not self._authorized(query):
            self._send_error_json(HTTPStatus.UNAUTHORIZED, "접근 토큰이 올바르지 않습니다")
            return
        try:
            if parsed.path == "/api/health":
                self._send_json({"status": "ok", "version": "0.16.0"})
            elif parsed.path == "/api/config":
                self._send_json(SERVER_CONFIG)
            elif parsed.path == "/api/scenarios":
                catalog = scenario_catalog()
                self._send_json([{
                    "key": key,
                    "name": value.get("name", key),
                    "description": value.get("description", ""),
                    "nodes": len(value.get("nodes", [])),
                    "lanes": len(value.get("lanes", [])),
                    "robots": len(value.get("robots", [])),
                } for key, value in catalog.items()])
            elif parsed.path.startswith("/api/scenarios/"):
                key = unquote(parsed.path.removeprefix("/api/scenarios/"))
                document = scenario_catalog().get(key)
                if document is None:
                    self._send_error_json(HTTPStatus.NOT_FOUND, "시나리오를 찾을 수 없습니다")
                else:
                    self._send_json(document)
            elif parsed.path == "/api/runs":
                self._send_json(MANAGER.list_runs())
            elif parsed.path == "/api/compare":
                self._send_json(MANAGER.compare(
                    query.get("before", [None])[0], query.get("after", [None])[0]))
            elif re.fullmatch(r"/api/runs/[A-Za-z0-9_-]+/stream", parsed.path):
                run_id = parsed.path.split("/")[3]
                self._serve_stream(MANAGER.get(run_id))
            elif re.fullmatch(r"/api/runs/[A-Za-z0-9_-]+/analysis", parsed.path):
                run_id = parsed.path.split("/")[3]
                self._send_json(analysis_payload(MANAGER.get(run_id)))
            elif re.fullmatch(r"/api/runs/[A-Za-z0-9_-]+/jsonl", parsed.path):
                record = MANAGER.get(parsed.path.split("/")[3])
                path = record.archived_result_path if record.archived_result_path.is_file() else record.global_result_path
                self._serve_file(path, "application/x-ndjson; charset=utf-8")
            elif re.fullmatch(r"/api/runs/[A-Za-z0-9_-]+/runtime-log", parsed.path):
                record = MANAGER.get(parsed.path.split("/")[3])
                self._serve_file(record.run_dir / "runtime.log", "text/plain; charset=utf-8")
            elif re.fullmatch(r"/api/runs/[A-Za-z0-9_-]+", parsed.path):
                record = MANAGER.get(parsed.path.split("/")[3])
                self._send_json(record.public_state())
            elif parsed.path == "/api/after-core/status":
                source = Path(query.get("source", [SERVER_CONFIG["after_source"]])[0]).expanduser()
                patched, planner = after_core_patch_status(source)
                self._send_json({"patched": patched, "planner": str(planner) if planner else "", "source": str(source)})
            elif parsed.path.startswith("/web/"):
                relative = Path(unquote(parsed.path.removeprefix("/web/")))
                candidate = (WEB_ROOT / relative).resolve()
                if WEB_ROOT.resolve() not in candidate.parents:
                    self.send_error(HTTPStatus.FORBIDDEN)
                else:
                    self._serve_file(candidate)
            elif parsed.path.startswith("/assets/"):
                relative = Path(unquote(parsed.path.removeprefix("/assets/")))
                assets = (ROOT / "assets").resolve()
                candidate = (assets / relative).resolve()
                if assets not in candidate.parents:
                    self.send_error(HTTPStatus.FORBIDDEN)
                else:
                    self._serve_file(candidate)
            elif parsed.path in {"/", "/index.html"}:
                self._serve_file(WEB_ROOT / "index.html", "text/html; charset=utf-8")
            else:
                self.send_error(HTTPStatus.NOT_FOUND)
        except KeyError:
            self._send_error_json(HTTPStatus.NOT_FOUND, "실행 ID를 찾을 수 없습니다")
        except (BrokenPipeError, ConnectionResetError):
            return
        except Exception as exc:
            self._send_error_json(HTTPStatus.INTERNAL_SERVER_ERROR, str(exc))

    def _serve_stream(self, record: RunRecord) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()
        log_index = 0
        event_index = 0
        final_sent = False
        while True:
            record.refresh_events()
            with record.lock:
                state = record.public_state()
                logs = record.log_lines[log_index:]
                events = record.events[event_index:]
                log_index = len(record.log_lines)
                event_index = len(record.events)
            payload = {"state": state, "logs": logs, "events": events}
            if state["status"] in FINAL_STATES:
                payload["analysis"] = analysis_payload(record)
                payload["comparison"] = MANAGER.compare()
                final_sent = True
            message = b"data: " + _json_dumps(payload) + b"\n\n"
            try:
                self.wfile.write(message)
                self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                return
            if final_sent:
                return
            time.sleep(0.25)

    def do_POST(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        query = parse_qs(parsed.query)
        if not self._authorized(query):
            self._send_error_json(HTTPStatus.UNAUTHORIZED, "접근 토큰이 올바르지 않습니다")
            return
        try:
            body = self._read_json()
            if parsed.path == "/api/runs":
                record = MANAGER.create(body)
                self._send_json(record.public_state(), HTTPStatus.ACCEPTED)
            elif parsed.path == "/api/scenarios/import-yaml":
                yaml_text = body.get("yaml_text")
                if not isinstance(yaml_text, str):
                    raise ValueError("yaml_text 문자열이 필요합니다")
                level = body.get("level")
                if level is not None:
                    level = _safe_text(level, "level", allow_empty=False)
                self._send_json(convert_building_map_yaml(yaml_text, level))
            elif re.fullmatch(r"/api/runs/[A-Za-z0-9_-]+/stop", parsed.path):
                self._send_json(MANAGER.stop(parsed.path.split("/")[3]))
            elif parsed.path == "/api/scenarios/save":
                document = validate_document(body.get("document"))
                raw_name = _safe_text(body.get("filename", document.get("name", "scenario")), "filename", allow_empty=False)
                filename = re.sub(r"[^A-Za-z0-9가-힣_-]+", "_", raw_name).strip("_") or "scenario"
                destination = SAVED_SCENARIO_ROOT / f"{filename}.json"
                destination.write_text(json.dumps(document, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
                self._send_json({"saved": True, "key": f"saved__{filename}", "path": str(destination)})
            elif parsed.path == "/api/after-core/prepare":
                if not SERVER_CONFIG["allow_core_patch"]:
                    self._send_error_json(HTTPStatus.FORBIDDEN, "서버에서 AFTER 코어 패치 기능이 비활성화되어 있습니다")
                    return
                before_source = Path(_safe_path(body.get("before_source"), "Before source", SERVER_CONFIG["before_source"]))
                after_workspace = Path(_safe_path(body.get("after_workspace"), "After workspace", SERVER_CONFIG["after_workspace"]))
                result = prepare_after_core(before_source, after_workspace)
                self._send_json(result)
            else:
                self.send_error(HTTPStatus.NOT_FOUND)
        except KeyError:
            self._send_error_json(HTTPStatus.NOT_FOUND, "실행 ID를 찾을 수 없습니다")
        except (ValueError, OSError, RuntimeError) as exc:
            self._send_error_json(HTTPStatus.BAD_REQUEST, str(exc))
        except Exception as exc:
            self._send_error_json(HTTPStatus.INTERNAL_SERVER_ERROR, str(exc))


def main() -> int:
    parser = argparse.ArgumentParser(description="Serve RMF Traffic Lab on the local network")
    parser.add_argument("--host", default=os.environ.get("RMF_LAB_HOST", "0.0.0.0"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("RMF_LAB_PORT", "8080")))
    args = parser.parse_args()
    if not WEB_ROOT.is_dir():
        print(f"Web UI directory not found: {WEB_ROOT}", file=sys.stderr)
        return 2
    server = ThreadingHTTPServer((args.host, args.port), RMFWebHandler)
    print(f"RMF Traffic Lab Web: http://{args.host}:{args.port}")
    print("Stop with Ctrl+C")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping RMF Traffic Lab Web")
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
