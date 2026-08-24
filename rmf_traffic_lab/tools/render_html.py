from __future__ import annotations

import html
import json
from pathlib import Path
from typing import Any


SERIES = [
    "var(--series-1)",
    "var(--series-2)",
    "var(--series-3)",
    "var(--series-4)",
    "var(--series-5)",
]

SCENARIOS = [
    {
        "name": "single_lane_bidirectional",
        "robots": 2,
        "core": "Planner + Negotiation + Schedule DB + DetectConflict",
        "purpose": "1차선 양방향 순차 통과",
        "expected": "한 로봇이 staging에서 기다리고 복도를 차례로 사용",
    },
    {
        "name": "single_path",
        "robots": 1,
        "core": "Planner + Planner::Debug",
        "purpose": "짧은 중앙길과 긴 우회로 비용 비교",
        "expected": "실제 RMF cost가 낮은 경로 선택",
    },
    {
        "name": "single_path_closed",
        "robots": 1,
        "core": "Planner + lane closure",
        "purpose": "중앙 lane 폐쇄",
        "expected": "폐쇄 lane을 제외하고 우회",
    },
    {
        "name": "speed_limit_choice",
        "robots": 1,
        "core": "Planner + speed limit",
        "purpose": "짧고 느린 길과 길고 빠른 길 비교",
        "expected": "거리만이 아니라 속도·가속 비용 반영",
    },
    {
        "name": "single_path_multi",
        "robots": 2,
        "core": "Planner + Negotiation + Schedule DB",
        "purpose": "같은 그래프 양끝 교환",
        "expected": "중앙·우회 경로와 대기시간 협상",
    },
    {
        "name": "head_on",
        "robots": 2,
        "core": "Negotiation failure baseline",
        "purpose": "대피 공간 없는 정면 교환",
        "expected": "물리적 해가 없으면 no proposal; 실행 금지",
    },
    {
        "name": "passing_bay",
        "robots": 2,
        "core": "Negotiation + holding points",
        "purpose": "대피 bay가 있는 정면 교환",
        "expected": "bay 또는 대기로 충돌 없는 proposal 탐색",
    },
    {
        "name": "t_junction",
        "robots": 3,
        "core": "Multi-agent Negotiation + Schedule DB",
        "purpose": "T자 교차로 경쟁",
        "expected": "세 itinerary의 time-space 순서 조정",
    },
    {
        "name": "cross_intersection",
        "robots": 4,
        "core": "Multi-agent Negotiation + Schedule DB",
        "purpose": "4방향 공용 교차로",
        "expected": "복잡도·실패·포화 관찰용",
    },
    {
        "name": "disconnected",
        "robots": 1,
        "core": "Planner failure classification",
        "purpose": "분리된 graph island",
        "expected": "연결 경로 없음으로 planning failure",
    },
    {
        "name": "staggered_departures",
        "robots": 3,
        "core": "Planner + Negotiation + Schedule DB",
        "purpose": "로봇별 요청 출발 시각 비교",
        "expected": "0/0/8초 Start를 시간축 경로에 반영",
    },
    {
        "name": "grid_3x3_multi",
        "robots": 4,
        "core": "Multi-agent Negotiation + Schedule DB",
        "purpose": "3×3 교차 경로",
        "expected": "작은 격자의 대기·우회·충돌 결과 비교",
    },
    {
        "name": "grid_5x5_multi",
        "robots": 6,
        "core": "Multi-agent Negotiation + Schedule DB",
        "purpose": "5×5 다중 로봇 스트레스",
        "expected": "우회 선택과 협상 조합 증가 관찰",
    },
    {
        "name": "grid_10x10_multi",
        "robots": 8,
        "core": "Multi-agent Negotiation + Schedule DB",
        "purpose": "10×10 대규모 스트레스",
        "expected": "탐색·협상 시간 및 포화 한계 관찰",
    },
]


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    events: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as stream:
        for line_number, raw_line in enumerate(stream, start=1):
            line = raw_line.strip()
            if not line:
                continue
            try:
                events.append(json.loads(line))
            except json.JSONDecodeError as exc:
                raise ValueError(f"Invalid JSONL at line {line_number}: {exc}") from exc
    return events


def _event(events: list[dict[str, Any]], name: str) -> dict[str, Any]:
    return next((event for event in events if event.get("event") == name), {})


def _fmt(value: Any, digits: int = 3) -> str:
    if value is None:
        return "—"
    if isinstance(value, bool):
        return "예" if value else "아니오"
    if isinstance(value, float):
        return f"{value:.{digits}f}"
    return str(value)


def render(jsonl_path: Path, html_path: Path) -> None:
    events = read_jsonl(jsonl_path)
    nodes = {
        int(event["id"]): event
        for event in events
        if event.get("event") == "graph_node"
    }
    lanes = {
        int(event["id"]): event
        for event in events
        if event.get("event") == "graph_lane"
    }
    if not nodes:
        raise ValueError("The JSONL does not contain graph_node events")

    run = _event(events, "run_started")
    graph_summary = _event(events, "graph_summary")
    traits = _event(events, "vehicle_traits")
    planner_config = _event(events, "planner_configuration")
    data_model = _event(events, "data_model")
    scenario = str(run.get("scenario", jsonl_path.stem))
    description = str(run.get("description", ""))

    width, height, margin = 980.0, 520.0, 78.0
    xs = [float(node["x"]) for node in nodes.values()]
    ys = [float(node["y"]) for node in nodes.values()]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    if max_x - min_x < 1e-9:
        min_x, max_x = min_x - 1.0, max_x + 1.0
    if max_y - min_y < 1e-9:
        min_y, max_y = min_y - 1.0, max_y + 1.0

    def project(x: float, y: float) -> tuple[float, float]:
        px = margin + (x - min_x) / (max_x - min_x) * (width - 2 * margin)
        py = height - margin - (y - min_y) / (max_y - min_y) * (height - 2 * margin)
        return px, py

    screen_velocity_scale_x = (width - 2 * margin) / (max_x - min_x)
    screen_velocity_scale_y = -(height - 2 * margin) / (max_y - min_y)

    robot_starts: dict[str, dict[str, Any]] = {}
    for event in events:
        if event.get("event") == "planning_request":
            robot_starts[str(event.get("robot", "R0"))] = {
                "start": int(event["start"]),
                "yaw": float(event.get("start_yaw_rad", 0.0)),
            }
        elif event.get("event") == "negotiation_request":
            for robot in event.get("robots", []):
                robot_starts[str(robot["name"])] = {
                    "start": int(robot["start"]),
                    "yaw": float(robot.get("start_yaw_rad", 0.0)),
                }

    robot_names = sorted(
        {
            str(event.get("robot"))
            for event in events
            if event.get("robot") is not None
            and event.get("event")
            in {"trajectory_point", "plan_waypoint", "astar_expand"}
        }
        | set(robot_starts)
    )

    safety_event = _event(events, "safety_verification")
    safety_passed = bool(
        safety_event.get("passed") and safety_event.get("executable_plan", True)
    )
    is_multi_robot = int(run.get("robot_count", len(robot_names))) > 1

    def trajectory_payload(
        robot_name: str, phase: str
    ) -> dict[str, Any] | None:
        trajectory = [
            event
            for event in events
            if event.get("event") == "trajectory_point"
            and event.get("robot") == robot_name
            and str(event.get("phase", "free_flow")) == phase
        ]
        trajectory.sort(
            key=lambda event: (
                float(event.get("time_s", 0.0)),
                int(event.get("route_index", 0)),
                int(event.get("sequence", 0)),
            )
        )
        if not trajectory:
            return None

        points = []
        for point in trajectory:
            x, y = float(point["x"]), float(point["y"])
            sx, sy = project(x, y)
            points.append(
                {
                    "t": float(point.get("time_s", 0.0)),
                    "x": x,
                    "y": y,
                    "sx": round(sx, 3),
                    "sy": round(sy, 3),
                    "yaw": float(point.get("yaw_rad", 0.0)),
                    "vx": float(point.get("vx", 0.0)),
                    "vy": float(point.get("vy", 0.0)),
                    "vyaw": float(point.get("vyaw", 0.0)),
                    "svx": float(point.get("vx", 0.0)) * screen_velocity_scale_x,
                    "svy": float(point.get("vy", 0.0)) * screen_velocity_scale_y,
                }
            )
        used_lanes = sorted(
            {
                int(lane_id)
                for event in events
                if event.get("event") == "plan_waypoint"
                and event.get("robot") == robot_name
                and str(event.get("phase", "free_flow")) == phase
                for lane_id in event.get("approach_lanes", [])
            }
        )
        waypoints = [
            event
            for event in events
            if event.get("event") == "plan_waypoint"
            and event.get("robot") == robot_name
            and str(event.get("phase", "free_flow")) == phase
        ]
        waypoints.sort(key=lambda event: float(event.get("time_s", 0.0)))
        return {
            "phase": phase,
            "points": points,
            "usedLanes": used_lanes,
            "waypoints": waypoints,
        }

    simulation_robots: list[dict[str, Any]] = []
    for robot_index, robot_name in enumerate(robot_names):
        plans = {}
        for phase in (
            "free_flow",
            "free_flow_baseline",
            "negotiated",
            "rejected_negotiated",
        ):
            payload = trajectory_payload(robot_name, phase)
            if payload:
                plans[phase] = payload

        start = robot_starts.get(robot_name, {"start": 0, "yaw": 0.0})
        start_node = nodes[int(start["start"])]
        sx, sy = project(float(start_node["x"]), float(start_node["y"]))
        static_plan = {
            "phase": "static_no_executable_plan",
            "points": [
                {
                    "t": 0.0,
                    "x": float(start_node["x"]),
                    "y": float(start_node["y"]),
                    "sx": round(sx, 3),
                    "sy": round(sy, 3),
                    "yaw": float(start.get("yaw", 0.0)),
                    "vx": 0.0,
                    "vy": 0.0,
                    "vyaw": 0.0,
                    "svx": 0.0,
                    "svy": 0.0,
                }
            ],
            "usedLanes": [],
            "waypoints": [],
        }
        plans["static"] = static_plan

        if "free_flow" in plans:
            safe_phase = "free_flow"
        elif safety_passed and "negotiated" in plans:
            safe_phase = "negotiated"
        else:
            safe_phase = "static"
        active_plan = plans[safe_phase]
        simulation_robots.append(
            {
                "name": robot_name,
                "color": robot_index % len(SERIES),
                "phase": safe_phase,
                "safePhase": safe_phase,
                "usedLanes": active_plan["usedLanes"],
                "points": active_plan["points"],
                "plans": plans,
            }
        )

    all_times = [point["t"] for robot in simulation_robots for point in robot["points"]]
    start_time = min(all_times) if all_times else 0.0
    end_time = max(all_times) if all_times else start_time

    svg: list[str] = []
    marker = (
        '<defs><marker id="arrow" viewBox="0 0 10 10" refX="8" refY="5" '
        'markerWidth="5" markerHeight="5" orient="auto-start-reverse">'
        '<path d="M 0 0 L 10 5 L 0 10 z" class="arrow-head" /></marker></defs>'
    )
    svg.append(marker)
    for lane_id, lane in sorted(lanes.items()):
        entry, exit = int(lane["entry"]), int(lane["exit"])
        x1, y1 = project(float(nodes[entry]["x"]), float(nodes[entry]["y"]))
        x2, y2 = project(float(nodes[exit]["x"]), float(nodes[exit]["y"]))
        dx, dy = x2 - x1, y2 - y1
        length = max(1.0, (dx * dx + dy * dy) ** 0.5)
        offset = 4.0 if lane_id % 2 == 0 else -4.0
        ox, oy = -dy / length * offset, dx / length * offset
        lane_class = "graph-lane closed-lane" if lane.get("closed") else "graph-lane"
        svg.append(
            f'<line x1="{x1 + ox:.1f}" y1="{y1 + oy:.1f}" '
            f'x2="{x2 + ox:.1f}" y2="{y2 + oy:.1f}" '
            f'class="{lane_class}" data-lane-id="{lane_id}" marker-end="url(#arrow)" />'
        )

    for robot in simulation_robots:
        color = SERIES[int(robot["color"])]
        for lane_id in robot["usedLanes"]:
            lane = lanes.get(int(lane_id))
            if lane is None:
                continue
            entry, exit = int(lane["entry"]), int(lane["exit"])
            x1, y1 = project(float(nodes[entry]["x"]), float(nodes[entry]["y"]))
            x2, y2 = project(float(nodes[exit]["x"]), float(nodes[exit]["y"]))
            svg.append(
                f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" '
                f'stroke="{color}" class="selected-lane" />'
            )
        path_points = " ".join(
            f'{point["sx"]:.1f},{point["sy"]:.1f}' for point in robot["points"]
        )
        if len(robot["points"]) >= 2:
            svg.append(
                f'<polyline points="{path_points}" stroke="{color}" '
                'class="trajectory-path" />'
            )

    for node_id, node in sorted(nodes.items()):
        x, y = project(float(node["x"]), float(node["y"]))
        classes = ["graph-node"]
        if node.get("holding"):
            classes.append("holding-node")
        if node.get("parking"):
            classes.append("parking-node")
        svg.append(
            f'<circle cx="{x:.1f}" cy="{y:.1f}" r="10" '
            f'class="{" ".join(classes)}" data-node-id="{node_id}" />'
        )
        label = html.escape(f'{node_id}: {node.get("name", node_id)}')
        svg.append(
            f'<text x="{x:.1f}" y="{y - 18:.1f}" text-anchor="middle" '
            f'class="node-label">{label}</text>'
        )

    for robot_index, robot in enumerate(simulation_robots):
        color = SERIES[int(robot["color"])]
        name = html.escape(str(robot["name"]))
        svg.append(
            f'<g id="robot-{robot_index}" class="robot" aria-label="{name}">'
            f'<circle r="14" fill="{color}" class="robot-body" />'
            f'<path d="M 0 0 L 23 0 L 17 -5 M 23 0 L 17 5" stroke="{color}" '
            'class="robot-heading" />'
            f'<text x="0" y="-22" text-anchor="middle" class="robot-label">{name}</text>'
            '</g>'
        )

    node_rows = "".join(
        "<tr>"
        f'<td class="mono">{node_id}</td>'
        f'<td>{html.escape(str(node.get("name", "")))}</td>'
        f'<td class="mono">({_fmt(node.get("x"))}, {_fmt(node.get("y"))})</td>'
        f'<td>{"holding " if node.get("holding") else ""}'
        f'{"parking " if node.get("parking") else ""}'
        f'{"passthrough" if node.get("passthrough") else ""}</td>'
        f'<td>{html.escape(str(node.get("mutex_group") or "—"))}</td>'
        f'<td class="mono">{html.escape(str(node.get("outgoing_lanes", [])))}</td>'
        "</tr>"
        for node_id, node in sorted(nodes.items())
    )
    lane_rows = "".join(
        "<tr>"
        f'<td class="mono">{lane_id}</td>'
        f'<td class="mono">{lane.get("entry")} → {lane.get("exit")}</td>'
        f'<td class="mono">{_fmt(lane.get("length_m"))}</td>'
        f'<td class="mono">{_fmt(lane.get("speed_limit_mps"))}</td>'
        f'<td>{html.escape(str(lane.get("mutex_group") or "—"))}</td>'
        f'<td>{"폐쇄" if lane.get("closed") else "열림"}</td>'
        "</tr>"
        for lane_id, lane in sorted(lanes.items())
    )

    plan_summaries = [
        event for event in events if event.get("event") == "plan_summary"
    ]
    successful_plans = [event for event in plan_summaries if event.get("success")]
    astar_expands = [event for event in events if event.get("event") == "astar_expand"]
    candidates = [event for event in events if event.get("event") == "route_candidate"]
    schedule_states = [
        event for event in events if event.get("event") == "schedule_database_state"
    ]
    schedule_operations = [
        event for event in events if event.get("event") == "schedule_database_operation"
    ]
    schedule_routes = [
        event for event in events if event.get("event") in
        {"schedule_itinerary_route", "schedule_database_route"}
    ]
    schedule_points = [
        event for event in events if event.get("event") in
        {"schedule_trajectory_point", "schedule_database_trajectory_point"}
    ]
    negotiation = _event(events, "negotiation_summary")
    rmf_proof = _event(events, "rmf_runtime_proof")
    diagnosis = _event(events, "solution_diagnosis")
    custom_scenario = _event(events, "custom_scenario_loaded")

    simulation_data = {
        "scenario": scenario,
        "startTime": start_time,
        "endTime": end_time,
        "robots": simulation_robots,
        "events": events,
        "nodes": list(nodes.values()),
        "lanes": list(lanes.values()),
        "safety": safety_event,
        "safetyPassed": safety_passed,
        "isMultiRobot": is_multi_robot,
        "requiredCenterDistance": 2.0 * float(traits.get("profile_radius_m", 0.3)),
    }
    data_json = json.dumps(
        simulation_data, ensure_ascii=False, separators=(",", ":")
    ).replace("</", "<\\/")

    legend = "".join(
        f'<span class="legend-item"><i style="background:{SERIES[int(robot["color"])]}"></i>'
        f'{html.escape(str(robot["name"]))} ({html.escape(str(robot["phase"]))})</span>'
        for robot in simulation_robots
    )
    legend += '<span class="legend-item"><i class="holding-swatch"></i>holding</span>'
    legend += '<span class="legend-item"><i class="parking-swatch"></i>parking</span>'

    why_text = "경로 후보가 없습니다. 연결성 또는 협상 결과를 확인하세요."
    selected_candidates = [candidate for candidate in candidates if candidate.get("selected_by_plan")]
    if selected_candidates:
        selected = selected_candidates[0]
        why_text = (
            f'{selected.get("robot")}: 후보 {selected.get("rank")}위, '
            f'RMF cost {_fmt(selected.get("rmf_cost"))}, '
            f'distance {_fmt(selected.get("distance_m"))} m 경로를 선택했습니다. '
            "A*는 실제 f=g+h가 가장 낮은 frontier부터 확장하고 첫 최적해에서 종료합니다."
        )

    if any(robot["phase"] == "negotiated" for robot in simulation_robots):
        phase_label = "DetectConflict 검증을 통과한 negotiated trajectory"
    elif any(robot["phase"] == "free_flow" for robot in simulation_robots):
        phase_label = "단일 로봇 free-flow trajectory"
    else:
        phase_label = "실행 가능한 협상 계획 없음 — 시작 위치에서 정지"
    has_executable_plan = any(
        robot["safePhase"] != "static" for robot in simulation_robots
    )
    safety_class = "safe" if has_executable_plan else "unsafe"
    safety_title = (
        "실행 허용: RMF 충돌 검증 통과"
        if has_executable_plan
        else "실행 금지: 검증된 다중 로봇 계획 없음"
    )
    negotiation_label = (
        "성공" if negotiation.get("success") else "해 없음"
    ) if negotiation else "미사용"

    scenario_rows = "".join(
        ("<tr class=\"current-scenario\">" if item["name"] == scenario else "<tr>")
        + f'<td class="mono">{html.escape(str(item["name"]))}</td>'
        + f'<td class="mono">{item["robots"]}</td>'
        + f'<td>{html.escape(str(item["core"]))}</td>'
        + f'<td>{html.escape(str(item["purpose"]))}</td>'
        + f'<td>{html.escape(str(item["expected"]))}</td>'
        + "</tr>"
        for item in SCENARIOS
    )
    schedule_used = bool(schedule_states or schedule_operations)
    schedule_status = (
        f"실제 DB 사용 · snapshot {len(schedule_states)}개 · operation {len(schedule_operations)}개"
        if schedule_used
        else "이 단일 로봇 시나리오는 Schedule DB 미사용"
    )
    diagnosis_status = str(diagnosis.get("status", "unknown"))
    diagnosis_class = "safe" if diagnosis_status == "solved" else "unsafe"
    diagnosis_title = {
        "solved": "실행 가능한 해가 확인됨",
        "no_solution": "실행 가능한 해가 없음",
    }.get(diagnosis_status, "진단 이벤트 없음")
    evidence_rows = "".join(
        f"<li><code>{html.escape(str(item))}</code></li>"
        for item in diagnosis.get("evidence", [])
    ) or "<li>기록된 진단 근거 없음</li>"
    action_rows = "".join(
        f"<li>{html.escape(str(item))}</li>"
        for item in diagnosis.get("recommended_actions", [])
    ) or "<li>원본 로그와 마지막 정상 이벤트를 확인하세요.</li>"
    custom_warnings = "".join(
        f"<li>{html.escape(str(item))}</li>"
        for item in custom_scenario.get("validation_warnings", [])
    ) or "<li>정적 입력검사 경고 없음</li>"

    html_text = f"""<!doctype html>
<html lang="ko">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>RMF Traffic Analyzer - {html.escape(scenario)}</title>
  <style>
    :root {{
      color-scheme: light;
      --bg: #f4f6fa; --fg: #182230; --panel: #fff; --muted: #667085;
      --border: #d7dee9; --lane: #c7d0dc; --accent: #f59e0b; --danger: #dc2626;
      --code: #eef2f7; --good: #087f5b; --focus: #2563eb;
      --series-1: #2563eb; --series-2: #dc2626; --series-3: #059669;
      --series-4: #9333ea; --series-5: #ea580c;
    }}
    * {{ box-sizing: border-box; }}
    body {{ margin: 0; background: var(--bg); color: var(--fg); font: 14px/1.5 system-ui, sans-serif; }}
    main {{ max-width: 1320px; margin: 24px auto 60px; padding: 0 18px; }}
    h1 {{ margin: 0; font-size: 27px; font-weight: 620; }}
    h2 {{ margin: 0 0 12px; font-size: 19px; }}
    h3 {{ margin: 0 0 8px; font-size: 15px; }}
    p {{ margin: 6px 0; }}
    .muted {{ color: var(--muted); }}
    .mono {{ font-family: ui-monospace, SFMono-Regular, Consolas, monospace; font-variant-numeric: tabular-nums; }}
    .topline {{ display:flex; justify-content:space-between; gap:16px; align-items:flex-start; margin-bottom:16px; }}
    .badge {{ border:1px solid var(--border); border-radius:999px; padding:5px 10px; color:var(--muted); white-space:nowrap; }}
    .metrics {{ display:grid; grid-template-columns:repeat(6,minmax(120px,1fr)); gap:10px; margin:14px 0; }}
    .metric, .panel {{ background:var(--panel); border:1px solid var(--border); border-radius:13px; }}
    .metric {{ padding:12px; }}
    .metric b {{ display:block; font-size:19px; font-weight:650; }}
    .metric span {{ color:var(--muted); font-size:12px; }}
    .tabs {{ display:flex; gap:5px; overflow:auto; border-bottom:1px solid var(--border); margin:18px 0 14px; }}
    .tab {{ border:0; background:transparent; color:var(--muted); padding:10px 13px; font:inherit; cursor:pointer; white-space:nowrap; border-bottom:2px solid transparent; }}
    .tab.active {{ color:var(--fg); border-color:var(--focus); }}
    .tab-panel {{ display:none; }}
    .tab-panel.active {{ display:block; }}
    .panel {{ padding:16px; margin-bottom:12px; }}
    .grid-2 {{ display:grid; grid-template-columns:minmax(0,1.45fr) minmax(280px,.55fr); gap:12px; }}
    .controls {{ display:flex; gap:10px; align-items:center; flex-wrap:wrap; margin-bottom:8px; }}
    button, select, input {{ font:inherit; color:var(--fg); background:var(--panel); border:1px solid var(--border); border-radius:8px; padding:7px 10px; }}
    button.primary {{ color:white; background:#2563eb; border-color:#2563eb; }}
    input[type="range"] {{ flex:1 1 260px; min-width:170px; padding:0; accent-color:var(--focus); }}
    input[type="search"] {{ min-width:260px; }}
    output {{ min-width:120px; }}
    svg {{ width:100%; height:auto; display:block; }}
    .graph-lane {{ stroke:var(--lane); stroke-width:5; stroke-linecap:round; opacity:.86; }}
    .graph-lane.live-active {{ stroke:var(--focus); stroke-width:7; opacity:1; }}
    .closed-lane {{ stroke:var(--danger); stroke-dasharray:7 6; }}
    .arrow-head {{ fill:var(--lane); }}
    .selected-lane {{ stroke-width:3; opacity:.8; }}
    .trajectory-path {{ fill:none; stroke-width:2; stroke-dasharray:5 5; opacity:.7; }}
    .graph-node {{ fill:var(--panel); stroke:var(--fg); stroke-width:2; }}
    .holding-node {{ fill:var(--accent); }}
    .parking-node {{ stroke-width:4; }}
    .node-label,.robot-label {{ fill:var(--fg); font-size:12px; paint-order:stroke; stroke:var(--panel); stroke-width:4px; stroke-linejoin:round; }}
    .robot-body {{ stroke:var(--panel); stroke-width:3; }}
    .robot-heading {{ fill:none; stroke-width:3; stroke-linecap:round; }}
    .legend {{ display:flex; flex-wrap:wrap; gap:14px; color:var(--muted); margin-top:8px; }}
    .legend-item {{ display:inline-flex; align-items:center; gap:6px; }}
    .legend-item i {{ width:22px; height:4px; border-radius:2px; display:inline-block; }}
    .legend-item .holding-swatch {{ width:12px; height:12px; border-radius:50%; background:var(--accent); border:1px solid var(--fg); }}
    .legend-item .parking-swatch {{ width:12px; height:12px; border-radius:50%; background:var(--panel); border:3px solid var(--fg); }}
    .callout {{ border-left:4px solid var(--focus); background:var(--code); padding:11px 13px; border-radius:7px; }}
    .pipeline {{ display:grid; grid-template-columns:repeat(5,1fr); gap:8px; }}
    .pipe-step {{ padding:11px; border:1px solid var(--border); border-radius:9px; background:var(--panel); }}
    .pipe-step b {{ display:block; }}
    .pipe-step span {{ font-size:12px; color:var(--muted); }}
    .table-wrap {{ overflow:auto; max-height:560px; border:1px solid var(--border); border-radius:9px; }}
    table {{ width:100%; border-collapse:collapse; }}
    th,td {{ padding:8px 10px; border-bottom:1px solid var(--border); text-align:left; vertical-align:top; white-space:nowrap; }}
    th {{ position:sticky; top:0; background:var(--panel); color:var(--muted); font-size:12px; z-index:1; }}
    tr.selected {{ background:color-mix(in srgb,var(--focus) 12%,transparent); }}
    code {{ background:var(--code); border-radius:5px; padding:2px 5px; }}
    .kv {{ display:grid; grid-template-columns:minmax(140px,.45fr) 1fr; gap:7px 12px; }}
    .kv dt {{ color:var(--muted); }} .kv dd {{ margin:0; overflow-wrap:anywhere; }}
    .search-card {{ display:grid; grid-template-columns:repeat(4,1fr); gap:8px; margin:10px 0; }}
    .search-card div {{ padding:10px; background:var(--code); border-radius:8px; }}
    .search-card b {{ display:block; font-size:18px; }}
    .json {{ white-space:pre-wrap; max-width:760px; font-size:12px; color:var(--muted); }}
    .safety-banner {{ margin-bottom:10px; padding:10px 12px; border-radius:8px; border:1px solid var(--border); background:var(--code); }}
    .safety-banner.safe {{ border-left:5px solid var(--good); }}
    .safety-banner.unsafe {{ border-left:5px solid var(--danger); }}
    .live-decision {{ width:100%; }}
    .live-decision td {{ white-space:normal; padding:7px 5px; }}
    .live-decision td:first-child {{ font-weight:600; white-space:nowrap; }}
    .process-strip {{ display:grid; grid-template-columns:repeat(6,1fr); gap:5px; margin:10px 0; }}
    .process-item {{ padding:7px; border:1px solid var(--border); border-radius:7px; color:var(--muted); font-size:11px; }}
    .process-item.done {{ color:var(--fg); border-color:var(--good); }}
    .process-item.active {{ color:var(--fg); background:var(--code); border-color:var(--focus); }}
    .clearance-safe {{ color:var(--good); }}
    .clearance-danger {{ color:var(--danger); font-weight:700; }}
    .db-flow {{ display:grid; grid-template-columns:repeat(5,1fr); gap:8px; margin:10px 0 16px; }}
    .db-node {{ padding:10px; border:1px solid var(--border); border-radius:9px; color:var(--muted); background:var(--panel); }}
    .db-node b {{ display:block; color:var(--fg); }}
    .db-node.active {{ border-color:var(--good); background:var(--code); }}
    .db-snapshot {{ display:grid; grid-template-columns:repeat(4,1fr); gap:8px; margin:10px 0; }}
    .db-snapshot div {{ padding:10px; background:var(--code); border-radius:8px; }}
    .db-snapshot b {{ display:block; font-size:18px; }}
    .current-scenario {{ background:color-mix(in srgb,var(--focus) 14%,transparent); }}
    .scenario-table td {{ white-space:normal; }}
    .proof-grid {{ display:grid; grid-template-columns:repeat(2,minmax(0,1fr)); gap:8px 16px; }}
    .proof-grid div {{ border-bottom:1px solid var(--border); padding:7px 0; overflow-wrap:anywhere; }}
    .diagnosis-grid {{ display:grid; grid-template-columns:minmax(0,.7fr) minmax(0,1.3fr); gap:12px; }}
    .diagnosis-list {{ margin:8px 0 0; padding-left:22px; }}
    .diagnosis-list li {{ margin:7px 0; white-space:normal; }}
    .raw-log {{ margin:0; padding:14px; max-height:720px; overflow:auto; background:var(--code); border:1px solid var(--border); border-radius:9px; white-space:pre; font:12px/1.55 ui-monospace,SFMono-Regular,Consolas,monospace; }}
    .raw-log-status {{ min-width:180px; }}
    @media (max-width:900px) {{ .metrics {{ grid-template-columns:repeat(3,1fr); }} .grid-2,.diagnosis-grid {{ grid-template-columns:1fr; }} .pipeline {{ grid-template-columns:1fr 1fr; }} .process-strip {{ grid-template-columns:repeat(3,1fr); }} .db-flow {{ grid-template-columns:1fr 1fr; }} }}
    @media (max-width:560px) {{ main {{ padding:0 10px; }} .metrics {{ grid-template-columns:1fr 1fr; }} .pipeline {{ grid-template-columns:1fr; }} .search-card,.db-snapshot,.proof-grid {{ grid-template-columns:1fr 1fr; }} }}
  </style>
</head>
<body>
<main id="rmf-analyzer">
  <div class="topline">
    <div><h1>RMF Traffic Analyzer</h1><p class="muted">{html.escape(scenario)} · {html.escape(description)}</p></div>
    <span class="badge">schema {html.escape(str(run.get("schema", "unknown")))}</span>
  </div>
  <section class="metrics">
    <div class="metric"><b>{len(nodes)}</b><span>waypoints</span></div>
    <div class="metric"><b>{len(lanes)}</b><span>directed lanes</span></div>
    <div class="metric"><b>{len(robot_names)}</b><span>robots</span></div>
    <div class="metric"><b>{len(astar_expands)}</b><span>real A* expansions</span></div>
    <div class="metric"><b>{len(candidates)}</b><span>forced candidates</span></div>
    <div class="metric"><b>{html.escape(negotiation_label)}</b><span>negotiation</span></div>
  </section>

  <nav class="tabs" aria-label="분석 화면">
    <button class="tab active" data-tab="overview">시뮬레이션</button>
    <button class="tab" data-tab="diagnosis">진단 요약</button>
    <button class="tab" data-tab="search">A* 검색</button>
    <button class="tab" data-tab="candidates">경로 선택 이유</button>
    <button class="tab" data-tab="graph">Navigation Graph</button>
    <button class="tab" data-tab="schedule">Schedule Database</button>
    <button class="tab" data-tab="negotiation">협상·안전</button>
    <button class="tab" data-tab="scenarios">시나리오·RMF 확인</button>
    <button class="tab" data-tab="sequence">전체 시퀀스</button>
    <button class="tab" data-tab="raw">원본 로그</button>
  </nav>

  <section id="tab-overview" class="tab-panel active">
    <div class="safety-banner {safety_class}"><b>{html.escape(safety_title)}</b> · {html.escape(phase_label)}</div>
    <div class="grid-2">
      <div class="panel">
        <div class="controls">
          <button id="play-button" class="primary" type="button">▶ 재생</button>
          <button id="reset-button" type="button">처음</button>
          <label>표시 계획 <select id="plan-mode"><option value="validated">검증된 실행 계획</option><option value="baseline">⚠ 충돌 미검증 free-flow 비교</option></select></label>
          <select id="speed-select" aria-label="재생 속도"><option value="0.5">0.5×</option><option value="1" selected>1×</option><option value="2">2×</option><option value="4">4×</option></select>
          <output id="clock-output" class="mono">0.00 s</output>
          <input id="time-slider" type="range" aria-label="시뮬레이션 시간">
        </div>
        <svg viewBox="0 0 {width:.0f} {height:.0f}" role="img" aria-label="RMF navigation graph simulation">{''.join(svg)}</svg>
        <div class="legend">{legend}</div>
        <p id="live-status" class="muted" aria-live="polite"></p>
      </div>
      <aside>
        <div class="panel"><h3>실시간 판단·안전 상태</h3><p id="live-plan-source" class="muted"></p><p id="live-clearance" class="mono"></p><table class="live-decision"><tbody id="live-decision-body"></tbody></table></div>
        <div class="panel"><h3>내부 처리 프로세스</h3><div class="process-strip"><div class="process-item done">1 Graph</div><div class="process-item done">2 A*</div><div id="process-negotiation" class="process-item">3 Negotiation</div><div id="process-safety" class="process-item">4 DetectConflict</div><div id="process-commit" class="process-item">5 DB commit</div><div id="execution-process" class="process-item">6 Execute</div></div><p class="muted">A*·협상·검증은 이동 전에 끝납니다. 재생 중에는 승인된 Schedule itinerary를 실행하며 현재 lane·대기·다음 waypoint를 표시합니다.</p></div>
        <div class="panel"><h3>왜 이 경로인가?</h3><p>{html.escape(why_text)}</p><div id="plan-summary"></div></div>
      </aside>
    </div>
  </section>

  <section id="tab-diagnosis" class="tab-panel">
    <div class="safety-banner {diagnosis_class}"><b>{html.escape(diagnosis_title)}</b> · category <code>{html.escape(str(diagnosis.get("category", "unknown")))}</code></div>
    <div class="panel">
      <h2>근본 원인</h2>
      <p>{html.escape(str(diagnosis.get("root_cause", "이 결과에는 solution_diagnosis 이벤트가 없습니다. 새 버전으로 다시 실행하세요.")))}</p>
      <dl class="kv"><dt>판정 상태</dt><dd>{html.escape(diagnosis_status)}</dd><dt>확신도</dt><dd>{html.escape(str(diagnosis.get("confidence", "unknown")))}</dd><dt>판정 근거 종류</dt><dd><code>{html.escape(str(diagnosis.get("basis", "not_recorded")))}</code></dd></dl>
      <p class="muted">RMF가 직접 제공한 실패 flag·no proposal·DetectConflict 결과는 확정 근거입니다. topology 분석으로 추정한 교착 원인은 <code>basis</code>에 inference로 명시하며, 공개 API가 주지 않는 정확한 branch rejection reason을 꾸며내지 않습니다.</p>
    </div>
    <div class="diagnosis-grid">
      <div class="panel"><h2>실제 관찰 근거</h2><ul class="diagnosis-list">{evidence_rows}</ul></div>
      <div class="panel"><h2>해를 만들기 위한 변경안</h2><ol class="diagnosis-list">{action_rows}</ol></div>
    </div>
    <div class="panel"><h2>커스텀 실험 해석</h2><div class="table-wrap"><table><thead><tr><th>변경</th><th>JSON 수정</th><th>확인하려는 근본 원인</th></tr></thead><tbody><tr><td>로봇 한 대 제거</td><td><code>robots</code> 배열에서 객체 하나 삭제</td><td>동시 요청의 최소 충돌 집합인지 확인</td></tr><tr><td>대기 노드 추가</td><td><code>holding:true</code> 노드와 진입 lane 추가</td><td>공유 자원 진입 전 시간 분리가 가능한지 확인</td></tr><tr><td>회피공간 추가</td><td>side node를 복도의 서로 다른 두 노드에 연결</td><td>단순 막다른 노드가 아니라 실제 passing loop가 필요한지 확인</td></tr><tr><td>lane 방향·폐쇄 변경</td><td><code>bidirectional</code>, <code>closed</code>, <code>closed_lanes</code></td><td>개별 로봇의 directed reachability 문제인지 확인</td></tr></tbody></table></div></div>
  </section>

  <section id="tab-search" class="tab-panel">
    <div class="panel">
      <h2>실제 Planner::Debug A* 추적</h2>
      <p class="muted">RMF 내부 frontier에서 실제로 꺼낸 값입니다. <code>g</code>는 누적 실제 비용, <code>h</code>는 남은 비용 추정, <code>f=g+h</code>는 우선순위입니다.</p>
      <div class="controls"><button id="search-play" class="primary" type="button">▶ 검색 재생</button><select id="search-robot"></select><output id="search-step-label" class="mono"></output><input id="search-slider" type="range" min="0" step="1"></div>
      <div class="search-card"><div><span>node</span><b id="search-node" class="mono">—</b></div><div><span>g(n)</span><b id="search-g" class="mono">—</b></div><div><span>h(n)</span><b id="search-h" class="mono">—</b></div><div><span>f(n)</span><b id="search-f" class="mono">—</b></div></div>
      <dl class="kv"><dt>waypoint / parent</dt><dd id="search-parent" class="mono"></dd><dt>frontier 크기</dt><dd id="search-queue" class="mono"></dd><dt>생성 child / terminal</dt><dd id="search-children" class="mono"></dd><dt>선택 이유</dt><dd id="search-reason"></dd><dt>노드 생성 순서</dt><dd id="search-path" class="mono"></dd></dl>
    </div>
    <div class="panel"><h3>확장 순서</h3><div class="table-wrap"><table><thead><tr><th>step</th><th>node</th><th>parent</th><th>WP</th><th>g</th><th>h</th><th>f</th><th>queue</th></tr></thead><tbody id="search-table"></tbody></table></div></div>
    <div class="callout">정확한 child rejection reason 전체는 현재 공개 Debug API에 없습니다. <code>generated_children=0</code>, terminal node, closure, topology는 볼 수 있지만 “각 branch가 왜 버려졌는지”까지 보려면 RMF 소스 내부 validator instrumentation이 다음 단계입니다.</div>
  </section>

  <section id="tab-candidates" class="tab-panel">
    <div class="panel"><h2>경로 후보별 실제 RMF 비용</h2><p class="muted">단순 path 하나만 남기고 나머지 lane을 닫은 뒤 실제 Planner로 재계획한 진단값입니다. 내부 A*를 흉내 낸 값이 아닙니다.</p><div class="controls"><select id="candidate-robot"></select></div><div class="table-wrap"><table><thead><tr><th>순위</th><th>waypoints</th><th>lanes</th><th>거리 m</th><th>RMF cost</th><th>최저 대비</th><th>도착 s</th><th>선택</th></tr></thead><tbody id="candidate-table"></tbody></table></div></div>
  </section>

  <section id="tab-graph" class="tab-panel">
    <div class="grid-2">
      <div class="panel"><h2>Waypoint DB</h2><div class="table-wrap"><table><thead><tr><th>ID</th><th>이름</th><th>좌표</th><th>속성</th><th>mutex</th><th>out lanes</th></tr></thead><tbody>{node_rows}</tbody></table></div></div>
      <aside class="panel"><h2>Planner 입력값</h2><dl class="kv"><dt>map</dt><dd>{html.escape(str(graph_summary.get("map", "—")))}</dd><dt>profile radius</dt><dd class="mono">{_fmt(traits.get("profile_radius_m"))} m</dd><dt>linear v / a</dt><dd class="mono">{_fmt(traits.get("linear_velocity_mps"))} / {_fmt(traits.get("linear_acceleration_mps2"))}</dd><dt>angular v / a</dt><dd class="mono">{_fmt(traits.get("angular_velocity_radps"))} / {_fmt(traits.get("angular_acceleration_radps2"))}</dd><dt>steering</dt><dd>{html.escape(str(traits.get("steering", "—")))}</dd><dt>traversal cost/m</dt><dd class="mono">{_fmt(planner_config.get("traversal_cost_per_meter"))}</dd><dt>hold 최소</dt><dd class="mono">{_fmt(planner_config.get("minimum_holding_time_s"))} s</dd><dt>saturation</dt><dd class="mono">{_fmt(planner_config.get("saturation_limit"), 0)}</dd><dt>closed lanes</dt><dd class="mono">{html.escape(str(planner_config.get("closed_lanes", [])))}</dd></dl></aside>
    </div>
    <div class="panel"><h2>Directed Lane DB</h2><div class="table-wrap"><table><thead><tr><th>ID</th><th>방향</th><th>길이 m</th><th>속도 제한</th><th>mutex</th><th>상태</th></tr></thead><tbody>{lane_rows}</tbody></table></div></div>
    <div class="callout"><b>Navigation DB라는 하나의 SQL DB가 있는 구조는 아닙니다.</b> 이 실험에서는 <code>rmf_traffic::agv::Graph</code> 객체가 waypoint와 directed lane을 메모리에 보관합니다. Schedule Database는 별개로 시간축 trajectory와 participant를 보관합니다.</div>
  </section>

  <section id="tab-schedule" class="tab-panel">
    <div class="callout"><b>{html.escape(schedule_status)}</b><br><code>rmf_traffic::schedule::Database</code>는 파일형 SQL DB가 아니라 실제 RMF의 메모리 schedule 구현입니다. Navigation Graph는 저장하지 않고 participant와 시간 파라미터가 있는 itinerary를 버전별 traffic state로 관리합니다.</div>
    <div class="panel">
      <h2>내부 데이터 흐름</h2>
      <div class="db-flow" aria-label="Schedule Database 내부 처리 흐름"><div id="db-flow-construct" class="db-node"><b>1 Database</b><span>construct · version</span></div><div id="db-flow-register" class="db-node"><b>2 Participant</b><span>description · profile</span></div><div id="db-flow-read" class="db-node"><b>3 Snapshot read</b><span>negotiation validator</span></div><div id="db-flow-set" class="db-node"><b>4 Participant::set</b><span>plan ID · itinerary</span></div><div id="db-flow-store" class="db-node"><b>5 Stored traffic</b><span>route · trajectory · time</span></div></div>
      <div class="controls"><button id="db-prev" type="button">← 이전</button><button id="db-next" type="button">다음 →</button><label>DB snapshot <select id="db-phase-select"></select></label><output id="db-phase-label" class="mono"></output></div>
      <div class="db-snapshot"><div><span>DB version</span><b id="db-version" class="mono">—</b></div><div><span>participants</span><b id="db-participants" class="mono">—</b></div><div><span>itinerary routes</span><b id="db-routes" class="mono">—</b></div><div><span>trajectory points</span><b id="db-points" class="mono">—</b></div></div>
      <p id="db-snapshot-meaning" class="muted"></p>
    </div>
    <div class="panel"><h2>실제 DB API operation</h2><p class="muted">각 행은 C++에서 실제 객체를 호출하기 직전·직후에 기록한 version입니다. 안전검증 실패나 no proposal이면 <code>Participant::set</code>을 호출하지 않습니다.</p><div class="table-wrap"><table><thead><tr><th>seq</th><th>action</th><th>API</th><th>participant</th><th>DB ver 전 → 후</th><th>itinerary ver 전 → 후</th><th>result</th></tr></thead><tbody id="schedule-operation-table"></tbody></table></div></div>
    <div class="panel"><h2>Database snapshot history</h2><div class="table-wrap"><table><thead><tr><th>seq</th><th>phase</th><th>DB version</th><th>participant IDs</th><th>storage</th><th>Graph 포함?</th></tr></thead><tbody id="schedule-state-table"></tbody></table></div></div>
    <div class="panel"><h2>선택 snapshot의 Participant</h2><div class="table-wrap"><table><thead><tr><th>ID</th><th>이름</th><th>owner</th><th>responsive</th><th>itinerary ver</th><th>progress ver</th><th>routes</th><th>points</th></tr></thead><tbody id="participant-table"></tbody></table></div></div>
    <div class="panel"><h2>선택 snapshot의 Itinerary Route</h2><div class="table-wrap"><table><thead><tr><th>participant</th><th>route</th><th>map</th><th>points</th><th>start s</th><th>finish s</th><th>duration s</th></tr></thead><tbody id="schedule-route-table"></tbody></table></div></div>
    <div class="panel"><h2>선택 snapshot의 Time-parameterized Trajectory</h2><div class="table-wrap"><table><thead><tr><th>participant</th><th>route:seq</th><th>time s</th><th>x</th><th>y</th><th>yaw</th><th>vx</th><th>vy</th><th>vyaw</th></tr></thead><tbody id="schedule-point-table"></tbody></table></div></div>
  </section>

  <section id="tab-negotiation" class="tab-panel">
    <div class="panel"><h2>실행 전 충돌 안전검증</h2><div class="table-wrap"><table><thead><tr><th>로봇 A</th><th>로봇 B</th><th>route 쌍</th><th>결과</th><th>충돌 시각</th><th>검사 API</th></tr></thead><tbody id="conflict-check-table"></tbody></table></div></div>
    <div class="panel"><h2>CentralizedNegotiation 로그</h2><div id="negotiation-log" class="mono muted"></div></div>
  </section>

  <section id="tab-scenarios" class="tab-panel">
    <div class="panel"><h2>커스텀 JSON 시나리오</h2><dl class="kv"><dt>현재 입력</dt><dd>{html.escape(str(custom_scenario.get("source_json", "built-in scenario")))}</dd><dt>실행 mode</dt><dd>{html.escape(str(custom_scenario.get("mode", "built-in")))}</dd><dt>nodes / directed lanes / robots</dt><dd class="mono">{_fmt(custom_scenario.get("node_count"), 0)} / {_fmt(custom_scenario.get("directed_lane_count"), 0)} / {_fmt(custom_scenario.get("robot_count"), 0)}</dd><dt>정적 경고</dt><dd><ul class="diagnosis-list">{custom_warnings}</ul></dd></dl><p><code>python3 run.py --setup ~/rmf_ws/install/setup.bash --scenario-file scenarios/custom_no_solution.json --timeout 60 --open</code></p><p class="muted">JSON의 <code>nodes</code>, <code>lanes</code>, <code>robots</code> 배열을 직접 수정합니다. lane의 <code>bidirectional:true</code>는 내부에서 두 directed lane으로 확장됩니다.</p></div>
    <div class="panel"><h2>현재 포함된 {len(SCENARIOS)}개 built-in 시나리오</h2><p class="muted">파란 배경 행이 지금 실행한 시나리오입니다. multi-robot 시나리오에서만 Negotiation과 Schedule Database가 사용됩니다.</p><div class="table-wrap"><table class="scenario-table"><thead><tr><th>scenario</th><th>로봇</th><th>실제 호출 core</th><th>실험 목적</th><th>관찰 포인트</th></tr></thead><tbody>{scenario_rows}</tbody></table></div></div>
    <div class="grid-2">
      <div class="panel"><h2>실제 RMF Traffic 사용 확인</h2><div class="proof-grid"><div><span class="muted">linked CMake target</span><br><code>{html.escape(str(rmf_proof.get("linked_target", "rmf_traffic::rmf_traffic")))}</code></div><div><span class="muted">path planning</span><br><code>{html.escape(str(rmf_proof.get("planner_api", "rmf_traffic::agv::Planner")))}</code></div><div><span class="muted">real A* debug</span><br><code>{html.escape(str(rmf_proof.get("search_api", "rmf_traffic::agv::Planner::Debug")))}</code></div><div><span class="muted">traffic negotiation</span><br><code>{html.escape(str(rmf_proof.get("negotiation_api", "rmf_traffic::agv::CentralizedNegotiation")))}</code></div><div><span class="muted">schedule state</span><br><code>{html.escape(str(rmf_proof.get("schedule_api", "rmf_traffic::schedule::Database")))}</code></div><div><span class="muted">continuous conflict check</span><br><code>{html.escape(str(rmf_proof.get("conflict_api", "rmf_traffic::DetectConflict::between")))}</code></div></div></div>
      <aside class="panel"><h2>mock와 실제의 경계</h2><dl class="kv"><dt>맵·요청</dt><dd>실험용으로 C++에서 생성</dd><dt>경로·cost·trajectory</dt><dd>실제 RMF Planner 결과</dd><dt>A* g/h/f</dt><dd>실제 Planner::Debug 노드</dd><dt>협상·대기</dt><dd>실제 CentralizedNegotiation</dd><dt>DB write</dt><dd>실제 Participant::set</dd><dt>Python</dt><dd>{html.escape(str(rmf_proof.get("python_role", "빌드·실행·HTML 렌더링만")))}</dd><dt>아직 제외</dt><dd>Fleet Adapter, ROS topic, Gazebo, 실제 로봇 제어</dd></dl></aside>
    </div>
    <div class="callout"><b>before / after 실험 권장점:</b> RMF 수정 전·후에 동일 scenario와 입력을 사용하고 JSONL의 A* expansion, plan cost, negotiation result/time, DB version, itinerary wait/finish time, conflict count를 비교해야 합니다. UI 애니메이션 모양만 비교하면 안 됩니다.</div>
  </section>

  <section id="tab-sequence" class="tab-panel">
    <div class="panel"><h2>전체 처리 시퀀스</h2><div class="pipeline"><div class="pipe-step"><b>1. Graph 입력</b><span>waypoint·lane·closure·traits</span></div><div class="pipe-step"><b>2. Free-flow A*</b><span>g/h/f frontier 확장</span></div><div class="pipe-step"><b>3. Plan 생성</b><span>waypoint·trajectory·cost</span></div><div class="pipe-step"><b>4. Traffic 협상</b><span>다중 로봇 time-space 검증</span></div><div class="pipe-step"><b>5. Schedule commit</b><span>participant itinerary 저장</span></div></div></div>
    <div class="panel"><div class="controls"><input id="event-filter" type="search" placeholder="event 또는 robot 필터"><output id="event-count"></output></div><div class="table-wrap"><table><thead><tr><th>seq</th><th>domain</th><th>event</th><th>robot</th><th>핵심 정보</th></tr></thead><tbody id="event-table"></tbody></table></div></div>
  </section>

  <section id="tab-raw" class="tab-panel">
    <div class="panel">
      <h2>가공하지 않은 전체 JSONL 이벤트</h2>
      <p class="muted">기본값은 모든 event와 모든 field입니다. <code>negotiation_log.message</code>, A* node, trajectory, Schedule DB operation을 요약하거나 생략하지 않고 JSON object 그대로 표시합니다.</p>
      <div class="controls"><label>event type <select id="raw-event-type"><option value="">전체</option></select></label><input id="raw-search" type="search" placeholder="원본 문자열 검색"><button id="raw-copy" type="button">표시 로그 복사</button><output id="raw-log-status" class="raw-log-status mono"></output></div>
      <pre id="raw-log-output" class="raw-log" aria-label="Unfiltered JSONL events"></pre>
    </div>
  </section>
</main>
<script>
(function () {{
  "use strict";
  const data = {data_json};
  const root = document.getElementById("rmf-analyzer");
  const byEvent = function(name) {{ return data.events.filter(function(e) {{ return e.event === name; }}); }};
  const fmt = function(value, digits) {{
    if (value === null || value === undefined) return "—";
    if (typeof value === "number") return value.toFixed(digits === undefined ? 3 : digits);
    if (Array.isArray(value)) return "[" + value.join(",") + "]";
    return String(value);
  }};
  root.querySelectorAll(".tab").forEach(function(button) {{
    button.addEventListener("click", function() {{
      root.querySelectorAll(".tab").forEach(function(x) {{ x.classList.remove("active"); }});
      root.querySelectorAll(".tab-panel").forEach(function(x) {{ x.classList.remove("active"); }});
      button.classList.add("active");
      root.querySelector("#tab-" + button.dataset.tab).classList.add("active");
    }});
  }});
  if(byEvent("negotiation_request").length)root.querySelector("#process-negotiation").classList.add("done");
  if(byEvent("safety_verification").length)root.querySelector("#process-safety").classList.add(data.safetyPassed?"done":"active");
  if(byEvent("schedule_commit").length)root.querySelector("#process-commit").classList.add("done");
  if(data.robots.some(function(robot){{return robot.safePhase!=="static";}}))root.querySelector("#execution-process").classList.add("active");

  const planSummary = root.querySelector("#plan-summary");
  const summaries = byEvent("plan_summary").filter(function(e) {{ return e.success; }});
  planSummary.innerHTML = summaries.length ? summaries.map(function(e) {{
    return "<p><b>" + e.robot + "</b> <span class='muted'>" + (e.phase || "") + "</span><br>cost <span class='mono'>" + fmt(e.cost) + "</span> · lanes <span class='mono'>" + fmt(e.used_lanes) + "</span></p>";
  }}).join("") : "<p class='muted'>성공한 plan 없음</p>";

  const playButton = root.querySelector("#play-button");
  const resetButton = root.querySelector("#reset-button");
  const planMode = root.querySelector("#plan-mode");
  const speedSelect = root.querySelector("#speed-select");
  const timeSlider = root.querySelector("#time-slider");
  const clock = root.querySelector("#clock-output");
  const liveStatus = root.querySelector("#live-status");
  const livePlanSource = root.querySelector("#live-plan-source");
  const liveClearance = root.querySelector("#live-clearance");
  const liveDecisionBody = root.querySelector("#live-decision-body");
  let startTime = Number(data.startTime), endTime = Number(data.endTime), duration = Math.max(0, endTime-startTime);
  let currentTime = startTime, playing = false, previousFrame = null;
  const baselineOption = planMode.querySelector('[value="baseline"]');
  baselineOption.disabled = !data.robots.some(function(robot) {{ return Boolean(robot.plans.free_flow_baseline); }});

  function activePlan(robot) {{
    if (planMode.value === "baseline") return robot.plans.free_flow_baseline || robot.plans.static;
    return robot.plans[robot.safePhase] || robot.plans.static;
  }}

  function setTimeBounds() {{
    const times=[];
    data.robots.forEach(function(robot) {{ activePlan(robot).points.forEach(function(point) {{ times.push(point.t); }}); }});
    startTime=times.length?Math.min.apply(null,times):0; endTime=times.length?Math.max.apply(null,times):startTime; duration=Math.max(0,endTime-startTime); currentTime=startTime;
    timeSlider.min=String(startTime); timeSlider.max=String(endTime); timeSlider.step="0.01"; timeSlider.value=String(startTime); playButton.disabled=duration<=0;
  }}

  function interpolate(points, time) {{
    if (!points.length) return null;
    if (time <= points[0].t) return Object.assign({{motion:"시작 대기",segment:0}}, points[0]);
    if (time >= points[points.length-1].t) return Object.assign({{motion:"완료",segment:Math.max(0,points.length-2)}}, points[points.length-1]);
    let low=0, high=points.length-1;
    while (high-low>1) {{ const middle=Math.floor((low+high)/2); if(points[middle].t<=time) low=middle; else high=middle; }}
    const a=points[low], b=points[high], dt=Math.max(1e-9,b.t-a.t), ratio=Math.max(0,Math.min(1,(time-a.t)/dt));
    const s=ratio,s2=s*s,s3=s2*s,h00=2*s3-3*s2+1,h10=s3-2*s2+s,h01=-2*s3+3*s2,h11=s3-s2;
    const dh00=6*s2-6*s,dh10=3*s2-4*s+1,dh01=-6*s2+6*s,dh11=3*s2-2*s;
    function hermite(p0,v0,p1,v1){{return h00*p0+h10*dt*v0+h01*p1+h11*dt*v1;}}
    function hermiteVelocity(p0,v0,p1,v1){{return (dh00*p0+dh10*dt*v0+dh01*p1+dh11*dt*v1)/dt;}}
    const yawDelta=Math.atan2(Math.sin(b.yaw-a.yaw),Math.cos(b.yaw-a.yaw)), yawEnd=a.yaw+yawDelta;
    const x=hermite(a.x,a.vx,b.x,b.vx),y=hermite(a.y,a.vy,b.y,b.vy),sx=hermite(a.sx,a.svx||0,b.sx,b.svx||0),sy=hermite(a.sy,a.svy||0,b.sy,b.svy||0),yaw=hermite(a.yaw,a.vyaw||0,yawEnd,b.vyaw||0);
    const vx=hermiteVelocity(a.x,a.vx,b.x,b.vx),vy=hermiteVelocity(a.y,a.vy,b.y,b.vy),vyaw=hermiteVelocity(a.yaw,a.vyaw||0,yawEnd,b.vyaw||0);
    let motion="이동"; if(Math.hypot(vx,vy)<.015) motion=Math.abs(vyaw)>.015?"제자리 회전":"Schedule 대기";
    return {{x:x,y:y,sx:sx,sy:sy,yaw:yaw,vx:vx,vy:vy,vyaw:vyaw,motion:motion,segment:low}};
  }}

  function nextWaypoint(plan,time) {{
    for(let i=0;i<plan.waypoints.length;i++) if(Number(plan.waypoints[i].time_s)>time+0.01) return plan.waypoints[i];
    return plan.waypoints.length?plan.waypoints[plan.waypoints.length-1]:null;
  }}

  function activeLane(next) {{
    if(!next||!Array.isArray(next.approach_lanes)||!next.approach_lanes.length)return null;
    const id=Number(next.approach_lanes[next.approach_lanes.length-1]); return data.lanes.find(function(lane){{return Number(lane.id)===id;}})||null;
  }}

  function renderFrame(time) {{
    currentTime=Math.max(startTime,Math.min(endTime,time)); timeSlider.value=String(currentTime); clock.value=(currentTime-startTime).toFixed(2)+" / "+duration.toFixed(2)+" s";
    const states=[], poses=[], decisions=[]; root.querySelectorAll(".graph-lane.live-active").forEach(function(lane){{lane.classList.remove("live-active");}});
    data.robots.forEach(function(robot,index) {{
      const plan=activePlan(robot), pose=interpolate(plan.points,currentTime), element=root.querySelector("#robot-"+index); if(!pose||!element)return;
      const degrees=pose.yaw*180/Math.PI; element.setAttribute("transform","translate("+pose.sx.toFixed(2)+" "+pose.sy.toFixed(2)+") rotate("+(-degrees).toFixed(2)+")");
      const label=element.querySelector(".robot-label"); if(label)label.setAttribute("transform","rotate("+degrees.toFixed(2)+")");
      const next=nextWaypoint(plan,currentTime), lane=activeLane(next); if(lane)root.querySelectorAll('[data-lane-id="'+lane.id+'"]').forEach(function(mark){{mark.classList.add("live-active");}});
      let reason="단일 로봇 A* 계획 실행";
      if(planMode.value==="baseline") reason="비교 전용: 다른 로봇의 trajectory를 고려하지 않은 독립 계획";
      else if(plan.phase==="static_no_executable_plan") reason="협상·충돌검증을 통과한 계획이 없어 이동 금지";
      else if(plan.phase==="negotiated"&&pose.motion==="Schedule 대기") reason="협상된 time-space itinerary의 대기 구간";
      else if(plan.phase==="negotiated") reason="DetectConflict 통과 Schedule plan"+(lane&&lane.mutex_group?" · mutex "+lane.mutex_group:"");
      states.push(robot.name+" "+pose.motion+" ("+pose.x.toFixed(2)+", "+pose.y.toFixed(2)+")"); poses.push({{name:robot.name,pose:pose}});
      decisions.push("<tr><td>"+robot.name+"</td><td><b>"+pose.motion+"</b> · lane "+(lane?lane.id:"—")+" · next WP "+(next&&next.graph_index!==null?next.graph_index:"—")+"<br><span class='muted'>"+reason+"</span></td></tr>");
    }});
    let minimum=Infinity,pair=""; for(let a=0;a<poses.length;a++)for(let b=a+1;b<poses.length;b++){{const distance=Math.hypot(poses[a].pose.x-poses[b].pose.x,poses[a].pose.y-poses[b].pose.y);if(distance<minimum){{minimum=distance;pair=poses[a].name+" ↔ "+poses[b].name;}}}}
    if(Number.isFinite(minimum)){{const clearance=minimum-Number(data.requiredCenterDistance||0.6),safe=clearance>=-1e-3;liveClearance.className="mono "+(safe?"clearance-safe":"clearance-danger");liveClearance.textContent=pair+" 중심거리 "+minimum.toFixed(3)+" m · 여유 "+clearance.toFixed(3)+" m "+(safe?"✓":"⚠ 겹침");}}else{{liveClearance.className="mono";liveClearance.textContent="단일 로봇";}}
    const activePhases=data.robots.map(function(robot){{return activePlan(robot).phase;}});
    livePlanSource.textContent=planMode.value==="baseline"?"⚠ free-flow baseline — RMF traffic 충돌 안전계획이 아님":(activePhases.every(function(phase){{return phase==="free_flow";}})?"단일 로봇 RMF free-flow plan":(data.safetyPassed?"실제 negotiated itinerary + DetectConflict 통과":"검증된 실행 계획 없음 — 로봇 정지"));
    liveDecisionBody.innerHTML=decisions.join(""); liveStatus.textContent=states.join(" · ");
  }}
  function setPlaying(value) {{ playing=value&&duration>0; playButton.textContent=playing?"❚❚ 일시정지":"▶ 재생"; previousFrame=null; if(playing)requestAnimationFrame(step); }}
  function step(timestamp) {{ if(!playing)return; if(previousFrame===null)previousFrame=timestamp; const next=currentTime+(timestamp-previousFrame)/1000*(Number(speedSelect.value)||1); previousFrame=timestamp; if(next>=endTime){{renderFrame(endTime);setPlaying(false);return;}} renderFrame(next);requestAnimationFrame(step); }}
  playButton.addEventListener("click",function(){{if(!playing&&currentTime>=endTime)renderFrame(startTime);setPlaying(!playing);}});
  resetButton.addEventListener("click",function(){{setPlaying(false);renderFrame(startTime);}});
  timeSlider.addEventListener("input",function(){{setPlaying(false);renderFrame(Number(timeSlider.value));}}); renderFrame(startTime);
  planMode.addEventListener("change",function(){{setPlaying(false);setTimeBounds();renderFrame(startTime);}});
  setTimeBounds(); renderFrame(startTime);

  const searchRobot=root.querySelector("#search-robot"), searchSlider=root.querySelector("#search-slider"), searchRows=root.querySelector("#search-table"), searchPlay=root.querySelector("#search-play");
  let searchTimer=null;
  const searchNames=Array.from(new Set(byEvent("astar_expand").map(function(e){{return e.robot;}})));
  searchRobot.innerHTML=searchNames.map(function(name){{return "<option>"+name+"</option>";}}).join("");
  function traceForRobot() {{ return byEvent("astar_expand").filter(function(e){{return e.robot===searchRobot.value;}}); }}
  function parentChain(trace,node) {{ const map=new Map(trace.map(function(e){{return [e.node_id,e];}})), result=[]; let current=node, guard=0; while(current&&guard++<100){{result.unshift(current.node_id+"@WP"+fmt(current.waypoint,0));current=map.get(current.parent_id);}} return result.join(" → "); }}
  function renderSearch() {{
    const trace=traceForRobot(); searchSlider.max=String(Math.max(0,trace.length-1)); const index=Math.min(Number(searchSlider.value)||0,Math.max(0,trace.length-1)); const e=trace[index];
    searchRows.innerHTML=trace.map(function(x,i){{return "<tr class='"+(i===index?"selected":"")+"'><td>"+x.step+"</td><td>"+x.node_id+"</td><td>"+fmt(x.parent_id,0)+"</td><td>"+fmt(x.waypoint,0)+"</td><td>"+fmt(x.g)+"</td><td>"+fmt(x.h)+"</td><td>"+fmt(x.f)+"</td><td>"+x.queue_size+"</td></tr>";}}).join("");
    if(!e){{root.querySelector("#search-step-label").value="trace 없음";return;}}
    const stepSummary=byEvent("astar_step_summary").find(function(x){{return x.robot===e.robot&&Number(x.step)===Number(e.step);}})||{{}};
    root.querySelector("#search-step-label").value=(index+1)+" / "+trace.length; root.querySelector("#search-node").textContent=e.node_id; root.querySelector("#search-g").textContent=fmt(e.g); root.querySelector("#search-h").textContent=fmt(e.h); root.querySelector("#search-f").textContent=fmt(e.f); root.querySelector("#search-parent").textContent="WP "+fmt(e.waypoint,0)+" / node "+fmt(e.parent_id,0); root.querySelector("#search-queue").textContent=e.queue_size+" nodes"; root.querySelector("#search-children").textContent=fmt(stepSummary.generated_children,0)+" / "+fmt(stepSummary.terminal_count,0); root.querySelector("#search-reason").textContent="frontier에서 f=g+h가 가장 낮은 실제 RMF node를 확장"+(stepSummary.solution_found?" · 이 step에서 최적해 발견":""); root.querySelector("#search-path").textContent=parentChain(trace,e);
  }}
  function stopSearchPlay(){{if(searchTimer!==null)window.clearInterval(searchTimer);searchTimer=null;searchPlay.textContent="▶ 검색 재생";}}
  searchPlay.addEventListener("click",function(){{if(searchTimer!==null){{stopSearchPlay();return;}}const trace=traceForRobot();if(!trace.length)return;if(Number(searchSlider.value)>=trace.length-1)searchSlider.value="0";searchPlay.textContent="❚❚ 정지";searchTimer=window.setInterval(function(){{const next=Number(searchSlider.value)+1;if(next>=trace.length){{stopSearchPlay();return;}}searchSlider.value=String(next);renderSearch();}},650);}});
  searchRobot.addEventListener("change",function(){{stopSearchPlay();searchSlider.value="0";renderSearch();}}); searchSlider.addEventListener("input",function(){{stopSearchPlay();renderSearch();}}); renderSearch();

  const candidateRobot=root.querySelector("#candidate-robot"), candidateTable=root.querySelector("#candidate-table");
  const candidateNames=Array.from(new Set(byEvent("route_candidate").map(function(e){{return e.robot;}})));
  candidateRobot.innerHTML=candidateNames.map(function(name){{return "<option>"+name+"</option>";}}).join("");
  function renderCandidates(){{const rows=byEvent("route_candidate").filter(function(e){{return e.robot===candidateRobot.value;}});candidateTable.innerHTML=rows.map(function(e){{return "<tr class='"+(e.selected_by_plan?"selected":"")+"'><td>"+e.rank+"</td><td class='mono'>"+fmt(e.waypoints)+"</td><td class='mono'>"+fmt(e.lanes)+"</td><td>"+fmt(e.distance_m)+"</td><td>"+fmt(e.rmf_cost)+"</td><td>"+fmt(e.delta_from_best)+"</td><td>"+fmt(e.finish_time_s)+"</td><td>"+(e.selected_by_plan?"✓":"")+"</td></tr>";}}).join("")||"<tr><td colspan='8'>후보 없음</td></tr>";}}
  candidateRobot.addEventListener("change",renderCandidates);renderCandidates();

  root.querySelector("#conflict-check-table").innerHTML=byEvent("pairwise_conflict_check").map(function(e){{return "<tr class='"+(e.passed?"":"selected")+"'><td>"+e.robot_a+"</td><td>"+e.robot_b+"</td><td>"+e.route_pair_checks+"</td><td>"+(e.passed?"통과 ✓":"충돌 ⚠")+"</td><td>"+fmt(e.earliest_conflict_time_s)+"</td><td class='mono'>"+e.method+"</td></tr>";}}).join("")||"<tr><td colspan='6'>다중 로봇 proposal이 없어 pairwise 검사를 실행하지 않음</td></tr>";
  root.querySelector("#negotiation-log").innerHTML=byEvent("negotiation_log").map(function(e){{return "<p>"+e.seq+" · "+e.message+"</p>";}}).join("")||"<p>협상 로그 없음</p>";

  const dbStates=byEvent("schedule_database_state"), dbOperations=byEvent("schedule_database_operation"), dbParticipants=byEvent("schedule_participant"), dbRoutes=byEvent("schedule_itinerary_route").concat(byEvent("schedule_database_route")), dbPoints=byEvent("schedule_trajectory_point").concat(byEvent("schedule_database_trajectory_point"));
  const dbPhaseSelect=root.querySelector("#db-phase-select"), dbPrev=root.querySelector("#db-prev"), dbNext=root.querySelector("#db-next");
  dbPhaseSelect.innerHTML=dbStates.length?dbStates.map(function(e,index){{return "<option value='"+index+"'>"+e.phase+" · DB v"+e.latest_version+"</option>";}}).join(""):"<option value='0'>Schedule DB 미사용</option>";
  root.querySelector("#schedule-operation-table").innerHTML=dbOperations.map(function(e){{
    const dbVersion=fmt(e.version_before,0)+" → "+fmt(e.version_after,0), itineraryVersion=fmt(e.itinerary_version_before,0)+" → "+fmt(e.itinerary_version_after,0);
    return "<tr><td>"+e.seq+"</td><td>"+e.action+"</td><td class='mono'>"+e.api+"</td><td>"+(e.name||fmt(e.participant_id,0))+"</td><td class='mono'>"+dbVersion+"</td><td class='mono'>"+itineraryVersion+"</td><td>"+e.result+"</td></tr>";
  }}).join("")||"<tr><td colspan='7'>이 시나리오는 Schedule Database를 생성하지 않음</td></tr>";
  root.querySelector("#schedule-state-table").innerHTML=dbStates.map(function(e,index){{return "<tr data-db-state='"+index+"'><td>"+e.seq+"</td><td>"+e.phase+"</td><td>"+e.latest_version+"</td><td class='mono'>"+fmt(e.participant_ids)+"</td><td>"+(e.storage||"in_memory")+"</td><td>"+(e.navigation_graph_stored_here?"예":"아니오")+"</td></tr>";}}).join("")||"<tr><td colspan='6'>single robot free-flow에서는 Schedule DB를 사용하지 않음</td></tr>";

  function renderDbFlow(phase){{
    const committed=phase==="proposal_committed"||String(phase).indexOf("commit_")===0;
    const active={{construct:Boolean(dbOperations.length),register:Boolean(dbParticipants.length),read:Boolean(byEvent("negotiation_request").length),set:committed,store:committed&&dbRoutes.length>0}};
    [["construct","db-flow-construct"],["register","db-flow-register"],["read","db-flow-read"],["set","db-flow-set"],["store","db-flow-store"]].forEach(function(pair){{root.querySelector("#"+pair[1]).classList.toggle("active",active[pair[0]]);}});
  }}
  function renderDbSnapshot(){{
    const index=Math.max(0,Math.min(Number(dbPhaseSelect.value)||0,Math.max(0,dbStates.length-1))), state=dbStates[index];
    root.querySelectorAll("[data-db-state]").forEach(function(row){{row.classList.toggle("selected",Number(row.dataset.dbState)===index);}});
    dbPrev.disabled=!dbStates.length||index===0; dbNext.disabled=!dbStates.length||index>=dbStates.length-1;
    if(!state){{root.querySelector("#db-phase-label").value="미사용";root.querySelector("#db-version").textContent="—";root.querySelector("#db-participants").textContent="0";root.querySelector("#db-routes").textContent="0";root.querySelector("#db-points").textContent="0";root.querySelector("#db-snapshot-meaning").textContent="단일 로봇 free-flow 계획은 Schedule DB나 Negotiation 없이 Planner만 호출합니다.";root.querySelector("#participant-table").innerHTML="<tr><td colspan='8'>participant 없음</td></tr>";root.querySelector("#schedule-route-table").innerHTML="<tr><td colspan='7'>itinerary 없음</td></tr>";root.querySelector("#schedule-point-table").innerHTML="<tr><td colspan='9'>trajectory 없음</td></tr>";renderDbFlow("");return;}}
    const phase=state.phase, participants=dbParticipants.filter(function(e){{return e.phase===phase;}}), routes=dbRoutes.filter(function(e){{return e.phase===phase;}}), points=dbPoints.filter(function(e){{return e.phase===phase;}});
    root.querySelector("#db-phase-label").value=(index+1)+" / "+dbStates.length; root.querySelector("#db-version").textContent=state.latest_version; root.querySelector("#db-participants").textContent=state.participant_count; root.querySelector("#db-routes").textContent=routes.length; root.querySelector("#db-points").textContent=points.length; root.querySelector("#db-snapshot-meaning").textContent=state.meaning;
    root.querySelector("#participant-table").innerHTML=participants.map(function(e){{return "<tr><td>"+e.participant_id+"</td><td>"+e.name+"</td><td>"+e.owner+"</td><td>"+(e.responsive?"예":"아니오")+"</td><td>"+e.itinerary_version+"</td><td>"+e.progress_version+"</td><td>"+e.route_count+"</td><td>"+e.trajectory_point_count+"</td></tr>";}}).join("")||"<tr><td colspan='8'>participant 없음</td></tr>";
    root.querySelector("#schedule-route-table").innerHTML=routes.map(function(e){{const routeId=e.route_id!==undefined?e.route_id:e.route_index;return "<tr><td>"+e.name+" (#"+e.participant_id+")</td><td>"+routeId+"</td><td>"+e.map+"</td><td>"+e.trajectory_point_count+"</td><td>"+fmt(e.start_time_s)+"</td><td>"+fmt(e.finish_time_s)+"</td><td>"+fmt(e.duration_s)+"</td></tr>";}}).join("")||"<tr><td colspan='7'>이 snapshot에는 itinerary route가 아직 없음</td></tr>";
    root.querySelector("#schedule-point-table").innerHTML=points.map(function(e){{const routeId=e.route_id!==undefined?e.route_id:e.route_index;return "<tr><td>"+e.name+" (#"+e.participant_id+")</td><td>"+routeId+":"+e.sequence+"</td><td>"+fmt(e.time_s)+"</td><td>"+fmt(e.x)+"</td><td>"+fmt(e.y)+"</td><td>"+fmt(e.yaw_rad)+"</td><td>"+fmt(e.vx)+"</td><td>"+fmt(e.vy)+"</td><td>"+fmt(e.vyaw)+"</td></tr>";}}).join("")||"<tr><td colspan='9'>이 snapshot에는 trajectory point가 아직 없음</td></tr>";
    renderDbFlow(phase);
  }}
  dbPhaseSelect.addEventListener("change",renderDbSnapshot); dbPrev.addEventListener("click",function(){{dbPhaseSelect.value=String(Math.max(0,(Number(dbPhaseSelect.value)||0)-1));renderDbSnapshot();}}); dbNext.addEventListener("click",function(){{dbPhaseSelect.value=String(Math.min(Math.max(0,dbStates.length-1),(Number(dbPhaseSelect.value)||0)+1));renderDbSnapshot();}}); renderDbSnapshot();

  const domains={{custom_scenario_loaded:"입력",run_started:"입력",rmf_runtime_proof:"RMF",runner_core_profile:"RMF",data_model:"입력",process_phase:"프로세스",graph_summary:"Graph",graph_node:"Graph",graph_lane:"Graph",vehicle_traits:"설정",planner_configuration:"설정",planning_request:"요청",negotiation_request:"요청",baseline_notice:"안전",astar_trace_started:"A*",astar_step_decision:"A*",astar_expand:"A*",astar_generated:"A*",astar_step_summary:"A*",astar_frontier_best:"A*",astar_trace_summary:"A*",plan_waypoint:"Plan",trajectory_point:"Plan",plan_summary:"Plan",route_candidate:"후보",route_choice_explanation:"후보",pairwise_conflict_check:"안전",safety_verification:"안전",schedule_database_operation:"Schedule",schedule_database_state:"Schedule",schedule_participant:"Schedule",schedule_itinerary_route:"Schedule",schedule_trajectory_point:"Schedule",schedule_database_route:"Schedule",schedule_database_trajectory_point:"Schedule",schedule_commit:"Schedule",negotiation_log:"협상",negotiation_summary:"협상",solution_diagnosis:"진단",runner_timeout:"진단"}};
  const eventFilter=root.querySelector("#event-filter"), eventTable=root.querySelector("#event-table"), eventCount=root.querySelector("#event-count");
  function eventInfo(e){{if(e.event==="astar_step_decision")return "selected node "+e.selected_node_id+" · g/h/f "+fmt(e.selected_g)+" / "+fmt(e.selected_h)+" / "+fmt(e.selected_f)+" · next f "+fmt(e.next_best_f);if(e.event==="astar_expand")return "node "+e.node_id+" · g/h/f "+fmt(e.g)+" / "+fmt(e.h)+" / "+fmt(e.f);if(e.event==="graph_lane")return e.entry+"→"+e.exit+" · "+fmt(e.length_m)+"m";if(e.event==="plan_summary")return "success "+e.success+" · cost "+fmt(e.cost)+" · lanes "+fmt(e.used_lanes);if(e.event==="trajectory_point"||e.event==="schedule_trajectory_point"||e.event==="schedule_database_trajectory_point")return "t="+fmt(e.time_s)+" · ("+fmt(e.x)+","+fmt(e.y)+")";if(e.event==="schedule_database_operation")return e.action+" · DB v"+fmt(e.version_before,0)+"→"+fmt(e.version_after,0)+" · "+e.result;if(e.message)return e.message;if(e.reason)return e.reason;return JSON.stringify(e);}}
  function renderEvents(){{const q=eventFilter.value.trim().toLowerCase();const filtered=data.events.filter(function(e){{return !q||(String(e.event)+" "+String(e.robot||"")+" "+String(domains[e.event]||"")).toLowerCase().includes(q);}});eventCount.value=filtered.length+" / "+data.events.length;eventTable.innerHTML=filtered.map(function(e){{return "<tr><td>"+fmt(e.seq,0)+"</td><td>"+(domains[e.event]||"기타")+"</td><td class='mono'>"+e.event+"</td><td>"+(e.robot||"—")+"</td><td><div class='json'>"+eventInfo(e)+"</div></td></tr>";}}).join("");}}
  eventFilter.addEventListener("input",renderEvents);renderEvents();

  const rawType=root.querySelector("#raw-event-type"),rawSearch=root.querySelector("#raw-search"),rawOutput=root.querySelector("#raw-log-output"),rawStatus=root.querySelector("#raw-log-status"),rawCopy=root.querySelector("#raw-copy");
  const rawTypes=Array.from(new Set(data.events.map(function(e){{return e.event;}}))).sort();
  rawType.innerHTML="<option value=''>전체</option>"+rawTypes.map(function(type){{return "<option>"+type+"</option>";}}).join("");
  function renderRaw(){{
    const type=rawType.value,q=rawSearch.value.toLowerCase();
    const lines=data.events.filter(function(e){{if(type&&e.event!==type)return false;const line=JSON.stringify(e);return !q||line.toLowerCase().includes(q);}}).map(function(e){{return JSON.stringify(e);}});
    rawOutput.textContent=lines.join("\\n");rawStatus.value=lines.length+" / "+data.events.length+" events";
  }}
  rawType.addEventListener("change",renderRaw);rawSearch.addEventListener("input",renderRaw);rawCopy.addEventListener("click",function(){{
    const textValue=rawOutput.textContent||"";
    if(navigator.clipboard&&navigator.clipboard.writeText){{navigator.clipboard.writeText(textValue).then(function(){{rawStatus.value="복사 완료 · "+rawStatus.value;}}).catch(function(){{rawStatus.value="복사 권한 없음";}});return;}}
    const temporary=document.createElement("textarea");temporary.value=textValue;document.body.appendChild(temporary);temporary.select();const copied=document.execCommand("copy");temporary.remove();rawStatus.value=copied?"복사 완료":"복사 실패";
  }});renderRaw();
}})();
</script>
</body>
</html>
"""
    html_path.write_text(html_text, encoding="utf-8")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Render RMF JSONL as an analysis UI")
    parser.add_argument("jsonl", type=Path)
    parser.add_argument("html", type=Path)
    args = parser.parse_args()
    render(args.jsonl, args.html)
