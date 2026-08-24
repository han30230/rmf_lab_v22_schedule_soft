#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import heapq
import json
import math
import os
import re
import shutil
import subprocess
import sys
import webbrowser
from pathlib import Path

from tools.render_html import render
from tools.scenario_templates import builtin_scenarios


ROOT = Path(__file__).resolve().parent
BUILD_DIR = ROOT / "build"
RESULT_DIR = ROOT / "results"
SCENARIOS = (
    "single_lane_bidirectional",
    "single_path",
    "single_path_closed",
    "speed_limit_choice",
    "single_path_multi",
    "occupied_corridor_detour",
    "head_on",
    "passing_bay",
    "t_junction",
    "cross_intersection",
    "disconnected",
    "staggered_departures",
    "dynamic_bottleneck_insertion",
    "dynamic_grid_5x5_insertion",
    "grid_3x3_multi",
    "grid_5x5_multi",
    "grid_10x10_multi",
    "S1_opposite_1v1",
    "S2_convoy_2v1",
    "S3_t_junction_deadlock",
    "S4_corridor_with_detour",
    "S5_delay_inside_corridor",
    "S6_comms_loss_inside",
    "S7_confirmed_release",
    "S8_all_detours_congested",
    "S9_hard_policy_off",
    "S10_zero_weight_equivalence",
)

SCENARIO_INFO = {
    "single_lane_bidirectional": "2 robots · one bidirectional lane · wait then sequential pass",
    "single_path": "1 robot · short center route vs longer detour",
    "single_path_closed": "1 robot · closed center lanes force a detour",
    "speed_limit_choice": "1 robot · short/slow route vs long/fast route",
    "single_path_multi": "2 robots · route and time negotiation on alternate paths",
    "occupied_corridor_detour": "2 robots · real RMF baseline overlap drives a detour",
    "head_on": "2 robots · no passing space; expected no-proposal baseline",
    "passing_bay": "2 robots · passing bay and holding-point negotiation",
    "t_junction": "3 robots · shared T-junction ordering",
    "cross_intersection": "4 robots · shared four-way intersection",
    "disconnected": "1 robot · disconnected graph planning failure",
    "staggered_departures": "3 robots · two at 0 s, one delayed until 8 s",
    "dynamic_bottleneck_insertion": "4 robots · 2 committed first + newcomers at 8/14 s · bypass comparison",
    "dynamic_grid_5x5_insertion": "8 robots · 4 initial + 4 staged newcomer insertions on a mesh",
    "grid_3x3_multi": "4 robots · editable 3x3 grid crossing negotiation",
    "grid_5x5_multi": "6 robots · editable 5x5 grid stress test",
    "grid_10x10_multi": "8 robots · editable 10x10 grid stress test",
    "S1_opposite_1v1": "acceptance · narrow corridor 1 vs 1 opposite direction",
    "S2_convoy_2v1": "acceptance · same-direction convoy plus opposite waiter",
    "S3_t_junction_deadlock": "acceptance · deterministic T-junction ownership",
    "S4_corridor_with_detour": "acceptance · baseline versus corridor-aware detour",
    "S5_delay_inside_corridor": "acceptance · +10 s delay shifts occupied interval",
    "S6_comms_loss_inside": "acceptance · unknown robot keeps ownership",
    "S7_confirmed_release": "acceptance · release only after confirmed exit",
    "S8_all_detours_congested": "acceptance · soft costs never erase feasibility",
    "S9_hard_policy_off": "acceptance · hard policy disabled",
    "S10_zero_weight_equivalence": "acceptance · zero policy weights match baseline",
}


def _clean_field(value: object, label: str, *, allow_empty: bool = False) -> str:
    if not isinstance(value, str):
        raise ValueError(f"{label} must be a string")
    if not allow_empty and not value:
        raise ValueError(f"{label} must not be empty")
    if "\t" in value or "\n" in value or "\r" in value:
        raise ValueError(f"{label} must not contain tabs or newlines")
    return value


def _finite_number(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label} must be a finite number")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{label} must be a finite number")
    return number


def _bool_field(value: object, label: str, default: bool = False) -> bool:
    if value is None:
        return default
    if not isinstance(value, bool):
        raise ValueError(f"{label} must be true or false")
    return value


def build_lane_penalty_configuration(
    source: Path,
    mode: str,
    automatic_penalty: float,
) -> dict:
    """Map editable scenario lanes to directed RMF lane penalty values."""
    source = source.expanduser().resolve()
    payload = json.loads(source.read_text(encoding="utf-8"))
    nodes = payload.get("nodes", [])
    lanes = payload.get("lanes", [])
    robots = payload.get("robots", [])
    if mode not in {"shared_corridor", "shortest_path", "manual"}:
        raise ValueError(
            "lane penalty mode must be shared_corridor, shortest_path or manual")
    if not math.isfinite(automatic_penalty) or automatic_penalty <= 0:
        raise ValueError("lane penalty value must be a positive finite number")

    explicit_closed = set(payload.get("closed_lanes", []))
    directed: list[dict] = []
    for source_index, lane in enumerate(lanes):
        entry = int(lane["from"])
        exit = int(lane["to"])
        directions = [(entry, exit)]
        if bool(lane.get("bidirectional", True)):
            directions.append((exit, entry))
        manual_raw = lane.get("after_penalty", 0.0)
        if isinstance(manual_raw, bool) or not isinstance(manual_raw, (int, float)):
            raise ValueError(f"lanes[{source_index}].after_penalty must be a number")
        manual_penalty = float(manual_raw)
        if not math.isfinite(manual_penalty) or manual_penalty < 0:
            raise ValueError(
                f"lanes[{source_index}].after_penalty must be finite and non-negative")
        for entry_id, exit_id in directions:
            directed_id = len(directed)
            start = nodes[entry_id]
            finish = nodes[exit_id]
            distance = math.hypot(
                float(finish["x"]) - float(start["x"]),
                float(finish["y"]) - float(start["y"]),
            )
            speed_limit = lane.get("speed_limit")
            speed = 0.7 if speed_limit is None else min(0.7, float(speed_limit))
            directed.append({
                "id": directed_id,
                "source_lane": source_index,
                "entry": entry_id,
                "exit": exit_id,
                "weight": distance / max(speed, 1e-9),
                "closed": bool(lane.get("closed", False)) or directed_id in explicit_closed,
                "manual_penalty": manual_penalty,
                "corridor": tuple(sorted((entry_id, exit_id))),
            })

    penalties: dict[int, float] = {}
    occupancy: dict[int, float] = {}
    selected_by_robot: dict[str, list[int]] = {}
    corridor_users: dict[tuple[int, int], set[str]] = {}
    if mode == "manual":
        penalties = {
            int(lane["id"]): float(lane["manual_penalty"])
            for lane in directed if lane["manual_penalty"] > 0
        }
    else:
        adjacency: dict[int, list[tuple[float, int, int]]] = {}
        for lane in directed:
            if lane["closed"]:
                continue
            adjacency.setdefault(int(lane["entry"]), []).append(
                (float(lane["weight"]), int(lane["exit"]), int(lane["id"])))
        for edges in adjacency.values():
            edges.sort(key=lambda item: (item[0], item[1], item[2]))

        for index, robot in enumerate(robots):
            start, goal = int(robot["start"]), int(robot["goal"])
            queue: list[tuple[float, int, tuple[int, ...]]] = [(0.0, start, ())]
            best: dict[int, float] = {start: 0.0}
            route: tuple[int, ...] | None = None
            while queue:
                cost, waypoint, used = heapq.heappop(queue)
                if cost > best.get(waypoint, math.inf) + 1e-12:
                    continue
                if waypoint == goal:
                    route = used
                    break
                for edge_cost, next_waypoint, lane_id in adjacency.get(waypoint, []):
                    next_cost = cost + edge_cost
                    if next_cost + 1e-12 < best.get(next_waypoint, math.inf):
                        best[next_waypoint] = next_cost
                        heapq.heappush(
                            queue, (next_cost, next_waypoint, used + (lane_id,)))
            name = str(robot.get("name", f"R{index}"))
            selected_by_robot[name] = [] if route is None else list(route)
            if route is not None:
                for lane_id in route:
                    corridor = tuple(directed[lane_id]["corridor"])
                    corridor_users.setdefault(corridor, set()).add(name)
                    if mode == "shortest_path":
                        penalties[lane_id] = automatic_penalty

        if mode == "shared_corridor":
            for lane in directed:
                users = corridor_users.get(tuple(lane["corridor"]), set())
                if len(users) >= 2:
                    occupancy[int(lane["id"])] = float(len(users))
                    penalties[int(lane["id"])] = (
                        automatic_penalty * float(len(users) - 1))

    spec = ",".join(
        f"{lane_id}:{penalty:.12g}"
        for lane_id, penalty in sorted(penalties.items())
    )
    occupancy_spec = ",".join(
        f"{lane_id}:{demand:.12g}"
        for lane_id, demand in sorted(occupancy.items())
    )
    return {
        "active": bool(penalties),
        "mode": mode,
        "automatic_penalty": automatic_penalty,
        "directed_lane_penalties": penalties,
        "directed_lane_occupancy": occupancy,
        "penalized_lane_count": len(penalties),
        "selected_baseline_lanes_by_robot": selected_by_robot,
        "shared_corridor_users": {
            f"{corridor[0]}-{corridor[1]}": sorted(users)
            for corridor, users in sorted(corridor_users.items())
            if len(users) >= 2
        },
        "environment_spec": spec,
        "occupancy_environment_spec": occupancy_spec,
        "meaning": (
            "shared_corridor mode finds physical corridors used by at least two "
            "robots' predicted shortest routes and lets the modified RMF core convert "
            "demand above capacity one into g-cost; shortest_path avoids each baseline "
            "route; manual uses each original lane's after_penalty value"
        ),
    }


def compile_custom_scenario(source: Path, destination: Path) -> tuple[str, list[str]]:
    source = source.expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(f"Custom scenario JSON does not exist: {source}")
    try:
        payload = json.loads(source.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise ValueError(f"Invalid custom scenario JSON: {exc}") from exc
    if not isinstance(payload, dict):
        raise ValueError("Custom scenario must be one JSON object")

    name = _clean_field(payload.get("name", source.stem), "name")
    if not re.fullmatch(r"[A-Za-z0-9_-]+", name):
        raise ValueError("name may contain only letters, numbers, '_' and '-'")
    description = _clean_field(
        payload.get("description", "User-defined RMF Traffic scenario"),
        "description",
        allow_empty=True,
    )
    map_name = _clean_field(payload.get("map", "L1"), "map")
    mode = payload.get("mode", "auto")
    if mode not in {"auto", "free_flow", "negotiation"}:
        raise ValueError("mode must be auto, free_flow or negotiation")

    nodes = payload.get("nodes")
    if not isinstance(nodes, list) or len(nodes) < 2:
        raise ValueError("nodes must contain at least two waypoint objects")
    compiled_nodes: list[tuple[str, float, float, bool, bool, bool, str]] = []
    for index, node in enumerate(nodes):
        if not isinstance(node, dict):
            raise ValueError(f"nodes[{index}] must be an object")
        compiled_nodes.append(
            (
                _clean_field(node.get("name", f"N{index}"), f"nodes[{index}].name"),
                _finite_number(node.get("x"), f"nodes[{index}].x"),
                _finite_number(node.get("y"), f"nodes[{index}].y"),
                _bool_field(node.get("holding"), f"nodes[{index}].holding"),
                _bool_field(node.get("parking"), f"nodes[{index}].parking"),
                _bool_field(node.get("passthrough"), f"nodes[{index}].passthrough"),
                _clean_field(
                    node.get("mutex_group", ""),
                    f"nodes[{index}].mutex_group",
                    allow_empty=True,
                ),
            )
        )

    lanes = payload.get("lanes")
    if not isinstance(lanes, list) or not lanes:
        raise ValueError("lanes must contain at least one lane object")
    explicit_closed = payload.get("closed_lanes", [])
    if not isinstance(explicit_closed, list) or any(
        isinstance(value, bool) or not isinstance(value, int) or value < 0
        for value in explicit_closed
    ):
        raise ValueError("closed_lanes must be an array of non-negative directed lane IDs")
    explicit_closed_set = set(explicit_closed)
    compiled_lanes: list[tuple[int, int, float | None, str, bool]] = []
    for index, lane in enumerate(lanes):
        if not isinstance(lane, dict):
            raise ValueError(f"lanes[{index}] must be an object")
        entry = lane.get("from", lane.get("entry"))
        exit = lane.get("to", lane.get("exit"))
        for value, label in ((entry, "from"), (exit, "to")):
            if isinstance(value, bool) or not isinstance(value, int):
                raise ValueError(f"lanes[{index}].{label} must be a node index")
            if value < 0 or value >= len(nodes):
                raise ValueError(f"lanes[{index}].{label} is outside the node array")
        if entry == exit:
            raise ValueError(f"lanes[{index}] cannot connect a node to itself")
        speed_raw = lane.get("speed_limit")
        speed = None if speed_raw is None else _finite_number(
            speed_raw, f"lanes[{index}].speed_limit"
        )
        if speed is not None and speed <= 0:
            raise ValueError(f"lanes[{index}].speed_limit must be positive")
        mutex = _clean_field(
            lane.get("mutex_group", ""),
            f"lanes[{index}].mutex_group",
            allow_empty=True,
        )
        source_closed = _bool_field(lane.get("closed"), f"lanes[{index}].closed")
        directions = [(entry, exit)]
        if _bool_field(lane.get("bidirectional"), f"lanes[{index}].bidirectional", True):
            directions.append((exit, entry))
        for directed_entry, directed_exit in directions:
            directed_id = len(compiled_lanes)
            compiled_lanes.append(
                (
                    directed_entry,
                    directed_exit,
                    speed,
                    mutex,
                    source_closed or directed_id in explicit_closed_set,
                )
            )
    if explicit_closed_set and max(explicit_closed_set) >= len(compiled_lanes):
        raise ValueError(
            f"closed_lanes contains an ID outside 0..{len(compiled_lanes)-1}"
        )

    directed_edge_to_lane: dict[tuple[int, int], int] = {}
    for lane_id, lane in enumerate(compiled_lanes):
        edge = (lane[0], lane[1])
        if edge in directed_edge_to_lane:
            raise ValueError(
                f"multiple directed lanes use edge {edge}; corridor mapping would be ambiguous")
        directed_edge_to_lane[edge] = lane_id
    corridor_rows: list[dict] = []
    lane_corridor_owner: dict[int, str] = {}
    raw_corridors = payload.get("corridors", [])
    if not isinstance(raw_corridors, list):
        raise ValueError("corridors must be an array")
    seen_corridor_ids: set[str] = set()
    for index, corridor in enumerate(raw_corridors):
        if not isinstance(corridor, dict):
            raise ValueError(f"corridors[{index}] must be an object")
        corridor_id = _clean_field(
            corridor.get("id", f"C{index}"), f"corridors[{index}].id")
        if corridor_id in seen_corridor_ids:
            raise ValueError(f"duplicate corridor id: {corridor_id}")
        seen_corridor_ids.add(corridor_id)
        capacity = corridor.get("capacity", 1)
        if isinstance(capacity, bool) or not isinstance(capacity, int) or capacity < 1:
            raise ValueError(f"corridors[{index}].capacity must be a positive integer")
        passing_allowed = _bool_field(
            corridor.get("passing_allowed"),
            f"corridors[{index}].passing_allowed")
        hard_block = _bool_field(
            corridor.get("hard_opposite_direction_block"),
            f"corridors[{index}].hard_opposite_direction_block", True)
        base_penalty = _finite_number(
            corridor.get("base_penalty", 0.0),
            f"corridors[{index}].base_penalty")
        if base_penalty < 0:
            raise ValueError(f"corridors[{index}].base_penalty must be non-negative")
        entry_a = corridor.get("holding_entry_a")
        entry_b = corridor.get("holding_entry_b")
        for value, label in ((entry_a, "holding_entry_a"), (entry_b, "holding_entry_b")):
            if value is not None and (
                isinstance(value, bool) or not isinstance(value, int)
                or value < 0 or value >= len(nodes)
            ):
                raise ValueError(f"corridors[{index}].{label} must be a node index or null")

        def resolve_edges(key: str, direction: str) -> list[int]:
            raw_edges = corridor.get(key, [])
            if not isinstance(raw_edges, list):
                raise ValueError(f"corridors[{index}].{key} must be an array")
            output: list[int] = []
            for edge_index, edge in enumerate(raw_edges):
                if (
                    not isinstance(edge, list) or len(edge) != 2
                    or any(isinstance(value, bool) or not isinstance(value, int) for value in edge)
                ):
                    raise ValueError(
                        f"corridors[{index}].{key}[{edge_index}] must be [from,to]")
                directed_id = directed_edge_to_lane.get((edge[0], edge[1]))
                if directed_id is None:
                    raise ValueError(
                        f"corridor {corridor_id} {direction} edge {edge} has no directed lane")
                previous = lane_corridor_owner.get(directed_id)
                if previous is not None and previous != corridor_id:
                    raise ValueError(
                        f"directed lane {directed_id} belongs to corridors {previous} and {corridor_id}")
                lane_corridor_owner[directed_id] = corridor_id
                output.append(directed_id)
            return output

        forward = resolve_edges("forward_edges", "A_TO_B")
        reverse = resolve_edges("reverse_edges", "B_TO_A")
        if not forward and not reverse:
            raise ValueError(f"corridor {corridor_id} must contain at least one directed lane")
        if set(forward).intersection(reverse):
            raise ValueError(f"corridor {corridor_id} maps a lane to both directions")
        corridor_rows.append({
            "id": corridor_id,
            "capacity": capacity,
            "passing_allowed": passing_allowed,
            "hard_block": hard_block,
            "holding_entry_a": entry_a,
            "holding_entry_b": entry_b,
            "base_penalty": base_penalty,
            "forward": forward,
            "reverse": reverse,
        })

    robots = payload.get("robots")
    if not isinstance(robots, list) or not robots:
        raise ValueError("robots must contain at least one robot object")
    compiled_robots: list[tuple[str, int, int, float, float, float]] = []
    seen_robot_names: set[str] = set()
    for index, robot in enumerate(robots):
        if not isinstance(robot, dict):
            raise ValueError(f"robots[{index}] must be an object")
        robot_name = _clean_field(
            robot.get("name", f"R{index}"), f"robots[{index}].name"
        )
        if robot_name in seen_robot_names:
            raise ValueError(f"Duplicate robot name: {robot_name}")
        seen_robot_names.add(robot_name)
        start, goal = robot.get("start"), robot.get("goal")
        for value, label in ((start, "start"), (goal, "goal")):
            if isinstance(value, bool) or not isinstance(value, int):
                raise ValueError(f"robots[{index}].{label} must be a node index")
            if value < 0 or value >= len(nodes):
                raise ValueError(f"robots[{index}].{label} is outside the node array")
        yaw = _finite_number(robot.get("yaw", 0.0), f"robots[{index}].yaw")
        start_time_s = _finite_number(
            robot.get("start_time_s", robot.get("start_time", 0.0)),
            f"robots[{index}].start_time_s",
        )
        if start_time_s < 0:
            raise ValueError(f"robots[{index}].start_time_s must be non-negative")
        insertion_time_s = _finite_number(
            robot.get("insertion_time_s", robot.get("dispatch_time_s", 0.0)),
            f"robots[{index}].insertion_time_s",
        )
        if insertion_time_s < 0:
            raise ValueError(f"robots[{index}].insertion_time_s must be non-negative")
        compiled_robots.append(
            (robot_name, start, goal, yaw, start_time_s, insertion_time_s))

    resolved_mode = "free_flow" if len(robots) == 1 else "negotiation" if mode == "auto" else mode
    if mode == "free_flow" and len(robots) != 1:
        raise ValueError("free_flow mode requires exactly one robot")
    if mode == "negotiation" and len(robots) < 2:
        raise ValueError("negotiation mode requires at least two robots")

    open_edges = [
        (entry, exit)
        for entry, exit, _speed, _mutex, closed in compiled_lanes
        if not closed
    ]
    warnings: list[str] = []
    for robot_name, _start, _goal, _yaw, start_time_s, insertion_time_s in compiled_robots:
        if start_time_s + 1e-9 < insertion_time_s:
            warnings.append(
                f"{robot_name}: start_time_s precedes insertion_time_s; insertion time will be used")
    building_map = payload.get("_building_map_import")
    if isinstance(building_map, dict):
        warnings.extend(
            str(message).replace("\t", " ").replace("\r", " ").replace("\n", " ")
            for message in building_map.get("warnings", [])
            if str(message).strip()
        )
    for robot_name, start, goal, _yaw, _start_time_s, _insertion_time_s in compiled_robots:
        frontier, visited = [start], {start}
        while frontier:
            current = frontier.pop()
            for entry, exit in open_edges:
                if entry == current and exit not in visited:
                    visited.add(exit)
                    frontier.append(exit)
        if goal not in visited:
            warnings.append(
                f"{robot_name}: no open directed graph path from node {start} to {goal}"
            )
    if len(robots) > 4:
        warnings.append(
            "More than four robots can make centralized negotiation grow rapidly; use --timeout"
        )

    runtime_rows: list[tuple] = []
    raw_runtime_events = payload.get("runtime_events", [])
    if not isinstance(raw_runtime_events, list):
        raise ValueError("runtime_events must be an array")
    for index, event in enumerate(raw_runtime_events):
        if not isinstance(event, dict):
            raise ValueError(f"runtime_events[{index}] must be an object")
        event_type = _clean_field(event.get("type", ""), f"runtime_events[{index}].type")
        robot = _clean_field(event.get("robot", ""), f"runtime_events[{index}].robot")
        if robot not in seen_robot_names:
            raise ValueError(f"runtime_events[{index}] refers to unknown robot {robot}")
        at_s = _finite_number(event.get("at_s"), f"runtime_events[{index}].at_s")
        if at_s < 0:
            raise ValueError(f"runtime_events[{index}].at_s must be non-negative")
        if event_type == "delay":
            delay_s = _finite_number(
                event.get("delay_s"), f"runtime_events[{index}].delay_s")
            reason = _clean_field(
                event.get("reason", "explicit_replan"),
                f"runtime_events[{index}].reason")
            runtime_rows.append((
                "DELAY", robot, at_s, delay_s,
                _bool_field(event.get("trigger_replan"),
                            f"runtime_events[{index}].trigger_replan", True),
                reason))
        elif event_type == "communication_loss":
            duration_s = _finite_number(
                event.get("duration_s"), f"runtime_events[{index}].duration_s")
            runtime_rows.append((
                "COMM_LOSS", robot, at_s, duration_s,
                _bool_field(event.get("release_on_timeout"),
                            f"runtime_events[{index}].release_on_timeout", False)))
        elif event_type == "checkpoint_release":
            corridor_id = _clean_field(
                event.get("corridor", ""), f"runtime_events[{index}].corridor")
            if corridor_id not in seen_corridor_ids:
                raise ValueError(
                    f"runtime_events[{index}] refers to unknown corridor {corridor_id}")
            runtime_rows.append((
                "CHECKPOINT_RELEASE", robot, at_s, corridor_id,
                _bool_field(event.get("checkpoint_confirmed"),
                            f"runtime_events[{index}].checkpoint_confirmed", True)))
        else:
            raise ValueError(
                f"runtime_events[{index}].type must be delay, communication_loss or checkpoint_release")

    lines = [
        "FORMAT\trmf_custom_v1",
        f"META\t{name}\t{description}",
        f"SOURCE_JSON\t{source}",
        f"MAP\t{map_name}",
        f"MODE\t{resolved_mode}",
        "DYNAMIC\t{}".format(str(bool(
            payload.get("dynamic_insertion", False)
            or any(robot[-1] > 0.0 for robot in compiled_robots)
        )).lower()),
    ]
    lines.extend(
        "NODE\t{}\t{:.12g}\t{:.12g}\t{}\t{}\t{}\t{}".format(
            node_name,
            x,
            y,
            str(holding).lower(),
            str(parking).lower(),
            str(passthrough).lower(),
            mutex,
        )
        for node_name, x, y, holding, parking, passthrough, mutex in compiled_nodes
    )
    lines.extend(
        "LANE\t{}\t{}\t{}\t{}\t{}".format(
            entry,
            exit,
            "-" if speed is None else f"{speed:.12g}",
            mutex,
            str(closed).lower(),
        )
        for entry, exit, speed, mutex, closed in compiled_lanes
    )
    for corridor in corridor_rows:
        lines.append(
            "CORRIDOR\t{}\t{}\t{}\t{}\t{}\t{}\t{:.12g}".format(
                corridor["id"], corridor["capacity"],
                str(corridor["passing_allowed"]).lower(),
                str(corridor["hard_block"]).lower(),
                "-" if corridor["holding_entry_a"] is None else corridor["holding_entry_a"],
                "-" if corridor["holding_entry_b"] is None else corridor["holding_entry_b"],
                corridor["base_penalty"],
            ))
        lines.extend(
            f"CORRIDOR_LANE\t{corridor['id']}\tA_TO_B\t{lane_id}"
            for lane_id in corridor["forward"])
        lines.extend(
            f"CORRIDOR_LANE\t{corridor['id']}\tB_TO_A\t{lane_id}"
            for lane_id in corridor["reverse"])
    lines.extend(
        f"ROBOT\t{robot_name}\t{start}\t{goal}\t{yaw:.12g}\t{start_time_s:.12g}\t{insertion_time_s:.12g}"
        for robot_name, start, goal, yaw, start_time_s, insertion_time_s in compiled_robots
    )
    for row in runtime_rows:
        if row[0] == "DELAY":
            lines.append(
                f"DELAY\t{row[1]}\t{row[2]:.12g}\t{row[3]:.12g}\t"
                f"{str(row[4]).lower()}\t{row[5]}")
        elif row[0] == "COMM_LOSS":
            lines.append(
                f"COMM_LOSS\t{row[1]}\t{row[2]:.12g}\t{row[3]:.12g}\t"
                f"{str(row[4]).lower()}")
        else:
            lines.append(
                f"CHECKPOINT_RELEASE\t{row[1]}\t{row[2]:.12g}\t{row[3]}\t"
                f"{str(row[4]).lower()}")
    lines.extend(f"WARNING\t{warning}" for warning in warnings)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return name, warnings


def find_ros_setup(explicit: Path | None) -> Path | None:
    if explicit is not None:
        setup = explicit.expanduser().resolve()
        if not setup.is_file():
            raise FileNotFoundError(f"ROS setup file does not exist: {setup}")
        return setup

    # If VS Code inherited a sourced ROS/RMF shell, keep that full environment.
    if os.environ.get("CMAKE_PREFIX_PATH"):
        return None

    ros_distro = os.environ.get("ROS_DISTRO")
    if ros_distro:
        candidate = Path("/opt/ros") / ros_distro / "setup.bash"
        if candidate.is_file():
            return candidate

    setups = list(Path("/opt/ros").glob("*/setup.bash"))
    preference = {"jazzy": 50, "humble": 40, "kilted": 30, "rolling": 20}
    setups.sort(
        key=lambda path: (preference.get(path.parent.name, 0), path.parent.name),
        reverse=True,
    )
    return setups[0] if setups else None


def environment_from_setup(setup: Path | None) -> dict[str, str]:
    if setup is None:
        return dict(os.environ)

    command = 'source "$1" && env -0'
    completed = subprocess.run(
        ["bash", "-c", command, "bash", str(setup)],
        check=True,
        stdout=subprocess.PIPE,
    )
    environment: dict[str, str] = {}
    for entry in completed.stdout.split(b"\0"):
        if not entry or b"=" not in entry:
            continue
        key, value = entry.split(b"=", 1)
        environment[key.decode()] = value.decode(errors="surrogateescape")
    return environment


def run_checked(command: list[str], environment: dict[str, str]) -> None:
    print("+", " ".join(command), flush=True)
    subprocess.run(command, cwd=ROOT, env=environment, check=True)


def rebuild_rmf_workspace(workspace: Path, base_setup: Path) -> None:
    workspace = workspace.expanduser().resolve()
    base_setup = base_setup.expanduser().resolve()
    if not workspace.is_dir():
        raise FileNotFoundError(f"RMF workspace does not exist: {workspace}")
    if not base_setup.is_file():
        raise FileNotFoundError(f"Base ROS setup does not exist: {base_setup}")
    environment = environment_from_setup(base_setup)
    command = [
        "colcon", "build", "--packages-select", "rmf_traffic",
        "--allow-overriding", "rmf_traffic", "--symlink-install",
        "--cmake-args", "-DCMAKE_BUILD_TYPE=RelWithDebInfo",
    ]
    print("+", " ".join(command), f"(cwd={workspace})", flush=True)
    subprocess.run(command, cwd=workspace, env=environment, check=True)


def _git_identity(
    source: Path | None,
) -> tuple[str | None, str | None, bool | None, str | None]:
    if source is None:
        return None, None, None, None
    source = source.expanduser().resolve()
    candidates = [source, source / "src" / "rmf_traffic"]
    for candidate in candidates:
        if not candidate.is_dir():
            continue
        completed = subprocess.run(
            ["git", "-C", str(candidate), "rev-parse", "HEAD"],
            text=True, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
        )
        if completed.returncode == 0:
            status = subprocess.run(
                ["git", "-C", str(candidate), "status", "--porcelain"],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, check=False,
            )
            diff = subprocess.run(
                ["git", "-C", str(candidate), "diff", "--binary", "HEAD"],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, check=False,
            )
            identity_bytes = status.stdout + b"\0" + diff.stdout
            dirty = bool(status.stdout.strip())
            diff_sha256 = hashlib.sha256(identity_bytes).hexdigest() if dirty else None
            return str(candidate), completed.stdout.strip(), dirty, diff_sha256
    return str(source), None, None, None


def append_core_profile(
    path: Path,
    *,
    label: str,
    setup: Path | None,
    build_dir: Path,
    rmf_source: Path | None,
    scenario_source: Path | None,
    binary: Path,
    environment: dict[str, str],
    lane_penalty_configuration: dict | None = None,
) -> None:
    next_seq = 0
    actual_occupancy_event: dict = {}
    if path.is_file():
        for raw_line in path.read_text(encoding="utf-8").splitlines():
            try:
                parsed = json.loads(raw_line)
                next_seq = max(next_seq, int(parsed.get("seq", -1)) + 1)
                if parsed.get("event") == "occupancy_penalty_configuration":
                    actual_occupancy_event = parsed
            except (json.JSONDecodeError, TypeError, ValueError):
                continue
    source_path, commit, source_dirty, source_diff_sha256 = _git_identity(rmf_source)
    scenario_sha256 = None
    if scenario_source is not None:
        resolved_scenario = scenario_source.expanduser().resolve()
        if resolved_scenario.is_file():
            scenario_sha256 = hashlib.sha256(resolved_scenario.read_bytes()).hexdigest()
    library = None
    try:
        completed = subprocess.run(
            ["ldd", str(binary)], env=environment, text=True,
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, check=False,
        )
        for line in completed.stdout.splitlines():
            if "librmf_traffic" in line:
                library = line.strip()
                break
    except OSError:
        pass
    effective_penalties = (
        actual_occupancy_event.get("directed_lane_penalties")
        if actual_occupancy_event else
        ({} if lane_penalty_configuration is None
         else lane_penalty_configuration.get("directed_lane_penalties", {}))
    )
    effective_occupancy = (
        actual_occupancy_event.get("directed_lane_occupancy")
        if actual_occupancy_event else
        ({} if lane_penalty_configuration is None
         else lane_penalty_configuration.get("directed_lane_occupancy", {}))
    )
    effective_corridors = (
        actual_occupancy_event.get("shared_corridor_users")
        if actual_occupancy_event else
        ({} if lane_penalty_configuration is None
         else lane_penalty_configuration.get("shared_corridor_users", {}))
    )
    event = {
        "seq": next_seq,
        "event": "runner_core_profile",
        "label": label,
        "setup_bash": None if setup is None else str(setup),
        "lab_build_dir": str(build_dir),
        "rmf_source": source_path,
        "rmf_source_commit": commit,
        "rmf_source_dirty": source_dirty,
        "rmf_source_diff_sha256": source_diff_sha256,
        "resolved_rmf_library": library,
        "scenario_sha256": scenario_sha256,
        "lane_penalty_active": bool(effective_penalties),
        "lane_penalty_mode": (
            None if lane_penalty_configuration is None
            else lane_penalty_configuration.get("mode")),
        "lane_penalty_value": (
            None if lane_penalty_configuration is None
            else lane_penalty_configuration.get("automatic_penalty")),
        "penalized_lane_count": len(effective_penalties or {}),
        "directed_lane_penalties": effective_penalties or {},
        "directed_lane_occupancy": effective_occupancy or {},
        "shared_corridor_users": effective_corridors or {},
        "occupancy_source": actual_occupancy_event.get(
            "source", "pre_run_route_prediction"),
        "dynamic_insertion_policy": environment.get(
            "RMF_TRAFFIC_LAB_DYNAMIC_POLICY", "fixed_existing"),
        "newcomer_penalty_value": environment.get(
            "RMF_TRAFFIC_LAB_NEWCOMER_PENALTY"),
        "traffic_policy_mode": environment.get(
            "RMF_TRAFFIC_LAB_POLICY_MODE", "baseline"),
        "random_seed": int(environment.get(
            "RMF_TRAFFIC_LAB_RANDOM_SEED", "0")),
        "policy_snapshot_path": environment.get(
            "RMF_TRAFFIC_LAB_POLICY_SNAPSHOT"),
        "policy_trace_path": environment.get("RMF_TRAFFIC_LAB_POLICY_TRACE"),
        "policy_weights": {
            "same_direction_per_second": environment.get("RMF_TRAFFIC_LAB_SAME_WEIGHT"),
            "opposite_direction_per_second": environment.get("RMF_TRAFFIC_LAB_OPPOSITE_WEIGHT"),
            "occupied_per_second": environment.get("RMF_TRAFFIC_LAB_OCCUPIED_WEIGHT"),
            "future_reservation_per_second": environment.get("RMF_TRAFFIC_LAB_FUTURE_WEIGHT"),
            "no_escape": environment.get("RMF_TRAFFIC_LAB_NO_ESCAPE_WEIGHT"),
            "static": environment.get("RMF_TRAFFIC_LAB_STATIC_WEIGHT"),
            "overlap_margin_s": environment.get("RMF_TRAFFIC_LAB_OVERLAP_MARGIN"),
            "schedule_soft_lambda": environment.get("RMF_TRAFFIC_LAB_SCHEDULE_SOFT_LAMBDA"),
            "schedule_soft_max_penalty": environment.get("RMF_TRAFFIC_LAB_SCHEDULE_SOFT_MAX_PENALTY"),
            "schedule_soft_same_weight": environment.get("RMF_TRAFFIC_LAB_SCHEDULE_SOFT_SAME_WEIGHT"),
            "schedule_soft_opposite_weight": environment.get("RMF_TRAFFIC_LAB_SCHEDULE_SOFT_OPPOSITE_WEIGHT"),
        },
        "penalty_selected_baseline_lanes_by_robot": (
            {} if lane_penalty_configuration is None
            else lane_penalty_configuration.get("selected_baseline_lanes_by_robot", {})),
        "meaning": "This identifies the RMF headers/library used for this result; compare it before and after a core modification",
    }
    with path.open("a", encoding="utf-8") as stream:
        json.dump(event, stream, ensure_ascii=False)
        stream.write("\n")


def open_result(path: Path) -> None:
    if os.environ.get("WSL_INTEROP") and shutil.which("wslpath"):
        converted = subprocess.run(
            ["wslpath", "-w", str(path)],
            check=True,
            text=True,
            stdout=subprocess.PIPE,
        ).stdout.strip()
        subprocess.Popen(
            ["cmd.exe", "/c", "start", "", converted],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        return
    webbrowser.open(path.resolve().as_uri())


def merge_policy_trace(trace_path: Path, result_path: Path) -> int:
    """Append core-emitted expansion rows with valid monotonic JSONL seq IDs."""
    if not trace_path.is_file() or trace_path.stat().st_size == 0:
        return 0
    next_seq = 0
    if result_path.is_file():
        for raw in result_path.read_text(encoding="utf-8").splitlines():
            try:
                next_seq = max(next_seq, int(json.loads(raw).get("seq", -1)) + 1)
            except (json.JSONDecodeError, TypeError, ValueError):
                continue
    merged = 0
    with result_path.open("a", encoding="utf-8") as output:
        for raw in trace_path.read_text(encoding="utf-8").splitlines():
            try:
                event = json.loads(raw)
            except json.JSONDecodeError:
                continue
            event["seq"] = next_seq
            next_seq += 1
            json.dump(event, output, ensure_ascii=False)
            output.write("\n")
            merged += 1
    return merged


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Build and run small experiments against the real rmf_traffic core"
    )
    parser.add_argument(
        "--scenario", choices=SCENARIOS, default="single_lane_bidirectional"
    )
    parser.add_argument(
        "--scenario-file",
        type=Path,
        help="User-editable custom scenario JSON; overrides --scenario",
    )
    parser.add_argument(
        "--setup",
        type=Path,
        help="Custom ROS/RMF setup.bash, e.g. ~/rmf_ws/install/setup.bash",
    )
    parser.add_argument(
        "--build-dir", type=Path, default=BUILD_DIR,
        help="Separate lab CMake directory, useful for Before/After RMF builds",
    )
    parser.add_argument(
        "--result-name",
        help="Output basename without extension; defaults to the scenario name",
    )
    parser.add_argument(
        "--core-label", default="default",
        help="Label recorded in JSONL, e.g. before or after_soft_penalty",
    )
    parser.add_argument(
        "--rmf-source", type=Path,
        help="RMF source or workspace path recorded with its git commit",
    )
    parser.add_argument(
        "--rebuild-rmf-workspace", type=Path,
        help="Run colcon build --packages-select rmf_traffic in this workspace first",
    )
    parser.add_argument(
        "--base-ros-setup", type=Path, default=Path("/opt/ros/jazzy/setup.bash"),
        help="Base ROS setup used when rebuilding a modified RMF workspace",
    )
    parser.add_argument(
        "--lane-penalty-mode",
        choices=("off", "shared_corridor", "shortest_path", "manual"),
        default="off",
        help=(
            "Experimental modified-core input: penalize shared predicted robot "
            "corridors, each pre-run shortest route, or manual lane values"),
    )
    parser.add_argument(
        "--lane-penalty-value",
        type=float,
        default=60.0,
        help="Cost added per directed lane in shortest_path mode (default: 60)",
    )
    parser.add_argument(
        "--traffic-mode",
        choices=("baseline", "soft", "schedule_soft", "hybrid", "hybrid_nego"),
        default="baseline",
        help="Corridor policy mode used by the modified DifferentialDrivePlanner",
    )
    parser.add_argument("--same-direction-weight", type=float, default=0.25)
    parser.add_argument("--opposite-direction-weight", type=float, default=8.0)
    parser.add_argument("--occupied-weight", type=float, default=1.5)
    parser.add_argument("--future-reservation-weight", type=float, default=0.6)
    parser.add_argument("--no-escape-weight", type=float, default=25.0)
    parser.add_argument("--static-policy-weight", type=float, default=0.0)
    parser.add_argument("--overlap-margin", type=float, default=0.25)
    parser.add_argument("--schedule-soft-lambda", type=float, default=0.25)
    parser.add_argument("--schedule-soft-max-penalty", type=float, default=10.0)
    parser.add_argument("--schedule-soft-same-weight", type=float, default=0.5)
    parser.add_argument("--schedule-soft-opposite-weight", type=float, default=1.5)
    parser.add_argument(
        "--dynamic-insertion-policy",
        choices=("fixed_existing", "after_nego"),
        default="fixed_existing",
        help=(
            "For dynamic scenarios, keep committed robots fixed. after_nego also "
            "feeds their used corridors into the modified RMF A* g-cost while "
            "planning each newcomer batch"
        ),
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=60.0,
        help="Maximum scenario runtime in seconds (default: 60)",
    )
    parser.add_argument(
        "--random-seed", type=int, default=0,
        help="Seed recorded for reproducible regression/stress scenarios",
    )
    parser.add_argument(
        "--skip-build",
        action="store_true",
        help="Run the existing binary without configuring/building it",
    )
    parser.add_argument(
        "--open",
        action="store_true",
        help="Open the generated simulation UI in the default browser",
    )
    parser.add_argument(
        "--no-html",
        action="store_true",
        help="Do not render an HTML report (used by the desktop simulator)",
    )
    parser.add_argument(
        "--list-scenarios",
        action="store_true",
        help="Print every included scenario and exit without building",
    )
    args = parser.parse_args()

    if args.list_scenarios:
        for name in SCENARIOS:
            print(f"{name:28} {SCENARIO_INFO[name]}")
        print("custom JSON                  --scenario-file scenarios/custom_no_solution.json")
        return 0

    if args.rebuild_rmf_workspace is not None:
        rebuild_rmf_workspace(args.rebuild_rmf_workspace, args.base_ros_setup)

    setup = find_ros_setup(args.setup)
    environment = environment_from_setup(setup)
    if setup is not None:
        print(f"ROS environment: {setup}")
    else:
        print("ROS environment: inherited from the current shell")

    build_dir = args.build_dir.expanduser().resolve()
    build_dir.mkdir(parents=True, exist_ok=True)
    RESULT_DIR.mkdir(exist_ok=True)

    scenario_name = args.scenario
    compiled_scenario: Path | None = None
    scenario_source_for_profile = args.scenario_file
    if args.scenario_file is not None:
        compiled_scenario = build_dir / "custom_scenario.rmf"
        scenario_name, warnings = compile_custom_scenario(
            args.scenario_file, compiled_scenario
        )
        print(f"Custom scenario: {args.scenario_file.expanduser().resolve()}")
        for warning in warnings:
            print(f"WARNING: {warning}", file=sys.stderr)
    elif (
        args.scenario.startswith("grid_")
        or args.scenario.startswith("S")
        or args.scenario in {
            "staggered_departures",
            "dynamic_bottleneck_insertion",
            "dynamic_grid_5x5_insertion",
        }
        or args.scenario == "occupied_corridor_detour"
        or args.lane_penalty_mode != "off"
        or args.traffic_mode != "baseline"
    ):
        generated_source = build_dir / f"{args.scenario}.json"
        generated_source.write_text(
            json.dumps(builtin_scenarios()[args.scenario], ensure_ascii=False, indent=2)
            + "\n",
            encoding="utf-8",
        )
        compiled_scenario = build_dir / "custom_scenario.rmf"
        scenario_source_for_profile = generated_source
        scenario_name, warnings = compile_custom_scenario(
            generated_source, compiled_scenario)
        print(f"Generated editable scenario: {generated_source}")
        for warning in warnings:
            print(f"WARNING: {warning}", file=sys.stderr)

    lane_penalty_configuration: dict | None = None
    environment.pop("RMF_TRAFFIC_LAB_LANE_PENALTIES", None)
    environment.pop("RMF_TRAFFIC_LAB_LANE_OCCUPANCY", None)
    environment.pop("RMF_TRAFFIC_LAB_OCCUPANCY_WEIGHT", None)
    environment.pop("RMF_TRAFFIC_LAB_OCCUPANCY_FREE_CAPACITY", None)
    environment.pop("RMF_TRAFFIC_LAB_PENALTY_MODE", None)
    environment.pop("RMF_TRAFFIC_LAB_DYNAMIC_POLICY", None)
    environment.pop("RMF_TRAFFIC_LAB_NEWCOMER_PENALTY", None)
    for variable in (
        "RMF_TRAFFIC_LAB_POLICY_MODE", "RMF_TRAFFIC_LAB_POLICY_SNAPSHOT",
        "RMF_TRAFFIC_LAB_POLICY_TRACE", "RMF_TRAFFIC_LAB_POLICY_GENERATION",
        "RMF_TRAFFIC_LAB_SAME_WEIGHT", "RMF_TRAFFIC_LAB_OPPOSITE_WEIGHT",
        "RMF_TRAFFIC_LAB_OCCUPIED_WEIGHT", "RMF_TRAFFIC_LAB_FUTURE_WEIGHT",
        "RMF_TRAFFIC_LAB_NO_ESCAPE_WEIGHT", "RMF_TRAFFIC_LAB_STATIC_WEIGHT",
        "RMF_TRAFFIC_LAB_OVERLAP_MARGIN",
        "RMF_TRAFFIC_LAB_SCHEDULE_SOFT_LAMBDA",
        "RMF_TRAFFIC_LAB_SCHEDULE_SOFT_MAX_PENALTY",
        "RMF_TRAFFIC_LAB_SCHEDULE_SOFT_SAME_WEIGHT",
        "RMF_TRAFFIC_LAB_SCHEDULE_SOFT_OPPOSITE_WEIGHT",
    ):
        environment.pop(variable, None)
    weight_values = {
        "RMF_TRAFFIC_LAB_SAME_WEIGHT": args.same_direction_weight,
        "RMF_TRAFFIC_LAB_OPPOSITE_WEIGHT": args.opposite_direction_weight,
        "RMF_TRAFFIC_LAB_OCCUPIED_WEIGHT": args.occupied_weight,
        "RMF_TRAFFIC_LAB_FUTURE_WEIGHT": args.future_reservation_weight,
        "RMF_TRAFFIC_LAB_NO_ESCAPE_WEIGHT": args.no_escape_weight,
        "RMF_TRAFFIC_LAB_STATIC_WEIGHT": args.static_policy_weight,
        "RMF_TRAFFIC_LAB_OVERLAP_MARGIN": args.overlap_margin,
        "RMF_TRAFFIC_LAB_SCHEDULE_SOFT_LAMBDA": args.schedule_soft_lambda,
        "RMF_TRAFFIC_LAB_SCHEDULE_SOFT_MAX_PENALTY": args.schedule_soft_max_penalty,
        "RMF_TRAFFIC_LAB_SCHEDULE_SOFT_SAME_WEIGHT": args.schedule_soft_same_weight,
        "RMF_TRAFFIC_LAB_SCHEDULE_SOFT_OPPOSITE_WEIGHT": args.schedule_soft_opposite_weight,
    }
    for variable, value in weight_values.items():
        if not math.isfinite(value) or value < 0:
            raise ValueError(f"{variable} must be finite and non-negative")
        environment[variable] = str(value)
    policy_snapshot_path = build_dir / "corridor_policy_snapshot.tsv"
    policy_trace_path = build_dir / "corridor_policy_trace.jsonl"
    policy_trace_path.unlink(missing_ok=True)
    environment["RMF_TRAFFIC_LAB_POLICY_MODE"] = args.traffic_mode
    environment["RMF_TRAFFIC_LAB_RANDOM_SEED"] = str(args.random_seed)
    environment["RMF_TRAFFIC_LAB_POLICY_SNAPSHOT"] = str(policy_snapshot_path)
    environment["RMF_TRAFFIC_LAB_POLICY_TRACE"] = str(policy_trace_path)
    environment["RMF_TRAFFIC_LAB_DYNAMIC_POLICY"] = args.dynamic_insertion_policy
    if args.dynamic_insertion_policy == "after_nego":
        environment["RMF_TRAFFIC_LAB_NEWCOMER_PENALTY"] = str(
            args.lane_penalty_value)
        lane_penalty_configuration = {
            "active": True,
            "mode": "after_nego",
            "automatic_penalty": args.lane_penalty_value,
            "directed_lane_penalties": {},
            "directed_lane_occupancy": {},
            "selected_baseline_lanes_by_robot": {},
            "shared_corridor_users": {},
        }
    if args.lane_penalty_mode != "off":
        if scenario_source_for_profile is None:
            raise ValueError("lane penalty mode needs an editable scenario JSON source")
        lane_penalty_configuration = build_lane_penalty_configuration(
            scenario_source_for_profile,
            args.lane_penalty_mode,
            args.lane_penalty_value,
        )
        specification = str(lane_penalty_configuration["environment_spec"])
        occupancy_specification = str(
            lane_penalty_configuration["occupancy_environment_spec"])
        environment["RMF_TRAFFIC_LAB_PENALTY_MODE"] = args.lane_penalty_mode
        if args.lane_penalty_mode == "shared_corridor":
            environment["RMF_TRAFFIC_LAB_OCCUPANCY_WEIGHT"] = str(
                args.lane_penalty_value)
            environment["RMF_TRAFFIC_LAB_OCCUPANCY_FREE_CAPACITY"] = "1"
        elif specification:
            environment["RMF_TRAFFIC_LAB_LANE_PENALTIES"] = specification
        if specification or occupancy_specification:
            print(
                "AFTER occupancy-aware core penalty: "
                f"mode={args.lane_penalty_mode} "
                f"lanes={lane_penalty_configuration['penalized_lane_count']} "
                f"penalty_spec={specification} "
                f"occupancy_spec={occupancy_specification}",
                flush=True,
            )
            if args.lane_penalty_mode == "shared_corridor":
                print(
                    "  final occupancy will be recomputed from real RMF free-flow "
                    "baseline plans inside the C++ runner",
                    flush=True,
                )
        else:
            print(
                "WARNING: lane penalty mode is enabled but no directed lanes were selected",
                file=sys.stderr,
            )

    if not args.skip_build:
        run_checked(
            [
                "cmake",
                "-S",
                str(ROOT),
                "-B",
                str(build_dir),
                "-DCMAKE_BUILD_TYPE=RelWithDebInfo",
                "-Urmf_traffic_DIR",
            ],
            environment,
        )
        jobs = str(max(1, min(os.cpu_count() or 1, 8)))
        run_checked(
            ["cmake", "--build", str(build_dir), "--parallel", jobs],
            environment,
        )

    binary = build_dir / "rmf_core_lab"
    if not binary.is_file():
        raise FileNotFoundError(
            f"Built executable was not found: {binary}. Remove --skip-build and retry."
        )

    result_name = args.result_name or scenario_name
    if not re.fullmatch(r"[A-Za-z0-9_-]+", result_name):
        raise ValueError("result-name may contain only letters, numbers, '_' and '-'")
    jsonl_path = RESULT_DIR / f"{result_name}.jsonl"
    html_path = RESULT_DIR / f"{result_name}.html"
    command = [
        str(binary),
        "--scenario",
        scenario_name,
        "--output",
        str(jsonl_path),
    ]
    if compiled_scenario is not None:
        command.extend(["--scenario-file", str(compiled_scenario)])
    print("+", " ".join(command), flush=True)
    timed_out = False
    try:
        subprocess.run(
            command,
            cwd=ROOT,
            env=environment,
            check=True,
            timeout=args.timeout,
        )
    except subprocess.TimeoutExpired:
        timed_out = True
        next_seq = 0
        if jsonl_path.is_file():
            for raw_line in jsonl_path.read_text(encoding="utf-8").splitlines():
                try:
                    next_seq = max(next_seq, int(json.loads(raw_line).get("seq", -1)) + 1)
                except (json.JSONDecodeError, TypeError, ValueError):
                    continue
        with jsonl_path.open("a", encoding="utf-8") as stream:
            timeout_events = [
                {
                    "seq": next_seq,
                    "event": "runner_timeout",
                    "timeout_s": args.timeout,
                    "interpretation": "The scenario did not finish within the runner limit",
                },
                {
                    "seq": next_seq + 1,
                    "event": "solution_diagnosis",
                    "status": "no_solution",
                    "category": "runner_timeout",
                    "confidence": "high",
                    "basis": "confirmed_by_process_timeout",
                    "root_cause": "The experiment exceeded the runner limit before RMF returned a final result",
                    "evidence": [f"timeout_s={args.timeout:g}"],
                    "recommended_actions": [
                        "Reduce robot count to find the minimum slow subset",
                        "Inspect the last raw negotiation or A* event",
                        "Increase --timeout only after distinguishing slow search from deadlock-like topology",
                    ],
                },
            ]
            for event in timeout_events:
                json.dump(event, stream, ensure_ascii=False)
                stream.write("\n")
        print(
            f"Scenario exceeded {args.timeout:g} seconds. "
            "This timeout is itself a useful failure observation.",
            file=sys.stderr,
        )

    merged_policy_rows = merge_policy_trace(policy_trace_path, jsonl_path)
    if merged_policy_rows:
        print(f"Merged {merged_policy_rows} RMF core corridor expansion rows")

    append_core_profile(
        jsonl_path,
        label=args.core_label,
        setup=setup,
        build_dir=build_dir,
        rmf_source=args.rmf_source,
        scenario_source=scenario_source_for_profile,
        binary=binary,
        environment=environment,
        lane_penalty_configuration=lane_penalty_configuration,
    )

    if not args.no_html:
        render(jsonl_path, html_path)
    print(f"JSONL: {jsonl_path}")
    if not args.no_html:
        print(f"HTML : {html_path}")
    if args.open and not args.no_html:
        open_result(html_path)
    return 124 if timed_out else 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (FileNotFoundError, subprocess.CalledProcessError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
