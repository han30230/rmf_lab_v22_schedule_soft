"""Editable JSON equivalents of the C++ lab's built-in scenarios."""

from __future__ import annotations

import copy
import math


def _node(name: str, x: float, y: float, *, holding: bool = False,
          parking: bool = False, passthrough: bool = False) -> dict:
    return {
        "name": name, "x": x, "y": y, "holding": holding,
        "parking": parking, "passthrough": passthrough,
    }


def _lane(a: int, b: int, *, speed: float | None = None,
          mutex: str = "", closed: bool = False,
          after_penalty: float = 0.0) -> dict:
    lane = {"from": a, "to": b, "bidirectional": True}
    if speed is not None:
        lane["speed_limit"] = speed
    if mutex:
        lane["mutex_group"] = mutex
    if closed:
        lane["closed"] = True
    if after_penalty > 0:
        lane["after_penalty"] = after_penalty
    return lane


def _robot(
    name: str,
    start: int,
    goal: int,
    yaw: float = 0.0,
    start_time_s: float = 0.0,
    insertion_time_s: float = 0.0,
) -> dict:
    return {
        "name": name,
        "start": start,
        "goal": goal,
        "yaw": yaw,
        "start_time_s": start_time_s,
        "insertion_time_s": insertion_time_s,
    }


def _corridor(
    corridor_id: str,
    forward_edges: list[tuple[int, int]],
    *,
    capacity: int = 1,
    passing_allowed: bool = False,
    hard_opposite_direction_block: bool = True,
    holding_entry_a: int | None = None,
    holding_entry_b: int | None = None,
    base_penalty: float = 0.0,
) -> dict:
    return {
        "id": corridor_id,
        "forward_edges": [[a, b] for a, b in forward_edges],
        "reverse_edges": [[b, a] for a, b in reversed(forward_edges)],
        "capacity": capacity,
        "passing_allowed": passing_allowed,
        "hard_opposite_direction_block": hard_opposite_direction_block,
        "holding_entry_a": holding_entry_a,
        "holding_entry_b": holding_entry_b,
        "base_penalty": base_penalty,
    }


def _delay(
    robot: str,
    at_s: float,
    delay_s: float,
    *,
    trigger_replan: bool = True,
    reason: str = "maximum_delay_exceeded",
) -> dict:
    return {
        "type": "delay",
        "robot": robot,
        "at_s": at_s,
        "delay_s": delay_s,
        "trigger_replan": trigger_replan,
        "reason": reason,
    }


def _communication_loss(robot: str, at_s: float, duration_s: float) -> dict:
    return {
        "type": "communication_loss",
        "robot": robot,
        "at_s": at_s,
        "duration_s": duration_s,
        "release_on_timeout": False,
    }


def _scenario(name: str, description: str, nodes: list[dict],
              lanes: list[dict], robots: list[dict]) -> dict:
    return {
        "name": name, "description": description, "map": "L1",
        "mode": "auto", "nodes": nodes, "lanes": lanes,
        "robots": robots, "closed_lanes": [], "corridors": [],
        "runtime_events": [],
    }


def _single_path(name: str, description: str, *, closed: bool = False,
                 speed_choice: bool = False, multi: bool = False) -> dict:
    nodes = [
        _node("START", -4, 0, holding=True, parking=True),
        _node("LEFT_GATE", -2, 0, holding=True),
        _node("CENTER", 0, 0, passthrough=True),
        _node("RIGHT_GATE", 2, 0, holding=True),
        _node("GOAL", 4, 0, holding=True, parking=True),
        _node("DETOUR_LEFT", -2, 2.5, holding=True),
        _node("DETOUR_RIGHT", 2, 2.5, holding=True),
        _node("DETOUR2_LEFT", -2, -3.2, holding=True, parking=True),
        _node("DETOUR2_RIGHT", 2, -3.2, holding=True, parking=True),
    ]
    center = 0.22 if speed_choice else None
    detour = 0.70 if speed_choice else None
    lanes = [
        _lane(0, 1), _lane(1, 2, speed=center),
        _lane(2, 3, speed=center, closed=closed), _lane(3, 4),
        _lane(1, 5, speed=detour), _lane(5, 6, speed=detour),
        _lane(6, 3, speed=detour),
        _lane(1, 7, speed=detour), _lane(7, 8, speed=detour),
        _lane(8, 3, speed=detour),
    ]
    robots = [_robot("R0", 0, 4)]
    if multi:
        robots = [_robot("R_LEFT", 0, 4), _robot("R_RIGHT", 4, 0, math.pi)]
    result = _scenario(name, description, nodes, lanes, robots)
    result["corridors"] = [
        _corridor(
            "C_CENTER", [(1, 2), (2, 3)],
            holding_entry_a=1, holding_entry_b=3),
        _corridor(
            "C_UPPER", [(1, 5), (5, 6), (6, 3)],
            capacity=2, passing_allowed=True,
            hard_opposite_direction_block=False,
            holding_entry_a=1, holding_entry_b=3),
        _corridor(
            "C_LOWER", [(1, 7), (7, 8), (8, 3)],
            capacity=2, passing_allowed=True,
            hard_opposite_direction_block=False,
            holding_entry_a=1, holding_entry_b=3),
    ]
    return result


def _grid_scenario(size: int, robot_count: int) -> dict:
    """Build an editable orthogonal grid with crossing multi-robot requests."""
    spacing = 2.2
    center = (size - 1) / 2.0
    nodes: list[dict] = []
    for row in range(size):
        for column in range(size):
            perimeter = row in {0, size - 1} or column in {0, size - 1}
            nodes.append(_node(
                f"G{row}_{column}",
                (column - center) * spacing,
                (center - row) * spacing,
                holding=True,
                parking=perimeter,
            ))

    def index(row: int, column: int) -> int:
        return row * size + column

    lanes: list[dict] = []
    for row in range(size):
        for column in range(size):
            if column + 1 < size:
                lanes.append(_lane(index(row, column), index(row, column + 1)))
            if row + 1 < size:
                lanes.append(_lane(index(row, column), index(row + 1, column)))

    middle = size // 2
    pairs = [
        (index(0, 0), index(size - 1, size - 1)),
        (index(size - 1, size - 1), index(0, 0)),
        (index(0, size - 1), index(size - 1, 0)),
        (index(size - 1, 0), index(0, size - 1)),
        (index(middle, 0), index(middle, size - 1)),
        (index(middle, size - 1), index(middle, 0)),
        (index(0, middle), index(size - 1, middle)),
        (index(size - 1, middle), index(0, middle)),
    ]
    robots = [
        _robot(f"R{number:02d}", start, goal)
        for number, (start, goal) in enumerate(pairs[:robot_count])
    ]
    result = _scenario(
        f"grid_{size}x{size}_multi",
        f"{size}x{size} orthogonal grid with {len(robots)} crossing robots; negotiation stress scenario",
        nodes,
        lanes,
        robots,
    )
    result["_editing_notes"] = [
        "All grid nodes are holding points so negotiation may insert waits",
        "More than four robots can make centralized negotiation grow rapidly",
        "Add, remove or retarget robots in the desktop simulator before running",
    ]
    return result


def _staggered_departures() -> dict:
    """Two immediate requests plus one request that is known to start later."""
    result = _grid_scenario(3, 3)
    result["name"] = "staggered_departures"
    result["description"] = (
        "Two robots depart at t=0 s and a third crossing robot departs at t=8 s"
    )
    result["nodes"].append(
        _node("DELAYED_STAGING", 0.0, 4.4, holding=True, parking=True)
    )
    result["lanes"].append(_lane(9, 1))
    result["robots"] = [
        _robot("R_EASTBOUND", 3, 5, 0.0, 0.0),
        _robot("R_WESTBOUND", 5, 3, math.pi, 0.0),
        _robot("R_DELAYED", 9, 7, -math.pi / 2, 8.0),
    ]
    result["_editing_notes"] = [
        "start_time_s is the requested RMF plan start time, not a dynamic task insertion",
        "The delayed robot waits on an external staging node before its route begins",
        "Change each robot's departure time directly in the desktop robot table",
        "Use 0/0/8 first, then reduce 8 s to observe when negotiation becomes harder",
    ]
    return result


def _dynamic_bottleneck_insertion() -> dict:
    """Existing routes are committed first, then newcomers enter later."""
    nodes = [
        _node("W_STAGE_A", -7.0, 1.4, holding=True, parking=True),
        _node("W_STAGE_B", -7.0, -1.4, holding=True, parking=True),
        _node("W_GATE", -5.0, 0.0, holding=True),
        _node("MAIN_W", -2.5, 0.0, passthrough=True),
        _node("MAIN_C", 0.0, 0.0, passthrough=True),
        _node("MAIN_E", 2.5, 0.0, passthrough=True),
        _node("E_GATE", 5.0, 0.0, holding=True),
        _node("E_STAGE_A", 7.0, 1.4, holding=True, parking=True),
        _node("E_STAGE_B", 7.0, -1.4, holding=True, parking=True),
        _node("LOWER_W", -3.2, -3.7, holding=True, parking=True),
        _node("LOWER_C", 0.0, -3.7, holding=True),
        _node("LOWER_E", 3.2, -3.7, holding=True, parking=True),
        _node("UPPER_W", -3.2, 4.4, holding=True, parking=True),
        _node("UPPER_C", 0.0, 4.4, holding=True),
        _node("UPPER_E", 3.2, 4.4, holding=True, parking=True),
    ]
    lanes = [
        _lane(0, 2), _lane(1, 2),
        _lane(2, 3, mutex="main_bottleneck"),
        _lane(3, 4, mutex="main_bottleneck"),
        _lane(4, 5, mutex="main_bottleneck"),
        _lane(5, 6, mutex="main_bottleneck"),
        _lane(6, 7), _lane(6, 8),
        _lane(2, 9), _lane(9, 10), _lane(10, 11), _lane(11, 6),
        _lane(2, 12), _lane(12, 13), _lane(13, 14), _lane(14, 6),
        _lane(9, 12), _lane(11, 14),
    ]
    robots = [
        _robot("R_EXISTING_A", 0, 7, 0.0, 0.0, 0.0),
        _robot("R_EXISTING_B", 1, 8, 0.0, 1.0, 0.0),
        _robot("R_NEW_8S", 7, 0, math.pi, 8.0, 8.0),
        _robot("R_NEW_14S", 8, 1, math.pi, 14.0, 14.0),
    ]
    result = _scenario(
        "dynamic_bottleneck_insertion",
        "Two routes are committed first; two newcomers are inserted later and can use two long bypass loops",
        nodes, lanes, robots)
    result["dynamic_insertion"] = True
    result["_editing_notes"] = [
        "Before keeps already committed itineraries fixed and plans each insertion batch against the real Schedule Database",
        "After_nego penalizes the corridors and mutex groups used by committed plans only while planning each newcomer",
        "Move insertion_time_s closer together to reproduce a harder burst insertion",
    ]
    return result


def _dynamic_grid_insertion() -> dict:
    result = _grid_scenario(5, 8)
    result["name"] = "dynamic_grid_5x5_insertion"
    result["description"] = (
        "Four initial crossing routes followed by four newcomer requests on a 5x5 mesh"
    )
    insertion_times = [0.0, 0.0, 0.0, 0.0, 6.0, 9.0, 12.0, 15.0]
    for robot, insertion in zip(result["robots"], insertion_times):
        robot["insertion_time_s"] = insertion
        robot["start_time_s"] = insertion
    result["dynamic_insertion"] = True
    result["_editing_notes"] = [
        "The first four participants are negotiated and committed at t=0",
        "Each later robot is registered only at its insertion time",
        "After_nego should distribute newcomers across less-used rows and columns when alternatives exist",
    ]
    return result


def _fab_scenario(
    fab: str,
    *,
    columns: int,
    connector_columns: tuple[int, ...],
    connector_nodes_per_segment: int,
    robot_count: int,
) -> dict:
    """Large three-aisle FAB graph with multi-node vertical connectors."""
    spacing = 2.0
    row_y = (8.0, 0.0, -8.0)
    row_names = ("TOP", "CENTER", "BOTTOM")
    nodes: list[dict] = []
    for row_name, y in zip(row_names, row_y):
        for column in range(columns):
            nodes.append(_node(
                f"{fab}_{row_name}_{column + 1:02d}", column * spacing, y,
                passthrough=True))

    lanes: list[dict] = []
    for row in range(3):
        base = row * columns
        for column in range(columns - 1):
            lanes.append(_lane(base + column, base + column + 1))

    connector_holding: list[int] = []
    for column in connector_columns:
        for segment, (start, finish, y_start, y_finish) in enumerate((
            (column, columns + column, row_y[0], row_y[1]),
            (columns + column, 2 * columns + column, row_y[1], row_y[2]),
        )):
            chain = [start]
            for index in range(connector_nodes_per_segment):
                ratio = (index + 1) / (connector_nodes_per_segment + 1)
                node_id = len(nodes)
                nodes.append(_node(
                    f"{fab}_V{column + 1:02d}_{segment}_{index + 1}",
                    column * spacing,
                    y_start + (y_finish - y_start) * ratio,
                    holding=True))
                connector_holding.append(node_id)
                chain.append(node_id)
            chain.append(finish)
            for left, right in zip(chain, chain[1:]):
                lanes.append(_lane(left, right))

    parking_nodes: list[int] = []
    pocket_columns = sorted(set((1, columns // 4, columns // 2,
                                 3 * columns // 4, columns - 2)))
    for row, (row_name, y) in enumerate(zip(row_names, row_y)):
        for pocket_index, column in enumerate(pocket_columns):
            offset = 2.0 if pocket_index % 2 == 0 else -2.0
            pocket = len(nodes)
            nodes.append(_node(
                f"{fab}_{row_name}_PARK_{column + 1:02d}",
                column * spacing, y + offset,
                holding=True, parking=True))
            lanes.append(_lane(row * columns + column, pocket))
            parking_nodes.append(pocket)

    pairs: list[tuple[int, int]] = []
    pockets_per_row = len(pocket_columns)
    for row in range(3):
        first = row * pockets_per_row
        last = first + pockets_per_row - 1
        pairs.extend(((parking_nodes[first], parking_nodes[last]),
                      (parking_nodes[last], parking_nodes[first])))
    pairs.extend([
        (parking_nodes[1], parking_nodes[-2]),
        (parking_nodes[-1], parking_nodes[2]),
        (parking_nodes[pockets_per_row + 1], parking_nodes[0]),
        (parking_nodes[3], parking_nodes[2 * pockets_per_row + 1]),
        (parking_nodes[2 * pockets_per_row], parking_nodes[pockets_per_row + 3]),
        (parking_nodes[pockets_per_row + 4], parking_nodes[2 * pockets_per_row + 2]),
    ])
    robots = [
        _robot(
            f"{fab}_R{index + 1:02d}", start, goal,
            0.0 if index % 2 == 0 else math.pi,
            start_time_s=float(max(0, index - 3) * 2))
        for index, (start, goal) in enumerate(pairs[:robot_count])]
    result = _scenario(
        f"{fab}_fab_3aisle_{robot_count}robots",
        f"{fab} FAB map: {columns} nodes on each aisle, "
        f"{len(connector_columns)} vertical lines with "
        f"{2 * connector_nodes_per_segment} holding nodes per line, "
        f"{len(parking_nodes)} parking pockets and {robot_count} robots",
        nodes, lanes, robots)
    result["_editing_notes"] = [
        "Main aisle nodes are passthrough; waits use connector holding nodes or side pockets",
        "Every vertical line has at least three intermediate nodes and connects all three aisles",
        "Robots mix same-row and cross-row requests with deterministic staggered starts",
        "Parking pockets are outside the main lane to avoid blocking through traffic",
    ]
    return result


def _p4_fab_scenario() -> dict:
    return _fab_scenario(
        "P4", columns=24,
        connector_columns=(2, 4, 7, 9, 12, 14, 17, 19, 22),
        connector_nodes_per_segment=3, robot_count=10)


def _p3_fab_scenario() -> dict:
    return _fab_scenario(
        "P3", columns=28,
        connector_columns=(2, 5, 8, 11, 14, 17, 20, 23, 25, 27),
        connector_nodes_per_segment=4, robot_count=12)


def _occupied_corridor_detour() -> dict:
    """One fixed center user and one robot that can choose the upper detour."""
    result = _single_path(
        "occupied_corridor_detour",
        "A fixed center user creates predicted occupancy while a flexible robot can detour",
    )
    result["robots"] = [
        _robot("R_OCCUPY", 1, 2, 0.0),
        _robot("R_DETOUR", 0, 4, 0.0),
    ]
    # Keep 2->3 available to the flexible robot but remove 3->2, so R_OCCUPY
    # cannot loop around the upper path to reach node 2.
    result["lanes"][2]["bidirectional"] = False
    return result


def _templates() -> dict[str, dict]:
    single_lane_nodes = [
        _node("W_START", -6, 1.6, holding=True, parking=True),
        _node("W_GOAL", -6, -1.6, holding=True, parking=True),
        _node("W_GATE", -4, 0, passthrough=True),
        _node("CORRIDOR_W", -2, 0, passthrough=True),
        _node("CORRIDOR_C", 0, 0, passthrough=True),
        _node("CORRIDOR_E", 2, 0, passthrough=True),
        _node("E_GATE", 4, 0, passthrough=True),
        _node("E_START", 6, -1.6, holding=True, parking=True),
        _node("E_GOAL", 6, 1.6, holding=True, parking=True),
    ]
    single_lane_lanes = [
        _lane(0, 2), _lane(1, 2),
        _lane(2, 3, mutex="one_lane_corridor"),
        _lane(3, 4, mutex="one_lane_corridor"),
        _lane(4, 5, mutex="one_lane_corridor"),
        _lane(5, 6, mutex="one_lane_corridor"),
        _lane(6, 7), _lane(6, 8),
    ]
    head_nodes = [
        _node("LEFT", -4, 0, holding=True, parking=True),
        _node("C1", -2, 0, passthrough=True), _node("C2", 0, 0, passthrough=True),
        _node("C3", 2, 0, passthrough=True),
        _node("RIGHT", 4, 0, holding=True, parking=True),
    ]
    passing_nodes = [
        _node("LEFT", -5, 0, holding=True, parking=True),
        _node("A", -2.5, 0, holding=True), _node("B", 0, 0, holding=True),
        _node("C", 2.5, 0, holding=True),
        _node("RIGHT", 5, 0, holding=True, parking=True),
        _node("BAY", 0, 2.2, holding=True, parking=True),
        _node("LOWER_BAY_W", -1.8, -2.7, holding=True, parking=True),
        _node("LOWER_BAY_E", 1.8, -2.7, holding=True, parking=True),
    ]
    t_nodes = [
        _node("WEST", -4, 0, holding=True, parking=True),
        _node("CENTER", 0, 0, holding=True),
        _node("EAST", 4, 0, holding=True, parking=True),
        _node("NORTH", 0, 4, holding=True, parking=True),
        _node("WAIT_W", -2, 0, holding=True), _node("WAIT_E", 2, 0, holding=True),
        _node("WAIT_N", 0, 2, holding=True),
    ]
    cross_nodes = t_nodes[:4] + [
        _node("SOUTH", 0, -4, holding=True, parking=True),
        _node("W_GATE", -2, 0, holding=True), _node("E_GATE", 2, 0, holding=True),
        _node("N_GATE", 0, 2, holding=True), _node("S_GATE", 0, -2, holding=True),
    ]
    result = {
        "single_lane_bidirectional": _scenario(
            "single_lane_bidirectional", "Two opposing robots share one bidirectional lane",
            single_lane_nodes, single_lane_lanes,
            [_robot("R_WEST", 0, 8), _robot("R_EAST", 7, 1, math.pi)]),
        "single_path": _single_path("single_path", "Short center route versus detour"),
        "single_path_closed": _single_path(
            "single_path_closed", "Closed center connection forces the detour", closed=True),
        "speed_limit_choice": _single_path(
            "speed_limit_choice", "Short slow route versus long fast route", speed_choice=True),
        "single_path_multi": _single_path(
            "single_path_multi", "Two robots negotiate on center and detour routes", multi=True),
        "occupied_corridor_detour": _occupied_corridor_detour(),
        "head_on": _scenario(
            "head_on", "Endpoint exchange in a corridor with no passing bay", head_nodes,
            [_lane(i, i + 1, mutex="corridor") for i in range(4)],
            [_robot("R_LEFT", 0, 4), _robot("R_RIGHT", 4, 0, math.pi)]),
        "passing_bay": _scenario(
            "passing_bay", "Head-on exchange with a passing loop", passing_nodes,
            [_lane(0, 1), _lane(1, 2, mutex="bottleneck"),
             _lane(2, 3, mutex="bottleneck"), _lane(3, 4), _lane(1, 5), _lane(5, 3),
             _lane(1, 6), _lane(6, 7), _lane(7, 3)],
            [_robot("R_LEFT", 0, 4), _robot("R_RIGHT", 4, 0, math.pi)]),
        "t_junction": _scenario(
            "t_junction", "Three robots compete for a T-junction", t_nodes,
            [_lane(0, 4), _lane(4, 1, mutex="junction"),
             _lane(1, 5, mutex="junction"), _lane(5, 2),
             _lane(1, 6, mutex="junction"), _lane(6, 3)],
            [_robot("R_WEST", 0, 2), _robot("R_EAST", 2, 0, math.pi),
             _robot("R_NORTH", 3, 0, -math.pi / 2)]),
        "cross_intersection": _scenario(
            "cross_intersection", "Four robots cross one shared intersection", cross_nodes,
            [_lane(0, 5), _lane(5, 1, mutex="intersection"),
             _lane(1, 6, mutex="intersection"), _lane(6, 2), _lane(3, 7),
             _lane(7, 1, mutex="intersection"), _lane(1, 8, mutex="intersection"),
             _lane(8, 4)],
            [_robot("R_WEST", 0, 2), _robot("R_EAST", 2, 0, math.pi),
             _robot("R_NORTH", 3, 4, -math.pi / 2),
             _robot("R_SOUTH", 4, 3, math.pi / 2)]),
        "disconnected": _scenario(
            "disconnected", "Start and goal lie on disconnected graph islands",
            [_node("START", -3, 0, holding=True, parking=True),
             _node("ISLAND_A", -1, 0, holding=True), _node("ISLAND_B", 1, 0, holding=True),
             _node("GOAL", 3, 0, holding=True, parking=True)],
            [_lane(0, 1), _lane(2, 3)], [_robot("R0", 0, 3)]),
        "staggered_departures": _staggered_departures(),
        "dynamic_bottleneck_insertion": _dynamic_bottleneck_insertion(),
        "dynamic_grid_5x5_insertion": _dynamic_grid_insertion(),
        "P4_fab_3aisle_10robots": _p4_fab_scenario(),
        "P3_fab_3aisle_12robots": _p3_fab_scenario(),
        "grid_3x3_multi": _grid_scenario(3, 4),
        "grid_5x5_multi": _grid_scenario(5, 6),
        "grid_10x10_multi": _grid_scenario(10, 8),
    }

    result["single_lane_bidirectional"]["corridors"] = [
        _corridor(
            "C1", [(2, 3), (3, 4), (4, 5), (5, 6)],
            holding_entry_a=2, holding_entry_b=6)
    ]
    result["head_on"]["corridors"] = [
        _corridor(
            "C1", [(0, 1), (1, 2), (2, 3), (3, 4)],
            holding_entry_a=0, holding_entry_b=4)
    ]
    result["passing_bay"]["corridors"] = [
        _corridor(
            "C_MAIN", [(1, 2), (2, 3)],
            holding_entry_a=1, holding_entry_b=3),
        _corridor(
            "C_UPPER_BAY", [(1, 5), (5, 3)],
            capacity=2, passing_allowed=True,
            hard_opposite_direction_block=False,
            holding_entry_a=1, holding_entry_b=3),
        _corridor(
            "C_LOWER_BAY", [(1, 6), (6, 7), (7, 3)],
            capacity=2, passing_allowed=True,
            hard_opposite_direction_block=False,
            holding_entry_a=1, holding_entry_b=3),
    ]
    result["occupied_corridor_detour"]["corridors"] = copy.deepcopy(
        result["single_path"]["corridors"])
    # This scenario intentionally removes directed 3->2, so keep the physical
    # corridor but do not claim a reverse lane that does not exist.
    result["occupied_corridor_detour"]["corridors"][0]["reverse_edges"] = [[2, 1]]
    result["dynamic_bottleneck_insertion"]["corridors"] = [
        _corridor(
            "C_MAIN", [(2, 3), (3, 4), (4, 5), (5, 6)],
            holding_entry_a=2, holding_entry_b=6),
        _corridor(
            "C_LOWER", [(2, 9), (9, 10), (10, 11), (11, 6)],
            capacity=2, passing_allowed=True,
            hard_opposite_direction_block=False,
            holding_entry_a=2, holding_entry_b=6),
        _corridor(
            "C_UPPER", [(2, 12), (12, 13), (13, 14), (14, 6)],
            capacity=2, passing_allowed=True,
            hard_opposite_direction_block=False,
            holding_entry_a=2, holding_entry_b=6),
    ]

    # S1-S10 are explicit, editable acceptance scenarios.  Some share the same
    # topology but differ in robots, mode hints, or runtime events.
    s1 = copy.deepcopy(result["single_lane_bidirectional"])
    s1.update(name="S1_opposite_1v1", description=(
        "S1 · narrow non-passing corridor · one robot per direction"))

    s2 = copy.deepcopy(result["single_lane_bidirectional"])
    s2.update(name="S2_convoy_2v1", description=(
        "S2 · two same-direction robots form a convoy; one opposite robot waits"))
    s2["robots"] = [
        _robot("R_A1", 0, 8, 0.0, 0.0),
        _robot("R_A2", 1, 7, 0.0, 1.5),
        _robot("R_B", 7, 1, math.pi, 0.5),
    ]

    s3 = copy.deepcopy(result["t_junction"])
    s3.update(name="S3_t_junction_deadlock", description=(
        "S3 · deterministic admission ordering at a three-robot T junction"))

    s4 = copy.deepcopy(result["passing_bay"])
    s4.update(name="S4_corridor_with_detour", description=(
        "S4 · narrow main corridor plus two passing alternatives"))

    s5 = copy.deepcopy(result["single_lane_bidirectional"])
    s5.update(name="S5_delay_inside_corridor", description=(
        "S5 · R_WEST receives +10 s delay while inside C1"))
    s5["robots"][1]["start_time_s"] = 12.0
    s5["robots"][1]["insertion_time_s"] = 12.0
    s5["dynamic_insertion"] = True
    s5["runtime_events"] = [
        _delay("R_WEST", 10.0, 10.0, reason="maximum_delay_exceeded")]

    s6 = copy.deepcopy(result["single_lane_bidirectional"])
    s6.update(name="S6_comms_loss_inside", description=(
        "S6 · communication loss does not release occupied C1"))
    s6["robots"][1]["start_time_s"] = 12.0
    s6["robots"][1]["insertion_time_s"] = 12.0
    s6["dynamic_insertion"] = True
    s6["runtime_events"] = [_communication_loss("R_WEST", 10.0, 20.0)]

    s7 = copy.deepcopy(result["single_lane_bidirectional"])
    s7.update(name="S7_confirmed_release", description=(
        "S7 · opposite direction admission follows confirmed exit checkpoint"))
    s7["robots"][1]["start_time_s"] = 28.0
    s7["robots"][1]["insertion_time_s"] = 28.0
    s7["dynamic_insertion"] = True
    s7["runtime_events"] = [{
        "type": "checkpoint_release", "robot": "R_WEST", "at_s": 26.0,
        "corridor": "C1", "checkpoint_confirmed": True,
    }]

    s8 = copy.deepcopy(result["passing_bay"])
    s8.update(name="S8_all_detours_congested", description=(
        "S8 · every alternate corridor has soft traffic cost but remains feasible"))
    s8["robots"].extend([
        _robot("R_UPPER_LOAD", 5, 3, 0.0, 0.0),
        _robot("R_LOWER_LOAD", 6, 7, 0.0, 0.0),
    ])

    s9 = copy.deepcopy(s1)
    s9.update(name="S9_hard_policy_off", description=(
        "S9 · same topology as S1; compare BASELINE/SOFT with hard policy disabled"))
    s9["recommended_mode"] = "soft"

    s10 = copy.deepcopy(s4)
    s10.update(name="S10_zero_weight_equivalence", description=(
        "S10 · set every policy weight to zero and compare SOFT with BASELINE"))
    s10["recommended_mode"] = "soft"
    s10["recommended_policy_weights"] = {
        "same_direction_per_second": 0.0,
        "opposite_direction_per_second": 0.0,
        "occupied_per_second": 0.0,
        "future_reservation_per_second": 0.0,
        "no_escape": 0.0,
        "static": 0.0,
    }
    for scenario in (s1, s2, s3, s4, s5, s6, s7, s8, s9, s10):
        result[scenario["name"]] = scenario
    return result


def builtin_scenarios() -> dict[str, dict]:
    """Return deep copies so GUI edits never mutate the master templates."""
    return copy.deepcopy(_templates())
