"""Deterministic stress-scenario generators for future batch regressions."""

from __future__ import annotations

import random

from tools.scenario_templates import _grid_scenario


def generate_grid_stress(
    size: int,
    robot_count: int,
    seed: int,
    *,
    random_start_time_max_s: float = 0.0,
) -> dict:
    if size not in {3, 5, 10}:
        raise ValueError("size must be 3, 5 or 10")
    if robot_count < 2:
        raise ValueError("robot_count must be at least 2")
    if random_start_time_max_s < 0:
        raise ValueError("random_start_time_max_s must be non-negative")
    rng = random.Random(seed)
    scenario = _grid_scenario(size, min(robot_count, 8))
    node_count = len(scenario["nodes"])
    available = list(range(node_count))
    rng.shuffle(available)
    robots = []
    for index in range(robot_count):
        start = available[index % node_count]
        goal = rng.randrange(node_count - 1)
        if goal >= start:
            goal += 1
        start_time = (
            rng.uniform(0.0, random_start_time_max_s)
            if random_start_time_max_s > 0 else 0.0)
        robots.append({
            "name": f"STRESS_R{index + 1:03d}",
            "start": start, "goal": goal, "yaw": 0.0,
            "start_time_s": round(start_time, 6),
            "insertion_time_s": 0.0,
        })
    scenario["name"] = f"stress_{size}x{size}_{robot_count}r_seed{seed}"
    scenario["description"] = (
        f"Deterministic stress case: {size}x{size}, {robot_count} robots, seed={seed}")
    scenario["robots"] = robots
    scenario["random_seed"] = seed
    return scenario
