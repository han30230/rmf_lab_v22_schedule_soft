"""Prepare an independent baseline-derived rmf_traffic workspace for SCHEDULE_SOFT.

The actual internal hook is shared with setup_after_core.py, but this helper always
copies from an unpatched BASELINE source into a dedicated workspace so OLD_SOFT/
HYBRID source trees are never used as the starting point.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

try:
    from tools.setup_after_core import PLANNER_RELATIVE_PATHS, prepare_after_core
except ModuleNotFoundError:  # direct: python tools/setup_schedule_soft_core.py
    from setup_after_core import PLANNER_RELATIVE_PATHS, prepare_after_core

SCHEDULE_SOFT_POLICY = "baseline_derived_schedule_snapshot_soft_cost_v1"
KNOWN_EXPERIMENT_MARKERS = (
    "RMF_TRAFFIC_LAB_SCHEDULE_CORRIDOR_POLICY_V",
    "RMF_TRAFFIC_LAB_LANE_PENALTY_V1",
)


def _planner_file(source: Path) -> Path:
    for relative in PLANNER_RELATIVE_PATHS:
        candidate = source / relative
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(f"DifferentialDrivePlanner.cpp를 찾을 수 없습니다: {source}")


def _verify_baseline_source(source: Path) -> None:
    planner = _planner_file(source)
    text = planner.read_text(encoding="utf-8")
    marker = next((m for m in KNOWN_EXPERIMENT_MARKERS if m in text), None)
    if marker:
        raise RuntimeError(
            "SCHEDULE_SOFT는 BASELINE에서만 분기해야 합니다. "
            f"입력 source에 기존 실험 patch marker가 있습니다: {marker} ({planner})")


def prepare_schedule_soft_core(before_source: Path, workspace: Path) -> dict:
    before_source = before_source.expanduser().resolve()
    workspace = workspace.expanduser().resolve()
    _verify_baseline_source(before_source)

    after_source = workspace / "src" / "rmf_traffic"
    marker = workspace / ".rmf_traffic_lab_schedule_soft.json"
    if after_source.exists() and not marker.is_file():
        raise RuntimeError(
            "SCHEDULE_SOFT workspace에 기존 source가 있지만 이 도구가 만든 workspace라는 "
            f"표시가 없습니다. 덮어쓰지 않습니다: {workspace}")

    result = prepare_after_core(before_source, workspace)
    metadata = dict(result)
    metadata.update({
        "policy": SCHEDULE_SOFT_POLICY,
        "variant": "schedule_soft",
        "derivation": "copied directly from BASELINE source; OLD_SOFT source is not used",
        "default_workspace": str(workspace),
        "safety_contract": {
            "feasibility_unchanged": True,
            "schedule_only": True,
            "self_itinerary_excluded": True,
            "one_fixed_snapshot_per_plan": True,
            "no_per_expansion_database_query": True,
            "lambda_zero_disables_hook": True,
            "refuse_old_soft_as_source": True,
            "refuse_unknown_existing_workspace": True,
        },
    })
    marker.write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return metadata


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Copy BASELINE rmf_traffic and prepare the independent SCHEDULE_SOFT core")
    parser.add_argument("--before-source", type=Path, default=Path("~/rmf_ws/src/rmf_traffic"))
    parser.add_argument("--workspace", type=Path, default=Path("~/rmf_ws_schedule_soft"))
    args = parser.parse_args()
    print(json.dumps(
        prepare_schedule_soft_core(args.before_source, args.workspace),
        ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
