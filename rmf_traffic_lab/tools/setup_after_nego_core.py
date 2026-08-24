#!/usr/bin/env python3
"""Prepare an isolated RMF core for staged newcomer detour experiments."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from tools.setup_after_core import after_core_patch_status, prepare_after_core


AFTER_NEGO_POLICY = "RMF_TRAFFIC_LAB_AFTER_NEGO_V1"


def prepare_after_nego_core(before_source: Path, workspace: Path) -> dict:
    result = prepare_after_core(before_source, workspace)
    result["policy"] = AFTER_NEGO_POLICY
    result["runtime_algorithm"] = (
        "Persist the real schedule::Database, keep committed itineraries fixed, "
        "and add their used corridor/mutex lanes to the modified "
        "DifferentialDrivePlanner g-cost only for each newcomer batch"
    )
    result["environment"] = (
        "RMF_TRAFFIC_LAB_DYNAMIC_POLICY=after_nego; "
        "RMF_TRAFFIC_LAB_NEWCOMER_PENALTY=cost"
    )
    metadata_path = Path(result["after_workspace"]) / ".rmf_traffic_lab_after_nego.json"
    metadata_path.write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    patched, planner = after_core_patch_status(Path(result["after_source"]))
    if not patched:
        raise RuntimeError(f"AFTER_NEGO planner patch verification failed: {planner}")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Copy rmf_traffic and prepare the real A* core for AFTER_NEGO")
    parser.add_argument(
        "--before-source", type=Path, default=Path("~/rmf_ws/src/rmf_traffic"))
    parser.add_argument(
        "--workspace", type=Path, default=Path("~/rmf_ws_after_nego"))
    args = parser.parse_args()
    print(json.dumps(
        prepare_after_nego_core(args.before_source, args.workspace),
        ensure_ascii=False,
        indent=2,
    ))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
