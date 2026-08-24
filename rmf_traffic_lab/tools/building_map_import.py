"""Convert an RMF/Traffic-Editor building-map YAML level into a lab scenario.

The converter intentionally keeps the original YAML attributes next to the
fields that the current lab runner understands.  This lets the editor round
trip corridor metadata without pretending that corridor width or a vendor
specific rotationAllowed flag is enforced by rmf_traffic::agv::Graph.
"""

from __future__ import annotations

import math
import re
from typing import Any

try:
    import yaml
except ImportError:  # pragma: no cover - depends on the target WSL image
    yaml = None


MAX_VERTICES = 20_000
MAX_LANES = 50_000


def _load_yaml(text: str) -> dict[str, Any]:
    if yaml is None:
        raise RuntimeError(
            "YAML 맵을 열려면 PyYAML이 필요합니다: "
            "python3 -m pip install -r requirements-web.txt")
    if not isinstance(text, str) or not text.strip():
        raise ValueError("YAML 파일 내용이 비어 있습니다")
    try:
        payload = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise ValueError(f"YAML 문법을 해석할 수 없습니다: {exc}") from exc
    if not isinstance(payload, dict):
        raise ValueError("building-map YAML 최상위 값은 객체여야 합니다")
    return payload


def available_levels(text: str) -> list[str]:
    payload = _load_yaml(text)
    levels = payload.get("levels")
    if not isinstance(levels, dict) or not levels:
        raise ValueError("YAML에 levels.<level> 구조가 없습니다")
    return [str(name) for name in levels]


def _finite(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{label}은 숫자여야 합니다")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label}은 숫자여야 합니다") from exc
    if not math.isfinite(number):
        raise ValueError(f"{label}은 유한한 숫자여야 합니다")
    return number


def _bool(value: Any, fallback: bool = False) -> bool:
    if value is None:
        return fallback
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"true", "yes", "1", "on"}:
            return True
        if normalized in {"false", "no", "0", "off", ""}:
            return False
    return bool(value)


def _properties(value: Any, label: str) -> dict[str, Any]:
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise ValueError(f"{label} 속성은 YAML 객체여야 합니다")
    return dict(value)


def _first(properties: dict[str, Any], *keys: str, fallback: Any = None) -> Any:
    for key in keys:
        if key in properties:
            return properties[key]
    return fallback


def _json_safe(value: Any) -> Any:
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return str(value)


def _vertex_record(raw: Any, index: int) -> tuple[float, float, str, dict[str, Any]]:
    label = f"vertices[{index}]"
    if isinstance(raw, (list, tuple)):
        if len(raw) < 2:
            raise ValueError(f"{label}에는 최소 x, y가 필요합니다")
        x, y = raw[0], raw[1]
        name = raw[2] if len(raw) >= 3 else ""
        properties = _properties(raw[3] if len(raw) >= 4 else {}, f"{label}[3]")
    elif isinstance(raw, dict):
        x, y = raw.get("x"), raw.get("y")
        name = raw.get("name", "")
        properties = _properties(
            raw.get("properties", raw.get("params", {})), f"{label}.properties")
        properties = {**properties, **{
            key: value for key, value in raw.items()
            if key not in {"x", "y", "name", "properties", "params"}
        }}
    else:
        raise ValueError(f"{label}는 배열 또는 객체여야 합니다")
    clean_name = str(name or properties.get("name") or f"V{index}").strip()
    clean_name = clean_name.replace("\t", " ").replace("\r", " ").replace("\n", " ")
    return _finite(x, f"{label}.x"), _finite(y, f"{label}.y"), clean_name, properties


def _lane_record(raw: Any, index: int) -> tuple[int, int, dict[str, Any]]:
    label = f"lanes[{index}]"
    if isinstance(raw, (list, tuple)):
        if len(raw) < 2:
            raise ValueError(f"{label}에는 최소 시작·종료 vertex index가 필요합니다")
        entry, exit = raw[0], raw[1]
        properties = _properties(raw[2] if len(raw) >= 3 else {}, f"{label}[2]")
    elif isinstance(raw, dict):
        entry = raw.get("from", raw.get("entry"))
        exit = raw.get("to", raw.get("exit"))
        properties = _properties(
            raw.get("properties", raw.get("params", {})), f"{label}.properties")
        properties = {**properties, **{
            key: value for key, value in raw.items()
            if key not in {"from", "to", "entry", "exit", "properties", "params"}
        }}
    else:
        raise ValueError(f"{label}는 배열 또는 객체여야 합니다")
    if isinstance(entry, bool) or isinstance(exit, bool):
        raise ValueError(f"{label}의 시작·종료 index가 올바르지 않습니다")
    try:
        return int(entry), int(exit), properties
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label}의 시작·종료 index는 정수여야 합니다") from exc


def convert_building_map_yaml(
    text: str,
    level_name: str | None = None,
) -> dict[str, Any]:
    payload = _load_yaml(text)
    levels = payload.get("levels")
    if not isinstance(levels, dict) or not levels:
        raise ValueError("YAML에 levels.<level> 구조가 없습니다")
    level_names = [str(name) for name in levels]
    selected = str(level_name) if level_name else level_names[0]
    if selected not in levels:
        raise ValueError(
            f"레벨 '{selected}'이 없습니다. 사용 가능: {', '.join(level_names)}")
    level = levels[selected]
    if not isinstance(level, dict):
        raise ValueError(f"levels.{selected} 값이 객체가 아닙니다")

    raw_vertices = level.get("vertices", level.get("waypoints", level.get("nodes")))
    raw_lanes = level.get("lanes")
    if not isinstance(raw_vertices, list) or not raw_vertices:
        raise ValueError(
            f"levels.{selected}.vertices가 없습니다. 보여주신 lanes 발췌가 아니라 "
            "vertices까지 포함된 전체 YAML 파일을 선택해야 합니다")
    if not isinstance(raw_lanes, list) or not raw_lanes:
        raise ValueError(f"levels.{selected}.lanes가 없습니다")
    if len(raw_vertices) > MAX_VERTICES:
        raise ValueError(f"vertex가 {MAX_VERTICES:,}개를 초과합니다")
    if len(raw_lanes) > MAX_LANES:
        raise ValueError(f"lane이 {MAX_LANES:,}개를 초과합니다")

    nodes: list[dict[str, Any]] = []
    source_vertex_properties: dict[str, dict[str, Any]] = {}
    for index, raw in enumerate(raw_vertices):
        x, y, name, properties = _vertex_record(raw, index)
        node = {
            "name": name,
            "x": x,
            "y": y,
            "holding": _bool(_first(
                properties, "holding", "is_holding_point", "holding_point")),
            "parking": _bool(_first(
                properties, "parking", "is_parking_spot", "parking_spot")),
            "passthrough": _bool(_first(
                properties, "passthrough", "is_passthrough_point", "passthrough_point")),
        }
        mutex = str(_first(
            properties, "mutex_group", "mutexGroup", fallback="") or "").strip()
        if mutex:
            node["mutex_group"] = mutex
        if _bool(_first(properties, "is_charger", "charger")):
            node["charger"] = True
        if properties:
            source_vertex_properties[str(index)] = _json_safe(properties)
        nodes.append(node)

    lanes: list[dict[str, Any]] = []
    source_lane_properties: dict[str, dict[str, Any]] = {}
    unsupported_rotation_count = 0
    corridor_count = 0
    for index, raw in enumerate(raw_lanes):
        entry, exit, properties = _lane_record(raw, index)
        if not 0 <= entry < len(nodes) or not 0 <= exit < len(nodes):
            raise ValueError(
                f"lanes[{index}]의 vertex index {entry}→{exit}가 "
                f"0..{len(nodes)-1} 범위를 벗어납니다")
        if entry == exit:
            raise ValueError(f"lanes[{index}]가 동일 vertex {entry}를 연결합니다")
        lane: dict[str, Any] = {
            "from": entry,
            "to": exit,
            # RMF building-map lanes are directed unless explicitly marked.
            "bidirectional": _bool(_first(
                properties, "bidirectional", "is_bidirectional"), False),
        }
        speed = _first(properties, "speed_limit", "speedLimit")
        if speed is not None:
            speed_value = _finite(speed, f"lanes[{index}].speed_limit")
            if speed_value <= 0:
                raise ValueError(f"lanes[{index}].speed_limit은 0보다 커야 합니다")
            lane["speed_limit"] = speed_value
        mutex = str(_first(
            properties, "mutex_group", "mutexGroup", fallback="") or "").strip()
        if mutex:
            lane["mutex_group"] = mutex
        if _bool(_first(properties, "closed", "is_closed")):
            lane["closed"] = True
        orientation = _first(properties, "orientation", "orientation_rad")
        if orientation is not None:
            lane["yaml_orientation_rad"] = _finite(
                orientation, f"lanes[{index}].orientation")
        rotation_allowed = _first(
            properties, "rotationAllowed", "rotation_allowed")
        if rotation_allowed is not None:
            lane["yaml_rotation_allowed"] = _bool(rotation_allowed)
            if not lane["yaml_rotation_allowed"]:
                unsupported_rotation_count += 1
        corridor = properties.get("corridor")
        if corridor is not None:
            corridor = _properties(corridor, f"lanes[{index}].corridor")
            left = _first(corridor, "leftWidth", "left_width")
            right = _first(corridor, "rightWidth", "right_width")
            if left is not None:
                lane["corridor_left_width"] = max(
                    0.0, _finite(left, f"lanes[{index}].corridor.leftWidth"))
            if right is not None:
                lane["corridor_right_width"] = max(
                    0.0, _finite(right, f"lanes[{index}].corridor.rightWidth"))
            reference = _first(corridor, "corridorRefPoint", "corridor_ref_point")
            if reference is not None:
                lane["corridor_ref_point"] = str(reference)
            lane["yaml_corridor"] = _json_safe(corridor)
            corridor_count += 1
        if properties:
            source_lane_properties[str(index)] = _json_safe(properties)
        lanes.append(lane)

    building_name = str(payload.get("building_name") or "building_map").strip()
    scenario_name = re.sub(
        r"[^A-Za-z0-9_-]+", "_", f"{building_name}_{selected}").strip("_")
    if not scenario_name:
        scenario_name = "imported_building_map"

    coordinate_system = str(
        level.get("coordinate_system", payload.get("coordinate_system", "unknown")))
    warnings = [
        "YAML lane은 bidirectional:true가 명시되지 않으면 방향성 Lane으로 가져왔습니다.",
        "현재 실제 Planner에는 좌표, 방향성 연결, speed_limit, mutex, holding/parking/passthrough가 반영됩니다.",
    ]
    if corridor_count:
        warnings.append(
            f"corridor 폭 속성 {corridor_count}개는 지도 폭 표시에 보존되지만 "
            "현재 rmf_traffic 충돌 geometry에는 직접 반영되지 않습니다.")
    if unsupported_rotation_count:
        warnings.append(
            f"rotationAllowed:false {unsupported_rotation_count}개는 원본 속성으로 보존되지만 "
            "현재 lab Graph compiler가 Lane 회전 제약으로 강제하지 않습니다.")
    if coordinate_system.lower() in {"reference_image", "image", "pixel"}:
        warnings.append(
            "reference_image 좌표로 보입니다. meter 변환 정보가 없는 파일이면 거리·주행시간이 "
            "픽셀 기준으로 계산될 수 있으므로 traffic-editor에서 생성된 metric nav graph인지 확인하세요.")
    if len(level_names) > 1:
        warnings.append(
            "현재 실험은 단일 level만 실행합니다. 웹의 YAML 레벨 선택에서 다른 층을 따로 불러올 수 있습니다.")

    document = {
        "name": scenario_name,
        "description": (
            f"Imported from building-map YAML '{building_name}', level '{selected}'"),
        "map": selected,
        "mode": "auto",
        "nodes": nodes,
        "lanes": lanes,
        "robots": [],
        "closed_lanes": [],
        "_building_map_import": {
            "building_name": building_name,
            "selected_level": selected,
            "available_levels": level_names,
            "coordinate_system": coordinate_system,
            "source_vertex_properties": source_vertex_properties,
            "source_lane_properties": source_lane_properties,
            "doors_preserved": bool(payload.get("doors")),
            "lifts_preserved": bool(payload.get("lifts")),
            "planner_applied_fields": [
                "vertex x/y", "directed lane entry/exit", "speed_limit",
                "mutex_group", "holding", "parking", "passthrough",
            ],
            "display_only_fields": [
                "corridor left/right width", "corridorRefPoint",
                "rotationAllowed", "lane orientation",
            ],
            "warnings": warnings,
        },
    }
    return {
        "document": document,
        "metadata": {
            "building_name": building_name,
            "selected_level": selected,
            "available_levels": level_names,
            "coordinate_system": coordinate_system,
            "node_count": len(nodes),
            "source_lane_count": len(lanes),
            "directed_lane_count": sum(
                2 if lane.get("bidirectional") else 1 for lane in lanes),
            "corridor_count": corridor_count,
            "rotation_restricted_count": unsupported_rotation_count,
            "warnings": warnings,
        },
    }
