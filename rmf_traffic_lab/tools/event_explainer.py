"""Korean explanations for rmf_core_lab JSONL events.

The explainer never replaces the raw event. It translates recorded RMF values
and explicitly calls out where the public/debug API does not expose a cause.
"""

from __future__ import annotations

import json
import math
import re
from collections import Counter
from typing import Iterable


PHASE_KO = {
    "graph_loaded": "내비게이션 그래프 적재",
    "participants_registered": "Schedule DB 참가자 등록",
    "negotiation_started": "중앙 집중식 협상 시작",
    "schedule_committed": "검증된 일정 DB 반영",
    "free_flow": "단일 로봇 자유 경로",
    "free_flow_baseline": "다른 로봇을 무시한 기준 경로",
    "negotiated": "협상으로 조정된 실행 경로",
    "rejected_negotiated": "안전검사에서 거부된 협상 경로",
    "registered": "참가자만 등록된 상태",
    "free_flow_committed": "단일 로봇 경로 저장 완료",
    "proposal_committed": "모든 협상 경로 저장 완료",
    "no_proposal": "협상안 없음",
    "proposal_rejected_by_safety_check": "충돌검사 실패로 저장 거부",
    "dynamic_all_stages_committed": "모든 동적 투입 일정 저장 완료",
}

EVENT_KO = {
    "run_started": "실험 시작",
    "rmf_runtime_proof": "실제 RMF 호출 정보",
    "process_phase": "처리 단계 전환",
    "graph_summary": "내비게이션 그래프 요약",
    "planner_graph_context": "Graph·Supergraph 구성",
    "graph_node": "그래프 노드",
    "graph_lane": "방향성 Lane",
    "vehicle_traits": "로봇 운동 특성",
    "planner_configuration": "Planner 설정",
    "validator_configuration": "Validator 구성",
    "planning_request": "경로 계획 요청",
    "astar_trace_started": "A* 추적 시작",
    "astar_step_decision": "A* 확장 노드 선택",
    "astar_expand": "A* 노드 확장",
    "astar_generated": "A* 자식 후보 생성",
    "astar_step_summary": "A* 한 단계 결과",
    "astar_frontier_best": "다음 우선 후보",
    "astar_trace_summary": "A* 탐색 요약",
    "route_candidate": "경로 후보 비교",
    "route_choice_explanation": "경로 선택 근거",
    "occupancy_penalty_configuration": "예상 점유 통로 penalty 계산",
    "plan_waypoint": "계획 Waypoint",
    "trajectory_point": "시간 좌표 궤적점",
    "plan_summary": "경로 계획 결과",
    "itinerary_summary": "Plan Itinerary",
    "route_summary": "Itinerary Route",
    "planner_timing": "Planner 계산 시간",
    "negotiation_request": "다중 로봇 협상 요청",
    "negotiation_log": "RMF 협상 내부 로그",
    "negotiation_summary": "협상 결과",
    "proposal_summary": "협상 Proposal 요약",
    "proposal_plan": "Proposal 참가자 Plan",
    "proposal_outcome": "Proposal 수락·거부 결과",
    "safety_verification": "연속시간 충돌 안전검사",
    "safety_pair_check": "로봇 쌍 충돌검사",
    "pairwise_conflict_check": "로봇 쌍 연속시간 충돌검사",
    "schedule_database_operation": "Schedule DB 작업",
    "schedule_model_schema": "Schedule DB 실제 객체 구조",
    "schedule_database_state": "Schedule DB 스냅샷",
    "schedule_participant": "Schedule 참가자",
    "schedule_database_route": "DB 저장 Route",
    "schedule_database_trajectory_point": "DB 저장 궤적점",
    "schedule_commit": "검증된 일정 저장",
    "solution_diagnosis": "최종 해 진단",
    "expectation": "시나리오 기대 결과 확인",
    "runner_core_profile": "사용한 RMF 코어 식별",
    "dynamic_run_started": "동적 투입 실행 시작",
    "dynamic_insertion_stage": "신규 로봇 투입 단계",
    "dynamic_negotiation_request": "신규 로봇 전용 협상 요청",
    "newcomer_penalty_configuration": "신규 로봇 우회 비용 계산",
    "dynamic_insertion_result": "동적 투입 단계 결과",
    "corridor_definition": "물리 Corridor 정의",
    "corridor_policy_snapshot": "Schedule 기반 정책 Snapshot",
    "corridor_schedule_interval": "Corridor 점유 시간 Index",
    "corridor_runtime_state": "Corridor Reservation 상태",
    "corridor_state_transition": "Corridor 상태 전이",
    "corridor_policy_expansion": "A* Corridor 정책 판정",
    "runtime_event_definition": "Runtime 이벤트 정의",
    "runtime_traffic_event": "Delay·통신 상태 적용",
    "replan_trigger": "명시적 Replan Trigger",
    "route_validator_result": "RMF RouteValidator 판정",
}

DIAGNOSIS_KO = {
    "executable_time_space_plan": (
        "실행 가능한 시공간 계획",
        "RMF 협상안이 생성됐고 모든 로봇 경로 쌍이 연속시간 충돌검사를 통과했습니다.",
    ),
    "continuous_time_overlap": (
        "연속시간 궤적 중첩",
        "협상안은 만들어졌지만 로봇 외곽 형상이 같은 시간대에 겹쳐 실행하면 위험합니다.",
    ),
    "individual_path_missing": (
        "개별 로봇 경로부터 없음",
        "최소 한 대가 다른 로봇이 없어도 목적지에 도달하지 못합니다. 협상으로 그래프 단절을 복구할 수는 없습니다.",
    ),
    "endpoint_exchange_without_buffer": (
        "끝점 맞교환을 위한 대피 공간 없음",
        "서로의 출발점을 목적지로 사용하는 로봇들이 하나의 통로만 공유하며, 먼저 비켜서 기다릴 독립 공간이 없습니다.",
    ),
    "single_route_no_yield_space": (
        "단일 경로에 양보 공간 없음",
        "각 로봇은 혼자서는 이동할 수 있지만 모두 같은 경로를 써야 해서 충돌을 피할 우회·대피 토폴로지가 없습니다.",
    ),
    "negotiation_no_proposal": (
        "협상 조합 탐색 실패",
        "개별 경로는 있지만 현재 토폴로지와 협상 제한 안에서 모든 로봇을 동시에 만족하는 충돌 없는 시공간 계획을 만들지 못했습니다.",
    ),
    "disconnected_topology": (
        "방향성 그래프 단절",
        "출발점에서 목적지로 이어지는 열린 방향성 Lane 연결이 없어 A*가 경로를 만들 수 없습니다.",
    ),
    "search_saturation": (
        "탐색 한도 도달",
        "해가 없다고 증명한 것이 아니라 설정된 탐색 포화 한도에 먼저 도달했습니다.",
    ),
    "planner_interrupted": (
        "Planner 중단",
        "경로 탐색이 완료되기 전에 외부 중단 조건이 발생했습니다.",
    ),
    "planner_no_solution": (
        "Planner가 해를 반환하지 않음",
        "연결성, Lane 폐쇄, 탐색 상태를 추가로 확인해야 합니다.",
    ),
    "dynamic_newcomer_no_proposal": (
        "고정된 기존 일정과 신규 로봇의 협상안 없음",
        "기존 로봇 itinerary는 Schedule DB에 그대로 둔 상태에서 신규 투입 로봇만 계획했지만 충돌 없는 제안이 만들어지지 않았습니다.",
    ),
    "dynamic_combined_plan_conflict": (
        "기존·신규 결합 궤적 충돌",
        "신규 로봇 제안은 생성됐지만 기존에 commit된 궤적까지 합친 연속시간 충돌검사를 통과하지 못했습니다.",
    ),
    "dynamic_all_insertions_committed": (
        "모든 신규 로봇 투입 성공",
        "각 신규 투입 batch가 기존 Schedule DB 일정과 협상하고 안전검사를 통과한 뒤 순서대로 commit됐습니다.",
    ),
}

ACTION_KO = {
    "Split each shared endpoint into separate start, goal and corridor-gate nodes":
        "공유 끝점을 출발·목적·복도 입구 노드로 분리하세요.",
    "Add a side bay node connected to two corridor nodes so it forms an actual alternate path":
        "사이드 베이를 복도의 서로 다른 두 노드에 연결해 실제 우회 루프를 만드세요.",
    "Mark staging or bay nodes as holding/parking points outside the bottleneck":
        "병목 밖의 대기·베이 노드를 holding/parking point로 지정하세요.",
    "Remove one robot or dispatch the requests in separate time windows to verify the resource-capacity cause":
        "로봇을 한 대 줄이거나 요청 시간을 분리해 자원 용량 문제가 원인인지 확인하세요.",
    "Add a passing loop: a side node must connect to two different corridor nodes":
        "사이드 노드를 복도의 서로 다른 두 지점에 연결해 통과 루프를 만드세요.",
    "Add holding points before entering the shared narrow section":
        "공유 협소 구간 진입 전에 holding point를 추가하세요.",
    "Reduce simultaneous robots to identify the minimum unsatisfiable set":
        "동시 로봇 수를 줄여 해를 막는 최소 로봇 조합을 찾으세요.",
    "Add or reverse the required directed lane between the disconnected components":
        "단절된 영역 사이에 필요한 방향 Lane을 추가하거나 방향을 뒤집으세요.",
    "Reopen any closed lane that removed the only route":
        "유일한 연결을 끊은 폐쇄 Lane을 다시 여세요.",
    "Move the robot start or goal onto the same reachable graph component":
        "출발지와 목적지를 서로 도달 가능한 동일 그래프 영역으로 옮기세요.",
    "Increase temporal separation or add a holding point before the shared resource":
        "공유 자원 진입 시간 간격을 늘리거나 진입 전 holding point를 추가하세요.",
    "Add a physically separated alternate lane or passing bay":
        "물리적으로 분리된 우회 Lane 또는 passing bay를 추가하세요.",
    "Do not commit the proposal until DetectConflict passes":
        "DetectConflict를 통과하기 전에는 협상안을 DB에 반영하지 마세요.",
    "Inspect the unfiltered negotiation log for rejected tables and submitted plans":
        "원본 협상 로그에서 거부된 table과 제출된 plan을 확인하세요.",
    "Add holding points before shared mutex or corridor segments":
        "공유 mutex·복도 구간 진입 전에 holding point를 추가하세요.",
    "Add a geometrically separate alternate route or passing bay":
        "기하학적으로 분리된 우회 경로나 passing bay를 추가하세요.",
    "Remove robots one at a time to find the minimum conflicting subset":
        "로봇을 한 대씩 제거해 충돌을 만드는 최소 조합을 찾으세요.",
    "If topology is sufficient, compare negotiator cost leeway, threshold and search saturation before and after code changes":
        "토폴로지가 충분하다면 코드 수정 전후의 cost leeway, threshold, 탐색 포화 한도를 비교하세요.",
    "Use this JSONL as the solved baseline for before/after comparison":
        "이 JSONL을 Before/After 비교의 정상 기준 결과로 사용하세요.",
    "Add a directed lane that connects the start component to the goal component":
        "출발 영역에서 목적 영역으로 이어지는 방향 Lane을 추가하세요.",
    "Add the reverse direction when bidirectional travel is intended":
        "양방향 이동이 목적이면 역방향 Lane도 추가하세요.",
    "Reopen a closed lane if it removed the only connection":
        "폐쇄 Lane이 유일한 연결이었다면 다시 여세요.",
    "Reduce graph branching or robot constraints to isolate the explosion":
        "그래프 분기나 로봇 제약을 줄여 탐색 폭증 원인을 분리하세요.",
    "Increase saturation only after checking for repeated or unnecessary states":
        "중복·불필요 상태를 확인한 뒤에만 saturation 한도를 높이세요.",
    "Remove the interrupt condition and rerun the same scenario":
        "중단 조건을 제거하고 동일 시나리오를 다시 실행하세요.",
    "Inspect the raw A* termination and candidate-path events":
        "원본 A* 종료 이벤트와 경로 후보 이벤트를 확인하세요.",
    "Add a directed lane or move start/goal if the graph is unreachable":
        "그래프가 단절됐다면 방향 Lane을 추가하거나 출발·목적 노드를 옮기세요.",
}

EVIDENCE_KO = {
    "robots": "협상 대상 로봇 수",
    "requested_start_times_s": "로봇별 요청 출발 시각(s)",
    "simple_path_counts": "로봇별 단순 경로 후보 수",
    "robots_with_alternate_paths": "대체 경로가 둘 이상인 로봇 수",
    "holding_points": "holding point 수",
    "parking_points": "parking point 수",
    "exact_endpoint_swap_pairs": "출발·목적지를 정확히 맞바꾸는 로봇 쌍 수",
    "closed_lanes": "폐쇄된 Lane ID",
    "disconnected": "RMF가 그래프 단절로 판정",
    "saturated": "RMF가 탐색 한도 도달로 판정",
    "interrupted": "RMF 탐색 중단 여부",
    "search_expansions": "실제 Debug A* 확장 횟수",
    "plan_cost": "RMF 최종 계획 비용",
}

SCHEDULE_ACTION_KO = {
    "construct": "빈 Schedule Database 생성",
    "register_participant": "로봇 참가자 정보 등록",
    "read_for_negotiation": "협상을 위해 현재 DB 읽기",
    "set_itinerary": "검증된 시간 궤적을 참가자 itinerary로 저장",
    "skip_commit": "협상안이 없어 저장 생략",
    "reject_commit": "안전검사 실패로 저장 거부",
    "register_dynamic_participant": "신규 로봇 participant 동적 등록",
    "set_dynamic_newcomer_itinerary": "신규 로봇의 검증된 itinerary 저장",
}

NEGOTIATION_ACTION_KO = {
    "select_table": "협상 Table 선택",
    "submit_plan": "계획 제출",
    "reject": "협상 분기 Reject",
    "forfeit": "협상 분기 Forfeit",
    "skip": "분기 건너뜀",
    "resolve": "협상 분기 완료",
    "other": "기타 RMF 로그",
}


def _fmt(value: object, digits: int = 3) -> str:
    if value is None or value == "":
        return "없음"
    if isinstance(value, bool):
        return "예" if value else "아니요"
    if isinstance(value, float):
        if not math.isfinite(value):
            return str(value)
        return f"{value:.{digits}f}"
    return str(value)


def _phase(value: object) -> str:
    text = str(value or "")
    return PHASE_KO.get(text, text or "공통")


def _movement(value: object) -> str:
    return {
        "start": "출발 자세",
        "rotate_in_place": "제자리 회전 후 방향 정렬",
        "wait": "같은 위치에서 시간 대기",
        "forward_traverse": "전진 주행",
    }.get(str(value or ""), str(value or "이동"))


def _movement_reason(event: dict) -> str:
    return {
        "start": "Planner 요청에 입력된 출발 위치와 초기 방향을 시작 자세로 채택했습니다.",
        "rotate_in_place": "후진을 금지했기 때문에 다음 전진 구간의 방향과 맞도록 제자리에서 회전합니다.",
        "wait": "위치를 바꾸지 않고 시간만 진행해 이벤트 또는 협상에서 정한 시공간 제약을 만족합니다.",
        "forward_traverse": "최종 RMF plan이 선택한 접근 Lane을 로봇의 전방 방향으로 통과합니다.",
    }.get(
        str(event.get("movement_type", "")),
        "최종 RMF plan에 포함된 waypoint와 시간 좌표를 따릅니다.",
    )


def _evidence_line(raw: object) -> str:
    text = str(raw)
    if "=" not in text:
        return text
    key, value = text.split("=", 1)
    return f"{EVIDENCE_KO.get(key, key)}: {value}"


def _astar_breakdown_text(event: dict) -> str:
    g = float(event.get("g", event.get("selected_g", 0)) or 0)
    h = float(event.get("h", event.get("selected_h", 0)) or 0)
    f = float(event.get("f", event.get("selected_f", g + h)) or 0)
    delta_g = event.get("delta_g_from_parent")
    route_elapsed = float(event.get("g_route_elapsed_s", 0) or 0)
    move_time = float(event.get("g_translation_time_s", 0) or 0)
    rotate_time = float(event.get("g_rotation_time_s", 0) or 0)
    wait_time = float(event.get("g_wait_time_s", 0) or 0)
    distance = float(event.get("g_translation_distance_m", 0) or 0)
    angle = float(event.get("g_rotation_angle_rad", 0) or 0)
    remainder = event.get("g_unexposed_remainder")
    graph_distance = event.get("h_graph_distance_m")
    graph_time = event.get("h_graph_cruise_time_s")
    turn_angle = event.get("h_first_turn_angle_rad")
    turn_time = event.get("h_first_turn_time_s")
    euclidean = event.get("h_euclidean_distance_m")
    euclidean_time = event.get("h_euclidean_cruise_time_s")
    h_gap = event.get("h_rmf_minus_graph_cruise_s")

    lines = [
        f"정확한 RMF 총합: g={g:.3f}, h={h:.3f}, f=g+h={f:.3f}",
    ]
    if delta_g is not None:
        lines.append(
            f"부모→현재 Δg={_fmt(delta_g)}. 부모 구간의 실제 route_from_parent 궤적을 "
            f"분류하면 총 {_fmt(route_elapsed)} s = 이동 {_fmt(move_time)} s + "
            f"회전 {_fmt(rotate_time)} s + 대기 {_fmt(wait_time)} s입니다. "
            f"이동거리 {_fmt(distance)} m, 누적 회전각 {_fmt(angle)} rad입니다."
        )
        lines.append(
            "Δg - 궤적 경과시간 = " + _fmt(remainder) +
            ". 이 차이는 soft lane penalty·event·기타 코어 비용이 섞일 수 있는 "
            "‘미노출 잔차’이며, 기본 Debug API만으로 항목별 확정 분리는 불가능합니다."
        )
    else:
        lines.append("루트 상태이므로 부모 구간 Δg와 이동·회전·대기 구간이 없습니다.")

    lines.append(
        f"h={h:.3f}은 Planner::Debug가 공개한 실제 RMF remaining_cost_estimate입니다. "
        "아래 값은 h 내부 직렬화가 아니라 같은 그래프·속도 제한으로 다시 계산한 검산용 하한입니다."
    )
    if graph_time is not None:
        lines.append(
            f"검산: 방향성 그래프 최단거리 {_fmt(graph_distance)} m / Lane 유효속도 = "
            f"순수 주행 {_fmt(graph_time)} s; 첫 Lane 정렬 회전 {_fmt(turn_angle)} rad → "
            f"정지-정지 최소 회전시간 {_fmt(turn_time)} s; RMF h - 순수 그래프 주행 = {_fmt(h_gap)} s."
        )
    if euclidean is not None:
        lines.append(
            f"더 느슨한 직선 하한: {_fmt(euclidean)} m / 0.7 m/s = "
            f"{_fmt(euclidean_time)} s. 방향성·가속·회전·대기 제약은 포함하지 않습니다."
        )
    return "\n".join(lines)


def event_title(event: dict) -> str:
    name = str(event.get("event", "unknown"))
    return EVENT_KO.get(name, name)


def classify_negotiation_message(message: str) -> tuple[str, str, str]:
    lower = message.lower()
    if "selected table" in lower:
        action = "select_table"
        explanation = "협상기가 다음으로 검토할 참가자·버전 조합의 table을 선택했습니다. 대괄호 값은 협상 table 계층 식별자입니다."
    elif "submitted plan" in lower:
        action = "submit_plan"
        explanation = "한 참가자가 현재 table 제약을 만족한다고 판단한 시간 포함 Plan을 제출했습니다. 아직 전체 Proposal 확정은 아닙니다."
    elif "rejected parent" in lower or "rejected" in lower:
        action = "reject"
        explanation = "하위 제안이 상위 협상 조건과 양립하지 않아 해당 협상 분기를 거부했습니다. 원문만으로 정확한 validator 조건까지는 노출되지 않습니다."
    elif "forfeited" in lower or "forfeit" in lower:
        action = "forfeit"
        explanation = "해당 참가자가 이 table 제약 아래 제출 가능한 Plan을 만들지 못해 그 협상 분기를 포기했습니다. 전체 협상 실패와 같은 뜻은 아닙니다."
    elif "skipping" in lower or "skipped" in lower:
        action = "skip"
        explanation = "이미 처리됐거나 더 이상 유효하지 않은 협상 분기를 건너뛰었습니다."
    elif "resolved" in lower or "finished" in lower:
        action = "resolve"
        explanation = "현재 협상 분기의 검토가 완료됐음을 나타냅니다."
    else:
        action = "other"
        explanation = "RMF CentralizedNegotiation이 남긴 원문입니다. 이 문장만으로 내부 조건을 단정하지 말고 전후 table·proposal 이벤트와 함께 확인해야 합니다."
    return action, NEGOTIATION_ACTION_KO[action], explanation


def explain_negotiation_log(message: str) -> str:
    return classify_negotiation_message(message)[2]


def explain_event(event: dict) -> str:
    kind = str(event.get("event", ""))
    seq = event.get("seq", "-")
    robot = event.get("robot", event.get("name", ""))
    prefix = f"[{seq}] {event_title(event)}"
    if robot:
        prefix += f" · {robot}"

    if kind == "run_started":
        return f"{prefix}\n시나리오 '{event.get('scenario', '')}'를 로봇 {event.get('robot_count', '?')}대로 시작했습니다."
    if kind == "process_phase":
        return f"{prefix}\n처리 단계가 '{_phase(event.get('phase'))}'로 바뀌었습니다. 원문 단계 설명: {event.get('label', '')}"
    if kind == "planner_graph_context":
        return (
            f"{prefix}\n실제 공개 Graph={event.get('graph_object')} · waypoint "
            f"{event.get('waypoint_count')}개 · directed Lane {event.get('directed_lane_count')}개입니다. "
            f"조회 API={event.get('graph_read_api')}.\n"
            "Supergraph는 RMF Planner가 내부 탐색·휴리스틱 구성에 사용하는 private 구현 계층이지만 "
            "현재 public/Planner::Debug API에는 내부 node·key·cache가 노출되지 않습니다. 따라서 Graph 표는 "
            "실제 조회값이고 Supergraph 화면은 노출 범위를 설명하는 것이지 가짜 내부값을 만들지 않습니다."
        )
    if kind == "graph_node":
        return (
            f"{prefix}\n실제 Graph waypoint {event.get('id')}({event.get('name')})입니다. "
            f"map={event.get('map')}, 좌표=({_fmt(event.get('x'))}, {_fmt(event.get('y'))}), "
            f"holding={_fmt(event.get('holding'))}, parking={_fmt(event.get('parking'))}, "
            f"passthrough={_fmt(event.get('passthrough'))}, mutex={event.get('mutex_group') or '없음'}. "
            f"outgoing={event.get('outgoing_lanes')}, incoming={event.get('incoming_lanes')}."
        )
    if kind == "graph_lane":
        return (
            f"{prefix}\n실제 directed Lane {event.get('id')}: {event.get('entry')}→{event.get('exit')}, "
            f"길이={_fmt(event.get('length_m'))} m, speed limit={_fmt(event.get('speed_limit_mps'))}, "
            f"실효속도={_fmt(event.get('effective_speed_mps'))} m/s, mutex={event.get('mutex_group') or '없음'}, "
            f"closed={_fmt(event.get('closed'))}. 역방향은 별도의 Lane ID입니다."
        )
    if kind == "validator_configuration":
        validator = event.get("planner_options_validator")
        return (
            f"{prefix}\nphase={event.get('phase')}, Planner RouteValidator={_fmt(validator)}, "
            f"Schedule 인지={_fmt(event.get('schedule_aware'))}, DB version={_fmt(event.get('schedule_database_version'))}. "
            f"Proposal 이후 안전검사는 {event.get('post_proposal_validator')}입니다. "
            + ("협상 내부 validator 객체와 호출별 reject reason은 public Result가 직접 노출하지 않으므로 raw negotiation log와 proposal 결과를 함께 봅니다."
               if event.get("validator_object_publicly_exposed") is False else
               "free-flow/baseline Planner::Options에는 validator가 nullptr로 입력됐습니다.")
        )
    if kind == "planning_request":
        insertion = float(event.get("insertion_time_s", 0) or 0)
        delayed_note = (
            " 미래 Start 이전의 정지 점유는 이 계획 itinerary에 포함되지 않으므로 "
            "공용 경로 밖 staging 출발점을 권장합니다."
            if float(event.get("start_time_s", 0) or 0) > 0 else ""
        )
        return (
            f"{prefix}\n노드 {event.get('start')}에서 노드 {event.get('goal')}까지, "
            f"요청 출발 시각 {_fmt(event.get('start_time_s', 0))} s, "
            f"Schedule DB 동적 투입 시각 {_fmt(insertion)} s, "
            f"초기 방향 {_fmt(event.get('start_yaw_rad'))} rad로 실제 RMF Planner에 요청했습니다. "
            + ("이 요청은 앞 단계의 itinerary가 commit된 뒤 신규 participant로 등록됐습니다."
               if insertion > 0 else
               "이 요청은 최초 batch에 등록됐으며 실행 중 동적으로 들어온 새 작업은 아닙니다.")
            + delayed_note
        )
    if kind == "dynamic_run_started":
        return (
            f"{prefix}\n정책={event.get('policy')}. 하나의 실제 Schedule Database를 모든 "
            "투입 단계에서 유지하고, 기존 itinerary는 고정한 채 신규 participant만 "
            "CentralizedNegotiation agent로 넣습니다."
        )
    if kind == "dynamic_insertion_stage":
        return (
            f"{prefix}\nstage {event.get('stage')}, t={_fmt(event.get('insertion_time_s'))} s에 "
            f"{event.get('new_robots', [])}를 등록합니다. 이미 commit된 plan은 "
            f"{event.get('existing_committed_count')}개이고 DB version은 "
            f"{event.get('schedule_version_before')}에서 이어집니다."
        )
    if kind == "dynamic_negotiation_request":
        return (
            f"{prefix}\n신규 agent {event.get('agent_count')}대만 협상하고, Schedule DB의 "
            f"기존 itinerary {event.get('fixed_existing_itinerary_count')}개는 고정 제약으로 "
            f"사용합니다. 실제 호출={event.get('api')}."
        )
    if kind == "newcomer_penalty_configuration":
        return (
            f"{prefix}\n정책={event.get('policy')}, 신규={event.get('newcomers', [])}. "
            f"기존 RMF plan이 이미 사용한 corridor/mutex를 찾아 directed Lane별 "
            f"soft g-cost {event.get('directed_lane_penalties', {})}를 신규 batch에만 "
            f"적용했습니다. 값이 커도 폐쇄가 아니라 우회 후보가 없으면 해당 Lane을 쓸 수 있습니다."
        )
    if kind == "dynamic_insertion_result":
        return (
            f"{prefix}\nstage {event.get('stage')} 결과 success={_fmt(event.get('success'))}, "
            f"신규 plan={event.get('new_plan_count', 0)}, penalty Lane={event.get('penalized_lane_count', 0)}. "
            f"기존 itinerary 보존={_fmt(event.get('existing_itineraries_preserved'))}, "
            f"실패 원인={event.get('reason', '')}."
        )
    if kind == "corridor_definition":
        return (
            f"{prefix}\n물리 통로 {event.get('corridor_id')}에 RMF directed lane "
            f"정방향 {event.get('lanes_forward', [])}, 역방향 "
            f"{event.get('lanes_reverse', [])}를 묶었습니다. capacity={event.get('capacity')}, "
            f"passing={_fmt(event.get('passing_allowed'))}, opposite hard block="
            f"{_fmt(event.get('hard_opposite_direction_block'))}. 이 관계는 RMF Graph 원본 "
            "필드가 아니라 실험기의 POLICY_DERIVED traffic resource입니다."
        )
    if kind == "corridor_policy_snapshot":
        return (
            f"{prefix}\nplanning invocation 직전에 Schedule version "
            f"{event.get('schedule_version')}을 실제 {event.get('query_api')}로 한 번 읽고, "
            f"Corridor {event.get('corridor_count')}개에 interval {event.get('interval_count')}개를 "
            f"index화했습니다. 탐색 도중에는 snapshot generation "
            f"{event.get('snapshot_generation')}을 고정해서 A* node마다 DB 전체를 다시 읽지 않습니다."
        )
    if kind == "corridor_schedule_interval":
        return (
            f"{prefix}\nSchedule participant={event.get('participant_id')}, "
            f"plan={event.get('plan_id')}, route={event.get('route_id')}가 Corridor "
            f"{event.get('corridor_id')}를 {event.get('direction')} 방향으로 "
            f"{_fmt(event.get('corridor_enter_s'))}~{_fmt(event.get('corridor_exit_s'))}초에 "
            f"사용합니다. trajectory 자체의 source={event.get('trajectory_source')}, "
            f"Corridor 연결/상태 분석 source={event.get('state_source')}."
        )
    if kind == "corridor_runtime_state":
        return (
            f"{prefix}\nCorridor {event.get('corridor_id')} state={event.get('state')}, "
            f"owner={event.get('owner')}, 방향={event.get('direction')}, "
            f"occupants={event.get('occupants')}, reserved={event.get('reserved_participants')}. "
            f"해제 조건은 '{event.get('release_condition')}'이며 예상 ETA만으로 FREE 처리하지 않습니다."
        )
    if kind == "corridor_state_transition":
        return (
            f"{prefix}\nCorridor {event.get('corridor_id')}가 "
            f"{event.get('from_state')}→{event.get('to_state')}로 전이했습니다. "
            f"시각={_fmt(event.get('at_s'))}초, owner={event.get('owner')}, "
            f"근거={event.get('reason')}."
        )
    if kind == "corridor_policy_expansion":
        overlaps = event.get("overlaps", [])
        return (
            f"{prefix}\nA* candidate {event.get('candidate_id')} (parent "
            f"{event.get('parent_id')})가 waypoint {event.get('current_waypoint')}→"
            f"{event.get('target_waypoint')}, lane={event.get('lane_ids')}로 Corridor "
            f"{event.get('corridor_id')} 진입을 검토했습니다. "
            f"판정={event.get('decision')}, reason={event.get('reason_code')}. "
            f"policy={_fmt(event.get('total_policy_penalty'))}는 g에만 더하고 h="
            f"{_fmt(event.get('h'))}는 바꾸지 않습니다. Schedule overlap {len(overlaps)}건을 "
            f"참조했고 final g/f={_fmt(event.get('final_g'))}/{_fmt(event.get('f'))}입니다."
        )
    if kind in {"runtime_event_definition", "runtime_traffic_event"}:
        return (
            f"{prefix}\nrobot={event.get('robot')}, type={event.get('type')}, "
            f"t={_fmt(event.get('at_s'))}초, value={_fmt(event.get('value_s'))}초. "
            f"Schedule 변경={_fmt(event.get('schedule_changed'))}, 실제/시뮬레이션 API="
            f"{event.get('schedule_api', 'definition only')}, replan 근거="
            f"{event.get('replan_trigger', event.get('detail', ''))}."
        )
    if kind == "replan_trigger":
        return (
            f"{prefix}\n주기적 재계획이 아니라 {event.get('reason')} 때문에 명시적으로 "
            f"다음 planning invocation을 시작합니다. Schedule 변경="
            f"{_fmt(event.get('schedule_changed'))}; 동작={event.get('action')}."
        )
    if kind == "route_validator_result":
        return (
            f"{prefix}\n실제 {event.get('validator')} 호출 결과 "
            f"decision={event.get('decision')}, reason={event.get('reason_code')}. "
            f"blocker participant/plan/route={event.get('blocker_participant')}/"
            f"{event.get('blocker_plan_id')}/{event.get('blocker_route_id')}, "
            f"conflict t={_fmt(event.get('conflict_time_s'))}. Corridor Admission 판정과 "
            "RMF 시공간 profile 충돌 판정을 같은 BLOCKED로 합치지 않습니다."
        )
    if kind in {"astar_step_decision", "astar_expand"}:
        g = float(event.get("g", event.get("selected_g", 0)) or 0)
        h = float(event.get("h", event.get("selected_h", 0)) or 0)
        f = float(event.get("f", event.get("selected_f", g + h)) or 0)
        next_f = event.get("next_best_f")
        margin = event.get("f_margin_to_next")
        reason = (
            f"frontier 우선순위 큐의 top이어서 선택됐습니다. 기록값은 g={g:.3f}, "
            f"h={h:.3f}, f=g+h={f:.3f}입니다."
        )
        if next_f is not None:
            reason += f" 선택 당시 차순위 f={float(next_f):.3f}, 차이={float(margin or 0):.3f}입니다."
            if abs(float(margin or 0)) < 1e-9:
                reason += " f가 동률이므로 최종 순서는 RMF 내부 비교자의 추가 기준에 의해 결정되며 현재 Debug API에는 그 세부 기준이 노출되지 않습니다."
        return f"{prefix}\n{reason}\n\n{_astar_breakdown_text(event)}"
    if kind == "astar_generated":
        return (
            f"{prefix}\n부모 노드 {event.get('parent_id')}를 확장하면서 후보 노드 {event.get('node_id')}가 frontier에 들어왔습니다. "
            f"g={_fmt(event.get('g'))}, h={_fmt(event.get('h'))}, f={_fmt(event.get('f'))}, "
            f"부모 대비 Δg={_fmt(event.get('delta_g_from_parent'))}, Δh={_fmt(event.get('delta_h_from_parent'))}, "
            f"Δf={_fmt(event.get('delta_f_from_parent'))}입니다. 생성되지 않은 분기의 정확한 탈락 사유는 기본 Debug API에 없습니다.\n\n"
            + _astar_breakdown_text(event)
        )
    if kind == "astar_step_summary":
        found = "이 단계에서 해를 찾았습니다." if event.get("solution_found") else "아직 해가 확정되지 않았습니다."
        return (
            f"{prefix}\n노드 {event.get('expanded_node_id')} 확장 후 자식 {event.get('generated_children')}개가 관찰됐고, "
            f"frontier에는 {event.get('frontier_size_after')}개가 남았습니다. {found}"
        )
    if kind == "astar_frontier_best":
        return (
            f"{prefix}\n현재 단계 종료 후 다음 확장 우선 후보입니다. "
            f"f={_fmt(event.get('f'))}=g {_fmt(event.get('g'))}+h {_fmt(event.get('h'))}.\n\n"
            + _astar_breakdown_text(event)
        )
    if kind == "astar_trace_summary":
        return (
            f"{prefix}\n실제 Planner::Debug 탐색에서 {event.get('expansions')}번 확장했고 "
            f"고유 노드 {event.get('unique_nodes_observed')}개를 관찰했습니다. 해 발견={_fmt(event.get('solution_found'))}, "
            f"step limit 도달={_fmt(event.get('step_limit_reached'))}."
        )
    if kind == "route_candidate":
        selected = "최종 계획에 사용됨" if event.get("selected_by_plan") else "선택되지 않음"
        return (
            f"{prefix}\n후보 {event.get('rank')}위, Lane {event.get('lanes')}, 거리 {_fmt(event.get('distance_m'))} m, "
            f"RMF 비용 {_fmt(event.get('rmf_cost'))}, 최저비용 대비 차이 {_fmt(event.get('delta_from_best'))}: {selected}. "
            "이 표는 각 단순 경로 외 Lane을 닫고 실제 Planner로 재계산한 분석용 비교이며 A* 내부 분기 자체는 아닙니다."
        )
    if kind == "route_choice_explanation":
        return (
            f"{prefix}\nA*가 반환한 경로는 관찰된 최적해입니다. 선택 후보 순위={_fmt(event.get('selected_rank'))}, "
            f"선택 비용={_fmt(event.get('selected_cost'))}, 다음 후보 비용={_fmt(event.get('next_best_cost'))}, "
            f"비용 차이={_fmt(event.get('cost_margin'))}."
        )
    if kind == "occupancy_penalty_configuration":
        return (
            f"{prefix}\n원본 RMF가 먼저 만든 로봇별 free-flow Lane "
            f"{event.get('baseline_lanes_by_robot', {})}을 비교했습니다. 같은 물리 통로를 "
            f"2대 이상이 사용할 때 사용자={event.get('shared_corridor_users', {})}, "
            f"directed lane 수요={event.get('directed_lane_occupancy', {})}로 계산했고, "
            f"수정 Planner의 실제 g에 더할 값은 {event.get('directed_lane_penalties', {})}입니다. "
            f"계산식={event.get('algorithm')}, 활성={_fmt(event.get('active'))}."
        )
    if kind == "plan_waypoint":
        recorded_reason = str(event.get("movement_reason", "")).strip()
        raw_note = f"\n기록된 원문 근거: {recorded_reason}" if recorded_reason else ""
        return (
            f"{prefix}\n계획 순번 {event.get('sequence')} · {_movement(event.get('movement_type'))}: "
            f"t={_fmt(event.get('time_s'))} s에 "
            f"({_fmt(event.get('x'))}, {_fmt(event.get('y'))}), 방향 {_fmt(event.get('yaw_rad'))} rad를 지나며 "
            f"접근 Lane은 {event.get('approach_lanes', [])}입니다. 직전 단계 대비 "
            f"Δt={_fmt(event.get('delta_time_s'))} s, Δ거리={_fmt(event.get('delta_distance_m'))} m, "
            f"Δyaw={_fmt(event.get('delta_yaw_rad'))} rad입니다.\n"
            f"선택 근거: {_movement_reason(event)}{raw_note}"
        )
    if kind == "plan_summary":
        if not event.get("success"):
            return f"{prefix}\nPlanner가 경로를 반환하지 못했습니다. disconnected={event.get('disconnected')}, saturated={event.get('saturated')}, interrupted={event.get('interrupted')}."
        return (
            f"{prefix}\n{_phase(event.get('phase'))} 결과 비용={_fmt(event.get('cost'))}, "
            f"종료시간={_fmt(event.get('finish_time_s'))} s, 사용 Lane={event.get('used_lanes', [])}, "
            f"waypoint={event.get('plan_waypoint_count')}, trajectory point={event.get('trajectory_point_count')}."
        )
    if kind == "itinerary_summary":
        return (
            f"{prefix}\n실제 {event.get('source_api')}가 반환한 {event.get('object_type')}입니다. "
            f"Route {event.get('route_count')}개이며 현재 phase={event.get('phase')}. "
            "이 단계의 Itinerary는 Plan 또는 Proposal 안의 값이고, Schedule DB에 실제 commit됐는지는 "
            "proposal_outcome과 schedule_database_operation을 따로 확인해야 합니다."
        )
    if kind == "route_summary":
        return (
            f"{prefix}\nItinerary[{event.get('route_index')}]의 실제 {event.get('object_type')}입니다. "
            f"map={event.get('map')}, Trajectory point={event.get('trajectory_point_count')}, "
            f"시간={_fmt(event.get('start_time_s'))}→{_fmt(event.get('finish_time_s'))} s, "
            f"duration={_fmt(event.get('duration_s'))} s. 조회={event.get('source_api')}."
        )
    if kind == "trajectory_point":
        return (
            f"{prefix}\nRoute[{event.get('route_index')}] Trajectory point {event.get('sequence')}입니다. "
            f"t={_fmt(event.get('time_s'))} s, pose=({_fmt(event.get('x'))}, {_fmt(event.get('y'))}, "
            f"yaw {_fmt(event.get('yaw_rad'))}), velocity=({_fmt(event.get('vx'))}, "
            f"{_fmt(event.get('vy'))}, yaw-rate {_fmt(event.get('vyaw'))}). "
            f"실제 객체={event.get('object_type', 'rmf_traffic::Trajectory::Waypoint')}, "
            f"조회={event.get('source_api', 'Route::trajectory()')}"
        )
    if kind == "negotiation_request":
        robots = event.get("robots", [])
        requests = ", ".join(
            f"{r.get('name')}({r.get('start')}→{r.get('goal')}, t={r.get('start_time_s', 0)}s)"
            for r in robots)
        penalty_note = ""
        if event.get("experimental_lane_penalty_active"):
            penalty_note = (
                " AFTER penalty 실험으로 긴 우회안도 버리지 않도록 "
                f"maximum_cost_leeway={_fmt(event.get('maximum_cost_leeway'))}, "
                f"minimum_cost_threshold={_fmt(event.get('minimum_cost_threshold'))}를 적용했습니다."
            )
        return f"{prefix}\n{len(robots)}대의 자유 경로를 동시에 만족하도록 시공간 충돌 회피 협상을 요청했습니다: {requests}.{penalty_note}"
    if kind == "negotiation_log":
        message = str(event.get("message", ""))
        action, action_ko, explanation = classify_negotiation_message(message)
        return (
            f"{prefix}\n분류={action_ko}({action}) · stage={_fmt(event.get('stage'))}. "
            f"{explanation}\n원문: {message}\n"
            "주의: 분류명은 UI 가독성을 위한 문자열 분류이고, 원문 자체가 실제 "
            "CentralizedNegotiation::Result::log 출력입니다."
        )
    if kind == "proposal_summary":
        return (
            f"{prefix}\n실제 {event.get('source_api')} 조회 결과 proposal 존재={_fmt(event.get('present'))}, "
            f"participant Plan={event.get('participant_plan_count')}, stage={_fmt(event.get('stage'))}, "
            f"commit 상태={event.get('commit_state')}. Proposal 생성은 아직 실행 승인이나 DB 저장을 뜻하지 않습니다."
        )
    if kind == "proposal_plan":
        return (
            f"{prefix}\nparticipant={event.get('participant_id')}의 실제 Proposal Plan입니다. "
            f"cost={_fmt(event.get('cost'))}, waypoint={event.get('waypoint_count')}, "
            f"Itinerary Route={event.get('itinerary_route_count')}, Trajectory point={event.get('trajectory_point_count')}, "
            f"finish={_fmt(event.get('finish_time_s'))} s. 이 시점에는 validated={_fmt(event.get('validated'))}, "
            f"committed={_fmt(event.get('committed'))}입니다."
        )
    if kind == "proposal_outcome":
        action = event.get("action")
        meanings = {
            "reject_no_proposal": "모든 협상 결과를 조합한 Proposal이 없어 DB commit을 수행하지 않았습니다.",
            "reject_after_detect_conflict": "Proposal은 있었지만 별도 연속시간 충돌검증을 통과하지 못해 거부했습니다.",
            "accept_and_commit": "Proposal이 충돌검증을 통과해 Participant::set으로 Schedule DB에 반영됐습니다.",
        }
        return (
            f"{prefix}\naction={action}, accepted={_fmt(event.get('accepted'))}, "
            f"committed={_fmt(event.get('committed'))}. {meanings.get(str(action), '')}\n"
            f"기록된 이유: {event.get('reason', '')}"
        )
    if kind == "negotiation_summary":
        success = bool(event.get("success")) and bool(event.get("executable_plan", True))
        return (
            f"{prefix}\n실행 가능한 협상 결과={'성공' if success else '실패'}, "
            f"계산시간={_fmt(event.get('elapsed_ms'))} ms, 계획 수={_fmt(event.get('proposal_plan_count'))}, "
            f"안전검증={_fmt(event.get('safety_verified'))}. {event.get('interpretation', '')}"
        )
    if kind in {"safety_verification", "safety_pair_check", "pairwise_conflict_check"}:
        pair = ""
        if event.get("robot_a") or event.get("robot_b"):
            pair = f" 대상={event.get('robot_a')}↔{event.get('robot_b')},"
        return (
            f"{prefix}\nRMF DetectConflict 연속시간 검사 결과{pair} "
            f"passed={_fmt(event.get('passed'))}, conflicts={_fmt(event.get('conflicts', event.get('conflict')))}, "
            f"검사 Route 쌍={_fmt(event.get('route_pair_checks'))}. "
            f"{event.get('reason', event.get('interpretation', ''))}"
        )
    if kind == "schedule_database_operation":
        action = str(event.get("action", ""))
        return (
            f"{prefix}\n{SCHEDULE_ACTION_KO.get(action, action)}. API={event.get('api')}, "
            f"DB version {event.get('version_before')}→{event.get('version_after')}, "
            f"participant={_fmt(event.get('participant_id'))}, 결과={event.get('result', '')}"
        )
    if kind == "schedule_model_schema":
        return (
            f"{prefix}\n실제 객체 계층={event.get('hierarchy')}. 쓰기={event.get('write_path')}, "
            f"읽기={event.get('read_path')}. JSONL은 별도 목업 DB가 아니라 실제 in-memory "
            "객체를 관찰해 평탄화한 표현이며 RMF의 바이너리 직렬화 형식은 아닙니다."
        )
    if kind == "schedule_database_state":
        return (
            f"{prefix}\n'{_phase(event.get('phase'))}' 시점의 실제 in-memory Database 조회 결과입니다. "
            f"DB version={event.get('latest_version')}, 참가자={event.get('participant_count')}, "
            f"저장 Route={event.get('stored_route_count')}. 실제 class={event.get('database_class', 'rmf_traffic::schedule::Database')}, "
            f"조회 View={event.get('view_class', 'rmf_traffic::schedule::Viewer::View')}. "
            "그래프 자체는 이 DB에 저장되지 않고 시간 포함 itinerary만 저장됩니다."
        )
    if kind == "schedule_participant":
        return (
            f"{prefix}\nparticipant ID={event.get('participant_id')}, plan ID={event.get('current_plan_id')}, "
            f"itinerary version={event.get('itinerary_version')}, route={event.get('route_count')}, "
            f"trajectory point={event.get('trajectory_point_count')}, responsive={_fmt(event.get('responsive'))}, "
            f"profile={event.get('profile_footprint', 'Circle')} r={_fmt(event.get('profile_radius_m', 0.3))} m. "
            f"실제 읽기={event.get('itinerary_read_api', event.get('read_from', 'Database::get_itinerary'))}."
        )
    if kind in {"schedule_database_route", "schedule_itinerary_route"}:
        return (
            f"{prefix}\nDB query_all()이 반환한 Route입니다. participant={event.get('participant_id')}, "
            f"plan={event.get('plan_id')}, route={event.get('route_id')}, 시간 {_fmt(event.get('start_time_s'))}→{_fmt(event.get('finish_time_s'))} s, "
            f"duration={_fmt(event.get('duration_s'))} s, point={event.get('trajectory_point_count')}."
        )
    if kind in {"schedule_database_trajectory_point", "schedule_trajectory_point"}:
        return (
            f"{prefix}\nDB에 저장된 시간 파라미터 궤적점입니다. t={_fmt(event.get('time_s'))} s, "
            f"pose=({_fmt(event.get('x'))}, {_fmt(event.get('y'))}, yaw {_fmt(event.get('yaw_rad'))}), "
            f"velocity=({_fmt(event.get('vx'))}, {_fmt(event.get('vy'))}, yaw-rate {_fmt(event.get('vyaw'))})."
        )
    if kind == "schedule_commit":
        return f"{prefix}\n안전검증을 통과한 plan {event.get('plan_id')}을 participant {event.get('participant_id')}의 itinerary로 저장했습니다. accepted={_fmt(event.get('accepted'))}."
    if kind == "solution_diagnosis":
        return diagnosis_text(event)
    if kind == "runner_core_profile":
        penalty = "사용 안 함"
        if event.get("lane_penalty_active"):
            penalty = (
                f"{event.get('lane_penalty_mode')} 모드, "
                f"directed lane {event.get('penalized_lane_count')}개, "
                f"기본 penalty={_fmt(event.get('lane_penalty_value'))}"
            )
        return (
            f"{prefix}\n코어 label={event.get('label')}, 실제 로딩 라이브러리={event.get('resolved_rmf_library')}, "
            f"commit={event.get('rmf_source_commit')}, 소스 수정={_fmt(event.get('rmf_source_dirty'))}.\n"
            f"AFTER Lane penalty: {penalty}. 적용값={event.get('directed_lane_penalties', {})}.\n"
            f"예상 점유량={event.get('directed_lane_occupancy', {})}, "
            f"공유 통로 로봇={event.get('shared_corridor_users', {})}."
        )
    return f"{prefix}\n{json.dumps({k: v for k, v in event.items() if k != 'seq'}, ensure_ascii=False)}"


def diagnosis_text(event: dict) -> str:
    if not event:
        return "아직 solution_diagnosis 이벤트가 없습니다. 실행 중이거나 Planner 결과가 기록되기 전입니다."
    category = str(event.get("category", "unknown"))
    title, meaning = DIAGNOSIS_KO.get(category, (category, str(event.get("root_cause", ""))))
    status = "해결됨" if event.get("status") == "solved" else "실행 가능한 해 없음"
    confidence = {
        "high": "높음", "medium_high": "중간 이상", "medium": "중간", "low": "낮음",
    }.get(str(event.get("confidence", "")), str(event.get("confidence", "")))
    lines = [
        f"판정: {status}",
        f"분류: {title} ({category})",
        f"진단 확신도: {confidence}",
        "",
        "쉽게 말하면",
        meaning,
        "",
        "RMF 원본 근거",
        f"• 판단 기반: {event.get('basis', '')}",
        f"• 원문 원인: {event.get('root_cause', '')}",
    ]
    evidence = event.get("evidence", [])
    if evidence:
        lines.extend(["", "관찰된 증거"])
        lines.extend(f"• {_evidence_line(item)}" for item in evidence)
    actions = event.get("recommended_actions", [])
    if actions:
        lines.extend(["", "어떻게 풀어볼까"])
        lines.extend(f"{index}. {ACTION_KO.get(str(item), str(item))}" for index, item in enumerate(actions, 1))
    lines.extend([
        "",
        "해석 주의",
        "이 진단은 RMF가 반환한 성공·실패 상태와 그래프 구조를 결합한 실험용 설명입니다. "
        "특히 negotiation_no_proposal은 협상 로그만으로 단 하나의 내부 원인을 확정한 것이 아닙니다.",
    ])
    return "\n".join(lines)



FAILURE_CATEGORY_MAP = {
    "individual_path_missing": "PLANNER_NO_SOLUTION",
    "disconnected_topology": "PLANNER_NO_SOLUTION",
    "planner_no_solution": "PLANNER_NO_SOLUTION",
    "search_saturation": "SEARCH_LIMIT_REACHED",
    "continuous_time_overlap": "SCHEDULE_CONFLICT",
    "dynamic_combined_plan_conflict": "SCHEDULE_CONFLICT",
    "negotiation_no_proposal": "NO_NEGOTIATION_ALTERNATIVE",
    "dynamic_newcomer_no_proposal": "NO_NEGOTIATION_ALTERNATIVE",
    "endpoint_exchange_without_buffer": "NO_PHYSICAL_ESCAPE",
    "single_route_no_yield_space": "NO_PHYSICAL_ESCAPE",
}


def _failure_location(event: dict) -> str:
    """Return only location facts that are explicitly present in the raw event."""
    if event.get("corridor_id") not in (None, ""):
        return f"corridor={event.get('corridor_id')}"
    if event.get("lane_id") not in (None, ""):
        return f"lane={event.get('lane_id')}"
    if event.get("lane_ids") not in (None, "", []):
        return f"lanes={event.get('lane_ids')}"
    if event.get("candidate_route_id") not in (None, ""):
        return f"route={event.get('candidate_route_id')}"
    if event.get("blocker_route_id") not in (None, ""):
        return f"blocker_route={event.get('blocker_route_id')}"
    return "UNKNOWN"


def _failure_time(event: dict) -> str:
    for key in (
        "earliest_conflict_time_s", "conflict_time_s", "predicted_enter_time",
        "time_s", "start_time_s",
    ):
        if event.get(key) is not None:
            return f"{_fmt(event.get(key))} s"
    return "UNKNOWN"


def failure_trace_records(events: Iterable[dict]) -> list[dict]:
    """Build a compact causal timeline from recorded RMF events only.

    This function does not fabricate missing RMF internals. If a location or cause
    is not present in the JSONL/public debug surface, it stays UNKNOWN.
    """
    rows: list[dict] = []
    for event in events:
        kind = str(event.get("event", ""))
        stage = ""
        status = ""
        actor = str(event.get("robot", ""))
        detail = ""

        if kind == "planning_request":
            stage, status = "PLANNER", "REQUEST"
            detail = f"start={event.get('start')} goal={event.get('goal')}"
        elif kind == "plan_summary":
            stage = "PLANNER"
            status = "SUCCESS" if event.get("success") else "FAIL"
            detail = (
                f"phase={event.get('phase')} cost={event.get('cost')} "
                f"lanes={event.get('used_lanes', [])} disconnected={event.get('disconnected')} "
                f"saturated={event.get('saturated')} interrupted={event.get('interrupted')}"
            )
        elif kind == "astar_trace_summary" and (
            not event.get("solution_found", True) or event.get("step_limit_reached")
        ):
            stage = "PLANNER"
            status = "SEARCH_LIMIT" if event.get("step_limit_reached") else "NO_SOLUTION"
            detail = (
                f"expansions={event.get('expansions')} solution_found={event.get('solution_found')} "
                f"step_limit_reached={event.get('step_limit_reached')}"
            )
        elif kind == "schedule_database_operation":
            action = str(event.get("action", ""))
            if action in {
                "register_participant", "read_for_negotiation", "set_itinerary",
                "set_dynamic_newcomer_itinerary", "skip_commit", "reject_commit",
            }:
                stage, status = "SCHEDULE", action.upper()
                actor = str(event.get("robot", event.get("participant_id", "")))
                detail = str(event.get("result", event.get("api", "")))
        elif kind == "route_validator_result" and event.get("decision") != "ACCEPT":
            stage, status = "CONFLICT/VALIDATOR", "REJECT"
            actor = str(event.get("participant_id", event.get("robot", "")))
            detail = (
                f"reason={event.get('reason', '')} blocker={event.get('blocker_participant', '')} "
                f"plan={event.get('blocker_plan_id', '')} route={event.get('blocker_route_id', '')}"
            )
        elif kind == "pairwise_conflict_check" and not event.get("passed", True):
            stage, status = "CONFLICT", "DETECTED"
            actor = f"{event.get('robot_a', '?')} ↔ {event.get('robot_b', '?')}"
            detail = f"route_pair_checks={event.get('route_pair_checks')}"
        elif kind == "safety_verification" and not event.get("passed", True):
            stage, status = "CONFLICT", "FAILED_SAFETY"
            detail = (
                f"conflicts={event.get('conflicts', '')} reason={event.get('reason', '')} "
                f"method={event.get('method', '')}"
            )
        elif kind in {"negotiation_request", "dynamic_negotiation_request"}:
            stage, status = "NEGOTIATION", "START"
            detail = f"participants={event.get('participants', event.get('robot_count', ''))}"
        elif kind == "negotiation_log":
            action, _label, _description = classify_negotiation_message(
                str(event.get("message", "")))
            if action in {"select_table", "submit_plan", "reject", "forfeit", "resolve"}:
                stage, status = "NEGOTIATION", action.upper()
                detail = str(event.get("message", ""))
        elif kind == "proposal_summary":
            stage = "NEGOTIATION"
            status = "PROPOSAL" if event.get("present") else "NO_PROPOSAL"
            detail = f"participant_plan_count={event.get('participant_plan_count', 0)}"
        elif kind == "proposal_outcome":
            stage, status = "NEGOTIATION", str(event.get("action", "OUTCOME")).upper()
            detail = str(event.get("reason", ""))
        elif kind == "negotiation_summary":
            stage = "NEGOTIATION"
            status = "SUCCESS" if event.get("success") and event.get("executable_plan", True) else "FAIL"
            detail = str(event.get("interpretation", ""))
        elif kind == "solution_diagnosis":
            stage = "FINAL"
            status = "SUCCESS" if event.get("status") == "solved" else "FAILED"
            detail = f"category={event.get('category')} root_cause={event.get('root_cause', '')}"

        if not stage:
            continue
        rows.append({
            "seq": event.get("seq", ""),
            "stage": stage,
            "actor": actor,
            "event": kind,
            "status": status,
            "location": _failure_location(event),
            "time": _failure_time(event),
            "detail": detail,
            "source": event.get(
                "source_api", event.get("method", event.get("api", event.get("basis", "")))),
            "event_raw": event,
        })
    return rows


def failure_analysis(events: Iterable[dict]) -> dict:
    """Classify the final blocking reason using recorded evidence, or UNKNOWN."""
    events = list(events)
    diagnoses = [e for e in events if e.get("event") == "solution_diagnosis"]
    diagnosis = diagnoses[-1] if diagnoses else {}
    category = str(diagnosis.get("category", ""))
    status = str(diagnosis.get("status", "unknown"))

    if status == "solved":
        primary = "SUCCESS"
    else:
        primary = FAILURE_CATEGORY_MAP.get(category, "")
        if not primary:
            if any(
                e.get("event") == "route_validator_result"
                and e.get("decision") != "ACCEPT" for e in events
            ):
                primary = "VALIDATOR_REJECT"
            elif any(
                e.get("event") == "negotiation_summary"
                and not e.get("success", False) for e in events
            ):
                primary = "NEGOTIATION_FAILED"
            elif any(
                e.get("event") == "pairwise_conflict_check"
                and not e.get("passed", True) for e in events
            ):
                primary = "SCHEDULE_CONFLICT"
            elif any(
                e.get("event") == "plan_summary"
                and not e.get("success", False) for e in events
            ):
                primary = "PLANNER_NO_SOLUTION"
            else:
                primary = "UNKNOWN"

    conflict = next((
        e for e in events
        if e.get("event") == "pairwise_conflict_check" and not e.get("passed", True)
    ), {})
    validator_reject = next((
        e for e in events
        if e.get("event") == "route_validator_result" and e.get("decision") != "ACCEPT"
    ), {})

    secondary: list[str] = []
    if primary != "SEARCH_LIMIT_REACHED" and any(
        e.get("event") == "astar_trace_summary" and e.get("step_limit_reached")
        for e in events
    ):
        secondary.append("SEARCH_LIMIT_REACHED")
    if primary != "VALIDATOR_REJECT" and validator_reject:
        secondary.append("VALIDATOR_REJECT")
    if primary != "SCHEDULE_CONFLICT" and conflict:
        secondary.append("SCHEDULE_CONFLICT")

    feasible_candidates = sum(
        e.get("event") == "route_candidate" and e.get("feasible")
        for e in events
    )
    proposal_summary = next((
        e for e in reversed(events) if e.get("event") == "proposal_summary"
    ), {})

    if primary == "PLANNER_NO_SOLUTION":
        improvement = "그래프 연결성·방향 Lane·폐쇄 Lane과 A* 종료 원인을 먼저 확인하세요."
    elif primary == "SEARCH_LIMIT_REACHED":
        improvement = "반복/불필요 상태를 확인한 뒤 saturation 한도나 탐색 정책을 조정하세요."
    elif primary == "VALIDATOR_REJECT":
        improvement = "Validator blocker·충돌 시각을 확인하고 해당 후보의 시간/경로 제약을 수정하세요."
    elif primary == "SCHEDULE_CONFLICT":
        improvement = "실제 충돌 시간대를 기준으로 holding·시간 분리·corridor admission/reservation을 검토하세요."
    elif primary == "NO_NEGOTIATION_ALTERNATIVE":
        if feasible_candidates > 1:
            improvement = "개별 우회 후보는 있으므로 Negotiation에서 alternative가 생성·선택되지 않는 이유와 비용/제약을 확인하세요."
        else:
            improvement = "협상 전에 실제 우회·대피 topology 또는 holding 공간이 있는지 먼저 확인하세요."
    elif primary == "NO_PHYSICAL_ESCAPE":
        improvement = "알고리즘보다 passing bay·holding point·corridor reservation 같은 물리/운영 자원 보강을 우선 검토하세요."
    elif primary == "NEGOTIATION_FAILED":
        improvement = "원본 negotiation reject/forfeit 로그와 proposal 유무를 확인해 실패 분기를 좁히세요."
    elif primary == "SUCCESS":
        improvement = "실행 가능한 계획입니다. 동일 조건의 Baseline/Modified 비교 근거로 사용하세요."
    else:
        improvement = "공개/기록된 이벤트만으로 원인을 확정할 수 없습니다. UNKNOWN을 유지하고 원본 JSONL을 추가 계측하세요."

    conflict_pair = "UNKNOWN"
    if conflict:
        conflict_pair = f"{conflict.get('robot_a', '?')} ↔ {conflict.get('robot_b', '?')}"
    conflict_location = _failure_location(validator_reject or conflict)
    conflict_time = _failure_time(conflict or validator_reject)

    return {
        "result": "SUCCESS" if status == "solved" else "FAILED",
        "primary_cause": primary,
        "secondary_causes": secondary,
        "diagnosis_category": category or "UNKNOWN",
        "diagnosis_confidence": diagnosis.get("confidence", "UNKNOWN"),
        "diagnosis_basis": diagnosis.get("basis", "UNKNOWN"),
        "root_cause": diagnosis.get("root_cause", "UNKNOWN"),
        "conflict_pair": conflict_pair,
        "conflict_location": conflict_location,
        "conflict_time": conflict_time,
        "feasible_route_candidates": feasible_candidates,
        "proposal_present": proposal_summary.get("present", "UNKNOWN"),
        "proposal_plan_count": proposal_summary.get("participant_plan_count", "UNKNOWN"),
        "negotiation_internal_alternative_count": "UNKNOWN (CentralizedNegotiation public Result does not expose every internal alternative)",
        "improvement": improvement,
    }


def failure_summary_text(events: Iterable[dict]) -> str:
    analysis = failure_analysis(events)
    rows = failure_trace_records(events)
    stage_status: dict[str, str] = {}
    for row in rows:
        stage_status[row["stage"]] = row["status"]
    lines = [
        f"RESULT: {analysis['result']}",
        "",
        f"Primary Cause: {analysis['primary_cause']}",
        f"Secondary Cause: {', '.join(analysis['secondary_causes']) if analysis['secondary_causes'] else '없음/확인 안 됨'}",
        f"Diagnosis: {analysis['diagnosis_category']} (confidence={analysis['diagnosis_confidence']})",
        f"Basis: {analysis['diagnosis_basis']}",
        "",
        "실제 처리 흐름",
        f"1. Planner: {stage_status.get('PLANNER', '기록 없음')}",
        f"2. Schedule: {stage_status.get('SCHEDULE', '기록 없음')}",
        f"3. Conflict/Validator: {stage_status.get('CONFLICT/VALIDATOR', stage_status.get('CONFLICT', '검출 기록 없음'))}",
        f"4. Negotiation: {stage_status.get('NEGOTIATION', '기록 없음')}",
        f"5. Final: {stage_status.get('FINAL', '기록 없음')}",
        "",
        f"Conflict: {analysis['conflict_pair']}",
        f"Location: {analysis['conflict_location']}",
        f"Conflict time: {analysis['conflict_time']}",
        f"Feasible route candidates (lab forced-path diagnostic): {analysis['feasible_route_candidates']}",
        f"Negotiation proposal present/count: {analysis['proposal_present']} / {analysis['proposal_plan_count']}",
        f"Negotiation internal alternative count: {analysis['negotiation_internal_alternative_count']}",
        "",
        "Diagnosis",
        str(analysis["root_cause"]),
        "",
        "개선 방향",
        str(analysis["improvement"]),
        "",
        "주의: 위 요약은 JSONL에 기록된 실제 RMF/실험 이벤트만 집계합니다. 공개 API에 없는 내부 원인·위치·alternative 수는 추정하지 않고 UNKNOWN으로 표시합니다.",
    ]
    return "\n".join(lines)

def schedule_guide_text(events: Iterable[dict] = ()) -> str:
    events = list(events)
    states = [e for e in events if e.get("event") == "schedule_database_state"]
    last = states[-1] if states else {}
    return "\n".join([
        "Schedule Database 해석 가이드",
        "",
        "• DB version: 참가자 등록이나 itinerary 변경 때 증가하는 전체 DB 변경 번호입니다.",
        "• participant ID: DB가 로봇 참가자를 구분하는 식별자입니다. 로봇 이름과 별개입니다.",
        "• plan ID: 해당 참가자가 새 계획을 구분하기 위해 발급한 번호입니다.",
        "• itinerary version: 참가자의 itinerary가 set/extend/delay/erase될 때 변하는 버전입니다.",
        "• progress version: reached 등 실행 진행도 갱신 버전입니다. 이 코어 실험은 실제 하드웨어 reached를 보내지 않습니다.",
        "• Route: 한 map에서 이어지는 시간 포함 Trajectory 묶음입니다.",
        "• Trajectory point: 특정 시각의 x, y, yaw와 vx, vy, yaw-rate입니다.",
        "• 실제 조회 계층: Database::query(query_all) → Viewer::View::Element → Route → Trajectory → Waypoint입니다.",
        "• UI 표/JSONL: 위 실제 객체를 행 형태로 평탄화한 관찰값입니다. 별도 목업 DB가 아니지만 RMF 내부 메모리 덤프나 공식 직렬화 포맷도 아닙니다.",
        "• corridor enter/exit: 실제 Route::trajectory timestamp를 graph lane 끝점과 연결한 POLICY_DERIVED 분석입니다. Route/Trajectory 자체는 SCHEDULE이며 corridor_id는 RMF 표준 Schedule 필드가 아닙니다.",
        "• cumulative delay: Participant::delay(Duration)가 실제 성공했을 때만 SCHEDULE 변경으로 표시합니다. reached checkpoint는 Fleet Adapter 피드백이 없어 simulator event로 명시한 경우만 표시합니다.",
        "• 미표시 내부값: storage index, patch cull history, dependency graph 내부 구조, 이 실험에서 발생하지 않은 inconsistency range입니다.",
        "• commit_1_of_N: 첫 번째 로봇 계획을 set한 직후 DB를 다시 읽은 스냅샷입니다.",
        "• proposal_committed: 모든 검증된 로봇 itinerary가 저장된 최종 스냅샷입니다.",
        "",
        f"현재 마지막 스냅샷: phase={_phase(last.get('phase'))}, DB version={_fmt(last.get('latest_version'))}, "
        f"participants={_fmt(last.get('participant_count'))}, routes={_fmt(last.get('stored_route_count'))}",
        "",
        "표의 행을 선택하면 아래에 해당 행의 구체적인 의미가 표시됩니다. Ctrl+C는 선택 셀, Ctrl+Shift+C는 전체 표를 TSV로 복사합니다.",
    ])


def schedule_model_text(events: Iterable[dict] = ()) -> str:
    events = list(events)
    schema = next(
        (event for event in events if event.get("event") == "schedule_model_schema"),
        {},
    )
    states = [
        event for event in events
        if event.get("event") == "schedule_database_state"
    ]
    participants = [
        event for event in events
        if event.get("event") == "schedule_participant"
    ]
    routes = [
        event for event in events
        if event.get("event") == "schedule_database_route"
    ]
    points = [
        event for event in events
        if event.get("event") == "schedule_database_trajectory_point"
    ]
    latest = states[-1] if states else {}
    return "\n".join([
        "실제 RMF Schedule 객체 구조",
        "",
        "rmf_traffic::schedule::Database",
        "├─ ParticipantDescription: name, owner, responsiveness, Profile",
        "├─ participant별 plan ID / itinerary version / progress version",
        "└─ Itinerary (vector<Route>)",
        "   └─ Route: map + Trajectory",
        "      └─ Trajectory::Waypoint: time + pose(x,y,yaw) + velocity(vx,vy,vyaw)",
        "",
        "실제 쓰기 호출",
        schema.get("write_path", "Database::register_participant → Participant::set(plan_id, itinerary)"),
        "",
        "실제 읽기 호출",
        schema.get(
            "read_path",
            "Database::query(query_all) → Viewer::View::Element → Route → Trajectory → Waypoint",
        ),
        "Database::get_participant / get_itinerary / get_current_plan_id / "
        "itinerary_version / get_current_progress_version",
        "",
        "현재 화면에 관찰된 실제 객체",
        f"DB phase={latest.get('phase', '실행 전')}, version={_fmt(latest.get('latest_version'))}",
        f"participant snapshot 행={len(participants)}, route 행={len(routes)}, trajectory point 행={len(points)}",
        "",
        "정확성 범위",
        "이 데이터는 C++ 코드가 실제 rmf_traffic::schedule::Database를 호출한 결과이며 "
        "별도 목업 DB가 아닙니다. 다만 사람이 표로 읽을 수 있도록 JSONL 행으로 "
        "평탄화했으므로 RMF 내부 메모리 배치나 공식 wire/binary serialization과 동일한 형식은 아닙니다.",
        "",
        "현재 생략되는 내부 구현",
        "storage index, patch cull/history, dependency graph 내부 노드, inconsistency ranges, "
        "Mirror 전송 패치. 이 값들은 public query 결과만으로 완전 복원되지 않습니다.",
    ])


def rmf_object_guide_text(events: Iterable[dict] = ()) -> str:
    events = list(events)
    negotiation_logs = [
        event for event in events if event.get("event") == "negotiation_log"
    ]
    action_counts = Counter(
        classify_negotiation_message(str(event.get("message", "")))[0]
        for event in negotiation_logs
    )
    counts = ", ".join(
        f"{NEGOTIATION_ACTION_KO.get(action, action)}={count}"
        for action, count in action_counts.items()
    ) or "아직 협상 로그 없음"
    return "\n".join([
        "RMF 실제 객체와 처리 흐름",
        "",
        "1. Graph",
        "Graph::get_waypoint/get_lane으로 읽은 실제 waypoint·directed Lane·속성입니다.",
        "",
        "2. Supergraph",
        "Planner 내부에서 Graph를 탐색 상태·휴리스틱 캐시와 결합하는 private 구현 계층입니다. "
        "현재 public/Planner::Debug API는 Supergraph node·key·cache를 직접 반환하지 않습니다. "
        "따라서 화면은 ‘내부 존재와 노출 한계’만 표시하고 값을 만들어내지 않습니다.",
        "",
        "3. Start / Goal / Validator",
        "scenario의 로봇 요청을 Plan::Start(time, waypoint, yaw)와 Plan::Goal(waypoint)로 변환합니다. "
        "free-flow baseline은 Planner::Options(nullptr)라 RouteValidator가 없습니다. 협상 시에는 "
        "SimpleNegotiator가 negotiation table과 Schedule DB 제약을 사용하지만 호출별 validator 객체와 "
        "reject reason 전체는 public Result에 노출되지 않습니다.",
        "",
        "4. Plan → Itinerary → Route → Trajectory",
        "Plan::get_itinerary()가 vector<Route>를 반환하고, 각 Route는 map 이름과 시간 포함 "
        "Trajectory를 가집니다. 각 Trajectory waypoint에는 time·pose·velocity가 있습니다.",
        "",
        "5. Proposal",
        "CentralizedNegotiation::Result::proposal()이 participant ID별 Plan 묶음을 반환합니다. "
        "Proposal 생성 직후에는 미검증·미저장 상태이며, DetectConflict 통과 후 Participant::set이 "
        "성공해야 실제 Schedule DB itinerary가 됩니다.",
        "",
        "6. Reject / Forfeit",
        "Reject는 특정 협상 분기를 거부하는 동작이고, Forfeit는 해당 참가자가 그 table 제약에서 "
        "제출 가능한 Plan을 만들지 못해 분기를 포기하는 동작입니다. 둘 다 한 번 발생했다고 전체 "
        "협상이 즉시 실패하는 것은 아닙니다. 표의 메시지는 실제 Result::log 원문이며 action만 "
        "가독성을 위해 분류했습니다.",
        "",
        f"현재 raw 협상 로그 분류: {counts}",
    ])


def astar_guide_text() -> str:
    return "\n".join([
        "A* 내부 과정 해석 가이드",
        "",
        "• g(n): 출발 상태에서 현재 탐색 상태까지 누적된 RMF 비용입니다.",
        "• h(n): 현재 상태에서 목표까지 남았다고 추정한 비용입니다.",
        "• f(n)=g(n)+h(n): frontier 확장 우선순위를 정하는 합계입니다.",
        "• astar_step_decision/expand: 현재 frontier의 top 노드를 꺼낸 실제 단계입니다.",
        "• astar_generated: 선택 노드를 확장한 뒤 frontier에서 관찰된 자식 후보입니다.",
        "• astar_frontier_best: 한 단계가 끝난 뒤 다음 top 후보입니다.",
        "• Δg: 부모에서 자식으로 이동하면서 증가한 실제 RMF 누적비용입니다.",
        "• 구간시간: Debug node.route_from_parent의 실제 Trajectory 시간차 합입니다.",
        "• 이동/회전/대기시간: 연속 궤적점의 위치·yaw 변화를 보고 구간시간을 분류한 관찰값입니다.",
        "• g 미노출차 = Δg - 구간시간. soft penalty·event·기타 내부 비용이 섞일 수 있어 한 항목으로 단정하지 않습니다.",
        "• 그래프주행시간: 현재 waypoint에서 goal까지 열린 방향 Lane의 거리/유효속도를 Dijkstra로 계산한 검산용 하한입니다.",
        "• 첫회전시간: 현재 yaw에서 검산 경로 첫 Lane 방향까지 각도를 각속도·각가속도 한도로 정지-정지 회전한 이론 최소시간입니다.",
        "• h-그래프시간: 실제 RMF h와 단순 그래프 순수주행 하한의 차이입니다. 가속·회전·이벤트 등 휴리스틱 내부 모델의 영향이 포함될 수 있습니다.",
        "• V3 SOFT/HYBRID에서는 고정 Schedule snapshot의 corridor interval과 실제 candidate trajectory timestamp를 비교한 policy penalty가 수정된 DifferentialDrivePlanner의 child g에만 더해집니다. h와 trajectory timestamp는 바뀌지 않습니다.",
        "• policy A* 표의 movement/rotation/wait는 실제 candidate Route::trajectory timestamp 분해값입니다. approach/event/alt cost 및 final_g는 수정 지점에서 기록한 RMF_CORE 값입니다.",
        "• overlap_duration은 두 trajectory interval이 실제로 겹친 시간이고 admission_overlap_duration은 safety margin까지 확장한 hard-admission 판정 창입니다.",
        "",
        "중요: g/h/f 총합과 frontier 순서는 실제 Planner::Debug 값입니다. 이동·회전·대기 분류와 h 하한은 실제 route/graph를 사용한 실험실 진단입니다. "
        "기본 Debug API는 soft penalty·event별 g 분해, QuickestPath h 내부 항목, 모든 탈락 분기 reason code를 공개하지 않습니다. "
        "완전한 항목별 값이 필요하면 수정할 rmf_traffic 코어의 비용 계산 지점에서 직접 instrumentation 해야 합니다.",
        "",
        "표의 행을 선택하면 실제 기록값을 이용한 선택 이유가 표시됩니다. Ctrl+C는 선택 셀, Ctrl+Shift+C는 전체 표 복사입니다.",
    ])


DECISION_EVENTS = {
    "run_started", "process_phase", "planner_graph_context",
    "validator_configuration", "planning_request", "astar_trace_started",
    "astar_step_decision", "astar_expand", "astar_generated", "astar_step_summary",
    "astar_frontier_best", "astar_trace_summary", "route_candidate",
    "route_choice_explanation", "occupancy_penalty_configuration",
    "dynamic_run_started", "dynamic_insertion_stage",
    "dynamic_negotiation_request", "newcomer_penalty_configuration",
    "dynamic_insertion_result",
    "plan_waypoint", "itinerary_summary", "route_summary", "plan_summary",
    "negotiation_request", "negotiation_log", "proposal_summary",
    "proposal_plan", "proposal_outcome", "negotiation_summary",
    "safety_verification", "safety_pair_check", "pairwise_conflict_check",
    "schedule_model_schema", "schedule_database_operation",
    "schedule_database_state", "schedule_commit", "solution_diagnosis",
    "runner_core_profile",
    "corridor_definition", "corridor_policy_snapshot",
    "corridor_schedule_interval", "corridor_runtime_state",
    "corridor_state_transition", "corridor_policy_expansion",
    "runtime_event_definition", "runtime_traffic_event", "replan_trigger",
    "route_validator_result",
}


def decision_records(events: Iterable[dict]) -> list[dict]:
    records = []
    for event in events:
        if event.get("event") not in DECISION_EVENTS:
            continue
        explanation = explain_event(event)
        lines = explanation.splitlines()
        decision = lines[0].split("] ", 1)[-1]
        reason = lines[1] if len(lines) > 1 else ""
        evidence_parts = []
        for key in (
            "g", "h", "f", "selected_g", "selected_h", "selected_f",
            "delta_g_from_parent", "g_route_elapsed_s",
            "g_translation_time_s", "g_rotation_time_s", "g_wait_time_s",
            "g_unexposed_remainder", "h_graph_cruise_time_s",
            "h_first_turn_time_s", "h_rmf_minus_graph_cruise_s",
            "graph_index", "approach_lanes", "movement_type", "delta_time_s",
            "delta_distance_m", "delta_yaw_rad", "used_lanes", "selected_rank",
            "selected_cost", "cost_margin", "plan_id", "latest_version",
            "start_time_s", "executable_plan", "safety_verified", "passed",
            "route_pair_checks",
            "directed_lane_occupancy", "directed_lane_penalties",
            "candidate_id", "parent_id", "current_waypoint", "target_waypoint",
            "lane_ids", "corridor_id", "direction", "predicted_enter_time",
            "predicted_exit_time", "same_direction_penalty",
            "opposite_direction_penalty", "corridor_occupancy_penalty",
            "total_policy_penalty", "decision", "reason_code",
        ):
            if event.get(key) is not None:
                evidence_parts.append(f"{key}={event.get(key)}")
        result = ""
        if event.get("event") == "solution_diagnosis":
            result = str(event.get("status", ""))
        elif "success" in event:
            result = "성공" if event.get("success") else "실패"
        elif "passed" in event:
            result = "통과" if event.get("passed") else "실패"
        records.append({
            "seq": event.get("seq", ""),
            "phase": _phase(event.get("phase", event.get("mode", ""))),
            "robot": event.get("robot", event.get("name", "")),
            "decision": decision,
            "reason": reason,
            "evidence": ", ".join(evidence_parts),
            "result": result,
            "event": event,
            "detail": explanation,
        })
    return records


def summarize_jsonl(events: Iterable[dict]) -> str:
    events = list(events)
    if not events:
        return "아직 읽은 JSONL 이벤트가 없습니다."
    by_name = Counter(str(e.get("event", "unknown")) for e in events)
    started = next((e for e in events if e.get("event") == "run_started"), {})
    graph = next((e for e in events if e.get("event") == "graph_summary"), {})
    traits = next((e for e in events if e.get("event") == "vehicle_traits"), {})
    plans = [e for e in events if e.get("event") == "plan_summary"]
    requests = [e for e in events if e.get("event") == "planning_request"]
    negotiation = [e for e in events if e.get("event") == "negotiation_summary"]
    proposal_summaries = [e for e in events if e.get("event") == "proposal_summary"]
    proposal_outcomes = [e for e in events if e.get("event") == "proposal_outcome"]
    negotiation_logs = [e for e in events if e.get("event") == "negotiation_log"]
    states = [e for e in events if e.get("event") == "schedule_database_state"]
    diagnoses = [e for e in events if e.get("event") == "solution_diagnosis"]
    profiles = [e for e in events if e.get("event") == "runner_core_profile"]
    lines = [
        "JSONL 요약",
        "",
        f"시나리오: {started.get('scenario', '확인 중')}",
        f"설명: {started.get('description', '')}",
        f"로봇 수: {started.get('robot_count', '?')}",
        "요청 출발 시각: " + (
            " | ".join(
                f"{request.get('robot')}={_fmt(request.get('start_time_s', 0))}s"
                for request in requests)
            if requests else "아직 기록 없음"),
        "Schedule DB 동적 투입 시각: " + (
            " | ".join(
                f"{request.get('robot')}={_fmt(request.get('insertion_time_s', 0))}s"
                for request in requests)
            if requests else "아직 기록 없음"),
        f"그래프: waypoint {graph.get('waypoint_count', '?')}개, 방향 Lane {graph.get('lane_count', '?')}개, map={graph.get('map', '')}",
        f"로봇 모델: 반경 {_fmt(traits.get('profile_radius_m'))} m, 최대 선속도 {_fmt(traits.get('linear_velocity_mps'))} m/s, "
        f"차동구동·후진 {'가능' if traits.get('reversible') else '불가(제자리 회전 후 전진)'}",
        f"총 JSONL 이벤트: {len(events)}개 ({len(by_name)}종)",
        "",
        "경로 계획 결과",
    ]
    if plans:
        for plan in plans:
            if plan.get("success"):
                lines.append(
                    f"• {plan.get('robot')} / {_phase(plan.get('phase'))}: 비용 {_fmt(plan.get('cost'))}, "
                    f"종료 {_fmt(plan.get('finish_time_s'))} s, Lane {plan.get('used_lanes', [])}, "
                    f"A* 확장 {_fmt(plan.get('planner_result_expansions'))}"
                )
            else:
                lines.append(
                    f"• {plan.get('robot')} / {_phase(plan.get('phase'))}: 해 없음 "
                    f"(disconnected={plan.get('disconnected')}, saturated={plan.get('saturated')}, interrupted={plan.get('interrupted')})"
                )
    else:
        lines.append("• 아직 plan_summary가 없습니다.")
    lines.extend(["", "실행 코어·우회 penalty"])
    if profiles:
        profile = profiles[-1]
        lines.append(
            f"• label={profile.get('label')}, source={profile.get('rmf_source')}, "
            f"실제 library={profile.get('resolved_rmf_library')}"
        )
        if profile.get("lane_penalty_active"):
            lines.append(
                f"• AFTER {profile.get('lane_penalty_mode')} 모드: directed lane "
                f"{profile.get('penalized_lane_count')}개에 비용을 추가했습니다. "
                f"적용값={profile.get('directed_lane_penalties', {})}"
            )
            if profile.get("lane_penalty_mode") == "shared_corridor":
                lines.append(
                    f"• 로봇 예상경로가 실제로 겹친 통로={profile.get('shared_corridor_users', {})}, "
                    f"directed lane별 예상 수요={profile.get('directed_lane_occupancy', {})}"
                )
        else:
            lines.append("• Lane penalty 비활성: 원본 RMF 비용 기준입니다.")
    else:
        lines.append("• 아직 runner_core_profile이 없습니다.")
    lines.extend(["", "협상·안전"])
    dynamic_results = [
        e for e in events if e.get("event") == "dynamic_insertion_result"]
    if dynamic_results:
        lines.append(
            f"• 동적 투입 stage {len(dynamic_results)}개 중 "
            f"{sum(bool(item.get('success')) for item in dynamic_results)}개 성공")
        for item in dynamic_results:
            lines.append(
                f"  - stage {item.get('stage')}: success={_fmt(item.get('success'))}, "
                f"new plan={item.get('new_plan_count', 0)}, reason={item.get('reason', '')}")
    if negotiation:
        item = negotiation[-1]
        lines.append(
            f"• 협상 success={_fmt(item.get('success'))}, executable={_fmt(item.get('executable_plan'))}, "
            f"safety={_fmt(item.get('safety_verified'))}, 계산 {_fmt(item.get('elapsed_ms'))} ms"
        )
    else:
        lines.append("• 단일 로봇 시나리오이거나 아직 협상 결과가 없습니다.")
    if proposal_summaries:
        present_count = sum(bool(item.get("present")) for item in proposal_summaries)
        lines.append(
            f"• Result::proposal 조회 {len(proposal_summaries)}회: "
            f"Proposal 있음 {present_count}회, 없음 {len(proposal_summaries) - present_count}회"
        )
    if proposal_outcomes:
        lines.append(
            "• Proposal 후속 판정: "
            + " | ".join(
                f"{item.get('action')}"
                f"(accepted={_fmt(item.get('accepted'))}, committed={_fmt(item.get('committed'))})"
                for item in proposal_outcomes
            )
        )
    if negotiation_logs:
        action_counts = Counter(
            classify_negotiation_message(str(item.get("message", "")))[0]
            for item in negotiation_logs
        )
        lines.append(
            "• 실제 Result::log 분류: "
            + ", ".join(
                f"{NEGOTIATION_ACTION_KO.get(action, action)}={count}"
                for action, count in action_counts.items()
            )
        )
    lines.extend(["", "Schedule Database"])
    if states:
        item = states[-1]
        lines.append(
            f"• 마지막 스냅샷 {_phase(item.get('phase'))}: DB version {item.get('latest_version')}, "
            f"participant {item.get('participant_count')}, stored route {item.get('stored_route_count')}"
        )
    else:
        lines.append("• 아직 DB 스냅샷이 없습니다.")
    lines.extend(["", "최종 진단"])
    if diagnoses:
        short = diagnosis_text(diagnoses[-1]).splitlines()
        lines.extend(f"• {line}" for line in short[:5] if line)
    else:
        lines.append("• 아직 최종 진단이 없습니다.")
    lines.extend([
        "",
        "원본과의 관계",
        "이 요약은 JSONL 값을 읽어 한글로 재구성한 것입니다. 판단 검증이나 Before/After 비교 시에는 반드시 옆의 원본 JSONL과 seq 번호를 함께 확인하세요.",
    ])
    return "\n".join(lines)


def explain_runtime_output(text: str) -> str:
    if not text.strip():
        return "아직 실행 출력이 없습니다."
    translations: list[str] = []
    for raw in text.splitlines():
        line = raw.strip()
        if not line:
            continue
        if line.startswith("+"):
            translations.append(f"• 실행 명령: {line[1:].strip()}")
        elif "Built target" in line or "Built target" in raw:
            translations.append(f"• C++ 빌드 완료: {line}")
        elif line.startswith("Scenario:"):
            translations.append(f"• 실행 시나리오: {line.split(':', 1)[1].strip()}")
        elif line.startswith("Plan cost:"):
            translations.append(f"• RMF가 계산한 최종 계획 비용: {line.split(':', 1)[1].strip()}")
        elif line.startswith("Used lanes:"):
            translations.append(f"• 최종 경로가 사용한 방향 Lane ID: {line.split(':', 1)[1].strip()}")
        elif line.startswith("Planning time:"):
            translations.append(f"• 단일 Planner 계산 시간: {line.split(':', 1)[1].strip()}")
        elif line.startswith("Real A* expansions:"):
            translations.append(f"• 실제 Planner::Debug가 확장한 노드 수: {line.split(':', 1)[1].strip()}")
        elif line.startswith("Expected route selected:"):
            translations.append(f"• 내장 시나리오 기대 경로와 일치: {line.split(':', 1)[1].strip()}")
        elif line.startswith("Negotiation result:"):
            translations.append(f"• 다중 로봇 협상 결과: {line.split(':', 1)[1].strip()}")
        elif line.startswith("Plans:"):
            translations.append(f"• 협상으로 생성된 로봇 계획 수: {line.split(':', 1)[1].strip()}")
        elif line.startswith("Schedule DB version:"):
            translations.append(f"• 저장 완료 후 Schedule DB 전체 버전: {line.split(':', 1)[1].strip()}")
        elif line.startswith("Calculation time:"):
            translations.append(f"• 협상 계산 시간: {line.split(':', 1)[1].strip()}")
        elif line.startswith("AFTER lane penalty:") or line.startswith(
            "AFTER occupancy-aware core penalty:"):
            translations.append(
                "• AFTER 입력: 로봇 예상 점유 통로·기존 최단경로 또는 수동 Lane을 실제 A* g-cost에서 "
                f"불리하게 만들었습니다. {line.split(':', 1)[1].strip()}")
        elif "[RMF_LANE_PENALTY]" in line or "[RMF_OCCUPANCY_PENALTY]" in line:
            translations.append(
                "• 수정 코어 증거: DifferentialDrivePlanner 내부 Lane penalty 코드가 "
                f"실제로 활성화됐습니다. {line}")
        elif re.search(r"warning|error|failed|no solution|no proposal", line, re.I):
            translations.append(f"• 주의가 필요한 원문: {line}")
    if not translations:
        return "실행 출력은 수신됐지만 번역 가능한 주요 상태 문장이 아직 없습니다. 원본 실행 로그를 확인하세요."
    translations.extend([
        "",
        "해석 메모",
        "CMake/컴파일 문장은 RMF 계산 결과가 아니라 실행 파일 준비 과정입니다. 실제 판단은 JSONL 요약과 스텝별 판단 근거에서 seq 번호로 확인하세요.",
    ])
    return "\n".join(translations)
