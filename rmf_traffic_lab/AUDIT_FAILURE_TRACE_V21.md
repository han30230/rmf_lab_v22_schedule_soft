# RMF v21 Failure Trace Audit / Implementation

## Audit

기존 v21에는 다음 실제 이벤트가 이미 존재했다.

- Planner/A*: `planning_request`, `plan_summary`, `astar_trace_summary`, `route_candidate`
- Schedule DB: `schedule_database_operation`, `schedule_database_state`, itinerary/route/trajectory events
- Validator/Conflict: `route_validator_result`, `pairwise_conflict_check`, `safety_verification`
- Negotiation: `negotiation_request`, raw `negotiation_log`, `proposal_summary`, `proposal_outcome`, `negotiation_summary`
- Final diagnosis: `solution_diagnosis`

부족했던 부분은 이 이벤트들이 여러 탭에 흩어져 있어서 Planner → Schedule → Conflict → Negotiation → Final 실패를 하나의 causal timeline으로 보기 어렵다는 점이었다.

## Added

`실패 원인 추적` 탭을 추가했다.

- Primary Cause 자동 분류
  - `PLANNER_NO_SOLUTION`
  - `VALIDATOR_REJECT`
  - `SCHEDULE_CONFLICT`
  - `NO_NEGOTIATION_ALTERNATIVE`
  - `NEGOTIATION_FAILED`
  - `SEARCH_LIMIT_REACHED`
  - `NO_PHYSICAL_ESCAPE`
  - `UNKNOWN`
- 단계별 Failure Timeline
- 실제 conflict pair/time 표시
- 실제 proposal 존재/plan count 표시
- 기록된 근거에 연결된 개선 방향 표시

## Data integrity rule

새 진단은 기존 JSONL의 실제 RMF/실험 이벤트만 집계한다.

- Conflict 위치가 raw event에 없으면 `UNKNOWN`
- `CentralizedNegotiation::Result`가 노출하지 않는 내부 alternative 개수는 `UNKNOWN`
- 불명확한 실패 원인은 억지로 분류하지 않고 `UNKNOWN`

## Modified files

- `tools/event_explainer.py`
- `simulator.py`
- `tests/test_event_explainer.py`

## Validation

```text
python -m pytest -q
71 passed, 58 subtests passed
```

현재 컨테이너에는 사용자 WSL의 수정 `librmf_traffic` 실행 환경/빌드 산출물이 없으므로 실제 native RMF end-to-end Scenario 실행은 수행하지 않았다. WSL에서 대표 실패 Scenario를 1회 실행해 `실패 원인 추적` 탭의 source API와 raw JSONL을 최종 확인해야 한다.
