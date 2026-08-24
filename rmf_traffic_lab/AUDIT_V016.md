# Open-RMF Traffic Lab v0.16 Audit 및 구현 보고서

## 1. 수정 전 구현 Audit

### 기존 AFTER soft penalty 위치

기존 V2는 `tools/setup_after_core.py`가 복사된 실제
`rmf_traffic/src/rmf_traffic/agv/planning/DifferentialDrivePlanner.cpp`를 패치했다.
`ScheduledDifferentialDriveExpander::expand_traversal()`에서 child `SearchNode`의 누적비용

```cpp
node->current_cost + entry_event_cost + alt->cost
```

뒤에 `rmf_lab_detour_penalty`를 더했다. 따라서 UI만의 값이 아니라 수정 라이브러리를 빌드하면
실제 A* g에 들어갔다.

### 기존 penalty 입력의 정확한 성격

- `shared_corridor`: C++ runner가 각 로봇의 실제 free-flow RMF Plan에서 사용 Lane을 읽고,
  unordered waypoint pair 또는 mutex group별 예상 사용자 수를 사전 집계했다.
- `shortest_path`: 실행 전 각 로봇의 최단경로 Lane에 정적 값을 지정했다.
- `manual`: 맵 JSON의 `after_penalty`를 directed Lane 값으로 변환했다.
- 셋 모두 planning invocation의 실제 Schedule Viewer trajectory 시간중첩을 조회한 값은 아니었다.
- 같은 방향/반대 방향을 구분하지 않았고 candidate의 예상 진입·이탈 시각도 사용하지 않았다.

### 기존 Validator 연결

- free-flow와 baseline Plan은 `Planner::Options(nullptr)`라
  `ScheduleRouteValidator`를 Planner 옵션으로 넣지 않았다.
- 다중 로봇은 실제 `CentralizedNegotiation`과 `SimpleNegotiator`를 사용했고,
  `SimpleNegotiator` 내부에서 실제 `NegotiatingRouteValidator`가 만들어졌다.
- 최종 proposal은 실제 `DetectConflict::between`으로 다시 검사했다.
- Schedule DB는 실제 in-memory `schedule::Database`였고 proposal 통과 후
  `Participant::set`으로 itinerary를 저장했다.

### 기존 모드와 UI 데이터 경계

- BEFORE: 원본 RMF install/source.
- AFTER: V2 g-cost 패치가 들어간 별도 RMF install/source.
- AFTER_NEGO: 같은 V2 패치와 persistent Schedule DB, 기존 itinerary 고정,
  newcomer-only negotiation 및 기존 사용 Lane penalty.
- 실제 RMF 값: Graph, Planner/Debug g·h·f, Plan, itinerary, Route, trajectory,
  proposal, raw negotiation log, Schedule query, DetectConflict.
- 별도 계산/추정: candidate path 열거, topology diagnosis, trajectory 구간의 이동/회전/대기 분류,
  V2 lane occupancy, 한글 설명. 예전 UI는 이 source 구분이 모든 표에 일관되지는 않았다.

## 2. v0.16에서 변경한 파일

| 파일 | 변경 내용 |
|---|---|
| `tools/setup_after_core.py` | 실제 Planner와 SimpleNegotiator에 V3 hook 설치, internal policy header 생성 |
| `src/rmf_core_lab.cpp` | 실제 Schedule snapshot/index, Corridor interval/state, delay, validator 계측, JSONL 출력 |
| `tools/traffic_policy.py` | Qt/ROS 독립 정책 reference model과 deterministic state machine |
| `tools/scenario_templates.py` | Corridor/runtime event 포함 27개 template 및 S1~S10 |
| `simulator.py` | 네 모드, Corridor 편집·overlay·상태 패널, 확장 A*/Schedule/Validator 표, delay/replan 편집 |
| `run.py` | 네 mode/weight CLI, Corridor/runtime compiler, snapshot/trace 병합 |
| `tools/event_explainer.py` | source 경계와 policy/validator/corridor event 한글 해석 |
| `tests/test_traffic_policy.py` | S1~S10과 추가 policy 안전 회귀 테스트 |
| `tests/test_after_core.py` | 실제 patch 위치·g-only·participant scope·생성 header C++17 컴파일 테스트 |
| `tests/test_scenario_templates.py` | Corridor/runtime schema 및 UI/core source 회귀 테스트 |
| `tests/test_event_explainer.py` | source/overlap/해석 회귀 테스트 |
| `README.md`, `package.xml`, `web_server.py` | v0.16 사용법과 버전 갱신 |

## 3. 실제 RMF original과 custom 코드 경계

원본 그대로 사용하는 부분은 Graph, VehicleTraits, Planner, QuickestPath heuristic,
RouteValidator collision semantics, Schedule Database, Participant, CentralizedNegotiation,
NegotiatingRouteValidator, Plan/Itinerary/Route/Trajectory, DetectConflict이다.

custom 부분은 다음 세 지점이다.

1. `DifferentialDrivePlanner.cpp` traversal expansion에서 internal policy provider를 한 번 호출한다.
2. hard block이면 기존 RouteValidator 호출보다 앞에서 그 신규 entry child를 생성하지 않는다.
3. 허용된 child의 stock g 식에 `total_policy_penalty`만 더한다.

`SimpleNegotiator.cpp` 수정은 협상 중 현재 participant ID를 thread-local scope에 넣어 자기 Schedule
interval을 제외하기 위한 계측 context이다. public header/exported symbol은 바꾸지 않는다.

## 4. 새로운 데이터 구조

- `CorridorDef`: id, forward/reverse Lane, capacity, passing, hard flag, holding entries, base cost.
- `PolicyScheduleInterval`: participant/plan/route, direction, enter/exit, state, owner,
  responsiveness, itinerary version, source.
- `Snapshot`: 한 planning invocation에 고정된 Corridor별 interval bucket.
- `CorridorDecision`: candidate/parent/waypoint/Lane/time, overlap rows, cost breakdown,
  hard/soft decision, source.
- `CorridorRuntime`: FREE/RESERVED/OCCUPIED/UNKNOWN_HOLD, owner direction, occupants,
  reservations, last update.

## 5. penalty 계산 공식

```text
actual_overlap = max(0, min(candidate_exit, other_exit)
                        - max(candidate_enter, other_enter))

same_direction_penalty = Σ actual_overlap_same × same_weight
opposite_direction_penalty = Σ actual_overlap_opposite × opposite_weight
corridor_occupancy_penalty =
    Σ occupied_overlap × occupied_weight
  + Σ future_reserved_overlap × future_weight

total_policy_penalty = corridor.base_penalty + static_weight
                     + same_direction_penalty
                     + opposite_direction_penalty
                     + corridor_occupancy_penalty
                     + no_escape_penalty

new_g = parent_g + RMF approach_cost + RMF entry_event_cost
      + RMF alt_cost + total_policy_penalty
f = new_g + stock_RMF_h
```

`overlap_margin`은 hard admission 안전창에만 사용하고 실제 overlap 초와 별도 기록한다.
policy는 h와 trajectory timestamp를 변경하지 않는다. 같은 physical corridor의 다중 Lane traversal과
continuation은 중복 과금하지 않는다.

## 6. hard admission 판정

HYBRID/HYBRID+NEGO에서 아래가 모두 참일 때만 block한다.

1. 새로운 corridor entry이다.
2. corridor가 non-passing이고 capacity가 1 이하이다.
3. opposite direction interval과 margin 포함 admission window가 겹친다.
4. 그 interval이 OCCUPIED/UNKNOWN_HOLD이거나 deterministic admission owner이다.
5. corridor의 hard flag가 활성화되어 있다.

세상 어딘가에 반대방향 로봇이 있다는 이유만으로 block하지 않는다. 이미 내부에 있는 participant는
Schedule 상태로 재식별하여 continuation/exit를 항상 허용한다.

## 7. reservation과 release

현재 occupant/unknown을 먼저 owner로 선택하고, 없으면 예약 요청/예상 진입시각과 participant ID로
deterministic winner를 정한다. 동일 방향은 RMF RouteValidator의 실제 trajectory separation 검사를
유지하면서 convoy 후보가 될 수 있다.

예상 exit 시간이 지난 것만으로 release하지 않는다. lab에서는 명시적 confirmed
`CHECKPOINT_RELEASE`가 있어야 제거한다. 통신 끊김은 `UNKNOWN_HOLD`로 보수적으로 유지한다.

## 8. Schedule 조회 방식과 성능

각 planning/negotiation invocation 직전에 실제
`Database::query(schedule::query_all())`을 한 번 호출한다. 실제 Route trajectory timestamp를
Corridor Lane 끝점에 연결하고 Corridor별 정렬 interval index를 만든다. A* expansion은 해당
Corridor bucket만 순회한다.

- snapshot 생성: 대략 `O(schedule routes × configured corridors × corridor lanes)`.
- expansion: `O(candidate corridor의 interval 수)`.
- 탐색 중 Schedule version 교체 없음.
- 100 ms DB 변경마다 자동 전체 replan하지 않음.

## 9. 방향 판별

Corridor editor의 `lanes_forward`는 `A_TO_B`, `lanes_reverse`는 `B_TO_A`로 정의한다.
candidate traversal의 실제 lane ID와 Schedule Route가 통과한 lane endpoint 순서를 같은 mapping에
대입한다. UNKNOWN은 opposite으로 단정하지 않는다.

## 10. 기존 RouteValidator와의 관계

- Corridor Admission: custom `POLICY_DERIVED` hard constraint.
- `ScheduleRouteValidator::find_conflict(Route)`: proposal을 실제 DB와 post-proposal 검사하며
  blocker participant/plan/route/checkpoint/time을 기록한다.
- `NegotiatingRouteValidator`: 수정하지 않은 `SimpleNegotiator` 내부 실제 경로. public Result가
  호출별 conflict를 노출하지 않으므로 UI에는 활성 경로와 비노출 사실만 표시한다.
- `DetectConflict::between`: 최종 participant 조합의 연속시간 profile 충돌 재검증.

## 11. 테스트 결과

현재 개발 컨테이너에서 다음을 확인했다.

- Python/static 회귀 테스트: 62개 통과.
- 생성된 `RmfLabCorridorPolicy.hpp`: `g++ -std=c++17 -Wall -Wextra -pedantic` standalone compile 통과.
- S1~S10 policy 기대조건, deterministic owner, occupant exit, zero weight equivalence,
  finite SOFT cost, margin 분리, duplicate charge 방지를 단위 테스트함.
- 모든 27개 template이 custom RMF intermediate schema로 compile됨.

이 컨테이너에는 ROS Jazzy/rmf_traffic/PySide6가 없어서 실제 `colcon build`, RMF 링크 CTest,
GUI live launch는 수행하지 못했다. 이는 사용자 RMF workspace에서 아래 명령으로 반드시 최종 확인해야
한다. 따라서 이 보고서는 full Jazzy integration 성공을 허위로 주장하지 않는다.

## 12. 네 모드 예상 비교

| 항목 | BASELINE | SOFT | HYBRID | HYBRID+NEGO |
|---|---|---|---|---|
| stock g/h | 그대로 | h 그대로, g에 policy | h 그대로, g에 policy | HYBRID와 동일 |
| 반대방향 entry | RMF negotiation/validator만 | 큰 유한 cost | owner와 겹치면 hard block | hard block + newcomer 개선 |
| 우회 유도 | stock cost | overlap cost로 유도 | overlap cost + admission | 동적 newcomer에 더 적극적 |
| 대체경로 없음 | stock 결과 | path를 제거하지 않음 | blocked entry는 wait/다른 timing 필요 | escalation 필요 가능 |

실제 plan 결과는 RMF 버전, topology, timing, weight에 따라 달라지므로 UI의 동일 scenario SHA와
JSONL을 기준으로 비교해야 한다.

## 13. 성능 영향

기존보다 planning invocation 시작 시 Schedule route-to-corridor index 생성비용과 expansion별 해당
bucket overlap 비교가 추가된다. 정책 활성 시 stock free-flow shortcut을 사용하지 않아 단일 로봇
탐색도 더 느릴 수 있다. weight가 지나치게 크면 cost leeway/threshold와 탐색량이 증가할 수 있다.

## 14. production과 다른 simulator-only 부분

- in-memory Schedule DB이며 운영 Mirror/Writer/Fleet Adapter 네트워크가 아니다.
- actual reached/location은 없고 checkpoint release와 comm loss를 simulator event로 입력한다.
- Corridor는 RMF 표준 Schedule 필드가 아니라 그래프 Lane을 묶는 custom metadata이다.
- route-to-corridor association은 trajectory가 graph Lane endpoint를 통과하는 것을 사용한다.
- priority aging과 방향전환 starvation 정책은 reference runtime에 최소 구조만 있고 운영 dispatcher와
  통합되지 않았다.
- material traffic conflict가 실제 fleet command를 자동 취소/replan하지 않는다.

## 15. production Fleet Adapter 적용 전 추가 작업

1. 실제 Adapter의 itinerary delay/reached/location callback을 Corridor runtime에 연결한다.
2. participant 등록/plan rollover/erase를 따라 reservation lifecycle을 영속화한다.
3. timeout 시 즉시 FREE가 아닌 운영용 fail-safe 및 수동 해제 권한을 설계한다.
4. task priority, aging, 최대 연속 convoy 수, 방향 전환 공정성을 현장 정책으로 정한다.
5. 여러 process/fleet이 보는 authoritative Corridor admission service 또는 Schedule extension을 설계한다.
6. 실제 robot footprint, braking distance, 통신 지연을 반영해 margin/holding entry를 검증한다.
7. 기존 Planner ABI/source 버전별 patch rebase와 upstream 변경 회귀 테스트를 자동화한다.
8. shadow mode → 제한 구역 canary → rollback 가능한 단계적 배포를 수행한다.

단순히 Planner g만 바꾸면 실제 운행의 corridor ownership이 fleet process 사이에서 원자적으로
보장되지 않는다. SOFT는 우회 확률을 높이지만 hard 안전 보장은 admission lifecycle과 실제 진행
피드백까지 연결되어야 한다.
