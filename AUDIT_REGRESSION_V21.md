# RMF Lab v21 Regression Audit

## 수정 전 구조 Audit

| Profile | setup/source | 실제 차이 |
|---|---|---|
| Baseline | 원본 RMF workspace/install | stock `rmf_traffic` 비용·협상 |
| Soft | 수정 workspace/install | `DifferentialDrivePlanner`의 Schedule 겹침 penalty를 g에 추가 |
| Hybrid | Soft와 같은 수정 library | 같은 hook에서 corridor 반대방향 신규 진입 hard admission 추가 |
| Hybrid+Nego | 별도 수정 workspace/install | Hybrid 정책 + 동적 newcomer stage의 `after_nego` lane penalty |

따라서 네 profile이 모두 서로 다른 source tree를 쓰는 구조는 아니다. Soft/Hybrid는 같은 수정
`librmf_traffic`을 쓰고 runtime policy mode가 다르다. Hybrid+Nego의 추가 효과는 이미 commit된
로봇을 고정한 동적 insertion Scenario에서만 발생하므로 정적 Scenario에서는 Hybrid와 결과가
같을 수 있다.

## 기존 Scenario 실행 경로

1. GUI document 또는 내장 template를 JSON으로 저장한다.
2. `run.py::compile_custom_scenario`가 C++ runner 입력 `.rmf`로 변환한다.
3. 선택한 setup 환경으로 `rmf_core_lab`을 configure/build/link한다.
4. C++가 실제 `Planner`, `Planner::Debug`, `CentralizedNegotiation`,
   `ScheduleRouteValidator`, `schedule::Database`, `DetectConflict`를 호출한다.
5. 원본 관측값을 JSONL에 기록하고 GUI가 이를 표시한다.

Python은 알고리즘 성공 결과나 경로를 합성하지 않는다.

## 기존 판정과 metric

- Success/No Solution: 마지막 실제 `solution_diagnosis.status`
- Conflict: `safety_verification.conflicts`; 없는 경우 실제 pairwise conflict check
- Deadlock: 직접적인 RMF public flag가 없으므로 확정된 no-proposal/timeout 중
  `endpoint_exchange_without_buffer`, `single_route_no_yield_space`, `runner_timeout` category만 분류
- 시간/거리/대기/최종 경로: 최종 `plan_waypoint`와 `plan_summary`
- Planning/expanded: `planner_timing`, `astar_trace_summary`
- Negotiation: `negotiation_summary`, 원본 negotiation log의 selected table
- Validator/policy: `route_validator_result`, `corridor_policy_expansion`

공개 API가 노출하지 않는 내부 negotiation validator의 호출별 reject 원인은 수집할 수 없다.
대신 실제 원문 log와 최종 진단을 보존한다.

## 추가된 Regression 실행

`regression_runner.py`가 Scenario별 `input.json` 하나를 동결하고 Baseline → Soft → Hybrid →
Hybrid+Nego를 순차 실행한다. 각 profile은 별도 lab CMake build directory를 사용한다. Scenario
SHA, random seed, 실제 VehicleTraits signature가 모두 같아야 `identical_input=true`가 된다.

또한 JSONL의 실제 linked library를 검증한다.

- Baseline library는 모든 수정 profile과 달라야 함
- Soft와 Hybrid library는 같아야 함
- Hybrid+Nego library는 Hybrid와 달라야 함
- 기록된 traffic mode는 요청 profile과 같아야 함

이 조건이 깨지면 `core_provenance.verified=false`로 남으므로 같은 library를 네 이름으로만 바꿔
실행한 결과를 정상 비교로 오인하지 않는다.

## Regression 규칙

- Baseline SUCCESS → Modified NO_SOLUTION: `REGRESSION`
- Baseline 안전 SUCCESS → Modified Deadlock/Conflict: `REGRESSION`
- Baseline 실패/Deadlock/Conflict → Modified 충돌 없는 SUCCESS: `IMPROVEMENT`
- 그 외: `NO_CHANGE`와 수치 delta를 직접 비교

Termination reason은 실제 diagnosis category를 정규화한다. 예를 들어
`search_saturation → SATURATION_LIMIT`, `disconnected_topology → NO_VALID_ROUTE`,
`dynamic_newcomer_no_proposal → NEGOTIATION_FAILED`이다. 원래 category는 별도 필드에 남는다.

## 변경 영향

- 기존 단일 Scenario 실행 버튼과 결과 파일 형식은 유지된다.
- `run.py`에는 기본값 0인 선택 인자 `--random-seed`만 추가되었다.
- Regression은 별도 process/build/result directory를 사용한다.
- P3/P4는 template만 확장되며 다른 27개 Scenario는 변경하지 않았다.

## 검증 상태

- Python syntax compile: 통과
- unit/integration source tests: 68/68 통과
- 29개 내장 Scenario의 custom `.rmf` 변환 test: 통과
- P3: 179 nodes, 196 lanes, 10 vertical axes, 12 robots
- P4: 141 nodes, 156 lanes, 9 vertical axes, 10 robots

이 배포 환경에는 사용자의 ROS Jazzy와 세 RMF install workspace가 없으므로 실제 네 library를
링크한 전체 native 결과 숫자는 생성하지 않았다. 사용자의 WSL에서 첫 Regression 실행 후
`core_provenance.verified`, `identical_input`, profile별 JSONL을 확인해야 한다.
