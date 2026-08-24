# SCHEDULE_SOFT V22 Audit / Implementation Notes

## 목적

BASELINE Open-RMF의 기존 Planner cost/feasibility를 최대한 유지하면서, Plan/Replan 시작 시점의 최신 Schedule DB snapshot에 등록된 다른 participant의 trajectory와 candidate 경로가 시공간적으로 겹칠 때만 작은 additive cost를 부여한다.

## v21 Audit에서 확인한 기존 상태

- 기존 OLD_SOFT/HYBRID는 실제 `DifferentialDrivePlanner` 내부 A* child `g`에 policy cost를 더하는 hook을 사용한다.
- `configure_policy_snapshot()`은 이미 planning invocation당 `Database::query(query_all)`을 1회 수행하고 snapshot 파일을 생성한다.
- 기존 snapshot은 static/free-flow admission reservation 및 `POLICY_DERIVED` interval을 포함할 수 있다.
- 기존 snapshot 생성 단계에서는 self itinerary를 제거하지 않았다. 기존 policy 평가 단계에서 participant context 기반 self skip은 있었으나, snapshot 자체에는 남을 수 있었다.
- 동적 scenario의 Delay는 사용 가능한 RMF API에서 실제 `Participant::delay(Duration)`를 호출한다.
- 현재 v21 dynamic runner는 Schedule update 자체로 기존 모든 로봇을 자동 Replan하지 않는다. trigger는 다음 explicit planning invocation이 최신 Schedule을 읽도록 하는 구조다.

## 이번 변경

### 독립 Variant

- `SCHEDULE_SOFT` 추가.
- 전용 workspace: `~/rmf_ws_schedule_soft`.
- `tools/setup_schedule_soft_core.py`가 BASELINE source에서만 분기한다.
- OLD_SOFT marker가 있는 source를 입력하면 중단한다.
- 기존 파일이 있는 정체 불명의 workspace는 자동 덮어쓰지 않는다.

### Cost

```text
schedule_soft_penalty
= min(lambda * overlap_duration * direction_weight의 합, max_penalty)

new_g
= original_rmf_g + schedule_soft_penalty
```

기본값:

- lambda = 0.25
- max_penalty = 10.0
- same_direction_weight = 0.5
- opposite_direction_weight = 1.5

`lambda=0` 또는 `max_penalty=0`이면 schedule_soft hook은 disabled로 간주하며 stock free-flow shortcut을 유지한다.

### Snapshot 안정성

- Plan/Replan invocation 시작 시 Schedule DB를 1회 조회한다.
- snapshot generation/version은 해당 Planning job 동안 고정된다.
- A* expansion 중 DB를 다시 query하지 않는다.
- snapshot은 corridor별 interval index로 읽히며 candidate는 관련 corridor bucket만 검사한다.

### Self itinerary 제외

`schedule_soft` 모드에서는 snapshot 생성 시 `participant_id`와 동일한 Schedule route를 제외한다. Policy evaluation 단계에서도 동일 participant interval은 다시 무시한다.

### 실제 Schedule interval만 사용

`schedule_soft` 모드에서는:

- admission/free-flow reservation을 snapshot에 넣지 않는다.
- 과거 interval을 `UNKNOWN_HOLD + 86400s` 형태로 확장하지 않는다.
- `source == SCHEDULE`인 실제 등록 interval만 cost 계산에 사용한다.
- OLD_SOFT의 static/no_escape/occupied/future cost를 사용하지 않는다.

### 시공간 중첩

현재 공간 필터는 실제 RMF `Route::trajectory`에서 corridor에 해당하는 시간 interval을 추출하여 물리 corridor 단위로 index한다. Temporal overlap이 0이면 cost는 0이다.

완전한 continuous geometry/profile collision sweep을 매 candidate마다 수행하는 구조는 이번 버전에 추가하지 않았다. 이는 계산량과 기존 RMF feasibility 로직 중복을 피하기 위한 보수적 선택이다. Validator/Negotiation의 실제 collision 판단은 기존 RMF 로직을 유지한다.

## 성능 계측

추가 로그/비교 지표:

- schedule snapshot count/version
- schedule query count
- queried participant count
- queried route count
- self-filtered route count
- candidate overlap check count
- planning time
- expanded nodes

## Replan 관련 정확한 범위

Schedule DB update와 Replan trigger는 분리한다.

```text
Delay / Schedule update
  -> Database version 변경
  -> 다음 명시적 Plan/Replan invocation
  -> 최신 snapshot 1회 취득
  -> 새 overlap cost 계산
```

현재 simulator는 실제 Fleet Adapter가 없기 때문에 `trigger_replan=true`만으로 기존 주행 로봇의 현재 pose를 임의 생성해 Planner를 재호출하지 않는다. 현재 pose를 모르는 상태에서 이를 구현하면 가짜 Replan이 되므로 하지 않았다. 신규 로봇 planning 등 다음 실제 Planner invocation에서는 최신 Schedule을 사용한다. 실제 기존 로봇 Replan까지 검증하려면 향후 Fleet Adapter/mock adapter가 제공하는 현재 graph position/start state를 입력으로 연결해야 한다.

## Side-effect 방지 Gate

1. BASELINE success -> SCHEDULE_SOFT success 유지
2. lambda=0 -> BASELINE과 최대한 동일
3. Schedule overlap 없음 -> penalty 0
4. overlap + bypass 있음 -> bypass 선호 가능
5. overlap + bypass 없음 -> candidate feasibility 자체는 유지
6. planning job 중 snapshot 고정
7. self itinerary 제외
8. A* expansion당 DB full scan 금지
9. OLD_SOFT 로직 비누적
10. 전체 Variant Compare All 유지

## 로컬 검증

현재 artifact 생성 환경에서:

- Python syntax compile 통과
- 생성 policy header C++17 compile test 통과
- pytest: 75 tests + 58 subtests 통과

ROS2/RMF toolchain이 없는 환경이라 실제 `colcon build`와 real RMF scenario 실행은 여기서 수행하지 못했다. 사용자 WSL의 ROS2 Jazzy/RMF 환경에서 `SCHEDULE_SOFT 코어 준비` 후 rebuild와 Regression 실행이 최종 검증 단계다.
