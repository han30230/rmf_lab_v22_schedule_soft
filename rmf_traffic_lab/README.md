# RMF Traffic Core Analyzer Lab

실제 Open-RMF Traffic Core를 직접 호출하는 실험실입니다. 이제 브라우저 없이 실행하는
PySide6 데스크톱 시뮬레이터와 기존 HTML 정밀 분석 화면을 모두 제공합니다.

- 경로 계획: `rmf_traffic::agv::Planner`
- 실제 A* 추적: `rmf_traffic::agv::Planner::Debug`
- 다중 로봇 협상: `rmf_traffic::agv::CentralizedNegotiation`
- 실행 전 충돌검증: `rmf_traffic::DetectConflict::between`
- Traffic DB: `rmf_traffic::schedule::Database`와 `Participant`
- 실제 route 검증: `rmf_traffic::agv::ScheduleRouteValidator::find_conflict`
- UI 위치·시간: `Plan::get_itinerary()`의 실제 RMF trajectory
- Python 역할: 맵 편집 UI, CMake 빌드·실행, JSONL 표시와 로봇 궤적 재생

Fleet Adapter, 실제 로봇 드라이버, Gazebo, ROS 토픽은 아직 사용하지 않습니다. 따라서 “RMF Traffic Core의 그래프 계획과 time-space 협상”을 다른 계층과 분리해 관찰할 수 있습니다.

현재 v0.16의 핵심 실험은 directed Lane 여러 개를 하나의 물리 Corridor로 묶고, 한 번의
Planner invocation 시작 시 실제 Schedule DB를 조회해 만든 고정 interval index를 수정된
`DifferentialDrivePlanner.cpp`의 traversal expansion에서 사용하는 것입니다. 정책 값은
`POLICY_DERIVED`, 실제 core 비용은 `RMF_CORE`, DB row는 `SCHEDULE`, 지연·통신 끊김 입력은
`SIMULATION_EVENT`로 구분합니다.

## 웹 시뮬레이터

데스크톱 앱의 맵 편집, 로봇 추가·삭제, Before/After 코어, 실제 실행 로그·JSONL,
Schedule Database, A*, 스텝별 판단 근거, trajectory 재생과 화면 분할 기능을 브라우저로
옮겼습니다. 웹 서버는 외부 프레임워크가 필요 없으며 WSL Ubuntu에서 다음처럼 실행합니다.

```bash
cd ~/rmf_traffic_lab
python3 web_server.py --host 0.0.0.0 --port 8080
```

또는 `./start_web.sh`를 실행합니다. VS Code에서는 Run and Debug의
`RMF Web Simulator (8080)`을 선택하고 F5를 누릅니다.

- 실행한 PC: `http://localhost:8080`
- 같은 사내망의 다른 PC: `http://<서버-PC-IP>:8080`

웹 페이지의 `변경사항 빌드 후 RMF 분석`은 브라우저 안에서 Planner를 흉내 내지 않습니다.
서버가 현재 편집한 JSON을 고유 실행 폴더에 저장한 뒤 기존 `run.py`와 C++
`rmf_core_lab`을 실행합니다. 따라서 실제 `Planner`, `Planner::Debug`,
`CentralizedNegotiation`, `DetectConflict`, `schedule::Database`를 쓰는 구조는 데스크톱과
같습니다. 여러 사용자가 동시에 버튼을 눌러도 RMF 빌드와 실행은 서버에서 한 건씩 처리하고,
로그와 JSONL만 각 브라우저에 실시간 전송합니다. 실행 증거는
`web_data/runs/<실행-ID>/`에 분리 보관됩니다.

WSL2에서 사내 다른 PC가 접속하지 못하면 먼저 Windows PowerShell의 `ipconfig`로 서버 PC의
사내 IPv4를 확인하고 Windows 방화벽에서 TCP 8080 인바운드를 허용하십시오. 회사 정책 또는
WSL 네트워크 모드 때문에 Windows→WSL 포트 전달이 필요한 환경도 있습니다. 서버 터미널에
`RMF Traffic Lab Web: http://0.0.0.0:8080`이 표시돼도 방화벽 허용을 뜻하지는 않습니다.

사내망 공개 시에는 최소한 접근 토큰을 설정하는 편이 안전합니다.

```bash
cd ~/rmf_traffic_lab
RMF_LAB_TOKEN='긴-임의-문자열' python3 web_server.py --host 0.0.0.0 --port 8080
```

접속자는 상단 `사용법`에서 같은 토큰을 한 번 저장합니다. 이 서버에는 사용자 계정·TLS·권한별
격리가 없으므로 인터넷에 직접 공개하지 마십시오. Nginx 등의 사내 리버스 프록시 뒤에 둘 때에는
SSE 경로 `/api/runs/<id>/stream`의 proxy buffering을 끄고 장시간 연결 timeout을 늘려야 합니다.
또한 웹의 코어 복사·패치 준비 기능은 서버 파일을 쓰므로 공용 조회 전용 서버는
`RMF_LAB_ALLOW_CORE_PATCH=0`으로 실행하십시오. 운영 Fleet Adapter나 운영 Schedule DB에는
연결하지 않으며, 이 격리된 Traffic Core 실험만 실행합니다.

### Building-map YAML 가져오기

웹 상단 `YAML 맵 열기` 또는 데스크톱 툴바의 같은 버튼에서 RMF/Traffic-Editor 계열
building-map YAML을 직접 열 수 있습니다. 여러 level이 있으면 웹에는 `YAML 레벨` 선택기가
생기고, 데스크톱은 가져올 level을 묻습니다. 현재 실험 엔진이 단일 map을 대상으로 하므로 층별로
하나씩 불러와 분석합니다. 바로 시험할 전체 예제는
`scenarios/building_map_yaml_example.yaml`에 들어 있습니다.

```yaml
building_name: p4_track_v05_260317_3
levels:
  L1:
    vertices:
      - [0.0, 0.0, START, {is_holding_point: true}]
      - [2.0, 0.0, GATE, {is_passthrough_point: true}]
    lanes:
      - - 0
        - 1
        - speed_limit: 1.0
          rotationAllowed: false
          corridor:
            leftWidth: 0.7
            rightWidth: 0.7
            corridorRefPoint: CONTOUR
```

가져오기 규칙:

- `vertices`의 x/y/name과 holding/parking/passthrough/mutex를 편집 노드로 변환
- `lanes`의 entry/exit, `speed_limit`, mutex, 폐쇄, `bidirectional`을 편집 Lane으로 변환
- YAML Lane은 `bidirectional:true`가 명시되지 않으면 방향성 Lane 한 개로 해석
- Corridor left/right 폭은 지도에 실제 축척의 반투명 폭으로 표시
- 원본 `orientation`, `rotationAllowed`, corridor 속성은 JSON 저장 시 함께 보존
- YAML에는 작업 요청이 없으므로 가져온 뒤 `로봇` 탭에서 로봇을 추가하고 start/goal node
  index를 지정해야 실행 가능

현재 실제 `rmf_traffic::agv::Graph` 실행에는 좌표, 방향성 연결, speed limit, mutex와 waypoint
대기 속성이 적용됩니다. `corridor.leftWidth/rightWidth`, `corridorRefPoint`, vendor 확장
`rotationAllowed`, Lane `orientation`은 현재 lab의 Graph compiler가 RMF 제약으로 강제하지 않고
표시·원본 보존만 합니다. Corridor 폭을 충돌 판정이나 A* 비용에 반영하려면 다음 단계에서
로봇 profile/corridor capacity 또는 validator 비용으로 명시적으로 연결해야 합니다.

사용자가 보여준 것처럼 `lanes` 부분만 발췌한 내용으로는 좌표를 알 수 없어 지도를 만들 수
없습니다. 실제로 열 파일에는 같은 level 아래의 전체 `vertices` 배열이 포함되어 있어야 합니다.

## 데스크톱 시뮬레이터

Windows 11의 WSLg/Ubuntu 터미널에서 다음을 한 번 실행합니다.

```bash
cd ~/rmf_traffic_lab
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -r requirements-gui.txt
python3 simulator.py
```

이번 배포본에는 Noto Sans KR 한글 글꼴이 포함되어 있으며 앱 시작 전에 자동으로
적용합니다. 이전 버전에서 메뉴가 `ㅁㅁ`로 보였다면 새 ZIP으로 덮어쓴 뒤 다시 실행하면
됩니다. 그래도 깨질 때에는 `sudo apt install -y fonts-noto-cjk`를 실행하십시오.

VS Code에서는 WSL로 폴더를 열고 Run and Debug의 `RMF Desktop Simulator`를 선택한 뒤
F5를 눌러도 됩니다. 창이 나타나지 않으면 `echo $DISPLAY`가 비어 있지 않은지와 WSLg가
활성화되어 있는지 먼저 확인하십시오.

시뮬레이터에서 가능한 작업:

- 드롭다운에서 P3/P4 FAB 맵을 포함한 내장 시나리오 29개와 `scenarios/` 예제 선택
- 캔버스 노드 드래그, 노드 추가·삭제, 두 노드를 선택해 양방향 lane 추가
- 노드의 holding/parking/passthrough/mutex 속성 편집
- lane의 양방향 여부, speed limit, mutex group, 폐쇄 여부 편집
- 물리 Corridor의 forward/reverse Lane, capacity, passing, hard admission, holding entry 편집
- 호환용 V2 lane penalty 편집과 현재 V4 Schedule-aware corridor policy 실험
- 로봇 추가·삭제 및 start/goal node index, yaw, 로봇별 `start_time_s`, `insertion_time_s` 편집
- `신규 로봇 동적 투입` 버튼으로 이전 batch보다 5초 늦은 newcomer 행을 즉시 추가
- `After_nego` 프로필에서 기존 Schedule DB itinerary를 유지한 newcomer-only 협상과 병목 우회 실험
- JSON 파일 열기·저장
- `변경사항 빌드 후 RMF 분석`으로 현재 편집본을 `run.py --scenario-file ... --no-html`에 전달
- 실제 RMF trajectory 재생과 원본 JSONL, 진단 요약, Schedule DB, A* 이벤트 확인
- trajectory의 실제 yaw에 맞춰 회전하는 차동구동 로봇, 진행 노즈·계획 경로·목표 표시
- 로봇별 이동·제자리 회전·대기·도착 상태와 현재 좌표·방향각 표시
- `실행 로그 요약`, `JSONL 요약`, 쉬운 `진단 요약`과 별도 진단 원본
- `스텝별 판단 근거`에서 A* 선택→후보 경로→협상→안전검사→DB commit 흐름 확인
- Schedule DB·A*·판단 표에서 `Ctrl+C` 선택 복사, `Ctrl+Shift+C` 전체 TSV 복사
- 상단 `사용법` 버튼에서 위 편집·실행 순서를 언제든 다시 확인
- 마우스 휠 중심 확대·축소, `+`, `−`, `100%`, `화면 맞춤`
- 가운데 마우스 버튼 드래그로 큰 맵 이동
- 기본 실행 시 최대화된 큰 창으로 시작하고, `창 최대화/복원` 또는 창 테두리로 크기 조절
- 지도-오른쪽 속성 패널과 지도-하단 결과 사이의 분할선을 드래그해 영역 비율 조절
- 위아래 분할선은 14px 파란 막대로 표시되며, 오른쪽 패널의 큰 form이 지도 최소 높이를 강제하지 않도록 세로 크기 정책 분리
- `지도 넓게`로 오른쪽 패널과 하단 결과를 줄이고, `하단 결과 접기/펼치기`로 지도를 창 아래까지 사용
- 실시간 목표·이동·경로·협상·DB 판단 카드는 오른쪽 상단의 `판단 접기/펼치기`로 별도 제어
- 하단 높이를 늘리면 Schedule DB·A*·판단 표도 세로로 같이 늘어나 더 많은 행 표시
- 궤적 재생은 0.25x, 0.5x, 1x, 2x, 4x, 8x 배속 지원(원본 RMF 궤적 시간은 불변)
- 종료 시 창 크기와 두 분할선 위치를 저장하여 다음 실행에서 그대로 복원

실행 중에도 `실행 로그`는 CMake/C++ 프로세스 출력을 그대로 이어 붙이고 바로 옆
`실행 로그 요약`은 빌드·계획·협상 상태를 핵심 문장으로 줄입니다. `원본 JSONL`,
`JSONL 요약`, `Schedule Database`, `A* 내부 과정`, `스텝별 판단 근거`는 250 ms마다
flush된 실제 이벤트를 다시 읽어 갱신합니다. 한글 설명은 원본을 대체하지 않으며 모든
설명은 동일한 `seq`의 JSONL 값과 연결됩니다.

첫 실행은 `변경사항 다시 빌드`를 켜 두십시오. 이후 C++ 소스를 바꾸지 않았다면 체크를 끄면 기존
실행 파일을 재사용하고 버튼 문구도 `빌드된 RMF로 계획 분석`으로 바뀝니다. `setup.bash` 기본값은 `~/rmf_ws/install/setup.bash`이며 본인
워크스페이스 위치가 다르면 상단 입력란만 바꾸면 됩니다.

PySide6 실행 때 `libEGL.so.1` 또는 `xcb` 관련 오류가 뜨면 WSL Ubuntu에서 다음 시스템
라이브러리를 설치한 뒤 다시 실행하십시오.

```bash
sudo apt update
sudo apt install -y libegl1 libgl1 libxkbcommon-x11-0 libxcb-cursor0
```

이 화면은 2D Traffic Core 실험기이며 Gazebo 물리 시뮬레이터는 아닙니다. 캔버스에서
보이는 이동 좌표는 UI가 임의로 만든 것이 아니라 결과 JSONL의 실제
`Plan::get_itinerary()` trajectory의 x·y·yaw를 보간한 값입니다. 로봇 앞쪽 흰색 노즈가
현재 yaw 방향이고, 점선은 선택된 계획 경로, 원은 목표 위치입니다. 협상에 실패하면 안전하지 않은
free-flow 비교 궤적을 실행 계획처럼 재생하지 않고 시작점에 정지시킵니다.
VehicleTraits는 `reversible=false`로 구성되어 후진하지 않습니다. 반대 방향 Lane으로
진입해야 할 때에는 제자리 회전으로 yaw를 맞춘 뒤 로봇 전방으로 주행합니다.

### SCHEDULE_SOFT · Schedule Snapshot 기반 Soft Cost

`SCHEDULE_SOFT`는 기존 OLD_SOFT 위에 누적하지 않고 BASELINE `rmf_traffic` source에서
독립 workspace(`~/rmf_ws_schedule_soft`)로 분기합니다. Planner/Replan 시작 시 실제
Schedule Database를 한 번 조회해 해당 planning job 동안 고정된 snapshot을 만들고,
self participant를 제외한 실제 `SCHEDULE` trajectory interval과 candidate corridor의
시간 중첩에 대해서만 작은 비용을 더합니다.

```text
new_g = original_rmf_g + min(lambda * overlap_duration * direction_weight, max_penalty)
```

- OLD_SOFT의 free-flow/POLICY_DERIVED/static/no_escape cost는 `schedule_soft` 모드에서 사용하지 않습니다.
- `lambda=0`이면 policy hook을 비활성화해 BASELINE 동일성 확인에 사용합니다.
- A* expansion마다 DB 전체를 재조회하지 않습니다. Plan 시작 시 `Database::query(query_all)` 1회 후 물리 corridor별로 index된 snapshot만 조회합니다.
- Replan 시 자기 자신의 기존 itinerary는 snapshot 생성 단계와 cost 계산 단계 모두에서 제외합니다.
- Delay는 지원되는 RMF에서 실제 `Participant::delay(Duration)`를 호출합니다. Schedule update가 자동으로 모든 기존 로봇의 Replan을 의미하지는 않습니다. 다음 명시적 planning job은 새 snapshot을 읽습니다.
- Simulator의 `SCHEDULE_SOFT 활성` 체크를 끄면 lambda=0으로 실행됩니다.

`SCHEDULE_SOFT 코어 준비 (BASELINE에서 분기)` 버튼은 OLD_SOFT patch marker가 있는 source를
BASELINE으로 사용하려 하면 중단하고, 다른 용도로 이미 존재하는 전용 workspace를 자동 덮어쓰지 않습니다.

### 5-Mode Regression Test

`5-Mode 비교 / Regression` 탭은 화면용 가상 결과를 만들지 않습니다. 선택한 Scenario JSON을
한 번만 동결한 뒤 기존 `run.py`와 C++ `rmf_core_lab`을 다음 순서로 실행합니다.

1. `baseline`: 원본 workspace의 `librmf_traffic`
2. `soft`: 수정 workspace의 Schedule 겹침 soft g-cost
3. `hybrid`: 같은 수정 library의 soft cost + 반대방향 corridor hard admission
4. `hybrid_nego`: 별도 workspace의 hybrid 정책 + 동적 newcomer `after_nego`

SOFT와 HYBRID는 의도적으로 같은 수정 binary를 사용하고
`RMF_TRAFFIC_LAB_POLICY_MODE`만 바꿉니다. HYBRID+NEGO는 별도 setup/install을 사용합니다.
정적 all-at-once Scenario에서는 newcomer stage가 없으므로 HYBRID와 HYBRID+NEGO가 같을 수
있으며, 이는 가짜 차이를 만들지 않는 정상 결과입니다.

실행 전 `RMF Core` 탭에서 SOFT/HYBRID 코어와 HYBRID+NEGO 코어를 준비하고 setup/source
경로를 확인합니다. 그다음:

- 현재 편집 중인 한 Scenario: `Compare All Versions`
- 등록된 전체 29개 Scenario: `Run All Scenarios`
- 장시간 실행 중단: `Regression 중지`

각 Scenario는 Map, node/lane/corridor, robot start/goal, start/insertion time, VehicleTraits의
속도·가속도·회전속도·radius, random seed를 동일 JSON/SHA로 재사용합니다. profile별 별도 CMake
build directory를 사용해 library cache 혼입을 막고, JSONL의 `runner_core_profile`에서 실제
`librmf_traffic`, source commit/dirty/diff SHA, policy mode를 다시 확인합니다. `summary.json`의
`identical_input=false` 또는 `core_provenance.verified=false`이면 비교 조건이 검증되지 않은
것이므로 결과 해석 전에 setup 경로를 수정해야 합니다.

표와 파일에는 실제 JSONL로부터 다음 값을 집계합니다.

- 최종 `solution_diagnosis`: SUCCESS/NO_SOLUTION과 termination reason
- `DetectConflict`/`safety_verification`: conflict/deadlock
- 최종 `plan_waypoint`/`plan_summary`: 총 주행시간, makespan, 대기시간, 거리, 우회거리, 최종 Lane 경로
- `planner_timing`/`astar_trace_summary`: planning time, expanded node 수
- `negotiation_summary`/실제 negotiation log: 협상 횟수·시간·round 수
- `ScheduleRouteValidator`와 policy trace: validator reject, hard block, 관측 penalty 합계

공개 RMF Result가 노출하지 않는 내부 validator 호출별 원인은 생성하지 않습니다. 확인 가능한
진단 category만 `NO_VALID_ROUTE`, `SATURATION_LIMIT`, `VALIDATOR_REJECT`,
`NEGOTIATION_FAILED`, `CONFLICT_DETECTED`, `SEARCH_EXHAUSTED`, `RUNNER_TIMEOUT` 등으로
정규화하며 원래 category와 JSONL은 그대로 보관합니다.

Baseline이 안전한 SUCCESS인데 수정본이 NO_SOLUTION, Deadlock 또는 Conflict이면
`REGRESSION`입니다. Baseline이 실패/Deadlock/Conflict이고 수정본이 충돌 없는 SUCCESS이면
`IMPROVEMENT`입니다. 나머지는 `NO_CHANGE`이며, 시간·거리·대기·탐색량은 표에서 수치로 직접
비교합니다.

결과 저장 구조:

```text
results/regression/<run-id>/
├── summary.json
├── summary.csv
└── 000_<scenario>/
    ├── input.json
    ├── baseline.jsonl / baseline.log
    ├── soft.jsonl / soft.log
    ├── hybrid.jsonl / hybrid.log
    └── hybrid_nego.jsonl / hybrid_nego.log
```

향후 반복 stress test는 `tools/stress_scenarios.py`의
`generate_grid_stress(size, robot_count, seed, random_start_time_max_s=...)`를 사용합니다.
3x3/5x5/10x10, 로봇 2/3/5/10대, 임의 start/goal/start time을 seed로 완전히 재현하도록
분리되어 있으며, 생성한 document를 같은 regression config의 네 profile에 그대로 전달하면 됩니다.

### P3 / P4 FAB 대형 맵

- P4: 가로 aisle당 24개 node, 세로 연결축 9개, 각 상·중/중·하 구간 중간 holding node
  3개, parking pocket 15개, 로봇 10대
- P3: 가로 aisle당 28개 node, 세로 연결축 10개, 각 상·중/중·하 구간 중간 holding node
  4개, parking pocket 15개, 로봇 12대

주 통로 node는 passthrough이고 대기는 세로 connector holding node 또는 통로 밖 parking
pocket에서 수행합니다. 두 맵 모두 세로 연결축이 9개 이상이라 장거리 우회와 병목 분산을
비교할 수 있습니다.

### 스텝별 판단 근거

새 탭은 JSONL을 시간순으로 다음 단계로 재구성합니다.

1. Planner 요청과 출발·목적 노드
2. 실제 `Planner::Debug` frontier에서 확장 노드를 선택한 이유
3. 생성된 자식의 g/h/f와 부모 대비 Δg/Δh/Δf
4. 단순 경로 후보별 실제 RMF 강제계획 비용과 최종 선택 여부
5. `CentralizedNegotiation.log(true)`의 table 선택·plan 제출·거부·skip 원문과 요약
6. `DetectConflict::between` 안전검사 결과
7. `Participant::set`과 DB version 변화, commit별 DB 스냅샷
8. 최종 해 진단과 권장 맵·로봇 변경

재생 중에는 현재 시간에서 다음으로 도달할 `plan_waypoint`의 판단 행을 자동 선택합니다.
지도 오른쪽 실시간 판단 카드에는 robot·node·approach lane뿐 아니라 이동 종류, Δ시간·거리·yaw,
최종 Lane과 비용, 후보 순위·차순위 비용차, 자유경로 대비 우회/시간조정, 협상·안전검사,
Schedule DB participant·plan·itinerary·DB version을 함께 표시합니다.

## 시나리오

터미널에서 전체 목록만 보려면 빌드 없이 다음을 실행합니다.

```bash
python3 run.py --list-scenarios
```

| 순서 | 이름 | 로봇 | 목적 |
|---:|---|---:|---|
| 1 | `single_lane_bidirectional` | 2 | 하나의 양방향 1차선에서 한 대가 외부 staging에 대기 후 순차 통과 |
| 2 | `single_path` | 1 | 짧은 중앙 경로와 긴 우회로의 실제 비용 비교 |
| 3 | `single_path_closed` | 1 | 중앙 lane 4·5 폐쇄 후 우회 여부 |
| 4 | `speed_limit_choice` | 1 | 짧지만 느린 길과 길지만 빠른 길 비교 |
| 5 | `single_path_multi` | 2 | 같은 그래프 양 끝 로봇의 경로·시간 협상 |
| 6 | `occupied_corridor_detour` | 2 | 원본 RMF 경로 중첩을 자동 검출해 대체 경로가 있는 로봇을 우회시키는 AFTER 기준 시나리오 |
| 7 | `head_on` | 2 | 대피 공간 없는 단일 복도의 기준 실패 사례 |
| 8 | `passing_bay` | 2 | 우회 bay를 추가했을 때 결과 비교 |
| 9 | `t_junction` | 3 | 세 로봇의 T자 교차로 경쟁 |
| 10 | `cross_intersection` | 4 | 네 방향 로봇의 공용 교차로 협상 |
| 11 | `disconnected` | 1 | start와 goal이 분리된 그래프의 실패 분류 |
| 12 | `staggered_departures` | 3 | 두 대는 0초, 교차하는 세 번째 로봇은 8초 뒤 출발 |
| 13 | `grid_3x3_multi` | 4 | 9개 노드 격자에서 대각 교환과 교차 협상 |
| 14 | `grid_5x5_multi` | 6 | 25개 노드·40개 양방향 원본 lane의 다중 로봇 스트레스 |
| 15 | `grid_10x10_multi` | 8 | 100개 노드·180개 원본 lane의 대규모 탐색·협상 스트레스 |
| 16 | `dynamic_bottleneck_insertion` | 4 | 초기 2대 commit 후 8초·14초에 신규 2대 투입, 중앙 병목과 상·하부 우회로 비교 |
| 17 | `dynamic_grid_5x5_insertion` | 8 | 5x5 mesh에서 초기 4대 뒤 신규 4대를 6·9·12·15초에 순차 투입 |

5×5와 10×10은 `CentralizedNegotiation` 조합 수가 빠르게 증가할 수 있는 의도적인
스트레스 시나리오입니다. 먼저 timeout을 180초 이상으로 올려 보고, 느리면 로봇을 한 대씩
줄여 최소 병목 조합을 찾으십시오. 캔버스에서 로봇을 더 추가할 수도 있습니다.

`head_on`의 해 없음은 자동으로 planner 버그를 뜻하지 않습니다. 그래프에 물리적으로 통과 가능한 topology가 없기 때문입니다. `passing_bay`와 비교해서 topology 변경이 협상 결과에 미치는 영향을 보십시오.

`staggered_departures`의 `start_time_s`는 세 요청을 계획 전에 모두 알고 있는 상태에서 각
RMF `Start`의 시각만 다르게 넣습니다. 실행 중 8초에 새 작업이 들어오는 동적 task insertion,
기존 itinerary 진행도 갱신과 재협상까지 모사하는 것은 아닙니다. 미래 `Start` 이전의 정지
점유는 최종 itinerary에 포함되지 않으므로, 예제의 지연 로봇은 공용 경로 밖
`DELAYED_STAGING` parking 노드에서 시작합니다.

### Center에 붙인 대피로가 잘 선택되지 않는 이유

`single_path_multi`의 중앙 병목은 `LEFT_GATE(1) - CENTER(2) - RIGHT_GATE(3)`입니다.
`1 - BAY - 3`처럼 병목 진입 전과 이탈 후를 직접 잇는 branch는 중앙 전체를 우회하는 독립
경로가 됩니다. 반면 `1 - BAY - 2` 또는 `2 - BAY - 3`은 중앙의 절반만 우회하고 결국 두
로봇이 `CENTER(2)`나 나머지 공용 lane에서 곧바로 다시 합류합니다. 이 짧은 loop도 물리 폭이
충분하고 접속점 점유 시각을 분리할 수 있으면 이론상 작은 교행 구간이 될 수 있지만, 현재 좌표·로봇
반경·공용 구간에서는 독립적인 회피 이득이 부족합니다. 한 노드에만 붙인 막다른 branch는 목적지로
이어지는 simple path가 아니며, Planner가 피난 행동을 위해 들어갔다 같은 노드로 되돌아오는 경로를
항상 생성해 준다고 가정하면 안 됩니다.

대피로가 실제 교행 공간이 되려면 병목의 서로 다른 양쪽 노드에 연결된 cycle이어야 하고,
branch 내부에 `holding:true`인 노드가 있어야 합니다. 메인 경로와 branch 사이의 실제 좌표
간격도 로봇 직경보다 충분해야 하며, 같은 mutex group을 양쪽 경로에 부여하면 별도 경로여도
동시에 점유할 수 없습니다.

### 세 번째 로봇과 jam/no-solution의 관계

RMF가 단순히 “jam이 예상된다”는 이유 하나로 바로 포기하는 것은 아닙니다. 알고 있는 모든 요청에
대해 경로와 시각을 함께 바꾸어 충돌 없는 proposal을 찾습니다. 그래도 proposal이 없으면 현재
topology, holding 위치, 로봇 footprint, 출발 시각, negotiator 비용·탐색 한도 안에서 실행 가능한
공동 계획을 찾지 못했다는 뜻입니다. 이것만으로 곧바로 RMF 버그라고 판정할 수는 없습니다.

정면의 두 로봇 뒤로 세 번째 로봇이 들어와 유일한 후퇴·대기 노드를 차지하면 실제 jam이 될 수
있습니다. 세 요청을 처음부터 알고 있으면 RMF가 세 번째 로봇의 진입을 늦추거나 우회하는 proposal을
시도합니다. 반대로 두 로봇이 이미 복도에 들어간 뒤 새 task가 동적으로 들어오면 schedule progress,
새 participant/task 반영, 재계획·재협상, 복도 진입 제어가 모두 필요합니다. 이 실험의
`staggered_departures`는 전자의 “미리 알려진 0/0/8초 요청”만 다룹니다.

이미 두 로봇이 nose-to-nose이고 후진도 금지된 뒤에는 Traffic Core가 물리적으로 로봇을 꺼낼 수
없습니다. 복도 양쪽 외부 holding gate, 비어 있는 passing bay, corridor mutex/token, 한 방향 통과
phase, 세 번째 로봇 admission control로 진입 전에 막는 구성이 필요합니다. 이미 발생한 jam은 Fleet
Adapter 또는 현장 복구 절차가 담당해야 합니다.

## 커스텀 맵·로봇·시나리오

`scenarios/custom_no_solution.json`과 `scenarios/custom_with_passing_bay.json`을 복사해서 수정할 수 있습니다. C++ 코드를 다시 작성할 필요는 없습니다.

```bash
python3 run.py \
  --setup ~/rmf_ws/install/setup.bash \
  --scenario-file scenarios/custom_no_solution.json \
  --timeout 60 \
  --open
```

JSON 구성:

```json
{
  "name": "my_scenario",
  "map": "L1",
  "mode": "auto",
  "nodes": [
    {"name": "A", "x": 0, "y": 0, "holding": true, "parking": true},
    {"name": "B", "x": 2, "y": 0, "passthrough": true}
  ],
  "lanes": [
    {"from": 0, "to": 1, "bidirectional": true, "mutex_group": "corridor"}
  ],
  "robots": [
    {"name": "R0", "start": 0, "goal": 1, "yaw": 0.0, "start_time_s": 0.0}
  ],
  "closed_lanes": []
}
```

- 로봇 제거/추가: `robots` 배열의 객체를 삭제/추가합니다.
- 로봇별 출발 지연: `start_time_s`를 초 단위로 지정합니다. 생략하면 0초입니다. 지연 로봇은
  공용 Lane 밖 staging/parking 노드에서 시작하는 구성을 권장합니다.
- 노드 추가: `nodes` 배열 끝에 추가하고 새 index를 lane에서 사용합니다.
- 회피공간 추가: side node를 복도의 서로 다른 두 노드에 연결해야 실제 passing loop가 됩니다. 막다른 노드 하나만 붙이면 교행 해가 생기지 않을 수 있습니다.
- 양방향 lane: `bidirectional:true`는 두 directed lane으로 확장됩니다.
- 대기 가능 지점: 병목 진입 전 노드나 bay에 `holding:true`를 지정합니다.
- lane 폐쇄: 원본 lane 전체는 `closed:true`, 확장된 특정 directed lane은 `closed_lanes` ID로 닫습니다.
- `mode:auto`: 로봇 1대면 free-flow Planner 결과를 실제 Schedule DB에 commit하고,
  2대 이상이면 CentralizedNegotiation·충돌검증을 거친 뒤 Schedule DB에 commit합니다.

입력은 Python에서 타입·index·중복 이름·directed reachability를 검사한 뒤 결정적인 중간 형식으로 변환합니다. 경로·협상·충돌검사는 계속 실제 C++ RMF Traffic Core가 수행합니다.

## 실행

WSL Ubuntu에서:

```bash
cd ~/rmf_traffic_lab
python3 run.py \
  --setup ~/rmf_ws/install/setup.bash \
  --scenario single_lane_bidirectional \
  --timeout 60 \
  --open
```

다음 실험은 시나리오 이름만 바꿉니다.

```bash
python3 run.py --setup ~/rmf_ws/install/setup.bash --scenario single_lane_bidirectional --timeout 60 --open
python3 run.py --setup ~/rmf_ws/install/setup.bash --scenario single_path_closed --open
python3 run.py --setup ~/rmf_ws/install/setup.bash --scenario speed_limit_choice --open
python3 run.py --setup ~/rmf_ws/install/setup.bash --scenario single_path_multi --timeout 60 --open
python3 run.py --setup ~/rmf_ws/install/setup.bash --scenario head_on --timeout 60 --open
python3 run.py --setup ~/rmf_ws/install/setup.bash --scenario passing_bay --timeout 60 --open
python3 run.py --setup ~/rmf_ws/install/setup.bash --scenario t_junction --timeout 60 --open
python3 run.py --setup ~/rmf_ws/install/setup.bash --scenario cross_intersection --timeout 60 --open
python3 run.py --setup ~/rmf_ws/install/setup.bash --scenario disconnected --open
python3 run.py --setup ~/rmf_ws/install/setup.bash --scenario staggered_departures --timeout 60 --open
```

한 번 빌드한 뒤 C++ 코드를 바꾸지 않았다면 `--skip-build`로 CMake 단계를 생략할 수 있습니다.

```bash
python3 run.py --setup ~/rmf_ws/install/setup.bash --scenario single_path --skip-build --open
```

VS Code에서는 WSL로 폴더를 연 다음 Run and Debug에서 `RMF 1`부터 `RMF 10`까지 선택하고 F5를 누르면 됩니다.

## UI에서 보는 정보

### 시뮬레이션

기본값은 실행 가능한 계획만 재생합니다. 다중 로봇 proposal은 모든 route 쌍이 `DetectConflict::between`을 통과해야 Schedule DB에 commit되고 `negotiated` 실행으로 표시됩니다. 협상 실패 또는 충돌검증 실패 시 로봇은 시작 위치에 정지합니다.

`⚠ 충돌 미검증 free-flow 비교`는 사용자가 직접 선택할 때만 재생됩니다. 이 모드는 각 로봇을 독립적으로 계획한 비교 자료이므로 서로 겹칠 수 있으며, RMF가 승인한 traffic plan이 아닙니다.

재생 위치는 RMF trajectory의 시각 사이를 보간해 표시합니다. 지도 오른쪽 실시간 판단 카드에서
현재 이동/회전/대기 상태, 다음 waypoint, 후보 비용, 협상 조정, 안전검사와 DB 반영 상태를
확인할 수 있습니다.

### A* 검색

`Planner::Debug`의 실제 search node를 한 step씩 보거나 `검색 재생`으로 자동 진행합니다.

- `g(n)`: 시작부터 현재 노드까지 누적된 실제 비용
- `h(n)`: goal까지의 RMF 남은 비용 추정
- `f(n)=g(n)+h(n)`: frontier 우선순위
- 부모 구간 route의 실제 시간과 이동·회전·대기 분류, 거리·회전각, `Δg-구간시간` 미노출 잔차
- h 검산용 방향 그래프 최단거리·순수 주행시간·첫 Lane 회전시간·RMF h와의 차이
- node ID, parent ID, waypoint, orientation, queue 크기
- expansion 순서와 생성된 child 수, terminal 수

이 API는 Open-RMF가 명시한 debug-only 불안정 API입니다. 학습과 회귀 분석에는 유용하지만 production 코드가 의존하면 안 됩니다. 공개 Debug API에는 모든 child branch의 정확한 rejection reason이 없습니다. 그 수준이 필요하면 다음 단계에서 RMF validator 내부를 source instrumentation 해야 합니다.

### 경로 선택 이유

작은 그래프의 모든 simple directed path를 최대 64개 열거합니다. 각 후보만 남기고 다른 lane을 닫은 `Planner`를 실제로 다시 실행해서 RMF cost, 도착 시간, 거리와 최저 후보 대비 차이를 기록합니다. 이 표는 경로 선택을 설명하기 위한 실험용 교차 검증이며 RMF 내부 A* 구현을 대체하지 않습니다.

### Navigation Graph

이 실험에서 “navigation DB”는 SQL DB가 아니라 메모리의 `rmf_traffic::agv::Graph`입니다.

- waypoint: map, 좌표, holding/passthrough/parking/charger, mutex group, merge radius, incoming/outgoing lanes
- directed lane: entry, exit, 길이, speed limit, effective speed, mutex group, closure
- vehicle traits: footprint radius, 선속도·가속도, 각속도·가속도, steering, reversible
- planner options: minimum holding time, saturation limit, traversal cost per meter, closure

### Schedule Database

`schedule::Database`는 navigation graph와 별개입니다. participant description과 시간축 itinerary를 저장합니다. 다중 로봇 시나리오는 등록 직후와 proposal commit 후의 DB version, participant ID, itinerary version, route 수, trajectory point 수를 기록합니다. pairwise conflict 표에는 실제 `DetectConflict` 검사 결과와 충돌 시각이 표시됩니다.

데스크톱 앱의 Schedule Database 탭은 Planner 결과를 Python에서 재구성하지 않습니다.
C++이 실제 `rmf_traffic::schedule::Database::query(query_all())`,
`get_participant()`, `get_itinerary()`를 호출해 현재 DB에 저장된 값을 다시 읽습니다.
각 `Participant::set()` 직후 snapshot을 남기므로 `commit_1_of_N`부터 route가 순서대로
누적되는 모습을 확인할 수 있습니다.

전용 `Schedule Database` 탭은 다음을 snapshot 단위로 표시합니다.

- 실제 DB API operation: construct, participant registration, negotiation read, `Participant::set`
- operation 직전·직후 DB version과 itinerary version
- participant ID, owner, responsiveness, progress version
- 저장된 itinerary route의 map, 시작·종료·duration
- 모든 time-parameterized trajectory point의 pose와 velocity
- `Database → ParticipantDescription → Itinerary → Route → Trajectory → Waypoint` 실제 객체 계층과 조회 API
- 안전검증 실패 또는 no proposal로 DB write를 건너뛴 이유

오른쪽 `용어·버전 가이드`에는 DB version, participant ID, plan ID, itinerary/progress version,
Route와 Trajectory point의 의미를 적었습니다. `실제 RMF 객체 구조` 탭에는 각 표가 어떤
C++ 객체·API에서 나온 값인지와 현재 생략되는 내부 index/history 범위를 구분했습니다. 각 표의 행을 누르면 C++에서 읽힌 실제 값을
별도의 `선택 행 해석` 영역에서 설명합니다. 왼쪽 표를 넓게 유지해 긴 trajectory를 스크롤할 수 있고,
`Ctrl+C`는 선택 셀, `Ctrl+Shift+C`는 헤더를 포함한 전체 표를
TSV로 복사하므로 Excel·메모장·Confluence에 바로 붙여 넣을 수 있습니다.

`협상·안전` 탭은 CentralizedNegotiation 로그와 `DetectConflict::between` 결과를 분리해 표시합니다.

## 동적 신규 로봇 투입과 After_nego

`start_time_s`와 `insertion_time_s`는 다릅니다.

- `start_time_s`: 생성할 trajectory가 시작할 수 있는 가장 이른 시각
- `insertion_time_s`: 해당 로봇 participant와 작업 요청을 실제 Schedule Database에 처음 넣는 시각

`dynamic_insertion:true`이거나 로봇 하나라도 `insertion_time_s>0`이면 C++ runner가 한 번의
all-at-once 협상 대신 stage 방식으로 실행합니다. 하나의 실제
`rmf_traffic::schedule::Database`를 끝까지 유지하며, 각 stage에서 다음 순서로 처리합니다.

1. 해당 시각의 신규 participant만 `make_participant`로 등록
2. 이미 commit된 participant itinerary를 DB에 그대로 유지
3. 신규 batch의 원본 free-flow 기준 경로를 실제 RMF Planner로 계산
4. 신규 participant만 `CentralizedNegotiation(database).solve(newcomer_agents)`에 전달
5. 기존 plan과 신규 proposal을 합쳐 `DetectConflict::between`으로 다시 안전검사
6. 통과한 신규 itinerary만 `Participant::set`으로 같은 DB에 추가

`BASELINE`은 이 stage 구조만 사용하고 기본 RMF 비용을 바꾸지 않습니다. `SOFT`는 기존
Schedule itinerary와 candidate의 corridor 통과시간이 실제로 겹친 초에 방향별 가중치를 곱해
`g`에만 더합니다. `HYBRID`는 여기에 non-passing corridor의 반대방향 신규 entry hard admission을
추가합니다. `HYBRID + NEGO`는 같은 정책과 기존 newcomer-only 우회 leeway 실험을 함께 사용합니다.

Lane을 닫는 정책이 아니므로 SOFT에서는 대체 경로가 없어도 유한한 비용으로 병목 Lane을 사용할
수 있습니다. HYBRID의 hard block도 corridor 안에 이미 있는 로봇의 continuation/exit에는 적용하지
않습니다. 신규-only 협상으로 해가 없으면 자동으로 기존 전체를 다시 짜지 않고 JSONL에
`dynamic_newcomer_no_proposal`을 남깁니다. 실제 운영 적용에서는 선택적 replan, task 지연,
입구 대기 중 하나로 escalation하는 정책이 별도로 필요합니다.

가장 간단한 비교 순서:

1. `dynamic_bottleneck_insertion` 선택
2. `BASELINE / BEFORE` 실행
3. RMF Core 탭에서 `Schedule-aware 코어 준비` 클릭
4. `SOFT`, `HYBRID`, `HYBRID + NEGO`를 차례로 실행
5. `5-Mode 비교` 탭에서 신규 로봇 Lane, 성공 stage 수, hard/validator 판정, DB route를 비교

직접 편집할 때에는 로봇 표의 `동적 투입(s)`에 0, 0, 8, 14처럼 입력합니다. 버튼으로
신규 로봇을 추가하면 현재 가장 늦은 투입 시각보다 5초 뒤로 자동 입력되며 start/goal은 수정할
수 있습니다. 우회로는 병목 입구와 출구를 실제로 연결하는 별도 loop여야 합니다.

## Schedule-aware Corridor Policy V4

정확한 A/B 비교를 위해 기본 RMF와 수정 RMF를 서로 다른 workspace/install에 둡니다.
`setup.bash` 파일을 수정하는 것이 아니라, 실행 전에 어느 install의 공유 라이브러리를 찾을지
선택하는 용도입니다.

```text
~/rmf_ws/                    # BASELINE: 원본 source/install
~/rmf_ws_modified/           # SOFT/HYBRID: 복사·수정 source/install
```

`python3 tools/setup_after_core.py` 또는 앱의 `Schedule-aware 코어 준비`는 다음을 수행합니다.

1. 원본 source를 수정 workspace로 복사합니다.
2. 실제 `DifferentialDrivePlanner.cpp`의 `ScheduledDifferentialDriveExpander::expand_traversal`
   child 생성 지점에 내부 policy hook을 추가합니다.
3. 실제 `SimpleNegotiator.cpp::respond`에 현재 협상 participant scope만 추가합니다.
4. 같은 internal 디렉터리에 `RmfLabCorridorPolicy.hpp`를 생성합니다.
5. 공개 RMF header와 exported symbol은 바꾸지 않습니다.

### 네 가지 비교 모드

| 모드 | 동적 soft cost | 반대방향 hard admission | RMF Validator/Negotiation | 추가 실험 |
|---|---:|---:|---|---|
| BASELINE / BEFORE | 없음 | 없음 | 원본 그대로 | 원본 source/install |
| SOFT | 있음 | 없음 | 원본 유지 | cost만으로 우회 확인 |
| HYBRID | 있음 | 있음 | 원본 유지 | FAB 권장 정책 |
| HYBRID + NEGO | 있음 | 있음 | 원본 유지 | 기존 newcomer-only detour leeway 추가 |

### Schedule 조회와 비용식

각 planning/negotiation invocation 직전에 C++ runner가 실제
`Database::query(schedule::query_all())`을 한 번 호출합니다. `Viewer::View::Element`의
participant/plan/route와 실제 `Route::trajectory()` timestamp를 Corridor의 directed Lane에
연결해 다음과 같은 고정 index를 만듭니다. 탐색 도중 Schedule version을 바꾸거나 A* node마다
DB 전체를 다시 순회하지 않습니다.

```text
C1 = [{participant, plan, route, direction, enter, exit, state, owner}, ...]

actual_overlap = max(0, min(candidate_exit, occupied_exit)
                        - max(candidate_enter, occupied_enter))

same_penalty     = actual_overlap_same × same_weight
opposite_penalty = actual_overlap_opposite × opposite_weight
occupancy        = actual_overlap × occupied_or_future_weight
policy_penalty   = corridor_base + static + same + opposite + occupancy + no_escape

new_g = parent_g + RMF approach_cost + RMF event_cost + RMF alt_cost
        + policy_penalty
f = new_g + 원본 RMF h
```

`overlap_margin`은 hard admission의 시간 안전창에만 사용하며 UI에는 실제 겹침 초와 분리해
표시합니다. penalty는 ranking용 비용입니다. candidate의 실제 trajectory timestamp와 h는
수정하지 않습니다. 모든 weight와 corridor base가 0인 SOFT는 BASELINE과 같은 g/h/f가 되도록
테스트합니다. 하나의 traversal이 같은 physical corridor의 Lane 여러 개를 지나거나 corridor
내부에서 이어지는 경우에는 한 번만 과금합니다.

### Corridor hard admission과 release

HYBRID 계열에서 다음 조건이 모두 참인 child만 `HARD_CORRIDOR_BLOCK`으로 제거합니다.

- candidate가 corridor의 새로운 entry임
- `passing_allowed=false`, `capacity<=1`, hard policy가 켜짐
- candidate와 반대 방향의 실제 Schedule interval이 admission safety window에서 겹침
- 그 반대 방향 interval이 현재 `OCCUPIED`/`UNKNOWN_HOLD`이거나 deterministic owner임

이미 corridor 안에 있는 participant는 자신의 Schedule 상태로 다시 식별해 approach Lane 정보가
없더라도 exit/continuation을 항상 허용합니다. owner는 현재 occupant/unknown을 우선하고, 그다음
예상 진입시각과 participant ID 순으로 하나만 결정합니다. 따라서 양쪽이 서로를 owner로 보고 둘 다
막는 대칭 교착을 만들지 않습니다.

상태는 `FREE → RESERVED → OCCUPIED → FREE`로 표시됩니다. 예상 exit 시간이 지났다는 이유만으로
release하지 않습니다. 실제 Fleet Adapter 진행정보가 없는 lab에서는 명시적
`CHECKPOINT_RELEASE` event를 출구 확인으로 사용합니다. 통신 끊김은 `UNKNOWN_HOLD`로 유지해
낙관적으로 FREE가 되지 않습니다. production에서는 이 event를 Adapter의 실제 reached/location
callback으로 교체해야 합니다.

### Validator 경계

- Corridor Admission은 custom 정책이며 source=`POLICY_DERIVED`입니다.
- 제안 경로는 실제 `ScheduleRouteValidator::find_conflict(Route)`로 다시 검사하고 source를
  `RMF_CORE/SCHEDULE`로 남깁니다.
- 협상 중 제약은 수정하지 않은 `SimpleNegotiator` 흐름의 실제 `NegotiatingRouteValidator`가
  처리합니다. public Result가 내부 모든 호출을 노출하지 않으므로 생성되지 않은 값을 만들지 않습니다.
- 최종 조합은 기존 `DetectConflict::between`으로 연속시간 profile 충돌을 재검증합니다.

### Delay와 명시적 replan

`DELAY` event가 발생하면 해당 RMF 버전이 지원하는 실제 `Participant::delay(Duration)`를 호출하고,
성공한 경우에만 cumulative delay와 Schedule version 변경을 표시합니다. 다음 명시적 planning
invocation이 이동된 trajectory를 다시 조회합니다. Schedule이 갱신될 때마다 전체 로봇을 주기적으로
replan하지 않습니다. `maximum_delay_exceeded`, lane closure, negotiation, custom material conflict
등 trigger와 이유는 별도 event로 기록됩니다.

### 준비·빌드·직접 실행

```bash
cd ~/rmf_traffic_lab

# 1) 수정 source 준비
python3 tools/setup_after_core.py \
  --before-source ~/rmf_ws/src/rmf_traffic \
  --after-workspace ~/rmf_ws_modified

# 2) 수정 rmf_traffic 실제 빌드
source /opt/ros/jazzy/setup.bash
cd ~/rmf_ws_modified
colcon build --packages-select rmf_traffic \
  --allow-overriding rmf_traffic \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# 3) 데스크톱 앱
source ~/rmf_ws_modified/install/setup.bash
cd ~/rmf_traffic_lab
python3 simulator.py
```

CLI로 HYBRID를 직접 재현할 때:

```bash
cd ~/rmf_traffic_lab
python3 run.py \
  --scenario S1_opposite_1v1 \
  --traffic-mode hybrid \
  --setup ~/rmf_ws_modified/install/setup.bash \
  --rmf-source ~/rmf_ws_modified/src/rmf_traffic \
  --same-direction-weight 0.25 \
  --opposite-direction-weight 8.0 \
  --occupied-weight 1.5 \
  --future-reservation-weight 0.6 \
  --no-escape-weight 25
```

BASELINE은 `--traffic-mode baseline`, 원본 `~/rmf_ws/install/setup.bash`와 원본 source를
사용합니다. S5 delay는 `S5_delay_inside_corridor`, 통신 끊김은
`S6_comms_loss_inside`, release는 `S7_confirmed_release`, weight=0 동등성은
`S10_zero_weight_equivalence`를 선택합니다.

상단 모드별 결과는 `results/gui_baseline.jsonl`, `gui_soft.jsonl`, `gui_hybrid.jsonl`,
`gui_hybrid_nego.jsonl`과 대응 build 디렉터리에 분리됩니다. 각 `runner_core_profile`에는 setup,
실제 `librmf_traffic`, source commit/dirty/diff SHA, scenario SHA가 남습니다. 같은 workspace의
install을 덮어쓰면 엄밀한 A/B가 아니므로 원본·수정 install을 분리하십시오.

### V2 호환 기능

UI의 `Legacy V2`와 수동 Lane penalty는 이전 결과 재현용으로 남겨 두었습니다. 이는 Schedule
timestamp가 아니라 사전 계산한 lane 수요/정적 값이며 V4 Schedule-aware 정책과 동일하지 않습니다.
현재 실험은 상단 네 모드와 Corridor 패널을 사용하십시오.

### 시나리오·RMF 확인

29개 시나리오의 로봇 수, 실제 호출 core, 실험 목적과 기대 결과를 한 표에서 확인합니다. P3/P4 맵은 3개 장거리 주행축, 9개 이상의 다중-node 세로 연결축, 통로 밖 parking pocket을 포함합니다. S1~S10은 hard admission, convoy, deadlock, 우회, delay, 통신 끊김, release, 혼잡, hard-off, weight=0 회귀를 직접 검증합니다. 같은 탭의 API 표는 이 실행 파일이 링크하고 호출하는 `Planner`, `Planner::Debug`, `CentralizedNegotiation`, `ScheduleRouteValidator`, `schedule::Database`, `DetectConflict::between`을 보여줍니다.

맵과 요청은 실험 입력으로 직접 만들지만 cost, A* search node, plan, trajectory, negotiation proposal, schedule write와 conflict 결과는 Python 목업이 아니라 실제 C++ RMF Traffic Core 결과입니다.

### 진단 요약

`solution_diagnosis` 이벤트와 전용 탭에서 다음을 구분합니다.

- `disconnected_topology`: start→goal open directed path 자체가 없음
- `search_saturation`: search limit 전에 해를 증명하지 못함
- `individual_path_missing`: 다중 로봇 중 한 대가 혼자서도 planning 불가
- `endpoint_exchange_without_buffer`: 점유 중인 양 끝점을 단일 경로로 교환하지만 비울 buffer가 없음
- `single_route_no_yield_space`: 모든 로봇이 동일한 유일 경로를 사용하고 회피 topology가 없음
- `continuous_time_overlap`: proposal은 있으나 `DetectConflict`가 footprint 충돌 확인
- `negotiation_no_proposal`: 개별 경로는 있지만 하나의 충돌 없는 time-space proposal로 결합하지 못함
- `runner_timeout`: 제한 시간 안에 최종 결과가 나오지 않음

확정 가능한 RMF result flag와 구조 기반 추론을 `basis`·`confidence`로 구분합니다. 공개 API에 없는 branch rejection reason을 임의로 단정하지 않습니다. 진단 탭은 관찰 근거와 노드·lane·로봇 수를 어떻게 바꿔 검증할지도 함께 표시합니다. 영문 `root_cause`, `basis`, `evidence`, `recommended_actions`는 `진단 원본` 탭에 그대로 보존하고, `진단 요약` 탭에서는 판정·쉬운 설명·관찰 증거·해결 실험 순서로 표시합니다.

### A* 상세 해석과 한계

`astar_step_decision`은 실제 `Planner::Debug` frontier top에서 선택된 node와 선택 시점의
g, h, f, 차순위 node의 f, 두 후보의 f 차이를 기록합니다. 자식 노드는 부모 대비 Δg, Δh,
Δf도 함께 기록합니다. 행을 누르면 “왜 지금 이 node를 확장했는가”를 실제 숫자로 설명합니다.

기본 RMF Debug API가 노출하는 g는 이동·회전·대기·event·validator 영향이 합쳐진 총비용입니다.
v0.14부터 `route_from_parent`의 실제 Trajectory를 구간별로 분석해 이동·회전·대기 시간,
이동거리, 회전각을 함께 기록하고 `Δg-궤적 경과시간`을 미노출 잔차로 표시합니다. h는 실제
`remaining_cost_estimate` 총합을 그대로 보존하면서 동일 방향 그래프에서 Dijkstra 순수 주행 하한,
첫 Lane 정렬 회전 하한과 차이를 별도 표시합니다. 이 진단값은 h 내부 직렬화가 아닙니다.
soft penalty/event별 완전한 g 분해와 QuickestPath h 내부 항목, 생성되지 않은 모든 branch의
정확한 rejection reason은 공개 API에 없으므로 수정본 코어 비용 계산 지점의 instrumentation이 필요합니다.

### 원본 로그

`원본 로그` 탭은 JSONL의 모든 event와 모든 field를 기본값으로 출력합니다. A*, plan, trajectory, Schedule DB operation, `CentralizedNegotiation.log(true)`의 문자열을 요약하지 않습니다. event type과 원본 문자열 검색은 화면 표시용이며 원본 JSONL 파일은 변경하지 않습니다.

### 전체 시퀀스

JSONL의 모든 이벤트에 증가하는 `seq`가 있습니다. Graph 입력 → free-flow A* → Plan 생성 → 후보 비교 → negotiation → schedule commit 흐름을 필터링해 볼 수 있습니다. 프로그램이 중간에 멈춰도 마지막으로 flush된 정상 이벤트까지 남습니다.

### RMF 객체·협상 원문 (v0.16)

`RMF 객체·협상 원문` 탭에서 실행 순서대로 다음 값을 복사·해석할 수 있습니다.

- `Graph·Supergraph`: `Graph::get_waypoint/get_lane`으로 읽은 waypoint·directed Lane과 Planner 내부 `rmf_traffic::agv::planning::Supergraph` 노출 범위
- `Start·Goal`, `Validator`: 실제 `Plan::Start`, `Plan::Goal`, free-flow의 `Planner::Options(nullptr)`, 협상 시 Schedule-aware 제약
- `Itinerary`, `Route`, `Trajectory`: `Plan::get_itinerary()`로부터 얻은 Route map, 시작·종료 시각, pose, velocity
- `Proposal`: `CentralizedNegotiation::Result::proposal()`의 participant별 `Plan`과 검증·DB commit 전후 상태
- `협상 전체 시퀀스`: 협상 요청 → table 선택 → plan 제출 → Reject/Forfeit → Proposal → `DetectConflict` → `Participant::set` 흐름
- `협상 원문 과정`: `.log(true)`로 켠 `CentralizedNegotiation::Result::log()` 문자열을 순서대로 보존
- `Reject·Forfeit`: `Rejected parent`, `Forfeited` 원문과 Proposal 후속 수락·거부 결과를 한 표에서 시간순으로 표시

정확성 경계도 화면에 명시합니다. Graph, Start/Goal, Plan/Itinerary/Route/Trajectory,
Proposal과 협상 원문은 실제 RMF 객체·API에서 읽은 값입니다. 반면 Supergraph는
RMF 소스의 internal header에 있고 public `Planner::Debug` API가 노드·키·캐시를 반환하지
않으므로, 시뮬레이터는 내부 값을 추정해 만들지 않고 노출 한계를 표시합니다. 협상
`Reject`/`Forfeit` action은 실제 원문을 보존한 채 UI에서 문자열로 분류한 것이며,
호출별 validator의 모든 세부 탈락 조건은 공개 Result API에 없습니다.

## 빌드가 짧은 이유

정상입니다. `~/rmf_ws/install` 또는 `/opt/ros/jazzy`의 Open-RMF 라이브러리는 이미 컴파일되어 있습니다. 이 프로젝트는 C++ 파일 하나만 컴파일한 뒤 기존 `rmf_traffic` 공유 라이브러리에 링크합니다. 그래서 최초 CMake configure도 수 초, 재빌드는 더 짧을 수 있습니다. Open-RMF 전체 source workspace를 처음부터 빌드하는 시간과는 다릅니다.

## 결과와 테스트

- `results/<scenario>.jsonl`: 원본 진단 이벤트
- `results/<scenario>.html`: 서버가 필요 없는 분석 UI
- `results/regression/<run-id>/summary.json`: 전체 machine-readable 비교와 provenance
- `results/regression/<run-id>/summary.csv`: Scenario × profile 비교표
- `results/regression/<run-id>/<scenario>/`: 동결 입력, profile별 원본 JSONL과 실행 로그

추후 RMF Traffic 코드를 수정해 before/after를 비교할 때에는 동일 scenario·동일 요청으로 각각의 JSONL을 보관하십시오. 최소 비교 항목은 A* expansion 수, plan/ideal cost, planning·negotiation 시간, proposal 성공 여부, DB version, itinerary wait/finish time, `DetectConflict` 충돌 수입니다. 애니메이션만 비교하면 알고리즘 변화와 렌더링 차이를 구분할 수 없습니다.

```bash
python3 -m unittest discover -s tests -v
cmake --build build
ctest --test-dir build --output-on-failure
```
