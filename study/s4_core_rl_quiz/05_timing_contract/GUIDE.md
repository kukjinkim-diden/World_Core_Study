# 05_timing_contract 상세 설명

## 큰 그림: 정책은 "학습된 조건" 에 묶여 있다

정책은 숫자를 받아 숫자를 내는 함수일 뿐이라, 조건이 틀려도 **에러를 내지 않습니다.** 예를 들면:

- 100 Hz 로 학습한 정책을 1 kHz 로 부르면 → 그냥 돕니다. 다만 history 의 "4 주기 전" 이 40 ms 전이 아니라 4 ms 전이 되고, 로봇은 이상하게 움직입니다.
- 관측이 157 칸이어야 하는데 145 칸만 채우면 → 나머지 12 칸에 쓰레기 값이 들어간 채로 돕니다.
- `joint_action_scale` 을 빠뜨려 0 이 되면 → 정책 출력이 전부 무시되고 기본 자세로 서 있기만 합니다.

Core 의 원칙은 **"조용히 이상하게 도느니 로드 단계에서 거절한다"** 입니다. 이 퀴즈는 그 거절 판정들을 만듭니다.

| 문항 | 묶는 조건 | 어긋나면 |
|---|---|---|
| Q1 | 정책 주기 = `decimation × physics_dt` | 로드 거절 |
| Q2 · Q3 | 1 kHz tick 위에서 그 주기를 지킨다 | tick 보다 빠른 주기는 활성화 거절 |
| Q4 | 관측·행동의 차원 = ONNX 의 입출력 차원 | 로드 거절 |
| Q5 | 중복 term, 게인, 정책 집합의 주기 | 로드 거절 |

각 문항은 **배경 → 테스트 따라가기 → 단계별 힌트 → 흔한 실수 → 레포 연결** 순서로 설명합니다. 정답 코드는 적지 않았습니다.

### 이 문서의 레포 표기

줄 번호는 `DIDEN_Core` 커밋 `386bae3a8` 기준입니다.

| 줄여 쓴 이름 | 경로 |
|---|---|
| `policy.h` | [DIDEN_Core/include/diden_core/applications/controller/runner/rl_controller/policy.h](../../../DIDEN_Core/include/diden_core/applications/controller/runner/rl_controller/policy.h) |
| `policy.cpp` | [DIDEN_Core/src/applications/controller/runner/rl_controller/policy.cpp](../../../DIDEN_Core/src/applications/controller/runner/rl_controller/policy.cpp) |
| `runner/thread.cpp` | [DIDEN_Core/src/applications/controller/runner/thread.cpp](../../../DIDEN_Core/src/applications/controller/runner/thread.cpp) |
| `action.cpp` | [DIDEN_Core/src/applications/controller/runner/rl_controller/action.cpp](../../../DIDEN_Core/src/applications/controller/runner/rl_controller/action.cpp) |
| `rl_controller.cpp` | [DIDEN_Core/src/applications/controller/runner/rl_controller/rl_controller.cpp](../../../DIDEN_Core/src/applications/controller/runner/rl_controller/rl_controller.cpp) |
| `controller_base.h` | [DIDEN_Core/include/diden_core/applications/controller/runner/controller_base.h](../../../DIDEN_Core/include/diden_core/applications/controller/runner/controller_base.h) |
| `controller_config.yaml` | [DIDEN_Core/config/diden_spider/controller_config.yaml](../../../DIDEN_Core/config/diden_spider/controller_config.yaml) |
| golden YAML | [DIDEN_World/tests/contract/seam3_policy_config.golden.yaml](../../../DIDEN_World/tests/contract/seam3_policy_config.golden.yaml) |

---

## Q1. 정책 주기 유도 (`derive_cycle_time_us`)

### 배경: decimation 이란

학습 시뮬레이터는 물리를 아주 잘게(예: 1 ms 마다) 계산합니다. 정책은 그보다 드물게(예: 10 물리 스텝마다 한 번) 불립니다. 이 비율이 **decimation** 입니다.

```
정책 주기 = physics_dt × decimation
          = 0.001 s × 10 = 0.01 s = 10 ms   (100 Hz)
```

정책 YAML 은 이 둘을 그대로 실어 옵니다 (golden YAML `79-80` 행).

```yaml
decimation: 10
physics_dt: 0.001
```

Core 는 여기서 주기를 µs 단위 정수로 **유도**합니다. 주기를 직접 적는 키(`control.cycle_time_us`)는 받지 않습니다 — 들어오면 **무시가 아니라 거절**합니다. 값이 둘이면 언젠가 어긋나고, "어느 쪽이 맞는지" 를 조용히 고르는 코드는 버그를 숨기기 때문입니다.

### 규칙

| 상황 | 결과 |
|---|---|
| `explicit_cycle_time_us` 가 들어 있다 | `nullopt` |
| `decimation` 또는 `physics_dt` 가 없다 | `nullopt` |
| 계산 결과가 0 이하 | `nullopt` |
| 그 외 | `static_cast<int>(physics_dt * decimation * 1e6)` |

### 버림(truncation)에 대하여

`static_cast<int>` 는 소수부를 **버립니다.** 대부분의 값에서는 문제가 없습니다.

| `physics_dt` | `decimation` | `double` 결과 | `int` |
|---|---|---|---|
| 0.001 | 10 | 10000.0 | 10000 |
| 0.005 | 4 | 20000.0 | 20000 |
| **0.0003** | **10** | **2999.9999999999995** | **2999** |

0.0003 은 2진 부동소수로 정확히 표현되지 않아, 곱한 결과가 3000 에 아주 조금 못 미칩니다. 버리면 2999 가 됩니다.

- 이것은 **레포의 실제 동작**이고, 퀴즈는 그것을 그대로 맞춥니다 (반올림으로 "고치지" 않습니다).
- 지금 쓰는 정책들(0.001 × 10 등)에서는 드러나지 않습니다. 다만 새 `physics_dt` 를 쓰는 정책이 오면 주기가 1 µs 짧게 잡힐 수 있다는 것은 알아 둘 만합니다.
- 1 µs 차이가 실제로 영향을 주는 곳: Q3 에서 보듯 1 kHz tick 위에서는 2999 든 3000 이든 3 tick 마다 돕니다. 하지만 자석 cooldown 환산(`ceil(ms / dt)`, 04 Q3)은 `dt` 가 2.999 ms 면 `ceil(30 / 2.999) = 11`, 3 ms 면 `10` 으로 갈립니다.
- 의도된 것인지 간과된 것인지는 코드만으로는 알 수 없었습니다.

### 테스트 따라가기

| 호출 | 기대 |
|---|---|
| `(10, 0.001)` | 10000 |
| `(4, 0.005)` | 20000 |
| `(1, 0.001)` | 1000 |
| `(nullopt, 0.001)`, `(10, nullopt)` | `nullopt` |
| `(10, 0.001, 10000)` | `nullopt` — 값이 맞아도 거절 |
| `(0, 0.001)`, `(10, 0.0)`, `(10, −0.001)` | `nullopt` |
| `(10, 0.0003)`, `(5, 0.0006)` | 2999 |

### 단계별 힌트

- `std::optional<int>` 인자는 `if (opt)` 로 값이 있는지 보고 `*opt` 로 꺼냅니다.
- 반환도 `std::optional<int>` 입니다. 실패는 `return std::nullopt;`, 성공은 `return 값;` 이면 됩니다.
- 곱하는 **순서**를 레포와 같게 두세요: `(physics_dt × decimation) × 1e6`. 부동소수 곱셈은 순서에 따라 마지막 자리가 달라질 수 있습니다.

### 흔한 실수

- **`std::lround` 로 반올림.** 2999 테스트가 3000 으로 실패합니다. 더 "옳아 보이는" 답이지만 레포와 다릅니다.
- **명시된 주기가 유도값과 같으면 통과.** 레포는 값을 비교하지 않고 키의 존재만으로 거절합니다.
- **누락 시 기본값(예: 1000) 반환.** 정책이 열 배 빠르게 돕니다.

### 레포 연결

- `policy.cpp:236-249` — `build_control` 안의 유도. `240-244` 명시 키 거절, `245-248` 누락 거절, `249` 계산.
- `policy.cpp:283-286` — 0 이하 거절.
- `policy.h:419-421` — `RawControl::cycle_time_us`. "받지 않기 위해 스키마에 남겨둔다" 는 주석이 있습니다. 스키마에서 아예 지우면 YAML 파서가 모르는 키를 조용히 무시하기 때문에, 일부러 읽어서 거절합니다.
- `policy.h:463-464` — `decimation`, `physics_dt` 가 `optional` 인 곳.
- `policy.h:251-253` — 도메인 타입의 `cycle_time_us` (0 = 미설정).

---

## Q2. `StepScheduler` — tick 위에서 주기 맞추기

### 배경: runner 는 1 kHz, 정책은 100 Hz

컨트롤러를 돌리는 runner 스레드는 **1 ms 마다** 깨어납니다 (`controller_config.yaml:7`, `cycle_time_us: 1000`). 이 1 ms 가 제어 주기의 해상도입니다.

정책은 10 ms 마다 돌아야 하므로, runner 는 깨어날 때마다 "지금 돌릴 차례인가" 를 확인하고 아니면 그냥 넘깁니다.

```
tick(ms):  0  1  2  3  4  5  6  7  8  9  10 11 ... 20
step:      ●  ·  ·  ·  ·  ·  ·  ·  ·  ·  ●  ·  ...  ●
```

방법은 "tick 을 10 번 세기" 가 아니라 **시각 비교**입니다.

```
start:  다음 목표 시각 = 지금                    (첫 step 은 바로)
due:    지금 < 다음 목표 시각  →  아직 아님 (false)
        아니면                 →  다음 목표 시각 += 주기,  돈다 (true)
```

tick 을 세지 않고 시각을 비교하는 이유는 Q3 에서 드러납니다 (tick 이 늦게 오는 경우).

### 활성화 거절

정책 주기가 tick 보다 짧으면(예: 500 µs) 1 ms 마다 깨는 스레드로는 지킬 수 없습니다. 조용히 1 ms 로 돌리면 학습 rate 와 어긋나므로 **활성화 자체를 거절**합니다. `start` 가 `false` 를 돌려주는 경우입니다.

### 테스트 따라가기

| 설정 | 확인 | 기대 |
|---|---|---|
| `start(0, 10000, 1000)` | 0 ~ 25000 을 1000 간격으로 | `{0, 10000, 20000}` 에서 true |
| `start(7000, 10000, 1000)` | `due(6000)` | false — 아직 시작 전 |
| | `due(7000)` | true — 활성화한 tick 에 바로 |
| | `due(8000)`, `due(16000)` | false |
| | `due(17000)` | true |
| `start(0, 1000, 1000)` | 0 ~ 3000 | 매 tick true |
| `start(0, 500, 1000)`, `start(0, 999, 1000)` | 반환값 | false |
| `start(0, 20000, 1000)` | 0 ~ 45000 | `{0, 20000, 40000}` |
| 같은 객체에 `start(50000, 10000, 1000)` | `due(50000)`, `due(59000)`, `due(60000)` | true, false, true — 새로 센다 |

### 단계별 힌트

1. 멤버는 둘이면 됩니다: 주기, 다음 목표 시각. 시각은 `std::int64_t` 로 두세요 (µs 단위로 35 분이면 `int` 범위를 넘습니다).
2. `start`: 거절 조건을 먼저 보고, 통과하면 두 멤버를 잡습니다.
3. `due`: 위의 세 줄 그대로. Q3 의 "한 주기 넘게 밀렸을 때" 처리는 Q3 에서 추가합니다.

### 흔한 실수

- **첫 목표를 `now + period` 로 잡음.** 활성화 후 한 주기 동안 아무 명령도 안 나갑니다. 레포는 "첫 step 은 이번 tick 에 바로" 입니다.
- **`now <= 다음 목표` 일 때 false.** 정확히 목표 시각에 온 tick 을 놓쳐 11 tick 간격이 됩니다.
- **`period == tick` 을 거절.** 거절 조건은 "짧다(`<`)" 입니다. 같으면 매 tick 돌면 됩니다.

### 레포 연결

- `runner/thread.cpp:111-198` — `thread_step`.
  - 활성화: `113-151`. 주기를 묻는 곳 `139-140`, 거절 `141-146`, 주기와 첫 목표 시각 `148-149`.
  - 차례 확인: `159-163`.
  - step 호출: `173`.
- **주기를 `on_enter()` 뒤에 묻는 이유**가 `131-138` 행 주석에 있습니다. `RlController` 는 `on_enter` 에서 정책을 로드하고, 그 전에 `cycle_time_us()` 를 물으면 폴백값(manager 주기 1000 µs)이 나옵니다 (`rl_controller.cpp:810-814`). 그러면 정책이 열 배 빠르게 돌고 자석 cooldown 도 같이 어긋납니다.
- `controller_base.h:136-141` — `cycle_time_us()` 의 규약.
- **단순화한 점**: 레포는 `std::chrono::steady_clock::now()` 를 함수 안에서 직접 읽고, 상태(`period`, `next_step_time`)는 파일 범위 변수입니다. 퀴즈는 테스트할 수 있게 시각을 인자로 받고 클래스로 묶었습니다. 정지 요청 처리(`153-157`)와 출력 검증(`175-189`)은 다루지 않습니다.

---

## Q3. `StepScheduler` — 지각과 어긋난 주기

### 배경: tick 은 정확히 오지 않는다

실제 스레드는 조금씩 늦게 깨어납니다. 가끔은 많이 늦습니다. 두 경우의 대응이 다릅니다.

**조금 늦었을 때 — 따라잡는다.**

```
목표:      0      10000     20000     30000
실제 tick: 0      10300     20000     30000
                   ↑ 300 µs 지각
```

10300 에 돌리되, 다음 목표는 `10300 + 10000` 이 아니라 **원래 격자의 20000** 에 둡니다 (`다음 목표 += 주기`). 그래야 지각이 누적되지 않고 장기적으로 정확히 100 Hz 가 유지됩니다. Q2 에서 tick 을 세지 않고 시각을 더해 나간 이유입니다.

**한 주기 넘게 밀렸을 때 — 포기한다.**

```
목표:      0      10000     20000     30000    (전부 지남)
실제 tick: 0      ─────── 멈춤 ───────  35000
```

35000 에 깨어나 보니 10000, 20000, 30000 을 놓쳤습니다. 이것을 따라잡겠다고 step 을 연달아 세 번 돌리면, 컨트롤러는 "30 ms 가 흘렀다" 고 생각하는데 실제 로봇은 그만큼 움직이지 않았습니다 — 명령이 튑니다. 그래서 놓친 것은 버리고 **지금부터 다시** 셉니다.

```
다음 목표 += 주기              (10000 → 20000)
다음 목표 < 지금 이면          (20000 < 35000)
    다음 목표 = 지금 + 주기    (45000)
```

### 경계: 정확히 한 주기 늦은 경우

조건은 `다음 목표 < 지금` (엄격한 부등호)입니다. 그래서 **정확히** 한 주기 늦은 것은 "넘게" 가 아닙니다.

| 호출 | 호출 전 목표 | `+= 주기` 뒤 | `< 지금`? | 결과 |
|---|---|---|---|---|
| `due(0)` | 0 | 10000 | | true |
| `due(20000)` | 10000 | 20000 | `20000 < 20000` 아님 | true, 목표 20000 유지 |
| `due(21000)` | 20000 | 30000 | | **true** — 바로 다음 tick 에 또 돈다 |
| `due(22000)` | 30000 | | | false |
| `due(30000)` | 30000 | 40000 | | true |

놓친 한 번을 다음 tick 에 따라잡는 셈입니다.

### 주기가 tick 의 배수가 아닐 때

주기 2500 µs, tick 1000 µs:

| tick | 목표 | 판정 | `+= 2500` 뒤 |
|---|---|---|---|
| 0 | 0 | 돈다 | 2500 |
| 1000, 2000 | 2500 | 아직 | |
| 3000 | 2500 | 돈다 | 5000 |
| 4000 | 5000 | 아직 | |
| 5000 | 5000 | 돈다 | 7500 |
| 6000, 7000 | 7500 | 아직 | |
| 8000 | 7500 | 돈다 | 10000 |
| 10000 | 10000 | 돈다 | 12500 |

간격이 3, 2, 3, 2 tick 으로 흔들리지만 10 ms 에 네 번, 평균 2.5 ms 입니다.

Q1 의 2999 µs 도 넣어 보면 `{0, 3000, 6000, 9000}` 입니다. 목표가 2999, 5998, 8997 로 격자보다 조금씩 앞서지만 tick 은 3000, 6000, 9000 에만 오기 때문입니다 (이 구간에서는. 오래 돌리면 목표가 tick 한 칸만큼 앞서는 순간이 와서 2 tick 간격이 한 번 끼어듭니다 — 언제인지 계산해 보세요).

### 테스트 따라가기

| 시나리오 | 호출 순서와 기대 |
|---|---|
| 조금 지각 | `due(0)` T, `due(10300)` T, `due(19900)` **F**, `due(20000)` T |
| 크게 지각 | `due(0)` T, `due(35000)` T, `due(36000)` F, `due(44000)` F, `due(45000)` T |
| 정확히 한 주기 | 위 표 |
| 2500 µs | `{0, 3000, 5000, 8000, 10000}` |
| 2999 µs | `{0, 3000, 6000, 9000}` |

### 단계별 힌트

- Q2 의 `due` 에 `if` 하나를 더하면 됩니다. 위치는 `+= 주기` **바로 뒤**입니다.

### 흔한 실수

- **항상 `다음 목표 = 지금 + 주기`.** 조금 지각 시나리오에서 `due(19900)` 은 false 로 맞지만 `due(20000)` 도 false 가 됩니다 (목표가 20300 이라서). 지각이 누적됩니다.
- **`while` 로 목표를 지금보다 뒤로 밀기.** 크게 지각했을 때 목표가 40000 이 됩니다. 레포는 "지금 + 주기"(45000)입니다. 격자를 버리고 지금을 새 기준으로 삼는 것입니다.
- **`<=` 로 비교.** 정확히 한 주기 시나리오의 `due(21000)` 이 false 가 됩니다.

### 레포 연결

- `runner/thread.cpp:165-171` — 주석과 네 줄의 코드. 주석은 "한 주기 이상 밀렸으면" 이라고 쓰지만 코드의 비교는 `<` 라서, 정확히 한 주기는 따라잡는 쪽입니다. 사소한 차이지만 **주석보다 코드가 실제 동작**이라는 좋은 예입니다.
- 왜 따라잡는가: "RL 정책은 학습 rate 에 묶여 있다" (`165` 행).
- 왜 포기하는가: "놓친 사이클을 몰아 돌면 컨트롤러의 고정 dt 가 실제 시간보다 앞서 나가 명령이 튄다" (`166-167` 행). `RlController` 는 실제 경과 시간을 재지 않고 매 step 을 고정 `dt_` 로 칩니다 (`rl_controller.cpp:193`, `660`).

---

## Q4. 차원 계약 (`validate_dims`, `remove_estimator_output`)

### 배경: YAML 과 ONNX 가 같은 정책인가

정책은 파일 두 개로 옵니다.
- `xxx.onnx`: 신경망. 입력 차원과 출력 차원을 알고 있습니다.
- `xxx.yaml`: 그 입력·출력의 **의미**(term 목록)와 설정.

둘이 다른 버전이면(YAML 만 고쳤다든지) 차원이 맞지 않습니다. 맞지 않는 채로 돌리면 관측 벡터의 일부가 채워지지 않은 채 추론에 들어갑니다.

### 검사 순서

```
1) ONNX 입력 차원  ==  Σ(obs term 의 dim) − (estimator_output 의 dim)
2) ONNX 출력 차원  ==  Σ(action term 의 dim)
3) joint_actions term 이 있으면:
     action_mean 의 길이 == joint_actions 의 dim
     joint_action_scale > 0
```

처음 걸린 것을 돌려줍니다. 레포는 에러 로그를 남기고 `false` 를 돌려주는데, 퀴즈는 어느 검사에 걸렸는지 테스트할 수 있게 `enum` 으로 돌려줍니다.

### `estimator_output` 이란

정책 신경망은 사실 두 부분입니다.

```
        ┌──────────── ONNX 한 파일 ─────────────┐
obs ──┬─▶ estimator ──▶ estimator_output ──┐     │
      │                                    ▼     │
      └──────────────────────────────▶ [이어 붙임] ─▶ actor ──▶ action
        └───────────────────────────────────────┘
```

- estimator 는 관측에서 직접 재기 어려운 값(예: 몸통 속도)을 추정하는 작은 신경망입니다.
- 학습 때의 관측 목록에는 `estimator_output` 이 term 으로 들어 있습니다.
- 배포용 ONNX 는 둘을 **합쳐서**(merged) 내보냅니다. 그래서 ONNX 의 입력은 `estimator_output` 을 **뺀** 나머지입니다.
- Core 는 그 term 을 채우지 않습니다. 입력 차원을 셀 때 빼고, 검증이 끝나면 목록에서도 지웁니다 (`remove_estimator_output`). 남겨 두면 03 의 `ObservationManager` 가 그 칸을 채우려 들고 차원이 다시 어긋납니다.

### 3) 이 막는 것

| 누락 | 조용히 벌어지는 일 |
|---|---|
| `action_mean` 길이 불일치 | 레포 주석(`policy.cpp:442-445`)의 설명: joint term 이 매 주기 아무것도 하지 않게 되어 `kp = 0` 인 채 나감 → 다리에 힘이 빠져 추락. 지금의 `apply` 는 길이를 다시 확인하지 않으므로(`action.cpp:36-37` 주석) 이 로드 검사가 유일한 방어선 |
| `joint_action_scale` 누락 (기본 0) | `목표 = mean + 0·raw` → 정책 출력이 전부 무시되고 기본 자세만 유지 |

둘 다 에러가 나지 않는 실패입니다. 그래서 로드 때 막습니다.

### 테스트 따라가기

golden 모양: 관측 합 157, 행동 `joint_actions 12` + `magnetic_actions 4`.

| `(입력, 출력, mean 길이, scale)` | 기대 |
|---|---|
| `(157, 16, 12, 0.4)` | `kOk` |
| `(145, 16, 12, 0.4)` | `kInputDimMismatch` |
| `(157, 12, 12, 0.4)` | `kOutputDimMismatch` |
| `(157, 16, 13, 0.4)`, `(157, 16, 0, 0.4)` | `kActionMeanSizeMismatch` |
| `(157, 16, 12, 0.0)`, `(…, −0.4)` | `kActionScaleInvalid` |
| `(1, 1, 0, 0.0)` | `kInputDimMismatch` — 먼저 검사하는 것 |
| `(157, 1, 0, 0.0)` | `kOutputDimMismatch` |
| `(157, 16, 0, 0.0)` | `kActionMeanSizeMismatch` |

`estimator_output(19)` 를 목록에 끼워 넣으면 합이 176 이 됩니다 (19 는 테스트용으로 정한 값입니다).
- ONNX 입력 157 → `kOk` (`176 − 19`)
- ONNX 입력 176 → `kInputDimMismatch`

`joint_actions` 가 없는 정책(`magnetic_actions` 만)은 `mean 길이 0, scale 0` 이어도 `kOk`.

`remove_estimator_output`: 12 개짜리 목록에서 하나가 빠져 11 개가 되고, 나머지 순서는 그대로입니다. 없으면 아무 일도 없습니다.

### 단계별 힌트

1. 목록을 한 번 돌면서 dim 합을 구하고, 특정 이름의 term 을 찾으면 그 주소를 기억합니다 (`const TermEntry*`, 없으면 `nullptr`).
2. 이름 상수(`kEstimatorOutputTerm`, `kJointActionsTerm`)가 `quiz.h` 에 있습니다. 문자열 리터럴을 여러 곳에 흩어 쓰지 마세요.
3. `action_mean_size` 는 `std::size_t`, dim 은 `int` 입니다. 비교할 때 한쪽으로 맞추세요 (`-Wall -Wextra` 가 부호가 다른 비교를 경고합니다).
4. `remove_estimator_output` 은 C++20 의 `std::erase_if(벡터, 조건)` 한 줄로 됩니다. 조건은 람다로.

### 흔한 실수

- **`estimator_output` 을 빼지 않음.** merged ONNX 에서 항상 `kInputDimMismatch` 가 납니다.
- **검사 순서.** 여러 개가 틀렸을 때 어느 것이 보고되는지가 달라집니다.
- **`joint_actions` 가 없는데 mean/scale 을 검사.** 자석만 제어하는 정책이 거절됩니다.
- **`scale >= 0` 으로 검사.** 0 이 바로 "누락" 입니다.

### 레포 연결

- `policy.cpp:420-469` — `Policy::validate_dims`. 합계 `424-425`, 입력 `430-436`, 출력 `437-440`, mean `446-451`, scale `455-458`, term 제거 `464-467`.
- `policy.cpp:294-300` — 특수 이름 두 개. "DidenWorld export 규약과 lockstep" 이라는 주석이 있습니다. World 쪽 짝: `packages/diden-env/src/diden_env/deploy/policy_schema.py:20-21`.
- `policy.cpp:685-704` — `Policy::load`. 순서는 YAML 읽기 → ONNX 로드 → 모션 파일 → `validate_dims`.
- 두 번째 관문: `rl_controller.cpp:221-227`. `ObservationManager` 를 실제로 조립한 뒤의 `total_dim()` 을 ONNX 입력 차원과 **다시** 비교합니다. 첫 관문은 YAML 에 적힌 dim 의 합이고, 두 번째는 실제 term 이 만든 dim 의 합입니다.
- **두 관문 모두 잡지 못하는 것**: 합은 같고 순서만 다른 경우. 이것은 Core 안에서는 검사할 방법이 없고, YAML 과 ONNX 를 같은 export 에서 함께 만든다는 절차에 기대고 있습니다.
- 참고: World 의 `policy_schema.py:33-35` 는 "Core 가 ONNX 의 `metadata_props` 에서 정책 설정을 읽는다" 고 적고 있지만, 기준 커밋의 Core 는 ONNX 옆의 같은 이름 `.yaml` 파일에서 읽습니다 (`policy.h:487-488`, `policy.cpp:503-508`). 두 레포의 설명이 어긋나 있는 부분입니다.

---

## Q5. 설정 정합성

### (a) `validate_term_list` — 중복 이름과 음수 dim

YAML 의 목록은 같은 이름이 두 번 나올 수 있습니다. 순서가 곧 배치이므로, 같은 이름이 둘이면 어느 쪽을 가리키는지 모호해집니다 (Q4 에서 `estimator_output` 을 "이름으로" 찾았던 것을 떠올리세요). 그래서 거절합니다.

| 목록 | 기대 |
|---|---|
| `dof_pos, dof_vel, command` | true |
| 빈 목록 | true |
| `dof_pos(0)` | true — dim 0 은 여기서는 통과 (03 의 dim 검사가 잡는다) |
| `dof_pos, dof_pos` | false |
| `dof_pos(12), command, dof_pos(24)` | false — dim 이 달라도 이름이 같으면 중복 |
| `dof_pos(−1)` | false |
| `dof_vel, dof_vel_history` | true — 이름이 다르면 다른 term |

마지막 줄: 접미사를 떼면 같은 base 지만, 중복 검사는 **YAML 에 적힌 이름 그대로** 합니다.

**힌트**: `std::unordered_set<std::string>` 의 `insert` 는 `(반복자, 새로 들어갔는가)` 쌍을 돌려줍니다. `.second` 가 false 면 이미 있던 이름입니다.

### (b) `validate_control_gains`

| `(kp, kd)` | 기대 | 이유 |
|---|---|---|
| `(50, 5)` | true | |
| `(50, 0)` | true | 감쇠 없음은 허용 |
| `(0, 5)`, `(−1, 5)` | false | 위치 토크가 0 이거나 역방향 → 다리가 풀린다 |
| `(50, −0.1)` | false | 음의 감쇠 → 발진 |

kp 는 `> 0`, kd 는 `>= 0` 입니다. 경계가 다릅니다.

### (c) `common_cycle_time_us` — 정책 집합의 주기

04 Q6 에서 본 것처럼 정책은 체인으로 이어질 수 있습니다. runner 는 주기를 **활성화 때 한 번만** 읽습니다 (Q2). 도는 중에 주기가 다른 정책으로 갈아타면 runner 는 모르는 채로 옛 주기로 계속 부릅니다. 그래서 한 집합의 정책은 주기가 전부 같아야 합니다.

| 입력 | 기대 |
|---|---|
| `{10000}` | 10000 |
| `{10000, 10000, 10000}` | 10000 |
| `{10000, 20000}` | `nullopt` |
| `{10000, 10000, 9999}` | `nullopt` |
| `{}` | `nullopt` |

### 흔한 실수

- **(a) 접미사를 뗀 이름으로 중복 검사.** 마지막 테스트가 실패합니다.
- **(b) 둘 다 `> 0`.** `kd = 0` 이 거절됩니다.
- **(c) 빈 `span` 에서 `front()` 호출.** 정의되지 않은 동작입니다. 빈 경우를 먼저 처리하세요.

### 레포 연결

- (a) `policy.cpp:59-66` `check_unique`, `68-91` `build_observations` (음수 dim `73-76`), `93-112` `build_actions`.
- (b) `policy.cpp:271-282`. 주석이 "silent 오동작으로 직결되는 값만 강제한다" 고 범위를 밝힙니다.
- (c) `policy.cpp:610-676` `Policy::resolve_policy_set`. 주기 비교는 `635-643` 행. 집합 크기 상한(8)은 `321-322` 행.
- **단순화한 점**: 레포의 `resolve_policy_set` 은 `next_policy` / `transitions` 링크를 따라가며 정책 집합을 모으고, 순환을 검사하고, 각 정책의 YAML 을 파싱합니다. 퀴즈는 그 끝에 있는 "주기가 전부 같은가" 만 다룹니다.
