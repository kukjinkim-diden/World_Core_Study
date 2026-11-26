# 04_action 상세 설명

## 큰 그림: 행동 벡터도 "term 을 이어 붙인 것"

정책(ONNX)이 내놓는 것은 float 16개입니다. 관측과 마찬가지로 YAML 이 배치를 정합니다.

```yaml
actions:
- {name: joint_actions, dim: 12}      # [0, 12)
- {name: magnetic_actions, dim: 4}    # [12, 16)
```

각 term 은 자기 구간의 숫자를 받아 `ControllerOutput` 의 해당 필드를 채웁니다.

```
행동 벡터 [ r0 … r11 | m0 … m3 ]
             │           │
             │           └─ tanh ─▶ 자석 gate ─▶ out.epm[0..4)   (+1 켜라 / −1 꺼라 / 0 그대로)
             └─ mean + scale·r ─▶ out.joint.q[0..12), kp, kd
```

`ControllerOutput` 은 컨트롤러 한 주기의 결과물입니다. 관절 18개(다리 12 + 팔 6)의 `q, dq, tau, kp, kd` 와 자석 8개(발 4 + 본체 4)의 신호를 담습니다. RL 정책이 채우는 것은 **다리 12개의 `q, kp, kd` 와 발 자석 4개**뿐이고, 팔은 정해진 자세를 유지합니다.

이 퀴즈는 그 변환(Q1 ~ Q4)과, 정상 경로가 아닐 때의 출력(Q5 safe-hold, Q6 정책 전환)을 만듭니다.

각 문항은 **배경 → 테스트 따라가기 → 단계별 힌트 → 흔한 실수 → 레포 연결** 순서로 설명합니다. 정답 코드는 적지 않았습니다.

### 이 문서의 레포 표기

줄 번호는 `DIDEN_Core` 커밋 `386bae3a8` 기준입니다.

| 줄여 쓴 이름 | 경로 |
|---|---|
| `action.h` | [DIDEN_Core/include/diden_core/applications/controller/runner/rl_controller/action.h](../../../DIDEN_Core/include/diden_core/applications/controller/runner/rl_controller/action.h) |
| `action.cpp` | [DIDEN_Core/src/applications/controller/runner/rl_controller/action.cpp](../../../DIDEN_Core/src/applications/controller/runner/rl_controller/action.cpp) |
| `policy.h` | [DIDEN_Core/include/diden_core/applications/controller/runner/rl_controller/policy.h](../../../DIDEN_Core/include/diden_core/applications/controller/runner/rl_controller/policy.h) |
| `rl_controller.cpp` | [DIDEN_Core/src/applications/controller/runner/rl_controller/rl_controller.cpp](../../../DIDEN_Core/src/applications/controller/runner/rl_controller/rl_controller.cpp) |
| `controller_base.h` | [DIDEN_Core/include/diden_core/applications/controller/runner/controller_base.h](../../../DIDEN_Core/include/diden_core/applications/controller/runner/controller_base.h) |
| `rl_config.yaml` | [DIDEN_Core/config/diden_spider/rl_config.yaml](../../../DIDEN_Core/config/diden_spider/rl_config.yaml) |
| golden YAML | [DIDEN_World/tests/contract/seam3_policy_config.golden.yaml](../../../DIDEN_World/tests/contract/seam3_policy_config.golden.yaml) |

---

## Q1. joint_actions (`apply_joint_action`)

### 배경: 정책은 "기본 자세로부터의 차이" 를 낸다

정책이 관절 각도를 직접 내면 학습이 어렵습니다. 대신 **기본 자세(`action_mean`)에서 얼마나 벗어날지**를 작은 숫자로 냅니다.

```
목표 각도[i] = action_mean[i] + joint_action_scale · raw[i]
```

- `action_mean`: 관절 12개의 기본 자세 [rad]. 정책이 0 만 내면 로봇은 이 자세로 섭니다.
- `joint_action_scale`: raw 1.0 이 몇 rad 인가. golden 정책은 0.4.
- 이 목표는 **위치 명령**입니다. 실제 토크는 모터 드라이브의 PD 제어기가 `kp·(목표 − 현재) − kd·속도` 로 만듭니다. 그래서 게인도 같이 내보냅니다. 정책 YAML 의 `joint_kp`, `joint_kd` 를 12개 관절에 똑같이 넣습니다.

건드리는 것은 **다리 12칸의 `q`, `kp`, `kd`** 뿐입니다. `dq`, `tau` 는 0 인 채로 둡니다 (속도·토크 feedforward 없음).

### 테스트 따라가기

golden 값: `scale = 0.4`, `mean = [−0.9377, −1.5307, 1.8549, …, 1.8552]`, `kp = 50`, `kd = 5`.

| raw | 확인 | 기대 | 계산 |
|---|---|---|---|
| 전부 0 | `q[0]` | −0.9377 | mean 그대로 |
| 전부 0.5 | `q[0]` | −0.7377 | `−0.9377 + 0.4·0.5` |
| 전부 0.5 | `q[2]` | 2.0549 | `1.8549 + 0.2` |
| `raw[i] = (i − 6)·0.25` | `q[0]` | −1.5377 | `−0.9377 + 0.4·(−1.5)` |
| | `q[6]` | −2.2071 | raw 0 |
| | `q[11]` | 2.3552 | `1.8552 + 0.4·1.25` |

세 번째 경우는 `out` 을 미리 "원래 값" 으로 채워 두고 부릅니다 (`q = 9`, `kp = kd = 7`, `dq = tau = 3`, `epm = 1`). 호출 뒤:
- `kp[0..12) = 50`, `kd[0..12) = 5`
- 팔 구간 `q[12..18) = 9`, `kp[12..18) = 7` — 그대로
- `dq`, `tau`, `epm` — 그대로

### 단계별 힌트

- `raw` 는 `std::span<const float>` 이고 계산은 `double` 입니다. `static_cast<double>(raw[i])` 로 올린 뒤 계산하세요.
- 반복 횟수는 `kNumMobileJoints` 입니다. `out.joint.q.size()`(18)로 돌면 팔까지 덮어씁니다.

### 흔한 실수

- **18칸을 전부 채움.** 팔의 게인이 다리 게인(50)으로 바뀝니다. 팔은 더 큰 게인(200)을 쓰는 다른 종류의 구동기입니다.
- **`scale·(mean + raw)`.** 괄호 위치.
- **`out` 을 새로 만들어 덮어씀.** 이 함수는 `out` 의 일부만 고칩니다. 자석 term 이 같은 `out` 의 다른 부분을 채우기 때문입니다.
- **`action_mean` 크기를 확인함.** 여기서는 하지 않습니다. 크기가 12 인 것은 정책을 올릴 때 이미 검증됐습니다 (05 Q4). 매 주기 도는 함수에 같은 검사를 또 넣지 않는 것이 레포의 방식입니다.

### 레포 연결

- `action.cpp:28-57` — `JointPositionActionTerm`. 핵심은 `apply` 의 `45-50` 행.
- `policy.h:98-102` — `Actions` (`joint_action_scale`, `action_mean`). `244-250` — `Control` (`joint_kp`, `joint_kd`).
- `controller_base.h:23-36` — `ControllerOutput`.
- golden YAML `68-72` 행: `joint_kp: 50.0`, `joint_kd: 5.0`, `joint_action_scale: 0.4`, `action_mean: [...]`.
- **단순화한 점**: 레포는 `use_motion_reference_offset` 이 켜져 있으면 `action_mean` 대신 모션 클립의 참조 자세를 offset 으로 씁니다 (`action.cpp:43-46`). 퀴즈는 `action_mean` 경로만 다룹니다.

---

## Q2. 자석 행동 이산화 (`discretize_magnet_action`)

### 배경: 연속값을 세 가지 명령으로

발바닥의 전자영구자석(EPM)은 켜짐/꺼짐 두 상태뿐이고, 한 번 펄스를 주면 그 상태가 유지됩니다. 그래서 매 주기 필요한 명령은 셋 중 하나입니다.

```
+1 = 켜라        −1 = 꺼라        0 = 그대로 둬라
```

정책의 자석 출력은 `tanh` 를 거쳐 `[−1, 1]` 범위의 연속값이 됩니다(Q3 앞 단계, 06 에서 붙입니다). 이것을 문턱 두 개로 자릅니다.

```
a > action_on_threshold   → +1
a < action_off_threshold  → −1
그 사이                    →  0
```

가운데 구간이 "그대로" 입니다. 정책이 확신이 없으면(0 근처) 아무 일도 일어나지 않습니다.

### 테스트 따라가기

golden: `on = 0.3`, `off = −0.3`.

| a | 기대 |
|---|---|
| 0.9 | +1 |
| −0.9 | −1 |
| 0.1, −0.2, 0 | 0 |

경계 (`on = 0.5`, `off = −0.5`): `0.5 → 0`, `−0.5 → 0`, `0.5001 → +1`, `−0.5001 → −1`.

비대칭 (`on = 0.6`, `off = −0.2`): `0.5 → 0`, `−0.25 → −1`.

### 단계별 힌트

- `if` 두 개와 `return 0`.
- 경계 테스트가 0.5 를 쓰는 이유: `a` 는 `float`, 문턱은 `double` 입니다. 비교할 때 `a` 가 `double` 로 올라가는데, `0.3f` 는 `double` 로 올리면 `0.30000001192…` 가 되어 `0.3` 보다 **큽니다.** 0.5 는 2진수로 정확히 표현되어 이런 문제가 없습니다.

### 흔한 실수

- **`>=`, `<=` 사용.** 경계에서 ±1 이 나옵니다.
- **`std::abs(a) > thr` 하나로 처리.** on/off 문턱이 비대칭일 수 있습니다.
- **`std::round` 사용.** 문턱이 ±0.5 일 때만 우연히 비슷합니다.

### 레포 연결

- `action.cpp:182-193` — `MagnetActionGate::tick` 의 2 단계.
- `policy.h:168-171` — `action_on_threshold`, `action_off_threshold`, `max_action_only`.
- golden YAML `74-78` 행.
- 주석(`action.h:34-35`)이 강조하는 점: **hysteresis 가 없습니다.** 직전 상태를 기억하지 않는 순수한 문턱 비교입니다. 학습 쪽(DIDEN_World)이 그렇게 되어 있어서 맞춘 것입니다.

---

## Q3. `MagnetActionGate` — 환산과 중재

### 배경: 왜 "gate" 인가

Q2 의 결과를 그대로 내보내면 안 되는 하드웨어 사정이 둘 있습니다.

- **코일 안정 시간**: 방금 스위칭한 자석은 일정 시간 다시 스위칭하면 안 됩니다. → 발마다 cooldown.
- **전원 전류 한계**: 자석 펄스는 순간 전류가 큽니다. 여러 발이 연달아 스위칭하면 전원이 버티지 못합니다. → 전체 공용 cooldown, 그리고 "한 주기에 한 발만".

gate 는 필터처럼 신호를 다듬는 것이 아니라, **이번 주기에 어느 발의 명령을 통과시킬지 판정**합니다. 통과하지 못한 명령은 버려집니다 (줄을 서지 않습니다).

`tick` 의 6 단계 전체는 이렇습니다. 이 문항(Q3)은 **cooldown 이 0 인 경우**, 즉 2 와 5 만 의미가 있는 경우를 봅니다.

| 단계 | 할 일 | 검사하는 문항 |
|---|---|---|
| 1 | 남은 cooldown 을 1 씩 줄인다 (0 미만으로는 안 내려감) | Q4 |
| 2 | 발마다 Q2 의 이산화 | Q3 |
| 3 | 자기 cooldown 이 남은 발 → 0 | Q4 |
| 4 | 공용 cooldown 이 남았으면 전부 → 0 | Q4 |
| 5 | `max_action_only` 면 살아남은 발 중 `|normalized|` 최대인 하나만 | Q3 |
| 6 | 살아남은 발마다 자기 cooldown 과 공용 cooldown 을 다시 채운다 | Q4 |

### `configure` — ms 를 tick 으로

cooldown 은 설정 파일에 **ms** 로 적혀 있습니다 (`rl_config.yaml`: 발 200 ms, 공용 50 ms). gate 는 주기마다 한 번 불리므로 **tick 수**로 바꿔 둡니다.

```
tick 수 = (ms <= 0) ? 0 : ceil(ms / control_dt_ms)
```

올림(`ceil`)인 이유: 50 ms 를 20 ms 주기로 세면 2.5 tick 인데, 2 tick(40 ms)만 기다리면 규격보다 짧습니다. 3 tick(60 ms)을 기다려야 안전합니다.

### 5 단계: 중재

`max_action_only` 가 켜져 있으면 한 주기에 **최대 한 발**만 통과합니다.

- 후보는 **이 시점에 0 이 아닌 발**뿐입니다. 문턱을 못 넘었거나 cooldown 에 막힌 발은 `|값|` 이 아무리 커도 후보가 아닙니다.
- 후보 중 `|normalized|` 가 가장 큰 발을 고릅니다.
- 같으면 **번호가 작은 발**입니다.

### 테스트 따라가기

`configure`:

| `(dt, foot, global)` [ms] | foot tick | global tick |
|---|---|---|
| `(10, 200, 50)` | 20 | 5 |
| `(20, 200, 50)` | 10 | 3 (`ceil(2.5)`) |
| `(10, 0, −5)` | 0 | 0 |
| `(10, 25, 1)` | 3 | 1 |

중재 (cooldown 0, 문턱 ±0.3):

| normalized | `max_action_only` | 기대 | 설명 |
|---|---|---|---|
| `(0.9, −0.9, 0.1, −0.2)` | off | `(1, −1, 0, 0)` | 문턱 넘은 발 전부 |
| `(0.5, −0.9, 0.4, 0)` | on | `(0, −1, 0, 0)` | `|−0.9|` 가 최대 |
| `(0.5, 0.4, 0, 0.7)` | on | `(0, 0, 0, 1)` | |
| `(0, 0.8, −0.8, 0.8)` | on | `(0, 1, 0, 0)` | 셋이 동률 → 번호가 작은 발 1 |
| `(0.2, −0.25, 0.1, 0)` | on | 전부 0 | 후보 없음 |
| `(0.5, −0.8, 0, 0)`, 문턱 `on 0.3 / off −0.9` | on | `(1, 0, 0, 0)` | 발 1 은 `|−0.8|` 로 가장 크지만 `−0.9` 보다 작지 않아 후보가 아님 |

### 단계별 힌트

1. 멤버 4 개: 발별 남은 cooldown(`std::array<int, 4>`), 공용 남은 cooldown, 환산해 둔 발 cooldown tick 수, 환산해 둔 공용 cooldown tick 수.
2. `configure` 의 환산은 두 번 쓰이므로 작은 람다로 빼 두면 좋습니다. `std::ceil` 은 `double` 을 돌려주므로 `static_cast<int>` 가 필요합니다.
3. `tick` 은 6 단계를 **주석으로 먼저 적고** 하나씩 채우세요. 이 문항에서는 1, 3, 4, 6 을 비워 둬도 통과합니다.
4. 5 단계: "지금까지 본 최대 `|값|`" 과 "그 발 번호" 를 들고 한 번 돕니다. 시작값은 번호 −1, 최대값 −1 처럼 "아직 없음" 을 뜻하는 값으로. 그다음 한 번 더 돌면서 고른 발이 아닌 것을 0 으로 만듭니다.
5. 동률에서 번호가 작은 발이 이기려면 비교가 `>` 여야 합니다 (`>=` 면 뒤의 발이 이깁니다).

### 흔한 실수

- **`|값|` 최대를 전체 발에서 고름.** 마지막 테스트가 잡습니다. 후보(0 이 아닌 발) 안에서만 고릅니다.
- **동률에서 뒤의 발.** `>=` 를 쓴 경우입니다.
- **`ceil` 대신 반올림/버림.** `(20, 200, 50)` 에서 global 이 2 가 됩니다.
- **`ms <= 0` 을 따로 처리하지 않음.** `ceil(−5/10) = ceil(−0.5) = −0` 이라 우연히 0 이 나오지만, `−15` 면 `−1` 이 됩니다.

### 레포 연결

- `action.h:27-91` — `MagnetActionGate`. `27-58` 행의 주석이 설계 배경 전체를 설명합니다.
- `action.cpp:157-162` — `configure`. `173-250` — `tick` (2 단계 `182-193`, 5 단계 `203-219`).
- `rl_config.yaml:9-10` — `foot_cooldown_ms: 200.0`, `global_cooldown_ms: 50.0`.
- `configure` 를 부르는 곳: `rl_controller.cpp:172-185`. `control_dt_ms` 로 **정책 주기**(`cycle_time_us × 1e-3`)를 넘깁니다. 1 kHz runner tick 이 아닙니다.
- **단순화한 점**
  - 레포의 `configure` 는 네 번째 인자 `stage_gap_ms` 를 받고, `tick` 에는 "In/Out staging" 단계(`action.cpp:221-230`, `242-246`)가 있습니다. 발 하나에 자석 cell 이 둘인 배선용이고, spider 는 cell 이 하나라 0 이 넘어가 꺼져 있습니다 (`rl_controller.cpp:175-182`).
  - 레포의 `tick` 은 결과를 참조 인자(`out_epm`)에 씁니다. 퀴즈는 반환값으로 돌려줍니다.
  - `foot_cooldown_ticks()` / `global_cooldown_ticks()` 는 테스트용 접근자입니다. 레포에는 없고, 대신 `timer_state()`(`252-262`)가 남은 cooldown 을 0 ~ 1 비율로 내놓아 `magnet_timer_state` 관측에 쓰입니다.
- cooldown 이 정책 YAML 이 아니라 `rl_config.yaml` 에 있는 이유: 하드웨어 제약이지 정책의 성질이 아니기 때문입니다 (`action.h:45-46`). golden YAML 의 `magnetic_foot_constraint_filter_steps: 40` / `electric_constraint_filter_steps: 10` 은 학습 쪽의 같은 개념이지만 Core 는 읽지 않습니다 (`policy.h:183-184`).

---

## Q4. `MagnetActionGate` — cooldown

### 배경

이제 1, 3, 4, 6 단계를 채웁니다. 핵심은 **순서**입니다.

```
tick 시작
  1) 카운터를 먼저 줄인다
  2) 이산화
  3) 4) 줄인 "뒤의" 카운터가 0 보다 크면 막는다
  5) 중재
  6) 통과한 발이 있으면 카운터를 다시 채운다
```

이 순서 때문에 "N tick cooldown" 은 **스위칭한 tick 으로부터 N tick 뒤에 다시 스위칭할 수 있다**는 뜻이 됩니다.

### tick 별 상태표

주기 10 ms, foot 30 ms(3 tick), global 20 ms(2 tick), `max_action_only` off. 표의 카운터는 그 tick 의 **1 단계(줄이기) 뒤, 6 단계(채우기) 전** 값입니다.

**(1) 서로 다른 발**

| tick | 입력 | foot0 | global | 판정 | 출력 | 6 단계 뒤 |
|---|---|---|---|---|---|---|
| t0 | 발 0 `+0.9` | 0 | 0 | 통과 | `(1,0,0,0)` | foot0 = 3, global = 2 |
| t1 | 발 1 `−0.9` | 2 | 1 | global 에 막힘 | `(0,0,0,0)` | 그대로 |
| t2 | 발 1 `−0.9` | 1 | 0 | 통과 | `(0,−1,0,0)` | foot1 = 3, global = 2 |
| t3 | 발 0 `+0.9` | 0 | 1 | foot0 은 풀렸지만 global 에 막힘 | `(0,0,0,0)` | |

**(2) 같은 명령을 계속 들고 있기** — 발 0 에 `+0.9` 를 8 tick 동안.

| tick | foot0 | global | 출력 |
|---|---|---|---|
| t0 | 0 | 0 | **+1** → foot0 = 3, global = 2 |
| t1 | 2 | 1 | 0 |
| t2 | 1 | 0 | 0 (foot0 에 막힘) |
| t3 | 0 | 0 | **+1** |
| t4, t5 | | | 0 |
| t6 | 0 | 0 | **+1** |

통과한 tick 은 `{0, 3, 6}` — 3 tick(30 ms) 간격입니다. 두 가지를 읽어 내세요.
- **gate 는 자석의 현재 상태를 모릅니다.** 이미 켜진 자석에 "켜라" 가 다시 나갑니다. 뒤 단계(EpmSequencer)도 "이미 켜져 있으니 무시" 하지는 않습니다 — 걸러 내는 조건은 "지금 펄스를 쏘는 중이거나 대기열이 차 있다" 뿐이라([epm_sequencer.cpp](../../../DIDEN_Core/src/applications/controller/manager/epm_sequencer.cpp) `57-66`, `76-89` 행), 한가하면 같은 방향의 펄스가 한 번 더 나갑니다.
- **막힌 tick 은 카운터를 다시 채우지 않습니다.** 채운다면 t1 에서 다시 3 이 되어 영원히 못 나갑니다.

**(3) `max_action_only` on, 두 발이 동시에 `+0.9`**

| tick | foot0 | foot1 | global | 출력 | 설명 |
|---|---|---|---|---|---|
| t0 | 0 | 0 | 0 | `(1,0,0,0)` | 동률 → 발 0 |
| t1 | 2 | 0 | 1 | 전부 0 | global |
| t2 | 1 | 0 | 0 | `(0,1,0,0)` | 발 0 은 자기 cooldown → 후보는 발 1 뿐 |
| t3 | 0 | 2 | 1 | 전부 0 | global |
| t4 | 0 | 1 | 0 | `(1,0,0,0)` | 발 0 이 다시 |

t2 가 중요합니다. 3 단계(발별 cooldown)가 5 단계(중재)보다 **먼저** 돌기 때문에, cooldown 중인 발 0 이 "가장 큰 발" 자리를 차지한 채 탈락하는 일이 없습니다.

**(4) 실제 설정** — 10 ms 주기, foot 200 ms(20), global 50 ms(5). 발 0 은 `+0.9` 를 계속, 발 1 은 t1 부터 `−0.9` 를 계속 (`max_action_only` off).

- 발 0: `{0, 20, 40}` — 200 ms 간격
- 발 1: `{5, 25}` — 발 0 의 스위칭 50 ms 뒤, 그다음은 자기 cooldown 200 ms 뒤

t20 에 발 0 이 통과할 수 있는 이유를 직접 확인해 보세요: global 은 t5 에 5 로 채워져 t10 에 0 이 됩니다.

### `reset`

남은 cooldown(발별, 공용)만 0 으로 만듭니다. `configure` 가 환산해 둔 tick 수는 유지합니다. 정책을 올릴 때 `reset()` → `configure(...)` 순서로 부릅니다.

### 단계별 힌트

1. 1 단계: `std::max(0, c − 1)`.
2. 3 단계는 발마다, 4 단계는 한 번에 전부(`fill(0)`).
3. 6 단계: 0 이 아닌 발마다 `발 cooldown = 환산값`, `공용 cooldown = 환산값`.
4. 위 (2)의 표를 종이에 직접 써 보면 순서 실수가 바로 보입니다.

### 흔한 실수

- **줄이기를 마지막에 함.** 스위칭한 바로 그 tick 에 채우고 줄이면 cooldown 이 1 tick 짧아집니다. (2)에서 `{0, 2, 4}` 가 나옵니다.
- **줄이기를 막는 검사 뒤에 함.** 반대로 1 tick 길어집니다. `{0, 4, 8}`.
- **중재를 cooldown 검사보다 먼저 함.** (3)의 t2 에서 아무도 못 나갑니다.
- **막힌 발도 cooldown 을 채움.**
- **`reset` 이 환산값까지 지움.** 그 뒤로 cooldown 이 없는 gate 가 됩니다.

### 레포 연결

- `action.cpp:150-155` — `reset`. `176-180` 1 단계, `195-201` 3·4 단계, `232-241` 6 단계.
- 통과하지 못한 명령을 버리는 것과 짝을 이루는 설정: `rl_controller.h:99` 의 `drop_epm_when_busy() → true`. 설명은 `controller_base.h:126-128` — "매 tick 판단하는 정책은 뒤늦은 발사가 sim 과 어긋난다".
- gate 를 통과한 ±1 이 실제 펄스가 되는 길: `ControllerOutput.epm` → runner 가 latch → manager 가 `consume_pending_epm_command()` 로 **한 번만** 꺼내 `EpmSequencer::trigger` ([controller_manager.cpp](../../../DIDEN_Core/src/applications/controller/manager/controller_manager.cpp) `390-395` 행). 정책 주기(10 ms)가 manager 주기(1 ms)보다 느려도 같은 명령이 열 번 나가지 않는 이유입니다.

---

## Q5. 팔 hold 와 safe-hold

### 배경 (a): 팔은 정책이 건드리지 않는다

RL 정책은 다리 12개만 제어합니다. 팔 6개는 **RL 이 시작된 순간의 자세**를 계속 유지합니다. 매 주기 `ControllerOutput` 의 팔 구간에 그 자세와 팔용 게인을 써넣습니다.

```
out.joint.q [12..18) = held_arm_pos
out.joint.kp[12..18) = arm_kp        (joint_pd_ctrl_config.yaml 의 팔 구간: 200)
out.joint.kd[12..18) = arm_kd        (20)
```

### 배경 (b): 추론이 실패하면 무엇을 내보내나

정책 출력에 NaN 이 섞였다고 합시다. 선택지를 비교해 봅니다.

| 선택 | 결과 |
|---|---|
| NaN 을 그대로 내보낸다 | 모터에 NaN 위치 명령 → 예측 불가 |
| 전부 0 을 내보낸다 | `kp = 0` → 다리에 힘이 빠져 주저앉는다. 벽에 붙어 있었다면 추락 |
| **직전 정상 목표를 유지한다** | 한 주기 전 자세로 버틴다 → 다음 주기에 정책이 회복되면 이어 간다 |

레포는 세 번째를 택합니다. 이것이 **safe-hold** 입니다.

```
다리 q  = last_target_dof_pos        (직전에 내보낸 정상 목표)
다리 kp = joint_kp, kd = joint_kd     (게인은 평소대로)
팔      = (a) 와 같다
dq, tau = 0
epm     = 전부 0                      (깨진 출력으로 자석을 건드리지 않는다)
```

### 테스트 따라가기

(a) `out` 을 `q = 9`, `kp = kd = 7` 로 채운 뒤 호출:
- `q[12] = 0.1`, `q[17] = 0.6`, `kp[12..18) = 200`, `kd[12..18) = 20`
- 다리 구간 `q[0..12) = 9`, `kp[0..12) = 7` — 그대로

(b) `last_target[i] = 0.1·i − 0.5`:
- `q[0] = −0.5`, `q[5] = 0.0`, `q[11] = 0.6`
- `kp[0..12) = 50`, `kd[0..12) = 5`
- 팔은 (a) 와 같음
- `dq`, `tau` 18칸 전부 0, `epm` 8칸 전부 0

### 단계별 힌트

- (a) 팔의 `i` 번째 관절은 `out` 의 `kNumMobileJoints + i` 번째입니다.
- (b) `ControllerOutput safe;` 로 시작하면 모든 필드가 0 입니다 (구조체 정의의 `{}` 초기화). 다리 12칸을 채우고 (a) 를 부르면 끝입니다.

### 흔한 실수

- **safe-hold 의 게인을 0 으로 둠.** 위 표의 두 번째 선택지가 됩니다.
- **현재 측정 자세를 유지.** 측정값 자체가 NaN 의 원인일 수 있습니다. 레포는 "직전에 **내보낸** 명령" 을 씁니다.
- **(a) 에서 다리 구간까지 건드림.**

### 레포 연결

- `rl_controller.cpp:556-571` — `make_safe_hold` (람다). `560-570` 행이 퀴즈의 (b) 와 같습니다.
- `rl_controller.cpp:750-755` — 정상 경로의 팔 hold. 퀴즈의 (a).
- 팔 자세를 잡아 두는 곳: `rl_controller.cpp:136-137` (`on_enter`).
- safe-hold 가 쓰이는 곳: `677-685` (추론 실패), `577-581` (정책 전환 실패). 06 Q6 에서 이어집니다.
- 추가 방어선: runner 스레드는 컨트롤러 출력에 NaN·음수 게인이 있으면 그 주기의 publish 를 건너뛰고(직전 명령 유지), 10 번 연속이면 컨트롤러를 끝냅니다 ([runner/thread.cpp](../../../DIDEN_Core/src/applications/controller/runner/thread.cpp) `30-36`, `43-75`, `175-189` 행). safe-hold 는 그 앞단에서 "유효한 출력" 을 만들어 주는 것입니다.

---

## Q6. 정책 전환 램프 (`blend_ticks`, `SwitchBlend`)

### 배경: 도는 중에 정책을 갈아탄다

레포는 정책을 **체인**으로 이을 수 있습니다 (예: 앉기 → 서기 → 걷기). Ready 상태로 돌아가지 않고 같은 자리에서 다음 정책으로 넘어갑니다.

문제는 두 정책의 자세가 정확히 같지 않다는 것입니다. 앞 정책의 마지막 목표와 뒤 정책의 첫 목표가 다르면 그 순간 명령이 **튑니다.** 그래서 전환 직후 몇 tick 동안은 둘을 섞습니다.

```
alpha  = 1 − (남은 tick / 전체 tick)
target = from + alpha · (target − from)
```

- `from`: 갈아타기 직전에 내보낸 목표 (고정).
- `target`: 새 정책이 이번 tick 에 낸 목표 (매 tick 다름).
- `alpha` 는 0 에서 시작해 tick 마다 `1/전체` 씩 커집니다.

전체 tick 수는 설정(`switch_blend_ms`, 기본 200 ms)에서 환산합니다.

```
blend_ticks = ceil( max(0, switch_blend_ms) · 1e-3 / dt_s )
```

### 테스트 따라가기

`blend_ticks`:

| `(ms, dt_s)` | 기대 |
|---|---|
| `(200, 0.01)` | 20 |
| `(200, 0.02)` | 10 |
| `(0, 0.01)` | 0 — 즉시 전환 |
| `(−50, 0.01)` | 0 |
| `(25, 0.01)` | 3 — `ceil(2.5)` |

4 tick 램프, `from = 1.0`, 새 목표 `3.0`:

| 호출 | 남은 tick (호출 전) | alpha | 결과 |
|---|---|---|---|
| 1 | 4 | 0 | 1.0 |
| 2 | 3 | 0.25 | 1.5 |
| 3 | 2 | 0.5 | 2.0 |
| 4 | 1 | 0.75 | 2.5 |
| 5 | 0 | — | 3.0 (손대지 않음) |

**첫 tick 의 alpha 가 0** 이라는 점을 보세요. 전환한 그 tick 에는 새 정책의 출력이 전혀 반영되지 않고 `from` 이 그대로 나갑니다. 그리고 alpha 는 1 에 도달하지 않습니다 — 0.75 다음은 램프가 끝난 상태(= 새 목표 그대로)입니다.

움직이는 목표 (2 tick 램프, `from = 1`): 목표 `5 → 7 → 9` 에 대해 `1.0`, `4.0`(`1 + 0.5·(7 − 1)`), `9.0`.

### 단계별 힌트

1. 멤버: `from`(12칸), 전체 tick 수, 남은 tick 수.
2. `apply` 는 남은 tick 이 0 이하면 바로 돌아갑니다.
3. `alpha` 계산은 정수 나눗셈이 되지 않게 `double` 로 바꾼 뒤 나눕니다.
4. `apply` 의 끝에서 남은 tick 을 1 줄입니다.

### 흔한 실수

- **정수 나눗셈.** `1 − remaining / ticks` 를 `int` 로 하면 alpha 가 0 아니면 1 입니다.
- **줄이고 나서 alpha 계산.** 첫 tick 의 alpha 가 0.25 가 되어 표가 한 칸씩 밀립니다.
- **`ticks = 0` 에서 0 으로 나눔.** `start(from, 0)` 뒤에는 `apply` 가 아무것도 하지 않아야 합니다. 남은 tick 검사를 먼저 하면 나눗셈까지 가지 않습니다.
- **`from` 을 매 tick 갱신.** `from` 은 전환 시점에 고정됩니다.

### 레포 연결

- `rl_controller.cpp:193-194` — `dt_` 와 `blend_ticks_` 계산.
- `rl_controller.cpp:573-583` — 전환 tick 에 `blend_from_ = last_target_dof_pos_` 를 잡고 `blend_remaining_ = blend_ticks_`.
- `rl_controller.cpp:757-763` — 램프 적용. 다리 12칸에만 적용합니다.
- 램프가 `last_target_dof_pos_` 갱신(`769` 행)보다 **앞**에 있는 이유가 `757-758` 행 주석에 있습니다: 관측(`previous_joint_actions`, `dof_error`)이 "실제로 나간 명령" 을 봐야 하기 때문입니다.
- `rl_config.yaml:18-19` — `switch_blend_ms: 200.0`.
- 자석은 섞지 않습니다. 이산 신호(±1)라 중간값이 없습니다 (`rl_controller.h:212` 주석).
- **단순화한 점**: 레포에서 이 로직은 `RlController::step` 안에 멤버 변수 셋으로 풀어져 있습니다. 퀴즈는 따로 테스트할 수 있게 `SwitchBlend` 클래스로 묶었습니다. 정책 체인 자체(다음 정책 로드, 전환 시점 판정)는 다루지 않고, 06 의 `RlPipeline` 에도 이 램프는 넣지 않습니다.
