# 03_obs_layout 상세 설명

## 큰 그림: YAML 의 목록이 곧 벡터의 설계도

정책 YAML 의 `observations` 는 **순서가 있는 목록**입니다. Core 는 이 목록을 위에서부터 읽어, 같은 순서로 값을 이어 붙입니다.

```yaml
observations:              # offset
- {name: command, dim: 3}                                              #  0
- {name: dof_pos, dim: 12}                                             #  3
- {name: dof_vel_history, dim: 36, history_length: 3, history_interval: 4}   # 15
                                                                       # 51 = total_dim
```

```
관측 벡터:  [ command(3) | dof_pos(12) | dof_vel_history(36) ]
인덱스:       0 .. 2       3 .. 14        15 .. 50
```

- 정책(ONNX)은 "15번째 칸부터 36개가 관절 속도 history" 라는 사실을 **위치로만** 압니다. 이름은 ONNX 안에 없습니다.
- 그래서 순서나 크기가 학습 때와 다르면 **에러 없이 엉뚱한 값이 들어갑니다.** 이것이 이 퀴즈(와 05 의 계약 검사)가 까다로운 이유입니다.

이 퀴즈에서 만드는 `ObservationManager` 가 하는 일은 셋입니다.

| 단계 | 언제 | 할 일 |
|---|---|---|
| `init` | 정책을 올릴 때 한 번 | 목록을 읽어 term 마다 "어떻게 채울지" 를 정해 둔다. 이상하면 거절 |
| `tick` | 매 주기 | history·필터가 있는 term 이 지금 값을 한 칸 쌓는다 |
| `build_actor_input` | 매 주기, tick 뒤 | 순서대로 자기 구간에 값을 쓴다 |

각 문항은 **배경 → 테스트 따라가기 → 단계별 힌트 → 흔한 실수 → 레포 연결** 순서로 설명합니다. 정답 코드는 적지 않았습니다.

### 이 문서의 레포 표기

줄 번호는 `DIDEN_Core` 커밋 `386bae3a8` 기준입니다.

| 줄여 쓴 이름 | 경로 |
|---|---|
| `observation.h` | [DIDEN_Core/include/diden_core/applications/controller/runner/rl_controller/observation.h](../../../DIDEN_Core/include/diden_core/applications/controller/runner/rl_controller/observation.h) |
| `observation.cpp` | [DIDEN_Core/src/applications/controller/runner/rl_controller/observation.cpp](../../../DIDEN_Core/src/applications/controller/runner/rl_controller/observation.cpp) |
| `policy.h` | [DIDEN_Core/include/diden_core/applications/controller/runner/rl_controller/policy.h](../../../DIDEN_Core/include/diden_core/applications/controller/runner/rl_controller/policy.h) |
| `rl_controller.h` | [DIDEN_Core/include/diden_core/applications/controller/runner/rl_controller/rl_controller.h](../../../DIDEN_Core/include/diden_core/applications/controller/runner/rl_controller/rl_controller.h) |
| `rl_controller.cpp` | [DIDEN_Core/src/applications/controller/runner/rl_controller/rl_controller.cpp](../../../DIDEN_Core/src/applications/controller/runner/rl_controller/rl_controller.cpp) |
| golden YAML | [DIDEN_World/tests/contract/seam3_policy_config.golden.yaml](../../../DIDEN_World/tests/contract/seam3_policy_config.golden.yaml) |

---

## Q1. term 이름 해석 (`strip_history_suffix`, `base_term_dim`)

### 배경: 이름 하나에 정보가 둘

`dof_vel_history` 라는 이름에는 두 가지가 들어 있습니다.
- **base**: `dof_vel` — 무슨 값인가.
- **`_history` 접미사**: 예전 YAML 표기에서 "history 를 입힌 것" 이라는 표시.

Core 는 접미사를 떼고 base 이름으로 "어떤 값을 뽑을지" 를 정합니다. history 를 실제로 입힐지는 이름이 아니라 **`history_length` 필드가 있는지**로 정합니다 (Q3). 그래서 `dof_vel` 에 `history_length` 를 적은 것과 `dof_vel_history` 에 적은 것은 같은 term 입니다. golden YAML 은 앞의 표기(`previous_magnetic_action` + `history_length: 2`)와 뒤의 표기(`dof_vel_history`)를 섞어 씁니다.

### 할 일

**(a) `strip_history_suffix`**
- 이름이 `_history` 로 끝나면 그 8 글자를 뗍니다.
- 단, 이름이 `_history` 보다 **길 때만** 뗍니다. `"_history"` 그 자체는 떼면 빈 문자열이 되므로 그대로 둡니다.
- 한 번만 뗍니다.

**(b) `base_term_dim`**

| base 이름 | dim | 무엇 |
|---|---|---|
| `imu_ang_vel` | 3 | 각속도 |
| `projected_gravity` | 3 | 01 Q2 |
| `command` | 3 | 01 Q5 |
| `dof_pos`, `dof_vel`, `dof_error` | 12 | 다리 관절 12개 |
| `target_dof_pos`, `previous_joint_actions` | 12 | 직전 관절 목표 |
| `previous_magnetic_action` | 4 | 다리 4개 |
| `gait_phi` | 8 | 다리 4 × (cos, sin) |
| 그 외 | −1 | 모르는 term |

12, 4, 8 은 `kNumMobileJoints`, `kNumLegs`, `2 * kNumLegs` 로 적으세요. 로봇이 바뀌면(walker 는 관절 13, 다리 2) 이 숫자들이 같이 바뀝니다.

### 테스트 따라가기

| 입력 | 기대 | 설명 |
|---|---|---|
| `"dof_vel_history"` | `"dof_vel"` | |
| `"dof_vel"` | `"dof_vel"` | 접미사 없음 |
| `"_history"` | `"_history"` | 떼면 빈 이름 → 떼지 않음 |
| `"history_of_dof_vel"` | 그대로 | 끝에 붙은 것만 접미사 |
| `"x_history_history"` | `"x_history"` | 한 번만 |
| `base_term_dim("dof_vel_history")` | −1 | 접미사를 떼는 것은 호출부의 일 |
| `base_term_dim("prev_magnetic_action")` | −1 | 철자가 다르면 다른 이름 |

### 단계별 힌트

- `std::string_view` 는 "문자열의 일부를 가리키는 창" 입니다. 02 의 `std::span` 과 같은 개념의 문자열 버전입니다.
- `name.ends_with(suffix)`, `name.remove_suffix(글자 수)`, `name.size()` 를 쓰면 됩니다. `remove_suffix` 는 문자를 지우는 것이 아니라 창의 끝을 당기는 것이라 복사가 없습니다.
- `base_term_dim` 은 `if (base_name == "...") return ...;` 를 나열하면 됩니다. `string_view` 는 `==` 로 문자열 리터럴과 비교할 수 있습니다.

### 흔한 실수

- **`_history` 가 "포함" 되어 있으면 뗌.** `ends_with` 가 아니라 `find` 로 찾으면 `history_of_dof_vel` 이 망가집니다.
- **길이 조건을 빼먹음.** `"_history"` 가 빈 이름이 됩니다.
- **`prev_magnetic_action` 을 받아 줌.** 레포의 클래스 이름과 주석에는 `prev_magnetic_action` 이라는 표기가 남아 있지만(`observation.cpp:459-466`), YAML 에서 받는 이름은 `previous_magnetic_action` 하나뿐입니다(`observation.cpp:659`).

### 레포 연결

- 접미사 떼기: `observation.cpp:788-794`. 접미사 상수는 `636` 행.
- 이름 → term: `observation.cpp:641-674` `make_base_term`. 이름을 `if` 로 나열한 구조가 퀴즈와 같습니다.
- 각 term 의 dim 은 클래스마다 `dim()` 에 있습니다: `46`(imu), `61`(gravity), `79`(dof_pos), `91`, `106`, `120`, `239`(gait), `423`(command), `465`(magnetic).
- `target_dof_pos` 와 `previous_joint_actions` 가 같은 term 으로 가는 곳: `655-658` 행.
- **단순화한 점**: 레포의 `make_base_term` 은 term **객체**(`std::unique_ptr<ObservationTerm>`)를 만들어 돌려주고, dim 은 그 객체에 묻습니다. 퀴즈는 객체를 만들지 않고 "이름 → dim" (Q1)과 "이름 → 값" (Q2) 두 함수로 나눴습니다. 레포에만 있는 term: `relative_foot_pos`, `gait_phase`, `standing_flag`, `magnet_timer_state`, `motion_*_target`, `estimator_output`.

---

## Q2. term 하나의 "지금 값" (`fill_base_term`)

### 배경

이름에 맞는 값을 `ObsInputs` 에서 꺼내 `out` 에 씁니다. 수식이 있는 것은 01 의 함수를 부르면 됩니다.

| base 이름 | 값 | 비고 |
|---|---|---|
| `imu_ang_vel` | `in.gyro` | |
| `projected_gravity` | `projected_gravity(quat_to_rotation(in.base_quat))` | 01 Q1 + Q2 |
| `command` | `command_obs(in.command, cfg.command_threshold)` | 01 Q5. clamp 는 이미 끝난 값 |
| `dof_pos` | `in.joint_q` | |
| `dof_vel` | `in.joint_dq` | |
| `dof_error` | `in.joint_q − in.last_target_dof_pos` | **현재 − 목표** |
| `target_dof_pos` | `in.last_target_dof_pos` | |
| `previous_joint_actions` | `in.last_target_dof_pos` | 위와 같은 값 |
| `previous_magnetic_action` | `in.last_magnetic_action` | |
| `gait_phi` | `gait_phi(in.gait_phase_ratio, cfg.leg_phase_offsets)` | 01 Q6 |

눈여겨볼 점이 셋 있습니다.

1. **`previous_joint_actions` 는 "정책의 raw 출력" 이 아닙니다.** 이름과 달리 직전에 내보낸 **관절 목표 각도**(= `action_mean + scale·raw`)입니다. `target_dof_pos` 와 완전히 같은 값입니다.
2. **`dof_error` 의 부호.** 현재 − 목표입니다. 반대로 하면 정책은 "밀어야 할 방향" 을 거꾸로 봅니다.
3. **scale 이 없습니다.** 다른 RL 프레임워크는 관측에 `obs_scale` 을 곱하거나 평균/분산으로 정규화합니다. 이 스택에서는 그 계산이 **ONNX 그래프 안에** 들어 있습니다(export 할 때 normalizer 를 같이 굽습니다). Core 는 물리 단위의 값을 그대로 넣습니다.

### 테스트 따라가기

입력: `gyro = (0.1, −0.2, 0.3)`, `base_quat` = roll +90°, `joint_q[i] = 0.1·i`, `joint_dq[i] = −i`, `last_target = 전부 1.0`, `command = (0.5, 0.001, −0.2)`, `thr = 0.003`, 위상 0.25 + trot.

| term | 기대 |
|---|---|
| `imu_ang_vel` | `(0.1, −0.2, 0.3)` |
| `projected_gravity` | `(0, −1, 0)` — 01 Q2 의 roll 90° |
| `command` | `(0.5, 0, −0.2)` — 0.001 은 deadzone |
| `dof_pos` | `0.0, 0.1, …, 1.1` |
| `dof_error` | `−1.0, −0.9, …, 0.1` — `0.1·i − 1` |
| `target_dof_pos`, `previous_joint_actions` | 전부 `1.0` |
| `gait_phi` | `[0,1, 0,−1, 0,−1, 0,1]` — 01 Q6 의 둘째 줄 |

모르는 이름(`"foo"`)이나 크기가 다른 `out` 을 주면 `false` 를 돌려주고 **`out` 을 건드리지 않습니다.**

### 단계별 힌트

1. 맨 먼저 `base_term_dim` 으로 dim 을 구해, −1 이거나 `out.size()` 와 다르면 `false`.
2. 값의 타입이 제각각입니다 (`std::array<double, 3>`, `std::array<double, 12>`, `std::array<float, 4>`, `std::array<float, 8>`). 전부 "인덱스로 읽을 수 있고 `float` 로 바꿀 수 있는 것" 이므로, **람다 하나**로 복사를 처리할 수 있습니다. 람다의 매개변수를 `const auto&` 로 받으면 어떤 배열이든 받습니다.
3. `dof_error` 만 두 배열의 차라서 따로 반복문이 필요합니다.
4. `double → float` 는 `static_cast<float>(...)` 로 명시합니다. 그냥 대입해도 되지만, 레포는 좁히는 변환을 항상 드러내 씁니다.

### 흔한 실수

- **검사 전에 씀.** 크기가 틀린 `out` 에 쓰면 범위를 벗어납니다. 테스트의 "건드리지 않는다" 검사는 이 버그를 드러내기 위한 것입니다.
- **`dof_error` 부호.**
- **`command` 에 다시 clamp.** clamp 는 06 의 `step` 이 한 번 합니다. 여기서는 deadzone 만 겁니다.
- **`previous_joint_actions` 에 다른 값을 넣음.** `in` 에는 raw 행동이 아예 없습니다.

### 레포 연결

- `observation.cpp` 의 각 `fill`: `48`(imu), `63-66`(gravity), `81`(dof_pos), `93`(dof_vel), `108`(dof_error — 주석 `100` 행에 부호 규약), `122`(target), `428-435`(command), `467`(magnetic), `318-337`(gait).
- 값의 출처: 센서 값은 `sensor_snap_`, 자세는 `state_snap_.q` → `kin_`, 명령은 `desired_velocity_snap_` 입니다. 전부 `RlController` 의 멤버이고 `step` 진입부에서 갱신됩니다 (`rl_controller.cpp:588-610`).
- 정규화가 ONNX 안에 있다는 근거: DIDEN_World `tools/deploy/export_merged_onnx.py:206` 의 docstring `action = actor( normalize( cat([obs, clamp(estimator(obs))]) ) )` 과 `264-265` 행(normalizer 의 `running_mean` / `running_var` 를 그래프에 싣는 부분). Core 쪽에는 관측에 곱하는 scale 이 없습니다.
- **단순화한 점**: 레포의 `command` term 은 `tick` 에서 명령을 멤버에 복사해 두고(`observation.cpp:426`) `fill` 은 그 사본을 씁니다. 퀴즈는 `fill` 때 바로 읽습니다. 한 주기 안에서는 입력이 바뀌지 않으므로 결과는 같습니다.

---

## Q3. `init` — 포장 방식과 차원 검증

### 배경: term 마다 "어떻게 포장할지" 를 정한다

Q2 로 뽑은 값을 그대로 쓰는 term 도 있고, 02 의 `HistoryTerm` 으로 감싸는 term 도 있습니다. `init` 은 YAML 한 줄(`TermEntry`)을 보고 그것을 정합니다.

### 판정표

term 마다 위에서부터 **먼저 맞는 것 하나**를 고릅니다.

| | 조건 | 포장 | 통과하려면 |
|---|---|---|---|
| (가) | base 이름이 `previous_joint_actions` 이고 `history_length` 가 **없다** | `HistoryTerm(base_dim, dim / base_dim, 1, 필터, kNewestFirst)` | `dim > 0` 이고 `dim` 이 `base_dim` 의 배수 |
| (나) | `history_length` 가 **있다** | `HistoryTerm(base_dim, length, interval, 필터, kOldestFirst)` | `length > 0`, `interval > 0`, `dim == base_dim · length` |
| (다) | `modifiers` 가 비어 있지 않다 | `HistoryTerm(base_dim, 1, 1, 필터, kOldestFirst)` | (아래 공통 검사) |
| (라) | 그 외 | 없음 | (아래 공통 검사) |

- `interval` 은 `history_interval` 이 없으면 1 입니다.
- 필터 체인은 `make_filter_chain(entry.modifiers)` 로 만들어 `HistoryTerm` 에 넘깁니다. (가)(나)에서 modifiers 가 비어 있으면 빈 체인이 넘어갑니다.
- **공통 검사**: 포장이 끝난 term 의 dim(포장했으면 `HistoryTerm::dim()`, 아니면 `base_dim`)이 YAML 의 `dim` 과 같아야 합니다.
- 그 전에, base 이름을 모르면(`base_term_dim == −1`) 바로 실패입니다.

### (가) 는 왜 따로 있나

`previous_joint_actions` 는 두 가지 학습 환경의 규약이 섞여 있습니다.

- RaiSim 에서 학습한 정책: `history_length` 를 적지 않고 **dim 으로 깊이를 줍니다.** `dim: 24` 면 "직전 목표, 그 전 목표" 순서(newest-first)로 2개.
- DIDEN_World 에서 export 한 정책: 다른 term 처럼 `history_length: 2` 를 적습니다. 이 경우 (가)의 조건에 맞지 않아 (나)로 내려가고, **oldest-first** 가 됩니다.

같은 이름, 같은 dim 인데 **`history_length` 를 적었느냐에 따라 앞뒤 12칸이 뒤바뀝니다.** golden YAML(`41-46` 행)은 `history_length: 2` 를 적으므로 (나)입니다.

### 테스트 따라가기

golden YAML 에서 `relative_foot_pos` 한 줄만 뺀 목록 → `init` 성공, `total_dim() == 145`.

```
3 + 3 + 12 + 12 + 8 + 36 + 36 + 24 + 8 + 3 = 145
```

| 목록 | 기대 | 어느 규칙 |
|---|---|---|
| 빈 목록 | 성공 (dim 0) | |
| `foo` 포함 | 실패 | 모르는 term |
| `dof_pos` dim 11 | 실패 | 공통 검사 |
| `dof_vel_history` dim 36, length 3, interval 4 | 성공 | (나) |
| `dof_vel` dim 36, length 3, interval 4 | 성공 | (나) — 접미사가 없어도 됨 |
| `dof_vel_history` dim 36, length 2 | 실패 | (나) `36 ≠ 12·2` |
| length 0 / interval 0 | 실패 | (나) |
| `dof_vel_history` dim 24, length 2, interval 없음 | 성공 | (나) interval = 1 |
| `dof_vel_history` dim 36, **history_length 없음** | 실패 | (라) → `36 ≠ 12` |
| `dof_vel_history` dim 12, history_length 없음 | 성공 | (라) — 그냥 `dof_vel` 과 같다 |
| `previous_joint_actions` dim 12 / 24 / 36 | 성공 | (가) 깊이 1 / 2 / 3 |
| `previous_joint_actions` dim 30 / 0 | 실패 | (가) |
| `target_dof_pos` dim 24 | 실패 | (가)는 이름에 걸린 규칙. (라) → `24 ≠ 12` |
| `imu_ang_vel` dim 3 + EMA | 성공 | (다) |
| `imu_ang_vel` dim 6 + EMA | 실패 | (다) → `6 ≠ 3` |

마지막으로, `init` 을 다시 부르면 이전 슬롯이 남지 않습니다 (`15` 였다가 `3`).

### 단계별 힌트

1. **멤버 설계부터.** term 하나의 정보를 담을 `struct`(레포의 이름은 `Slot`)를 `private` 에 만들고, `std::vector<Slot>` 로 들고 있으세요. 슬롯에 필요한 것:
   - base 이름 (Q2 를 부를 때 씁니다). `std::string` 으로 **복사**해 두세요. `string_view` 로 들고 있으면 `init` 에 넘어온 `terms` 가 사라진 뒤 허공을 가리킵니다.
   - YAML 의 dim, base dim.
   - 포장했다면 그 `HistoryTerm`. 없을 수도 있으니 `std::unique_ptr<HistoryTerm>` (없으면 `nullptr`)가 편합니다.
2. 그 밖의 멤버: `total_dim`, `ObsConfig` 사본, `const ObsInputs*`(참조를 멤버로 들고 있으려면 포인터가 편합니다 — `&inputs` 로 주소를 잡습니다).
3. `init` 의 맨 앞에서 슬롯과 `total_dim` 을 비웁니다.
4. term 마다: base 이름 → base dim → 필터 체인 → 판정표 → 공통 검사 → 슬롯 추가, `total_dim += entry.dim`.
5. `std::make_unique<HistoryTerm>(...)` 에 필터 체인을 넘길 때 `std::move` 가 필요합니다.
6. `Slot` 을 벡터에 넣을 때도 `unique_ptr` 때문에 이동이 필요합니다. 지정 초기화(`Slot{.base_name = ..., .history = std::move(history)}`)와 `push_back` 을 같이 쓰면 깔끔합니다.

### 흔한 실수

- **(나)를 (가)보다 먼저 검사.** 결과는 같아 보이지만, 조건을 "`history_length` 가 없고 이름이 …" 로 정확히 쓰지 않으면 `previous_joint_actions` + `history_length` 가 newest-first 로 가 버립니다. Q5 의 golden 표기 테스트가 잡습니다.
- **접미사로 history 여부를 판단.** `dof_vel_history` + `history_length` 없음이 `36 ≠ 12` 로 실패해야 하는데 통과합니다.
- **`init` 재호출 시 누적.** `total_dim` 이 `18` 이 됩니다.
- **실패했는데 슬롯이 반쯤 남음.** 레포도 그렇습니다(호출부가 실패를 보고 활성화를 포기합니다). 퀴즈도 실패 뒤의 상태는 검사하지 않습니다.

### 레포 연결

- `observation.cpp:776-867` — `ObservationManager::init`.
  - 초기화 `780-782`, 접미사 `791-794`, 모르는 term `803-806`, 필터 체인 `814-815`.
  - (가) `823-829` — 바로 위 `817-822` 행의 주석이 두 규약을 설명합니다.
  - (나) `830-842`, (다) `843-847`, 공통 검사 `850-853`, 슬롯 추가 `855-856`.
- `policy.h:81-87` — `TermEntry`. 퀴즈의 구조체와 같습니다.
- `observation.h:162-209` — `ObservationManager` 선언. 이동 금지는 `169-171` 행.
- **단순화한 점**: 레포의 `init` 은 `estimator_output` 과 `standing_flag` 에 대한 검사가 더 있습니다 (`808-811`, `859-864`). `estimator_output` 은 05 Q4 에서 다룹니다.
- 레포에서 `init` 을 부르는 곳: `rl_controller.cpp:214-219`. 바로 뒤 `223-227` 행이 `total_dim()` 과 ONNX 입력 차원을 비교합니다 (06 Q3).

---

## Q4. YAML 순서 = 벡터 배치 (`build_actor_input`)

### 배경

`build_actor_input(out)` 은 슬롯을 **`init` 에서 넣은 순서 그대로** 돌면서, 각 term 에게 "네 구간은 여기부터 여기까지" 를 잘라 줍니다.

```
offset = 0
슬롯마다:
    구간 = out 의 [offset, offset + dim)
    이 term 의 값을 구간에 쓴다
    offset += dim
```

이 문항은 포장 없는 term((라))만 검사합니다. 그런 term 은 Q2 의 `fill_base_term` 을 부르면 끝입니다.

`inputs` 는 `init` 때 **참조로 잡아 둔 것**이라, 호출부가 값을 바꾸면 다음 `build` 는 새 값을 봅니다. 레포에서 `RlController` 가 매 주기 스냅샷을 덮어쓰고, term 들은 그 스냅샷의 참조를 계속 들고 있는 구조와 같습니다.

### 테스트 따라가기

```
offset:  0          3            15              18             30                          34
         command(3) dof_pos(12)  imu_ang_vel(3)  dof_error(12)  previous_magnetic_action(4)
```

| 확인 | 기대 |
|---|---|
| `out[0..3)` | `(0.5, 0, −0.2)` |
| `out[3..6)` | `(0.0, 0.1, 0.2)` — dof_pos 의 앞 |
| `out[12..15)` | `(0.9, 1.0, 1.1)` — dof_pos 의 뒤 |
| `out[15..18)` | `(0.1, −0.2, 0.3)` |
| `out[18..21)` | `(−1.0, −0.9, −0.8)` |
| `out[30..34)` | `(0.9, −0.9, 0.1, 0)` |

같은 term 을 다른 순서로 넣은 manager 는 그 순서대로 내놓습니다.

그 뒤 `in.gyro` 와 `in.command` 를 바꾸고 다시 `tick` + `build` 하면 새 값이 보입니다 (참조).

마지막 검사: 길이 40 인 버퍼의 앞 34 칸만 `span` 으로 넘기면, 뒤 6 칸은 그대로(−999)여야 합니다.

### 단계별 힌트

- 구간 자르기는 `out.subspan(offset, dim)` 입니다. 복사가 아니라 같은 메모리의 일부를 가리키는 새 `span` 입니다.
- `subspan` 의 인자는 `std::size_t` 입니다. `int` 를 넘기면 부호 변환 경고가 날 수 있으니 `static_cast<std::size_t>` 로 바꾸거나, offset 을 처음부터 `std::size_t` 로 두세요.
- `build_actor_input` 은 `const` 멤버 함수입니다. 멤버를 바꿀 수 없습니다 (바꿀 필요도 없습니다).
- `fill_base_term` 의 반환값(`bool`)은 `init` 이 이미 검증했으므로 여기서는 쓰지 않습니다. `[[nodiscard]]` 가 없으니 그냥 버려도 되지만, 버린다는 뜻을 `(void)` 로 드러낼 수 있습니다.

### 흔한 실수

- **`init` 에서 값을 복사해 둠.** 참조 테스트가 실패합니다.
- **offset 을 `base_dim` 으로 누적.** 포장 없는 term 에서는 같지만 Q5 에서 틀어집니다. 슬롯의 (YAML) dim 으로 누적하세요.
- **`inputs` 포인터를 확인 없이 역참조.** `init` 을 부르기 전에 `build` 가 불리면 `nullptr` 입니다. 방어 한 줄을 넣어 두면 안전합니다.

### 레포 연결

- `observation.cpp:891-908` — `fill_internal`. offset 누적은 `896`, `906` 행, 구간 자르기는 `902`, `904` 행의 `out.segment(out_offset, slot.dim)`.
- `883-885` — `build_actor_input` 은 `fill_internal` 을 부르는 한 줄입니다. 같은 함수가 로깅용(`build_actor_input_raw`)과 예전 estimator 경로(`build_estimator_input`)에도 쓰입니다.
- 참조를 잡는 구조: `observation.h:155-157` 의 주석과 `rl_controller.h:170-178` (스냅샷 멤버들).

---

## Q5. history · 필터 term (`tick`)

### 배경

포장된 term((가)(나)(다))은 값이 **`tick` 에서만** 전진합니다.

```
tick:   포장된 슬롯마다
            지금 값 = fill_base_term(base 이름, …)
            슬롯의 HistoryTerm::tick(지금 값)        ← 필터 → push
build:  포장된 슬롯은 HistoryTerm::fill(구간)
```

- 한 주기에 `tick` 한 번, 그다음 `build`. 이 순서를 지키는 것은 호출부(06 의 `step`)의 책임입니다.
- `build` 를 여러 번 불러도 값은 같습니다. `tick` 없이 입력만 바꿔도 포장된 term 의 출력은 변하지 않습니다.

### 테스트 따라가기

**(나) `dof_pos_history`, 길이 2**

| 동작 | 출력 `[0..12)` | 출력 `[12..24)` | 설명 |
|---|---|---|---|
| `q = 1`, tick | 0 | 1 | 첫 tick: 과거 칸은 0 (02 Q5 참고) |
| `q = 2`, tick | 1 | 2 | oldest-first |
| `q = 3`, **tick 없이** build | 1 | 2 | tick 을 안 했으므로 그대로 |

**(가) `previous_joint_actions`, dim 24, `history_length` 없음**

`last_target = 1` 로 tick, `= 2` 로 tick → `[2 ×12, 1 ×12]` (newest-first: 직전이 앞).

**같은 이름 + `history_length: 2`** (golden 표기) → `[1 ×12, 2 ×12]` (oldest-first).

**(다) `imu_ang_vel` + EMA(0.5)**: `gyro = (4, −8, 0)` tick → `(4, −8, 0)`. `gyro = 0` tick → `(2, −4, 0)`.

**섞인 배치**: `command(3)` + `dof_vel_history(36, 길이 3, 간격 4)`. `dq = 1 … 9` 로 9 번 tick.

```
offset:  0          3            15           27           39
         command    dq @ t−8     dq @ t−4     dq @ t
                    (전부 1)      (전부 5)      (전부 9)
```

**필터 + history**: `previous_magnetic_action`, 길이 2, EMA(0.5). `(1, −1, 0, 0)` tick, `(0, 0, 0, 0)` tick → `[1, −1, 0, 0 | 0.5, −0.5, 0, 0]`.

### 단계별 힌트

1. `tick` 에서 "지금 값" 을 잠깐 담을 버퍼가 필요합니다. 매번 `std::vector` 를 새로 만들지 말고 멤버로 하나 두세요 (가장 큰 base dim 인 12 칸이면 충분합니다). 레포도 제어 루프 안에서 힙 할당을 피하려고 이런 scratch 버퍼를 멤버로 둡니다.
2. 그 버퍼의 앞 `base_dim` 칸만 `span` 으로 잘라 `fill_base_term` 에 넘기고, 같은 `span` 을 `HistoryTerm::tick` 에 넘깁니다.
3. `build` 에서는 슬롯의 `HistoryTerm` 이 있으면 `fill`, 없으면 Q4 의 경로입니다.

### 흔한 실수

- **`build` 안에서 history 를 전진시킴.** `build` 는 `const` 라 컴파일 단계에서 막힙니다. `mutable` 로 우회하지 마세요.
- **포장 없는 term 도 `tick` 에서 무언가 함.** 할 일이 없습니다.
- **(가)와 (나)의 순서를 반대로 만듦.** Q3 의 판정표를 다시 확인하세요.
- **scratch 버퍼를 `base_dim` 이 아니라 `dim` 으로 자름.** history term 의 `dim` 은 `base_dim × 길이` 입니다. `fill_base_term` 이 크기 불일치로 `false` 를 돌려주고 아무것도 쓰지 않습니다 → 0 만 쌓입니다.

### 레포 연결

- `observation.cpp:873-877` — `ObservationManager::tick`. 모든 슬롯의 `tick` 을 부릅니다. 포장 없는 term 은 기본 구현이 아무것도 하지 않습니다 (`observation.h:51`).
- `observation.cpp:595-607` — `HistorySource::tick`. base 의 `tick` 과 `fill` 을 부르고(`596-597`), 필터(`598`), 칸 밀기(`601-606`).
- 호출 순서: `rl_controller.cpp:660` (`tick`) → `667` (`build_actor_input`).
- golden YAML `41-46` 행: `previous_joint_actions` + `history_length: 2`. 위에서 본 것처럼 Core 에서는 oldest-first 로 처리됩니다. **DIDEN_World 의 학습 쪽이 이 term 을 실제로 어떤 순서로 쌓는지는 이 퀴즈를 만들면서 확인하지 않았습니다.** Core 의 (나) 경로 주석(`observation.h:139-140`)은 "DidenWorld `*_history` 규약 = oldest-first" 라고 적고 있습니다.
