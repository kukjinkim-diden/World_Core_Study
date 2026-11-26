# 06_pipeline 상세 설명

## 큰 그림: 값 하나가 센서에서 모터까지 가는 길

01 ~ 05 에서 만든 부품을 이제 한 줄로 잇습니다. 레포에서 값이 흐르는 길과 그것을 맡은 스레드는 이렇습니다.

```
 DREC RX / Simulator
        │  g_system_state.sensor_raw, state
        ▼
┌─ Ctrl_Runner 스레드 (1 kHz tick) ───────────────────────────────┐
│  10 ms 마다 RlController::step()          ← 05 StepScheduler    │
│     ① 스냅샷 + 명령 clamp                  ← 01                  │
│     ② obs tick → 관측 벡터                 ← 02, 03              │
│     ③ 추론 (ONNX Runtime)                  ← Q2                  │
│     ④ 행동 → ControllerOutput              ← 04                  │
│     ⑤ last_* 갱신                                                │
│  → publish(ControllerOutput)                                     │
└───────────────────────────────────────────────────────────────────┘
        │  최신 출력 1개 (output slot)
        ▼
┌─ Controller(manager) 스레드 (1 kHz) ─┐
│  latest_output → g_system_state.cmd.joint                        │
│  자석 신호는 한 번만 꺼내 EpmSequencer 로                          │
└──────────────────────────────────────┘
        │  cmd.joint (q, dq, tau, kp, kd)
        ▼
┌─ DREC TX 스레드 (500 Hz) ────────────┐
│  build_command(): 관절 좌표 → 모터 좌표, double → float   ← Q1    │
│  → MotorCommand × 12 → UDP                                       │
└──────────────────────────────────────┘
```

세 스레드의 주기가 전부 다릅니다 (정책 100 Hz, manager 1 kHz, TX 500 Hz). 정책이 돌지 않는 9 ms 동안에도 **직전 명령이 계속 모터로 나갑니다.**

이 퀴즈의 문항과 레포의 대응:

| 문항 | 퀴즈 | 레포 |
|---|---|---|
| Q1 | `to_motor_commands` | TX 스레드의 `build_command` |
| Q2 | `run_inference` | `InferenceEngine::run` |
| Q3 | `RlPipeline::init` | `Policy::load` + `RlController::on_enter` / `activate_policy` |
| Q4 ~ Q6 | `RlPipeline::step` | `RlController::step` |

새로 배우는 수식은 거의 없습니다. 어려운 것은 **순서**와 **실패했을 때의 경로**입니다.

각 문항은 **배경 → 테스트 따라가기 → 단계별 힌트 → 흔한 실수 → 레포 연결** 순서로 설명합니다. 정답 코드는 적지 않았습니다.

### 이 문서의 레포 표기

줄 번호는 `DIDEN_Core` 커밋 `386bae3a8` 기준입니다.

| 줄여 쓴 이름 | 경로 |
|---|---|
| `rl_controller.h` | [DIDEN_Core/include/diden_core/applications/controller/runner/rl_controller/rl_controller.h](../../../DIDEN_Core/include/diden_core/applications/controller/runner/rl_controller/rl_controller.h) |
| `rl_controller.cpp` | [DIDEN_Core/src/applications/controller/runner/rl_controller/rl_controller.cpp](../../../DIDEN_Core/src/applications/controller/runner/rl_controller/rl_controller.cpp) |
| `policy.cpp` | [DIDEN_Core/src/applications/controller/runner/rl_controller/policy.cpp](../../../DIDEN_Core/src/applications/controller/runner/rl_controller/policy.cpp) |
| `action.cpp` | [DIDEN_Core/src/applications/controller/runner/rl_controller/action.cpp](../../../DIDEN_Core/src/applications/controller/runner/rl_controller/action.cpp) |
| `inference_engine.cpp` | [DIDEN_Core/src/engine/inference/inference_engine.cpp](../../../DIDEN_Core/src/engine/inference/inference_engine.cpp) |
| `runner/thread.cpp` | [DIDEN_Core/src/applications/controller/runner/thread.cpp](../../../DIDEN_Core/src/applications/controller/runner/thread.cpp) |
| `controller_manager.cpp` | [DIDEN_Core/src/applications/controller/manager/controller_manager.cpp](../../../DIDEN_Core/src/applications/controller/manager/controller_manager.cpp) |
| `tx/thread.cpp` | [DIDEN_Core/src/driver/drec_baseboard/tx/thread.cpp](../../../DIDEN_Core/src/driver/drec_baseboard/tx/thread.cpp) |
| `tx/protocol.h` | [DIDEN_Core/include/diden_core/driver/drec_baseboard/tx/protocol.h](../../../DIDEN_Core/include/diden_core/driver/drec_baseboard/tx/protocol.h) |
| `drec_baseboard_config.yaml` | [DIDEN_Core/config/diden_spider/drec_baseboard_config.yaml](../../../DIDEN_Core/config/diden_spider/drec_baseboard_config.yaml) |

---

## Q1. 관절 명령 → `MotorCommand` (`to_motor_commands`)

### 배경: 관절 좌표와 모터 좌표

컨트롤러는 "관절 3번을 1.2 rad 로" 라고 말합니다. 그런데 모터가 듣는 숫자는 다릅니다.

- **방향(direction)**: 모터가 거꾸로 달려 있으면 부호가 반대입니다. `+1` 또는 `−1`.
- **영점(offset)**: 모터의 0 과 관절의 0 이 다른 위치입니다.

```
position = (q + offset) · direction      ← offset 을 먼저 더하고, 방향을 곱한다
velocity = dq · direction                ← 속도·토크에는 offset 이 없다
torque   = tau · direction
kp, kd   = 그대로                         ← 게인은 크기라서 방향을 곱하지 않는다
```

그다음 `double` 을 `float` 로 줄여 패킷에 싣습니다. `MotorCommand` 는 01_constexpr 퀴즈에서 본 `#pragma pack(1)` 구조체입니다 (실제 크기는 24 바이트 — LED 필드가 16 비트입니다).

### "float 로 실을 수 있는가" 와 fail-closed

값이 NaN 이거나 float 의 범위(약 ±3.4e38)를 넘으면 그대로 실을 수 없습니다. 이때 무엇을 싣느냐가 안전 문제입니다.

| 필드 | 실을 수 없을 때 |
|---|---|
| `pos` | 0 |
| `vel`, `tau` | 0 |
| `kp` | 0 — 그리고 **`pos` 가 실을 수 없을 때도 0** |
| `kd` | 0 |

`kp` 줄이 핵심입니다. 위치가 깨져서 `pos = 0` 을 실었는데 `kp = 50` 이 살아 있으면, 모터는 "0 rad 로 가라" 는 진짜 명령으로 받아들여 그쪽으로 힘껏 당깁니다. 그래서 위치를 믿을 수 없으면 **위치 게인을 같이 끕니다.** `kd` 는 남겨 두어 감쇠(브레이크)만 걸리게 합니다. 실패했을 때 "아무것도 안 하는 쪽" 이 아니라 "덜 위험한 쪽" 으로 떨어지게 하는 것을 fail-closed 라고 합니다.

게인이 **음수**인 것도 "실을 수 없는 값" 으로 칩니다 (PD 가 발산합니다).

### 테스트 따라가기

`direction` 은 다리마다 `[−1, −1, +1]`, `offset[i] = 0.5·i`. 명령은 전부 `q = 1, dq = 2, tau = 4, kp = 50, kd = 5`.

| 축 | direction | offset | pos | vel | tau |
|---|---|---|---|---|---|
| 0 | −1 | 0 | `(1 + 0)·−1 = −1` | −2 | −4 |
| 1 | −1 | 0.5 | `(1 + 0.5)·−1 = −1.5` | −2 | −4 |
| 2 | +1 | 1.0 | `(1 + 1)·1 = 2` | 2 | 4 |
| 11 | +1 | 5.5 | 6.5 | | |

`kp = 50`, `kd = 5` 는 전 축 그대로. `mf_high_time`, `mf_low_time`, `led_rgb` 는 0.

깨진 값:

| 조작 | 기대 |
|---|---|
| `q[0] = NaN` | `pos = 0`, **`kp = 0`**, `kd = 5`, `vel = −2` (같은 축의 다른 필드는 정상) |
| `dq[1] = Inf` | `vel = 0`. `pos = −1.5`, `kp = 50` 은 정상 |
| `tau[2] = 1e39` | `tau = 0` (float 최댓값 초과) |
| `kp[3] = −1` | `kp = 0`, `kd = 5`, `pos = −2.5` |
| `kd[4] = NaN` | `kd = 0`, `kp = 50` |
| 축 5 (멀쩡) | 영향 없음 |

### 단계별 힌트

1. "실을 수 있는가" 는 세 번 이상 쓰이므로 람다로 빼 두세요. `std::isfinite` 와 `std::numeric_limits<float>::max()`(을 `double` 로 바꾼 값)를 씁니다.
2. 축마다: 세 값(position, velocity, torque)을 `double` 로 먼저 계산 → 유효성 세 개(`position_valid`, `kp_valid`, `kd_valid`)를 `bool` 로 → 다섯 필드를 채웁니다.
3. `std::array<MotorCommand, 12> motors{};` 로 만들면 전부 0 으로 시작합니다. 자석·LED 필드는 손대지 않으면 됩니다.

### 흔한 실수

- **`q · direction + offset`.** 축 1 에서 `−1 + 0.5 = −0.5` 가 나옵니다. offset 은 관절 좌표에서 더합니다.
- **게인에 direction 을 곱함.** `kp = −50` 이 됩니다.
- **NaN 인 `pos` 에 `kp` 를 살려 둠.** 위에서 설명한 바로 그 사고입니다.
- **`static_cast<float>(NaN)` 을 그대로 실음.** NaN 은 캐스팅해도 NaN 입니다. 검사가 먼저입니다.
- **18 개를 돌림.** DREC 보드에 실리는 것은 `motor.count`(12) 개입니다.

### 레포 연결

- `tx/protocol.h:21-30` — `MotorCommand`.
- `tx/thread.cpp:291-373` — `build_command`. 모터 루프는 `334-350` 행. `kp` 를 끄는 이유가 `346-347` 행 주석에 있습니다.
- `tx/thread.cpp:33-41` — `wire_representable`, `to_wire_float`.
- `drec_baseboard_config.yaml`: `motor.count: 12` (`53` 행), `offset` (`56`), `direction` (`57`), TX 주기 2000 µs (`33`).
- 그 앞 단계: `controller_manager.cpp:342-420` `run_and_publish` 가 runner 의 최신 출력을 `g_system_state.cmd.joint` 로 옮깁니다 (`403-410`). manager 스레드가 1 kHz 로 이것을 부릅니다.
- `tx/thread.cpp:332-333` 의 TODO 주석: offset 은 펌웨어가 영점 설정을 지원하면 없어질 임시 보정입니다.
- **단순화한 점**
  - 자석 펄스 필드(`mf_high_time`, `mf_low_time`)와 LED 는 `build_command` 의 뒷부분(`352-370`)이 다른 상태(`epm_tx`)에서 채웁니다. 퀴즈는 0 으로 둡니다.
  - 패킷 헤더(타임스탬프, 모터 전원 비트마스크)와 checksum 은 다루지 않습니다. 02_byte 퀴즈의 수신 쪽과 짝입니다.
  - 팔 6개가 어느 경로로 나가는지는 이 퀴즈의 범위 밖이라 확인하지 않았습니다 (설정 폴더에 `rb3_config.yaml` 이 따로 있습니다).

---

## Q2. 추론 한 번 (`run_inference`)

### 배경: 신경망 호출을 감싸는 가드

ONNX Runtime 에게 "이 입력으로 계산해 줘" 라고 하는 것 자체는 함수 호출 하나입니다. 레포의 `InferenceEngine::run` 은 그 앞뒤에 검사를 둘러칩니다.

```
1) 엔진이 로드돼 있는가
2) 입력·출력 버퍼의 크기가 모델과 맞는가
3) 입력에 NaN/Inf 가 없는가          ← 있으면 추론을 "부르지 않는다"
4) 추론 호출                          ← ONNX Runtime 이 예외를 던지면 실패
5) 출력에 NaN/Inf 가 없는가
```

- 3) 을 하는 이유: NaN 을 넣으면 NaN 이 나올 뿐이라 돌릴 가치가 없습니다.
- 5) 를 하는 이유: 입력이 멀쩡해도 모델 내부에서 발산할 수 있습니다.
- 실패하면 `false` 만 돌려줍니다. **무엇을 할지는 부른 쪽이 정합니다** (Q6 의 safe-hold).

퀴즈에서는 ONNX Runtime 자리에 `std::function` 을 둡니다. `Model::forward` 가 4) 에 해당하고, 예외 대신 `false` 를 돌려주는 것으로 실패를 표현합니다.

### `std::function` 한 줄 소개

`std::function<bool(std::span<const float>, std::span<float>)>` 는 "이런 모양으로 부를 수 있는 것은 무엇이든 담는 상자" 입니다. 람다, 일반 함수, 함수 객체 모두 들어갑니다.

- 비어 있을 수 있습니다. `if (!model.forward)` 로 확인합니다. 빈 것을 부르면 `std::bad_function_call` 예외가 납니다.
- 테스트는 여기에 **호출 횟수를 세고 정해진 값을 내놓는 람다**를 넣습니다. 그래서 "NaN 입력일 때 부르지 않았는가" 를 검사할 수 있습니다.

### 테스트 따라가기

모델: 입력 3, 출력 2. `forward` 는 출력을 `fill` 로 채우고 `ok` 를 돌려줍니다.

| 상황 | 반환 | `calls` |
|---|---|---|
| 정상 (`fill = 0.5`) | true, 출력 `(0.5, 0.5)` | 1 |
| 입력 길이 2 | false | 1 — 부르지 않음 |
| 출력 길이 3 | false | 1 |
| 입력에 NaN | false | 1 — 부르지 않음 |
| `fill = NaN` | false | 2 — 불렀고, 결과가 깨짐 |
| `fill = Inf` | false | |
| `ok = false` | false | |
| `forward` 가 비어 있음 | false | |

### 단계별 힌트

- 02 의 `all_finite` 를 그대로 씁니다.
- 검사 다섯 개를 순서대로 `if (...) return false;` 로 나열하면 됩니다.
- 크기 비교: `input.size()` 는 `std::size_t`, `model.input_dim` 은 `int` 입니다.

### 흔한 실수

- **NaN 검사를 호출 뒤에만 함.** `calls == 1` 검사에서 2 가 나옵니다.
- **빈 `forward` 를 호출.** 예외로 프로그램이 죽습니다.
- **출력 검사를 빼먹음.**

### 레포 연결

- `inference_engine.cpp:122-159` — `InferenceEngine::run`. 1) `125-128`, 2) `129-136`, 3) `138-140`, 4) `142-151`, 5) `154`, 예외 처리 `155-158`.
- `inference_engine.cpp:45-49` — `check_finite`. NaN 을 발견해도 `DEBUG_LOG` 만 남깁니다. 심각도 판단은 호출자 몫이라는 주석이 `42-44` 행에 있습니다.
- 입출력 버퍼를 복사하지 않고 그대로 ONNX Runtime 에 넘기는 부분(`143-151`)은 보너스 07 에서 직접 해 봅니다.
- **단순화한 점**: 레포의 엔진은 생성 → `init`(ONNX 로드, 입출력 차원 추출) → `run` 의 수명을 갖는 클래스입니다. 퀴즈는 차원과 실행 함수만 담은 구조체로 줄였습니다. `init` 쪽(`55-110`)도 07 에서 다룹니다.

---

## Q3. `init` — 로드 시점의 계약

### 배경: 실패할 수 있는 일은 전부 여기서

레포의 설계 원칙 하나: **`step()` 은 실패하지 않는다.** 1 kHz 로 도는 제어 루프 안에서 "설정이 이상한데요" 를 발견하면 할 수 있는 일이 없습니다. 그래서 검사할 수 있는 것은 전부 그 전에 끝냅니다.

레포에서는 두 단계에 걸쳐 있습니다.
- `Policy::load` — YAML 파싱·검증, ONNX 로드, 차원 계약.
- `RlController::on_enter` / `activate_policy` — 초기 자세 잡기, gate 설정, 두 Manager 조립.

퀴즈의 `init` 은 이 둘을 합친 것입니다. 05 에서 만든 판정 함수들을 순서대로 부르는 것이 대부분입니다.

### 거절 목록

| 검사 | 쓰는 함수 |
|---|---|
| `model.forward` 가 비어 있다 | |
| 주기를 유도할 수 없다 | 05 `derive_cycle_time_us` |
| term 목록에 중복·음수 dim | 05 `validate_term_list` (obs, actions 둘 다) |
| 게인이 이상하다 | 05 `validate_control_gains` |
| YAML ↔ ONNX 차원 | 05 `validate_dims` |
| 모르는 action term, 또는 dim 이 로봇과 다르다 | `joint_actions` 는 12, `magnetic_actions` 는 4 |
| obs term 조립 실패 | 03 `ObservationManager::init` (`estimator_output` 을 뺀 목록으로) |
| 조립한 관측의 dim ≠ ONNX 입력 | `total_dim()` 과 비교 |

마지막 두 줄이 05 의 `validate_dims` 와 겹쳐 보이지만 다른 것을 봅니다.
- `validate_dims`: YAML 에 **적힌** dim 의 합.
- 여기: 실제 term 이 **만든** dim 의 합. 예를 들어 `{name: foo, dim: 3}` 은 합으로는 맞지만 Core 가 모르는 term 입니다.

### 성공했을 때 잡아 둘 상태

| 상태 | 값 | 이유 |
|---|---|---|
| 정책 주기 | 유도한 값 | `cycle_time_us()` 가 돌려줌 |
| 자석 gate | `reset()` 뒤 `configure(주기[ms], foot, global)` | cooldown 은 **정책 주기** 기준 tick 수 |
| `last_target_dof_pos` | `now.joint_q` 의 앞 12개 | 첫 주기의 `dof_error`(현재 − 직전 목표)가 0 이 되게 |
| 팔 hold 자세 | `now.joint_q` 의 뒤 6개 | 이 순간의 팔 자세를 계속 유지 |
| `last_magnetic_action` | 0 | |
| 관측·행동 버퍼 | ONNX 입출력 차원 크기 | 매 주기 새로 할당하지 않게 |
| action term 의 위치 | `joint_actions` 와 `magnetic_actions` 가 행동 벡터의 몇 번째부터인가 | step 에서 잘라 쓸 offset |

`last_target_dof_pos` 를 0 이 아니라 **현재 자세**로 잡는 것이 중요합니다. 0 으로 두면 첫 주기에 정책은 "목표가 0 rad 였는데 지금 관절은 1.5 rad 에 있다" 는 큰 오차를 봅니다.

### 테스트 따라가기

테스트용 정책: 관측 43 (`command 3 | dof_pos 12 | dof_error 12 | previous_joint_actions 12 | previous_magnetic_action 4`), 행동 16, `decimation 10 × physics_dt 0.001`.

| 변형 | 기대 |
|---|---|
| 그대로 | 성공, `cycle_time_us() == 10000` |
| `init` 전 | `cycle_time_us() == 0`, `step` 은 전부 0, 정책 호출 없음 |
| ONNX 입력 42 / 출력 15 | 실패 |
| `forward` 없음 | 실패 |
| `decimation` 없음 | 실패 |
| `action_mean` 11개 / `scale = 0` / `kp = 0` | 실패 |
| obs 에 `dof_pos` 두 번 | 실패 |
| obs 첫 줄을 `{foo, 3}` 으로 | 실패 — 합은 43 그대로지만 모르는 term |
| action 둘째 줄을 `{wheel_actions, 4}` 로 | 실패 — 합은 16 그대로 |
| `joint_actions` dim 13 (ONNX 출력 17, mean 13개로 맞춤) | 실패 — 로봇의 관절 수와 다름 |
| obs 에 `estimator_output(19)` 추가, ONNX 입력 43 | 성공. `step` 때 정책이 받는 관측 길이 43 |
| 실패한 `init` 뒤 | `cycle_time_us() == 0`, `step` 은 0 출력. 다시 `init` 에 성공하면 정상 |

### 단계별 힌트

1. **멤버 설계.** 필요한 것을 먼저 나열해 보세요: 정책 설정 사본, 모델, `ObsInputs`, `ObservationManager`, `MagnetActionGate`, `HoldConfig`, 팔 hold 자세, 관측 버퍼, 행동 버퍼, 주기, action term 의 offset 둘, "준비됐는가" 표시.
2. **선언 순서.** `ObservationManager` 는 `ObsInputs` 멤버의 주소를 들고 있습니다. C++ 은 멤버를 **선언 순서대로 만들고 역순으로 부숩니다.** 가리키는 쪽(`ObservationManager`)을 가리켜지는 쪽(`ObsInputs`)보다 **뒤에** 선언하세요.
3. `init` 의 맨 앞에서 "준비됐는가" 를 false 로, 주기를 0 으로 되돌립니다. 그래야 중간에 `return false` 로 나가도 "실패한 상태" 가 됩니다.
4. 검사는 위 표의 순서대로. `spec` 을 멤버에 복사한 뒤 그 **사본**에서 `remove_estimator_output` 을 부릅니다 (인자는 `const` 입니다).
5. action term 을 돌면서 이름별 offset 을 기록합니다. offset 은 0 에서 시작해 term 의 dim 만큼 누적합니다. 없는 term 은 −1 로 두면 step 에서 "있는가" 를 확인하기 쉽습니다.
6. `Model` 은 값으로 받았으므로 `std::move(model)` 로 멤버에 옮깁니다.
7. 모든 것이 끝난 **마지막 줄**에서 "준비됐는가" 를 true 로.

### 흔한 실수

- **`ObservationManager` 에 지역 변수의 참조를 넘김.** `init` 이 끝나면 그 변수는 사라지고, `step` 에서 허공을 읽습니다. 반드시 멤버 `ObsInputs` 를 넘기세요.
- **`estimator_output` 을 지우지 않고 obs 를 조립.** `ObservationManager` 가 모르는 term 이라 실패합니다.
- **준비 표시를 처음에 true 로 세움.** 중간에 실패해도 `step` 이 돕니다.
- **gate 를 runner tick(1 ms)으로 configure.** cooldown 이 열 배 짧아집니다. Q5 의 자석 검사에서 드러납니다.
- **`last_target_dof_pos` 를 0 으로 둠.** Q4 의 첫 주기 `dof_error` 검사가 잡습니다.

### 레포 연결

- `policy.cpp:685-704` — `Policy::load`.
- `rl_controller.cpp:95-150` — `on_enter`. 팔 hold `137`, `last_target_dof_pos_` 초기화 `139-140` 행.
- `rl_controller.cpp:153-336` — `activate_policy`. gate `171-185`, 버퍼 `187-190`, `dt_` `193`, `ObservationManager` `213-219`, dim 재확인 `221-227`, `ActionManager` `229-234`, 자석 term offset `252-261`.
- `action.cpp:108-136` — `ActionManager::init`. 모르는 이름 `121-124`, dim 불일치 `126-129`, offset 누적 `131-133`.
- 멤버 선언 순서: `rl_controller.h:102-104` 와 `240-243` 의 주석. 두 Manager 를 **맨 마지막**에 선언합니다.
- **단순화한 점**
  - 레포의 `cycle_time_us()` 는 정책이 없을 때 0 이 아니라 manager 주기(1000)를 폴백으로 돌려줍니다 (`rl_controller.cpp:810-814`).
  - 레포는 정책 여러 개(체인)를 한꺼번에 로드하고(`106-121`), MuJoCo 기구학을 초기화하고(`123-128`), 모션 클립·자석 override 를 준비합니다. 전부 뺐습니다.
  - 레포의 action term 은 `ActionManager` 가 객체로 들고 있습니다. 퀴즈는 offset 두 개로 줄였습니다.

---

## Q4. `step` — 첫 주기

### 배경: 일곱 단계

```
0) 준비되지 않았으면 전부 0 인 출력
1) 스냅샷을 ObsInputs 로 옮긴다. 속도 명령은 여기서 clamp
2) obs.tick()
3) obs.build_actor_input(관측 버퍼)
4) run_inference(모델, 관측 버퍼, 행동 버퍼)        → 실패하면 safe-hold 를 return (Q6)
5) 행동 분배: joint_actions → 관절 목표,  magnetic_actions → tanh → gate → epm
6) 팔 hold
7) last_target_dof_pos, last_magnetic_action 갱신
```

5) 의 자석 경로에서 **tanh** 가 처음 나옵니다. 정책의 raw 출력은 범위가 없는 숫자인데, `tanh` 가 그것을 `[−1, 1]` 로 눌러 줍니다. 04 의 gate 가 받는 `normalized` 가 이 값입니다.

### 테스트 따라가기

초기 자세: 다리 `q[i] = 0.1·i`, 팔 `q[12 + i] = 1.0 + 0.1·i`. 명령 `(1.0, 0.001, −0.1)`. 가짜 정책은 관절 raw 를 전부 0.5, 자석 raw 를 `(2, −2, 0.1, 0)` 으로 냅니다.

**정책이 받은 관측** (43 칸):

| 구간 | term | 기대 | 설명 |
|---|---|---|---|
| `[0, 3)` | command | `(0.3, 0, −0.1)` | 1.0 → clamp 0.3, 0.001 → deadzone 0 |
| `[3, 15)` | dof_pos | `0.0, 0.1, …, 1.1` | |
| `[15, 27)` | dof_error | 전부 0 | 직전 목표 = init 때의 현재 자세 |
| `[27, 39)` | previous_joint_actions | `0.0, 0.1, 0.2, …` | 같은 이유로 현재 자세 |
| `[39, 43)` | previous_magnetic_action | 전부 0 | |

**내보낸 출력**:

| 필드 | 기대 | 계산 |
|---|---|---|
| `q[0]`, `q[11]` | −0.7377, 2.0552 | `mean + 0.4·0.5` |
| `kp[0..12)`, `kd[0..12)` | 50, 5 | |
| `q[12]`, `q[17]` | 1.0, 1.5 | init 때의 팔 자세 |
| `kp[12..18)`, `kd[12..18)` | 200, 20 | |
| `dq`, `tau` | 전부 0 | |
| `epm[0..4)` | `(1, 0, 0, 0)` | 아래 |
| `epm[4..8)` | 전부 0 | 본체 자석은 정책 영역이 아님 |

자석: `tanh(2) = 0.964`, `tanh(−2) = −0.964`, `tanh(0.1) = 0.0997`. 문턱 ±0.3 을 넘는 것은 발 0(+1)과 발 1(−1). `max_action_only` 가 켜져 있고 `|값|` 이 같으므로 번호가 작은 **발 0** 만 통과합니다.

마지막 검사: 그 뒤 스냅샷의 팔 관절이 9.0 으로 바뀌어도 출력의 팔 목표는 1.0 그대로입니다 (hold 자세는 init 때 한 번 잡습니다).

### 단계별 힌트

1. 스냅샷의 관절 배열은 18 칸, `ObsInputs` 는 12 칸입니다. 앞 12 개만 옮깁니다.
2. 행동 버퍼에서 구간 자르기: `std::span<const float>(action_buf).subspan(offset, 길이)`.
3. 자석: 4 칸짜리 `std::array<float, 4>` 에 `std::tanh` 값을 담고 → `gate.tick(…)` → 결과 4 칸을 `out.epm` 의 앞 4 칸에 복사.
4. 7) 에서 쓸 tanh 값은 5) 에서 만든 그 배열입니다. 다시 계산하지 마세요.
5. `ControllerOutput out;` 은 0 으로 시작하므로 `dq`, `tau`, `epm[4..8)` 은 그대로 두면 됩니다.

### 흔한 실수

- **clamp 를 빼먹음.** command 관측이 `(1.0, 0, −0.1)` 이 됩니다.
- **tick 과 build 의 순서.** 포장 없는 term 만 있는 이 테스트에서는 티가 안 나지만, history term 이 있으면 한 주기 늦은 값이 나갑니다.
- **tanh 를 빼먹음.** raw 2 와 −2 는 어차피 문턱을 넘으므로 이 테스트는 통과하고, Q5 의 `previous_magnetic_action` 검사(`0.964`)에서 걸립니다.
- **팔 hold 를 매 주기 스냅샷에서 다시 읽음.** 마지막 검사가 잡습니다.

### 레포 연결

- `rl_controller.cpp:548-793` — `step`. 퀴즈의 단계와 대응:

| 퀴즈 | 레포 | 줄 |
|---|---|---|
| 0) | `ready_` 가드 | `551-554` |
| 1) | 스냅샷 `588-595`, clamp | `602-607` |
| | (기구학 갱신, 모션 클립 — 퀴즈에 없음) | `610`, `615-657` |
| 2) | `obs_manager_->tick(dt_)` | `660` |
| 3) | `build_actor_input` | `667` |
| 4) | `engine().run` | `677-685` |
| 5) | `action_manager_->apply` | `697` |
| 6) | 팔 hold | `750-755` |
| | (전환 램프 — 04 Q6) | `759-763` |
| 7) | `last_target_dof_pos_` `769`, `last_magnetic_action_` | `777-790` |

- 자석 term 의 tanh: `action.cpp:76-88`. `magnetic_action_scale` 을 곱하지 않는 이유가 `63-66` 행 주석에 있습니다 (학습 쪽이 항상 1.0).
- 행동 벡터를 잘라 term 에 넘기는 곳: `action.cpp:142-146`.

---

## Q5. `step` — 주기 사이에 이어지는 값

### 배경: 정책은 자기가 방금 한 일을 본다

7) 에서 갱신한 두 값은 **다음 주기의 관측**이 됩니다.

```
주기 n:    … → 정책 → 목표 T_n ─────────────┐
                       tanh(자석) M_n ──────┤
                                            ▼  last_target_dof_pos, last_magnetic_action
주기 n+1:  관측 [ …, dof_error = q − T_n, previous_joint_actions = T_n, previous_magnetic_action = M_n ] → 정책 → …
```

이 되먹임이 한 주기라도 어긋나면(예: 갱신을 빼먹거나, 다른 값으로 갱신하면) 정책은 학습 때와 다른 세상을 봅니다.

두 가지를 구분하세요.
- `previous_joint_actions` 는 **실제로 내보낸 목표**입니다 (raw 가 아님).
- `previous_magnetic_action` 은 **tanh 값**입니다. gate 를 통과했는지와 무관합니다. cooldown 에 막혀 실제로는 스위칭하지 않았어도, 정책의 "의도" 가 그대로 실립니다.

### 테스트 따라가기

로봇은 가만히 있다고 가정합니다 (스냅샷 불변, 명령 0).

| 주기 | 정책 출력 (관절, 자석) | 그 주기에 정책이 본 관측 | 출력 |
|---|---|---|---|
| 1 | `0.5`, `(2, −2, 0.1, 0)` | (Q4 와 같음) | 목표 `mean + 0.2`, epm `(1,0,0,0)` |
| 2 | `0`, `(2, −2, 0, 0)` | `previous_joint_actions = mean + 0.2` → `[27] = −0.7377`, `[38] = 2.0552` | 목표 `mean`, epm 전부 0 |
| | | `dof_error = 0.1·i − (mean + 0.2)` → `[15] = 0.7377`, `[26] = −0.9552` | |
| | | `previous_magnetic_action = (0.964, −0.964, 0.0997, 0)` | |
| 3 | 같음 | `previous_joint_actions = mean` → `[27] = −0.9377` | epm 전부 0 |
| 4, 5 | 같음 | | epm 전부 0 |
| 6 | 같음 | | epm `(0, −1, 0, 0)` |

자석을 따라가 봅니다 (정책 주기 10 ms → foot 20 tick, global 5 tick).

| 주기 | global (줄인 뒤) | foot0 | 판정 |
|---|---|---|---|
| 1 | 0 | 0 | 발 0 통과 → global = 5, foot0 = 20 |
| 2 | 4 | 19 | 막힘 |
| 3 | 3 | 18 | 막힘 |
| 4 | 2 | 17 | 막힘 |
| 5 | 1 | 16 | 막힘 |
| 6 | 0 | 15 | 발 0 은 자기 cooldown → 남은 후보 발 1(−1) 통과 |

`init` 에서 gate 를 1 ms 로 configure 했다면 global 이 50 tick 이 되어 주기 6 에서도 막힙니다.

**1 kHz 루프 검사.** 테스트의 뒷부분은 runner 스레드를 흉내 냅니다.

```
0 ms 부터 25 ms 까지 1 ms 마다:
    scheduler.due(now) 이면  latest = pipeline.step(snap)
    motors = to_motor_commands(latest.joint, …)      ← 매 tick
```

- 정책 호출은 3 번 (0, 10, 20 ms).
- 26 tick **전부**에서 `motors[0].pos == −0.7377`, `kp == 50`. 정책이 돌지 않은 tick 에도 직전 명령이 나갑니다.

### 단계별 힌트

- 이 문항은 Q4 의 7) 을 제대로 했는지를 검사합니다. 새로 만들 것은 없습니다.
- 실패하면 어느 주기의 어느 칸이 틀렸는지 `[FAIL]` 줄에서 확인하고, 위 표와 비교하세요.

### 흔한 실수

- **`last_target` 을 raw 로 갱신.** `[27]` 에 `0.5` 가 나옵니다.
- **`last_magnetic_action` 을 gate 출력(±1/0)으로 갱신.** `(1, 0, 0, 0)` 이 나옵니다. 학습 때 정책이 본 것은 tanh 값입니다.
- **`last_magnetic_action` 을 raw 로 갱신.** `(2, −2, 0.1, 0)`.
- **갱신을 5) 보다 앞에서 함.** 한 주기 전의 값이 계속 나옵니다.

### 레포 연결

- `rl_controller.cpp:765-790` — 7) 단계. `last_magnetic_action_` 은 경우에 따라 넷으로 갈립니다 (`777-790`). 퀴즈가 다루는 것은 세 번째 가지(`784-787`: 정책이 `magnetic_actions` 를 출력 → `tanh(raw)`)와 네 번째(`788-790`: 자석 term 없음 → 0)입니다.
- `last_target_dof_pos_` 갱신이 전환 램프 **뒤**, 즉 "실제로 나간 명령" 이 확정된 뒤에 있다는 점 (`757-758` 주석, `769`).
- 정책이 돌지 않는 tick 에 직전 명령이 유지되는 구조:
  - `runner/thread.cpp:159-163` — 주기 전이면 `return` (publish 하지 않음 → output slot 에 직전 값이 남음).
  - `controller_manager.cpp:388` — manager 는 매 ms `latest_output()` 을 읽어 `cmd.joint` 에 씁니다.
  - 자석 신호만은 예외로 **한 번만** 소비됩니다 (`390-395`). 같은 ±1 이 열 번 나가면 안 되기 때문입니다.

---

## Q6. `step` — 추론 실패와 safe-hold

### 배경: 4) 에서 실패하면

`run_inference` 가 false 를 돌려주는 경우는 셋입니다 (Q2).
- 관측에 NaN/Inf — 센서 값이 깨졌다.
- 추론 자체가 실패 — ONNX Runtime 오류.
- 행동에 NaN/Inf — 모델이 발산했다.

어느 경우든 대응은 같습니다: **04 의 `make_safe_hold` 를 만들어 바로 `return`.**

"바로 return" 이 중요합니다. 5) ~ 7) 을 건너뛰므로:

| 건너뛴 것 | 결과 |
|---|---|
| 5) 관절 분배 | 깨진 출력이 모터로 가지 않는다 |
| 5) 자석 gate | 자석을 건드리지 않고, **cooldown 도 줄지 않는다** |
| 7) `last_target` 갱신 | 다음 주기에도 "마지막 정상 목표" 가 그대로 남아 있다 |

반면 1) ~ 3) 은 이미 돌았습니다. history 는 한 칸 전진했고, 필터 상태도 갱신됐습니다.

### 필터가 NaN 을 삼키는 경우

02 Q1 에서 필터는 NaN 입력을 받으면 직전 출력을 내보냈습니다. 그 결과:

| 깨진 센서 값이 들어간 term | 관측 벡터 | 추론 |
|---|---|---|
| 필터 **없는** term | NaN 포함 | 부르지 않음 → safe-hold |
| 필터 **있는** term | 직전 값으로 대체되어 유한 | **그대로 돈다** |

golden 정책은 센서 term 대부분에 EMA 가 걸려 있습니다. 그래서 gyro 가 한 주기 NaN 을 내도 safe-hold 에 빠지지 않고 직전 값으로 넘어갑니다. 좋은 점(순간 노이즈에 강함)과 위험한 점(센서가 죽어도 조용함)이 함께 있습니다.

### 테스트 따라가기

| 주기 | 무슨 일 | 정책 호출 수 | 출력 |
|---|---|---|---|
| 1 | 정상. 관절 0.5, 자석 `(2, 0, 0, 0)` | 1 | 목표 `mean + 0.2`, epm `(1,0,0,0)` |
| 2 | 행동의 `[3]` 이 NaN | 2 | safe-hold |
| 3 | `forward` 가 false | 3 | safe-hold |
| 4 | 스냅샷 `joint_q[0] = NaN` | **3** (부르지 않음) | safe-hold |
| 5 | 정상 복귀. 관절 0, 자석 `(0, −2, 0, 0)` | 4 | 목표 `mean` |

safe-hold 의 내용 (테스트의 `is_safe_hold`):
- `q[0] = −0.7377`, `q[11] = 2.0552` — 주기 1 의 목표 그대로
- `kp[0..12) = 50`, `kd[0..12) = 5`
- 팔: `q[12] = 1.0`, `kp[12..18) = 200`
- `epm` 8 칸 전부 0

주기 2 를 자세히 보면: 행동 벡터에서 NaN 은 관절 3번 하나뿐이고, 자석 쪽 `(0, −2, 0, 0)` 은 멀쩡합니다. 그래도 **출력 전체를 버립니다.** "멀쩡해 보이는 부분만 쓰기" 는 하지 않습니다.

주기 5 에서 정책이 본 관측:
- `previous_joint_actions[0..3) = (−0.7377, −1.3307, 2.0549)` — 주기 1 의 목표. 주기 2 ~ 4 동안 갱신되지 않았다.
- `previous_magnetic_action = (0.964, 0, 0, 0)` — 주기 1 의 tanh.

자석 cooldown (global 5 tick): 주기 1 에서 5 로 채워진 뒤, **성공한 주기에만** 줄어듭니다.

| 주기 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 |
|---|---|---|---|---|---|---|---|---|---|
| gate 가 돌았나 | O | X | X | X | O | O | O | O | O |
| global (줄인 뒤) | 0 → 5 | 5 | 5 | 5 | 4 | 3 | 2 | 1 | 0 |
| epm | `(1,0,0,0)` | — | — | — | 0 | 0 | 0 | 0 | `(0,−1,0,0)` |

실패가 없었다면 발 1 은 주기 6 에 통과했을 것입니다.

마지막 블록: `imu_ang_vel` term 을 하나 더 붙인 정책(관측 46)에서 gyro 에 NaN 을 넣습니다.
- 필터 없음 → 두 번째 주기에 정책을 부르지 않는다 (호출 수 1).
- EMA(0.5) → 부른다 (호출 수 2).

### 단계별 힌트

- `if (!run_inference(…)) return make_safe_hold(…);` 한 줄이 4) 의 전부입니다.
- `make_safe_hold` 에 넘길 "직전 목표" 는 `ObsInputs` 안의 `last_target_dof_pos` 입니다.
- 필터 블록은 새로 만들 것이 없습니다. 02 와 03 을 제대로 풀었다면 저절로 통과합니다.

### 흔한 실수

- **실패했는데 5) ~ 7) 을 돌림.** `last_target` 이 NaN 으로 갱신되거나, gate 가 돌아 cooldown 표가 어긋납니다.
- **safe-hold 를 "현재 측정 자세" 로 만듦.** 주기 4 에서 그 측정 자세가 바로 NaN 입니다.
- **gate 를 4) 보다 앞에서 돌림.** 주기 9 대신 주기 6 에 통과합니다.
- **입력 NaN 을 pipeline 이 따로 검사.** `run_inference` 가 이미 합니다. 같은 검사를 두 곳에 두지 마세요.

### 레포 연결

- `rl_controller.cpp:675-690` — 추론 호출과 실패 처리. 실패 로그는 **상태가 바뀔 때만** 남깁니다 (`679-683`, `686-690`). 1 kHz 루프에서 매번 로그를 찍으면 로그가 폭주하기 때문입니다. 퀴즈에는 로그가 없어 이 부분은 뺐습니다.
- `rl_controller.cpp:556-571` — `make_safe_hold`.
- `rl_controller.h:234-238` — "NaN / Inf 가드는 `InferenceEngine::run` 이 전담한다" 는 주석.
- 필터의 NaN 규약: [filter.h](../../../DIDEN_Core/include/diden_core/applications/controller/runner/rl_controller/filter.h) `37-39` 행.
- 그 바깥의 방어선들 — 어느 층이 무엇을 막는지 정리하면:

| 층 | 막는 것 | 대응 | 위치 |
|---|---|---|---|
| 필터 | 센서의 순간 NaN | 직전 출력 유지 | `filter.cpp:29-32` |
| `InferenceEngine::run` | 관측/행동의 NaN | false 반환 | `inference_engine.cpp:138-140`, `154` |
| `RlController::step` | 추론 실패 | safe-hold 출력 | `rl_controller.cpp:677-685` |
| runner 스레드 | 컨트롤러 출력의 NaN·음수 게인·범위 밖 자석 신호 | 그 주기 publish 생략, 10 연속이면 컨트롤러 종료 | `runner/thread.cpp:43-75`, `175-189` |
| TX 스레드 | 그래도 남은 NaN | 0 을 싣고 kp 를 끔 (Q1) | `tx/thread.cpp:340-349` |

- **단순화한 점**: 레포에는 실패 연속 횟수 카운터(`run_fail_count_`)와 복귀 로그가 있습니다. 또 safe-hold 주기에도 로깅용 관측 publish(`668-673`)는 이미 끝난 상태입니다. 퀴즈는 출력과 내부 상태만 맞춥니다.
