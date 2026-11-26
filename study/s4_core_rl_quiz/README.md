# RL 제어 스택 퀴즈 — obs · filter · action · timing · pipeline

DIDEN_Core 가 학습된 정책(ONNX)을 실제 로봇에서 돌리는 코드, 즉 `rl_controller/` 의 핵심을 **직접 다시 만들어 보는** 것이 목표입니다.

```
        매 정책 주기 (예: 10 ms)
┌──────────┐   ┌────────────────────┐   ┌──────────┐   ┌─────────────────────┐   ┌──────────────┐
│ 센서 사본 │ → │ 관측 벡터(float N개) │ → │ 정책 ONNX │ → │ 행동 벡터(float 16개) │ → │ 관절 목표·자석 │ → 모터
└──────────┘   └────────────────────┘   └──────────┘   └─────────────────────┘   └──────────────┘
     01 수식 · 02 필터/history · 03 배치        06 Q2          04 행동 해석                06 Q1
                              05 "몇 Hz 로, 몇 차원으로" 의 계약 · 06 전체를 한 줄로
```

기준 코드는 `World_Core_Study/DIDEN_Core` (커밋 `386bae3a8`) 입니다. GUIDE 의 줄 번호는 전부 이 커밋 기준입니다.

## 진행 방법

```bash
cd study/s4_core_rl_quiz
make 01            # 01 의 전체 문항 검사
make 01 Q=1        # 01 의 1번 문항만 (다른 문항의 테스트는 컴파일에서 빠짐)
make 01 Q="1 2 3"  # 여러 문항
make all           # 01 ~ 06 전부
make verify        # solutions/ 의 정답이 전 문항을 통과하는지 확인
make clean
```

- **고치는 파일**: 각 폴더의 `quiz.h` 하나. `// TODO` 가 붙은 곳을 채웁니다. 클래스의 멤버 변수는 직접 설계합니다.
- **고치지 않는 파일**: `test.cpp`, `Makefile`, `common/`.
- 안 푼 문항은 컴파일은 되고 실행하면 `[FAIL]` 이 나옵니다. `[FAIL]` 줄에 찍힌 식과 줄 번호가 가장 큰 힌트입니다.
- **순서대로 푸세요.** 03 은 01·02 의 `quiz.h` 를, 06 은 01 ~ 05 를 전부 `#include` 합니다. 앞 문제가 틀려 있으면 뒤 문제가 엉뚱한 곳에서 실패합니다.
- 표준 라이브러리만 씁니다 (C++20. host 의 g++ 15 와 dev container 의 g++ 13 에서 확인했습니다). 레포는 Eigen 을 쓰는데, 대응 관계는 02 의 `quiz.h` 머리말에 적어 두었습니다.

| 문제 | 개념 | 레포에서 대응하는 곳 |
|---|---|---|
| 01_obs_math | 쿼터니언 → 회전행렬, `projected_gravity`, 속도 명령 clamp · deadzone, `gait_phi` 인코딩 | `rl_controller/observation.cpp` 의 각 `*Source::fill`, `policy.h` `CommandLimits`, `policy.cpp` `build_command_limits` |
| 02_history_filter | EMA · notch IIR 필터, 필터 체인(`unique_ptr` + 가상 함수), history 버퍼의 두 가지 출력 순서, 첫 tick 의 동작 | `rl_controller/filter.cpp`, `observation.cpp` `HistorySource` |
| 03_obs_layout | term 목록(YAML 순서) → 관측 벡터, 이름 dispatch, history 포장 규칙, 차원 검증 | `observation.cpp` `make_base_term`, `ObservationManager::init / tick / fill_internal` |
| 04_action | 관절 목표 = `action_mean + scale·raw`, 자석 gate(문턱 · 중재 · cooldown), 팔 hold, safe-hold, 정책 전환 램프 | `rl_controller/action.cpp`, `rl_controller.cpp` `step` 의 safe-hold · blend |
| 05_timing_contract | 정책 주기 = `decimation × physics_dt`, 1 kHz tick 위의 주기 스케줄링, 차원 계약, 설정 정합성 | `policy.cpp` `build_control` · `validate_dims` · `resolve_policy_set`, `runner/thread.cpp` |
| 06_pipeline | 제어 한 주기 전체, 추론 NaN 가드, 주기 사이 되먹임, 관절 명령 → `MotorCommand` | `rl_controller.cpp` `on_enter` · `activate_policy` · `step`, `engine/inference/inference_engine.cpp`, `drec_baseboard/tx/thread.cpp` `build_command` |
| 07_onnx_bonus *(보너스)* | 진짜 ONNX Runtime 으로 모델 로드 · 추론 | `engine/inference/inference_engine.cpp` |

07 은 ONNX Runtime 이 설치된 dev container 에서만 빌드됩니다 (`make bonus`). `make all` 과 `make verify` 에는 들어 있지 않습니다. 컨테이너를 띄우는 방법은 [07_onnx_bonus/GUIDE.md](07_onnx_bonus/GUIDE.md) 의 "준비" 에 있습니다.

## 퀴즈가 레포와 다른 점 (단순화)

퀴즈는 레포의 동작과 상수를 그대로 따르되, 아래는 일부러 뺐습니다. 문항별로는 각 GUIDE 의 "레포 연결" 에 다시 적었습니다.

- **Eigen → 표준 라이브러리.** `Eigen::Ref<TensorVector>` 자리에 `std::span<float>`, `segment(offset, n)` 자리에 `subspan(offset, n)`.
- **전역 상태 없음.** 레포의 term 은 `g_system_state` 에서 뜬 스냅샷의 참조를 잡습니다. 퀴즈는 `ObsInputs` / `Snapshot` 구조체를 인자로 넘깁니다.
- **기구학 없음.** 레포는 MuJoCo(`ModelQuery`)로 base 회전행렬과 발 위치를 구합니다. 퀴즈는 쿼터니언에서 회전행렬을 직접 만들고, `relative_foot_pos` term 은 다루지 않습니다.
- **gait 위상은 밖에서 받습니다.** 레포의 `GaitPhiSource` 는 위상을 내부에서 적분합니다 (적응형 주기, 명령 0 일 때 동결, 자석 릴리즈 게이트). 퀴즈는 위상 비율을 받아 (cos, sin) 으로 인코딩하는 부분만 만듭니다.
- **모션 트래킹 없음.** 모션 클립 재생, `motion_*_target` term, `use_motion_reference_offset`, 클립 기반 자석 스케줄, 자석 phase override 는 뺐습니다.
- **자석 gate 의 In/Out staging 없음.** 발 하나가 cell 두 개인 배선(walker)용 단계입니다. spider 는 cell 이 하나라 실제로도 꺼져 있습니다.
- **YAML 파싱 · 번들 · 정책 체인 로드 없음.** 파싱이 끝난 구조체에서 시작합니다.

## 정답과 검증

`solutions/<문제>/quiz.h` 에 참고 정답이 있습니다. 충분히 고민한 뒤에, 또는 통과한 다음 자기 풀이와 비교할 때 보세요.

- `make verify` 는 이 정답으로 01 ~ 06 의 전 문항이 경고 없이(`-Werror`) 통과하는지 확인합니다. 테스트가 의심스러울 때 돌려 보면 됩니다. 여러분의 `quiz.h` 는 건드리지 않습니다 (`.verify/` 에 사본을 만들어 빌드합니다).
- `make verify-bonus` 는 07 의 정답을 같은 방식으로 확인합니다 (dev container 안에서).
- `make check-stubs` 는 퀴즈를 만든 쪽의 점검 도구입니다. 아직 풀지 않은 `quiz.h` 가 문항마다 `[FAIL]` 을 내는지(안 풀었는데 통과하는 문항이 없는지) 봅니다. 풀기 시작한 뒤에는 쓸 일이 없습니다.

---

## 문제별 "생각해 볼 질문"

코드가 통과한 뒤 말로 설명할 수 있는지 확인해 보세요.

### 01 obs math
1. `projected_gravity` 는 왜 yaw(제자리 회전)에는 변하지 않나? 정책 입장에서 이 성질이 왜 편리한가?
2. 레포는 `projected_gravity` 를 IMU 가속도계에서 바로 읽지 않고 추정된 자세(쿼터니언)에서 계산한다. 로봇이 가속 중일 때 두 값은 어떻게 달라지나?
3. `CommandLimits` 의 기본 바운드는 왜 "무한대" 가 아니라 5.0 인가? gRPC 로 `inf` 가 들어오면 clamp 뒤에 무엇이 되나?
4. clamp 를 `command` term 과 `gait_phi` term 이 각자 하면 안 되고, 스냅샷을 뜨는 자리 한 곳에서만 해야 하는 이유는?
5. `command_obs` 는 0 인데 `command_active` 는 true 인 명령이 들어오면 로봇은 어떻게 움직일 것 같은가? (정책은 "명령 0" 을 보는데 gait 위상은 흘러간다)
6. `min > max` 검사를 `if (lower > upper)` 로 쓰면 NaN 바운드에서 어떻게 되나? 그 상태로 `std::clamp` 를 부르면?

### 02 history & filter
1. EMA 의 첫 sample 을 0 에서 시작하지 않고 그 sample 로 잡는(warm start) 이유는? 0 에서 시작하면 로봇을 켠 직후 정책은 무엇을 보게 되나?
2. 필터가 NaN 입력을 받았을 때 "직전 출력을 내보내는" 것과 "NaN 을 그대로 내보내는" 것 중 어느 쪽이 더 안전한가? 각각의 경우 06 의 safe-hold 는 언제 걸리나?
3. `FilterChain` 은 왜 `std::vector<EmaFilter>` 가 아니라 `std::vector<std::unique_ptr<FilterBase>>` 인가? `unique_ptr` 때문에 `HistoryTerm` 은 어떤 제약을 갖게 되나?
4. `history_length: 3, history_interval: 4` 는 정책 주기 10 ms 에서 몇 ms 전까지를 보나? 보관 칸 수가 3 이 아니라 9 인 이유는?
5. history 가 0 에서 시작하면(warm start 없음) 첫 몇 주기 동안 정책은 "과거에 관절 속도가 0 이었다" 고 본다. `dof_error_history` 라면 큰 문제가 아닌데 `dof_pos_history` 라면 왜 문제가 되나?
6. 레포의 `observation.h` 주석과 `observation.cpp` 구현이 warm start 에 대해 서로 다른 말을 한다. 이런 불일치를 발견하면 무엇을 근거로 "실제 동작" 을 판단해야 하나?

### 03 obs layout
1. 관측 벡터의 순서가 학습 때와 한 칸이라도 어긋나면 에러가 나나? 안 난다면 무엇으로 막고 있나? (05 Q4, 그리고 차원 합이 같은 채로 순서만 바뀐 경우는?)
2. `previous_joint_actions` 는 같은 이름인데 `history_length` 를 적느냐에 따라 출력 순서가 뒤집힌다. 정책을 export 하는 쪽(DIDEN_World)과 무엇을 맞춰야 하나?
3. `ObservationManager` 가 `inputs` 를 값으로 복사하지 않고 참조로 잡는 이유는? 그 대가로 지켜야 하는 것은? (06 의 멤버 선언 순서)
4. history 는 `tick()` 에서만 전진하고 `build_actor_input()` 은 `const` 다. 한 주기에 `tick()` 을 두 번 부르면 무엇이 깨지나?
5. Core 는 관측에 scale 이나 평균/분산 정규화를 곱하지 않는다. 그 계산은 어디에 들어 있나? Core 에서 또 정규화하면 어떻게 되나?

### 04 action
1. `action_mean` 은 무엇을 뜻하나? 정책이 0 만 출력하면 로봇은 어떤 자세가 되나?
2. `joint_action_scale` 을 YAML 에서 빠뜨려 0 이 되면 무슨 일이 생기나? 에러 없이 조용히 벌어지는 일이라는 점에서 왜 위험한가? (05 Q4 가 막는다)
3. 자석 gate 는 "지금 자석이 켜져 있는지" 를 모른다. 정책이 +0.9 를 계속 내면 200 ms 마다 ON 신호가 다시 나간다 — 이것은 문제인가? (힌트: `drop_epm_when_busy`, EpmSequencer)
4. cooldown 을 ms 가 아니라 tick 수로 들고 있으면, 정책 주기가 다른 정책으로 바꿀 때 무엇을 다시 해야 하나?
5. safe-hold 에서 kp/kd 를 0 으로 두면 안 되는 이유는? 반대로, 위치가 NaN 일 때 kp 를 살려 두면 안 되는 이유는? (06 Q1)
6. 정책 전환 램프의 첫 tick 은 alpha 가 0 이라 새 정책의 출력이 전혀 반영되지 않는다. 이것이 관측의 `previous_joint_actions` 에는 어떻게 보이나?

### 05 timing & contract
1. 정책 YAML 에 `cycle_time_us` 를 직접 적는 것을 왜 "무시" 가 아니라 "거절" 하나?
2. `static_cast<int>(0.0003 * 10 * 1e6)` 이 2999 가 되는 이유는? 3000 을 얻으려면 어떻게 써야 하나? 2999 µs 여도 1 kHz tick 위에서는 실제 주기가 같은 이유는? 그럼 차이가 드러나는 곳은 어디인가? (자석 cooldown 의 `ceil(ms / dt)`)
3. 놓친 주기를 몰아서 돌리면(catch-up) 왜 위험한가? 반대로 조금 늦은 tick 뒤에 "지금 + 주기" 로 다시 세면 무엇이 누적되나?
4. 정책 주기를 `on_enter()` **전에** 물으면 어떤 값이 나오고(레포 `cycle_time_us()` 의 폴백), 그 결과 무슨 일이 벌어지나?
5. `estimator_output` term 은 YAML 에는 있는데 Core 가 채우지 않는다. 그 값은 어디서 만들어지나?
6. 차원 합만 비교하는 검사가 잡지 못하는 실수는 무엇인가? 그것은 어디에서 잡히나?

### 06 pipeline
1. `step()` 의 1) ~ 7) 중 순서를 바꾸면 안 되는 쌍을 세 개 찾아 보라. (예: 명령 clamp 와 obs tick)
2. 추론이 실패한 주기에 `last_target_dof_pos` 를 갱신하지 않는 이유는? 갱신한다면 무엇으로?
3. 센서 NaN 이 필터가 걸린 term 으로 들어오면 safe-hold 가 걸리지 않고 추론이 계속 돈다. 이것은 좋은 일인가 나쁜 일인가? NaN 이 1 초 동안 계속되면?
4. 정책은 10 ms 마다 돌고 TX 스레드는 2 ms 마다 패킷을 보낸다. 그 사이의 4 개 패킷에는 무엇이 실리나? 자석 신호(±1)도 5 번 실리면 어떻게 되나? (레포 `consume_pending_epm_command`)
5. `RlPipeline` 의 멤버 선언 순서를 바꿔 `ObservationManager` 를 `ObsInputs` 보다 먼저 두면 무슨 일이 생길 수 있나?
6. 위치가 NaN 일 때 `MotorCommand.kp` 까지 0 으로 만드는 것과, runner 가 그런 출력을 아예 publish 하지 않는 것(`output_rejection`)은 각각 어느 단계의 방어인가? 둘 다 필요한 이유는?

### 07 onnx (보너스)
1. 세션을 `init` 에서 한 번만 만들고 `run` 에서는 만들지 않는 이유는? 세션 생성이 제어 루프 안에 들어가면 어떤 일이 생기나?
2. ONNX 가 말해 주는 shape 의 첫 차원이 `-1` 인 이유는? 이것을 1 로 고쳐 쓰는 것이 안전한 근거는?
3. `Ort::Exception` 을 `init` / `run` 안에서 잡지 않고 밖으로 내보내면 어느 스레드가 어떻게 되나?
4. 출력 텐서를 미리 묶어(pre-bind) 넘기는 것과 `Run` 의 반환값에서 결과를 꺼내는 것은 무엇이 다른가?
5. 입력이 유한한데 출력이 `Inf` 가 되는 실제 상황에는 어떤 것이 있을까? 그때 Core 는 무엇을 내보내나? (06 Q6)

---

## 막혔을 때

먼저 **각 폴더의 `GUIDE.md`** 를 보세요. 문항마다 배경, 테스트 따라가기, 단계별 힌트, 흔한 실수, 레포 연결이 정리되어 있습니다. 정답 코드는 적지 않았습니다.

- [01_obs_math/GUIDE.md](01_obs_math/GUIDE.md) — 회전행렬과 중력 투영, 명령 제한
- [02_history_filter/GUIDE.md](02_history_filter/GUIDE.md) — 필터 수식을 손으로 따라가기, history 의 칸 배치 그림
- [03_obs_layout/GUIDE.md](03_obs_layout/GUIDE.md) — 포장 규칙 판정표, offset 계산
- [04_action/GUIDE.md](04_action/GUIDE.md) — 자석 gate 의 tick 별 상태표
- [05_timing_contract/GUIDE.md](05_timing_contract/GUIDE.md) — 스케줄러 시각표, 차원 계약
- [06_pipeline/GUIDE.md](06_pipeline/GUIDE.md) — step 의 순서와 실패 경로
- [07_onnx_bonus/GUIDE.md](07_onnx_bonus/GUIDE.md) — (보너스) 컨테이너 준비, ONNX Runtime API 표

- `[FAIL]` 줄의 식을 그대로 읽으세요. 테스트는 손으로 따라갈 수 있는 숫자만 씁니다.
- 한 문항에 집중하려면 `make 03 Q=4` 처럼 돌리세요.
- 테스트가 틀린 것 같으면 `make verify` 로 정답이 통과하는지 먼저 확인하세요.
- 풀고 나서 Claude 에게 "s4 quiz 03 리뷰해줘" 라고 하면 풀이를 검토합니다.

## 문제를 고치거나 추가할 때 (제작자용)

`quiz.h` 와 `solutions/*/quiz.h` 는 `tools/tpl/NN_name.h` 한 파일에서 만들어집니다. 템플릿 안의 `//@sol` ~ `//@stub` ~ `//@end` 구간이 정답과 빈칸으로 갈립니다.

```bash
python3 tools/gen.py tools/tpl .     # quiz.h 와 solutions/ 를 다시 생성 — 풀던 quiz.h 가 덮어써지므로 먼저 복사해 둘 것
make verify && make check-stubs
```
