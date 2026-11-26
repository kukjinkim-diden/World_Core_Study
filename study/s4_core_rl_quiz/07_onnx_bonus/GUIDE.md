# 07_onnx_bonus 상세 설명 (보너스)

이 문제는 **선택**입니다. 01 ~ 06 을 마친 뒤, "06 Q2 에서 `std::function` 으로 가려 둔 자리에 실제로 무엇이 있는가" 가 궁금할 때 풀어 보세요.

## 준비: dev container 에서만 빌드됩니다

ONNX Runtime 은 host 에 없고 Core 개발용 컨테이너 이미지 안에만 있습니다 (`/usr/local/include/onnxruntime_cxx_api.h`). host 에서 `make bonus` 를 치면 안내 문구와 함께 바로 끝납니다.

가장 간단한 방법은 **퀴즈 폴더를 마운트한 일회용 컨테이너**를 띄우는 것입니다. Core 를 빌드하는 컨테이너는 건드리지 않습니다.

```bash
cd study/s4_core_rl_quiz

# 쓰고 있는 이미지 이름 확인 (이 문서를 쓸 때는 ghcr.io/diden-robotics/core:2.1.1 이었습니다)
docker ps --format '{{.Image}}'

docker run --rm -it \
    --user "$(id -u):$(id -g)" -e HOME=/tmp \
    -v "$PWD":/q -w /q \
    --entrypoint bash \
    ghcr.io/diden-robotics/core:2.1.1

# 컨테이너 안에서
make bonus            # 07 전체
make bonus Q=2        # 2번 문항만
make verify-bonus     # 정답(solutions/)이 통과하는지
```

- `--user` 를 주지 않으면 빌드 산출물(`07_onnx_bonus/build/`)이 root 소유로 남습니다.
- `--rm` 이라 컨테이너에서 나오면 컨테이너는 사라집니다. 고친 `quiz.h` 는 마운트된 폴더에 있으므로 남습니다.
- 01 ~ 06 과 달리 이 폴더는 **CMake** 로 빌드합니다. 컨테이너의 ONNX Runtime 이 정적 라이브러리라 protobuf · re2 · abseil 을 함께 링크해야 하는데, 그 목록을 `find_package(onnxruntime)` 이 대신 챙겨 줍니다. 레포의 `src/engine/inference/CMakeLists.txt` 와 같은 방법입니다. `make bonus` 가 CMake 를 불러 주므로 직접 칠 일은 없습니다.
- 빌드 에러는 `07_onnx_bonus/build.log` 에도 남습니다.

## 큰 그림: `.onnx` 파일 하나가 추론이 되기까지

```
.onnx 바이트 ──(1) 세션 만들기──▶ Ort::Session
                                     │
                    (2) 입력·출력의 이름과 shape 을 물어본다
                                     │
        관측 버퍼 ──(3) 텐서로 감싼다──┤
        행동 버퍼 ──(3) 텐서로 감싼다──┤
                                     ▼
                         (4) session.Run(...)  →  행동 버퍼에 결과가 써진다
```

- (1)(2)는 정책을 올릴 때 **한 번**, (3)(4)는 **매 주기**입니다. 세션 생성은 수백 ms 가 걸릴 수 있어서 제어 루프 안에서 하면 안 됩니다.
- ONNX 모델의 입출력은 **이름**으로 가리킵니다. DIDEN_World 의 export 는 입력을 `"input"`, 출력을 `"output"` 으로 고정합니다. 하지만 엔진은 이름을 가정하지 않고 모델에게 물어봅니다.
- shape 의 첫 차원은 batch 입니다. export 가 batch 를 가변(dynamic)으로 내보내기 때문에 모델에게 물으면 `-1` 이 나옵니다. 로봇에서는 항상 한 번에 하나만 추론하므로 1 로 봅니다.

### 테스트용 모델

`models/` 에 세 개가 들어 있습니다. `make_models.py` 로 만든 것이고, 다시 만들 필요는 없습니다.

| 파일 | 내용 | 쓰는 곳 |
|---|---|---|
| `tiny_policy.onnx` | 입력 `[batch, 6]` → 출력 `[batch, 3]` | Q1 ~ Q3 |
| `dynamic_feature.onnx` | 입력 `[batch, n]` — batch 가 아닌 차원이 가변 | Q1 (거절해야 함) |
| `two_inputs.onnx` | 입력이 두 개 | Q1 (거절해야 함) |

`tiny_policy.onnx` 는 손으로 계산할 수 있는 선형 모델입니다.

```
out0 = x0 + x1
out1 = 2·x2 − x3
out2 = 0.5·x4 + 0.5·x5 + 1
```

### 이 문서의 레포 표기

줄 번호는 `DIDEN_Core` 커밋 `386bae3a8` 기준입니다.

| 줄여 쓴 이름 | 경로 |
|---|---|
| `inference_engine.h` | [DIDEN_Core/include/diden_core/engine/inference/inference_engine.h](../../../DIDEN_Core/include/diden_core/engine/inference/inference_engine.h) |
| `inference_engine.cpp` | [DIDEN_Core/src/engine/inference/inference_engine.cpp](../../../DIDEN_Core/src/engine/inference/inference_engine.cpp) |
| `policy.cpp` | [DIDEN_Core/src/applications/controller/runner/rl_controller/policy.cpp](../../../DIDEN_Core/src/applications/controller/runner/rl_controller/policy.cpp) |

### ONNX Runtime C++ API 에서 쓰는 것들

이 퀴즈에 필요한 API 는 아래가 전부입니다. 정확한 인자는 컨테이너의 `/usr/local/include/onnxruntime_cxx_api.h` 를 열어 확인하세요 — 헤더를 읽고 맞는 오버로드를 고르는 것도 이 문제의 일부입니다.

| 이름 | 역할 |
|---|---|
| `Ort::Env` | 로깅 등 런타임 전역 환경. 세션보다 오래 살아야 한다 |
| `Ort::SessionOptions` | 세션 설정. `SetIntraOpNumThreads` 로 스레드 수를 정한다 |
| `Ort::Session` | 로드된 모델. 생성자 중에 "메모리의 바이트(포인터 + 길이)" 를 받는 것이 있다 |
| `GetInputCount` / `GetOutputCount` | 입력·출력의 개수 |
| `GetInputNameAllocated` / `GetOutputNameAllocated` | 이름. 돌려받은 것에서 문자열을 복사해 둔다 |
| `GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape()` | shape (`std::vector<int64_t>`). 가변 차원은 `-1` |
| `Ort::MemoryInfo::CreateCpu` | "이 메모리는 CPU 에 있다" 는 표시. 텐서를 만들 때 필요 |
| `Ort::Value::CreateTensor<float>` | **이미 있는 버퍼**를 텐서로 감싼다 (복사하지 않는다) |
| `Session::Run` | 추론 |
| `Ort::Exception` | 위의 어느 것이든 실패하면 던진다 |

---

## Q1. `init` — 모델 로드와 차원

### 배경: 생성자와 `init` 을 나누는 이유

"객체를 만드는 것" 과 "모델을 로드하는 것" 을 한 단계로 합치면, 로드가 실패했을 때 반쯤 만들어진 객체가 남습니다. 레포는 둘을 나눕니다.

- **생성자**: 실패하지 않는 일만 합니다 (`Ort::Env` 만들기).
- **`init`**: 실패할 수 있는 일을 하고, 성공/실패를 `bool` 로 알립니다.

`init` 이 성공하기 전의 엔진은 "유효하지 않음" 상태이고, `run` 은 그 상태에서 false 만 돌려줍니다.

### 예외를 경계에서 잡는다

ONNX Runtime 은 실패를 `Ort::Exception` 으로 던집니다. 레포의 규칙은 **예외를 던지는 라이브러리와 만나는 바로 그 자리에서 잡아 `bool` 로 바꾸는 것**입니다. `init` 밖으로 예외가 새어 나가면 제어 스레드가 죽습니다.

깨진 파일을 주면 세션 생성자가 던집니다. `try { ... } catch (const Ort::Exception&) { ... }` 로 `init` 의 몸통 전체를 감쌉니다.

### 차원 계산

```
shape = [-1, 6]      (모델에게 물어본 그대로)
      → [1, 6]       첫 차원이 음수면 1 로 (batch)
      → 음수가 더 남아 있나?  있으면 실패
      → 차원의 곱 = 6        이것이 input_dim
```

`dynamic_feature.onnx` 는 `[-1, -1]` 입니다. 첫 차원을 고쳐도 `[1, -1]` 이라 실패해야 합니다. 입력 벡터의 길이를 정할 수 없는 모델입니다.

### 테스트 따라가기

| 상황 | `init` | `is_valid` | 차원 |
|---|---|---|---|
| `init` 전 | | false | |
| `tiny_policy.onnx` | true | true | 6, 3 |
| `0x5A` 64 바이트 | false | false | 0, 0 |
| 빈 바이트 | false | | |
| `dynamic_feature.onnx` | false | false | |
| `two_inputs.onnx` | false | false | |
| 유효하던 엔진에 깨진 바이트로 다시 `init` | false | **false** | |
| 그 엔진에 다시 `tiny_policy.onnx` | true | true | 6 |

끝에서 두 번째 줄이 중요합니다. 재시도가 실패했는데 이전 모델이 살아 있으면 "올렸다고 생각한 것과 다른 정책" 이 돕니다.

### 단계별 힌트

1. **멤버 설계.** `Ort::Env`, 세션, `Ort::MemoryInfo`, 입력·출력 이름(`std::string`), 입력·출력 shape(`std::vector<std::int64_t>`), 차원 둘, 유효 표시.
2. `Ort::Session` 과 `Ort::MemoryInfo` 는 기본 생성자가 없습니다(정확히는 "빈 것" 을 만드는 방법이 따로 있지만, 레포는 그 대신 `std::optional` 을 씁니다). `std::optional<Ort::Session>` 으로 선언하고 `init` 에서 `emplace(...)` 로 만듭니다. 버릴 때는 `reset()`.
3. `Ort::Env` 는 생성자의 초기화 리스트에서 만듭니다. 인자는 로그 레벨(`ORT_LOGGING_LEVEL_WARNING`)과 이름 문자열입니다.
4. `init` 의 **맨 앞**에서 유효 표시를 내리고 세션을 버리고 차원을 0 으로 되돌립니다. 그래야 중간에 어디서 실패하든 "유효하지 않음" 상태로 남습니다.
5. 이름: `GetInputNameAllocated(0, allocator)` 는 스마트 포인터 같은 것을 돌려줍니다. `.get()` 이 `char*` 이고, 이것으로 `std::string` 을 만들어 **복사**해 둡니다. allocator 는 `Ort::AllocatorWithDefaultOptions` 입니다.
6. shape 을 다루는 작은 함수 셋(첫 차원 고치기, 음수가 남았는지, 전부 곱하기)을 `private static` 으로 빼 두면 `init` 이 읽기 쉬워집니다.
7. 모든 검사를 통과한 **마지막**에 유효 표시를 올립니다.

### 흔한 실수

- **예외를 잡지 않음.** 깨진 바이트 테스트에서 프로그램이 `terminate` 로 죽습니다. `[FAIL]` 도 못 찍고 끝나면 이 경우입니다.
- **`catch` 에서 상태를 되돌리지 않음.** 세션이 반쯤 살아 있거나 차원이 이전 값으로 남습니다.
- **이름을 `const char*` 로 들고 있음.** `GetInputNameAllocated` 가 돌려준 객체가 사라지면 그 포인터는 무효입니다. `std::string` 으로 복사하세요.
- **모든 음수 차원을 1 로 바꿈.** `dynamic_feature.onnx` 가 통과해 버립니다. 고치는 것은 **첫 차원만**입니다.
- **`Ort::Env` 를 `init` 의 지역 변수로 둠.** `init` 이 끝나면 사라지는데 세션은 그것을 계속 씁니다.

### 레포 연결

- `inference_engine.h:41-75` — 클래스 선언. 생성자와 `init` 을 나눈 이유가 `32-34` 행 주석에 있습니다.
- `inference_engine.cpp:53` — 생성자 (`Ort::Env`).
- `inference_engine.cpp:55-110` — `init`. 상태 초기화 `58-59`, 세션 `62-67`, 입출력 개수 `70-79`, 이름 `83-84`, shape `87-90`, dynamic 검사 `92-96`, 차원 `98-99`, `MemoryInfo` `101`, 예외 `105-109`.
- shape 보조 함수: `23-29` (`flatten_dim`), `31-35` (`fix_dynamic_batch`), `38-40` (`has_dynamic_dim`).
- 모델 바이트가 오는 곳: `policy.cpp:691-695`. 번들에서 꺼낸 `std::span<const std::byte>` 를 그대로 넘깁니다. 스레드 수는 `rl_config.yaml` 의 `ort.n_threads`(1)입니다.
- CPU 만 씁니다 (`inference_engine.cpp:65` 주석). GPU 실행 제공자는 켜지 않습니다.
- **단순화한 점**: 레포는 실패 사유를 `ERROR_LOG` 로 남깁니다. 퀴즈는 로그 없이 false 만 돌려줍니다. 레포는 allocator 를 멤버로 두고(`inference_engine.h:64`) 퀴즈의 정답은 지역 변수로 둡니다 — 어느 쪽이든 이름을 복사한 뒤에는 필요 없습니다.

---

## Q2. `run` — 추론

### 배경: 복사 없이 버퍼를 넘긴다

`Ort::Value::CreateTensor<float>(메모리 정보, 포인터, 원소 수, shape 포인터, shape 길이)` 는 새 메모리를 만들지 않고 **주어진 포인터를 텐서로 감싸기만** 합니다.

- 입력: 관측 버퍼를 감쌉니다.
- 출력: 행동 버퍼를 감싸서 `Run` 에 **미리 넘깁니다.** 그러면 ONNX Runtime 이 결과를 그 버퍼에 바로 씁니다. 결과를 꺼내 복사하는 단계가 없습니다.

매 주기 불리는 함수라서 이렇게 힙 할당과 복사를 피합니다.

한 가지 걸리는 점: `CreateTensor` 는 `float*` (비-const)를 요구하는데 입력은 `std::span<const float>` 입니다. ONNX Runtime 이 입력을 고치지 않는다는 것을 알고 있으므로, 레포는 여기서 `const_cast` 를 씁니다. `const_cast` 는 가능한 한 피하는 도구이지만, "읽기만 하는 C API 가 비-const 포인터를 받는" 경우가 정당한 쓰임 중 하나입니다.

### `Run` 의 인자

```
Run(실행 옵션, 입력 이름 배열, 입력 텐서 배열, 입력 개수, 출력 이름 배열, 출력 텐서 배열, 출력 개수)
```

- 이름 배열은 `const char*` 의 배열입니다. 원소가 하나뿐이어도 배열(또는 원소 하나의 주소)로 넘깁니다.
- 실행 옵션은 `Ort::RunOptions{nullptr}` (기본값).

### 테스트 따라가기

| 입력 | 기대 출력 | 계산 |
|---|---|---|
| `(1, 2, 3, 4, 5, 6)` | `(3, 2, 6.5)` | `1+2`, `2·3−4`, `0.5·5+0.5·6+1` |
| 전부 0 | `(0, 0, 1)` | bias 만 |
| `(−1, 1, 0, 2, 4, 0)` × 3 번 | `(0, −2, 3)` | 같은 세션을 반복 사용 |
| 길이 5 인 입력 | false | |
| 길이 4 인 출력 | false, 출력 버퍼 그대로 | |
| `init` 안 한 엔진 | false | |

### 단계별 힌트

1. 검사 순서: 유효한가 → 크기 → (Q3 의 입력 검사) → 텐서 두 개 만들기 → `Run` → (Q3 의 출력 검사).
2. shape 은 Q1 에서 저장해 둔 것(batch 를 1 로 고친 것)을 씁니다. `.data()` 와 `.size()`.
3. 이름은 `std::string` 의 `.c_str()` 로 `const char*` 를 얻어 길이 1 짜리 `std::array<const char*, 1>` 에 담습니다.
4. `Run` 도 예외를 던질 수 있습니다. `try` / `catch` 로 감쌉니다.

### 흔한 실수

- **출력을 미리 묶지 않고 `Run` 의 반환값에서 꺼내 복사.** 동작은 하지만 매 주기 할당이 생깁니다. 레포의 방식은 pre-bind 입니다.
- **shape 을 `[-1, 6]` 그대로 넘김.** 텐서를 만들 때 실패합니다.
- **`.c_str()` 로 얻은 포인터를 멤버 문자열이 바뀐 뒤에 씀.** `Run` 직전에 얻으세요.
- **크기 검사 없이 `CreateTensor`.** 버퍼보다 큰 텐서를 만들어 범위 밖을 읽고 씁니다.

### 레포 연결

- `inference_engine.cpp:122-159` — `run`. 유효성 `125-128`, 크기 `129-136`, 텐서 `145-146`, 이름 배열 `149-150`, `Run` `151`.
- `const_cast` 와 그 이유: `143-145` 행 주석.
- pre-bind 의 이유: `148` 행 주석.
- 레포의 `run` 은 `Eigen::Ref<TensorVector>` 를 받습니다. `.data()` 로 포인터를 꺼내는 것은 `std::span` 과 같습니다.

---

## Q3. NaN 가드

### 배경

06 Q2 에서 만든 것과 같은 규칙입니다. 이번에는 진짜 모델로 확인합니다.

- **입력에 NaN/Inf** → `Run` 을 부르지 않고 false. 출력 버퍼는 건드리지 않습니다.
- **출력에 NaN/Inf** → false. (출력 버퍼에는 이미 그 값이 써져 있습니다. 쓰지 말라는 신호가 false 입니다.)

출력만 깨지는 경우를 만들려고, 테스트는 float 의 범위를 넘기는 입력을 씁니다.

```
x0 = x1 = 3e38      (float 최댓값은 약 3.4e38 — 각각은 유한)
out0 = x0 + x1 = 6e38  → float 로 표현할 수 없음 → +Inf
```

실제 정책에서는 가중치가 깨졌거나 활성값이 폭주할 때 같은 일이 생깁니다.

### 테스트 따라가기

| 입력 | 반환 | 출력 버퍼 |
|---|---|---|
| `x2 = NaN` | false | −999 그대로 (부르지 않았다) |
| `x5 = Inf` | false | −999 그대로 |
| `(3e38, 3e38, 0, 0, 0, 0)` | false | `(Inf, 0, 1)` 이 써져 있음 — 검사하지 않음 |
| 그 뒤 `(1, 2, 3, 4, 5, 6)` | true, `out0 = 3` | 실패가 엔진을 망가뜨리지 않는다 |

### 단계별 힌트

- `std::isfinite` 로 `span` 전체를 보는 작은 함수 하나를 `private static` 으로 두고 두 번 씁니다.
- 입력 검사는 텐서를 만들기 **전**, 출력 검사는 `Run` **뒤**입니다.

### 흔한 실수

- **입력 검사를 `Run` 뒤에 함.** 출력 버퍼가 −999 가 아니게 됩니다.
- **출력이 깨졌을 때 버퍼를 0 으로 지움.** 레포는 그렇게 하지 않습니다. false 를 받은 호출자(`RlController::step`)가 버퍼를 쓰지 않고 safe-hold 로 빠집니다.

### 레포 연결

- `inference_engine.cpp:45-49` — `check_finite`. 발견해도 `DEBUG_LOG` 뿐입니다 (`42-44` 주석: 심각도 판단은 호출자 몫).
- `inference_engine.cpp:137-140` — 입력 검사. `152-154` — 출력 검사.
- 호출자의 대응: [rl_controller.cpp](../../../DIDEN_Core/src/applications/controller/runner/rl_controller/rl_controller.cpp) `675-685` 행 (06 Q6).

---

## 더 해 볼 것

- 06 의 `Model::forward` 에 이 엔진을 끼워 보세요. `forward` 는 `std::function` 이므로, `OnnxEngine` 을 캡처한 람다 한 줄이면 됩니다. 다만 `tiny_policy.onnx` 는 6 → 3 이라 06 의 정책 설정(관측 43, 행동 16)과 맞지 않습니다. `init` 이 거절하는 것을 확인하는 것도 좋은 실험입니다.
- 스터디의 파이썬 환경(`.venv`)에는 `onnx` 가 들어 있습니다. `make_models.py` 를 고쳐 43 → 16 짜리 선형 모델을 만들고 06 의 pipeline 전체를 진짜 ONNX 로 돌려 보세요.
- 실제 정책 `.onnx` 의 입출력 이름과 shape 을 `OnnxEngine` 으로 찍어 보고, 같은 이름의 `.yaml` 에 적힌 term dim 의 합과 비교해 보세요 (05 Q4 의 계약).
