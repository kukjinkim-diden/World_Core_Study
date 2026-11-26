# 02_history_filter 상세 설명

## 큰 그림: term 하나가 거치는 두 단계

01 에서 만든 값은 "지금 이 순간" 의 값입니다. 정책 YAML 은 term 마다 두 가지 가공을 더 지정할 수 있습니다.

```yaml
- name: dof_vel_history
  dim: 36
  history_length: 3          # 최근 3개를
  history_interval: 4        # 4 주기 간격으로
  modifiers:
  - {type: ema, alpha: 0.5}  # 그 전에 EMA 필터를 건다
```

처리 순서는 **필터 먼저, history 나중**입니다.

```
지금 값(raw) ──▶ [ 필터 체인 ] ──▶ 필터된 값 ──▶ [ history 버퍼에 push ] ──▶ 최근 N개를 이어 붙여 출력
                   Q1 ~ Q3                            Q4 ~ Q5
                 └─────────────────── Q6 이 둘을 묶는다 ───────────────────┘
```

학습 때 시뮬레이터(DIDEN_World)가 똑같은 가공을 했기 때문에, Core 도 **같은 수식을 같은 순서로** 해야 정책이 학습 때와 같은 입력을 봅니다. 이 퀴즈의 수식이 하나하나 까다로운 이유입니다.

각 문항은 **배경 → 테스트 따라가기 → 단계별 힌트 → 흔한 실수 → 레포 연결** 순서로 설명합니다. 정답 코드는 적지 않았습니다.

### 이 문서의 레포 표기

줄 번호는 `DIDEN_Core` 커밋 `386bae3a8` 기준입니다.

| 줄여 쓴 이름 | 경로 |
|---|---|
| `filter.h` | [DIDEN_Core/include/diden_core/applications/controller/runner/rl_controller/filter.h](../../../DIDEN_Core/include/diden_core/applications/controller/runner/rl_controller/filter.h) |
| `filter.cpp` | [DIDEN_Core/src/applications/controller/runner/rl_controller/filter.cpp](../../../DIDEN_Core/src/applications/controller/runner/rl_controller/filter.cpp) |
| `observation.h` | [DIDEN_Core/include/diden_core/applications/controller/runner/rl_controller/observation.h](../../../DIDEN_Core/include/diden_core/applications/controller/runner/rl_controller/observation.h) |
| `observation.cpp` | [DIDEN_Core/src/applications/controller/runner/rl_controller/observation.cpp](../../../DIDEN_Core/src/applications/controller/runner/rl_controller/observation.cpp) |
| `policy.h` | [DIDEN_Core/include/diden_core/applications/controller/runner/rl_controller/policy.h](../../../DIDEN_Core/include/diden_core/applications/controller/runner/rl_controller/policy.h) |

### C++ 준비: `std::span` 과 가상 함수

이 퀴즈의 함수들은 `std::vector<float>` 를 직접 받지 않고 `std::span` 을 받습니다.

- `std::span<const float> x`: "어딘가에 있는 float 여러 개" 를 **읽기만** 하는 창. `std::vector<float>` 를 그대로 넘길 수 있습니다.
- `std::span<float> y`: 같은 창인데 **쓸 수 있음**. `y[i] = ...` 로 호출부의 메모리에 바로 씁니다.
- 복사가 일어나지 않습니다. 03 에서 큰 관측 벡터의 일부 구간만 잘라 넘길 때 이 성질을 씁니다.

`FilterBase` 는 **가상 함수**를 가진 인터페이스입니다.

- `EmaFilter` 와 `NotchFilter` 는 `FilterBase` 를 상속하고 `reset`, `step` 을 `override` 합니다.
- 호출부는 `FilterBase*` (정확히는 `std::unique_ptr<FilterBase>`)만 알고 `step` 을 부릅니다. 실제로 어느 필터의 `step` 이 불릴지는 실행 중에 정해집니다.
- 이 구조 덕분에 Q3 의 체인은 "필터가 무엇이든 순서대로 부른다" 로 끝납니다.

---

## Q1. `all_finite` 와 `EmaFilter`

### 배경: 가장 단순한 저역 통과 필터

**EMA**(exponential moving average, 지수 이동 평균)는 "새 값을 조금만 믿고, 나머지는 직전 출력을 유지" 하는 필터입니다.

```
y[n] = α·x[n] + (1 − α)·y[n−1]
```

- `α = 1` 이면 필터 없음 (새 값 그대로).
- `α` 가 작을수록 느리게 따라가고 노이즈가 더 눌립니다.
- golden 정책은 `α = 0.5` 를 씁니다.

두 가지 규약이 더 있습니다.

1. **warm start.** 첫 sample 이 들어오면 `y[−1]` 이 없습니다. 0 으로 치면 로봇을 켠 직후 값이 0 에서 서서히 올라오는 가짜 움직임이 생깁니다. 그래서 첫 sample 은 **그대로 내보내고**, 내부 상태를 그 값으로 잡습니다.
2. **NaN/Inf 가드.** 입력 벡터에 유한하지 않은 값이 하나라도 있으면, 그 sample 을 통째로 버립니다. 상태는 그대로 두고 **직전 출력**을 다시 내보냅니다. EMA 는 직전 출력을 계속 먹는 구조라, NaN 이 한 번 들어가면 영원히 NaN 이 되기 때문입니다.

`all_finite` 는 그 가드에 쓰는 작은 함수이고, 06 에서 추론 입출력 검사에도 다시 씁니다.

### 테스트 따라가기

`α = 0.5`, 2 채널:

| 입력 | 출력 | 계산 |
|---|---|---|
| `(4, −8)` | `(4, −8)` | 첫 sample → 그대로 |
| `(0, −8)` | `(2, −8)` | `0.5·0 + 0.5·4`, `0.5·(−8) + 0.5·(−8)` |
| `(0, 0)` | `(1, −4)` | |
| `(3, 0)` | `(2, −2)` | `0.5·3 + 0.5·1` |

`α = 0.75`: `4 → 4`, `0 → 1` (`0.75·0 + 0.25·4`), `0 → 0.25`.

NaN:

| 입력 | 출력 | 설명 |
|---|---|---|
| `(4, 4)` | `(4, 4)` | warm start |
| `(NaN, 100)` | `(4, 4)` | 한 채널만 NaN 이어도 sample 전체를 버린다. 100 도 반영되지 않는다 |
| `(Inf, 0)` | `(4, 4)` | |
| `(0, 0)` | `(2, 2)` | `0.5·0 + 0.5·4` — 중간의 두 sample 은 없었던 것처럼 |

첫 sample 부터 NaN: `NaN → 0` (reset 이 만든 0), 그다음 `6 → 6` (이때 warm start), `0 → 3`.

in-place: `p.step(buf, buf)` 처럼 입력과 출력이 같은 버퍼여도 결과가 같아야 합니다.

### 단계별 힌트

**`all_finite`**
- `std::isfinite(v)` 는 NaN 과 ±Inf 에서 false 입니다 (`<cmath>`).
- 반복문으로 하나씩 봐도 되고, `<algorithm>` 의 `std::ranges::all_of` 를 써도 됩니다.
- 빈 입력은 true 입니다 ("유한하지 않은 원소가 없다").

**`EmaFilter`**
1. 멤버로 무엇이 필요한지 먼저 정하세요: `α`, 직전 출력(채널 수만큼), "첫 sample 을 받았는가" 표시.
2. 생성자는 `ModifierSpec` 에서 `alpha` 를 꺼내 `float` 로 저장합니다. 관측 파이프라인은 전부 float 입니다.
3. `reset(n)`: 직전 출력을 0 으로 채운 길이 n 의 벡터로 만들고, "첫 sample" 표시를 되돌립니다.
4. `step(x, y)` 의 순서:
   - 먼저 `all_finite(x)` 를 봅니다. 아니면 직전 출력을 `y` 에 복사하고 끝.
   - 첫 sample 이면 직전 출력을 `x` 로 잡습니다.
   - 아니면 채널마다 식을 계산해 **직전 출력을 갱신**합니다.
   - 마지막에 직전 출력을 `y` 에 복사합니다.

### 흔한 실수

- **NaN 을 채널별로 걸러 냄.** 레포는 sample 단위로 버립니다. `(NaN, 100)` 뒤에 `(4, 4)` 가 아니라 `(4, 52)` 가 나오면 이 경우입니다.
- **첫 NaN 에서 "첫 sample 을 받았다" 고 표시함.** 그러면 다음 정상 sample 에서 warm start 가 안 되고 `0.5·6 + 0.5·0 = 3` 이 나옵니다.
- **`y` 에 먼저 쓰고 `x` 를 읽음.** in-place 호출에서 `x` 가 이미 덮어써진 뒤입니다. "상태를 갱신 → 상태를 y 에 복사" 순서면 안전합니다.
- **double 로 계산.** 테스트 숫자는 float 로 정확히 떨어지는 값이라 차이가 안 나지만, 레포는 float 로 계산합니다.

### 레포 연결

- `filter.h:23-49` — `FilterBase` 와 공통 규약 (`32-39` 행에 warm start, in-place, NaN 규약이 적혀 있습니다).
- `filter.cpp:18-42` — `EmaFilter`. NaN 가드 `29-32`, warm start `34-36`, 수식 `39` 행.
- 레포의 수식은 `α·x − (α − 1)·y` 로 적혀 있습니다 (`filter.h:52`, `filter.cpp:39`). `(1 − α)` 대신 `(α − 1)` 을 저장해 두고 빼는 것으로, 값은 같습니다. DIDEN_World 의 필터 계수 표기(`y = X·B − Y·A`)를 그대로 따른 것입니다.
- `all_finite` 에 해당하는 것은 Eigen 의 `x.array().isFinite().all()` 입니다.

---

## Q2. `NotchFilter`

### 배경: 한 주파수만 골라 지우기

모터나 구조물이 특정 주파수(예: 12 Hz)로 떨면 관절 속도 신호에 그 떨림이 실립니다. **notch 필터**는 그 주파수 근처만 지우고 나머지는 통과시킵니다.

구현은 **biquad**(2차 IIR)입니다. 직전 입력 2개와 직전 출력 2개를 기억합니다.

```
y[n] = b0·x[n] + b1·x[n−1] + b2·x[n−2] − a1·y[n−1] − a2·y[n−2]
```

계수 5개는 설정값 3개(`f0` = 중심 주파수, `bw` = 대역폭, `fs` = 샘플링 주파수)에서 계산합니다.

```
w0 = 2π·f0/fs        Q = f0/bw        a = sin(w0)/(2Q)        a0 = 1 + a

b0 = 1/a0            b1 = −2·cos(w0)/a0        b2 = 1/a0
                     a1 = −2·cos(w0)/a0        a2 = (1 − a)/a0
```

계산은 double 로 하고, 저장할 때 float 로 줄입니다.

### 손으로 따라갈 수 있는 계수

테스트는 일부러 계산이 쉬운 값을 골랐습니다: `f0 = 25`, `bw = 50`, `fs = 100`.

- `w0 = 2π·25/100 = π/2` → `cos(w0) = 0`, `sin(w0) = 1`
- `Q = 25/50 = 0.5` → `a = 1/(2·0.5) = 1` → `a0 = 2`
- `b0 = 0.5`, `b1 = 0`, `b2 = 0.5`, `a1 = 0`, `a2 = 0`

그래서 이 필터는 그냥 다음과 같습니다.

```
y[n] = 0.5·x[n] + 0.5·x[n−2]
```

### 테스트 따라가기

위 계수로, 입력 `1, 0, −1, 0, 1` (샘플링 100 Hz 에서 25 Hz 로 진동하는 신호):

| n | x[n] | x[n−2] | y[n] | 설명 |
|---|---|---|---|---|
| 0 | 1 | (1) | 1 | 첫 sample: 과거 4칸을 전부 1 로 잡고 그대로 내보냄 |
| 1 | 0 | (1) | 0.5 | `x[n−2]` 는 warm start 가 채운 1 |
| 2 | −1 | 1 | 0 | |
| 3 | 0 | 0 | 0 | |
| 4 | 1 | −1 | 0 | 25 Hz 성분이 사라졌다 |

실제 계수(`f0 = 12, bw = 4, fs = 100`, `policy.h:54` 의 예):
- `b0 = b2 ≈ 0.8975926`, `b1 = a1 ≈ −1.3086337`, `a2 ≈ 0.7951852`
- 일정한 입력 `(3, −1)` 을 계속 넣으면 `(3, −1)` 이 그대로 나옵니다. 0 Hz 는 통과하기 때문입니다 (`b0 + b1 + b2 = 1 + a1 + a2` 임을 확인해 보세요).
- 그 뒤 첫 채널을 3 → 0 으로 떨어뜨리면: `y = b0·0 + b1·3 + b2·3 − a1·3 − a2·3`. `b1 = a1` 이라 상쇄되고 `3·(b2 − a2) ≈ 0.3072` 가 남습니다. 이 테스트는 **재귀 항(`a1`, `a2`)을 실제로 쓰는지** 확인합니다.

### 단계별 힌트

**`notch_coeffs`**
- 위 공식을 그대로 옮기면 됩니다. `π` 는 `std::numbers::pi_v<double>`.
- `cos(π/2)` 는 컴퓨터에서 정확히 0 이 아니라 `6e-17` 쯤 나옵니다. 테스트는 오차를 허용합니다.

**`NotchFilter`**
1. 멤버: 계수 5개, 직전 입력 2개(`x[n−1]`, `x[n−2]`), 직전 출력 2개(`y[n−1]`, `y[n−2]`). 각각 채널 수만큼의 벡터입니다.
2. `step` 의 큰 순서는 EMA 와 같습니다: NaN 가드 → 첫 sample 이면 warm start → 아니면 수식.
3. 수식 계산이 끝난 뒤 **상태를 한 칸씩 민다**: `x[n−2] ← x[n−1]`, `x[n−1] ← x`, `y[n−2] ← y[n−1]`, `y[n−1] ← 방금 출력`.
4. in-place 에 주의: `x[n−1] ← x` 를 하기 전에 `y` 에 쓰면, `x` 와 `y` 가 같은 버퍼일 때 방금 출력이 `x[n−1]` 로 들어갑니다. 레포는 출력을 임시 버퍼에 먼저 계산해 두고, 상태를 다 민 다음 마지막에 `y` 에 씁니다.
5. NaN 일 때 내보내는 "직전 출력" 은 `y[n−1]` 입니다.

### 흔한 실수

- **상태를 미는 순서.** `x[n−1] ← x` 를 먼저 하고 `x[n−2] ← x[n−1]` 을 하면 둘 다 `x` 가 됩니다. 오래된 것부터 옮기세요.
- **warm start 에서 일부만 채움.** 과거 4칸(입력 2 + 출력 2)을 전부 첫 sample 로 잡아야 "원래부터 그 값이 계속 들어오고 있었던 것" 과 같아집니다.
- **`a1`, `a2` 의 부호.** 식에서 **빼는** 항입니다. 쉬운 계수에서는 둘 다 0 이라 티가 안 나고, 실제 계수의 `0.3072` 테스트에서 드러납니다.
- **첫 sample 에서도 상태를 밀어야 합니다.** warm start 로 전부 같은 값이니 결과는 같지만, 분기 구조를 "출력 계산" 과 "상태 밀기" 로 나누면 실수가 줄어듭니다.

### 레포 연결

- `filter.h:67-90` — `NotchFilter` 선언. `68-70` 행에 공식이 있습니다 (Audio EQ Cookbook 의 notch biquad).
- `filter.cpp:46-58` — 계수 계산. `60-67` reset. `69-93` step (NaN 가드 `71-74`, warm start `76-83`, 수식 `85`, 상태 밀기 `87-90`, 출력 `92`).
- **단순화한 점**: 레포는 계수 계산을 생성자 안에서 합니다. 퀴즈는 테스트하기 쉽게 `notch_coeffs` 함수로 꺼냈습니다.
- golden 정책 YAML 에는 notch 를 쓰는 term 이 없습니다 (전부 EMA). `policy.h:53-54` 의 예와 `observation.cpp:576-577` 의 주석("per-term Notch+EMA")으로 보아 다른 정책에서 쓰는 것으로 보이지만, 어느 정책인지는 확인하지 못했습니다.

---

## Q3. `FilterChain`

### 배경: modifier 목록 = 필터를 순서대로 이어 붙인 것

YAML 의 `modifiers` 는 목록입니다. 여러 개면 순서대로 겁니다 (앞 필터의 출력이 다음 필터의 입력).

```yaml
modifiers:
- {type: notch, center_freq: 12.0, bandwidth: 4.0, sample_rate: 100.0}
- {type: ema, alpha: 0.5}
```

C++ 로는 `std::vector<std::unique_ptr<FilterBase>>` 입니다.

- 원소 타입이 `EmaFilter` 일 수도 `NotchFilter` 일 수도 있어서, 공통 부모의 포인터로 담습니다.
- `std::unique_ptr` 는 "이 객체의 주인은 나 하나" 라는 스마트 포인터입니다. 벡터가 사라지면 필터도 같이 사라집니다. `delete` 를 직접 부를 일이 없습니다.
- `std::make_unique<EmaFilter>(spec)` 으로 만듭니다.

### 할 일

| 함수 | 할 일 |
|---|---|
| `make_filter_chain(mods)` | `mods` 를 앞에서부터 돌면서 `type` 에 맞는 필터를 만들어 벡터에 넣는다 |
| `reset_filter_chain(chain, n)` | 모든 필터의 `reset(n)` 을 부른다 |
| `apply_filter_chain(chain, x, y)` | 비어 있으면 `y = x`. 아니면 첫 필터는 `x → y`, 둘째부터는 `y → y` |

### 테스트 따라가기

EMA(0.5) 두 단, 입력 `8, 0, 0`:

| 입력 | 1단 출력 | 2단 출력 | 설명 |
|---|---|---|---|
| 8 | 8 | 8 | 둘 다 warm start |
| 0 | 4 | 6 | 2단: `0.5·4 + 0.5·8` |
| 0 | 2 | 4 | 2단: `0.5·2 + 0.5·6` |

`[notch, ema]` 와 `[ema, notch]` 에 `1, 0, −1` 을 넣으면 둘 다 마지막에 `0.375` 가 나옵니다 (둘 다 선형 필터라 순서를 바꿔도 같습니다). 중간값은 다릅니다.
- `[notch, ema]`: notch 출력 `1, 0.5, 0` → ema `1, 0.75, 0.375`
- `[ema, notch]`: ema 출력 `1, 0.5, −0.25` → notch `1, 0.75, 0.5·(−0.25) + 0.5·1 = 0.375`

NaN: `[ema, ema]` 에 `8` 다음 `NaN` 을 넣으면 `8`. 1단이 직전 출력 8 을 내보내므로 2단은 NaN 을 보지 못합니다.

### 단계별 힌트

1. `make_filter_chain`: `switch (spec.type)` 으로 나누고, 각 경우에 `chain.push_back(std::make_unique<...>(spec))`.
2. `apply_filter_chain` 에서 둘째 필터부터는 `step(y, y)` 입니다. 그래서 Q1·Q2 의 `step` 이 in-place 에 안전해야 했습니다.
3. 빈 체인의 `y = x` 는 `std::ranges::copy(x, y.begin())` 또는 반복문으로.

### 흔한 실수

- **`FilterChain` 을 복사하려 함.** `unique_ptr` 는 복사가 안 됩니다. 함수에서 `return chain;` 은 이동(move)이라 괜찮고, 다른 곳에 넘길 때는 `std::move` 가 필요합니다.
- **빈 체인에서 `chain.front()` 호출.** 빈 벡터의 `front()` 는 정의되지 않은 동작입니다. 빈 경우를 먼저 처리하세요.
- **모든 단계가 `x` 를 읽음.** 그러면 마지막 필터 하나만 건 것과 같아집니다. EMA 두 단 테스트에서 `6` 대신 `4` 가 나옵니다.
- **`reset` 없이 `step`.** 채널 수가 0 인 상태입니다. 체인을 만든 쪽이 `reset_filter_chain` 을 부를 책임이 있습니다 (Q6 의 생성자).

### 레포 연결

- `filter.h:92-106` — `FilterChain` 과 세 함수의 선언.
- `filter.cpp:97-111` `make_filter_chain`, `113-117` `reset_filter_chain`, `119-129` `apply_filter_chain`.
- `policy.h:57-67` — `ModifierSpec`. 퀴즈의 구조체와 필드가 같습니다.
- `policy.cpp` 의 `to_modifier`(`39-57` 행)가 YAML 을 `ModifierSpec` 으로 바꿉니다. 모르는 `type` 은 파싱 단계에서 에러가 납니다.

---

## Q4. `HistoryBuffer`

### 배경: "최근 N 개" 를 간격을 두고 보기

정책은 한 순간의 값만으로는 추세를 알 수 없습니다. 그래서 최근 값 몇 개를 이어 붙여 줍니다.

설정은 둘입니다.
- `history_length` (N): 몇 개를 보여 줄 것인가.
- `update_interval` (I): 몇 주기 간격으로 고를 것인가.

`N = 3, I = 4` 이면 "지금, 4 주기 전, 8 주기 전" 을 보여 줍니다. 그러려면 최근 **9 주기**를 전부 보관해야 합니다.

```
보관 칸 수 = (N − 1)·I + 1
```

### 칸 배치

레포의 방식은 단순합니다. **칸 0 이 가장 최근**이고, push 할 때마다 전부 한 칸씩 뒤로 밉니다.

```
push 전:   칸0  칸1  칸2  칸3  칸4  칸5  칸6  칸7  칸8
           t    t-1  t-2  t-3  t-4  t-5  t-6  t-7  t-8
                                                    └ 밀려나 사라짐
push 후:   new  t    t-1  t-2  t-3  t-4  t-5  t-6  t-7
```

`fill` 은 여기서 `I` 칸 간격으로 `N` 개를 고릅니다. 고르는 순서가 두 가지입니다.

| 순서 | 출력 블록 i 가 읽는 칸 | `N = 3, I = 4` 일 때 |
|---|---|---|
| `kOldestFirst` | `(N − 1 − i)·I` | 칸 8, 칸 4, 칸 0 → `[t−8, t−4, t]` |
| `kNewestFirst` | `i·I` | 칸 0, 칸 4, 칸 8 → `[t, t−4, t−8]` |

- oldest-first 는 DIDEN_World 의 `*_history` 규약입니다. 시간 순서대로 읽힙니다.
- newest-first 는 RaiSim 에서 학습한 정책의 `previous_joint_actions` 규약입니다 ("직전 목표, 그 전 목표" 순서).
- **어느 것을 쓰는지는 03 Q3 에서 term 의 표기에 따라 정해집니다.** 이 문항은 둘 다 만들 수 있게만 합니다.

sample 이 벡터(`base_dim > 1`)일 때는 sample 단위로 통째로 붙입니다. `base_dim = 2` 에 `(1, 2)`, `(3, 4)` 를 넣으면 oldest-first 는 `[1, 2, 3, 4]`, newest-first 는 `[3, 4, 1, 2]` 입니다.

### 테스트 따라가기

`N = 3, I = 1`, 스칼라:

| push | oldest-first | newest-first | 설명 |
|---|---|---|---|
| 5 | `[0, 0, 5]` | `[5, 0, 0]` | 과거 칸은 아직 0 |
| 6, 7 | `[5, 6, 7]` | `[7, 6, 5]` | |
| 8 | `[6, 7, 8]` | | 5 가 밀려남 |

`N = 3, I = 4`: 1 부터 9 까지 push → `[1, 5, 9]`. 10 을 더 push → `[2, 6, 10]`.

`history_length = 0, update_interval = 0` 으로 만들면 둘 다 1 로 올려서 "최근 값 하나" 가 됩니다.

### 단계별 힌트

1. 멤버: `base_dim`, 길이 `N`, 간격 `I`, 순서, 그리고 칸들. 칸은 `std::vector<Tensor>` (sample 하나가 `Tensor`) 로 두면 레포와 같은 모양입니다.
2. 생성자: `N`, `I` 를 `std::max(1, ...)` 로 올리고, 칸 수를 계산해 0 으로 채운 sample 들로 만듭니다.
3. `push`: 뒤에서부터 `칸[i] = 칸[i−1]` 로 밀고, 칸 0 에 새 sample 을 넣습니다.
4. `fill`: 블록 `i` (0 … N−1)마다 위 표의 식으로 읽을 칸을 정하고, `out` 의 `i·base_dim` 위치부터 `base_dim` 개를 복사합니다.

### 흔한 실수

- **앞에서부터 밀기.** `칸[1] = 칸[0]`, `칸[2] = 칸[1]`, … 순서로 하면 전부 칸 0 의 값이 됩니다. 뒤에서부터 미세요.
- **칸 수를 N 으로.** 간격이 1 일 때만 맞습니다. `I = 4` 테스트(`slot_count() == 9`)가 잡습니다.
- **`size_t` 로 거꾸로 도는 반복문.** `for (size_t i = size − 1; i >= 0; −−i)` 는 끝나지 않습니다 (`size_t` 는 음수가 없어서 0 에서 한 번 더 빼면 아주 큰 수가 됩니다). 조건을 `i > 0` 으로 두고 `칸[i] = 칸[i−1]` 로 쓰면 됩니다.
- **`fill` 이 상태를 바꿈.** `fill` 은 `const` 입니다. 두 번 불러도 같은 값이 나와야 합니다.

### 레포 연결

- `observation.cpp:578-633` — `HistorySource`. 칸 수 계산 `582`, 칸 밀기 `601-606`, 순서별 읽기 `616-621` 행.
- `observation.h:127-146` — 출력 순서의 설명과 `HistoryOrder` enum.
- 레포는 이것을 "ring buffer" 라고 부르지만, 실제로는 인덱스를 돌리는 방식이 아니라 **매번 전부 미는** 방식입니다. 칸이 많아야 9개 남짓이라 이쪽이 단순하고 충분히 빠릅니다.
- **단순화한 점**: 레포의 `HistorySource` 는 필터 전 값을 담는 칸(`raw_buffer_`)을 하나 더 들고 있습니다 (`586`, `603`, `611` 행). 로그에 필터 전/후를 나란히 남기기 위한 것이고 정책 입력과는 무관해서 퀴즈에서는 뺐습니다.

---

## Q5. `warm_start`

### 배경: 헤더 주석과 구현이 다른 곳

Q4 에서 본 것처럼, 첫 push 뒤에는 과거 칸이 0 으로 남습니다. `[0, 0, 5]`.

그런데 레포의 헤더 주석은 이렇게 적혀 있습니다 (`observation.h:135-136`):

> 첫 tick 은 전 슬롯을 그 값으로 warm start 한다 (DidenWorld CircularBuffer 규약) — 기동 직후 과거 슬롯이 0 으로 남지 않는다.

**스터디 기준 커밋(`386bae3a8`)의 구현은 그렇게 하지 않습니다.** `observation.cpp:582-606` 을 보면 칸을 0 으로 만들고(`583` 행), `tick` 은 조건 없이 밀고 넣기만 합니다.

정리하면 이렇습니다.

| 어디 | 첫 값 5 가 들어온 뒤 (`N = 3`) |
|---|---|
| DIDEN_World 학습 시뮬레이터 (`circular_buffer.py` 의 `append`, `177-183` 행) | `[5, 5, 5]` — 전 칸을 첫 값으로 채움 |
| DIDEN_Core `386bae3a8` 의 주석 | `[5, 5, 5]` 라고 적혀 있음 |
| DIDEN_Core `386bae3a8` 의 구현 | `[0, 0, 5]` |
| DIDEN_Core 다른 브랜치 (`origin/feature/697-rgmt-command-window`, 커밋 `ec14f0f0b`) | `warm_start` 플래그가 생김. 기본값 false → `[0, 0, 5]`, true → `[5, 5, 5]` |

그 브랜치의 주석은 "`warm_start = true` 면 첫 tick 에 전 슬롯을 그 값으로 채운다 (false = 0 에서 시작)" 로 바뀌어 있고, 정책 YAML 의 키(`history_warm_start`)에는 "false = 0 으로 시작 — 기존 동작", "누락 = false (기존 정책 YAML 호환)" 이라고 적혀 있습니다. 즉 기준 커밋의 주석이 틀린 것이고, 이후에 선택 기능으로 정리된 것으로 보입니다. 기동 직후 몇 주기 동안 학습 때와 다른 history 를 보게 되는데, 그것을 기본값으로 둔 것이 **의도인지 호환성 때문에 남겨 둔 것인지**는 코드만으로는 알 수 없었습니다 — 팀에 물어볼 만한 질문입니다.

이 문항은 그 플래그를 만듭니다. Q4 는 기본값(false)의 동작이고, 기준 커밋의 실제 동작입니다.

### 할 일

- `warm_start == true` 이고 **아직 한 번도 push 한 적이 없으면**: 모든 칸을 그 sample 로 채우고 끝냅니다.
- 그 외에는 Q4 의 push 그대로입니다.

### 테스트 따라가기

`N = 3, I = 2` (칸 5개), warm start:

| push | 칸 (0 … 4) | 출력 `[칸4, 칸2, 칸0]` |
|---|---|---|
| 5 | `5 5 5 5 5` | `[5, 5, 5]` |
| 6 | `6 5 5 5 5` | `[5, 5, 6]` |
| 7 | `7 6 5 5 5` | `[5, 5, 7]` |
| 8 | `8 7 6 5 5` | `[5, 6, 8]` |

첫 값이 0 인 경우도 테스트합니다. `0` 을 push 한 뒤 `4` 를 push 하면 `[4, 0]` (newest-first) 이어야 합니다. 두 번째 push 가 "또 첫 push" 로 취급되면 `[4, 4]` 가 나옵니다.

### 단계별 힌트

- "push 한 적이 있는가" 를 기억하는 `bool` 멤버가 하나 필요합니다.
- "칸이 전부 0 인가" 로 판단하면 안 됩니다 (위의 첫 값 0 테스트).

### 흔한 실수

- **warm start 한 뒤에 또 밀고 넣음.** 결과는 같아 보이지만(전부 같은 값이라), 그 브랜치의 구현은 채운 뒤 바로 `return` 합니다. 어느 쪽이든 테스트는 통과합니다.
- **기본값을 true 로 바꿈.** Q4 가 깨집니다. 기준 커밋의 동작은 false 입니다.

### 레포 연결

- 기준 커밋: `observation.h:135-136` (주석), `observation.cpp:580-607` (구현 — warm start 없음).
- 다른 브랜치의 구현은 이렇게 확인할 수 있습니다 (읽기만 합니다):
  ```bash
  cd DIDEN_Core
  git show ec14f0f0b:src/applications/controller/runner/rl_controller/observation.cpp | grep -n "warm" -A6
  ```
- DIDEN_World: `packages/diden-common/src/diden_common/buffers/circular_buffer.py` 의 `append` (`142` 행부터. 첫 값으로 전 칸을 채우는 부분은 `177-183` 행).
- 필터의 warm start(Q1, Q2)는 이것과 별개이고, 기준 커밋에서도 **실제로 동작합니다.** 헷갈리지 않도록 표로 정리합니다.

| | 첫 sample 처리 | 기준 커밋 |
|---|---|---|
| EMA / notch 필터 | 그 값으로 상태를 잡음 | 구현되어 있음 |
| history 버퍼 | 과거 칸을 0 으로 둠 | 주석만 warm start 라고 되어 있음 |

---

## Q6. `HistoryTerm` (필터 → history)

### 배경

Q3 의 체인과 Q4 의 버퍼를 한 클래스로 묶습니다. 레포의 `HistorySource` 가 하는 일의 핵심입니다.

```
tick(raw):  raw ──▶ 필터 체인 ──▶ 필터된 값 ──▶ history.push
fill(out):  history.fill(out)
```

두 가지를 기억하세요.

1. **history 에 쌓이는 것은 필터를 거친 값입니다.** 쌓아 둔 raw 를 꺼낼 때마다 필터하는 것이 아닙니다.
2. **길이 1, 간격 1 이면 "필터만 건 term"** 이 됩니다. 레포는 history 가 없는 term 에 modifier 가 붙었을 때 별도 클래스를 만들지 않고 이렇게 감쌉니다.

### 테스트 따라가기

EMA(0.5) + `N = 2`, oldest-first:

| tick 입력 | 필터 출력 | history 출력 | 설명 |
|---|---|---|---|
| 4 | 4 | `[0, 4]` | 필터는 warm start, history 는 0 에서 시작 |
| 0 | 2 | `[4, 2]` | raw 로 쌓았다면 `[4, 0]` 이었을 것 |
| 0 | 1 | `[2, 1]` | |

길이 1 + EMA(0.5): `8 → [8]`, `0 → [4]`, `NaN → [4]` (필터가 막아 줌).

필터 없이 `NaN` 을 넣으면 history 에 NaN 이 그대로 들어갑니다. **NaN 을 막는 것은 필터이지 history 가 아닙니다.** 이 차이는 06 Q6 에서 다시 나옵니다.

### 단계별 힌트

1. 멤버: 필터 체인, `HistoryBuffer`, 필터 출력을 잠깐 담을 `Tensor`(길이 `base_dim`).
2. 생성자에서 할 일이 둘입니다: `HistoryBuffer` 를 인자대로 만들기, 필터 체인을 `base_dim` 채널로 reset 하기.
3. `HistoryBuffer` 는 기본 생성자가 없습니다. 멤버로 두려면 **멤버 초기화 리스트**(`: history_(인자들)`)에서 만들어야 합니다. 생성자 본문 `{ }` 안에서는 이미 늦습니다.
4. `FilterChain filters` 는 값으로 받았으므로 멤버로 옮길 때 `std::move(filters)` 가 필요합니다.
5. `tick` 은 두 줄, `fill` 과 `dim` 은 한 줄씩입니다.

### 흔한 실수

- **history 에 raw 를 push.** 둘째 줄에서 `[4, 0]` 이 나옵니다.
- **필터 체인 reset 을 빼먹음.** 필터의 내부 벡터 길이가 0 이라 `step` 이 아무것도 쓰지 않거나 범위를 벗어납니다.
- **`filters` 를 복사하려 함.** 컴파일 에러 메시지에 `use of deleted function ... unique_ptr` 가 보이면 `std::move` 가 빠진 것입니다.
- **멤버 선언 순서와 초기화 리스트 순서가 다름.** 멤버는 **선언 순서**대로 초기화됩니다. 초기화 리스트를 다른 순서로 쓰면 `-Wreorder` 경고가 납니다.

### 레포 연결

- `observation.cpp:580-590` — 생성자. `589` 행에서 필터 체인을 reset 합니다.
- `observation.cpp:595-607` — `tick`. `596-598` 에서 base 값을 뽑아 필터에 통과시키고, `601-606` 에서 칸을 밀고 넣습니다.
- **단순화한 점**: 레포의 `HistorySource` 는 base term 객체(`std::unique_ptr<ObservationTerm>`)를 직접 품고 있어서 `tick` 이 스스로 값을 뽑습니다. 퀴즈의 `HistoryTerm` 은 값을 인자(`raw`)로 받습니다. 값을 뽑는 일은 03 의 `ObservationManager` 가 합니다.
- "필터만 건 term" 으로 감싸는 곳: `observation.cpp:843-847`.
