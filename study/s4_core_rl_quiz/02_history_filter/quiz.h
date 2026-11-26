// ============================================================================
// 02. history & filter — "관측 한 항에 시간축을 입힌다"
// ============================================================================
// 이 파일만 수정한다. 검사: `make 02` (문항별: `make 02 Q=4`)
//
// 01 에서 만든 것은 "지금 이 순간"의 값이다. 정책은 여기에 두 가지 가공을 더 얹어 받는다.
//   - filter  : 센서 노이즈를 눌러 주는 IIR 필터 (EMA, notch). term 마다 따로 건다.
//   - history : 최근 N 개의 값을 이어 붙인 것 (정책이 속도·추세를 볼 수 있게).
//
// 레포: src/applications/controller/runner/rl_controller/filter.cpp
//       src/applications/controller/runner/rl_controller/observation.cpp (HistorySource)
//
// 레포는 Eigen 을 쓰고 이 퀴즈는 표준 라이브러리만 쓴다. 대응 관계:
//   Eigen::Matrix<float, Dynamic, 1> (TensorVector)  ↔  std::vector<float> (Tensor)
//   Eigen::Ref<TensorVector> out                    ↔  std::span<float> out
//   out.segment(offset, n)                          ↔  out.subspan(offset, n)
// ============================================================================
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <numbers>
#include <span>
#include <utility>
#include <vector>

namespace quiz {

// 레포: include/diden_core/engine/inference/tensor.h — 관측/행동은 전부 float 1차원 벡터다.
using Tensor = std::vector<float>;

// 정책 YAML 의 modifiers 한 항목. 레포: policy.h ModifierSpec
//   {type: ema,   alpha: 0.75}
//   {type: notch, center_freq: 12.0, bandwidth: 4.0, sample_rate: 100.0}
struct ModifierSpec {
    enum class Type {
        kEma,
        kNotch,
    };
    Type type = Type::kEma;
    double alpha = 0.0;        // EMA
    double center_freq = 0.0;  // Notch [Hz]
    double bandwidth = 0.0;    // Notch [Hz]
    double sample_rate = 0.0;  // Notch [Hz]
};

// 채널별로 독립 적용되는 필터의 인터페이스. 레포: filter.h FilterBase
//   - reset(n) 을 step() 전에 1 회 불러 채널 수를 정한다.
//   - x 와 y 가 같은 버퍼여도(in-place) 결과가 같아야 한다.
class FilterBase {
public:
    virtual ~FilterBase() = default;
    virtual auto reset(int n) -> void = 0;
    virtual auto step(std::span<const float> x, std::span<float> y) -> void = 0;
};

// ----------------------------------------------------------------------------
// Q1. all_finite 와 EmaFilter
//   (a) all_finite: 모든 원소가 유한(NaN 도 Inf 도 아님)하면 true. 빈 입력은 true.
//   (b) EmaFilter:  y[n] = α·x[n] + (1-α)·y[n-1]     (계산은 float 로)
//         - 첫 sample 은 그대로 내보내고 내부 상태를 그 값으로 잡는다 (warm start).
//         - 입력에 NaN/Inf 가 하나라도 있으면 상태를 건드리지 않고 "직전 출력"을 내보낸다.
//           (첫 sample 부터 NaN 이면 reset 의 0 이 나가고, 다음 정상 sample 에서 warm start)
//   레포: filter.cpp EmaFilter::step
// ----------------------------------------------------------------------------
inline auto all_finite(std::span<const float> values) -> bool {  // TODO
    (void)values;
    return false;
}

class EmaFilter : public FilterBase {
public:
    explicit EmaFilter(const ModifierSpec& spec) {  // TODO
        (void)spec;
    }

    auto reset(int n) -> void override {  // TODO
        (void)n;
    }

    auto step(std::span<const float> x, std::span<float> y) -> void override {  // TODO
        (void)x;
        (void)y;
    }

private:
    // TODO: 필요한 멤버 변수
};

// ----------------------------------------------------------------------------
// Q2. NotchFilter — 특정 주파수 하나만 지우는 2차 IIR (biquad)
//   (a) notch_coeffs: 계수 계산 (double 로 계산한 뒤 float 로 저장)
//         w0 = 2π·f0/fs     Q = f0/bw     a = sin(w0)/(2Q)     a0 = 1 + a
//         b0 = 1/a0         b1 = -2·cos(w0)/a0      b2 = 1/a0
//         a1 = -2·cos(w0)/a0                         a2 = (1 - a)/a0
//       (f0 = center_freq, bw = bandwidth, fs = sample_rate)
//   (b) NotchFilter::step:
//         y[n] = b0·x[n] + b1·x[n-1] + b2·x[n-2] - a1·y[n-1] - a2·y[n-2]
//         - 첫 sample: 과거 값 4개(x[n-1], x[n-2], y[n-1], y[n-2])를 전부 그 sample 로 잡고
//           그 sample 을 그대로 내보낸다.
//         - NaN/Inf 입력: 상태를 건드리지 않고 직전 출력을 내보낸다 (EMA 와 같은 규약).
//   레포: filter.cpp NotchFilter
// ----------------------------------------------------------------------------
struct NotchCoeffs {
    float b0 = 0.0f;
    float b1 = 0.0f;
    float b2 = 0.0f;
    float a1 = 0.0f;
    float a2 = 0.0f;
};

inline auto notch_coeffs(const ModifierSpec& spec) -> NotchCoeffs {  // TODO
    (void)spec;
    return NotchCoeffs{};
}

class NotchFilter : public FilterBase {
public:
    explicit NotchFilter(const ModifierSpec& spec) {  // TODO
        (void)spec;
    }

    auto reset(int n) -> void override {  // TODO
        (void)n;
    }

    auto step(std::span<const float> x, std::span<float> y) -> void override {  // TODO
        (void)x;
        (void)y;
    }

private:
    // TODO: 필요한 멤버 변수
};

// ----------------------------------------------------------------------------
// Q3. FilterChain — modifier 목록을 순서대로 이어 붙인 것
//   make_filter_chain : ModifierSpec 목록 → 필터 객체 목록 (등장 순서 그대로)
//   reset_filter_chain: 체인의 모든 필터를 n 채널로 reset
//   apply_filter_chain: 순서대로 통과. 빈 체인이면 y = x 그대로.
//                       (앞 단계의 출력이 다음 단계의 입력이다)
//   레포: filter.cpp make_filter_chain / reset_filter_chain / apply_filter_chain
// ----------------------------------------------------------------------------
using FilterChain = std::vector<std::unique_ptr<FilterBase>>;

inline auto make_filter_chain(const std::vector<ModifierSpec>& mods) -> FilterChain {  // TODO
    (void)mods;
    return {};
}

inline auto reset_filter_chain(const FilterChain& chain, int n) -> void {  // TODO
    (void)chain;
    (void)n;
}

inline auto apply_filter_chain(const FilterChain& chain, std::span<const float> x, std::span<float> y) -> void {  // TODO
    (void)chain;
    (void)x;
    (void)y;
}

// ----------------------------------------------------------------------------
// Q4. HistoryBuffer — 최근 값들을 간격을 두고 꺼내 보는 버퍼
//   base_dim 짜리 sample 을 push 할 때마다 한 칸씩 쌓고, fill 은 그중
//   history_length 개를 update_interval 칸 간격으로 골라 이어 붙인다.
//
//     보관 칸 수 = (history_length - 1) · update_interval + 1
//     dim()      = history_length · base_dim
//
//   출력 순서 (t = 가장 최근 push):
//     kOldestFirst : [ t-(N-1)·interval, ..., t-interval, t ]      ← *_history 의 규약
//     kNewestFirst : [ t, t-interval, ..., t-(N-1)·interval ]      ← previous_joint_actions 의 규약
//
//   history_length / update_interval 이 1 보다 작게 들어오면 1 로 올린다.
//   아직 채워지지 않은 과거 칸은 0 이다 (warm_start == false 일 때).
//   레포: observation.cpp HistorySource (생성자, tick 의 ring 밀기, fill_from)
// ----------------------------------------------------------------------------
enum class HistoryOrder {
    kOldestFirst,
    kNewestFirst,
};

class HistoryBuffer {
public:
    // ------------------------------------------------------------------------
    // Q5. warm_start
    //   warm_start == true 이면 "첫 push" 가 모든 칸을 그 sample 로 채운다
    //   (과거 칸이 0 으로 남지 않는다). 두 번째 push 부터는 Q4 와 같다.
    //
    //   주의: 스터디 기준 커밋(386bae3a8)의 HistorySource 에는 이 동작이 "없다" (항상 0 에서 시작).
    //         observation.h 의 주석은 warm start 한다고 적혀 있지만 구현과 어긋나 있고,
    //         다른 브랜치(feature/697-rgmt-command-window)에서 이 플래그가 추가됐다.
    //         그래서 기본값이 false 다. 자세한 내용은 GUIDE.md Q5.
    // ------------------------------------------------------------------------
    HistoryBuffer(int base_dim, int history_length, int update_interval, HistoryOrder order, bool warm_start = false) {  // TODO
        (void)base_dim;
        (void)history_length;
        (void)update_interval;
        (void)order;
        (void)warm_start;
    }

    [[nodiscard]] auto dim() const -> int {  // TODO
        return 0;
    }

    // 내부에 보관하는 sample 칸 수
    [[nodiscard]] auto slot_count() const -> int {  // TODO
        return 0;
    }

    // sample.size() == base_dim
    auto push(std::span<const float> sample) -> void {  // TODO
        (void)sample;
    }

    // out.size() == dim()
    auto fill(std::span<float> out) const -> void {  // TODO
        (void)out;
    }

private:
    // TODO: 필요한 멤버 변수
};

// ----------------------------------------------------------------------------
// Q6. HistoryTerm — "필터 → history" 를 한 덩어리로
//   tick(raw): raw sample 을 필터 체인에 통과시킨 뒤 그 결과를 history 에 push 한다.
//              (history 에 쌓이는 것은 필터를 거친 값이다)
//   fill(out): history 의 fill 그대로.
//   생성자에서 필터 체인을 base_dim 채널로 reset 해 둔다.
//
//   history_length = 1, update_interval = 1 로 만들면 "필터만 건 term" 이 된다 —
//   레포는 history 없는 term 에 modifier 가 붙었을 때 실제로 이렇게 감싼다.
//   레포: observation.cpp HistorySource::tick
// ----------------------------------------------------------------------------
class HistoryTerm {
public:
    HistoryTerm(int base_dim, int history_length, int update_interval, FilterChain filters, HistoryOrder order, bool warm_start = false)
    {  // TODO
        (void)base_dim;
        (void)history_length;
        (void)update_interval;
        (void)filters;
        (void)order;
        (void)warm_start;
    }

    [[nodiscard]] auto dim() const -> int {  // TODO
        return 0;
    }

    // raw.size() == base_dim
    auto tick(std::span<const float> raw) -> void {  // TODO
        (void)raw;
    }

    // out.size() == dim()
    auto fill(std::span<float> out) const -> void {  // TODO
        (void)out;
    }

private:
    // TODO: 필요한 멤버 변수
};

}  // namespace quiz
