// ============================================================================
// 05. timing & contract — "정책이 학습된 조건과 어긋난 채로는 돌리지 않는다"
// ============================================================================
// 이 파일만 수정한다. 검사: `make 05` (문항별: `make 05 Q=2`)
//
// 정책은 학습할 때의 조건에 묶여 있다 — 몇 Hz 로 불렸는지, 관측이 몇 차원이었는지.
// Core 는 그 조건을 정책 YAML 에서 읽어 와서, 맞지 않으면 "조용히 이상하게 도는" 대신
// 로드 단계에서 거절한다. 이 퀴즈는 그 판정들을 만든다.
//
// 레포: src/applications/controller/runner/rl_controller/policy.cpp (build_control, validate_dims, ...)
//       src/applications/controller/runner/thread.cpp (주기 스케줄링)
// ============================================================================
#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <unordered_set>
#include <vector>

#include "../03_obs_layout/quiz.h"

namespace quiz {

// ----------------------------------------------------------------------------
// Q1. 정책 주기 유도
//   정책 주기 [µs] = physics_dt [s] × decimation × 1e6     (계산 순서도 이대로: (dt · decimation) · 1e6)
//   double 결과를 int 로 바꿀 때는 static_cast<int> — 소수부를 "버린다" (반올림이 아니다).
//
//   다음 경우는 nullopt (로드 거절):
//     - explicit_cycle_time_us 가 들어 있다  (정책 YAML 에 control.cycle_time_us 를 직접 적은 경우.
//                                            주기의 출처를 하나로 묶으려고, 무시하지 않고 거절한다)
//     - decimation 또는 physics_dt 가 없다
//     - 계산 결과가 0 이하다
//   레포: policy.cpp build_control
// ----------------------------------------------------------------------------
inline auto derive_cycle_time_us(std::optional<int> decimation, std::optional<double> physics_dt, std::optional<int> explicit_cycle_time_us = std::nullopt) -> std::optional<int> {  // TODO
    if (explicit_cycle_time_us) return std::nullopt;
    if (!decimation || !physics_dt) return std::nullopt;
    const int cycle_time_us = static_cast<int>(*physics_dt * *decimation * 1e6);
    if (cycle_time_us <= 0) return std::nullopt;
    return cycle_time_us;
}

// ----------------------------------------------------------------------------
// Q2 ~ Q3. StepScheduler — 1 kHz tick 위에서 정책 주기 맞추기
//
//   runner 스레드는 tick_us(1000 µs = 1 kHz)마다 깨어난다. 정책은 그보다 느리게(예: 10000 µs)
//   돌아야 하므로, 깨어날 때마다 "지금 step 을 돌릴 차례인가" 를 묻는다.
//
//   start(now_us, period_us, tick_us) — 컨트롤러 활성화 시 1 회
//       period_us < tick_us 이면 false (tick 보다 빠른 주기는 지킬 수 없다 → 활성화 거절).
//       아니면 주기를 기억하고, "다음 step 목표 시각" 을 now_us 로 잡은 뒤 true.
//       (그래서 첫 step 은 활성화한 그 tick 에 바로 돈다)
//
//   due(now_us) — 매 tick. true 면 이번 tick 에 step 을 돌린다.
//       now_us < 다음 목표 시각        → false
//       아니면: 다음 목표 시각 += period
//               그래도 다음 목표 시각 < now_us 이면 (한 주기 넘게 밀렸다) 다음 목표 시각 = now_us + period
//               → true
//
// Q2 는 딱 맞아떨어지는 경우, Q3 은 tick 이 늦게 온 경우와 주기가 tick 의 배수가 아닌 경우를 검사한다.
//
//   레포: thread.cpp thread_step (period / next_step_time)
//   단순화: 레포는 std::chrono::steady_clock 을 직접 읽는다. 테스트하기 쉽게 시각을 인자로 받는다.
// ----------------------------------------------------------------------------
class StepScheduler {
public:
    [[nodiscard]] auto start(std::int64_t now_us, int period_us, int tick_us) -> bool {  // TODO
        if (period_us < tick_us) return false;
        period_us_ = period_us;
        next_step_us_ = now_us;
        return true;
    }

    [[nodiscard]] auto due(std::int64_t now_us) -> bool {  // TODO
        if (now_us < next_step_us_) return false;
        next_step_us_ += period_us_;
        if (next_step_us_ < now_us) {
            next_step_us_ = now_us + period_us_;
        }
        return true;
    }

private:
    // TODO: 필요한 멤버 변수
    std::int64_t period_us_ = 0;
    std::int64_t next_step_us_ = 0;
};

// ----------------------------------------------------------------------------
// Q4. 차원 계약 — 정책 YAML 과 ONNX 가 같은 정책을 말하고 있는가
//   아래 순서대로 검사해서 처음 걸린 것을 돌려준다. 전부 통과하면 kOk.
//
//     1) model_input_dim  == (obs term dim 합) - (estimator_output term 의 dim, 없으면 0)
//                                                              아니면 kInputDimMismatch
//     2) model_output_dim == (action term dim 합)              아니면 kOutputDimMismatch
//     3) action term 에 joint_actions 가 있으면
//          action_mean_size == joint_actions 의 dim            아니면 kActionMeanSizeMismatch
//          joint_action_scale > 0                              아니면 kActionScaleInvalid
//
//   estimator_output 은 "ONNX 안에서 만들어지는 관측"이다. YAML 의 목록에는 있지만 Core 가
//   채워 넣는 값이 아니므로 입력 차원에서 빼고, 검증이 끝나면 목록에서도 지운다:
//     remove_estimator_output(obs) — 이름이 estimator_output 인 term 을 obs 에서 제거
//
//   레포: policy.cpp Policy::validate_dims
// ----------------------------------------------------------------------------
inline constexpr std::string_view kEstimatorOutputTerm = "estimator_output";
inline constexpr std::string_view kJointActionsTerm = "joint_actions";

enum class DimCheck {
    kOk,
    kInputDimMismatch,
    kOutputDimMismatch,
    kActionMeanSizeMismatch,
    kActionScaleInvalid,
};

inline auto validate_dims(const std::vector<TermEntry>& obs, const std::vector<TermEntry>& actions, int model_input_dim, int model_output_dim, std::size_t action_mean_size,
                          double joint_action_scale) -> DimCheck {  // TODO
    int obs_total = 0;
    const TermEntry* estimator = nullptr;
    for (const auto& t : obs) {
        obs_total += t.dim;
        if (t.name == kEstimatorOutputTerm && estimator == nullptr) estimator = &t;
    }
    const int estimator_dim = estimator != nullptr ? estimator->dim : 0;
    int act_total = 0;
    const TermEntry* joint_actions = nullptr;
    for (const auto& t : actions) {
        act_total += t.dim;
        if (t.name == kJointActionsTerm && joint_actions == nullptr) joint_actions = &t;
    }

    if (model_input_dim != obs_total - estimator_dim) return DimCheck::kInputDimMismatch;
    if (model_output_dim != act_total) return DimCheck::kOutputDimMismatch;
    if (joint_actions != nullptr) {
        if (static_cast<int>(action_mean_size) != joint_actions->dim) return DimCheck::kActionMeanSizeMismatch;
        if (joint_action_scale <= 0.0) return DimCheck::kActionScaleInvalid;
    }
    return DimCheck::kOk;
}

inline auto remove_estimator_output(std::vector<TermEntry>& obs) -> void {  // TODO
    std::erase_if(obs, [](const TermEntry& t) -> bool { return t.name == kEstimatorOutputTerm; });
}

// ----------------------------------------------------------------------------
// Q5. 설정 정합성 — 조용한 오동작으로 직결되는 값만 막는다
//   (a) validate_term_list: term 이름이 전부 다르고(중복 금지) dim 이 음수가 아니면 true.
//         순서가 곧 배치라서, 같은 이름이 두 번 나오면 어느 쪽인지 모호하다.
//   (b) validate_control_gains: joint_kp > 0 이고 joint_kd >= 0 이면 true.
//         kp 가 0 이하면 다리에 힘이 안 들어가고, kd 가 음수면 발진한다.
//   (c) common_cycle_time_us: 한 번에 올리는 정책 집합의 주기가 전부 같으면 그 값, 아니면 nullopt.
//         빈 집합도 nullopt. (runner 는 주기를 활성화 때 한 번만 읽으므로, 도는 중에 갈아탈
//         정책들은 주기가 같아야 한다)
//   레포: policy.cpp check_unique / build_observations, build_control, Policy::resolve_policy_set
// ----------------------------------------------------------------------------
inline auto validate_term_list(const std::vector<TermEntry>& terms) -> bool {  // TODO
    std::unordered_set<std::string> seen;
    for (const auto& t : terms) {
        if (!seen.insert(t.name).second) return false;
        if (t.dim < 0) return false;
    }
    return true;
}

inline auto validate_control_gains(double joint_kp, double joint_kd) -> bool {  // TODO
    return joint_kp > 0.0 && joint_kd >= 0.0;
}

inline auto common_cycle_time_us(std::span<const int> cycle_times_us) -> std::optional<int> {  // TODO
    if (cycle_times_us.empty()) return std::nullopt;
    for (int c : cycle_times_us) {
        if (c != cycle_times_us.front()) return std::nullopt;
    }
    return cycle_times_us.front();
}

}  // namespace quiz
