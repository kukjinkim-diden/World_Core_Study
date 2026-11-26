// ============================================================================
// 04. action — "정책이 낸 숫자를 로봇 명령으로 바꾼다"
// ============================================================================
// 이 파일만 수정한다. 검사: `make 04` (문항별: `make 04 Q=4`)
//
// 정책(ONNX)의 출력은 의미 없는 float 16개다 (관절 12 + 자석 4). 이것을
//   - 관절 목표 각도 + PD 게인                (joint_actions)
//   - 발 자석을 켜라/꺼라/그대로 (+1 / -1 / 0)  (magnetic_actions)
// 로 바꾸고, 출력이 깨졌을 때(NaN)와 정책을 갈아탈 때의 안전장치까지 만든다.
//
// 레포: src/applications/controller/runner/rl_controller/action.cpp
//       src/applications/controller/runner/rl_controller/rl_controller.cpp (safe-hold, blend)
// ============================================================================
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <span>
#include <vector>

#include "../01_obs_math/quiz.h"

namespace quiz {

// 레포: include/diden_core/engine/models/diden_spider/constants.h
inline constexpr int kNumManipulatorJoints = 6;                                   // rb3 팔
inline constexpr int kNumJoints = kNumMobileJoints + kNumManipulatorJoints;        // 18 — [다리 12, 팔 6]
inline constexpr int kEpmDim = 8;                                                  // 발 자석 4 + 본체 자석 4

// 컨트롤러 한 주기의 출력. 레포: controller_base.h ControllerOutput
struct ControllerOutput {
    struct JointCommand {
        std::array<double, kNumJoints> q{};    // 목표 위치 [rad]
        std::array<double, kNumJoints> dq{};   // 목표 속도 [rad/s]
        std::array<double, kNumJoints> tau{};  // feedforward 토크 [N·m]
        std::array<double, kNumJoints> kp{};
        std::array<double, kNumJoints> kd{};
    } joint;

    // 자석(EPM) 신호: >0 ON, <0 OFF, 0 = 그대로. 앞 4칸이 발 자석이다.
    std::array<int, kEpmDim> epm{};
};

// 정책 YAML 의 control 블록 중 joint_actions 가 쓰는 값. 레포: policy.h Actions / Control
struct ActionConfig {
    double joint_action_scale = 0.0;
    std::vector<double> action_mean;  // 길이 12
    double joint_kp = 0.0;
    double joint_kd = 0.0;
};

// 정책 YAML 의 magnet_options. 레포: policy.h MagnetStateMachine
struct MagnetConfig {
    double action_on_threshold = 0.0;
    double action_off_threshold = 0.0;
    bool max_action_only = false;
};

// ----------------------------------------------------------------------------
// Q1. joint_actions — 관절 목표
//   raw 는 정책 출력 중 관절 몫 12개.
//     out.joint.q[i]  = action_mean[i] + joint_action_scale · raw[i]      (i = 0..11, double 로 계산)
//     out.joint.kp[i] = joint_kp,  out.joint.kd[i] = joint_kd             (i = 0..11)
//   팔(12..17)과 dq, tau, epm 은 건드리지 않는다.
//   레포: action.cpp JointPositionActionTerm::apply
//   단순화: 레포에는 offset 으로 action_mean 대신 모션 클립의 참조 자세를 쓰는 모드
//           (use_motion_reference_offset)가 있다. 여기서는 action_mean 만 다룬다.
// ----------------------------------------------------------------------------
inline auto apply_joint_action(std::span<const float> raw, const ActionConfig& cfg, ControllerOutput& out) -> void {  // TODO
    //@sol
    for (std::size_t i = 0; i < static_cast<std::size_t>(kNumMobileJoints); ++i) {
        out.joint.q[i] = cfg.action_mean[i] + cfg.joint_action_scale * static_cast<double>(raw[i]);
        out.joint.kp[i] = cfg.joint_kp;
        out.joint.kd[i] = cfg.joint_kd;
    }
    //@stub
    (void)raw;
    (void)cfg;
    (void)out;
    //@end
}

// ----------------------------------------------------------------------------
// Q2. 자석 행동 이산화
//   a 는 tanh 를 거친 값([-1, 1]).
//     a > action_on_threshold  → +1 (켜라)
//     a < action_off_threshold → -1 (꺼라)
//     그 외                     →  0 (그대로)
//   경계값(a == threshold)은 0 이다.
//   레포: action.cpp MagnetActionGate::tick 의 2) discretize
// ----------------------------------------------------------------------------
inline auto discretize_magnet_action(float a, const MagnetConfig& cfg) -> int {  // TODO
    //@sol
    if (a > cfg.action_on_threshold) return 1;
    if (a < cfg.action_off_threshold) return -1;
    return 0;
    //@stub
    (void)a;
    (void)cfg;
    return 0;
    //@end
}

// ----------------------------------------------------------------------------
// Q3 ~ Q4. MagnetActionGate — "이번 주기에 어느 발 자석을 스위칭해도 되는가"
//
//   자석은 한 번 스위칭하면 얼마 동안 다시 스위칭하면 안 된다 (코일 안정, 전원 전류 한계).
//   gate 는 발마다 cooldown 카운터 하나, 전체 공용 cooldown 카운터 하나를 들고 있다.
//
//   configure(control_dt_ms, foot_cooldown_ms, global_cooldown_ms)
//       cooldown [ms] 을 tick 수로 환산해 둔다:  ms <= 0 이면 0, 아니면 ceil(ms / control_dt_ms)
//   reset()
//       남은 cooldown 을 전부 0 으로 (환산해 둔 tick 수는 유지)
//   tick(normalized, cfg) → 발 4개의 신호 (+1 / -1 / 0).  아래 순서를 그대로 지킬 것:
//       1) 남은 cooldown 을 1 씩 줄인다 (0 아래로는 내려가지 않는다)
//       2) 발마다 Q2 의 이산화
//       3) 자기 cooldown 이 남은 발은 0 으로
//       4) 공용 cooldown 이 남아 있으면 전부 0 으로
//       5) max_action_only 이면: 아직 0 이 아닌 발 중 |normalized| 가 가장 큰 하나만 남긴다
//          (같으면 번호가 작은 발)
//       6) 살아남은 발마다: 그 발의 cooldown 과 공용 cooldown 을 다시 채운다
//
// Q3 은 cooldown 이 0 인 상태(2, 5 번)와 configure 의 환산을,
// Q4 는 cooldown 이 걸린 상태(1, 3, 4, 6 번)를 검사한다.
//
//   레포: action.cpp MagnetActionGate
//   단순화: 레포에는 발 하나가 cell 두 개(In/Out)로 나뉜 배선을 위한 staging 단계와
//           timer_state() 관측이 더 있다. spider 는 cell 이 하나라 staging 이 꺼져 있다 — 여기서는 뺐다.
//           foot_cooldown_ticks() / global_cooldown_ticks() 는 테스트용 접근자다 (레포에는 없다).
// ----------------------------------------------------------------------------
class MagnetActionGate {
public:
    static constexpr int kN = kNumLegs;  // 4

    auto reset() -> void {  // TODO
        //@sol
        per_foot_cooldown_.fill(0);
        global_cooldown_ = 0;
        //@stub
        //@end
    }

    auto configure(double control_dt_ms, double foot_cooldown_ms, double global_cooldown_ms) -> void {  // TODO
        //@sol
        const auto to_ticks = [control_dt_ms](double ms) -> int { return ms <= 0.0 ? 0 : static_cast<int>(std::ceil(ms / control_dt_ms)); };
        foot_cooldown_ticks_ = to_ticks(foot_cooldown_ms);
        global_cooldown_ticks_ = to_ticks(global_cooldown_ms);
        //@stub
        (void)control_dt_ms;
        (void)foot_cooldown_ms;
        (void)global_cooldown_ms;
        //@end
    }

    [[nodiscard]] auto foot_cooldown_ticks() const -> int {  // TODO
        //@sol
        return foot_cooldown_ticks_;
        //@stub
        return -1;
        //@end
    }

    [[nodiscard]] auto global_cooldown_ticks() const -> int {  // TODO
        //@sol
        return global_cooldown_ticks_;
        //@stub
        return -1;
        //@end
    }

    auto tick(const std::array<float, kN>& normalized, const MagnetConfig& cfg) -> std::array<int, kN> {  // TODO
        //@sol
        // 1) cooldown 카운트다운
        for (auto& c : per_foot_cooldown_) {
            c = std::max(0, c - 1);
        }
        global_cooldown_ = std::max(0, global_cooldown_ - 1);

        // 2) 이산화
        std::array<int, kN> state{};
        for (std::size_t i = 0; i < state.size(); ++i) {
            state[i] = discretize_magnet_action(normalized[i], cfg);
        }

        // 3) per-foot cooldown
        for (std::size_t i = 0; i < state.size(); ++i) {
            if (per_foot_cooldown_[i] > 0) state[i] = 0;
        }

        // 4) global cooldown
        if (global_cooldown_ > 0) state.fill(0);

        // 5) max_action_only
        if (cfg.max_action_only) {
            int best = -1;
            float best_abs = -1.0f;
            for (std::size_t i = 0; i < state.size(); ++i) {
                if (state[i] == 0) continue;
                const float a = std::abs(normalized[i]);
                if (a > best_abs) {
                    best_abs = a;
                    best = static_cast<int>(i);
                }
            }
            for (std::size_t i = 0; i < state.size(); ++i) {
                if (static_cast<int>(i) != best) state[i] = 0;
            }
        }

        // 6) timer 재장전
        for (std::size_t i = 0; i < state.size(); ++i) {
            if (state[i] == 0) continue;
            per_foot_cooldown_[i] = foot_cooldown_ticks_;
            global_cooldown_ = global_cooldown_ticks_;
        }
        return state;
        //@stub
        (void)normalized;
        (void)cfg;
        return {};
        //@end
    }

private:
    // TODO: 필요한 멤버 변수
    //@sol
    std::array<int, kN> per_foot_cooldown_{};  // 남은 cooldown [tick]
    int global_cooldown_ = 0;
    int foot_cooldown_ticks_ = 0;
    int global_cooldown_ticks_ = 0;
    //@end
};

// ----------------------------------------------------------------------------
// Q5. 팔 hold 와 safe-hold
//   RL 정책은 다리만 제어한다. 팔은 RL 이 시작된 순간의 자세를 계속 잡고 있는다.
//   (a) apply_manipulator_hold: out 의 팔 구간(12..17)에
//         q = held_arm_pos,  kp = cfg.arm_kp,  kd = cfg.arm_kd       (다리 구간은 건드리지 않는다)
//   (b) make_safe_hold: 추론이 실패한 주기(NaN 등)에 대신 내보낼 출력.
//         다리 q  = last_target_dof_pos   (직전에 내보낸 정상 목표를 그대로 유지)
//         다리 kp = cfg.joint_kp, kd = cfg.joint_kd
//         팔      = (a) 와 같다
//         dq, tau, epm = 전부 0
//   레포: rl_controller.cpp RlController::step 의 make_safe_hold 와 6) Manipulator hold
// ----------------------------------------------------------------------------
struct HoldConfig {
    double joint_kp = 0.0;  // 정책 YAML control.joint_kp
    double joint_kd = 0.0;
    std::array<double, kNumManipulatorJoints> arm_kp{};  // joint_pd_ctrl_config.yaml 의 팔 구간
    std::array<double, kNumManipulatorJoints> arm_kd{};
};

inline auto apply_manipulator_hold(const std::array<double, kNumManipulatorJoints>& held_arm_pos, const HoldConfig& cfg, ControllerOutput& out) -> void {  // TODO
    //@sol
    for (std::size_t i = 0; i < static_cast<std::size_t>(kNumManipulatorJoints); ++i) {
        const std::size_t j = static_cast<std::size_t>(kNumMobileJoints) + i;
        out.joint.q[j] = held_arm_pos[i];
        out.joint.kp[j] = cfg.arm_kp[i];
        out.joint.kd[j] = cfg.arm_kd[i];
    }
    //@stub
    (void)held_arm_pos;
    (void)cfg;
    (void)out;
    //@end
}

inline auto make_safe_hold(const std::array<double, kNumMobileJoints>& last_target_dof_pos, const std::array<double, kNumManipulatorJoints>& held_arm_pos, const HoldConfig& cfg)
    -> ControllerOutput {  // TODO
    //@sol
    ControllerOutput safe;
    for (std::size_t i = 0; i < static_cast<std::size_t>(kNumMobileJoints); ++i) {
        safe.joint.q[i] = last_target_dof_pos[i];
        safe.joint.kp[i] = cfg.joint_kp;
        safe.joint.kd[i] = cfg.joint_kd;
    }
    apply_manipulator_hold(held_arm_pos, cfg, safe);
    return safe;
    //@stub
    (void)last_target_dof_pos;
    (void)held_arm_pos;
    (void)cfg;
    return {};
    //@end
}

// ----------------------------------------------------------------------------
// Q6. 정책 전환 램프 (blend)
//   도는 중에 정책을 갈아타면 두 정책의 목표 자세가 달라 명령이 튄다. 그래서 갈아탄 뒤
//   몇 tick 동안은 "갈아타기 직전 목표(from)" 에서 "새 정책의 목표" 로 서서히 넘어간다.
//
//   (a) blend_ticks(switch_blend_ms, dt_s) = ceil( max(0, switch_blend_ms) · 1e-3 / dt_s )
//   (b) SwitchBlend
//         start(from, ticks): 램프 시작. 남은 tick = ticks.
//         apply(target)     : 남은 tick 이 0 이면 아무것도 하지 않는다. 아니면
//                               alpha  = 1 - 남은 tick / ticks
//                               target = from + alpha · (target - from)
//                             로 덮어쓰고 남은 tick 을 1 줄인다.
//         active()          : 남은 tick > 0
//   레포: rl_controller.cpp activate_policy 의 blend_ticks_, step 의 6.5) 체인 전환 램프
// ----------------------------------------------------------------------------
inline auto blend_ticks(double switch_blend_ms, double dt_s) -> int {  // TODO
    //@sol
    return static_cast<int>(std::ceil(std::max(0.0, switch_blend_ms) * 1e-3 / dt_s));
    //@stub
    (void)switch_blend_ms;
    (void)dt_s;
    return -1;
    //@end
}

class SwitchBlend {
public:
    auto start(const std::array<double, kNumMobileJoints>& from, int ticks) -> void {  // TODO
        //@sol
        from_ = from;
        ticks_ = ticks;
        remaining_ = ticks;
        //@stub
        (void)from;
        (void)ticks;
        //@end
    }

    auto apply(std::array<double, kNumMobileJoints>& target) -> void {  // TODO
        //@sol
        if (remaining_ <= 0) return;
        const double alpha = 1.0 - static_cast<double>(remaining_) / static_cast<double>(ticks_);
        for (std::size_t i = 0; i < target.size(); ++i) {
            target[i] = from_[i] + alpha * (target[i] - from_[i]);
        }
        --remaining_;
        //@stub
        (void)target;
        //@end
    }

    [[nodiscard]] auto active() const -> bool {  // TODO
        //@sol
        return remaining_ > 0;
        //@stub
        return false;
        //@end
    }

private:
    // TODO: 필요한 멤버 변수
    //@sol
    std::array<double, kNumMobileJoints> from_{};
    int ticks_ = 0;
    int remaining_ = 0;
    //@end
};

}  // namespace quiz
