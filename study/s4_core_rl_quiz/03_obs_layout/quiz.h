// ============================================================================
// 03. obs layout — "term 목록을 순서대로 이어 붙여 관측 벡터를 만든다"
// ============================================================================
// 이 파일만 수정한다. 검사: `make 03` (문항별: `make 03 Q=3`)
//
// 정책 YAML 의 observations 는 "이름 붙은 항의 목록"이고, 목록의 순서가 곧 벡터의 배치다.
//   observations:
//     - {name: gait_phi, dim: 8}
//     - {name: imu_ang_vel, dim: 3, modifiers: [{type: ema, alpha: 0.5}]}
//     - {name: dof_vel_history, dim: 36, history_length: 3, history_interval: 4}
// 이 퀴즈는 그 목록을 읽어 관측 벡터를 채우는 ObservationManager 를 만든다.
// 01 의 수식과 02 의 HistoryTerm 을 그대로 가져다 쓴다 (01·02 를 먼저 풀 것).
//
// 레포: src/applications/controller/runner/rl_controller/observation.cpp
//       (make_base_term, ObservationManager::init / tick / fill_internal)
// ============================================================================
#pragma once

#include <array>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "../01_obs_math/quiz.h"
#include "../02_history_filter/quiz.h"

namespace quiz {

// 정책 YAML 의 observations / actions 한 줄. 레포: policy.h TermEntry
struct TermEntry {
    std::string name;
    int dim = 0;
    std::optional<int> history_length;    // nullopt = history 없음
    std::optional<int> history_interval;  // history_length 가 있을 때만 의미 (없으면 1)
    std::vector<ModifierSpec> modifiers;  // 비어 있으면 필터 없음
};

// 컨트롤러가 매 step 진입부에서 떠 두는 "이번 주기의 입력" 사본.
// 레포에서는 RlController 의 멤버 여러 개(sensor_snap_, state_snap_, desired_velocity_snap_,
// last_target_dof_pos_, last_magnetic_action_)에 나뉘어 있고, 각 term 이 그 참조를 잡고 읽는다.
struct ObsInputs {
    Vec3 gyro{};                                              // sensor_raw.imu.gyro [rad/s]
    Quat base_quat{};                                         // state.q[3..7) — (w, x, y, z)
    std::array<double, kNumMobileJoints> joint_q{};           // sensor_raw.joint.q 의 앞 12개 [rad]
    std::array<double, kNumMobileJoints> joint_dq{};          // sensor_raw.joint.dq 의 앞 12개 [rad/s]
    Vec3 command{};                                           // clamp 가 끝난 속도 명령 (vx, vy, wz)
    std::array<double, kNumMobileJoints> last_target_dof_pos{};  // 직전 주기에 내보낸 관절 목표
    std::array<float, kNumLegs> last_magnetic_action{};       // 직전 주기의 tanh(자석 행동)
    double gait_phase_ratio = 0.0;                            // 단순화: 보행 위상은 밖에서 준다
};

// term 이 읽는 정책 설정. 레포: policy.h Control::command_threshold, Locomotions::leg_phase_offsets
struct ObsConfig {
    double command_threshold = 0.0;
    std::array<double, kNumLegs> leg_phase_offsets{};
};

inline constexpr std::string_view kHistorySuffix = "_history";
inline constexpr std::string_view kPrevJointActionsKey = "previous_joint_actions";

// ----------------------------------------------------------------------------
// Q1. term 이름 해석
//   (a) strip_history_suffix: 이름이 "_history" 로 끝나면 그 접미사를 뗀다.
//         "dof_vel_history" → "dof_vel"        "dof_vel" → "dof_vel"
//       단, 접미사를 떼고 나서 남는 글자가 없으면 떼지 않는다 ("_history" → "_history").
//   (b) base_term_dim: (접미사를 뗀) 이름의 "history 를 입히기 전" 차원. 모르는 이름은 -1.
//
//         imu_ang_vel 3             projected_gravity 3       command 3
//         dof_pos 12                dof_vel 12                dof_error 12
//         target_dof_pos 12         previous_joint_actions 12
//         previous_magnetic_action 4                          gait_phi 8
//
//       12 / 4 / 8 은 숫자를 직접 쓰지 말고 01 의 상수(kNumMobileJoints, kNumLegs)로 적을 것.
//   레포: observation.cpp ObservationManager::init 의 dispatch_name, make_base_term, 각 Source 의 dim()
//   단순화: 레포에는 term 이 더 있다 (relative_foot_pos, gait_phase, standing_flag,
//           magnet_timer_state, motion_*_target, estimator_output). 퀴즈는 위 10개만 다룬다.
// ----------------------------------------------------------------------------
inline auto strip_history_suffix(std::string_view name) -> std::string_view {  // TODO
    (void)name;
    return {};
}

inline auto base_term_dim(std::string_view base_name) -> int {  // TODO
    (void)base_name;
    return -1;
}

// ----------------------------------------------------------------------------
// Q2. term 하나의 "지금 값" 채우기
//   base_name 에 해당하는 값을 out 에 float 로 쓴다 (out.size() == base_term_dim(base_name)).
//   모르는 이름이거나 out 크기가 맞지 않으면 아무것도 쓰지 않고 false.
//
//     imu_ang_vel              = in.gyro
//     projected_gravity        = 01 의 projected_gravity(quat_to_rotation(in.base_quat))
//     command                  = 01 의 command_obs(in.command, cfg.command_threshold)
//     dof_pos                  = in.joint_q
//     dof_vel                  = in.joint_dq
//     dof_error                = in.joint_q - in.last_target_dof_pos        ← 부호 주의 (현재 - 목표)
//     target_dof_pos           = in.last_target_dof_pos
//     previous_joint_actions   = in.last_target_dof_pos   (이름만 다르고 같은 값이다)
//     previous_magnetic_action = in.last_magnetic_action
//     gait_phi                 = 01 의 gait_phi(in.gait_phase_ratio, cfg.leg_phase_offsets)
//
//   scale 이나 정규화는 곱하지 않는다 — 관측 정규화는 ONNX 그래프 안에 들어 있다.
//   레포: observation.cpp 의 각 *Source::fill
// ----------------------------------------------------------------------------
inline auto fill_base_term(std::string_view base_name, const ObsInputs& in, const ObsConfig& cfg, std::span<float> out) -> bool {  // TODO
    (void)base_name;
    (void)in;
    (void)cfg;
    (void)out;
    return false;
}

// ----------------------------------------------------------------------------
// Q3 ~ Q5. ObservationManager
//
//   init(terms, cfg, inputs)  term 목록을 순서대로 읽어 내부 슬롯을 만든다. 실패하면 false.
//   total_dim()               모든 term 의 dim 합 (= 관측 벡터 길이)
//   tick()                    매 주기 1 회. history/필터가 있는 term 이 "지금 값"을 한 칸 쌓는다.
//   build_actor_input(out)    관측 벡터를 채운다. out.size() == total_dim().
//
//   inputs 는 "참조"로 잡는다 — init 때 값을 복사해 두는 것이 아니라, tick/build 때마다
//   호출부가 갱신해 둔 최신 값을 읽는다. (그래서 inputs 는 manager 보다 오래 살아야 한다)
//
// Q3. init — term 마다 아래를 순서대로 판정한다. 하나라도 어긋나면 false.
//   1) base 이름 = strip_history_suffix(name).  base_term_dim 이 -1 이면 모르는 term → false
//   2) 포장 방식 (위에서부터 먼저 맞는 것 하나):
//      (가) base 이름이 previous_joint_actions 이고 history_length 가 "없다"
//             → dim 이 base_dim 의 양의 배수여야 한다. 깊이 = dim / base_dim, 간격 1, kNewestFirst
//      (나) history_length 가 있다
//             → length > 0, interval(없으면 1) > 0, dim == base_dim · length 여야 한다. kOldestFirst
//      (다) modifiers 가 비어 있지 않다
//             → 길이 1, 간격 1 짜리 history 로 감싼다 (필터만 건 term)
//      (라) 그 외 → 포장 없음. 매번 fill_base_term 으로 바로 채운다.
//      (가)(나)(다) 는 02 의 HistoryTerm 으로 만든다. modifiers 는 make_filter_chain 으로.
//   3) 완성된 term 의 dim 이 YAML 의 dim 과 다르면 false
//   init 을 다시 부르면 이전 슬롯은 버리고 처음부터 다시 만든다.
//
// Q4. tick / build_actor_input — 포장 없는 term
//   build 는 슬롯을 "YAML 순서 그대로" 돌면서 자기 구간(offset, dim)에 값을 쓴다.
//
// Q5. tick / build_actor_input — history·필터 term
//   tick: 포장된 term 마다 fill_base_term 으로 지금 값을 뽑아 HistoryTerm::tick 에 넣는다.
//   build: 포장된 term 은 HistoryTerm::fill 로 채운다.
//
//   레포: observation.cpp ObservationManager::init / tick / fill_internal
//   단순화: 레포의 tick(dt) 는 dt 를 받아 gait 위상을 적분한다. 여기서는 위상을 밖에서 주므로 인자가 없다.
// ----------------------------------------------------------------------------
class ObservationManager {
public:
    ObservationManager() = default;
    // 내부 슬롯이 밖의 inputs 를 가리키므로, 복사/이동으로 주소가 바뀌면 위험하다 — 레포도 금지해 둔다.
    ObservationManager(const ObservationManager&) = delete;
    auto operator=(const ObservationManager&) -> ObservationManager& = delete;

    [[nodiscard]] auto init(const std::vector<TermEntry>& terms, const ObsConfig& cfg, const ObsInputs& inputs) -> bool {  // TODO
        (void)terms;
        (void)cfg;
        (void)inputs;
        return false;
    }

    [[nodiscard]] auto total_dim() const -> int {  // TODO
        return 0;
    }

    auto tick() -> void {  // TODO
    }

    auto build_actor_input(std::span<float> out) const -> void {  // TODO
        (void)out;
    }

private:
    // TODO: 필요한 멤버 변수.
    //   힌트: term 하나의 정보를 담는 struct(레포의 Slot)를 만들고 std::vector 로 들고 있는다.
    //         HistoryTerm 은 복사할 수 없으니 std::unique_ptr 로 들고 있으면 편하다.
};

}  // namespace quiz
