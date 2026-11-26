// ============================================================================
// 06. pipeline — "제어 한 주기: 센서 → 관측 → 추론 → 행동 → 모터 명령"
// ============================================================================
// 이 파일만 수정한다. 검사: `make 06` (문항별: `make 06 Q=4`)
//
// 01 ~ 05 에서 만든 부품을 한 줄로 잇는다. 새로 배우는 수식은 거의 없고,
// "무엇을 어떤 순서로 부르는가" 와 "실패했을 때 무엇을 내보내는가" 가 전부다.
// (01 ~ 05 의 quiz.h 를 그대로 include 한다 — 앞 문제를 먼저 통과시킬 것)
//
// 추론(ONNX Runtime)은 std::function 으로 바꿔 끼울 수 있게 해서, ONNX 없이도 테스트한다.
//
// 레포: src/applications/controller/runner/rl_controller/rl_controller.cpp (on_enter, activate_policy, step)
//       src/engine/inference/inference_engine.cpp (run)
//       src/driver/drec_baseboard/tx/thread.cpp (build_command)
// ============================================================================
#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <span>
#include <utility>
#include <vector>

#include "../01_obs_math/quiz.h"
#include "../02_history_filter/quiz.h"
#include "../03_obs_layout/quiz.h"
#include "../04_action/quiz.h"
#include "../05_timing_contract/quiz.h"

namespace quiz {

// DREC 보드에 실제로 달린 모터 축 수. 레포: config/diden_spider/drec_baseboard_config.yaml motor.count
// (다리 12개만이다 — ControllerOutput 의 뒤 6칸(팔)은 이 보드로 나가지 않는다)
inline constexpr int kMotorCount = 12;

// PC → 보드 와이어 포맷의 모터 한 축. 레포: include/diden_core/driver/drec_baseboard/tx/protocol.h
#pragma pack(push, 1)
struct MotorCommand {
    float pos;                 // 목표 위치 [rad]
    float vel;                 // 목표 속도 [rad/s]
    float kp;                  // 위치 게인
    float kd;                  // 속도 게인
    float tau;                 // 목표 토크 [N·m]
    std::int8_t mf_high_time;  // 자석 펄스 (이 퀴즈에서는 항상 0)
    std::uint8_t mf_low_time;
    std::uint16_t led_rgb;
};
#pragma pack(pop)
static_assert(sizeof(MotorCommand) == 24);

// ----------------------------------------------------------------------------
// Q1. 관절 명령 → MotorCommand
//   컨트롤러는 "관절 좌표" 로 말하고 모터는 "모터 좌표" 로 듣는다. 축 i (0..11) 마다:
//
//     position = (q[i] + offset[i]) · direction[i]
//     velocity = dq[i] · direction[i]
//     torque   = tau[i] · direction[i]
//
//   그리고 double → float 로 줄여 싣는다. 이때 "float 로 실을 수 있는 값" 인지 확인한다:
//     실을 수 있다 = 유한하고(NaN/Inf 아님) |값| <= float 최댓값
//
//     pos = position 을 실을 수 있으면 그 값, 아니면 0
//     vel, tau = 실을 수 있으면 그 값, 아니면 0
//     kp  = position 을 실을 수 있고, kp[i] 도 실을 수 있고 0 이상이면 그 값, 아니면 0
//     kd  = kd[i] 를 실을 수 있고 0 이상이면 그 값, 아니면 0
//     mf_high_time, mf_low_time, led_rgb = 0
//
//   kp 규칙에 주목: 위치가 깨져서 0 을 실었는데 kp 가 살아 있으면 모터가 0 rad 로 끌려간다.
//   그래서 위치가 깨지면 kp 도 같이 끈다 (kd 만 남아 감쇠만 걸린다).
//   레포: tx/thread.cpp build_command 의 모터 루프, wire_representable / to_wire_float
//   단순화: 자석 펄스(mf_*)와 LED 는 다른 경로(EpmSequencer → epm_tx)가 채운다. 여기서는 0 으로 둔다.
// ----------------------------------------------------------------------------
inline auto to_motor_commands(const ControllerOutput::JointCommand& cmd, const std::array<double, kMotorCount>& offset, const std::array<double, kMotorCount>& direction)
    -> std::array<MotorCommand, kMotorCount> {  // TODO
    (void)cmd;
    (void)offset;
    (void)direction;
    return {};
}

// ----------------------------------------------------------------------------
// Q2. 추론 한 번 (NaN 가드 포함)
//   Model 은 "입력 차원, 출력 차원, 실행 함수" 묶음이다. forward 가 ONNX Runtime 의
//   session.Run() 자리이고, 테스트는 여기에 가짜 함수를 끼운다.
//
//   run_inference 는 아래 순서로 확인하고, 하나라도 걸리면 false:
//     1) model.forward 가 비어 있다 (로드되지 않은 엔진)
//     2) input.size() != input_dim  또는  output.size() != output_dim
//     3) input 에 NaN/Inf 가 있다          → forward 를 "부르지 않고" false
//     4) forward 가 false 를 돌려줬다      (ONNX Runtime 예외에 해당)
//     5) output 에 NaN/Inf 가 있다
//   전부 통과하면 true.
//   레포: inference_engine.cpp InferenceEngine::run
// ----------------------------------------------------------------------------
struct Model {
    int input_dim = 0;
    int output_dim = 0;
    std::function<bool(std::span<const float> input, std::span<float> output)> forward;
};

inline auto run_inference(const Model& model, std::span<const float> input, std::span<float> output) -> bool {  // TODO
    (void)model;
    (void)input;
    (void)output;
    return false;
}

// ----------------------------------------------------------------------------
// Q3 ~ Q6. RlPipeline — RlController 의 뼈대
// ----------------------------------------------------------------------------

// 정책 YAML 에서 온 것 (정책마다 다르다). 레포: policy.h PolicyConfig
struct PolicySpec {
    std::vector<TermEntry> observations;
    std::vector<TermEntry> actions;  // {joint_actions, 12}, {magnetic_actions, 4}
    ObsConfig obs;
    ActionConfig action;
    MagnetConfig magnet;
    CommandLimits command_limits;
    std::optional<int> decimation;
    std::optional<double> physics_dt;
};

// 로봇 쪽 설정 (정책과 무관). 레포: config/diden_spider/rl_config.yaml, joint_pd_ctrl_config.yaml
struct RuntimeConfig {
    double foot_cooldown_ms = 200.0;
    double global_cooldown_ms = 50.0;
    std::array<double, kNumManipulatorJoints> arm_kp{};
    std::array<double, kNumManipulatorJoints> arm_kd{};
};

// step 진입부에서 전역 상태를 떠 온 사본. 레포: g_system_state.sensor_raw / state / desired.velocity
struct Snapshot {
    Vec3 gyro{};
    Quat base_quat{};
    std::array<double, kNumJoints> joint_q{};   // [다리 12, 팔 6]
    std::array<double, kNumJoints> joint_dq{};
    Vec3 desired_velocity{};                    // 운영자가 보낸 원본 (clamp 전)
    double gait_phase_ratio = 0.0;
};

class RlPipeline {
public:
    RlPipeline() = default;
    // 내부의 ObservationManager 가 이 객체의 멤버(ObsInputs)를 가리킨다 — 복사/이동 금지.
    RlPipeline(const RlPipeline&) = delete;
    auto operator=(const RlPipeline&) -> RlPipeline& = delete;

    // ------------------------------------------------------------------------
    // Q3. init — 로드 시점의 계약 검사 + 초기 상태 잡기 (레포의 Policy::load + on_enter)
    //   아래 중 하나라도 어긋나면 false. false 를 돌려준 뒤에는 step() 이 0 출력만 낸다.
    //     - model.forward 가 비어 있다
    //     - 05 derive_cycle_time_us(spec.decimation, spec.physics_dt) 가 nullopt
    //     - 05 validate_term_list 가 observations / actions 중 하나라도 false
    //     - 05 validate_control_gains(joint_kp, joint_kd) 가 false
    //     - 05 validate_dims(...) 가 kOk 가 아니다
    //     - actions 에 joint_actions(dim 12) / magnetic_actions(dim 4) 가 아닌 것이 있다
    //       (이름을 모르거나, 이름은 맞는데 dim 이 다르다)
    //     - (estimator_output 을 뺀) observations 로 03 ObservationManager::init 이 실패한다
    //     - ObservationManager::total_dim() != model.input_dim
    //
    //   성공하면 다음 상태를 잡아 둔다:
    //     - cycle_time_us()            = 유도한 정책 주기
    //     - 자석 gate                   = reset 후 configure(주기[ms], foot_cooldown_ms, global_cooldown_ms)
    //     - last_target_dof_pos        = now.joint_q 의 앞 12개   (첫 주기의 dof_error 가 0 이 되도록)
    //     - 팔 hold 자세                = now.joint_q 의 뒤 6개
    //     - last_magnetic_action       = 0
    // ------------------------------------------------------------------------
    [[nodiscard]] auto init(const PolicySpec& spec, const RuntimeConfig& runtime, Model model, const Snapshot& now) -> bool {  // TODO
        (void)spec;
        (void)runtime;
        (void)model;
        (void)now;
        return false;
    }

    // init 이 성공하기 전에는 0.
    [[nodiscard]] auto cycle_time_us() const -> int {  // TODO
        return 0;
    }

    // ------------------------------------------------------------------------
    // Q4 ~ Q6. step — 제어 한 주기
    //   init 이 성공하지 않았으면 ControllerOutput{} (전부 0) 을 돌려준다.
    //
    //   1) 스냅샷을 ObsInputs 로 옮긴다: gyro, base_quat, joint_q/dq 의 앞 12개, gait_phase_ratio.
    //      속도 명령은 여기서 딱 한 번 clamp 한다:  command = clamp_command(desired_velocity, command_limits)
    //   2) ObservationManager::tick()
    //   3) build_actor_input 으로 관측 벡터를 만든다
    //   4) run_inference. 실패하면 → make_safe_hold(last_target_dof_pos, 팔 hold 자세, 게인) 를 "바로 return".
    //      (아래 5 ~ 7 을 건너뛴다 — gate 도 돌지 않고 last_* 도 갱신되지 않는다)
    //   5) 행동을 term 순서대로 잘라 적용한다:
    //        joint_actions    → 04 apply_joint_action
    //        magnetic_actions → 원소마다 tanh → gate.tick → 결과를 out.epm 의 앞 4칸에
    //   6) 04 apply_manipulator_hold
    //   7) 다음 주기의 관측이 읽을 값을 갱신한다:
    //        last_target_dof_pos  = 방금 내보낸 out.joint.q 의 앞 12개
    //        last_magnetic_action = 5) 의 tanh 값 (magnetic_actions term 이 없는 정책이면 0)
    //
    // Q4 는 첫 주기, Q5 는 주기 사이에 이어지는 값(7 → 다음 1 ~ 3)과 자석 cooldown,
    // Q6 은 4) 의 실패 경로를 검사한다.
    //
    //   레포: rl_controller.cpp RlController::step
    //   단순화: 모션 클립 재생, 정책 체인 전환(04 Q6 의 blend), 자석 phase override, 로깅용 publish 는 뺐다.
    // ------------------------------------------------------------------------
    [[nodiscard]] auto step(const Snapshot& snap) -> ControllerOutput {  // TODO
        (void)snap;
        return {};
    }

private:
    // TODO: 필요한 멤버 변수.
    //   힌트: ObsInputs 를 멤버로 두고 ObservationManager 에 그 참조를 넘긴다.
    //         선언 순서에 주의 — 멤버는 선언 순서대로 생성되고 역순으로 소멸한다.
};

}  // namespace quiz
