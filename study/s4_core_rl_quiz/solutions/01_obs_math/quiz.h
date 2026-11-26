// ============================================================================
// 01. obs math — "관측(observation) 한 항의 값은 어떻게 계산되는가"
// ============================================================================
// 이 파일만 수정한다. test.cpp 는 건드리지 않는다.
// 검사: s4_core_rl_quiz/ 에서 `make 01` (문항별: `make 01 Q=2`)
//
// RL 정책은 매 주기 "관측 벡터"(float 배열) 하나를 입력으로 받는다. 그 벡터는
// 여러 항(term)을 이어 붙인 것이고, 이 퀴즈는 항 하나하나의 "값"을 만드는 수식을 다룬다.
// 벡터로 이어 붙이는 일은 03 에서 한다.
//
// 레포: src/applications/controller/runner/rl_controller/observation.cpp
//       include/diden_core/applications/controller/runner/rl_controller/policy.h (CommandLimits)
// 막히면 이 폴더의 GUIDE.md 를 본다.
// ============================================================================
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <optional>

namespace quiz {

// 레포: include/diden_core/engine/models/diden_spider/constants.h
inline constexpr int kNumLegs = 4;
inline constexpr int kNumJointsPerLeg = 3;
inline constexpr int kNumMobileJoints = kNumLegs * kNumJointsPerLeg;  // 12

using Vec3 = std::array<double, 3>;
using Vec3f = std::array<float, 3>;
using Mat3 = std::array<std::array<double, 3>, 3>;  // R[행][열]

// 쿼터니언. 순서는 MuJoCo 와 같은 (w, x, y, z) — state.q[3..7) 에 이 순서로 들어 있다.
struct Quat {
    double w{1.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

// ----------------------------------------------------------------------------
// Q1. 쿼터니언 → 회전행렬
//   단위 쿼터니언 q = (w, x, y, z) 를 3x3 회전행렬 R 로 바꿔라.
//   R 은 "body 좌표의 벡터를 world 좌표로" 옮긴다:  v_world = R · v_body
//   (R 의 열 3개 = body 의 x, y, z 축을 world 좌표로 쓴 것)
//
//   레포에서는 이 변환을 직접 하지 않고 MuJoCo(mj_kinematics)가 해 준 결과
//   xmat 을 읽는다 (ModelQuery::get_body_rotation). 여기서는 그 안쪽을 손으로 만든다.
//   q 는 단위 쿼터니언이라고 가정한다 (정규화는 하지 않아도 된다).
// ----------------------------------------------------------------------------
inline auto quat_to_rotation(const Quat& q) -> Mat3 {  // TODO
    const double w = q.w;
    const double x = q.x;
    const double y = q.y;
    const double z = q.z;
    return Mat3{{
        {1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)},
        {2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)},
        {2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)},
    }};
}

// ----------------------------------------------------------------------------
// Q2. projected_gravity
//   world 중력 방향 g_w = (0, 0, -1) 을 body 좌표로 옮긴 값 (크기 1 인 방향 벡터).
//     g_b = Rᵀ · g_w
//   결과는 float 3개 — 관측 벡터는 전부 float 이다.
//   레포: observation.cpp ProjectedGravitySource::fill
// ----------------------------------------------------------------------------
inline auto projected_gravity(const Mat3& rotation) -> Vec3f {  // TODO
    // Rᵀ · (0, 0, -1) = -(R 의 3번째 행)
    return Vec3f{
        static_cast<float>(-rotation[2][0]),
        static_cast<float>(-rotation[2][1]),
        static_cast<float>(-rotation[2][2]),
    };
}

// ----------------------------------------------------------------------------
// Q3. 속도 명령 제한(CommandLimits) 만들기
//   정책 YAML 의 control.command_limits 를 읽어 채널별 [lower, upper] 를 만든다.
//   채널 순서는 [lin_vel_x, lin_vel_y, ang_vel_z].
//
//     - 채널 자체가 없으면(nullopt)            → 그 채널은 [-kDefaultBound, +kDefaultBound]
//     - min 만 / max 만 있으면                  → 없는 쪽만 기본 바운드
//     - 만들고 보니 lower <= upper 가 아니면    → 그 채널을 통째로 기본 바운드로 되돌린다
//       (min > max 뿐 아니라 NaN 바운드도 여기 걸려야 한다)
//
//   레포: policy.h CommandLimits, policy.cpp build_command_limits
// ----------------------------------------------------------------------------
struct CommandLimits {
    static constexpr int kDim = 3;
    static constexpr double kDefaultBound = 5.0;  // [m/s, rad/s]

    Vec3 lower{-kDefaultBound, -kDefaultBound, -kDefaultBound};
    Vec3 upper{kDefaultBound, kDefaultBound, kDefaultBound};
};

struct RawCommandBound {
    std::optional<double> min;
    std::optional<double> max;
};

// [lin_vel_x, lin_vel_y, ang_vel_z] — 정책 YAML 에 안 적힌 채널은 nullopt
using RawCommandLimits = std::array<std::optional<RawCommandBound>, CommandLimits::kDim>;

inline auto build_command_limits(const RawCommandLimits& raw) -> CommandLimits {  // TODO
    CommandLimits out;
    for (int i = 0; i < CommandLimits::kDim; ++i) {
        const auto& bound = raw[static_cast<std::size_t>(i)];
        if (bound) {
            if (bound->min) out.lower[static_cast<std::size_t>(i)] = *bound->min;
            if (bound->max) out.upper[static_cast<std::size_t>(i)] = *bound->max;
        }
        // NaN 은 어떤 비교도 false 라서 "!(a <= b)" 형태로 써야 함께 걸린다.
        if (!(out.lower[static_cast<std::size_t>(i)] <= out.upper[static_cast<std::size_t>(i)])) {
            out.lower[static_cast<std::size_t>(i)] = -CommandLimits::kDefaultBound;
            out.upper[static_cast<std::size_t>(i)] = CommandLimits::kDefaultBound;
        }
    }
    return out;
}

// ----------------------------------------------------------------------------
// Q4. 명령 clamp
//   cmd = (vx, vy, wz) 를 채널별로 [lower, upper] 안으로 자른다.
//   레포: policy.h CommandLimits::clamp — RlController::step 이 명령 스냅샷을 뜨는 자리에서
//        딱 한 번 부른다. 그 뒤의 obs term 은 전부 "이미 잘린 값"을 읽는다.
// ----------------------------------------------------------------------------
inline auto clamp_command(const Vec3& cmd, const CommandLimits& limits) -> Vec3 {  // TODO
    Vec3 out = cmd;
    for (std::size_t i = 0; i < out.size(); ++i) {
        out[i] = std::clamp(cmd[i], limits.lower[i], limits.upper[i]);
    }
    return out;
}

// ----------------------------------------------------------------------------
// Q5. command 관측과 deadzone
//   (a) command_obs: 성분별로 |v| < threshold 이면 0, 아니면 그대로 (float 로).
//       레포: observation.cpp GeneratedCommandsSource::fill
//   (b) command_active: "명령이 들어와 있는가" — gait 위상을 진행시킬지 정하는 판정.
//         max( sqrt(vx² + vy²), |wz| ) > threshold
//       레포: observation.cpp GaitPhiSource::tick (cmd_mag / kCommanded)
//
//   두 판정은 기준이 다르다 — (a) 는 성분별·미만(<), (b) 는 크기·초과(>).
//   test.cpp 에 "obs 는 0 인데 active 는 true" 인 입력이 있다.
// ----------------------------------------------------------------------------
inline auto command_obs(const Vec3& cmd, double threshold) -> Vec3f {  // TODO
    Vec3f out{};
    for (std::size_t i = 0; i < out.size(); ++i) {
        const double value = std::abs(cmd[i]) < threshold ? 0.0 : cmd[i];
        out[i] = static_cast<float>(value);
    }
    return out;
}

inline auto command_active(const Vec3& cmd, double threshold) -> bool {  // TODO
    const double magnitude = std::max(std::sqrt(cmd[0] * cmd[0] + cmd[1] * cmd[1]), std::abs(cmd[2]));
    return magnitude > threshold;
}

// ----------------------------------------------------------------------------
// Q6. gait_phi — 다리별 보행 위상을 (cos, sin) 으로
//   phase_ratio        : 한 걸음 주기 안에서 지금 어디인가, [0, 1)
//   leg_phase_offsets  : 다리별 위상 차 (비율). 다리 순서 RR, RL, FR, FL.
//                        crawl = [0, 0.25, 0.5, 0.75], trot = [0, 0.5, 0.5, 0]
//
//     phi_i = 2π · phase_ratio + 2π · fmod(offset_i, 1.0)
//     out   = [cos(phi_0), sin(phi_0), cos(phi_1), sin(phi_1), ...]   (다리별로 붙여서, cos 먼저)
//
//   레포: observation.cpp GaitPhiSource 생성자(offsets_rad_) + fill
//   단순화: 레포는 phase_ratio 를 term 내부에서 적분한다(적응형 주기, 명령 0 이면 동결).
//           여기서는 밖에서 받은 phase_ratio 를 인코딩하는 부분만 만든다.
// ----------------------------------------------------------------------------
inline auto gait_phi(double phase_ratio, const std::array<double, kNumLegs>& leg_phase_offsets) -> std::array<float, 2 * kNumLegs> {  // TODO
    constexpr double k2pi = 2.0 * std::numbers::pi_v<double>;
    std::array<float, 2 * kNumLegs> out{};
    const double base = phase_ratio * k2pi;
    for (std::size_t i = 0; i < leg_phase_offsets.size(); ++i) {
        const double phi = base + std::fmod(leg_phase_offsets[i], 1.0) * k2pi;
        out[2 * i] = static_cast<float>(std::cos(phi));
        out[2 * i + 1] = static_cast<float>(std::sin(phi));
    }
    return out;
}

}  // namespace quiz
