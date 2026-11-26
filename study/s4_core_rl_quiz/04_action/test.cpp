// 수정 금지 — quiz.h 를 고쳐서 통과시킬 것.
#include <cmath>
#include <initializer_list>
#include <vector>

#include "check.h"
#include "quiz.h"

namespace {

using quiz::ControllerOutput;
using quiz::MagnetConfig;

using Epm4 = std::array<int, 4>;

auto near(double a, double b, double eps = 1e-9) -> bool {
    return std::abs(a - b) <= eps;
}

// a[first, first + count) 가 전부 value 인가
template <std::size_t N>
auto range_all(const std::array<double, N>& a, std::size_t first, std::size_t count, double value) -> bool {
    for (std::size_t i = first; i < first + count; ++i) {
        if (!near(a[i], value)) return false;
    }
    return true;
}

// golden 정책 YAML 의 control 블록 값
auto golden_action_config() -> quiz::ActionConfig {
    return quiz::ActionConfig{
        .joint_action_scale = 0.4,
        .action_mean = {-0.9377, -1.5307, 1.8549, 0.9331, 1.5307, -1.855, -2.2071, 1.5323, -1.8552, 2.202, -1.5324, 1.8552},
        .joint_kp = 50.0,
        .joint_kd = 5.0,
    };
}

// gate 를 같은 입력으로 n tick 돌려, 발 foot 이 0 이 아닌 신호를 낸 tick 번호를 모은다.
auto fire_ticks(quiz::MagnetActionGate& gate, const MagnetConfig& cfg, int n, int foot, const std::array<float, 4>& held) -> std::vector<int> {
    std::vector<int> fired;
    for (int t = 0; t < n; ++t) {
        const Epm4 out = gate.tick(held, cfg);
        if (out[static_cast<std::size_t>(foot)] != 0) fired.push_back(t);
    }
    return fired;
}

}  // namespace

int main() {
#ifdef TEST_Q1
    SECTION("Q1. joint_actions");
    {
        const quiz::ActionConfig cfg = golden_action_config();

        // raw 가 전부 0 이면 목표 = action_mean (정책의 "기본 자세")
        ControllerOutput zero;
        quiz::apply_joint_action(std::vector<float>(12, 0.0f), cfg, zero);
        CHECK(near(zero.joint.q[0], -0.9377));
        CHECK(near(zero.joint.q[5], -1.855));
        CHECK(near(zero.joint.q[11], 1.8552));

        // raw = 0.5 → mean + 0.4·0.5 = mean + 0.2
        ControllerOutput half;
        quiz::apply_joint_action(std::vector<float>(12, 0.5f), cfg, half);
        CHECK(near(half.joint.q[0], -0.7377));
        CHECK(near(half.joint.q[2], 2.0549));
        CHECK(near(half.joint.q[11], 2.0552));

        // 관절마다 다른 raw: raw[i] = (i - 6)·0.25
        std::vector<float> ramp(12);
        for (std::size_t i = 0; i < 12; ++i) ramp[i] = (static_cast<float>(i) - 6.0f) * 0.25f;
        ControllerOutput out;
        out.joint.q.fill(9.0);    // "원래 들어 있던 값" 표시
        out.joint.kp.fill(7.0);
        out.joint.kd.fill(7.0);
        out.joint.dq.fill(3.0);
        out.joint.tau.fill(3.0);
        out.epm.fill(1);
        quiz::apply_joint_action(ramp, cfg, out);
        CHECK(near(out.joint.q[0], -1.5377));   // -0.9377 + 0.4·(-1.5)
        CHECK(near(out.joint.q[6], -2.2071));   // raw 0
        CHECK(near(out.joint.q[11], 2.3552));   // 1.8552 + 0.4·1.25
        CHECK(range_all(out.joint.kp, 0, 12, 50.0));
        CHECK(range_all(out.joint.kd, 0, 12, 5.0));

        // 다리 12개 밖은 건드리지 않는다
        CHECK(range_all(out.joint.q, 12, 6, 9.0));
        CHECK(range_all(out.joint.kp, 12, 6, 7.0));
        CHECK(range_all(out.joint.kd, 12, 6, 7.0));
        CHECK(range_all(out.joint.dq, 0, 18, 3.0));
        CHECK(range_all(out.joint.tau, 0, 18, 3.0));
        CHECK(out.epm == (std::array<int, 8>{1, 1, 1, 1, 1, 1, 1, 1}));
    }
#endif

#ifdef TEST_Q2
    SECTION("Q2. 자석 행동 이산화");
    {
        const MagnetConfig golden{.action_on_threshold = 0.3, .action_off_threshold = -0.3, .max_action_only = true};
        CHECK(quiz::discretize_magnet_action(0.9f, golden) == 1);
        CHECK(quiz::discretize_magnet_action(-0.9f, golden) == -1);
        CHECK(quiz::discretize_magnet_action(0.1f, golden) == 0);
        CHECK(quiz::discretize_magnet_action(-0.2f, golden) == 0);
        CHECK(quiz::discretize_magnet_action(0.0f, golden) == 0);

        // 경계는 0. (0.5 는 float 와 double 에서 값이 정확히 같아 경계 검사에 쓸 수 있다)
        const MagnetConfig half{.action_on_threshold = 0.5, .action_off_threshold = -0.5, .max_action_only = false};
        CHECK(quiz::discretize_magnet_action(0.5f, half) == 0);
        CHECK(quiz::discretize_magnet_action(-0.5f, half) == 0);
        CHECK(quiz::discretize_magnet_action(0.5001f, half) == 1);
        CHECK(quiz::discretize_magnet_action(-0.5001f, half) == -1);

        // on / off 문턱은 대칭이 아니어도 된다
        const MagnetConfig skewed{.action_on_threshold = 0.6, .action_off_threshold = -0.2, .max_action_only = false};
        CHECK(quiz::discretize_magnet_action(0.5f, skewed) == 0);
        CHECK(quiz::discretize_magnet_action(-0.25f, skewed) == -1);
    }
#endif

#ifdef TEST_Q3
    SECTION("Q3. MagnetActionGate — 환산과 중재");
    {
        // cooldown [ms] → tick. rl_config.yaml: foot 200 ms, global 50 ms. 정책 주기 10 ms.
        quiz::MagnetActionGate gate;
        gate.configure(10.0, 200.0, 50.0);
        CHECK(gate.foot_cooldown_ticks() == 20);
        CHECK(gate.global_cooldown_ticks() == 5);
        gate.configure(20.0, 200.0, 50.0);         // 정책 주기가 20 ms 이면: 50/20 = 2.5 → 올림
        CHECK(gate.foot_cooldown_ticks() == 10);
        CHECK(gate.global_cooldown_ticks() == 3);
        gate.configure(10.0, 0.0, -5.0);           // 0 이하는 cooldown 없음
        CHECK(gate.foot_cooldown_ticks() == 0);
        CHECK(gate.global_cooldown_ticks() == 0);
        gate.configure(10.0, 25.0, 1.0);           // 조금이라도 넘으면 한 tick 을 더 기다린다
        CHECK(gate.foot_cooldown_ticks() == 3);
        CHECK(gate.global_cooldown_ticks() == 1);

        // 아래는 cooldown 0 — 매 tick 이 독립이다.
        const MagnetConfig all{.action_on_threshold = 0.3, .action_off_threshold = -0.3, .max_action_only = false};
        const MagnetConfig one{.action_on_threshold = 0.3, .action_off_threshold = -0.3, .max_action_only = true};
        quiz::MagnetActionGate g;
        g.configure(10.0, 0.0, 0.0);

        // max_action_only 가 꺼져 있으면 문턱을 넘은 발이 전부 나간다
        CHECK(g.tick({0.9f, -0.9f, 0.1f, -0.2f}, all) == (Epm4{1, -1, 0, 0}));
        CHECK(g.tick({0.0f, 0.0f, 0.0f, 0.0f}, all) == (Epm4{0, 0, 0, 0}));

        // 켜져 있으면 |값| 이 가장 큰 발 하나만
        CHECK(g.tick({0.5f, -0.9f, 0.4f, 0.0f}, one) == (Epm4{0, -1, 0, 0}));
        CHECK(g.tick({0.5f, 0.4f, 0.0f, 0.7f}, one) == (Epm4{0, 0, 0, 1}));
        // 같으면 번호가 작은 발
        CHECK(g.tick({0.0f, 0.8f, -0.8f, 0.8f}, one) == (Epm4{0, 1, 0, 0}));
        // 문턱을 넘은 발이 없으면 아무도 안 나간다
        CHECK(g.tick({0.2f, -0.25f, 0.1f, 0.0f}, one) == (Epm4{0, 0, 0, 0}));

        // "가장 큰 발" 은 문턱을 넘은 발 중에서만 고른다:
        //   on 0.3 / off -0.9 에서 발 1 은 |-0.8| 로 가장 크지만 -0.9 보다 작지 않아 후보가 아니다
        const MagnetConfig skewed{.action_on_threshold = 0.3, .action_off_threshold = -0.9, .max_action_only = true};
        CHECK(g.tick({0.5f, -0.8f, 0.0f, 0.0f}, skewed) == (Epm4{1, 0, 0, 0}));
    }
#endif

#ifdef TEST_Q4
    SECTION("Q4. MagnetActionGate — cooldown");
    {
        const MagnetConfig all{.action_on_threshold = 0.3, .action_off_threshold = -0.3, .max_action_only = false};
        const MagnetConfig one{.action_on_threshold = 0.3, .action_off_threshold = -0.3, .max_action_only = true};

        // 손으로 따라가기: 주기 10 ms, foot 30 ms(3 tick), global 20 ms(2 tick)
        quiz::MagnetActionGate g;
        g.configure(10.0, 30.0, 20.0);
        CHECK(g.tick({0.9f, 0.0f, 0.0f, 0.0f}, all) == (Epm4{1, 0, 0, 0}));   // t0: 발 0 스위칭 → foot0=3, global=2
        CHECK(g.tick({0.0f, -0.9f, 0.0f, 0.0f}, all) == (Epm4{0, 0, 0, 0}));  // t1: global=1 → 다른 발도 막힌다
        CHECK(g.tick({0.0f, -0.9f, 0.0f, 0.0f}, all) == (Epm4{0, -1, 0, 0})); // t2: global=0 → 발 1 통과
        CHECK(g.tick({0.9f, 0.0f, 0.0f, 0.0f}, all) == (Epm4{0, 0, 0, 0}));   // t3: 발 0 은 foot0=0 이지만 global=1

        // 같은 명령을 계속 들고 있으면 cooldown 이 풀릴 때마다 다시 나간다 (gate 는 자석의 현재 상태를 모른다).
        // 막힌 tick 은 cooldown 을 다시 채우지 않는다 — 채우면 여기서 영원히 못 나간다.
        quiz::MagnetActionGate held;
        held.configure(10.0, 30.0, 20.0);
        CHECK(fire_ticks(held, all, 8, 0, {0.9f, 0.0f, 0.0f, 0.0f}) == (std::vector<int>{0, 3, 6}));

        // global 이 foot 보다 길면 global 이 간격을 정한다
        quiz::MagnetActionGate slow_global;
        slow_global.configure(10.0, 20.0, 40.0);
        CHECK(fire_ticks(slow_global, all, 9, 0, {0.9f, 0.0f, 0.0f, 0.0f}) == (std::vector<int>{0, 4, 8}));

        // max_action_only 가 꺼져 있으면 같은 tick 에 두 발이 함께 나갈 수 있다
        quiz::MagnetActionGate both;
        both.configure(10.0, 30.0, 20.0);
        CHECK(both.tick({0.9f, 0.9f, 0.0f, 0.0f}, all) == (Epm4{1, 1, 0, 0}));

        // 켜져 있으면 한 tick 에 하나씩, global cooldown 만큼 띄워서
        quiz::MagnetActionGate seq;
        seq.configure(10.0, 30.0, 20.0);
        CHECK(seq.tick({0.9f, 0.9f, 0.0f, 0.0f}, one) == (Epm4{1, 0, 0, 0}));  // t0: 동률 → 발 0
        CHECK(seq.tick({0.9f, 0.9f, 0.0f, 0.0f}, one) == (Epm4{0, 0, 0, 0}));  // t1: global
        CHECK(seq.tick({0.9f, 0.9f, 0.0f, 0.0f}, one) == (Epm4{0, 1, 0, 0}));  // t2: 발 0 은 자기 cooldown 중 → 후보는 발 1 뿐
        CHECK(seq.tick({0.9f, 0.9f, 0.0f, 0.0f}, one) == (Epm4{0, 0, 0, 0}));  // t3: global
        CHECK(seq.tick({0.9f, 0.9f, 0.0f, 0.0f}, one) == (Epm4{1, 0, 0, 0}));  // t4: 발 0 cooldown 끝 (t0 + 3 은 t3 였지만 global 에 막혔다)

        // reset 은 남은 cooldown 을 지운다 (환산해 둔 tick 수는 그대로)
        quiz::MagnetActionGate r;
        r.configure(10.0, 30.0, 20.0);
        CHECK(r.tick({0.9f, 0.0f, 0.0f, 0.0f}, all) == (Epm4{1, 0, 0, 0}));
        r.reset();
        CHECK(r.tick({0.9f, 0.0f, 0.0f, 0.0f}, all) == (Epm4{1, 0, 0, 0}));
        CHECK(r.tick({0.9f, 0.0f, 0.0f, 0.0f}, all) == (Epm4{0, 0, 0, 0}));
        CHECK(r.foot_cooldown_ticks() == 3);

        // 실제 설정: 주기 10 ms, foot 200 ms(20 tick), global 50 ms(5 tick)
        //   발 0 은 +0.9 를 계속 들고, 발 1 은 t1 부터 -0.9 를 계속 든다.
        quiz::MagnetActionGate real;
        real.configure(10.0, 200.0, 50.0);
        std::vector<int> foot0;
        std::vector<int> foot1;
        for (int t = 0; t <= 40; ++t) {
            const Epm4 out = real.tick({0.9f, t >= 1 ? -0.9f : 0.0f, 0.0f, 0.0f}, all);
            if (out[0] != 0) foot0.push_back(t);
            if (out[1] != 0) foot1.push_back(t);
        }
        CHECK(foot0 == (std::vector<int>{0, 20, 40}));  // 같은 발: 200 ms 간격
        CHECK(foot1 == (std::vector<int>{5, 25}));      // 다른 발: 50 ms 뒤, 그다음은 자기 200 ms
    }
#endif

#ifdef TEST_Q5
    SECTION("Q5. 팔 hold 와 safe-hold");
    {
        quiz::HoldConfig cfg;
        cfg.joint_kp = 50.0;
        cfg.joint_kd = 5.0;
        cfg.arm_kp.fill(200.0);  // joint_pd_ctrl_config.yaml 의 팔 게인
        cfg.arm_kd.fill(20.0);
        const std::array<double, 6> kHeldArm{0.1, 0.2, 0.3, 0.4, 0.5, 0.6};

        // (a) 팔 구간만 채운다
        ControllerOutput out;
        out.joint.q.fill(9.0);
        out.joint.kp.fill(7.0);
        out.joint.kd.fill(7.0);
        quiz::apply_manipulator_hold(kHeldArm, cfg, out);
        CHECK(near(out.joint.q[12], 0.1) && near(out.joint.q[17], 0.6));
        CHECK(range_all(out.joint.kp, 12, 6, 200.0));
        CHECK(range_all(out.joint.kd, 12, 6, 20.0));
        CHECK(range_all(out.joint.q, 0, 12, 9.0));   // 다리는 그대로
        CHECK(range_all(out.joint.kp, 0, 12, 7.0));
        CHECK(range_all(out.joint.kd, 0, 12, 7.0));

        // (b) safe-hold
        std::array<double, 12> last_target{};
        for (std::size_t i = 0; i < 12; ++i) last_target[i] = 0.1 * static_cast<double>(i) - 0.5;
        const ControllerOutput safe = quiz::make_safe_hold(last_target, kHeldArm, cfg);
        CHECK(near(safe.joint.q[0], -0.5) && near(safe.joint.q[5], 0.0) && near(safe.joint.q[11], 0.6));
        CHECK(range_all(safe.joint.kp, 0, 12, 50.0));  // 게인을 0 으로 두면 다리가 풀려 주저앉는다
        CHECK(range_all(safe.joint.kd, 0, 12, 5.0));
        CHECK(near(safe.joint.q[12], 0.1) && near(safe.joint.q[17], 0.6));
        CHECK(range_all(safe.joint.kp, 12, 6, 200.0));
        CHECK(range_all(safe.joint.kd, 12, 6, 20.0));
        CHECK(range_all(safe.joint.dq, 0, 18, 0.0));
        CHECK(range_all(safe.joint.tau, 0, 18, 0.0));
        CHECK(safe.epm == (std::array<int, 8>{}));     // 깨진 출력으로 자석을 건드리지 않는다
    }
#endif

#ifdef TEST_Q6
    SECTION("Q6. 정책 전환 램프");
    {
        // rl_config.yaml: switch_blend_ms 200. 정책 주기 10 ms → 20 tick
        CHECK(quiz::blend_ticks(200.0, 0.01) == 20);
        CHECK(quiz::blend_ticks(200.0, 0.02) == 10);
        CHECK(quiz::blend_ticks(0.0, 0.01) == 0);     // 0 = 즉시 전환
        CHECK(quiz::blend_ticks(-50.0, 0.01) == 0);   // 음수도 0
        CHECK(quiz::blend_ticks(25.0, 0.01) == 3);    // 2.5 → 올림

        std::array<double, 12> from{};
        from.fill(1.0);
        const auto blended = [](quiz::SwitchBlend& blend, double target_value) -> double {
            std::array<double, 12> target{};
            target.fill(target_value);
            blend.apply(target);
            return target[0];
        };

        // 4 tick 램프: 1.0 에서 3.0 으로
        quiz::SwitchBlend blend;
        CHECK(!blend.active());
        blend.start(from, 4);
        CHECK(blend.active());
        CHECK(near(blended(blend, 3.0), 1.0));  // alpha 0    — 첫 tick 은 from 그대로다
        CHECK(near(blended(blend, 3.0), 1.5));  // alpha 0.25
        CHECK(near(blended(blend, 3.0), 2.0));  // alpha 0.5
        CHECK(near(blended(blend, 3.0), 2.5));  // alpha 0.75
        CHECK(!blend.active());
        CHECK(near(blended(blend, 3.0), 3.0));  // 램프 끝 — 손대지 않는다

        // 새 정책의 목표가 매 tick 움직여도 그 tick 의 목표를 기준으로 섞는다
        quiz::SwitchBlend moving;
        moving.start(from, 2);
        CHECK(near(blended(moving, 5.0), 1.0));  // alpha 0
        CHECK(near(blended(moving, 7.0), 4.0));  // alpha 0.5: 1 + 0.5·(7 - 1)
        CHECK(near(blended(moving, 9.0), 9.0));

        // 관절마다 따로 섞는다: from[i] = i, 새 목표는 전부 0
        quiz::SwitchBlend per_joint;
        std::array<double, 12> ramp_from{};
        for (std::size_t i = 0; i < 12; ++i) ramp_from[i] = static_cast<double>(i);
        per_joint.start(ramp_from, 2);
        std::array<double, 12> first{};
        per_joint.apply(first);   // alpha 0
        CHECK(near(first[0], 0.0) && near(first[3], 3.0) && near(first[11], 11.0));
        std::array<double, 12> second{};
        per_joint.apply(second);  // alpha 0.5
        CHECK(near(second[3], 1.5) && near(second[11], 5.5));

        // ticks 0 = 램프 없음
        quiz::SwitchBlend none;
        none.start(from, 0);
        CHECK(!none.active());
        CHECK(near(blended(none, 3.0), 3.0));
    }
#endif

    return report();
}
