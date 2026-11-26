// 수정 금지 — quiz.h 를 고쳐서 통과시킬 것.
#include <cmath>
#include <initializer_list>
#include <limits>
#include <vector>

#include "check.h"
#include "quiz.h"

namespace {

using quiz::ControllerOutput;
using quiz::Model;
using quiz::PolicySpec;
using quiz::RuntimeConfig;
using quiz::Snapshot;
using quiz::Tensor;
using quiz::TermEntry;

const float kNaNf = std::numeric_limits<float>::quiet_NaN();
const double kNaN = std::numeric_limits<double>::quiet_NaN();
const double kInf = std::numeric_limits<double>::infinity();

auto near(double a, double b, double eps = 1e-5) -> bool {
    return std::abs(a - b) <= eps;
}

template <std::size_t N>
auto range_all(const std::array<double, N>& a, std::size_t first, std::size_t count, double value) -> bool {
    for (std::size_t i = first; i < first + count; ++i) {
        if (!near(a[i], value)) return false;
    }
    return true;
}

auto seg_near(const Tensor& v, std::size_t offset, std::initializer_list<float> want) -> bool {
    if (offset + want.size() > v.size()) return false;
    std::size_t i = offset;
    for (float w : want) {
        if (!near(v[i], w)) return false;
        ++i;
    }
    return true;
}

auto term(std::string name, int dim) -> TermEntry {
    return TermEntry{.name = std::move(name), .dim = dim, .history_length = std::nullopt, .history_interval = std::nullopt, .modifiers = {}};
}

// golden 정책 YAML 의 action_mean
const std::vector<double> kMean{-0.9377, -1.5307, 1.8549, 0.9331, 1.5307, -1.855, -2.2071, 1.5323, -1.8552, 2.202, -1.5324, 1.8552};

// 테스트용 정책.
//   관측 43:  0 command(3) | 3 dof_pos(12) | 15 dof_error(12) | 27 previous_joint_actions(12) | 39 previous_magnetic_action(4)
//   행동 16:  0 joint_actions(12) | 12 magnetic_actions(4)
auto make_spec() -> PolicySpec {
    PolicySpec spec;
    spec.observations = {term("command", 3), term("dof_pos", 12), term("dof_error", 12), term("previous_joint_actions", 12), term("previous_magnetic_action", 4)};
    spec.actions = {term("joint_actions", 12), term("magnetic_actions", 4)};
    spec.obs = quiz::ObsConfig{.command_threshold = 0.003, .leg_phase_offsets = {0.0, 0.25, 0.5, 0.75}};
    spec.action = quiz::ActionConfig{.joint_action_scale = 0.4, .action_mean = kMean, .joint_kp = 50.0, .joint_kd = 5.0};
    spec.magnet = quiz::MagnetConfig{.action_on_threshold = 0.3, .action_off_threshold = -0.3, .max_action_only = true};
    spec.command_limits.lower = {-0.1, -0.2, -0.15};
    spec.command_limits.upper = {0.3, 0.2, 0.15};
    spec.decimation = 10;
    spec.physics_dt = 0.001;
    return spec;
}

auto make_runtime() -> RuntimeConfig {
    RuntimeConfig rt;  // cooldown 은 rl_config.yaml 기본값 (foot 200 ms, global 50 ms)
    rt.arm_kp.fill(200.0);
    rt.arm_kd.fill(20.0);
    return rt;
}

// 다리 q[i] = 0.1·i, 팔 q[12 + i] = 1.0 + 0.1·i
auto make_snapshot() -> Snapshot {
    Snapshot s;
    for (std::size_t i = 0; i < 12; ++i) s.joint_q[i] = 0.1 * static_cast<double>(i);
    for (std::size_t i = 0; i < 6; ++i) s.joint_q[12 + i] = 1.0 + 0.1 * static_cast<double>(i);
    return s;
}

// 가짜 정책. 받은 관측을 기록하고, 정해 둔 행동을 그대로 내놓는다.
struct FakePolicy {
    int calls = 0;
    Tensor last_obs;
    Tensor action = Tensor(16, 0.0f);
    bool ok = true;

    auto model(int input_dim = 43, int output_dim = 16) -> Model {
        return Model{.input_dim = input_dim, .output_dim = output_dim, .forward = [this](std::span<const float> in, std::span<float> out) -> bool {
                         ++calls;
                         last_obs.assign(in.begin(), in.end());
                         for (std::size_t i = 0; i < out.size() && i < action.size(); ++i) out[i] = action[i];
                         return ok;
                     }};
    }

    // 관절 12개는 전부 joint, 자석 4개는 m0..m3
    auto set_action(float joint, float m0, float m1, float m2, float m3) -> void {
        action.assign(16, joint);
        action[12] = m0;
        action[13] = m1;
        action[14] = m2;
        action[15] = m3;
    }
};

auto foot_epm(const ControllerOutput& out) -> std::array<int, 4> {
    return {out.epm[0], out.epm[1], out.epm[2], out.epm[3]};
}

}  // namespace

int main() {
#ifdef TEST_Q1
    SECTION("Q1. 관절 명령 → MotorCommand");
    {
        // drec_baseboard_config.yaml 의 direction 패턴: 다리마다 [-1, -1, 1]
        std::array<double, 12> direction{};
        std::array<double, 12> offset{};
        for (std::size_t i = 0; i < 12; ++i) {
            direction[i] = (i % 3 == 2) ? 1.0 : -1.0;
            offset[i] = 0.5 * static_cast<double>(i);  // 0, 0.5, 1.0, ...
        }

        ControllerOutput::JointCommand cmd;
        cmd.q.fill(1.0);
        cmd.dq.fill(2.0);
        cmd.tau.fill(4.0);
        cmd.kp.fill(50.0);
        cmd.kd.fill(5.0);
        const auto m = quiz::to_motor_commands(cmd, offset, direction);

        // 축 0: direction -1, offset 0   → pos = (1 + 0)·-1
        CHECK(near(m[0].pos, -1.0) && near(m[0].vel, -2.0) && near(m[0].tau, -4.0));
        // 축 1: direction -1, offset 0.5 → pos = (1 + 0.5)·-1   (offset 을 더한 "뒤에" 방향을 곱한다)
        CHECK(near(m[1].pos, -1.5) && near(m[1].vel, -2.0) && near(m[1].tau, -4.0));
        // 축 2: direction +1, offset 1.0 → pos = (1 + 1)·1
        CHECK(near(m[2].pos, 2.0) && near(m[2].vel, 2.0) && near(m[2].tau, 4.0));
        // 게인에는 방향을 곱하지 않는다
        CHECK(near(m[0].kp, 50.0) && near(m[0].kd, 5.0));
        CHECK(near(m[11].pos, 6.5) && near(m[11].kp, 50.0));
        CHECK(m[0].mf_high_time == 0 && m[0].mf_low_time == 0 && m[0].led_rgb == 0);

        // 깨진 값은 float 로 실리기 전에 걸러진다
        ControllerOutput::JointCommand bad = cmd;
        bad.q[0] = kNaN;    // 위치가 깨졌다 → pos 0, 그리고 kp 도 0 (kd 는 유지)
        bad.dq[1] = kInf;   // 속도만 깨졌다 → vel 만 0
        bad.tau[2] = 1e39;  // float 최댓값(약 3.4e38)을 넘는다 → 0
        bad.kp[3] = -1.0;   // 음수 게인 → 0
        bad.kd[4] = kNaN;
        const auto b = quiz::to_motor_commands(bad, offset, direction);
        CHECK(b[0].pos == 0.0f && b[0].kp == 0.0f && near(b[0].kd, 5.0));
        CHECK(near(b[0].vel, -2.0));  // 같은 축의 다른 필드는 살아 있다
        CHECK(near(b[1].pos, -1.5) && b[1].vel == 0.0f && near(b[1].kp, 50.0));
        CHECK(near(b[2].pos, 2.0) && b[2].tau == 0.0f);
        CHECK(near(b[3].pos, -2.5) && b[3].kp == 0.0f && near(b[3].kd, 5.0));
        CHECK(near(b[4].kp, 50.0) && b[4].kd == 0.0f);
        CHECK(near(b[5].pos, 3.5) && near(b[5].kp, 50.0) && near(b[5].kd, 5.0));  // 멀쩡한 축은 영향이 없다
    }
#endif

#ifdef TEST_Q2
    SECTION("Q2. 추론 한 번 (NaN 가드)");
    {
        int calls = 0;
        float fill = 0.5f;
        bool ok = true;
        const Model model{.input_dim = 3, .output_dim = 2, .forward = [&](std::span<const float>, std::span<float> out) -> bool {
                              ++calls;
                              for (float& v : out) v = fill;
                              return ok;
                          }};
        const Tensor in{1.0f, 2.0f, 3.0f};
        Tensor out(2, 0.0f);

        CHECK(quiz::run_inference(model, in, out));
        CHECK(calls == 1 && near(out[0], 0.5) && near(out[1], 0.5));

        // 크기가 맞지 않으면 부르지 않는다
        Tensor short_in{1.0f, 2.0f};
        Tensor long_out(3, 0.0f);
        CHECK(!quiz::run_inference(model, short_in, out));
        CHECK(!quiz::run_inference(model, in, long_out));
        CHECK(calls == 1);

        // 입력이 깨졌으면 추론을 돌리지 않는다 (NaN 을 넣으면 NaN 이 나올 뿐이다)
        const Tensor nan_in{1.0f, kNaNf, 3.0f};
        CHECK(!quiz::run_inference(model, nan_in, out));
        CHECK(calls == 1);

        // 출력이 깨졌다
        fill = kNaNf;
        CHECK(!quiz::run_inference(model, in, out));
        CHECK(calls == 2);
        fill = std::numeric_limits<float>::infinity();
        CHECK(!quiz::run_inference(model, in, out));

        // forward 자체가 실패했다
        fill = 0.5f;
        ok = false;
        CHECK(!quiz::run_inference(model, in, out));

        // 로드되지 않은 엔진
        const Model empty{.input_dim = 3, .output_dim = 2, .forward = nullptr};
        CHECK(!quiz::run_inference(empty, in, out));
    }
#endif

#ifdef TEST_Q3
    SECTION("Q3. init — 로드 시점의 계약");
    {
        const RuntimeConfig rt = make_runtime();
        const Snapshot now = make_snapshot();
        FakePolicy policy;

        // init 전: 주기 0, step 은 전부 0 인 출력
        {
            quiz::RlPipeline p;
            CHECK(p.cycle_time_us() == 0);
            const ControllerOutput out = p.step(now);
            CHECK(range_all(out.joint.q, 0, 18, 0.0) && range_all(out.joint.kp, 0, 18, 0.0));
            CHECK(policy.calls == 0);
        }

        // 정상
        {
            quiz::RlPipeline p;
            CHECK(p.init(make_spec(), rt, policy.model(), now));
            CHECK(p.cycle_time_us() == 10000);  // decimation 10 × physics_dt 0.001
        }

        const auto ok = [&](const PolicySpec& spec, Model model) -> bool {
            quiz::RlPipeline p;
            return p.init(spec, rt, std::move(model), now);
        };

        // ONNX 와 YAML 의 차원이 다르다
        CHECK(!ok(make_spec(), policy.model(42, 16)));
        CHECK(!ok(make_spec(), policy.model(43, 15)));
        // 로드되지 않은 엔진
        CHECK(!ok(make_spec(), Model{.input_dim = 43, .output_dim = 16, .forward = nullptr}));

        // 주기를 유도할 수 없다
        PolicySpec no_decimation = make_spec();
        no_decimation.decimation = std::nullopt;
        CHECK(!ok(no_decimation, policy.model()));

        // action_mean / scale
        PolicySpec short_mean = make_spec();
        short_mean.action.action_mean.pop_back();
        CHECK(!ok(short_mean, policy.model()));
        PolicySpec no_scale = make_spec();
        no_scale.action.joint_action_scale = 0.0;
        CHECK(!ok(no_scale, policy.model()));

        // 게인
        PolicySpec no_kp = make_spec();
        no_kp.action.joint_kp = 0.0;
        CHECK(!ok(no_kp, policy.model()));

        // 중복 term
        PolicySpec dup = make_spec();
        dup.observations = {term("dof_pos", 12), term("dof_pos", 12), term("dof_error", 12), term("command", 3), term("previous_magnetic_action", 4)};
        CHECK(!ok(dup, policy.model()));

        // 차원 합은 맞지만 모르는 obs term (dim 검사만으로는 못 잡는다)
        PolicySpec unknown_obs = make_spec();
        unknown_obs.observations[0] = term("foo", 3);
        CHECK(!ok(unknown_obs, policy.model()));

        // 차원 합은 맞지만 모르는 action term
        PolicySpec unknown_act = make_spec();
        unknown_act.actions[1] = term("wheel_actions", 4);
        CHECK(!ok(unknown_act, policy.model()));

        // joint_actions 의 dim 이 로봇과 다르다 (ONNX·action_mean 까지 13 으로 맞춰도 거절)
        PolicySpec wrong_joint = make_spec();
        wrong_joint.actions[0] = term("joint_actions", 13);
        wrong_joint.action.action_mean.push_back(0.0);
        CHECK(!ok(wrong_joint, policy.model(43, 17)));

        // estimator_output: YAML 에는 있지만 ONNX 입력(43)에는 없다 → 통과하고, 관측에서도 빠진다
        PolicySpec with_est = make_spec();
        with_est.observations.insert(with_est.observations.begin() + 1, term("estimator_output", 19));
        {
            quiz::RlPipeline p;
            CHECK(p.init(with_est, rt, policy.model(), now));
            policy.calls = 0;
            (void)p.step(now);
            CHECK(policy.calls == 1 && policy.last_obs.size() == 43);
        }

        // 실패한 init 뒤에는 돌지 않는다. 다시 init 에 성공하면 돈다.
        {
            quiz::RlPipeline p;
            CHECK(!p.init(make_spec(), rt, policy.model(42, 16), now));
            CHECK(p.cycle_time_us() == 0);
            policy.calls = 0;
            const ControllerOutput out = p.step(now);
            CHECK(policy.calls == 0 && range_all(out.joint.kp, 0, 18, 0.0));
            CHECK(p.init(make_spec(), rt, policy.model(), now));
            (void)p.step(now);
            CHECK(policy.calls == 1);
        }
    }
#endif

#ifdef TEST_Q4
    SECTION("Q4. step — 첫 주기");
    {
        FakePolicy policy;
        quiz::RlPipeline p;
        CHECK(p.init(make_spec(), make_runtime(), policy.model(), make_snapshot()));

        Snapshot snap = make_snapshot();
        snap.desired_velocity = {1.0, 0.001, -0.1};  // x 는 한계(0.3) 밖, y 는 deadzone(0.003) 안
        policy.set_action(0.5f, 2.0f, -2.0f, 0.1f, 0.0f);
        const ControllerOutput out = p.step(snap);

        // ── 정책이 받은 관측 ──
        CHECK(policy.calls == 1);
        CHECK(policy.last_obs.size() == 43);
        CHECK(seg_near(policy.last_obs, 0, {0.3f, 0.0f, -0.1f}));           // command: clamp → deadzone
        CHECK(seg_near(policy.last_obs, 3, {0.0f, 0.1f, 0.2f}));            // dof_pos
        CHECK(seg_near(policy.last_obs, 12, {0.9f, 1.0f, 1.1f}));
        CHECK(seg_near(policy.last_obs, 15, {0.0f, 0.0f, 0.0f}));           // dof_error: 첫 주기는 목표 = 현재 자세
        CHECK(seg_near(policy.last_obs, 24, {0.0f, 0.0f, 0.0f}));
        CHECK(seg_near(policy.last_obs, 27, {0.0f, 0.1f, 0.2f}));           // previous_joint_actions = 현재 자세
        CHECK(seg_near(policy.last_obs, 39, {0.0f, 0.0f, 0.0f, 0.0f}));     // previous_magnetic_action

        // ── 내보낸 명령 ──
        CHECK(near(out.joint.q[0], -0.7377));    // mean + 0.4·0.5
        CHECK(near(out.joint.q[11], 2.0552));
        CHECK(range_all(out.joint.kp, 0, 12, 50.0));
        CHECK(range_all(out.joint.kd, 0, 12, 5.0));
        CHECK(near(out.joint.q[12], 1.0) && near(out.joint.q[17], 1.5));  // 팔: init 시점 자세 유지
        CHECK(range_all(out.joint.kp, 12, 6, 200.0));
        CHECK(range_all(out.joint.kd, 12, 6, 20.0));
        CHECK(range_all(out.joint.dq, 0, 18, 0.0) && range_all(out.joint.tau, 0, 18, 0.0));
        // 자석: tanh(2) = 0.964, tanh(-2) = -0.964, tanh(0.1) = 0.0997. max_action_only → 동률이면 발 0
        CHECK(foot_epm(out) == (std::array<int, 4>{1, 0, 0, 0}));
        CHECK(out.epm[4] == 0 && out.epm[7] == 0);  // 본체 자석은 정책이 건드리지 않는다

        // 팔 hold 는 "init 때" 자세다 — 그 뒤에 팔이 움직여도 목표는 따라가지 않는다
        Snapshot arm_moved = make_snapshot();
        arm_moved.joint_q[12] = 9.0;
        const ControllerOutput next = p.step(arm_moved);
        CHECK(near(next.joint.q[12], 1.0));
    }
#endif

#ifdef TEST_Q5
    SECTION("Q5. step — 주기 사이에 이어지는 값");
    {
        FakePolicy policy;
        quiz::RlPipeline p;
        CHECK(p.init(make_spec(), make_runtime(), policy.model(), make_snapshot()));
        const Snapshot snap = make_snapshot();  // 로봇은 가만히 있다고 하자 (q 불변, 명령 0)

        // 주기 1: 관절 raw 0.5, 자석 raw (2, -2, 0.1, 0)
        policy.set_action(0.5f, 2.0f, -2.0f, 0.1f, 0.0f);
        ControllerOutput out = p.step(snap);
        CHECK(foot_epm(out) == (std::array<int, 4>{1, 0, 0, 0}));

        // 주기 2: 직전에 "내보낸 명령" 이 관측으로 돌아온다
        policy.set_action(0.0f, 2.0f, -2.0f, 0.0f, 0.0f);
        out = p.step(snap);
        CHECK(policy.calls == 2);
        CHECK(seg_near(policy.last_obs, 27, {-0.7377f, -1.3307f, 2.0549f}));  // previous_joint_actions = mean + 0.2
        CHECK(seg_near(policy.last_obs, 38, {2.0552f}));
        CHECK(seg_near(policy.last_obs, 15, {0.7377f, 1.4307f, -1.8549f}));   // dof_error = q - 직전 목표 = 0.1·i - (mean + 0.2)
        CHECK(seg_near(policy.last_obs, 26, {-0.9552f}));
        // previous_magnetic_action = tanh(raw) — gate 를 통과했는지와 무관하게 정책의 "의도" 가 실린다
        CHECK(seg_near(policy.last_obs, 39, {0.9640276f, -0.9640276f, 0.0996680f, 0.0f}));
        CHECK(near(out.joint.q[0], -0.9377));  // raw 0 → mean
        // 자석: 주기 1 에서 발 0 이 스위칭했으므로 global cooldown (50 ms = 5 주기)
        CHECK(foot_epm(out) == (std::array<int, 4>{0, 0, 0, 0}));

        // 주기 3: 직전 목표가 mean 으로 바뀌었다
        out = p.step(snap);
        CHECK(seg_near(policy.last_obs, 27, {-0.9377f, -1.5307f, 1.8549f}));
        CHECK(foot_epm(out) == (std::array<int, 4>{0, 0, 0, 0}));

        // 주기 4, 5: 아직 cooldown.  주기 6: global 이 풀렸다 — 발 0 은 자기 cooldown(200 ms) 중이라 발 1 이 나간다
        CHECK(foot_epm(p.step(snap)) == (std::array<int, 4>{0, 0, 0, 0}));
        CHECK(foot_epm(p.step(snap)) == (std::array<int, 4>{0, 0, 0, 0}));
        CHECK(foot_epm(p.step(snap)) == (std::array<int, 4>{0, -1, 0, 0}));

        // ── 1 kHz tick 위에서: runner 는 1 ms 마다 깨지만 정책은 10 ms 마다 한 번만 돈다 ──
        FakePolicy slow;
        quiz::RlPipeline q;
        CHECK(q.init(make_spec(), make_runtime(), slow.model(), make_snapshot()));
        quiz::StepScheduler scheduler;
        CHECK(scheduler.start(0, q.cycle_time_us(), 1000));
        std::array<double, 12> unit{};
        unit.fill(1.0);
        const std::array<double, 12> zero{};
        slow.set_action(0.5f, 0.0f, 0.0f, 0.0f, 0.0f);
        ControllerOutput latest;
        int ticks_with_command = 0;
        for (int now_us = 0; now_us <= 25000; now_us += 1000) {
            if (scheduler.due(now_us)) latest = q.step(snap);
            // 정책이 돌지 않은 tick 에도 직전 명령이 그대로 모터로 나간다
            const auto motors = quiz::to_motor_commands(latest.joint, zero, unit);
            if (near(motors[0].pos, -0.7377) && near(motors[0].kp, 50.0)) ++ticks_with_command;
        }
        CHECK(slow.calls == 3);             // t = 0, 10, 20 ms
        CHECK(ticks_with_command == 26);    // 26 tick 전부
    }
#endif

#ifdef TEST_Q6
    SECTION("Q6. step — 추론 실패와 safe-hold");
    {
        FakePolicy policy;
        quiz::RlPipeline p;
        CHECK(p.init(make_spec(), make_runtime(), policy.model(), make_snapshot()));
        const Snapshot snap = make_snapshot();

        // 주기 1: 정상. 목표 = mean + 0.2, 발 0 자석 스위칭
        policy.set_action(0.5f, 2.0f, 0.0f, 0.0f, 0.0f);
        ControllerOutput out = p.step(snap);
        CHECK(near(out.joint.q[0], -0.7377) && foot_epm(out) == (std::array<int, 4>{1, 0, 0, 0}));

        const auto is_safe_hold = [](const ControllerOutput& o) -> bool {
            return near(o.joint.q[0], -0.7377) && near(o.joint.q[11], 2.0552)       // 직전 정상 목표 유지
                   && range_all(o.joint.kp, 0, 12, 50.0) && range_all(o.joint.kd, 0, 12, 5.0)  // 게인도 유지 (0 이면 주저앉는다)
                   && near(o.joint.q[12], 1.0) && range_all(o.joint.kp, 12, 6, 200.0)          // 팔 hold
                   && o.epm == std::array<int, 8>{};                                           // 자석은 건드리지 않는다
        };

        // 주기 2: 정책 출력에 NaN — 자석 쪽은 멀쩡해 보여도 출력 전체를 버린다
        policy.set_action(-3.0f, 0.0f, -2.0f, 0.0f, 0.0f);
        policy.action[3] = kNaNf;
        out = p.step(snap);
        CHECK(policy.calls == 2);
        CHECK(is_safe_hold(out));

        // 주기 3: 추론 자체가 실패
        policy.set_action(-3.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        policy.ok = false;
        out = p.step(snap);
        CHECK(policy.calls == 3);
        CHECK(is_safe_hold(out));
        policy.ok = true;

        // 주기 4: 센서 값이 NaN → 관측이 깨졌으므로 추론을 부르지도 않는다
        Snapshot broken = snap;
        broken.joint_q[0] = kNaN;
        out = p.step(broken);
        CHECK(policy.calls == 3);
        CHECK(is_safe_hold(out));

        // 주기 5: 복귀. safe-hold 동안 last_target 은 갱신되지 않았다 → 관측의 직전 목표는 주기 1 의 값
        policy.set_action(0.0f, 0.0f, -2.0f, 0.0f, 0.0f);
        out = p.step(snap);
        CHECK(policy.calls == 4);
        CHECK(seg_near(policy.last_obs, 27, {-0.7377f, -1.3307f, 2.0549f}));
        CHECK(seg_near(policy.last_obs, 39, {0.9640276f, 0.0f, 0.0f, 0.0f}));  // 주기 1 의 tanh(2)
        CHECK(near(out.joint.q[0], -0.9377));

        // 실패한 주기에는 gate 가 돌지 않는다 → cooldown 은 "성공한 주기" 로만 줄어든다.
        //   주기 1 에서 global = 5. 주기 2 ~ 4 는 실패. 주기 5 → 4, 6 → 3, 7 → 2, 8 → 1, 9 → 0 (발 1 통과)
        CHECK(foot_epm(out) == (std::array<int, 4>{0, 0, 0, 0}));              // 주기 5
        CHECK(foot_epm(p.step(snap)) == (std::array<int, 4>{0, 0, 0, 0}));     // 6
        CHECK(foot_epm(p.step(snap)) == (std::array<int, 4>{0, 0, 0, 0}));     // 7
        CHECK(foot_epm(p.step(snap)) == (std::array<int, 4>{0, 0, 0, 0}));     // 8
        CHECK(foot_epm(p.step(snap)) == (std::array<int, 4>{0, -1, 0, 0}));    // 9

        // ── 필터가 있는 term 은 NaN 을 흡수한다 ──
        //   같은 gyro NaN 이라도: 필터 없는 imu_ang_vel → 관측이 깨져 safe-hold,
        //                        EMA 가 걸린 imu_ang_vel → 필터가 직전 값을 내보내 추론이 그대로 돈다
        const auto run_with_gyro_nan = [&](bool with_filter) -> int {
            PolicySpec spec = make_spec();
            TermEntry gyro = term("imu_ang_vel", 3);
            if (with_filter) gyro.modifiers.push_back(quiz::ModifierSpec{.type = quiz::ModifierSpec::Type::kEma, .alpha = 0.5});
            spec.observations.push_back(gyro);  // 관측 46
            FakePolicy fake;
            quiz::RlPipeline pipe;
            if (!pipe.init(spec, make_runtime(), fake.model(46, 16), make_snapshot())) return -1;
            Snapshot s = make_snapshot();
            s.gyro = {1.0, 2.0, 3.0};
            (void)pipe.step(s);
            s.gyro = {kNaN, 2.0, 3.0};
            (void)pipe.step(s);
            return fake.calls;
        };
        CHECK(run_with_gyro_nan(false) == 1);
        CHECK(run_with_gyro_nan(true) == 2);
    }
#endif

    return report();
}
