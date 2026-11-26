// 수정 금지 — quiz.h 를 고쳐서 통과시킬 것.
#include <cmath>
#include <initializer_list>
#include <vector>

#include "check.h"
#include "quiz.h"

namespace {

using quiz::ModifierSpec;
using quiz::ObsConfig;
using quiz::ObsInputs;
using quiz::Tensor;
using quiz::TermEntry;

constexpr float kSentinel = -999.0f;  // "아무도 안 썼다" 표시

auto near(float a, float b, float eps = 1e-5f) -> bool {
    return std::abs(a - b) <= eps;
}

// out[offset, offset + want.size()) 가 want 와 같은가
auto seg_near(const Tensor& out, std::size_t offset, std::initializer_list<float> want) -> bool {
    if (offset + want.size() > out.size()) return false;
    std::size_t i = offset;
    for (float w : want) {
        if (!near(out[i], w)) return false;
        ++i;
    }
    return true;
}

// out[offset, offset + count) 가 전부 value 인가
auto seg_all(const Tensor& out, std::size_t offset, std::size_t count, float value) -> bool {
    if (offset + count > out.size()) return false;
    for (std::size_t i = offset; i < offset + count; ++i) {
        if (!near(out[i], value)) return false;
    }
    return true;
}

auto ema(double alpha) -> ModifierSpec {
    return ModifierSpec{.type = ModifierSpec::Type::kEma, .alpha = alpha};
}

auto term(std::string name, int dim) -> TermEntry {
    return TermEntry{.name = std::move(name), .dim = dim, .history_length = std::nullopt, .history_interval = std::nullopt, .modifiers = {}};
}

auto history(std::string name, int dim, int length, int interval) -> TermEntry {
    return TermEntry{.name = std::move(name), .dim = dim, .history_length = length, .history_interval = interval, .modifiers = {}};
}

auto filtered(TermEntry entry, double alpha) -> TermEntry {
    entry.modifiers.push_back(ema(alpha));
    return entry;
}

// 관측 벡터를 만들어 돌려준다. 크기는 테스트가 정한다 (stub 의 total_dim() == 0 이어도 죽지 않게).
auto build(const quiz::ObservationManager& mgr, std::size_t size) -> Tensor {
    Tensor out(size, kSentinel);
    mgr.build_actor_input(out);
    return out;
}

template <std::size_t N>
auto filled(double value) -> std::array<double, N> {
    std::array<double, N> a{};
    a.fill(value);
    return a;
}

}  // namespace

int main() {
#ifdef TEST_Q1
    SECTION("Q1. term 이름 해석");
    {
        CHECK(quiz::strip_history_suffix("dof_vel_history") == "dof_vel");
        CHECK(quiz::strip_history_suffix("dof_vel") == "dof_vel");
        CHECK(quiz::strip_history_suffix("previous_joint_actions_history") == "previous_joint_actions");
        CHECK(quiz::strip_history_suffix("_history") == "_history");                    // 떼면 빈 이름 → 떼지 않는다
        CHECK(quiz::strip_history_suffix("history_of_dof_vel") == "history_of_dof_vel");  // 끝에 붙은 것만
        CHECK(quiz::strip_history_suffix("x_history_history") == "x_history");          // 한 번만

        CHECK(quiz::base_term_dim("imu_ang_vel") == 3);
        CHECK(quiz::base_term_dim("projected_gravity") == 3);
        CHECK(quiz::base_term_dim("command") == 3);
        CHECK(quiz::base_term_dim("dof_pos") == 12);
        CHECK(quiz::base_term_dim("dof_vel") == 12);
        CHECK(quiz::base_term_dim("dof_error") == 12);
        CHECK(quiz::base_term_dim("target_dof_pos") == 12);
        CHECK(quiz::base_term_dim("previous_joint_actions") == 12);
        CHECK(quiz::base_term_dim("previous_magnetic_action") == 4);
        CHECK(quiz::base_term_dim("gait_phi") == 8);
        CHECK(quiz::base_term_dim("dof_vel_history") == -1);  // 접미사를 떼는 것은 호출부의 일
        CHECK(quiz::base_term_dim("prev_magnetic_action") == -1);  // 비슷한 이름은 다른 이름이다
        CHECK(quiz::base_term_dim("") == -1);
    }
#endif

#ifdef TEST_Q2
    SECTION("Q2. term 하나의 값");
    {
        ObsInputs in;
        in.gyro = {0.1, -0.2, 0.3};
        in.base_quat = quiz::Quat{std::sqrt(0.5), std::sqrt(0.5), 0.0, 0.0};  // roll +90°
        for (std::size_t i = 0; i < 12; ++i) {
            in.joint_q[i] = 0.1 * static_cast<double>(i);    // 0.0, 0.1, ..., 1.1
            in.joint_dq[i] = -1.0 * static_cast<double>(i);  // 0, -1, ..., -11
        }
        in.last_target_dof_pos = filled<12>(1.0);
        in.command = {0.5, 0.001, -0.2};
        in.last_magnetic_action = {0.9f, -0.9f, 0.1f, 0.0f};
        in.gait_phase_ratio = 0.25;
        const ObsConfig cfg{.command_threshold = 0.003, .leg_phase_offsets = {0.0, 0.5, 0.5, 0.0}};

        const auto fill = [&](std::string_view name, std::size_t size) -> Tensor {
            Tensor out(size, kSentinel);
            (void)quiz::fill_base_term(name, in, cfg, out);
            return out;
        };

        CHECK(seg_near(fill("imu_ang_vel", 3), 0, {0.1f, -0.2f, 0.3f}));
        CHECK(seg_near(fill("projected_gravity", 3), 0, {0.0f, -1.0f, 0.0f}));
        CHECK(seg_near(fill("command", 3), 0, {0.5f, 0.0f, -0.2f}));  // 0.001 은 deadzone
        CHECK(seg_near(fill("dof_pos", 12), 0, {0.0f, 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f, 0.9f, 1.0f, 1.1f}));
        CHECK(seg_near(fill("dof_vel", 12), 0, {0, -1, -2, -3, -4, -5, -6, -7, -8, -9, -10, -11}));
        // dof_error = 현재 - 목표
        CHECK(seg_near(fill("dof_error", 12), 0, {-1.0f, -0.9f, -0.8f, -0.7f, -0.6f, -0.5f, -0.4f, -0.3f, -0.2f, -0.1f, 0.0f, 0.1f}));
        CHECK(seg_all(fill("target_dof_pos", 12), 0, 12, 1.0f));
        CHECK(seg_all(fill("previous_joint_actions", 12), 0, 12, 1.0f));
        CHECK(seg_near(fill("previous_magnetic_action", 4), 0, {0.9f, -0.9f, 0.1f, 0.0f}));
        CHECK(seg_near(fill("gait_phi", 8), 0, {0, 1, 0, -1, 0, -1, 0, 1}));

        Tensor ok(3, kSentinel);
        CHECK(quiz::fill_base_term("imu_ang_vel", in, cfg, ok));

        // 모르는 이름 / 크기 불일치 → false, out 은 건드리지 않는다
        Tensor untouched(3, kSentinel);
        CHECK(!quiz::fill_base_term("foo", in, cfg, untouched));
        CHECK(seg_all(untouched, 0, 3, kSentinel));
        Tensor wrong_size(4, kSentinel);
        CHECK(!quiz::fill_base_term("imu_ang_vel", in, cfg, wrong_size));
        CHECK(seg_all(wrong_size, 0, 4, kSentinel));
    }
#endif

#ifdef TEST_Q3
    SECTION("Q3. init — 포장 방식과 차원 검증");
    {
        const ObsInputs in;
        const ObsConfig cfg;

        // DidenWorld golden 정책(seam3_policy_config.golden.yaml)의 observations 에서
        // 퀴즈가 다루지 않는 relative_foot_pos 한 줄만 뺀 것.
        const std::vector<TermEntry> golden{
            filtered(term("imu_ang_vel", 3), 0.5),
            filtered(term("projected_gravity", 3), 0.5),
            filtered(term("dof_pos", 12), 0.5),
            filtered(term("dof_vel", 12), 0.5),
            term("gait_phi", 8),
            filtered(history("dof_vel_history", 36, 3, 4), 0.5),
            filtered(history("dof_error_history", 36, 3, 4), 0.5),
            filtered(history("previous_joint_actions", 24, 2, 1), 0.5),
            filtered(history("previous_magnetic_action", 8, 2, 1), 0.5),
            term("command", 3),
        };
        quiz::ObservationManager mgr;
        CHECK(mgr.init(golden, cfg, in));
        CHECK(mgr.total_dim() == 145);  // 3+3+12+12+8+36+36+24+8+3

        const auto ok = [&](const std::vector<TermEntry>& terms) -> bool {
            quiz::ObservationManager m;
            return m.init(terms, cfg, in);
        };

        CHECK(ok({}));                                         // term 이 없어도 실패는 아니다 (dim 0)
        CHECK(!ok({term("dof_pos", 12), term("foo", 3)}));    // 모르는 term
        CHECK(!ok({term("dof_pos", 11)}));                     // YAML dim 이 term 과 다르다
        CHECK(!ok({term("imu_ang_vel", 0)}));

        // (나) history_length 가 있는 term
        CHECK(ok({history("dof_vel_history", 36, 3, 4)}));
        CHECK(ok({history("dof_vel", 36, 3, 4)}));             // 접미사가 없어도 history_length 가 있으면 history
        CHECK(!ok({history("dof_vel_history", 36, 2, 4)}));    // 36 != 12·2
        CHECK(!ok({history("dof_vel_history", 0, 0, 1)}));     // length 는 양수
        CHECK(!ok({history("dof_vel_history", 36, 3, 0)}));    // interval 도 양수
        TermEntry no_interval = history("dof_vel_history", 24, 2, 1);
        no_interval.history_interval = std::nullopt;           // interval 누락 = 1
        CHECK(ok({no_interval}));
        CHECK(!ok({term("dof_vel_history", 36)}));             // 접미사만 있고 history_length 가 없으면 base dim(12) 과 비교된다
        CHECK(ok({term("dof_vel_history", 12)}));

        // (가) previous_joint_actions: history_length 없이 dim 으로 깊이를 준다
        CHECK(ok({term("previous_joint_actions", 12)}));
        CHECK(ok({term("previous_joint_actions", 24)}));
        CHECK(ok({term("previous_joint_actions", 36)}));
        CHECK(!ok({term("previous_joint_actions", 30)}));      // 12 의 배수가 아니다
        CHECK(!ok({term("previous_joint_actions", 0)}));
        CHECK(!ok({term("target_dof_pos", 24)}));              // 같은 값을 내는 term 이지만 이 규칙은 이름에 걸려 있다

        // (다) 필터만 건 term: dim 은 base 그대로
        CHECK(ok({filtered(term("imu_ang_vel", 3), 0.5)}));
        CHECK(!ok({filtered(term("imu_ang_vel", 6), 0.5)}));

        // init 을 다시 부르면 처음부터 다시 만든다 (누적되지 않는다)
        quiz::ObservationManager again;
        CHECK(again.init({term("dof_pos", 12), term("command", 3)}, cfg, in));
        CHECK(again.total_dim() == 15);
        CHECK(again.init({term("command", 3)}, cfg, in));
        CHECK(again.total_dim() == 3);
    }
#endif

#ifdef TEST_Q4
    SECTION("Q4. YAML 순서 = 벡터 배치");
    {
        ObsInputs in;
        in.gyro = {0.1, -0.2, 0.3};
        in.base_quat = quiz::Quat{std::sqrt(0.5), 0.0, std::sqrt(0.5), 0.0};  // pitch +90° → 중력이 body +x
        for (std::size_t i = 0; i < 12; ++i) in.joint_q[i] = 0.1 * static_cast<double>(i);
        in.last_target_dof_pos = filled<12>(1.0);
        in.command = {0.5, 0.001, -0.2};
        in.last_magnetic_action = {0.9f, -0.9f, 0.1f, 0.0f};
        const ObsConfig cfg{.command_threshold = 0.003, .leg_phase_offsets = {0.0, 0.25, 0.5, 0.75}};

        // offset:  0          3         15             18          30                          34
        //          command(3) dof_pos(12) imu_ang_vel(3) dof_error(12) previous_magnetic_action(4)
        quiz::ObservationManager mgr;
        CHECK(mgr.init({term("command", 3), term("dof_pos", 12), term("imu_ang_vel", 3), term("dof_error", 12), term("previous_magnetic_action", 4)}, cfg, in));
        CHECK(mgr.total_dim() == 34);
        mgr.tick();
        Tensor out = build(mgr, 34);
        CHECK(seg_near(out, 0, {0.5f, 0.0f, -0.2f}));
        CHECK(seg_near(out, 3, {0.0f, 0.1f, 0.2f}));       // dof_pos 의 앞 3개
        CHECK(seg_near(out, 12, {0.9f, 1.0f, 1.1f}));      // dof_pos 의 뒤 3개
        CHECK(seg_near(out, 15, {0.1f, -0.2f, 0.3f}));
        CHECK(seg_near(out, 18, {-1.0f, -0.9f, -0.8f}));   // dof_error 의 앞 3개
        CHECK(seg_near(out, 30, {0.9f, -0.9f, 0.1f, 0.0f}));

        // 같은 term 을 순서만 바꾸면 벡터도 그 순서로 바뀐다
        quiz::ObservationManager swapped;
        CHECK(swapped.init({term("previous_magnetic_action", 4), term("imu_ang_vel", 3), term("command", 3)}, cfg, in));
        swapped.tick();
        out = build(swapped, 10);
        CHECK(seg_near(out, 0, {0.9f, -0.9f, 0.1f, 0.0f}));
        CHECK(seg_near(out, 4, {0.1f, -0.2f, 0.3f}));
        CHECK(seg_near(out, 7, {0.5f, 0.0f, -0.2f}));

        // 01 의 수식이 그대로 들어온다
        quiz::ObservationManager math;
        CHECK(math.init({term("projected_gravity", 3), term("gait_phi", 8)}, cfg, in));
        math.tick();
        out = build(math, 11);
        CHECK(seg_near(out, 0, {1.0f, 0.0f, 0.0f}));
        CHECK(seg_near(out, 3, {1, 0, 0, 1, -1, 0, 0, -1}));

        // inputs 는 참조다: init 뒤에 값이 바뀌면 다음 build 가 새 값을 본다
        in.gyro = {7.0, 8.0, 9.0};
        in.command = {0.0, 0.0, 0.1};
        mgr.tick();
        out = build(mgr, 34);
        CHECK(seg_near(out, 0, {0.0f, 0.0f, 0.1f}));
        CHECK(seg_near(out, 15, {7.0f, 8.0f, 9.0f}));

        // build 는 자기 구간만 쓴다 — out 이 더 길어도 뒤쪽은 건드리지 않는다
        Tensor longer(40, kSentinel);
        mgr.build_actor_input(std::span<float>(longer).first(34));
        CHECK(seg_near(longer, 30, {0.9f, -0.9f, 0.1f, 0.0f}));
        CHECK(seg_all(longer, 34, 6, kSentinel));
    }
#endif

#ifdef TEST_Q5
    SECTION("Q5. history · 필터 term");
    {
        ObsInputs in;
        const ObsConfig cfg;

        // (나) oldest-first: [과거, 현재]
        quiz::ObservationManager hist;
        CHECK(hist.init({history("dof_pos_history", 24, 2, 1)}, cfg, in));
        in.joint_q = filled<12>(1.0);
        hist.tick();
        Tensor out = build(hist, 24);
        CHECK(seg_all(out, 0, 12, 0.0f));   // 첫 tick: 과거 칸은 0 (이 커밋에는 warm start 가 없다)
        CHECK(seg_all(out, 12, 12, 1.0f));
        in.joint_q = filled<12>(2.0);
        hist.tick();
        out = build(hist, 24);
        CHECK(seg_all(out, 0, 12, 1.0f));
        CHECK(seg_all(out, 12, 12, 2.0f));

        // tick 없이 build 만 다시 부르면 값은 그대로다 (history 는 tick 에서만 전진한다)
        in.joint_q = filled<12>(3.0);
        out = build(hist, 24);
        CHECK(seg_all(out, 0, 12, 1.0f));
        CHECK(seg_all(out, 12, 12, 2.0f));

        // (가) previous_joint_actions, history_length 없음 → newest-first: [직전, 그 전]
        quiz::ObservationManager newest;
        CHECK(newest.init({term("previous_joint_actions", 24)}, cfg, in));
        in.last_target_dof_pos = filled<12>(1.0);
        newest.tick();
        in.last_target_dof_pos = filled<12>(2.0);
        newest.tick();
        out = build(newest, 24);
        CHECK(seg_all(out, 0, 12, 2.0f));
        CHECK(seg_all(out, 12, 12, 1.0f));

        // 같은 이름이라도 history_length 를 적으면 (나) 로 빠진다 → oldest-first.
        // DidenWorld golden 정책이 바로 이 표기다 (history_length: 2).
        quiz::ObservationManager golden_style;
        CHECK(golden_style.init({history("previous_joint_actions", 24, 2, 1)}, cfg, in));
        in.last_target_dof_pos = filled<12>(1.0);
        golden_style.tick();
        in.last_target_dof_pos = filled<12>(2.0);
        golden_style.tick();
        out = build(golden_style, 24);
        CHECK(seg_all(out, 0, 12, 1.0f));
        CHECK(seg_all(out, 12, 12, 2.0f));

        // (다) 필터만 건 term: EMA 0.5
        quiz::ObservationManager ema_only;
        CHECK(ema_only.init({filtered(term("imu_ang_vel", 3), 0.5)}, cfg, in));
        in.gyro = {4.0, -8.0, 0.0};
        ema_only.tick();
        CHECK(seg_near(build(ema_only, 3), 0, {4.0f, -8.0f, 0.0f}));
        in.gyro = {0.0, 0.0, 0.0};
        ema_only.tick();
        CHECK(seg_near(build(ema_only, 3), 0, {2.0f, -4.0f, 0.0f}));

        // 간격이 있는 history + 포장 없는 term 이 섞인 배치
        //   offset: 0 command(3) | 3 dof_vel_history(36 = 12 × 3, 간격 4) | 39
        quiz::ObservationManager mixed;
        CHECK(mixed.init({term("command", 3), history("dof_vel_history", 36, 3, 4)}, ObsConfig{.command_threshold = 0.003, .leg_phase_offsets = {}}, in));
        in.command = {0.3, 0.0, 0.0};
        for (int k = 1; k <= 9; ++k) {
            in.joint_dq = filled<12>(static_cast<double>(k));
            mixed.tick();
        }
        out = build(mixed, 39);
        CHECK(seg_near(out, 0, {0.3f, 0.0f, 0.0f}));
        CHECK(seg_all(out, 3, 12, 1.0f));   // 8 tick 전
        CHECK(seg_all(out, 15, 12, 5.0f));  // 4 tick 전
        CHECK(seg_all(out, 27, 12, 9.0f));  // 지금

        // 필터 + history: history 에 쌓이는 것은 필터를 거친 값
        quiz::ObservationManager both;
        CHECK(both.init({filtered(history("previous_magnetic_action", 8, 2, 1), 0.5)}, cfg, in));
        in.last_magnetic_action = {1.0f, -1.0f, 0.0f, 0.0f};
        both.tick();
        in.last_magnetic_action = {0.0f, 0.0f, 0.0f, 0.0f};
        both.tick();
        out = build(both, 8);
        CHECK(seg_near(out, 0, {1.0f, -1.0f, 0.0f, 0.0f}));
        CHECK(seg_near(out, 4, {0.5f, -0.5f, 0.0f, 0.0f}));
    }
#endif

    return report();
}
