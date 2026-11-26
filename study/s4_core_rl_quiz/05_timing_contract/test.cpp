// 수정 금지 — quiz.h 를 고쳐서 통과시킬 것.
#include <optional>
#include <vector>

#include "check.h"
#include "quiz.h"

namespace {

using quiz::DimCheck;
using quiz::TermEntry;

auto term(std::string name, int dim) -> TermEntry {
    return TermEntry{.name = std::move(name), .dim = dim, .history_length = std::nullopt, .history_interval = std::nullopt, .modifiers = {}};
}

// tick_us 간격으로 [0, until_us] 를 돌면서 due() 가 true 였던 시각을 모은다.
auto due_times(quiz::StepScheduler& s, int tick_us, int until_us) -> std::vector<int> {
    std::vector<int> fired;
    for (int now = 0; now <= until_us; now += tick_us) {
        if (s.due(now)) fired.push_back(now);
    }
    return fired;
}

}  // namespace

int main() {
#ifdef TEST_Q1
    SECTION("Q1. 정책 주기 유도");
    {
        // golden 정책: decimation 10, physics_dt 0.001 → 10 ms = 100 Hz
        CHECK(quiz::derive_cycle_time_us(10, 0.001) == std::optional<int>(10000));
        CHECK(quiz::derive_cycle_time_us(4, 0.005) == std::optional<int>(20000));   // 50 Hz
        CHECK(quiz::derive_cycle_time_us(1, 0.001) == std::optional<int>(1000));    // 1 kHz

        // 둘 중 하나라도 없으면 유도할 수 없다 — 임의의 기본 주기로 떨어지지 않는다
        CHECK(!quiz::derive_cycle_time_us(std::nullopt, 0.001).has_value());
        CHECK(!quiz::derive_cycle_time_us(10, std::nullopt).has_value());
        CHECK(!quiz::derive_cycle_time_us(std::nullopt, std::nullopt).has_value());

        // 정책 YAML 이 control.cycle_time_us 를 직접 적었다 → 값이 맞든 틀리든 거절
        CHECK(!quiz::derive_cycle_time_us(10, 0.001, 10000).has_value());
        CHECK(!quiz::derive_cycle_time_us(10, 0.001, 5000).has_value());

        // 0 이하의 주기는 성립하지 않는다
        CHECK(!quiz::derive_cycle_time_us(0, 0.001).has_value());
        CHECK(!quiz::derive_cycle_time_us(10, 0.0).has_value());
        CHECK(!quiz::derive_cycle_time_us(10, -0.001).has_value());

        // static_cast<int> 는 버림이다: 0.0003 × 10 × 1e6 = 2999.9999999999995 → 2999
        // (반올림이면 3000. 레포의 실제 동작이 버림이라 그대로 맞춘다 — GUIDE.md Q1 참고)
        CHECK(quiz::derive_cycle_time_us(10, 0.0003) == std::optional<int>(2999));
        CHECK(quiz::derive_cycle_time_us(5, 0.0006) == std::optional<int>(2999));
    }
#endif

#ifdef TEST_Q2
    SECTION("Q2. StepScheduler — tick 위에서 주기 맞추기");
    {
        // 1 kHz tick, 100 Hz 정책: 10 tick 에 한 번
        quiz::StepScheduler s;
        CHECK(s.start(0, 10000, 1000));
        CHECK(due_times(s, 1000, 25000) == (std::vector<int>{0, 10000, 20000}));

        // 활성화 시각이 0 이 아니어도 첫 step 은 그 tick 에 바로 돈다
        quiz::StepScheduler late_start;
        CHECK(late_start.start(7000, 10000, 1000));
        CHECK(!late_start.due(6000));   // (활성화 전 시각 — 아직 차례가 아니다)
        CHECK(late_start.due(7000));
        CHECK(!late_start.due(8000));
        CHECK(!late_start.due(16000));
        CHECK(late_start.due(17000));

        // 주기 == tick: 매 tick
        quiz::StepScheduler every;
        CHECK(every.start(0, 1000, 1000));
        CHECK(due_times(every, 1000, 3000) == (std::vector<int>{0, 1000, 2000, 3000}));

        // tick 보다 빠른 주기는 지킬 수 없다 → 활성화 거절
        quiz::StepScheduler too_fast;
        CHECK(!too_fast.start(0, 500, 1000));
        CHECK(!too_fast.start(0, 999, 1000));

        // 50 Hz 정책
        quiz::StepScheduler slow;
        CHECK(slow.start(0, 20000, 1000));
        CHECK(due_times(slow, 1000, 45000) == (std::vector<int>{0, 20000, 40000}));

        // start 를 다시 부르면 그 시각부터 새로 센다 (컨트롤러 교체)
        CHECK(slow.start(50000, 10000, 1000));
        CHECK(slow.due(50000));
        CHECK(!slow.due(59000));
        CHECK(slow.due(60000));
    }
#endif

#ifdef TEST_Q3
    SECTION("Q3. StepScheduler — 지각과 어긋난 주기");
    {
        // 조금 늦은 tick: 돌리되, 목표 시각은 원래 격자(10000 의 배수)에 둔다 → 장기 주기가 유지된다
        quiz::StepScheduler jitter;
        CHECK(jitter.start(0, 10000, 1000));
        CHECK(jitter.due(0));
        CHECK(jitter.due(10300));    // 300 µs 지각
        CHECK(!jitter.due(19900));   // 다음 목표는 20300 이 아니라 20000 이다
        CHECK(jitter.due(20000));

        // 한 주기 넘게 밀렸으면 따라잡기를 포기하고 "지금 + 주기" 부터 다시 센다
        quiz::StepScheduler stalled;
        CHECK(stalled.start(0, 10000, 1000));
        CHECK(stalled.due(0));
        CHECK(stalled.due(35000));   // 목표 10000 → 20000 인데 이미 지났다 → 45000
        CHECK(!stalled.due(36000));  // 놓친 20000, 30000 을 몰아서 돌리지 않는다
        CHECK(!stalled.due(44000));
        CHECK(stalled.due(45000));

        // 정확히 한 주기 늦은 것은 "넘게" 가 아니다 → 한 번은 따라잡는다
        quiz::StepScheduler edge;
        CHECK(edge.start(0, 10000, 1000));
        CHECK(edge.due(0));
        CHECK(edge.due(20000));      // 목표 10000 → 20000 (지금과 같다 — 유지)
        CHECK(edge.due(21000));      // 그래서 바로 다음 tick 에 또 돈다
        CHECK(!edge.due(22000));
        CHECK(edge.due(30000));

        // 주기가 tick 의 배수가 아닐 때 (2500 µs): 간격은 3, 2, 3, 2 tick 으로 흔들리지만 평균은 맞는다
        quiz::StepScheduler odd;
        CHECK(odd.start(0, 2500, 1000));
        CHECK(due_times(odd, 1000, 10000) == (std::vector<int>{0, 3000, 5000, 8000, 10000}));

        // Q1 의 버림이 만든 2999 µs: 3 tick 간격으로 돈다 (2999 든 3000 이든 1 kHz tick 위에서는 같다)
        quiz::StepScheduler truncated;
        CHECK(truncated.start(0, 2999, 1000));
        CHECK(due_times(truncated, 1000, 9000) == (std::vector<int>{0, 3000, 6000, 9000}));
    }
#endif

#ifdef TEST_Q4
    SECTION("Q4. 차원 계약");
    {
        // golden 정책의 모양: 관측 합 157, 행동 12 + 4
        const std::vector<TermEntry> obs{term("imu_ang_vel", 3), term("projected_gravity", 3), term("dof_pos", 12), term("dof_vel", 12), term("relative_foot_pos", 12),
                                         term("gait_phi", 8),    term("dof_vel_history", 36),  term("dof_error_history", 36), term("previous_joint_actions", 24),
                                         term("previous_magnetic_action", 8), term("command", 3)};
        const std::vector<TermEntry> act{term("joint_actions", 12), term("magnetic_actions", 4)};

        CHECK(quiz::validate_dims(obs, act, 157, 16, 12, 0.4) == DimCheck::kOk);
        CHECK(quiz::validate_dims(obs, act, 145, 16, 12, 0.4) == DimCheck::kInputDimMismatch);   // YAML 에서 term 하나가 빠진 ONNX
        CHECK(quiz::validate_dims(obs, act, 157, 12, 12, 0.4) == DimCheck::kOutputDimMismatch);  // 자석 없는 ONNX
        CHECK(quiz::validate_dims(obs, act, 157, 16, 13, 0.4) == DimCheck::kActionMeanSizeMismatch);
        CHECK(quiz::validate_dims(obs, act, 157, 16, 0, 0.4) == DimCheck::kActionMeanSizeMismatch);  // action_mean 누락
        CHECK(quiz::validate_dims(obs, act, 157, 16, 12, 0.0) == DimCheck::kActionScaleInvalid);    // scale 누락(기본 0)
        CHECK(quiz::validate_dims(obs, act, 157, 16, 12, -0.4) == DimCheck::kActionScaleInvalid);

        // 여러 개가 어긋나면 먼저 검사하는 것이 나온다
        CHECK(quiz::validate_dims(obs, act, 1, 1, 0, 0.0) == DimCheck::kInputDimMismatch);
        CHECK(quiz::validate_dims(obs, act, 157, 1, 0, 0.0) == DimCheck::kOutputDimMismatch);
        CHECK(quiz::validate_dims(obs, act, 157, 16, 0, 0.0) == DimCheck::kActionMeanSizeMismatch);

        // estimator_output: YAML 목록에는 있지만 ONNX 입력에는 없다 (ONNX 안에서 만들어진다)
        std::vector<TermEntry> with_est = obs;
        with_est.insert(with_est.begin() + 2, term("estimator_output", 19));
        CHECK(quiz::validate_dims(with_est, act, 157, 16, 12, 0.4) == DimCheck::kOk);            // 176 - 19
        CHECK(quiz::validate_dims(with_est, act, 176, 16, 12, 0.4) == DimCheck::kInputDimMismatch);

        // joint_actions 가 없는 정책은 action_mean / scale 을 보지 않는다
        const std::vector<TermEntry> magnet_only{term("magnetic_actions", 4)};
        CHECK(quiz::validate_dims(obs, magnet_only, 157, 4, 0, 0.0) == DimCheck::kOk);

        // 검증이 끝나면 estimator_output 을 목록에서 지운다 — 나머지 순서는 그대로
        quiz::remove_estimator_output(with_est);
        CHECK(with_est.size() == 11);
        CHECK(with_est.size() == 11 && with_est[1].name == "projected_gravity" && with_est[2].name == "dof_pos");
        std::vector<TermEntry> none = obs;
        quiz::remove_estimator_output(none);  // 없으면 아무 일도 없다
        CHECK(none.size() == 11);
        std::vector<TermEntry> only{term("estimator_output", 19)};
        quiz::remove_estimator_output(only);
        CHECK(only.empty());
    }
#endif

#ifdef TEST_Q5
    SECTION("Q5. 설정 정합성");
    {
        CHECK(quiz::validate_term_list({term("dof_pos", 12), term("dof_vel", 12), term("command", 3)}));
        CHECK(quiz::validate_term_list({}));
        CHECK(quiz::validate_term_list({term("dof_pos", 0)}));                          // dim 0 은 여기서는 통과 (다른 검사가 잡는다)
        CHECK(!quiz::validate_term_list({term("dof_pos", 12), term("dof_pos", 12)}));   // 중복
        CHECK(!quiz::validate_term_list({term("dof_pos", 12), term("command", 3), term("dof_pos", 24)}));
        CHECK(!quiz::validate_term_list({term("dof_pos", -1)}));                        // 음수 dim
        CHECK(quiz::validate_term_list({term("dof_vel", 12), term("dof_vel_history", 36)}));  // 이름이 다르면 다른 term

        CHECK(quiz::validate_control_gains(50.0, 5.0));
        CHECK(quiz::validate_control_gains(50.0, 0.0));    // kd 0 은 허용
        CHECK(!quiz::validate_control_gains(0.0, 5.0));    // kp 0 은 거절
        CHECK(!quiz::validate_control_gains(-1.0, 5.0));
        CHECK(!quiz::validate_control_gains(50.0, -0.1));

        CHECK(quiz::common_cycle_time_us(std::vector<int>{10000}) == std::optional<int>(10000));
        CHECK(quiz::common_cycle_time_us(std::vector<int>{10000, 10000, 10000}) == std::optional<int>(10000));
        CHECK(!quiz::common_cycle_time_us(std::vector<int>{10000, 20000}).has_value());
        CHECK(!quiz::common_cycle_time_us(std::vector<int>{10000, 10000, 9999}).has_value());
        CHECK(!quiz::common_cycle_time_us(std::vector<int>{}).has_value());
    }
#endif

    return report();
}
