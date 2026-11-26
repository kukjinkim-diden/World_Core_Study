// 수정 금지 — quiz.h 를 고쳐서 통과시킬 것.
#include <cmath>
#include <limits>

#include "check.h"
#include "quiz.h"

namespace {

using quiz::Mat3;
using quiz::Quat;
using quiz::Vec3;

auto near(double a, double b, double eps = 1e-6) -> bool {
    return std::abs(a - b) <= eps;
}

template <typename A, std::size_t N>
auto near3(const std::array<A, N>& got, std::initializer_list<double> want, double eps = 1e-6) -> bool {
    if (got.size() != want.size()) return false;
    std::size_t i = 0;
    for (double w : want) {
        if (!near(static_cast<double>(got[i]), w, eps)) return false;
        ++i;
    }
    return true;
}

auto mat_near(const Mat3& got, const Mat3& want) -> bool {
    for (std::size_t r = 0; r < 3; ++r) {
        for (std::size_t c = 0; c < 3; ++c) {
            if (!near(got[r][c], want[r][c])) return false;
        }
    }
    return true;
}

// cos(45°) = sin(45°). 90° 회전 쿼터니언은 (c, c·축) 이다.
const double kC45 = std::sqrt(0.5);

}  // namespace

int main() {
#ifdef TEST_Q1
    SECTION("Q1. 쿼터니언 → 회전행렬");
    {
        const Mat3 kIdentity{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
        CHECK(mat_near(quiz::quat_to_rotation(Quat{1, 0, 0, 0}), kIdentity));

        // yaw +90° (z 축): body x 축이 world y 를 본다
        const Mat3 kYaw90{{{0, -1, 0}, {1, 0, 0}, {0, 0, 1}}};
        CHECK(mat_near(quiz::quat_to_rotation(Quat{kC45, 0, 0, kC45}), kYaw90));

        // roll +90° (x 축): body y 축이 world z(위)를 본다
        const Mat3 kRoll90{{{1, 0, 0}, {0, 0, -1}, {0, 1, 0}}};
        CHECK(mat_near(quiz::quat_to_rotation(Quat{kC45, kC45, 0, 0}), kRoll90));

        // pitch +90° (y 축): body x 축이 world -z(아래)를 본다
        const Mat3 kPitch90{{{0, 0, 1}, {0, 1, 0}, {-1, 0, 0}}};
        CHECK(mat_near(quiz::quat_to_rotation(Quat{kC45, 0, kC45, 0}), kPitch90));

        // 180° 뒤집기 (x 축): (w, x, y, z) = (0, 1, 0, 0)
        const Mat3 kFlip{{{1, 0, 0}, {0, -1, 0}, {0, 0, -1}}};
        CHECK(mat_near(quiz::quat_to_rotation(Quat{0, 1, 0, 0}), kFlip));
    }
#endif

#ifdef TEST_Q2
    SECTION("Q2. projected_gravity");
    {
        // Q1 과 독립으로 검사하려고 회전행렬을 손으로 적어 넣는다.
        const Mat3 kIdentity{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
        const Mat3 kYaw90{{{0, -1, 0}, {1, 0, 0}, {0, 0, 1}}};
        const Mat3 kRoll90{{{1, 0, 0}, {0, 0, -1}, {0, 1, 0}}};
        const Mat3 kPitch90{{{0, 0, 1}, {0, 1, 0}, {-1, 0, 0}}};
        const Mat3 kFlip{{{1, 0, 0}, {0, -1, 0}, {0, 0, -1}}};

        CHECK(near3(quiz::projected_gravity(kIdentity), {0, 0, -1}));  // 똑바로 서 있음
        CHECK(near3(quiz::projected_gravity(kYaw90), {0, 0, -1}));     // 제자리 회전은 중력 방향을 안 바꾼다
        CHECK(near3(quiz::projected_gravity(kRoll90), {0, -1, 0}));    // 왼쪽 옆구리(+y)가 하늘을 봄 → 중력이 body -y
        CHECK(near3(quiz::projected_gravity(kPitch90), {1, 0, 0}));    // 코를 땅으로 → 중력이 body +x
        CHECK(near3(quiz::projected_gravity(kFlip), {0, 0, 1}));       // 뒤집힘 (천장 보행) → body +z

        // roll 30°: sin30 = 0.5, cos30 = 0.8660254
        const double c = std::cos(std::numbers::pi / 6.0);
        const double s = 0.5;
        const Mat3 kRoll30{{{1, 0, 0}, {0, c, -s}, {0, s, c}}};
        CHECK(near3(quiz::projected_gravity(kRoll30), {0, -0.5, -0.8660254}));
    }
#endif

#ifdef TEST_Q3
    SECTION("Q3. CommandLimits 만들기");
    {
        using quiz::CommandLimits;
        using quiz::RawCommandBound;
        using quiz::RawCommandLimits;
        const double kNaN = std::numeric_limits<double>::quiet_NaN();

        // 아무것도 안 적힌 정책 → 전 채널 ±5
        const CommandLimits none = quiz::build_command_limits(RawCommandLimits{});
        CHECK(none.lower == (Vec3{-5.0, -5.0, -5.0}));
        CHECK(none.upper == (Vec3{5.0, 5.0, 5.0}));

        // policy.h 주석의 예: x [-0.1, 0.3], y 는 max 만, wz 는 채널 누락
        RawCommandLimits raw{};
        raw[0] = RawCommandBound{.min = -0.1, .max = 0.3};
        raw[1] = RawCommandBound{.min = std::nullopt, .max = 0.2};
        const CommandLimits a = quiz::build_command_limits(raw);
        CHECK(a.lower == (Vec3{-0.1, -5.0, -5.0}));
        CHECK(a.upper == (Vec3{0.3, 0.2, 5.0}));

        // min > max → 그 채널만 기본 바운드로 되돌린다 (다른 채널은 그대로)
        RawCommandLimits inverted{};
        inverted[0] = RawCommandBound{.min = -0.1, .max = 0.3};
        inverted[2] = RawCommandBound{.min = 0.5, .max = -0.5};
        const CommandLimits b = quiz::build_command_limits(inverted);
        CHECK(b.lower == (Vec3{-0.1, -5.0, -5.0}));
        CHECK(b.upper == (Vec3{0.3, 5.0, 5.0}));

        // NaN 바운드 → 역시 기본 바운드로
        RawCommandLimits nan_bound{};
        nan_bound[1] = RawCommandBound{.min = kNaN, .max = 0.2};
        const CommandLimits c = quiz::build_command_limits(nan_bound);
        CHECK(c.lower[1] == -5.0);
        CHECK(c.upper[1] == 5.0);
    }
#endif

#ifdef TEST_Q4
    SECTION("Q4. 명령 clamp");
    {
        quiz::CommandLimits limits;
        limits.lower = Vec3{-0.1, -0.2, -0.15};
        limits.upper = Vec3{0.3, 0.2, 0.15};

        CHECK(quiz::clamp_command(Vec3{0.2, 0.1, -0.1}, limits) == (Vec3{0.2, 0.1, -0.1}));   // 범위 안 → 그대로
        CHECK(quiz::clamp_command(Vec3{1.0, -1.0, 0.1}, limits) == (Vec3{0.3, -0.2, 0.1}));   // 채널마다 따로 자른다
        CHECK(quiz::clamp_command(Vec3{-1.0, 0.0, 9.0}, limits) == (Vec3{-0.1, 0.0, 0.15}));  // 앞/뒤 한계가 다르다

        // 정책 YAML 이 limit 을 안 적었어도 무한대 명령이 정책에 닿지는 않는다
        const double kInf = std::numeric_limits<double>::infinity();
        const quiz::CommandLimits defaults;
        CHECK(quiz::clamp_command(Vec3{kInf, -kInf, 100.0}, defaults) == (Vec3{5.0, -5.0, 5.0}));
    }
#endif

#ifdef TEST_Q5
    SECTION("Q5. command 관측과 deadzone");
    {
        const double kThr = 0.003;  // golden 정책 YAML 의 command_threshold

        CHECK(near3(quiz::command_obs(Vec3{0.5, -0.2, 0.1}, kThr), {0.5, -0.2, 0.1}));
        CHECK(near3(quiz::command_obs(Vec3{0.002, -0.0029, 0.5}, kThr), {0.0, 0.0, 0.5}));  // 성분별로 따로
        // 경계: |v| == threshold 는 "미만"이 아니므로 살아남는다
        CHECK(near3(quiz::command_obs(Vec3{0.003, -0.003, 0.0}, kThr), {0.003, -0.003, 0.0}, 1e-9));
        CHECK(quiz::command_obs(Vec3{0.002, 0.0, 0.0}, kThr)[0] == 0.0f);  // 작은 값이 아니라 정확히 0

        CHECK(quiz::command_active(Vec3{0.5, 0.0, 0.0}, kThr));
        CHECK(quiz::command_active(Vec3{0.0, 0.0, -0.01}, kThr));   // 회전만 있어도 명령이다
        CHECK(!quiz::command_active(Vec3{0.0, 0.0, 0.0}, kThr));
        CHECK(!quiz::command_active(Vec3{0.003, 0.0, 0.0}, kThr));  // 경계: "초과"가 아니다

        // 두 판정이 갈리는 입력: 성분은 각각 0.003 미만이지만 크기는 0.00354
        const Vec3 kDiagonal{0.0025, 0.0025, 0.0};
        CHECK(near3(quiz::command_obs(kDiagonal, kThr), {0.0, 0.0, 0.0}, 0.0));
        CHECK(quiz::command_active(kDiagonal, kThr));
    }
#endif

#ifdef TEST_Q6
    SECTION("Q6. gait_phi");
    {
        const std::array<double, 4> kCrawl{0.0, 0.25, 0.5, 0.75};
        const std::array<double, 4> kTrot{0.0, 0.5, 0.5, 0.0};

        // 위상 0, crawl: 다리마다 90° 씩 밀려 있다. 출력은 [cos, sin] 쌍 4개.
        CHECK(near3(quiz::gait_phi(0.0, kCrawl), {1, 0, 0, 1, -1, 0, 0, -1}));
        // 위상 1/4 (= 90°), trot: 대각선 다리끼리 같은 값
        CHECK(near3(quiz::gait_phi(0.25, kTrot), {0, 1, 0, -1, 0, -1, 0, 1}));
        // 위상 1/2, crawl
        CHECK(near3(quiz::gait_phi(0.5, kCrawl), {-1, 0, 0, -1, 1, 0, 0, 1}));
        // offset 은 한 바퀴(1.0)를 넘어도 같은 위상이다: 1.25 == 0.25
        const std::array<double, 4> kWrapped{1.0, 1.25, 2.5, 0.75};
        CHECK(near3(quiz::gait_phi(0.0, kWrapped), {1, 0, 0, 1, -1, 0, 0, -1}));
        // 위상 1/8 (= 45°), offset 없음: cos = sin = 0.7071068
        const std::array<double, 4> kNone{0.0, 0.0, 0.0, 0.0};
        CHECK(near3(quiz::gait_phi(0.125, kNone), {0.7071068, 0.7071068, 0.7071068, 0.7071068, 0.7071068, 0.7071068, 0.7071068, 0.7071068}));
    }
#endif

    return report();
}
