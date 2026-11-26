// 수정 금지 — quiz.h 를 고쳐서 통과시킬 것.
#include <cmath>
#include <initializer_list>
#include <limits>
#include <vector>

#include "check.h"
#include "quiz.h"

namespace {

using quiz::HistoryOrder;
using quiz::ModifierSpec;
using quiz::Tensor;

const float kNaN = std::numeric_limits<float>::quiet_NaN();
const float kInf = std::numeric_limits<float>::infinity();
constexpr float kSentinel = -999.0f;  // "아무도 안 썼다" 표시

auto vec_near(const Tensor& got, std::initializer_list<float> want, float eps = 1e-5f) -> bool {
    if (got.size() != want.size()) return false;
    std::size_t i = 0;
    for (float w : want) {
        if (!(std::abs(got[i] - w) <= eps)) return false;
        ++i;
    }
    return true;
}

auto ema(double alpha) -> ModifierSpec {
    return ModifierSpec{.type = ModifierSpec::Type::kEma, .alpha = alpha};
}

auto notch(double center_freq, double bandwidth, double sample_rate) -> ModifierSpec {
    return ModifierSpec{.type = ModifierSpec::Type::kNotch, .center_freq = center_freq, .bandwidth = bandwidth, .sample_rate = sample_rate};
}

// 필터에 sample 하나를 넣고 출력을 돌려준다.
auto run(quiz::FilterBase& f, std::initializer_list<float> x) -> Tensor {
    const Tensor in(x);
    Tensor out(in.size(), kSentinel);
    f.step(in, out);
    return out;
}

// history 의 현재 출력. 크기는 테스트가 정한다 (stub 이 dim() == 0 이어도 죽지 않게).
template <typename H>
auto snapshot(const H& h, std::size_t size) -> Tensor {
    Tensor out(size, kSentinel);
    h.fill(out);
    return out;
}

}  // namespace

int main() {
#ifdef TEST_Q1
    SECTION("Q1. all_finite 와 EmaFilter");
    {
        CHECK(quiz::all_finite(Tensor{1.0f, -2.0f, 0.0f}));
        CHECK(quiz::all_finite(Tensor{}));
        CHECK(!quiz::all_finite(Tensor{1.0f, kNaN}));
        CHECK(!quiz::all_finite(Tensor{-kInf, 1.0f}));

        // α = 0.5, 2 채널. 채널끼리는 서로 영향을 주지 않는다.
        quiz::EmaFilter f(ema(0.5));
        f.reset(2);
        CHECK(vec_near(run(f, {4.0f, -8.0f}), {4.0f, -8.0f}));  // 첫 sample: 그대로 (warm start)
        CHECK(vec_near(run(f, {0.0f, -8.0f}), {2.0f, -8.0f}));  // 0.5·0 + 0.5·4
        CHECK(vec_near(run(f, {0.0f, 0.0f}), {1.0f, -4.0f}));
        CHECK(vec_near(run(f, {3.0f, 0.0f}), {2.0f, -2.0f}));   // 0.5·3 + 0.5·1

        // α = 0.75 (policy.h 주석의 예): 새 값에 더 무게
        quiz::EmaFilter g(ema(0.75));
        g.reset(1);
        CHECK(vec_near(run(g, {4.0f}), {4.0f}));
        CHECK(vec_near(run(g, {0.0f}), {1.0f}));   // 0.75·0 + 0.25·4
        CHECK(vec_near(run(g, {0.0f}), {0.25f}));

        // NaN/Inf: 직전 출력을 그대로 내보내고, 상태는 오염되지 않는다
        quiz::EmaFilter h(ema(0.5));
        h.reset(2);
        CHECK(vec_near(run(h, {4.0f, 4.0f}), {4.0f, 4.0f}));
        CHECK(vec_near(run(h, {kNaN, 100.0f}), {4.0f, 4.0f}));  // 한 채널만 NaN 이어도 sample 전체를 버린다
        CHECK(vec_near(run(h, {kInf, 0.0f}), {4.0f, 4.0f}));
        CHECK(vec_near(run(h, {0.0f, 0.0f}), {2.0f, 2.0f}));    // 0.5·0 + 0.5·4 — NaN 이 없었던 것처럼

        // 첫 sample 부터 NaN: reset 의 0 이 나가고, 다음 정상 sample 에서 warm start
        quiz::EmaFilter k(ema(0.5));
        k.reset(1);
        CHECK(vec_near(run(k, {kNaN}), {0.0f}));
        CHECK(vec_near(run(k, {6.0f}), {6.0f}));  // 0.5·6 + 0.5·0 = 3 이 아니다
        CHECK(vec_near(run(k, {0.0f}), {3.0f}));

        // in-place: 입력과 출력이 같은 버퍼여도 된다
        quiz::EmaFilter p(ema(0.5));
        p.reset(1);
        Tensor buf{8.0f};
        p.step(buf, buf);
        buf[0] = 0.0f;
        p.step(buf, buf);
        CHECK(vec_near(buf, {4.0f}));

        // reset 은 warm start 를 다시 건다
        p.reset(1);
        CHECK(vec_near(run(p, {-2.0f}), {-2.0f}));
    }
#endif

#ifdef TEST_Q2
    SECTION("Q2. NotchFilter");
    {
        // 손으로 따라가기 쉬운 계수: f0 = fs/4 → cos(w0) = 0, sin(w0) = 1.  bw = 2·f0 → Q = 0.5 → a = 1, a0 = 2
        //   b0 = 0.5, b1 = 0, b2 = 0.5, a1 = 0, a2 = 0   →   y[n] = 0.5·x[n] + 0.5·x[n-2]
        const quiz::NotchCoeffs easy = quiz::notch_coeffs(notch(25.0, 50.0, 100.0));
        CHECK(vec_near(Tensor{easy.b0, easy.b1, easy.b2, easy.a1, easy.a2}, {0.5f, 0.0f, 0.5f, 0.0f, 0.0f}));

        // policy.h 주석의 예: {center_freq: 12, bandwidth: 4, sample_rate: 100}
        const quiz::NotchCoeffs real = quiz::notch_coeffs(notch(12.0, 4.0, 100.0));
        CHECK(vec_near(Tensor{real.b0, real.b1, real.b2}, {0.8975926f, -1.3086337f, 0.8975926f}));
        CHECK(vec_near(Tensor{real.a1, real.a2}, {-1.3086337f, 0.7951852f}));

        quiz::NotchFilter f(notch(25.0, 50.0, 100.0));
        f.reset(1);
        CHECK(vec_near(run(f, {1.0f}), {1.0f}));   // 첫 sample: 그대로. 과거 4칸이 전부 1 이 된다
        CHECK(vec_near(run(f, {0.0f}), {0.5f}));   // 0.5·0  + 0.5·x[n-2](=1)
        CHECK(vec_near(run(f, {-1.0f}), {0.0f}));  // 0.5·-1 + 0.5·x[n-2](=1)
        CHECK(vec_near(run(f, {0.0f}), {0.0f}));   // 0.5·0  + 0.5·x[n-2](=0)
        CHECK(vec_near(run(f, {1.0f}), {0.0f}));   // 0.5·1  + 0.5·x[n-2](=-1)  ← 25 Hz 성분(1,0,-1,0,...)이 지워진다

        // 일정한 입력(0 Hz)은 그대로 통과한다 — 실제 계수로
        quiz::NotchFilter dc(notch(12.0, 4.0, 100.0));
        dc.reset(2);
        CHECK(vec_near(run(dc, {3.0f, -1.0f}), {3.0f, -1.0f}));
        CHECK(vec_near(run(dc, {3.0f, -1.0f}), {3.0f, -1.0f}, 1e-4f));
        CHECK(vec_near(run(dc, {3.0f, -1.0f}), {3.0f, -1.0f}, 1e-4f));

        // 재귀 항(a1, a2)까지 쓰는지: 실제 계수에서 3 → 0 으로 떨어질 때
        //   y = b0·0 + b1·3 + b2·3 - a1·3 - a2·3 = 3·(b2 - a2) = 3·(0.8975926 - 0.7951852)
        CHECK(vec_near(run(dc, {0.0f, -1.0f}), {0.3072222f, -1.0f}, 1e-4f));

        // NaN: 직전 출력 유지, 상태 불변
        quiz::NotchFilter g(notch(25.0, 50.0, 100.0));
        g.reset(1);
        CHECK(vec_near(run(g, {1.0f}), {1.0f}));
        CHECK(vec_near(run(g, {0.0f}), {0.5f}));
        CHECK(vec_near(run(g, {kNaN}), {0.5f}));
        CHECK(vec_near(run(g, {-1.0f}), {0.0f}));  // NaN 이 없었던 것처럼 이어진다

        // in-place
        quiz::NotchFilter p(notch(25.0, 50.0, 100.0));
        p.reset(1);
        Tensor buf{1.0f};
        p.step(buf, buf);
        buf[0] = 0.0f;
        p.step(buf, buf);
        CHECK(vec_near(buf, {0.5f}));
        buf[0] = -1.0f;
        p.step(buf, buf);
        CHECK(vec_near(buf, {0.0f}));
    }
#endif

#ifdef TEST_Q3
    SECTION("Q3. FilterChain");
    {
        // 빈 체인 = 그대로 통과
        const quiz::FilterChain empty = quiz::make_filter_chain({});
        CHECK(empty.empty());
        const Tensor x{1.5f, -2.5f};
        Tensor y(2, kSentinel);
        quiz::apply_filter_chain(empty, x, y);
        CHECK(vec_near(y, {1.5f, -2.5f}));

        // EMA(0.5) 두 단
        const quiz::FilterChain two = quiz::make_filter_chain({ema(0.5), ema(0.5)});
        CHECK(two.size() == 2);
        quiz::reset_filter_chain(two, 1);
        Tensor out(1, kSentinel);
        quiz::apply_filter_chain(two, Tensor{8.0f}, out);
        CHECK(vec_near(out, {8.0f}));  // 둘 다 warm start
        quiz::apply_filter_chain(two, Tensor{0.0f}, out);
        CHECK(vec_near(out, {6.0f}));  // 1단: 4,  2단: 0.5·4 + 0.5·8
        quiz::apply_filter_chain(two, Tensor{0.0f}, out);
        CHECK(vec_near(out, {4.0f}));  // 1단: 2,  2단: 0.5·2 + 0.5·6

        // 종류가 섞인 체인: type 에 맞는 필터가 등장 순서대로 만들어져야 한다
        //   입력 1, 0, -1 에 대해 (notch 는 Q2 의 쉬운 계수, ema 는 0.5)
        const quiz::FilterChain ne = quiz::make_filter_chain({notch(25.0, 50.0, 100.0), ema(0.5)});
        const quiz::FilterChain en = quiz::make_filter_chain({ema(0.5), notch(25.0, 50.0, 100.0)});
        CHECK(ne.size() == 2 && en.size() == 2);
        quiz::reset_filter_chain(ne, 1);
        quiz::reset_filter_chain(en, 1);
        Tensor a(1, kSentinel);
        Tensor b(1, kSentinel);
        for (float v : {1.0f, 0.0f, -1.0f}) {
            quiz::apply_filter_chain(ne, Tensor{v}, a);
            quiz::apply_filter_chain(en, Tensor{v}, b);
        }
        // [notch, ema]: notch 출력 1, 0.5, 0   → ema 1, 0.75, 0.375
        CHECK(vec_near(a, {0.375f}));
        // [ema, notch]: ema 출력 1, 0.5, -0.25 → notch 1, 0.75, 0.5·(-0.25) + 0.5·1 = 0.375
        CHECK(vec_near(b, {0.375f}));
        // (둘 다 선형 필터라 순서를 바꿔도 값이 같다 — 단계별 중간값은 다르니 손으로 따라가 볼 것)

        // NaN 은 첫 단에서 막혀 뒤 단까지 가지 않는다
        const quiz::FilterChain guard = quiz::make_filter_chain({ema(0.5), ema(0.5)});
        quiz::reset_filter_chain(guard, 1);
        Tensor g(1, kSentinel);
        quiz::apply_filter_chain(guard, Tensor{8.0f}, g);
        quiz::apply_filter_chain(guard, Tensor{kNaN}, g);
        CHECK(vec_near(g, {8.0f}));  // 1단이 직전 출력 8 을 내보내고, 2단은 0.5·8 + 0.5·8
    }
#endif

#ifdef TEST_Q4
    SECTION("Q4. HistoryBuffer");
    {
        // 길이 3, 간격 1, 스칼라 sample
        quiz::HistoryBuffer oldest(1, 3, 1, HistoryOrder::kOldestFirst);
        quiz::HistoryBuffer newest(1, 3, 1, HistoryOrder::kNewestFirst);
        CHECK(oldest.dim() == 3);
        CHECK(oldest.slot_count() == 3);

        // 첫 push: 과거 칸은 0 으로 남아 있다
        oldest.push(Tensor{5.0f});
        newest.push(Tensor{5.0f});
        CHECK(vec_near(snapshot(oldest, 3), {0.0f, 0.0f, 5.0f}));
        CHECK(vec_near(snapshot(newest, 3), {5.0f, 0.0f, 0.0f}));

        oldest.push(Tensor{6.0f});
        newest.push(Tensor{6.0f});
        oldest.push(Tensor{7.0f});
        newest.push(Tensor{7.0f});
        CHECK(vec_near(snapshot(oldest, 3), {5.0f, 6.0f, 7.0f}));
        CHECK(vec_near(snapshot(newest, 3), {7.0f, 6.0f, 5.0f}));

        // 가득 찬 뒤: 가장 오래된 값이 밀려난다
        oldest.push(Tensor{8.0f});
        CHECK(vec_near(snapshot(oldest, 3), {6.0f, 7.0f, 8.0f}));

        // fill 은 상태를 바꾸지 않는다 (몇 번을 불러도 같은 값)
        CHECK(vec_near(snapshot(oldest, 3), {6.0f, 7.0f, 8.0f}));

        // golden 정책의 dof_vel_history: 길이 3, 간격 4 → 9칸을 보관하고 4칸 간격으로 3개를 꺼낸다
        quiz::HistoryBuffer sparse(1, 3, 4, HistoryOrder::kOldestFirst);
        CHECK(sparse.dim() == 3);
        CHECK(sparse.slot_count() == 9);
        for (int k = 1; k <= 9; ++k) sparse.push(Tensor{static_cast<float>(k)});
        CHECK(vec_near(snapshot(sparse, 3), {1.0f, 5.0f, 9.0f}));
        sparse.push(Tensor{10.0f});
        CHECK(vec_near(snapshot(sparse, 3), {2.0f, 6.0f, 10.0f}));

        // sample 이 벡터일 때: sample 단위로 통째로 이어 붙인다 (원소를 섞지 않는다)
        quiz::HistoryBuffer wide(2, 2, 1, HistoryOrder::kOldestFirst);
        CHECK(wide.dim() == 4);
        wide.push(Tensor{1.0f, 2.0f});
        wide.push(Tensor{3.0f, 4.0f});
        CHECK(vec_near(snapshot(wide, 4), {1.0f, 2.0f, 3.0f, 4.0f}));
        quiz::HistoryBuffer wide_newest(2, 2, 1, HistoryOrder::kNewestFirst);
        wide_newest.push(Tensor{1.0f, 2.0f});
        wide_newest.push(Tensor{3.0f, 4.0f});
        CHECK(vec_near(snapshot(wide_newest, 4), {3.0f, 4.0f, 1.0f, 2.0f}));

        // 길이·간격이 1 보다 작으면 1 로 올린다
        quiz::HistoryBuffer clamped(2, 0, 0, HistoryOrder::kOldestFirst);
        CHECK(clamped.dim() == 2);
        CHECK(clamped.slot_count() == 1);
        clamped.push(Tensor{9.0f, 8.0f});
        CHECK(vec_near(snapshot(clamped, 2), {9.0f, 8.0f}));
    }
#endif

#ifdef TEST_Q5
    SECTION("Q5. warm_start");
    {
        quiz::HistoryBuffer warm(1, 3, 2, HistoryOrder::kOldestFirst, /*warm_start=*/true);
        warm.push(Tensor{5.0f});
        CHECK(vec_near(snapshot(warm, 3), {5.0f, 5.0f, 5.0f}));  // 첫 push 가 전 칸을 채운다
        warm.push(Tensor{6.0f});
        CHECK(vec_near(snapshot(warm, 3), {5.0f, 5.0f, 6.0f}));
        warm.push(Tensor{7.0f});
        CHECK(vec_near(snapshot(warm, 3), {5.0f, 5.0f, 7.0f}));  // 간격 2: 5,5,5,6,7 중 [t-4, t-2, t]
        warm.push(Tensor{8.0f});
        CHECK(vec_near(snapshot(warm, 3), {5.0f, 6.0f, 8.0f}));

        // 기본값(false)은 Q4 의 동작 그대로다
        quiz::HistoryBuffer cold(1, 3, 2, HistoryOrder::kOldestFirst);
        cold.push(Tensor{5.0f});
        CHECK(vec_near(snapshot(cold, 3), {0.0f, 0.0f, 5.0f}));

        // 첫 값이 0 이어도 warm start 는 한 번만 일어난다
        quiz::HistoryBuffer zero_first(1, 2, 1, HistoryOrder::kNewestFirst, /*warm_start=*/true);
        zero_first.push(Tensor{0.0f});
        zero_first.push(Tensor{4.0f});
        CHECK(vec_near(snapshot(zero_first, 2), {4.0f, 0.0f}));
        zero_first.push(Tensor{9.0f});
        CHECK(vec_near(snapshot(zero_first, 2), {9.0f, 4.0f}));
    }
#endif

#ifdef TEST_Q6
    SECTION("Q6. HistoryTerm (필터 → history)");
    {
        // history 에 쌓이는 것은 "필터를 거친 값"이다
        quiz::HistoryTerm term(1, 2, 1, quiz::make_filter_chain({ema(0.5)}), HistoryOrder::kOldestFirst);
        CHECK(term.dim() == 2);
        term.tick(Tensor{4.0f});
        CHECK(vec_near(snapshot(term, 2), {0.0f, 4.0f}));  // 필터는 warm start, history 는 0 에서 시작
        term.tick(Tensor{0.0f});
        CHECK(vec_near(snapshot(term, 2), {4.0f, 2.0f}));  // raw 는 [4, 0], 필터 뒤는 [4, 2]
        term.tick(Tensor{0.0f});
        CHECK(vec_near(snapshot(term, 2), {2.0f, 1.0f}));

        // 필터 없는 history: raw 그대로 쌓인다
        quiz::HistoryTerm plain(2, 2, 1, quiz::make_filter_chain({}), HistoryOrder::kNewestFirst);
        CHECK(plain.dim() == 4);
        plain.tick(Tensor{1.0f, 2.0f});
        plain.tick(Tensor{3.0f, 4.0f});
        CHECK(vec_near(snapshot(plain, 4), {3.0f, 4.0f, 1.0f, 2.0f}));

        // 길이 1 = "필터만 건 term": 매 tick 필터 출력 하나만 보인다
        quiz::HistoryTerm filter_only(1, 1, 1, quiz::make_filter_chain({ema(0.5)}), HistoryOrder::kOldestFirst);
        CHECK(filter_only.dim() == 1);
        filter_only.tick(Tensor{8.0f});
        CHECK(vec_near(snapshot(filter_only, 1), {8.0f}));
        filter_only.tick(Tensor{0.0f});
        CHECK(vec_near(snapshot(filter_only, 1), {4.0f}));

        // NaN sample: 필터가 직전 출력으로 막아 주므로 history 에 NaN 이 들어가지 않는다
        filter_only.tick(Tensor{kNaN});
        CHECK(vec_near(snapshot(filter_only, 1), {4.0f}));

        // 필터가 없으면 NaN 이 그대로 쌓인다 — 막는 것은 필터이지 history 가 아니다
        quiz::HistoryTerm unguarded(1, 1, 1, quiz::make_filter_chain({}), HistoryOrder::kOldestFirst);
        unguarded.tick(Tensor{kNaN});
        CHECK(std::isnan(snapshot(unguarded, 1)[0]));

        // warm_start 는 history 로 전달된다
        quiz::HistoryTerm warm(1, 3, 1, quiz::make_filter_chain({}), HistoryOrder::kOldestFirst, /*warm_start=*/true);
        warm.tick(Tensor{5.0f});
        CHECK(vec_near(snapshot(warm, 3), {5.0f, 5.0f, 5.0f}));
    }
#endif

    return report();
}
