// 수정 금지 — quiz.h 를 고쳐서 통과시킬 것.
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>
#include <vector>

#include "check.h"
#include "quiz.h"

namespace {

// .onnx 파일을 통째로 읽어 바이트 배열로 돌려준다. (레포에서는 번들이 이 바이트를 준다)
auto read_file(const std::string& path) -> std::vector<std::byte> {
    std::ifstream in(path, std::ios::binary);
    const std::vector<char> chars((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    std::vector<std::byte> bytes(chars.size());
    for (std::size_t i = 0; i < chars.size(); ++i) bytes[i] = static_cast<std::byte>(chars[i]);
    return bytes;
}

auto near(float a, float b, float eps = 1e-5f) -> bool {
    return std::abs(a - b) <= eps;
}

constexpr float kSentinel = -999.0f;

}  // namespace

int main() {
    // tiny_policy.onnx:  입력 [batch, 6] → 출력 [batch, 3]
    //   out0 = x0 + x1        out1 = 2·x2 − x3        out2 = 0.5·x4 + 0.5·x5 + 1
    const std::vector<std::byte> tiny = read_file("models/tiny_policy.onnx");
    if (tiny.empty()) {
        std::printf("models/tiny_policy.onnx 을 읽지 못했다 — 07_onnx_bonus 폴더에서 실행할 것\n");
        return 1;
    }

#ifdef TEST_Q1
    SECTION("Q1. init — 모델 로드와 차원");
    {
        quiz::OnnxEngine engine;
        CHECK(!engine.is_valid());        // init 전
        CHECK(engine.init(tiny));
        CHECK(engine.is_valid());
        CHECK(engine.input_dim() == 6);   // batch 는 dynamic(-1) → 1 로 보고 1 × 6
        CHECK(engine.output_dim() == 3);

        // ONNX 가 아닌 바이트: ONNX Runtime 이 예외를 던진다 → 잡아서 false
        const std::vector<std::byte> garbage(64, std::byte{0x5A});
        quiz::OnnxEngine bad;
        CHECK(!bad.init(garbage));
        CHECK(!bad.is_valid());
        CHECK(bad.input_dim() == 0 && bad.output_dim() == 0);
        quiz::OnnxEngine empty;
        CHECK(!empty.init({}));

        // batch 가 아닌 차원이 dynamic → 입력 길이를 정할 수 없다
        quiz::OnnxEngine dyn;
        CHECK(!dyn.init(read_file("models/dynamic_feature.onnx")));
        CHECK(!dyn.is_valid());

        // 입력이 둘인 모델 → 이 엔진은 입력 하나 / 출력 하나만 다룬다
        quiz::OnnxEngine two;
        CHECK(!two.init(read_file("models/two_inputs.onnx")));
        CHECK(!two.is_valid());

        // 유효하던 엔진에 깨진 모델로 다시 init → 이전 모델이 남아 있으면 안 된다
        CHECK(!engine.init(garbage));
        CHECK(!engine.is_valid());
        CHECK(engine.init(tiny));         // 다시 올리면 정상
        CHECK(engine.is_valid() && engine.input_dim() == 6);
    }
#endif

#ifdef TEST_Q2
    SECTION("Q2. run — 추론");
    {
        quiz::OnnxEngine engine;
        CHECK(engine.init(tiny));

        std::vector<float> out(3, kSentinel);
        CHECK(engine.run(std::vector<float>{1, 2, 3, 4, 5, 6}, out));
        CHECK(near(out[0], 3.0f));   // 1 + 2
        CHECK(near(out[1], 2.0f));   // 2·3 − 4
        CHECK(near(out[2], 6.5f));   // 0.5·5 + 0.5·6 + 1

        CHECK(engine.run(std::vector<float>(6, 0.0f), out));
        CHECK(near(out[0], 0.0f) && near(out[1], 0.0f) && near(out[2], 1.0f));   // bias 만 남는다

        // 같은 엔진을 여러 번 불러도 된다 (세션은 init 에서 한 번만 만든다)
        for (int i = 0; i < 3; ++i) {
            CHECK(engine.run(std::vector<float>{-1, 1, 0, 2, 4, 0}, out) && near(out[0], 0.0f) && near(out[1], -2.0f) && near(out[2], 3.0f));
        }

        // 크기가 맞지 않으면 돌리지 않는다
        std::vector<float> wrong_out(4, kSentinel);
        CHECK(!engine.run(std::vector<float>(5, 0.0f), out));
        CHECK(!engine.run(std::vector<float>(6, 0.0f), wrong_out));
        CHECK(wrong_out[0] == kSentinel);

        // init 하지 않은 엔진
        quiz::OnnxEngine cold;
        CHECK(!cold.run(std::vector<float>(6, 0.0f), out));
    }
#endif

#ifdef TEST_Q3
    SECTION("Q3. NaN 가드");
    {
        quiz::OnnxEngine engine;
        CHECK(engine.init(tiny));

        // 입력 NaN → Run 을 부르지 않는다 (출력 버퍼가 그대로다)
        std::vector<float> out(3, kSentinel);
        std::vector<float> nan_in(6, 0.0f);
        nan_in[2] = std::numeric_limits<float>::quiet_NaN();
        CHECK(!engine.run(nan_in, out));
        CHECK(out[0] == kSentinel && out[1] == kSentinel && out[2] == kSentinel);

        std::vector<float> inf_in(6, 0.0f);
        inf_in[5] = std::numeric_limits<float>::infinity();
        CHECK(!engine.run(inf_in, out));
        CHECK(out[0] == kSentinel);

        // 입력은 유한하지만 출력이 넘친다: 3e38 + 3e38 > float 최댓값 → out0 = +Inf
        CHECK(!engine.run(std::vector<float>{3e38f, 3e38f, 0, 0, 0, 0}, out));

        // 실패한 뒤에도 엔진은 멀쩡하다
        CHECK(engine.run(std::vector<float>{1, 2, 3, 4, 5, 6}, out) && near(out[0], 3.0f));
    }
#endif

    return report();
}
