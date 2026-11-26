// ============================================================================
// 07. (보너스) onnx — "진짜 ONNX Runtime 으로 추론 한 번"
// ============================================================================
// 이 파일만 수정한다. 검사: dev container 안에서 `make bonus` (문항별: `make bonus Q=2`)
//
// ONNX Runtime 이 설치된 dev container 에서만 빌드된다 (host 에는 헤더가 없다).
// `make all` / `make verify` 에는 들어 있지 않다. 준비 방법은 GUIDE.md.
//
// 06 Q2 의 run_inference 는 추론 자리에 std::function 을 끼웠다. 여기서는 그 자리에
// 실제 ONNX Runtime 세션을 넣는다. 레포의 InferenceEngine 과 같은 구조다.
//
// 레포: src/engine/inference/inference_engine.cpp
//       include/diden_core/engine/inference/inference_engine.h
// ============================================================================
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include <onnxruntime_cxx_api.h>

namespace quiz {

// ----------------------------------------------------------------------------
// OnnxEngine — 입력 하나, 출력 하나짜리 ONNX 모델을 돌리는 엔진
//
// Q1. init — 모델 로드와 입출력 차원 추출
//   model 은 .onnx 파일의 내용(바이트)이다. 파일 경로가 아니다 — 레포는 실행파일에 내장된
//   번들에서 바이트를 꺼내 넘긴다 (04_bundle 퀴즈).
//
//   순서:
//     1) Ort::SessionOptions 를 만들고 스레드 수(SetIntraOpNumThreads)를 n_threads 로 둔다
//     2) Ort::Session 을 "메모리의 바이트" 로 만든다
//     3) 입력이 1 개, 출력이 1 개가 아니면 실패
//     4) 입력·출력의 이름과 shape 을 읽어 둔다
//     5) shape 의 첫 차원(batch)이 음수(dynamic)면 1 로 바꾼다
//     6) 그러고도 음수인 차원이 남아 있으면 실패 (입력 길이를 정할 수 없다)
//     7) input_dim / output_dim = shape 의 모든 차원의 곱
//
//   ONNX Runtime 은 실패를 예외(Ort::Exception)로 알린다. init 밖으로 예외가 나가면 안 된다 —
//   잡아서 false 를 돌려준다. 실패한 뒤에는 is_valid() == false, 차원은 0 이다.
//   init 을 다시 부르면 이전 세션은 버린다.
//
// Q2. run — 추론
//     - 유효하지 않은 엔진이면 false
//     - input.size() != input_dim() 또는 output.size() != output_dim() 이면 false
//     - 입력 버퍼와 출력 버퍼를 "복사 없이" Ort::Value 텐서로 감싸서 Session::Run 을 부른다
//       (출력 텐서를 미리 묶어 두면 ONNX Runtime 이 output 의 메모리에 바로 쓴다)
//     - 예외가 나면 false
//
// Q3. NaN 가드
//     - 입력에 NaN/Inf 가 있으면 Run 을 부르지 않고 false (output 은 건드리지 않는다)
//     - Run 뒤 출력에 NaN/Inf 가 있으면 false
//
//   레포: inference_engine.cpp InferenceEngine::init / run
// ----------------------------------------------------------------------------
class OnnxEngine {
public:
    // Ort::Env 는 세션보다 오래 살아야 한다. 멤버로 두고 생성자에서 만든다.
    OnnxEngine()
    = default;  // TODO: Ort::Env 멤버를 추가하고 여기서 초기화하라

    [[nodiscard]] auto init(std::span<const std::byte> model, int n_threads = 1) -> bool {  // TODO
        (void)model;
        (void)n_threads;
        return false;
    }

    [[nodiscard]] auto is_valid() const -> bool {  // TODO
        return false;
    }

    [[nodiscard]] auto input_dim() const -> int {  // TODO
        return 0;
    }

    [[nodiscard]] auto output_dim() const -> int {  // TODO
        return 0;
    }

    [[nodiscard]] auto run(std::span<const float> input, std::span<float> output) -> bool {  // TODO
        (void)input;
        (void)output;
        return false;
    }

private:
    // TODO: 필요한 멤버 변수와 보조 함수.
    //   힌트: Ort::Session 과 Ort::MemoryInfo 는 기본 생성자가 없다 → std::optional 로 들고 있다가 emplace.
};

}  // namespace quiz
