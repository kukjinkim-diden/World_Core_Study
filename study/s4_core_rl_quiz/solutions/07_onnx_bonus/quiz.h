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
        : env_(ORT_LOGGING_LEVEL_WARNING, "s4_quiz") {}

    [[nodiscard]] auto init(std::span<const std::byte> model, int n_threads = 1) -> bool {  // TODO
        valid_ = false;
        session_.reset();
        input_dim_ = 0;
        output_dim_ = 0;

        try {
            Ort::SessionOptions opts;
            opts.SetIntraOpNumThreads(n_threads);
            opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

            session_.emplace(env_, model.data(), model.size(), opts);

            if (session_->GetInputCount() != 1 || session_->GetOutputCount() != 1) {
                session_.reset();
                return false;
            }

            Ort::AllocatorWithDefaultOptions allocator;
            input_name_ = session_->GetInputNameAllocated(0, allocator).get();
            output_name_ = session_->GetOutputNameAllocated(0, allocator).get();

            input_shape_ = session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
            output_shape_ = session_->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
            fix_dynamic_batch(input_shape_);
            fix_dynamic_batch(output_shape_);
            if (has_dynamic_dim(input_shape_) || has_dynamic_dim(output_shape_)) {
                session_.reset();
                return false;
            }

            input_dim_ = flatten_dim(input_shape_);
            output_dim_ = flatten_dim(output_shape_);
            mem_info_.emplace(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));
            valid_ = true;
            return true;
        } catch (const Ort::Exception&) {
            session_.reset();
            input_dim_ = 0;
            output_dim_ = 0;
            return false;
        }
    }

    [[nodiscard]] auto is_valid() const -> bool {  // TODO
        return valid_;
    }

    [[nodiscard]] auto input_dim() const -> int {  // TODO
        return input_dim_;
    }

    [[nodiscard]] auto output_dim() const -> int {  // TODO
        return output_dim_;
    }

    [[nodiscard]] auto run(std::span<const float> input, std::span<float> output) -> bool {  // TODO
        if (!valid_ || !session_ || !mem_info_) return false;
        if (input.size() != static_cast<std::size_t>(input_dim_)) return false;
        if (output.size() != static_cast<std::size_t>(output_dim_)) return false;
        if (!finite(input)) return false;

        try {
            // CreateTensor 가 비-const 포인터를 요구한다. 읽기만 하므로 const_cast 로 넘긴다.
            auto in_tensor = Ort::Value::CreateTensor<float>(*mem_info_, const_cast<float*>(input.data()), input.size(), input_shape_.data(), input_shape_.size());
            auto out_tensor = Ort::Value::CreateTensor<float>(*mem_info_, output.data(), output.size(), output_shape_.data(), output_shape_.size());

            const std::array<const char*, 1> input_names{input_name_.c_str()};
            const std::array<const char*, 1> output_names{output_name_.c_str()};
            session_->Run(Ort::RunOptions{nullptr}, input_names.data(), &in_tensor, 1, output_names.data(), &out_tensor, 1);
            return finite(output);
        } catch (const Ort::Exception&) {
            return false;
        }
    }

private:
    // TODO: 필요한 멤버 변수와 보조 함수.
    //   힌트: Ort::Session 과 Ort::MemoryInfo 는 기본 생성자가 없다 → std::optional 로 들고 있다가 emplace.
    static auto fix_dynamic_batch(std::vector<std::int64_t>& shape) -> void {
        if (!shape.empty() && shape[0] < 0) shape[0] = 1;
    }

    static auto has_dynamic_dim(const std::vector<std::int64_t>& shape) -> bool {
        return std::ranges::any_of(shape, [](std::int64_t d) -> bool { return d < 0; });
    }

    static auto flatten_dim(const std::vector<std::int64_t>& shape) -> int {
        std::int64_t product = 1;
        for (std::int64_t d : shape) product *= d;
        return static_cast<int>(product);
    }

    static auto finite(std::span<const float> values) -> bool {
        return std::ranges::all_of(values, [](float v) -> bool { return std::isfinite(v); });
    }

    Ort::Env env_;
    std::optional<Ort::Session> session_;
    std::optional<Ort::MemoryInfo> mem_info_;
    std::string input_name_;
    std::string output_name_;
    std::vector<std::int64_t> input_shape_;
    std::vector<std::int64_t> output_shape_;
    int input_dim_ = 0;
    int output_dim_ = 0;
    bool valid_ = false;
};

}  // namespace quiz
