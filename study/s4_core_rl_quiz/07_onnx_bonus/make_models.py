#!/usr/bin/env python3
"""07 보너스용 초소형 ONNX 모델 생성기.

models/ 안의 .onnx 파일은 이 스크립트로 만든 것이다. 이미 만들어 넣어 두었으므로
퀴즈를 풀 때 이 스크립트를 돌릴 필요는 없다 (onnx 파이썬 패키지가 있어야 돈다).

    cd study/s4_core_rl_quiz/07_onnx_bonus
    uv run python make_models.py        # 스터디 .venv 에 onnx 가 들어 있다

DIDEN_World 의 export 와 같은 모양으로 만든다 (tools/deploy 의 ONNX_EXPORT_CONFIG):
입력 이름 "input", 출력 이름 "output", opset 11, 첫 차원(batch)만 dynamic.
"""
from pathlib import Path

import numpy as np
import onnx
from onnx import TensorProto, helper, numpy_helper

OUT = Path(__file__).parent / "models"
OPSET = [helper.make_opsetid("", 11)]


def save(graph: onnx.GraphProto, name: str) -> None:
    model = helper.make_model(graph, opset_imports=OPSET, producer_name="s4_core_rl_quiz")
    model.ir_version = 6
    onnx.checker.check_model(model)
    OUT.mkdir(exist_ok=True)
    onnx.save(model, OUT / name)
    print("wrote", OUT / name)


def tiny_policy() -> None:
    # output = input @ W + b      (입력 6, 출력 3)
    #   out0 = x0 + x1
    #   out1 = 2·x2 − x3
    #   out2 = 0.5·x4 + 0.5·x5 + 1
    w = np.array(
        [[1, 0, 0], [1, 0, 0], [0, 2, 0], [0, -1, 0], [0, 0, 0.5], [0, 0, 0.5]],
        dtype=np.float32,
    )
    b = np.array([0, 0, 1], dtype=np.float32)
    graph = helper.make_graph(
        nodes=[helper.make_node("Gemm", ["input", "W", "b"], ["output"])],
        name="tiny_policy",
        inputs=[helper.make_tensor_value_info("input", TensorProto.FLOAT, ["batch", 6])],
        outputs=[helper.make_tensor_value_info("output", TensorProto.FLOAT, ["batch", 3])],
        initializer=[numpy_helper.from_array(w, "W"), numpy_helper.from_array(b, "b")],
    )
    save(graph, "tiny_policy.onnx")


def dynamic_feature() -> None:
    # batch 가 아닌 차원이 dynamic — 입력 길이를 정할 수 없는 모델
    graph = helper.make_graph(
        nodes=[helper.make_node("Identity", ["input"], ["output"])],
        name="dynamic_feature",
        inputs=[helper.make_tensor_value_info("input", TensorProto.FLOAT, ["batch", "n"])],
        outputs=[helper.make_tensor_value_info("output", TensorProto.FLOAT, ["batch", "n"])],
    )
    save(graph, "dynamic_feature.onnx")


def two_inputs() -> None:
    # 입력이 둘인 모델
    graph = helper.make_graph(
        nodes=[helper.make_node("Add", ["a", "b"], ["output"])],
        name="two_inputs",
        inputs=[
            helper.make_tensor_value_info("a", TensorProto.FLOAT, ["batch", 3]),
            helper.make_tensor_value_info("b", TensorProto.FLOAT, ["batch", 3]),
        ],
        outputs=[helper.make_tensor_value_info("output", TensorProto.FLOAT, ["batch", 3])],
    )
    save(graph, "two_inputs.onnx")


if __name__ == "__main__":
    tiny_policy()
    dynamic_feature()
    two_inputs()
