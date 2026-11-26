ALL_Q := 1 2 3
include ../common/rules.mk

# ONNX Runtime 이 설치된 dev container 에서만 빌드된다.
# 링크할 라이브러리가 많아 이 폴더만 CMake 를 쓴다 (CMakeLists.txt 참고).
ORT_HEADER ?= /usr/local/include/onnxruntime_cxx_api.h
BUILD_DIR := build

empty :=
space := $(empty) $(empty)
QUIZ_Q := $(subst $(space),;,$(strip $(Q)))

.PHONY: run clean
run:
	@test -f $(ORT_HEADER) || { \
		echo "ONNX Runtime 헤더($(ORT_HEADER))가 없다."; \
		echo "07 은 dev container 안에서만 빌드된다 — 방법은 07_onnx_bonus/GUIDE.md 의 '준비' 참고."; \
		exit 1; }
	@cmake -S . -B $(BUILD_DIR) -DQUIZ_Q="$(QUIZ_Q)" -DQUIZ_WERROR=$(if $(WERROR),ON,OFF) > $(BUILD_DIR).log 2>&1 \
		&& cmake --build $(BUILD_DIR) >> $(BUILD_DIR).log 2>&1 \
		|| { cat $(BUILD_DIR).log; exit 1; }
	@./$(BUILD_DIR)/quiz

clean:
	@rm -rf $(BUILD_DIR) $(BUILD_DIR).log quiz
