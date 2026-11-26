# 사용법
#   make 01            # 01 문제의 전체 문항 검사
#   make 01 Q="1 2"    # 01 문제의 1, 2번 문항만 검사 (나머지 문항 테스트는 컴파일에서 제외)
#   make all           # 01 ~ 06 전부
#   make verify        # solutions/ 의 정답으로 전 문항 통과 확인 (채점기 자체 점검용)
#   make check-stubs   # 아직 안 푼 quiz.h 가 문항마다 FAIL 을 내는지 확인 (퀴즈 제작자용)
#   make bonus         # 07: 진짜 ONNX Runtime 으로 추론 (dev container 전용, make all 에 포함 안 됨)
#   make verify-bonus  # 07 의 정답 검증 (dev container 전용)
#   make clean
PROBLEMS := 01 02 03 04 05 06
DIRS := 01_obs_math 02_history_filter 03_obs_layout 04_action 05_timing_contract 06_pipeline
BONUS_DIR := 07_onnx_bonus
VERIFY_DIR := .verify

.PHONY: all clean verify verify-bonus check-stubs bonus 07 $(PROBLEMS)

all: $(PROBLEMS)

01:
	@$(MAKE) --no-print-directory -C 01_obs_math run $(if $(Q),Q="$(Q)")
02:
	@$(MAKE) --no-print-directory -C 02_history_filter run $(if $(Q),Q="$(Q)")
03:
	@$(MAKE) --no-print-directory -C 03_obs_layout run $(if $(Q),Q="$(Q)")
04:
	@$(MAKE) --no-print-directory -C 04_action run $(if $(Q),Q="$(Q)")
05:
	@$(MAKE) --no-print-directory -C 05_timing_contract run $(if $(Q),Q="$(Q)")
06:
	@$(MAKE) --no-print-directory -C 06_pipeline run $(if $(Q),Q="$(Q)")

bonus 07:
	@$(MAKE) --no-print-directory -C $(BONUS_DIR) run $(if $(Q),Q="$(Q)")

# 정답 검증: test.cpp 는 그대로 두고 quiz.h 만 solutions/ 의 것으로 바꾼 사본 트리(.verify/)를 만들어 빌드한다.
# (test.cpp 가 "quiz.h" 를, 뒤 문제의 quiz.h 가 "../01_obs_math/quiz.h" 를 상대 경로로 include 하므로
#  폴더 배치를 그대로 복사해야 정답끼리 맞물린다.) -Werror 로 경고도 실패로 본다.
verify:
	@rm -rf $(VERIFY_DIR) && mkdir -p $(VERIFY_DIR)
	@cp -r common $(VERIFY_DIR)/
	@for d in $(DIRS); do \
		mkdir -p $(VERIFY_DIR)/$$d && cp $$d/test.cpp $$d/Makefile $(VERIFY_DIR)/$$d/ && cp solutions/$$d/quiz.h $(VERIFY_DIR)/$$d/ || exit 1; \
	done
	@for d in $(DIRS); do \
		printf '── verify %s ── ' $$d; \
		$(MAKE) --no-print-directory -C $(VERIFY_DIR)/$$d run WERROR=1 > $(VERIFY_DIR)/$$d.log 2>&1 \
			&& tail -n 1 $(VERIFY_DIR)/$$d.log \
			|| { echo "FAILED"; cat $(VERIFY_DIR)/$$d.log; exit 1; }; \
	done
	@echo "verify: 전 문항 통과"

# 07 은 ONNX Runtime 이 필요해서 따로 둔다 (dev container 안에서만 돈다).
verify-bonus:
	@rm -rf $(VERIFY_DIR)/$(BONUS_DIR) && mkdir -p $(VERIFY_DIR)/$(BONUS_DIR)
	@cp -r common $(VERIFY_DIR)/
	@cp -r $(BONUS_DIR)/test.cpp $(BONUS_DIR)/Makefile $(BONUS_DIR)/CMakeLists.txt $(BONUS_DIR)/models $(VERIFY_DIR)/$(BONUS_DIR)/
	@cp solutions/$(BONUS_DIR)/quiz.h $(VERIFY_DIR)/$(BONUS_DIR)/
	@printf '── verify %s ── ' $(BONUS_DIR)
	@$(MAKE) --no-print-directory -C $(VERIFY_DIR)/$(BONUS_DIR) run WERROR=1 > $(VERIFY_DIR)/$(BONUS_DIR).log 2>&1 \
		&& tail -n 1 $(VERIFY_DIR)/$(BONUS_DIR).log \
		|| { echo "FAILED"; cat $(VERIFY_DIR)/$(BONUS_DIR).log; exit 1; }

check-stubs:
	@./tools/check_stubs.sh

clean:
	@for d in $(DIRS) $(BONUS_DIR); do $(MAKE) --no-print-directory -C $$d clean; done
	@rm -rf $(VERIFY_DIR)
