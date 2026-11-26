# 각 문제 폴더의 Makefile 이 include 하는 공통 규칙. 수정할 필요 없음.
CXX      ?= g++
CC       ?= gcc
CXXFLAGS := -std=c++20 -Wall -Wextra -g -O0 -I../common
CFLAGS   := -std=c11 -Wall -Wextra -g -O0

# make verify 가 켠다 — 정답(solutions/)은 경고 하나 없이 빌드되어야 한다.
ifdef WERROR
CXXFLAGS += -Werror
endif

# Q 를 안 주면 전체 문항
Q ?= $(ALL_Q)
QFLAGS := $(foreach q,$(Q),-DTEST_Q$(q))
