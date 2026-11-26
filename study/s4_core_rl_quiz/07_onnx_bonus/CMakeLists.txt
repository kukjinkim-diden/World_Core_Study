# 07 보너스 전용. 직접 부르지 말고 `make bonus` 로 돌린다 (Makefile 이 이 파일을 쓴다).
#
# 01 ~ 06 은 g++ 한 줄로 빌드하지만, dev container 의 ONNX Runtime 은 정적 라이브러리라
# protobuf · re2 · abseil 을 함께 링크해야 한다. 그 목록을 손으로 적는 대신 레포와 같은 방법
# (find_package(onnxruntime) → onnxruntime::onnxruntime) 을 쓴다.
# 레포: DIDEN_Core/src/engine/inference/CMakeLists.txt
cmake_minimum_required(VERSION 3.22)
project(s4_onnx_bonus LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(onnxruntime REQUIRED)

# QUIZ_Q: 검사할 문항 번호 목록 (예: "1;3"). Makefile 이 Q 를 넘겨 준다.
set(QUIZ_Q "1;2;3" CACHE STRING "검사할 문항")
set(QUIZ_WERROR OFF CACHE BOOL "경고를 에러로")

add_executable(quiz test.cpp)
target_include_directories(quiz PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../common)
target_compile_options(quiz PRIVATE -Wall -Wextra -g -O0)
if(QUIZ_WERROR)
    target_compile_options(quiz PRIVATE -Werror)
endif()
foreach(q IN LISTS QUIZ_Q)
    target_compile_definitions(quiz PRIVATE TEST_Q${q})
endforeach()
target_link_libraries(quiz PRIVATE onnxruntime::onnxruntime)
