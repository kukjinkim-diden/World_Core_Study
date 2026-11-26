// 퀴즈 공용 테스트 헬퍼. 수정할 필요 없음.
#pragma once

#include <cstdio>

inline int g_pass = 0;
inline int g_fail = 0;

#define SECTION(name) std::printf("\n== %s ==\n", name)

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (cond) {                                                          \
            ++g_pass;                                                        \
            std::printf("  [PASS] %s\n", #cond);                             \
        } else {                                                             \
            ++g_fail;                                                        \
            std::printf("  [FAIL] %s  (%s:%d)\n", #cond, __FILE__, __LINE__); \
        }                                                                    \
    } while (0)

// static_assert 로 검사한 문항은 컴파일이 됐다는 사실 자체가 통과다.
#define COMPILE_TIME_PASS(what)                                  \
    do {                                                         \
        ++g_pass;                                                \
        std::printf("  [PASS] (compile-time) %s\n", what);       \
    } while (0)

inline auto report() -> int {
    std::printf("\n결과: %d 통과 / %d 실패\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
