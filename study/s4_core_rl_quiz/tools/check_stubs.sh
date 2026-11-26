#!/usr/bin/env bash
# 퀴즈 제작자용 점검: "아직 안 푼" quiz.h 로 문항을 하나씩 돌려서
#   - 컴파일이 되고
#   - 비정상 종료(segfault 등) 없이 끝나고
#   - [FAIL] 이 적어도 하나 나오는지 (= 안 풀었는데 통과하는 문항이 없는지)
# 확인한다. quiz.h 를 풀기 시작한 뒤에는 의미가 없다 (푼 문항이 "통과함" 으로 잡힌다).
set -u
cd "$(dirname "$0")/.."

status=0
for dir in 0[1-6]_*/; do
    dir=${dir%/}
    questions=$(sed -n 's/^ALL_Q := //p' "$dir/Makefile")
    for q in $questions; do
        out=$(make --no-print-directory -C "$dir" run Q="$q" 2>&1)
        code=$?
        pass=$(grep -c '\[PASS\]' <<<"$out")
        fail=$(grep -c '\[FAIL\]' <<<"$out")
        if ! grep -q '^결과:' <<<"$out"; then
            verdict="비정상 종료 또는 컴파일 실패 (exit $code)"
            status=1
        elif [ "$fail" -eq 0 ]; then
            verdict="안 풀었는데 통과함"
            status=1
        else
            verdict="ok"
        fi
        printf '%-20s Q%s  PASS %3d  FAIL %3d  %s\n' "$dir" "$q" "$pass" "$fail" "$verdict"
        [ "$verdict" = "ok" ] || sed 's/^/    | /' <<<"$out" | tail -n 15
    done
    make --no-print-directory -C "$dir" clean
done
[ "$status" -eq 0 ] && echo "check-stubs: 전 문항이 FAIL 을 낸다 (비정상 종료 없음)" || echo "check-stubs: 문제 있음"
exit $status
