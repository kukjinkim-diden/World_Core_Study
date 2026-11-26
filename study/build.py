"""섹터별 parts/*.txt 로부터 문제 노트북과 풀이 노트북을 만든다.

    python study/build.py                  # 모든 섹터의 문제/풀이 노트북 생성
    python study/build.py s1               # 이름이 s1 로 시작하는 섹터만
    python study/build.py s1 --run         # 풀이 셀을 순서대로 실제 실행해 검증 (Jupyter 불필요)
    python study/build.py s1 --run --upto 05   # parts 00..05 까지만 실행
    python study/build.py s1 --run --slow  # 'slow' 표시된 셀(학습 등)까지 실행

형식은 DIDEN_World 의 packages/diden-loco/study/build.py (feature/compact-loco-nuitka) 와 같다.
part 파일 안에서 셀은 ``#%% <kind>`` 줄로 시작한다. kind:
  md    마크다운
  code  주어지는 코드 (그대로 보여 줌)
  ex    연습문제: ``#>>`` 와 ``#<<`` 사이가 풀이. 문제 노트북에서는 TODO + NotImplementedError 로 바뀐다.
        ``#>> 힌트`` 처럼 쓰면 힌트가 TODO 옆에 남는다. ``#>>!`` 는 raise 를 넣지 않는다(클래스 본문 등).
  test  채점 셀 (그대로 보여 줌)
kind 뒤에 ``slow`` 를 붙이면(``#%% code slow``) --slow 없이는 --run 에서 건너뛴다.
"""
import json
import sys
import time
import traceback
from pathlib import Path

HERE = Path(__file__).resolve().parent


def parse(text):
    cells, kind, buf = [], None, []
    for line in text.splitlines():
        if line.startswith("#%% "):
            if kind:
                cells.append((kind, "\n".join(buf).strip("\n")))
            kind, buf = line[4:].strip(), []
        else:
            buf.append(line)
    if kind:
        cells.append((kind, "\n".join(buf).strip("\n")))
    return cells


def strip_solution(src):
    out, skipping = [], False
    for line in src.splitlines():
        s = line.strip()
        if s.startswith("#>>"):
            indent = line[: len(line) - len(line.lstrip())]
            class_level = s.startswith("#>>!")
            hint = s[4 if class_level else 3:].strip()
            out.append(f"{indent}# TODO: {hint or '여기를 구현하세요'}")
            if not class_level:
                out.append(f"{indent}raise NotImplementedError")
            skipping = True
        elif s.startswith("#<<"):
            skipping = False
        elif not skipping:
            out.append(line)
    return "\n".join(out)


def keep_solution(src):
    return "\n".join(l for l in src.splitlines() if not l.strip().startswith(("#>>", "#<<")))


def cell(kind, src):
    if kind == "md":
        return {"cell_type": "markdown", "metadata": {}, "source": src + "\n"}
    return {"cell_type": "code", "metadata": {}, "execution_count": None, "outputs": [], "source": src + "\n"}


def notebook(cells):
    return {"cells": cells, "nbformat": 4, "nbformat_minor": 5,
            "metadata": {"kernelspec": {"display_name": "Python 3", "language": "python", "name": "python3"},
                         "language_info": {"name": "python", "pygments_lexer": "ipython3"}}}


def build(sector, args):
    upto = args[args.index("--upto") + 1] if "--upto" in args else None
    parts = sorted((sector / "parts").glob("*.txt"))
    problems, solutions, runnable = [], [], []
    for p in parts:
        for kind, src in parse(p.read_text()):
            base, slow = kind.split()[0], "slow" in kind.split()
            if base == "md":
                problems.append(cell("md", src)), solutions.append(cell("md", src))
                continue
            sol = keep_solution(src) if base == "ex" else src
            problems.append(cell("code", strip_solution(src) if base == "ex" else src))
            solutions.append(cell("code", sol))
            runnable.append((p.name, base, slow, sol))
    (sector / f"{sector.name}.ipynb").write_text(json.dumps(notebook(problems), ensure_ascii=False, indent=1))
    (sector / f"{sector.name}_solutions.ipynb").write_text(json.dumps(notebook(solutions), ensure_ascii=False, indent=1))
    n_ex = sum(1 for _, b, _, _ in runnable if b == "ex")
    print(f"[{sector.name}] wrote {len(problems)} cells ({n_ex} exercises) from {len(parts)} parts")
    if "--run" not in args:
        return
    import os
    os.chdir(sector)  # 노트북과 같은 작업 디렉터리에서 실행
    os.environ.setdefault("MPLBACKEND", "Agg")  # plt.show() 가 창을 띄우며 멈추지 않게
    ns = {"__name__": "__main__"}
    for name, base, slow, src in runnable:
        if upto and name[:2] > upto:
            break
        if slow and "--slow" not in args:
            continue
        t = time.time()
        try:
            exec(compile(src, f"<{name}:{base}>", "exec"), ns)
        except Exception:
            print(f"\n=== FAILED in {name} [{base}] ===\n{src[:800]}\n...")
            traceback.print_exc()
            sys.exit(1)
        if time.time() - t > 5:
            print(f"   ({name} {base} cell took {time.time() - t:.1f}s)")
    print(f"[{sector.name}] ALL SOLUTION CELLS RAN")


def main():
    args = sys.argv[1:]
    prefix = args[0] if args and not args[0].startswith("--") else ""
    sectors = sorted(d for d in HERE.iterdir() if (d / "parts").is_dir() and d.name.startswith(prefix))
    if not sectors:
        sys.exit(f"no sector matching {prefix!r}")
    if "--run" in args and len(sectors) != 1:
        sys.exit("--run 은 섹터 하나만 지정해서 쓴다 (예: python study/build.py s1 --run)")
    for s in sectors:
        build(s, args)


if __name__ == "__main__":
    main()
