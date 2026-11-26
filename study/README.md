# study — DIDEN_World / DIDEN_Core 를 직접 구현하며 이해하기

목표는 두 레포의 핵심 코드를 **손으로 다시 만들어 보는 것** 입니다. 읽기만 해서는 남지 않는 부분(순서, 좌표계, 단위, 경계 조건)이 구현하면서 드러납니다.

## 문서

| 파일 | 내용 |
|---|---|
| [MODULE_MAP.md](MODULE_MAP.md) | 세 레포에서 무엇이 어디에 있고 어떤 순서로 불리는가. 읽는 순서 포함 |
| [QUESTION_TREE.md](QUESTION_TREE.md) | `questions.md` 의 질문 13 개에서 뻗은 질문 트리. 어느 문제가 어느 질문에 답하는지 |

## 섹터

| 섹터 | 주제 | 형식 | 문제 수 | 상태 |
|---|---|---|---|---|
| [s0_mujoco_scene](s0_mujoco_scene/) | MuJoCo 로 씬·오브젝트·로봇 구성하기 | 주피터 노트북 | 26 | 완성, 풀이 검증됨 |
| [s1_ball_env](s1_ball_env/) | 공 굴리기 환경 — World 를 축소판으로 처음부터 (환경 + PPO + ONNX 배포) | 주피터 노트북 | 26 | 완성, 풀이 검증됨 (학습 포함) |
| s2 | World: Spider 보행 MDP (관측·액션·보상·이벤트를 실제 값으로) | 노트북 | | 예정 |
| s3 | World: 학습 알고리즘 심화와 배포 | 노트북 | | 예정 |
| [s4_core_rl_quiz](s4_core_rl_quiz/) | Core: RL 제어 스택 (관측 조립, 이력·필터, 액션, 주기·계약, 전체 tick) | C++ 퀴즈 | 34 + 보너스 3 | 완성, 정답 검증됨 |
| s5 | Core: 상태 추정 (InEKF) | C++ 퀴즈 | | 예정 |
| s6 | Core: RRT 경로 계획 | C++ 퀴즈 | | 예정 |
| s7 | Core: NMPC 보행 | C++ 퀴즈 | | 예정 |
| s8 | Core: 하드웨어 통신·스레드 | C++ 퀴즈 | | 예정 (기존 `DIDEN_Core/quiz/01~05` 가 이 가지의 시작) |

World 쪽은 파이썬 노트북, Core 쪽은 C++ 퀴즈로 나눴습니다. 각 레포의 언어로 구현해야 실제 코드를 고칠 때 그대로 쓸 수 있기 때문입니다. C++ 퀴즈는 dev container 의 `/workspaces/DIDEN_Core/quiz` 와 같은 형식입니다(`make 01`, `make 01 Q=1`, `quiz.h` 만 수정, `GUIDE.md`).

## C++ 퀴즈 (s4)

```bash
cd study/s4_core_rl_quiz
make 01            # 01 의 전체 문항
make 01 Q=1        # 1번 문항만
make verify        # solutions/ 의 정답이 전 문항을 통과하는지 확인
```

호스트의 g++ 와 Core dev container 의 g++ 둘 다에서 빌드됩니다. 보너스(07, 진짜 ONNX Runtime 으로 추론)만 dev container 가 필요합니다. dev container 에서 풀려면 이 폴더를 컨테이너 안으로 복사하거나 마운트합니다. 자세한 것은 [s4_core_rl_quiz/README.md](s4_core_rl_quiz/README.md).

## 노트북 (s0, s1)

### 준비

```bash
cd ~/World_Core_Study
git submodule update --init DIDEN_Asset     # 로봇·오브젝트 MJCF (노트북이 직접 읽는다)
uv sync                                     # .venv 생성. 물리·학습 라이브러리는 DIDEN_World 와 같은 버전
```

VS Code 에서 노트북을 열고 커널로 `.venv` 를 고르거나, `uv run jupyter lab study/` 로 엽니다.

- s0 는 CPU 만 씁니다.
- s1 은 NVIDIA GPU 가 필요합니다. 첫 실행 때 Warp 가 물리 커널을 컴파일하느라 1~2 분 걸립니다. 학습은 RTX 5070 에서 약 2 분입니다.

### 파일

```
s1_ball_env/
  s1_ball_env.ipynb             ← 여기에 푼다 (# TODO 를 채운다)
  s1_ball_env_solutions.ipynb   ← 풀이. 30 분 넘게 막혔을 때만
  parts/*.txt                   ← 두 노트북의 원본
```

### 푸는 법

1. `# TODO` 가 있는 셀에서 `raise NotImplementedError` 를 지우고 본문을 직접 칩니다.
2. 바로 아래 채점 셀을 실행합니다. ✅ 면 다음으로, ❌ 면 메시지를 읽고 고칩니다.
3. "먼저 예상해 보기" 는 실행 전에 답을 적어 봅니다.
4. Part 끝의 "실제 코드에서는" 을 읽고, 거기 적힌 파일을 열어 방금 만든 것과 비교합니다.
5. "부숴 보기" 는 일부러 망가뜨려 결과를 예상하고 확인하는 실험입니다. 유지보수 감각은 여기서 가장 많이 생깁니다.

노트북은 위에서 아래로 이어집니다. 앞 Part 에서 만든 함수를 뒤에서 씁니다.

### 문제를 고치거나 추가할 때

노트북을 직접 고치지 말고 `parts/*.txt` 를 고친 뒤 다시 만듭니다. **다시 만들면 문제 노트북이 덮어써지므로, 풀던 노트북은 먼저 다른 이름으로 복사해 둡니다.**

```bash
uv run python study/build.py                 # 모든 섹터의 문제/풀이 노트북 생성
uv run python study/build.py s1 --run        # s1 의 풀이 셀을 순서대로 실행해 검증 (학습 등 느린 셀 제외)
uv run python study/build.py s1 --run --slow # 학습까지 포함 (약 5 분)
```

`parts/*.txt` 의 형식(`#%% md | code | ex | test`, 풀이 구간 `#>>` ~ `#<<`)은 `build.py` 맨 위에 적혀 있습니다. DIDEN_World 의 `packages/diden-loco/study/` (브랜치 `feature/compact-loco-nuitka`) 와 같은 형식입니다.

## 이 노트북들이 실제 코드와 다른 점

구조와 이름은 실제 코드를 따랐지만 내용은 줄였습니다. 무엇을 뺐는지는 각 노트북의 첫 셀과 Part 끝에 적어 두었습니다. s0·s1 의 채점은 해석해(물리 공식)나 MuJoCo 자체의 계산과 비교합니다. World 의 함수와 출력을 직접 비교하는 채점은 s2 부터 넣을 예정입니다(그러려면 `DIDEN_World` 를 `uv sync --all-packages` 로 설치해야 합니다).
