# 핵심 모듈 지도

DIDEN_World, DIDEN_Core, DIDEN_Asset 세 레포에서 "무엇이 어디에 있고 어떤 순서로 불리는가" 를 정리한 문서입니다.

- 기준 커밋: World `af709b058`, Core `386bae3a8`, Asset `f76cb0e` (이 레포의 서브모듈이 가리키는 커밋).
- 만든 방법: 코드를 **읽어서** 정리했습니다. 실행해서 확인한 것은 섹터 0·1 노트북에 쓰인 부분(Asset 의 MJCF, MuJoCo/MuJoCo-Warp 동작)뿐입니다. 줄 번호는 위 커밋 기준이고, 코드가 바뀌면 어긋납니다.
- 경로 표기: World 의 패키지 경로는 `packages/<패키지>/src/<모듈>/` 를 줄여 모듈 이름부터 씁니다. 예: `diden_env/managers/reward_manager.py` = `DIDEN_World/packages/diden-env/src/diden_env/managers/reward_manager.py`.

---

## 1. 세 레포의 관계

```
 DIDEN_Asset ── robot.xml, object/*.xml ──┬──────────────────────────────┐
 (MJCF, STL)                              ▼                              ▼
                              DIDEN_World (Python)               DIDEN_Core (C++)
                              시뮬레이션에서 정책 학습            로봇(또는 시뮬레이터)에서 실행
                                          │                              ▲
                                          └── merged ONNX + policy_config ┘      (SEAM 3)
                                          ┌── 기록(HDF5) → World 형식 ────┘
                                          ▼
                                   sim 과 real 비교
```

| | World | Core |
|---|---|---|
| 언어 | Python (torch, mujoco-warp) | C++20 (Eigen, MuJoCo C API, ONNX Runtime) |
| 시뮬레이션 | GPU, 환경 4096 개, 실시간보다 훨씬 빠르게 | CPU, 로봇 1 대, 벽시계 1 kHz |
| 제어기 | 학습되는 신경망 정책 | RL 정책 실행, NMPC 보행, 관절 궤적 추종 |
| 로봇 모델 | `assets/DIDEN_Asset` 서브모듈 | `rsc/diden_asset` 서브모듈 |
| 설정 | 파이썬 `@configclass`, `task_configs/*.yaml` | `config/<robot>/*.yaml` (실행파일에 내장) |

두 레포가 반드시 일치해야 하는 것(§5)이 유지보수에서 가장 조심할 부분입니다.

---

## 2. DIDEN_Asset

| 경로 | 내용 |
|---|---|
| `robot/<robot>/robot.xml` | 로봇 MJCF. 변형은 `robot_<variant>.xml`. 같은 이름의 `.urdf`, `STL/` |
| `robot/common/{material,geom,joint,actuator}.xml` | default class 정의만 담은 include 파일. 모터 종류별 `armature`, 토크 한계가 여기 있다 |
| `object/{sphere,box,...}.xml`, `object/common.xml` | 씬에 놓는 구조물. `contype=1, conaffinity=0`, 밀도 0.01 |
| `texture/*.png` | 재질 |
| `docs/claude/diden_spider_convention.md` | Spider 모델 규약 (토폴로지, 좌표계 원점, 충돌, EPM, 관성, URDF↔MJCF 일치) |

`robot/diden_spider_v2/robot.xml` 요약 (562 줄):

- `<option timestep="0.0005" integrator="implicitfast" noslip_iterations="20">`, `<compiler angle="radian">`
- 트리: `base`(freejoint `root`) → `Torso`(17.6 kg) → 다리 4 개(RR, RL, FR, FL 순) + RB3 매니퓰레이터. 전체 57.5 kg, `nq=37`, `nu=18`.
- 다리 하나: `Leg_X_Link_01`(Joint_01, z축) → `Link_02`(Joint_02, x축) → `Link_03`(Joint_03, x축) → `Foot_X_Link_01/02/03`(수동 yaw/pitch/roll).
- 액추에이터: 토크 `<motor>` 18 개 (다리 12, 팔 6). 게인 없음.
- 센서: `accelerometer`/`gyro` (site `Torso_IMU_site_IAM`), 관절별 `jointpos`/`jointvel`/`jointactuatorfrc`.
- 자석: `Torso_EPM_*_collision`, `Body_*_EPMForcePoint_01`, 발마다 `Foot_X_EPMForcePoint_01~04` (힘 작용점용 body).
- 키프레임: `zero_pose`, `sit_pose`, `stand_pose`.

**주의**: Asset 의 `README.md` 는 일부가 실제 구조와 다릅니다(`scene/`, `texture/common/` 은 없고, `diden_spider_legacy/` 는 실제로 `robot/diden_spider/`).

→ 직접 만들어 보기: **섹터 0** 전체.

---

## 3. DIDEN_World

### 3.1 패키지와 의존 방향

위에 있는 패키지가 아래 것을 import 할 수 있고 반대는 금지입니다(`pyproject.toml` 의 `[tool.importlinter]`, `tests/guards/test_package_dag.py`).

```
diden_configs      작업(task) 정의, 로봇별 MDP 설정, sim profile, 등록
diden_rl           PPO 등 학습 알고리즘 ("frozen island" — 거의 고치지 않는 부분)
diden_isaaclab ┐
diden_raisim   ├   엔진별 구현 (서로 import 금지)
diden_mjlab    ┘   ← 기본 엔진. mjlab / mujoco_warp 를 import 하는 유일한 패키지
diden_env          매니저, 환경 뼈대, 엔진과 무관한 MDP 함수, 배포용 직렬화
diden_domain       자석·접촉·보행 등 도메인 로직
diden_common       설정 기반, SimAdapter 인터페이스, 수학, 로봇 상수
tools/             (패키지 밖) dw CLI: train, play, deploy, fleet, sky
```

### 3.2 세 개의 SEAM (경계)

| | 무엇과 무엇 사이 | 실체 | 지키는 테스트 |
|---|---|---|---|
| SEAM 1 | 환경 → 학습 | `step()` 의 5-튜플 `(obs, reward, terminated, timeout, extras)` — `diden_common/contract.py` | `tests/contract/test_seam1_5tuple` |
| SEAM 2 | 환경 → 엔진 | `SimAdapter` — `diden_common/sim_adapter.py`, 포트 6 개는 `sim_adapter_ports.py` | `tests/contract/test_sim_adapter_protocol`, `tests/guards/` |
| SEAM 3 | World → Core | merged ONNX + `policy_config` 메타데이터 | `tests/contract/test_seam3_policy_config` |

`SimAdapter` 의 포트: `RootStatePort`(몸통 자세·속도), `BodyQueryPort`(링크 위치), `JointControlPort`(관절 상태·목표), `TerrainPort`, `SimWritePort`(상태·외력·모델 값 쓰기), `ContactPort`(접촉). MuJoCo-Warp 구현체는 `diden_mjlab/sim_adapter.py` 의 `MujocoWarpSimAdapter`. 읽기는 스텝 단위로 캐시됩니다(`diden_common/sim_state_cache.py`).

### 3.3 `dw train` 이 부르는 것

1. `tools/cli.py: main` → `tools/engines/mjwarp/train.py: main`
2. 작업 찾기: `tools/task_config_cli.py: resolve_task_or_config` — YAML 이름이면 `diden_configs/task_config/loader.py`, 숫자면 등록된 gym id 를 정렬한 순번(`tools/task_utils.py`)
3. 환경 설정 만들기: `env_cfg_for` → 등록 정보는 `diden_configs/registry/{spec,register,manifest}.py`
4. 러너 설정: `tools/rl_support.py: resolve_runner_cfg` → `diden_configs/robots/diden_spider/agents/diden_rl_ppo_cfg.py`
5. 환경 생성: `MujocoWarpMagnetEnv` (`diden_mjlab/env/mjwarp_magnet_env.py`)
   - `ManagerBasedEnv.__init__` (`diden_env/env/manager_based_env.py`): `create_scene` → `EventManager` → `launch_simulation_runtime`
   - `MujocoWarpEnv.create_scene` (`diden_mjlab/env/mjwarp_env.py`): `MujocoWarpScene` → 컴파일 → 환경별 모델 필드 확장
   - `load_managers` (`diden_env/env/manager_based_rl_env.py`): 커스텀 매니저 → Command → Action/Observation → Termination → Reward → Curriculum
6. 학습: `OnPolicyEstimatorRunner.learn` (`diden_rl/runners/on_policy_runner.py`) → `_collect_experience`(`alg.act` → `env.step` → `process_env_step`) → `compute_returns` → `alg.update`

**주의**: `dw train 1` 의 "1" 이 가리키는 작업은 설치된 엔진에 따라 달라집니다(RaiSim 이 설치되어 있으면 정렬 순서가 바뀜). `dw task` 로 확인하거나 작업 id 를 직접 씁니다.

### 3.4 한 스텝의 순서

`ManagerBasedRlEnv.step` (`diden_env/env/manager_based_rl_env.py`):

1. 캐시 무효화
2. `action_manager.process_action`
3. `perform_physics_step` (`diden_mjlab/env/mjwarp_env.py`) — `decimation` 번: 외력 지우기 → (지연된 서브스텝에서) `apply_action` → `physics_step` 이벤트 → `sim.step` → `scene.update` → `on_physics_substep`
4. 센서 지연 노이즈 재추출, 카운터, regime 마스크
5. 커스텀 매니저 `advance` (locomotion → contact)
6. `termination_manager.compute`
7. `reward_manager.compute`
8. 리셋: 시간 초과 환경의 terminal obs 저장 → `_reset_idx` → `post_reset_sim_sync`
9. `command_manager.compute`
10. `observation_manager.compute`

시간: Spider 는 `dt=0.001`, `decimation=10` → 정책 100 Hz. 에피소드 40 초, 환경 4096 개 (`diden_configs/sim_profiles/mujoco_warp/diden_spider.py`, `robots/diden_spider/magnet_locomotion_env_cfg.py`).

→ 직접 만들어 보기: **섹터 1 Part 11**.

### 3.5 매니저

모든 term 설정은 `ManagerTermBaseCfg(func, params, curriculum, regime)` 를 상속합니다(`diden_common/manager_term_cfg.py`). func 이 함수인지 클래스인지는 `ManagerBase._resolve_common_term_cfg` 가 처리합니다(`diden_env/managers/manager_base.py`).

| 매니저 (`diden_env/managers/`) | 하는 일 | 섹터 1 |
|---|---|---|
| `ActionManager` | 액션 벡터를 term 별로 나눔. 서브스텝마다 `apply_action` | Part 6 |
| `ObservationManager` | 그룹(actor/critic), 노이즈, clip, EMA, scale, 이력 | Part 7 |
| `EventManager` | mode: prestartup / startup / reset / physics_step | Part 10 |
| `RewardManager` | `(Σ add) × (Π mul)`, 항마다 `w·raw·dt → transformation` | Part 8 |
| `TerminationManager` | 항들의 OR. `time_out=True` 인 항은 truncated 쪽 | Part 9 |
| `CommandManager` | 타이머로 명령 재추출 | Part 5 |
| `CurriculumManager` | 리셋 때 또는 iteration 단위로 난이도 조정 | — |
| `LocomotionManager` (커스텀) | 보행 위상 → 발 궤적 | 섹터 2 |
| `ContactManager` (커스텀) | 접촉 센서 구동 | 섹터 2 |

### 3.6 Spider 자석 보행 MDP (`DidenSpiderMagnet-Locomotion-v0`)

정의는 `diden_configs/robots/diden_spider/magnet_locomotion_mdp.py` 한 파일에 있습니다.

**관측** — 그룹 이름은 `actor`, `critic`.

| actor 항 | 노이즈 | 비고 |
|---|---|---|
| `imu_ang_vel` | ±0.05, 에피소드 편향 ±0.3 | EMA, 지연 그룹 `imu` |
| `projected_gravity` | 기울기 ±0.05 rad, 편향 ±0.08 rad | EMA |
| `dof_pos`, `dof_vel` | ±0.05, ±0.5 | 모터 관절 12 개, 지연 그룹 `encoder` |
| `relative_foot_pos` | ±0.06 | 몸통 좌표계 |
| `gait_phi` | — | 발마다 cos/sin |
| `dof_vel_history`, `dof_error_history` | | 이력 3, 간격 4 |
| `previous_joint_actions`, `previous_magnetic_action` | — | 이력 2 |
| `command` | — | |

- scale, clip 을 쓰는 항은 없습니다. 정규화는 네트워크 입구에서 합니다.
- `critic` = actor 항들의 노이즈 없는 버전 + `base_lin_vel`, `feet_contact_indicator`, `feet_height`, `magnetic_force_state`. 이 네 개가 **상태 추정기의 학습 목표** 이기도 합니다.
- 정책 네트워크의 실제 입력은 actor 관측 + 추정기 출력입니다(`diden_rl/runners/on_policy_estimator_runner.py`).

**액션**

- `joint_pos`: `raw × scale + offset` → PD 목표 (`diden_env/mdp/actions/joint_actions.py`). `scale = 0.2 rad` (`ACTION_SCALE`), `offset = DEFAULT_JOINT_ANGLES_XML` (`diden_common/robots/diden_spider/constants.py`).
- `magnet` (4): `tanh` → 자석 상태기계 (`diden_domain/mdp/actions/magnet_actions.py`).
- 액추에이터: MuJoCo 내장 position 액추에이터, kp=100, kd=20, 토크 한계 200 N·m (`diden_mjlab/robots/diden_spider/cfg.py`).
- 액션 지연: 에피소드마다 3~5 ms + 스텝마다 ±2 ms.

**보상** — add 항 3 개: `tracking_lin_vel`(100), `tracking_ang_vel`(75), `gait`(100). 나머지 20 여 개는 전부 mul 항이고 `small_exp` 로 배율이 됩니다: 발 접촉 속도·떨림·미끄럼, 스윙 궤적 추종, 몸통 각속도·수직 속도·높이·기울기, 관절 속도/위치/가속도/저크, 액션 평활도(1·2차), 토크, 자석 관련 6 개. 일부 가중치는 iteration 에 따라 커집니다.

**종료**: `time_out`, NaN, 몸통 속도 초과, 금지 부위 접촉, 기울기 60° 초과.

**이벤트**: `physics_step` — 자석 상태기계, 관절 stick-slip 마찰. `reset` — 관절 마찰, IK 리셋 자세, PD 게인, 로터 관성, 마찰, 발 강성/감쇠, 링크 질량, 무게중심. startup/interval/push 는 쓰지 않습니다.

**명령**: `UniformValidPosturalVelocityCommand`, 3~6 초마다 재추출, ±0.15 m/s, ±0.24 rad/s.

**주의 — 설정값이 나중에 덮어써지는 곳**: 액션 스케일(설정 0.4 → 실제 0.2), PD 게인 랜덤화(설정 25~35 → 실제 100/20 의 ±10 %). 그리고 `ObservationManager` 의 docstring 에 적힌 처리 순서는 코드와 다릅니다(코드: compute → noise → clip → EMA → scale → history).

### 3.7 씬과 시뮬레이션 (`diden_mjlab`)

- `scene/scene.py`: `MjSpec.from_file(scene.xml)` → 지형 attach → 엔티티마다 `attach(prefix="<이름>/")` → `compile()`
- `robots/diden_spider/spec.py`: `robot.xml` 을 읽어 바닥·키프레임·모터 삭제, 발목 `armature` 설정, 매니퓰레이터 용접
- `scene/terrains/terrain_importer.py`: plane / generator / longi / wall / accesshole
- 병렬 환경: 모델 하나를 mujoco_warp 가 `nworld = num_envs` 로 배치. `env_origins` 는 리셋 기준점으로만 사용
- 접촉: `diden_domain/contact/native_contact_sensor.py` (mjlab 의 접촉 센서 백엔드는 의도적으로 비어 있음)
- 자석: 발마다 4 상태(Detached / AirOn / Weak / Strong), `diden_domain/mdp/events/magnetic.py`. 힘 = 크기 × 접촉 법선, `EpmPointSet` 의 점들에 외력으로 가함. Strong 2000 N
- sim profile (`diden_configs/sim_profiles/mujoco_warp/diden_spider.py`): `dt=0.001`, `decimation=10`, `cone="elliptic"`, `impratio=10`, `nconmax=120`, `njmax=450`

→ 직접 만들어 보기: **섹터 0 Part 8~9**, **섹터 1 Part 1·3**.

### 3.8 학습 (`diden_rl`) 과 배포

| | 파일 | 섹터 1 |
|---|---|---|
| 러너 | `runners/on_policy_runner.py`, `on_policy_estimator_runner.py` | Part 15 |
| 알고리즘 | `algorithms/ppo.py`, `ppo_estimator.py` | Part 14 |
| 저장소·GAE | `storage/rollout_storage.py` | Part 13 |
| 네트워크 | `modules/actor_critic.py`, `state_estimator.py` | Part 12 |
| 관측 정규화 | `modules/utils.py: RunningNormalization` | Part 12 |
| ONNX 내보내기 | `tools/deploy/export_merged_onnx.py` | Part 17 |
| `policy_config` 만들기 | `diden_env/deploy/{serialize,schema,validate,policy_schema}.py` | Part 17 |

Spider 하이퍼파라미터: 은닉층 (512, 256, 64), tanh, 초기 std 3.0, 스텝 32, epoch 5, 미니배치 4, 적응형 학습률(KL 0.01), γ 0.99, λ 0.95.

ONNX 그래프는 `actor(normalize(cat(obs, clamp(estimator(obs)))))`. 메타데이터 키는 `model_kind`, `policy_config`, `provenance`. opset 11, `dynamo=False`.

### 3.9 테스트 (`tests/`)

| 폴더 | 지키는 것 |
|---|---|
| `contract/` | SEAM 3 개의 형태 |
| `guards/` | 정적 검사: 패키지 의존 방향, 엔진 누수, 관측·액션 지문, SI 단위, 수치 안전성 |
| `physics/` | 엔진 물리를 외부 기준 데이터와 비교, EPM 유지력, 마찰 |
| `parity/` | IsaacLab 과의 일치 (Isaac Sim 이 있을 때만) |

### 3.10 읽는 순서 (World)

1. `docs/design/architecture.md`, `docs/guides/code_architecture.md`
2. `diden_common/`: `configclass.py` → `manager_term_cfg.py` → `manager_term_base.py` → `contract.py` → `sim_adapter_ports.py` → `sim_adapter.py`
3. `diden_env/managers/`: `manager_base.py` → `event_manager.py` → `action_manager.py` → `observation_manager.py` → `reward_manager.py` → `termination_manager.py`
4. `diden_env/env/`: `manager_based_env.py` → `manager_based_rl_env.py`
5. `diden_mjlab/`: `env/mjwarp_env.py` → `scene/scene.py` → `robots/diden_spider/{spec,cfg}.py` → `sim_adapter.py`
6. `diden_configs/robots/diden_spider/`: `magnet_locomotion_mdp.py` → `magnet_locomotion_env_cfg.py`, 그리고 `sim_profiles/mujoco_warp/diden_spider.py`
7. `diden_configs/registry/`, `task_config/loader.py`, `tools/cli.py`, `tools/engines/mjwarp/train.py`
8. `diden_rl/`: `runners/on_policy_runner.py` → `algorithms/ppo.py` → `storage/rollout_storage.py` → `modules/actor_critic.py`
9. `diden_domain/magnet/`, `diden_domain/mdp/events/magnetic.py`
10. `diden_env/deploy/serialize.py`, `tools/deploy/export_merged_onnx.py`

---

## 4. DIDEN_Core

### 4.1 계층

| 폴더 | 맡은 일 |
|---|---|
| `apps/` | 로봇별 조립: 설정 로드, 스레드 등록 (`main_diden_spider.cpp`) |
| `applications/` | 제어기(manager, runner), 상태기계, 작업(task), 모션, 시각화 |
| `engine/` | `SystemState`, 추정기, 시뮬레이터, 솔버(기구학/IK), 추론(ONNX Runtime), 기록, 스레딩 |
| `interfaces/` | gRPC 서버 |
| `driver/` | DREC 베이스보드, Rainbow RB3 프로토콜 |
| `hal/` | UDP/TCP 소켓 |
| `common/` | 번들, YAML, 로거, 수학 |

로봇 종류는 `DIDENSPIDER`, `DIDENWALKER` 둘입니다(`include/diden_core/engine/core/robot_config.h`). Humanoid 는 Core 에 없습니다.

### 4.2 스레드 (Spider, `apps/main_diden_spider.cpp`)

| 스레드 | 우선순위 | 주기 |
|---|---|---|
| DREC_Rx / DREC_Tx | RT 99 | 수신 대기 / 500 Hz |
| Controller (manager) | RT 99 | 1 kHz |
| Ctrl_Runner | RT 98 | 1 kHz tick (제어기가 그 중 일부만 사용) |
| Estimator | RT 99 | 1 kHz |
| Simulator | RT 99 | 1 kHz (시뮬레이션 모드에서만) |
| State_Manager | RT 98 | 2 kHz |
| Task_Manager | RT 98 | 100 Hz |
| Motion_Planner | 일반 | 50 Hz |
| RB3_Rx / RB3_Tx | RT 98 | 500 Hz |
| gRPC_Server, Recorder, Visualizer 등 | | |

- 모든 스레드가 전역 `g_system_state` 하나를 공유합니다(`include/diden_core/engine/core/system_state.h`).
- `PeriodicTask` 는 sleep 후 spin 대기로 주기를 맞추고 지터를 기록합니다(`src/engine/threading/periodic_task.cpp`).
- CPU 고정은 `real_robot` 모드에서만 적용됩니다.

### 4.3 시작 경로

`main()` → `ThreadManager` → 신호 처리 등록 → 번들 4 개 mount (config, asset, policy, motion — 실행파일의 ELF 섹션에 내장) → overlay 로드 → `load_config_from_bundle()` (`config/<stem>.yaml`) → 스레드 등록 → `launch()` → `join()`.

→ 기존 퀴즈 `quiz/01~04` (constexpr, std::byte, extern "C", bundle) 가 이 시작 경로의 C++ 기초입니다.

### 4.4 RL 제어 스택

위치: `include/diden_core/applications/controller/runner/rl_controller/` 와 `src/` 의 같은 경로. 파일: `policy`, `observation`, `action`, `filter`, `motion_clip`, `rl_controller`, `config.h`. 추론은 `src/engine/inference/inference_engine.cpp`.

- **로드**: ONNX Runtime, CPU, 입력 1 개·출력 1 개. gRPC `SetRunningController(CONTROLLER_RL, policy_path)` 로 요청되고 실제 로드는 제어 스레드의 `on_enter` 에서.
- **관측**: 정책 YAML 에 적힌 순서대로 항을 만든다. 이름은 World 의 관측 항 이름과 같다. 이력은 접미사 `_history` 또는 `history_length`/`history_interval`. 항별 EMA·노치 필터. **스케일·정규화는 하지 않는다** (ONNX 안에 있음).
  - `projected_gravity` 의 자세는 IMU 가 아니라 **추정기의 상태** 에서 온다 (MuJoCo 의 `xmat` 을 `ModelQuery` 로 읽어 `Rᵀ·(0,0,−1)`).
  - `previous_joint_actions` 는 raw 액션이 아니라 **직전 관절 목표 각도** 다. `history_length` 가 있으면 오래된 것부터, 없으면 최신부터 쌓는다.
  - 이력 버퍼는 이 커밋에서 **0 으로 시작** 한다(헤더 주석은 warm start 라고 적혀 있지만 구현은 아님). World 의 `CircularBuffer` 는 첫 값으로 채우므로 처음 몇 tick 의 관측이 학습 때와 다르다. 필터의 첫 표본 warm start 는 구현되어 있다.
- **액션**: `q = action_mean + joint_action_scale × raw`. 게인은 정책의 `joint_kp`/`joint_kd`. 자석은 `tanh(raw)` → `MagnetActionGate` → ±1.
- **주기**: 정책 주기 = `decimation × physics_dt` (µs 로 바꿀 때 반올림이 아니라 버림). 러너는 tick 을 세지 않고 `steady_clock` 과 `next_step_time` 을 비교해 주기가 찼을 때 실행한다.
- **검증**: `.onnx` 옆에 같은 이름의 `.yaml` 필수. 관측 차원 합(추정기 출력 제외) = ONNX 입력, 액션 차원 합 = ONNX 출력. 두 단계로 검사한다(`Policy::validate_dims`, 그리고 조립된 관측 길이를 `activate_policy` 에서 다시). 합이 같은 채 순서만 바뀐 경우는 잡지 못한다.
- **설정이 오는 곳**: 이 커밋의 Core 는 같은 이름의 `.yaml` 에서만 읽는다. World 의 `policy_schema.py` 주석은 Core 가 ONNX `metadata_props` 에서 읽는다고 적고 있어 서로 다르다 — 팀에 확인할 것.
- **안전**: 여러 겹이다. 필터는 NaN 표본을 삼키고 직전 출력을 낸다 → 추론 입력·출력에 NaN 이 있으면 직전 목표 유지(safe-hold) → 러너는 무효 출력이 10 번 이어지면 제어기를 멈춘다. 정책 전환 시 선형 블렌딩.

→ 직접 만들어 보기: **섹터 1 Part 17** (파이썬 축소판), **섹터 4** (C++ 퀴즈).

### 4.5 상태 추정 (`src/engine/estimator/`)

- 종류: simple, invariant_ekf, complementary, sim_bypass, vision_bypass, vision_complementary. Spider 는 `invariant_ekf`.
- 입력: 관절 q/dq/τ, IMU, 접촉 센서, 정기구학.
- 접촉 판정: 관절 토크로부터 외력 추정(`τ_ext = M q̈ + C q̇ + g − τ` 를 `Jᵀ` 로 되돌림) → 30 N 넘게 0.2 초 유지 → 접촉 센서와 AND.
- InEKF: right-invariant EKF (Hartley 2020). 상태 = IMU 좌표계의 `[R v p | d₁..d_K]` + 자이로/가속도 편향. 접촉 발의 정기구학으로 보정. NIS 게이트, ZUPT.
- 출력 `state` (`system_state.h`): `q`(25 = 위치 3, 쿼터니언 4, 관절 18), `dq`(24), `foot_contact`, `epm_phase`, `epm_state`, `ankle_q`, `support_plane_rot`, `height_from_support_plane`.

### 4.6 경로 계획 — RRT (`applications/task/tasks/motion_tracking_task/rrt_planner/`)

- 변형: 양방향 RRT-Connect + RRT* 식 부모 선택 (재배선 없음, 첫 해에서 종료).
- 상태 공간: 스윙 다리의 관절 3 개 + 몸통 위치 3 + 자세 3. 지지 다리 3 개는 해석적 IK.
- 충돌 검사: MuJoCo `mj_geomDistance`, geom group 3(로봇)과 1(구조물).
- 후처리: elastic band → 경로 복구 → 속도 제한 타이밍.
- 결과가 가는 곳: `JointPathPlanner` → `JointTrajectory` → `JointTrackingController`.

**중요**: RRT 가 만든 경로를 NMPC 가 따라가지 **않습니다**. RRT 는 한 동작(모션 시퀀스)의 관절 경로를 만들고 관절 추종 제어기가 실행합니다. NMPC 쪽으로 가는 것은 작업 공간의 발 목표를 가진 모션(`NmpcMotionPlan`)이나, waypoint 작업의 pure-pursuit 추종기가 만든 **속도 명령** 입니다.

### 4.7 NMPC 보행 (`applications/controller/runner/nmpc_controller/`)

- 모델: 단일 강체(SRBD), SO(3) 위. 상태 18 (R 9, 몸통 각속도, 무게중심 위치·속도), 입력 = 발 4 개의 지면 반력(세계 좌표계).
- 비용: 자세·각속도·위치·속도 오차 + 입력 − 기준(mg/지지 발 수).
- 제약: 마찰 원뿔(μ = 0.15), 자석 붙은 발은 당기는 힘 허용, 관절 토크 한계(`Jᵀf`), 스윙 발 힘 0.
- 풀이: proximal Gauss-Newton 2 회, QP 는 qpSWIFT. 지평 8 × 0.05 초, 100 Hz.
- 보행: 위상 오프셋, 스윙을 PreRelease → Releasing → Travel → Press → Engaging → Stance 로 나눔(자석 펄스에 맞춤). 발 디딤은 Raibert 휴리스틱.
- 관절 수준: `τ_ff = −Jᵀf`. 스윙 다리는 damped least squares IK. 게인은 모드별(swing 700/20, stance 0/100, hold 400/100, press 0/40).
- 전신 제어기(WBC)는 따로 없습니다.

### 4.8 하드웨어 통신 (`driver/`, `hal/`)

- PC ↔ DREC 베이스보드: UDP. `StatusPacket`(`MotorStatus[17]`), `CommandPacket`(`MotorCommand{pos, vel, kp, kd, tau, mf_high/low, led}`). STX/ETX + checksum. packed struct.
- PD 는 모터 드라이버 안에서 돕니다(PC 는 목표와 게인을 보냄). 베이스보드 ↔ 모터 드라이버 구간은 펌웨어가 이 레포에 없어 확인하지 못했습니다.
- RB3 팔: TCP.
- 시뮬레이션 모드: DREC 스레드를 등록하지 않고, Simulator 스레드가 `sensor_raw` 를 쓰고 `cmd` 를 읽어 `τ = kp(q*−q) + kd(dq*−dq) + τ_ff` 를 직접 계산합니다(`src/engine/simulator/thread.cpp`).

### 4.9 Core 안의 MuJoCo (`src/engine/simulator/`)

- 번들의 에셋을 `mjVFS` 에 올림 → `mj_parseXML` → mjSpec 수정(바닥, 센서/액추에이터 에뮬레이션, 구조물 `mjs_attach`) → `mj_compile`.
- 제어기와 동기화하지 않습니다. 벽시계 1 kHz 로 `mj_step` 한 번씩.
- 자석: Detached / Attached / WeakAttached. 힘 = −F·접촉 법선을 `xfrc_applied` 로. 붙었을 때 2000 N.

### 4.10 상태기계, 작업, 기록

- FSM (boost-sml): Booting → Preparing → Ready → Running, 어디서든 Fault / EStop (`applications/state_manager/fsm.h`).
- TaskManager: 추정기 초기화 → 제어기 준비·진입·시작 → 실행 → 종료 → Ready 복귀.
- 조작: `scripts/` 의 grpcurl 스크립트 (`test_rl_locomotion.sh`, `test_nmpc_locomotion.sh`, `task_manager/`).
- 기록: 1 kHz 샘플 → HDF5. 보기는 `tools/log_viewer/` (inekf, nmpc, rl_io, rrt 뷰어).

### 4.11 빌드와 테스트

- CMake 프리셋 `{debug,release}-diden-{spider,spider-ho,walker,walker-lidar}`. debug 만 테스트 ON.
- vcpkg 의존성: eigen3, mujoco, onnxruntime, qpswift, grpc, protobuf, hdf5, manif, boost-ext-sml, reflectcpp, yaml-cpp 등.
- 테스트 6 개: swing_waypoints, keyframe_body, rrt_query_log, inekf_contact_gate, motion_replay, yaml_eigen.

### 4.12 읽는 순서 (Core)

1. `ARCHITECTURE.md`
2. `engine/core/system_state.h`, `engine/models/diden_spider/constants.h`
3. `engine/threading/periodic_task.cpp`, `thread_manager.cpp`, `apps/main_diden_spider.cpp`, `common/bundle/manager.cpp`
4. `hal/ethernet/udp_socket.h`, `driver/drec_baseboard/{rx,tx}/protocol.h`, `rx/thread.cpp`
5. `engine/simulator/thread.cpp`, `simulator.cpp`
6. `engine/estimator/invariant_ekf/filter.h` → `filter.cpp`, `estimator/thread.cpp`
7. `applications/controller/runner/controller_base.h`, `runner/thread.cpp`, `manager/controller_manager.cpp`, `state_manager/fsm.h`
8. `engine/inference/inference_engine.cpp`, `rl_controller/policy.{h,cpp}` → `observation.cpp` → `action.cpp` → `rl_controller.cpp`
9. `nmpc_controller/nmpc_so3_solver.cpp` → `nmpc_controller.cpp`
10. `motion_tracking_task/rrt_planner/rrt_connect.h`, `motion_tracking_task/task.cpp`

**이 체크아웃에서 비어 있는 것**: `proto/`, `rsc/diden_asset/` (초기화되지 않은 서브모듈), `policy/` (`.gitkeep` 만). 그래서 gRPC 메시지 정의와 실제 정책 YAML 예시는 여기서 볼 수 없습니다.

---

## 5. 두 레포가 일치해야 하는 것

| 약속 | World 쪽 | Core 쪽 |
|---|---|---|
| 로봇 모델 | `assets/DIDEN_Asset` 서브모듈 커밋 | `rsc/diden_asset` 서브모듈 커밋 |
| 관측 항의 이름·순서·차원·이력 | `magnet_locomotion_mdp.py` → `policy_config.observations` | `rl_controller/observation.cpp` 의 항 구현 |
| 관측 항의 **계산 방법** (좌표계, 어느 센서) | `diden_env/mdp/observations/` | `observation.cpp` |
| 액션 → 관절 목표 | `raw × ACTION_SCALE + action_offset` | `action_mean + joint_action_scale × raw` |
| PD 게인 | 액추에이터 설정 kp/kd | 정책 YAML 의 `joint_kp`/`joint_kd` → 모터 드라이버 |
| 정책 주기 | `decimation × dt` | `decimation × physics_dt` |
| 관측 정규화 | ONNX 그래프 안 | (하지 않음) |
| 정책 설정의 전달 경로 | ONNX `metadata_props` 의 `policy_config` | 같은 이름의 `.yaml` (이 커밋 기준. 위 §4.4 참고) |
| 이력 버퍼의 첫 값 | 첫 표본으로 채움 | 0 으로 시작 (이 커밋 기준) |
| 자석 명령 문턱값·쿨다운 | 자석 상태기계 설정 | `MagnetActionGate` |
| 관절 순서 | 엔티티의 관절 이름 순서 | `constants.h` |
| 모션 클립 형식 | `tools/motion/export_core_bin.py` | `motion_clip.h` (MOT2) |
| 기록 형식 | MotionLib h5 | Recorder 의 내보내기 |

같은 로봇을 세 가지 시뮬레이터 설정으로 돌린다는 점도 기억해 둡니다.

| | `robot.xml` 그대로 | World | Core 시뮬레이터 |
|---|---|---|---|
| timestep | 0.0005 | 0.001 | 0.001 |
| 미끄럼 대책 | `noslip_iterations=20` | `cone=elliptic`, `impratio=10` | (설정 파일에서 확인 필요) |
| 액추에이터 | 토크 모터 | 내장 position (암시적 감쇠) | 토크 모터 + PD 직접 계산 |
| 동기화 | — | 정책과 물리가 정확히 맞물림 | 제어기와 무관하게 벽시계로 진행 |
