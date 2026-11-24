# 시뮬레이터와 학습·평가 코드 설계

이 문서는 저장소의 코드가 어떻게 나뉘고 서로 어떻게 맞물리는지 설명합니다. 관측, 서브골, 메시지의 세부 값은 [interface.md](interface.md), Unity에서 여는 방법은 [unity_setup.md](unity_setup.md)에 있습니다.

## 1. 전체 구조

```text
학습  mlagents-learn configs/stageN.yaml ──gRPC──> Unity 빌드 (planner = random, 플래너 채널 침묵)
                                                    방 16개 × MosquitoAgent

평가  evaluate.py ──gRPC + PlannerChannel──> Unity 빌드 (planner = external)
        │  OnnxPolicy (학습된 정책, Python에서 실행)
        │  EvalLoop  (요청 처리, 지연 주입, 로그)
        └─HTTP──> planner_server (VLM: transformers | vLLM | mock)
```

- 학습에는 VLM이 없습니다. 무작위 플래너가 Unity 안에서 서브골을 고르고, ML-Agents가 배속 20으로 PPO를 돌립니다.
- 평가에서는 모든 조건(서브골 고정, 마스킹 규칙, 정답 규칙, VLM, 이미지 제외 VLM)이 같은 경로를 씁니다. Unity는 결정이 필요할 때 요청을 보내고, Python이 조건에 맞는 지시를 돌려줍니다. 서브골 고정과 정답 규칙의 지시는 Unity가 요청에 넣어 보내고, 마스킹 규칙은 Python이 요청의 `view`만 보고 계산하므로, 이 세 조건은 VLM 없이 지연 0으로 돌아갑니다.
- 하위 정책은 평가 때 Python에서 ONNX로 돌립니다(`OnnxPolicy`). Unity는 매 결정마다 Python을 기다리므로, VLM이 계산하는 동안 시뮬레이션은 저절로 멈춥니다.

## 2. Unity 시뮬레이터

### 씬

메뉴 **MosquitoSim → Build Main Scene**이 `Assets/Scenes/Main.unity`를 만듭니다. 한 씬으로 모든 단계와 환경 분할을 돌리고, 방 모양과 가구는 에피소드마다 런타임에 만듭니다.

```text
Global      SimSettings(물리 0.01초), PlannerChannelRegistrar
Room_00..15 (12 m 간격)
  RoomArea + RoomGenerator + ObstacleSpawner + CandidatePoints + CueObjects
  ├─ Net            NetObstacle (eval_net에서만 켜짐)
  ├─ Human          HumanController, HandController, 몸 5개 + 피부 S1~S5 (레이어 8)
  ├─ Hand           손 구(충돌체 없음)
  ├─ PlannerCameras 모기 시점 1 + 전경 4 (꺼져 있다가 요청 때만 렌더)
  └─ Mosquito       MosquitoMotor, MosquitoAgent, BehaviorParameters(관측 59, 행동 3), DecisionRequester(5)
```

### 스크립트

| 파일 | 역할 |
|---|---|
| `Env/SplitConfig.cs` | 환경 파라미터 읽기(`EnvParams`), 분할·방 종류, 평가 시드 |
| `Env/RoomGenerator.cs` | 방 껍데기: 4×3, 5×4, ㄱ자 / 6×3, 칸막이 5×5 |
| `Env/ObstacleSpawner.cs` | 에피소드마다 방 모양에 맞는 배치 하나를 골라(침실, 거실, 식당, 서재, 원룸 / 세트 B: 주방, 욕실, 주방+욕실) 단계별로 놓음. 각 단계는 벽이나 앞 단계 가구 기준: 침대 머리는 벽, 옆에 협탁, 소파 앞 커피 테이블과 마주 보는 안락의자, 식탁 둘레 의자, 책상 앞 의자, 소파·침대 위 쿠션, 옷장은 모서리. 선택 단계, 벽, 모델 변형, 위치는 에피소드 RNG. 사람 시작 위치·걷는 경로·문 앞은 비워 둠. 가구 팩이 없으면 예전 도형 가구 |
| `Env/RoomDecor.cs`, `Env/RoomPlacement.cs` | 방 구조(문, 창문 1~2개와 커튼, 액자, 러그, 1단계부터 기둥)와 1단계부터 장애물 3~6개(펜던트 조명·매단 화분 = 공중 장애물, 벽 선반과 책, 방을 가로지르는 빨랫줄과 수건, 스탠드, 화분, 상자 더미). 도형으로 만들어 풀링. 방 크기는 형태별 기본값에서 축마다 -8~+15 %, 벽·바닥 색은 팔레트에서(가끔 포인트 벽) |
| `Env/FurnitureItem.cs`, `Editor/FurnitureCatalog.cs` | Furniture Mega Pack 프리팹을 방마다 다른 모델로 미리 넣어 둠(비활성). 팩의 크기가 들쭉날쭉해 종류별 표준 크기(예: 탁자 높이 0.75 m)로 맞추고, 충돌체(메시 콜라이더)를 붙임 |
| `Env/HumanAvatar.cs` | Mixamo 캐릭터: 뼈에 피부 영역·몸 충돌체, 서기↔걷기 블렌드, 180도 회전, 경계 시 머리 돌리기, 휘두르기 손 IK |
| `Env/HumanController.cs` | 서 있기 / 걷기, 경계(윙윙 0.4초 또는 흡혈 중 확률, 3~5초 제자리에서 ±60° 훑기), 발견(시야각 ±60° + 시선), 가시성 |
| `Env/HandController.cs` | 휘두르기: 발견 + 0.9 m 안 → 반응 지연 → 직선 스윙 → 쿨다운, 6 cm 안이면 잡힘. EVADE 점검용 강제 휘두르기 |
| `Env/NetObstacle.cs` | 모기장 1.4×1.4×2.0 m, 한쪽 열림 |
| `Env/CueObjects.cs` | 피부 덮개(착지 불가), 끈끈이 트랩(닿으면 실패) |
| `Env/CandidatePoints.cs` | 0.25 m 격자, 결정마다 후보 P1~P8과 기하 정답 |
| `Env/RoomArea.cs` | 에피소드 리셋 순서 관리, 빈 지점 샘플링 |
| `Agent/MosquitoMotor.cs` | 직접 적분 + SphereCast, 착지·이륙, 트랩 접촉 |
| `Agent/ObsLayout.cs`, `ObsBuilder.cs` | 관측 59차원 |
| `Agent/SubgoalTracker.cs` | 지시의 목표 지점, 보상 성형, 달성·시간 초과 |
| `Agent/Planners.cs` | 고정, 규칙, 무작위 플래너 |
| `Agent/StateDescriber.cs` | 플래너 뷰(`PlannerView`)와 그 뷰로 만든 VLM 지시문 |
| `Agent/PlannerCameras.cs` | 5대 렌더, ID 라벨(Set-of-Mark), PNG 저장 |
| `Agent/PlannerChannel.cs` | Python과 주고받는 사이드 채널 |
| `Agent/MosquitoAgent.cs` | 위 요소를 묶는 에이전트 |
| `Editor/SceneBuilder.cs` | 씬 생성 메뉴 |

### 에피소드 리셋 순서

1. 환경 파라미터를 읽고 시드를 정합니다. 학습은 매번 새 시드, 검증·평가는 `(eval_seed, 분할, 방, 방의 에피소드 번호)`로 정해, 모든 조건이 같은 배치를 받습니다.
2. 학습이면 20% 확률로 이전 단계 설정을 씁니다(`prev_stage_mix`).
3. 방 모양 → 사람 위치(2단계부터 걷는 경로) → 모기장 → 가구(사람에게서 0.8 m, 모기장이면 1.3 m 떨어뜨림) → 단서 물체 → 손 → 전경 카메라 위치 → 후보 격자 순서로 만듭니다.
4. 모기를 사람에게서 1 m 이상 떨어진 빈 공중에 놓고, 첫 지시로 가장 가까운 피부로의 APPROACH를 줍니다. 곧바로 `episode_start` 사건으로 플래너를 부릅니다.

### 한 고정 스텝(0.01초)의 순서

1. Python에서 온 지시를 적용(현재 에피소드의 것만)
2. 모터 이동. APPROACH이고 목표 피부가 덮이지 않았을 때만 그 피부에 착지 가능
3. 트랩에 닿으면 −1, 종료
4. 충돌 −0.01, 시간 −0.005/초
5. 2단계부터 사람과 손 갱신. 잡히면 −1, 종료. 경계 시작·끝, 손 활성 사건 기록
6. 목표 지점 갱신과 보상 성형(거리 변화 × 0.1, 회피는 멀어질수록), 서브골 달성(+0.3)·시간 초과(접근이면 그 피부 제외) 사건 기록
7. 착지 중이면 흡혈 시간을 세고, `feed_time`이 차면 +1, 종료
8. 사건이 있으면 플래너 호출(직전 호출 후 1초가 안 됐으면 사건을 보류)

### 단계와 분할

| | 0단계 | 1단계 | 2단계 |
|---|---|---|---|
| 방 | 학습 유형 | 학습 유형 | 학습 유형 |
| 가구 | 없음 | 세트 A 3~5개 | 세트 A 3~5개 |
| 사람 | 서 있음 | 서 있음 | 걷기(0.5~0.9 m/s), 경계, 손 |
| 학습 서브골 | 접근(무작위 피부) | 접근, 숨기, 이동 | + 손이 움직일 때 절반 확률로 회피 |

| 평가 분할 | 바뀌는 것 |
|---|---|
| eval_familiar | 학습 유형, 평가 시드 |
| eval_new_room | 6×3 긴 방, 칸막이 5×5 |
| eval_new_object | 가구 세트 B + 피부 덮개 1~2곳 + 끈끈이 트랩 1~2개 |
| eval_new_human | 4점 순환 경로 0.8~1.3 m/s, 손 반응 0.15~0.3초·속도 6~8 m/s |
| eval_net | 사람 주위 모기장(사람은 서 있음) |

모기향(3단계, 확장)은 아직 구현하지 않았고, 관측의 위험 지점 칸은 0입니다.

### 레이어와 후보 지점

- 레이어 8(사람)은 가시성·경로 검사에서 빼고, 레이어 9(라벨)는 플래너 카메라만 봅니다. 이름 없는 레이어 번호를 그대로 써서 프로젝트 설정을 바꿀 필요가 없습니다.
- 후보 지점은 결정마다 다시 뽑습니다. 가려진 지점(모기와 가까운 순) 최대 4개, 모기장 네 면 바깥, 트랩 0.2 m 안 지점 최대 2개, 나머지는 가장 먼 점 샘플링입니다. 서로 0.5 m 이상 떨어지고, 순서는 그 결정의 시드로 섞습니다.
- Unity는 결정마다 후보 목록을 저장해 두고, 나중에 도착한 지시의 ID를 그 결정의 목록으로 해석합니다.

## 3. 플래너

| planner 값 | 이름 | 쓰임 |
|---|---|---|
| 0 | Fixed | 가장 가까운 피부로 APPROACH |
| 1 | Random | 학습 |
| 2 | Rule | 정답 규칙. Unity 안 빠른 확인용(평가의 `rule_oracle`과 같은 규칙) |
| 3 | External | 평가: Python이 조건별로 답함 |
| 4 | Probe | 직접 지정 점검: 에피소드마다 서브골 하나, 재계획 없음(`--condition probe`) |

- **사건:** `episode_start`, `subgoal_achieved`, `subgoal_timeout`, `alert_start`, `alert_end`, `hand_active`. 호출 사이는 최소 1초입니다.
- **학습 잠금:** 무작위 플래너는 흡혈 중에는 위협 사건(경계 시작, 손 활성)일 때만 바꾸고, 그 밖에는 결정 스텝마다 1% 확률(평균 5초에 한 번)로 중간 교체합니다.
- **마스킹 규칙(`rule_masked`, Python):** 요청의 `view`(VLM 지시문과 같은 정보)만 읽습니다. 손이 1 m 안이면 회피 → 경계 중이면 모기에서 가장 먼 후보로 숨기 → 그 외 가장 가까운 피부로 접근.
- **정답 규칙(`rule_oracle`, Unity `Planners.Rule`):** 손이 1 m 안에서 움직이면 회피 → 경계 중이면 가려진 가장 가까운 후보로 숨기 → 목표 피부까지 직선이 막혔으면 사람까지 길이 열린 가장 가까운 후보로 이동 → 그 외에는 손에서 가장 먼 피부(1단계 이하는 가장 가까운 피부)로 접근. 덮개와 트랩은 보지 않으며, 접근이 시간 초과된 피부만 제외합니다.

## 4. 학습 코드

| 파일 | 내용 |
|---|---|
| `configs/stage0.yaml` | PPO(256×2, batch 2048, buffer 20480, lr 3e-4), 500만 스텝, 착지 커리큘럼(6→4→2 cm, 0.6→0.3 m/s, 흡혈 1→3초) |
| `configs/stage1.yaml` | 0단계에서 이어서 300만 스텝, 최종 착지 조건, 이전 단계 20% |
| `configs/stage2.yaml` | 1단계에서 이어서 100만 스텝, 5만 스텝마다 체크포인트(적응 지점 0·5만·20만·100만) |
| `scripts/run_stage.sh` | 단계와 시드를 받아 실행, 1단계부터 `--initialize-from stage{N-1}_s{seed}` 자동 |
| `python/training/smoke_test.py` | 랜덤 행동으로 관측 크기·범위 점검과 처리량 측정 |

TensorBoard에서 `Mosquito/Success`, `Mosquito/Caught`, `Mosquito/Trapped`, `Mosquito/Collisions`를 봅니다.

## 5. 평가 코드

| 파일 | 내용 |
|---|---|
| `python/evaluation/evaluate.py` | 조건 하나 × 분할 하나 실행, `MLAgentsEnv` 어댑터 |
| `python/evaluation/eval_loop.py` | `EvalLoop`: 정책 실행, 요청 처리, 지연 주입, 할당량 관리 |
| `python/evaluation/metrics.py` | 조건별 요약, 짝지은 부트스트랩 95% 신뢰구간 |
| `python/mosquitosim/channel.py` | 사이드 채널(GUID는 C#과 같음) |
| `python/mosquitosim/latency.py` | `measured`, `fixed:<초>`, `realtime`과 지시 예약 |
| `python/mosquitosim/onnx_policy.py` | ONNX 정책 실행(결정론적 출력) |
| `python/mosquitosim/planner_client.py`, `prompt.py` | 플래너 서버 호출, 시스템 프롬프트 |
| `python/planner_server/` | VLM 서버(`transformers`, `openai` 호환 = vLLM, `mock`) |

### 한 평가 실행의 흐름

1. `evaluate.py`가 Unity에 환경 파라미터(`planner=3`, 분할, 단계, 평가 시드, 최종 착지 조건)와 채널 설정(이미지 폴더, VLM이면 이미지 캡처)을 보냅니다.
2. 매 스텝 `EvalLoop`가 관측을 받아 ONNX 정책으로 행동을 내고, 예약 시간이 된 지시를 보낸 뒤 한 스텝 진행합니다(시뮬레이션 0.05초).
3. 요청이 오면 조건에 따라 처리합니다.
   - 서브골 고정·정답 규칙·마스킹 규칙: `fixed_command`·`rule_command`, 또는 `view`로 계산한 마스킹 규칙 지시를 지연 0으로 예약
   - VLM·이미지 제외 VLM: 플래너 서버를 부르고(그동안 Unity는 멈춤), 응답을 검증한 뒤 `요청 시각 + d`에 예약. 무효면 보내지 않아 직전 지시가 유지됨
   - `realtime`: 별도 스레드로 부르고 시뮬레이션은 계속 진행
4. 에피소드가 끝나면 그 방의 예약을 지우고 결과를 기록합니다. 방마다 할당량(예: 30에피소드면 앞의 14개 방이 2개, 나머지가 1개)을 채우면 끝납니다. 할당량을 넘은 에피소드의 요청에는 고정 지시만 돌려줘 VLM을 부르지 않습니다.
5. 결과는 `<out>/<조건>__<분할>__p<정책 시드>s<평가 시드>__<지연>/`에 `episodes.csv`, `decisions.jsonl`, `meta.json`, `images/`로 남습니다.

### 실행 예

```bash
# VLM 서버 (GPU 기기, 별도 venv)
python python/planner_server/server.py --backend transformers --model Qwen/Qwen3-VL-8B-Instruct

# 2단계 끝, 새 물체, 네 조건
for c in fixed rule_masked rule_oracle vlm vlm_noimg; do
  python python/evaluation/evaluate.py --env builds/Main/MosquitoSim.x86_64 \
    --policy results/stage2_s1/Mosquito.onnx --policy-seed 1 --stage 2 \
    --split new_object --condition $c --latency measured --eval-seed 1 --episodes 30
done

# 지연 곡선 (기준 설정 VLM)
for d in 0 0.25 0.5 1 2; do
  python python/evaluation/evaluate.py ... --condition vlm --latency fixed:$d
done

python python/evaluation/metrics.py results/eval --baseline rule_masked
```

## 6. 검증 상태와 남은 일

- **검증한 것:** Python 테스트 53개(관측 규격, C# 상수 대조, 플래너 응답 파싱, 가짜 환경으로 평가 루프·지연 주입·할당량·무효 응답·에피소드 경계, 모의 플래너 서버 왕복, 부트스트랩, ONNX 정책). C#은 Unity·ML-Agents API를 흉내 낸 스텁으로 형식 검사만 했습니다. 사이드 채널 코드는 실제 `mlagents_envs` 1.1.0 패키지로 송수신을 확인했습니다.
- **Unity에서 확인한 것(11/21):** Unity 6000.3.29f1 + URP 17.3에서 컴파일 오류 0, 씬 생성, Play, 2단계 규칙 플래너 자동 조종(에피소드 대부분 성공), 관측 이상 0. 자동 테스트 루프(`AutoTest/request.txt`)로 확인했습니다.
- **Windows 빌드에서 확인한 것(11/22, RTX 5070 PC, Windows 11, Python 3.10.11, 시드 1, 외부 에셋 없이 도형 사람·도형 가구·구 모기):** 에디터 컴파일 오류 0, 씬 생성, Windows 플레이어 빌드(147초, 125 MB), `smoke_test.py` 통과(관측 59, 값 이상 0, 방 16개 한 빌드에서 6,396 decisions/s), `mlagents-learn`이 빌드 4개에 연결(`--no-graphics --time-scale 20 --num-envs 4`, CPU 학습). 첫 체크포인트 저장이 torch 2.14.1에서 실패해 `requirements.txt`에 `torch==2.2.2`를 고정했고, 고정 후 Windows에서 체크포인트·최종 onnx 저장까지 확인했습니다. **0단계:** 500만 스텝 16분 53초. 착지 커리큘럼은 125만 스텝 전에 마지막 단계(2 cm, 0.3 m/s, 3초)에 도달했고, 마지막 10% 구간의 `Mosquito/Success`는 0.998, 에피소드 길이는 평균 216 결정(약 11초)입니다. **1단계(0단계 정책에서 이어서, 무작위 서브골):** 300만 스텝 10분 23초. 누적 보상 −1.01 → 0.74(끝까지 오르는 중), 에피소드당 충돌 212 → 37. `Mosquito/Success`는 0.69 근처에서 변하지 않았는데, 이 통계는 흡혈 성공만 세므로 숨기·이동 서브골이 섞이는 1단계에서는 서브골 달성률을 뜻하지 않습니다. `docs/interface.md`의 직접 지정 점검(서브골별 달성률)이 아직 구현돼 있지 않아 1단계가 기준(서브골별 80%)을 넘었는지는 판단하지 못했습니다. **1단계 정책의 평가(`evaluate.py`, 평가 시드 1, 30 에피소드, 성공 수):** val은 fixed 23 / rule 26, new_room은 fixed 26 / rule 27, net은 fixed 4 / rule 5. 에피소드당 충돌은 val에서 fixed 1,012 / rule 268입니다. 모기장(net)에서는 두 조건 모두 대부분 시간 초과였습니다. 같은 평가를 두 번 돌려 결과가 같음을 확인했습니다. 이 평가에서 사이드 채널이 한글을 `?`로 바꾸는 문제를 찾아 고쳤습니다(`JsonBuilder`, `channel.py`). 모두 도형 에셋 기준의 예행 실행이며 본 실험 결과로 쓰지 않습니다. 실행 절차는 `scripts/windows/README.md`.
- **0단계 정책의 일반화 점검(11/22, 같은 PC, 도형 에셋, 정책 시드 1):** 학습 중 성공률 0.998이 학습 조건에만 맞춰진 값인지 보려고, 0단계 정책을 `fixed` 조건으로 평가 시드 1·2·3에서 각 30회씩 평가했다(`--stage 0`, 커리큘럼 마지막 단계 값). 성공은 `val` 88/90(97.8%), `new_human` 87/90(96.7%), `new_room` 70/90(77.8%)이고, 실패 25건은 모두 시간 초과였으며 잡히거나 갇힌 경우는 없었다. 방 형태별로 나누면 사각형 방(R4x3, R5x4)은 120회 모두 성공, L자 방은 60회 중 5회 실패, 긴 방(Long6x3)은 45회 중 1회 실패, 칸막이 방(Partition5x5)은 45회 중 19회 실패다. 실패한 에피소드는 대부분 충돌 횟수가 5,800회 이상으로, 60초 내내 벽에 붙어 있었다. 방 형태는 에피소드 기록에 없어서 `EpisodeSeed.Make`와 같은 시드로 `System.Random`의 첫 값을 다시 계산해 구했고, `decisions.jsonl`의 후보 지점 분포와 30회 모두 일치하는 것을 확인했다. 즉 0단계 정책은 열린 공간의 접근·흡혈은 새 시드에서도 유지하지만 안쪽 벽을 돌아가지는 못한다. 0단계에는 장애물이 없으므로 예상된 한계이며, 1단계 정책에서 같은 실패가 줄어드는지 방 형태별로 확인해야 한다. 0단계에서는 사람이 움직이지 않아 `new_human`은 `val`과 같은 조건이다.
- **에셋 빌드 본 실행(11/23, RTX 5070 PC, 코드 `aa12f29`, 외부 에셋 포함 빌드 1,551 MB, 학습 시드 1·2·3, CPU 학습, 빌드 4개 × 방 16개):** 0단계는 시드당 500만 스텝에 약 20분(초당 약 4,000스텝), 1단계는 0단계 정책에서 시작해 300만 스텝에 약 12분이 걸렸다. **0단계**는 세 시드 모두 통과했다(`fixed`, 평가 시드 1, 30회: `val` 29·29·29, `new_room` 26·25·26). **1단계는 직접 지정 점검에서 세 서브골 모두 기준(정책 시드 3개 평균 80 %)에 못 미쳤다**(`val`, 평가 시드 1·2·3 × 30회, 정책 시드별 90회): APPROACH 71.1 %·83.3 %·82.2 %(평균 78.9 %), HIDE 23.3 %·25.6 %·24.4 %(평균 24.4 %), MOVE_TO 0 %·0 %·0 %(270회 모두 시간 초과). 정책 시드 1의 기록을 보면 MOVE_TO는 목표 쪽으로 움직이기는 하지만(시작 거리 중앙값 2.29 m → 가장 가까웠던 거리 중앙값 0.91 m, 최소 0.17 m) 0.15 m 안에 든 적이 없고, 충돌 횟수 중앙값은 0이라 벽에 막힌 것은 아니다. HIDE는 90회 중 숨을 곳에 도착한 적이 없고(`probe_arrived` 0회), 달성 21회는 도착하지 못한 자리에서 발각되지 않은 경우다. 행동 0 기준선은 APPROACH 0/30, HIDE 0/30, MOVE_TO 0/30이다. 학습 중 통계(정책 시드 1)도 같다: `Subgoal/HIDE/Achieved`는 300만 스텝 내내 0, `Subgoal/MOVE_TO/Achieved`는 마지막 구간 평균 0.03이다. 즉 지점 목표(MOVE_TO·HIDE)는 학습 중에 달성 자체가 거의 일어나지 않아 배우지 못했다. 플래너 조건별 흡혈 성공(1단계 정책, 평가 시드 1, 정책 시드 3개 합계 90회)은 `val`에서 `fixed` 85, `rule_masked` 68, `rule_oracle` 90, `new_room`에서 62·54·73, `net`에서 25·22·40이다. 설계 값(보상, 달성 반경, 커리큘럼)은 바꾸지 않았다.
- **1단계 보상 실험 9종(11/24 새벽, 같은 PC·같은 빌드에 학습 전용 스위치만 추가, 브랜치 `exp/reward-sweep`):** 본 실행의 0단계 정책(시드 1·2·3)에서 1단계만 다시 학습했다. 스위치는 `feed_gate`(APPROACH가 아닐 때 흡혈 없음), `achieve_bonus`(0.3→1.0), `hide_hold` 커리큘럼(0.5→5초), `hide_hold_reward`(반경 안 초당 0.1), `moveto_radius` 커리큘럼(0.5→0.15 m), gamma 0.995, 600만 스텝이고, 평가는 항상 고정 규칙으로 판정한다. 직접 지정 점검(`val`, 정책 시드 3개 평균, APPROACH / HIDE / MOVE_TO): 기준 78.9 / 24.4 / 0.0 %, v1(gate) 77.0 / 25.6 / 0.4, v2(hold 커리큘럼+보상) 84.4 / 23.7 / 0.0, v3(gate+hold 커리큘럼) 81.5 / 22.6 / 0.4, v4(gate+hold 보상) 80.4 / 23.3 / 0.4, v5(gate+둘 다) 81.9 / 23.3 / 0.0, v6(gate+bonus 1.0) 79.3 / 25.2 / 0.0, v7(gate+반경 커리큘럼) 79.6 / 25.3 / 0.4, v8(전부) 75.6 / 24.4 / 0.0, v9(전부, 600만 스텝) 78.9 / 27.0 / 0.4. **어느 구성도 MOVE_TO·HIDE를 바꾸지 못했다.** v7의 학습 중 통계를 보면 반경 0.5 m 구간에서는 MOVE_TO 달성이 32 %였지만 반경이 줄면서 17 → 9 → 4 → 2 %로 떨어져, 달성 보상을 받고도 더 가까이 가는 쪽으로 바뀌지 않았다. **원인으로 보이는 것(정책 파일을 Unity 없이 직접 조사):** 관측의 `target_skin_*`은 MOVE_TO·HIDE 중에도 마지막 피부를 가리키는데, 정책은 `target_skin`과 `subgoal_target` 두 벡터를 합친 방향으로 가속하고 서브골 원-핫은 거의 쓰지 않는다. 피부를 1.5 m에 두고 서브골 목표를 다른 방향에 두면, 가속 방향과 서브골 목표 방향의 코사인이 목표 거리 2.0 m에서 0.77~0.85, 1.0 m에서 0.5, 0.4 m에서 0.2, 0.15 m에서 0.06 이하다(0단계 정책과 v7 정책이 같은 모양). 원-핫을 바꿨을 때의 행동 차이는 평균 0.1 이하다. 0단계에서는 두 목표가 항상 같은 점이어서 이렇게 학습된 것으로 보이며, 점검에서 목표까지 거리가 시작의 60 % 정도까지만 줄고 충돌 없이 멈춘 것과 맞는다. 설계 값은 `main`에서 바꾸지 않았다.
- **관측 개입 실험(11/24 오전, 브랜치 `exp/skin-obs`):** 원인 확인을 위해 MOVE_TO·HIDE 동안 `target_skin_*` 칸에 서브골 목표(경유점)를 넣도록 바꾼 빌드로 1단계만 다시 학습했다(0단계 정책은 본 실행 것, 시드 1·2·3). 먼저 재학습 없이 평가 코드에서만 같은 치환을 해 본 결과(`val`, 평가 시드 1, 30회)는 1단계 정책의 MOVE_TO 달성 0 → 15, HIDE 도착 0 → 13, 최소 거리 중앙값 1.12 m → 0.15 m였다. 재학습 후 직접 지정 점검(정책 시드 3개 평균, APPROACH / HIDE / MOVE_TO): 기존 설정 그대로 60.7 / 73.7 / 69.3 %, 흡혈 막기 추가 63.7 / 74.1 / 66.3, 흡혈 막기+HIDE 커리큘럼+머무는 보상 63.7 / 79.6 / 74.1(개입 전 78.9 / 24.4 / 0.0). 플래너 조건별 흡혈 성공(`val`, 90회 중)은 `fixed` 83~87, `rule_oracle` 87~90, `rule_masked` 56~62다. 즉 두 목표 칸이 어긋나는 것이 MOVE_TO·HIDE 실패의 원인임은 확인됐지만, 이 치환은 APPROACH를 61~64 %로 떨어뜨리고 어느 구성도 80 %를 넘지 못한다. 정책 입력 점검에서도 이 정책들은 여전히 불합격이어서(서브골을 읽게 된 것이 아니라 헷갈릴 여지가 없어진 것), 해결책이 아니라 원인 확인용 개입으로 기록한다.
- **아직 Unity에서 확인할 것:** 빌드 + `smoke_test.py`, 사이드 채널이 첫 리셋 전에 설정을 받는지, 첫 `OnEpisodeBegin`이 환경 파라미터를 받은 뒤에 불리는지, 세트 B 가구의 정면 방향.
- **아직 없는 것:** 잠재 텐서 C 방식 백엔드, 모기향(3단계), 결정 장면 재생 벤치마크(`latency/replay_bench.py`), 그림 스크립트.
- **조정할 값:** 손 반응·속도, 경계 조건, 착지 커리큘럼 기준(보상 0.8), 후보 수와 비율. 모두 pilot에서 정합니다.
