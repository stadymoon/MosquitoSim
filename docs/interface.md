# 인터페이스 규격 v3

하위 정책, 플래너, 평가 코드가 공유하는 규격입니다. 계획 문서의 "인터페이스 규격 v3"과 "최종 점검(11/21)"을 코드 기준으로 옮겼습니다. 관측 순서는 `python/mosquitosim/obs_spec.py`가 기준이고, `unity/Assets/Scripts/Agent/ObsLayout.cs`가 같은 값을 갖습니다. 둘이 다르면 `python/tests/test_cs_layout.py`가 실패합니다.

## 시간

- 물리 고정 간격 0.01초(`SimSettings`). 1.5m/s에서 한 스텝 1.5cm로, 착지 허용 오차 2cm보다 작습니다.
- 결정 주기 5스텝(0.05초), 결정 사이에는 직전 행동을 반복합니다.
- 에피소드 60초 = 6,000스텝(`MaxStep`).

## 행동

연속 3차원, 각 축 −1~1, 월드 축. 크기를 1로 자른 뒤 목표 가속도(최대 5m/s²)로 쓰고, 속도는 1.5m/s에서 자릅니다.

착지 중에는 위치가 피부에 고정되고, 피부 바깥 방향(법선) 성분이 0.5를 넘는 행동이 오면 0.3m/s로 이륙합니다.

## 관측 (59차원, 월드 축)

서브골 원-핫은 4칸(접근·숨기·회피·이동)입니다. 대기는 11/21에 제거했고, 이동(MOVE_TO)은 11/21에 채택했습니다. 회피 칸은 2단계부터 쓰입니다. `target_skin_*`(3~6)은 플래너가 마지막으로 고른 피부 영역입니다.

| 시작 | 크기 | 이름 | 값 | 없는 단계 |
|---|---|---|---|---|
| 0 | 3 | self_velocity | 속도 / 1.5 | |
| 3 | 3 | target_skin_rel_pos | (목표 피부 영역 중심 − 모기) / 5, [−1, 1]로 자름 | |
| 6 | 1 | target_skin_dist | 거리 / 5, 최대 1 | |
| 7 | 3 | subgoal_target_rel_pos | (서브골 목표 지점 − 모기) / 5 | |
| 10 | 1 | subgoal_target_dist | 거리 / 5 | |
| 11 | 3 | human_velocity | 사람 속도 / 2 | 0~1단계 |
| 14 | 1 | human_alert | 경계 0/1 | 0~1단계 |
| 15 | 3 | hand_rel_pos | (손 − 모기) / 5 | 0~1단계 |
| 18 | 3 | hand_velocity | 손 속도 / 8 | 0~1단계 |
| 21 | 1 | hand_active | 0/1 | 0~1단계 |
| 22 | 3 | hazard_rel_pos | (위험 지점 − 모기) / 5 | 위험 없음 |
| 25 | 1 | hazard_active | 0/1 | 위험 없음 |
| 26 | 1 | hazard_exposure | 누적 노출 / 한도 | 위험 없음 |
| 27 | 26 | rays | 거리 / 2 (안 맞으면 1) | |
| 53 | 4 | subgoal_onehot | APPROACH, HIDE, EVADE, MOVE_TO | |
| 57 | 1 | remaining_time | 남은 시간 비율 | |
| 58 | 1 | landed | 0/1 | |

레이 방향은 {−1, 0, 1}³에서 원점을 뺀 26개를 x, y, z 순서로 중첩해 나열하고 정규화한 것입니다.

회피는 손이 생기는 2단계부터 학습합니다. 0·1단계에는 가상 위험 지점을 쓰지 않습니다(최종 점검 1-3, 11/21 결정).

## 플래너 지시: 서브골 + 대상 ID

플래너는 서브골 하나와 대상 ID를 고릅니다. 하위 정책은 바뀌지 않고, 고른 ID가 가리키는 지점이 서브골 목표 지점(7~10)에 들어갑니다. 파싱과 검증은 `mosquitosim.subgoals.parse_planner_output`.

```json
{"subgoal": "APPROACH", "target": "S2"}
{"subgoal": "HIDE", "target": "P3"}
{"subgoal": "MOVE_TO", "target": "P6"}
{"subgoal": "EVADE"}
```

- **피부 ID:** 사람마다 고정. S1 머리, S2 왼팔, S3 오른팔, S4 왼쪽 종아리, S5 오른쪽 종아리. 어느 피부든 흡혈을 마치면 성공이고, 착지는 APPROACH로 고른 영역에만 됩니다.
- **후보 지점:** 리셋 때 빈 공간에 0.25m 격자를 만들고, 결정마다 8개를 뽑아 P1~P8로 번호를 붙입니다. 4개는 사람 머리에서 가려진 지점, 4개는 방 곳곳에 퍼진 지점(가장 먼 점 샘플링), 서로 0.5m 이상. 모기장 평가에서는 네 면 바깥 0.3m에 하나씩 반드시 포함하고 열린 면도 똑같이 다룹니다. 번호 순서는 평가 시드로 섞습니다.
- **표시:** ID를 전경·모기 시점 이미지에 덧그립니다(Set-of-Mark). 지시문에는 ID와 모기로부터의 거리만.
- **조건별 정보:** 규칙은 후보마다 가려짐 여부, 사람까지 직선 열림, 손과의 거리를 정확히 받음(상한). VLM은 이미지 + 지시문, 이미지 제외 VLM은 지시문만(하한).
- **규칙 플래너:** 손이 1m 안에서 다가오면 EVADE → 경계 중이면 가장 가까운 가려진 후보로 HIDE(없으면 시선 반대쪽 가장 가까운 후보) → 목표 피부로의 직선이 막혔으면 모기에서 직선으로 갈 수 있고 사람까지 직선이 열린 가장 가까운 후보로 MOVE_TO(둘 다 만족하는 후보가 없으면 사람까지만 열린 후보) → 그 외 손에서 가장 먼 피부로 APPROACH(접근이 시간 초과된 피부는 그 에피소드에서 제외). 규칙은 학습 환경 기준으로 만든 뒤 고치지 않습니다. 2단계 새 물체 평가의 단서 물체(피부 덮개, 끈끈이 트랩)는 규칙 입력에 들어가지 않습니다.
- **무효:** 없는 서브골, 이번 결정에 제시되지 않은 ID, 필요한 대상 누락. EVADE의 대상은 무시.

## 서브골

| id | 서브골 | 목표 지점 | 달성(학습) | 보상 |
|---|---|---|---|---|
| 0 | APPROACH | 고른 피부 영역(S1~S5) 중심 | 착지 후 흡혈 시간 유지 | 거리 감소 × 0.1, 흡혈 +1 |
| 1 | HIDE | 고른 후보 지점(P1~P8) | 0.15m 이내 1초 | 거리 감소 × 0.1, 달성 +0.3 |
| 2 | EVADE | 위험 지점(손, 모기향). 2단계부터 | 1.5m 이상 | 거리 증가 × 0.1, 달성 +0.3 |
| 3 | MOVE_TO | 고른 후보 지점(P1~P8) | 0.15m 이내 도달(멈출 필요 없음) | 거리 감소 × 0.1, 달성 +0.3 |

공통: 잡힘 −1, 모기향 노출 초과 −1, 충돌 −0.01(스텝당), 시간 −0.005/초.

교체 규칙(학습): 달성, 실패, 시간 초과(8초) 때 교체. 결정마다 평균 5초에 한 번 무작위 중간 교체. 흡혈 중에는 무작위 중간 교체를 하지 않지만, 2단계부터 위협 사건(손 활성 등) 때의 사건 교체는 착지 중에도 허용합니다(최종 점검 1-2).

직접 지정 점검(평가): 서브골 하나를 지정해 8초 안에 달성하는지 봅니다.

## 단계별 학습 서브골

| 단계 | 플래너 | 서브골 |
|---|---|---|
| 0 | fixed | APPROACH만 |
| 1 | random | APPROACH(무작위 피부), HIDE(무작위 가려진 후보), MOVE_TO(무작위 후보) |
| 2~ | random | 위 + EVADE |

## 환경 파라미터 (ML-Agents environment_parameters)

| 이름 | 값 | 읽는 곳 |
|---|---|---|
| stage | 0, 1, 2 (3은 확장, 미구현) | `EnvParams` |
| env_split | 0 train, 1 val, 2 eval_familiar, 3 eval_new_room, 4 eval_new_object, 5 eval_new_human, 6 eval_net | `EnvParams` |
| planner | 0 fixed, 1 random(학습), 2 rule(Unity 안), 3 external(평가: Python이 조건별로 답함) | `EnvParams` |
| eval_seed | 정수. 검증·평가 에피소드의 배치를 정함 | `EnvParams` |
| landing_radius | 착지 허용 거리(m), 커리큘럼 0.06 → 0.02 | `EnvParams` |
| landing_speed | 착지 허용 속도(m/s), 0.6 → 0.3 | `EnvParams` |
| feed_time | 흡혈 시간(초), 1 → 3 | `EnvParams` |
| time_penalty | 초당 시간 페널티 0.005 | `EnvParams` |
| prev_stage_mix | 학습 때 이전 단계 설정을 쓸 확률 0.2 | `EnvParams` |
| bite_alert_chance | 2단계부터, 흡혈 1초마다 사람이 경계할 확률 0.25 | `EnvParams` |

평가 조건(fixed, rule, vlm, vlm_noimg), 지연 방식(`measured`, `fixed:<초>`, `realtime`), VLM 설정은 Unity 파라미터가 아니라 `evaluate.py`와 플래너 서버의 인자입니다.

## 플래너 호출 시점

2초 주기 호출은 없고 사건 때만 부릅니다(11/21, VLM 호출 오버헤드 감소). 사건: 에피소드 시작, 서브골 종료(달성·실패·8초 시간 초과), 경계 시작·끝, 손 활성, 모기향 등장. 같은 순간의 사건은 한 번으로 묶고, 호출 사이는 최소 1초입니다. 규칙 플래너도 같은 시점에만 판단합니다. 착지 중에도 사건이 생기면 부릅니다.

## 플래너 입출력

요청(Unity → planner_server, JSON):

```json
{
  "episode_id": "eval_new_room/s1/e017",
  "sim_time": 14.2,
  "instruction": "[과제] ... [상황] ... [출력] ...",
  "images": {"mosquito": "<png base64>", "overview": ["...", "...", "...", "..."]},
  "candidates": [{"id": "P1", "pos": [0.4, 1.1, -0.8], "dist": 0.7, "hidden": true, "clear_path": false}],
  "rule_command": {"subgoal": "APPROACH", "target": "S2"},
  "vlm_config": "c_bf16"
}
```

요청에는 이번 결정의 후보 목록(`"candidates": [{"id": "P1", "pos": [...], "dist": 0.7, "hidden": true, "clear_path": false}, ...]`)을 함께 보냅니다. `hidden`, `clear_path`는 규칙 플래너와 로그용이고 VLM 프롬프트에는 넣지 않습니다.

응답: 위 "플래너 지시" 형식. 무효면 직전 지시를 유지합니다.

### 사이드 채널 메시지 (PlannerChannel, GUID 6f1c2a5e-8d3b-4c7a-9e21-5b4d3c2a1f00)

| 방향 | type | 주요 필드 |
|---|---|---|
| Python → Unity | config | image_dir, log, capture_images, image_size. 받기 전에는 Unity가 아무것도 보내지 않음 |
| Python → Unity | command | room, episode, decision, command. 다른 에피소드의 지시는 무시 |
| Unity → Python | request | room, episode, decision, sim_time, stage, events, current_command, landed, alert, hand_active, skins[], candidates[], instruction, images[](VLM일 때), rule_command, fixed_command |
| Unity → Python | decision | request와 같은 필드 + command (Unity 안 플래너의 로그) |
| Unity → Python | episode_end | room, room_type, episode, success, caught, trapped, timeout, sim_time, collisions, decisions, reward |

에피소드 ID는 `<분할>/s<평가 시드>/r<방>e<방의 에피소드 번호>`입니다.

## 평가 루프와 지연 주입 (11/21 확정)

VLM 호출은 Python 평가 루프(`evaluate.py`)가 맡습니다. Unity는 사이드 채널(`PlannerChannel`)로 사건, 이미지, 후보 목록을 보내고 지시를 받습니다. ML-Agents에서 Unity는 매 결정마다 Python 응답을 기다리므로, VLM이 계산하는 동안 시뮬레이션은 따로 멈출 필요 없이 기다립니다.

1. 시뮬레이션 시간 t에 사건이 오면 입력을 캡처하고 VLM을 부릅니다.
2. 서버는 지시와 함께 대기열 시간을 뺀 순수 계산 시간 `service_time`을 돌려줍니다.
3. 지연 d를 정합니다. `measured`는 d = service_time, `fixed:x`는 d = x, `realtime`은 배속 1에서 실제 시계로 기다림.
4. 시뮬레이션 시간 d초 동안 직전 지시로 진행하고, t+d에 새 지시를 적용합니다.

학습(무작위 플래너)에는 VLM이 없으므로 배속 20 그대로입니다. `measured`의 d는 측정 GPU에 따라 달라지므로 VLM 호출은 기준 GPU와 같은 모델에서만 돌리고, 호출마다 d를 로그에 남겨 재현에 씁니다. 첫 응답 전에는 가장 가까운 피부로의 APPROACH로 시작합니다.

## 로그

결정 로그 한 줄: 시각, 에피소드 ID, 입력(이미지 파일 경로, 지시문), 후보 목록과 기하 정답, 원문 응답, 서브골과 대상 ID, 유효 여부, 지연, 같은 상황의 규칙 지시. 기하 정답으로 "HIDE로 고른 지점이 실제로 가려진 비율", "MOVE_TO로 고른 지점에서 사람까지 길이 열린 비율"을 계산합니다. 이 로그가 지연 실험의 재생 데이터입니다.
