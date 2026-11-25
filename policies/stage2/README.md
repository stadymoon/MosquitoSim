# 2단계 하위 정책 (main `a6b73a7`, 11/26)

- `Mosquito_s<n>.onnx`: 최종(100만 스텝), `Mosquito_s<n>_50k.onnx`, `Mosquito_s<n>_200k.onnx`: 적응 지점 체크포인트
- 학습: `configs/stage2.yaml` 그대로, `--initialize-from t_h2_seg_rule50_s<n>`(= `policies/stage1_seg_rule50`), 시드 1·2·3
- **미달**: EVADE가 통과하지 못했다(아래). 다른 실험의 출발점으로 남겨 둔다.

직접 지정 점검(`val`, 평가 시드 1~3 × 30회, 새 `metrics.py --probe`, HIDE는 도착+미발각만 성공), 정책 시드 3개 평균:

| 적응 지점 | APPROACH | HIDE | MOVE_TO | EVADE |
|---|---|---|---|---|
| 0 (1단계 정책) | 83.3 | 39.6 | 98.5 | 4.1 |
| 5만 | 80.7 | 44.1 | 98.1 | 4.4 |
| 20만 | 77.8 | 50.2 | 98.1 | 5.2 |
| 100만 | 79.3 | 59.1 | 92.6 | 12.6 |

자세한 내용은 `docs/architecture.md`의 11/26 항목.
