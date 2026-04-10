# Training Set

2026-04-10 수집. 현재 장착 상태(팔뚝 고정 마운트) 기준.

## 파일 설명

### ml_000 — 물건 잡고 팔 올림/내림 (정상 사용)
- **수집 방법**: 물건을 잡고 팔을 올렸다 내리기 5회 반복
- **797행, 39.8초**
- **FSR 범위**: 187 ~ 23141 (물건 잡으면 ~20000+)
- **라벨링** (ml_000_auto_labeled.csv):
  - `1` (UP): FSR 상승 구간 + FSR 고점 유지 = 307행
  - `0` (DOWN): FSR 하강 + idle = 385행
  - `-1` (unlabeled): 전환 구간 = 105행 (학습에서 제외)
- **auto_label.py state machine으로 자동 라벨링**

### ml_012 — 빈손으로 팔만 움직임 (FSR 없음)
- **수집 방법**: 물건 없이 팔만 올렸다 내림
- **112행, 5.5초**
- **FSR 범위**: -1 ~ 13231 (대부분 0 근처, 일부 손목 접촉)
- **라벨링** (ml_012_labeled.csv):
  - `0` (DOWN): 전부 = 112행
  - **이유**: 물건 없이 팔만 움직이는 건 치료 동작 아님

### ml_013 — 팔 고정, FSR만 누름/놓음
- **수집 방법**: 팔 안 움직이고 FSR만 세게 눌렀다 놓기 반복
- **789행, 39.4초**
- **FSR 범위**: -1 ~ 23409 (강하게 누르면 ~23000)
- **라벨링** (ml_013_labeled.csv):
  - `0` (DOWN): 전부 = 789행
  - **이유**: 팔 안 움직이면 FSR 아무리 세도 치료 아님

## 라벨 규칙 요약

```
FSR + 팔 움직임  → UP (1)
FSR만 (팔 고정)  → DOWN (0)
팔만 (FSR 없음)  → DOWN (0)
idle             → DOWN (0)
```

## CSV 컬럼

`ms, fsr, ax, ay, az, gx, gy, gz, tx100, ty100 [, label]`

- `ms`: 타임스탬프 (밀리초)
- `fsr`: ADS1115 raw (signed int16)
- `ax,ay,az`: ICM42670P accel raw (±2g, int16)
- `gx,gy,gz`: ICM42670P gyro raw (±2000dps, int16)
- `tx100,ty100`: tilt × 100 (adaptive LPF 후, 부팅 기준 상대값)
- `label`: 0=DOWN, 1=UP, -1=unlabeled

## 학습 명령

```bash
python3 tools/ml/train.py \
  tools/ml/data/training_set/ml_000_auto_labeled.csv \
  tools/ml/data/training_set/ml_012_labeled.csv \
  tools/ml/data/training_set/ml_013_labeled.csv
```

데이터 증강 (FSR 0.7~1.2x, 시간 0.8~1.2x)이 train.py에 내장.
