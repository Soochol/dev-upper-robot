---
name: ml-workflow
description: ML 학습 데이터 수집 → 라벨링 → 학습 → 배포 워크플로우 자동화. "시작"이라고 하면 수집 모드로 빌드+플래시, "완료"라고 하면 RTT 또는 SD 카드에서 데이터를 가져와 학습 파이프라인을 실행. "ml 시작", "데이터 수집", "학습 시작", "모델 배포" 같은 요청에도 반응.
---

# ML Workflow

ML 학습 데이터 수집 → 라벨링 → Random Forest 학습 → 펌웨어 배포까지 자동화.

## 워크플로우 개요

```
"시작" → 수집 모드 빌드+플래시 → FUNC_SW1/SW2로 녹화
"완료" → RTT로 데이터 전송 → 라벨링 → 학습 → 배포
```

## "시작" 플로우

사용자가 "시작", "데이터 수집 시작", "ml 시작"이라고 하면:

1. 수집 모드로 빌드:
```bash
export STM32CLT_PATH=/opt/st/stm32cubeclt_1.21.0
export PATH="$STM32CLT_PATH/STM32CubeProgrammer/bin:$STM32CLT_PATH/GNU-tools-for-STM32/bin:$PATH"
rm -rf build/Debug
cmake --preset Debug -DDATA_COLLECT=ON
cmake --build build/Debug -j$(nproc)
```

2. 플래시 (build-flash 스킬 활용 또는 직접):
```bash
.claude/skills/build-flash/build_flash.sh
```
주의: DATA_COLLECT=ON으로 빌드된 ELF가 flash되어야 함. build_flash.sh는 기존 build/Debug/PSP.elf를 사용하므로, cmake 빌드 후 바로 실행.

3. 사용자에게 안내:
```
수집 모드로 플래시 완료.
- FUNC_SW1 버튼: 녹화 시작 (LED 녹색)
- FUNC_SW2 버튼: 녹화 종료 + 자동 RTT 전송
완료되면 "완료"라고 말씀하세요.
```

## "완료" 플로우 — RTT 경로 (기본)

사용자가 "완료", "수집 완료", "데이터 가져와"라고 하면:

1. RTT 캡처로 데이터 수신:
```bash
cd /home/dev/code/dev-upper-robot
python3 tools/ml/rtt_capture.py --timeout 120
```
사용자에게 "SW2를 눌러 데이터를 전송하세요"라고 안내.
캡처 완료 시 `tools/ml/data/ml_NNN.csv` 저장.

2. 인터랙티브 라벨링 (또는 사용자가 auto 요청 시 자동):
```bash
# 인터랙티브 라벨링
python3 tools/ml/workflow.py label tools/ml/data/ml_NNN.csv

# 또는 자동 라벨링 (빠른 반복)
python3 tools/ml/workflow.py auto-label tools/ml/data/ml_NNN.csv
```

3. 학습:
```bash
python3 tools/ml/workflow.py train
```
정확도, confusion matrix, feature importance 결과를 사용자에게 보고.

4. 배포 여부 확인 후:
```bash
rm -rf build/Debug
cmake --preset Debug -DML_TRIGGER=ON
cmake --build build/Debug -j$(nproc)
.claude/skills/build-flash/build_flash.sh
```

## "완료" 플로우 — SD 카드 경로 (백업)

사용자가 "SD 카드로 가져와" 또는 RTT가 실패한 경우:

1. 안내: "SD 카드를 보드에서 빼서 PC USB 카드리더에 연결하세요."
2. SD 카드 감지 및 복사:
```bash
python3 tools/ml/workflow.py sd-copy
```
3. 이후 라벨링/학습/배포는 동일.

## 전체 자동 파이프라인

```bash
# SD 카드 경로 + 자동 라벨링
python3 tools/ml/workflow.py full --auto-label

# SD 경로 지정
python3 tools/ml/workflow.py full --sd-path /media/user/SD

# RTT 방식은 rtt_capture.py로 수동 → workflow.py train
```

## 파일 구조

| 파일 | 역할 |
|------|------|
| `tools/ml/workflow.py` | 파이프라인 오케스트레이터 |
| `tools/ml/rtt_capture.py` | SWD/RTT 기반 데이터 수신 |
| `tools/ml/visualize.py` | 인터랙티브 라벨링 |
| `tools/ml/train.py` | RF 학습 + m2cgen C 코드 생성 |
| `tools/ml/validate.py` | Python/C 피처 스큐 검증 |
| `tools/ml/data/` | 수집 데이터 저장소 (.gitignore) |
| `Core/Src/app/model_rf.c` | m2cgen 생성 모델 (덮어쓰기) |
