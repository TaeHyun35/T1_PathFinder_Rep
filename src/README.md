# src — 코드

주제가 확정되면 모듈별 폴더를 추가합니다. 예:

```
src/
├── firmware/     # MCU(ESP32/Arduino) 서보·센서 제어
├── perception/   # 카메라, 손·물체 인식
├── control/      # 관절 매핑, 그립 제어
└── learning/     # 데이터 수집, 학습·추론
```

각 폴더에는 실행 방법을 적은 `README.md`를 둡니다.
