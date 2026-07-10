# DOA Sensor Node

ESP32-S3와 INMP441 마이크 4개를 십자 배열로 배치해 평면 음원의 방향을 추정하고,
시리얼 또는 HTTP/SSE 대시보드로 표시하는 프로젝트입니다.

## 구성

- `src/main.cpp`: 48kHz 오디오 수집, 축별 TDOA와 2D 방위 계산
- `include/pins.h`: I2S 핀, 마이크 간격, 축별 지연 보정값
- `dashboard/serve.py`: 시리얼을 여러 브라우저에 SSE로 중계
- `dashboard/doa-compass.html`: Web Serial/SSE 나침반 화면

## 빌드와 업로드

```powershell
pio run
pio run --target upload
```

기본 업로드 포트는 `COM3`입니다. 실제 보드는 ESP32-S3-WROOM-1 N16R8로 설정되어
16MB 플래시와 8MB OPI PSRAM 구성을 사용합니다.

## 대시보드

```powershell
python -m pip install -r requirements.txt
python dashboard\serve.py --com COM3 --port 8765
```

`DOA_COM`, `DOA_BAUD`, `DOA_PORT` 환경변수로도 기본값을 지정할 수 있습니다.
서버가 출력한 PC 주소를 같은 네트워크의 태블릿이나 휴대폰에서 열면 됩니다.

## 캘리브레이션

1. `src/main.cpp`의 `DEBUG_FRAMES`를 잠시 `1`로 바꿉니다.
2. 배열 중앙선에서 여러 번 소리를 내고 축별 `lag` 평균을 구합니다.
3. 그 평균을 `include/pins.h`의 `LAG_OFFSET_X`, `LAG_OFFSET_Y`에 입력합니다.
4. 좌우 또는 상하가 반대로 표시되면 해당 `LAG_SIGN_*`의 부호를 바꿉니다.
5. 운영 시에는 시리얼 출력이 DMA 처리를 방해하지 않도록 `DEBUG_FRAMES`를 다시 `0`으로 둡니다.

펌웨어는 이벤트를 한 줄 JSON으로 출력합니다. `mode=2d` 레코드에는 `phi`, 축별
`lag`, 정규화 상관 피크, 신뢰도, 벡터 크기가 포함됩니다. 한 축만 검출되면 방위를
단정하지 않고 가능한 두 각도 후보를 출력합니다. 대시보드는 이전 텍스트 형식도
계속 읽을 수 있습니다.
