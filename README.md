# DOA Sensor Node

청각장애인 위험 소리 방향 감지 웨어러블. ESP32-S3와 INMP441 마이크 4개를 안경 프레임에
십자 배열로 장착해 소리 방향을 360° 추정(TDOA + GCC-PHAT)하고, IMU로 착용자가 고개를
돌려도 방향을 세계좌표로 유지하며, YAMNet으로 소리 종류(화재경보·경적·사이렌 등)를
분류해 방향과 함께 대시보드에 표시합니다. 배경·원리·실측 성과·개발 과정의 상세 기록은
[`docs/factsheet.md`](docs/factsheet.md)를 참고하세요.

## 구성

- `src/main.cpp`: 48kHz 오디오 수집, 축별 TDOA(GCC-PHAT+온셋 정렬)와 2D 방위 계산, IMU(yaw) 융합, WiFi/USB 오디오·이벤트 스트리밍
- `include/pins.h`: I2S 핀, 마이크 간격, 축별 지연·게인 보정값, 장착 회전 보정
- `include/wifi_secrets.h.example`: WiFi 접속정보 템플릿 (`wifi_secrets.h`로 복사해 사용, 저장소에는 미포함)
- `dashboard/serve.py`: 시리얼(USB) 또는 WiFi(TCP)로 들어오는 이벤트를 여러 브라우저에 SSE로 중계, YAMNet 분류·커스텀 소리 지문 매칭 관리
- `dashboard/classifier.py`: YAMNet 기반 소리 분류 + 사용자 커스텀 소리 등록(few-shot 임베딩 매칭)
- `dashboard/doa-compass.html`: 대시보드 화면 (나침반/링/4방향 패널 3가지 뷰, 소리 등록 UI)
- `대시보드시작.bat`: 무선(WiFi) 모드로 대시보드 서버를 바로 실행하는 단축 스크립트

## 빌드와 업로드

1. `include/wifi_secrets.h.example`을 같은 폴더에 `wifi_secrets.h`로 복사하고 실제 WiFi SSID/비밀번호를 입력합니다.
   (이 파일은 `.gitignore`에 등록되어 저장소에는 올라가지 않습니다. ESP32는 2.4GHz 대역만 지원합니다.)

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

### 무선(WiFi) 모드

기기가 WiFi에 연결되어 있으면(펌웨어 `ENABLE_WIFI 1`, 기본값) USB 없이도 동작합니다.

```powershell
python dashboard\serve.py --net doa-node.local
```

또는 `대시보드시작.bat`을 그대로 실행하면 됩니다. WiFi 연결이 끊기면 자동으로 USB
시리얼 경로로 폴백합니다. YAMNet 분류를 끄려면 `--no-classify` 옵션을 추가하세요
(최초 실행 시 YAMNet 모델을 인터넷에서 내려받아 캐시합니다).

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

## 오픈소스 사용 내역

- **YAMNet** (Google, [Apache License 2.0](https://github.com/tensorflow/models/blob/master/research/audioset/yamnet/LICENSE)) — AudioSet 521클래스로 사전학습된 오디오 분류 모델. `dashboard/classifier.py`에서 TensorFlow Hub를 통해 사용, 재학습 없이 그대로 활용하며 커스텀 소리 등록에는 임베딩 출력만 이용합니다.
