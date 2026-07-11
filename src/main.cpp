// ============================================================
//  DOA 3단계: 2D 십자(+) 배열 — 좌/우 + 상/하 (평면 360°)
//  INMP441 ×4  ·  ESP32-S3-WROOM-1 N16R8
//    가로쌍 X = I2S0 (GPIO 4/5/6)    M1=왼쪽(Left)  M2=오른쪽(Right)
//    세로쌍 Y = I2S1 (GPIO 15/16/17) M3=위(Left)    M4=아래(Right)
//
//  원리: 쌍 '내부'에서만 시간차를 구한다 (τx, τy).
//        두 I2S는 같은 크리스탈 → 샘플레이트 동일 → 사건 정렬만 맞으면 OK.
//        φ = atan2(sy, -sx)  → 0°=오른쪽, 90°=위 (반시계)
//
//  ※ 1D 좌우 버전 백업: staging\main_1d_leftright.cpp.bak
// ============================================================
#include <Arduino.h>
#include <driver/i2s.h>
#include <math.h>
#include "mbedtls/base64.h"
#include "pins.h"

#define FRAMES 1024
static int32_t bufX[FRAMES * 2];
static int32_t bufY[FRAMES * 2];
static float   Lx[FRAMES], Rx[FRAMES];
static float   Ly[FRAMES], Ry[FRAMES];

static int   MAXLAG;
static float corr[64];
static float physicalLagX;
static float physicalLagY;

// ---- 안정화 파라미터 ----
//  ※ 밴드패스(이동평균 차: MA8−MA48, 중심정렬 ≈ 550Hz~2.6kHz) 후 rms 기준.
//    - 아래쪽 차단: 저주파 소음/클럭 앨리어싱 → conf 붕괴 방지
//    - 위쪽 차단: 6cm 간격의 공간 앨리어싱 한계(2.9kHz) 위 제거 → lag 널뜀 방지
//    - 통과대역 게인 ≈ 1 (차분 HPF의 과도한 인밴드 감쇠 문제 해결)
#define ENERGY_GATE 2500.0f
#define MAF 8    // 빠른 이동평균 (LPF ≈ 2.6kHz)
#define MAS 48   // 느린 이동평균 (LPF ≈ 550Hz) — 빼서 HPF 역할
#define MAOFF ((MAS - MAF) / 2)   // 중심 정렬 오프셋 = 20
//  이벤트 래치: 양 축이 래치되는 즉시(또는 EVENT_FRAMES 초과 시) 판정 →
//  이후 REFRACT_FRAMES 동안 새 이벤트 시작을 막아 잔향 꼬리 재발화 억제.
//  (지연 최소화: 대부분 1~2프레임(~40ms) 안에 판정)
#define CORR_GATE      0.35f // 정규화 상관 피크의 최소값
#define CONF_GATE      0.12f // 주 피크와 사이드로브의 상대 분리도
#define SIDEBAND_SKIP  2     // 주 피크 주변 메인로브는 2차 피크 탐색에서 제외
#define LAG_MARGIN     1.5f  // 실측: M4 직접음 lagY가 -9.0~-9.41까지 나옴(물리한계 8.4).
                             // 보간 오버슈트+간격 오차 감안해 0.75→1.5로 완화 (9.15에서 기각되던
                             // 정상 '아래' 이벤트 복구). ±MAXLAG(12) 가장자리 가짜 피크는 여전히 차단.
#define VECTOR_MIN_NORM 0.20f
#define VECTOR_MAX_NORM 1.20f
#define EVENT_FRAMES   4    // 한 축만 잡혔을 때 다른 축을 기다리는 최대 프레임 (~85ms)
#define AXIS_PAIR_MAX_FRAMES 1 // 두 축은 같은 프레임 또는 바로 다음 프레임까지만 결합
#define REFRACT_FRAMES 10   // 판정 후 불응기 (~210ms) — 잔향 재발화 방지

#define DEBUG_FRAMES 0   // 운영 중 시리얼/DMA 지연 방지. 보정할 때만 1로 변경

// ---- 오디오 스트리밍 (노트북 분류기용) ----
//  M1(X쌍 Left) 원신호를 48kHz→16kHz로 데시메이션(3샘플 평균), 16bit로 축소해
//  base64 JSON 라인으로 전송. DOA용 밴드패스와 별개의 "광대역 경로"라
//  화재경보(≈3.1kHz) 같은 고음도 분류기에 그대로 전달된다.
//  대역폭: ~975B/프레임 × 46.9프레임/s ≈ 46kB/s → 921600bps(92kB/s)의 ~50%.
#define AUDIO_STREAM 1
#define AUDIO_DECIM  3                       // 48k / 3 = 16k (YAMNet 입력 규격)
#define SERIAL_BAUD  921600                  // 115200으론 오디오 전송 불가

#if AUDIO_STREAM
static int16_t       audioPcm[FRAMES / AUDIO_DECIM];
static unsigned char audioB64[((FRAMES / AUDIO_DECIM) * 2 + 2) / 3 * 4 + 8];
static uint32_t      audioSeq = 0;

static void streamAudio(const int32_t* buf, int n) {
  int m = n / AUDIO_DECIM;
  for (int i = 0; i < m; i++) {
    int32_t s = 0;
    for (int k = 0; k < AUDIO_DECIM; k++)
      s += buf[2 * (i * AUDIO_DECIM + k)] >> 8;   // M1 = Left 슬롯, 24bit
    s = (s / AUDIO_DECIM) >> 8;                    // 24bit → 16bit
    if (s > 32767) s = 32767;
    if (s < -32768) s = -32768;
    audioPcm[i] = (int16_t)s;
  }
  size_t olen = 0;
  if (mbedtls_base64_encode(audioB64, sizeof(audioB64), &olen,
                            (const unsigned char*)audioPcm,
                            (size_t)m * 2) != 0) return;
  Serial.printf("{\"type\":\"audio\",\"seq\":%lu,\"sr\":%d,\"n\":%d,\"data\":\"",
                (unsigned long)audioSeq++, SAMPLE_RATE / AUDIO_DECIM, m);
  Serial.write(audioB64, olen);
  Serial.println("\"}");
}
#endif

// 이벤트 래치 상태 — 축별 "첫 유효 프레임"(=직접음)의 lag를 래치.
// (최대 rms 프레임은 잔향이 더 클 수 있어 직접음 보장이 안 됨 — 실측으로 확인)
static bool  inEvent = false;
static int   evCnt = 0;
static int   refract = 0;               // 판정 후 남은 불응 프레임
static bool  evHasX = false, evHasY = false;
static float evRmsX = 0, evLagX = 0;    // evRms*는 표시용 최대 세기
static float evRmsY = 0, evLagY = 0;
static float evCorrX = 0, evCorrY = 0;
static float evConfX = 0, evConfY = 0;
static uint32_t evFrameX = 0, evFrameY = 0;
static uint32_t frameNo = 0;

// 시리얼 출력 throttle (delay() 대신 — DMA를 계속 비워야 X/Y가 정렬됨)
static uint32_t lastPrint = 0;
#define PRINT_MS 120

// ---------------- I2S 초기화 (포트/핀 파라미터화) ----------------
static bool setupI2S(i2s_port_t port, int sck, int ws, int sd) {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  i2s_pin_config_t pins = {
    .bck_io_num = sck, .ws_io_num = ws,
    .data_out_num = I2S_PIN_NO_CHANGE, .data_in_num = sd
  };
  esp_err_t err = i2s_driver_install(port, &cfg, 0, NULL);
  if (err == ESP_OK) err = i2s_set_pin(port, &pins);
  if (err == ESP_OK) err = i2s_zero_dma_buffer(port);
  if (err != ESP_OK) {
    Serial.printf("{\"type\":\"error\",\"source\":\"i2s\",\"port\":%d,\"code\":%d}\n",
                  (int)port, (int)err);
    return false;
  }
  return true;
}

// ---------- 한 쌍 처리: 분리 → DC제거 → 상관 → 보간 → 신뢰도 ----------
//  반환 = rms.  out_lagF = 서브샘플 지연(양수면 Left쪽에서 소리), out_ok = 신뢰 여부
static float processPair(const int32_t* buf, float* L, float* R, int n,
                         float& out_lagF, bool& out_ok, float& out_conf,
                         float& out_peak) {
  for (int i = 0; i < n; i++) {
    L[i] = (float)(buf[2 * i]     >> 8);   // 32bit 슬롯 안의 24bit 데이터
    R[i] = (float)(buf[2 * i + 1] >> 8);
  }

  // 밴드패스 = 중심 정렬된 이동평균의 차 (MA8 − MA48 ≈ 550Hz~2.6kHz, 게인≈1)
  //  y[i] = mean(x[i+MAOFF .. i+MAOFF+MAF-1]) − mean(x[i .. i+MAS-1])
  //  두 창의 중심이 i+23.5로 일치 → 위상 정렬. DC는 정확히 소거.
  //  L/R에 동일 필터 → 군지연 동일 → TDOA(lag)에 영향 없음.
  {
    float fL = 0, fR = 0, wL = 0, wR = 0;
    for (int i = 0; i < MAF; i++) { fL += L[i + MAOFF]; fR += R[i + MAOFF]; }
    for (int i = 0; i < MAS; i++) { wL += L[i]; wR += R[i]; }
    int m = n - MAS;
    for (int i = 0; i < m; i++) {
      float oL = fL * (1.0f / MAF) - wL * (1.0f / MAS);
      float oR = fR * (1.0f / MAF) - wR * (1.0f / MAS);
      fL += L[i + MAOFF + MAF] - L[i + MAOFF];
      fR += R[i + MAOFF + MAF] - R[i + MAOFF];
      wL += L[i + MAS] - L[i];
      wR += R[i + MAS] - R[i];
      L[i] = oL; R[i] = oR;   // 읽기(i, i+20, i+28, i+48)가 모두 끝난 뒤 i에 기록
    }
    n = m;
  }

  float eng = 0;
  for (int i = 0; i < n; i++)
    eng += L[i] * L[i] + R[i] * R[i];
  float rms = sqrtf(eng / (2.0f * n));

  out_ok = false; out_lagF = 0; out_conf = 0; out_peak = 0;
  if (rms < ENERGY_GATE) return rms;        // 에너지 게이트

  // 정규화 시간영역 상호상관. 마이크별 레벨 차이와 lag별 겹침 길이 차이를 제거한다.
  float best = -2.0f; int bestLag = 0;
  for (int lag = -MAXLAG; lag <= MAXLAG; lag++) {
    int a = (lag < 0) ? -lag : 0;
    int b = (lag < 0) ? n : n - lag;
    float s = 0, eL = 0, eR = 0;
    for (int i = a; i < b; i++) {
      float lv = L[i], rv = R[i + lag];
      s += lv * rv; eL += lv * lv; eR += rv * rv;
    }
    float c = (eL > 1e-9f && eR > 1e-9f) ? s / sqrtf(eL * eR) : 0;
    if (c > 1.0f) c = 1.0f;
    if (c < -1.0f) c = -1.0f;
    corr[lag + MAXLAG] = c;
    if (c > best) { best = c; bestLag = lag; }
  }

  // 포물선 보간 → 서브샘플 정밀도
  float frac = 0; int k = bestLag + MAXLAG;
  if (k > 0 && k < 2 * MAXLAG) {
    float A = corr[k - 1], B = corr[k], C = corr[k + 1];
    float den = A - 2 * B + C;
    if (fabsf(den) > 1e-6f) frac = 0.5f * (A - C) / den;
    if (frac > 1.0f) frac = 1.0f;
    if (frac < -1.0f) frac = -1.0f;
  }
  out_lagF = bestLag + frac;

  // 신뢰도 = 메인로브 밖에서 가장 큰 사이드로브와 주 피크의 상대 분리도.
  float second = -2.0f;
  for (int i = 0; i <= 2 * MAXLAG; i++) {
    if (abs(i - k) <= SIDEBAND_SKIP) continue;
    if (corr[i] > second) second = corr[i];
  }
  out_peak = best;
  out_conf = (best - second) / (fabsf(best) + 1e-6f);
  if (out_conf < 0) out_conf = 0;
  if (out_conf > 1) out_conf = 1;
  out_ok = best >= CORR_GATE && out_conf >= CONF_GATE;
  return rms;
}

static float clampUnit(float v) {
  return v > 1.0f ? 1.0f : (v < -1.0f ? -1.0f : v);
}

static void print2dEvent(uint32_t frame, float phi, float strength,
                         float rmsX, float rmsY, float lagX, float lagY,
                         float corrX, float corrY, float confX, float confY,
                         float vectorNorm) {
  Serial.printf(
    "{\"type\":\"doa\",\"mode\":\"2d\",\"frame\":%lu,\"phi\":%.1f,"
    "\"strength\":%.0f,\"rmsX\":%.0f,\"rmsY\":%.0f,\"lagX\":%.3f,\"lagY\":%.3f,"
    "\"corrX\":%.3f,\"corrY\":%.3f,\"confX\":%.3f,\"confY\":%.3f,\"vectorNorm\":%.3f}\n",
    (unsigned long)frame, phi, strength, rmsX, rmsY, lagX, lagY,
    corrX, corrY, confX, confY, vectorNorm);
}

static void printAxisEvent(uint32_t frame, char axis, float lag, float strength,
                           float rms, float corrPeak, float conf) {
  float component = (lag / (float)SAMPLE_RATE) * SOUND_SPEED /
                    (axis == 'x' ? MIC_SPACING_M : MIC_SPACING_Y);
  float candidateA, candidateB;
  if (axis == 'x') {
    // sx는 왼쪽이 +이므로 화면 x성분 cos(phi)는 -sx다.
    candidateA = acosf(clampUnit(-component)) * 180.0f / PI;
    candidateB = fmodf(360.0f - candidateA, 360.0f);
  } else {
    candidateA = asinf(clampUnit(component)) * 180.0f / PI;
    if (candidateA < 0) candidateA += 360.0f;
    candidateB = 180.0f - candidateA;
    if (candidateB < 0) candidateB += 360.0f;
  }
  Serial.printf(
    "{\"type\":\"doa\",\"mode\":\"axis\",\"frame\":%lu,\"axis\":\"%c\","
    "\"phi\":null,\"candidateA\":%.1f,\"candidateB\":%.1f,\"strength\":%.0f,\"rms\":%.0f,"
    "\"lag\":%.3f,\"corr\":%.3f,\"conf\":%.3f}\n",
    (unsigned long)frame, axis, candidateA, candidateB, strength, rms, lag, corrPeak, conf);
}

void setup() {
  Serial.setTxBufferSize(4096);   // 오디오 라인(~1KB)을 논블로킹으로 — DMA 지연 방지
  Serial.begin(SERIAL_BAUD);
  delay(1500);

  physicalLagX = MIC_SPACING_M / SOUND_SPEED * SAMPLE_RATE;
  physicalLagY = MIC_SPACING_Y / SOUND_SPEED * SAMPLE_RATE;
  float largestPhysicalLag = physicalLagX > physicalLagY ? physicalLagX : physicalLagY;
  MAXLAG = (int)ceilf(largestPhysicalLag) + 3;
  if (MAXLAG > 30) MAXLAG = 30;

  Serial.println("\n=== DOA 3단계: 2D 십자 배열 (좌우 + 상하, 360°) ===");
  Serial.printf("가로 dx=%.1fcm (I2S0 GPIO%d/%d/%d)\n",
                MIC_SPACING_M * 100, PIN_I2S_SCK, PIN_I2S_WS, PIN_I2S_SD);
  Serial.printf("세로 dy=%.1fcm (I2S1 GPIO%d/%d/%d)\n",
                MIC_SPACING_Y * 100, PIN_I2S_SCK_Y, PIN_I2S_WS_Y, PIN_I2S_SD_Y);
  Serial.printf("물리지연 X=%.2f Y=%.2f samples  탐색=+-%d  gate=%.0f  corr>=%.2f conf>=%.2f\n",
                physicalLagX, physicalLagY, MAXLAG, ENERGY_GATE, CORR_GATE, CONF_GATE);

  bool i2sX = setupI2S(I2S_PORT,   PIN_I2S_SCK,   PIN_I2S_WS,   PIN_I2S_SD);
  bool i2sY = setupI2S(I2S_PORT_Y, PIN_I2S_SCK_Y, PIN_I2S_WS_Y, PIN_I2S_SD_Y);
  if (!i2sX || !i2sY) {
    Serial.println("{\"type\":\"error\",\"source\":\"startup\",\"message\":\"I2S initialization failed\"}");
    while (true) delay(1000);
  }

  Serial.printf(
    "{\"type\":\"system\",\"sampleRate\":%d,\"frameSamples\":%d,"
    "\"physicalLagX\":%.3f,\"physicalLagY\":%.3f,\"corrGate\":%.2f,\"confGate\":%.2f,"
    "\"psram\":%s,\"psramBytes\":%u}\n",
    SAMPLE_RATE, FRAMES, physicalLagX, physicalLagY, CORR_GATE, CONF_GATE,
    psramFound() ? "true" : "false", (unsigned int)ESP.getPsramSize());

  Serial.println("\n[검증] M1쪽 톡톡→왼쪽 / M3쪽 톡톡→위 / 대각 박수→사분면\n");
}

void loop() {
#if DEBUG_FRAMES
  uint32_t loopStartedUs = micros();
#endif
  size_t brX = 0, brY = 0;
  // 두 포트를 매 루프 비운다 (delay 금지 — 링버퍼 넘치면 X/Y 시간대가 어긋남)
  if (i2s_read(I2S_PORT,   bufX, sizeof(bufX), &brX, portMAX_DELAY) != ESP_OK) return;
  if (i2s_read(I2S_PORT_Y, bufY, sizeof(bufY), &brY, portMAX_DELAY) != ESP_OK) return;
  int nX = brX / (sizeof(int32_t) * 2);
  int nY = brY / (sizeof(int32_t) * 2);
  if (nX < FRAMES || nY < FRAMES) return;
  uint32_t currentFrame = ++frameNo;

#if AUDIO_STREAM
  streamAudio(bufX, nX);   // M1 원신호(광대역) → 노트북 분류기
#endif

  float lagX, lagY, confX, confY, corrX, corrY; bool okX, okY;
  float rmsX = processPair(bufX, Lx, Rx, nX, lagX, okX, confX, corrX);
  float rmsY = processPair(bufY, Ly, Ry, nY, lagY, okY, confY, corrY);

  // 배선 부호와 중앙 음원에서 실측한 고정 지연 bias를 보정한다.
  lagX = lagX * LAG_SIGN_X - LAG_OFFSET_X;
  lagY = lagY * LAG_SIGN_Y - LAG_OFFSET_Y;

  // 프린트 사이 최대 rms 추적 (120ms 출력 주기 사이 피크를 놓치지 않게)
  static float pkX = 0, pkY = 0;
  if (rmsX > pkX) pkX = rmsX;
  if (rmsY > pkY) pkY = rmsY;

#if DEBUG_FRAMES
  // 게이트 통과 프레임은 즉시 출력 (throttle 무시) — conf가 왜 떨어지는지 확인용
  if (rmsX >= ENERGY_GATE)
    Serial.printf("  [X프레임] n=%lu rms=%.0f lag=%+.2f corr=%.2f conf=%.2f %s\n",
                  (unsigned long)currentFrame, rmsX, lagX, corrX, confX, okX ? "OK" : "탈락");
  if (rmsY >= ENERGY_GATE)
    Serial.printf("  [Y프레임] n=%lu rms=%.0f lag=%+.2f corr=%.2f conf=%.2f %s\n",
                  (unsigned long)currentFrame, rmsY, lagY, corrY, confY, okY ? "OK" : "탈락");
#endif

  // ---- 이벤트 래치 ----
  // 유효 = 게이트+신뢰도 통과 & 물리적으로 가능한 lag (|lag| ≤ 8.4+여유).
  // (상관 창 가장자리(±MAXLAG)의 가짜 피크 배제)
  // 첫 유효 프레임 = 직접음 → 그 lag로 이벤트당 판정 1회.
  if (refract > 0) refract--;

  bool validX = okX && fabsf(lagX) <= physicalLagX + LAG_MARGIN;
  bool validY = okY && fabsf(lagY) <= physicalLagY + LAG_MARGIN;
  if ((validX || validY) && refract == 0) {
    if (!inEvent) {
      inEvent = true; evCnt = 0; evHasX = evHasY = false;
      evRmsX = evRmsY = evCorrX = evCorrY = evConfX = evConfY = 0;
    }
    if (validX) {
      if (!evHasX) {
        evLagX = lagX; evCorrX = corrX; evConfX = confX;
        evFrameX = currentFrame; evHasX = true;
      }
      if (rmsX > evRmsX) evRmsX = rmsX;
    }
    if (validY) {
      if (!evHasY) {
        evLagY = lagY; evCorrY = corrY; evConfY = confY;
        evFrameY = currentFrame; evHasY = true;
      }
      if (rmsY > evRmsY) evRmsY = rmsY;
    }
  }

  // 서로 한 프레임 이내에 잡힌 축만 같은 음파 사건으로 결합한다.
  uint32_t frameGap = evFrameX > evFrameY ? evFrameX - evFrameY : evFrameY - evFrameX;
  bool pairReady = evHasX && evHasY && frameGap <= AXIS_PAIR_MAX_FRAMES;
  if (inEvent && (pairReady || ++evCnt >= EVENT_FRAMES)) {
    inEvent = false;
    refract = REFRACT_FRAMES;
    bool hX = evHasX, hY = evHasY;

    // 늦게 들어온 다른 축이나 물리적으로 불가능한 벡터는 2D로 합치지 않는다.
    if (hX && hY && !pairReady) {
      float qualityX = evCorrX * evConfX;
      float qualityY = evCorrY * evConfY;
      if (qualityX >= qualityY) hY = false; else hX = false;
    }

    // 성분: sin(theta) = tau * c / d.   +sx = 왼쪽 성분,  +sy = 위 성분
    float sx = 0, sy = 0;
    if (hX) {
      sx = (evLagX / (float)SAMPLE_RATE) * SOUND_SPEED / MIC_SPACING_M;
    }
    if (hY) {
      sy = (evLagY / (float)SAMPLE_RATE) * SOUND_SPEED / MIC_SPACING_Y;
    }

    if (hX && hY) {
      float vectorNorm = sqrtf(sx * sx + sy * sy);
      if (vectorNorm >= VECTOR_MIN_NORM && vectorNorm <= VECTOR_MAX_NORM) {
        // 표시는 수학 표준(0°=오른쪽, 90°=위, 반시계) → x축에 -sx
        float phi = atan2f(sy, -sx) * 180.0f / PI;
        if (phi < 0) phi += 360.0f;
        print2dEvent(currentFrame, phi, evRmsX > evRmsY ? evRmsX : evRmsY,
                     evRmsX, evRmsY, evLagX, evLagY,
                     evCorrX, evCorrY, evConfX, evConfY, vectorNorm);
      } else {
        float qualityX = evCorrX * evConfX;
        float qualityY = evCorrY * evConfY;
        if (qualityX >= qualityY)
          printAxisEvent(currentFrame, 'x', evLagX, evRmsX, evRmsX, evCorrX, evConfX);
        else
          printAxisEvent(currentFrame, 'y', evLagY, evRmsY, evRmsY, evCorrY, evConfY);
      }
    } else if (hX) {
      printAxisEvent(currentFrame, 'x', evLagX, evRmsX, evRmsX, evCorrX, evConfX);
    } else {
      printAxisEvent(currentFrame, 'y', evLagY, evRmsY, evRmsY, evCorrY, evConfY);
    }
    return;
  }

  // ---- 평시 출력 (throttle) ----
  uint32_t now = millis();
  if (now - lastPrint < PRINT_MS) return;
  lastPrint = now;
  if (!inEvent) {
#if DEBUG_FRAMES
    Serial.printf("… 대기 (피크rmsX=%.0f 피크rmsY=%.0f loop=%luus) …\n",
                  pkX, pkY, (unsigned long)(micros() - loopStartedUs));
#endif
    pkX = pkY = 0;
  }
}
