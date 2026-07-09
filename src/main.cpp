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
#include "pins.h"

#define FRAMES 1024
static int32_t bufX[FRAMES * 2];
static int32_t bufY[FRAMES * 2];
static float   Lx[FRAMES], Rx[FRAMES];
static float   Ly[FRAMES], Ry[FRAMES];

static int   MAXLAG;
static float corr[64];

// ---- 안정화 파라미터 ----
//  ※ 밴드패스(이동평균 차: MA8−MA48, 중심정렬 ≈ 550Hz~2.6kHz) 후 rms 기준.
//    - 아래쪽 차단: 저주파 소음/클럭 앨리어싱 → conf 붕괴 방지
//    - 위쪽 차단: 6cm 간격의 공간 앨리어싱 한계(2.9kHz) 위 제거 → lag 널뜀 방지
//    - 통과대역 게인 ≈ 1 (차분 HPF의 과도한 인밴드 감쇠 문제 해결)
#define ENERGY_GATE 2500.0f
#define MAF 8    // 빠른 이동평균 (LPF ≈ 2.6kHz)
#define MAS 48   // 느린 이동평균 (LPF ≈ 550Hz) — 빼서 HPF 역할
#define MAOFF ((MAS - MAF) / 2)   // 중심 정렬 오프셋 = 20
//  이벤트 래치: 게이트를 넘는 순간부터 EVENT_FRAMES 동안을 한 "소리 이벤트"로 묶고,
//  그중 rms가 가장 큰 프레임(=직접음)의 lag로 딱 1번 판정.
//  (잔향 꼬리 프레임이 방향을 오염시키는 문제 해결)
#define CONF_GATE    0.30f
#define EVENT_FRAMES 15   // 이벤트 수집 길이 ≈ 15×21ms ≈ 320ms

#define DEBUG_FRAMES 1   // 게이트 통과 프레임마다 rms/lag/conf 출력 (튜닝용)

// 이벤트 래치 상태 — 축별 "첫 유효 프레임"(=직접음)의 lag를 래치.
// (최대 rms 프레임은 잔향이 더 클 수 있어 직접음 보장이 안 됨 — 실측으로 확인)
static bool  inEvent = false;
static int   evCnt = 0;
static bool  evHasX = false, evHasY = false;
static float evRmsX = 0, evLagX = 0;    // evRms*는 표시용 최대 세기
static float evRmsY = 0, evLagY = 0;

// 시리얼 출력 throttle (delay() 대신 — DMA를 계속 비워야 X/Y가 정렬됨)
static uint32_t lastPrint = 0;
#define PRINT_MS 120

// ---------------- I2S 초기화 (포트/핀 파라미터화) ----------------
static void setupI2S(i2s_port_t port, int sck, int ws, int sd) {
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
  i2s_driver_install(port, &cfg, 0, NULL);
  i2s_set_pin(port, &pins);
  i2s_zero_dma_buffer(port);
}

// ---------- 한 쌍 처리: 분리 → DC제거 → 상관 → 보간 → 신뢰도 ----------
//  반환 = rms.  out_lagF = 서브샘플 지연(양수면 Left쪽에서 소리), out_ok = 신뢰 여부
static float processPair(const int32_t* buf, float* L, float* R, int n,
                         float& out_lagF, bool& out_ok, float& out_conf) {
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

  double eng = 0;
  for (int i = 0; i < n; i++)
    eng += (double)L[i] * L[i] + (double)R[i] * R[i];
  float rms = sqrtf((float)(eng / (2.0 * n)));

  out_ok = false; out_lagF = 0; out_conf = 0;
  if (rms < ENERGY_GATE) return rms;        // 에너지 게이트

  // 시간영역 상호상관
  float best = -1e30f; int bestLag = 0;
  for (int lag = -MAXLAG; lag <= MAXLAG; lag++) {
    int a = (lag < 0) ? -lag : 0;
    int b = (lag < 0) ? n : n - lag;
    double s = 0;
    for (int i = a; i < b; i++) s += (double)L[i] * R[i + lag];
    corr[lag + MAXLAG] = (float)s;
    if (s > best) { best = (float)s; bestLag = lag; }
  }

  // 포물선 보간 → 서브샘플 정밀도
  float frac = 0; int k = bestLag + MAXLAG;
  if (k > 0 && k < 2 * MAXLAG) {
    float A = corr[k - 1], B = corr[k], C = corr[k + 1];
    float den = A - 2 * B + C;
    if (fabsf(den) > 1e-3f) frac = 0.5f * (A - C) / den;
  }
  out_lagF = bestLag + frac;

  // 신뢰도 = 피크가 평균보다 얼마나 튀는가
  double sum = 0; for (int i = 0; i <= 2 * MAXLAG; i++) sum += corr[i];
  float mean = (float)(sum / (2 * MAXLAG + 1));
  out_conf = (best - mean) / (fabsf(best) + 1e-6f);
  if (out_conf >= CONF_GATE) out_ok = true;
  return rms;
}

// φ(0~360, 0°=오른쪽 · 90°=위 · 반시계) → 8방위
static const char* compass(float deg) {
  static const char* d8[8] = {
    "오른쪽 ▶", "↗ 우상", "▲ 위  ", "↖ 좌상",
    "◀ 왼쪽 ", "↙ 좌하", "▼ 아래", "↘ 우하"
  };
  int idx = (int)floorf((deg + 22.5f) / 45.0f) & 7;
  return d8[idx];
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  MAXLAG = (int)ceilf(MIC_SPACING_M / SOUND_SPEED * SAMPLE_RATE) + 3;
  if (MAXLAG > 30) MAXLAG = 30;

  Serial.println("\n=== DOA 3단계: 2D 십자 배열 (좌우 + 상하, 360°) ===");
  Serial.printf("가로 dx=%.1fcm (I2S0 GPIO%d/%d/%d)\n",
                MIC_SPACING_M * 100, PIN_I2S_SCK, PIN_I2S_WS, PIN_I2S_SD);
  Serial.printf("세로 dy=%.1fcm (I2S1 GPIO%d/%d/%d)\n",
                MIC_SPACING_Y * 100, PIN_I2S_SCK_Y, PIN_I2S_WS_Y, PIN_I2S_SD_Y);
  Serial.printf("최대지연=+-%d  gate=%.0f  conf>=%.2f  이벤트창=%d프레임\n",
                MAXLAG, ENERGY_GATE, CONF_GATE, EVENT_FRAMES);

  setupI2S(I2S_PORT,   PIN_I2S_SCK,   PIN_I2S_WS,   PIN_I2S_SD);    // 가로 X
  setupI2S(I2S_PORT_Y, PIN_I2S_SCK_Y, PIN_I2S_WS_Y, PIN_I2S_SD_Y);  // 세로 Y

  Serial.println("\n[검증] M1쪽 톡톡→왼쪽 / M3쪽 톡톡→위 / 대각 박수→사분면\n");
}

void loop() {
  size_t brX = 0, brY = 0;
  // 두 포트를 매 루프 비운다 (delay 금지 — 링버퍼 넘치면 X/Y 시간대가 어긋남)
  if (i2s_read(I2S_PORT,   bufX, sizeof(bufX), &brX, portMAX_DELAY) != ESP_OK) return;
  if (i2s_read(I2S_PORT_Y, bufY, sizeof(bufY), &brY, portMAX_DELAY) != ESP_OK) return;
  int nX = brX / (sizeof(int32_t) * 2);
  int nY = brY / (sizeof(int32_t) * 2);
  if (nX < FRAMES || nY < FRAMES) return;

  float lagX, lagY, confX, confY; bool okX, okY;
  float rmsX = processPair(bufX, Lx, Rx, nX, lagX, okX, confX);
  float rmsY = processPair(bufY, Ly, Ry, nY, lagY, okY, confY);

  // X쌍은 Left채널 마이크가 물리적으로 오른쪽에 장착됨 (4방향 실측으로 확인)
  // → 부호 반전으로 보정. 이후 코드는 "+lagX = 왼쪽" 관례 그대로.
  lagX = -lagX;

  // 프린트 사이 최대 rms 추적 (120ms 출력 주기 사이 피크를 놓치지 않게)
  static float pkX = 0, pkY = 0;
  if (rmsX > pkX) pkX = rmsX;
  if (rmsY > pkY) pkY = rmsY;

#if DEBUG_FRAMES
  // 게이트 통과 프레임은 즉시 출력 (throttle 무시) — conf가 왜 떨어지는지 확인용
  if (rmsX >= ENERGY_GATE)
    Serial.printf("  [X프레임] rms=%.0f lag=%+.2f conf=%.2f %s\n",
                  rmsX, lagX, confX, okX ? "OK" : "탈락");
  if (rmsY >= ENERGY_GATE)
    Serial.printf("  [Y프레임] rms=%.0f lag=%+.2f conf=%.2f %s\n",
                  rmsY, lagY, confY, okY ? "OK" : "탈락");
#endif

  // ---- 이벤트 래치 ----
  // 유효 = 게이트+신뢰도 통과 & 물리적으로 가능한 lag (|lag| ≤ 8.4+여유).
  // (상관 창 가장자리(±MAXLAG)의 가짜 피크 배제)
  // 첫 유효 프레임 = 직접음 → 그 lag로 이벤트당 판정 1회.
  bool validX = okX && fabsf(lagX) <= (float)MAXLAG - 1.5f;
  bool validY = okY && fabsf(lagY) <= (float)MAXLAG - 1.5f;
  if (validX || validY) {
    if (!inEvent) { inEvent = true; evCnt = 0; evHasX = evHasY = false; evRmsX = evRmsY = 0; }
    if (validX) { if (!evHasX) { evLagX = lagX; evHasX = true; } if (rmsX > evRmsX) evRmsX = rmsX; }
    if (validY) { if (!evHasY) { evLagY = lagY; evHasY = true; } if (rmsY > evRmsY) evRmsY = rmsY; }
  }

  if (inEvent && ++evCnt >= EVENT_FRAMES) {
    inEvent = false;
    bool hX = evHasX, hY = evHasY;

    // 성분: sin(theta) = tau * c / d.   +sx = 왼쪽 성분,  +sy = 위 성분
    float sx = 0, sy = 0;
    if (hX) {
      sx = (evLagX / (float)SAMPLE_RATE) * SOUND_SPEED / MIC_SPACING_M;
      sx = sx > 1.0f ? 1.0f : (sx < -1.0f ? -1.0f : sx);
    }
    if (hY) {
      sy = (evLagY / (float)SAMPLE_RATE) * SOUND_SPEED / MIC_SPACING_Y;
      sy = sy > 1.0f ? 1.0f : (sy < -1.0f ? -1.0f : sy);
    }

    if (hX && hY) {
      // 표시는 수학 표준(0°=오른쪽, 90°=위, 반시계) → x축에 -sx
      float phi = atan2f(sy, -sx) * 180.0f / PI;
      if (phi < 0) phi += 360.0f;

      int pos = (int)roundf(phi / 360.0f * 31.0f);
      if (pos < 0) pos = 0; if (pos > 31) pos = 31;
      char bar[33]; memset(bar, '-', 32); bar[32] = 0; bar[pos] = 'O';

      Serial.printf("★ %s  phi=%5.1f°  (세기 %.0f)  [%s]\n",
                    compass(phi), phi, evRmsX > evRmsY ? evRmsX : evRmsY, bar);
    } else if (hX) {
      const char* s = (evLagX > 0.3f) ? "◀ 왼쪽 " : (evLagX < -0.3f) ? "오른쪽 ▶" : " 정면  ";
      Serial.printf("★ [X축만] %s  %4.0f°\n", s, fabsf(asinf(sx) * 180.0f / PI));
    } else {
      const char* s = (evLagY > 0.3f) ? "▲ 위  " : (evLagY < -0.3f) ? "▼ 아래" : " 중앙  ";
      Serial.printf("★ [Y축만] %s  %4.0f°\n", s, fabsf(asinf(sy) * 180.0f / PI));
    }
    return;
  }

  // ---- 평시 출력 (throttle) ----
  uint32_t now = millis();
  if (now - lastPrint < PRINT_MS) return;
  lastPrint = now;
  if (!inEvent) {
    Serial.printf("… 대기 (피크rmsX=%.0f  피크rmsY=%.0f) …\n", pkX, pkY);
    pkX = pkY = 0;
  }
}
