// ============================================================
//  진단용: 4채널 레벨 미터 (DOA 아님)
//  M1/M2 = I2S0 L/R,  M3/M4 = I2S1 L/R
//  각 마이크의 RMS와 피크를 따로 출력 → 죽은 채널/노이즈원 식별
// ============================================================
#include <Arduino.h>
#include <driver/i2s.h>
#include <math.h>
#include "pins.h"

#define FRAMES 1024
static int32_t bufX[FRAMES * 2];
static int32_t bufY[FRAMES * 2];

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

// 한 채널(슬롯 off=0:L, 1:R)의 RMS/피크/정확히 0인 샘플 수
static void chanStats(const int32_t* buf, int n, int off,
                      float& rms, int32_t& peak, int& zeros) {
  double mean = 0;
  for (int i = 0; i < n; i++) mean += (double)(buf[2 * i + off] >> 8);
  mean /= n;
  double e = 0; peak = 0; zeros = 0;
  for (int i = 0; i < n; i++) {
    int32_t v = buf[2 * i + off] >> 8;
    if (v == 0) zeros++;
    int32_t a = v < 0 ? -v : v;
    if (a > peak) peak = a;
    double d = v - mean;
    e += d * d;
  }
  rms = sqrtf((float)(e / n));
}

static void bar(float rms, char* out) {   // 로그 스케일 20칸
  int len = 0;
  if (rms > 1) len = (int)(log10f(rms) * 6.0f);
  if (len < 0) len = 0; if (len > 20) len = 20;
  memset(out, '#', len); out[len] = 0;
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n=== 진단: 4채널 레벨 미터 ===");
  Serial.printf("X(I2S0) GPIO%d/%d/%d   Y(I2S1) GPIO%d/%d/%d\n",
                PIN_I2S_SCK, PIN_I2S_WS, PIN_I2S_SD,
                PIN_I2S_SCK_Y, PIN_I2S_WS_Y, PIN_I2S_SD_Y);
  setupI2S(I2S_PORT,   PIN_I2S_SCK,   PIN_I2S_WS,   PIN_I2S_SD);
  setupI2S(I2S_PORT_Y, PIN_I2S_SCK_Y, PIN_I2S_WS_Y, PIN_I2S_SD_Y);
  Serial.println("각 마이크를 하나씩 손가락으로 톡톡 쳐보세요.\n");
  Serial.println("     M1(좌)        M2(우)        M3(위)        M4(아래)");
}

void loop() {
  size_t brX = 0, brY = 0;
  if (i2s_read(I2S_PORT,   bufX, sizeof(bufX), &brX, portMAX_DELAY) != ESP_OK) return;
  if (i2s_read(I2S_PORT_Y, bufY, sizeof(bufY), &brY, portMAX_DELAY) != ESP_OK) return;
  int nX = brX / (sizeof(int32_t) * 2);
  int nY = brY / (sizeof(int32_t) * 2);
  if (nX < FRAMES || nY < FRAMES) return;

  float r[4]; int32_t pk[4]; int z[4];
  chanStats(bufX, nX, 0, r[0], pk[0], z[0]);   // M1 = I2S0 Left
  chanStats(bufX, nX, 1, r[1], pk[1], z[1]);   // M2 = I2S0 Right
  chanStats(bufY, nY, 0, r[2], pk[2], z[2]);   // M3 = I2S1 Left
  chanStats(bufY, nY, 1, r[3], pk[3], z[3]);   // M4 = I2S1 Right

  static uint32_t last = 0;
  if (millis() - last < 250) return;
  last = millis();

  char b0[24], b1[24], b2[24], b3[24];
  bar(r[0], b0); bar(r[1], b1); bar(r[2], b2); bar(r[3], b3);

  Serial.printf("rms  M1=%8.0f  M2=%8.0f  M3=%8.0f  M4=%8.0f\n", r[0], r[1], r[2], r[3]);
  Serial.printf("peak M1=%8d  M2=%8d  M3=%8d  M4=%8d\n", pk[0], pk[1], pk[2], pk[3]);
  Serial.printf("zero M1=%8d  M2=%8d  M3=%8d  M4=%8d   (1024중 정확히 0인 샘플)\n",
                z[0], z[1], z[2], z[3]);
  Serial.printf("M1|%-20s|\nM2|%-20s|\nM3|%-20s|\nM4|%-20s|\n\n", b0, b1, b2, b3);
}
