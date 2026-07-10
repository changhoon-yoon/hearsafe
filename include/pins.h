// ============================================================
//  핀 설정 — 여기만 바꾸면 보드(S3 ↔ C3) 이식 가능
// ============================================================
#pragma once

// ---- I2S 핀 (INMP441 2개 공유) ----
//  ESP32-S3-WROOM-1 N16R8 안전핀 (GPIO4/5/6)
//  ※ C3로 내릴 때도 GPIO4/5/6 그대로 사용 가능
#define PIN_I2S_SCK   4   // BCLK (SCK)  — 두 마이크 공유
#define PIN_I2S_WS    5   // LRCLK (WS)  — 두 마이크 공유
#define PIN_I2S_SD    6   // DIN (SD)    — 두 마이크 공유 (데이터)

// ---- [2D 확장] 세로쌍(Y) = I2S1 핀 (N16R8 안전핀) ----
//  ※ 35/36/37 = OPI PSRAM 전용(금지). 15/16/17은 안전.
#define PIN_I2S_SCK_Y 15  // BCLK  — 세로쌍 M3/M4 공유
#define PIN_I2S_WS_Y  16  // LRCLK — 세로쌍 M3/M4 공유
#define PIN_I2S_SD_Y  17  // DIN   — 세로쌍 M3/M4 공유 (데이터)
#define I2S_PORT_Y    I2S_NUM_1
#define MIC_SPACING_Y 0.06f   // 세로쌍 간격(m). 가로와 달라도 됨.

// ---- 오디오 설정 ----
#define SAMPLE_RATE   48000   // DOA 정밀도 위해 48kHz
#define I2S_PORT      I2S_NUM_0

// ---- DOA 물리 파라미터 ----
#define MIC_SPACING_M 0.06f   // ★ 두 마이크 중심 간 거리(m). 실제 잰 값으로 수정! (예: 6cm=0.06)
#define SOUND_SPEED   343.0f  // 음속 m/s

// ---- 축별 실측 보정 ----
// processPair()의 원시 lag에 먼저 SIGN을 곱하고, 중앙 음원에서 측정한 고정 bias를 뺀다.
// 중앙에서 여러 번 측정한 lag 평균이 +0.18이라면 LAG_OFFSET_*를 0.18f로 설정한다.
#define LAG_SIGN_X   -1.0f    // 현재 X쌍은 실장 방향이 채널 표기와 반대
#define LAG_SIGN_Y    1.0f
#define LAG_OFFSET_X  0.0f    // 단위: sample
#define LAG_OFFSET_Y  0.0f

// 배선 확인용:
//   Mic #1 : L/R -> GND  => 왼쪽 채널 (Left)
//   Mic #2 : L/R -> VDD  => 오른쪽 채널 (Right)
