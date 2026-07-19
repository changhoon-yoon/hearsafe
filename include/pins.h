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
#define MIC_SPACING_Y 0.17f   // 세로쌍 간격(m). 2026-07-19 재실장 실측 17cm

// ---- 오디오 설정 ----
#define SAMPLE_RATE   48000   // DOA 정밀도 위해 48kHz
#define I2S_PORT      I2S_NUM_0

// ---- DOA 물리 파라미터 ----
#define MIC_SPACING_M 0.17f   // ★ 두 마이크 중심 간 거리(m). 2026-07-19 재실장 실측 17cm
                              //   물리 최대 lag 23.8샘플(구 6cm=8.4) — 각도 해상도 ~3배 향상.
                              //   대가: 1kHz+ 순음은 탐색창 내 다중 피크 가능(광대역 소리는 무관)
#define SOUND_SPEED   343.0f  // 음속 m/s

// ---- 축별 실측 보정 ----
// processPair()의 원시 lag에 먼저 SIGN을 곱하고, 중앙 음원에서 측정한 고정 bias를 뺀다.
// 중앙에서 여러 번 측정한 lag 평균이 +0.18이라면 LAG_OFFSET_*를 0.18f로 설정한다.
#define LAG_SIGN_X    1.0f    // 실측 확정(+1): 4점 탭 테스트에서 왼쪽=m2/오른쪽=m1 (X쌍 좌우 스왑 실장),
                              // 위=m3/아래=m4 정상. 원 배선(왼쪽=m1) 시절의 -1에서 반전.
                              // ※ 방향 검증은 반드시 박수로 — 바람/문지르기는 국소 잡음이라 판정 불가
#define LAG_SIGN_Y    1.0f
#define LAG_OFFSET_X  0.0f    // 단위: sample
#define LAG_OFFSET_Y  0.0f

// 배선 확인용:
//   Mic #1 : L/R -> GND  => 왼쪽 채널 (Left)
//   Mic #2 : L/R -> VDD  => 오른쪽 채널 (Right)

// ---- IMU (MPU-9250/9255/6500, I2C) — 착용형 회전 보정용 자이로 ----
//  배선: VCC->3V3, GND->GND, SCL->GPIO9, SDA->GPIO8, AD0->GND(주소 0x68)
//  ※ 전원은 마이크 스타노드가 아니라 보드 3V3 핀에서 직접 따올 것 (접촉불량 전례)
#define PIN_IMU_SDA  8
#define PIN_IMU_SCL  9
#define IMU_ADDR     0x68
// 요yaw 부호: 배열을 위에서 봤을 때 반시계 회전 시 yaw가 증가해야 함(phi와 동일 좌표계).
// 실기 테스트에서 화살표가 반대로 돌면 -1.0f로 뒤집는다.
#define YAW_SIGN     1.0f
