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
#define LAG_SIGN_X    1.0f    // 2026-07-20 (2차 재장착): 사용자가 X쌍 좌우를 하드웨어상 반대로
                              // 장착(불가피) → 직전 실측 정답(-1)에서 반전.
                              // ★ 부호 결정은 슬롯 이론 말고 오직 실측 표시로: 왼쪽 박수 → 180°대가 정답.
                              // ※ 검증은 박수 + 프레임 절연(수건/착용). 배선 변경 시마다 재확인
#define LAG_SIGN_Y    1.0f
// Y쌍 감도 보정: X/Y 모듈이 다른 제조사 IC라 Y가 ~4.3배 셈 (2026-07-20 45도 대각
// 박수 실측, 이벤트별 rmsY/rmsX 중앙값 4.26). Y 샘플에 곱해 게이트/세기 눈높이를
// X와 맞춘다. 방향(TDOA)은 크기 무관이라 각도에는 영향 없음. 마이크 교체 시 재측정.
#define Y_GAIN        0.24f
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
