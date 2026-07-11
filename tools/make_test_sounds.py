# 테스트 사운드 생성 — 스피커로 재생해 분류/DOA 엔드투엔드 검증용
#   fire_t3.wav : 3.2kHz 사인, ISO 8201 T-3 패턴(0.5s x3 + 1.5s 휴지) x2
#   horn.wav    : 400Hz + 배음 스택 1.5초 (자동차 경적 근사)
#   clap.wav    : 백색잡음 버스트 30ms x3 (박수 근사)
import math
import os
import random
import struct
import wave

SR = 44100
OUT = os.path.dirname(os.path.abspath(__file__))


def write_wav(name, samples):
    path = os.path.join(OUT, name)
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(SR)
        w.writeframes(b"".join(struct.pack("<h", max(-32767, min(32767, int(s * 32767)))) for s in samples))
    print("wrote", path, f"{len(samples)/SR:.1f}s")


def tone(freq, sec, amp=0.8):
    return [amp * math.sin(2 * math.pi * freq * i / SR) for i in range(int(SR * sec))]


def silence(sec):
    return [0.0] * int(SR * sec)


# ---- 화재경보 (T-3, 3.2kHz) ----
t3 = []
for _cycle in range(2):
    for _beep in range(3):
        t3 += tone(3200, 0.5) + silence(0.5)
    t3 += silence(1.0)   # 마지막 0.5 + 1.0 = 1.5s 휴지
write_wav("fire_t3.wav", t3)

# ---- 경적 (400Hz 기본파 + 배음) ----
horn = []
for i in range(int(SR * 1.5)):
    t = i / SR
    v = 0.5 * math.sin(2 * math.pi * 400 * t) \
      + 0.3 * math.sin(2 * math.pi * 800 * t) \
      + 0.2 * math.sin(2 * math.pi * 1200 * t)
    horn.append(v * 0.9)
write_wav("horn.wav", horn)

# ---- 박수 (잡음 버스트 x3) ----
clap = []
rng = random.Random(42)
for _ in range(3):
    n = int(SR * 0.03)
    clap += [rng.uniform(-1, 1) * math.exp(-6.0 * i / n) for i in range(n)]
    clap += silence(0.5)
write_wav("clap.wav", clap)
