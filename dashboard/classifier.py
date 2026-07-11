# ============================================================
#  YAMNet 소리 분류기 (노트북 계산용)
#  serve.py가 ESP32의 {"type":"audio"} 라인을 feed_line()으로 넘기면
#  백그라운드 스레드가 0.5초마다 최근 ~1초 창을 분류해 콜백으로 보낸다.
#
#  - 입력: 16kHz 16bit mono (펌웨어 AUDIO_STREAM 규격 = YAMNet 규격)
#  - 모델: TF-Hub YAMNet (AudioSet 521클래스, 사전학습) — 최초 1회 다운로드
#  - TF 로딩은 지연 임포트: 분류기 없이도 serve.py는 정상 동작
# ============================================================
import base64
import csv
import json
import threading
import time

import numpy as np

WINDOW_SEC = 0.975   # YAMNet 내부 프레이밍(0.96s)과 맞춘 분석 창
HOP_SEC = 0.5        # 분류 주기
MIN_SCORE = 0.10     # 이 미만 top1은 보고하지 않음 (침묵/애매)

# AudioSet display_name → 한국어 라벨 (주요 관심 클래스만; 없으면 원문 표기)
KO = {
    "Fire alarm": "🔥 화재경보",
    "Smoke detector, smoke alarm": "🔥 화재경보",
    "Siren": "🚨 사이렌",
    "Civil defense siren": "🚨 민방위 사이렌",
    "Ambulance (siren)": "🚑 구급차 사이렌",
    "Police car (siren)": "🚓 경찰차 사이렌",
    "Fire engine, fire truck (siren)": "🚒 소방차 사이렌",
    "Vehicle horn, car horn, honking": "📯 자동차 경적",
    "Air horn, truck horn": "📯 트럭 경적",
    "Doorbell": "🔔 초인종",
    "Ding-dong": "🔔 초인종",
    "Knock": "🚪 노크",
    "Clapping": "👏 박수",
    "Applause": "👏 박수",
    "Hands": "👏 손뼉",
    "Speech": "💬 말소리",
    "Shout": "📢 외침",
    "Yell": "📢 외침",
    "Screaming": "😱 비명",
    "Baby cry, infant cry": "👶 아기 울음",
    "Dog": "🐕 개",
    "Bark": "🐕 개 짖음",
    "Glass": "🍷 유리",
    "Shatter": "💥 깨지는 소리",
    "Alarm": "⏰ 경보음",
    "Alarm clock": "⏰ 알람시계",
    "Beep, bleep": "🔊 삐 소리",
    "Buzzer": "🔊 버저",
    "Boom": "💥 쿵",
    "Silence": "조용함",
    "Music": "🎵 음악",
    "Water": "💧 물소리",
    "Typing": "⌨️ 타이핑",
}

DANGER = {
    "🔥 화재경보", "🚨 사이렌", "🚨 민방위 사이렌", "🚑 구급차 사이렌",
    "🚓 경찰차 사이렌", "🚒 소방차 사이렌", "📯 자동차 경적", "📯 트럭 경적",
    "😱 비명", "💥 깨지는 소리", "⏰ 경보음", "💥 쿵",
}


def decode_audio_line(line: str):
    """{"type":"audio",...} JSON 라인 → (float32 [-1,1] 파형, 샘플레이트). 실패 시 None."""
    try:
        rec = json.loads(line)
        if rec.get("type") != "audio":
            return None
        pcm = np.frombuffer(base64.b64decode(rec["data"]), dtype="<i2")
        return pcm.astype(np.float32) / 32768.0, int(rec.get("sr", 16000))
    except Exception:
        return None


class SoundClassifier:
    def __init__(self, on_result, sr: int = 16000):
        self.on_result = on_result
        self.sr = sr
        self.buf = np.zeros(0, dtype=np.float32)
        self.lock = threading.Lock()
        self.ready = False
        self.model = None
        self.names = []
        self.fed = 0
        threading.Thread(target=self._load, daemon=True).start()

    # ---------- 모델 로딩 (수십 초 걸릴 수 있어 백그라운드) ----------
    def _load(self):
        try:
            import tensorflow as tf          # noqa: 지연 임포트
            import tensorflow_hub as hub
            print("[classify] YAMNet 로딩 중… (최초 실행 시 모델 다운로드)")
            self.model = hub.load("https://tfhub.dev/google/yamnet/1")
            class_map = self.model.class_map_path().numpy().decode("utf-8")
            with tf.io.gfile.GFile(class_map) as f:
                self.names = [row["display_name"] for row in csv.DictReader(f)]
            self.ready = True
            print(f"[classify] YAMNet 준비 완료 ({len(self.names)}클래스)")
            self.on_result({"type": "class_status", "ready": True})
            threading.Thread(target=self._worker, daemon=True).start()
        except Exception as e:
            print(f"[classify] 로딩 실패 — 분류 비활성: {e}")
            self.on_result({"type": "class_status", "ready": False, "error": str(e)[:120]})

    # ---------- 오디오 공급 (시리얼 스레드에서 호출) ----------
    def feed_line(self, line: str):
        decoded = decode_audio_line(line)
        if decoded is None:
            return
        pcm, sr = decoded
        if sr != self.sr:
            self.sr = sr
        with self.lock:
            self.buf = np.concatenate([self.buf, pcm])
            keep = self.sr * 2   # 최근 2초만 유지
            if self.buf.size > keep:
                self.buf = self.buf[-keep:]
            self.fed += pcm.size

    # ---------- 주기 분류 ----------
    def _worker(self):
        need = int(self.sr * WINDOW_SEC)
        while True:
            time.sleep(HOP_SEC)
            with self.lock:
                if self.buf.size < need:
                    continue
                x = self.buf[-need:].copy()
            try:
                scores, _, _ = self.model(x)
                mean = scores.numpy().mean(axis=0)
            except Exception as e:
                print(f"[classify] 추론 오류: {e}")
                continue
            idx = mean.argsort()[-3:][::-1]
            top = []
            for i in idx:
                name = self.names[i] if i < len(self.names) else str(i)
                ko = KO.get(name, name)
                top.append({
                    "label": name,
                    "labelKo": ko,
                    "score": round(float(mean[i]), 3),
                    "danger": ko in DANGER,
                })
            if top and top[0]["score"] >= MIN_SCORE:
                self.on_result({"type": "class", "top": top})
