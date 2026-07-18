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
import os
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
    "Whistling": "😗 휘파람",
    "Whistle": "😗 휘파람",
    "Silence": "조용함",
    "Music": "🎵 음악",
    "Water": "💧 물소리",
    "Typing": "⌨️ 타이핑",
}

DANGER = {
    "🔥 화재경보", "🚨 사이렌", "🚨 민방위 사이렌", "🚑 구급차 사이렌",
    "🚓 경찰차 사이렌", "🚒 소방차 사이렌", "📯 자동차 경적", "📯 트럭 경적",
    "😱 비명", "💥 깨지는 소리", "⏰ 경보음", "💥 쿵",
    "😗 휘파람",   # 야간 테스트용 — 사이렌 대신 휘파람으로 방향 표시 확인 (대회 전 제거 가능)
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
        # 사용자 소리 등록(few-shot 지문): 등록된 소리의 YAMNet 임베딩 목록
        self.custom_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "custom_sounds.json")
        self.custom = self._load_custom()
        self.capture = None   # 등록 녹음 진행 상태
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
            if self.capture is not None and self.capture["collected"] < self.capture["need"]:
                self.capture["buf"].append(pcm)
                self.capture["collected"] += pcm.size

    # ---------- 주기 분류 ----------
    def _worker(self):
        need = int(self.sr * WINDOW_SEC)
        while True:
            time.sleep(HOP_SEC)
            with self.lock:
                if self.buf.size < need:
                    continue
                x = self.buf[-need:].copy()
            self._finalize_capture()
            try:
                scores, emb, _ = self.model(x)
                mean = scores.numpy().mean(axis=0)
                evec = emb.numpy().mean(axis=0)
                evec = evec / (np.linalg.norm(evec) + 1e-9)
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
            self._match_custom(evec, top)
            if top and top[0]["score"] >= MIN_SCORE:
                self.on_result({"type": "class", "top": top})

    # ---------- 사용자 소리 등록 (few-shot 임베딩 지문) ----------
    def _load_custom(self):
        try:
            with open(self.custom_path, encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            return []

    def _save_custom(self):
        with open(self.custom_path, "w", encoding="utf-8") as f:
            json.dump(self.custom, f, ensure_ascii=False)

    def start_capture(self, name, label_ko, danger, seconds):
        """seconds 동안 들어오는 오디오로 소리 지문을 만들어 저장 (블로킹).
        같은 이름으로 반복 등록하면 테이크가 쌓여 인식률이 좋아진다."""
        if not self.ready:
            return {"ok": False, "error": "YAMNet이 아직 로딩 중입니다 — 잠시 후 다시"}
        done = threading.Event()
        cap = {"name": name, "labelKo": label_ko, "danger": danger,
               "need": int(self.sr * seconds), "buf": [], "collected": 0,
               "done": done, "result": None}
        with self.lock:
            if self.capture is not None:
                return {"ok": False, "error": "다른 등록이 진행 중입니다"}
            self.capture = cap
        if not done.wait(seconds + 20):
            with self.lock:
                self.capture = None
            return {"ok": False, "error": "오디오가 들어오지 않습니다 (시리얼 연결 확인)"}
        return cap["result"]

    def _finalize_capture(self):
        """등록 녹음이 다 모였으면 임베딩 지문을 만들어 저장 (worker 스레드에서 실행)."""
        with self.lock:
            cap = self.capture
            if cap is None or cap["collected"] < cap["need"]:
                return
            x = np.concatenate(cap["buf"])[:cap["need"]]
            self.capture = None
        rms = float(np.sqrt(np.mean(x * x)))
        if rms < 0.003:
            cap["result"] = {"ok": False,
                             "error": f"소리가 너무 작습니다 (RMS {rms:.4f}) — 더 가까이/크게 다시"}
            cap["done"].set()
            return
        try:
            _, emb, _ = self.model(x)
            v = emb.numpy().mean(axis=0)
            v = (v / (np.linalg.norm(v) + 1e-9)).tolist()
        except Exception as e:
            cap["result"] = {"ok": False, "error": str(e)[:120]}
            cap["done"].set()
            return
        entry = next((c for c in self.custom if c["name"] == cap["name"]), None)
        if entry is None:
            entry = {"name": cap["name"], "labelKo": cap["labelKo"],
                     "danger": cap["danger"], "threshold": 0.72, "embeddings": []}
            self.custom.append(entry)
        entry["embeddings"] = (entry["embeddings"] + [v])[-10:]   # 테이크 최대 10개
        entry["danger"] = cap["danger"]
        self._save_custom()
        print(f"[custom] '{cap['name']}' 등록 (테이크 {len(entry['embeddings'])}개, RMS {rms:.3f})")
        cap["result"] = {"ok": True, "name": cap["name"],
                         "takes": len(entry["embeddings"]), "rms": round(rms, 4)}
        cap["done"].set()

    def _match_custom(self, evec, top):
        """현재 창 임베딩을 등록 지문들과 비교, 문턱 이상이면 top1로 삽입."""
        best_sim, best_c = 0.0, None
        for c in self.custom:
            if not c.get("embeddings"):
                continue
            s = max(float(np.dot(evec, np.asarray(t, dtype=np.float32)))
                    for t in c["embeddings"])
            if s > best_sim:
                best_sim, best_c = s, c
        if best_c is None:
            return
        th = best_c.get("threshold", 0.72)
        if best_sim >= th:
            top.insert(0, {"label": best_c["name"], "labelKo": best_c["labelKo"],
                           "score": round(best_sim, 3),
                           "danger": bool(best_c.get("danger")), "custom": True})
            del top[3:]
        elif best_sim >= 0.55:
            # 문턱 튜닝용: 아깝게 미달한 유사도를 서버 콘솔에 보여준다
            print(f"[custom] {best_c['name']} 유사도 {best_sim:.2f} — 문턱 {th} 미달")

    def remove_custom(self, name):
        before = len(self.custom)
        self.custom = [c for c in self.custom if c["name"] != name]
        self._save_custom()
        return before - len(self.custom)
