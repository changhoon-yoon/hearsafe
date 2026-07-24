# ============================================================
#  DOA 대시보드 중계 서버
#  ESP32(COM3) 시리얼 → HTTP/SSE 중계 → PC·iPad·폰 브라우저 동시 시청
#
#  실행:  python dashboard\serve.py
#  접속:  PC     http://localhost:8765
#         iPad   http://<PC IP>:8765   (같은 WiFi, 실행 시 IP 출력됨)
#
#  ※ COM3은 이 서버가 독점 — pio monitor / Web Serial 연결은 먼저 닫을 것
#  ※ 첫 실행 시 Windows 방화벽 팝업 → "허용" 클릭 (iPad 접속에 필요)
# ============================================================
import argparse
import collections
import ipaddress
import json
import math
import os
import queue
import socket
import sys
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import serial

# Windows 콘솔/리다이렉트가 cp949일 때 이모지·한글 출력으로 죽지 않게
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

PORT = 8765
COM = "COM3"
BAUD = 921600   # 펌웨어 AUDIO_STREAM(오디오 스트리밍) 규격과 일치
HTML = os.path.join(os.path.dirname(os.path.abspath(__file__)), "doa-compass.html")
CLIENT_QUEUE_SIZE = 100

clients = set()
clients_lock = threading.Lock()
latest_status = "::status:: 서버 시작됨 — 시리얼 연결 대기 중"
ser_handle = None   # 명령 전송 핸들: 시리얼 포트 또는 소켓 어댑터
NET = None          # "--net host[:port]" 지정 시 (호스트, 포트) — 무선 모드
latest_class_status = None   # 마지막 class_status JSON — 늦게 접속한 브라우저에도 전달
classifier = None   # main에서 초기화 (없어도 서버는 정상 동작)

# ---- 세션 기록/재현 (하드웨어 없이 실행 검증용) ----
# 실제 장치에서 수신한 원본 라인을 타임스탬프와 함께 저장해두면(--record),
# 하드웨어 없는 PC에서도 --replay로 같은 process_line() 경로를 그대로 태워
# 방향 판정·소리 분류·경보 융합까지 동일하게 재현할 수 있다 (심사용 실행 수단).
_record_fp = None
_record_t0 = None


def record_line(raw: str):
    global _record_t0
    if _record_fp is None:
        return
    if _record_t0 is None:
        _record_t0 = time.time()
    rec = json.dumps({"t": round(time.time() - _record_t0, 3), "raw": raw}, ensure_ascii=False)
    _record_fp.write(rec + "\n")
    _record_fp.flush()


def handle_line(raw: str):
    """장치(시리얼/무선)에서 온 원본 라인 1개: 필요 시 기록 후 공용 처리."""
    record_line(raw)
    process_line(raw)

# ---- 분류(무슨 소리) × DOA(어느 방향) 융합 ----
# 최근 DOA 이벤트를 기억해 두고, 위험 소리로 분류되는 순간 시간창 안의
# 방향과 묶어 {"type":"alert"} 이벤트를 발행한다.
recent_doa = collections.deque(maxlen=40)   # (time.time(), doa dict)
DOA_FUSE_WINDOW = 2.0    # 분류 창(~1s) + 전송 지연을 덮는 결합 허용 시간
ALERT_MIN_SCORE = 0.30   # 이 점수 이상 분류일 때만 방향과 융합
# 방향 화살표를 만들 필요가 없는 배경/무음 라벨
IGNORE_LABELS = {
    "Silence", "Inside, small room", "Inside, large room or hall",
    "Outside, rural or natural", "Outside, urban or manmade",
    "White noise", "Pink noise", "Static", "Environmental noise", "Hum",
}


# ---- 지속 소음원 습관화 ----
# 팬·공기청정기처럼 한 방향에서 쉬지 않고 나는 소리는 이벤트를 계속 발생시켜
# 불응기를 점유하고(진짜 소리가 확률적으로 씹힘) 화살표를 도배한다.
# 최근 10초에 같은 방향(±25°) 이벤트가 15회 이상이면 그 방향을 억제한다.
# (박수 3~4회 세션은 ~8회 수준이라 억제되지 않음)
doa_history = collections.deque(maxlen=300)   # (time, phi)
HABIT_WINDOW = 10.0
HABIT_ARC = 25.0
HABIT_COUNT = 15
_habit_notice = {}   # 방향(30° 구간) → 마지막 안내 시각


def _angdiff(a, b):
    d = abs(a - b) % 360.0
    return d if d <= 180.0 else 360.0 - d


def habituated(phi: float) -> bool:
    now = time.time()
    cnt = sum(1 for (t, p) in doa_history
              if now - t <= HABIT_WINDOW and _angdiff(p, phi) <= HABIT_ARC)
    if cnt < HABIT_COUNT:
        return False
    zone = int(phi // 30) * 30
    if now - _habit_notice.get(zone, 0) > 30:
        _habit_notice[zone] = now
        print(f"[habit] 🔇 지속 소음 방향 억제 중: ~{phi:.0f}° ({cnt}회/10초)")
        broadcast(f"::status:: 🔇 지속 소음 방향(~{phi:.0f}°) 자동 무시 중")
    return True


# ---- 반사 플립 가드 ----
# 박수 직접음 판정 직후 0.5~1초 뒤 벽 반사가 "반대편" 이벤트를 하나 더 만드는
# 문제(실측: 왼쪽 박수 후 -20대 lag 반사 클러스터). 첫 판정은 지연 없이 통과시키고,
# 직후 짧은 시간 안의 좌우 모순 판정만 반사로 보고 억제한다.
FLIP_GUARD_SEC = 1.2
_lr_last = {"side": 0, "t": 0.0}


def _lr_side(rec):
    """이벤트의 좌우 반구 판정: -1=왼쪽, +1=오른쪽, 0=판정 불가/정면·후면."""
    phi = rec.get("phi")
    if isinstance(phi, (int, float)):
        c = math.cos(math.radians(phi))
    elif rec.get("axis") == "x" and isinstance(rec.get("candidateA"), (int, float)):
        c = -1.0 if 90 < rec["candidateA"] < 270 else 1.0
    else:
        return 0
    if abs(c) < 0.2:
        return 0
    return -1 if c < 0 else 1


def remember_doa(line: str):
    """doa 라인 처리: 습관화 판정 후 억제 여부를 돌려준다 (True=억제)."""
    try:
        rec = json.loads(line)
        if rec.get("type") != "doa":
            return False
        now = time.time()
        side = _lr_side(rec)
        if side != 0:
            if (_lr_last["side"] not in (0, side)
                    and now - _lr_last["t"] <= FLIP_GUARD_SEC):
                return True   # 직전 판정과 좌우 모순 + 시간창 안 = 반사로 억제
            _lr_last["side"] = side
            _lr_last["t"] = now
        phi = rec.get("phi")
        if isinstance(phi, (int, float)):
            doa_history.append((time.time(), float(phi)))
            if habituated(float(phi)):
                return True
        recent_doa.append((time.time(), rec))
    except Exception:
        pass
    return False


def on_class_result(payload):
    """분류기 콜백: 결과를 중계하고, 인식된 소리를 최근 방향과 융합해 발행.
    위험 소리 → type=alert, 일반 소리 → type=sound (대시보드가 모드에 따라 표시)."""
    global latest_class_status
    if payload.get("type") == "class_status":
        latest_class_status = json.dumps(payload, ensure_ascii=False)
    broadcast(json.dumps(payload, ensure_ascii=False))
    if payload.get("type") != "class":
        return
    top = payload["top"][0]
    if top["label"] in IGNORE_LABELS or top["score"] < ALERT_MIN_SCORE:
        return
    now = time.time()
    cands = [(t, r) for (t, r) in list(recent_doa) if now - t <= DOA_FUSE_WINDOW]
    best = None
    for t, r in reversed(cands):            # 2D 판정 우선, 그다음 최신
        if r.get("mode") == "2d":
            best = (t, r)
            break
    if best is None and cands:
        best = cands[-1]
    if best is None:
        # 방향 정보가 없어도 위험 소리는 알림 자체를 놓치면 안 된다 (방향 없이 발행).
        # 일반 소리는 화살표를 만들 수 없으므로 발행하지 않음.
        if not top.get("danger"):
            return
        fused = {
            "type": "alert",
            "label": top["label"], "labelKo": top["labelKo"], "score": top["score"],
            "danger": True, "phi": None, "mode": None,
            "candidateA": None, "candidateB": None, "strength": None, "ageMs": None,
        }
        broadcast(json.dumps(fused, ensure_ascii=False))
        print(f"[alert] {top['labelKo']} {top['score']:.2f} → 방향 없음")
        return
    t, r = best
    fused = {
        "type": "alert" if top.get("danger") else "sound",
        "label": top["label"], "labelKo": top["labelKo"], "score": top["score"],
        "danger": bool(top.get("danger")),
        "phi": r.get("phi"), "mode": r.get("mode"),
        "candidateA": r.get("candidateA"), "candidateB": r.get("candidateB"),
        "strength": r.get("strength"), "ageMs": int((now - t) * 1000),
    }
    broadcast(json.dumps(fused, ensure_ascii=False))
    print(f"[{fused['type']}] {top['labelKo']} {top['score']:.2f} → phi={r.get('phi')} ({r.get('mode')})")


def broadcast(line: str):
    global latest_status
    with clients_lock:
        if line.startswith("::status::"):
            latest_status = line
        for q in list(clients):
            try:
                q.put_nowait(line)
            except queue.Full:
                # 느린 화면에는 오래된 로그보다 최신 방향을 보여준다.
                try:
                    q.get_nowait()
                    q.put_nowait(line)
                except (queue.Empty, queue.Full):
                    pass


def process_line(line: str):
    """수신 라인 1개 처리 (시리얼/무선 공용)."""
    if not line:
        return
    # 오디오 스트림은 브라우저로 중계하지 않고 분류기에만 공급 (대역폭 절약)
    if line.startswith('{"type":"audio"'):
        if classifier is not None:
            classifier.feed_line(line)
        return
    if line.startswith('{"type":"doa"'):
        if remember_doa(line):   # 습관화/플립가드 억제 시 중계 생략
            return
    broadcast(line)


def net_thread():
    """ESP32의 WiFi TCP 서버(doa-node.local:9000)에 접속해 라인 스트림 수신."""
    global ser_handle
    host, port = NET
    while True:
        try:
            sock = socket.create_connection((host, port), timeout=8)
            sock.settimeout(20)

            class _Cmd:                      # /zeroyaw 등 명령용 어댑터
                def write(self, b):
                    sock.sendall(b)
            ser_handle = _Cmd()
            print(f"[net] {host}:{port} 연결됨 (무선)")
            broadcast("::status:: 무선(WiFi) 연결됨 — 듣는 중")
            acc = bytearray()
            while True:
                chunk = sock.recv(16384)
                if not chunk:
                    raise ConnectionError("원격 종료")
                acc += chunk
                if len(acc) > 1_000_000:
                    del acc[:-4096]
                while True:
                    nl = acc.find(b"\n")
                    if nl < 0:
                        break
                    raw = acc[:nl]
                    del acc[:nl + 1]
                    handle_line(raw.decode("utf-8", errors="replace").strip())
        except Exception as e:
            ser_handle = None
            print(f"[net] {host}:{port} 재접속 대기… ({e})")
            broadcast(f"::status:: 무선 재접속 중… ({host})")
            time.sleep(3)


def serial_thread():
    """COM 포트를 계속 재시도하며 읽어서 전 클라이언트에 중계."""
    while True:
        try:
            # 포트를 열기 전에 DTR/RTS를 내려 연결 순간의 보드 리셋을 피한다.
            sp = serial.Serial()
            sp.port = COM
            sp.baudrate = BAUD
            sp.timeout = 1
            sp.dtr = False
            sp.rts = False
            sp.open()
            with sp:
                global ser_handle
                ser_handle = sp
                print(f"[serial] {COM} 연결됨")
                broadcast("::status:: 시리얼 연결됨 — 듣는 중")
                try:
                    sp.set_buffer_size(rx_size=1 << 20)   # OS 수신 버퍼 넉넉히 (드롭 방지)
                except Exception:
                    pass
                # ※ readline() 금지 — pyserial readline은 1바이트씩 읽어서
                #   오디오 스트리밍(~60KB/s)을 못 따라가고, OS 버퍼에 데이터가
                #   계속 쌓여 지연이 수 초씩 누적된다 (실측 5~10초). 일괄 읽기로 처리.
                acc = bytearray()
                last_lag_check = time.time()
                while True:
                    chunk = sp.read(sp.in_waiting or 1)
                    if not chunk:
                        continue
                    acc += chunk
                    if len(acc) > 1_000_000:   # 개행 없는 비정상 폭주 방어
                        del acc[:-4096]
                    while True:
                        nl = acc.find(b"\n")
                        if nl < 0:
                            break
                        raw = acc[:nl]
                        del acc[:nl + 1]
                        handle_line(raw.decode("utf-8", errors="replace").strip())
                    now = time.time()
                    if now - last_lag_check >= 5.0:
                        last_lag_check = now
                        lag = sp.in_waiting
                        if lag > 16384:
                            print(f"[serial] 경고: 수신 밀림 {lag}B (~{lag / 60000:.1f}초 지연)")
        except Exception as e:
            ser_handle = None
            print(f"[serial] {COM} 대기 중… ({e})")
            broadcast(f"::status:: 시리얼 대기 중 ({COM} 사용 불가 — 다른 프로그램이 잡고 있나?)")
            time.sleep(2)


def replay_thread(path: str):
    """--record로 저장해둔 실제 세션 로그를 원래 타이밍대로 재생.
    하드웨어(ESP32) 없이도 process_line()과 동일 경로로 방향 판정·소리
    분류·경보 융합을 그대로 재현한다 (심사자가 실행 가능한 형태로 확인하는 용도)."""
    print(f"[replay] 재현 세션 로드: {path}")
    broadcast(f"::status:: 🔁 재현 모드 — 저장된 세션 재생 중 ({os.path.basename(path)})")
    with open(path, "r", encoding="utf-8") as f:
        lines = [json.loads(ln) for ln in f if ln.strip()]
    t_prev = 0.0
    for rec in lines:
        wait = rec["t"] - t_prev
        if wait > 0:
            time.sleep(min(wait, 5.0))   # 개별 공백은 최대 5초로 눌러서 재생이 늘어지지 않게
        t_prev = rec["t"]
        process_line(rec["raw"])
    print("[replay] 세션 재생 종료")
    broadcast("::status:: 🔁 재현 세션 종료 (파일 끝)")


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path in ("/", "/index.html"):
            try:
                with open(HTML, "rb") as f:
                    body = f.read()
            except OSError:
                self.send_response(500)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        elif self.path == "/events":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream; charset=utf-8")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            q = queue.Queue(maxsize=CLIENT_QUEUE_SIZE)
            with clients_lock:
                q.put_nowait(latest_status)
                if latest_class_status:   # YAMNet 준비 완료가 접속 전이었어도 상태 전달
                    q.put_nowait(latest_class_status)
                clients.add(q)
            print(f"[sse] 시청자 접속: {self.client_address[0]} (총 {len(clients)}명)")
            try:
                while True:
                    try:
                        line = q.get(timeout=15)
                        payload = "data: " + json.dumps({"line": line}) + "\n\n"
                    except queue.Empty:
                        payload = ": keepalive\n\n"   # 연결 유지 핑
                    self.wfile.write(payload.encode("utf-8"))
                    self.wfile.flush()
            except Exception:
                pass
            finally:
                with clients_lock:
                    clients.discard(q)
                print(f"[sse] 시청자 종료: {self.client_address[0]} (총 {len(clients)}명)")
        elif self.path.startswith("/register"):
            # 사용자 소리 등록: 지금부터 sec초 동안 마이크 소리를 지문으로 저장
            qs = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
            name = (qs.get("name") or [""])[0].strip()
            sec = float((qs.get("sec") or ["6"])[0])
            danger = (qs.get("danger") or ["0"])[0] in ("1", "true")
            if classifier is None:
                body = {"ok": False, "error": "분류기가 꺼져 있습니다 (--no-classify?)"}
            elif not name:
                body = {"ok": False, "error": "이름이 없습니다 (?name=...)"}
            else:
                ko = (qs.get("ko") or [""])[0].strip() or ("📌 " + name)
                broadcast(f"::status:: 🎙 '{name}' 등록 녹음 중 — 지금 소리를 들려주세요!")
                body = classifier.start_capture(name, ko, danger, min(max(sec, 2.0), 15.0))
                broadcast("::status:: 시리얼 연결됨 — 듣는 중")
            self._json(body)

        elif self.path == "/imuretry":
            # IMU 재초기화 요청 — 접촉 복구 후 RST 없이 무선으로 재시도
            if ser_handle is not None:
                try:
                    ser_handle.write(b"I")
                    self._json({"ok": True})
                except Exception as e:
                    self._json({"ok": False, "error": str(e)[:80]})
            else:
                self._json({"ok": False, "error": "장치 미연결"})

        elif self.path == "/zeroyaw":
            # 현재 자세를 IMU yaw 영점으로 — 착용 정자세 기준 설정
            if ser_handle is not None:
                try:
                    ser_handle.write(b"Z")
                    self._json({"ok": True})
                except Exception as e:
                    self._json({"ok": False, "error": str(e)[:80]})
            else:
                self._json({"ok": False, "error": "시리얼 미연결"})

        elif self.path == "/customs":
            items = [] if classifier is None else [
                {"name": c["name"], "labelKo": c["labelKo"], "danger": c["danger"],
                 "takes": len(c.get("embeddings", [])),
                 "threshold": c.get("threshold", 0.72),
                 "clip": bool(c.get("clip"))}
                for c in classifier.custom]
            self._json({"ok": True, "sounds": items})

        elif self.path.startswith("/clip"):
            # 등록 소리 미리듣기 WAV 제공
            qs = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
            name = (qs.get("name") or [""])[0]
            entry = next((c for c in (classifier.custom if classifier else [])
                          if c["name"] == name), None)
            path = (os.path.join(classifier.clip_dir, entry["clip"])
                    if entry and entry.get("clip") else None)
            if not path or not os.path.exists(path):
                self.send_response(404)
                self.end_headers()
                return
            with open(path, "rb") as f:
                data = f.read()
            self.send_response(200)
            self.send_header("Content-Type", "audio/wav")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        elif self.path.startswith("/unregister"):
            qs = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
            name = (qs.get("name") or [""])[0].strip()
            n = classifier.remove_custom(name) if (classifier and name) else 0
            self._json({"ok": n > 0, "removed": n})

        else:
            self.send_response(404)
            self.end_headers()

    def _json(self, obj):
        data = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, *args):  # 기본 액세스 로그 끄기
        pass


def lan_ips() -> list:
    """모든 IPv4 후보를 모아 사설 WiFi 대역(192.168.x) 우선으로 정렬.
    (VPN 터널 주소(10.x, 100.x)가 기본 라우트를 차지해도 진짜 LAN IP를 보여주기 위함)"""
    ips = set()
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ips.add(info[4][0])
    except Exception:
        pass
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ips.add(s.getsockname()[0])
    except Exception:
        pass
    finally:
        s.close()
    ips.discard("127.0.0.1")
    def priority(ip: str):
        addr = ipaddress.ip_address(ip)
        if ip.startswith("192.168."):
            return 0, ip
        if addr.is_private:
            return 1, ip
        return 2, ip

    return sorted(ips, key=priority)


class Server(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def parse_args():
    parser = argparse.ArgumentParser(description="ESP32 DOA serial-to-SSE dashboard server")
    parser.add_argument("--com", default=os.getenv("DOA_COM", COM), help="serial port (default: COM3)")
    parser.add_argument("--baud", type=int, default=int(os.getenv("DOA_BAUD", BAUD)))
    parser.add_argument("--port", type=int, default=int(os.getenv("DOA_PORT", PORT)))
    parser.add_argument("--no-classify", action="store_true",
                        help="YAMNet 소리 분류 끄기 (TF 미설치 환경 등)")
    parser.add_argument("--net", default=os.getenv("DOA_NET", ""),
                        help="무선 모드: ESP32 주소 (예: doa-node.local 또는 192.168.0.50:9000)")
    parser.add_argument("--record", default="",
                        help="수신 원본 라인을 타임스탬프와 함께 파일로 저장 (재현용 세션 캡처)")
    parser.add_argument("--replay", default="",
                        help="--record로 저장한 세션 파일을 재생 (하드웨어 없이 실행 확인용, --com/--net 대신 사용)")
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    COM, BAUD, PORT = args.com, args.baud, args.port
    if not args.no_classify:
        try:
            from classifier import SoundClassifier
            classifier = SoundClassifier(on_result=on_class_result)
            print("[classify] 분류기 시작 (YAMNet은 백그라운드 로딩)")
        except Exception as e:
            print(f"[classify] 분류기 비활성: {e}")
            latest_class_status = json.dumps(
                {"type": "class_status", "ready": False, "error": str(e)[:120]},
                ensure_ascii=False)
    else:
        latest_class_status = json.dumps(
            {"type": "class_status", "ready": False, "error": "--no-classify로 꺼짐"},
            ensure_ascii=False)
    if args.record:
        _record_fp = open(args.record, "w", encoding="utf-8")
        print(f"[record] 세션 기록: {args.record}")

    if args.replay:
        threading.Thread(target=replay_thread, args=(args.replay,), daemon=True).start()
    elif args.net:
        h, _, p = args.net.partition(":")
        NET = (h, int(p) if p else 9000)
        threading.Thread(target=net_thread, daemon=True).start()
        print(f"[net] 무선 모드: {NET[0]}:{NET[1]} 접속 시도")
    else:
        threading.Thread(target=serial_thread, daemon=True).start()
    print("=" * 50)
    mode = "  (재현 모드)" if args.replay else ("  (무선 모드)" if args.net else "")
    print("  🧭 DOA 대시보드 중계 서버" + mode)
    print(f"  PC   : http://localhost:{PORT}")
    for i, ip in enumerate(lan_ips()):
        tag = "iPad : " if i == 0 else "  또는 "
        note = "   ← 같은 WiFi 후보" if ipaddress.ip_address(ip).is_private else "   (VPN 주소일 수 있음)"
        print(f"  {tag}http://{ip}:{PORT}{note}")
    print("  종료 : Ctrl+C")
    print("=" * 50)
    server = Server(("0.0.0.0", PORT), Handler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n[server] 종료")
    finally:
        server.server_close()
