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
import ipaddress
import json
import os
import queue
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import serial

PORT = 8765
COM = "COM3"
BAUD = 115200
HTML = os.path.join(os.path.dirname(os.path.abspath(__file__)), "doa-compass.html")
CLIENT_QUEUE_SIZE = 100

clients = set()
clients_lock = threading.Lock()
latest_status = "::status:: 서버 시작됨 — 시리얼 연결 대기 중"


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
                print(f"[serial] {COM} 연결됨")
                broadcast("::status:: 시리얼 연결됨 — 듣는 중")
                while True:
                    raw = sp.readline()
                    if not raw:
                        continue
                    line = raw.decode("utf-8", errors="replace").strip()
                    if line:
                        broadcast(line)
        except Exception as e:
            print(f"[serial] {COM} 대기 중… ({e})")
            broadcast(f"::status:: 시리얼 대기 중 ({COM} 사용 불가 — 다른 프로그램이 잡고 있나?)")
            time.sleep(2)


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
        else:
            self.send_response(404)
            self.end_headers()

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
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    COM, BAUD, PORT = args.com, args.baud, args.port
    threading.Thread(target=serial_thread, daemon=True).start()
    print("=" * 50)
    print("  🧭 DOA 대시보드 중계 서버")
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
