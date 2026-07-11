# SSE 검증 클라이언트 — /events를 N초간 수신해 이벤트 종류별로 출력
#   사용: python tools\sse_probe.py [초] [필터...]
#   예:   python tools\sse_probe.py 15 class doa
import json
import sys
import time
import urllib.request

for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

secs = float(sys.argv[1]) if len(sys.argv) > 1 else 10
want = set(sys.argv[2:]) or None
url = "http://localhost:8765/events"

req = urllib.request.Request(url, headers={"Accept": "text/event-stream"})
resp = urllib.request.urlopen(req, timeout=secs + 5)
t0 = time.time()
counts = {}
try:
    while time.time() - t0 < secs:
        raw = resp.readline()
        if not raw:
            break
        line = raw.decode("utf-8", errors="replace").strip()
        if not line.startswith("data:"):
            continue
        try:
            rec = json.loads(json.loads(line[5:].strip()).get("line", "{}") or "{}")
        except Exception:
            continue
        t = rec.get("type", "?")
        counts[t] = counts.get(t, 0) + 1
        if want is None or t in want:
            print(f"[{time.time()-t0:5.1f}s] {json.dumps(rec, ensure_ascii=False)[:200]}")
finally:
    resp.close()
print("counts:", counts)
