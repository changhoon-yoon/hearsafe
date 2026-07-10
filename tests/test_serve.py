import json
import queue
import sys
import threading
import unittest
import urllib.request
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "dashboard"))

import serve  # noqa: E402


class ServeTests(unittest.TestCase):
    def setUp(self):
        with serve.clients_lock:
            serve.clients.clear()
            serve.latest_status = "::status:: test-ready"

    def test_broadcast_drops_oldest_item_for_slow_client(self):
        client = queue.Queue(maxsize=2)
        with serve.clients_lock:
            serve.clients.add(client)

        serve.broadcast("first")
        serve.broadcast("second")
        serve.broadcast("newest")

        self.assertEqual(client.get_nowait(), "second")
        self.assertEqual(client.get_nowait(), "newest")

    def test_dashboard_and_initial_sse_status(self):
        server = serve.Server(("127.0.0.1", 0), serve.Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        base = f"http://127.0.0.1:{server.server_port}"
        try:
            with urllib.request.urlopen(base + "/", timeout=2) as response:
                self.assertEqual(response.status, 200)
                self.assertIn("DOA 실시간 나침반", response.read().decode("utf-8"))

            with urllib.request.urlopen(base + "/events", timeout=2) as response:
                line = response.readline().decode("utf-8").strip()
                self.assertTrue(line.startswith("data: "))
                payload = json.loads(line.removeprefix("data: "))
                self.assertEqual(payload["line"], "::status:: test-ready")
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2)


if __name__ == "__main__":
    unittest.main()
