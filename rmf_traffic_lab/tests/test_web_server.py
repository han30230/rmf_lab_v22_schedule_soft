import json
import sys
import threading
import unittest
from http.server import ThreadingHTTPServer
from pathlib import Path
from urllib.error import HTTPError
from urllib.request import Request, urlopen


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.scenario_templates import builtin_scenarios  # noqa: E402
from web_server import RMFWebHandler, validate_document  # noqa: E402


class WebServerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server = ThreadingHTTPServer(("127.0.0.1", 0), RMFWebHandler)
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()
        cls.base = f"http://127.0.0.1:{cls.server.server_address[1]}"

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join(timeout=2)

    def get_json(self, path):
        with urlopen(self.base + path, timeout=3) as response:
            return json.loads(response.read().decode("utf-8"))

    def test_every_builtin_scenario_is_accepted(self):
        scenarios = builtin_scenarios()
        self.assertGreaterEqual(len(scenarios), 15)
        for name, document in scenarios.items():
            with self.subTest(name=name):
                validated = validate_document(document)
                self.assertTrue(validated["nodes"])

    def test_health_catalog_and_static_ui(self):
        self.assertEqual(self.get_json("/api/health")["status"], "ok")
        catalog = self.get_json("/api/scenarios")
        self.assertTrue(any(item["key"] == "single_lane_bidirectional" for item in catalog))
        with urlopen(self.base + "/", timeout=3) as response:
            page = response.read().decode("utf-8")
        self.assertIn("RMF Traffic Core Web", page)
        self.assertIn("Schedule Database", page)
        with urlopen(self.base + "/web/app.js", timeout=3) as response:
            script = response.read().decode("utf-8")
        self.assertIn("Planner::Debug", script)

    def test_invalid_scenario_is_rejected_before_run(self):
        body = json.dumps({
            "profile": "before",
            "document": {"name": "bad", "nodes": [], "lanes": [], "robots": []},
        }).encode("utf-8")
        request = Request(
            self.base + "/api/runs",
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with self.assertRaises(HTTPError) as caught:
            urlopen(request, timeout=3)
        self.assertEqual(caught.exception.code, 400)
        payload = json.loads(caught.exception.read().decode("utf-8"))
        self.assertIn("노드", payload["error"])

    def test_building_yaml_import_endpoint(self):
        yaml_text = """
building_name: api_map
levels:
  L1:
    vertices:
      - [0, 0, A, {is_holding_point: true}]
      - [2, 0, B, {}]
    lanes:
      - [0, 1, {speed_limit: 1.0, rotationAllowed: false}]
"""
        body = json.dumps({"yaml_text": yaml_text, "level": "L1"}).encode("utf-8")
        request = Request(
            self.base + "/api/scenarios/import-yaml",
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urlopen(request, timeout=3) as response:
            payload = json.loads(response.read().decode("utf-8"))
        self.assertEqual(payload["document"]["name"], "api_map_L1")
        self.assertEqual(payload["metadata"]["directed_lane_count"], 1)
        self.assertFalse(payload["document"]["lanes"][0]["yaml_rotation_allowed"])


if __name__ == "__main__":
    unittest.main()
