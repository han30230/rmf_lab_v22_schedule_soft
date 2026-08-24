import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.building_map_import import (  # noqa: E402
    available_levels,
    convert_building_map_yaml,
)
from run import compile_custom_scenario  # noqa: E402


BUILDING_YAML = """
building_name: p4_track_v05_260317_3
doors: {}
levels:
  L1:
    vertices:
      - [0.0, 0.0, START, {is_holding_point: true, is_parking_spot: true}]
      - [2.0, 0.0, GATE, {is_passthrough_point: true}]
      - [4.0, 0.0, GOAL, {is_holding_point: true}]
    lanes:
      - - 0
        - 1
        - orientation: 0.0
          speed_limit: 1.0
          rotationAllowed: false
          corridor:
            leftWidth: 0.7
            rightWidth: 0.7
            corridorRefPoint: CONTOUR
      - [1, 2, {bidirectional: true, speed_limit: 0.8}]
  L2:
    vertices:
      - [0, 0, A, {}]
      - [1, 0, B, {}]
    lanes:
      - [0, 1, {}]
"""


class BuildingMapImportTest(unittest.TestCase):
    def test_converts_selected_level_and_preserves_lane_metadata(self):
        result = convert_building_map_yaml(BUILDING_YAML, "L1")
        document = result["document"]
        metadata = result["metadata"]
        self.assertEqual(document["name"], "p4_track_v05_260317_3_L1")
        self.assertEqual(document["map"], "L1")
        self.assertEqual(len(document["nodes"]), 3)
        self.assertTrue(document["nodes"][0]["holding"])
        self.assertTrue(document["nodes"][0]["parking"])
        self.assertFalse(document["lanes"][0]["bidirectional"])
        self.assertTrue(document["lanes"][1]["bidirectional"])
        self.assertEqual(document["lanes"][0]["speed_limit"], 1.0)
        self.assertFalse(document["lanes"][0]["yaml_rotation_allowed"])
        self.assertEqual(document["lanes"][0]["corridor_left_width"], 0.7)
        self.assertEqual(document["lanes"][0]["corridor_right_width"], 0.7)
        self.assertEqual(document["lanes"][0]["corridor_ref_point"], "CONTOUR")
        self.assertEqual(metadata["directed_lane_count"], 3)
        self.assertEqual(document["robots"], [])

    def test_lists_and_selects_multiple_levels(self):
        self.assertEqual(available_levels(BUILDING_YAML), ["L1", "L2"])
        result = convert_building_map_yaml(BUILDING_YAML, "L2")
        self.assertEqual(result["document"]["map"], "L2")
        self.assertEqual(len(result["document"]["nodes"]), 2)

    def test_requires_vertices_not_only_lane_excerpt(self):
        with self.assertRaisesRegex(ValueError, "vertices"):
            convert_building_map_yaml("levels:\n  L1:\n    lanes:\n      - [0, 1, {}]\n")

    def test_imported_map_compiles_for_real_runner_after_robot_is_added(self):
        document = convert_building_map_yaml(BUILDING_YAML, "L1")["document"]
        document["robots"] = [{
            "name": "R0", "start": 0, "goal": 2, "yaw": 0.0,
            "start_time_s": 0.0,
        }]
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "scenario.json"
            output = Path(directory) / "scenario.rmf"
            source.write_text(json.dumps(document, ensure_ascii=False), encoding="utf-8")
            name, warnings = compile_custom_scenario(source, output)
            compiled = output.read_text(encoding="utf-8")
        self.assertEqual(name, "p4_track_v05_260317_3_L1")
        self.assertIn("LANE\t0\t1\t1", compiled)
        self.assertIn("ROBOT\tR0\t0\t2", compiled)
        self.assertTrue(any("rotationAllowed" in warning for warning in warnings))

    def test_packaged_yaml_example_is_importable(self):
        source = ROOT / "scenarios" / "building_map_yaml_example.yaml"
        result = convert_building_map_yaml(source.read_text(encoding="utf-8"), "L1")
        self.assertEqual(result["metadata"]["node_count"], 6)
        self.assertEqual(result["metadata"]["source_lane_count"], 10)
        self.assertEqual(result["metadata"]["corridor_count"], 8)


if __name__ == "__main__":
    unittest.main()
