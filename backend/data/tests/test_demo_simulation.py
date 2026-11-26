import math
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from backend.data import session_store
from backend.data.demo_simulation import (
    DEMO_SAMPLE_LIMIT,
    MAX_DEMO_POINTS,
    generate_demo_point,
    sample_indices,
)

CONFIG = {
    "x_length_m": 10.0,
    "y_length_m": 10.0,
    "block_size_m": 5.0,
    "points_per_block": 125,
    "reflector_height_m": 1.5,
    "wheel_circumference_m": 0.5,
    "start_x_m": 0.0,
    "start_y_m": 0.0,
    "start_yaw_deg": 0.0,
    "tilt_mount_offset_deg": 0.0,
}


class DemoSimulationTest(unittest.TestCase):
    def test_sample_indices_cover_single_point(self):
        self.assertEqual(sample_indices(1, DEMO_SAMPLE_LIMIT), [0])

    def test_sample_indices_respects_zero_limit(self):
        self.assertEqual(sample_indices(10, 0), [])

    def test_sample_indices_cover_evenly_without_duplicates(self):
        indices = sample_indices(10, 4)
        self.assertEqual(indices, [0, 3, 6, 9])
        self.assertLessEqual(len(sample_indices(1_000_000, DEMO_SAMPLE_LIMIT)), DEMO_SAMPLE_LIMIT)

    def test_point_limit_allows_one_to_a_million(self):
        self.assertEqual(MAX_DEMO_POINTS, 1_000_000)

    def test_generated_points_vary_every_visible_variable(self):
        points = [
            generate_demo_point("session-1", CONFIG, 0, 0, index, 16, "2026-01-01T00:00:00+00:00")
            for index in range(16)
        ]
        fields = (
            "x_m",
            "y_m",
            "depth_mm",
            "lidar_distance_m",
            "theta_v_deg",
            "theta_h_deg",
            "imu_pitch_deg",
            "yaw_deg",
            "platform_x_m",
            "platform_y_m",
        )
        for field in fields:
            values = {round(point[field], 6) for point in points}
            self.assertGreater(len(values), 1, field)

    def test_generated_depth_tracks_terrain_within_noise(self):
        point = generate_demo_point("session-1", CONFIG, 0, 0, 7, 16, "2026-01-01T00:00:00+00:00")
        self.assertLess(abs(point["depth_mm"] - point["raw"]["terrain_height_mm"]), 20.0)

    def test_generated_geometry_is_physically_bounded(self):
        point = generate_demo_point("session-1", CONFIG, 0, 0, 7, 16, "2026-01-01T00:00:00+00:00")
        self.assertGreater(point["lidar_distance_m"], 0.0)
        self.assertTrue(-89.0 <= point["theta_v_deg"] <= 89.0)
        self.assertTrue(-180.0 <= point["theta_h_deg"] <= 180.0)
        self.assertTrue(0.0 <= point["x_m"] <= 5.0)
        self.assertTrue(0.0 <= point["y_m"] <= 5.0)

    def test_generation_is_deterministic(self):
        first = generate_demo_point("session-1", CONFIG, 0, 0, 3, 16, "2026-01-01T00:00:00+00:00")
        second = generate_demo_point("session-1", CONFIG, 0, 0, 3, 16, "2026-01-01T00:00:00+00:00")
        self.assertEqual(first, second)


class DemoSessionStoreTest(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.patcher = patch.object(session_store, "SESSIONS_DIR", Path(self.temp_dir.name))
        self.patcher.start()
        self.addCleanup(self.patcher.stop)
        self.addCleanup(self.temp_dir.cleanup)

    def test_simulated_count_is_used_for_block_status(self):
        session = session_store.create_session(CONFIG, "demo", session_id="demo-session")
        point = generate_demo_point("demo-session", CONFIG, 0, 0, 0, 16, "2026-01-01T00:00:00+00:00")

        def mutate(work):
            work["points"].append(point)
            work["blocks"][0]["demo_simulated_count"] = 123
            return work

        session = session_store.update_session("demo-session", mutate)
        block = session["blocks"][0]
        self.assertEqual(block["status"], "scanning")

        def complete(work):
            work["blocks"][0]["demo_simulated_count"] = 124
            return work

        session = session_store.update_session("demo-session", complete)
        self.assertEqual(session["blocks"][0]["status"], "complete")

    def test_clear_blocks_resets_simulated_count(self):
        session_store.create_session(CONFIG, "demo", session_id="demo-session")
        point = generate_demo_point("demo-session", CONFIG, 0, 0, 0, 16, "2026-01-01T00:00:00+00:00")

        def mutate(work):
            work["points"].append(point)
            work["blocks"][0]["demo_simulated_count"] = 124
            return work

        session_store.update_session("demo-session", mutate)
        session = session_store.clear_blocks("demo-session", [{"row": 0, "col": 0}], True)
        block = session["blocks"][0]
        self.assertEqual(block["demo_simulated_count"], 0)
        self.assertEqual(block["status"], "needs_rescan")

    def test_demo_next_index_is_preserved_and_reset(self):
        session_store.create_session(CONFIG, "demo", session_id="demo-session")

        def mutate(work):
            work["blocks"][0]["demo_next_index"] = 42
            return work

        session = session_store.update_session("demo-session", mutate)
        self.assertEqual(session["blocks"][0]["demo_next_index"], 42)
        session = session_store.clear_blocks("demo-session", [{"row": 0, "col": 0}], True)
        self.assertEqual(session["blocks"][0]["demo_next_index"], 0)

    def test_reconfigure_preserves_demo_progress(self):
        session_store.create_session(CONFIG, "demo", session_id="demo-session")

        def mutate(work):
            work["blocks"][0]["demo_next_index"] = 42
            work["blocks"][0]["demo_simulated_count"] = 7
            return work

        session_store.update_session("demo-session", mutate)
        session = session_store.reconfigure_session("demo-session", {})
        block = session["blocks"][0]
        self.assertEqual(block["demo_next_index"], 42)
        self.assertEqual(block["demo_simulated_count"], 7)

    def test_reconfigure_maps_demo_progress_to_new_geometry(self):
        session_store.create_session(CONFIG, "demo", session_id="demo-session")
        point = generate_demo_point("demo-session", CONFIG, 0, 1, 0, 16, "2026-01-01T00:00:00+00:00")

        def mutate(work):
            work["points"].append(point)
            block = next(b for b in work["blocks"] if b["row"] == 0 and b["col"] == 1)
            block["demo_next_index"] = 42
            block["demo_simulated_count"] = 7
            return work

        session_store.update_session("demo-session", mutate)
        session = session_store.reconfigure_session("demo-session", {
            "x_length_m": 10.0,
            "y_length_m": 10.0,
            "block_size_m": 10.0,
        })
        block = session["blocks"][0]
        self.assertEqual(len(session["blocks"]), 1)
        self.assertEqual(block["demo_next_index"], 42)
        self.assertEqual(block["demo_simulated_count"], 7)
        self.assertEqual(session["points"][0]["block_row"], 0)
        self.assertEqual(session["points"][0]["block_col"], 0)

    def test_summary_percent_is_capped_at_one_hundred(self):
        session_store.create_session(CONFIG, "demo", session_id="demo-session")

        def mutate(work):
            work["blocks"][0]["demo_simulated_count"] = 1000
            return work

        session = session_store.update_session("demo-session", mutate)
        summary = session_store.session_summary(session)
        self.assertEqual(summary["completion"]["percent"], 100.0)


if __name__ == "__main__":
    unittest.main()
