import tempfile
import unittest
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from unittest.mock import patch

from backend.data import session_figures, session_store
from backend.data.demo_simulation import _terrain_height_mm, generate_demo_point
from backend.data.scan_area import ScanAreaConfig

CONFIG = {
    "x_length_m": 10.0,
    "y_length_m": 10.0,
    "block_size_m": 5.0,
    "points_per_block": 4,
    "reflector_height_m": 1.5,
    "wheel_circumference_m": 0.5,
    "start_x_m": 0.0,
    "start_y_m": 0.0,
    "start_yaw_deg": 0.0,
    "tilt_mount_offset_deg": 0.0,
}


class SessionFiguresTest(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        root = Path(self.temp_dir.name)
        self.sessions_dir = root / "sessions"
        self.figures_dir = root / "figures"
        self.exports_dir = root / "exports"
        self.patchers = [
            patch.object(session_store, "SESSIONS_DIR", self.sessions_dir),
            patch.object(session_figures, "FIGURES_DIR", self.figures_dir),
            patch.object(session_figures, "EXPORTS_DIR", self.exports_dir),
        ]
        for patcher in self.patchers:
            patcher.start()
            self.addCleanup(patcher.stop)
        self.addCleanup(self.temp_dir.cleanup)

    def _seed_session(self):
        session_store.create_session(CONFIG, "figure session", session_id="figure-session")
        points = []
        for row in range(2):
            for col in range(2):
                for index in range(4):
                    points.append(
                        generate_demo_point(
                            "figure-session", CONFIG, row, col, index, 4, "2026-01-01T00:00:00+00:00"
                        )
                    )

        def mutate(session):
            session["points"].extend(points)
            for block in session["blocks"]:
                block["demo_simulated_count"] = 4
            return session

        return session_store.update_session("figure-session", mutate)

    def _capture_cli_plot(self, session):
        fake_png = self.figures_dir / "fake.png"
        fake_png.parent.mkdir(parents=True, exist_ok=True)
        fake_png.write_bytes(b"\x89PNG\r\n\x1a\n")
        calls = {"init": {}, "plot": None}

        class FakePlotter:
            def __init__(self, area, **kwargs):
                self.area = area
                self.save_dir = kwargs["save_dir"]
                calls["init"] = kwargs

            def plot(self, x, y, z, **kwargs):
                calls["plot"] = (x, y, z, kwargs)
                (Path(self.save_dir) / kwargs["filename"]).write_bytes(fake_png.read_bytes())
                return Path(self.save_dir) / kwargs["filename"]

        with patch.object(session_figures, "TerrainPlotter", FakePlotter):
            artifact = session_figures.build_session_artifacts(session)
        return calls, artifact

    def test_build_writes_png_csv_and_returns_paths(self):
        session = self._seed_session()
        artifact = session_figures.build_session_artifacts(session)
        figure_path = Path(artifact["figure_path"])
        csv_path = Path(artifact["csv_path"])
        self.assertTrue(figure_path.is_file())
        self.assertTrue(csv_path.is_file())
        self.assertEqual(figure_path.read_bytes()[:8], b"\x89PNG\r\n\x1a\n")
        self.assertIn("id,block_row,block_col", csv_path.read_text(encoding="utf-8"))
        self.assertEqual(artifact["point_count"], 16)
        self.assertEqual(artifact["figure_url"], "/api/sessions/figure-session/figure.png")
        self.assertEqual(artifact["csv_url"], "/api/sessions/figure-session/points.csv")

    def test_generate_persists_artifacts_and_survives_later_updates(self):
        self._seed_session()
        session = session_figures.generate_session_artifacts("figure-session")
        self.assertIn("artifacts", session)
        self.assertEqual(session["artifacts"]["figure"]["point_count"], 16)
        stored = session_store.load_session("figure-session")
        self.assertEqual(
            stored["artifacts"]["figure"]["figure_path"], session["artifacts"]["figure"]["figure_path"]
        )

        def rename(work):
            work["name"] = "renamed"
            return work

        session_store.update_session("figure-session", rename)
        stored = session_store.load_session("figure-session")
        self.assertIn("figure", stored.get("artifacts", {}))

    def test_artifact_filenames_are_stable_per_session(self):
        figure_path = session_figures.figure_path_for("figure-session")
        csv_path = session_figures.csv_path_for("figure-session")
        self.assertEqual(figure_path.name, "session_figure.png")
        self.assertEqual(csv_path.name, "scan_points.csv")
        self.assertEqual(figure_path.parent.name, "figure-session")
        self.assertEqual(csv_path.parent.name, "figure-session")

    def test_artifact_paths_reject_unsafe_session_ids(self):
        with self.assertRaises(ValueError):
            session_figures.figure_path_for("../escape")
        with self.assertRaises(ValueError):
            session_figures.csv_path_for("bad/id")

    def test_artifacts_are_invalidated_when_scan_data_changes(self):
        self._seed_session()
        session = session_figures.generate_session_artifacts("figure-session")
        self.assertIn("figure", session["artifacts"])

        def edit_point(work):
            work["points"][0]["depth_mm"] += 1
            return work

        stored = session_store.update_session("figure-session", edit_point)
        self.assertNotIn("artifacts", stored)

    def test_generation_preserves_forced_complete_status(self):
        session_store.create_session(CONFIG, "forced", session_id="figure-session")
        point = generate_demo_point("figure-session", CONFIG, 0, 0, 0, 4, "2026-01-01T00:00:00+00:00")

        def complete(work):
            work["points"].append(point)
            work["_force_status"] = "complete"
            return work

        session_store.update_session("figure-session", complete)
        session = session_figures.generate_session_artifacts("figure-session")
        self.assertEqual(session["status"], "complete")

    def test_generation_preserves_forced_draft_status(self):
        self._seed_session()

        def draft(work):
            work["_force_status"] = "draft"
            return work

        session_store.update_session("figure-session", draft)
        session = session_figures.generate_session_artifacts("figure-session")
        self.assertEqual(session["status"], "draft")

    def test_build_uses_cli_terrain_plotter(self):
        session = self._seed_session()
        calls, _ = self._capture_cli_plot(session)

        self.assertEqual(
            calls["init"],
            {
                "save_dir": self.figures_dir / "figure-session",
                "show": False,
                "pause_s": 0.0,
                "grid_step_m": 1.0,
                "height_limit_mm": 1.0,
                "z_axis_limit_mm": 2.0,
                "box_z_aspect": 25.0,
            },
        )
        x, y, z, kwargs = calls["plot"]
        self.assertEqual((len(x), len(y), len(z)), (16, 16, 16))
        self.assertEqual(kwargs["title"], "Scan")
        self.assertEqual(kwargs["filename"], ".session_figure.png.tmp.png")
        self.assertEqual(kwargs["scan_index"], 4)
        self.assertEqual(kwargs["total_scans"], 4)
        self.assertEqual((kwargs["row"], kwargs["col"]), (1, 1))
        self.assertEqual(kwargs["reflector_xy"], (5.0, 0.0))
        self.assertEqual(len(kwargs["error"]), 16)

        x_mesh, y_mesh, terrain_grid = kwargs["true_surface"]
        self.assertEqual(terrain_grid.shape, (11, 11))
        self.assertAlmostEqual(
            float(terrain_grid[3, 4]),
            _terrain_height_mm(float(x_mesh[3, 4]), float(y_mesh[3, 4])),
            places=6,
        )

    def test_build_accepts_matrix_point_counts(self):
        session = self._seed_session()

        def set_counts(work):
            work["config"]["points_per_block"] = [[1, 1], [1, 1]]
            return work

        session = session_store.update_session("figure-session", set_counts)
        calls, _ = self._capture_cli_plot(session)
        self.assertEqual(calls["plot"][3]["total_scans"], 4)

    def test_terrain_grid_is_capped_for_large_sessions(self):
        area = ScanAreaConfig(
            x_length_m=1000.0, y_length_m=1000.0, block_size_m=500.0, points_per_block=1
        )
        x_values, y_values = session_figures._grid_values(area)
        self.assertLessEqual(len(x_values) * len(y_values), session_figures.MAX_TERRAIN_GRID_CELLS)

    def test_partial_error_data_keeps_error_panel_usable(self):
        session = self._seed_session()
        session["points"][0]["raw"].pop("measurement_error_mm", None)
        calls, _ = self._capture_cli_plot(session)
        errors = calls["plot"][3]["error"]
        self.assertEqual(len(errors), 16)
        self.assertTrue(errors[0] != errors[0])
        self.assertTrue(all(value == value for value in errors[1:]))

    def test_absent_error_data_is_not_fabricated_as_zero(self):
        session = self._seed_session()
        for point in session["points"]:
            point["raw"].pop("measurement_error_mm", None)
        calls, _ = self._capture_cli_plot(session)
        errors = calls["plot"][3]["error"]
        self.assertTrue(all(value != value for value in errors))

    def test_hardware_reflector_coordinates_are_used(self):
        session = self._seed_session()
        raw = session["points"][0]["raw"]
        raw.pop("reflector_xy_m", None)
        raw["reflector_x_m"] = 3.0
        raw["reflector_y_m"] = 2.0
        calls, _ = self._capture_cli_plot(session)
        self.assertEqual(calls["plot"][3]["reflector_xy"], (3.0, 2.0))

    def test_parallel_generation_keeps_artifacts_consistent(self):
        self._seed_session()
        with ThreadPoolExecutor(max_workers=2) as executor:
            futures = [
                executor.submit(session_figures.generate_session_artifacts, "figure-session")
                for _ in range(2)
            ]
            sessions = [future.result() for future in futures]

        for session in sessions:
            artifact = session["artifacts"]["figure"]
            self.assertEqual(artifact["source_fingerprint"], session_store.session_fingerprint(session))
            self.assertEqual(Path(artifact["figure_path"]).read_bytes()[:8], b"\x89PNG\r\n\x1a\n")
            self.assertIn("id,block_row,block_col", Path(artifact["csv_path"]).read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
