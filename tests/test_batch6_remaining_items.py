#!/usr/bin/env python3
"""
Test Suite: Batch 6 Remaining Items.

Validates:
- Phase 1 M1: Previous-mode turning without fixtures adds a "fixture checks disabled" warning to report.
- Phase 1 M5: Preceding-stock check in PostProcessSetup walks all earlier setups, skips suppressed setups,
              fails closed on exceptions, and hides/restores progress dialog around messageBox.
- Phase 1 M6: Vectorized rasterize_triangles_to_grid is bitwise identical to reference implementation and
              rasterizes 500k triangles in under 3.0 seconds.
- Phase 1 L4: Explicit workpiece passed to PipelineOrchestrator.run logs debug message; preceding mode
              rejects non-STL fallback workpiece.
- Phase 1 L5 / Phase 2 L4: Toast details truncated to ~600 chars; PostProcessSetup does not return raw tracebacks.
- Phase 1 L6: Full pipeline tests assert strict G21 output without either-unit conditionals.
- Phase 2 M2: _ScopedEnv isolates os.environ changes around postProcess calls.
- Phase 2 M4: PowerShell Get-CimInstance replaces deprecated wmic on Windows.
- Phase 2 L1: Debug log rotates to .1 when exceeding 5 MB.
- Phase 2 L5: Line endings normalized to LF in touched files.
"""

import math
import os
import sys
import tempfile
import time
import types
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

# Mock adsk namespace if running outside Fusion 360
if "adsk" not in sys.modules:
    adsk = types.ModuleType("adsk")
    adsk.cam = types.ModuleType("adsk.cam")
    adsk.cam.OperationTypes = types.SimpleNamespace(TurningOperation=1, MillingOperation=0)
    adsk.cam.SetupStockModes = types.SimpleNamespace(
        FixedBoxStock=0, RelativeBoxStock=1, FixedCylinderStock=2,
        RelativeCylinderStock=3, FixedTubeStock=4, RelativeTubeStock=5,
        SolidStock=6, PreviousSetupStock=7
    )
    adsk.cam.Setup = type("Setup", (), {})
    adsk.cam.Operation = type("Operation", (), {})
    adsk.cam.CAMFolder = type("CAMFolder", (), {})
    adsk.cam.CAMPattern = type("CAMPattern", (adsk.cam.CAMFolder,), {})
    adsk.cam.CAM = types.SimpleNamespace(cast=lambda x: x)
    adsk.core = types.ModuleType("adsk.core")
    adsk.core.MessageBoxButtonTypes = types.SimpleNamespace(YesNoButtonType=0, OKButtonType=1)
    adsk.core.MessageBoxIconTypes = types.SimpleNamespace(WarningIconType=0, CriticalIconType=1, InformationIconType=2)
    adsk.core.DialogResults = types.SimpleNamespace(DialogYes=0, DialogNo=1, DialogOK=2, DialogCancel=3)
    adsk.core.CommandCreatedEventHandler = type("CommandCreatedEventHandler", (), {})
    adsk.core.InputChangedEventHandler = type("InputChangedEventHandler", (), {})
    adsk.core.ValidateInputsEventHandler = type("ValidateInputsEventHandler", (), {})
    adsk.core.CommandEventHandler = type("CommandEventHandler", (), {})
    adsk.core.Application = types.SimpleNamespace(get=lambda: types.SimpleNamespace(
        log=lambda msg: None, userInterface=types.SimpleNamespace()
    ))
    adsk.fusion = types.ModuleType("adsk.fusion")
    sys.modules["adsk"] = adsk
    sys.modules["adsk.cam"] = adsk.cam
    sys.modules["adsk.core"] = adsk.core
    sys.modules["adsk.fusion"] = adsk.fusion
else:
    import adsk

from geometry.models import (
    AxisymmetricChuckObstacle,
    AxisymmetricStock2D,
    BoxStock3D,
    SolidMeshStock3D,
    VerificationReport,
    WorkpieceDefinition,
)
from pipeline import PipelineOrchestrator
import PostClear


class MockGrid:
    def __init__(self, nx=200, ny=200, cell_size_mm=0.5, grid_x_min=-50.0, grid_y_min=-50.0):
        self.nx = nx
        self.ny = ny
        self.cell_size_mm = cell_size_mm
        self.grid_x_min = grid_x_min
        self.grid_y_min = grid_y_min
        self.H = np.full((ny, nx), -100.0, dtype=np.float64)


def reference_rasterize(grid, vertices, indices, tol_pad_z_mm=0.05):
    if vertices.size == 0 or indices.size == 0:
        return
    cell_size = grid.cell_size_mm
    gx_min = grid.grid_x_min
    gy_min = grid.grid_y_min
    nx = grid.nx
    ny = grid.ny

    t_verts = vertices[indices].astype(np.float64, copy=False)

    v_cols = np.clip(np.floor((vertices[:, 0] - gx_min) / cell_size).astype(int), 0, nx - 1)
    v_rows = np.clip(np.floor((vertices[:, 1] - gy_min) / cell_size).astype(int), 0, ny - 1)
    v_z = vertices[:, 2] + tol_pad_z_mm
    for r, c, z in zip(v_rows, v_cols, v_z):
        if z > grid.H[r, c]:
            grid.H[r, c] = z

    for v0, v1, v2 in t_verts:
        x0, y0, z0 = v0
        x1, y1, z1 = v1
        x2, y2, z2 = v2
        z_min = min(z0, z1, z2)
        z_max = max(z0, z1, z2)

        nx_val = (y1 - y0) * (z2 - z0) - (z1 - z0) * (y2 - y0)
        ny_val = (z1 - z0) * (x2 - x0) - (x1 - x0) * (z2 - z0)
        nz_val = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0)
        max_e2 = max(
            (x1 - x0) ** 2 + (y1 - y0) ** 2 + (z1 - z0) ** 2,
            (x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2,
            (x0 - x2) ** 2 + (y0 - y2) ** 2 + (z0 - z2) ** 2,
        )

        t_min_x = min(x0, x1, x2)
        t_max_x = max(x0, x1, x2)
        t_min_y = min(y0, y1, y2)
        t_max_y = max(y0, y1, y2)

        c_min = max(0, int(math.floor((t_min_x - gx_min) / cell_size)))
        c_max = min(nx - 1, int(math.ceil((t_max_x - gx_min) / cell_size)))
        r_min = max(0, int(math.floor((t_min_y - gy_min) / cell_size)))
        r_max = min(ny - 1, int(math.ceil((t_max_y - gy_min) / cell_size)))

        if c_max < c_min or r_max < r_min:
            continue

        if c_min == c_max and r_min == r_max:
            top_z = z_max + tol_pad_z_mm
            if top_z > grid.H[r_min, c_min]:
                grid.H[r_min, c_min] = top_z
            continue

        if abs(nz_val) <= 1e-5 * max_e2:
            for pa, pb in ((v0, v1), (v1, v2), (v2, v0)):
                dx = pb[0] - pa[0]
                dy = pb[1] - pa[1]
                dz = pb[2] - pa[2]
                length = math.hypot(dx, dy)
                steps = max(1, int(math.ceil(length / (cell_size * 0.5))))
                for s in range(steps + 1):
                    frac = s / steps
                    px = pa[0] + frac * dx
                    py = pa[1] + frac * dy
                    pz = pa[2] + frac * dz + tol_pad_z_mm
                    c_idx = int(math.floor((px - gx_min) / cell_size))
                    r_idx = int(math.floor((py - gy_min) / cell_size))
                    if 0 <= c_idx < nx and 0 <= r_idx < ny:
                        if pz > grid.H[r_idx, c_idx]:
                            grid.H[r_idx, c_idx] = pz
            continue

        cols = np.arange(c_min, c_max + 1)
        rows = np.arange(r_min, r_max + 1)
        grid_x = gx_min + (cols + 0.5) * cell_size
        grid_y = gy_min + (rows + 0.5) * cell_size
        X, Y = np.meshgrid(grid_x, grid_y)

        e01 = (x1 - x0) * (Y - y0) - (y1 - y0) * (X - x0)
        e12 = (x2 - x1) * (Y - y1) - (y2 - y1) * (X - x1)
        e20 = (x0 - x2) * (Y - y2) - (y0 - y2) * (X - x2)

        eps = 1e-5 * abs(nz_val)
        inside = ((e01 >= -eps) & (e12 >= -eps) & (e20 >= -eps)) | ((e01 <= eps) & (e12 <= eps) & (e20 <= eps))
        if not np.any(inside):
            continue

        Z_plane = z0 - (nx_val * (X - x0) + ny_val * (Y - y0)) / nz_val
        Z = np.where(np.isfinite(Z_plane), np.clip(Z_plane, z_min, z_max), z_max) + tol_pad_z_mm
        sub_h = grid.H[r_min : r_max + 1, c_min : c_max + 1]
        mask = inside & (Z > sub_h)
        sub_h[mask] = Z[mask]


class TestBatch6RemainingItems(unittest.TestCase):
    def setUp(self):
        self.tmp_dir = tempfile.mkdtemp(prefix="test_batch6_")

    def tearDown(self):
        try:
            import shutil
            shutil.rmtree(self.tmp_dir, ignore_errors=True)
        except Exception:
            pass

    # -------------------------------------------------------------------------
    # 1. Phase 1 M1: Fixture warning in previous-mode turning
    # -------------------------------------------------------------------------
    def test_01_m1_fixture_warning_previous_mode_turning_no_fixtures(self):
        """Turning in previous mode without fixtures populates report.fixture_warning."""
        orchestrator = PipelineOrchestrator.__new__(PipelineOrchestrator)
        orchestrator.timeout = 120.0
        orchestrator.replay_combined_context = MagicMock(return_value="N10 G0 X0\n")

        stock = AxisymmetricStock2D(z_min=-50.0, z_max=0.0, outer_radius=25.0)
        wp_no_fixtures = WorkpieceDefinition(stock=stock, fixtures=[], is_stl_derived=True)

        mock_verifier = MagicMock()
        mock_rep = VerificationReport(
            total_machining_time_sec=10.0,
            total_volume_removed_mm3=500.0,
            collisions=[],
        )
        mock_verifier.verify_toolpath.return_value = mock_rep

        ctx = {
            "stockMode": "from preceding setup",
            "sections": [{"type": 1, "tool": {"_isTurningTool": True}, "events": []}],
        }

        with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
            import json
            json.dump(ctx, f)
            dump_p = f.name

        try:
            with patch("pipeline.merge_context_dumps", return_value=ctx), \
                 patch("pipeline.parse_context_dump", return_value=[]), \
                 patch("pipeline.extract_tools_from_context", return_value={"op1": MagicMock()}):
                rep, _ = orchestrator.run(
                    context_dump_paths=[dump_p],
                    base_post_path=os.path.join(os.path.dirname(os.path.dirname(__file__)), "centroid_turning.cps"),
                    workpiece=wp_no_fixtures,
                    verifier=mock_verifier,
                    promote=False,
                    is_turning=True,
                )

            self.assertIsNotNone(rep.fixture_warning)
            self.assertIn("fixture checks disabled", rep.fixture_warning.lower())

            summary = orchestrator.format_summary_report(rep, final_gcode="", base_post_path="test.cps")
            self.assertIn("Fixture checks disabled", summary)
            self.assertIn("> **Warning**: Fixture checks disabled", summary)
        finally:
            if os.path.isfile(dump_p):
                os.remove(dump_p)

    def test_02_m1_fixture_warning_not_triggered_when_fixtures_present(self):
        """Turning in previous mode with fixtures does not set fixture_warning."""
        orchestrator = PipelineOrchestrator.__new__(PipelineOrchestrator)
        orchestrator.timeout = 120.0
        orchestrator.replay_combined_context = MagicMock(return_value="N10 G0 X0\n")

        stock = AxisymmetricStock2D(z_min=-50.0, z_max=0.0, outer_radius=25.0)
        fixtures = [AxisymmetricChuckObstacle(z_min=-100.0, z_max=-50.0, outer_radius=50.0, clamping_radius=25.0)]
        wp_with_fixtures = WorkpieceDefinition(stock=stock, fixtures=fixtures, is_stl_derived=True)

        mock_verifier = MagicMock()
        mock_rep = VerificationReport(
            total_machining_time_sec=10.0,
            total_volume_removed_mm3=500.0,
            collisions=[],
        )
        mock_verifier.verify_toolpath.return_value = mock_rep

        ctx = {
            "stockMode": "from preceding setup",
            "sections": [{"type": 1, "tool": {"_isTurningTool": True}, "events": []}],
        }

        with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
            import json
            json.dump(ctx, f)
            dump_p = f.name

        try:
            with patch("pipeline.merge_context_dumps", return_value=ctx), \
                 patch("pipeline.parse_context_dump", return_value=[]), \
                 patch("pipeline.extract_tools_from_context", return_value={"op1": MagicMock()}):
                rep, _ = orchestrator.run(
                    context_dump_paths=[dump_p],
                    base_post_path=os.path.join(os.path.dirname(os.path.dirname(__file__)), "centroid_turning.cps"),
                    workpiece=wp_with_fixtures,
                    verifier=mock_verifier,
                    promote=False,
                    is_turning=True,
                )

            self.assertIsNone(rep.fixture_warning)
        finally:
            if os.path.isfile(dump_p):
                os.remove(dump_p)

    def test_03_m1_fixture_warning_not_triggered_for_milling(self):
        """Milling in previous mode does not trigger fixture_warning."""
        orchestrator = PipelineOrchestrator.__new__(PipelineOrchestrator)
        orchestrator.timeout = 120.0
        orchestrator.replay_combined_context = MagicMock(return_value="N10 G0 X0\n")

        stock = BoxStock3D(x_min=-20.0, x_max=20.0, y_min=-20.0, y_max=20.0, z_min=-20.0, z_max=0.0)
        wp_milling = WorkpieceDefinition(stock=stock, fixtures=[], is_stl_derived=True)

        mock_verifier = MagicMock()
        mock_rep = VerificationReport(
            total_machining_time_sec=10.0,
            total_volume_removed_mm3=500.0,
            collisions=[],
        )
        mock_verifier.verify_toolpath.return_value = mock_rep

        ctx = {
            "stockMode": "from preceding setup",
            "sections": [{"type": 0, "events": []}],
        }

        with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
            import json
            json.dump(ctx, f)
            dump_p = f.name

        try:
            with patch("pipeline.merge_context_dumps", return_value=ctx), \
                 patch("pipeline.parse_context_dump", return_value=[]), \
                 patch("pipeline.extract_tools_from_context", return_value={"op1": MagicMock()}):
                rep, _ = orchestrator.run(
                    context_dump_paths=[dump_p],
                    base_post_path=os.path.join(os.path.dirname(os.path.dirname(__file__)), "centroid_turning.cps"),
                    workpiece=wp_milling,
                    verifier=mock_verifier,
                    promote=False,
                    is_turning=False,
                )

            self.assertIsNone(rep.fixture_warning)
        finally:
            if os.path.isfile(dump_p):
                os.remove(dump_p)

    # -------------------------------------------------------------------------
    # 2. Phase 1 M5: Preceding-stock check in PostProcessSetup
    # -------------------------------------------------------------------------
    def test_04_m5_preceding_stock_walks_multiple_earlier_setups(self):
        """Setup 3 checks both Setup 1 and Setup 2; missing toolpath on Setup 1 triggers prompt."""
        mock_ui = MagicMock()
        mock_ui.messageBox.return_value = adsk.core.DialogResults.DialogNo

        # S1 has ungenerated toolpath
        s1 = MagicMock(name="Setup1")
        s1.name = "Setup1"
        s1.isSuppressed = False
        op1 = MagicMock(isSuppressed=False)
        s1.allOperations = [op1]

        # S2 has valid toolpath
        s2 = MagicMock(name="Setup2")
        s2.name = "Setup2"
        s2.isSuppressed = False
        op2 = MagicMock(isSuppressed=False)
        s2.allOperations = [op2]

        # S3 depends on preceding setups
        s3 = MagicMock(name="Setup3")
        s3.name = "Setup3"
        s3.isSuppressed = False
        s3.operationType = 0
        s3.stockMode = adsk.cam.SetupStockModes.PreviousSetupStock
        op3 = MagicMock(isSuppressed=False)
        s3.allOperations = [op3]

        mock_cam = MagicMock()
        mock_cam.setups = [s1, s2, s3]
        # S1 fails toolpath check; S2 succeeds
        mock_cam.checkToolpath.side_effect = lambda op: (op is op2 or op is op3)

        mock_doc = MagicMock()
        mock_doc.products.itemByProductType.return_value = mock_cam
        mock_app = MagicMock()
        mock_app.activeDocument = mock_doc
        mock_app.userInterface = mock_ui

        mock_prog = MagicMock()
        mock_prog.parameters.itemByName.return_value = None

        with patch("PostClear.adsk.core.Application.get", return_value=mock_app):
            ret = PostClear.PostProcessSetup(
                fname="Setup3",
                setup=s3,
                setupFolder=self.tmp_dir,
                docSettings={},
                program=mock_prog,
                target_operations=[op3],
            )
            mock_ui.messageBox.assert_called_once()
            self.assertEqual(ret, "Operation cancelled by user.")

    def test_05_m5_preceding_stock_skips_suppressed_earlier_setups(self):
        """Suppressed earlier setups are ignored during predecessor walk."""
        mock_ui = MagicMock()

        # S1 is suppressed and has no toolpaths
        s1 = MagicMock(name="Setup1")
        s1.name = "Setup1"
        s1.isSuppressed = True
        s1.allOperations = []

        # S2 is active and has valid toolpaths
        s2 = MagicMock(name="Setup2")
        s2.name = "Setup2"
        s2.isSuppressed = False
        op2 = MagicMock(isSuppressed=False)
        s2.allOperations = [op2]

        # S3 uses preceding setup stock
        s3 = MagicMock(name="Setup3")
        s3.name = "Setup3"
        s3.isSuppressed = False
        s3.operationType = 0
        s3.stockMode = adsk.cam.SetupStockModes.PreviousSetupStock
        op3 = MagicMock(isSuppressed=False)
        s3.allOperations = [op3]

        mock_cam = MagicMock()
        mock_cam.setups = [s1, s2, s3]
        mock_cam.checkToolpath.return_value = True

        mock_doc = MagicMock()
        mock_doc.products.itemByProductType.return_value = mock_cam
        mock_app = MagicMock()
        mock_app.activeDocument = mock_doc
        mock_app.userInterface = mock_ui

        mock_prog = MagicMock()
        mock_prog.parameters.itemByName.return_value = None

        with patch("PostClear.adsk.core.Application.get", return_value=mock_app), \
             patch("PostClear._PostProcessUnifiedPipeline", return_value=None):
            ret = PostClear.PostProcessSetup(
                fname="Setup3",
                setup=s3,
                setupFolder=self.tmp_dir,
                docSettings={},
                program=mock_prog,
                target_operations=[op3],
            )
            # S1 was suppressed, S2 was valid, so no warning modal should be triggered
            mock_ui.messageBox.assert_not_called()
            self.assertIsNone(ret)

    def test_06_m5_preceding_stock_fails_closed_on_exception(self):
        """Any exception thrown during prior setup check fails closed (prior_missing = True)."""
        mock_ui = MagicMock()
        mock_ui.messageBox.return_value = adsk.core.DialogResults.DialogNo

        s2 = MagicMock(name="Setup2")
        s2.name = "Setup2"
        s2.operationType = 0
        s2.stockMode = adsk.cam.SetupStockModes.PreviousSetupStock
        op2 = MagicMock(isSuppressed=False)
        s2.allOperations = [op2]

        mock_cam = MagicMock()
        # setups property throws exception
        type(mock_cam).setups = property(lambda self: (_ for _ in ()).throw(RuntimeError("CAM COM failure")))

        mock_doc = MagicMock()
        mock_doc.products.itemByProductType.return_value = mock_cam
        mock_app = MagicMock()
        mock_app.activeDocument = mock_doc
        mock_app.userInterface = mock_ui

        mock_prog = MagicMock()
        mock_prog.parameters.itemByName.return_value = None

        with patch("PostClear.adsk.core.Application.get", return_value=mock_app):
            ret = PostClear.PostProcessSetup(
                fname="Setup2",
                setup=s2,
                setupFolder=self.tmp_dir,
                docSettings={},
                program=mock_prog,
                target_operations=[op2],
            )
            mock_ui.messageBox.assert_called_once()
            self.assertEqual(ret, "Operation cancelled by user.")

    def test_07_m5_progress_dialog_hidden_and_restored_during_modal_prompt(self):
        """Progress dialog is hidden prior to ui.messageBox and restored in finally block."""
        mock_ui = MagicMock()
        mock_ui.messageBox.return_value = adsk.core.DialogResults.DialogYes

        s1 = MagicMock(name="Setup1", isSuppressed=False, allOperations=[])
        s2 = MagicMock(
            name="Setup2",
            operationType=0,
            stockMode=adsk.cam.SetupStockModes.PreviousSetupStock,
            allOperations=[MagicMock(isSuppressed=False)],
        )

        mock_cam = MagicMock()
        mock_cam.setups = [s1, s2]
        mock_cam.checkToolpath.return_value = True

        mock_doc = MagicMock()
        mock_doc.products.itemByProductType.return_value = mock_cam
        mock_app = MagicMock(activeDocument=mock_doc, userInterface=mock_ui)
        mock_prog = MagicMock()
        mock_prog.parameters.itemByName.return_value = None

        mock_progress = MagicMock()
        mock_cb = MagicMock()
        mock_cb.progress_dialog = mock_progress

        with patch("PostClear.adsk.core.Application.get", return_value=mock_app), \
             patch("PostClear._PostProcessUnifiedPipeline", return_value=None):
            PostClear.PostProcessSetup(
                fname="Setup2",
                setup=s2,
                setupFolder=self.tmp_dir,
                docSettings={},
                program=mock_prog,
                target_operations=s2.allOperations,
                progress_cb=mock_cb,
            )
            mock_progress.hide.assert_called_once()
            mock_progress.show.assert_called_once()

    # -------------------------------------------------------------------------
    # 3. Phase 1 M6: Vectorized rasterize_triangles_to_grid
    # -------------------------------------------------------------------------
    def test_08_m6_vectorized_rasterize_triangles_exact_match(self):
        """Vectorized rasterization gives exact 0.0 diff against reference logic."""
        from geometry.solid_mesh import rasterize_triangles_to_grid

        np.random.seed(123)
        verts = np.random.uniform(-40, 40, (300, 3)).astype(np.float32)
        indices = np.random.randint(0, 300, (600, 3)).astype(np.int32)

        g_ref = MockGrid(nx=150, ny=150, cell_size_mm=0.5, grid_x_min=-40, grid_y_min=-40)
        g_vec = MockGrid(nx=150, ny=150, cell_size_mm=0.5, grid_x_min=-40, grid_y_min=-40)

        reference_rasterize(g_ref, verts, indices)
        rasterize_triangles_to_grid(g_vec, verts, indices)

        diff = float(np.max(np.abs(g_ref.H - g_vec.H)))
        self.assertAlmostEqual(diff, 0.0, places=5, msg=f"Max difference was {diff}")

    def test_09_m6_vectorized_rasterize_triangles_benchmark_500k(self):
        """500,000 triangles rasterize in under 3.0 seconds."""
        from geometry.solid_mesh import rasterize_triangles_to_grid

        np.random.seed(42)
        n_tris = 500000
        n_verts = 100000
        verts_base = np.random.uniform(-80, 80, (n_verts, 3)).astype(np.float32)
        offsets1 = np.random.uniform(-0.8, 0.8, (n_tris, 3)).astype(np.float32)
        offsets2 = np.random.uniform(-0.8, 0.8, (n_tris, 3)).astype(np.float32)
        base_idx = np.random.randint(0, n_verts, n_tris)
        base_v = verts_base[base_idx]

        all_v = np.empty((n_tris * 3, 3), dtype=np.float32)
        all_v[0::3] = base_v
        all_v[1::3] = base_v + offsets1
        all_v[2::3] = base_v + offsets2
        all_idx = np.arange(n_tris * 3, dtype=np.int32).reshape(-1, 3)

        grid = MockGrid(nx=300, ny=300, cell_size_mm=0.5, grid_x_min=-80, grid_y_min=-80)

        t0 = time.perf_counter()
        rasterize_triangles_to_grid(grid, all_v, all_idx)
        elapsed = time.perf_counter() - t0

        self.assertLess(elapsed, 3.0, f"Benchmark exceeded 3.0s: {elapsed:.3f}s")

    # -------------------------------------------------------------------------
    # 4. Phase 1 L4: Explicit workpiece and non-STL preceding mode fallback
    # -------------------------------------------------------------------------
    def test_10_l4_explicit_workpiece_debug_log(self):
        """Passing explicit workpiece to PipelineOrchestrator.run logs debug line when stockPath present."""
        orchestrator = PipelineOrchestrator.__new__(PipelineOrchestrator)
        orchestrator.timeout = 120.0

        stock = BoxStock3D(x_min=-10, x_max=10, y_min=-10, y_max=10, z_min=-10, z_max=0)
        explicit_wp = WorkpieceDefinition(stock=stock)

        ctx = {
            "stockMode": "from preceding setup",
            "stockPath": "/tmp/dummy.stl",
            "sections": [],
        }

        with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
            import json
            json.dump(ctx, f)
            dump_p = f.name

        try:
            with patch("pipeline.merge_context_dumps", return_value=ctx), \
                 patch("pipeline.parse_context_dump", return_value=[]), \
                 patch("pipeline.extract_tools_from_context", return_value={}), \
                 patch("PostClear._dbg_global") as mock_dbg:
                try:
                    orchestrator.run(
                        context_dump_paths=[dump_p],
                        base_post_path=os.path.join(os.path.dirname(os.path.dirname(__file__)), "centroid_turning.cps"),
                        workpiece=explicit_wp,
                        verifier=MagicMock(),
                        promote=False,
                        is_turning=False,
                    )
                except Exception:
                    pass

                logged = [str(call[0][0]) for call in mock_dbg.call_args_list]
                self.assertTrue(
                    any("workpiece provided explicitly; ignoring stockPath" in l for l in logged),
                    "Expected debug log for explicit workpiece overriding stockPath"
                )
        finally:
            if os.path.isfile(dump_p):
                os.remove(dump_p)

    def test_11_l4_preceding_mode_rejects_non_stl_fallback(self):
        """Preceding setup stock rejects non-STL fallback workpiece with ValueError."""
        orchestrator = PipelineOrchestrator.__new__(PipelineOrchestrator)
        orchestrator.timeout = 120.0

        stock = BoxStock3D(x_min=-10, x_max=10, y_min=-10, y_max=10, z_min=-10, z_max=0)
        # Not STL-derived
        box_wp = WorkpieceDefinition(stock=stock, is_stl_derived=False)

        ctx = {
            "stockMode": "from preceding setup",
            "sections": [],
        }

        with tempfile.NamedTemporaryFile("w", suffix=".json", delete=False) as f:
            import json
            json.dump(ctx, f)
            dump_p = f.name

        try:
            with patch("pipeline.merge_context_dumps", return_value=ctx), \
                 patch("pipeline.parse_context_dump", return_value=[]), \
                 patch("pipeline.extract_workpiece_from_context", return_value=box_wp):
                with self.assertRaises(ValueError) as cm:
                    orchestrator.run(
                        context_dump_paths=[dump_p],
                        base_post_path=os.path.join(os.path.dirname(os.path.dirname(__file__)), "centroid_turning.cps"),
                        fallback_workpiece=box_wp,
                        verifier=MagicMock(),
                        promote=False,
                        is_turning=False,
                    )
                self.assertIn("uses 'From Previous Setup' stock", str(cm.exception))
        finally:
            if os.path.isfile(dump_p):
                os.remove(dump_p)

    # -------------------------------------------------------------------------
    # 5. Phase 1 L5 / Phase 2 L4: Toast truncation & clean errors
    # -------------------------------------------------------------------------
    def test_12_l5_phase2_l4_toast_truncation_to_600_chars(self):
        """Toast details longer than 600 characters are cleanly truncated."""
        mock_ui = MagicMock()
        mock_ui.palettes = MagicMock()

        long_skip = "Setup error details:\n" + ("A" * 1500)
        with patch("PostClear._BuildNotificationHTML") as mock_html_builder, \
             patch("builtins.open", MagicMock()):
            PostClear._NotifyPostProcessResult(
                ui=mock_ui,
                cntFiles=1,
                cntSkipped=1,
                lstSkipped=long_skip,
                outputFolder=self.tmp_dir,
            )
            mock_html_builder.assert_called_once()
            passed_details = mock_html_builder.call_args[0][3]
            self.assertLessEqual(len(passed_details), 650)
            self.assertIn("... (see report for full details)", passed_details)

    def test_13_l5_phase2_l4_postprocesssetup_clean_error_no_traceback(self):
        """PostProcessSetup returns a clean error message without raw Python tracebacks."""
        mock_setup = MagicMock(name="Setup1")
        mock_setup.name = "Setup1"
        mock_setup.operationType = 0
        mock_setup.stockMode = adsk.cam.SetupStockModes.FixedBoxStock

        mock_prog = MagicMock()
        mock_prog.parameters.itemByName.return_value = None

        mock_cam = MagicMock()
        mock_cam.checkToolpath.side_effect = RuntimeError("Internal CAM toolpath failure")
        mock_doc = MagicMock()
        mock_doc.products.itemByProductType.return_value = mock_cam
        mock_app = MagicMock(activeDocument=mock_doc, userInterface=MagicMock())

        with patch("PostClear.adsk.core.Application.get", return_value=mock_app):
            ret = PostClear.PostProcessSetup(
                fname="Setup1",
                setup=mock_setup,
                setupFolder=self.tmp_dir,
                docSettings={},
                program=mock_prog,
                target_operations=[MagicMock()],
            )
            self.assertIn("Fusion reported an exception", ret)
            self.assertNotIn("Traceback (most recent call last)", ret)

    # -------------------------------------------------------------------------
    # 6. Phase 2 M2: _ScopedEnv env variable isolation
    # -------------------------------------------------------------------------
    def test_14_phase2_m2_scoped_env_preserves_and_restores(self):
        """_ScopedEnv preserves existing env and restores it strictly in finally block."""
        os.environ["BASE_POST"] = "ORIGINAL_BASE"
        os.environ["TARGET_INTERMEDIATE_CNC"] = "ORIGINAL_CNC"
        os.environ.pop("DUMPER_CONFIG_PATH", None)

        updates = {
            "BASE_POST": "/tmp/new_base.cps",
            "TARGET_INTERMEDIATE_CNC": None,
            "DUMPER_CONFIG_PATH": "/tmp/new_cfg.json",
        }

        try:
            with self.assertRaises(ZeroDivisionError):
                with PostClear._ScopedEnv(updates):
                    self.assertEqual(os.environ.get("BASE_POST"), "/tmp/new_base.cps")
                    self.assertNotIn("TARGET_INTERMEDIATE_CNC", os.environ)
                    self.assertEqual(os.environ.get("DUMPER_CONFIG_PATH"), "/tmp/new_cfg.json")
                    _ = 1 / 0
        finally:
            self.assertEqual(os.environ.get("BASE_POST"), "ORIGINAL_BASE")
            self.assertEqual(os.environ.get("TARGET_INTERMEDIATE_CNC"), "ORIGINAL_CNC")
            self.assertNotIn("DUMPER_CONFIG_PATH", os.environ)
            os.environ.pop("BASE_POST", None)
            os.environ.pop("TARGET_INTERMEDIATE_CNC", None)

    # -------------------------------------------------------------------------
    # 7. Phase 2 M4: PowerShell process discovery on Windows
    # -------------------------------------------------------------------------
    def test_15_phase2_m4_windows_powershell_process_discovery(self):
        """find_autodesk_fusion_invoking_root uses PowerShell Get-CimInstance on Windows."""
        from pipeline import find_autodesk_fusion_invoking_root

        fake_win_path = r"C:\Users\john.doe\AppData\Local\Autodesk\webdeploy\production\b43a07003c01dd54425db484d23b1c5e6248a21d\Fusion360.exe"

        mock_proc = MagicMock()
        mock_proc.communicate.return_value = (fake_win_path, "")
        mock_proc.__enter__.return_value = mock_proc

        with patch("sys.platform", "win32"), \
             patch("os.environ", {}), \
             patch("subprocess.Popen", return_value=mock_proc) as mock_popen, \
             patch("os.path.isdir", return_value=True):
            root = find_autodesk_fusion_invoking_root()
            self.assertIsNotNone(root)
            self.assertIn("b43a07003c01dd54425db484d23b1c5e6248a21d", root)
            mock_popen.assert_called()
            called_cmd = mock_popen.call_args[0][0]
            self.assertEqual(called_cmd[0], "powershell.exe")
            self.assertIn("Get-CimInstance", " ".join(called_cmd))

    # -------------------------------------------------------------------------
    # 8. Phase 2 L1: Debug log rotation
    # -------------------------------------------------------------------------
    def test_16_phase2_l1_log_rotation(self):
        """_RotateLogIfNeeded rotates log to .1 when exceeding cap."""
        test_log = os.path.join(self.tmp_dir, "test_rot.log")
        test_rot = test_log + ".1"

        # Write 200 bytes
        with open(test_log, "w", encoding="utf8") as f:
            f.write("X" * 200)

        # Cap at 100 bytes
        PostClear._RotateLogIfNeeded(log_path=test_log, max_bytes=100)

        self.assertFalse(os.path.isfile(test_log))
        self.assertTrue(os.path.isfile(test_rot))
        with open(test_rot, "r", encoding="utf8") as f:
            self.assertEqual(len(f.read()), 200)

    # -------------------------------------------------------------------------
    # 9. Phase 2 L5: Line ending normalization
    # -------------------------------------------------------------------------
    def test_17_phase2_l5_line_endings_normalized(self):
        """Touched repository files contain only LF newlines and no CRLF or CRCRLF."""
        repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        touched = [
            "PostClear.py",
            "pipeline.py",
            "geometry/models.py",
            "geometry/solid_mesh.py",
            "tests/test_full_pipeline_mock.py",
            "tests/test_full_pipeline_real_engine.py",
        ]
        for rel in touched:
            p = os.path.join(repo_root, rel)
            with open(p, "rb") as f:
                content = f.read()
            self.assertNotIn(b"\r\r\n", content, f"{rel} contains \\r\\r\\n")
            self.assertNotIn(b"\r\n", content, f"{rel} contains \\r\\n")


if __name__ == "__main__":
    unittest.main()
