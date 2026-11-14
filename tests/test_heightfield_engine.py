#!/usr/bin/env python3
"""
Unit and Integration Test Suite for 3D In-Process Workpiece (IPW) Heightfield Grid Engine.

Validates:
1. HeightfieldGrid discrete coordinate mapping and initial bounds allocation.
2. Mathematical tool bottom profile generation (flat, ball, bull-nose, conical).
3. 3D linear cutting moves and analytical volumetric MRR accuracy.
4. Multi-pass depth stepping, stock carving, and air move detection.
5. Rapid collision detection into uncut stock (RAPID_INTO_STOCK).
6. Toolholder collision detection into raw stock (HOLDER_STRIKE).
7. Unfluted tool shank rubbing detection (SHANK_RUB).
8. Occurrence-gated rapid (G1 -> G0) promotion rules.
9. Circular arc sweeping (G2/G3) in the XY plane.
10. Full toolpath verification workflow via verify_toolpath().
"""

import math
import unittest
import numpy as np

from geometry.engine_interface import BaseVerificationEngine, IVerificationEngine
from geometry.heightfield_engine import (
    HeightfieldGrid,
    MillHeightfieldVerifier,
    evaluate_tool_bottom_profile,
)
from geometry.models import (
    BoxStock3D,
    CollisionKind,
    MillToolGeometry,
    MotionKind,
    MotionSegment,
    ToolAssembly,
    Vector3D,
    VectorKey,
    WorkpieceDefinition,
)


def _make_segment(
    seg_id: int,
    motion: MotionKind,
    start: Vector3D,
    end: Vector3D,
    feedrate: float = 1000.0,
    duration_sec: float = 1.0,
    sec_id: int = 0,
    op_id: str = "Op1",
    is_arc: bool = False,
    arc_center: Vector3D = None,
    metadata: dict = None,
) -> MotionSegment:
    vkey = VectorKey(
        sec_id=sec_id,
        start_x=round(start.x, 3),
        start_y=round(start.y, 3),
        start_z=round(start.z, 3),
        end_x=round(end.x, 3),
        end_y=round(end.y, 3),
        end_z=round(end.z, 3),
    )
    meta = dict(metadata or {})
    meta.setdefault("sec_idx", sec_id)
    meta.setdefault("event_idx", seg_id)
    return MotionSegment(
        segment_id=seg_id,
        op_id=op_id,
        sec_id=sec_id,
        motion=motion,
        start=start,
        end=end,
        feedrate_mmpm=feedrate,
        spindle_rpm=8000.0,
        duration_sec=duration_sec,
        vector_key=vkey,
        is_arc=is_arc,
        arc_center=arc_center,
        metadata=meta,
    )


class TestHeightfieldGrid(unittest.TestCase):
    """Test low-level discrete regular grid geometry."""

    def test_grid_initialization_and_reset(self):
        stock = BoxStock3D(x_min=-20.0, x_max=20.0, y_min=-30.0, y_max=30.0, z_min=-15.0, z_max=0.0)
        grid = HeightfieldGrid(cell_size_mm=0.5, pad_mm=5.0)
        grid.reset(stock)

        # Expected dimensions with 5mm padding:
        # X: [-25, 25] -> width 50 -> ~101 cells
        # Y: [-35, 35] -> height 70 -> ~141 cells
        self.assertGreater(grid.nx, 95)
        self.assertGreater(grid.ny, 135)

        # Center inside stock should be z_max (0.0)
        col_c = int((0.0 - grid.grid_x_min) / grid.cell_size_mm)
        row_c = int((0.0 - grid.grid_y_min) / grid.cell_size_mm)
        self.assertAlmostEqual(float(grid.H[row_c, col_c]), 0.0, places=3)

        # Corner outside stock should be z_floor (-15.0)
        self.assertAlmostEqual(float(grid.H[0, 0]), -15.0, places=3)

    def test_tool_bottom_profile_formulas(self):
        # 1. Flat endmill (D=10, rc=0)
        r = np.array([0.0, 2.5, 5.0], dtype=np.float32)
        flat_z = evaluate_tool_bottom_profile(r, tool_radius=5.0, corner_radius=0.0)
        np.testing.assert_allclose(flat_z, [0.0, 0.0, 0.0])

        # 2. Ball endmill (D=10, rc=5.0)
        ball_z = evaluate_tool_bottom_profile(r, tool_radius=5.0, corner_radius=5.0)
        # At r=0: z=0; at r=5.0: z=5.0; at r=3.0: 5 - sqrt(25-9) = 1.0
        r_test = np.array([0.0, 3.0, 5.0], dtype=np.float32)
        ball_test = evaluate_tool_bottom_profile(r_test, tool_radius=5.0, corner_radius=5.0)
        np.testing.assert_allclose(ball_test, [0.0, 1.0, 5.0], atol=1e-5)

        # 3. Bull-nose endmill (D=10, R=5, rc=1.0 -> flat center r_flat=4.0)
        r_bull = np.array([0.0, 2.0, 4.0, 5.0], dtype=np.float32)
        bull_z = evaluate_tool_bottom_profile(r_bull, tool_radius=5.0, corner_radius=1.0)
        self.assertAlmostEqual(bull_z[0], 0.0)
        self.assertAlmostEqual(bull_z[2], 0.0)  # at r=4.0 (end of flat)
        self.assertAlmostEqual(bull_z[3], 1.0)  # at r=5.0 (outer corner)

        # 4. 90-degree Chamfer tool / Spot drill (D=6.35, R=3.175, included angle 90 deg -> half angle 45 deg)
        # z(r) = r / tan(45) = r
        r_chamfer = np.array([0.0, 1.0, 2.0, 3.175], dtype=np.float32)
        chamfer_z = evaluate_tool_bottom_profile(r_chamfer, tool_radius=3.175, corner_radius=0.0, taper_angle_deg=90.0)
        np.testing.assert_allclose(chamfer_z, [0.0, 1.0, 2.0, 3.175], atol=1e-5)

        # 5. 118-degree Drill (D=10, included angle 118 deg -> half angle 59 deg)
        # z(r) = r / tan(59 deg) = 0.60086 * r
        r_drill = np.array([0.0, 5.0], dtype=np.float32)
        drill_z = evaluate_tool_bottom_profile(r_drill, tool_radius=5.0, corner_radius=0.0, taper_angle_deg=118.0)
        np.testing.assert_allclose(drill_z, [0.0, 5.0 / math.tan(math.radians(59.0))], atol=1e-5)


class TestMillHeightfieldVerifier(unittest.TestCase):
    """Test complete 3D milling verifier against kinematic toolpaths."""

    def setUp(self):
        # Stock: 60 x 60 x 20 mm, top at Z=0.0
        self.stock = BoxStock3D(x_min=-30.0, x_max=30.0, y_min=-30.0, y_max=30.0, z_min=-20.0, z_max=0.0)
        self.workpiece = WorkpieceDefinition(stock=self.stock)

        # Flat Endmill: D=10mm (R=5mm), flute=20mm, overall=40mm, holder_D=30mm, holder_L=40mm
        self.tool_geom = MillToolGeometry(
            diameter=10.0,
            corner_radius=0.0,
            flute_length=20.0,
            overall_length=40.0,
            holder_diameter=30.0,
            holder_length=40.0,
        )
        self.tool = ToolAssembly(tool_id=1, name="10mm Flat Endmill", geometry=self.tool_geom)

        self.verifier = MillHeightfieldVerifier(cell_size_mm=0.5, pad_mm=5.0, promote_margin_mm=0.0)
        self.verifier.reset(self.workpiece)
        self.verifier.load_tool(self.tool)

    def test_single_linear_slot_cut_volume_accuracy(self):
        """A linear slot cut from X=-10 to X=10 at Z=-2.0 should remove analytical volume."""
        # Slot: length = 20mm + tool_D (with end caps) = 30mm.
        # Rectangular middle = 20 * 10 * 2 = 400 mm3.
        # Two semi-circular end caps = pi * R^2 * depth = pi * 25 * 2 = ~157.08 mm3.
        # Total analytical volume ~ 557.08 mm3.
        move = _make_segment(
            seg_id=1,
            motion=MotionKind.LINEAR,
            start=Vector3D(-10.0, 0.0, -2.0),
            end=Vector3D(10.0, 0.0, -2.0),
            feedrate=600.0,
            duration_sec=2.0,
        )
        eval_res = self.verifier.step(move)

        self.assertFalse(eval_res.is_air_move)
        self.assertEqual(len(eval_res.collisions), 0)

        analytical_vol = 20.0 * 10.0 * 2.0 + math.pi * (5.0 ** 2) * 2.0
        # Grid discrete approximation is within 5% of continuous analytical volume
        self.assertAlmostEqual(eval_res.material_removed_mm3, analytical_vol, delta=analytical_vol * 0.05)
        self.assertGreater(eval_res.instantaneous_mrr, 0.0)

        # Re-stepping identical move must remove ZERO material and be detected as an air move
        move_pass2 = _make_segment(
            seg_id=2,
            motion=MotionKind.LINEAR,
            start=Vector3D(-10.0, 0.0, -2.0),
            end=Vector3D(10.0, 0.0, -2.0),
            feedrate=600.0,
            duration_sec=2.0,
        )
        eval2 = self.verifier.step(move_pass2)
        self.assertTrue(eval2.is_air_move)
        self.assertAlmostEqual(eval2.material_removed_mm3, 0.0, places=5)

    def test_multi_depth_pass_occurrence_gating(self):
        """Pass 1 cuts; Pass 2 at same depth clears air; verifier sets min_occur=2."""
        seg_pass1 = _make_segment(
            seg_id=1,
            motion=MotionKind.LINEAR,
            start=Vector3D(-5.0, 0.0, -1.0),
            end=Vector3D(5.0, 0.0, -1.0),
        )
        seg_pass2 = _make_segment(
            seg_id=2,
            motion=MotionKind.LINEAR,
            start=Vector3D(-5.0, 0.0, -1.0),
            end=Vector3D(5.0, 0.0, -1.0),
        )

        res1 = self.verifier.step(seg_pass1)
        self.assertFalse(res1.is_air_move)

        res2 = self.verifier.step(seg_pass2)
        self.assertTrue(res2.is_air_move)

        report = self.verifier.generate_report()
        self.assertIn((0, 2), report.promoted_events)
        self.assertNotIn((0, 1), report.promoted_events)

    def test_rapid_into_stock_collision(self):
        """A rapid (G0) move traversing into raw stock must trigger RAPID_INTO_STOCK."""
        # Rapid move traversing through stock at Z=-3.0
        rapid_crash = _make_segment(
            seg_id=10,
            motion=MotionKind.RAPID,
            start=Vector3D(-25.0, 0.0, -3.0),
            end=Vector3D(25.0, 0.0, -3.0),
        )
        eval_res = self.verifier.step(rapid_crash)

        self.assertEqual(len(eval_res.collisions), 1)
        col = eval_res.collisions[0]
        self.assertEqual(col.kind, CollisionKind.RAPID_INTO_STOCK)
        self.assertGreater(col.penetration_volume_mm3, 100.0)

    def test_holder_strike_collision(self):
        """Plunging deeper than tool overall stickout length causes toolholder to crash into stock."""
        # Tool stickout = 40mm. Stock is from Z=0 down to -20mm.
        # If tip plunges to Z=-50mm, the holder (starting at tip + 40 = -10mm) hits stock top (Z=0).
        deep_plunge = _make_segment(
            seg_id=20,
            motion=MotionKind.LINEAR,
            start=Vector3D(0.0, 0.0, 5.0),
            end=Vector3D(0.0, 0.0, -45.0),
            feedrate=200.0,
            duration_sec=15.0,
        )
        eval_res = self.verifier.step(deep_plunge)

        holder_cols = [c for c in eval_res.collisions if c.kind == CollisionKind.HOLDER_STRIKE]
        self.assertGreaterEqual(len(holder_cols), 1)
        self.assertIn("Toolholder collided with uncut stock", holder_cols[0].description)

    def test_shank_rub_collision(self):
        """Cutting with depth exceeding the flute length triggers SHANK_RUB."""
        # Flute length is 20mm. Plunge depth to Z=-25mm exceeds flute length by 5mm.
        deep_cut = _make_segment(
            seg_id=30,
            motion=MotionKind.LINEAR,
            start=Vector3D(0.0, 0.0, -25.0),
            end=Vector3D(10.0, 0.0, -25.0),
            feedrate=300.0,
            duration_sec=2.0,
        )
        eval_res = self.verifier.step(deep_cut)

        shank_cols = [c for c in eval_res.collisions if c.kind == CollisionKind.SHANK_RUB]
        self.assertGreaterEqual(len(shank_cols), 1)
        self.assertIn("shank rubbed into raw stock", shank_cols[0].description)

    def test_pure_z_plus_retract_air_promotion(self):
        """Pure vertical Z+ retract move reaching clearance height is promoted."""
        retract_move = _make_segment(
            seg_id=40,
            motion=MotionKind.LINEAR,
            start=Vector3D(5.0, 5.0, -2.0),
            end=Vector3D(5.0, 5.0, 15.0),
            metadata={"z_clearance_mm": 10.0},
        )
        eval_res = self.verifier.step(retract_move)
        self.assertTrue(eval_res.is_air_move)
        self.assertEqual(len(eval_res.collisions), 0)

    def test_circular_arc_sweeping(self):
        """Circular arc move (G2/G3) in XY plane correctly carves arc channel."""
        # 90-degree CCW arc from (10, 0) to (0, 10) around center (0, 0) at Z=-1.0
        arc_move = _make_segment(
            seg_id=50,
            motion=MotionKind.ARC_CCW,
            start=Vector3D(10.0, 0.0, -1.0),
            end=Vector3D(0.0, 10.0, -1.0),
            is_arc=True,
            arc_center=Vector3D(0.0, 0.0, -1.0),
            duration_sec=1.5,
        )
        eval_res = self.verifier.step(arc_move)

        self.assertFalse(eval_res.is_air_move)
        self.assertGreater(eval_res.material_removed_mm3, 50.0)
        self.assertEqual(len(eval_res.collisions), 0)

    def test_verify_toolpath_orchestration_workflow(self):
        """Run complete multi-segment toolpath through verify_toolpath convenience loop."""
        moves = [
            _make_segment(1, MotionKind.RAPID, Vector3D(0.0, 0.0, 20.0), Vector3D(-10.0, 0.0, 5.0)),
            _make_segment(2, MotionKind.LINEAR, Vector3D(-10.0, 0.0, 5.0), Vector3D(-10.0, 0.0, -2.0)),
            _make_segment(3, MotionKind.LINEAR, Vector3D(-10.0, 0.0, -2.0), Vector3D(10.0, 0.0, -2.0)),
            _make_segment(4, MotionKind.LINEAR, Vector3D(10.0, 0.0, -2.0), Vector3D(10.0, 0.0, 20.0), metadata={"z_clearance_mm": 15.0}),
        ]
        tools_map = {"Op1": self.tool}

        report = self.verifier.verify_toolpath(moves, tools_map, self.workpiece)

        self.assertIsInstance(report.total_volume_removed_mm3, float)
        self.assertGreater(report.total_volume_removed_mm3, 0.0)
        self.assertEqual(len(report.collisions), 0)
        self.assertGreater(len(report.mrr_timeline), 0)

        # Move 4 (vertical retract) should be promoted
        self.assertIn((0, moves[3].segment_id), report.promoted_events)

    def test_ball_endmill_carving(self):
        """Ball endmill carves a hemispherical/curved bottom trench."""
        ball_geom = MillToolGeometry(
            diameter=10.0,
            corner_radius=5.0,  # Ball endmill: rc = D/2
            flute_length=20.0,
            overall_length=40.0,
        )
        ball_tool = ToolAssembly(tool_id=2, name="10mm Ball Endmill", geometry=ball_geom)
        self.verifier.reset(self.workpiece)
        self.verifier.load_tool(ball_tool)

        move = _make_segment(
            seg_id=1,
            motion=MotionKind.LINEAR,
            start=Vector3D(-5.0, 0.0, -2.0),
            end=Vector3D(5.0, 0.0, -2.0),
            duration_sec=1.0,
        )
        res = self.verifier.step(move)
        self.assertFalse(res.is_air_move)
        self.assertGreater(res.material_removed_mm3, 0.0)

        # Centerline bottom at (c+0.5)*cs matches analytical ball profile
        cs = self.verifier.grid.cell_size_mm
        col_c = int((0.0 - self.verifier.grid.grid_x_min) / cs)
        row_c = int((0.0 - self.verifier.grid.grid_y_min) / cs)
        y_c = self.verifier.grid.grid_y_min + (row_c + 0.5) * cs
        expected_center_z = -2.0 + (5.0 - math.sqrt(25.0 - y_c * y_c))
        self.assertAlmostEqual(float(self.verifier.grid.H[row_c, col_c]), expected_center_z, places=5)

        row_edge = int((3.75 - self.verifier.grid.grid_y_min) / cs)
        y_edge = self.verifier.grid.grid_y_min + (row_edge + 0.5) * cs
        expected_edge_z = -2.0 + (5.0 - math.sqrt(25.0 - y_edge * y_edge))
        self.assertAlmostEqual(float(self.verifier.grid.H[row_edge, col_c]), expected_edge_z, places=5)
        self.assertGreater(float(self.verifier.grid.H[row_edge, col_c]), float(self.verifier.grid.H[row_c, col_c]) + 1.0)

    def test_canned_cycle_moves_never_promoted(self):
        """Moves with metadata['in_cycle']=True are never authorized for rapid promotion."""
        cycle_move = _make_segment(
            seg_id=60,
            motion=MotionKind.LINEAR,
            start=Vector3D(0.0, 0.0, 15.0),
            end=Vector3D(0.0, 0.0, 25.0),
            metadata={"in_cycle": True},
        )
        res = self.verifier.step(cycle_move)
        self.assertFalse(res.is_air_move)
        report = self.verifier.generate_report()
        self.assertNotIn((0, 60), report.promoted_events)

    def test_radius_compensation_moves_never_promoted(self):
        """Moves with metadata['rc'] != 0 or 'rc_transition' are never authorized for rapid promotion."""
        self.verifier.reset(self.workpiece)
        self.verifier.load_tool(self.tool)

        # Move with rc=1 (in the air, well above stock z_max=0)
        rc_move = _make_segment(
            seg_id=61,
            motion=MotionKind.LINEAR,
            start=Vector3D(0.0, 0.0, 15.0),
            end=Vector3D(10.0, 0.0, 15.0),
            metadata={"rc": 1},
        )
        res_rc = self.verifier.step(rc_move)
        self.assertFalse(res_rc.is_air_move)
        self.assertNotIn((0, 61), self.verifier.generate_report().promoted_events)

        # Same move with rc=0 is an air move and authorized for promotion
        self.verifier.reset(self.workpiece)
        self.verifier.load_tool(self.tool)
        no_rc_move = _make_segment(
            seg_id=62,
            motion=MotionKind.LINEAR,
            start=Vector3D(0.0, 0.0, 15.0),
            end=Vector3D(10.0, 0.0, 15.0),
            metadata={"rc": 0},
        )
        res_no_rc = self.verifier.step(no_rc_move)
        self.assertTrue(res_no_rc.is_air_move)
        self.assertIn((0, 62), self.verifier.generate_report().promoted_events)

        # Transition move (rc_transition=True) is also blocked
        self.verifier.reset(self.workpiece)
        self.verifier.load_tool(self.tool)
        trans_move = _make_segment(
            seg_id=63,
            motion=MotionKind.LINEAR,
            start=Vector3D(0.0, 0.0, 15.0),
            end=Vector3D(10.0, 0.0, 15.0),
            metadata={"rc": 0, "rc_transition": True},
        )
        res_trans = self.verifier.step(trans_move)
        self.assertFalse(res_trans.is_air_move)
        self.assertNotIn((0, 63), self.verifier.generate_report().promoted_events)

    def test_mrr_spike_detection(self):
        """Heavy step-down cut among light cuts triggers an MRR spike."""
        self.verifier.reset(self.workpiece)
        self.verifier.load_tool(self.tool)

        # Several light cuts (depth 0.1 mm)
        for i in range(10):
            m = _make_segment(
                seg_id=100 + i,
                motion=MotionKind.LINEAR,
                start=Vector3D(-10.0, float(i * 2 - 10), -0.1),
                end=Vector3D(10.0, float(i * 2 - 10), -0.1),
                duration_sec=2.0,
            )
            self.verifier.step(m)

        # One sudden deep plunge/slot (depth 5.0 mm, 50x volume)
        heavy_cut = _make_segment(
            seg_id=200,
            motion=MotionKind.LINEAR,
            start=Vector3D(-10.0, 20.0, -5.0),
            end=Vector3D(10.0, 20.0, -5.0),
            duration_sec=0.5,
        )
        self.verifier.step(heavy_cut)

        report = self.verifier.generate_report()
        self.assertGreater(len(report.mrr_spikes), 0)
        self.assertEqual(report.mrr_spikes[0].segment_id, 200)

    def test_generate_mill_verification_plot_end_to_end(self):
        """Verify that generate_mill_verification_plot creates a valid dual-panel 3D PNG."""
        import os
        import tempfile
        from geometry.plotter import generate_mill_verification_plot

        moves = [
            _make_segment(1, MotionKind.RAPID, Vector3D(-20.0, -20.0, 15.0), Vector3D(-15.0, 0.0, 5.0), op_id="Facing"),
            _make_segment(2, MotionKind.LINEAR, Vector3D(-15.0, 0.0, 5.0), Vector3D(-15.0, 0.0, -2.0), op_id="Facing"),
            _make_segment(3, MotionKind.LINEAR, Vector3D(-15.0, 0.0, -2.0), Vector3D(15.0, 0.0, -2.0), duration_sec=3.0, op_id="Facing"),
            _make_segment(4, MotionKind.LINEAR, Vector3D(15.0, 0.0, -2.0), Vector3D(15.0, 0.0, 15.0), metadata={"z_clearance_mm": 10.0}, op_id="Facing"),
        ]
        tools_map = {"Facing": self.tool}
        report = self.verifier.verify_toolpath(moves, tools_map, self.workpiece)

        with tempfile.TemporaryDirectory() as tmp_dir:
            out_png = os.path.join(tmp_dir, "mill_verification.png")
            generate_mill_verification_plot(
                workpiece=self.workpiece,
                grid=self.verifier.grid,
                segments=moves,
                report=report,
                tools_by_op=tools_map,
                out_png_path=out_png,
                unit_is_inch=False,
            )
            self.assertTrue(os.path.exists(out_png))
            self.assertGreater(os.path.getsize(out_png), 10000)

    def test_plot_mrr_timeline_gapless_and_clean_spikes(self):
        """Verify _plot_mrr_timeline creates gapless operation spans, plots all spikes, and has no text labels."""
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from geometry.plotter import _plot_mrr_timeline

        # 2 operations: Op1 (1.0s rapid + 2.0s cut) and Op2 (1.0s cut + 1.0s rapid retract)
        moves = [
            _make_segment(1, MotionKind.RAPID, Vector3D(-20, 0, 10), Vector3D(-10, 0, 0), duration_sec=1.0, op_id="Facing"),
            _make_segment(2, MotionKind.LINEAR, Vector3D(-10, 0, -1), Vector3D(10, 0, -1), duration_sec=2.0, op_id="Facing"),
            _make_segment(3, MotionKind.LINEAR, Vector3D(10, 0, -3), Vector3D(10, 10, -3), duration_sec=1.0, op_id="Contour"),
            _make_segment(4, MotionKind.RAPID, Vector3D(10, 10, 10), Vector3D(0, 0, 20), duration_sec=1.0, op_id="Contour"),
        ]
        tools_map = {"Facing": self.tool, "Contour": self.tool}
        report = self.verifier.verify_toolpath(moves, tools_map, self.workpiece)

        fig, ax = plt.subplots()
        try:
            _plot_mrr_timeline(ax, report, moves, vol_scale=1.0, vol_unit="mm³")
            # 1. Check xlim covers entire sequence: 1+2+1+1 = 5.0 seconds
            self.assertAlmostEqual(ax.get_xlim()[0], 0.0)
            self.assertAlmostEqual(ax.get_xlim()[1], 5.0)

            # 2. Check no text annotations were placed on ax
            self.assertEqual(len(ax.texts), 0, "Spike text annotations should not be created")

            # 3. Check legend does not contain tool flute or holder
            handles, labels = ax.get_legend_handles_labels()
            for lbl in labels:
                self.assertNotIn("Flute", lbl)
                self.assertNotIn("Holder", lbl)
        finally:
            plt.close(fig)

    def test_chamfer_operation_mrr_is_virtually_zero(self):
        """Verify that a 2D chamfer operation using a conical chamfer tool produces virtually zero MRR."""
        # 1. Flat endmill cuts initial rough pocket at Z=-2.0
        rough_cut = _make_segment(
            1, MotionKind.LINEAR, Vector3D(-5, 0, -2.0), Vector3D(5, 0, -2.0),
            feedrate=600.0, duration_sec=2.0, op_id="Roughing"
        )
        self.verifier.step(rough_cut)

        # 2. Load 90-degree chamfer tool (Tool 9: D=6.35, R=3.175, taper_angle_deg=90.0)
        chamfer_geom = MillToolGeometry(
            diameter=6.35,
            corner_radius=0.0,
            flute_length=15.0,
            overall_length=50.0,
            taper_angle_deg=90.0,
            holder_diameter=30.0,
            holder_length=40.0,
        )
        chamfer_tool = ToolAssembly(tool_id=9, name="1/4 90deg Chamfer Mill", geometry=chamfer_geom)
        self.verifier.load_tool(chamfer_tool)

        # 3. Run 2D Chamfer move along the top edge beveling a 0.2mm edge break
        # Tip is at Z=-2.1, center is offset at X=7.0 (slot edge at X=5.0).
        # At slot edge (dr=2.0): z_cone = -2.1 + 2.0 = -0.1 mm, taking a tiny 0.1mm corner bevel.
        chamfer_move = _make_segment(
            2, MotionKind.LINEAR, Vector3D(7.0, -3.0, -2.1), Vector3D(7.0, 3.0, -2.1),
            feedrate=1000.0, duration_sec=1.0, op_id="2D Chamfer1"
        )
        res = self.verifier.step(chamfer_move)

        # Material removed by the chamfer cut must be virtually zero (< 1.25 mm³)
        self.assertLess(res.material_removed_mm3, 1.25)
        self.assertLess(res.instantaneous_mrr, 75.0)
        self.assertEqual(len(res.collisions), 0)

    def test_mrr_spikes_capped_at_top_3_per_operation(self):
        """Verify that BaseVerificationEngine.generate_report selects top 3 MRR spikes per operation."""
        engine = MillHeightfieldVerifier()
        engine.reset(self.workpiece)

        # Op1: 30 baseline cutting samples at 50 mm3/min + 5 spikes (500, 900, 600, 800, 700)
        for i in range(30):
            engine.current_time_sec = float(i)
            engine.record_mrr_sample(
                segment_id=i,
                duration_sec=1.0,
                volume_removed_mm3=50.0 / 60.0,
                op_id="Op1",
            )
        for idx, rate in enumerate([500.0, 900.0, 600.0, 800.0, 700.0]):
            engine.current_time_sec = 40.0 + float(idx * 2)
            engine.record_mrr_sample(
                segment_id=100 + idx,
                duration_sec=1.0,
                volume_removed_mm3=rate / 60.0,
                op_id="Op1",
            )

        # Op2: 30 baseline cutting samples at 50 mm3/min + 4 spikes (400, 1000, 450, 950)
        for i in range(30):
            engine.current_time_sec = 60.0 + float(i)
            engine.record_mrr_sample(
                segment_id=150 + i,
                duration_sec=1.0,
                volume_removed_mm3=50.0 / 60.0,
                op_id="Op2",
            )
        for idx, rate in enumerate([400.0, 1000.0, 450.0, 950.0]):
            engine.current_time_sec = 100.0 + float(idx * 2)
            engine.record_mrr_sample(
                segment_id=200 + idx,
                duration_sec=1.0,
                volume_removed_mm3=rate / 60.0,
                op_id="Op2",
            )

        report = engine.generate_report()

        # Exactly 3 spikes from Op1 and 3 spikes from Op2 = 6 spikes total
        op1_spikes = [s for s in report.mrr_spikes if s.op_id == "Op1"]
        op2_spikes = [s for s in report.mrr_spikes if s.op_id == "Op2"]

        self.assertEqual(len(op1_spikes), 3)
        self.assertEqual(len(op2_spikes), 3)
        self.assertEqual(len(report.mrr_spikes), 6)

        # For Op1, top 3 rates are 900, 800, 700
        op1_rates = sorted([s.mrr_rate_mm3_per_min for s in op1_spikes], reverse=True)
        self.assertAlmostEqual(op1_rates[0], 900.0, places=1)
        self.assertAlmostEqual(op1_rates[1], 800.0, places=1)
        self.assertAlmostEqual(op1_rates[2], 700.0, places=1)

        # For Op2, top 3 rates are 1000, 950, 450
        op2_rates = sorted([s.mrr_rate_mm3_per_min for s in op2_spikes], reverse=True)
        self.assertAlmostEqual(op2_rates[0], 1000.0, places=1)
        self.assertAlmostEqual(op2_rates[1], 950.0, places=1)
        self.assertAlmostEqual(op2_rates[2], 450.0, places=1)

        # Verify chronological ordering across report.mrr_spikes
        timestamps = [s.timestamp_sec for s in report.mrr_spikes]
        self.assertEqual(timestamps, sorted(timestamps))

    def test_promotion_clearance_wall_at_tool_r_plus_margin(self):
        """
        Phase 2 H2: Verify that mill heightfield promotion clearance checks lateral clearance
        with clearance_r = tool_r + promote_margin_mm + cell_size * sqrt(2)/2.
        - Tool radius = 5.0 mm, promote_margin_mm = 0.5 mm, cell_size = 0.5 mm.
        - Wall at tool_r + 0.2 mm (X = 5.2 mm): inside clearance_r (~5.85 mm), so promotion is BLOCKED.
        - Wall at tool_r + 1.0 mm (X = 6.0 mm): outside clearance_r, so promotion is ALLOWED.
        """
        tool_r = 5.0
        tool = ToolAssembly(
            tool_id=1,
            name="Flat Endmill D10",
            geometry=MillToolGeometry(
                diameter=tool_r * 2.0,
                corner_radius=0.0,
                flute_length=25.0,
                overall_length=50.0,
                holder_diameter=30.0,
                holder_length=40.0,
            ),
        )

        verifier = MillHeightfieldVerifier(cell_size_mm=0.5, pad_mm=5.0, promote_margin_mm=0.5)

        # 1. Wall at tool_r + 0.2 mm (x_min = 5.2 mm)
        stock_wall_close = BoxStock3D(x_min=5.2, x_max=20.0, y_min=-20.0, y_max=20.0, z_min=-20.0, z_max=0.0)
        verifier.reset(WorkpieceDefinition(stock=stock_wall_close))
        verifier.load_tool(tool)

        # Feed move passing beside the wall along Y at X=0, Z=-5.0 (tool outer edge at X=5.0)
        move_near_wall = _make_segment(
            seg_id=1,
            motion=MotionKind.LINEAR,
            start=Vector3D(0.0, -10.0, -5.0),
            end=Vector3D(0.0, 10.0, -5.0),
            feedrate=600.0,
            duration_sec=2.0,
        )
        eval_near = verifier.step(move_near_wall)
        self.assertEqual(eval_near.material_removed_mm3, 0.0, "Tool must not cut stock")
        self.assertEqual(len(eval_near.collisions), 0, "No collision")
        self.assertFalse(eval_near.is_air_move, "Promotion must be BLOCKED for wall at tool_r + 0.2 mm")

        # 2. Wall at tool_r + 1.0 mm (x_min = 6.0 mm)
        stock_wall_far = BoxStock3D(x_min=6.0, x_max=20.0, y_min=-20.0, y_max=20.0, z_min=-20.0, z_max=0.0)
        verifier.reset(WorkpieceDefinition(stock=stock_wall_far))
        verifier.load_tool(tool)

        move_far_wall = _make_segment(
            seg_id=2,
            motion=MotionKind.LINEAR,
            start=Vector3D(0.0, -10.0, -5.0),
            end=Vector3D(0.0, 10.0, -5.0),
            feedrate=600.0,
            duration_sec=2.0,
        )
        eval_far = verifier.step(move_far_wall)
        self.assertEqual(eval_far.material_removed_mm3, 0.0)
        self.assertEqual(len(eval_far.collisions), 0)
        self.assertTrue(eval_far.is_air_move, "Promotion must be ALLOWED for wall at tool_r + 1.0 mm")

    def test_vertical_retract_falls_back_to_sec_clearance_when_meta_z_clearance_is_none(self):
        """
        Verify that when metadata contains {'z_clearance_mm': None} (as emitted by context_parser
        when z_clearance_mm is absent), _step_linear falls back to _sec_clearance_mm[sec_id]
        and promotes a pure vertical Z+ retract to clearance height.
        """
        verifier = MillHeightfieldVerifier(cell_size_mm=0.5, pad_mm=5.0, promote_margin_mm=0.5)
        verifier.reset(self.workpiece)
        verifier.load_tool(self.tool)

        # Initial positioning at Z=15.0 establishes _sec_clearance_mm[0] = 15.0
        init_move = _make_segment(
            seg_id=1,
            motion=MotionKind.RAPID,
            start=Vector3D(0.0, 0.0, 15.0),
            end=Vector3D(10.0, 10.0, 15.0),
            metadata={"z_clearance_mm": None},
        )
        verifier.step(init_move)

        # Pure vertical Z+ retract from inside pocket (Z=-5) to Z=15 with z_clearance_mm=None
        retract_move = _make_segment(
            seg_id=2,
            motion=MotionKind.LINEAR,
            start=Vector3D(10.0, 10.0, -5.0),
            end=Vector3D(10.0, 10.0, 15.0),
            feedrate=500.0,
            metadata={"z_clearance_mm": None},
        )
        eval_retract = verifier.step(retract_move)
        self.assertTrue(eval_retract.is_air_move)
        self.assertIn((0, 2), verifier.promoted_events)

    def test_arc_mrr_aggregation_single_sample(self):
        """Cutting arc move records 1 aggregated MRRSample without chordal dilution."""
        self.verifier.reset(self.workpiece)
        self.verifier.load_tool(self.tool)

        arc_move = _make_segment(
            seg_id=501,
            motion=MotionKind.ARC_CCW,
            start=Vector3D(10.0, 0.0, -1.5),
            end=Vector3D(0.0, 10.0, -1.5),
            is_arc=True,
            arc_center=Vector3D(0.0, 0.0, -1.5),
            duration_sec=1.5,
            op_id="ArcOp",
        )
        res = self.verifier.step(arc_move)

        self.assertGreater(res.material_removed_mm3, 0.0)
        self.assertEqual(len(self.verifier.mrr_timeline), 1)
        sample = self.verifier.mrr_timeline[0]
        expected_mrr = (res.material_removed_mm3 / arc_move.duration_sec) * 60.0
        self.assertAlmostEqual(sample.mrr_rate_mm3_per_min, expected_mrr, places=4)
        self.assertAlmostEqual(res.instantaneous_mrr, expected_mrr, places=4)
        self.assertAlmostEqual(sample.volume_removed_mm3, res.material_removed_mm3, places=4)
        self.assertGreater(sample.depth_of_cut_mm, 0.0)
        self.assertEqual(sample.op_id, "ArcOp")
        self.assertNotIn((0, 501), self.verifier.promoted_events)

    def test_time_weighted_mrr_immune_to_short_lead_in_segments(self):
        """Short low-MRR micro-segments do not drag down time-weighted mean MRR."""
        engine = MillHeightfieldVerifier()
        engine.reset(self.workpiece)

        t = 0.0
        # 200 short (0.015 s) helical ramp / lead-in micro-segments at 50 mm3/min
        for i in range(200):
            t += 0.015
            engine.current_time_sec = t
            engine.record_mrr_sample(
                segment_id=i,
                duration_sec=0.015,
                volume_removed_mm3=(50.0 / 60.0) * 0.015,
                op_id="Pocket1",
            )

        # 10 long (1.5 s) steady-state cuts at 3000-4500 mm3/min
        steady_rates = [3000.0, 3500.0, 4000.0, 4200.0, 4500.0, 3200.0, 3800.0, 4100.0, 4400.0, 3600.0]
        for idx, rate in enumerate(steady_rates):
            t += 1.5
            engine.current_time_sec = t
            engine.record_mrr_sample(
                segment_id=300 + idx,
                duration_sec=1.5,
                volume_removed_mm3=(rate / 60.0) * 1.5,
                op_id="Pocket1",
            )

        report_no_spike = engine.generate_report()
        self.assertEqual(len(report_no_spike.mrr_spikes), 0)

        # Add a genuine spike (> 2.5x the ~3438 mm3/min time-weighted mean)
        t += 0.5
        engine.current_time_sec = t
        engine.record_mrr_sample(
            segment_id=999,
            duration_sec=0.5,
            volume_removed_mm3=(12000.0 / 60.0) * 0.5,
            op_id="Pocket1",
        )

        report_with_spike = engine.generate_report()
        self.assertEqual(len(report_with_spike.mrr_spikes), 1)
        self.assertEqual(report_with_spike.mrr_spikes[0].segment_id, 999)

    def test_per_operation_mrr_isolation_constant_drilling(self):
        """Constant-MRR drilling operation paired with a lighter operation reports 0 spikes."""
        engine = MillHeightfieldVerifier()
        engine.reset(self.workpiece)

        t = 0.0
        # Light finishing operation at 200 mm3/min
        for i in range(50):
            t += 1.0
            engine.current_time_sec = t
            engine.record_mrr_sample(
                segment_id=i,
                duration_sec=1.0,
                volume_removed_mm3=200.0 / 60.0,
                op_id="LightFinish",
            )

        # Constant-MRR drilling operation: 6 pecks all at 1772.73 mm3/min
        for j in range(6):
            t += 0.8
            engine.current_time_sec = t
            engine.record_mrr_sample(
                segment_id=500 + j,
                duration_sec=0.8,
                volume_removed_mm3=(1772.73 / 60.0) * 0.8,
                op_id="Drill7",
            )

        report = engine.generate_report()
        drill_spikes = [s for s in report.mrr_spikes if s.op_id == "Drill7"]
        self.assertEqual(len(drill_spikes), 0)
        self.assertEqual(len(report.mrr_spikes), 0)

    def test_arc_collision_deduplicated_to_single_record_per_kind(self):
        """Arc move causing shank rub / holder strike records 1 CollisionRecord per kind, not per chordal step."""
        short_stickout_tool = ToolAssembly(
            tool_id=3,
            name="10mm Short Stickout Endmill",
            geometry=MillToolGeometry(
                diameter=10.0,
                corner_radius=0.0,
                flute_length=15.0,
                overall_length=22.0,
                holder_diameter=30.0,
                holder_length=40.0,
            ),
        )
        self.verifier.reset(self.workpiece)
        self.verifier.load_tool(short_stickout_tool)

        # 90-degree CCW arc (9 chordal steps) at Z=-25.0:
        # shank bottom (-10.0) and holder bottom (-3.0) are both below stock top (0.0).
        arc_move = _make_segment(
            seg_id=502,
            motion=MotionKind.ARC_CCW,
            start=Vector3D(10.0, 0.0, -25.0),
            end=Vector3D(0.0, 10.0, -25.0),
            is_arc=True,
            arc_center=Vector3D(0.0, 0.0, -25.0),
            duration_sec=1.5,
            op_id="DeepArc",
        )
        res = self.verifier.step(arc_move)

        shank_cols = [c for c in self.verifier.collisions if c.kind == CollisionKind.SHANK_RUB]
        holder_cols = [c for c in self.verifier.collisions if c.kind == CollisionKind.HOLDER_STRIKE]
        self.assertEqual(len(shank_cols), 1)
        self.assertEqual(len(holder_cols), 1)
        self.assertEqual(len(self.verifier.collisions), 2)
        self.assertEqual(len(res.collisions), 2)
        self.assertNotIn((0, 502), self.verifier.promoted_events)


if __name__ == "__main__":
    unittest.main()

