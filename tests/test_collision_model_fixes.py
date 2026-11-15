#!/usr/bin/env python3
"""
Unit tests for verification-engine fixes C1, C2, C3, R3, and MRR duration floor.
"""

import math
import unittest

import numpy as np

from geometry.heightfield_engine import HeightfieldGrid, MillHeightfieldVerifier
from geometry.mill_engine import MillClearanceVerifier
from geometry.models import (
    BoxStock3D,
    CollisionKind,
    MillToolGeometry,
    MotionKind,
    MotionSegment,
    SolidMeshStock3D,
    ToolAssembly,
    Vector3D,
    WorkpieceDefinition,
)
from geometry.solid_mesh import rasterize_triangles_to_grid


def _make_box_mesh(
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    z_min: float,
    z_max: float,
):
    """Build a watertight 12-triangle mesh for an axis-aligned box."""
    verts = np.array(
        [
            [x_min, y_min, z_min],  # 0
            [x_max, y_min, z_min],  # 1
            [x_max, y_max, z_min],  # 2
            [x_min, y_max, z_min],  # 3
            [x_min, y_min, z_max],  # 4
            [x_max, y_min, z_max],  # 5
            [x_max, y_max, z_max],  # 6
            [x_min, y_max, z_max],  # 7
        ],
        dtype=np.float32,
    )
    indices = np.array(
        [
            [0, 2, 1], [0, 3, 2],  # bottom (-Z)
            [4, 5, 6], [4, 6, 7],  # top (+Z)
            [0, 1, 5], [0, 5, 4],  # front (-Y)
            [2, 3, 7], [2, 7, 6],  # back (+Y)
            [0, 4, 7], [0, 7, 3],  # left (-X)
            [1, 2, 6], [1, 6, 5],  # right (+X)
        ],
        dtype=np.int32,
    )
    return verts, indices


def _make_segment(
    seg_id: int,
    motion: MotionKind,
    start: Vector3D,
    end: Vector3D,
    *,
    op_id: str = "Op1",
    sec_id: int = 0,
    feedrate: float = 1000.0,
    duration_sec: float = 1.0,
    is_arc: bool = False,
    arc_center: Vector3D = None,
    arc_plane: str = "XY",
) -> MotionSegment:
    return MotionSegment(
        segment_id=seg_id,
        op_id=op_id,
        sec_id=sec_id,
        motion=motion,
        start=start,
        end=end,
        feedrate_mmpm=feedrate,
        spindle_rpm=12000.0,
        duration_sec=duration_sec,
        is_arc=is_arc,
        arc_center=arc_center,
        arc_plane=arc_plane,
    )


class TestC1RasterizerAndCellConvention(unittest.TestCase):
    """C1: Triangle Z clamping, scale-relative sliver handling, and (c+0.5)*cs convention."""

    def test_sliver_triangle_clamped_in_small_multi_and_fallback(self):
        grid = HeightfieldGrid(cell_size_mm=0.5, pad_mm=0.0)
        grid.grid_x_min = -10.0
        grid.grid_y_min = -10.0
        grid.nx = 60
        grid.ny = 60
        grid.H = np.full((60, 60), -10.0, dtype=np.float64)

        # Triangle 1 (small-multi: ~2x2 cells): nearly collinear in XY with steep Z,
        # where raw plane extrapolation outside the triangle edges would spike well above z_max.
        # Triangle 2 (large fallback: spans > 20x20 cells): near-vertical sliver in XY
        # with |nz| small relative to squared edge length.
        # Triangle 3 (large fallback): steep plane whose corners sit inside Z in [0, 10]
        # while bounding box corners extrapolate outside [0, 10].
        verts = np.array(
            [
                # Tri 1: small-multi steep sliver (z in [0, 5])
                [0.0, 0.0, 0.0],
                [0.8, 0.0, 0.0],
                [0.4, 1e-4, 5.0],
                # Tri 2: large fallback near-vertical sliver (edge ~15mm, |nz| ~ 1e-5 * edge^2)
                [-8.0, -8.0, 1.0],
                [7.0, 7.0, 2.0],
                [-0.5, -0.5 + 1e-5, 12.0],
                # Tri 3: large fallback steep ramp (z in [0, 10])
                [2.0, -8.0, 0.0],
                [14.0, -7.9, 10.0],
                [8.0, 8.0, 10.0],
                # Tri 4: flat reference cap at z = 8.0
                [-8.0, 4.0, 8.0],
                [-2.0, 4.0, 8.0],
                [-5.0, 8.0, 8.0],
            ],
            dtype=np.float32,
        )
        indices = np.array([[0, 1, 2], [3, 4, 5], [6, 7, 8], [9, 10, 11]], dtype=np.int32)
        pad = 0.05
        rasterize_triangles_to_grid(grid, verts, indices, tol_pad_z_mm=pad)

        max_tri_z = float(verts[:, 2].max())
        self.assertTrue(np.all(np.isfinite(grid.H)))
        self.assertLessEqual(float(grid.H.max()), max_tri_z + pad + 1e-6)

        # Verify flat cap actually rasterized at 8.0 + pad
        col_flat = int(math.floor((-5.0 - grid.grid_x_min) / grid.cell_size_mm))
        row_flat = int(math.floor((5.5 - grid.grid_y_min) / grid.cell_size_mm))
        self.assertAlmostEqual(float(grid.H[row_flat, col_flat]), 8.0 + pad, places=4)

    def test_cell_center_convention_consistent_across_grid_methods(self):
        cs = 0.5
        pad = 10.0
        stock = BoxStock3D(0.0, 20.0, 0.0, 20.0, 0.0, 30.0)
        grid = HeightfieldGrid(cell_size_mm=cs, pad_mm=pad)
        grid.reset(stock)

        xs, ys = grid.get_mesh_coordinates(0, grid.nx, 0, grid.ny)
        self.assertAlmostEqual(float(xs[0, 0]), grid.grid_x_min + 0.5 * cs, places=6)
        self.assertAlmostEqual(float(ys[0, 0]), grid.grid_y_min + 0.5 * cs, places=6)
        self.assertAlmostEqual(
            float(xs[0, -1]), grid.grid_x_min + (grid.nx - 0.5) * cs, places=6
        )

        # Subgrid bounds for [0, 20] must include all cell centers in [0, 20]
        c_min, c_max, r_min, r_max = grid.get_subgrid_bounds(0.0, 0.0, 20.0, 20.0, 0.0)
        sub_xs, sub_ys = grid.get_mesh_coordinates(c_min, c_max, r_min, r_max)
        self.assertLessEqual(float(sub_xs.min()), 0.25 + 1e-6)
        self.assertGreaterEqual(float(sub_xs.max()), 19.75 - 1e-6)
        self.assertLessEqual(float(sub_ys.min()), 0.25 + 1e-6)
        self.assertGreaterEqual(float(sub_ys.max()), 19.75 - 1e-6)

        # Compare BoxStock3D vs SolidMeshStock3D interior height alignment
        verts, idx = _make_box_mesh(0.0, 20.0, 0.0, 20.0, 0.0, 30.0)
        mesh_grid = HeightfieldGrid(cell_size_mm=cs, pad_mm=pad)
        mesh_grid.reset_from_mesh(verts, idx, tol_pad_z_mm=0.05)
        cx = int(math.floor((10.0 - mesh_grid.grid_x_min) / cs))
        cy = int(math.floor((10.0 - mesh_grid.grid_y_min) / cs))
        self.assertAlmostEqual(float(grid.H[cy, cx]), 30.0, places=4)
        self.assertAlmostEqual(float(mesh_grid.H[cy, cx]), 30.05, places=4)


class TestC2TangentWallDiscretization(unittest.TestCase):
    """C2: Lateral tolerance and minimum penetration volume on SHANK_RUB / HOLDER_STRIKE."""

    def _run_wall_pass(self, stock, x_pass: float, **verifier_kwargs):
        tool = ToolAssembly(
            tool_id=1,
            name="6mm Flat",
            geometry=MillToolGeometry(
                diameter=6.0,
                corner_radius=0.0,
                flute_length=19.05,
                overall_length=50.0,
                holder_diameter=30.0,
                holder_length=40.0,
            ),
        )
        verifier = MillHeightfieldVerifier(**verifier_kwargs)
        verifier.reset(WorkpieceDefinition(stock=stock))
        verifier.load_tool(tool)
        seg = _make_segment(
            1,
            MotionKind.LINEAR,
            Vector3D(x_pass, -10.0, -25.0),
            Vector3D(x_pass, 10.0, -25.0),
        )
        return verifier.step(seg), verifier

    def test_tangent_pass_no_false_shank_rub_box_and_stl(self):
        box_stock = BoxStock3D(0.0, 20.0, -20.0, 20.0, -30.0, 0.0)
        verts, idx = _make_box_mesh(0.0, 20.0, -20.0, 20.0, -30.0, 0.0)
        stl_stock = SolidMeshStock3D.from_mesh(verts, idx)

        for label, stock in (("BoxStock3D", box_stock), ("SolidMeshStock3D", stl_stock)):
            # Exact tangent on +X wall (X = 20 + 3 = 23) and -X wall (X = 0 - 3 = -3)
            for x_tangent in (23.0, -3.0):
                res, _ = self._run_wall_pass(stock, x_tangent)
                shank_cols = [c for c in res.collisions if c.kind == CollisionKind.SHANK_RUB]
                self.assertEqual(
                    len(shank_cols),
                    0,
                    f"Unexpected SHANK_RUB on {label} tangent pass at X={x_tangent}: {shank_cols}",
                )

            # 1 mm into the +X wall (X = 22.0) and -X wall (X = -2.0) must trigger SHANK_RUB
            for x_intrude in (22.0, -2.0):
                res, _ = self._run_wall_pass(stock, x_intrude)
                shank_cols = [c for c in res.collisions if c.kind == CollisionKind.SHANK_RUB]
                self.assertGreater(
                    len(shank_cols),
                    0,
                    f"Expected SHANK_RUB on {label} 1mm intrusion at X={x_intrude}",
                )

    def test_holder_strike_and_configurable_volume_and_conservative_removal(self):
        stock = BoxStock3D(0.0, 20.0, -20.0, 20.0, -30.0, 0.0)
        # Tip at Z=-55 puts holder bottom (overall_length=50) at Z=-5.0 (below stock top Z=0)
        tool = ToolAssembly(
            tool_id=2,
            name="HolderCheck",
            geometry=MillToolGeometry(
                diameter=6.0,
                corner_radius=0.0,
                flute_length=19.05,
                overall_length=50.0,
                holder_diameter=30.0,
                holder_length=40.0,
            ),
        )
        # Holder radius = 15.0: exact tangent pass at X = 20 + 15 = 35.0 -> no HOLDER_STRIKE
        v_tan = MillHeightfieldVerifier()
        v_tan.reset(WorkpieceDefinition(stock=stock))
        v_tan.load_tool(tool)
        res_tan = v_tan.step(
            _make_segment(1, MotionKind.LINEAR, Vector3D(35.0, -5.0, -55.0), Vector3D(35.0, 5.0, -55.0))
        )
        self.assertEqual([c for c in res_tan.collisions if c.kind == CollisionKind.HOLDER_STRIKE], [])

        # Intruding 2 mm into +X wall (X = 33.0) -> HOLDER_STRIKE
        v_hit = MillHeightfieldVerifier()
        v_hit.reset(WorkpieceDefinition(stock=stock))
        v_hit.load_tool(tool)
        res_hit = v_hit.step(
            _make_segment(2, MotionKind.LINEAR, Vector3D(33.0, -5.0, -55.0), Vector3D(33.0, 5.0, -55.0))
        )
        self.assertTrue(any(c.kind == CollisionKind.HOLDER_STRIKE for c in res_hit.collisions))

        # High min_collision_volume_mm3 suppresses small-volume intrusion when configured
        v_hi_vol = MillHeightfieldVerifier(min_collision_volume_mm3=5000.0)
        v_hi_vol.reset(WorkpieceDefinition(stock=stock))
        v_hi_vol.load_tool(tool)
        res_suppressed = v_hi_vol.step(
            _make_segment(3, MotionKind.LINEAR, Vector3D(22.0, -5.0, -25.0), Vector3D(22.0, 5.0, -25.0))
        )
        self.assertEqual([c for c in res_suppressed.collisions if c.kind == CollisionKind.SHANK_RUB], [])

        # Conservative material removal & rapid promotion clearance remain intact
        v_cons = MillHeightfieldVerifier()
        v_cons.reset(WorkpieceDefinition(stock=stock))
        v_cons.load_tool(tool)
        # Rapid at Z=0.1mm is within promote_margin_mm=0.5mm of Z=0 stock -> must NOT promote, no rapid collision
        rapid_near = v_cons.step(
            _make_segment(4, MotionKind.RAPID, Vector3D(5.0, 0.0, 0.1), Vector3D(15.0, 0.0, 0.1))
        )
        self.assertEqual(len(rapid_near.collisions), 0)
        cut_in = v_cons.step(
            _make_segment(5, MotionKind.LINEAR, Vector3D(19.0, -5.0, -5.0), Vector3D(19.0, 5.0, -5.0))
        )
        self.assertGreater(cut_in.material_removed_mm3, 0.0)


class TestC3ToolModel(unittest.TestCase):
    """C3: shaft_diameter, body_length, holder_sections, and no-holder skip."""

    def test_necked_tool_no_false_shank_rub(self):
        stock = BoxStock3D(-20.0, 20.0, -20.0, 20.0, -30.0, 0.0)
        # Pre-cut 10mm wide slot (walls at X = -5.0 and X = +5.0)
        slot_cutter = ToolAssembly(
            tool_id=1,
            name="10mm Slot",
            geometry=MillToolGeometry(
                diameter=10.0,
                corner_radius=0.0,
                flute_length=15.0,
                overall_length=50.0,
                holder_diameter=0.0,
                holder_length=0.0,
            ),
        )
        necked_tool = ToolAssembly(
            tool_id=2,
            name="10mm Necked 6mm Shank",
            geometry=MillToolGeometry(
                diameter=10.0,
                corner_radius=0.0,
                flute_length=10.0,
                overall_length=50.0,
                body_length=40.0,
                shaft_diameter=6.0,
                holder_diameter=0.0,
                holder_length=0.0,
            ),
        )
        v = MillHeightfieldVerifier()
        v.reset(WorkpieceDefinition(stock=stock))
        v.load_tool(slot_cutter)
        v.step(_make_segment(1, MotionKind.LINEAR, Vector3D(0.0, -10.0, -10.0), Vector3D(0.0, 10.0, -10.0)))

        v.load_tool(necked_tool)
        # Deep 20mm pass inside the 10mm slot: shank diameter 6mm clears the 10mm slot walls
        res = v.step(_make_segment(2, MotionKind.LINEAR, Vector3D(0.0, -10.0, -20.0), Vector3D(0.0, 10.0, -20.0)))
        self.assertEqual([c for c in res.collisions if c.kind == CollisionKind.SHANK_RUB], [])

    def test_wider_shaft_catches_rub(self):
        stock = BoxStock3D(0.0, 20.0, -20.0, 20.0, -30.0, 0.0)
        wide_shaft_tool = ToolAssembly(
            tool_id=3,
            name="6mm Cutter 10mm Shank",
            geometry=MillToolGeometry(
                diameter=6.0,
                corner_radius=0.0,
                flute_length=10.0,
                overall_length=50.0,
                body_length=40.0,
                shaft_diameter=10.0,
                holder_diameter=0.0,
                holder_length=0.0,
            ),
        )
        v = MillHeightfieldVerifier()
        v.reset(WorkpieceDefinition(stock=stock))
        v.load_tool(wide_shaft_tool)
        # Pass at X = -3.8: cutter (r=3) clears X=0 wall, but 10mm shaft (r=5) intrudes by 1.2mm
        res = v.step(_make_segment(1, MotionKind.LINEAR, Vector3D(-3.8, -10.0, -20.0), Vector3D(-3.8, 10.0, -20.0)))
        self.assertTrue(any(c.kind == CollisionKind.SHANK_RUB for c in res.collisions))

    def test_body_length_vs_overall_length_holder_bottom(self):
        stock = BoxStock3D(0.0, 20.0, -20.0, 20.0, -30.0, 0.0)
        seg = _make_segment(1, MotionKind.LINEAR, Vector3D(30.0, -5.0, -26.0), Vector3D(30.0, 5.0, -26.0))

        # body_length=25 -> holder bottom at Z=-26+25 = -1.0 <= 0 -> HOLDER_STRIKE (r=15 hits X=20)
        tool_short_body = ToolAssembly(
            tool_id=4,
            name="ShortBody",
            geometry=MillToolGeometry(
                diameter=6.0,
                corner_radius=0.0,
                flute_length=15.0,
                overall_length=50.0,
                body_length=25.0,
                holder_diameter=30.0,
                holder_length=40.0,
            ),
        )
        v1 = MillHeightfieldVerifier()
        v1.reset(WorkpieceDefinition(stock=stock))
        v1.load_tool(tool_short_body)
        res1 = v1.step(seg)
        self.assertTrue(any(c.kind == CollisionKind.HOLDER_STRIKE for c in res1.collisions))

        # body_length=0 -> fallback overall_length=50 -> holder bottom at Z=+24.0 > 0 -> no HOLDER_STRIKE
        tool_fallback = ToolAssembly(
            tool_id=5,
            name="FallbackOverall",
            geometry=MillToolGeometry(
                diameter=6.0,
                corner_radius=0.0,
                flute_length=15.0,
                overall_length=50.0,
                body_length=0.0,
                holder_diameter=30.0,
                holder_length=40.0,
            ),
        )
        v2 = MillHeightfieldVerifier()
        v2.reset(WorkpieceDefinition(stock=stock))
        v2.load_tool(tool_fallback)
        res2 = v2.step(seg)
        self.assertEqual([c for c in res2.collisions if c.kind == CollisionKind.HOLDER_STRIKE], [])

    def test_holder_sections_stepped_profile(self):
        stock = BoxStock3D(0.0, 20.0, -20.0, 20.0, -40.0, 0.0)
        # Lower holder section 10mm tall x 12mm dia (r=6), upper section 15mm tall x 30mm dia (r=15)
        stepped_tool = ToolAssembly(
            tool_id=6,
            name="SteppedHolder",
            geometry=MillToolGeometry(
                diameter=6.0,
                corner_radius=0.0,
                flute_length=15.0,
                overall_length=45.0,
                body_length=20.0,
                holder_diameter=0.0,
                holder_length=0.0,
                holder_sections=[(10.0, 12.0), (15.0, 30.0)],
            ),
        )
        v = MillHeightfieldVerifier()
        v.reset(WorkpieceDefinition(stock=stock))
        v.load_tool(stepped_tool)

        # At tip Z=-25, holder starts at Z=-5: lower section (r=6) spans [-5, +5], upper (r=15) spans [+5, +20].
        # Pass at X=28 (8mm from X=20 wall): lower section clears (6 < 8), upper section is above Z=0 -> no strike.
        res_clear = v.step(
            _make_segment(1, MotionKind.LINEAR, Vector3D(28.0, -5.0, -25.0), Vector3D(28.0, 5.0, -25.0))
        )
        self.assertEqual([c for c in res_clear.collisions if c.kind == CollisionKind.HOLDER_STRIKE], [])

        # At tip Z=-35, holder starts at Z=-15: upper section (r=15) starts at Z=-5 <= 0 -> hits X=20 wall!
        res_hit = v.step(
            _make_segment(2, MotionKind.LINEAR, Vector3D(28.0, -5.0, -35.0), Vector3D(28.0, 5.0, -35.0))
        )
        self.assertTrue(any(c.kind == CollisionKind.HOLDER_STRIKE for c in res_hit.collisions))

    def test_no_holder_skips_check_and_records_unchecked_ops_in_both_engines(self):
        stock = BoxStock3D(0.0, 20.0, -20.0, 20.0, -60.0, 0.0)
        no_holder_tool = ToolAssembly(
            tool_id=7,
            name="NoHolder",
            geometry=MillToolGeometry(
                diameter=6.0,
                corner_radius=0.0,
                flute_length=15.0,
                overall_length=30.0,
                holder_diameter=0.0,
                holder_length=0.0,
                holder_sections=None,
            ),
        )
        seg = _make_segment(
            1,
            MotionKind.LINEAR,
            Vector3D(28.0, -5.0, -50.0),
            Vector3D(28.0, 5.0, -50.0),
            op_id="OpNoHolder",
        )

        # Heightfield engine
        hf = MillHeightfieldVerifier()
        hf.reset(WorkpieceDefinition(stock=stock))
        hf.load_tool(no_holder_tool)
        res_hf = hf.step(seg)
        self.assertEqual([c for c in res_hf.collisions if c.kind == CollisionKind.HOLDER_STRIKE], [])
        self.assertIn("OpNoHolder", hf.holder_unchecked_ops)
        hf.reset(WorkpieceDefinition(stock=stock))
        self.assertEqual(len(hf.holder_unchecked_ops), 0)

        # Analytical clearance engine (mill_engine.py)
        mc = MillClearanceVerifier()
        mc.reset(WorkpieceDefinition(stock=stock))
        mc.load_tool(no_holder_tool)
        res_mc = mc.step(seg)
        self.assertEqual([c for c in res_mc.collisions if c.kind == CollisionKind.HOLDER_STRIKE], [])
        self.assertIn("OpNoHolder", mc.holder_unchecked_ops)
        mc.reset(WorkpieceDefinition(stock=stock))
        self.assertEqual(len(mc.holder_unchecked_ops), 0)


class TestR3NonXYArcs(unittest.TestCase):
    """R3: Support ZX and YZ arcs in _step_arc."""

    def test_zx_lead_in_arc_carves_in_zx_plane_only(self):
        stock = BoxStock3D(0.0, 30.0, -10.0, 10.0, -10.0, 0.0)
        tool = ToolAssembly(
            tool_id=1,
            name="6mm Flat",
            geometry=MillToolGeometry(
                diameter=6.0,
                corner_radius=0.0,
                flute_length=20.0,
                overall_length=50.0,
                holder_diameter=30.0,
                holder_length=40.0,
            ),
        )
        v = MillHeightfieldVerifier(cell_size_mm=0.5)
        v.reset(WorkpieceDefinition(stock=stock))
        v.load_tool(tool)

        # Quarter-circle ZX lead-in arc at Y=0:
        # Center=(15, 0, 0), Start=(10, 0, 0), End=(15, 0, -5).
        # In right-handed (u=Z, v=X): start=(0,-5) angle=-pi/2, end=(-5,0) angle=pi -> CW quarter turn.
        arc_zx = _make_segment(
            1,
            MotionKind.ARC_CW,
            Vector3D(10.0, 0.0, 0.0),
            Vector3D(15.0, 0.0, -5.0),
            is_arc=True,
            arc_center=Vector3D(15.0, 0.0, 0.0),
            arc_plane="ZX",
        )
        res = v.step(arc_zx)
        self.assertGreater(res.material_removed_mm3, 10.0)

        # At arc endpoint (X=15, Y=0) tool bottom reaches Z=-5.0
        c_end = int(math.floor((15.0 - v.grid.grid_x_min) / v.grid.cell_size_mm))
        r_mid = int(math.floor((0.0 - v.grid.grid_y_min) / v.grid.cell_size_mm))
        self.assertAlmostEqual(float(v.grid.H[r_mid, c_end]), -5.0, delta=0.1)

        # At X=11.464, Y=0, flat r=3mm disk reaches Z=-4.971 while tool sweeps from (10,0) to (15,-5)
        c_mid = int(math.floor((11.464 - v.grid.grid_x_min) / v.grid.cell_size_mm))
        self.assertAlmostEqual(float(v.grid.H[r_mid, c_mid]), -4.95, delta=0.15)

        # Symmetric across Y=0 (characteristic of ZX plane, NOT XY arc which swings through Y=+5)
        r_plus = int(math.floor((1.5 - v.grid.grid_y_min) / v.grid.cell_size_mm))
        r_minus = int(math.floor((-1.5 - v.grid.grid_y_min) / v.grid.cell_size_mm))
        self.assertAlmostEqual(float(v.grid.H[r_plus, c_end]), float(v.grid.H[r_minus, c_end]), places=2)

        # Off the ZX plane at Y = +6.0 (tool radius is 3.0): must remain untouched at Z = 0.0
        r_off = int(math.floor((6.0 - v.grid.grid_y_min) / v.grid.cell_size_mm))
        self.assertAlmostEqual(float(v.grid.H[r_off, c_mid]), 0.0, places=4)

    def test_yz_arc_carves_in_yz_plane(self):
        stock = BoxStock3D(-10.0, 30.0, -15.0, 15.0, -10.0, 0.0)
        tool = ToolAssembly(
            tool_id=1,
            name="6mm Flat",
            geometry=MillToolGeometry(
                diameter=6.0,
                corner_radius=0.0,
                flute_length=20.0,
                overall_length=50.0,
                holder_diameter=30.0,
                holder_length=40.0,
            ),
        )
        v = MillHeightfieldVerifier(cell_size_mm=0.5)
        v.reset(WorkpieceDefinition(stock=stock))
        v.load_tool(tool)

        # Quarter-circle YZ arc at X=10:
        # Center=(10, 0, 0), Start=(10, 5, 0), End=(10, 0, -5).
        # In right-handed (u=Y, v=Z): start=(5,0) angle=0, end=(0,-5) angle=-pi/2 -> CW quarter turn.
        arc_yz = _make_segment(
            1,
            MotionKind.ARC_CW,
            Vector3D(10.0, 5.0, 0.0),
            Vector3D(10.0, 0.0, -5.0),
            is_arc=True,
            arc_center=Vector3D(10.0, 0.0, 0.0),
            arc_plane="YZ",
        )
        res = v.step(arc_yz)
        self.assertGreater(res.material_removed_mm3, 10.0)

        # At arc endpoint (X=10, Y=0) tool bottom reaches Z=-5.0
        c_mid = int(math.floor((10.0 - v.grid.grid_x_min) / v.grid.cell_size_mm))
        r_end = int(math.floor((0.0 - v.grid.grid_y_min) / v.grid.cell_size_mm))
        self.assertAlmostEqual(float(v.grid.H[r_end, c_mid]), -5.0, delta=0.1)

        # Flat r=3mm disk sweeps Y=3.536 down to Z~-4.95, and X=+15 (off YZ plane) stays untouched at 0.0
        r_mid = int(math.floor((3.536 - v.grid.grid_y_min) / v.grid.cell_size_mm))
        self.assertAlmostEqual(float(v.grid.H[r_mid, c_mid]), -4.95, delta=0.15)
        c_off = int(math.floor((15.0 - v.grid.grid_x_min) / v.grid.cell_size_mm))
        self.assertAlmostEqual(float(v.grid.H[r_end, c_off]), 0.0, places=4)


class TestMRRDurationFloor(unittest.TestCase):
    """MRR spike suppression for sub-floor micro-duration moves."""

    def test_micro_duration_floor_and_window_aggregation(self):
        stock = BoxStock3D(-20.0, 60.0, -20.0, 20.0, -20.0, 0.0)
        tool = ToolAssembly(
            tool_id=1,
            name="10mm Flat",
            geometry=MillToolGeometry(
                diameter=10.0,
                corner_radius=0.0,
                flute_length=20.0,
                overall_length=50.0,
                holder_diameter=30.0,
                holder_length=40.0,
            ),
        )
        v = MillHeightfieldVerifier(min_mrr_duration_sec=0.05)
        v.reset(WorkpieceDefinition(stock=stock))
        v.load_tool(tool)

        # Single 0.1 ms move removing ~250 mm³ (without floor -> 1.5e8 mm³/min!)
        tiny_seg = _make_segment(
            1,
            MotionKind.LINEAR,
            Vector3D(0.0, 0.0, -5.0),
            Vector3D(5.0, 0.0, -5.0),
            op_id="OpMicro",
            duration_sec=1e-4,
        )
        res = v.step(tiny_seg)
        expected_max_rate = res.material_removed_mm3 / 0.05 * 60.0
        self.assertLess(res.instantaneous_mrr, 1e6)
        self.assertAlmostEqual(res.instantaneous_mrr, expected_max_rate, places=2)

        # Add normal reference baseline cuts so median is realistic, plus check total time stays uninflated
        v.step(_make_segment(2, MotionKind.LINEAR, Vector3D(-15.0, 10.0, -5.0), Vector3D(5.0, 10.0, -5.0), op_id="OpMicro", duration_sec=2.0))
        v.step(_make_segment(3, MotionKind.LINEAR, Vector3D(-15.0, -10.0, -5.0), Vector3D(5.0, -10.0, -5.0), op_id="OpMicro", duration_sec=2.0))
        report = v.generate_report()
        self.assertAlmostEqual(report.total_machining_time_sec, 4.0001, places=4)
        self.assertEqual(len(report.mrr_spikes), 0)

        # Configurable larger floor lowers rate proportionally
        v2 = MillHeightfieldVerifier(min_mrr_duration_sec=0.2)
        v2.reset(WorkpieceDefinition(stock=stock))
        v2.load_tool(tool)
        res2 = v2.step(tiny_seg)
        self.assertAlmostEqual(res2.instantaneous_mrr, res2.material_removed_mm3 / 0.2 * 60.0, places=2)

    def test_sustained_short_moves_aggregate_into_spike(self):
        stock = BoxStock3D(-20.0, 80.0, -20.0, 20.0, -20.0, 0.0)
        tool = ToolAssembly(
            tool_id=1,
            name="10mm Flat",
            geometry=MillToolGeometry(
                diameter=10.0,
                corner_radius=0.0,
                flute_length=20.0,
                overall_length=50.0,
                holder_diameter=30.0,
                holder_length=40.0,
            ),
        )
        v = MillHeightfieldVerifier(min_mrr_duration_sec=0.05)
        v.reset(WorkpieceDefinition(stock=stock))
        v.load_tool(tool)

        # Baseline gentle cuts in OpBurst
        for idx, x0 in enumerate((-15.0, -5.0, 5.0)):
            v.step(
                _make_segment(
                    idx + 1,
                    MotionKind.LINEAR,
                    Vector3D(x0, -10.0, -1.0),
                    Vector3D(x0 + 5.0, -10.0, -1.0),
                    op_id="OpBurst",
                    duration_sec=1.0,
                )
            )
        # Sustained burst of 10 rapid deep cuts of 0.006s each (total 0.06s >= 0.05s) removing deep material
        for k in range(10):
            x_start = 15.0 + k * 4.0
            v.step(
                _make_segment(
                    10 + k,
                    MotionKind.LINEAR,
                    Vector3D(x_start, 10.0, -15.0),
                    Vector3D(x_start + 4.0, 10.0, -15.0),
                    op_id="OpBurst",
                    duration_sec=0.006,
                )
            )
        report = v.generate_report()
        self.assertGreaterEqual(len(report.mrr_spikes), 1)
        self.assertGreater(report.mrr_spikes[0].mrr_rate_mm3_per_min, 10000.0)


if __name__ == "__main__":
    unittest.main()
