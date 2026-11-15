#!/usr/bin/env python3
"""
Unit and integration tests for Phase 1: From Solid Stock Support.
"""

import json
import math
import os
import sys
import tempfile
import types
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

# Ensure mock adsk module exists if running outside Fusion
if "adsk" not in sys.modules:
    adsk = types.ModuleType("adsk")
    adsk.cam = types.ModuleType("adsk.cam")
    adsk.cam.OperationTypes = types.SimpleNamespace(TurningOperation=1, MillingOperation=0)
    adsk.cam.CAM = types.SimpleNamespace(cast=lambda x: x)
    adsk.core = types.ModuleType("adsk.core")
    adsk.core.CommandCreatedEventHandler = type("CommandCreatedEventHandler", (), {})
    adsk.core.InputChangedEventHandler = type("InputChangedEventHandler", (), {})
    adsk.core.ValidateInputsEventHandler = type("ValidateInputsEventHandler", (), {})
    adsk.core.CommandEventHandler = type("CommandEventHandler", (), {})
    adsk.core.Application = types.SimpleNamespace(get=lambda: types.SimpleNamespace(log=lambda msg: None))
    adsk.fusion = types.ModuleType("adsk.fusion")
    sys.modules["adsk"] = adsk
    sys.modules["adsk.cam"] = adsk.cam
    sys.modules["adsk.core"] = adsk.core
    sys.modules["adsk.fusion"] = adsk.fusion
else:
    adsk = sys.modules["adsk"]
    if not hasattr(adsk.cam, "CAM"):
        adsk.cam.CAM = types.SimpleNamespace(cast=lambda x: x)

if not hasattr(adsk.core, "MessageBoxIconTypes"):
    adsk.core.MessageBoxIconTypes = types.SimpleNamespace(
        InformationIconType=0,
        WarningIconType=1,
        CriticalIconType=2,
        QuestionIconType=3,
    )
if not hasattr(adsk.core, "MessageBoxButtonTypes"):
    adsk.core.MessageBoxButtonTypes = types.SimpleNamespace(
        OKButtonType=0,
        YesNoButtonType=1,
    )
if not hasattr(adsk.core, "DialogResults"):
    adsk.core.DialogResults = types.SimpleNamespace(
        DialogYes=0,
        DialogNo=1,
        DialogOK=2,
    )
if not hasattr(adsk.core, "ObjectCollection"):
    class MockObjectCollection(list):
        @classmethod
        def create(cls):
            return cls()
        def add(self, item):
            self.append(item)
    adsk.core.ObjectCollection = MockObjectCollection

if not hasattr(adsk.cam, "SetupStockModes"):
    adsk.cam.SetupStockModes = types.SimpleNamespace(
        FixedBoxStock=0,
        RelativeBoxStock=1,
        FixedCylinderStock=2,
        RelativeCylinderStock=3,
        FixedTubeStock=4,
        RelativeTubeStock=5,
        SolidStock=6,
        PreviousSetupStock=7,
    )
if not hasattr(adsk.cam, "PostOutputUnitOptions"):
    adsk.cam.PostOutputUnitOptions = types.SimpleNamespace(
        DocumentUnitsOutput=0,
        InchesOutput=1,
        MillimetersOutput=2,
    )
if not hasattr(adsk.fusion, "TriangleMeshQualityOptions"):
    adsk.fusion.TriangleMeshQualityOptions = types.SimpleNamespace(
        LowQualityTriangleMesh=8,
        NormalQualityTriangleMesh=11,
        HighQualityTriangleMesh=13,
        VeryHighQualityTriangleMesh=15,
    )
if not hasattr(adsk.cam, "PostProcessInput"):
    class MockPostProcessInput:
        @classmethod
        def create(cls, programName, postConfiguration, outputFolder, outputUnits):
            inst = cls()
            inst.programName = programName
            inst.postConfiguration = postConfiguration
            inst.outputFolder = outputFolder
            inst.outputUnits = outputUnits
            inst.isOpenInEditor = False
            return inst
    adsk.cam.PostProcessInput = MockPostProcessInput


from geometry.heightfield_engine import HeightfieldGrid, MillHeightfieldVerifier
from geometry.lathe_engine import Lathe2DIPWVerifier
from geometry.models import (
    AxisymmetricStock2D,
    BoxStock3D,
    MillToolGeometry,
    MotionKind,
    MotionSegment,
    SolidMeshStock3D,
    ToolAssembly,
    Vector3D,
    WorkpieceDefinition,
)
from geometry.solid_mesh import (
    extract_mesh_from_fusion_setup,
    extract_solid_workpiece_from_fusion_setup,
    rasterize_triangles_to_grid,
    revolve_mesh_to_lathe_stock,
    transform_design_points_to_setup_wcs,
)
import PostClear


class TestSolidMeshStock(unittest.TestCase):
    """Test suite for 3D solid mesh stock processing and simulation."""

    def test_transform_design_points_to_setup_wcs(self):
        """Verify coordinate frame conversion and cm -> mm scaling."""
        # 1. Identity transform with origin at (10, 20, 30) cm
        mock_wcs = MagicMock()
        mock_origin = MagicMock(x=10.0, y=20.0, z=30.0)
        mock_x = MagicMock(x=1.0, y=0.0, z=0.0)
        mock_y = MagicMock(x=0.0, y=1.0, z=0.0)
        mock_z = MagicMock(x=0.0, y=0.0, z=1.0)
        mock_wcs.getAsCoordinateSystem.return_value = (mock_origin, mock_x, mock_y, mock_z)

        # Points in design space: (10, 20, 30) cm and (15, 22, 33) cm
        pts_design_cm = np.array([
            [10.0, 20.0, 30.0],
            [15.0, 22.0, 33.0],
        ], dtype=np.float64)

        pts_setup_mm = transform_design_points_to_setup_wcs(pts_design_cm, mock_wcs)

        # First point should be at WCS origin: (0, 0, 0) mm
        np.testing.assert_allclose(pts_setup_mm[0], [0.0, 0.0, 0.0], atol=1e-5)
        # Second point: dx=5 cm -> 50 mm, dy=2 cm -> 20 mm, dz=3 cm -> 30 mm
        np.testing.assert_allclose(pts_setup_mm[1], [50.0, 20.0, 30.0], atol=1e-5)

        # 2. 90-degree Z rotation: setup X points along design Y, setup Y points along design -X
        mock_wcs_rot = MagicMock()
        del mock_wcs_rot.copy  # Force strategy 2
        mock_x_rot = MagicMock(x=0.0, y=1.0, z=0.0)
        mock_y_rot = MagicMock(x=-1.0, y=0.0, z=0.0)
        mock_z_rot = MagicMock(x=0.0, y=0.0, z=1.0)
        # Test 5-tuple SWIG return format (success, origin, x, y, z)
        mock_wcs_rot.getAsCoordinateSystem.return_value = (True, mock_origin, mock_x_rot, mock_y_rot, mock_z_rot)

        pts_rot_mm = transform_design_points_to_setup_wcs(pts_design_cm, mock_wcs_rot)
        np.testing.assert_allclose(pts_rot_mm[0], [0.0, 0.0, 0.0], atol=1e-5)
        # (15, 22, 33) - (10, 20, 30) = (5, 2, 3) cm
        # dot with X_rot (0, 1, 0) = 2 cm -> 20 mm
        # dot with Y_rot (-1, 0, 0) = -5 cm -> -50 mm
        # dot with Z_rot (0, 0, 1) = 3 cm -> 30 mm
        np.testing.assert_allclose(pts_rot_mm[1], [20.0, -50.0, 30.0], atol=1e-5)

        # 3. Strategy 1: Matrix3D copy().invert() with getCell (native Fusion 360 C++ Matrix3D)
        class MockMatrix3D:
            def __init__(self, m_cells):
                self.m = np.array(m_cells, dtype=np.float64)
            def copy(self):
                return MockMatrix3D(self.m.copy())
            def invert(self):
                self.m = np.linalg.inv(self.m)
                return True
            def getCell(self, r, c):
                return float(self.m[r, c])

        # Matrix transforming Setup WCS -> Design Space:
        # Rotated 90 deg around Z, translated by (10, 20, 30) cm
        wcs_matrix = MockMatrix3D([
            [0.0, -1.0, 0.0, 10.0],
            [1.0,  0.0, 0.0, 20.0],
            [0.0,  0.0, 1.0, 30.0],
            [0.0,  0.0, 0.0,  1.0],
        ])
        pts_mat_mm = transform_design_points_to_setup_wcs(pts_design_cm, wcs_matrix)
        np.testing.assert_allclose(pts_mat_mm[0], [0.0, 0.0, 0.0], atol=1e-5)
        np.testing.assert_allclose(pts_mat_mm[1], [20.0, -50.0, 30.0], atol=1e-5)

    def test_transform_design_points_to_setup_wcs_native_fusion_cam_mm_origin(self):
        """
        Verify native Fusion 360 CAM WCS matrix where translation is in millimeters (e.g. OilLid).
        Design coordinates (cm): X in [-5.45, 6.895], Y in [-1.58, 1.72], Z in [0.0, 1.27].
        WCS Origin (mm): Top center at [7.224314, 0.721522, 12.7].
        In setup WCS, stock must be centered at X=0, Y=0, with top surface at Z=0 and bottom at Z=-12.7 mm.
        """
        class MockFusionMatrix3D:
            def __init__(self, m_cells):
                self.m = np.array(m_cells, dtype=np.float64)
            def copy(self):
                return MockFusionMatrix3D(self.m.copy())
            def invert(self):
                self.m = np.linalg.inv(self.m)
                return True
            def getCell(self, r, c):
                return float(self.m[r, c])

        # Setup WCS -> Design Space matrix with mm translation
        cam_wcs = MockFusionMatrix3D([
            [1.0, 0.0, 0.0,  7.22431426],
            [0.0, 1.0, 0.0,  0.72152152],
            [0.0, 0.0, 1.0, 12.70000000],
            [0.0, 0.0, 0.0,  1.0],
        ])

        # Design points in centimeters (cm)
        pts_design_cm = np.array([
            [-5.45021582, -1.57867205, 0.0],
            [ 6.89507866,  1.72297633, 1.27],
        ], dtype=np.float64)

        pts_wcs_mm = transform_design_points_to_setup_wcs(pts_design_cm, cam_wcs)

        # In mm:
        # X: [-54.5021582 - 7.22431426, 68.9507866 - 7.22431426] = [-61.72647, +61.72647] (symmetric!)
        np.testing.assert_allclose(pts_wcs_mm[0, 0], -61.72647, atol=1e-3)
        np.testing.assert_allclose(pts_wcs_mm[1, 0],  61.72647, atol=1e-3)

        # Y: [-15.78672 - 0.72152, 17.22976 - 0.72152] = [-16.50824, +16.50824] (symmetric!)
        np.testing.assert_allclose(pts_wcs_mm[0, 1], -16.50824, atol=1e-3)
        np.testing.assert_allclose(pts_wcs_mm[1, 1],  16.50824, atol=1e-3)

        # Z: [0.0 - 12.7, 12.7 - 12.7] = [-12.7, 0.0] mm
        np.testing.assert_allclose(pts_wcs_mm[0, 2], -12.7, atol=1e-3)
        np.testing.assert_allclose(pts_wcs_mm[1, 2],   0.0, atol=1e-3)

    def test_extract_mesh_from_fusion_setup_with_occurrence(self):
        """Verify extraction from an Occurrence containing child BRepBody."""
        mock_setup = MagicMock()
        mock_setup.name = "OccurrenceSetup"
        mock_setup.workCoordinateSystem = None

        mock_body = MagicMock()
        mock_calc = MagicMock()
        mock_mesh = MagicMock()
        mock_mesh.nodeCoordinatesAsDouble = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        mock_mesh.nodeIndices = [0, 1, 2]
        mock_calc.calculate.return_value = mock_mesh
        mock_body.meshManager.createMeshCalculator.return_value = mock_calc

        mock_occ = MagicMock(spec=["bRepBodies", "meshBodies", "childOccurrences"])
        mock_occ.bRepBodies.count = 1
        mock_occ.bRepBodies.item.return_value = mock_body
        mock_occ.meshBodies.count = 0
        mock_occ.childOccurrences.count = 0

        mock_setup.stockSolids = [mock_occ]
        v_mm, idx = extract_mesh_from_fusion_setup(mock_setup)
        self.assertEqual(len(v_mm), 3)
        np.testing.assert_allclose(v_mm[1], [10.0, 0.0, 0.0])

    def test_solid_mesh_stock_3d_model(self):
        """Verify SolidMeshStock3D bounds and initialization."""
        verts = np.array([
            [-10.0, -20.0, -5.0],
            [30.0, 40.0, 15.0],
            [10.0, 10.0, 0.0],
        ], dtype=np.float32)
        idx = np.array([[0, 1, 2]], dtype=np.int32)

        stock = SolidMeshStock3D.from_mesh(verts, idx)
        self.assertEqual(stock.x_min, -10.0)
        self.assertEqual(stock.x_max, 30.0)
        self.assertEqual(stock.y_min, -20.0)
        self.assertEqual(stock.y_max, 40.0)
        self.assertEqual(stock.z_min, -5.0)
        self.assertEqual(stock.z_max, 15.0)

        with self.assertRaises(ValueError):
            SolidMeshStock3D.from_mesh(np.empty((0, 3)), np.empty((0, 3)))

    def test_rasterize_triangles_to_heightfield_flat_box(self):
        """Verify rasterizing a flat rectangular top surface into HeightfieldGrid."""
        # 100 x 50 mm box from X=[0, 100], Y=[0, 50], Z=[-20, 10]
        # Top surface at Z=10 made of 2 triangles
        verts = np.array([
            [0.0, 0.0, 10.0],
            [100.0, 0.0, 10.0],
            [100.0, 50.0, 10.0],
            [0.0, 50.0, 10.0],
            [0.0, 0.0, -20.0],
            [100.0, 50.0, -20.0],
        ], dtype=np.float32)
        tris = np.array([
            [0, 1, 2],  # Top triangle 1
            [0, 2, 3],  # Top triangle 2
        ], dtype=np.int32)

        grid = HeightfieldGrid(cell_size_mm=1.0, pad_mm=5.0)
        grid.reset_from_mesh(verts, tris, tol_pad_z_mm=0.0)

        # Outside the [0, 100] x [0, 50] stock box (e.g. at grid padding), height should be z_floor (-20.0)
        pad_col = 1
        pad_row = 1
        self.assertAlmostEqual(grid.H[pad_row, pad_col], -20.0, places=2)

        # Inside the stock box (e.g. X=50, Y=25), height should be 10.0
        c_mid = int((50.0 - grid.grid_x_min) / grid.cell_size_mm)
        r_mid = int((25.0 - grid.grid_y_min) / grid.cell_size_mm)
        self.assertAlmostEqual(grid.H[r_mid, c_mid], 10.0, places=2)

    def test_rasterize_triangles_to_heightfield_sloped_wedge(self):
        """Verify rasterizing an inclined wedge correctly computes continuous plane heights."""
        # Wedge from X=[0, 40], Y=[0, 20], sloping from Z=0 at X=0 to Z=20 at X=40
        verts = np.array([
            [0.0, 0.0, 0.0],
            [40.0, 0.0, 20.0],
            [40.0, 20.0, 20.0],
            [0.0, 20.0, 0.0],
        ], dtype=np.float32)
        tris = np.array([
            [0, 1, 2],
            [0, 2, 3],
        ], dtype=np.int32)

        grid = HeightfieldGrid(cell_size_mm=1.0, pad_mm=5.0)
        grid.reset_from_mesh(verts, tris, tol_pad_z_mm=0.0)

        # Check height at X=10 and X=30 against exact cell center along the sloped plane Z = 0.5 * X
        c_10 = int((10.0 - grid.grid_x_min) / grid.cell_size_mm)
        r_10 = int((10.0 - grid.grid_y_min) / grid.cell_size_mm)
        x_c10 = grid.grid_x_min + (c_10 + 0.5) * grid.cell_size_mm
        self.assertLess(abs(x_c10 - 10.0), grid.cell_size_mm)
        self.assertAlmostEqual(grid.H[r_10, c_10], 0.5 * x_c10, places=2)

        c_30 = int((30.0 - grid.grid_x_min) / grid.cell_size_mm)
        r_30 = int((10.0 - grid.grid_y_min) / grid.cell_size_mm)
        x_c30 = grid.grid_x_min + (c_30 + 0.5) * grid.cell_size_mm
        self.assertLess(abs(x_c30 - 30.0), grid.cell_size_mm)
        self.assertAlmostEqual(grid.H[r_30, c_30], 0.5 * x_c30, places=2)

    def test_mill_heightfield_verifier_with_solid_stock(self):
        """Verify MillHeightfieldVerifier executes toolpath verification on SolidMeshStock3D."""
        verts = np.array([
            [0.0, 0.0, 10.0],
            [60.0, 0.0, 10.0],
            [60.0, 40.0, 10.0],
            [0.0, 40.0, 10.0],
            [0.0, 0.0, -10.0],
            [60.0, 40.0, -10.0],
        ], dtype=np.float32)
        tris = np.array([
            [0, 1, 2],
            [0, 2, 3],
        ], dtype=np.int32)

        stock = SolidMeshStock3D.from_mesh(verts, tris)
        wp = WorkpieceDefinition(stock=stock)

        verifier = MillHeightfieldVerifier(cell_size_mm=1.0, pad_mm=5.0)
        verifier.reset(wp)
        self.assertTrue(verifier.has_valid_stock)

        # Mount 10 mm endmill
        tool = ToolAssembly(
            tool_id=1,
            name="10mm Flat Endmill",
            geometry=MillToolGeometry(
                diameter=10.0,
                corner_radius=0.0,
                flute_length=30.0,
                overall_length=50.0,
            ),
        )
        verifier.load_tool(tool)

        # Linear cutting move at Z=5 (cutting 5 mm into the 10 mm top stock)
        start_pt = Vector3D(10.0, 20.0, 5.0)
        end_pt = Vector3D(50.0, 20.0, 5.0)
        seg = MotionSegment(
            segment_id=1,
            op_id="op1",
            sec_id=1,
            motion=MotionKind.LINEAR,
            start=start_pt,
            end=end_pt,
            feedrate_mmpm=1000.0,
            spindle_rpm=5000.0,
            duration_sec=2.4,
        )
        eval_res = verifier.step(seg)
        self.assertFalse(eval_res.is_air_move)
        self.assertGreater(eval_res.material_removed_mm3, 0.0)

        # Verify stock height was carved to Z=5
        c_mid = int((30.0 - verifier.grid.grid_x_min) / verifier.grid.cell_size_mm)
        r_mid = int((20.0 - verifier.grid.grid_y_min) / verifier.grid.cell_size_mm)
        self.assertAlmostEqual(verifier.grid.H[r_mid, c_mid], 5.0, delta=0.2)

    def test_revolve_mesh_to_lathe_stock_centered(self):
        """Verify revolving a centered cylinder mesh produces a valid AxisymmetricStock2D."""
        # Generate 16-sided cylinder of radius 20 mm, length 60 mm (Z=-50 to Z=10) centered on Z
        theta = np.linspace(0, 2 * np.pi, 16, endpoint=False)
        x_ring = 20.0 * np.cos(theta)
        y_ring = 20.0 * np.sin(theta)

        verts_bottom = np.column_stack([x_ring, y_ring, np.full(16, -50.0)])
        verts_top = np.column_stack([x_ring, y_ring, np.full(16, 10.0)])
        verts = np.vstack([verts_bottom, verts_top])

        # Generate side quads as triangles
        tris = []
        for i in range(16):
            nxt = (i + 1) % 16
            # bottom i, top i, top nxt
            tris.append([i, i + 16, nxt + 16])
            # bottom i, top nxt, bottom nxt
            tris.append([i, nxt + 16, nxt])
        tris = np.array(tris, dtype=np.int32)

        wp = revolve_mesh_to_lathe_stock(verts, tris, tol_pad_r_mm=0.0)
        self.assertIsInstance(wp.stock, AxisymmetricStock2D)
        self.assertAlmostEqual(wp.stock.z_min, -50.0, places=2)
        self.assertAlmostEqual(wp.stock.z_max, 10.0, places=2)
        self.assertAlmostEqual(wp.stock.outer_radius, 20.0, places=1)
        self.assertIsNotNone(wp.stock.polygon)
        self.assertFalse(wp.stock.polygon.is_empty)

        # Verify with Lathe2DIPWVerifier.reset
        lathe_verifier = Lathe2DIPWVerifier()
        lathe_verifier.reset(wp)
        self.assertFalse(lathe_verifier.ipw.is_empty)
        # Bounding box of IPW: min Z=-50, max Z=10, max R=20
        min_z, min_r, max_z, max_r = lathe_verifier.ipw.bounds
        self.assertAlmostEqual(min_z, -50.0, places=2)
        self.assertAlmostEqual(max_z, 10.0, places=2)
        self.assertAlmostEqual(max_r, 20.0, places=1)

    def test_revolve_mesh_to_lathe_stock_off_center_rejected(self):
        """Verify revolving an off-center solid stock is rejected with an explicit error."""
        # Cylinder offset in X by 10.0 mm
        theta = np.linspace(0, 2 * np.pi, 8, endpoint=False)
        x_ring = 10.0 + 20.0 * np.cos(theta)
        y_ring = 20.0 * np.sin(theta)
        verts = np.vstack([
            np.column_stack([x_ring, y_ring, np.full(8, -50.0)]),
            np.column_stack([x_ring, y_ring, np.full(8, 10.0)]),
        ])
        tris = np.array([[0, 1, 8], [1, 9, 8]], dtype=np.int32)

        with self.assertRaises(ValueError) as err_ctx:
            revolve_mesh_to_lathe_stock(verts, tris)
        self.assertIn("not centered on the turning spindle axis", str(err_ctx.exception))

    def test_fusion_setup_solid_stock_extraction_and_pipeline(self):
        """Verify full pipeline integration extracting mock solid stock from Setup."""
        mock_setup = MagicMock()
        mock_setup.name = "Op1SolidMilling"
        mock_setup.stockMode = adsk.cam.SetupStockModes.SolidStock
        mock_setup.operationType = 0  # Milling

        # Mock WCS: identity matrix
        mock_wcs = MagicMock()
        mock_origin = MagicMock(x=0.0, y=0.0, z=0.0)
        mock_x = MagicMock(x=1.0, y=0.0, z=0.0)
        mock_y = MagicMock(x=0.0, y=1.0, z=0.0)
        mock_z = MagicMock(x=0.0, y=0.0, z=1.0)
        mock_wcs.getAsCoordinateSystem.return_value = (mock_origin, mock_x, mock_y, mock_z)
        mock_setup.workCoordinateSystem = mock_wcs

        # Mock BRepBody with meshManager
        mock_body = MagicMock()
        mock_calc = MagicMock()
        mock_mesh = MagicMock()
        # Flat surface 50 x 30 mm, Z=10 mm (in cm: 5.0 x 3.0 x 1.0)
        mock_mesh.nodeCoordinatesAsDouble = [
            0.0, 0.0, 1.0,
            5.0, 0.0, 1.0,
            5.0, 3.0, 1.0,
            0.0, 3.0, 1.0,
        ]
        mock_mesh.nodeIndices = [0, 1, 2, 0, 2, 3]
        mock_calc.calculate.return_value = mock_mesh
        mock_body.meshManager.createMeshCalculator.return_value = mock_calc

        mock_setup.stockSolids = [mock_body]

        # 1. extract_mesh_from_fusion_setup returns vertices in mm
        v_mm, idx = extract_mesh_from_fusion_setup(mock_setup)
        self.assertEqual(len(v_mm), 4)
        np.testing.assert_allclose(v_mm[1], [50.0, 0.0, 10.0])

        # 2. extract_solid_workpiece_from_fusion_setup returns SolidMeshStock3D
        wp = extract_solid_workpiece_from_fusion_setup(mock_setup, is_turning=False)
        self.assertIsInstance(wp.stock, SolidMeshStock3D)
        self.assertEqual(wp.stock.x_max, 50.0)
        self.assertEqual(wp.stock.y_max, 30.0)
        self.assertEqual(wp.stock.z_max, 10.0)

        # 3. PostProcessUnifiedPipeline executes with solid stock
        import json
        with tempfile.TemporaryDirectory() as tmp_dir:
            dummy_cps = os.path.join(tmp_dir, "dummy.cps")
            with open(dummy_cps, "w", encoding="utf-8") as f:
                f.write('description = "Dummy Post";\ncapabilities = CAPABILITY_MILLING;\n')

            mock_prog = MagicMock()
            mock_prog.parameters.itemByName.return_value = MagicMock(value=MagicMock(value="tormach"))

            mock_op = MagicMock()
            mock_op.isSuppressed = False
            mock_op.hasToolpath = True
            p_tool = MagicMock()
            p_tool.value.value = 1
            mock_op.tool.parameters.itemByName.return_value = p_tool
            mock_setup.allOperations = [mock_op]

            def fake_post_process(target_ops, post_input):
                out_json = os.path.join(post_input.outputFolder, f"{post_input.programName}.json")
                with open(out_json, "w", encoding="utf-8") as f:
                    json.dump({"unit": 1, "sections": []}, f)
                return True

            mock_cam = MagicMock()
            mock_cam.postProcess.side_effect = fake_post_process

            mock_app = MagicMock()
            mock_app.activeDocument.products.itemByProductType.return_value = mock_cam

            # Mock pipeline orchestrator run
            mock_report = MagicMock()
            mock_report.collisions = []
            mock_report.promoted_count = 0
            mock_report.spike_count = 0
            mock_report.peak_mrr = 0.0
            mock_report.total_machining_time_sec = 10.0
            mock_report.total_volume_removed_mm3 = 500.0
            mock_report.promoted_events = set()
            mock_report.mrr_spikes = []
            mock_report.summary_text = "Verification Success"

            with patch("PostClear.adsk.core.Application.get", return_value=mock_app), \
                 patch("PostClear._ResolveBasePostPath", return_value=dummy_cps), \
                 patch("pipeline.PipelineOrchestrator.run", return_value=(mock_report, "G0 X0 Y0")):
                res = PostClear._PostProcessUnifiedPipeline(
                    fname="Op1SolidMilling",
                    setup=mock_setup,
                    docSettings={},
                    program=mock_prog,
                    path=os.path.join(tmp_dir, "solid_milling.nc"),
                    fileExt=".nc",
                )
                self.assertIsNone(res)  # Clean success returns None (no error)


class TestPreviousSetupStockSupport(unittest.TestCase):
    """Unit tests for Phase 2: From Previous Setup Stock via exported STL mesh."""

    def setUp(self):
        self.tmp_dir = tempfile.mkdtemp(prefix="stl_stock_test_")
        # Define a simple 3D box mesh (2 triangles on top face, etc.)
        # Box from (-10, -10, 0) to (10, 10, 20)
        self.triangles = [
            # Top face (Z=20)
            ((0.0, 0.0, 1.0), (-10.0, -10.0, 20.0), (10.0, -10.0, 20.0), (10.0, 10.0, 20.0)),
            ((0.0, 0.0, 1.0), (-10.0, -10.0, 20.0), (10.0, 10.0, 20.0), (-10.0, 10.0, 20.0)),
            # Bottom face (Z=0)
            ((0.0, 0.0, -1.0), (-10.0, -10.0, 0.0), (10.0, 10.0, 0.0), (10.0, -10.0, 0.0)),
            ((0.0, 0.0, -1.0), (-10.0, -10.0, 0.0), (-10.0, 10.0, 0.0), (10.0, 10.0, 0.0)),
        ]
        self.bin_stl = os.path.join(self.tmp_dir, "box_bin.stl")
        self._write_binary_stl(self.bin_stl, self.triangles)

        self.ascii_stl = os.path.join(self.tmp_dir, "box_ascii.stl")
        self._write_ascii_stl(self.ascii_stl, self.triangles)

    def tearDown(self):
        import shutil
        if os.path.isdir(self.tmp_dir):
            shutil.rmtree(self.tmp_dir, ignore_errors=True)

    def _write_binary_stl(self, filepath: str, triangles):
        import struct
        with open(filepath, "wb") as f:
            f.write(b"Binary STL Test Header" + b"\x00" * 58)  # 80 bytes
            f.write(struct.pack("<I", len(triangles)))
            for norm, v1, v2, v3 in triangles:
                f.write(struct.pack("<3f", *norm))
                f.write(struct.pack("<3f", *v1))
                f.write(struct.pack("<3f", *v2))
                f.write(struct.pack("<3f", *v3))
                f.write(struct.pack("<H", 0))

    def _write_ascii_stl(self, filepath: str, triangles):
        with open(filepath, "w", encoding="utf-8") as f:
            f.write("solid box\n")
            for norm, v1, v2, v3 in triangles:
                f.write(f"  facet normal {norm[0]} {norm[1]} {norm[2]}\n")
                f.write("    outer loop\n")
                f.write(f"      vertex {v1[0]} {v1[1]} {v1[2]}\n")
                f.write(f"      vertex {v2[0]} {v2[1]} {v2[2]}\n")
                f.write(f"      vertex {v3[0]} {v3[1]} {v3[2]}\n")
                f.write("    endloop\n")
                f.write("  endfacet\n")
            f.write("endsolid box\n")

    def test_load_binary_stl(self):
        from geometry.solid_mesh import load_stl_mesh
        verts, indices = load_stl_mesh(self.bin_stl, is_inch=False)
        self.assertEqual(len(indices), 4)
        self.assertEqual(len(verts), 12)
        self.assertAlmostEqual(float(verts[:, 2].min()), 0.0, places=4)
        self.assertAlmostEqual(float(verts[:, 2].max()), 20.0, places=4)

    def test_load_binary_stl_with_inch_scaling(self):
        from geometry.solid_mesh import load_stl_mesh
        verts, indices = load_stl_mesh(self.bin_stl, is_inch=True)
        self.assertEqual(len(indices), 4)
        self.assertAlmostEqual(float(verts[:, 2].max()), 20.0 * 25.4, places=2)

    def test_load_ascii_stl(self):
        from geometry.solid_mesh import load_stl_mesh
        verts, indices = load_stl_mesh(self.ascii_stl, is_inch=False)
        self.assertEqual(len(indices), 4)
        self.assertEqual(len(verts), 12)
        self.assertAlmostEqual(float(verts[:, 2].min()), 0.0, places=4)
        self.assertAlmostEqual(float(verts[:, 2].max()), 20.0, places=4)

    def test_extract_workpiece_from_stl_milling(self):
        from geometry.solid_mesh import extract_workpiece_from_stl
        from geometry.models import SolidMeshStock3D
        wp = extract_workpiece_from_stl(self.bin_stl, is_turning=False, is_inch=False)
        self.assertIsInstance(wp.stock, SolidMeshStock3D)
        self.assertAlmostEqual(wp.stock.x_min, -10.0, places=2)
        self.assertAlmostEqual(wp.stock.x_max, 10.0, places=2)
        self.assertAlmostEqual(wp.stock.z_max, 20.0, places=2)

    def test_extract_workpiece_from_stl_turning(self):
        from geometry.solid_mesh import extract_workpiece_from_stl
        from geometry.models import AxisymmetricStock2D
        wp = extract_workpiece_from_stl(self.bin_stl, is_turning=True, is_inch=False)
        self.assertIsInstance(wp.stock, AxisymmetricStock2D)
        self.assertAlmostEqual(wp.stock.z_min, 0.0, places=1)
        self.assertAlmostEqual(wp.stock.z_max, 20.0, places=1)
        # Bounding box hypot(-10, -10) = 14.14 mm + 0.05 pad
        self.assertGreater(wp.stock.outer_radius, 14.0)

    def test_context_parser_extracts_workpiece_from_stock_path(self):
        from context_parser import extract_workpiece_from_context
        from geometry.models import SolidMeshStock3D
        ctx = {
            "unit": 1,
            "stockMode": "previous-setup",
            "stockPath": self.bin_stl,
            "sections": [{"type": 0, "tool": {"number": 1}}],
        }
        wp = extract_workpiece_from_context(ctx, fallback_workpiece=None, is_turning=False)
        self.assertIsNotNone(wp)
        self.assertIsInstance(wp.stock, SolidMeshStock3D)

    def test_pipeline_raises_value_error_if_previous_stock_has_no_mesh(self):
        from pipeline import PipelineOrchestrator
        orchestrator = PipelineOrchestrator(
            dumper_cps="universal_dumper.cps",
            replayer_cps="universal_replayer.cps",
        )
        fake_dump = {
            "unit": 1,
            "stockMode": "previous-setup",
            "sections": [],
        }
        json_path = os.path.join(self.tmp_dir, "batch_prev_no_mesh.json")
        dummy_cps = os.path.join(self.tmp_dir, "base_dummy.cps")
        dummy_cnc = os.path.join(self.tmp_dir, "base_dummy.cnc")
        with open(dummy_cps, "w", encoding="utf-8") as f:
            f.write('description = "Dummy"; capabilities = CAPABILITY_MILLING;\n')
        with open(dummy_cnc, "w", encoding="utf-8") as f:
            f.write('(bootstrap cnc)\n')

        import json
        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(fake_dump, f)

        with self.assertRaises(ValueError) as cm:
            orchestrator.run(
                context_dump_paths=[json_path],
                base_post_path=dummy_cps,
                bootstrap_cnc_path=dummy_cnc,
                final_cnc_path=os.path.join(self.tmp_dir, "out.nc"),
                promote=False,
            )
        self.assertIn("uses 'From Previous Setup' stock", str(cm.exception))

    def test_postclear_unified_pipeline_previous_setup_proceeds_to_pass_1(self):
        import PostClear
        import json
        mock_setup = MagicMock()
        mock_setup.name = "Op2PrecedingSetup"
        p_mode = MagicMock()
        p_mode.value.value = 7  # PreviousSetupStock
        p_mode.value = 7
        mock_setup.parameters.itemByName.return_value = p_mode

        dummy_cps = os.path.join(self.tmp_dir, "dummy.cps")
        with open(dummy_cps, "w", encoding="utf-8") as f:
            f.write('description = "Dummy Post";\ncapabilities = CAPABILITY_MILLING;\n')

        mock_prog = MagicMock()
        mock_prog.parameters.itemByName.return_value = MagicMock(value=MagicMock(value="tormach"))

        mock_op = MagicMock()
        mock_op.isSuppressed = False
        mock_op.hasToolpath = True
        p_tool = MagicMock()
        p_tool.value.value = 1
        mock_op.tool.parameters.itemByName.return_value = p_tool
        mock_setup.allOperations = [mock_op]

        def fake_post_process(target_ops, post_input):
            out_json = os.path.join(post_input.outputFolder, f"{post_input.programName}.json")
            with open(out_json, "w", encoding="utf-8") as f:
                json.dump({
                    "unit": 1,
                    "stockMode": "previous-setup",
                    "stockPath": self.bin_stl,
                    "sections": []
                }, f)
            return True

        mock_cam = MagicMock()
        mock_cam.postProcess.side_effect = fake_post_process

        mock_app = MagicMock()
        mock_app.activeDocument.products.itemByProductType.return_value = mock_cam

        mock_report = MagicMock()
        mock_report.collisions = []
        mock_report.promoted_count = 0
        mock_report.spike_count = 0
        mock_report.peak_mrr = 0.0
        mock_report.total_machining_time_sec = 10.0
        mock_report.total_volume_removed_mm3 = 500.0
        mock_report.promoted_events = set()
        mock_report.mrr_spikes = []
        mock_report.summary_text = "Verification Success"

        with patch("PostClear.adsk.core.Application.get", return_value=mock_app), \
             patch("PostClear._ResolveBasePostPath", return_value=dummy_cps), \
             patch("pipeline.PipelineOrchestrator.run", return_value=(mock_report, "G0 X0 Y0")):
            res = PostClear._PostProcessUnifiedPipeline(
                fname="Op2PrecedingSetup",
                setup=mock_setup,
                docSettings={},
                program=mock_prog,
                path=os.path.join(self.tmp_dir, "prev_stock.nc"),
                fileExt=".nc",
            )
            # Returns None on success (does not reject early)
            self.assertIsNone(res)

    def test_context_parser_missing_stock_path_file_raises_filenotfound(self):
        from context_parser import extract_workpiece_from_context
        ctx = {
            "unit": 1,
            "stockMode": "previous-setup",
            "stockPath": os.path.join(self.tmp_dir, "non_existent_stock.stl"),
            "sections": [],
        }
        with self.assertRaises(FileNotFoundError) as cm:
            extract_workpiece_from_context(ctx, fallback_workpiece=None, is_turning=False)
        self.assertIn("Exported stock STL mesh file not found", str(cm.exception))

    def test_context_parser_corrupt_stock_stl_raises_runtime_error(self):
        from context_parser import extract_workpiece_from_context
        corrupt_stl = os.path.join(self.tmp_dir, "corrupt.stl")
        with open(corrupt_stl, "wb") as f:
            f.write(b"NOT A VALID STL FILE" * 10)
        ctx = {
            "unit": 1,
            "stockMode": "previous-setup",
            "stockPath": corrupt_stl,
            "sections": [],
        }
        with self.assertRaises(RuntimeError) as cm:
            extract_workpiece_from_context(ctx, fallback_workpiece=None, is_turning=False)
        self.assertIn("Failed to load stock from STL mesh", str(cm.exception))

    def test_load_binary_stl_with_trailing_padding_bytes(self):
        from geometry.solid_mesh import load_stl_mesh
        trail_stl = os.path.join(self.tmp_dir, "box_trail.stl")
        self._write_binary_stl(trail_stl, self.triangles)
        with open(trail_stl, "ab") as f:
            f.write(b"\x00\x00")
        verts, indices = load_stl_mesh(trail_stl, is_inch=False)
        self.assertEqual(len(indices), 4)
        self.assertEqual(len(verts), 12)
        self.assertAlmostEqual(float(verts[:, 2].min()), 0.0, places=4)
        self.assertAlmostEqual(float(verts[:, 2].max()), 20.0, places=4)

    def test_load_binary_stl_rejects_nan_vertices(self):
        from geometry.solid_mesh import load_stl_mesh
        nan_triangles = [
            ((0.0, 0.0, 1.0), (-10.0, -10.0, 20.0), (10.0, -10.0, 20.0), (float("nan"), 10.0, 20.0)),
        ]
        nan_stl = os.path.join(self.tmp_dir, "nan.stl")
        self._write_binary_stl(nan_stl, nan_triangles)
        with self.assertRaises(ValueError) as cm:
            load_stl_mesh(nan_stl, is_inch=False)
        self.assertIn("non-finite", str(cm.exception))

    def test_load_binary_stl_rejects_inf_vertices(self):
        from geometry.solid_mesh import load_stl_mesh
        inf_triangles = [
            ((0.0, 0.0, 1.0), (-10.0, -10.0, 20.0), (float("inf"), -10.0, 20.0), (10.0, 10.0, 20.0)),
        ]
        inf_stl = os.path.join(self.tmp_dir, "inf.stl")
        self._write_binary_stl(inf_stl, inf_triangles)
        with self.assertRaises(ValueError) as cm:
            load_stl_mesh(inf_stl, is_inch=False)
        self.assertIn("non-finite", str(cm.exception))

    def test_load_ascii_stl_rejects_nan_vertices(self):
        from geometry.solid_mesh import load_stl_mesh
        nan_ascii = os.path.join(self.tmp_dir, "nan_ascii.stl")
        with open(nan_ascii, "w", encoding="utf-8") as f:
            f.write("solid nan_box\n")
            f.write("  facet normal 0 0 1\n")
            f.write("    outer loop\n")
            f.write("      vertex nan -10.0 20.0\n")
            f.write("      vertex 10.0 -10.0 20.0\n")
            f.write("      vertex 10.0 10.0 20.0\n")
            f.write("    endloop\n")
            f.write("  endfacet\n")
            f.write("endsolid nan_box\n")
        with self.assertRaises(ValueError) as cm:
            load_stl_mesh(nan_ascii, is_inch=False)
        self.assertIn("non-finite", str(cm.exception))

    def test_context_parser_fixedbox_mode_with_stock_path_returns_box_stock(self):
        from context_parser import extract_workpiece_from_context
        from geometry.models import BoxStock3D
        ctx = {
            "unit": 1,
            "stockMode": "fixedbox",
            "stockPath": self.bin_stl,
            "workpiece": {
                "lower": {"x": -20.0, "y": -20.0, "z": 0.0},
                "upper": {"x": 20.0, "y": 20.0, "z": 40.0},
            },
            "sections": [],
        }
        wp = extract_workpiece_from_context(ctx, fallback_workpiece=None, is_turning=False)
        self.assertIsNotNone(wp)
        self.assertIsInstance(wp.stock, BoxStock3D)
        self.assertAlmostEqual(wp.stock.x_min, -20.0)
        self.assertAlmostEqual(wp.stock.x_max, 20.0)
        self.assertAlmostEqual(wp.stock.z_max, 40.0)

    def test_merge_context_dumps_takes_stock_path_only_from_batch_0(self):
        from context_parser import merge_context_dumps
        dump0 = {
            "sections": [],
            "globalParams": {},
            "stockPath": "",
        }
        dump1 = {
            "sections": [],
            "globalParams": {},
            "stockPath": "/path/to/batch1_stock.stl",
        }
        merged = merge_context_dumps([dump0, dump1])
        self.assertEqual(merged.get("stockPath"), "")

        # When batch 0 has a stockPath, it is preserved
        dump0_with_path = {
            "sections": [],
            "globalParams": {},
            "stockPath": "/path/to/batch0_stock.stl",
        }
        merged2 = merge_context_dumps([dump0_with_path, dump1])
        self.assertEqual(merged2.get("stockPath"), "/path/to/batch0_stock.stl")

    def test_merge_context_dumps_propagates_stock_export_from_batch_0(self):
        from context_parser import merge_context_dumps
        se0 = {
            "enabled": True,
            "hasParam": False,
            "srcPath": "",
            "srcExists": False,
            "destPath": "",
            "error": "custom error b0",
        }
        se1 = {
            "enabled": False,
            "hasParam": True,
            "srcPath": "/b1",
            "srcExists": True,
            "destPath": "/b1_dest",
            "error": "",
        }
        dump0 = {"sections": [], "globalParams": {}, "stockExport": se0}
        dump1 = {"sections": [], "globalParams": {}, "stockExport": se1}
        merged = merge_context_dumps([dump0, dump1])
        self.assertEqual(merged.get("stockExport"), se0)

    def test_pipeline_previous_mode_value_error_includes_stock_export_diagnostic(self):
        from pipeline import PipelineOrchestrator
        orchestrator = PipelineOrchestrator(
            dumper_cps="universal_dumper.cps",
            replayer_cps="universal_replayer.cps",
        )
        dummy_cps = os.path.join(self.tmp_dir, "dummy_diag.cps")
        with open(dummy_cps, "w", encoding="utf-8") as f:
            f.write('description = "Dummy Post";\ncapabilities = CAPABILITY_MILLING;\n')
        dummy_cnc = os.path.join(self.tmp_dir, "dummy_diag.cnc")
        with open(dummy_cnc, "w", encoding="utf-8") as f:
            f.write("N10 G00 X0 Y0 Z10\n")

        # 1. With stockExport populated
        se = {
            "enabled": True,
            "hasParam": False,
            "srcPath": "",
            "srcExists": False,
            "destPath": "",
            "error": "simulated global parameter missing",
        }
        fake_dump = {
            "programName": "TestOpDiag",
            "stockMode": "previous-setup",
            "unit": 1,
            "sections": [],
            "stockExport": se,
        }
        json_path = os.path.join(self.tmp_dir, "ctx_diag.json")
        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(fake_dump, f)

        with self.assertRaises(ValueError) as cm:
            orchestrator.run(
                context_dump_paths=[json_path],
                base_post_path=dummy_cps,
                bootstrap_cnc_path=dummy_cnc,
                final_cnc_path=os.path.join(self.tmp_dir, "out_diag.nc"),
                promote=False,
            )
        err_msg = str(cm.exception)
        self.assertIn("uses 'From Previous Setup' stock", err_msg)
        self.assertIn("stockExport: enabled=True, hasParam=False", err_msg)
        self.assertIn("error='simulated global parameter missing'", err_msg)

        # 2. Without stockExport
        fake_dump_no_se = {
            "programName": "TestOpDiag2",
            "stockMode": "previous-setup",
            "unit": 1,
            "sections": [],
        }
        json_path2 = os.path.join(self.tmp_dir, "ctx_diag2.json")
        with open(json_path2, "w", encoding="utf-8") as f:
            json.dump(fake_dump_no_se, f)

        with self.assertRaises(ValueError) as cm2:
            orchestrator.run(
                context_dump_paths=[json_path2],
                base_post_path=dummy_cps,
                bootstrap_cnc_path=dummy_cnc,
                final_cnc_path=os.path.join(self.tmp_dir, "out_diag2.nc"),
                promote=False,
            )
        err_msg2 = str(cm2.exception)
        self.assertIn("uses 'From Previous Setup' stock", err_msg2)
        self.assertIn("[stockExport: none]", err_msg2)

    def test_universal_dumper_export_stock_gated_by_stock_mode(self):
        from pipeline import PipelineOrchestrator
        orch = PipelineOrchestrator()
        if not orch.post_bin:
            raise unittest.SkipTest("Autodesk post binary not available")
        cnc = "tests/fixtures/roughing_test.cnc"
        cps = "centroid_turning.cps"
        if not os.path.isfile(cnc) or not os.path.isfile(cps):
            raise unittest.SkipTest("Required fixture or vendor post not found")

        cfg_file = os.path.join(self.tmp_dir, "universal_dumper_config.json")
        old_env = os.environ.get("DUMPER_CONFIG_PATH")
        try:
            os.environ["DUMPER_CONFIG_PATH"] = cfg_file

            # 1. previous mode enables stockExport
            with open(cfg_file, "w", encoding="utf-8") as f:
                json.dump({"stockMode": "previous"}, f)
            out1 = os.path.join(self.tmp_dir, "dump_prev.json")
            orch.dump_intermediate_to_json(cnc, cps, out1)
            with open(out1, "r", encoding="utf-8") as f:
                ctx1 = json.load(f)
            self.assertIn("stockExport", ctx1)
            self.assertTrue(ctx1["stockExport"]["enabled"])

            # 2. non-previous mode disables stockExport
            with open(cfg_file, "w", encoding="utf-8") as f:
                json.dump({"stockMode": "fixedbox"}, f)
            out2 = os.path.join(self.tmp_dir, "dump_box.json")
            orch.dump_intermediate_to_json(cnc, cps, out2)
            with open(out2, "r", encoding="utf-8") as f:
                ctx2 = json.load(f)
            self.assertIn("stockExport", ctx2)
            self.assertFalse(ctx2["stockExport"]["enabled"])

            # 3. dead cfg.stockPath is ignored and does not inject stockPath even when the file exists
            self.assertTrue(os.path.isfile(self.bin_stl))
            with open(cfg_file, "w", encoding="utf-8") as f:
                json.dump({"stockMode": "previous", "stockPath": self.bin_stl}, f)
            out3 = os.path.join(self.tmp_dir, "dump_dead_branch.json")
            orch.dump_intermediate_to_json(cnc, cps, out3)
            with open(out3, "r", encoding="utf-8") as f:
                ctx3 = json.load(f)
            self.assertEqual(ctx3.get("stockPath", ""), "")
        finally:
            if old_env is None:
                os.environ.pop("DUMPER_CONFIG_PATH", None)
            else:
                os.environ["DUMPER_CONFIG_PATH"] = old_env

    def test_notify_post_process_result_propagates_skip_errors_in_details(self):
        import PostClear
        mock_ui = MagicMock()
        mock_ui.palettes = MagicMock()

        verify_results = [
            {"setup": "Setup1", "cnc": "/path/setup1.nc", "report": "/path/setup1_report.md", "kinds": {}},
            {"setup": "Setup2", "cnc": "/path/setup2.nc", "report": "/path/setup2_report.md", "error": "Previous stock missing", "kinds": {}},
        ]
        skip_msg = "Failed on setup Setup2: PostProcess pipeline error: Setup 'Setup2' uses 'From Previous Setup' stock, but Fusion 360 did not produce a preceding stock mesh."

        with patch("PostClear._BuildNotificationHTML") as mock_build_html, \
             patch("PostClear.open", unittest.mock.mock_open(), create=True):
            PostClear._NotifyPostProcessResult(
                ui=mock_ui,
                cntFiles=1,
                cntSkipped=1,
                lstSkipped=skip_msg,
                outputFolder="/output/nc",
                verifyResults=verify_results,
            )
            # Verify _BuildNotificationHTML received the skip error message in details
            mock_build_html.assert_called_once()
            args, _ = mock_build_html.call_args
            title, status_type, msg, details, actions = args
            self.assertEqual(status_type, "warning")
            self.assertIn("Failed on setup Setup2", details)
            self.assertIn("From Previous Setup", details)

    def test_postprocesssetup_warns_when_preceding_stock_missing_and_cancels_on_no(self):
        import PostClear
        mock_ui = MagicMock()
        mock_ui.messageBox.return_value = adsk.core.DialogResults.DialogNo

        # Setup 1 (preceding setup) with missing toolpath
        s1 = MagicMock()
        s1.name = "Setup1"
        op1 = MagicMock()
        op1.isSuppressed = False
        s1.allOperations = [op1]

        # Setup 2 using PreviousSetupStock
        s2 = MagicMock()
        s2.name = "Setup2"
        s2.operationType = 0
        s2.stockMode = adsk.cam.SetupStockModes.PreviousSetupStock
        op2 = MagicMock()
        op2.isSuppressed = False
        s2.allOperations = [op2]

        mock_cam = MagicMock()
        mock_cam.setups = [s1, s2]
        # checkToolpath returns False for op1
        mock_cam.checkToolpath.side_effect = lambda op: op is op2

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
            # Verify modal warning was called with the exact prompt
            mock_ui.messageBox.assert_called_once()
            prompt = mock_ui.messageBox.call_args[0][0]
            self.assertIn("Preceding setup stock is likely missing", prompt)
            self.assertIn("proceed?", prompt.lower())
            self.assertEqual(ret, "Operation cancelled by user.")

    def test_postprocesssetup_warns_when_preceding_stock_missing_and_proceeds_on_yes(self):
        import PostClear
        mock_ui = MagicMock()
        mock_ui.messageBox.return_value = adsk.core.DialogResults.DialogYes

        s1 = MagicMock()
        s1.name = "Setup1"
        op1 = MagicMock()
        op1.isSuppressed = False
        s1.allOperations = [op1]

        s2 = MagicMock()
        s2.name = "Setup2"
        s2.operationType = 0
        s2.stockMode = adsk.cam.SetupStockModes.PreviousSetupStock
        op2 = MagicMock()
        op2.isSuppressed = False
        s2.allOperations = [op2]

        mock_cam = MagicMock()
        mock_cam.setups = [s1, s2]
        mock_cam.checkToolpath.return_value = False

        mock_doc = MagicMock()
        mock_doc.products.itemByProductType.return_value = mock_cam
        mock_app = MagicMock()
        mock_app.activeDocument = mock_doc
        mock_app.userInterface = mock_ui

        mock_prog = MagicMock()
        mock_prog.parameters.itemByName.return_value = None

        with patch("PostClear.adsk.core.Application.get", return_value=mock_app), \
             patch("PostClear._PostProcessUnifiedPipeline", return_value=None) as mock_unified:
            ret = PostClear.PostProcessSetup(
                fname="Setup2",
                setup=s2,
                setupFolder=self.tmp_dir,
                docSettings={},
                program=mock_prog,
                target_operations=[op2],
            )
            mock_ui.messageBox.assert_called_once()
            # Since user clicked Yes, it proceeded to _PostProcessUnifiedPipeline
            mock_unified.assert_called_once()
            self.assertIsNone(ret)

    def test_revolve_cone_taper_accuracy(self):
        """H3: Cone r 5->25 over Z 0->100 with 64 segments has model radius >= true_r - 0.01 mm."""
        from shapely.geometry import LineString
        n = 64
        tris = []
        for i in range(n):
            a0 = 2 * math.pi * i / n
            a1 = 2 * math.pi * (i + 1) / n
            p00 = (5.0 * math.cos(a0), 5.0 * math.sin(a0), 0.0)
            p01 = (5.0 * math.cos(a1), 5.0 * math.sin(a1), 0.0)
            p10 = (25.0 * math.cos(a0), 25.0 * math.sin(a0), 100.0)
            p11 = (25.0 * math.cos(a1), 25.0 * math.sin(a1), 100.0)
            tris += [(p00, p01, p11), (p00, p11, p10)]
            tris += [((0.0, 0.0, 0.0), p01, p00), ((0.0, 0.0, 100.0), p10, p11)]

        verts = np.array(tris, dtype=np.float32).reshape(-1, 3)
        indices = np.arange(len(verts), dtype=np.int32).reshape(-1, 3)

        wp = revolve_mesh_to_lathe_stock(verts, indices, max_radial_offset_mm=None, tol_pad_r_mm=0.0)
        self.assertIsNotNone(wp.stock.polygon)
        poly = wp.stock.polygon

        worst_diff = 0.0
        for z in np.linspace(1.0, 99.0, 50):
            true_r = 5.0 + 20.0 * z / 100.0
            inter = poly.intersection(LineString([(z, 0.0), (z, 1000.0)]))
            self.assertFalse(inter.is_empty, f"Intersection empty at z={z}")
            model_r = inter.bounds[3]
            diff = model_r - true_r
            self.assertGreaterEqual(model_r, true_r - 0.01, f"Underestimation at z={z}: model={model_r}, true={true_r}")
            worst_diff = min(worst_diff, diff)
        self.assertGreaterEqual(worst_diff, -0.01)

    def test_revolve_empty_slab_gets_zero_radius(self):
        """H3: Slabs with no triangle overlap get radius 0."""
        from shapely.geometry import LineString
        def _cyl_tris(r, z0, z1, n=16):
            theta = np.linspace(0, 2 * np.pi, n, endpoint=False)
            x = r * np.cos(theta)
            y = r * np.sin(theta)
            v0 = np.column_stack([x, y, np.full(n, z0)])
            v1 = np.column_stack([x, y, np.full(n, z1)])
            res = []
            for i in range(n):
                nxt = (i + 1) % n
                res.append([v0[i], v1[i], v1[nxt]])
                res.append([v0[i], v1[nxt], v0[nxt]])
                res.append([[0, 0, z0], v0[nxt], v0[i]])
                res.append([[0, 0, z1], v1[i], v1[nxt]])
            return np.array(res, dtype=np.float32)

        c1 = _cyl_tris(10.0, 0.0, 10.0)
        c2 = _cyl_tris(10.0, 20.0, 30.0)
        tris = np.vstack([c1, c2])
        verts = tris.reshape(-1, 3)
        indices = np.arange(len(verts), dtype=np.int32).reshape(-1, 3)

        wp = revolve_mesh_to_lathe_stock(verts, indices, z_slice_step_mm=1.0, tol_pad_r_mm=0.0, max_radial_offset_mm=None)
        poly = wp.stock.polygon

        # Test at z=15.0 (inside empty slab gap)
        inter = poly.intersection(LineString([(15.0, 0.0), (15.0, 1000.0)]))
        model_r = inter.bounds[3] if not inter.is_empty else 0.0
        self.assertEqual(model_r, 0.0)

    def test_context_parser_stock_unit_scaling(self):
        """H2: context_parser uses stockUnit (not post unit) to scale STL."""
        from context_parser import extract_workpiece_from_context
        # 1. stockUnit = "mm" while post unit = 0 (IN)
        ctx_mm = {
            "unit": 0,
            "stockUnit": "mm",
            "stockMode": "previous-setup",
            "stockPath": self.bin_stl,  # box Z=0..20 mm
            "sections": [],
        }
        wp_mm = extract_workpiece_from_context(ctx_mm, is_turning=False)
        self.assertAlmostEqual(wp_mm.stock.z_max, 20.0, places=2)

        # 2. stockUnit = "in" while post unit = 1 (MM)
        ctx_in = {
            "unit": 1,
            "stockUnit": "in",
            "stockMode": "previous-setup",
            "stockPath": self.bin_stl,  # box Z=0..20 inches -> 508 mm
            "sections": [],
        }
        wp_in = extract_workpiece_from_context(ctx_in, is_turning=False)
        self.assertAlmostEqual(wp_in.stock.z_max, 20.0 * 25.4, places=2)

    def test_context_parser_workpiece_hint_cross_check(self):
        """H2: workpieceHint selects scale matching candidate set within 5% or fails."""
        from context_parser import extract_workpiece_from_context
        # STL box has extents: dx=20, dy=20, dz=20
        # Hint matching scale 1.0 (20 mm)
        ctx_scale1 = {
            "unit": 1,
            "stockMode": "previous-setup",
            "stockPath": self.bin_stl,
            "workpieceHint": {
                "lower": {"x": -10.0, "y": -10.0, "z": 0.0},
                "upper": {"x": 10.0, "y": 10.0, "z": 20.0},
            },
            "sections": [],
        }
        wp1 = extract_workpiece_from_context(ctx_scale1, is_turning=False)
        self.assertAlmostEqual(wp1.stock.z_max, 20.0, places=2)

        # Hint matching scale 1.0 with unit=0 (inches): both STL and hint in inches -> 508 mm
        ctx_scale1_inch = {
            "unit": 0,
            "stockMode": "previous-setup",
            "stockPath": self.bin_stl,
            "workpieceHint": {
                "lower": {"x": -10.0, "y": -10.0, "z": 0.0},
                "upper": {"x": 10.0, "y": 10.0, "z": 20.0},
            },
            "sections": [],
        }
        wp1_inch = extract_workpiece_from_context(ctx_scale1_inch, is_turning=False)
        self.assertAlmostEqual(wp1_inch.stock.z_max, 508.0, places=1)

        # Hint matching scale 25.4 (508 mm)
        ctx_scale254 = {
            "unit": 1,
            "stockMode": "previous-setup",
            "stockPath": self.bin_stl,
            "workpieceHint": {
                "lower": {"x": -254.0, "y": -254.0, "z": 0.0},
                "upper": {"x": 254.0, "y": 254.0, "z": 508.0},
            },
            "sections": [],
        }
        wp254 = extract_workpiece_from_context(ctx_scale254, is_turning=False)
        self.assertAlmostEqual(wp254.stock.z_max, 508.0, places=1)

        # Hint matching scale 1/25.4 (0.7874 in)
        ctx_scale_inv = {
            "unit": 0,
            "stockMode": "previous-setup",
            "stockPath": self.bin_stl,
            "workpieceHint": {
                "lower": {"x": -10.0 / 25.4, "y": -10.0 / 25.4, "z": 0.0},
                "upper": {"x": 10.0 / 25.4, "y": 10.0 / 25.4, "z": 20.0 / 25.4},
            },
            "sections": [],
        }
        wp_inv = extract_workpiece_from_context(ctx_scale_inv, is_turning=False)
        self.assertAlmostEqual(wp_inv.stock.z_max, 20.0, places=2)

        # Hint mismatch failure: extents 200 mm (none of {1, 25.4, 1/25.4} matches within 5%)
        ctx_mismatch = {
            "unit": 1,
            "stockMode": "previous-setup",
            "stockPath": self.bin_stl,
            "workpieceHint": {
                "lower": {"x": 0.0, "y": 0.0, "z": 0.0},
                "upper": {"x": 200.0, "y": 200.0, "z": 200.0},
            },
            "sections": [],
        }
        with self.assertRaises(ValueError) as cm:
            extract_workpiece_from_context(ctx_mismatch, is_turning=False)
        self.assertIn("do not match workpieceHint", str(cm.exception))
        self.assertIn("within 5%", str(cm.exception))

    def test_get_document_stock_unit_helper(self):
        """H2: _GetDocumentStockUnit detects 'mm' or 'in' from unitsManager."""
        import PostClear
        mock_doc = MagicMock()
        mock_doc.unitsManager.defaultLengthUnits = "in"
        self.assertEqual(PostClear._GetDocumentStockUnit(mock_doc), "in")

        mock_doc.unitsManager.defaultLengthUnits = "mm"
        self.assertEqual(PostClear._GetDocumentStockUnit(mock_doc), "mm")

        mock_doc.unitsManager.defaultLengthUnits = "inches"
        self.assertEqual(PostClear._GetDocumentStockUnit(mock_doc), "in")

        self.assertEqual(PostClear._GetDocumentStockUnit(None), "mm")

    def test_pipeline_stl_overlap_milling(self):
        """H4: Milling STL stock requires >= 50% toolpath bounding-box volume overlap."""
        from pipeline import PipelineOrchestrator
        dummy_cps = os.path.join(self.tmp_dir, "dummy_h4_m.cps")
        with open(dummy_cps, "w", encoding="utf-8") as f:
            f.write('description = "Dummy Post";\ncapabilities = CAPABILITY_MILLING;\n')
        dummy_cnc = os.path.join(self.tmp_dir, "dummy_h4_m.cnc")
        with open(dummy_cnc, "w", encoding="utf-8") as f:
            f.write("N10 G00 X0 Y0 Z10\n")

        orchestrator = PipelineOrchestrator(dumper_cps="universal_dumper.cps", replayer_cps="universal_replayer.cps")

        # Stock box: X=[-10, 10], Y=[-10, 10], Z=[0, 20]
        # 1. Toolpath with 100% overlap
        tp_pass = {
            "programName": "H4MillPass",
            "stockMode": "previous-setup",
            "stockPath": self.bin_stl,
            "unit": 1,
            "sections": [
                {
                    "id": 0,
                    "type": 0,
                    "events": [
                        {"type": "onLinear", "x": -5.0, "y": -5.0, "z": 5.0, "f": 1000.0},
                        {"type": "onLinear", "x": 5.0, "y": 5.0, "z": 15.0, "f": 1000.0},
                    ],
                }
            ],
        }
        json_pass = os.path.join(self.tmp_dir, "h4_m_pass.json")
        with open(json_pass, "w", encoding="utf-8") as f:
            json.dump(tp_pass, f)

        report_p = os.path.join(self.tmp_dir, "h4_m_report.md")
        with patch.object(orchestrator, "replay_combined_context", return_value="G1 X0 Y0\n"):
            rep, _ = orchestrator.run(
                context_dump_paths=[json_pass],
                base_post_path=dummy_cps,
                bootstrap_cnc_path=dummy_cnc,
                report_output_path=report_p,
                promote=False,
            )
        self.assertIsNotNone(rep.stock_bounding_box)
        self.assertIsNotNone(rep.toolpath_bounding_box)
        with open(report_p, "r", encoding="utf-8") as f:
            report_text = f.read()
        self.assertIn("Stock & Toolpath Alignment", report_text)
        self.assertIn(rep.stock_bounding_box, report_text)

        # 2. Toolpath with < 50% overlap: cutting moves in X=[15, 35], Y=[15, 35], Z=[5, 15] (outside stock)
        tp_fail = {
            "programName": "H4MillFail",
            "stockMode": "previous-setup",
            "stockPath": self.bin_stl,
            "unit": 1,
            "sections": [
                {
                    "id": 0,
                    "type": 0,
                    "events": [
                        {"type": "onLinear", "x": 15.0, "y": 15.0, "z": 5.0, "f": 1000.0},
                        {"type": "onLinear", "x": 35.0, "y": 35.0, "z": 15.0, "f": 1000.0},
                    ],
                }
            ],
        }
        json_fail = os.path.join(self.tmp_dir, "h4_m_fail.json")
        with open(json_fail, "w", encoding="utf-8") as f:
            json.dump(tp_fail, f)

        with self.assertRaises(ValueError) as cm:
            orchestrator.run(
                context_dump_paths=[json_fail],
                base_post_path=dummy_cps,
                bootstrap_cnc_path=dummy_cnc,
                promote=False,
            )
        self.assertIn("insufficient volume overlap", str(cm.exception))
        self.assertIn("Toolpath bounding box", str(cm.exception))
        self.assertIn("Stock bounding box", str(cm.exception))

        # 3. Constant-Z planar cut out-of-bounds (dz == 0 at Z=30, stock Z=[0, 20])
        tp_const_z = {
            "programName": "H4MillConstZFail",
            "stockMode": "previous-setup",
            "stockPath": self.bin_stl,
            "unit": 1,
            "sections": [
                {
                    "id": 0,
                    "type": 0,
                    "events": [
                        {"type": "onLinear", "sx": -5.0, "sy": -5.0, "sz": 30.0, "x": 5.0, "y": 5.0, "z": 30.0, "f": 1000.0},
                    ],
                }
            ],
        }
        json_const_z = os.path.join(self.tmp_dir, "h4_m_const_z.json")
        with open(json_const_z, "w", encoding="utf-8") as f:
            json.dump(tp_const_z, f)

        with self.assertRaises(ValueError) as cm_cz:
            orchestrator.run(
                context_dump_paths=[json_const_z],
                base_post_path=dummy_cps,
                bootstrap_cnc_path=dummy_cnc,
                promote=False,
            )
        self.assertIn("insufficient volume overlap", str(cm_cz.exception))

    def test_pipeline_stl_overlap_turning(self):
        """H4: Turning STL stock requires >= 50% toolpath Z span overlap."""
        from pipeline import PipelineOrchestrator
        dummy_cps = os.path.join(self.tmp_dir, "dummy_h4_t.cps")
        with open(dummy_cps, "w", encoding="utf-8") as f:
            f.write('description = "Dummy Post";\ncapabilities = CAPABILITY_TURNING;\n')
        dummy_cnc = os.path.join(self.tmp_dir, "dummy_h4_t.cnc")
        with open(dummy_cnc, "w", encoding="utf-8") as f:
            f.write("N10 G00 X0 Z10\n")

        orchestrator = PipelineOrchestrator(dumper_cps="universal_dumper.cps", replayer_cps="universal_replayer.cps")

        # Stock box Z is [0, 20]
        # 1. Passing turning toolpath: cutting moves from Z=5 to Z=15 (Z span 10 mm, 100% inside [0, 20])
        tp_pass = {
            "programName": "H4TurnPass",
            "stockMode": "previous-setup",
            "stockPath": self.bin_stl,
            "unit": 1,
            "sections": [
                {
                    "id": 0,
                    "type": 1,
                    "tool": {"_isTurningTool": True, "type": 1},
                    "events": [
                        {"type": "onLinear", "x": 5.0, "y": 0.0, "z": 5.0, "f": 100.0},
                        {"type": "onLinear", "x": 5.0, "y": 0.0, "z": 15.0, "f": 100.0},
                    ],
                }
            ],
        }
        json_pass = os.path.join(self.tmp_dir, "h4_t_pass.json")
        with open(json_pass, "w", encoding="utf-8") as f:
            json.dump(tp_pass, f)

        mock_tool = MagicMock()
        with patch.object(orchestrator, "replay_combined_context", return_value="G1 Z5\n"), \
             patch("pipeline.extract_tools_from_context", return_value={"0": mock_tool}):
            rep, _ = orchestrator.run(
                context_dump_paths=[json_pass],
                base_post_path=dummy_cps,
                bootstrap_cnc_path=dummy_cnc,
                promote=False,
                is_turning=True,
            )
        self.assertIsNotNone(rep.stock_bounding_box)
        self.assertIsNotNone(rep.toolpath_bounding_box)

        # 2. Failing turning toolpath: cutting moves from Z=25 to Z=45 (Z span 20 mm, overlap with [0, 20] is 0%)
        tp_fail = {
            "programName": "H4TurnFail",
            "stockMode": "previous-setup",
            "stockPath": self.bin_stl,
            "unit": 1,
            "sections": [
                {
                    "id": 0,
                    "type": 1,
                    "tool": {"_isTurningTool": True, "type": 1},
                    "events": [
                        {"type": "onLinear", "x": 5.0, "y": 0.0, "z": 25.0, "f": 100.0},
                        {"type": "onLinear", "x": 5.0, "y": 0.0, "z": 45.0, "f": 100.0},
                    ],
                }
            ],
        }
        json_fail = os.path.join(self.tmp_dir, "h4_t_fail.json")
        with open(json_fail, "w", encoding="utf-8") as f:
            json.dump(tp_fail, f)

        with patch("pipeline.extract_tools_from_context", return_value={"0": mock_tool}):
            with self.assertRaises(ValueError) as cm:
                orchestrator.run(
                    context_dump_paths=[json_fail],
                    base_post_path=dummy_cps,
                    bootstrap_cnc_path=dummy_cnc,
                    promote=False,
                    is_turning=True,
                )
        self.assertIn("insufficient overlap", str(cm.exception))
        self.assertIn("Toolpath bounding box", str(cm.exception))
        self.assertIn("Stock bounding box", str(cm.exception))

        # 3. Constant-Z facing cut out-of-bounds (dz == 0 at Z=30, stock Z=[0, 20])
        tp_const_z = {
            "programName": "H4TurnConstZFail",
            "stockMode": "previous-setup",
            "stockPath": self.bin_stl,
            "unit": 1,
            "sections": [
                {
                    "id": 0,
                    "type": 1,
                    "tool": {"_isTurningTool": True, "type": 1},
                    "events": [
                        {"type": "onLinear", "sx": 5.0, "sy": 0.0, "sz": 30.0, "x": 15.0, "y": 0.0, "z": 30.0, "f": 100.0},
                    ],
                }
            ],
        }
        json_const_z = os.path.join(self.tmp_dir, "h4_t_const_z.json")
        with open(json_const_z, "w", encoding="utf-8") as f:
            json.dump(tp_const_z, f)

        with patch("pipeline.extract_tools_from_context", return_value={"0": mock_tool}):
            with self.assertRaises(ValueError) as cm_cz:
                orchestrator.run(
                    context_dump_paths=[json_const_z],
                    base_post_path=dummy_cps,
                    bootstrap_cnc_path=dummy_cnc,
                    promote=False,
                    is_turning=True,
                )
        self.assertIn("insufficient overlap", str(cm_cz.exception))

    def test_universal_dumper_export_stock_safe_mode_and_staging(self):
        """Verify universal_dumper_export_stock.cps enables exportStock in post --interrogate safe mode."""
        import shutil
        import subprocess
        import PostClear
        from pipeline import find_autodesk_post_binary

        with open("universal_dumper.cps", "r", encoding="utf-8") as f:
            cps_src = f.read()
        self.assertIn("getConfigurationPath", cps_src)
        self.assertIn("export_stock", cps_src)

        cfg_path = PostClear._WriteDumperConfig(
            target_dir=self.tmp_dir,
            stock_mode="previous-setup",
            export_stock=True,
            set_env=False,
        )
        with open(cfg_path, "r", encoding="utf-8") as f:
            cfg_data = json.load(f)
        self.assertTrue(cfg_data.get("export_stock"))

        # If Autodesk post binary is available, verify actual `post --interrogate` safe-mode output
        try:
            post_exe = find_autodesk_post_binary()
        except Exception:
            post_exe = None
        if post_exe and os.path.isfile(post_exe):
            staged_export = os.path.join(self.tmp_dir, "universal_dumper_export_stock.cps")
            shutil.copy2("universal_dumper.cps", staged_export)

            res_default = subprocess.run(
                [post_exe, "--interrogate", os.path.abspath("universal_dumper.cps")],
                capture_output=True,
                text=True,
                timeout=15,
            )
            self.assertEqual(res_default.returncode, 0)
            self.assertIn('"exportStock":false', res_default.stdout.replace(" ", ""))

            res_export = subprocess.run(
                [post_exe, "--interrogate", staged_export],
                capture_output=True,
                text=True,
                timeout=15,
            )
            self.assertEqual(res_export.returncode, 0)
            self.assertIn('"exportStock":true', res_export.stdout.replace(" ", ""))

    def test_roundstock2_inprocess_stock_hint_and_overlap(self):
        """Verify in-process stock with machined axes and G1 clearance retracts passes H2 and H4."""
        from context_parser import extract_workpiece_from_context
        from pipeline import PipelineOrchestrator

        # 1. H2: In-process stock STL is 20x20x20 (X machined down from 28 -> 20, Z from 22 -> 20, Y unmachined = 20)
        ctx_inprocess = {
            "unit": 1,
            "stockMode": "previous-setup",
            "stockPath": self.bin_stl,
            "workpieceHint": {
                "lower": {"x": -14.0, "y": -10.0, "z": 0.0},
                "upper": {"x": 14.0, "y": 10.0, "z": 22.0},
            },
            "sections": [],
        }
        wp = extract_workpiece_from_context(ctx_inprocess, is_turning=False)
        self.assertIsNotNone(wp)
        self.assertAlmostEqual(wp.stock.x_max, 10.0, places=2)

        # 2. H4: RoundStock2 milling toolpath with vertical G1 retracts above st_max_z (Z=35 vs st_max_z=20)
        # and cutter center stepping over by tool radius (X=12.5 with ø6mm tool on X=[-10, 10] stock)
        dummy_cps = os.path.join(self.tmp_dir, "dummy_rs2.cps")
        with open(dummy_cps, "w", encoding="utf-8") as f:
            f.write('description = "Dummy Post";\ncapabilities = CAPABILITY_MILLING;\n')
        dummy_cnc = os.path.join(self.tmp_dir, "dummy_rs2.cnc")
        with open(dummy_cnc, "w", encoding="utf-8") as f:
            f.write("N10 G00 X0 Y0 Z10\n")

        tp_rs2 = {
            "programName": "RoundStock2",
            "stockMode": "previous-setup",
            "stockPath": self.bin_stl,
            "unit": 1,
            "sections": [
                {
                    "id": 0,
                    "type": 0,
                    "tool": {"number": 2, "type": 9, "diameter": 6.0},
                    "events": [
                        {"type": "onLinear", "sx": -4.8, "sy": -4.9, "sz": 35.0, "x": -4.8, "y": -4.9, "z": 13.0, "f": 500.0},
                        {"type": "onLinear", "sx": -4.8, "sy": -4.9, "sz": 13.0, "x": 12.5, "y": 4.1, "z": 18.0, "f": 1000.0},
                        {"type": "onLinear", "sx": 12.5, "sy": 4.1, "sz": 18.0, "x": 12.5, "y": 4.1, "z": 35.0, "f": 1000.0},
                    ],
                }
            ],
        }
        json_rs2 = os.path.join(self.tmp_dir, "rs2_pass.json")
        with open(json_rs2, "w", encoding="utf-8") as f:
            json.dump(tp_rs2, f)

        orchestrator = PipelineOrchestrator(dumper_cps="universal_dumper.cps", replayer_cps="universal_replayer.cps")
        with patch.object(orchestrator, "replay_combined_context", return_value="G1 X0 Y0\n"):
            rep, _ = orchestrator.run(
                context_dump_paths=[json_rs2],
                base_post_path=dummy_cps,
                bootstrap_cnc_path=dummy_cnc,
                promote=False,
            )
        self.assertIsNotNone(rep.stock_bounding_box)

    def test_pipeline_stl_overlap_milling_thin_plate_through_drill(self):
        """H4: Thin-plate through-hole drilling passes when >= 50% of stock Z thickness is overlapped."""
        from pipeline import PipelineOrchestrator
        from scripts.harvest_corpus_fusion import write_binary_stl

        verts = np.array(
            [
                [-20.0, -20.0, -2.5], [20.0, -20.0, -2.5], [20.0, 20.0, -2.5], [-20.0, 20.0, -2.5],
                [-20.0, -20.0, 2.5], [20.0, -20.0, 2.5], [20.0, 20.0, 2.5], [-20.0, 20.0, 2.5],
            ],
            dtype=np.float32,
        )
        idxs = np.array(
            [
                [0, 1, 2], [0, 2, 3],
                [4, 6, 5], [4, 7, 6],
                [0, 4, 5], [0, 5, 1],
                [1, 5, 6], [1, 6, 2],
                [2, 6, 7], [2, 7, 3],
                [3, 7, 4], [3, 4, 0],
            ],
            dtype=np.int32,
        )
        plate_stl = os.path.join(self.tmp_dir, "thin_plate.stl")
        write_binary_stl(plate_stl, verts, idxs)

        dummy_cps = os.path.join(self.tmp_dir, "dummy_plate.cps")
        with open(dummy_cps, "w", encoding="utf-8") as f:
            f.write('description = "Dummy Post";\ncapabilities = CAPABILITY_MILLING;\n')
        dummy_cnc = os.path.join(self.tmp_dir, "dummy_plate.cnc")
        with open(dummy_cnc, "w", encoding="utf-8") as f:
            f.write("N10 G00 X0 Y0 Z5\n")

        orchestrator = PipelineOrchestrator(dumper_cps="universal_dumper.cps", replayer_cps="universal_replayer.cps")

        tp_pass = {
            "programName": "ThinPlateThroughDrill",
            "stockMode": "previous-setup",
            "stockPath": plate_stl,
            "unit": 1,
            "sections": [
                {
                    "id": 0,
                    "type": 0,
                    "tool": {"number": 1, "type": 3, "diameter": 6.35},
                    "events": [
                        {"type": "onLinear", "sx": 0.0, "sy": 0.0, "sz": 2.5, "x": 0.0, "y": 0.0, "z": -12.7, "f": 500.0},
                    ],
                }
            ],
        }
        json_pass = os.path.join(self.tmp_dir, "thin_plate_pass.json")
        with open(json_pass, "w", encoding="utf-8") as f:
            json.dump(tp_pass, f)

        with patch.object(orchestrator, "replay_combined_context", return_value="G1 X0 Y0 Z-12.7\n"):
            rep, _ = orchestrator.run(
                context_dump_paths=[json_pass],
                base_post_path=dummy_cps,
                bootstrap_cnc_path=dummy_cnc,
                promote=False,
            )
        self.assertIsNotNone(rep.stock_bounding_box)

        # Misaligned deep plunge overlapping only 1.0 mm (20% < 50%) of the 5.0 mm plate still fails H4
        tp_fail = {
            "programName": "ThinPlateShallowFail",
            "stockMode": "previous-setup",
            "stockPath": plate_stl,
            "unit": 1,
            "sections": [
                {
                    "id": 0,
                    "type": 0,
                    "tool": {"number": 1, "type": 3, "diameter": 6.35},
                    "events": [
                        {"type": "onLinear", "sx": 0.0, "sy": 0.0, "sz": -1.5, "x": 0.0, "y": 0.0, "z": -15.0, "f": 500.0},
                    ],
                }
            ],
        }
        json_fail = os.path.join(self.tmp_dir, "thin_plate_fail.json")
        with open(json_fail, "w", encoding="utf-8") as f:
            json.dump(tp_fail, f)

        with self.assertRaises(ValueError) as cm:
            orchestrator.run(
                context_dump_paths=[json_fail],
                base_post_path=dummy_cps,
                bootstrap_cnc_path=dummy_cnc,
                promote=False,
            )
        self.assertIn("insufficient volume overlap", str(cm.exception))

    def test_h4_rejects_plus_and_minus_z_wcs_shifts_with_retracts(self):
        """H4: +Z and -Z WCS shifts cannot be masked by vertical G1 plunge/retract legs."""
        from pipeline import PipelineOrchestrator

        dummy_cps = os.path.join(self.tmp_dir, "dummy_wcs_shift.cps")
        with open(dummy_cps, "w", encoding="utf-8") as f:
            f.write('description = "Dummy Post";\ncapabilities = CAPABILITY_MILLING;\n')
        dummy_cnc = os.path.join(self.tmp_dir, "dummy_wcs_shift.cnc")
        with open(dummy_cnc, "w", encoding="utf-8") as f:
            f.write("N10 G00 X0 Y0 Z50\n")

        orchestrator = PipelineOrchestrator(dumper_cps="universal_dumper.cps", replayer_cps="universal_replayer.cps")

        # 1. +Z WCS shift grazing top 0.5 mm of Z=[0, 20] stock (cuts Z=19.5..35.0 + retract Z=45.0)
        tp_plus_z = {
            "programName": "PlusZWcsShift",
            "stockMode": "previous-setup",
            "stockPath": self.bin_stl,
            "unit": 1,
            "sections": [
                {
                    "id": 0,
                    "type": 0,
                    "tool": {"number": 1, "type": 9, "diameter": 6.0},
                    "events": [
                        {"type": "onLinear", "sx": 0.0, "sy": 0.0, "sz": 45.0, "x": 0.0, "y": 0.0, "z": 19.5, "feed": 500.0},
                        {"type": "onLinear", "sx": 0.0, "sy": 0.0, "sz": 19.5, "x": 5.0, "y": 0.0, "z": 35.0, "feed": 1000.0},
                        {"type": "onLinear", "sx": 5.0, "sy": 0.0, "sz": 35.0, "x": 5.0, "y": 0.0, "z": 45.0, "feed": 1000.0},
                    ],
                }
            ],
        }
        p_plus = os.path.join(self.tmp_dir, "plus_z_shift.json")
        with open(p_plus, "w", encoding="utf-8") as f:
            json.dump(tp_plus_z, f)

        with self.assertRaises(ValueError) as cm_plus:
            orchestrator.run(
                context_dump_paths=[p_plus],
                base_post_path=dummy_cps,
                bootstrap_cnc_path=dummy_cnc,
                promote=False,
            )
        self.assertIn("insufficient volume overlap", str(cm_plus.exception))

        # 2. -Z WCS shift below Z=[0, 20] stock connected by vertical G1 plunge/retract to Z=+25.0
        tp_minus_z = {
            "programName": "MinusZWcsShift",
            "stockMode": "previous-setup",
            "stockPath": self.bin_stl,
            "unit": 1,
            "sections": [
                {
                    "id": 0,
                    "type": 0,
                    "tool": {"number": 1, "type": 9, "diameter": 6.0},
                    "events": [
                        {"type": "onLinear", "sx": 0.0, "sy": 0.0, "sz": 25.0, "x": 0.0, "y": 0.0, "z": -40.0, "feed": 500.0},
                        {"type": "onLinear", "sx": 0.0, "sy": 0.0, "sz": -40.0, "x": 5.0, "y": 5.0, "z": -25.0, "feed": 1000.0},
                        {"type": "onLinear", "sx": 5.0, "sy": 5.0, "sz": -25.0, "x": 5.0, "y": 5.0, "z": 25.0, "feed": 1000.0},
                    ],
                }
            ],
        }
        p_minus = os.path.join(self.tmp_dir, "minus_z_shift.json")
        with open(p_minus, "w", encoding="utf-8") as f:
            json.dump(tp_minus_z, f)

        with self.assertRaises(ValueError) as cm_minus:
            orchestrator.run(
                context_dump_paths=[p_minus],
                base_post_path=dummy_cps,
                bootstrap_cnc_path=dummy_cnc,
                promote=False,
            )
        self.assertIn("insufficient volume overlap", str(cm_minus.exception))

    def test_h4_passes_finish_facing_on_stock_top_with_lead_and_xy_park(self):
        """H4: Finish facing directly on Z == st_max_z with vertical lead-in and XY park passes."""
        from pipeline import PipelineOrchestrator

        dummy_cps = os.path.join(self.tmp_dir, "dummy_face_top.cps")
        with open(dummy_cps, "w", encoding="utf-8") as f:
            f.write('description = "Dummy Post";\ncapabilities = CAPABILITY_MILLING;\n')
        dummy_cnc = os.path.join(self.tmp_dir, "dummy_face_top.cnc")
        with open(dummy_cnc, "w", encoding="utf-8") as f:
            f.write("N10 G00 X0 Y0 Z35\n")

        orchestrator = PipelineOrchestrator(dumper_cps="universal_dumper.cps", replayer_cps="universal_replayer.cps")

        # Stock is [-10, 10] x [-10, 10] x [0, 20]; ø10mm face mill extends by ~1.5*R to X=17.5,
        # cuts on Z=20.0 (mov=14) after a 0.32 mm vertical lead-in, then parks at (100, 100, 35).
        tp_face = {
            "programName": "FaceOnStockTop",
            "stockMode": "previous-setup",
            "stockPath": self.bin_stl,
            "unit": 1,
            "sections": [
                {
                    "id": 0,
                    "type": 0,
                    "params": {"operation-comment": "Face1", "operation:zRetract": 35.0},
                    "tool": {"number": 1, "type": 9, "diameter": 10.0},
                    "events": [
                        {"type": "onLinear", "sx": -15.0, "sy": 0.0, "sz": 35.0, "x": -15.0, "y": 0.0, "z": 20.32, "feed": 1000.0, "movement": 2},
                        {"type": "onLinear", "sx": -15.0, "sy": 0.0, "sz": 20.32, "x": -14.5, "y": 0.0, "z": 20.0, "feed": 1000.0},
                        {"type": "onLinear", "sx": -14.5, "sy": 0.0, "sz": 20.0, "x": 17.5, "y": 0.0, "z": 20.0, "feed": 1500.0, "movement": 14},
                        {"type": "onLinear", "sx": 17.5, "sy": 0.0, "sz": 20.0, "x": 17.5, "y": 0.0, "z": 35.0, "feed": 1500.0},
                        {"type": "onLinear", "sx": 17.5, "sy": 0.0, "sz": 35.0, "x": 100.0, "y": 100.0, "z": 35.0, "feed": 3000.0},
                    ],
                }
            ],
        }
        p_face = os.path.join(self.tmp_dir, "face_top.json")
        with open(p_face, "w", encoding="utf-8") as f:
            json.dump(tp_face, f)

        with patch.object(orchestrator, "replay_combined_context", return_value="G1 X0 Y0\n"):
            rep, _ = orchestrator.run(
                context_dump_paths=[p_face],
                base_post_path=dummy_cps,
                bootstrap_cnc_path=dummy_cnc,
                promote=False,
            )
        self.assertIsNotNone(rep.stock_bounding_box)

    def test_h4_multi_instance_pattern_passes_and_blocks_outside_promotions(self):
        """H4: Multi-instance CAM pattern passes alignment on Instance #1 and blocks promotions on unmodeled copies."""
        from pipeline import PipelineOrchestrator

        dummy_cps = os.path.join(self.tmp_dir, "dummy_pattern.cps")
        with open(dummy_cps, "w", encoding="utf-8") as f:
            f.write('description = "Dummy Post";\ncapabilities = CAPABILITY_MILLING;\n')
        dummy_cnc = os.path.join(self.tmp_dir, "dummy_pattern.cnc")
        with open(dummy_cnc, "w", encoding="utf-8") as f:
            f.write("N10 G00 X0 Y0 Z30\n")

        orchestrator = PipelineOrchestrator(dumper_cps="universal_dumper.cps", replayer_cps="universal_replayer.cps")

        # Instance #1 (sec 0) cuts inside [-10, 10] x [-10, 10] x [0, 20] with an above-stock air traverse at Z=25;
        # Instance #2 (sec 1) is a CAM pattern duplicate ("Pocket1") at X=[195, 205] outside solid_stock.stl.
        tp_pattern = {
            "programName": "PatternSetup",
            "stockMode": "previous-setup",
            "stockPath": self.bin_stl,
            "unit": 1,
            "sections": [
                {
                    "id": 0,
                    "type": 0,
                    "params": {"operation-comment": "Pocket1"},
                    "tool": {"number": 1, "type": 9, "diameter": 6.0},
                    "events": [
                        {"type": "onLinear", "sx": -6.0, "sy": -6.0, "sz": 25.0, "x": 6.0, "y": 6.0, "z": 25.0, "feed": 2000.0, "movement": 2},
                        {"type": "onLinear", "sx": -5.0, "sy": -5.0, "sz": 15.0, "x": 5.0, "y": 5.0, "z": 10.0, "feed": 1000.0, "movement": 13},
                    ],
                },
                {
                    "id": 1,
                    "type": 0,
                    "params": {"operation-comment": "Pocket1"},
                    "tool": {"number": 1, "type": 9, "diameter": 6.0},
                    "events": [
                        {"type": "onLinear", "sx": 195.0, "sy": -5.0, "sz": 15.0, "x": 205.0, "y": 5.0, "z": 10.0, "feed": 1000.0, "movement": 13},
                    ],
                },
            ],
        }
        p_pat = os.path.join(self.tmp_dir, "pattern_pass.json")
        with open(p_pat, "w", encoding="utf-8") as f:
            json.dump(tp_pattern, f)

        with patch.object(orchestrator, "replay_combined_context", return_value="G0 X6 Y6\n"):
            rep, _ = orchestrator.run(
                context_dump_paths=[p_pat],
                base_post_path=dummy_cps,
                bootstrap_cnc_path=dummy_cnc,
                promote=True,
            )
        # Instance #1's verified air link in sec 0 may be promoted, whereas Instance #2 (sec 1) MUST NEVER be promoted!
        self.assertIn((0, 0), rep.promoted_events)
        self.assertFalse(any(sec_i == 1 for sec_i, _ in rep.promoted_events))

        # By contrast, an unrelated out-of-bounds operation ("OtherMisalignedOp") MUST fail H4!
        tp_non_pattern = json.loads(json.dumps(tp_pattern))
        tp_non_pattern["sections"][1]["params"]["operation-comment"] = "OtherMisalignedOp"
        p_non_pat = os.path.join(self.tmp_dir, "non_pattern_fail.json")
        with open(p_non_pat, "w", encoding="utf-8") as f:
            json.dump(tp_non_pattern, f)

        with self.assertRaises(ValueError) as cm_np:
            orchestrator.run(
                context_dump_paths=[p_non_pat],
                base_post_path=dummy_cps,
                bootstrap_cnc_path=dummy_cnc,
                promote=True,
            )
        self.assertIn("insufficient volume overlap", str(cm_np.exception))

    def test_h4_turning_link_retract_passes_and_shifted_straddle_fails(self):
        """H4: Turning keeps MOVEMENT_CUTTING==2 cuts, ignores MOVEMENT_LINK_TRANSITION==4 retracts, and rejects shifted cuts."""
        from pipeline import PipelineOrchestrator

        dummy_cps = os.path.join(self.tmp_dir, "dummy_turn_h4_fix.cps")
        with open(dummy_cps, "w", encoding="utf-8") as f:
            f.write('description = "Dummy Post";\ncapabilities = CAPABILITY_TURNING;\n')
        dummy_cnc = os.path.join(self.tmp_dir, "dummy_turn_h4_fix.cnc")
        with open(dummy_cnc, "w", encoding="utf-8") as f:
            f.write("N10 G00 X20 Z25\n")

        orchestrator = PipelineOrchestrator(dumper_cps="universal_dumper.cps", replayer_cps="universal_replayer.cps")
        mock_tool = MagicMock()

        # Stock Z=[0, 20]: cutting (MOVEMENT_CUTTING=2) at Z=[2, 18] with a MOVEMENT_LINK_TRANSITION=4 safe-Z retract to Z=60 passes
        tp_turn_ok = {
            "programName": "TurnSafeZPass",
            "stockMode": "previous-setup",
            "stockPath": self.bin_stl,
            "unit": 1,
            "sections": [
                {
                    "id": 0,
                    "type": 1,
                    "tool": {"_isTurningTool": True, "type": 1},
                    "events": [
                        {"type": "onLinear", "sx": 8.0, "sy": 0.0, "sz": 18.0, "x": 8.0, "y": 0.0, "z": 2.0, "feed": 100.0, "movement": 2},
                        {"type": "onLinear", "sx": 8.0, "sy": 0.0, "sz": 2.0, "x": 25.0, "y": 0.0, "z": 60.0, "feed": 1000.0, "movement": 4},
                    ],
                }
            ],
        }
        p_ok = os.path.join(self.tmp_dir, "turn_safez_ok.json")
        with open(p_ok, "w", encoding="utf-8") as f:
            json.dump(tp_turn_ok, f)

        with patch.object(orchestrator, "replay_combined_context", return_value="G1 Z2\n"), \
             patch("pipeline.extract_tools_from_context", return_value={"0": mock_tool}):
            rep, _ = orchestrator.run(
                context_dump_paths=[p_ok],
                base_post_path=dummy_cps,
                bootstrap_cnc_path=dummy_cnc,
                promote=False,
                is_turning=True,
            )
        self.assertIsNotNone(rep.stock_bounding_box)

        # Shifted turning cut at Z=[-30, -10] with MOVEMENT_LINK_TRANSITION=4 safe-Z retract to Z=+35 MUST fail!
        tp_turn_bad = {
            "programName": "TurnShiftedStraddleFail",
            "stockMode": "previous-setup",
            "stockPath": self.bin_stl,
            "unit": 1,
            "sections": [
                {
                    "id": 0,
                    "type": 1,
                    "tool": {"_isTurningTool": True, "type": 1},
                    "events": [
                        {"type": "onLinear", "sx": 8.0, "sy": 0.0, "sz": -10.0, "x": 8.0, "y": 0.0, "z": -30.0, "feed": 100.0, "movement": 2},
                        {"type": "onLinear", "sx": 8.0, "sy": 0.0, "sz": -30.0, "x": 25.0, "y": 0.0, "z": 35.0, "feed": 1000.0, "movement": 4},
                    ],
                }
            ],
        }
        p_bad = os.path.join(self.tmp_dir, "turn_safez_bad.json")
        with open(p_bad, "w", encoding="utf-8") as f:
            json.dump(tp_turn_bad, f)

        with patch("pipeline.extract_tools_from_context", return_value={"0": mock_tool}):
            with self.assertRaises(ValueError) as cm_tbad:
                orchestrator.run(
                    context_dump_paths=[p_bad],
                    base_post_path=dummy_cps,
                    bootstrap_cnc_path=dummy_cnc,
                    promote=False,
                    is_turning=True,
                )
        self.assertIn("insufficient overlap", str(cm_tbad.exception))

        # Shifted +Z MOVEMENT_CUTTING=2 pass (Z=19.5 -> 45.0) after a short lead-in at Z=19.5..20.0 MUST also fail!
        tp_turn_mov2_shift = {
            "programName": "TurnMov2PlusZFail",
            "stockMode": "previous-setup",
            "stockPath": self.bin_stl,
            "unit": 1,
            "sections": [
                {
                    "id": 0,
                    "type": 1,
                    "tool": {"_isTurningTool": True, "type": 1},
                    "events": [
                        {"type": "onLinear", "sx": 8.0, "sy": 0.0, "sz": 20.0, "x": 8.0, "y": 0.0, "z": 19.5, "feed": 100.0, "movement": 1},
                        {"type": "onLinear", "sx": 8.0, "sy": 0.0, "sz": 19.5, "x": 8.0, "y": 0.0, "z": 45.0, "feed": 200.0, "movement": 2},
                    ],
                }
            ],
        }
        p_m2_bad = os.path.join(self.tmp_dir, "turn_mov2_bad.json")
        with open(p_m2_bad, "w", encoding="utf-8") as f:
            json.dump(tp_turn_mov2_shift, f)

        with patch("pipeline.extract_tools_from_context", return_value={"0": mock_tool}):
            with self.assertRaises(ValueError) as cm_tm2:
                orchestrator.run(
                    context_dump_paths=[p_m2_bad],
                    base_post_path=dummy_cps,
                    bootstrap_cnc_path=dummy_cnc,
                    promote=False,
                    is_turning=True,
                )
        self.assertIn("insufficient overlap", str(cm_tm2.exception))

    def test_h4_movement2_cutting_with_helical_leadin_passes_and_sloped_shift_fails(self):
        """H4: MOVEMENT_CUTTING==2 horizontal finishing with above-stock MOVEMENT_LEAD_IN==1 ramp passes."""
        from pipeline import PipelineOrchestrator

        dummy_cps = os.path.join(self.tmp_dir, "dummy_mov2_mill.cps")
        with open(dummy_cps, "w", encoding="utf-8") as f:
            f.write('description = "Dummy Post";\ncapabilities = CAPABILITY_MILLING;\n')
        dummy_cnc = os.path.join(self.tmp_dir, "dummy_mov2_mill.cnc")
        with open(dummy_cnc, "w", encoding="utf-8") as f:
            f.write("N10 G00 X0 Y0 Z30\n")

        orchestrator = PipelineOrchestrator(dumper_cps="universal_dumper.cps", replayer_cps="universal_replayer.cps")

        # Stock is [-10, 10] x [-10, 10] x [0, 20]. Helical lead-in (movement=1) starts at Z=28 -> Z=18,
        # followed by horizontal finishing cuts (movement=2 == MOVEMENT_CUTTING) at Z=18.
        tp_mov2_ok = {
            "programName": "PillCrusherHorizFinish",
            "stockMode": "previous-setup",
            "stockPath": self.bin_stl,
            "unit": 1,
            "sections": [
                {
                    "id": 0,
                    "type": 0,
                    "params": {"operation-comment": "Horizontal Top finish"},
                    "tool": {"number": 1, "type": 9, "diameter": 6.0},
                    "events": [
                        {
                            "type": "onLinear",
                            "sx": -4.0, "sy": -4.0, "sz": 28.0,
                            "x": -3.0, "y": -3.0, "z": 18.0,
                            "feed": 600.0,
                            "movement": 1,
                        },
                        {
                            "type": "onLinear",
                            "sx": -3.0, "sy": -3.0, "sz": 18.0,
                            "x": 5.0, "y": 5.0, "z": 18.0,
                            "feed": 1200.0,
                            "movement": 2,
                        },
                    ],
                }
            ],
        }
        p_mov2_ok = os.path.join(self.tmp_dir, "mov2_horiz_ok.json")
        with open(p_mov2_ok, "w", encoding="utf-8") as f:
            json.dump(tp_mov2_ok, f)

        with patch.object(orchestrator, "replay_combined_context", return_value="G1 X5 Y5\n"):
            rep, _ = orchestrator.run(
                context_dump_paths=[p_mov2_ok],
                base_post_path=dummy_cps,
                bootstrap_cnc_path=dummy_cnc,
                promote=False,
            )
        self.assertIsNotNone(rep.stock_bounding_box)

        # A +Z WCS shift where a sloped movement=2 cut runs Z=19.5 -> Z=35.0 MUST still fail!
        tp_mov2_shift = {
            "programName": "Mov2PlusZShift",
            "stockMode": "previous-setup",
            "stockPath": self.bin_stl,
            "unit": 1,
            "sections": [
                {
                    "id": 0,
                    "type": 0,
                    "tool": {"number": 1, "type": 9, "diameter": 6.0},
                    "events": [
                        {
                            "type": "onLinear",
                            "sx": -5.0, "sy": -5.0, "sz": 19.5,
                            "x": 5.0, "y": 5.0, "z": 35.0,
                            "feed": 1200.0,
                            "movement": 2,
                        },
                    ],
                }
            ],
        }
        p_mov2_shift = os.path.join(self.tmp_dir, "mov2_plusz_shift.json")
        with open(p_mov2_shift, "w", encoding="utf-8") as f:
            json.dump(tp_mov2_shift, f)

        with self.assertRaises(ValueError) as cm_m2:
            orchestrator.run(
                context_dump_paths=[p_mov2_shift],
                base_post_path=dummy_cps,
                bootstrap_cnc_path=dummy_cnc,
                promote=False,
            )
        self.assertIn("insufficient volume overlap", str(cm_m2.exception))

    def test_h4_spoilboard_breakthrough_passes_with_matching_workpiece_hint_and_rejects_mismatch(self):
        """H4: Spoilboard dowel breakthrough passes when workpieceHint confirms WCS, rejects when mismatched/absent."""
        from pipeline import PipelineOrchestrator

        dummy_cps = os.path.join(self.tmp_dir, "dummy_spoilboard.cps")
        with open(dummy_cps, "w", encoding="utf-8") as f:
            f.write('description = "Dummy Post";\ncapabilities = CAPABILITY_MILLING;\n')
        dummy_cnc = os.path.join(self.tmp_dir, "dummy_spoilboard.cnc")
        with open(dummy_cnc, "w", encoding="utf-8") as f:
            f.write("N10 G00 X0 Y0 Z30\n")

        orchestrator = PipelineOrchestrator(dumper_cps="universal_dumper.cps", replayer_cps="universal_replayer.cps")

        # Stock is [-10, 10] x [-10, 10] x [0, 20]; dowel boring starts inside pocket floor at Z=2.0
        # and penetrates 12 mm into spoilboard bed (Z=-12.0), so raw Z overlap is 2/14 = 14.3% < 50%.
        tp_spoil_ok = {
            "programName": "DowelHolesInBed",
            "stockMode": "previous-setup",
            "stockPath": self.bin_stl,
            "unit": 1,
            "workpieceHint": {
                "lower": {"x": -10.0, "y": -10.0, "z": 0.0},
                "upper": {"x": 10.0, "y": 10.0, "z": 20.0},
            },
            "sections": [
                {
                    "id": 0,
                    "type": 0,
                    "params": {"operation-comment": "HolesInBed"},
                    "tool": {"number": 1, "type": 9, "diameter": 6.0},
                    "events": [
                        {
                            "type": "onLinear",
                            "sx": -3.0, "sy": -3.0, "sz": 2.0,
                            "x": 3.0, "y": 3.0, "z": -12.0,
                            "feed": 800.0,
                            "movement": 6,
                        },
                    ],
                }
            ],
        }
        p_spoil_ok = os.path.join(self.tmp_dir, "spoil_ok.json")
        with open(p_spoil_ok, "w", encoding="utf-8") as f:
            json.dump(tp_spoil_ok, f)

        with patch.object(orchestrator, "replay_combined_context", return_value="G1 Z-12\n"):
            rep, _ = orchestrator.run(
                context_dump_paths=[p_spoil_ok],
                base_post_path=dummy_cps,
                bootstrap_cnc_path=dummy_cnc,
                promote=False,
            )
        self.assertIsNotNone(rep.stock_bounding_box)

        # Without workpieceHint, or when workpieceHint disagrees with a WCS-shifted STL, H4 MUST reject!
        tp_no_hint = json.loads(json.dumps(tp_spoil_ok))
        del tp_no_hint["workpieceHint"]
        p_no_hint = os.path.join(self.tmp_dir, "spoil_no_hint.json")
        with open(p_no_hint, "w", encoding="utf-8") as f:
            json.dump(tp_no_hint, f)

        with self.assertRaises(ValueError) as cm_nh:
            orchestrator.run(
                context_dump_paths=[p_no_hint],
                base_post_path=dummy_cps,
                bootstrap_cnc_path=dummy_cnc,
                promote=False,
            )
        self.assertIn("insufficient volume overlap", str(cm_nh.exception))

        tp_bad_hint = json.loads(json.dumps(tp_spoil_ok))
        tp_bad_hint["workpieceHint"] = {
            "lower": {"x": -10.0, "y": -10.0, "z": -15.0},
            "upper": {"x": 10.0, "y": 10.0, "z": 5.0},
        }
        p_bad_hint = os.path.join(self.tmp_dir, "spoil_bad_hint.json")
        with open(p_bad_hint, "w", encoding="utf-8") as f:
            json.dump(tp_bad_hint, f)

        with self.assertRaises(ValueError) as cm_bh:
            orchestrator.run(
                context_dump_paths=[p_bad_hint],
                base_post_path=dummy_cps,
                bootstrap_cnc_path=dummy_cnc,
                promote=False,
            )
        self.assertIn("insufficient volume overlap", str(cm_bh.exception))

    def test_h4_face_on_stock_top_with_circular_zx_lead_in_and_out_passes(self):
        """H4: Facing on Z == st_max_z with ZX circular lead-in (movement=1) and lead-out (movement=3) arcs passes."""
        from pipeline import PipelineOrchestrator

        dummy_cps = os.path.join(self.tmp_dir, "dummy_circ_lead.cps")
        with open(dummy_cps, "w", encoding="utf-8") as f:
            f.write('description = "Dummy Post";\ncapabilities = CAPABILITY_MILLING;\n')
        dummy_cnc = os.path.join(self.tmp_dir, "dummy_circ_lead.cnc")
        with open(dummy_cnc, "w", encoding="utf-8") as f:
            f.write("N10 G00 X0 Y0 Z30\n")

        orchestrator = PipelineOrchestrator(dumper_cps="universal_dumper.cps", replayer_cps="universal_replayer.cps")

        # Stock is [-10, 10] x [-10, 10] x [0, 20]; ZX circular lead-in (Z=20.32 -> 20.0, movement=1),
        # finish face cut at Z=20.0 (movement=14), ZX circular lead-out (Z=20.0 -> 20.32, movement=3).
        tp_circ_face = {
            "programName": "PinsProjectFace",
            "stockMode": "previous-setup",
            "stockPath": self.bin_stl,
            "unit": 1,
            "sections": [
                {
                    "id": 0,
                    "type": 0,
                    "params": {"operation-comment": "Face1", "operation:retractHeight_value": 25.0},
                    "tool": {"number": 12, "type": 9, "diameter": 3.175},
                    "events": [
                        {
                            "type": "onLinear",
                            "sx": 4.2, "sy": -4.6, "sz": 25.0,
                            "x": 4.2, "y": -4.6, "z": 20.32,
                            "feed": 76.2,
                            "movement": 10,
                        },
                        {
                            "type": "onCircular",
                            "cw": 0,
                            "plane": 1,
                            "sx": 4.2, "sy": -4.6, "sz": 20.32,
                            "x": 3.88, "y": -4.6, "z": 20.0,
                            "cx": 3.88, "cy": -4.6, "cz": 20.32,
                            "radius": 0.32,
                            "sweep": 1.5707963,
                            "feed": 579.0,
                            "movement": 1,
                        },
                        {
                            "type": "onLinear",
                            "sx": 3.88, "sy": -4.6, "sz": 20.0,
                            "x": -3.88, "y": -4.6, "z": 20.0,
                            "feed": 579.0,
                            "movement": 14,
                        },
                        {
                            "type": "onCircular",
                            "cw": 0,
                            "plane": 1,
                            "sx": -3.88, "sy": -4.6, "sz": 20.0,
                            "x": -4.2, "y": -4.6, "z": 20.32,
                            "cx": -3.88, "cy": -4.6, "cz": 20.32,
                            "radius": 0.32,
                            "sweep": 1.5707963,
                            "feed": 579.0,
                            "movement": 3,
                        },
                    ],
                }
            ],
        }
        p_cf = os.path.join(self.tmp_dir, "circ_face_ok.json")
        with open(p_cf, "w", encoding="utf-8") as f:
            json.dump(tp_circ_face, f)

        with patch.object(orchestrator, "replay_combined_context", return_value="G1 X-3.88\n"):
            rep, _ = orchestrator.run(
                context_dump_paths=[p_cf],
                base_post_path=dummy_cps,
                bootstrap_cnc_path=dummy_cnc,
                promote=False,
            )
        self.assertIsNotNone(rep.stock_bounding_box)


if __name__ == "__main__":
    unittest.main()
