import ast
import importlib
import os
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

import dep_bootstrap
import context_parser
import pipeline
from geometry.models import AxisymmetricChuckObstacle, AxisymmetricStock2D, SolidMeshStock3D
from geometry.solid_mesh import workpiece_from_stl_mesh


class TestStage3AcyclicDebugLogger(unittest.TestCase):
    def test_no_postclear_imports_in_core_modules(self):
        """pipeline.py, context_parser.py, and geometry/solid_mesh.py must never import PostClear."""
        targets = (
            os.path.join(REPO_ROOT, "pipeline.py"),
            os.path.join(REPO_ROOT, "context_parser.py"),
            os.path.join(REPO_ROOT, "geometry", "solid_mesh.py"),
        )
        for path in targets:
            with open(path, "r", encoding="utf-8") as f:
                tree = ast.parse(f.read(), filename=path)
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    for alias in node.names:
                        self.assertNotEqual(
                            alias.name.split(".")[0],
                            "PostClear",
                            f"{path} unexpectedly imports PostClear",
                        )
                elif isinstance(node, ast.ImportFrom):
                    mod = (node.module or "").split(".")[0]
                    self.assertNotEqual(
                        mod,
                        "PostClear",
                        f"{path} unexpectedly imports from PostClear",
                    )

    def test_reload_preserves_state_and_rotates_log(self):
        saved_log_path = dep_bootstrap.LOG_PATH
        saved_enabled = dep_bootstrap.is_debug_log_enabled()
        try:
            with tempfile.TemporaryDirectory() as tmp_dir:
                log_path = os.path.join(tmp_dir, "stage3_dbg.log")
                dep_bootstrap.LOG_PATH = log_path
                dep_bootstrap.set_debug_log_enabled(True)

                reloaded = importlib.reload(dep_bootstrap)
                self.assertTrue(reloaded.is_debug_log_enabled())
                self.assertEqual(reloaded.LOG_PATH, log_path)

                reloaded.dbg_log("first line")
                self.assertTrue(os.path.isfile(log_path))

                # Trigger .1 rotation cleanly via max_bytes threshold
                with open(log_path, "w", encoding="utf-8") as f:
                    f.write("x" * 128)
                reloaded.rotate_log_if_needed(log_path, max_bytes=64)
                self.assertTrue(os.path.isfile(log_path + ".1"))
                self.assertFalse(os.path.exists(log_path))

                reloaded.dbg_log("after rotation")
                with open(log_path, "r", encoding="utf-8") as f:
                    self.assertIn("after rotation", f.read())
        finally:
            dep_bootstrap.LOG_PATH = saved_log_path
            dep_bootstrap.set_debug_log_enabled(saved_enabled)

    def test_postclear_log_path_sync_and_teardown_preservation(self):
        import PostClear

        saved_boot_path = dep_bootstrap.LOG_PATH
        saved_pc_path = PostClear._LOG_PATH
        saved_last_synced = PostClear._last_synced_log_path
        saved_enabled = dep_bootstrap.is_debug_log_enabled()
        try:
            with tempfile.TemporaryDirectory() as tmp_dir:
                baseline_log = os.path.join(tmp_dir, "baseline.log")
                custom_boot_log = os.path.join(tmp_dir, "boot_override.log")
                pc_override_log = os.path.join(tmp_dir, "pc_override.log")
                after_log = os.path.join(tmp_dir, "after_restore.log")

                PostClear._LOG_PATH = baseline_log
                PostClear._last_synced_log_path = baseline_log
                dep_bootstrap.LOG_PATH = baseline_log

                # 1. Patching dep_bootstrap.LOG_PATH must NOT be clobbered by _SetDebugLogEnabled
                with patch.object(dep_bootstrap, "LOG_PATH", custom_boot_log):
                    PostClear._SetDebugLogEnabled(True)
                    self.assertEqual(dep_bootstrap.LOG_PATH, custom_boot_log)
                    PostClear._dbg_global("boot-override-msg")
                    self.assertEqual(dep_bootstrap.LOG_PATH, custom_boot_log)
                    self.assertTrue(os.path.isfile(custom_boot_log))
                    with open(custom_boot_log, "r", encoding="utf-8") as f:
                        self.assertIn("boot-override-msg", f.read())

                # 2. Patching PostClear._LOG_PATH syncs during patch AND resyncs back after teardown
                with patch.object(PostClear, "_LOG_PATH", pc_override_log):
                    PostClear._SetDebugLogEnabled(True)
                    self.assertEqual(dep_bootstrap.LOG_PATH, pc_override_log)
                    PostClear._dbg_global("pc-override-msg")
                    with open(pc_override_log, "r", encoding="utf-8") as f:
                        self.assertIn("pc-override-msg", f.read())

                # After patch.object restores PostClear._LOG_PATH, _dbg_global resyncs dep_bootstrap.LOG_PATH
                PostClear._dbg_global("reverted-msg")
                self.assertEqual(dep_bootstrap.LOG_PATH, baseline_log)
                with open(baseline_log, "r", encoding="utf-8") as f:
                    self.assertIn("reverted-msg", f.read())

                with patch.object(PostClear, "_LOG_PATH", after_log):
                    PostClear._dbg_global("second-override")
                    self.assertEqual(dep_bootstrap.LOG_PATH, after_log)
                self.assertEqual(PostClear._LOG_PATH, baseline_log)
                PostClear._SetDebugLogEnabled(False)
                self.assertEqual(dep_bootstrap.LOG_PATH, baseline_log)
        finally:
            dep_bootstrap.LOG_PATH = saved_boot_path
            PostClear._LOG_PATH = saved_pc_path
            PostClear._last_synced_log_path = saved_last_synced
            PostClear._SetDebugLogEnabled(saved_enabled)


class TestStage3PostDiscovery(unittest.TestCase):
    def test_scan_capabilities_and_compatibility(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            mill_cps = os.path.join(tmp_dir, "mill.cps")
            turn_cps = os.path.join(tmp_dir, "turn.cps")
            none_cps = os.path.join(tmp_dir, "none.cps")
            with open(mill_cps, "w", encoding="utf-8") as f:
                f.write('description = "Mill Post"; capabilities = CAPABILITY_MILLING;\n')
            with open(turn_cps, "w", encoding="utf-8") as f:
                f.write('description = "Turn Post"; capabilities = CAPABILITY_TURNING;\n')
            with open(none_cps, "w", encoding="utf-8") as f:
                f.write('description = "Generic";\n')

            self.assertEqual(pipeline.scan_cps_capabilities(mill_cps), {"turning": False, "milling": True})
            self.assertEqual(pipeline.scan_cps_capabilities(turn_cps), {"turning": True, "milling": False})
            self.assertIsNone(pipeline.scan_cps_capabilities(none_cps))

            self.assertTrue(pipeline.is_post_compatible(mill_cps, is_turning=False))
            self.assertFalse(pipeline.is_post_compatible(mill_cps, is_turning=True))
            self.assertTrue(pipeline.is_post_compatible(turn_cps, is_turning=True))
            self.assertFalse(pipeline.is_post_compatible(turn_cps, is_turning=False))
            self.assertTrue(pipeline.is_post_compatible(none_cps, is_turning=True))

    def test_tiered_resolution_preference_and_ambiguity(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            t1_a = os.path.join(tmp_dir, "tier1_a")
            t1_b = os.path.join(tmp_dir, "tier1_b")
            t2 = os.path.join(tmp_dir, "tier2")
            for d in (t1_a, t1_b, t2):
                os.makedirs(d, exist_ok=True)

            p_user = os.path.join(t1_a, "custom.cps")
            p_sys = os.path.join(t2, "custom.cps")
            with open(p_user, "w", encoding="utf-8") as f:
                f.write('description = "Custom User"; capabilities = CAPABILITY_MILLING;\n')
            with open(p_sys, "w", encoding="utf-8") as f:
                f.write('description = "Custom Sys"; capabilities = CAPABILITY_MILLING;\n')

            prog = SimpleNamespace(
                postConfiguration=SimpleNamespace(name="custom.cps", description="Custom")
            )
            resolved = pipeline.resolve_vendor_post_path(
                program=prog,
                is_turning=False,
                search_dirs_by_tier=([t1_a], [t2], []),
            )
            self.assertEqual(resolved, os.path.abspath(p_user))

            # Duplicate compatible post in same tier -> ValueError
            p_dup = os.path.join(t1_b, "custom.cps")
            with open(p_dup, "w", encoding="utf-8") as f:
                f.write('description = "Custom User 2"; capabilities = CAPABILITY_MILLING;\n')
            with self.assertRaises(ValueError) as cm:
                pipeline.resolve_vendor_post_path(
                    program=prog,
                    is_turning=False,
                    search_dirs_by_tier=([t1_a, t1_b], [t2], []),
                )
            self.assertIn("Ambiguous post processor match", str(cm.exception))

            # resolve_post_file honors provided search_dirs order
            self.assertEqual(
                pipeline.resolve_post_file("custom", search_dirs=[t2, t1_a]),
                os.path.abspath(p_sys),
            )


class TestStage3SharedHelpers(unittest.TestCase):
    def test_infer_stl_is_inch_strict_vs_non_strict(self):
        # 1x1x1 raw verts vs 25.4mm workpieceHint (unit=1 mm) -> infers inches (True)
        unit_box_verts = np.array(
            [
                [0.0, 0.0, 0.0],
                [1.0, 1.0, 1.0],
            ],
            dtype=np.float64,
        )
        ctx_in_box = {
            "unit": 1,
            "workpieceHint": {
                "lower": {"x": 0.0, "y": 0.0, "z": 0.0},
                "upper": {"x": 25.4, "y": 25.4, "z": 25.4},
            },
        }
        self.assertTrue(context_parser.infer_stl_is_inch(unit_box_verts, ctx_in_box, strict=True))

        # Divergent extents (>5% for all candidate scales)
        ctx_bad = {
            "unit": 1,
            "stockUnit": "in",
            "workpieceHint": {
                "lower": {"x": 0.0, "y": 0.0, "z": 0.0},
                "upper": {"x": 100.0, "y": 100.0, "z": 100.0},
            },
        }
        with self.assertRaises(ValueError):
            context_parser.infer_stl_is_inch(unit_box_verts, ctx_bad, strict=True)
        # strict=False falls back gracefully to stockUnit/unit
        self.assertTrue(context_parser.infer_stl_is_inch(unit_box_verts, ctx_bad, strict=False))

    def test_workpiece_from_stl_mesh_milling_and_turning(self):
        verts = np.array(
            [
                [-10.0, -10.0, 0.0],
                [10.0, -10.0, 0.0],
                [10.0, 10.0, 20.0],
                [-10.0, 10.0, 20.0],
            ],
            dtype=np.float64,
        )
        indices = np.array([[0, 1, 2], [0, 2, 3]], dtype=np.int32)

        wp_mill = workpiece_from_stl_mesh(verts, indices, is_turning=False, stl_label="test.stl")
        self.assertIsInstance(wp_mill.stock, SolidMeshStock3D)
        self.assertTrue(wp_mill.is_stl_derived)
        self.assertEqual(wp_mill.fixtures, [])

        wp_turn = workpiece_from_stl_mesh(verts, indices, is_turning=True, fixtures=None)
        self.assertIsInstance(wp_turn.stock, AxisymmetricStock2D)
        self.assertTrue(wp_turn.is_stl_derived)
        self.assertEqual(len(wp_turn.fixtures), 1)
        self.assertIsInstance(wp_turn.fixtures[0], AxisymmetricChuckObstacle)

    def test_is_truthy_flag(self):
        for truthy in (True, 1, 2.5, "true", "TRUE", "1", "yes", " Yes "):
            self.assertTrue(context_parser.is_truthy_flag(truthy), f"Expected truthy: {truthy!r}")
        for falsy in (False, None, 0, 0.0, "false", "0", "no", "", []):
            self.assertFalse(context_parser.is_truthy_flag(falsy), f"Expected falsy: {falsy!r}")

    def test_partition_operations_by_tool(self):
        import PostClear

        def _mk_op(name: str, tool_val):
            p = SimpleNamespace(value=SimpleNamespace(value=tool_val))
            params = SimpleNamespace(itemByName=lambda k: p if k == "tool_number" else None)
            return SimpleNamespace(name=name, tool=SimpleNamespace(parameters=params))

        ops = [
            _mk_op("op1", 1),
            _mk_op("op2", 1),
            _mk_op("op3", 3),
            _mk_op("op4", None),
        ]
        batches_none = PostClear.partition_operations_by_tool(ops)
        self.assertEqual([(t, [o.name for o in lst]) for t, lst in batches_none], [
            (1, ["op1", "op2"]),
            (3, ["op3"]),
            (None, ["op4"]),
        ])
        batches_int = PostClear.partition_operations_by_tool(ops, default_tool_number=0, coerce_int=True)
        self.assertEqual([(t, [o.name for o in lst]) for t, lst in batches_int], [
            (1, ["op1", "op2"]),
            (3, ["op3"]),
            (0, ["op4"]),
        ])


if __name__ == "__main__":
    unittest.main()
