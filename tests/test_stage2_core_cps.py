#!/usr/bin/env python3
"""
Focused Stage 2 Core & CPS unit and integration tests covering:
1. context_parser.py:
   - Non-mutating parse_context_dump on dicts (no redundant deepcopy) & consistent metadata on onCyclePoint/onDwell.
   - merge_context_dumps isolation (input dicts unmutated even when merged output is mutated; file-path merging).
   - Unified arc-normal / arc_plane classification (_classify_arc_normal & parse_context_dump).
   - Shared is_turning_section / is_turning_context & extract_workpiece_hint_bbox_mm helpers.
   - Direct MillToolGeometry construction without __dataclass_fields__ introspection.
2. pipeline.py & geometry/heightfield_engine.py:
   - Single-pass detect_unmodeled_pattern_sections in PipelineOrchestrator.run + standalone MillHeightfieldVerifier.verify_toolpath.
   - Consolidated tilted / multi-axis section collection (sections primary + empty-sections segment fallback).
   - Deduplicated format_summary_report operation tables & MRR spike rows.
3. universal_replayer.cps & universal_dumper.cps:
   - getZRange / getGlobalZRange, getGlobalInitialPosition / getGlobalFinalPosition, __applyPostProp, __replaySubMoves.
   - _setVendorProperty / _readVendorProperty in universal_dumper.cps.
"""

import copy
import json
import os
import subprocess
import tempfile
import unittest
from unittest.mock import MagicMock, patch

import context_parser
from context_parser import (
    _classify_arc_normal,
    apply_promotions_to_context,
    extract_tools_from_context,
    extract_workpiece_hint_bbox_mm,
    is_turning_context,
    is_turning_section,
    load_context_dump,
    merge_context_dumps,
    parse_context_dump,
)
from geometry.heightfield_engine import MillHeightfieldVerifier
from geometry.models import (
    BoxStock3D,
    CollisionKind,
    CollisionRecord,
    MRRSample,
    MotionKind,
    MotionSegment,
    Vector3D,
    VerificationReport,
    WorkpieceDefinition,
)
from pipeline import (
    PipelineOrchestrator,
    find_autodesk_post_binary,
    find_autodesk_system_posts_dir,
)


def _identity_wp():
    return {
        "right": {"x": 1.0, "y": 0.0, "z": 0.0},
        "up": {"x": 0.0, "y": 1.0, "z": 0.0},
        "forward": {"x": 0.0, "y": 0.0, "z": 1.0},
    }


def _zero_wo():
    return {"x": 0.0, "y": 0.0, "z": 0.0}


class TestStage2ContextParser(unittest.TestCase):
    def test_parse_context_dump_non_mutating_and_consistent_metadata(self):
        raw_ctx = {
            "unit": 1,
            "sections": [
                {
                    "id": 0,
                    "type": 0,
                    "workPlane": _identity_wp(),
                    "workOrigin": _zero_wo(),
                    "params": {
                        "operation-comment": "DrillAndDwell",
                        "operation:zClearance": 15.0,
                        "operation:zRetract": 5.0,
                    },
                    "tool": {"number": 1, "diameter": 6.0},
                    "events": [
                        {"type": "onRapid", "sx": 0.0, "sy": 0.0, "sz": 15.0, "x": 10.0, "y": 10.0, "z": 5.0},
                        {
                            "type": "onCyclePoint",
                            "sx": 10.0,
                            "sy": 10.0,
                            "sz": 5.0,
                            "x": 10.0,
                            "y": 10.0,
                            "z": -5.0,
                            "expandedMoves": [
                                {"type": "onLinear", "x": 10.0, "y": 10.0, "z": -5.0, "feed": 200.0},
                                {"type": "onRapid", "x": 10.0, "y": 10.0, "z": 5.0},
                            ],
                        },
                        {"type": "onDwell", "sec": 0.5},
                    ],
                }
            ],
        }
        snapshot = copy.deepcopy(raw_ctx)

        with patch("context_parser.copy.deepcopy", wraps=copy.deepcopy) as spy_dc:
            segs = parse_context_dump(raw_ctx)

        self.assertEqual(spy_dc.call_count, 0, "parse_context_dump should not deepcopy dict inputs")
        self.assertEqual(raw_ctx, snapshot)
        # load_context_dump must still return an isolated deep copy
        cloned = load_context_dump(raw_ctx)
        self.assertIsNot(cloned, raw_ctx)
        cloned["sections"][0]["params"]["operation-comment"] = "Mutated"
        self.assertEqual(raw_ctx["sections"][0]["params"]["operation-comment"], "DrillAndDwell")

        # onRapid + 2 cycle expandedMoves + onDwell = 4 segments, all with consistent metadata
        self.assertEqual(len(segs), 4)
        for seg in segs:
            self.assertEqual(seg.metadata["z_clearance_mm"], 15.0)
            self.assertEqual(seg.metadata["z_retract_mm"], 5.0)
            self.assertEqual(seg.metadata["sec_type"], 0)
        self.assertEqual(segs[1].metadata["in_cycle"], True)
        self.assertEqual(segs[3].motion, MotionKind.DWELL)

    def test_merge_context_dumps_isolation_dicts_and_files(self):
        b0 = {
            "unit": 1,
            "rapidMode": {"highFeedMapping": 0, "useG0": True},
            "globalParams": {"p0": "v0"},
            "sections": [
                {
                    "id": 0,
                    "type": 0,
                    "params": {"operation-comment": "Op0"},
                    "tool": {"number": 1, "diameter": 6.0},
                    "events": [
                        {"type": "onLinear", "sx": 0.0, "sy": 0.0, "sz": 5.0, "x": 20.0, "y": 0.0, "z": 5.0, "feed": 500.0, "rc": 0}
                    ],
                }
            ],
        }
        b1 = {
            "unit": 1,
            "globalParams": {"p1": "v1"},
            "workpieceHint": {"lower": {"x": -10, "y": -10, "z": -10}, "upper": {"x": 10, "y": 10, "z": 0}},
            "sections": [
                {
                    "id": 0,
                    "type": 0,
                    "params": {"operation-comment": "Op1"},
                    "tool": {"number": 2, "diameter": 8.0},
                    "events": [],
                }
            ],
        }
        b0_orig = copy.deepcopy(b0)
        b1_orig = copy.deepcopy(b1)

        merged = merge_context_dumps([b0, b1])
        self.assertEqual([s["id"] for s in merged["sections"]], [0, 1])
        self.assertEqual(merged["globalParams"], {"p0": "v0", "p1": "v1"})
        self.assertIn("workpieceHint", merged)

        # Mutating merged output must not leak into source dicts
        apply_promotions_to_context(merged, {(0, 0)})
        merged["rapidMode"]["useG0"] = False
        merged["workpieceHint"]["upper"]["z"] = 999
        merged["globalParams"]["p0"] = "mutated"
        self.assertEqual(b0, b0_orig)
        self.assertEqual(b1, b1_orig)

        # File path inputs merge without redundant section deepcopies
        with tempfile.TemporaryDirectory() as td:
            p0 = os.path.join(td, "b0.json")
            p1 = os.path.join(td, "b1.json")
            with open(p0, "w", encoding="utf-8") as f:
                json.dump(b0, f)
            with open(p1, "w", encoding="utf-8") as f:
                json.dump(b1, f)
            with patch("context_parser.copy.deepcopy", wraps=copy.deepcopy) as spy_dc:
                merged_files = merge_context_dumps([p0, p1])
            self.assertEqual(spy_dc.call_count, 0)
            self.assertEqual([s["id"] for s in merged_files["sections"]], [0, 1])

    def test_unified_arc_plane_classification(self):
        self.assertEqual(_classify_arc_normal(0.0, 0.0, 1.0, True, "XY"), ("XY", True, False))
        self.assertEqual(_classify_arc_normal(0.0, 0.0, -1.0, True, "XY"), ("XY", False, False))
        self.assertEqual(_classify_arc_normal(0.0, 1.0, 0.0, False, "XY"), ("ZX", False, False))
        self.assertEqual(_classify_arc_normal(0.0, -1.0, 0.0, False, "XY"), ("ZX", True, False))
        self.assertEqual(_classify_arc_normal(1.0, 0.0, 0.0, True, "XY"), ("YZ", True, False))
        self.assertEqual(_classify_arc_normal(-1.0, 0.0, 0.0, True, "XY"), ("YZ", False, False))
        self.assertEqual(_classify_arc_normal(0.6, 0.0, 0.8, True, "XY"), ("XY", True, True))

        # Section transform rotating local +Z arc normal onto +X -> YZ plane; negative det flips CW
        ctx = {
            "unit": 1,
            "sections": [
                {
                    "id": 0,
                    "type": 0,
                    "workPlane": {
                        "right": {"x": 0.0, "y": 1.0, "z": 0.0},
                        "up": {"x": 0.0, "y": 0.0, "z": 1.0},
                        "forward": {"x": 1.0, "y": 0.0, "z": 0.0},
                    },
                    "workOrigin": _zero_wo(),
                    "params": {"operation-comment": "SideArc"},
                    "tool": {"number": 1, "diameter": 6.0},
                    "events": [
                        {
                            "type": "onCircular",
                            "cw": True,
                            "plane": 0,
                            "sx": 0.0, "sy": 5.0, "sz": 0.0,
                            "cx": 0.0, "cy": 0.0, "cz": 0.0,
                            "x": 5.0, "y": 0.0, "z": 0.0,
                            "radius": 5.0, "sweep": 1.5708, "feed": 300.0,
                        },
                        {
                            "type": "onCircular",
                            "cw": True,
                            "plane": -1,
                            "nx": 0.0, "ny": 0.0, "nz": 1.0,
                            "sx": 5.0, "sy": 0.0, "sz": 0.0,
                            "cx": 0.0, "cy": 0.0, "cz": 0.0,
                            "x": 0.0, "y": 5.0, "z": 0.0,
                            "radius": 5.0, "sweep": 1.5708, "feed": 300.0,
                        },
                    ],
                },
                {
                    "id": 1,
                    "type": 0,
                    "workPlane": _identity_wp(),
                    "workOrigin": _zero_wo(),
                    "params": {"operation-comment": "FreePlane"},
                    "tool": {"number": 1, "diameter": 6.0},
                    "events": [
                        {
                            "type": "onCircular",
                            "cw": True,
                            "plane": -1,
                            "nx": 0.0, "ny": -1.0, "nz": 0.0,
                            "sx": 0.0, "sy": 0.0, "sz": 5.0,
                            "cx": 0.0, "cy": 0.0, "cz": 0.0,
                            "x": 5.0, "y": 0.0, "z": 0.0,
                            "radius": 5.0, "sweep": 1.5708, "feed": 300.0,
                        }
                    ],
                },
            ],
        }
        segs = parse_context_dump(ctx)
        self.assertEqual(len(segs), 3)
        self.assertEqual(segs[0].arc_plane, "YZ")
        self.assertEqual(segs[0].motion, MotionKind.ARC_CW)
        # Local +Z rotated by forward=(1,0,0) -> +X -> YZ; local nz=1
        # Wait: in second event, explicit nx=0, ny=0, nz=1 in local frame -> forward=(1,0,0) -> YZ
        self.assertEqual(segs[1].arc_plane, "YZ")
        # Identity WP with plane=-1 and normal (0,-1,0) -> ZX with cw flipped to CCW
        self.assertEqual(segs[2].arc_plane, "ZX")
        self.assertEqual(segs[2].motion, MotionKind.ARC_CCW)

    def test_is_turning_and_workpiece_hint_bbox_helpers(self):
        self.assertFalse(is_turning_section({"type": 0, "tool": {"_isTurningTool": False}}))
        self.assertTrue(is_turning_section({"type": 1}))
        self.assertTrue(is_turning_section({"type": 0, "tool": {"_isTurningTool": True}}))
        self.assertFalse(is_turning_context({"sections": [{"type": 0}]}))
        self.assertTrue(is_turning_context({"sections": [{"type": 0}, {"type": 0, "tool": {"_isTurningTool": True}}]}))

        # Inch conversion
        ctx_in = {
            "unit": 0,
            "workpieceHint": {"lower": {"x": -1.0, "y": -2.0, "z": -0.5}, "upper": {"x": 1.0, "y": 2.0, "z": 0.0}},
        }
        bbox_in = extract_workpiece_hint_bbox_mm(ctx_in)
        self.assertIsNotNone(bbox_in)
        self.assertAlmostEqual(bbox_in[0], -25.4)
        self.assertAlmostEqual(bbox_in[3], 50.8)
        self.assertAlmostEqual(bbox_in[4], -12.7)

        # Metric and invalid/degenerate boxes
        ctx_mm = {
            "unit": 1,
            "workpieceHint": {"lower": {"x": 0.0, "y": 0.0, "z": -10.0}, "upper": {"x": 20.0, "y": 30.0, "z": 0.0}},
        }
        self.assertEqual(extract_workpiece_hint_bbox_mm(ctx_mm), (0.0, 20.0, 0.0, 30.0, -10.0, 0.0))
        self.assertIsNone(extract_workpiece_hint_bbox_mm({"unit": 1, "workpieceHint": {"lower": {"x": 5, "y": 0, "z": 0}, "upper": {"x": 5, "y": 10, "z": 10}}}))
        self.assertIsNone(extract_workpiece_hint_bbox_mm({"unit": 1}))

    def test_extract_tools_direct_mill_geometry_construction(self):
        ctx = {
            "unit": 1,
            "sections": [
                {
                    "id": 0,
                    "type": 0,
                    "params": {"operation-comment": "EndMillOp"},
                    "tool": {
                        "number": 5,
                        "diameter": 10.0,
                        "cornerRadius": 0.5,
                        "fluteLength": 20.0,
                        "overallLength": 60.0,
                        "bodyLength": 35.0,
                        "shoulderLength": 25.0,
                        "shaftDiameter": 10.0,
                        "_holderSections": [{"length": 20.0, "diameter": 32.0}],
                    },
                }
            ],
        }
        tools = extract_tools_from_context(ctx, is_turning=False)
        geom = tools["EndMillOp"].geometry
        self.assertEqual(geom.body_length, 35.0)
        self.assertEqual(geom.shoulder_length, 25.0)
        self.assertEqual(geom.shaft_diameter, 10.0)
        self.assertEqual(geom.holder_sections, [(20.0, 32.0)])
        self.assertEqual(geom.holder_diameter, 32.0)
        self.assertEqual(geom.holder_length, 20.0)


class TestStage2PipelineAndHeightfield(unittest.TestCase):
    def test_pipeline_avoids_double_pattern_scan_and_standalone_still_scans(self):
        # Two sibling sections with same op_id: sec 0 inside stock, sec 1 outside stock (unmodeled pattern copy)
        ctx = {
            "unit": 1,
            "workpiece": {"lower": {"x": -20.0, "y": -20.0, "z": -10.0}, "upper": {"x": 20.0, "y": 20.0, "z": 0.0}},
            "sections": [
                {
                    "id": 0,
                    "type": 0,
                    "workPlane": _identity_wp(),
                    "workOrigin": _zero_wo(),
                    "params": {"operation-comment": "PatternOp"},
                    "tool": {"number": 1, "diameter": 6.0, "fluteLength": 15.0, "overallLength": 40.0},
                    "events": [
                        {"type": "onLinear", "sx": 0.0, "sy": 0.0, "sz": -2.0, "x": 10.0, "y": 0.0, "z": -2.0, "feed": 300.0, "rc": 0},
                    ],
                },
                {
                    "id": 1,
                    "type": 0,
                    "workPlane": _identity_wp(),
                    "workOrigin": {"x": 200.0, "y": 0.0, "z": 0.0},
                    "params": {"operation-comment": "PatternOp"},
                    "tool": {"number": 1, "diameter": 6.0, "fluteLength": 15.0, "overallLength": 40.0},
                    "events": [
                        {"type": "onLinear", "sx": 0.0, "sy": 0.0, "sz": 5.0, "x": 10.0, "y": 0.0, "z": 5.0, "feed": 300.0, "rc": 0},
                    ],
                },
            ],
        }
        with tempfile.TemporaryDirectory() as td:
            ctx_path = os.path.join(td, "ctx.json")
            with open(ctx_path, "w", encoding="utf-8") as f:
                json.dump(ctx, f)
            dummy_post = os.path.join(td, "dummy.cps")
            with open(dummy_post, "w", encoding="utf-8") as f:
                f.write("stub")

            orch = PipelineOrchestrator(post_bin=dummy_post)
            verifier = MillHeightfieldVerifier(cell_size_mm=1.0)

            import sys
            hfe = sys.modules[MillHeightfieldVerifier.__module__]
            with patch.object(
                hfe,
                "detect_unmodeled_pattern_sections",
                wraps=hfe.detect_unmodeled_pattern_sections,
            ) as spy_detect, patch.object(orch, "replay_combined_context", return_value="M30\n"):
                report, _ = orch.run(
                    context_dump_paths=[ctx_path],
                    base_post_path=dummy_post,
                    bootstrap_cnc_path=dummy_post,
                    verifier=verifier,
                    promote=True,
                )

            self.assertEqual(spy_detect.call_count, 1, "PipelineOrchestrator.run + MillHeightfieldVerifier must scan pattern sections once")
            self.assertEqual(verifier._unmodeled_pattern_sec_indices, {1})
            self.assertNotIn((1, 0), report.promoted_events)

            # Standalone MillHeightfieldVerifier.verify_toolpath still runs pattern detection
            segs = parse_context_dump(ctx)
            tools = extract_tools_from_context(ctx, is_turning=False)
            wp = WorkpieceDefinition(stock=BoxStock3D(-20.0, 20.0, -20.0, 20.0, -10.0, 0.0))
            standalone_verifier = MillHeightfieldVerifier(cell_size_mm=1.0)
            with patch.object(
                hfe,
                "detect_unmodeled_pattern_sections",
                wraps=hfe.detect_unmodeled_pattern_sections,
            ) as spy_standalone:
                rep_standalone = standalone_verifier.verify_toolpath(segs, tools, wp)
            self.assertEqual(spy_standalone.call_count, 1)
            self.assertEqual(standalone_verifier._unmodeled_pattern_sec_indices, {1})
            self.assertNotIn((1, 0), rep_standalone.promoted_events)

    def test_tilted_multiaxis_segment_fallback_when_sections_empty(self):
        with tempfile.TemporaryDirectory() as td:
            ctx_path = os.path.join(td, "empty.json")
            with open(ctx_path, "w", encoding="utf-8") as f:
                json.dump(
                    {
                        "unit": 1,
                        "workpiece": {"lower": {"x": -10, "y": -10, "z": -10}, "upper": {"x": 10, "y": 10, "z": 0}},
                        "sections": [],
                    },
                    f,
                )
            dummy_post = os.path.join(td, "dummy.cps")
            with open(dummy_post, "w", encoding="utf-8") as f:
                f.write("stub")

            synth_segs = [
                MotionSegment(
                    segment_id=1,
                    op_id="TiltedSynthOp",
                    sec_id=0,
                    motion=MotionKind.LINEAR,
                    start=Vector3D(0, 0, 5),
                    end=Vector3D(10, 0, 5),
                    feedrate_mmpm=300.0,
                    spindle_rpm=1000.0,
                    duration_sec=2.0,
                    metadata={"sec_idx": 0, "event_idx": 0, "tilted_section": True},
                )
            ]
            fake_verifier = MagicMock()
            fake_verifier.holder_unchecked_ops = set()
            fake_verifier.verify_toolpath.return_value = VerificationReport(
                total_machining_time_sec=2.0,
                total_volume_removed_mm3=0.0,
                collisions=[],
                promoted_events={(0, 0)},
                operation_names=["TiltedSynthOp"],
            )

            orch = PipelineOrchestrator(post_bin=dummy_post)
            with patch("pipeline.parse_context_dump", return_value=synth_segs), patch.object(
                orch, "replay_combined_context", return_value="M30\n"
            ):
                rep, _ = orch.run(
                    context_dump_paths=[ctx_path],
                    base_post_path=dummy_post,
                    bootstrap_cnc_path=dummy_post,
                    verifier=fake_verifier,
                    promote=True,
                )
            self.assertEqual(rep.tilted_ops, ["TiltedSynthOp"])
            self.assertEqual(rep.promoted_events, set())

    def test_format_summary_report_tables_and_mrr_spikes(self):
        orch = PipelineOrchestrator(post_bin=__file__)
        # 1. Zero-collision report with Safe, Warning, and Not verified rows
        rep_safe = VerificationReport(
            total_machining_time_sec=60.0,
            total_volume_removed_mm3=100.0,
            collisions=[],
            operation_names=["SafeOp", "UncheckedOp", "TiltedOp"],
            mrr_spikes=[],
        )
        rep_safe.holder_unchecked_ops = ["UncheckedOp"]
        rep_safe.tilted_ops = ["TiltedOp"]
        md_safe = orch.format_summary_report(rep_safe, "M30\n", "/tmp/fanuc.cps", restored_count=2)
        self.assertIn("### Operations Verification Summary", md_safe)
        self.assertIn("| SafeOp | ✅ Safe | 0 | None |", md_safe)
        self.assertIn("| UncheckedOp | ⚠️ Warning | 0 | holder not checked |", md_safe)
        self.assertIn("| TiltedOp | ⚠️ Not verified | 0 | not verified (tilted work plane) |", md_safe)

        # 2. Collision + MRR spike report (including an op not in operation_names)
        rep_col = VerificationReport(
            total_machining_time_sec=30.0,
            total_volume_removed_mm3=50.0,
            collisions=[
                CollisionRecord(
                    segment_id=4,
                    kind=CollisionKind.SHANK_RUB,
                    timestamp_sec=1.5,
                    contact_point=Vector3D(1.0, 2.0, -3.0),
                    penetration_volume_mm3=2.0,
                    description="Shank rub",
                    op_id="ColOp",
                )
            ],
            operation_names=["ColOp", "CleanWarnOp"],
            mrr_spikes=[
                MRRSample(1.0, 0.5, 4, 1, 12.0, 1440.0, 2.0, op_id="ColOp"),
                MRRSample(5.0, 0.5, 9, 2, 18.0, 2160.0, 3.0, op_id="ExtraSpikeOp"),
            ],
        )
        rep_col.holder_unchecked_ops = ["CleanWarnOp"]
        md_col = orch.format_summary_report(rep_col, "M30\n", "/tmp/fanuc.cps", restored_count=0)
        self.assertIn("### Collisions by Operation", md_col)
        self.assertIn("| **ColOp** | ⚠️ **Collision** | **1** | `SHANK_RUB` (1) |", md_col)
        self.assertIn("| CleanWarnOp | ⚠️ Warning | 0 | holder not checked |", md_col)
        self.assertIn("| **Total** | | **1** | |", md_col)
        self.assertIn("| ColOp | Move 4 | 1.00 s | 1440.00 | 12.00 | 2.00 |", md_col)
        self.assertIn("| ExtraSpikeOp | Move 9 | 5.00 s | 2160.00 | 18.00 | 3.00 |", md_col)


class TestStage2CpsPosts(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        cls.dumper_cps = os.path.join(cls.repo_root, "universal_dumper.cps")
        cls.replayer_cps = os.path.join(cls.repo_root, "universal_replayer.cps")
        cls.milling_cnc = os.path.join(cls.repo_root, "tests", "fixtures", "milling_drill_test.cnc")
        try:
            cls.post_bin = find_autodesk_post_binary()
        except FileNotFoundError:
            cls.post_bin = None
        try:
            cls.posts_dir = find_autodesk_system_posts_dir()
        except FileNotFoundError:
            cls.posts_dir = None

    def test_replayer_getZRange_applyPostProp_replaySubMoves_and_dumper_props(self):
        if not self.post_bin or not self.posts_dir:
            self.skipTest("Autodesk post binary or system posts directory not available")
        fanuc_cps = os.path.join(self.posts_dir, "fanuc.cps")
        if not os.path.isfile(fanuc_cps) or not os.path.isfile(self.milling_cnc):
            self.skipTest("fanuc.cps or milling_drill_test.cnc fixture not found")

        with tempfile.TemporaryDirectory() as td:
            # 1. Verify universal_dumper.cps property helpers read/override maximumSpindleSpeed & useG0
            dump_cfg = {
                "basePost": fanuc_cps,
                "postProperties": {"maximumSpindleSpeed": 7777, "useG0": False},
            }
            cfg_path = os.path.join(td, "dumper_cfg.json")
            with open(cfg_path, "w", encoding="utf-8") as f:
                json.dump(dump_cfg, f)
            dump_json = os.path.join(td, "dump.json")
            env_dump = os.environ.copy()
            env_dump["DUMPER_CONFIG_PATH"] = cfg_path
            res_dump = subprocess.run(
                [self.post_bin, "--quiet", self.dumper_cps, self.milling_cnc, dump_json],
                env=env_dump,
                capture_output=True,
                text=True,
                timeout=30,
            )
            self.assertEqual(res_dump.returncode, 0, res_dump.stderr)
            with open(dump_json, "r", encoding="utf-8") as f:
                dumped = json.load(f)
            self.assertEqual(dumped.get("maximumSpindleSpeed"), 7777)
            self.assertEqual((dumped.get("rapidMode") or {}).get("useG0"), False)

            # 2. Verify universal_replayer.cps exercises getZRange, getGlobalInitialPosition,
            #    getGlobalFinalPosition, __applyPostProp, and __replaySubMoves via a probe vendor post
            probe_cps = os.path.join(td, "probe_vendor.cps")
            with open(probe_cps, "w", encoding="utf-8") as f:
                f.write(
                    'description = "Probe Vendor";\n'
                    'capabilities = CAPABILITY_MILLING;\n'
                    'extension = "nc";\n'
                    'properties = { customProp: { title: "Custom", type: "integer", value: 1 } };\n'
                    'function onSection() {\n'
                    '  var zr1 = currentSection.getZRange();\n'
                    '  var zr2 = currentSection.getGlobalZRange();\n'
                    '  writeln("ZRANGE:" + zr1.getMinimum() + ":" + zr2.getMaximum() + ":" + (currentSection.getZRange === currentSection.getGlobalZRange));\n'
                    '  var gip = currentSection.getGlobalInitialPosition();\n'
                    '  var gfp = currentSection.getGlobalFinalPosition();\n'
                    '  writeln("ENDPTS:" + Math.round(gip.x) + "," + Math.round(gfp.x));\n'
                    '  writeln("PROP:" + getProperty("customProp"));\n'
                    '}\n'
                    'function onLinear(x, y, z, feed) { writeln("G1:" + Math.round(x) + "," + Math.round(y) + "," + Math.round(z)); }\n'
                    'function onRapid(x, y, z) { writeln("G0:" + Math.round(x) + "," + Math.round(y) + "," + Math.round(z)); }\n'
                    'function onCircular(cw, cx, cy, cz, x, y, z, feed) { linearize(0.01); }\n'
                    'function onCyclePoint(x, y, z) { repositionToCycleClearance(cycle, x, y, z); expandCyclePoint(x, y, z); }\n'
                    'function onDwell(sec) { writeln("G4:" + sec); }\n'
                )

            replay_ctx = {
                "unit": 1,
                "programName": "1001",
                "programComment": "Stage2Probe",
                "postProperties": {"customProp": 42},
                "globalParams": {},
                "comments": [],
                "manualNC": [],
                "sections": [
                    {
                        "id": 0,
                        "type": 0,
                        "workOffset": 1,
                        "workPlane": _identity_wp(),
                        "workOrigin": {"x": 10.0, "y": 0.0, "z": 0.0},
                        "initialPosition": {"x": 1.0, "y": 0.0, "z": 5.0},
                        "finalPosition": {"x": 9.0, "y": 0.0, "z": 5.0},
                        "zRangeMin": -5.0,
                        "zRangeMax": 15.0,
                        "initSpindleSpeed": 1000,
                        "initMovement": 0,
                        "initRadiusComp": 0,
                        "params": {"operation-comment": "ProbeOp"},
                        "tool": {"number": 1, "diameter": 6.0, "spindleRPM": 1000},
                        "events": [
                            {
                                "type": "onCircular",
                                "cw": True, "plane": 0,
                                "sx": 1.0, "sy": 0.0, "sz": 5.0,
                                "cx": 0.0, "cy": 0.0, "cz": 5.0,
                                "x": 3.0, "y": 4.0, "z": 5.0, "feed": 200,
                                "radius": 5.0, "sweep": 1.0, "nx": 0.0, "ny": 0.0, "nz": 1.0,
                                "linearizedMoves": [{"type": "onLinear", "x": 3.0, "y": 4.0, "z": 5.0, "feed": 200}],
                            },
                            {"type": "onCycle", "cycleType": "drilling", "cycle": {}, "points": [{"x": 9.0, "y": 0.0, "z": -5.0}]},
                            {
                                "type": "onCyclePoint",
                                "sx": 3.0, "sy": 4.0, "sz": 5.0,
                                "x": 9.0, "y": 0.0, "z": -5.0,
                                "first": True, "last": True,
                                "reposMoves": [{"type": "onRapid", "x": 9.0, "y": 0.0, "z": 15.0}],
                                "afterReposMoves": [
                                    {"type": "onLinear", "x": 9.0, "y": 0.0, "z": -5.0, "feed": 150},
                                    {"type": "onDwell", "sec": 0.25},
                                ],
                                "expandedMoves": [],
                            },
                            {"type": "onCycleEnd"},
                        ],
                    }
                ],
            }
            replay_ctx_path = os.path.join(td, "replay_ctx.json")
            with open(replay_ctx_path, "w", encoding="utf-8") as f:
                json.dump(replay_ctx, f)
            out_nc = os.path.join(td, "out.nc")
            diag_json = os.path.join(td, "diag.json")
            env_rep = os.environ.copy()
            env_rep["BASE_POST"] = probe_cps
            env_rep["CONTEXT_JSON"] = replay_ctx_path
            env_rep["REPLAY_DIAG_JSON"] = diag_json
            res_rep = subprocess.run(
                [self.post_bin, "--quiet", self.replayer_cps, self.milling_cnc, out_nc],
                env=env_rep,
                capture_output=True,
                text=True,
                timeout=30,
            )
            self.assertEqual(res_rep.returncode, 0, res_rep.stderr)
            with open(out_nc, "r", encoding="utf-8") as f:
                nc_text = f.read()
            self.assertIn("ZRANGE:-5:15:true", nc_text)
            self.assertIn("ENDPTS:11,19", nc_text)
            self.assertIn("PROP:42", nc_text)
            self.assertIn("G1:3,4,5", nc_text)
            self.assertIn("G0:9,0,15", nc_text)
            self.assertIn("G1:9,0,-5", nc_text)
            self.assertIn("G4:0.25", nc_text)


if __name__ == "__main__":
    unittest.main()
