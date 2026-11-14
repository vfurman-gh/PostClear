#!/usr/bin/env python3
"""
Tests for frame-safety, tool-data, and pipeline fixes:
- C3 tool data: _holderSections / _shaftSections export & extract_tools_from_context (no phantom 40/60*scale)
- R1 frame consistency: universal_replayer checks R≈W and T≈O, suppresses promotions on mismatch, sidecar/marker diag
- R2 tilted sections: forward != +Z metadata & drop promotions + "not verified (tilted work plane)"
- R4 turning transform skip: sec_type == 1 skips work-plane transform
- R5 section-entry current position: inherits previous section's local end mapped by current frame
- R6 synthesized bounding box: includes arc extents (linearizedMoves & center±radius fallback)
- R8 multi-axis simultaneous sections: fail closed (no promotions, warning, no crash)
- _dbg_pipe per-section workPlane/workOrigin logging
- Report warnings: frame mismatch, tilted not verified, multi-axis, holder not checked
"""

import copy
import json
import os
import sys
import tempfile
import unittest
from unittest.mock import MagicMock, patch

from context_parser import (
    apply_promotions_to_context,
    extract_tools_from_context,
    parse_context_dump,
)
from geometry.models import (
    BoxStock3D,
    MillToolGeometry,
    MotionKind,
    ToolAssembly,
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


def _make_box_wp():
    return WorkpieceDefinition(
        stock=BoxStock3D(x_min=-50.0, x_max=200.0, y_min=-50.0, y_max=200.0, z_min=-30.0, z_max=10.0)
    )


class TestFrameSafetyAndToolDataFixes(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        cls.milling_cnc = os.path.join(cls.repo_root, "tests", "fixtures", "milling_drill_test.cnc")
        try:
            cls.post_bin = find_autodesk_post_binary()
        except FileNotFoundError:
            cls.post_bin = None
        try:
            cls.posts_dir = find_autodesk_system_posts_dir()
        except FileNotFoundError:
            cls.posts_dir = None

    # ------------------------------------------------------------------
    # Task 1: C3 Tool Data
    # ------------------------------------------------------------------

    def test_c3_dumper_exports_holder_and_shaft_sections(self):
        if not self.post_bin or not os.path.isfile(self.milling_cnc):
            self.skipTest("Autodesk post binary or milling_drill_test.cnc not available")
        orch = PipelineOrchestrator(post_bin=self.post_bin)
        with tempfile.TemporaryDirectory() as tmp_dir:
            base_post = os.path.join(tmp_dir, "base.cps")
            with open(base_post, "w", encoding="utf-8") as f:
                f.write('description = "Base"; capabilities = CAPABILITY_MILLING; unit = IN;\n')
            dump_json = os.path.join(tmp_dir, "dump.json")
            ctx = orch.dump_context(self.milling_cnc, base_post, dump_json)
            self.assertGreater(len(ctx["sections"]), 0)
            t = ctx["sections"][0]["tool"]
            self.assertIn("_holderSections", t)
            self.assertIn("_shaftSections", t)
            self.assertIsInstance(t["_holderSections"], list)
            self.assertIsInstance(t["_shaftSections"], list)

    def test_c3_extract_tools_no_phantom_holder_and_new_fields(self):
        # Inch document with no holder defined (holderDiameter=0, no _holderSections)
        ctx_no_holder = {
            "unit": 0,  # IN
            "sections": [
                {
                    "id": 0,
                    "type": 0,
                    "params": {"operation-comment": "NoHolderOp"},
                    "tool": {
                        "number": 1,
                        "diameter": 0.5,
                        "cornerRadius": 0.0,
                        "fluteLength": 1.0,
                        "bodyLength": 1.5,
                        "shoulderLength": 1.25,
                        "shaftDiameter": 0.375,
                        "overallLength": 3.0,
                        "holderDiameter": 0.0,
                        "holderLength": 0.0,
                        "_holderSections": [],
                        "_shaftSections": [],
                    },
                }
            ],
        }
        tools = extract_tools_from_context(ctx_no_holder, is_turning=False)
        geom = tools["NoHolderOp"].geometry
        self.assertIsInstance(geom, MillToolGeometry)
        self.assertAlmostEqual(geom.diameter, 12.7, places=4)
        self.assertAlmostEqual(geom.holder_diameter, 0.0, places=6)
        self.assertAlmostEqual(geom.holder_length, 0.0, places=6)
        known = getattr(MillToolGeometry, "__dataclass_fields__", {})
        if "body_length" in known:
            self.assertAlmostEqual(geom.body_length, 1.5 * 25.4, places=4)
        if "shoulder_length" in known:
            self.assertAlmostEqual(geom.shoulder_length, 1.25 * 25.4, places=4)
        if "shaft_diameter" in known:
            self.assertAlmostEqual(geom.shaft_diameter, 0.375 * 25.4, places=4)
        if "holder_sections" in known:
            self.assertIsNone(geom.holder_sections)

        # Inch document with holder sections and shaft sections (holderDiameter=0, derived from sections)
        ctx_sections = {
            "unit": 0,  # IN
            "sections": [
                {
                    "id": 0,
                    "type": 0,
                    "params": {"operation-comment": "SecHolderOp"},
                    "tool": {
                        "number": 2,
                        "diameter": 0.25,
                        "fluteLength": 0.75,
                        "overallLength": 2.5,
                        "bodyLength": 1.2,
                        "holderDiameter": 0.0,
                        "holderLength": 0.0,
                        "_holderSections": [
                            {"length": 0.5, "diameter": 1.0},
                            {"length": 1.5, "diameter": 2.0},
                        ],
                        "_shaftSections": [
                            {"length": 0.5, "diameter": 0.5},
                        ],
                    },
                }
            ],
        }
        tools2 = extract_tools_from_context(ctx_sections, is_turning=False)
        geom2 = tools2["SecHolderOp"].geometry
        self.assertAlmostEqual(geom2.holder_diameter, 2.0 * 25.4, places=4)
        self.assertAlmostEqual(geom2.holder_length, 2.0 * 25.4, places=4)
        if "shaft_diameter" in known:
            self.assertAlmostEqual(geom2.shaft_diameter, 0.5 * 25.4, places=4)
        if "holder_sections" in known:
            self.assertEqual(len(geom2.holder_sections), 2)
            self.assertAlmostEqual(geom2.holder_sections[0][0], 0.5 * 25.4, places=4)
            self.assertAlmostEqual(geom2.holder_sections[0][1], 1.0 * 25.4, places=4)
            self.assertAlmostEqual(geom2.holder_sections[1][0], 1.5 * 25.4, places=4)
            self.assertAlmostEqual(geom2.holder_sections[1][1], 2.0 * 25.4, places=4)

    # ------------------------------------------------------------------
    # Task 2: R1 Frame Consistency (Real Post Binary Integration)
    # ------------------------------------------------------------------

    def test_r1_frame_mismatch_suppresses_promotions_and_warns_real_post(self):
        if not self.post_bin or not os.path.isfile(self.milling_cnc):
            self.skipTest("Autodesk post binary or milling_drill_test.cnc not available")

        with tempfile.TemporaryDirectory() as tmp_dir:
            # 3-axis vendor post that calls setRotation(workPlane) ONLY (does not call setTranslation)
            vendor_3ax = os.path.join(tmp_dir, "vendor_3ax_rot_only.cps")
            with open(vendor_3ax, "w", encoding="utf-8") as f:
                f.write(
                    'description = "Vendor3AxRotOnly";\n'
                    'vendor = "Test";\n'
                    'certificationLevel = 2;\n'
                    'extension = "nc";\n'
                    'capabilities = CAPABILITY_MILLING;\n'
                    'var fmt = createFormat({decimals: 4});\n'
                    'function onSection() {\n'
                    '  setRotation(currentSection.workPlane);\n'
                    '  writeln("(" + getParameter("operation-comment", "") + ")");\n'
                    '}\n'
                    'function onRapid(x, y, z) { writeln("G0 X" + fmt.format(x) + " Y" + fmt.format(y) + " Z" + fmt.format(z)); }\n'
                    'function onLinear(x, y, z, feed) { writeln("G1 X" + fmt.format(x) + " Y" + fmt.format(y) + " Z" + fmt.format(z) + " F" + fmt.format(feed)); }\n'
                )

            tool_dict = {
                "number": 1,
                "type": 9,
                "diameter": 6.0,
                "fluteLength": 20.0,
                "overallLength": 50.0,
                "cornerRadius": 0.0,
                "spindleRPM": 10000,
                "clockwise": True,
            }
            ctx = {
                "unit": 1,  # mm
                "programName": "R1Test",
                "programComment": "",
                "globalParams": {},
                "comments": [],
                "workpiece": {"lower": {"x": -20, "y": -20, "z": -10}, "upper": {"x": 120, "y": 120, "z": 5}},
                "sections": [
                    # Section 0: workOrigin = 0 -> consistent -> air retract keeps promotion (G0)
                    {
                        "id": 0,
                        "type": 0,
                        "workOrigin": _zero_wo(),
                        "workPlane": _identity_wp(),
                        "initialPosition": {"x": 0.0, "y": 0.0, "z": 20.0},
                        "finalPosition": {"x": 10.0, "y": 0.0, "z": 20.0},
                        "endPos": {"x": 10.0, "y": 0.0, "z": 20.0},
                        "initSpindleSpeed": 10000,
                        "endSpindleSpeed": 10000,
                        "initMovement": 0,
                        "initRadiusComp": 0,
                        "params": {"operation-comment": "Consistent_Op", "operation:zClearance": 20.0},
                        "tool": tool_dict,
                        "postComments": [],
                        "events": [
                            {"type": "onLinear", "sx": 0.0, "sy": 0.0, "sz": 20.0, "x": 10.0, "y": 0.0, "z": 20.0, "feed": 2000.0, "movement": 5, "rc": 0, "rpm": 10000},
                            {"type": "onLinear", "sx": 10.0, "sy": 0.0, "sz": 20.0, "x": 10.0, "y": 0.0, "z": 0.0, "feed": 500.0, "movement": 1, "rc": 0, "rpm": 10000},
                        ],
                    },
                    # Section 1: workOrigin = (50, 0, 0) != 0 -> T=(0,0,0) != O -> mismatch -> G1 (no promotion)
                    {
                        "id": 1,
                        "type": 0,
                        "workOrigin": {"x": 50.0, "y": 0.0, "z": 0.0},
                        "workPlane": _identity_wp(),
                        "initialPosition": {"x": 0.0, "y": 0.0, "z": 20.0},
                        "finalPosition": {"x": 20.0, "y": 0.0, "z": 20.0},
                        "endPos": {"x": 20.0, "y": 0.0, "z": 20.0},
                        "initSpindleSpeed": 10000,
                        "endSpindleSpeed": 10000,
                        "initMovement": 0,
                        "initRadiusComp": 0,
                        "params": {"operation-comment": "OffsetOrigin_Op", "operation:zClearance": 20.0},
                        "tool": tool_dict,
                        "postComments": [],
                        "events": [
                            {"type": "onLinear", "sx": 0.0, "sy": 0.0, "sz": 20.0, "x": 20.0, "y": 0.0, "z": 20.0, "feed": 2000.0, "movement": 5, "rc": 0, "rpm": 10000},
                            {"type": "onLinear", "sx": 20.0, "sy": 0.0, "sz": 20.0, "x": 20.0, "y": 0.0, "z": 0.0, "feed": 500.0, "movement": 1, "rc": 0, "rpm": 10000},
                        ],
                    },
                ],
            }
            ctx_path = os.path.join(tmp_dir, "ctx.json")
            with open(ctx_path, "w", encoding="utf-8") as f:
                json.dump(ctx, f)

            orch = PipelineOrchestrator(post_bin=self.post_bin)
            out_nc = os.path.join(tmp_dir, "out.nc")
            report_md = os.path.join(tmp_dir, "out_report.md")
            report, gcode = orch.run(
                context_dump_paths=[ctx_path],
                base_post_path=vendor_3ax,
                bootstrap_cnc_path=self.milling_cnc,
                final_cnc_path=out_nc,
                report_output_path=report_md,
                promote=True,
            )

            # Sidecar diag reported mismatch on OffsetOrigin_Op
            self.assertTrue(len(report.frame_warnings) >= 1)
            self.assertTrue(any("OffsetOrigin_Op" in w for w in report.frame_warnings))
            self.assertIn((0, 0), report.promoted_events)
            self.assertNotIn((1, 0), report.promoted_events)

            sec0_part, sec1_part = gcode.split("(OffsetOrigin_Op)", 1)
            self.assertIn("G0 X10 Y0 Z20", sec0_part)
            self.assertNotIn("G0 X20 Y0 Z20", sec1_part)
            self.assertIn("G1 X20 Y0 Z20", sec1_part)
            self.assertNotIn("POSTCLEAR_REPLAY_DIAG", gcode)

            with open(report_md, "r", encoding="utf-8") as rf:
                md_text = rf.read()
            self.assertIn("OffsetOrigin_Op", md_text)
            self.assertIn("Frame", md_text)

            # Also verify fallback trailing comment marker path when REPLAY_DIAG_JSON is empty
            out_fb = os.path.join(tmp_dir, "out_fb.nc")
            ctx_prom = copy.deepcopy(ctx)
            ctx_prom["sections"][0]["events"][0]["promote"] = True
            ctx_prom["sections"][1]["events"][0]["promote"] = True
            with patch.dict(os.environ, {"REPLAY_DIAG_JSON": ""}):
                gcode_fb = orch.replay_combined_context(
                    combined_context=ctx_prom,
                    bootstrap_cnc_path=self.milling_cnc,
                    base_post_path=vendor_3ax,
                    final_cnc_path=out_fb,
                )
            self.assertNotIn("POSTCLEAR_REPLAY_DIAG", gcode_fb)
            with open(out_fb, "r", encoding="utf-8") as fbf:
                self.assertNotIn("POSTCLEAR_REPLAY_DIAG", fbf.read())
            self.assertIsNotNone(orch.last_replay_diag)
            self.assertEqual(len(orch.last_replay_diag["frameMismatches"]), 1)
            self.assertEqual(orch.last_replay_diag["frameMismatches"][0]["op"], "OffsetOrigin_Op")

    # ------------------------------------------------------------------
    # Task 3: R2 Tilted Sections
    # ------------------------------------------------------------------

    def test_r2_tilted_section_metadata_drops_promotions_and_reports_note(self):
        ctx = {
            "unit": 1,
            "workpiece": {"lower": {"x": -50, "y": -50, "z": -20}, "upper": {"x": 50, "y": 50, "z": 10}},
            "sections": [
                {
                    "id": 0,
                    "type": 0,
                    "workOrigin": _zero_wo(),
                    "workPlane": {
                        "right": {"x": 1.0, "y": 0.0, "z": 0.0},
                        "up": {"x": 0.0, "y": 0.0, "z": 1.0},
                        "forward": {"x": 0.0, "y": -1.0, "z": 0.0},
                    },
                    "params": {"operation-comment": "Tilted_Op"},
                    "tool": {"number": 1, "diameter": 6.0, "fluteLength": 15.0, "overallLength": 40.0},
                    "events": [
                        {"type": "onLinear", "sx": 0.0, "sy": 0.0, "sz": 25.0, "x": 15.0, "y": 0.0, "z": 25.0, "feed": 1000.0, "movement": 5},
                    ],
                },
                {
                    "id": 1,
                    "type": 0,
                    "workOrigin": _zero_wo(),
                    "workPlane": _identity_wp(),
                    "params": {"operation-comment": "Flat_Op"},
                    "tool": {"number": 1, "diameter": 6.0, "fluteLength": 15.0, "overallLength": 40.0},
                    "events": [
                        {"type": "onLinear", "sx": 0.0, "sy": 0.0, "sz": 25.0, "x": 15.0, "y": 0.0, "z": 25.0, "feed": 1000.0, "movement": 5},
                    ],
                },
            ],
        }
        segs = parse_context_dump(ctx)
        self.assertEqual(len(segs), 2)
        self.assertTrue(segs[0].metadata.get("tilted_section"))
        self.assertFalse(segs[1].metadata.get("tilted_section", False))

        with tempfile.TemporaryDirectory() as tmp_dir:
            ctx_path = os.path.join(tmp_dir, "tilted.json")
            with open(ctx_path, "w", encoding="utf-8") as f:
                json.dump(ctx, f)
            dummy_post = os.path.join(tmp_dir, "dummy.cps")
            with open(dummy_post, "w", encoding="utf-8") as f:
                f.write("stub")
            report_path = os.path.join(tmp_dir, "report.md")

            orch = PipelineOrchestrator(post_bin=dummy_post)
            with patch.object(orch, "replay_combined_context", return_value="M30\n"):
                report, _ = orch.run(
                    context_dump_paths=[ctx_path],
                    base_post_path=dummy_post,
                    bootstrap_cnc_path=dummy_post,
                    report_output_path=report_path,
                    promote=True,
                )
            self.assertNotIn((0, 0), report.promoted_events)
            self.assertIn((1, 0), report.promoted_events)
            self.assertIn("Tilted_Op", report.tilted_ops)
            with open(report_path, "r", encoding="utf-8") as rf:
                md = rf.read()
            self.assertIn("not verified (tilted work plane)", md)
            self.assertIn("Tilted_Op", md)

    # ------------------------------------------------------------------
    # Task 4: R4 Turning Skips Work-Plane Transform
    # ------------------------------------------------------------------

    def test_r4_turning_skips_workplane_transform(self):
        ctx = {
            "unit": 1,
            "sections": [
                {
                    "id": 0,
                    "type": 1,  # TYPE_TURNING
                    "workOrigin": {"x": 100.0, "y": 200.0, "z": 300.0},
                    "workPlane": {
                        "right": {"x": -1.0, "y": 0.0, "z": 0.0},
                        "up": {"x": 0.0, "y": 1.0, "z": 0.0},
                        "forward": {"x": 0.0, "y": 0.0, "z": -1.0},
                    },
                    "params": {"operation-comment": "SubspindleTurn"},
                    "tool": {"number": 1, "_isTurningTool": True},
                    "events": [
                        {"type": "onRapid", "sx": 0.0, "sy": 0.0, "sz": 0.0, "x": 12.0, "y": 0.0, "z": -25.0},
                        {"type": "onLinear", "sx": 12.0, "sy": 0.0, "sz": -25.0, "x": 12.0, "y": 0.0, "z": -50.0, "feed": 200.0},
                    ],
                }
            ],
        }
        segs = parse_context_dump(ctx)
        self.assertEqual(len(segs), 2)
        self.assertAlmostEqual(segs[0].end.x, 12.0)
        self.assertAlmostEqual(segs[0].end.y, 0.0)
        self.assertAlmostEqual(segs[0].end.z, -25.0)
        self.assertAlmostEqual(segs[1].end.x, 12.0)
        self.assertAlmostEqual(segs[1].end.z, -50.0)
        self.assertFalse(segs[0].metadata.get("tilted_section", False))

    # ------------------------------------------------------------------
    # Task 5: R5 Section-Entry Current Position
    # ------------------------------------------------------------------

    def test_r5_section_entry_current_position_inherits_previous_end(self):
        if not self.post_bin or not os.path.isfile(self.milling_cnc):
            self.skipTest("Autodesk post binary or milling_drill_test.cnc not available")

        with tempfile.TemporaryDirectory() as tmp_dir:
            probe_post = os.path.join(tmp_dir, "probe_r5.cps")
            with open(probe_post, "w", encoding="utf-8") as f:
                f.write(
                    'description = "ProbeR5";\n'
                    'capabilities = CAPABILITY_MILLING;\n'
                    'extension = "nc";\n'
                    'var fmt = createFormat({decimals: 4});\n'
                    'function onSection() {\n'
                    '  var pre = getCurrentPosition();\n'
                    '  writeln("SEC" + currentSection.getId() + "_PRE=" + fmt.format(pre.x) + "," + fmt.format(pre.y) + "," + fmt.format(pre.z));\n'
                    '  setRotation(currentSection.workPlane);\n'
                    '  setTranslation(currentSection.workOrigin);\n'
                    '  var post = getCurrentPosition();\n'
                    '  writeln("SEC" + currentSection.getId() + "_POST=" + fmt.format(post.x) + "," + fmt.format(post.y) + "," + fmt.format(post.z));\n'
                    '}\n'
                )

            tool_dict = {"number": 1, "type": 9, "diameter": 6.0}
            ctx = {
                "unit": 1,
                "programName": "R5",
                "programComment": "",
                "globalParams": {},
                "comments": [],
                "sections": [
                    {
                        "id": 0,
                        "type": 0,
                        "workOrigin": _zero_wo(),
                        "workPlane": _identity_wp(),
                        "initialPosition": {"x": 1.0, "y": 2.0, "z": 50.0},
                        "finalPosition": {"x": 10.0, "y": 20.0, "z": 30.0},
                        "endPos": {"x": 10.0, "y": 20.0, "z": 30.0},
                        "initSpindleSpeed": 1000,
                        "endSpindleSpeed": 1000,
                        "initMovement": 0,
                        "initRadiusComp": 0,
                        "params": {"operation-comment": "Op0"},
                        "tool": tool_dict,
                        "postComments": [],
                        "events": [],
                    },
                    {
                        "id": 1,
                        "type": 0,
                        "workOrigin": {"x": 100.0, "y": 0.0, "z": 0.0},
                        "workPlane": {
                            "right": {"x": -1.0, "y": 0.0, "z": 0.0},
                            "up": {"x": 0.0, "y": -1.0, "z": 0.0},
                            "forward": {"x": 0.0, "y": 0.0, "z": 1.0},
                        },
                        "initialPosition": {"x": 5.0, "y": 6.0, "z": 7.0},
                        "finalPosition": {"x": 5.0, "y": 6.0, "z": 7.0},
                        "endPos": {"x": 5.0, "y": 6.0, "z": 7.0},
                        "initSpindleSpeed": 1000,
                        "endSpindleSpeed": 1000,
                        "initMovement": 0,
                        "initRadiusComp": 0,
                        "params": {"operation-comment": "Op1"},
                        "tool": tool_dict,
                        "postComments": [],
                        "events": [],
                    },
                ],
            }
            orch = PipelineOrchestrator(post_bin=self.post_bin)
            out_nc = os.path.join(tmp_dir, "r5.nc")
            gcode = orch.replay_combined_context(
                combined_context=ctx,
                bootstrap_cnc_path=self.milling_cnc,
                base_post_path=probe_post,
                final_cnc_path=out_nc,
            )
            # First section keeps its initialPosition (1, 2, 50)
            self.assertIn("SEC0_PRE=1,2,50", gcode)
            # Second section entry starts at prev end (10, 20, 30) in previous frame, then mapped by new frame: (-10+100, -20, 30) = (90, -20, 30)
            self.assertIn("SEC1_PRE=10,20,30", gcode)
            self.assertIn("SEC1_POST=90,-20,30", gcode)

    def test_r5_regression_shopbot_fanuc_haas_linuxcnc(self):
        if not self.post_bin or not self.posts_dir or not os.path.isfile(self.milling_cnc):
            self.skipTest("Autodesk post binary, system posts, or fixture not available")

        orch = PipelineOrchestrator(post_bin=self.post_bin)
        for post_name in ("shopbot.cps", "fanuc.cps", "haas.cps", "linuxcnc.cps"):
            post_path = os.path.join(self.posts_dir, post_name)
            if not os.path.isfile(post_path):
                continue
            with self.subTest(post=post_name), tempfile.TemporaryDirectory() as tmp_dir:
                b0 = orch.dump_context(self.milling_cnc, post_path, os.path.join(tmp_dir, "b0.json"))
                two_sec = copy.deepcopy(b0)
                if not str(two_sec.get("programName", "")).strip().isdigit():
                    two_sec["programName"] = "1001"
                s1 = copy.deepcopy(b0["sections"][0])
                s1["id"] = 1
                s1["params"]["operation-comment"] = "Op2_R5"
                two_sec["sections"].append(s1)
                out_nc = os.path.join(tmp_dir, f"out_{post_name}.nc")
                gcode = orch.replay_combined_context(
                    combined_context=two_sec,
                    bootstrap_cnc_path=self.milling_cnc,
                    base_post_path=post_path,
                    final_cnc_path=out_nc,
                )
                self.assertGreater(len(gcode.strip()), 0)

    # ------------------------------------------------------------------
    # Task 6: R6 Synthesized BoundingBox Includes Arc Extents
    # ------------------------------------------------------------------

    def test_r6_synthesized_bounding_box_includes_arc_extents(self):
        if not self.post_bin or not os.path.isfile(self.milling_cnc):
            self.skipTest("Autodesk post binary or milling_drill_test.cnc not available")

        with tempfile.TemporaryDirectory() as tmp_dir:
            probe_post = os.path.join(tmp_dir, "probe_r6.cps")
            with open(probe_post, "w", encoding="utf-8") as f:
                f.write(
                    'description = "ProbeR6";\n'
                    'capabilities = CAPABILITY_MILLING;\n'
                    'extension = "nc";\n'
                    'var fmt = createFormat({decimals: 4});\n'
                    'function onSection() {\n'
                    '  var lbb = currentSection.getBoundingBox();\n'
                    '  var gbb = currentSection.getGlobalBoundingBox();\n'
                    '  writeln("LBB" + currentSection.getId() + "=" + fmt.format(lbb.lower.x) + "," + fmt.format(lbb.lower.y) + "," + fmt.format(lbb.upper.x) + "," + fmt.format(lbb.upper.y));\n'
                    '  writeln("GBB" + currentSection.getId() + "=" + fmt.format(gbb.lower.x) + "," + fmt.format(gbb.lower.y) + "," + fmt.format(gbb.upper.x) + "," + fmt.format(gbb.upper.y));\n'
                    '}\n'
                )

            tool_dict = {"number": 1, "type": 9, "diameter": 6.0}
            # Sec 0: semicircle from (0, 10, 0) to (0, -10, 0) via (+10, 0, 0) with linearizedMoves
            # Sec 1: same arc WITHOUT linearizedMoves (conservative center±radius fallback) + workOrigin=(100, 50, 0)
            ctx = {
                "unit": 1,
                "programName": "R6",
                "programComment": "",
                "globalParams": {},
                "comments": [],
                "sections": [
                    {
                        "id": 0,
                        "type": 0,
                        "workOrigin": _zero_wo(),
                        "workPlane": _identity_wp(),
                        "initialPosition": {"x": 0.0, "y": 10.0, "z": 0.0},
                        "endPos": {"x": 0.0, "y": -10.0, "z": 0.0},
                        "initSpindleSpeed": 1000,
                        "endSpindleSpeed": 1000,
                        "initMovement": 0,
                        "initRadiusComp": 0,
                        "params": {"operation-comment": "ArcLin"},
                        "tool": tool_dict,
                        "postComments": [],
                        "events": [
                            {
                                "type": "onCircular",
                                "cw": True,
                                "sx": 0.0, "sy": 10.0, "sz": 0.0,
                                "cx": 0.0, "cy": 0.0, "cz": 0.0,
                                "x": 0.0, "y": -10.0, "z": 0.0,
                                "plane": 0,
                                "radius": 10.0,
                                "feed": 500.0,
                                "linearizedMoves": [
                                    {"x": 10.0, "y": 0.0, "z": 0.0, "feed": 500.0},
                                    {"x": 0.0, "y": -10.0, "z": 0.0, "feed": 500.0},
                                ],
                            }
                        ],
                    },
                    {
                        "id": 1,
                        "type": 0,
                        "workOrigin": {"x": 100.0, "y": 50.0, "z": 0.0},
                        "workPlane": _identity_wp(),
                        "initialPosition": {"x": 0.0, "y": 10.0, "z": 0.0},
                        "endPos": {"x": 0.0, "y": -10.0, "z": 0.0},
                        "initSpindleSpeed": 1000,
                        "endSpindleSpeed": 1000,
                        "initMovement": 0,
                        "initRadiusComp": 0,
                        "params": {"operation-comment": "ArcFallback"},
                        "tool": tool_dict,
                        "postComments": [],
                        "events": [
                            {
                                "type": "onCircular",
                                "cw": True,
                                "sx": 0.0, "sy": 10.0, "sz": 0.0,
                                "cx": 0.0, "cy": 0.0, "cz": 0.0,
                                "x": 0.0, "y": -10.0, "z": 0.0,
                                "plane": 0,
                                "radius": 10.0,
                                "feed": 500.0,
                                "linearizedMoves": [],
                            }
                        ],
                    },
                ],
            }
            orch = PipelineOrchestrator(post_bin=self.post_bin)
            gcode = orch.replay_combined_context(
                combined_context=ctx,
                bootstrap_cnc_path=self.milling_cnc,
                base_post_path=probe_post,
                final_cnc_path=os.path.join(tmp_dir, "r6.nc"),
            )
            self.assertIn("LBB0=0,-10,10,10", gcode)
            self.assertIn("GBB0=0,-10,10,10", gcode)
            self.assertIn("LBB1=-10,-10,10,10", gcode)
            self.assertIn("GBB1=90,40,110,60", gcode)

    # ------------------------------------------------------------------
    # Task 7: R8 Multi-Axis Simultaneous Sections
    # ------------------------------------------------------------------

    def test_r8_multiaxis_fails_closed_no_promotion_no_crash(self):
        ctx = {
            "unit": 1,
            "workpiece": {"lower": {"x": -50, "y": -50, "z": -20}, "upper": {"x": 50, "y": 50, "z": 10}},
            "sections": [
                {
                    "id": 0,
                    "type": 0,
                    "isMultiAxis": True,
                    "workOrigin": _zero_wo(),
                    "workPlane": _identity_wp(),
                    "params": {"operation-comment": "Swarf5Ax_Op"},
                    "tool": {"number": 1, "diameter": 6.0, "fluteLength": 15.0, "overallLength": 40.0},
                    "events": [
                        {"type": "onLinear", "sx": 0.0, "sy": 0.0, "sz": 25.0, "x": 15.0, "y": 0.0, "z": 25.0, "feed": 1000.0, "movement": 5},
                        {"type": "onLinear5D", "x": 20.0, "y": 5.0, "z": 15.0, "a": 0.1, "b": 0.2, "c": 0.0, "feed": 500.0},
                    ],
                },
            ],
        }
        # Direct apply_promotions_to_context guard
        ctx_copy = copy.deepcopy(ctx)
        self.assertEqual(apply_promotions_to_context(ctx_copy, {(0, 0)}), 0)
        self.assertNotIn("promote", ctx_copy["sections"][0]["events"][0])

        with tempfile.TemporaryDirectory() as tmp_dir:
            ctx_path = os.path.join(tmp_dir, "ma.json")
            with open(ctx_path, "w", encoding="utf-8") as f:
                json.dump(ctx, f)
            dummy_post = os.path.join(tmp_dir, "dummy.cps")
            with open(dummy_post, "w", encoding="utf-8") as f:
                f.write("stub")
            report_path = os.path.join(tmp_dir, "report.md")
            orch = PipelineOrchestrator(post_bin=dummy_post)
            with patch.object(orch, "replay_combined_context", return_value="M30\n"):
                report, _ = orch.run(
                    context_dump_paths=[ctx_path],
                    base_post_path=dummy_post,
                    bootstrap_cnc_path=dummy_post,
                    report_output_path=report_path,
                    promote=True,
                )
            self.assertEqual(len(report.promoted_events), 0)
            self.assertIn("Swarf5Ax_Op", report.multi_axis_ops)
            with open(report_path, "r", encoding="utf-8") as rf:
                md = rf.read()
            self.assertIn("Swarf5Ax_Op", md)
            self.assertIn("Multi-Axis", md)

    # ------------------------------------------------------------------
    # Task 8: _dbg_pipe Per-Section Logging
    # ------------------------------------------------------------------

    def test_dbg_pipe_logs_section_workplane_and_workorigin(self):
        import types

        inserted_adsk = "adsk" not in sys.modules
        if inserted_adsk:
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

        import PostClear

        saved_log = PostClear._LOG_PATH
        saved_enabled = PostClear._debug_log_enabled
        try:
            with tempfile.TemporaryDirectory() as tmp_dir:
                log_file = os.path.join(tmp_dir, "dbg.log")
                PostClear._LOG_PATH = log_file
                PostClear._SetDebugLogEnabled(True)

                ctx = {
                    "unit": 1,
                    "workpiece": {"lower": {"x": -10, "y": -10, "z": -10}, "upper": {"x": 50, "y": 50, "z": 5}},
                    "sections": [
                        {
                            "id": 0,
                            "type": 0,
                            "workOrigin": {"x": 12.5, "y": -3.25, "z": 0.5},
                            "workPlane": {
                                "right": {"x": -1.0, "y": 0.0, "z": 0.0},
                                "up": {"x": 0.0, "y": -1.0, "z": 0.0},
                                "forward": {"x": 0.0, "y": 0.0, "z": 1.0},
                            },
                            "params": {"operation-comment": "LoggedOp_1"},
                            "tool": {"number": 1, "diameter": 6.0},
                            "events": [
                                {"type": "onLinear", "sx": 0.0, "sy": 0.0, "sz": 0.0, "x": 5.0, "y": 0.0, "z": 0.0, "feed": 500.0},
                            ],
                        }
                    ],
                }
                ctx_path = os.path.join(tmp_dir, "ctx.json")
                with open(ctx_path, "w", encoding="utf-8") as f:
                    json.dump(ctx, f)
                dummy_post = os.path.join(tmp_dir, "dummy.cps")
                with open(dummy_post, "w", encoding="utf-8") as f:
                    f.write("stub")

                orch = PipelineOrchestrator(post_bin=dummy_post)
                with patch.object(orch, "replay_combined_context", return_value="M30\n"):
                    orch.run(
                        context_dump_paths=[ctx_path],
                        base_post_path=dummy_post,
                        bootstrap_cnc_path=dummy_post,
                        promote=False,
                    )

                with open(log_file, "r", encoding="utf-8") as lf:
                    log_text = lf.read()
                matching = [ln for ln in log_text.splitlines() if "LoggedOp_1" in ln and "workPlane" in ln]
                self.assertEqual(len(matching), 1)
                line = matching[0]
                self.assertIn("right=(-1.0000,0.0000,0.0000)", line)
                self.assertIn("up=(0.0000,-1.0000,0.0000)", line)
                self.assertIn("forward=(0.0000,0.0000,1.0000)", line)
                self.assertIn("workOrigin=(12.5000,-3.2500,0.5000)", line)
        finally:
            PostClear._LOG_PATH = saved_log
            PostClear._SetDebugLogEnabled(saved_enabled)

    # ------------------------------------------------------------------
    # Task 9: Report Renders All New Warnings (Including holder_unchecked_ops)
    # ------------------------------------------------------------------

    def test_report_renders_holder_unchecked_and_all_new_warnings(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            ctx = {
                "unit": 1,
                "workpiece": {"lower": {"x": -10, "y": -10, "z": -10}, "upper": {"x": 50, "y": 50, "z": 5}},
                "sections": [
                    {
                        "id": 0,
                        "type": 0,
                        "workOrigin": _zero_wo(),
                        "workPlane": _identity_wp(),
                        "params": {"operation-comment": "NoHolderOp"},
                        "tool": {"number": 1, "diameter": 6.0, "holderDiameter": 0.0, "holderLength": 0.0},
                        "events": [
                            {"type": "onLinear", "sx": 0.0, "sy": 0.0, "sz": 0.0, "x": 5.0, "y": 0.0, "z": 0.0, "feed": 500.0},
                        ],
                    }
                ],
            }
            ctx_path = os.path.join(tmp_dir, "ctx.json")
            with open(ctx_path, "w", encoding="utf-8") as f:
                json.dump(ctx, f)
            dummy_post = os.path.join(tmp_dir, "dummy.cps")
            with open(dummy_post, "w", encoding="utf-8") as f:
                f.write("stub")
            report_path = os.path.join(tmp_dir, "rep.md")

            fake_verifier = MagicMock()
            fake_verifier.holder_unchecked_ops = {"NoHolderOp"}
            fake_verifier.verify_toolpath.return_value = VerificationReport(
                total_machining_time_sec=1.0,
                total_volume_removed_mm3=10.0,
                collisions=[],
                operation_names=["NoHolderOp"],
            )

            orch = PipelineOrchestrator(post_bin=dummy_post)
            with patch.object(orch, "replay_combined_context", return_value="M30\n"):
                report, _ = orch.run(
                    context_dump_paths=[ctx_path],
                    base_post_path=dummy_post,
                    bootstrap_cnc_path=dummy_post,
                    verifier=fake_verifier,
                    report_output_path=report_path,
                    promote=False,
                )
            self.assertEqual(report.holder_unchecked_ops, ["NoHolderOp"])
            with open(report_path, "r", encoding="utf-8") as rf:
                md = rf.read()
            self.assertIn("holder not checked", md.lower())
            self.assertIn("NoHolderOp", md)

            # Verify format_summary_report renders all four new warning categories cleanly together
            combined_report = VerificationReport(
                total_machining_time_sec=2.0,
                total_volume_removed_mm3=20.0,
                collisions=[],
                operation_names=["MismatchOp", "TiltedOp", "MultiAxisOp", "NoHolderOp"],
            )
            combined_report.frame_warnings = ["Op MismatchOp: post workOrigin differs"]
            combined_report.frame_mismatch_ops = ["MismatchOp"]
            combined_report.tilted_ops = ["TiltedOp"]
            combined_report.multi_axis_ops = ["MultiAxisOp"]
            combined_report.holder_unchecked_ops = ["NoHolderOp"]
            all_md = orch.format_summary_report(combined_report, "M30\n", dummy_post, restored_count=0)
            self.assertIn("Frame Mismatch", all_md)
            self.assertIn("not verified (tilted work plane)", all_md)
            self.assertIn("Multi-Axis", all_md)
            self.assertIn("holder not checked", all_md)


if __name__ == "__main__":
    unittest.main()
