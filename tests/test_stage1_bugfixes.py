#!/usr/bin/env python3
"""
Focused Stage 1 regression tests covering all 6 bug fixes:
1. Full-line rapid-restoration comments (including ShopBot .sbp `' Rapid restored` above J3 with no trailing comma).
2. _GetNcProgramExtension defaults empty/missing/non-NC extension to `.cnc`.
3. dog_leg_warning propagates to has_warnings and toast warning message.
4. VerificationReport.promoted_events matches actual safe promoted events stamped into replayed G-code.
5. classify_stock_mode handles all stock variants cleanly; previous-setup turning setups attach AxisymmetricChuckObstacle.from_stock; universal_dumper.cps detects prev/rest stock modes.
6. universal_replayer.cps:
   6a. expandManualNC treats empty `expanded: []` as handled (no fallback to vendor.onCommand).
   6b. POSTCLEAR_REPLAY_DIAG inline comment is suppressed when frameMismatchSections is empty.
"""

import json
import math
import os
import struct
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import MagicMock, patch

# Ensure minimal adsk mocks exist so tests run both standalone and under unittest discover
if "adsk" not in sys.modules:
    adsk = types.ModuleType("adsk")
    adsk.cam = types.ModuleType("adsk.cam")
    adsk.core = types.ModuleType("adsk.core")
    adsk.fusion = types.ModuleType("adsk.fusion")
    sys.modules["adsk"] = adsk
    sys.modules["adsk.cam"] = adsk.cam
    sys.modules["adsk.core"] = adsk.core
    sys.modules["adsk.fusion"] = adsk.fusion
else:
    adsk = sys.modules["adsk"]
    for sub in ("cam", "core", "fusion"):
        if not hasattr(adsk, sub):
            mod = types.ModuleType(f"adsk.{sub}")
            setattr(adsk, sub, mod)
            sys.modules[f"adsk.{sub}"] = mod

for _handler_cls in (
    "CommandCreatedEventHandler",
    "InputChangedEventHandler",
    "ValidateInputsEventHandler",
    "CommandEventHandler",
):
    if not hasattr(adsk.core, _handler_cls):
        setattr(adsk.core, _handler_cls, type(_handler_cls, (), {}))
if not hasattr(adsk.core, "Application"):
    adsk.core.Application = types.SimpleNamespace(get=lambda: types.SimpleNamespace(log=lambda _msg: None))
if not hasattr(adsk.cam, "OperationTypes"):
    adsk.cam.OperationTypes = types.SimpleNamespace(TurningOperation=1, MillingOperation=0)
if not hasattr(adsk.cam, "CAM"):
    adsk.cam.CAM = types.SimpleNamespace(cast=lambda x: x)
if not hasattr(adsk.cam, "PostOutputUnitOptions"):
    adsk.cam.PostOutputUnitOptions = types.SimpleNamespace(
        DocumentUnitsOutput=0, InchesOutput=1, MillimetersOutput=2
    )
if not hasattr(adsk.cam, "PostProcessInput"):
    adsk.cam.PostProcessInput = types.SimpleNamespace(
        create=lambda prog, cfg, out_dir, *_a: MagicMock(
            programName=prog, postConfiguration=cfg, outputFolder=out_dir
        )
    )

from context_parser import (
    classify_stock_mode,
    extract_workpiece_from_context,
    parse_context_dump,
)
from geometry.models import (
    AxisymmetricChuckObstacle,
    AxisymmetricStock2D,
    CollisionKind,
    CollisionRecord,
    MotionKind,
    Vector3D,
    VerificationReport,
    WorkpieceDefinition,
)
from pipeline import (
    PipelineOrchestrator,
    find_autodesk_post_binary,
    find_autodesk_system_posts_dir,
)


def _write_binary_stl_cylinder(stl_path: str, radius: float = 25.0, z_min: float = -50.0, z_max: float = 0.0, n_segs: int = 24):
    """Write a minimal closed binary STL cylinder around the Z axis."""
    triangles = []
    for i in range(n_segs):
        a0 = 2.0 * math.pi * i / n_segs
        a1 = 2.0 * math.pi * ((i + 1) % n_segs) / n_segs
        x0, y0 = radius * math.cos(a0), radius * math.sin(a0)
        x1, y1 = radius * math.cos(a1), radius * math.sin(a1)
        triangles.append(((x0, y0, z_min), (x1, y1, z_min), (x1, y1, z_max)))
        triangles.append(((x0, y0, z_min), (x1, y1, z_max), (x0, y0, z_max)))
        triangles.append(((0.0, 0.0, z_min), (x1, y1, z_min), (x0, y0, z_min)))
        triangles.append(((0.0, 0.0, z_max), (x0, y0, z_max), (x1, y1, z_max)))

    with open(stl_path, "wb") as f:
        f.write(b"binary stl cylinder".ljust(80, b"\0"))
        f.write(struct.pack("<I", len(triangles)))
        for v0, v1, v2 in triangles:
            f.write(struct.pack("<3f", 0.0, 0.0, 0.0))
            f.write(struct.pack("<3f", *v0))
            f.write(struct.pack("<3f", *v1))
            f.write(struct.pack("<3f", *v2))
            f.write(struct.pack("<H", 0))


def _identity_wp():
    return {
        "right": {"x": 1.0, "y": 0.0, "z": 0.0},
        "up": {"x": 0.0, "y": 1.0, "z": 0.0},
        "forward": {"x": 0.0, "y": 0.0, "z": 1.0},
    }


def _zero_wo():
    return {"x": 0.0, "y": 0.0, "z": 0.0}


class TestStage1BugFixes(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        cls.dumper_cps = os.path.join(cls.repo_root, "universal_dumper.cps")
        cls.replayer_cps = os.path.join(cls.repo_root, "universal_replayer.cps")
        cls.turning_cnc = os.path.join(cls.repo_root, "tests", "fixtures", "roughing_test.cnc")
        cls.milling_cnc = os.path.join(cls.repo_root, "tests", "fixtures", "milling_drill_test.cnc")
        cls.centroid_cps = os.path.join(cls.repo_root, "centroid_turning.cps")
        try:
            cls.post_bin = find_autodesk_post_binary()
        except FileNotFoundError:
            cls.post_bin = None
        try:
            cls.posts_dir = find_autodesk_system_posts_dir()
        except FileNotFoundError:
            cls.posts_dir = None

    # ------------------------------------------------------------------
    # Bug #1: Full-line Rapid restored comment (ShopBot .sbp & replayer source)
    # ------------------------------------------------------------------

    def test_bug1_replayer_no_writeln_wrapper_args_and_shopbot_standalone_comment(self):
        with open(self.replayer_cps, "r", encoding="utf-8") as f:
            src = f.read()
        self.assertIn("buildVendor = new Function(", src)
        self.assertNotIn("_promoCommentSuffix", src)
        self.assertNotIn("_origWriteln", src)

        if not self.post_bin or not self.posts_dir:
            self.skipTest("Autodesk post binary or system posts dir not available")
        shopbot_cps = os.path.join(self.posts_dir, "shopbot.cps")
        if not os.path.isfile(shopbot_cps) or not os.path.isfile(self.milling_cnc):
            self.skipTest("shopbot.cps or milling_drill_test.cnc not available")

        orch = PipelineOrchestrator(
            dumper_cps=self.dumper_cps,
            replayer_cps=self.replayer_cps,
            post_bin=self.post_bin,
        )
        ctx = {
            "unit": 0,  # IN
            "programName": "ShopBotBug1",
            "programComment": "",
            "globalParams": {},
            "comments": [],
            "workpiece": {"lower": {"x": -1, "y": -1, "z": -1}, "upper": {"x": 5, "y": 5, "z": 0}},
            "sections": [{
                "id": 0,
                "type": 0,
                "workOrigin": _zero_wo(),
                "workPlane": _identity_wp(),
                "initialPosition": {"x": 0.0, "y": 0.0, "z": 1.0},
                "finalPosition": {"x": 1.0, "y": 0.0, "z": -0.25},
                "endPos": {"x": 1.0, "y": 0.0, "z": -0.25},
                "initSpindleSpeed": 12000,
                "endSpindleSpeed": 12000,
                "initMovement": 0,
                "initRadiusComp": 0,
                "params": {"operation-comment": "Pocket1"},
                "tool": {
                    "number": 1, "type": 9, "diameter": 0.25,
                    "fluteLength": 1.0, "overallLength": 2.5, "cornerRadius": 0.0,
                    "spindleRPM": 12000, "clockwise": True,
                },
                "postComments": [],
                "events": [
                    {
                        "type": "onLinear",
                        "sx": 0.0, "sy": 0.0, "sz": 1.0,
                        "x": 1.0, "y": 0.0, "z": 0.5,
                        "feed": 60.0, "movement": 5, "rc": 0, "rpm": 12000,
                        "promote": True,
                    },
                    {
                        "type": "onLinear",
                        "sx": 1.0, "sy": 0.0, "sz": 0.5,
                        "x": 1.0, "y": 0.0, "z": -0.25,
                        "feed": 30.0, "movement": 1, "rc": 0, "rpm": 12000,
                    },
                ],
            }],
        }

        with tempfile.TemporaryDirectory() as tmp_dir:
            out_sbp = os.path.join(tmp_dir, "shopbot_promoted.sbp")
            sbp = orch.replay_combined_context(
                combined_context=ctx,
                base_post_path=shopbot_cps,
                bootstrap_cnc_path=self.milling_cnc,
                final_cnc_path=out_sbp,
            )

            # Standalone full-line ShopBot comment directly above a J3 rapid line, never inline with comma
            self.assertRegex(sbp, r"(?m)^' Rapid restored\r?\nJ3,")
            for line in sbp.splitlines():
                if "Rapid restored" in line:
                    self.assertEqual(line.strip(), "' Rapid restored")
                    self.assertNotIn(",", line)

    # ------------------------------------------------------------------
    # Bug #2: _GetNcProgramExtension defaults empty/missing/non-NC to .cnc
    # ------------------------------------------------------------------

    def test_bug2_get_nc_program_extension_defaults_to_cnc(self):
        import PostClear

        def _make_prog(ext_val):
            p = MagicMock()
            param = MagicMock()
            param.value = MagicMock()
            param.value.value = ext_val
            p.parameters.itemByName.side_effect = lambda name: param if name == "nc_program_nc_extension" else None
            return p

        self.assertEqual(PostClear._GetNcProgramExtension(_make_prog("")), ".cnc")
        self.assertEqual(PostClear._GetNcProgramExtension(_make_prog("   ")), ".cnc")
        self.assertEqual(PostClear._GetNcProgramExtension(_make_prog(None)), ".cnc")
        self.assertEqual(PostClear._GetNcProgramExtension(_make_prog("json")), ".cnc")
        self.assertEqual(PostClear._GetNcProgramExtension(_make_prog(".json")), ".cnc")
        self.assertEqual(PostClear._GetNcProgramExtension(_make_prog("nc")), ".nc")
        self.assertEqual(PostClear._GetNcProgramExtension(_make_prog(".sbp")), ".sbp")

        no_params_prog = MagicMock()
        no_params_prog.parameters = None
        self.assertEqual(PostClear._GetNcProgramExtension(no_params_prog), ".cnc")
        self.assertEqual(PostClear._GetNcProgramExtension(None), ".cnc")

    # ------------------------------------------------------------------
    # Bug #3: dog_leg_warning -> has_warnings and toast warning details
    # ------------------------------------------------------------------

    def test_bug3_dog_leg_warning_propagates_to_warning_status_and_toast(self):
        import PostClear

        captured_html = []

        mock_ui = MagicMock()
        mock_ui.palettes.itemById.return_value = None

        def _capture_palette(_pid, _title, html_path, *_args):
            if html_path.startswith("file://"):
                from urllib.parse import urlparse
                from urllib.request import url2pathname
                html_path = url2pathname(urlparse(html_path).path)
            with open(html_path, "r", encoding="utf-8") as f:
                captured_html.append(f.read())
            return MagicMock()

        mock_ui.palettes.add2.side_effect = _capture_palette

        results = [{
            "setup": "Setup1",
            "cnc": "/tmp/s1.cnc",
            "reportText": "# Report",
            "report": "",
            "png": "",
            "promoted": 0,
            "spikes": 0,
            "volume_mm3": 0.0,
            "kinds": {},
            "warning": "Rapid promotion disabled: MAP_MULTI dog-leg detected",
            "error": None,
            "nc_program": "Prog1",
        }]

        with patch("PostClear.adsk.core.Application.get", return_value=MagicMock()), \
             patch("PostClear.platform.system", return_value="Linux"):
            PostClear._NotifyPostProcessResult(
                ui=mock_ui,
                cntFiles=1,
                cntSkipped=0,
                lstSkipped="",
                outputFolder="/tmp",
                verifyResults=results,
                is_fatal=False,
            )

        self.assertEqual(len(captured_html), 1)
        toast_html = captured_html[0]
        self.assertIn("Post Processing Completed with Warnings", toast_html)
        self.assertIn("MAP_MULTI dog-leg detected", toast_html)
        self.assertNotIn("Completed Successfully", toast_html)

        # Also verify _PostProcessUnifiedPipeline copies report.dog_leg_warning into verifyResults[-1]["warning"]
        saved_results = list(PostClear.verifyResults)
        try:
            PostClear.verifyResults.clear()
            with tempfile.TemporaryDirectory() as tmp_dir:
                out_nc = os.path.join(tmp_dir, "out.nc")
                fake_cps = os.path.join(tmp_dir, "fake.cps")
                with open(fake_cps, "w", encoding="utf-8") as f:
                    f.write("// post")
                mock_setup = MagicMock()
                mock_setup.name = "DogLegSetup"
                mock_setup.operationType = 0
                mock_setup.stockMode = 0
                mock_op = MagicMock()
                mock_op.name = "Op1"
                mock_op.tool = MagicMock(number=1)
                mock_op.isSuppressed = False
                mock_op.hasToolpath = True
                mock_prog = MagicMock()
                mock_prog.name = "Prog1"
                mock_prog.parameters.itemByName.return_value = MagicMock(value=MagicMock(value="fake"))

                fake_ctx = {
                    "unit": 1,
                    "workpiece": {"lower": {"x": -10, "y": -10, "z": -10}, "upper": {"x": 50, "y": 50, "z": 5}},
                    "comments": [],
                    "sections": [{
                        "id": 0, "type": 0,
                        "workPlane": _identity_wp(), "workOrigin": _zero_wo(),
                        "params": {"operation-comment": "Op1"},
                        "tool": {"number": 1, "diameter": 6.0},
                        "events": [],
                    }],
                }

                def _fake_post_process(_target_ops, post_input):
                    with open(os.path.join(post_input.outputFolder, f"{post_input.programName}.json"), "w", encoding="utf-8") as bf:
                        json.dump(fake_ctx, bf)
                    with open(os.path.join(post_input.outputFolder, "bootstrap.intermediate.cnc"), "w", encoding="utf-8") as bcf:
                        bcf.write("; bootstrap\n")
                    return True

                fake_report = VerificationReport(
                    total_machining_time_sec=5.0,
                    total_volume_removed_mm3=10.0,
                    collisions=[],
                    promoted_events=set(),
                    mrr_timeline=[],
                    mrr_spikes=[],
                    dog_leg_warning="Rapid promotion disabled: vendor uses highFeedMapping=MAP_MULTI",
                )

                mock_app = MagicMock()
                mock_doc = MagicMock()
                mock_cam = MagicMock()
                mock_cam.postProcess.side_effect = _fake_post_process
                mock_doc.products.itemByProductType.return_value = mock_cam
                mock_app.activeDocument = mock_doc

                mock_wp = WorkpieceDefinition(
                    stock=AxisymmetricStock2D(z_min=-20.0, z_max=0.0, outer_radius=15.0, inner_radius=0.0)
                )

                def _create_ppi(prog, cfg, out_dir, *_a):
                    m = MagicMock()
                    m.programName = prog
                    m.postConfiguration = cfg
                    m.outputFolder = out_dir
                    return m

                with patch("PostClear.adsk.core.Application.get", return_value=mock_app), \
                     patch("PostClear.adsk.cam.PostProcessInput.create", side_effect=_create_ppi), \
                     patch("PostClear._ResolveBasePostPath", return_value=fake_cps), \
                     patch("PostClear._ExtractWorkpieceFromSetup", return_value=mock_wp), \
                     patch("pipeline.PipelineOrchestrator.run", return_value=(fake_report, "G0 X0\n")):
                    res = PostClear._PostProcessUnifiedPipeline(
                        fname="out",
                        setup=mock_setup,
                        docSettings=dict(PostClear.defaultSettings),
                        program=mock_prog,
                        path=out_nc,
                        fileExt=".nc",
                        target_operations=[mock_op],
                    )
                self.assertIsNone(res)
                self.assertEqual(len(PostClear.verifyResults), 1)
                self.assertIn("MAP_MULTI", PostClear.verifyResults[0]["warning"])
        finally:
            PostClear.verifyResults[:] = saved_results

    # ------------------------------------------------------------------
    # Bug #4: report.promoted_events matches actual replayed G-code count
    # ------------------------------------------------------------------

    def test_bug4_report_promoted_events_matches_replayed_gcode(self):
        if not self.post_bin or not os.path.isfile(self.turning_cnc) or not os.path.isfile(self.centroid_cps):
            self.skipTest("Autodesk post binary or turning fixture not available")

        orch = PipelineOrchestrator(
            dumper_cps=self.dumper_cps,
            replayer_cps=self.replayer_cps,
            post_bin=self.post_bin,
        )
        with tempfile.TemporaryDirectory() as tmp_dir:
            dump_json = os.path.join(tmp_dir, "dump.json")
            orch.dump_intermediate_to_json(self.turning_cnc, self.centroid_cps, dump_json)
            with open(dump_json, "r", encoding="utf-8") as f:
                ctx = json.load(f)
            segments = parse_context_dump(ctx)

            # Pick two long linear air/positioning moves (length >= 2mm so length filter keeps them)
            air_moves = []
            for seg in segments:
                if seg.motion == MotionKind.LINEAR and seg.metadata.get("movement") in (2, 5):
                    dist = math.sqrt(
                        (seg.end.x - seg.start.x) ** 2
                        + (seg.end.y - seg.start.y) ** 2
                        + (seg.end.z - seg.start.z) ** 2
                    )
                    if dist >= 2.0:
                        air_moves.append(seg)
            self.assertGreaterEqual(len(air_moves), 2, "Fixture needs at least 2 linear moves >= 2mm")
            seg_colliding, seg_safe = air_moves[0], air_moves[1]
            ev_colliding = (seg_colliding.metadata["sec_idx"], seg_colliding.metadata["event_idx"])
            ev_safe = (seg_safe.metadata["sec_idx"], seg_safe.metadata["event_idx"])

            # Fake verifier returns BOTH events as promoted, plus a collision matching seg_colliding's (sec_idx, event_idx)
            fake_verifier = MagicMock()
            fake_verifier.verify_toolpath.return_value = VerificationReport(
                total_machining_time_sec=10.0,
                total_volume_removed_mm3=50.0,
                collisions=[
                    CollisionRecord(
                        segment_id=seg_colliding.segment_id,
                        kind=CollisionKind.RAPID_INTO_STOCK,
                        timestamp_sec=1.0,
                        contact_point=Vector3D(0.0, 0.0, 0.0),
                        penetration_volume_mm3=5.0,
                        description="Colliding rapid candidate",
                        sec_idx=ev_colliding[0],
                        event_idx=ev_colliding[1],
                    )
                ],
                promoted_events={ev_colliding, ev_safe},
                mrr_timeline=[],
                mrr_spikes=[],
            )
            wp = WorkpieceDefinition(
                stock=AxisymmetricStock2D(z_min=-63.5, z_max=0.0, outer_radius=15.875, inner_radius=0.0)
            )
            tools_by_op = {}

            out_nc = os.path.join(tmp_dir, "parity.nc")
            report_md = os.path.join(tmp_dir, "parity_report.md")
            report, gcode = orch.run(
                context_dump_paths=[dump_json],
                base_post_path=self.centroid_cps,
                bootstrap_cnc_path=self.turning_cnc,
                workpiece=wp,
                tools_by_op=tools_by_op,
                verifier=fake_verifier,
                final_cnc_path=out_nc,
                report_output_path=report_md,
                promote=True,
            )

            self.assertEqual(report.promoted_events, {ev_safe})
            self.assertEqual(gcode.count(";RAPID RESTORED"), len(report.promoted_events))

            # Also verify promote=False clears report.promoted_events
            out_nc_off = os.path.join(tmp_dir, "parity_off.nc")
            report_off, gcode_off = orch.run(
                context_dump_paths=[dump_json],
                base_post_path=self.centroid_cps,
                bootstrap_cnc_path=self.turning_cnc,
                workpiece=wp,
                tools_by_op=tools_by_op,
                verifier=fake_verifier,
                final_cnc_path=out_nc_off,
                promote=False,
            )
            self.assertEqual(report_off.promoted_events, set())
            self.assertNotIn(";RAPID RESTORED", gcode_off)

    # ------------------------------------------------------------------
    # Bug #5: classify_stock_mode, turning previous-setup chuck, dumper prev/rest
    # ------------------------------------------------------------------

    def test_bug5_classify_stock_mode_and_previous_setup_turning_chuck(self):
        self.assertEqual(classify_stock_mode("prev"), "previous_setup")
        self.assertEqual(classify_stock_mode("previous-setup"), "previous_setup")
        self.assertEqual(classify_stock_mode("rest"), "previous_setup")
        self.assertEqual(classify_stock_mode("From Preceding Setup"), "previous_setup")
        self.assertEqual(classify_stock_mode("solid"), "solid")
        self.assertEqual(classify_stock_mode("fixed-cylinder"), "cylinder")
        self.assertEqual(classify_stock_mode("relative-round"), "cylinder")
        self.assertEqual(classify_stock_mode("fixed-tube"), "tube")
        self.assertEqual(classify_stock_mode("relative-box"), "box")
        self.assertEqual(classify_stock_mode(""), "unknown")
        self.assertEqual(classify_stock_mode(None), "unknown")

        with tempfile.TemporaryDirectory() as tmp_dir:
            stl_path = os.path.join(tmp_dir, "prev_stock.stl")
            _write_binary_stl_cylinder(stl_path, radius=25.0, z_min=-50.0, z_max=0.0)
            for mode in ("prev", "rest"):
                ctx = {
                    "unit": 1,
                    "stockMode": mode,
                    "stockPath": stl_path,
                    "sections": [{"type": 1, "id": 0, "events": []}],
                }
                wp = extract_workpiece_from_context(ctx)
                self.assertIsNotNone(wp)
                self.assertIsInstance(wp.stock, AxisymmetricStock2D)
                self.assertTrue(wp.is_stl_derived)
                self.assertEqual(len(wp.fixtures), 1)
                self.assertIsInstance(wp.fixtures[0], AxisymmetricChuckObstacle)
                self.assertGreaterEqual(wp.stock.outer_radius, 25.0)
                self.assertLessEqual(wp.stock.outer_radius, 25.1)

        # Verify universal_dumper.cps checks prev and rest alongside solid
        with open(self.dumper_cps, "r", encoding="utf-8") as f:
            dumper_src = f.read()
        self.assertIn('sMode.indexOf("prev")', dumper_src)
        self.assertIn('sMode.indexOf("rest")', dumper_src)

    # ------------------------------------------------------------------
    # Bug #6a & #6b: expandManualNC empty expanded & POSTCLEAR_REPLAY_DIAG suppression
    # ------------------------------------------------------------------

    def test_bug6_manual_nc_empty_expanded_and_diag_suppressed_on_clean_frames(self):
        if not self.post_bin or not os.path.isfile(self.milling_cnc):
            self.skipTest("Autodesk post binary or milling_drill_test.cnc not available")

        orch = PipelineOrchestrator(
            dumper_cps=self.dumper_cps,
            replayer_cps=self.replayer_cps,
            post_bin=self.post_bin,
        )

        with tempfile.TemporaryDirectory() as tmp_dir:
            vendor_cps = os.path.join(tmp_dir, "manual_test_vendor.cps")
            with open(vendor_cps, "w", encoding="utf-8") as f:
                f.write(
                    'description = "ManualTestVendor";\n'
                    'vendor = "Test";\n'
                    'certificationLevel = 2;\n'
                    'extension = "nc";\n'
                    'capabilities = CAPABILITY_MILLING;\n'
                    'var fmt = createFormat({decimals: 4});\n'
                    'function onSection() { setRotation(currentSection.workPlane); writeln("O0001"); }\n'
                    'function onManualNC(command, value) { expandManualNC(command, value); }\n'
                    'function onRapid(x, y, z) { writeln("G0 X" + fmt.format(x) + " Y" + fmt.format(y) + " Z" + fmt.format(z)); }\n'
                    'function onLinear(x, y, z, feed) { writeln("G1 X" + fmt.format(x) + " Y" + fmt.format(y) + " Z" + fmt.format(z) + " F" + fmt.format(feed)); }\n'
                    'function onCommand(cmd) { writeln("FALLBACK_ONCOMMAND:" + cmd); }\n'
                    'function onClose() { writeln("M30"); }\n'
                )

            ctx = {
                "unit": 1,
                "programName": "Bug6Test",
                "programComment": "",
                "globalParams": {},
                "comments": [],
                "workpiece": {"lower": {"x": -10, "y": -10, "z": -10}, "upper": {"x": 50, "y": 50, "z": 5}},
                "sections": [{
                    "id": 0,
                    "type": 0,
                    "workOrigin": _zero_wo(),
                    "workPlane": _identity_wp(),
                    "initialPosition": {"x": 0.0, "y": 0.0, "z": 10.0},
                    "finalPosition": {"x": 10.0, "y": 0.0, "z": 10.0},
                    "endPos": {"x": 10.0, "y": 0.0, "z": 10.0},
                    "initSpindleSpeed": 5000,
                    "endSpindleSpeed": 5000,
                    "initMovement": 0,
                    "initRadiusComp": 0,
                    "params": {"operation-comment": "Op1", "operation:zClearance": 10.0},
                    "tool": {"number": 1, "type": 9, "diameter": 6.0, "spindleRPM": 5000, "clockwise": True},
                    "postComments": [],
                    "events": [
                        {"type": "onManualNC", "command": 2, "value": "STOP_EMPTY", "expanded": []},
                        {"type": "onManualNC", "command": 3, "value": "STOP_OMITTED"},
                        {
                            "type": "onLinear",
                            "sx": 0.0, "sy": 0.0, "sz": 10.0,
                            "x": 10.0, "y": 0.0, "z": 10.0,
                            "feed": 1000.0, "movement": 5, "rc": 0, "rpm": 5000,
                            "promote": True,
                        },
                    ],
                }],
            }

            ctx_path = os.path.join(tmp_dir, "ctx.json")
            with open(ctx_path, "w", encoding="utf-8") as f:
                json.dump(ctx, f)

            raw_out_nc = os.path.join(tmp_dir, "raw_replayed.nc")
            # Intentionally point REPLAY_DIAG_JSON to a nonexistent subdir so _writeTextFile fails (wroteDiagFile == False)
            unwritable_diag = os.path.join(tmp_dir, "missing_subdir", "diag.json")
            env = os.environ.copy()
            env["BASE_POST"] = os.path.abspath(vendor_cps)
            env["CONTEXT_JSON"] = os.path.abspath(ctx_path)
            env["REPLAY_DIAG_JSON"] = unwritable_diag

            res = subprocess.run(
                [self.post_bin, "--quiet", self.replayer_cps, self.milling_cnc, raw_out_nc],
                env=env,
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(res.returncode, 0, f"Raw post invocation failed:\n{res.stdout}\n{res.stderr}")
            with open(raw_out_nc, "r", encoding="utf-8") as f:
                raw_gcode = f.read()

            # Bug #6a: expanded: [] MUST be handled without calling onCommand(2);
            # omitted expanded (command 3) MUST still fall through to onCommand(3).
            self.assertNotIn("FALLBACK_ONCOMMAND:2", raw_gcode)
            self.assertIn("FALLBACK_ONCOMMAND:3", raw_gcode)
            # Bug #6b: clean frame check must NOT emit POSTCLEAR_REPLAY_DIAG into raw post output even when diag file wasn't written
            self.assertNotIn("POSTCLEAR_REPLAY_DIAG", raw_gcode)
            # Promoted rapid succeeded normally
            self.assertIn("G0 X", raw_gcode)

            # Also verify replay_combined_context observes diagMissing when diag file cannot be written
            wrapped_out_nc = os.path.join(tmp_dir, "wrapped_replayed.nc")
            with patch.dict(os.environ, {"REPLAY_DIAG_JSON": unwritable_diag}):
                orch.replay_combined_context(
                    combined_context=ctx,
                    base_post_path=vendor_cps,
                    bootstrap_cnc_path=self.milling_cnc,
                    final_cnc_path=wrapped_out_nc,
                )
            self.assertTrue((orch.last_replay_diag or {}).get("diagMissing"))

    # ------------------------------------------------------------------
    # Stage 1 Cycle Cleanup: afterReposMoves skip & __didRepos fallback
    # ------------------------------------------------------------------

    def test_stage1_cycle_after_repos_omitted_and_replayer_fallback(self):
        if not self.post_bin or not os.path.isfile(self.milling_cnc):
            self.skipTest("Autodesk post binary or milling_drill_test.cnc not available")

        orch = PipelineOrchestrator(
            dumper_cps=self.dumper_cps,
            replayer_cps=self.replayer_cps,
            post_bin=self.post_bin,
        )

        with tempfile.TemporaryDirectory() as tmp_dir:
            base_cps = os.path.join(tmp_dir, "cycle_base.cps")
            with open(base_cps, "w", encoding="utf-8") as f:
                f.write(
                    'description = "CycleBase";\n'
                    'vendor = "Test";\n'
                    'certificationLevel = 2;\n'
                    'extension = "nc";\n'
                    'capabilities = CAPABILITY_MILLING;\n'
                    'var fmt = createFormat({decimals: 4});\n'
                    'function onSection() { setRotation(currentSection.workPlane); writeln("O0002"); }\n'
                    'function onRapid(x, y, z) { writeln("G0 X" + fmt.format(x) + " Y" + fmt.format(y) + " Z" + fmt.format(z)); }\n'
                    'function onLinear(x, y, z, feed) { writeln("G1 X" + fmt.format(x) + " Y" + fmt.format(y) + " Z" + fmt.format(z) + " F" + fmt.format(feed)); }\n'
                    'function onCycle() {}\n'
                    'function onCyclePoint(x, y, z) { repositionToCycleClearance(cycle, x, y, z); expandCyclePoint(x, y, z); }\n'
                    'function onCycleEnd() {}\n'
                    'function onClose() { writeln("M30"); }\n'
                )

            # (a) Dumper includes afterReposMoves when reposMoves > 0, and omits it when reposMoves is empty
            dump_with_repos = os.path.join(tmp_dir, "cycle_dump_repos.json")
            orch.dump_intermediate_to_json(self.milling_cnc, base_cps, dump_with_repos)
            with open(dump_with_repos, "r", encoding="utf-8") as f:
                dumped_repos = json.load(f)
            repos_pts = [
                ev
                for sec in dumped_repos.get("sections", [])
                for ev in sec.get("events", [])
                if ev.get("type") == "onCyclePoint"
            ]
            self.assertGreater(len(repos_pts), 0)
            self.assertTrue(any(len(pt.get("reposMoves") or []) > 0 and "afterReposMoves" in pt for pt in repos_pts))

            no_repos_cps = os.path.join(tmp_dir, "cycle_no_repos.cps")
            with open(no_repos_cps, "w", encoding="utf-8") as f:
                f.write(
                    'capabilities = CAPABILITY_MILLING;\n'
                    'repositionToCycleClearance = function() {};\n'
                )
            dump_no_repos = os.path.join(tmp_dir, "cycle_dump_no_repos.json")
            orch.dump_intermediate_to_json(self.milling_cnc, no_repos_cps, dump_no_repos)
            with open(dump_no_repos, "r", encoding="utf-8") as f:
                dumped = json.load(f)

            cycle_pts = [
                ev
                for sec in dumped.get("sections", [])
                for ev in sec.get("events", [])
                if ev.get("type") == "onCyclePoint"
            ]
            self.assertGreater(len(cycle_pts), 0, "Expected cycle points in milling_drill_test.cnc")
            for pt in cycle_pts:
                self.assertEqual(pt.get("reposMoves"), [])
                self.assertNotIn("afterReposMoves", pt)
                self.assertGreater(len(pt.get("expandedMoves") or []), 0)

            # (b) Replayer falls back to expandedMoves when __didRepos is true and afterReposMoves is absent
            ctx = {
                "unit": 1,
                "programName": "CycleFallbackTest",
                "programComment": "",
                "globalParams": {},
                "comments": [],
                "workpiece": {"lower": {"x": -10, "y": -10, "z": -10}, "upper": {"x": 50, "y": 50, "z": 5}},
                "sections": [{
                    "id": 0,
                    "type": 0,
                    "workOrigin": _zero_wo(),
                    "workPlane": _identity_wp(),
                    "initialPosition": {"x": 0.0, "y": 0.0, "z": 15.0},
                    "finalPosition": {"x": 12.0, "y": 24.0, "z": 5.0},
                    "endPos": {"x": 12.0, "y": 24.0, "z": 5.0},
                    "initSpindleSpeed": 4000,
                    "endSpindleSpeed": 4000,
                    "initMovement": 0,
                    "initRadiusComp": 0,
                    "params": {"operation-comment": "Drill1"},
                    "tool": {"number": 2, "type": 3, "diameter": 8.0, "spindleRPM": 4000, "clockwise": True},
                    "postComments": [],
                    "events": [
                        {
                            "type": "onCycle",
                            "cycleType": "drilling",
                            "cycle": {"clearance": 15.0, "retract": 5.0, "depth": 5.0, "feedrate": 250.0},
                            "points": [{"x": 12.0, "y": 24.0, "z": -5.0}],
                        },
                        {
                            "type": "onCyclePoint",
                            "sx": 0.0, "sy": 0.0, "sz": 15.0,
                            "x": 12.0, "y": 24.0, "z": -5.0,
                            "first": True,
                            "last": True,
                            "reposMoves": [{"type": "onRapid", "x": 12.0, "y": 24.0, "z": 15.0, "movement": 0}],
                            "expandedMoves": [
                                {"type": "onRapid", "x": 12.0, "y": 24.0, "z": 5.0, "movement": 0},
                                {"type": "onLinear", "x": 12.0, "y": 24.0, "z": -5.0, "feed": 250.0, "movement": 1},
                            ],
                        },
                        {"type": "onCycleEnd"},
                    ],
                }],
            }

            out_nc = os.path.join(tmp_dir, "cycle_fallback.nc")
            gcode = orch.replay_combined_context(
                combined_context=ctx,
                base_post_path=base_cps,
                bootstrap_cnc_path=self.milling_cnc,
                final_cnc_path=out_nc,
            )
            self.assertIn("G0 X12 Y24 Z15", gcode)
            self.assertIn("G0 X12 Y24 Z5", gcode)
            self.assertIn("G1 X12 Y24 Z-5 F250", gcode)


if __name__ == "__main__":
    unittest.main()
