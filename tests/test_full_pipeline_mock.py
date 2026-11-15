#!/usr/bin/env python3
"""
Test Suite: End-to-End Pipeline Integration with Mock Verification Engine.

Tests:
1. Unified 2-pass pipeline execution:
   Intermediate .cnc -> universal_dumper.cps -> ContextParser -> MockEngine -> universal_replayer.cps -> final .cnc + report.md.
2. Verified restoration of G0 rapid retracts and re-arming of G1 and feedrates in final output.
3. Verification report generation and metric accuracy.
"""

import copy
import json
import os
import tempfile
import unittest
from typing import Dict, List

from geometry.models import (
    MotionSegment,
    MoveEvaluation,
    ToolAssembly,
    LatheToolGeometry,
    ToolCompensationQuadrant,
    AxisymmetricStock2D,
    WorkpieceDefinition,
    VerificationReport,
)
from geometry.engine_interface import IVerificationEngine
from pipeline import PipelineOrchestrator
from shapely.geometry import Polygon


class PipelineMockVerifier(IVerificationEngine):
    """
    Mock engine that flags known clearance retracts as air moves:
    Specifically, any retract where tool moves from outer cut (X > 10 mm)
    inward towards clearance (X ~ 5.969 mm) at constant Z.
    """

    def __init__(self):
        self.moves: List[MotionSegment] = []

    def reset(self, workpiece: WorkpieceDefinition) -> None:
        self.moves = []

    def load_tool(self, tool: ToolAssembly) -> None:
        pass

    def step(self, move: MotionSegment) -> MoveEvaluation:
        self.moves.append(move)
        # Retract move signature in roughing_test fixture:
        # Move starts at X > 10.0 and ends at X < 7.0 at same Z
        is_air = move.start.x > 10.0 and move.end.x < 7.0 and abs(move.start.z - move.end.z) < 1e-3
        return MoveEvaluation(
            segment_id=move.segment_id,
            is_air_move=is_air,
            material_removed_mm3=0.0 if is_air else 25.0,
            instantaneous_mrr=0.0 if is_air else 250.0,
        )

    def generate_report(self) -> VerificationReport:
        promoted_events = set()
        for m in self.moves:
            # Check retract signature
            if m.start.x > 10.0 and m.end.x < 7.0 and abs(m.start.z - m.end.z) < 1e-3:
                sec_idx = m.metadata.get("sec_idx", m.sec_id) if m.metadata else m.sec_id
                ev_idx = m.metadata.get("event_idx", m.segment_id) if m.metadata else m.segment_id
                promoted_events.add((sec_idx, ev_idx))

        return VerificationReport(
            total_machining_time_sec=142.5,
            total_volume_removed_mm3=1250.0,
            collisions=[],
            promoted_events=promoted_events,
            mrr_timeline=[],
            mrr_spikes=[],
        )


class TestFullPipelineMock(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        cls.fixture_cnc = os.path.join(cls.repo_root, "tests", "fixtures", "roughing_test.cnc")
        cls.base_cps = os.path.join(cls.repo_root, "centroid_turning.cps")

        if not os.path.isfile(cls.fixture_cnc) or not os.path.isfile(cls.base_cps):
            raise unittest.SkipTest("Prerequisites not found for pipeline test")

    def test_end_to_end_pipeline_with_mock_engine(self):
        """Execute complete pipeline from intermediate .cnc to final .cnc and report."""
        orchestrator = PipelineOrchestrator()

        stock = AxisymmetricStock2D(z_min=-2.5, z_max=0.0, outer_radius=0.625)
        workpiece = WorkpieceDefinition(stock=stock)

        dummy_poly = Polygon([(0, 0), (1, 0), (1, 1), (0, 1)])
        tool6 = ToolAssembly(
            tool_id=6,
            name="Profile Boring Tool",
            geometry=LatheToolGeometry(
                insert_shape="C",
                nose_radius=0.008,
                hand="R",
                quadrant=ToolCompensationQuadrant.P3,
                cutter_polygon=dummy_poly,
                holder_polygon=dummy_poly,
            ),
        )
        tools_by_op = {
            "Profile Roughing8": tool6,
            "Profile Roughing8 (2)": tool6,
            "Profile Finishing6": tool6,
        }

        verifier = PipelineMockVerifier()

        with tempfile.TemporaryDirectory() as tmp_dir:
            out_cnc = os.path.join(tmp_dir, "final_machined.cnc")
            out_report = os.path.join(tmp_dir, "verification_report.md")

            report, gcode = orchestrator.run(
                intermediate_cnc_path=self.fixture_cnc,
                base_post_path=self.base_cps,
                workpiece=workpiece,
                tools_by_op=tools_by_op,
                verifier=verifier,
                final_cnc_path=out_cnc,
                report_output_path=out_report,
                unit=1,
            )

            # 1. Assertions on VerificationReport
            self.assertEqual(len(report.collisions), 0, "Expected zero collisions")
            self.assertGreater(len(report.promoted_events), 0, "Expected air moves to be promoted")

            # 2. Assertions on Final G-Code
            self.assertTrue(os.path.isfile(out_cnc))
            self.assertIn("G21", gcode)
            self.assertIn(";RAPID RESTORED\nN32 G0 X11.938", gcode, "Expected promoted G0 move with standalone comment above block N32")
            self.assertIn("N33 G1 Z-1.524 F0.0508", gcode, "Expected restored modal G1 on block N33")
            self.assertIn(";UNIVERSAL WRAPPER RESTORED", gcode)

            # 3. Assertions on Markdown Report
            self.assertTrue(os.path.isfile(out_report))
            with open(out_report, "r") as f:
                report_content = f.read()

            self.assertIn("# Toolpath Verification & Rapid-Promotion Report", report_content)
            self.assertIn("Zero collisions detected", report_content)
            self.assertNotIn("Rapid-Promotion Authorization Rules", report_content)

    def test_end_to_end_pipeline_no_promotions_omits_trailer(self):
        """
        Verify that when rapid promotion is disabled (promote=False) or no moves are restored,
        the replayer does NOT emit the UNIVERSAL WRAPPER RESTORED trailer comment.
        """
        with tempfile.TemporaryDirectory() as tmp_dir:
            out_cnc = os.path.join(tmp_dir, "final_unpromoted.cnc")
            out_report = os.path.join(tmp_dir, "report_unpromoted.md")

            tools_by_op = {
                "Profile Roughing": ToolAssembly(
                    tool_id=1,
                    name="Roughing Tool",
                    geometry=LatheToolGeometry(
                        insert_shape="W",
                        nose_radius=0.8,
                        hand="R",
                        quadrant=ToolCompensationQuadrant.P3,
                        cutter_polygon=Polygon([(0, 0), (5, 0), (5, 5), (0, 5)]),
                        holder_polygon=Polygon([(2, 1), (2, 4), (10, 4), (10, 1)]),
                    ),
                )
            }
            workpiece = WorkpieceDefinition(
                stock=AxisymmetricStock2D(z_min=-50.0, z_max=0.0, outer_radius=20.0, inner_radius=0.0)
            )

            orchestrator = PipelineOrchestrator(
                dumper_cps=os.path.join(self.repo_root, "universal_dumper.cps"),
                replayer_cps=os.path.join(self.repo_root, "universal_replayer.cps"),
            )
            verifier = PipelineMockVerifier()

            report, gcode = orchestrator.run(
                intermediate_cnc_path=self.fixture_cnc,
                base_post_path=self.base_cps,
                workpiece=workpiece,
                tools_by_op=tools_by_op,
                verifier=verifier,
                final_cnc_path=out_cnc,
                report_output_path=out_report,
                promote=False,
            )

            self.assertTrue(os.path.isfile(out_cnc))
            # Trailer comment MUST NOT be present when restoredCount is 0
            self.assertNotIn(";UNIVERSAL WRAPPER RESTORED", gcode)

    def test_pipeline_dog_leg_rapid_mode_multi_disables_promotion(self):
        """(a) context with rapidMode.highFeedMapping = MAP_MULTI gives zero promotions and report warning."""
        with tempfile.TemporaryDirectory() as tmp_dir:
            orchestrator = PipelineOrchestrator()
            stock = AxisymmetricStock2D(z_min=-2.5, z_max=0.0, outer_radius=0.625)
            workpiece = WorkpieceDefinition(stock=stock)
            dummy_poly = Polygon([(0, 0), (1, 0), (1, 1), (0, 1)])
            tool6 = ToolAssembly(
                tool_id=6,
                name="Profile Boring Tool",
                geometry=LatheToolGeometry(
                    insert_shape="C",
                    nose_radius=0.008,
                    hand="R",
                    quadrant=ToolCompensationQuadrant.P3,
                    cutter_polygon=dummy_poly,
                    holder_polygon=dummy_poly,
                ),
            )
            tools_by_op = {"Profile Roughing8": tool6}
            verifier = PipelineMockVerifier()

            dump_p = os.path.join(tmp_dir, "batch_0.json")
            orchestrator.dump_intermediate_to_json(self.fixture_cnc, self.base_cps, dump_p)
            with open(dump_p, "r", encoding="utf-8") as f:
                ctx = json.load(f)
            ctx["rapidMode"] = {"highFeedMapping": 1, "useG0": None}
            with open(dump_p, "w", encoding="utf-8") as f:
                json.dump(ctx, f)

            out_cnc = os.path.join(tmp_dir, "dog_leg_multi.cnc")
            out_report = os.path.join(tmp_dir, "dog_leg_multi_report.md")

            report, gcode = orchestrator.run(
                context_dump_paths=[dump_p],
                base_post_path=self.base_cps,
                workpiece=workpiece,
                tools_by_op=tools_by_op,
                verifier=verifier,
                final_cnc_path=out_cnc,
                report_output_path=out_report,
                bootstrap_cnc_path=self.fixture_cnc,
            )

            # Verification still runs:
            self.assertEqual(len(report.collisions), 0)
            # Promotions must be 0:
            self.assertEqual(len(report.promoted_events), 0)
            self.assertIsNotNone(report.dog_leg_warning)
            expected_msg = "Rapid promotion disabled: post High feedrate mapping = 1 / useG0=false (controller may dog-leg rapids)"
            self.assertEqual(report.dog_leg_warning, expected_msg)

            # Report markdown must contain the notice:
            with open(out_report, "r", encoding="utf-8") as f:
                report_md = f.read()
            self.assertIn(expected_msg, report_md)

            # G-code must not have rapid restored comments:
            self.assertNotIn(";RAPID RESTORED", gcode)
            self.assertNotIn(";UNIVERSAL WRAPPER RESTORED", gcode)

    def test_pipeline_dog_leg_rapid_mode_no_mapping_allows_promotion(self):
        """(b) context with rapidMode.highFeedMapping = NO_MAPPING allows normal promotion."""
        with tempfile.TemporaryDirectory() as tmp_dir:
            orchestrator = PipelineOrchestrator()
            stock = AxisymmetricStock2D(z_min=-2.5, z_max=0.0, outer_radius=0.625)
            workpiece = WorkpieceDefinition(stock=stock)
            dummy_poly = Polygon([(0, 0), (1, 0), (1, 1), (0, 1)])
            tool6 = ToolAssembly(
                tool_id=6,
                name="Profile Boring Tool",
                geometry=LatheToolGeometry(
                    insert_shape="C",
                    nose_radius=0.008,
                    hand="R",
                    quadrant=ToolCompensationQuadrant.P3,
                    cutter_polygon=dummy_poly,
                    holder_polygon=dummy_poly,
                ),
            )
            tools_by_op = {"Profile Roughing8": tool6}
            verifier = PipelineMockVerifier()

            dump_p = os.path.join(tmp_dir, "batch_0.json")
            orchestrator.dump_intermediate_to_json(self.fixture_cnc, self.base_cps, dump_p)
            with open(dump_p, "r", encoding="utf-8") as f:
                ctx = json.load(f)
            ctx["rapidMode"] = {"highFeedMapping": 0, "useG0": True}
            with open(dump_p, "w", encoding="utf-8") as f:
                json.dump(ctx, f)

            out_cnc = os.path.join(tmp_dir, "no_mapping.cnc")
            out_report = os.path.join(tmp_dir, "no_mapping_report.md")

            report, gcode = orchestrator.run(
                context_dump_paths=[dump_p],
                base_post_path=self.base_cps,
                workpiece=workpiece,
                tools_by_op=tools_by_op,
                verifier=verifier,
                final_cnc_path=out_cnc,
                report_output_path=out_report,
                bootstrap_cnc_path=self.fixture_cnc,
            )

            self.assertIsNone(report.dog_leg_warning)
            self.assertGreater(len(report.promoted_events), 0)
            self.assertIn(";RAPID RESTORED", gcode)

    def test_pipeline_dog_leg_rapid_mode_use_g0_false_disables_promotion(self):
        """(c) context with rapidMode.useG0 = False gives zero promotions and report warning."""
        with tempfile.TemporaryDirectory() as tmp_dir:
            orchestrator = PipelineOrchestrator()
            stock = AxisymmetricStock2D(z_min=-2.5, z_max=0.0, outer_radius=0.625)
            workpiece = WorkpieceDefinition(stock=stock)
            dummy_poly = Polygon([(0, 0), (1, 0), (1, 1), (0, 1)])
            tool6 = ToolAssembly(
                tool_id=6,
                name="Profile Boring Tool",
                geometry=LatheToolGeometry(
                    insert_shape="C",
                    nose_radius=0.008,
                    hand="R",
                    quadrant=ToolCompensationQuadrant.P3,
                    cutter_polygon=dummy_poly,
                    holder_polygon=dummy_poly,
                ),
            )
            tools_by_op = {"Profile Roughing8": tool6}
            verifier = PipelineMockVerifier()

            dump_p = os.path.join(tmp_dir, "batch_0.json")
            orchestrator.dump_intermediate_to_json(self.fixture_cnc, self.base_cps, dump_p)
            with open(dump_p, "r", encoding="utf-8") as f:
                ctx = json.load(f)
            ctx["rapidMode"] = {"highFeedMapping": 0, "useG0": False}
            with open(dump_p, "w", encoding="utf-8") as f:
                json.dump(ctx, f)

            out_cnc = os.path.join(tmp_dir, "use_g0_false.cnc")
            out_report = os.path.join(tmp_dir, "use_g0_false_report.md")

            report, gcode = orchestrator.run(
                context_dump_paths=[dump_p],
                base_post_path=self.base_cps,
                workpiece=workpiece,
                tools_by_op=tools_by_op,
                verifier=verifier,
                final_cnc_path=out_cnc,
                report_output_path=out_report,
                bootstrap_cnc_path=self.fixture_cnc,
            )

            self.assertEqual(len(report.promoted_events), 0)
            self.assertIsNotNone(report.dog_leg_warning)
            expected_msg = "Rapid promotion disabled: post High feedrate mapping = 0 / useG0=false (controller may dog-leg rapids)"
            self.assertEqual(report.dog_leg_warning, expected_msg)

            with open(out_report, "r", encoding="utf-8") as f:
                report_md = f.read()
            self.assertIn(expected_msg, report_md)
            self.assertNotIn(";RAPID RESTORED", gcode)

    def test_pipeline_dog_leg_rapid_mode_missing_retains_promotion(self):
        """(d) context without rapidMode key (legacy dumps) keeps promotion enabled."""
        with tempfile.TemporaryDirectory() as tmp_dir:
            orchestrator = PipelineOrchestrator()
            stock = AxisymmetricStock2D(z_min=-2.5, z_max=0.0, outer_radius=0.625)
            workpiece = WorkpieceDefinition(stock=stock)
            dummy_poly = Polygon([(0, 0), (1, 0), (1, 1), (0, 1)])
            tool6 = ToolAssembly(
                tool_id=6,
                name="Profile Boring Tool",
                geometry=LatheToolGeometry(
                    insert_shape="C",
                    nose_radius=0.008,
                    hand="R",
                    quadrant=ToolCompensationQuadrant.P3,
                    cutter_polygon=dummy_poly,
                    holder_polygon=dummy_poly,
                ),
            )
            tools_by_op = {"Profile Roughing8": tool6}
            verifier = PipelineMockVerifier()

            dump_p = os.path.join(tmp_dir, "batch_0.json")
            orchestrator.dump_intermediate_to_json(self.fixture_cnc, self.base_cps, dump_p)
            with open(dump_p, "r", encoding="utf-8") as f:
                ctx = json.load(f)
            ctx.pop("rapidMode", None)
            with open(dump_p, "w", encoding="utf-8") as f:
                json.dump(ctx, f)

            out_cnc = os.path.join(tmp_dir, "legacy_dump.cnc")
            out_report = os.path.join(tmp_dir, "legacy_dump_report.md")

            report, gcode = orchestrator.run(
                context_dump_paths=[dump_p],
                base_post_path=self.base_cps,
                workpiece=workpiece,
                tools_by_op=tools_by_op,
                verifier=verifier,
                final_cnc_path=out_cnc,
                report_output_path=out_report,
                bootstrap_cnc_path=self.fixture_cnc,
            )

            self.assertIsNone(report.dog_leg_warning)
            self.assertGreater(len(report.promoted_events), 0)
            self.assertIn(";RAPID RESTORED", gcode)


class TestUniversalReplayerPromotions(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        cls.fixture_cnc = os.path.join(cls.repo_root, "tests", "fixtures", "roughing_test.cnc")
        cls.base_cps = os.path.join(cls.repo_root, "centroid_turning.cps")
        cls.orchestrator = PipelineOrchestrator(
            dumper_cps=os.path.join(cls.repo_root, "universal_dumper.cps"),
            replayer_cps=os.path.join(cls.repo_root, "universal_replayer.cps"),
        )
        if not os.path.isfile(cls.fixture_cnc) or not os.path.isfile(cls.base_cps):
            raise unittest.SkipTest("Prerequisites not found for replayer test")

    def test_replayer_promotion_via_ev_promote_and_trailer_emission(self):
        """
        Verify that:
        1. When ev.promote === true, universal_replayer promotes onLinear to rapid
           and emits the UNIVERSAL WRAPPER RESTORED trailer comment.
        2. When restoredCount === 0, no trailer comment is emitted.
        """
        with tempfile.TemporaryDirectory() as tmp_dir:
            b0_path = os.path.join(tmp_dir, "batch_0.json")
            self.orchestrator.dump_intermediate_to_json(self.fixture_cnc, self.base_cps, b0_path)
            with open(b0_path, "r", encoding="utf-8") as f:
                b0_ctx = json.load(f)

            # Subtest 1: With ev.promote = True on a moving linear event
            ctx_promoted = copy.deepcopy(b0_ctx)
            promoted_linear = None
            for ev in ctx_promoted["sections"][0]["events"]:
                if ev["type"] == "onLinear" and (ev["x"] != ev["sx"] or ev["y"] != ev["sy"] or ev["z"] != ev["sz"]):
                    ev["promote"] = True
                    promoted_linear = ev
                    break
            self.assertIsNotNone(promoted_linear)

            out_promoted = os.path.join(tmp_dir, "replayer_promoted.nc")
            gcode_prom = self.orchestrator.replay_combined_context(
                combined_context=ctx_promoted,
                base_post_path=self.base_cps,
                bootstrap_cnc_path=self.fixture_cnc,
                final_cnc_path=out_promoted,
            )
            self.assertIn(";RAPID RESTORED", gcode_prom)
            self.assertIn(";UNIVERSAL WRAPPER RESTORED 1 G0 RAPID MOVES", gcode_prom)

            # Subtest 2: Without ev.promote (restoredCount == 0)
            ctx_unpromoted = copy.deepcopy(b0_ctx)
            for ev in ctx_unpromoted["sections"][0]["events"]:
                ev.pop("promote", None)

            out_unpromoted = os.path.join(tmp_dir, "replayer_unpromoted.nc")
            gcode_unprom = self.orchestrator.replay_combined_context(
                combined_context=ctx_unpromoted,
                base_post_path=self.base_cps,
                bootstrap_cnc_path=self.fixture_cnc,
                final_cnc_path=out_unpromoted,
            )
            self.assertNotIn(";RAPID RESTORED", gcode_unprom)
            self.assertNotIn(";UNIVERSAL WRAPPER RESTORED", gcode_unprom)

    def test_replayer_sync_threading_move_never_promoted(self):
        """
        Verify that an onLinear move with sync=True (e.g. threading pass in air)
        is NEVER promoted by universal_replayer, even if ev.promote === true
        and even if an external PROMOTED_MOVES_JSON map or env var is set.
        """
        with tempfile.TemporaryDirectory() as tmp_dir:
            b0_path = os.path.join(tmp_dir, "batch_0.json")
            self.orchestrator.dump_intermediate_to_json(self.fixture_cnc, self.base_cps, b0_path)
            with open(b0_path, "r", encoding="utf-8") as f:
                b0_ctx = json.load(f)

            ctx_sync = copy.deepcopy(b0_ctx)
            ctx_sync["sections"][0]["params"]["operation:threadPitch"] = 1.5
            target_ev = None
            for ev in ctx_sync["sections"][0]["events"]:
                if ev["type"] == "onLinear" and (ev["x"] != ev["sx"] or ev["y"] != ev["sy"] or ev["z"] != ev["sz"]):
                    ev["sync"] = True
                    ev["promote"] = True
                    target_ev = ev
                    break
            self.assertIsNotNone(target_ev)

            # Plant an old-style PROMOTED_MOVES_JSON file
            map_file = os.path.join(tmp_dir, "promoted_map.json")
            sx_i = round(float(target_ev["sx"]) * 10000)
            sy_i = round(float(target_ev["sy"]) * 10000)
            sz_i = round(float(target_ev["sz"]) * 10000)
            ex_i = round(float(target_ev["x"]) * 10000)
            ey_i = round(float(target_ev["y"]) * 10000)
            ez_i = round(float(target_ev["z"]) * 10000)
            k = f"0:{sx_i},{sy_i},{sz_i}->{ex_i},{ey_i},{ez_i}"
            with open(map_file, "w") as f:
                json.dump({k: {"minOccur": 1}}, f)

            out_nc = os.path.join(tmp_dir, "sync_test.nc")
            old_env = os.environ.get("PROMOTED_MOVES_JSON")
            os.environ["PROMOTED_MOVES_JSON"] = map_file
            try:
                gcode = self.orchestrator.replay_combined_context(
                    combined_context=ctx_sync,
                    base_post_path=self.base_cps,
                    bootstrap_cnc_path=self.fixture_cnc,
                    final_cnc_path=out_nc,
                )
            finally:
                if old_env is None:
                    os.environ.pop("PROMOTED_MOVES_JSON", None)
                else:
                    os.environ["PROMOTED_MOVES_JSON"] = old_env

            # Move MUST NOT be promoted to rapid; it emits G32 threading move
            self.assertNotIn(";RAPID RESTORED", gcode)
            self.assertNotIn(";UNIVERSAL WRAPPER RESTORED", gcode)
            self.assertIn("G32", gcode)

    def test_replayer_safety_net_blocks_promotion_when_dog_leg_active(self):
        """
        Verify that universal_replayer safety net detects dog-leg rapids from the vendor post
        (e.g. highFeedMapping != 0 or useG0 == false) after vendor onOpen(), suppresses promotion
        of ev.promote moves, does not increment restoredCount, and logs a comment explaining why.
        """
        with tempfile.TemporaryDirectory() as tmp_dir:
            b0_path = os.path.join(tmp_dir, "batch_0.json")
            self.orchestrator.dump_intermediate_to_json(self.fixture_cnc, self.base_cps, b0_path)
            with open(b0_path, "r", encoding="utf-8") as f:
                b0_ctx = json.load(f)

            ctx_promoted = copy.deepcopy(b0_ctx)
            target_ev = None
            for ev in ctx_promoted["sections"][0]["events"]:
                if ev["type"] == "onLinear" and (ev["x"] != ev["sx"] or ev["y"] != ev["sy"] or ev["z"] != ev["sz"]):
                    ev["promote"] = True
                    target_ev = ev
                    break
            self.assertIsNotNone(target_ev)

            # Create a wrapped vendor post that activates dog-leg rapids in onOpen()
            dog_leg_post = os.path.join(tmp_dir, "dog_leg_vendor.cps")
            with open(dog_leg_post, "w", encoding="utf-8") as f:
                f.write(f'include("{self.base_cps}");\n')
                f.write('var _origOnOpen = onOpen;\n')
                f.write('onOpen = function() {\n')
                f.write('  if (_origOnOpen) _origOnOpen();\n')
                f.write('  highFeedMapping = 1; // HIGH_FEED_MAP_MULTI\n')
                f.write('};\n')

            out_nc = os.path.join(tmp_dir, "dog_leg_replayer_test.nc")
            gcode = self.orchestrator.replay_combined_context(
                combined_context=ctx_promoted,
                base_post_path=dog_leg_post,
                bootstrap_cnc_path=self.fixture_cnc,
                final_cnc_path=out_nc,
            )

            # Promotion must be suppressed:
            self.assertNotIn(";RAPID RESTORED", gcode)
            self.assertNotIn(";UNIVERSAL WRAPPER RESTORED", gcode)
            # Replayer safety net must log comment explaining why:
            self.assertIn("rapid promotion disabled", gcode.lower())
            self.assertIn("dog-leg rapids", gcode.lower())


if __name__ == "__main__":
    unittest.main()
