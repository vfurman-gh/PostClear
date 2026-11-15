#!/usr/bin/env python3
"""
Unit and regression test suite for context_parser promotion gating and radius compensation handling:
- Item 1 (C1): Remove cross-section origSecId fallback key regression test.
- Item 2 (H1): G1 moves under radius compensation (rc != 0) and transition moves are never promoted.
"""

import unittest
from context_parser import (
    apply_promotions_to_context,
    merge_context_dumps,
    parse_context_dump,
)


class TestContextParserPromotionsC1(unittest.TestCase):
    """
    Regression test for C1: Verify cross-section promotion aliasing via origSecId fallback is eliminated.
    """

    def test_orig_sec_id_fallback_does_not_promote_unauthorized_section(self):
        """
        Two batch dumps (unit=1), each with one section containing identical onLinear move.
        Rules authorize only section 0's move. Section 1 must NOT be promoted via fallback to origSecId.
        """
        mv = {"type": "onLinear", "sx": 0, "sy": 0, "sz": 5, "x": 10, "y": 0, "z": 5, "feed": 100}
        b0 = {"unit": 1, "sections": [{"events": [dict(mv)]}]}  # section 0
        b1 = {"unit": 1, "sections": [{"events": [dict(mv)]}]}  # section 1

        ctx = merge_context_dumps([b0, b1])
        self.assertEqual(len(ctx["sections"]), 2)
        self.assertEqual(ctx["sections"][0].get("id"), 0)
        self.assertEqual(ctx["sections"][1].get("id"), 1)

        promoted_events = {(0, 0)}
        promoted_count = apply_promotions_to_context(ctx, promoted_events)

        self.assertEqual(promoted_count, 1)
        self.assertTrue(ctx["sections"][0]["events"][0].get("promote"))
        self.assertFalse(ctx["sections"][1]["events"][0].get("promote", False))


class TestContextParserRadiusCompensationH1(unittest.TestCase):
    """
    Validation and defense-in-depth tests for H1:
    - parse_context_dump marks rc_transition on the linear following onRadiusCompensation,
      and on the first linear after rc returns to 0.
    - apply_promotions_to_context never promotes an event with rc != 0 even if a rule matches.
    """

    def test_parse_context_dump_marks_rc_and_rc_transition(self):
        """
        parse_context_dump puts rc into metadata, marks rc_transition on the linear
        following onRadiusCompensation, and marks the first linear after rc returns to 0 as rc_transition.
        """
        raw_ctx = {
            "unit": 1,
            "sections": [
                {
                    "id": 0,
                    "events": [
                        {"type": "onRapid", "sx": 0, "sy": 0, "sz": 10, "x": 0, "y": 0, "z": 10},
                        {"type": "onRadiusCompensation", "rc": 1},
                        {"type": "onLinear", "sx": 0, "sy": 0, "sz": 10, "x": 10, "y": 0, "z": 10, "feed": 100, "rc": 1},
                        {"type": "onLinear", "sx": 10, "sy": 0, "sz": 10, "x": 20, "y": 0, "z": 10, "feed": 100, "rc": 1},
                        {"type": "onRadiusCompensation", "rc": 0},
                        {"type": "onLinear", "sx": 20, "sy": 0, "sz": 10, "x": 30, "y": 0, "z": 10, "feed": 100, "rc": 0},
                        {"type": "onLinear", "sx": 30, "sy": 0, "sz": 10, "x": 40, "y": 0, "z": 10, "feed": 100, "rc": 0},
                    ],
                }
            ],
        }

        segments = parse_context_dump(raw_ctx)
        # 1 rapid + 4 linear = 5 segments
        linear_segs = [s for s in segments if s.metadata.get("sec_type") is not None or "rc" in s.metadata]
        self.assertEqual(len(linear_segs), 4)

        # Move 1: First linear after onRadiusCompensation (activation transition)
        self.assertEqual(linear_segs[0].metadata.get("rc"), 1)
        self.assertTrue(linear_segs[0].metadata.get("rc_transition"))

        # Move 2: Second linear with rc=1 (steady state under compensation)
        self.assertEqual(linear_segs[1].metadata.get("rc"), 1)
        self.assertFalse(linear_segs[1].metadata.get("rc_transition"))

        # Move 3: First linear after onRadiusCompensation returning to 0 (deactivation transition)
        self.assertEqual(linear_segs[2].metadata.get("rc"), 0)
        self.assertTrue(linear_segs[2].metadata.get("rc_transition"))

        # Move 4: Subsequent linear with rc=0 (steady state with compensation off)
        self.assertEqual(linear_segs[3].metadata.get("rc"), 0)
        self.assertFalse(linear_segs[3].metadata.get("rc_transition"))

    def test_parse_context_dump_marks_transition_when_rc_drops_to_zero_without_event(self):
        """
        If onRadiusCompensation event is absent but linear rc drops from 1 to 0,
        the first linear with rc=0 is marked rc_transition.
        """
        raw_ctx = {
            "unit": 1,
            "sections": [
                {
                    "id": 0,
                    "events": [
                        {"type": "onLinear", "sx": 0, "sy": 0, "sz": 10, "x": 10, "y": 0, "z": 10, "feed": 100, "rc": 1},
                        {"type": "onLinear", "sx": 10, "sy": 0, "sz": 10, "x": 20, "y": 0, "z": 10, "feed": 100, "rc": 0},
                        {"type": "onLinear", "sx": 20, "sy": 0, "sz": 10, "x": 30, "y": 0, "z": 10, "feed": 100, "rc": 0},
                    ],
                }
            ],
        }

        segments = parse_context_dump(raw_ctx)
        self.assertEqual(len(segments), 3)
        self.assertEqual(segments[0].metadata.get("rc"), 1)
        self.assertEqual(segments[1].metadata.get("rc"), 0)
        self.assertTrue(segments[1].metadata.get("rc_transition"))
        self.assertEqual(segments[2].metadata.get("rc"), 0)
        self.assertFalse(segments[2].metadata.get("rc_transition"))

    def test_apply_promotions_to_context_skips_rc_active_moves(self):
        """
        apply_promotions_to_context does NOT promote an event with rc != 0
        even if its (sec_idx, event_idx) is in promoted_events.
        """
        mv_with_rc = {
            "type": "onLinear",
            "sx": 0,
            "sy": 0,
            "sz": 5,
            "x": 10,
            "y": 0,
            "z": 5,
            "feed": 100,
            "rc": 1,
        }
        ctx = {"unit": 1, "sections": [{"id": 0, "events": [mv_with_rc]}]}
        promoted_events = {(0, 0)}

        promoted_count = apply_promotions_to_context(ctx, promoted_events)
        self.assertEqual(promoted_count, 0)
        self.assertFalse(ctx["sections"][0]["events"][0].get("promote", False))

    def test_apply_promotions_per_event_rc_guard(self):
        """
        Verify per-event promotion gating: event #1 has rc=1 (blocked), event #2 has rc=0 (promoted).
        When promoted_events = {(0, 0), (0, 1)}, apply_promotions_to_context returns 1:
        event #2 is promoted, event #1 is skipped due to rc!=0 guard.
        """
        mv_rc1 = {
            "type": "onLinear",
            "sx": 0,
            "sy": 0,
            "sz": 5,
            "x": 10,
            "y": 0,
            "z": 5,
            "feed": 100,
            "rc": 1,
        }
        mv_rc0 = {
            "type": "onLinear",
            "sx": 0,
            "sy": 0,
            "sz": 5,
            "x": 10,
            "y": 0,
            "z": 5,
            "feed": 100,
            "rc": 0,
        }
        ctx = {"unit": 1, "sections": [{"id": 0, "events": [mv_rc1, mv_rc0]}]}
        promoted_events = {(0, 0), (0, 1)}

        promoted_count = apply_promotions_to_context(ctx, promoted_events)
        self.assertEqual(promoted_count, 1)
        self.assertFalse(ctx["sections"][0]["events"][0].get("promote", False))
        self.assertTrue(ctx["sections"][0]["events"][1].get("promote"))


class TestMergeContextDumpsRapidMode(unittest.TestCase):
    """
    Validation tests for merge_context_dumps preserving rapidMode.
    """

    def test_merge_propagates_rapid_mode_from_batch_0(self):
        b0 = {"unit": 1, "rapidMode": {"highFeedMapping": 1, "useG0": False}, "sections": []}
        b1 = {"unit": 1, "rapidMode": {"highFeedMapping": 0, "useG0": True}, "sections": []}
        merged = merge_context_dumps([b0, b1])
        self.assertEqual(merged.get("rapidMode"), {"highFeedMapping": 1, "useG0": False})

    def test_merge_propagates_rapid_mode_from_later_batch_if_absent_in_b0(self):
        b0 = {"unit": 1, "sections": []}
        b1 = {"unit": 1, "rapidMode": {"highFeedMapping": 5, "useG0": None}, "sections": []}
        merged = merge_context_dumps([b0, b1])
        self.assertEqual(merged.get("rapidMode"), {"highFeedMapping": 5, "useG0": None})

    def test_merge_omits_rapid_mode_if_absent_in_all(self):
        b0 = {"unit": 1, "sections": []}
        b1 = {"unit": 1, "sections": []}
        merged = merge_context_dumps([b0, b1])
        self.assertNotIn("rapidMode", merged)


class TestCallerOnlyPromotionLengthFilter(unittest.TestCase):
    """
    Tests for caller-only 1.5 mm promotion length filtering (`filter_promoted_events_by_length`
    and `PipelineOrchestrator.run()`).
    """

    def test_01_engines_do_not_filter_by_length_and_deprecation_warnings(self):
        """Verify engines mark 0.25 mm air moves as air and warn on deprecated min_promote_len_mm."""
        import warnings
        from shapely.geometry import box
        from context_parser import DEFAULT_MIN_PROMOTE_LEN_MM, filter_promoted_events_by_length
        from geometry.heightfield_engine import MillHeightfieldVerifier
        from geometry.lathe_engine import Lathe2DIPWVerifier
        from geometry.models import (
            AxisymmetricStock2D,
            BoxStock3D,
            LatheToolGeometry,
            MillToolGeometry,
            MotionKind,
            MotionSegment,
            ToolAssembly,
            ToolCompensationQuadrant,
            Vector3D,
            WorkpieceDefinition,
        )

        self.assertAlmostEqual(DEFAULT_MIN_PROMOTE_LEN_MM, 1.5)

        wp = WorkpieceDefinition(stock=BoxStock3D(-10.0, 10.0, -10.0, 10.0, -10.0, 0.0))
        tool = ToolAssembly(tool_id=1, name="EM", geometry=MillToolGeometry(diameter=6.0, corner_radius=0.0, flute_length=15.0, overall_length=40.0))
        verifier = MillHeightfieldVerifier()
        verifier.reset(wp)
        verifier.load_tool(tool)
        p1, p2 = Vector3D(0.0, 0.0, 10.0), Vector3D(0.25, 0.0, 10.0)
        short_seg = MotionSegment(
            segment_id=0,
            op_id="op1",
            sec_id=0,
            motion=MotionKind.LINEAR,
            start=p1,
            end=p2,
            feedrate_mmpm=500.0,
            spindle_rpm=5000.0,
            duration_sec=0.03,
            metadata={"sec_idx": 0, "event_idx": 0},
        )
        self.assertTrue(verifier.step(short_seg).is_air_move)
        self.assertIn((0, 0), verifier.promoted_events)

        wp_lathe = WorkpieceDefinition(stock=AxisymmetricStock2D(z_min=-20.0, z_max=0.0, outer_radius=10.0))
        tool_lathe = ToolAssembly(
            tool_id=2,
            name="Turn",
            geometry=LatheToolGeometry(
                insert_shape="C",
                nose_radius=0.4,
                hand="R",
                quadrant=ToolCompensationQuadrant.P3,
                cutter_polygon=box(0, 0, 5, 5),
                holder_polygon=box(2, 2, 15, 30),
            ),
        )
        lathe_v = Lathe2DIPWVerifier()
        lathe_v.reset(wp_lathe)
        lathe_v.load_tool(tool_lathe)
        lp1, lp2 = Vector3D(15.0, 0.0, 0.0), Vector3D(15.0, 0.0, -0.25)
        lathe_short = MotionSegment(
            segment_id=0,
            op_id="opL",
            sec_id=0,
            motion=MotionKind.LINEAR,
            start=lp1,
            end=lp2,
            feedrate_mmpm=200.0,
            spindle_rpm=1500.0,
            duration_sec=0.03,
            metadata={"sec_idx": 0, "event_idx": 0},
        )
        self.assertTrue(lathe_v.step(lathe_short).is_air_move)
        self.assertIn((0, 0), lathe_v.promoted_events)

        with warnings.catch_warnings(record=True) as w_list:
            warnings.simplefilter("always", DeprecationWarning)
            m_dep = MillHeightfieldVerifier(min_promote_len_mm=0.5)
            l_dep = Lathe2DIPWVerifier(min_promote_len_mm=0.35)
            self.assertEqual(m_dep.min_promote_len_mm, 0.5)
            self.assertEqual(l_dep.min_promote_len_mm, 0.35)
            self.assertGreaterEqual(len(w_list), 2)

        self.assertEqual(filter_promoted_events_by_length({(99, 99)}, [short_seg], min_len_mm=1.5), set())
        self.assertEqual(filter_promoted_events_by_length({(0, 0)}, [short_seg], min_len_mm=0.0), {(0, 0)})
        for bad_val in (-0.1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                filter_promoted_events_by_length({(0, 0)}, [short_seg], min_len_mm=bad_val)

    def test_02_pipeline_orchestrator_milling_and_overrides(self):
        """Verify PipelineOrchestrator.run() filters at 1.5 mm and supports constructor/run overrides."""
        import json
        import os
        import tempfile
        from unittest.mock import patch
        from geometry.models import BoxStock3D, WorkpieceDefinition
        from pipeline import PipelineOrchestrator

        wp = WorkpieceDefinition(stock=BoxStock3D(-10.0, 10.0, -10.0, 10.0, -10.0, 0.0))
        with tempfile.TemporaryDirectory() as tmp_dir:
            dummy_cps = os.path.join(tmp_dir, "dummy.cps")
            with open(dummy_cps, "w", encoding="utf-8") as f:
                f.write('description = "Dummy";\ncapabilities = CAPABILITY_MILLING;\n')
            dummy_cnc = os.path.join(tmp_dir, "dummy.cnc")
            with open(dummy_cnc, "w", encoding="utf-8") as f:
                f.write("G0 X0 Y0 Z10\n")

            for bad_val in (-1.0, float("nan"), float("inf")):
                with self.assertRaises(ValueError):
                    PipelineOrchestrator(post_bin=dummy_cps, min_promote_len_mm=bad_val)

            ctx_data = {
                "programName": "LenFilterTest",
                "unit": 1,
                "workpiece": {
                    "stock": {
                        "type": "box",
                        "xMin": -10.0,
                        "xMax": 10.0,
                        "yMin": -10.0,
                        "yMax": 10.0,
                        "zMin": -10.0,
                        "zMax": 0.0,
                    }
                },
                "sections": [
                    {
                        "id": 0,
                        "type": 0,
                        "tool": {"number": 1, "type": 9, "diameter": 6.0},
                        "events": [
                            {"type": "onLinear", "sx": 0.0, "sy": 0.0, "sz": 10.0, "x": 1.0, "y": 0.0, "z": 10.0, "f": 500.0},
                            {"type": "onLinear", "sx": 1.0, "sy": 0.0, "sz": 10.0, "x": 2.5, "y": 0.0, "z": 10.0, "f": 500.0},
                            {"type": "onLinear", "sx": 2.5, "sy": 0.0, "sz": 10.0, "x": 7.5, "y": 0.0, "z": 10.0, "f": 500.0},
                        ],
                    }
                ],
            }
            ctx_path = os.path.join(tmp_dir, "ctx.json")
            with open(ctx_path, "w", encoding="utf-8") as f:
                json.dump(ctx_data, f)

            orch = PipelineOrchestrator(post_bin=dummy_cps)
            for bad_val in (-1.0, float("nan"), float("inf")):
                with self.assertRaises(ValueError):
                    orch.run(context_dump_paths=[ctx_path], base_post_path=dummy_cps, min_promote_len_mm=bad_val)

            with patch.object(orch, "replay_combined_context", return_value="G0 X0\n"):
                rep, _ = orch.run(
                    context_dump_paths=[ctx_path],
                    base_post_path=dummy_cps,
                    bootstrap_cnc_path=dummy_cnc,
                    workpiece=wp,
                    promote=True,
                )
                self.assertNotIn((0, 0), rep.promoted_events)
                self.assertIn((0, 1), rep.promoted_events)
                self.assertIn((0, 2), rep.promoted_events)

                rep_zero, _ = orch.run(
                    context_dump_paths=[ctx_path],
                    base_post_path=dummy_cps,
                    bootstrap_cnc_path=dummy_cnc,
                    workpiece=wp,
                    promote=True,
                    min_promote_len_mm=0.0,
                )
                self.assertEqual(rep_zero.promoted_events, {(0, 0), (0, 1), (0, 2)})

                rep_3mm, _ = orch.run(
                    context_dump_paths=[ctx_path],
                    base_post_path=dummy_cps,
                    bootstrap_cnc_path=dummy_cnc,
                    workpiece=wp,
                    promote=True,
                    min_promote_len_mm=3.0,
                )
                self.assertEqual(rep_3mm.promoted_events, {(0, 2)})

            orch_custom = PipelineOrchestrator(post_bin=dummy_cps, min_promote_len_mm=3.0)
            with patch.object(orch_custom, "replay_combined_context", return_value="G0 X0\n"):
                rep_ctor, _ = orch_custom.run(
                    context_dump_paths=[ctx_path],
                    base_post_path=dummy_cps,
                    bootstrap_cnc_path=dummy_cnc,
                    workpiece=wp,
                    promote=True,
                )
                self.assertEqual(rep_ctor.promoted_events, {(0, 2)})

    def test_03_inch_turning_and_clearance_retract_filtering(self):
        """Verify inch scaling, turning length filtering, and MillClearanceVerifier Z+ retract filtering."""
        import json
        import os
        import tempfile
        from unittest.mock import patch
        from shapely.geometry import box
        from geometry.lathe_engine import Lathe2DIPWVerifier
        from geometry.mill_engine import MillClearanceVerifier
        from geometry.models import (
            AxisymmetricStock2D,
            BoxStock3D,
            LatheToolGeometry,
            ToolAssembly,
            ToolCompensationQuadrant,
            WorkpieceDefinition,
        )
        from pipeline import PipelineOrchestrator

        wp = WorkpieceDefinition(stock=BoxStock3D(-10.0, 10.0, -10.0, 10.0, -10.0, 0.0))
        wp_lathe = WorkpieceDefinition(stock=AxisymmetricStock2D(z_min=-20.0, z_max=0.0, outer_radius=10.0))
        tool_lathe = ToolAssembly(
            tool_id=2,
            name="Turn",
            geometry=LatheToolGeometry(
                insert_shape="C",
                nose_radius=0.4,
                hand="R",
                quadrant=ToolCompensationQuadrant.P3,
                cutter_polygon=box(0, 0, 5, 5),
                holder_polygon=box(2, 2, 15, 30),
            ),
        )

        with tempfile.TemporaryDirectory() as tmp_dir:
            dummy_cps = os.path.join(tmp_dir, "dummy.cps")
            with open(dummy_cps, "w", encoding="utf-8") as f:
                f.write('description = "Dummy";\ncapabilities = CAPABILITY_MILLING;\n')
            dummy_cnc = os.path.join(tmp_dir, "dummy.cnc")
            with open(dummy_cnc, "w", encoding="utf-8") as f:
                f.write("G0 X0 Y0 Z10\n")

            orch = PipelineOrchestrator(post_bin=dummy_cps)
            with patch.object(orch, "replay_combined_context", return_value="G0 X0\n"):
                # 1. Inch units (unit=0): 0.05 in (1.27 mm) dropped, 0.06 in (1.524 mm) kept
                ctx_inch = {
                    "programName": "InchLenTest",
                    "unit": 0,
                    "sections": [
                        {
                            "id": 0,
                            "type": 0,
                            "tool": {"number": 1, "type": 9, "diameter": 0.25},
                            "events": [
                                {"type": "onLinear", "sx": 0.0, "sy": 0.0, "sz": 0.5, "x": 0.05, "y": 0.0, "z": 0.5, "f": 20.0},
                                {"type": "onLinear", "sx": 0.05, "sy": 0.0, "sz": 0.5, "x": 0.11, "y": 0.0, "z": 0.5, "f": 20.0},
                            ],
                        }
                    ],
                }
                ctx_inch_p = os.path.join(tmp_dir, "ctx_inch.json")
                with open(ctx_inch_p, "w", encoding="utf-8") as f:
                    json.dump(ctx_inch, f)
                rep_in, _ = orch.run(
                    context_dump_paths=[ctx_inch_p],
                    base_post_path=dummy_cps,
                    bootstrap_cnc_path=dummy_cnc,
                    workpiece=wp,
                    promote=True,
                )
                self.assertNotIn((0, 0), rep_in.promoted_events)
                self.assertIn((0, 1), rep_in.promoted_events)

                # 2. Turning via run(): engine promotes both (0, 0) and (0, 1), caller drops 1.0 mm (0, 0)
                ctx_turn = {
                    "programName": "TurnLenTest",
                    "unit": 1,
                    "sections": [
                        {
                            "id": 0,
                            "type": 1,
                            "tool": {"number": 2, "type": 1, "_isTurningTool": True},
                            "events": [
                                {"type": "onLinear", "sx": 15.0, "sy": 0.0, "sz": 0.0, "x": 15.0, "y": 0.0, "z": -1.0, "f": 200.0},
                                {"type": "onLinear", "sx": 15.0, "sy": 0.0, "sz": -1.0, "x": 15.0, "y": 0.0, "z": -3.0, "f": 200.0},
                            ],
                        }
                    ],
                }
                ctx_turn_p = os.path.join(tmp_dir, "ctx_turn.json")
                with open(ctx_turn_p, "w", encoding="utf-8") as f:
                    json.dump(ctx_turn, f)
                lathe_v = Lathe2DIPWVerifier()
                rep_turn, _ = orch.run(
                    context_dump_paths=[ctx_turn_p],
                    base_post_path=dummy_cps,
                    bootstrap_cnc_path=dummy_cnc,
                    workpiece=wp_lathe,
                    tools_by_op={"0": tool_lathe},
                    verifier=lathe_v,
                    is_turning=True,
                    promote=True,
                )
                self.assertIn((0, 0), lathe_v.promoted_events)
                self.assertNotIn((0, 0), rep_turn.promoted_events)
                self.assertIn((0, 1), rep_turn.promoted_events)

                # 3. MillClearanceVerifier: engine promotes both (0, 0) (1.0 mm) and (0, 1) (2.0 mm); caller filters out (0, 0)
                ctx_clr = {
                    "programName": "ClearanceRetractLenTest",
                    "unit": 1,
                    "sections": [
                        {
                            "id": 0,
                            "type": 0,
                            "zClearance": 10.0,
                            "tool": {"number": 1, "type": 9, "diameter": 6.0},
                            "events": [
                                {"type": "onLinear", "sx": 0.0, "sy": 0.0, "sz": 9.0, "x": 0.0, "y": 0.0, "z": 10.0, "f": 500.0},
                                {"type": "onLinear", "sx": 0.0, "sy": 0.0, "sz": 8.0, "x": 0.0, "y": 0.0, "z": 10.0, "f": 500.0},
                            ],
                        }
                    ],
                }
                ctx_clr_p = os.path.join(tmp_dir, "ctx_clr.json")
                with open(ctx_clr_p, "w", encoding="utf-8") as f:
                    json.dump(ctx_clr, f)
                clr_v = MillClearanceVerifier()
                rep_clr, _ = orch.run(
                    context_dump_paths=[ctx_clr_p],
                    base_post_path=dummy_cps,
                    bootstrap_cnc_path=dummy_cnc,
                    workpiece=wp,
                    verifier=clr_v,
                    promote=True,
                )
                self.assertIn((0, 0), clr_v.promoted_events)
                self.assertNotIn((0, 0), rep_clr.promoted_events)
                self.assertIn((0, 1), rep_clr.promoted_events)


if __name__ == "__main__":
    unittest.main()
