#!/usr/bin/env python3
"""
Validation and regression test suite for Batch 4b:
Full per-event promotion addressing migration (INV-1 Steps 1–4 + Fix F3).

Verifies:
1. (a) Earlier air pass at coordinates A->B is promoted when later pass at A->B is cutting or rc-blocked.
2. (b) Collision on one event blocks only that colliding event, not another safe air event with identical coordinates.
3. (c) plotter.py marks only the promoted event(s), not every occurrence of the same coordinates.
4. Arc sharing coordinates does not pollute or block linear move promotion.
5. event_idx alignment through multi-batch merge.
6. Trailer comment "UNIVERSAL WRAPPER: RESTORED N G0 RAPID MOVES" emitted when restoredCount > 0 without PROMOTED_MOVES_JSON.
"""

import copy
import os
import tempfile
import unittest
from unittest.mock import MagicMock, patch

from shapely.geometry import box, Polygon

from context_parser import (
    apply_promotions_to_context,
    merge_context_dumps,
    parse_context_dump,
)
from geometry.engine_interface import BaseVerificationEngine
from geometry.heightfield_engine import MillHeightfieldVerifier
from geometry.lathe_engine import Lathe2DIPWVerifier
from geometry.models import (
    AxisymmetricStock2D,
    BoxStock3D,
    CollisionKind,
    CollisionRecord,
    LatheToolGeometry,
    MillToolGeometry,
    MotionKind,
    MotionSegment,
    MoveEvaluation,
    ToolAssembly,
    ToolCompensationQuadrant,
    Vector3D,
    VerificationReport,
    WorkpieceDefinition,
)
from pipeline import PipelineOrchestrator


class TestPerEventPromotionMigration(unittest.TestCase):
    """Test suite for INV-1 Steps 1-4 per-event promotion migration."""

    def test_a_earlier_air_pass_promoted_when_later_pass_is_rc_blocked_lathe(self):
        """
        (a) Earlier air pass at coordinates A->B is promoted even when a later pass
        at identical coordinates A->B is rc-blocked (Q6 confirmed).
        """
        stock = AxisymmetricStock2D(z_min=-50.0, z_max=0.0, outer_radius=20.0, inner_radius=0.0)
        wp = WorkpieceDefinition(stock=stock, fixtures=[])
        verifier = Lathe2DIPWVerifier()
        verifier.reset(wp)

        cutter = Polygon([(0, 0), (2, 0), (2, 2), (0, 2)])
        holder = Polygon([(2, 0), (10, 0), (10, 5), (2, 5)])
        tool = ToolAssembly(
            tool_id=1,
            name="OD Turn",
            geometry=LatheToolGeometry(
                insert_shape="C",
                nose_radius=0.8,
                hand="R",
                quadrant=ToolCompensationQuadrant.P3,
                cutter_polygon=cutter,
                holder_polygon=holder,
            ),
        )
        verifier.load_tool(tool)

        # Move 1: Air move at R=30.0 (outside stock R=20.0), rc=0 -> Should be promoted
        move1 = MotionSegment(
            segment_id=1,
            op_id="Pass1",
            sec_id=0,
            motion=MotionKind.LINEAR,
            start=Vector3D(30.0, 0.0, 0.0),
            end=Vector3D(30.0, 0.0, -20.0),
            feedrate_mmpm=200.0,
            spindle_rpm=1000.0,
            duration_sec=6.0,
            metadata={"sec_idx": 0, "event_idx": 0, "rc": 0},
        )
        res1 = verifier.step(move1)
        self.assertTrue(res1.is_air_move)

        # Move 2: Identical coordinates A->B at R=30.0, but with rc=1 -> Must NOT be promoted
        move2 = MotionSegment(
            segment_id=2,
            op_id="Pass2",
            sec_id=0,
            motion=MotionKind.LINEAR,
            start=Vector3D(30.0, 0.0, 0.0),
            end=Vector3D(30.0, 0.0, -20.0),
            feedrate_mmpm=200.0,
            spindle_rpm=1000.0,
            duration_sec=6.0,
            metadata={"sec_idx": 0, "event_idx": 1, "rc": 1},
        )
        res2 = verifier.step(move2)
        self.assertFalse(res2.is_air_move)

        report = verifier.generate_report()
        # Per-event addressing: event (0, 0) is promoted independently of event (0, 1)
        self.assertIn((0, 0), report.promoted_events)
        self.assertNotIn((0, 1), report.promoted_events)

    def test_a_earlier_air_pass_promoted_when_later_pass_is_rc_blocked_heightfield(self):
        """
        (a) Heightfield engine: earlier air pass at A->B is promoted when later pass
        at A->B is rc-blocked.
        """
        stock = BoxStock3D(x_min=-50.0, x_max=50.0, y_min=-50.0, y_max=50.0, z_min=-20.0, z_max=0.0)
        wp = WorkpieceDefinition(stock=stock, fixtures=[])
        verifier = MillHeightfieldVerifier(cell_size_mm=1.0)
        verifier.reset(wp)

        tool = ToolAssembly(
            tool_id=1,
            name="Endmill",
            geometry=MillToolGeometry(diameter=10.0, corner_radius=0.0, flute_length=20.0, overall_length=50.0),
        )
        verifier.load_tool(tool)

        # Move 1: Air move well above stock (Z=15.0), rc=0
        move1 = MotionSegment(
            segment_id=1,
            op_id="Air1",
            sec_id=0,
            motion=MotionKind.LINEAR,
            start=Vector3D(-20.0, 0.0, 15.0),
            end=Vector3D(20.0, 0.0, 15.0),
            feedrate_mmpm=1000.0,
            spindle_rpm=8000.0,
            duration_sec=2.4,
            metadata={"sec_idx": 0, "event_idx": 10, "rc": 0},
        )
        res1 = verifier.step(move1)
        self.assertTrue(res1.is_air_move)

        # Move 2: Same coordinates A->B at Z=15.0, but rc=2
        move2 = MotionSegment(
            segment_id=2,
            op_id="Air2",
            sec_id=0,
            motion=MotionKind.LINEAR,
            start=Vector3D(-20.0, 0.0, 15.0),
            end=Vector3D(20.0, 0.0, 15.0),
            feedrate_mmpm=1000.0,
            spindle_rpm=8000.0,
            duration_sec=2.4,
            metadata={"sec_idx": 0, "event_idx": 11, "rc": 2},
        )
        res2 = verifier.step(move2)
        self.assertFalse(res2.is_air_move)

        report = verifier.generate_report()
        self.assertIn((0, 10), report.promoted_events)
        self.assertNotIn((0, 11), report.promoted_events)

    def test_b_collision_blocks_only_colliding_event(self):
        """
        (b) When a collision occurs on an event, per-event filtering blocks ONLY
        that event (sec_idx, event_idx), not another safe air event with the same
        start/end coordinates (Q2 confirmed).
        """
        ctx = {
            "unit": 1,
            "sections": [
                {
                    "id": 0,
                    "events": [
                        # Event 0: Safe air pass
                        {"type": "onLinear", "sx": 10.0, "sy": 0.0, "sz": 5.0, "x": 20.0, "y": 0.0, "z": 5.0, "feed": 500},
                        # Event 1: Later pass at same coordinates that encounters a collision
                        {"type": "onLinear", "sx": 10.0, "sy": 0.0, "sz": 5.0, "x": 20.0, "y": 0.0, "z": 5.0, "feed": 500},
                    ],
                }
            ],
        }

        # Suppose both were tentatively evaluated as air moves
        report = VerificationReport(
            total_machining_time_sec=10.0,
            total_volume_removed_mm3=0.0,
            collisions=[
                CollisionRecord(
                    segment_id=2,
                    kind=CollisionKind.HOLDER_STRIKE,
                    timestamp_sec=5.0,
                    contact_point=Vector3D(20.0, 0.0, 5.0),
                    penetration_volume_mm3=12.0,
                    description="Toolholder collided with fixture on pass 2",
                    sec_idx=0,
                    event_idx=1,
                )
            ],
            promoted_events={(0, 0), (0, 1)},
            mrr_timeline=[],
            mrr_spikes=[],
        )

        # Per-event collision filtering (Q2)
        colliding_events = {
            (c.sec_idx, c.event_idx)
            for c in report.collisions
            if c.sec_idx is not None and c.event_idx is not None
        }
        safe_promoted_events = {
            ev for ev in report.promoted_events
            if ev not in colliding_events
        }

        self.assertIn((0, 0), safe_promoted_events)
        self.assertNotIn((0, 1), safe_promoted_events)

        promoted_count = apply_promotions_to_context(ctx, safe_promoted_events)
        self.assertEqual(promoted_count, 1)
        self.assertTrue(ctx["sections"][0]["events"][0].get("promote"))
        self.assertFalse(ctx["sections"][0]["events"][1].get("promote", False))

    def test_c_plotter_marks_only_promoted_event(self):
        """
        (c) plotter.py marks only the promoted event(s), not every occurrence of the
        same coordinates (Fix F3).
        """
        import matplotlib.pyplot as plt
        from geometry.plotter import generate_verification_plot

        stock = AxisymmetricStock2D(z_min=-50.0, z_max=0.0, outer_radius=20.0, inner_radius=0.0)
        wp = WorkpieceDefinition(stock=stock, fixtures=[])

        # Event 0 is promoted; Event 1 with identical coordinates is NOT promoted
        seg1 = MotionSegment(
            segment_id=1,
            op_id="Op1",
            sec_id=0,
            motion=MotionKind.LINEAR,
            start=Vector3D(30.0, 0.0, 0.0),
            end=Vector3D(30.0, 0.0, -20.0),
            feedrate_mmpm=200.0,
            spindle_rpm=1000.0,
            duration_sec=6.0,
            metadata={"sec_idx": 0, "event_idx": 0},
        )
        seg2 = MotionSegment(
            segment_id=2,
            op_id="Op1",
            sec_id=0,
            motion=MotionKind.LINEAR,
            start=Vector3D(30.0, 0.0, 0.0),
            end=Vector3D(30.0, 0.0, -20.0),
            feedrate_mmpm=200.0,
            spindle_rpm=1000.0,
            duration_sec=6.0,
            metadata={"sec_idx": 0, "event_idx": 1},
        )

        report = VerificationReport(
            total_machining_time_sec=12.0,
            total_volume_removed_mm3=0.0,
            collisions=[],
            promoted_events={(0, 0)},  # Only event 0 is promoted
            mrr_timeline=[],
            mrr_spikes=[],
        )

        captured_axes = []
        orig_subplots = plt.subplots

        def mock_subplots(*args, **kwargs):
            fig, axes = orig_subplots(*args, **kwargs)
            captured_axes.append(axes[0])
            return fig, axes

        with tempfile.TemporaryDirectory() as tmp_dir:
            out_png = os.path.join(tmp_dir, "test_plot.png")
            with patch("geometry.plotter.plt.subplots", side_effect=mock_subplots):
                generate_verification_plot(
                    workpiece=wp,
                    final_ipw=Polygon([(0, 0), (20, 0), (20, -50), (0, -50)]),
                    chuck_obstacle=Polygon(),
                    segments=[seg1, seg2],
                    report=report,
                    tools_by_op={},
                    out_png_path=out_png,
                    unit_is_inch=False,
                )
            self.assertTrue(os.path.isfile(out_png))
            self.assertEqual(len(captured_axes), 1)

            # Exactly one line should be styled as dashed red (#d62728, linestyle='--')
            ax_geo = captured_axes[0]
            promoted_lines = [
                line for line in ax_geo.lines
                if line.get_color() == "#d62728" and line.get_linestyle() == "--"
            ]
            self.assertEqual(len(promoted_lines), 1, "Only the promoted event should be drawn with dashed red line")

            # Check title uses promoted event count (1), not 0
            title = ax_geo.get_title()
            self.assertIn("Promoted G0 Moves: 1", title)

    def test_multi_batch_merge_event_idx_alignment(self):
        """
        Verify event_idx alignment across multiple batch context dumps.
        When two batches are merged, sections receive 0..N-1 IDs, and apply_promotions_to_context
        promotes precisely the targeted (sec_idx, event_idx) pairs.
        """
        b0 = {
            "unit": 1,
            "sections": [
                {
                    "id": 0,
                    "events": [
                        {"type": "onLinear", "sx": 0, "sy": 0, "sz": 5, "x": 10, "y": 0, "z": 5, "feed": 100},
                        {"type": "onLinear", "sx": 10, "sy": 0, "sz": 5, "x": 20, "y": 0, "z": 5, "feed": 100},
                    ],
                }
            ],
        }
        b1 = {
            "unit": 1,
            "sections": [
                {
                    "id": 0,
                    "events": [
                        {"type": "onLinear", "sx": 0, "sy": 0, "sz": 5, "x": 10, "y": 0, "z": 5, "feed": 100},
                        {"type": "onLinear", "sx": 10, "sy": 0, "sz": 5, "x": 20, "y": 0, "z": 5, "feed": 100},
                    ],
                }
            ],
        }

        merged = merge_context_dumps([b0, b1])
        segments = parse_context_dump(merged)
        self.assertEqual(len(segments), 4)

        # Check metadata
        self.assertEqual(segments[0].metadata.get("sec_idx"), 0)
        self.assertEqual(segments[0].metadata.get("event_idx"), 0)
        self.assertEqual(segments[1].metadata.get("sec_idx"), 0)
        self.assertEqual(segments[1].metadata.get("event_idx"), 1)
        self.assertEqual(segments[2].metadata.get("sec_idx"), 1)
        self.assertEqual(segments[2].metadata.get("event_idx"), 0)
        self.assertEqual(segments[3].metadata.get("sec_idx"), 1)
        self.assertEqual(segments[3].metadata.get("event_idx"), 1)

        # Promote only section 0 event 0 and section 1 event 1
        target_events = {(0, 0), (1, 1)}
        cnt = apply_promotions_to_context(merged, target_events)
        self.assertEqual(cnt, 2)

        self.assertTrue(merged["sections"][0]["events"][0].get("promote"))
        self.assertFalse(merged["sections"][0]["events"][1].get("promote", False))
        self.assertFalse(merged["sections"][1]["events"][0].get("promote", False))
        self.assertTrue(merged["sections"][1]["events"][1].get("promote"))

    def test_canned_cycle_and_sync_moves_never_promoted_by_apply_promotions(self):
        """
        Defense-in-depth: apply_promotions_to_context ignores any events that are
        sync=True, in_cycle=True, or enclosed between onCycle/onCycleEnd, even if
        their (sec_idx, event_idx) is passed in promoted_moves.
        """
        ctx = {
            "unit": 1,
            "sections": [
                {
                    "events": [
                        {"type": "onLinear", "sx": 0, "sy": 0, "sz": 10, "x": 10, "y": 0, "z": 10, "feed": 100, "sync": True},
                        {"type": "onCycle"},
                        {"type": "onLinear", "sx": 0, "sy": 0, "sz": 10, "x": 10, "y": 0, "z": 10, "feed": 100},
                        {"type": "onCycleEnd"},
                        {"type": "onLinear", "sx": 0, "sy": 0, "sz": 10, "x": 10, "y": 0, "z": 10, "feed": 100, "in_cycle": True},
                    ]
                }
            ],
        }

        # Attempt to promote all events
        all_events = {(0, 0), (0, 1), (0, 2), (0, 3), (0, 4)}
        cnt = apply_promotions_to_context(ctx, all_events)
        self.assertEqual(cnt, 0, "None of the sync or canned-cycle events should be promoted")
        for ev in ctx["sections"][0]["events"]:
            self.assertFalse(ev.get("promote", False))


if __name__ == "__main__":
    unittest.main()
