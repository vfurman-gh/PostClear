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


if __name__ == "__main__":
    unittest.main()
