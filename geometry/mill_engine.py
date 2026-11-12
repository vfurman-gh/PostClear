"""
Simple Milling Verification Engine (`MillClearanceVerifier`).

Implements `IVerificationEngine` for Milling / Router setups in the unified
2-pass Context-Dump & Single-Pass Replay pipeline. Conservatively authorizes
rapid (`G0`) promotion strictly for pure `Z+` upward movements (`dx == 0, dy == 0, dz > 0`)
that retract to or above the operation's clearance height (`zClearance`).
"""

import math
from typing import Dict, List, Optional, Sequence, Set

from .engine_interface import IVerificationEngine
from .heightfield_engine import HeightfieldGrid, MillHeightfieldVerifier

__all__ = [
    "MillHeightfieldVerifier",
    "HeightfieldGrid",
    "MillClearanceVerifier",
]
from .models import (
    CollisionRecord,
    MotionKind,
    MotionSegment,
    MoveEvaluation,
    MRRSample,
    PromotedEventKey,
    ToolAssembly,
    VerificationReport,
    WorkpieceDefinition,
)


class MillClearanceVerifier(IVerificationEngine):
    """
    Conservative Milling verifier that only promotes pure `Z+` vertical retract
    moves (`dx == 0, dy == 0, end.z > start.z`) reaching at or above the
    operation's clearance height (`zClearance`).
    """

    def __init__(
        self,
        xy_tol_mm: float = 1e-3,
        z_tol_mm: float = 1e-3,
    ) -> None:
        self.xy_tol_mm = float(xy_tol_mm)
        self.z_tol_mm = float(z_tol_mm)
        self.workpiece: Optional[WorkpieceDefinition] = None
        self.active_tool: Optional[ToolAssembly] = None
        self._sec_clearance_mm: Dict[int, float] = {}
        self.total_machining_time_sec = 0.0
        self.all_collisions: List[CollisionRecord] = []
        self.promoted_events: Set[PromotedEventKey] = set()
        self._blocked_events: Set[PromotedEventKey] = set()

    def reset(self, workpiece: WorkpieceDefinition) -> None:
        """Initialize or reset the workpiece state and per-section clearance cache."""
        stock = getattr(workpiece, "stock", None)
        if stock is None:
            raise ValueError(f"MillClearanceVerifier requires valid stock, got {type(stock)}")
        self.workpiece = workpiece
        self.active_tool = None
        self._sec_clearance_mm.clear()
        self.total_machining_time_sec = 0.0
        self.all_collisions.clear()
        self.promoted_events.clear()
        self._blocked_events.clear()

    def reset_workpiece(self, workpiece: WorkpieceDefinition) -> None:
        """Alias for reset() for backwards compatibility."""
        self.reset(workpiece)

    def load_tool(self, tool: ToolAssembly) -> None:
        """Mount the active tool."""
        self.active_tool = tool

    def _resolve_clearance_z_mm(self, segment: MotionSegment) -> Optional[float]:
        """Resolve the target clearance Z height (in mm) for `segment.sec_id`."""
        meta = segment.metadata or {}
        z_clear = meta.get("z_clearance_mm")
        if z_clear is not None:
            return float(z_clear)

        z_retract = meta.get("z_retract_mm")
        if z_retract is not None:
            return float(z_retract)

        if segment.sec_id in self._sec_clearance_mm:
            return self._sec_clearance_mm[segment.sec_id]

        if self.workpiece is not None and getattr(self.workpiece, "stock", None) is not None:
            stock = self.workpiece.stock
            if hasattr(stock, "z_max"):
                return float(stock.z_max)

        return None

    def _promotion_blocked(self, move: MotionSegment) -> bool:
        """Return True if rapid promotion is blocked for this move."""
        meta = getattr(move, "metadata", None) or {}
        return int(meta.get("rc", 0) or 0) != 0 or bool(meta.get("rc_transition", False))

    def record_event_evaluation(
        self,
        move: MotionSegment,
        is_air_move: bool,
        collisions: Sequence[CollisionRecord] = (),
    ) -> None:
        """
        Record per-event promotion verdict.
        An event (sec_idx, event_idx) is promoted if and only if it is an eligible
        linear feed move and all its segments are verified air and collision-free.
        """
        meta = getattr(move, "metadata", None) or {}
        sec_idx = meta.get("sec_idx", move.sec_id)
        event_idx = meta.get("event_idx")
        if event_idx is None:
            event_idx = move.segment_id
        ev_key: PromotedEventKey = (sec_idx, event_idx)

        in_cycle = bool(getattr(move, "in_cycle", False) or meta.get("in_cycle", False))
        is_sync = bool(getattr(move, "sync", False) or meta.get("sync", False))
        is_blocked = self._promotion_blocked(move)

        if (
            is_air_move
            and not in_cycle
            and not is_sync
            and not is_blocked
            and move.motion == MotionKind.LINEAR
            and not collisions
        ):
            if ev_key not in self._blocked_events:
                self.promoted_events.add(ev_key)
        else:
            self._blocked_events.add(ev_key)
            self.promoted_events.discard(ev_key)

    def step_segment(
        self,
        segment: MotionSegment,
        tool: Optional[ToolAssembly] = None,
    ) -> MoveEvaluation:
        """
        Evaluate a single kinematic move.
        Only pure `Z+` linear moves outside canned cycles that reach `>= clearance_z`
        are marked as `is_air_move = True` for `G0` promotion.
        """
        vkey_str = segment.vector_key.to_wrapper_str()
        meta = segment.metadata or {}
        sec_idx = meta.get("sec_idx", segment.sec_id)
        event_idx = meta.get("event_idx", segment.segment_id)

        if segment.motion == MotionKind.RAPID:
            top_z = max(segment.start.z, segment.end.z)
            prev_z = self._sec_clearance_mm.get(segment.sec_id)
            if prev_z is None or top_z > prev_z:
                self._sec_clearance_mm[segment.sec_id] = top_z
            return MoveEvaluation(
                segment_id=segment.segment_id,
                vector_key=vkey_str,
                is_air_move=True,
                material_removed_mm3=0.0,
                instantaneous_mrr=0.0,
                collisions=[],
                sec_idx=sec_idx,
                event_idx=event_idx,
            )

        in_cycle = bool(meta.get("in_cycle", False))

        is_air_move = False
        if segment.motion == MotionKind.LINEAR and not in_cycle and not self._promotion_blocked(segment):
            dx = segment.end.x - segment.start.x
            dy = segment.end.y - segment.start.y
            dz = segment.end.z - segment.start.z
            is_pure_z_plus = (math.hypot(dx, dy) <= self.xy_tol_mm) and (dz > self.z_tol_mm)
            if is_pure_z_plus:
                clearance_z = self._resolve_clearance_z_mm(segment)
                if clearance_z is not None and segment.end.z >= (clearance_z - self.z_tol_mm):
                    is_air_move = True

        return MoveEvaluation(
            segment_id=segment.segment_id,
            vector_key=vkey_str,
            is_air_move=is_air_move,
            material_removed_mm3=0.0,
            instantaneous_mrr=0.0,
            collisions=[],
            sec_idx=sec_idx,
            event_idx=event_idx,
        )

    def step(self, move: MotionSegment) -> MoveEvaluation:
        """
        Evaluate a single kinematic move against clearance rules and record metrics.
        """
        eval_res = self.step_segment(move, self.active_tool)
        self.total_machining_time_sec += max(0.0, float(move.duration_sec))

        if eval_res.collisions:
            self.all_collisions.extend(eval_res.collisions)

        self.record_event_evaluation(move, is_air_move=eval_res.is_air_move, collisions=eval_res.collisions)

        return eval_res

    def generate_report(self) -> VerificationReport:
        """Compile VerificationReport with promoted_events for Z+ clearance retracts."""
        return VerificationReport(
            total_machining_time_sec=self.total_machining_time_sec,
            total_volume_removed_mm3=0.0,
            collisions=list(self.all_collisions),
            promoted_events=set(self.promoted_events),
            mrr_timeline=[],
            mrr_spikes=[],
        )

    def verify_toolpath(
        self,
        moves: Optional[Sequence[MotionSegment]] = None,
        tools_by_op: Optional[Dict[str, ToolAssembly]] = None,
        workpiece: Optional[WorkpieceDefinition] = None,
        segments: Optional[Sequence[MotionSegment]] = None,
    ) -> VerificationReport:
        """
        Run the complete Milling verification pass across all segments and return
        a `VerificationReport` with `promoted_events` for `Z+` clearance retracts.
        """
        moves_seq = moves if moves is not None else (segments or [])
        if workpiece is not None:
            self.reset(workpiece)
        current_op = None

        for move in moves_seq:
            if move.op_id != current_op:
                current_op = move.op_id
                if tools_by_op and current_op in tools_by_op:
                    self.load_tool(tools_by_op[current_op])
            self.step(move)

        return self.generate_report()
