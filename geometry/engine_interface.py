"""
Verification Engine Interface Contract and Base Implementation.
"""

from abc import ABC, abstractmethod
from typing import Dict, List, Optional, Sequence, Set

from .models import (
    CollisionRecord,
    MRRSample,
    MotionKind,
    MotionSegment,
    MoveEvaluation,
    PromotedEventKey,
    ToolAssembly,
    VerificationReport,
    WorkpieceDefinition,
)


class IVerificationEngine(ABC):
    """Abstract contract for collision verification, IPW tracking, and rapid promotion."""

    @abstractmethod
    def reset(self, workpiece: WorkpieceDefinition) -> None:
        """Initialize or reset the In-Process Workpiece (IPW) and obstacle state."""
        pass

    @abstractmethod
    def load_tool(self, tool: ToolAssembly) -> None:
        """Mount the active tool and holder assembly."""
        pass

    @abstractmethod
    def step(self, move: MotionSegment) -> MoveEvaluation:
        """
        Atomically evaluate the move against the current IPW and fixtures,
        mutate the IPW (carve swept volume), advance the clock, and return metrics.
        """
        pass

    @abstractmethod
    def generate_report(self) -> VerificationReport:
        """Compile collisions, MRR timeline, statistical spikes, and promotion list."""
        pass

    def verify_toolpath(
        self,
        moves: Optional[Sequence[MotionSegment]] = None,
        tools_by_op: Optional[Dict[str, ToolAssembly]] = None,
        workpiece: Optional[WorkpieceDefinition] = None,
        segments: Optional[Sequence[MotionSegment]] = None,
    ) -> VerificationReport:
        """Convenience orchestration loop executing a complete toolpath evaluation."""
        moves_seq = moves if moves is not None else (segments or [])
        if workpiece is not None:
            self.reset(workpiece)
        current_op = None
        op_order: List[str] = []

        for move in moves_seq:
            if move.op_id and move.op_id not in op_order:
                op_order.append(move.op_id)
            if move.op_id != current_op:
                current_op = move.op_id
                if tools_by_op and current_op in tools_by_op:
                    self.load_tool(tools_by_op[current_op])

            self.step(move)

        if hasattr(self, "operation_names"):
            self.operation_names = op_order

        return self.generate_report()


class BaseVerificationEngine(IVerificationEngine):
    """
    Base implementation providing shared state tracking, timing,
    vector occurrence gating, MRR spike detection, and report compilation.
    """

    def __init__(self, min_mrr_duration_sec: float = 0.05) -> None:
        self.workpiece: Optional[WorkpieceDefinition] = None
        self.active_tool: Optional[ToolAssembly] = None
        self.current_time_sec: float = 0.0
        self.total_machining_time_sec: float = 0.0
        self.total_volume_removed_mm3: float = 0.0
        self.min_mrr_duration_sec: float = max(1e-4, float(min_mrr_duration_sec))
        self.collisions: List[CollisionRecord] = []
        self.mrr_timeline: List[MRRSample] = []
        self.promoted_events: Set[PromotedEventKey] = set()
        self._blocked_events: Set[PromotedEventKey] = set()
        self.operation_names: List[str] = []
        self.holder_unchecked_ops: Set[str] = set()

    def _reset_base_state(self, workpiece: WorkpieceDefinition) -> None:
        """Reset the shared timing, collision, MRR, and vector occurrence collections."""
        self.workpiece = workpiece
        self.active_tool = None
        self.current_time_sec = 0.0
        self.total_machining_time_sec = 0.0
        self.total_volume_removed_mm3 = 0.0
        self.collisions.clear()
        self.mrr_timeline.clear()
        self.promoted_events.clear()
        self._blocked_events.clear()
        self.operation_names.clear()
        self.holder_unchecked_ops.clear()

    def _promotion_blocked(self, move: MotionSegment) -> bool:
        """
        Check if rapid promotion must be blocked for this move.
        Moves under active radius compensation (rc != 0) or transition moves
        (rc_transition=True) must never be promoted to rapids.
        """
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

    def record_mrr_sample(
        self,
        segment_id: int,
        duration_sec: float,
        volume_removed_mm3: float,
        depth_of_cut_mm: float = 0.0,
        op_id: str = "",
    ) -> MRRSample:
        """Record volumetric removal rate sample in timeline."""
        raw_dur = max(1e-4, float(duration_sec))
        eff_dur = max(self.min_mrr_duration_sec, raw_dur)
        mrr_rate = (volume_removed_mm3 / eff_dur) * 60.0
        # self.current_time_sec has been advanced by dur at the beginning of step();
        # compute start time so timestamp_sec reflects the actual beginning of the cut.
        t_sample_start = max(0.0, self.current_time_sec - max(0.0, float(duration_sec)))
        sample = MRRSample(
            timestamp_sec=t_sample_start,
            duration_sec=raw_dur,
            segment_id=segment_id,
            tool_id=self.active_tool.tool_id if self.active_tool else 0,
            volume_removed_mm3=volume_removed_mm3,
            mrr_rate_mm3_per_min=mrr_rate,
            depth_of_cut_mm=depth_of_cut_mm,
            op_id=op_id,
        )
        self.mrr_timeline.append(sample)
        self.total_volume_removed_mm3 += volume_removed_mm3
        return sample

    def generate_report(self) -> VerificationReport:
        """
        Compile VerificationReport with strictly validated monotonic promotion rules,
        collating collisions, total volume removed, and statistical MRR spikes.
        Top 3 spikes per operation are selected (relative to each operation's
        time-weighted mean MRR) and sorted chronologically.
        """
        active_samples = [
            s for s in self.mrr_timeline
            if s.mrr_rate_mm3_per_min > 1.0 and s.duration_sec > 0.0
        ]
        spikes: List[MRRSample] = []
        if active_samples:
            samples_by_op: Dict[str, List[MRRSample]] = {}
            for s in active_samples:
                op_key = s.op_id if s.op_id else "General"
                samples_by_op.setdefault(op_key, []).append(s)

            top_spikes: List[MRRSample] = []
            for op_key, op_samples in samples_by_op.items():
                op_dur = sum(s.duration_sec for s in op_samples)
                op_vol = sum(s.volume_removed_mm3 for s in op_samples)
                op_mean_mrr = (op_vol / op_dur) * 60.0 if op_dur > 0.0 else 0.0
                threshold = op_mean_mrr * 2.5

                # Aggregate consecutive sub-floor samples into windows >= min_mrr_duration_sec
                # so short ~0.1 ms moves cannot generate unphysical spike rates.
                op_spikes: List[MRRSample] = []
                win_samples: List[MRRSample] = []
                win_dur = 0.0
                win_vol = 0.0

                def _flush_window() -> None:
                    nonlocal win_samples, win_dur, win_vol
                    if not win_samples:
                        return
                    if win_dur >= self.min_mrr_duration_sec:
                        win_rate = (win_vol / win_dur) * 60.0
                        if win_rate > threshold:
                            rep = max(win_samples, key=lambda x: x.volume_removed_mm3)
                            op_spikes.append(
                                MRRSample(
                                    timestamp_sec=win_samples[0].timestamp_sec,
                                    duration_sec=win_dur,
                                    segment_id=rep.segment_id,
                                    tool_id=rep.tool_id,
                                    volume_removed_mm3=win_vol,
                                    mrr_rate_mm3_per_min=win_rate,
                                    depth_of_cut_mm=max(x.depth_of_cut_mm for x in win_samples),
                                    op_id=rep.op_id,
                                )
                            )
                    win_samples = []
                    win_dur = 0.0
                    win_vol = 0.0

                for s in op_samples:
                    if s.duration_sec >= self.min_mrr_duration_sec:
                        _flush_window()
                        if s.mrr_rate_mm3_per_min > threshold:
                            op_spikes.append(s)
                    else:
                        win_samples.append(s)
                        win_dur += s.duration_sec
                        win_vol += s.volume_removed_mm3
                        if win_dur >= self.min_mrr_duration_sec:
                            _flush_window()
                _flush_window()

                sorted_op = sorted(op_spikes, key=lambda s: s.mrr_rate_mm3_per_min, reverse=True)[:3]
                top_spikes.extend(sorted_op)

            # Sort selected spikes chronologically for timeline display
            spikes = sorted(top_spikes, key=lambda s: (s.timestamp_sec, s.segment_id))

        return VerificationReport(
            total_machining_time_sec=self.total_machining_time_sec,
            total_volume_removed_mm3=self.total_volume_removed_mm3,
            collisions=list(self.collisions),
            promoted_events=set(self.promoted_events),
            mrr_timeline=list(self.mrr_timeline),
            mrr_spikes=spikes,
            operation_names=list(getattr(self, "operation_names", [])),
            holder_unchecked_ops=sorted(str(x) for x in getattr(self, "holder_unchecked_ops", ())),
        )
