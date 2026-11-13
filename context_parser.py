"""
JSON Context Merger, Motion Segment Normalizer, and Promotion Injector.

Processes structured JSON context dumps emitted by `universal_dumper.cps`:
1. `merge_context_dumps`: Combines multiple single-tool batch JSON dumps into a
   single unified multi-section context dictionary, re-indexing section IDs
   monotonically (`0..N-1`) while preserving batch provenance.
2. `parse_context_dump`: Converts a single or merged JSON context dictionary into
   canonical metric `MotionSegment` sequences (`mm`, `mm/min`, `sec`) for
   `IVerificationEngine` simulation.
3. `apply_promotions_to_context`: Annotates authorized air moves directly in the
   combined JSON context (`ev["promote"] = True`) while respecting occurrence
   thresholds (`min_occur`) and canned-cycle / synchronized-threading guards.
"""

import copy
import json
import math
import os
from typing import Any, Dict, List, Optional, Sequence, Set, Tuple, Union

try:
    import numpy as np
except (ImportError, OSError):
    np = None

from geometry.models import (
    MotionKind,
    MotionSegment,
    Vector3D,
    VectorKey,
    PromotedEventKey,
)


def load_context_dump(source: Union[str, Dict[str, Any]]) -> Dict[str, Any]:
    """Load a JSON context dump from a file path or return a deep copy of a dict."""
    if isinstance(source, str):
        with open(source, "r", encoding="utf-8") as f:
            return json.load(f)
    if isinstance(source, dict):
        return copy.deepcopy(source)
    raise TypeError(f"Expected file path or dict, got {type(source)}")


def merge_context_dumps(sources: Sequence[Union[str, Dict[str, Any]]]) -> Dict[str, Any]:
    """
    Merge one or more single-tool `universal_dumper.cps` JSON dumps into a single
    unified multi-tool context dictionary ready for verification and replay.
    """
    if not sources:
        raise ValueError("merge_context_dumps requires at least one source dump")

    dumps = [load_context_dump(s) for s in sources]
    merged = copy.deepcopy(dumps[0])
    merged["sections"] = []

    if not merged.get("maximumSpindleSpeed"):
        for d in dumps:
            if d.get("maximumSpindleSpeed"):
                merged["maximumSpindleSpeed"] = d["maximumSpindleSpeed"]
                break

    if "rapidMode" not in merged:
        for d in dumps:
            if "rapidMode" in d:
                merged["rapidMode"] = copy.deepcopy(d["rapidMode"])
                break

    global_sec_id = 0
    for batch_idx, d in enumerate(dumps):
        for k, v in d.get("globalParams", {}).items():
            if k not in merged["globalParams"]:
                merged["globalParams"][k] = v

        for sec in d.get("sections", []):
            sec_copy = copy.deepcopy(sec)
            sec_copy["origSecId"] = sec.get("id", 0)
            sec_copy["batchIdx"] = batch_idx
            sec_copy["id"] = global_sec_id
            merged["sections"].append(sec_copy)
            global_sec_id += 1

    return merged


def parse_context_dump(
    source: Union[str, Dict[str, Any]],
    default_rapid_feed_mmpm: float = 5000.0,
) -> List[MotionSegment]:
    """
    Translate a single or merged `universal_dumper.cps` JSON context dump into a
    normalized metric `List[MotionSegment]` for `IVerificationEngine`.
    """
    data = load_context_dump(source)
    # In CAM360/post: IN = 0, MM = 1
    unit = data.get("unit", 0)
    scale = 25.4 if unit == 0 else 1.0

    segments: List[MotionSegment] = []
    segment_id = 0

    # In CAM360/post: FEED_PER_REVOLUTION = 1 (or 2 in turning depending on post),
    # check section feedMode or parameter
    for sec_idx, sec in enumerate(data.get("sections", [])):
        sec_id = sec.get("id", sec_idx)
        batch_idx = sec.get("batchIdx", 0)
        orig_sec_id = sec.get("origSecId", sec_id)
        params = sec.get("params", {})
        op_name = params.get("operation-comment", f"Section_{sec_id}")
        tool_info = sec.get("tool", {})
        tool_id = int(tool_info.get("number", 0))
        spindle_rpm = float(sec.get("initSpindleSpeed", tool_info.get("spindleRPM", 1000.0)) or 1000.0)
        sec_type = sec.get("type", 0)  # 0 = TYPE_MILLING, 1 = TYPE_TURNING
        feed_mode = sec.get("feedMode", 0)

        is_turning = (sec_type == 1) or bool(tool_info.get("_isTurningTool", False))
        is_css = is_turning and (
            tool_info.get("_spindleMode") == 1
            or tool_info.get("spindleMode") == 1
            or bool(params.get("operation:tool_useConstantSurfaceSpeed"))
            or sec.get("spindleMode") == 1
        )
        raw_surface_speed = (
            params.get("operation:tool_surfaceSpeed")
            or tool_info.get("surfaceSpeed")
            or tool_info.get("_surfaceSpeed")
            or sec.get("surfaceSpeed")
            or 0.0
        )
        surface_speed_mmpm = float(raw_surface_speed) * scale

        # Resolve maximum spindle speed
        sec_max_rpm = None
        for cand in (
            params.get("operation:tool_maximumSpindleSpeed"),
            tool_info.get("maximumSpindleSpeed"),
            tool_info.get("_maximumSpindleSpeed"),
            sec.get("maximumSpindleSpeed"),
            data.get("maximumSpindleSpeed"),
            data.get("postProperties", {}).get("maximumSpindleSpeed") if isinstance(data.get("postProperties"), dict) else None,
        ):
            if cand is not None:
                try:
                    v = float(cand)
                    if v > 0:
                        sec_max_rpm = v
                        break
                except (TypeError, ValueError):
                    pass
        max_spindle_rpm = sec_max_rpm if sec_max_rpm is not None else 5000.0

        z_clearance_mm = None
        for k in ("operation:zClearance", "operation:clearanceHeight_value"):
            if k in params and params[k] is not None:
                try:
                    z_clearance_mm = float(params[k]) * scale
                    break
                except (TypeError, ValueError):
                    pass

        z_retract_mm = None
        for k in ("operation:zRetract", "operation:retractHeight_value", "clearance"):
            if k in params and params[k] is not None:
                try:
                    z_retract_mm = float(params[k]) * scale
                    break
                except (TypeError, ValueError):
                    pass

        def _calc_move_feed_and_rpm(
            raw_feed: float,
            s_pos: Vector3D,
            e_pos: Vector3D,
            ev_rpm: Optional[float] = None,
        ) -> Tuple[float, float]:
            feed_mm = float(raw_feed) * scale
            if is_css and surface_speed_mmpm > 0:
                r_eff = 0.5 * (abs(s_pos.x) + abs(e_pos.x))
                if r_eff > 1e-4:
                    calc_rpm = surface_speed_mmpm / (2.0 * math.pi * r_eff)
                    active_rpm = min(max_spindle_rpm, calc_rpm)
                else:
                    active_rpm = max_spindle_rpm
            else:
                active_rpm = float(ev_rpm or spindle_rpm) if ev_rpm is not None else spindle_rpm
                if active_rpm <= 0:
                    active_rpm = 1000.0

            is_feed_per_rev = (
                feed_mode in (1, 3)
                or bool(params.get("operation:tool_useFeedPerRevolution"))
                or (sec_type == 1 and feed_mode != 0 and float(raw_feed) < 2.0)
            )

            if is_feed_per_rev:
                feed_mmpm = feed_mm * active_rpm
            else:
                feed_mmpm = feed_mm

            return feed_mmpm, active_rpm

        def _to_mmpm(raw_feed: float, rpm: float, fm: int) -> float:
            feed_mm = float(raw_feed) * scale
            # In turning (sec_type == 1), feedMode == 1 or small per-rev values when fm != 0
            # Wait: in Fusion Post Kernel, FEED_PER_MINUTE = 0, FEED_PER_REVOLUTION = 1
            if fm == 1 or (sec_type == 1 and fm != 0 and float(raw_feed) < 2.0):
                eff_rpm = rpm if rpm > 0 else 1000.0
                return feed_mm * eff_rpm
            return feed_mm

        pending_rc_event = False
        prev_linear_rc = int(sec.get("initRadiusComp", 0) or 0)
        in_cycle_active = False

        for ev_idx, ev in enumerate(sec.get("events", [])):
            ev_type = ev.get("type")
            if ev_type == "onCycle":
                in_cycle_active = True
                continue
            if ev_type == "onCycleEnd":
                in_cycle_active = False
                continue
            if ev_type == "onSpindleSpeed":
                spindle_rpm = float(ev.get("s", spindle_rpm) or spindle_rpm)
                continue
            if ev_type == "onFeedMode":
                feed_mode = int(ev.get("fm", feed_mode))
                continue
            if ev_type == "onRadiusCompensation":
                pending_rc_event = True
                continue

            if ev_type == "onRapid":
                start_pos = Vector3D(
                    float(ev.get("sx", 0.0)) * scale,
                    float(ev.get("sy", 0.0)) * scale,
                    float(ev.get("sz", 0.0)) * scale,
                )
                end_pos = Vector3D(
                    float(ev.get("x", 0.0)) * scale,
                    float(ev.get("y", 0.0)) * scale,
                    float(ev.get("z", 0.0)) * scale,
                )
                dist = math.hypot(end_pos.x - start_pos.x, end_pos.y - start_pos.y, end_pos.z - start_pos.z)
                if dist < 1e-9:
                    continue
                duration = (dist / max(1.0, default_rapid_feed_mmpm)) * 60.0
                segment_id += 1
                segments.append(
                    MotionSegment(
                        segment_id=segment_id,
                        op_id=op_name,
                        sec_id=sec_id,
                        motion=MotionKind.RAPID,
                        start=start_pos,
                        end=end_pos,
                        feedrate_mmpm=default_rapid_feed_mmpm,
                        spindle_rpm=float(ev.get("rpm", spindle_rpm) or spindle_rpm),
                        duration_sec=duration,
                        vector_key=VectorKey.from_floats(sec_id, start_pos, end_pos),
                        is_arc=False,
                        arc_center=None,
                        arc_plane="ZX" if sec_type == 1 else "XY",
                        metadata={
                            "tool_id": tool_id,
                            "in_cycle": in_cycle_active,
                            "batch_idx": batch_idx,
                            "batch_sec_id": orig_sec_id,
                            "sec_idx": sec_idx,
                            "event_idx": ev_idx,
                            "z_clearance_mm": z_clearance_mm,
                            "z_retract_mm": z_retract_mm,
                            "sec_type": sec_type,
                        },
                    )
                )

            elif ev_type == "onLinear":
                start_pos = Vector3D(
                    float(ev.get("sx", 0.0)) * scale,
                    float(ev.get("sy", 0.0)) * scale,
                    float(ev.get("sz", 0.0)) * scale,
                )
                end_pos = Vector3D(
                    float(ev.get("x", 0.0)) * scale,
                    float(ev.get("y", 0.0)) * scale,
                    float(ev.get("z", 0.0)) * scale,
                )
                dist = math.hypot(end_pos.x - start_pos.x, end_pos.y - start_pos.y, end_pos.z - start_pos.z)
                if dist < 1e-9:
                    continue
                cur_rc = int(ev.get("rc", 0) or 0)
                is_rc_transition = pending_rc_event or (prev_linear_rc != 0 and cur_rc == 0)
                pending_rc_event = False
                prev_linear_rc = cur_rc
                raw_ev_rpm = float(ev.get("rpm", 0.0)) if ev.get("rpm") else None
                feed_mmpm, rpm = _calc_move_feed_and_rpm(float(ev.get("feed", 0.0)), start_pos, end_pos, raw_ev_rpm)
                duration = (dist / max(1.0, feed_mmpm)) * 60.0 if feed_mmpm > 0 else 0.0
                segment_id += 1
                segments.append(
                    MotionSegment(
                        segment_id=segment_id,
                        op_id=op_name,
                        sec_id=sec_id,
                        motion=MotionKind.LINEAR,
                        start=start_pos,
                        end=end_pos,
                        feedrate_mmpm=feed_mmpm,
                        spindle_rpm=rpm,
                        duration_sec=duration,
                        vector_key=VectorKey.from_floats(sec_id, start_pos, end_pos),
                        is_arc=False,
                        arc_center=None,
                        arc_plane="ZX" if sec_type == 1 else "XY",
                        metadata={
                            "tool_id": tool_id,
                            "in_cycle": in_cycle_active or bool(ev.get("sync", False)),
                            "sync": bool(ev.get("sync", False)),
                            "movement": ev.get("movement", 1),
                            "batch_idx": batch_idx,
                            "batch_sec_id": orig_sec_id,
                            "sec_idx": sec_idx,
                            "event_idx": ev_idx,
                            "z_clearance_mm": z_clearance_mm,
                            "z_retract_mm": z_retract_mm,
                            "sec_type": sec_type,
                            "rc": cur_rc,
                            "rc_transition": is_rc_transition,
                        },
                    )
                )

            elif ev_type == "onCircular":
                start_pos = Vector3D(
                    float(ev.get("sx", 0.0)) * scale,
                    float(ev.get("sy", 0.0)) * scale,
                    float(ev.get("sz", 0.0)) * scale,
                )
                end_pos = Vector3D(
                    float(ev.get("x", 0.0)) * scale,
                    float(ev.get("y", 0.0)) * scale,
                    float(ev.get("z", 0.0)) * scale,
                )
                center_pos = Vector3D(
                    float(ev.get("cx", 0.0)) * scale,
                    float(ev.get("cy", 0.0)) * scale,
                    float(ev.get("cz", 0.0)) * scale,
                )
                raw_ev_rpm = float(ev.get("rpm", 0.0)) if ev.get("rpm") else None
                feed_mmpm, rpm = _calc_move_feed_and_rpm(float(ev.get("feed", 0.0)), start_pos, end_pos, raw_ev_rpm)
                radius_mm = float(ev.get("radius", 0.0)) * scale
                sweep_rad = abs(float(ev.get("sweep", 0.0)))
                chord = math.hypot(end_pos.x - start_pos.x, end_pos.y - start_pos.y, end_pos.z - start_pos.z)
                dist = max(chord, radius_mm * sweep_rad)
                duration = (dist / max(1.0, feed_mmpm)) * 60.0 if feed_mmpm > 0 else 0.0
                cw = bool(ev.get("cw", True))
                motion = MotionKind.ARC_CW if cw else MotionKind.ARC_CCW

                plane_idx = int(ev.get("plane", 0))
                plane_str = {0: "XY", 1: "ZX", 2: "YZ"}.get(plane_idx, "ZX" if sec_type == 1 else "XY")
                segment_id += 1
                segments.append(
                    MotionSegment(
                        segment_id=segment_id,
                        op_id=op_name,
                        sec_id=sec_id,
                        motion=motion,
                        start=start_pos,
                        end=end_pos,
                        feedrate_mmpm=feed_mmpm,
                        spindle_rpm=rpm,
                        duration_sec=duration,
                        vector_key=VectorKey.from_floats(sec_id, start_pos, end_pos),
                        is_arc=True,
                        arc_center=center_pos,
                        arc_plane=plane_str,
                        metadata={
                            "tool_id": tool_id,
                            "in_cycle": in_cycle_active,
                            "batch_idx": batch_idx,
                            "batch_sec_id": orig_sec_id,
                            "sec_idx": sec_idx,
                            "event_idx": ev_idx,
                        },
                    )
                )

            elif ev_type == "onCyclePoint":
                # Use pre-captured expandedMoves to simulate full physical hole/cycle trajectory in IPW
                for sub_mv in ev.get("expandedMoves", []):
                    sub_t = sub_mv.get("type")
                    if sub_t not in ("onRapid", "onLinear"):
                        continue
                    s_pos = Vector3D(
                        float(sub_mv.get("sx", 0.0)) * scale,
                        float(sub_mv.get("sy", 0.0)) * scale,
                        float(sub_mv.get("sz", 0.0)) * scale,
                    )
                    e_pos = Vector3D(
                        float(sub_mv.get("x", 0.0)) * scale,
                        float(sub_mv.get("y", 0.0)) * scale,
                        float(sub_mv.get("z", 0.0)) * scale,
                    )
                    is_rap = (sub_t == "onRapid")
                    if is_rap:
                        f_mmpm = default_rapid_feed_mmpm
                        rpm = float(sub_mv.get("rpm", spindle_rpm) or spindle_rpm)
                    else:
                        raw_ev_rpm = float(sub_mv.get("rpm", 0.0)) if sub_mv.get("rpm") else None
                        f_mmpm, rpm = _calc_move_feed_and_rpm(float(sub_mv.get("feed", 0.0)), s_pos, e_pos, raw_ev_rpm)
                    dist = math.hypot(e_pos.x - s_pos.x, e_pos.y - s_pos.y, e_pos.z - s_pos.z)
                    duration = (dist / max(1.0, f_mmpm)) * 60.0 if f_mmpm > 0 else 0.0
                    segment_id += 1
                    segments.append(
                        MotionSegment(
                            segment_id=segment_id,
                            op_id=op_name,
                            sec_id=sec_id,
                            motion=MotionKind.RAPID if is_rap else MotionKind.LINEAR,
                            start=s_pos,
                            end=e_pos,
                            feedrate_mmpm=f_mmpm,
                            spindle_rpm=rpm,
                            duration_sec=duration,
                            vector_key=VectorKey.from_floats(sec_id, s_pos, e_pos),
                            is_arc=False,
                            arc_center=None,
                            arc_plane="ZX" if sec_type == 1 else "XY",
                            metadata={
                                "tool_id": tool_id,
                                "in_cycle": True,
                                "batch_idx": batch_idx,
                                "batch_sec_id": orig_sec_id,
                                "sec_idx": sec_idx,
                                "event_idx": ev_idx,
                            },
                        )
                    )

            elif ev_type == "onDwell":
                dwell_sec = float(ev.get("sec", 0.0))
                if segments:
                    cur_p = segments[-1].end
                else:
                    init_p = sec.get("initialPosition", {"x": 0.0, "y": 0.0, "z": 0.0})
                    cur_p = Vector3D(float(init_p["x"]) * scale, float(init_p["y"]) * scale, float(init_p["z"]) * scale)
                segment_id += 1
                segments.append(
                    MotionSegment(
                        segment_id=segment_id,
                        op_id=op_name,
                        sec_id=sec_id,
                        motion=MotionKind.DWELL,
                        start=cur_p,
                        end=cur_p,
                        feedrate_mmpm=0.0,
                        spindle_rpm=spindle_rpm,
                        duration_sec=dwell_sec,
                        vector_key=VectorKey.from_floats(sec_id, cur_p, cur_p),
                        is_arc=False,
                        arc_center=None,
                        metadata={
                            "tool_id": tool_id,
                            "in_cycle": in_cycle_active,
                            "batch_idx": batch_idx,
                            "batch_sec_id": orig_sec_id,
                            "sec_idx": sec_idx,
                            "event_idx": ev_idx,
                        },
                    )
                )

    return segments


def apply_promotions_to_context(
    context_data: Dict[str, Any],
    promoted_moves: Union[Set[PromotedEventKey], Sequence[PromotedEventKey]],
) -> int:
    """
    Annotate authorized `onLinear` moves directly inside `context_data` with
    `ev["promote"] = True`, skipping moves inside canned cycles or speed-feed synchronization.

    Addresses events strictly by per-event key `(sec_idx, event_idx)`.

    Returns the total number of individual `onLinear` move events promoted.
    """
    if not promoted_moves:
        return 0

    sections = context_data.get("sections", [])
    promoted_count = 0
    event_set: Set[Tuple[int, int]] = set(promoted_moves) if not isinstance(promoted_moves, set) else promoted_moves

    for s_idx, sec in enumerate(sections):
        in_cycle = False
        events = sec.get("events", [])
        for e_idx, ev in enumerate(events):
            ev_type = ev.get("type")
            if ev_type == "onCycle":
                in_cycle = True
                continue
            if ev_type == "onCycleEnd":
                in_cycle = False
                continue
            if (s_idx, e_idx) in event_set:
                # Verify event eligibility (defense in depth)
                if (
                    ev_type == "onLinear"
                    and not in_cycle
                    and not ev.get("sync", False)
                    and not ev.get("in_cycle", False)
                    and int(ev.get("rc", 0) or 0) == 0
                ):
                    ev["promote"] = True
                    promoted_count += 1

    return promoted_count


DEFAULT_MIN_PROMOTE_LEN_MM: float = 1.5


def filter_promoted_events_by_length(
    promoted_events: Sequence[Tuple[int, int]],
    segments: Sequence[MotionSegment],
    min_len_mm: float = DEFAULT_MIN_PROMOTE_LEN_MM,
) -> Set[Tuple[int, int]]:
    """
    Filter promoted event keys `(sec_idx, event_idx)` to keep only events whose
    3D travel length across `segments` is `>= min_len_mm` (in mm).

    Fails closed (drops the key) if an event key is not found in `segments`.
    Setting `min_len_mm=0.0` disables length filtering.
    """
    val = float(min_len_mm)
    if not math.isfinite(val) or val < 0.0:
        raise ValueError(f"min_len_mm must be a finite number >= 0, got {min_len_mm!r}")
    if not promoted_events:
        return set()
    event_set: Set[Tuple[int, int]] = set(promoted_events)
    if val == 0.0:
        return event_set

    event_lengths: Dict[Tuple[int, int], float] = {}
    for seg in segments:
        s_meta = getattr(seg, "metadata", None) or {}
        sec_i = s_meta.get("sec_idx", seg.sec_id)
        ev_i = s_meta.get("event_idx")
        if ev_i is None:
            ev_i = seg.segment_id
        ev_k = (sec_i, ev_i)
        seg_len = math.sqrt(
            (seg.end.x - seg.start.x) ** 2
            + (seg.end.y - seg.start.y) ** 2
            + (seg.end.z - seg.start.z) ** 2
        )
        event_lengths[ev_k] = event_lengths.get(ev_k, 0.0) + seg_len

    return {
        ev for ev in event_set
        if event_lengths.get(ev, 0.0) >= val
    }


def extract_workpiece_from_context(
    context_data: Dict[str, Any],
    fallback_workpiece: Any = None,
    is_turning: Optional[bool] = None,
) -> Any:
    """
    Construct a metric `WorkpieceDefinition` (`mm`) from `context_data["workpiece"]`
    (and chuck obstacles / stock dimensions from globalParams if turning).
    Supports:
    - Turning: AxisymmetricStock2D with AxisymmetricChuckObstacle fixtures.
    - Milling: BoxStock3D (prismatic stock bounding box).
    """
    from geometry.models import (
        AxisymmetricChuckObstacle,
        AxisymmetricStock2D,
        BoxStock3D,
        WorkpieceDefinition,
    )

    def _dbg_ctx(msg: str) -> None:
        try:
            from PostClear import _dbg_global
            _dbg_global(msg)
        except Exception:
            pass

    stock_mode = str(
        context_data.get("stockMode")
        or context_data.get("globalParams", {}).get("job:stockMode")
        or context_data.get("globalParams", {}).get("stockMode")
        or ""
    ).lower()
    _dbg_ctx(f"extract_workpiece_from_context: stock_mode='{stock_mode}', fallback={fallback_workpiece}")

    is_previous_mode = any(k in stock_mode for k in ("previous", "preceding", "prev", "rest"))

    # 1. Native exported STL stock mesh (only for previous/preceding modes)
    stl_path = context_data.get("stockPath")
    if stl_path:
        if is_previous_mode:
            if not os.path.isfile(stl_path):
                raise FileNotFoundError(
                    f"Exported stock STL mesh file not found: '{stl_path}'."
                )
            if np is None:
                try:
                    import numpy as _np
                    globals()["np"] = _np
                except ImportError as exc:
                    raise RuntimeError(
                        "Loading in-process STL stock requires 'numpy', which is not installed "
                        "(offline first-time setup failed). Connect to the internet and re-run Post Clear."
                    ) from exc
            try:
                from geometry.solid_mesh import (
                    load_stl_mesh,
                    revolve_mesh_to_lathe_stock,
                    SolidMeshStock3D,
                )
                from geometry.models import WorkpieceDefinition

                try:
                    raw_verts, raw_indices = load_stl_mesh(stl_path, is_inch=False)
                except Exception as stl_err:
                    raise RuntimeError(
                        f"Failed to load stock from STL mesh '{stl_path}': {stl_err}"
                    ) from stl_err

                stl_min = raw_verts.min(axis=0)
                stl_max = raw_verts.max(axis=0)
                stl_extents = stl_max - stl_min

                # Base stock unit determination (Phase 1 H2)
                stock_unit_str = context_data.get("stockUnit")
                if stock_unit_str:
                    is_inch = str(stock_unit_str).lower() in ("in", "inch", "inches")
                else:
                    is_inch = (context_data.get("unit", 0) == 0)

                # Cross-check against workpieceHint if captured
                hint = context_data.get("workpieceHint")
                if hint and isinstance(hint, dict) and "lower" in hint and "upper" in hint:
                    hint_lower = hint["lower"]
                    hint_upper = hint["upper"]
                    hint_extents = np.array([
                        abs(float(hint_upper.get("x", 0.0)) - float(hint_lower.get("x", 0.0))),
                        abs(float(hint_upper.get("y", 0.0)) - float(hint_lower.get("y", 0.0))),
                        abs(float(hint_upper.get("z", 0.0)) - float(hint_lower.get("z", 0.0))),
                    ], dtype=np.float64)

                    candidate_scales = [1.0, 25.4, 1.0 / 25.4]
                    active_dims = (hint_extents > 1e-4) & (stl_extents > 1e-4)
                    if np.any(active_dims):
                        best_scale = None
                        min_err = float("inf")
                        for s in candidate_scales:
                            scaled_stl = stl_extents[active_dims] * s
                            target_hint = hint_extents[active_dims]
                            rel_errors = np.abs(scaled_stl - target_hint) / np.maximum(target_hint, 1e-6)
                            # In-process stock from a preceding setup may have 1-2 axes machined smaller
                            # than workpieceHint while keeping at least one axis unmachined and never
                            # exceeding workpieceHint by > 5%.
                            if np.all(scaled_stl <= target_hint * 1.05):
                                scale_err = float(np.min(rel_errors))
                            else:
                                scale_err = float(np.max(rel_errors))
                            if scale_err < min_err:
                                min_err = scale_err
                                best_scale = s

                        if min_err > 0.05:
                            raise ValueError(
                                f"STL bounding box extents {stl_extents.tolist()} do not match workpieceHint "
                                f"{hint_extents.tolist()} within 5% for any candidate scale {candidate_scales} "
                                f"(best match scale {best_scale} error: {min_err*100:.1f}%)."
                            )

                        if abs(best_scale - 25.4) < 1e-3:
                            is_inch = True
                        elif abs(best_scale - (1.0 / 25.4)) < 1e-3:
                            is_inch = False
                        elif abs(best_scale - 1.0) < 1e-3:
                            is_inch = (context_data.get("unit", 0) == 0)

                turning_mode = is_turning
                if turning_mode is None:
                    turning_mode = any(
                        sec.get("type") == 1 or bool(sec.get("tool", {}).get("_isTurningTool", False))
                        for sec in context_data.get("sections", [])
                    )
                fb_fixtures = getattr(fallback_workpiece, "fixtures", None) if fallback_workpiece else None
                _dbg_ctx(f"extract_workpiece_from_context: loading stock from exported STL '{stl_path}' (turning={turning_mode}, inch={is_inch})")

                vertices_mm = raw_verts * 25.4 if is_inch else raw_verts
                if turning_mode:
                    wp = revolve_mesh_to_lathe_stock(
                        vertices_mm,
                        raw_indices,
                        max_radial_offset_mm=None,
                        fixtures=fb_fixtures,
                    )
                else:
                    stock = SolidMeshStock3D.from_mesh(vertices_mm, raw_indices)
                    _dbg_ctx(
                        f"extract_workpiece_from_context: created SolidMeshStock3D from '{stl_path}' "
                        f"X=[{stock.x_min:.2f}, {stock.x_max:.2f}], Y=[{stock.y_min:.2f}, {stock.y_max:.2f}], Z=[{stock.z_min:.2f}, {stock.z_max:.2f}]"
                    )
                    wp = WorkpieceDefinition(stock=stock, fixtures=fb_fixtures or [])
                wp.is_stl_derived = True
                return wp
            except (ValueError, FileNotFoundError, RuntimeError):
                raise
            except Exception as stl_err:
                raise RuntimeError(
                    f"Failed to load stock from STL mesh '{stl_path}': {stl_err}"
                ) from stl_err
        else:
            _dbg_ctx(
                f"extract_workpiece_from_context: ignoring stockPath '{stl_path}' "
                f"because stockMode is '{stock_mode}' (not previous/preceding)"
            )

    if is_previous_mode:
        return fallback_workpiece
    if any(k in stock_mode for k in ("solid", "body")):
        return fallback_workpiece

    wp = context_data.get("workpiece")
    if not wp or not isinstance(wp, dict):
        stock_data = context_data.get("stock")
        if isinstance(stock_data, dict):
            if "lower" in stock_data and "upper" in stock_data:
                wp = stock_data
            elif "x_min" in stock_data:
                wp = {
                    "lower": {
                        "x": stock_data.get("x_min", 0.0),
                        "y": stock_data.get("y_min", 0.0),
                        "z": stock_data.get("z_min", 0.0),
                    },
                    "upper": {
                        "x": stock_data.get("x_max", 0.0),
                        "y": stock_data.get("y_max", 0.0),
                        "z": stock_data.get("z_max", 0.0),
                    },
                }
    if not wp or not isinstance(wp, dict):
        return fallback_workpiece

    lower = wp.get("lower") or {}
    upper = wp.get("upper") or {}
    unit = context_data.get("unit", 0)
    scale = 25.4 if unit == 0 else 1.0

    if is_turning is None:
        # Auto-detect turning if any section is turning type (1) or has turning tool
        is_turning = any(
            sec.get("type") == 1 or bool(sec.get("tool", {}).get("_isTurningTool", False))
            for sec in context_data.get("sections", [])
        )

    if not is_turning:
        x_min = float(lower.get("x", 0.0)) * scale
        x_max = float(upper.get("x", 0.0)) * scale
        y_min = float(lower.get("y", 0.0)) * scale
        y_max = float(upper.get("y", 0.0)) * scale
        z_min = float(lower.get("z", 0.0)) * scale
        z_max = float(upper.get("z", 0.0)) * scale
        if x_max <= x_min + 1e-6 or y_max <= y_min + 1e-6 or z_max <= z_min + 1e-6:
            return fallback_workpiece
        return WorkpieceDefinition(
            stock=BoxStock3D(
                x_min=x_min,
                x_max=x_max,
                y_min=y_min,
                y_max=y_max,
                z_min=z_min,
                z_max=z_max,
            ),
            fixtures=[],
        )

    z_min = float(lower.get("z", 0.0)) * scale
    z_max = float(upper.get("z", 0.0)) * scale
    r_outer = max(
        abs(float(upper.get("x", 0.0))),
        abs(float(lower.get("x", 0.0))),
        abs(float(upper.get("y", 0.0))),
        abs(float(lower.get("y", 0.0))),
    ) * scale

    if r_outer <= 1e-6 or z_max <= z_min + 1e-6:
        return fallback_workpiece

    # Inner radius from globalParams or fallback_workpiece
    r_inner = 0.0
    g_params = context_data.get("globalParams") or {}
    for k in ("stock-diameter-inner", "job:stockDiameterInner"):
        if k in g_params and g_params[k]:
            try:
                r_inner = float(g_params[k]) * 0.5 * scale
                break
            except (TypeError, ValueError):
                pass

    fb_stock = getattr(fallback_workpiece, "stock", None) if fallback_workpiece else None
    if r_inner <= 0.0 and fb_stock is not None and hasattr(fb_stock, "inner_radius"):
        fb_r_in = float(fb_stock.inner_radius)
        if 0.0 < fb_r_in < r_outer:
            r_inner = fb_r_in

    fixtures = []
    fb_fixtures = getattr(fallback_workpiece, "fixtures", None) if fallback_workpiece else None
    if fb_fixtures:
        dz = (z_min - float(fb_stock.z_min)) if (fb_stock is not None and hasattr(fb_stock, "z_min")) else 0.0
        for f in fb_fixtures:
            if hasattr(f, "z_min") and hasattr(f, "z_max"):
                fixtures.append(
                    AxisymmetricChuckObstacle(
                        z_min=float(f.z_min) + dz,
                        z_max=float(f.z_max) + dz,
                        outer_radius=max(float(getattr(f, "outer_radius", r_outer + 30.0)), r_outer + 10.0),
                        clamping_radius=r_outer,
                    )
                )
            else:
                fixtures.append(f)

    return WorkpieceDefinition(
        stock=AxisymmetricStock2D(
            z_min=z_min,
            z_max=z_max,
            outer_radius=r_outer,
            inner_radius=r_inner,
        ),
        fixtures=fixtures,
    )


def extract_tools_from_context(
    context_data: Dict[str, Any],
    fallback_tools: Any = None,
    is_turning: Optional[bool] = None,
) -> Dict[str, Any]:
    """
    Construct exact metric `ToolAssembly` objects (`mm`) from the kernel-provided
    profiles in `context_data["sections"]`.
    - Turning operations: LatheToolGeometry using 2D SVG profiles (_cutterSVG / _holderSVG).
    - Milling operations: MillToolGeometry using parametric tool dimensions.
    """
    from geometry.models import (
        LatheToolGeometry,
        MillToolGeometry,
        ToolAssembly,
        ToolCompensationQuadrant,
    )
    from geometry.legacy_adapter import transform_tool_svg

    unit = context_data.get("unit", 0)
    scale = 25.4 if unit == 0 else 1.0
    tools_by_op: Dict[str, Any] = dict(fallback_tools) if fallback_tools else {}

    for sec_idx, sec in enumerate(context_data.get("sections", [])):
        sec_id = sec.get("id", sec_idx)
        sec_type = sec.get("type", 0)
        params = sec.get("params") or {}
        op_name = params.get("operation-comment", f"Section_{sec_id}")
        t = sec.get("tool") or {}
        cutter_svg = t.get("_cutterSVG", "")
        holder_svg = t.get("_holderSVG", "")

        is_turning_tool = bool(t.get("_isTurningTool", False)) or (sec_type == 1)
        if is_turning is False:
            is_turning_tool = False
        elif is_turning is True:
            is_turning_tool = True

        if not is_turning_tool or not cutter_svg:
            # Construct MillToolGeometry for milling tools
            dia = float(t.get("diameter", 0.0) or 0.0) * scale
            cr = float(t.get("cornerRadius", t.get("corner_radius", t.get("noseRadius", 0.0))) or 0.0) * scale
            flute_l = float(t.get("fluteLength", t.get("flute_length", 0.0)) or 0.0) * scale
            overall_l = float(t.get("overallLength", t.get("overall_length", 0.0)) or 0.0) * scale
            raw_taper = t.get("_taperAngle", t.get("taperAngle", t.get("taper_angle", 0.0))) or 0.0
            taper_val = float(raw_taper)
            if 0.0 < taper_val <= math.pi:
                taper_a = math.degrees(taper_val)
            else:
                taper_a = taper_val
            holder_d = float(t.get("holderDiameter", t.get("holder_diameter", 40.0)) or 40.0) * scale
            holder_l = float(t.get("holderLength", t.get("holder_length", 60.0)) or 60.0) * scale
            mill_geom = MillToolGeometry(
                diameter=dia,
                corner_radius=cr,
                flute_length=flute_l,
                overall_length=overall_l,
                taper_angle_deg=taper_a,
                holder_diameter=holder_d,
                holder_length=holder_l,
            )
            tools_by_op[op_name] = ToolAssembly(
                tool_id=int(t.get("number", 0) or 0),
                name=op_name,
                geometry=mill_geom,
            )
            continue

        try:
            from shapely.affinity import scale as shapely_scale
        except (ImportError, OSError):
            continue

        comp_d = t.get("_compDisplacement") or {"x": 0.0, "y": 0.0}
        comp = (float(comp_d.get("x", 0.0)), float(comp_d.get("y", 0.0)))
        if "operation:toolOrientation" in params:
            orient_deg = float(params["operation:toolOrientation"])
        else:
            orient_deg = float(sec.get("toolOrientation", 0.0) or 0.0) * 180.0 / math.pi

        ttype = str(t.get("_toolTypeName", "") or "")
        desc = str(t.get("_description", "") or t.get("description", "") or "")
        is_boring = (
            "boring" in ttype.lower()
            or "bore" in ttype.lower()
            or "boring" in desc.lower()
            or "bore" in op_name.lower()
        )
        if not ttype and is_boring:
            ttype = "boring turning"

        c_poly = transform_tool_svg(cutter_svg, orient_deg, comp, "REAR", tool_type=ttype)
        if c_poly.is_empty:
            continue
        h_poly = transform_tool_svg(holder_svg, orient_deg, comp, "REAR", tool_type=ttype) if holder_svg else c_poly.difference(c_poly)

        c_poly_mm = shapely_scale(c_poly, xfact=scale, yfact=scale, origin=(0, 0))
        h_poly_mm = shapely_scale(h_poly, xfact=scale, yfact=scale, origin=(0, 0)) if not h_poly.is_empty else h_poly
        h_clean_mm = h_poly_mm.difference(c_poly_mm.buffer(0.0015 * scale)) if not h_poly_mm.is_empty else h_poly_mm

        geom = LatheToolGeometry(
            insert_shape=ttype or "rhombic",
            nose_radius=float(t.get("noseRadius", t.get("cornerRadius", 0.0)) or 0.0) * scale,
            hand=str(t.get("hand", "R") or "R"),
            quadrant=ToolCompensationQuadrant.P3,
            cutter_polygon=c_poly_mm,
            holder_polygon=h_clean_mm,
            is_boring=is_boring,
        )
        tools_by_op[op_name] = ToolAssembly(
            tool_id=int(t.get("number", 0) or 0),
            name=op_name,
            geometry=geom,
        )

    return tools_by_op
