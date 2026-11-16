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

from dep_bootstrap import dbg_log

from geometry.models import (
    MotionKind,
    MotionSegment,
    Vector3D,
    PromotedEventKey,
)


def classify_stock_mode(raw: Any) -> str:
    """Normalize a Fusion/context stockMode string into a canonical stock category."""
    s = str(raw or "").lower().strip()
    if any(k in s for k in ("previous", "preceding", "prev", "rest")):
        return "previous_setup"
    if any(k in s for k in ("solid", "body")):
        return "solid"
    if "tube" in s:
        return "tube"
    if "cylinder" in s or "round" in s:
        return "cylinder"
    if any(k in s for k in ("box", "fixed", "relative")):
        return "box"
    return "unknown"


def is_turning_section(sec: Dict[str, Any]) -> bool:
    """Return True if a context section dictionary represents turning."""
    if not isinstance(sec, dict):
        return False
    tool_info = sec.get("tool")
    return (sec.get("type", 0) == 1) or bool(isinstance(tool_info, dict) and tool_info.get("_isTurningTool", False))


def is_turning_context(ctx_data: Dict[str, Any]) -> bool:
    """Return True if any section in a context dump represents turning."""
    if not isinstance(ctx_data, dict):
        return False
    return any(is_turning_section(sec) for sec in (ctx_data.get("sections") or []))


def _bbox_dict_to_mm(
    box: Any,
    unit: Any,
) -> Optional[Tuple[float, float, float, float, float, float]]:
    """Convert a `{"lower": {x,y,z}, "upper": {x,y,z}}` box dict in `unit` to mm bounds."""
    if not isinstance(box, dict):
        return None
    lo = box.get("lower")
    hi = box.get("upper")
    if not isinstance(lo, dict) or not isinstance(hi, dict):
        return None
    try:
        scale = 25.4 if int(unit if unit is not None else 0) == 0 else 1.0
        x0, x1 = float(lo["x"]) * scale, float(hi["x"]) * scale
        y0, y1 = float(lo["y"]) * scale, float(hi["y"]) * scale
        z0, z1 = float(lo["z"]) * scale, float(hi["z"]) * scale
    except (KeyError, TypeError, ValueError):
        return None
    if not (x1 > x0 + 1e-6 and y1 > y0 + 1e-6 and z1 > z0 + 1e-6):
        return None
    return (x0, x1, y0, y1, z0, z1)


def extract_workpiece_hint_bbox_mm(
    ctx_data: Dict[str, Any],
) -> Optional[Tuple[float, float, float, float, float, float]]:
    """Return `workpieceHint` bounds `(x_min, x_max, y_min, y_max, z_min, z_max)` in mm, or None."""
    if not isinstance(ctx_data, dict):
        return None
    return _bbox_dict_to_mm(ctx_data.get("workpieceHint"), ctx_data.get("unit", 0))


def _as_context_dict(source: Union[str, Dict[str, Any]]) -> Dict[str, Any]:
    """Read-only context accessor: load JSON if `source` is a path, or return `dict` as-is."""
    if isinstance(source, str):
        with open(source, "r", encoding="utf-8") as f:
            return json.load(f)
    if isinstance(source, dict):
        return source
    raise TypeError(f"Expected file path or dict, got {type(source)}")


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

    loaded: List[Tuple[Dict[str, Any], bool]] = []
    for s in sources:
        if isinstance(s, str):
            with open(s, "r", encoding="utf-8") as f:
                loaded.append((json.load(f), True))
        elif isinstance(s, dict):
            loaded.append((s, False))
        else:
            raise TypeError(f"Expected file path or dict, got {type(s)}")

    first_dump, first_fresh = loaded[0]
    merged: Dict[str, Any] = {
        k: (v if first_fresh else copy.deepcopy(v))
        for k, v in first_dump.items()
        if k != "sections"
    }
    merged["sections"] = []
    if not isinstance(merged.get("globalParams"), dict):
        merged["globalParams"] = {}

    if not merged.get("maximumSpindleSpeed"):
        for d, _ in loaded:
            if d.get("maximumSpindleSpeed"):
                merged["maximumSpindleSpeed"] = d["maximumSpindleSpeed"]
                break

    if "rapidMode" not in merged:
        for d, is_fresh in loaded:
            if "rapidMode" in d:
                merged["rapidMode"] = d["rapidMode"] if is_fresh else copy.deepcopy(d["rapidMode"])
                break

    if not merged.get("workpieceHint"):
        for d, is_fresh in loaded:
            if d.get("workpieceHint"):
                merged["workpieceHint"] = d["workpieceHint"] if is_fresh else copy.deepcopy(d["workpieceHint"])
                break

    global_sec_id = 0
    for d, is_fresh in loaded:
        for k, v in (d.get("globalParams") or {}).items():
            if k not in merged["globalParams"]:
                merged["globalParams"][k] = v if is_fresh else copy.deepcopy(v)

        for sec in d.get("sections", []):
            sec_copy = sec if is_fresh else copy.deepcopy(sec)
            sec_copy["id"] = global_sec_id
            merged["sections"].append(sec_copy)
            global_sec_id += 1

    return merged


def _classify_arc_normal(
    nx: float,
    ny: float,
    nz: float,
    cw: bool,
    default_plane: str,
) -> Tuple[str, bool, bool]:
    """Map an arc plane normal vector to `("XY"|"ZX"|"YZ", cw_flipped, tilted_arc)`."""
    if abs(nx) < 1e-5 and abs(ny) < 1e-5 and abs(abs(nz) - 1.0) < 1e-4:
        return "XY", (not cw if nz < 0.0 else cw), False
    if abs(nx) < 1e-5 and abs(nz) < 1e-5 and abs(abs(ny) - 1.0) < 1e-4:
        return "ZX", (not cw if ny < 0.0 else cw), False
    if abs(ny) < 1e-5 and abs(nz) < 1e-5 and abs(abs(nx) - 1.0) < 1e-4:
        return "YZ", (not cw if nx < 0.0 else cw), False
    return default_plane, cw, True


def parse_context_dump(
    source: Union[str, Dict[str, Any]],
    default_rapid_feed_mmpm: float = 5000.0,
) -> List[MotionSegment]:
    """
    Translate a single or merged `universal_dumper.cps` JSON context dump into a
    normalized metric `List[MotionSegment]` for `IVerificationEngine`.
    """
    data = _as_context_dict(source)
    # In CAM360/post: IN = 0, MM = 1
    unit = data.get("unit", 0)
    scale = 25.4 if unit == 0 else 1.0

    segments: List[MotionSegment] = []
    segment_id = 0

    # In CAM360/post: FEED_PER_REVOLUTION = 1 (or 2 in turning depending on post),
    # check section feedMode or parameter
    for sec_idx, sec in enumerate(data.get("sections", [])):
        sec_id = sec.get("id", sec_idx)
        params = sec.get("params", {})
        op_name = params.get("operation-comment", f"Section_{sec_id}")
        tool_info = sec.get("tool", {})
        spindle_rpm = float(sec.get("initSpindleSpeed", tool_info.get("spindleRPM", 1000.0)) or 1000.0)
        sec_type = sec.get("type", 0)  # 0 = TYPE_MILLING, 1 = TYPE_TURNING
        feed_mode = sec.get("feedMode", 0)

        is_turning = is_turning_section(sec)
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

        # Section Working Plane -> Setup WCS transformation:
        # Raw section events are recorded in the section's Working Plane coordinate
        # system (`cancelTransformation()` in universal_dumper.cps). Convert to Setup WCS
        # via `p_wcs = workPlane * p_local + workOrigin` so IPW simulation and stock
        # overlap checks align with the Setup WCS stock geometry.
        raw_wp = sec.get("workPlane")
        if isinstance(raw_wp, dict):
            wp_r = raw_wp.get("right") if isinstance(raw_wp.get("right"), dict) else {}
            wp_u = raw_wp.get("up") if isinstance(raw_wp.get("up"), dict) else {}
            wp_f = raw_wp.get("forward") if isinstance(raw_wp.get("forward"), dict) else {}
            wp_rx, wp_ry, wp_rz = float(wp_r.get("x", 1.0)), float(wp_r.get("y", 0.0)), float(wp_r.get("z", 0.0))
            wp_ux, wp_uy, wp_uz = float(wp_u.get("x", 0.0)), float(wp_u.get("y", 1.0)), float(wp_u.get("z", 0.0))
            wp_fx, wp_fy, wp_fz = float(wp_f.get("x", 0.0)), float(wp_f.get("y", 0.0)), float(wp_f.get("z", 1.0))
        else:
            wp_rx, wp_ry, wp_rz = 1.0, 0.0, 0.0
            wp_ux, wp_uy, wp_uz = 0.0, 1.0, 0.0
            wp_fx, wp_fy, wp_fz = 0.0, 0.0, 1.0

        raw_wo = sec.get("workOrigin")
        if isinstance(raw_wo, dict):
            wo_x, wo_y, wo_z = float(raw_wo.get("x", 0.0)), float(raw_wo.get("y", 0.0)), float(raw_wo.get("z", 0.0))
        else:
            wo_x, wo_y, wo_z = 0.0, 0.0, 0.0

        has_sec_transform = not (
            abs(wp_rx - 1.0) < 1e-12 and abs(wp_ry) < 1e-12 and abs(wp_rz) < 1e-12
            and abs(wp_ux) < 1e-12 and abs(wp_uy - 1.0) < 1e-12 and abs(wp_uz) < 1e-12
            and abs(wp_fx) < 1e-12 and abs(wp_fy) < 1e-12 and abs(wp_fz - 1.0) < 1e-12
            and abs(wo_x) < 1e-12 and abs(wo_y) < 1e-12 and abs(wo_z) < 1e-12
        )
        # R4: Turning verifier operates in lathe-local X/Z coordinates; skip work-plane transform.
        if sec_type == 1:
            has_sec_transform = False

        wp_det = (
            wp_rx * (wp_uy * wp_fz - wp_uz * wp_fy)
            - wp_ux * (wp_ry * wp_fz - wp_rz * wp_fy)
            + wp_fx * (wp_ry * wp_uz - wp_rz * wp_uy)
        )
        is_z_aligned_wp = (
            abs(wp_fz - 1.0) < 1e-5 and abs(wp_rz) < 1e-5 and abs(wp_uz) < 1e-5
        )
        # R2: Heightfield verifier models a vertical +Z tool; sections with forward != +Z cannot be verified.
        is_tilted_section = (sec_type != 1) and not (
            abs(wp_fx) < 1e-5 and abs(wp_fy) < 1e-5 and abs(wp_fz - 1.0) < 1e-5
        )
        # R8: Simultaneous multi-axis sections are not supported by 3-axis IPW verification.
        is_multi_axis = bool(sec.get("isMultiAxis", False)) or bool(sec.get("has5DMoves", False))

        default_plane = "ZX" if sec_type == 1 else "XY"

        def _to_wcs_pos(x_raw: float, y_raw: float, z_raw: float) -> Vector3D:
            if not has_sec_transform:
                return Vector3D(x_raw * scale, y_raw * scale, z_raw * scale)
            return Vector3D(
                (wp_rx * x_raw + wp_ux * y_raw + wp_fx * z_raw + wo_x) * scale,
                (wp_ry * x_raw + wp_uy * y_raw + wp_fy * z_raw + wo_y) * scale,
                (wp_rz * x_raw + wp_uz * y_raw + wp_fz * z_raw + wo_z) * scale,
            )

        def _resolve_z_param(keys: Tuple[str, ...]) -> Optional[float]:
            if has_sec_transform and not is_z_aligned_wp:
                return None
            for k in keys:
                if k in params and params[k] is not None:
                    try:
                        raw_z = float(params[k])
                        return (raw_z + wo_z) * scale if has_sec_transform else raw_z * scale
                    except (TypeError, ValueError):
                        pass
            return None

        z_clearance_mm = _resolve_z_param(("operation:zClearance", "operation:clearanceHeight_value"))
        z_retract_mm = _resolve_z_param(("operation:zRetract", "operation:retractHeight_value", "clearance"))

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

        pending_rc_event = False
        prev_linear_rc = int(sec.get("initRadiusComp", 0) or 0)
        in_cycle_active = False

        for ev_idx, ev in enumerate(sec.get("events", [])):
            def _emit_segment(
                motion: MotionKind,
                start: Vector3D,
                end: Vector3D,
                feedrate_mmpm: float,
                rpm: float,
                duration_sec: float,
                *,
                in_cycle: bool,
                is_arc: bool = False,
                arc_center: Optional[Vector3D] = None,
                arc_plane: Optional[str] = None,
                extra_meta: Optional[Dict[str, Any]] = None,
            ) -> None:
                nonlocal segment_id
                segment_id += 1
                meta: Dict[str, Any] = {
                    "in_cycle": in_cycle,
                    "sec_idx": sec_idx,
                    "event_idx": ev_idx,
                    "z_clearance_mm": z_clearance_mm,
                    "z_retract_mm": z_retract_mm,
                    "sec_type": sec_type,
                }
                if extra_meta:
                    meta.update(extra_meta)
                if is_tilted_section:
                    meta["tilted_section"] = True
                if is_multi_axis:
                    meta["multi_axis"] = True
                segments.append(
                    MotionSegment(
                        segment_id=segment_id,
                        op_id=op_name,
                        sec_id=sec_id,
                        motion=motion,
                        start=start,
                        end=end,
                        feedrate_mmpm=feedrate_mmpm,
                        spindle_rpm=rpm,
                        duration_sec=duration_sec,
                        is_arc=is_arc,
                        arc_center=arc_center,
                        arc_plane=arc_plane if arc_plane is not None else default_plane,
                        metadata=meta,
                    )
                )

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
                start_pos = _to_wcs_pos(
                    float(ev.get("sx", 0.0)),
                    float(ev.get("sy", 0.0)),
                    float(ev.get("sz", 0.0)),
                )
                end_pos = _to_wcs_pos(
                    float(ev.get("x", 0.0)),
                    float(ev.get("y", 0.0)),
                    float(ev.get("z", 0.0)),
                )
                dist = math.hypot(end_pos.x - start_pos.x, end_pos.y - start_pos.y, end_pos.z - start_pos.z)
                if dist < 1e-9:
                    continue
                duration = (dist / max(1.0, default_rapid_feed_mmpm)) * 60.0
                _emit_segment(
                    MotionKind.RAPID,
                    start_pos,
                    end_pos,
                    default_rapid_feed_mmpm,
                    float(ev.get("rpm", spindle_rpm) or spindle_rpm),
                    duration,
                    in_cycle=in_cycle_active,
                )

            elif ev_type == "onLinear":
                start_pos = _to_wcs_pos(
                    float(ev.get("sx", 0.0)),
                    float(ev.get("sy", 0.0)),
                    float(ev.get("sz", 0.0)),
                )
                end_pos = _to_wcs_pos(
                    float(ev.get("x", 0.0)),
                    float(ev.get("y", 0.0)),
                    float(ev.get("z", 0.0)),
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
                is_sync = bool(ev.get("sync", False))
                _emit_segment(
                    MotionKind.LINEAR,
                    start_pos,
                    end_pos,
                    feed_mmpm,
                    rpm,
                    duration,
                    in_cycle=in_cycle_active or is_sync,
                    extra_meta={
                        "sync": is_sync,
                        "movement": ev.get("movement", 1),
                        "rc": cur_rc,
                        "rc_transition": is_rc_transition,
                    },
                )

            elif ev_type == "onCircular":
                start_pos = _to_wcs_pos(
                    float(ev.get("sx", 0.0)),
                    float(ev.get("sy", 0.0)),
                    float(ev.get("sz", 0.0)),
                )
                end_pos = _to_wcs_pos(
                    float(ev.get("x", 0.0)),
                    float(ev.get("y", 0.0)),
                    float(ev.get("z", 0.0)),
                )
                center_pos = _to_wcs_pos(
                    float(ev.get("cx", 0.0)),
                    float(ev.get("cy", 0.0)),
                    float(ev.get("cz", 0.0)),
                )
                raw_ev_rpm = float(ev.get("rpm", 0.0)) if ev.get("rpm") else None
                feed_mmpm, rpm = _calc_move_feed_and_rpm(float(ev.get("feed", 0.0)), start_pos, end_pos, raw_ev_rpm)
                radius_mm = float(ev.get("radius", 0.0)) * scale
                sweep_rad = abs(float(ev.get("sweep", 0.0)))
                chord = math.hypot(end_pos.x - start_pos.x, end_pos.y - start_pos.y, end_pos.z - start_pos.z)
                dist = max(chord, radius_mm * sweep_rad)
                duration = (dist / max(1.0, feed_mmpm)) * 60.0 if feed_mmpm > 0 else 0.0
                cw = bool(ev.get("cw", True))
                plane_idx = int(ev.get("plane", 0))
                plane_str = {0: "XY", 1: "ZX", 2: "YZ"}.get(plane_idx, default_plane)
                tilted_arc = False
                if sec_type != 1 and (has_sec_transform or plane_idx not in (0, 1, 2)):
                    has_explicit_normal = "nx" in ev and "ny" in ev and "nz" in ev
                    if not has_sec_transform and not has_explicit_normal:
                        tilted_arc = True
                    else:
                        if has_explicit_normal:
                            nx0, ny0, nz0 = float(ev["nx"]), float(ev["ny"]), float(ev["nz"])
                        else:
                            nx0, ny0, nz0 = {0: (0.0, 0.0, 1.0), 1: (0.0, 1.0, 0.0), 2: (1.0, 0.0, 0.0)}.get(
                                plane_idx, (0.0, 0.0, 1.0)
                            )
                        if has_sec_transform:
                            nx = wp_rx * nx0 + wp_ux * ny0 + wp_fx * nz0
                            ny = wp_ry * nx0 + wp_uy * ny0 + wp_fy * nz0
                            nz = wp_rz * nx0 + wp_uz * ny0 + wp_fz * nz0
                            if wp_det < 0.0:
                                cw = not cw
                        else:
                            nx, ny, nz = nx0, ny0, nz0
                        plane_str, cw, tilted_arc = _classify_arc_normal(nx, ny, nz, cw, plane_str)

                if tilted_arc and ev.get("linearizedMoves"):
                    prev_pt = start_pos
                    for lin_mv in ev.get("linearizedMoves", []):
                        nxt_pt = _to_wcs_pos(
                            float(lin_mv.get("x", 0.0)),
                            float(lin_mv.get("y", 0.0)),
                            float(lin_mv.get("z", 0.0)),
                        )
                        sub_d = math.hypot(nxt_pt.x - prev_pt.x, nxt_pt.y - prev_pt.y, nxt_pt.z - prev_pt.z)
                        if sub_d < 1e-9:
                            continue
                        sub_f, sub_rpm = _calc_move_feed_and_rpm(
                            float(lin_mv.get("feed", ev.get("feed", 0.0))), prev_pt, nxt_pt, raw_ev_rpm
                        )
                        sub_dur = (sub_d / max(1.0, sub_f)) * 60.0 if sub_f > 0 else 0.0
                        _emit_segment(
                            MotionKind.LINEAR,
                            prev_pt,
                            nxt_pt,
                            sub_f,
                            sub_rpm,
                            sub_dur,
                            in_cycle=True,
                            extra_meta={"movement": lin_mv.get("movement", ev.get("movement"))},
                        )
                        prev_pt = nxt_pt
                else:
                    _emit_segment(
                        MotionKind.ARC_CW if cw else MotionKind.ARC_CCW,
                        start_pos,
                        end_pos,
                        feed_mmpm,
                        rpm,
                        duration,
                        in_cycle=in_cycle_active,
                        is_arc=True,
                        arc_center=center_pos,
                        arc_plane=plane_str,
                        extra_meta={"movement": ev.get("movement")},
                    )

            elif ev_type == "onCyclePoint":
                # Use pre-captured expandedMoves to simulate full physical hole/cycle trajectory in IPW
                if any(k in ev for k in ("sx", "sy", "sz")):
                    prev_sub_pos = _to_wcs_pos(
                        float(ev.get("sx", 0.0)),
                        float(ev.get("sy", 0.0)),
                        float(ev.get("sz", 0.0)),
                    )
                elif segments:
                    prev_sub_pos = segments[-1].end
                else:
                    init_p = sec.get("initialPosition", {"x": 0.0, "y": 0.0, "z": 0.0})
                    prev_sub_pos = _to_wcs_pos(float(init_p["x"]), float(init_p["y"]), float(init_p["z"]))

                for sub_mv in ev.get("expandedMoves", []):
                    sub_t = sub_mv.get("type")
                    if sub_t not in ("onRapid", "onLinear"):
                        continue
                    if any(k in sub_mv for k in ("sx", "sy", "sz")):
                        s_pos = _to_wcs_pos(
                            float(sub_mv.get("sx", 0.0)),
                            float(sub_mv.get("sy", 0.0)),
                            float(sub_mv.get("sz", 0.0)),
                        )
                    else:
                        s_pos = prev_sub_pos
                    e_pos = _to_wcs_pos(
                        float(sub_mv.get("x", 0.0)),
                        float(sub_mv.get("y", 0.0)),
                        float(sub_mv.get("z", 0.0)),
                    )
                    prev_sub_pos = e_pos
                    is_rap = (sub_t == "onRapid")
                    if is_rap:
                        f_mmpm = default_rapid_feed_mmpm
                        rpm = float(sub_mv.get("rpm", spindle_rpm) or spindle_rpm)
                    else:
                        raw_ev_rpm = float(sub_mv.get("rpm", 0.0)) if sub_mv.get("rpm") else None
                        f_mmpm, rpm = _calc_move_feed_and_rpm(float(sub_mv.get("feed", 0.0)), s_pos, e_pos, raw_ev_rpm)
                    dist = math.hypot(e_pos.x - s_pos.x, e_pos.y - s_pos.y, e_pos.z - s_pos.z)
                    duration = (dist / max(1.0, f_mmpm)) * 60.0 if f_mmpm > 0 else 0.0
                    _emit_segment(
                        MotionKind.RAPID if is_rap else MotionKind.LINEAR,
                        s_pos,
                        e_pos,
                        f_mmpm,
                        rpm,
                        duration,
                        in_cycle=True,
                    )

            elif ev_type == "onDwell":
                dwell_sec = float(ev.get("sec", 0.0))
                if segments:
                    cur_p = segments[-1].end
                else:
                    init_p = sec.get("initialPosition", {"x": 0.0, "y": 0.0, "z": 0.0})
                    cur_p = _to_wcs_pos(float(init_p["x"]), float(init_p["y"]), float(init_p["z"]))
                _emit_segment(
                    MotionKind.DWELL,
                    cur_p,
                    cur_p,
                    0.0,
                    spindle_rpm,
                    dwell_sec,
                    in_cycle=in_cycle_active,
                    arc_plane="XY",
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
        if sec.get("isMultiAxis") or sec.get("has5DMoves"):
            continue
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


def infer_stl_is_inch(
    raw_verts: Any,
    context_data: Dict[str, Any],
    strict: bool = True,
) -> bool:
    """
    Infer whether unscaled STL mesh vertices (`raw_verts`) are in inches vs millimeters
    from `stockUnit` / `unit` cross-checked against `workpieceHint`.

    When `strict=True`, raises `ValueError` if `workpieceHint` extents diverge by > 5%
    across all candidate scales `(1.0, 25.4, 1.0 / 25.4)`. When `strict=False`, falls back
    to the baseline `stockUnit` / `unit` inference instead of raising.
    """
    import numpy as np

    context_data = _as_context_dict(context_data)
    stock_unit_str = context_data.get("stockUnit")
    if stock_unit_str:
        is_inch = str(stock_unit_str).lower() in ("in", "inch", "inches")
    else:
        is_inch = (context_data.get("unit", 0) == 0)

    if raw_verts is None or len(raw_verts) == 0:
        return is_inch

    wh_mm = extract_workpiece_hint_bbox_mm(context_data)
    if wh_mm is None:
        return is_inch

    stl_min = raw_verts.min(axis=0)
    stl_max = raw_verts.max(axis=0)
    stl_extents = stl_max - stl_min

    dump_scale = 25.4 if int(context_data.get("unit", 0) or 0) == 0 else 1.0
    hint_extents = np.array([
        (wh_mm[1] - wh_mm[0]) / dump_scale,
        (wh_mm[3] - wh_mm[2]) / dump_scale,
        (wh_mm[5] - wh_mm[4]) / dump_scale,
    ], dtype=np.float64)

    candidate_scales = [1.0, 25.4, 1.0 / 25.4]
    active_dims = (hint_extents > 1e-4) & (stl_extents > 1e-4)
    if not np.any(active_dims):
        return is_inch

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
        if strict:
            raise ValueError(
                f"STL bounding box extents {stl_extents.tolist()} do not match workpieceHint "
                f"{hint_extents.tolist()} within 5% for any candidate scale {candidate_scales} "
                f"(best match scale {best_scale} error: {min_err*100:.1f}%)."
            )
        return is_inch

    if abs(best_scale - 25.4) < 1e-3:
        return True
    if abs(best_scale - (1.0 / 25.4)) < 1e-3:
        return False
    if abs(best_scale - 1.0) < 1e-3:
        return (context_data.get("unit", 0) == 0)
    return is_inch


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
    context_data = _as_context_dict(context_data)
    from geometry.models import (
        AxisymmetricChuckObstacle,
        AxisymmetricStock2D,
        BoxStock3D,
        WorkpieceDefinition,
    )

    stock_mode = str(
        context_data.get("stockMode")
        or context_data.get("globalParams", {}).get("job:stockMode")
        or context_data.get("globalParams", {}).get("stockMode")
        or ""
    ).lower()
    dbg_log(f"extract_workpiece_from_context: stock_mode='{stock_mode}', fallback={fallback_workpiece}")

    mode_cat = classify_stock_mode(stock_mode)
    is_previous_mode = (mode_cat == "previous_setup")

    # 1. Native exported STL stock mesh (only for previous/preceding modes)
    stl_path = context_data.get("stockPath")
    if stl_path:
        if is_previous_mode:
            if not os.path.isfile(stl_path):
                raise FileNotFoundError(
                    f"Exported stock STL mesh file not found: '{stl_path}'."
                )
            try:
                import numpy as np  # noqa: F401
            except (ImportError, OSError) as exc:
                raise RuntimeError(
                    "Loading in-process STL stock requires 'numpy', which is not installed "
                    "(offline first-time setup failed). Connect to the internet and re-run Post Clear."
                ) from exc
            try:
                from geometry.solid_mesh import load_stl_mesh, workpiece_from_stl_mesh

                try:
                    raw_verts, raw_indices = load_stl_mesh(stl_path, is_inch=False)
                except Exception as stl_err:
                    raise RuntimeError(
                        f"Failed to load stock from STL mesh '{stl_path}': {stl_err}"
                    ) from stl_err

                is_inch = infer_stl_is_inch(raw_verts, context_data, strict=True)

                turning_mode = is_turning
                if turning_mode is None:
                    turning_mode = is_turning_context(context_data)
                fb_fixtures = getattr(fallback_workpiece, "fixtures", None) if fallback_workpiece else None
                dbg_log(f"extract_workpiece_from_context: loading stock from exported STL '{stl_path}' (turning={turning_mode}, inch={is_inch})")

                vertices_mm = raw_verts * 25.4 if is_inch else raw_verts
                return workpiece_from_stl_mesh(
                    vertices_mm,
                    raw_indices,
                    is_turning=turning_mode,
                    fixtures=fb_fixtures,
                    stl_label=stl_path,
                )
            except (ValueError, FileNotFoundError, RuntimeError):
                raise
            except Exception as stl_err:
                raise RuntimeError(
                    f"Failed to load stock from STL mesh '{stl_path}': {stl_err}"
                ) from stl_err
        else:
            dbg_log(
                f"extract_workpiece_from_context: ignoring stockPath '{stl_path}' "
                f"because stockMode is '{stock_mode}' (not previous/preceding)"
            )

    if mode_cat in ("previous_setup", "solid"):
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
        is_turning = is_turning_context(context_data)

    if not is_turning:
        box_mm = _bbox_dict_to_mm(wp, unit)
        if box_mm is None:
            return fallback_workpiece
        x_min, x_max, y_min, y_max, z_min, z_max = box_mm
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

    stock = AxisymmetricStock2D(
        z_min=z_min,
        z_max=z_max,
        outer_radius=r_outer,
        inner_radius=r_inner,
    )
    fixtures = []
    fb_fixtures = getattr(fallback_workpiece, "fixtures", None) if fallback_workpiece else None
    if fb_fixtures:
        dz = (z_min - float(fb_stock.z_min)) if (fb_stock is not None and hasattr(fb_stock, "z_min")) else 0.0
        for f in fb_fixtures:
            if hasattr(f, "z_min") and hasattr(f, "z_max"):
                fixtures.append(
                    AxisymmetricChuckObstacle.from_stock(
                        stock,
                        z_min=float(f.z_min) + dz,
                        z_max=float(f.z_max) + dz,
                        outer_radius=max(float(getattr(f, "outer_radius", r_outer + 30.0)), r_outer + 10.0),
                    )
                )
            else:
                fixtures.append(f)

    return WorkpieceDefinition(
        stock=stock,
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
    context_data = _as_context_dict(context_data)
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

        is_turning_tool = is_turning_section(sec)
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
            body_l = float(t.get("bodyLength", t.get("body_length", 0.0)) or 0.0) * scale
            shoulder_l = float(t.get("shoulderLength", t.get("shoulder_length", 0.0)) or 0.0) * scale
            shaft_d = float(t.get("shaftDiameter", t.get("shaft_diameter", 0.0)) or 0.0) * scale
            if shaft_d <= 0.0:
                raw_shaft_secs = t.get("_shaftSections")
                if raw_shaft_secs is None:
                    raw_shaft_secs = t.get("shaft_sections")
                if isinstance(raw_shaft_secs, (list, tuple)):
                    shaft_dias = []
                    for item in raw_shaft_secs:
                        if isinstance(item, dict):
                            sd = float(item.get("diameter", 0.0) or 0.0) * scale
                        elif isinstance(item, (list, tuple)) and len(item) >= 2:
                            sd = float(item[1] or 0.0) * scale
                        else:
                            sd = 0.0
                        if sd > 0.0:
                            shaft_dias.append(sd)
                    if shaft_dias:
                        shaft_d = max(shaft_dias)

            raw_taper = t.get("_taperAngle", t.get("taperAngle", t.get("taper_angle", 0.0))) or 0.0
            taper_val = float(raw_taper)
            if 0.0 < taper_val <= math.pi:
                taper_a = math.degrees(taper_val)
            else:
                taper_a = taper_val

            raw_holder_secs = t.get("_holderSections")
            if raw_holder_secs is None:
                raw_holder_secs = t.get("holder_sections")
            holder_sections: Optional[List[Tuple[float, float]]] = None
            if isinstance(raw_holder_secs, (list, tuple)) and len(raw_holder_secs) > 0:
                parsed_secs: List[Tuple[float, float]] = []
                for item in raw_holder_secs:
                    if isinstance(item, dict):
                        s_len = float(item.get("length", 0.0) or 0.0) * scale
                        s_dia = float(item.get("diameter", 0.0) or 0.0) * scale
                    elif isinstance(item, (list, tuple)) and len(item) >= 2:
                        s_len = float(item[0] or 0.0) * scale
                        s_dia = float(item[1] or 0.0) * scale
                    else:
                        continue
                    if s_len > 0.0 or s_dia > 0.0:
                        parsed_secs.append((s_len, s_dia))
                if parsed_secs:
                    holder_sections = parsed_secs

            # 0.0 = no holder defined (never substitute mm defaults scaled by inch factor)
            holder_d = float(t.get("holderDiameter", t.get("holder_diameter", 0.0)) or 0.0) * scale
            holder_l = float(t.get("holderLength", t.get("holder_length", 0.0)) or 0.0) * scale
            if holder_sections:
                if holder_d <= 0.0:
                    holder_d = max((d for _, d in holder_sections), default=0.0)
                if holder_l <= 0.0:
                    holder_l = sum(l for l, _ in holder_sections)

            mill_geom = MillToolGeometry(
                diameter=dia,
                corner_radius=cr,
                flute_length=flute_l,
                overall_length=overall_l,
                taper_angle_deg=taper_a,
                holder_diameter=holder_d,
                holder_length=holder_l,
                body_length=body_l,
                shoulder_length=shoulder_l,
                shaft_diameter=shaft_d,
                holder_sections=holder_sections,
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


# Autodesk CPS C++ Section.getType() & Tool.getType() constants:
#   Section.type: 0 = TYPE_MILLING, 1 = TYPE_TURNING, 2 = TYPE_WIRE, 3 = TYPE_JET, 4 = TYPE_ADDITIVE
#   Tool.type:    27 = TOOL_PROBE, 29 = TOOL_WATER_JET, 30 = TOOL_LASER_CUTTER, 36 = TOOL_PLASMA_CUTTER, 37 = TOOL_MARKER
_NON_MACHINING_TOOL_TYPES = frozenset({29, 30, 36, 37})
_PROBE_TOOL_TYPE = 27


def is_truthy_flag(val: Any) -> bool:
    """Return True when a Fusion/JSON flag represents a truthy boolean, numeric, or string value."""
    if val is True:
        return True
    if val is False or val is None:
        return False
    if isinstance(val, (int, float)) and not isinstance(val, bool):
        return abs(float(val)) > 1e-9
    if isinstance(val, str):
        return val.strip().lower() in ("true", "1", "yes")
    return False


_is_truthy_flag = is_truthy_flag


def check_section_dict_capability(
    sec: Dict[str, Any],
    setup_name: str = "Setup",
    is_turning: bool = False,
    dump_unit: Optional[int] = None,
    fallback_idx: int = 0,
    check_probing: bool = True,
) -> Optional[str]:
    """
    Check whether a dumped section from `universal_dumper.cps` uses capabilities
    unsupported by Fusion for Personal Use and PostClear using CPS C++ enums,
    section geometry, and Fusion's structured boolean taxonomy flags.
    """
    if not isinstance(sec, dict):
        return None
    params = sec.get("params") if isinstance(sec.get("params"), dict) else {}
    tool = sec.get("tool") if isinstance(sec.get("tool"), dict) else {}
    raw_comment = params.get("operation-comment")
    op_name = (
        raw_comment.strip()
        if isinstance(raw_comment, str) and raw_comment.strip()
        else f"Section_{sec.get('id', fallback_idx)}"
    )
    s_type = sec.get("type", 0)
    t_type = tool.get("type")

    if (
        s_type not in (0, 1)
        or t_type in _NON_MACHINING_TOOL_TYPES
        or is_truthy_flag(params.get("operation:isJetStrategy"))
        or is_truthy_flag(params.get("operation:isAdditiveStrategy"))
    ):
        return (
            f"Unsupported operation '{op_name}' in setup '{setup_name}': "
            f"wire, jet/cutting, and additive toolpaths are not supported "
            f"(PostClear supports 2D/3-axis Milling and 2-axis Turning)."
        )

    if check_probing and (
        t_type == _PROBE_TOOL_TYPE
        or is_truthy_flag(params.get("operation:isProbingStrategy"))
        or is_truthy_flag(params.get("operation:isInspectionStrategy"))
    ):
        return (
            f"Unsupported operation '{op_name}' in setup '{setup_name}': "
            f"probing and inspection toolpaths are not supported by PostClear."
        )

    ma_type = params.get("operation:multiAxisMachiningType")
    if (
        sec.get("isMultiAxis")
        or sec.get("has5DMoves")
        or is_truthy_flag(params.get("operation:wrap_cylinder"))
        or (isinstance(ma_type, str) and ma_type.strip().lower() not in ("", "three_axis", "3_axis", "none"))
    ):
        return (
            f"Unsupported multi-axis / rotary-wrap operation '{op_name}' in setup '{setup_name}': "
            f"4-axis rotary wrap and simultaneous multi-axis toolpaths are not supported "
            f"(PostClear supports 2D/3-axis Milling and 2-axis Turning)."
        )

    wp = sec.get("workPlane")
    if s_type != 1 and isinstance(wp, dict) and isinstance(wp.get("forward"), dict):
        fwd = wp["forward"]
        fx, fy, fz = float(fwd.get("x", 0.0)), float(fwd.get("y", 0.0)), float(fwd.get("z", 1.0))
        if not (abs(fx) < 1e-5 and abs(fy) < 1e-5 and abs(fz - 1.0) < 1e-5):
            return (
                f"Unsupported 3+2 tilted Tool Orientation in operation '{op_name}' "
                f"(setup '{setup_name}', tool axis=({fx:.3f}, {fy:.3f}, {fz:.3f})): "
                f"PostClear supports only 2D/3-axis Milling aligned with the setup +Z axis and 2-axis Turning."
            )

    if is_turning and s_type != 1:
        is_mill = is_truthy_flag(params.get("operation:isMillingStrategy")) or is_truthy_flag(
            params.get("operation:isHoleMillingStrategy")
        )
        if is_mill or not is_truthy_flag(params.get("operation:isDrillingStrategy")):
            return (
                f"Unsupported mill-turn live-tool operation '{op_name}' in turning setup '{setup_name}': "
                f"milling operations inside a Turning setup require C/Y-axis live tooling and are not supported."
            )
        tol_xy = 0.001 if dump_unit == 0 else 0.025
        for ev in sec.get("events") or []:
            if not isinstance(ev, dict):
                continue
            ev_type = ev.get("type")
            is_hole_plunge = (
                ev_type == "onLinear"
                and float(ev.get("z", 0.0) or 0.0) < float(ev.get("sz", ev.get("z", 0.0)) or 0.0) - 1e-6
                and (
                    ev.get("movement") in (2, 7)
                    or (
                        abs(float(ev.get("x", 0.0) or 0.0) - float(ev.get("sx", 0.0) or 0.0)) < 1e-5
                        and abs(float(ev.get("y", 0.0) or 0.0) - float(ev.get("sy", 0.0) or 0.0)) < 1e-5
                    )
                )
            )
            if ev_type == "onCyclePoint" or is_hole_plunge:
                vx = float(ev.get("x", 0.0) or 0.0)
                vy = float(ev.get("y", 0.0) or 0.0)
                if math.hypot(vx, vy) > tol_xy:
                    return (
                        f"Unsupported mill-turn off-center drilling operation '{op_name}' "
                        f"in turning setup '{setup_name}': off-center axial holes require C/Y-axis live tooling "
                        f"and are not supported (PostClear supports 2-axis Turning and spindle-centerline Z-drilling)."
                    )

    return None


def check_context_dict_capability(
    dump_data: Dict[str, Any],
    setup_name: str = "Setup",
    is_turning: bool = False,
    check_probing: bool = True,
    start_sec_idx: int = 0,
) -> Optional[str]:
    """Check all sections in a single parsed dump dictionary for unsupported capabilities."""
    if not isinstance(dump_data, dict):
        return None
    dump_unit = dump_data.get("unit")
    unit_int = dump_unit if isinstance(dump_unit, int) and not isinstance(dump_unit, bool) else None
    for offset, sec in enumerate(dump_data.get("sections") or []):
        err = check_section_dict_capability(
            sec,
            setup_name=setup_name,
            is_turning=is_turning,
            dump_unit=unit_int,
            fallback_idx=start_sec_idx + offset,
            check_probing=check_probing,
        )
        if err:
            return err
    return None


def check_dumped_sections_capability(
    batch_json_paths: Sequence[str],
    setup_name: str = "Setup",
    is_turning: bool = False,
    check_probing: bool = True,
) -> Optional[str]:
    """Inspect a sequence of dumped batch JSON files for unsupported capabilities."""
    sec_counter = 0
    for jp in batch_json_paths:
        if not jp or not os.path.isfile(jp):
            continue
        try:
            dump_data = _as_context_dict(jp)
        except Exception:
            continue
        if not isinstance(dump_data, dict):
            continue
        err = check_context_dict_capability(
            dump_data,
            setup_name=setup_name,
            is_turning=is_turning,
            check_probing=check_probing,
            start_sec_idx=sec_counter,
        )
        if err:
            return err
        sec_counter += len(dump_data.get("sections") or [])
    return None
