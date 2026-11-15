"""
3D In-Process Workpiece (IPW) & Collision Verification Engine using Heightfield Grid (Z-Map).

Provides:
- `HeightfieldGrid`: Discrete 2.5D/3D regular grid (numpy array) tracking workpiece top surface.
- `MillHeightfieldVerifier`: Production-grade milling verifier implementing `IVerificationEngine`
  via `BaseVerificationEngine`. Performs:
  • Volumetric material removal rate (MRR) tracking and statistical spike detection.
  • Rapid move into uncut stock collision detection (`RAPID_INTO_STOCK`).
  • Toolholder collision detection against uncut stock (`HOLDER_STRIKE`).
  • Non-fluted shank rubbing detection (`SHANK_RUB`).
  • G1 -> G0 rapid promotion authorization with multi-pass occurrence gating.
  • Parametric cutter profiles (flat, ball, bull-nose/torus, drill/chamfer).
  • Full linear and circular arc (G2/G3) toolpath sweeping.
"""

from __future__ import annotations

import math
import os
import re
import sys
import warnings
from typing import Any, Callable, Dict, List, Optional, Sequence, Set, Tuple, Union

# Ensure packages directory is discoverable for Fusion 360's Python 3.14 environment
if sys.version_info >= (3, 14):
    _addon_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    _packages_dir = os.path.join(_addon_dir, "packages")
    if os.path.isdir(_packages_dir) and _packages_dir not in sys.path:
        sys.path.insert(0, _packages_dir)

try:
    import numpy as np
except Exception:
    np = None

from .engine_interface import BaseVerificationEngine
from .models import (
    BoxStock3D,
    CollisionKind,
    CollisionRecord,
    MillToolGeometry,
    MotionKind,
    MotionSegment,
    MoveEvaluation,
    SolidMeshStock3D,
    ToolAssembly,
    Vector3D,
    VerificationReport,
    WorkpieceDefinition,
)


class HeightfieldGrid:
    """
    Discrete 2.5D/3D regular grid representing the top surface Z(X, Y) of the workpiece.
    """

    def __init__(
        self,
        cell_size_mm: float = 0.5,
        pad_mm: float = 10.0,
    ) -> None:
        if np is None:
            raise ImportError("numpy is required for HeightfieldGrid but could not be imported")
        self.cell_size_mm = max(0.05, float(cell_size_mm))
        self.pad_mm = max(0.0, float(pad_mm))
        self.cell_area_mm2 = self.cell_size_mm * self.cell_size_mm

        # World bounds
        self.stock_x_min = 0.0
        self.stock_x_max = 0.0
        self.stock_y_min = 0.0
        self.stock_y_max = 0.0
        self.stock_z_min = 0.0
        self.stock_z_max = 0.0

        # Padded grid bounds
        self.grid_x_min = 0.0
        self.grid_x_max = 0.0
        self.grid_y_min = 0.0
        self.grid_y_max = 0.0
        self.z_floor = 0.0

        self.nx = 0
        self.ny = 0
        self.H = np.zeros((0, 0), dtype=np.float32)

    def reset(self, stock: BoxStock3D) -> None:
        """Initialize or reset heightfield grid from a 3D bounding box."""
        self.stock_x_min = float(stock.x_min)
        self.stock_x_max = float(stock.x_max)
        self.stock_y_min = float(stock.y_min)
        self.stock_y_max = float(stock.y_max)
        self.stock_z_min = float(stock.z_min)
        self.stock_z_max = float(stock.z_max)

        span_x = (self.stock_x_max - self.stock_x_min) + 2.0 * self.pad_mm
        span_y = (self.stock_y_max - self.stock_y_min) + 2.0 * self.pad_mm
        self.nx = max(2, int(math.ceil(span_x / self.cell_size_mm)) + 1)
        self.ny = max(2, int(math.ceil(span_y / self.cell_size_mm)) + 1)
        cx = 0.5 * (self.stock_x_min + self.stock_x_max)
        cy = 0.5 * (self.stock_y_min + self.stock_y_max)
        self.grid_x_min = cx - 0.5 * self.nx * self.cell_size_mm
        self.grid_x_max = cx + 0.5 * self.nx * self.cell_size_mm
        self.grid_y_min = cy - 0.5 * self.ny * self.cell_size_mm
        self.grid_y_max = cy + 0.5 * self.ny * self.cell_size_mm
        self.z_floor = self.stock_z_min

        # Allocate grid
        # Outside stock bounds: z_floor. Inside stock bounds: stock_z_max.
        # Cell (r, c) represents cell center (grid_x_min + (c + 0.5)*cs, grid_y_min + (r + 0.5)*cs).
        self.H = np.full((self.ny, self.nx), self.z_floor, dtype=np.float32)

        col_min = max(0, int(math.ceil((self.stock_x_min - self.grid_x_min) / self.cell_size_mm - 0.5 - 1e-9)))
        col_max = min(self.nx, int(math.floor((self.stock_x_max - self.grid_x_min) / self.cell_size_mm - 0.5 + 1e-9)) + 1)
        row_min = max(0, int(math.ceil((self.stock_y_min - self.grid_y_min) / self.cell_size_mm - 0.5 - 1e-9)))
        row_max = min(self.ny, int(math.floor((self.stock_y_max - self.grid_y_min) / self.cell_size_mm - 0.5 + 1e-9)) + 1)

        if col_max <= col_min and self.nx > 0:
            c_mid = max(0, min(self.nx - 1, int(math.floor((0.5 * (self.stock_x_min + self.stock_x_max) - self.grid_x_min) / self.cell_size_mm))))
            col_min, col_max = c_mid, c_mid + 1
        if row_max <= row_min and self.ny > 0:
            r_mid = max(0, min(self.ny - 1, int(math.floor((0.5 * (self.stock_y_min + self.stock_y_max) - self.grid_y_min) / self.cell_size_mm))))
            row_min, row_max = r_mid, r_mid + 1

        if col_max > col_min and row_max > row_min:
            self.H[row_min:row_max, col_min:col_max] = np.float32(self.stock_z_max)

    def reset_from_mesh(
        self,
        vertices: np.ndarray,
        indices: np.ndarray,
        tol_pad_z_mm: float = 0.05,
    ) -> None:
        """Initialize heightfield grid by rasterizing a 3D triangular mesh in setup WCS (mm)."""
        if vertices is None or indices is None or len(vertices) == 0 or len(indices) == 0:
            raise ValueError("HeightfieldGrid requires valid non-empty vertices and indices")

        v = np.asarray(vertices, dtype=np.float32)
        idx = np.asarray(indices, dtype=np.int32)
        min_pt = v.min(axis=0)
        max_pt = v.max(axis=0)

        self.stock_x_min = float(min_pt[0])
        self.stock_x_max = float(max_pt[0])
        self.stock_y_min = float(min_pt[1])
        self.stock_y_max = float(max_pt[1])
        self.stock_z_min = float(min_pt[2])
        self.stock_z_max = float(max_pt[2]) + tol_pad_z_mm

        span_x = (self.stock_x_max - self.stock_x_min) + 2.0 * self.pad_mm
        span_y = (self.stock_y_max - self.stock_y_min) + 2.0 * self.pad_mm
        self.nx = max(2, int(math.ceil(span_x / self.cell_size_mm)) + 1)
        self.ny = max(2, int(math.ceil(span_y / self.cell_size_mm)) + 1)
        cx = 0.5 * (self.stock_x_min + self.stock_x_max)
        cy = 0.5 * (self.stock_y_min + self.stock_y_max)
        self.grid_x_min = cx - 0.5 * self.nx * self.cell_size_mm
        self.grid_x_max = cx + 0.5 * self.nx * self.cell_size_mm
        self.grid_y_min = cy - 0.5 * self.ny * self.cell_size_mm
        self.grid_y_max = cy + 0.5 * self.ny * self.cell_size_mm
        self.z_floor = self.stock_z_min

        # Allocate grid to floor
        self.H = np.full((self.ny, self.nx), self.z_floor, dtype=np.float32)

        from geometry.solid_mesh import rasterize_triangles_to_grid
        rasterize_triangles_to_grid(self, v, idx, tol_pad_z_mm=tol_pad_z_mm)

    def reset_from_solid(self, solid_stock: SolidMeshStock3D, tol_pad_z_mm: float = 0.05) -> None:
        """Initialize heightfield grid from a SolidMeshStock3D."""
        self.reset_from_mesh(solid_stock.vertices, solid_stock.indices, tol_pad_z_mm=tol_pad_z_mm)

    def get_subgrid_bounds(
        self,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
        radius_mm: float,
    ) -> Tuple[int, int, int, int]:
        """Compute grid index bounds (col_min, col_max, row_min, row_max) enclosing swept tool."""
        min_x = min(x1, x2) - radius_mm
        max_x = max(x1, x2) + radius_mm
        min_y = min(y1, y2) - radius_mm
        max_y = max(y1, y2) + radius_mm

        col_min = max(0, int(math.floor((min_x - self.grid_x_min) / self.cell_size_mm - 0.5)))
        col_max = min(self.nx, int(math.ceil((max_x - self.grid_x_min) / self.cell_size_mm - 0.5)) + 1)
        row_min = max(0, int(math.floor((min_y - self.grid_y_min) / self.cell_size_mm - 0.5)))
        row_max = min(self.ny, int(math.ceil((max_y - self.grid_y_min) / self.cell_size_mm - 0.5)) + 1)

        return col_min, col_max, row_min, row_max

    def get_mesh_coordinates(
        self,
        col_min: int,
        col_max: int,
        row_min: int,
        row_max: int,
    ) -> Tuple[Any, Any]:
        """Construct 2D spatial coordinate grids (X, Y) at cell centers for a subgrid window."""
        xs = self.grid_x_min + (np.arange(col_min, col_max, dtype=np.float64) + 0.5) * self.cell_size_mm
        ys = self.grid_y_min + (np.arange(row_min, row_max, dtype=np.float64) + 0.5) * self.cell_size_mm
        return np.meshgrid(xs, ys)


def evaluate_tool_bottom_profile(
    radius_arr: Any,
    tool_radius: float,
    corner_radius: float,
    taper_angle_deg: float = 0.0,
) -> Any:
    """
    Vectorized computation of tool bottom relative height Z_rel(r) >= 0.
    At tool tip center (r=0), Z_rel = 0.
    """
    if np is None:
        raise ImportError("numpy is required for evaluate_tool_bottom_profile but could not be imported")
    r = np.maximum(0.0, radius_arr)
    R = max(1e-4, float(tool_radius))
    rc = max(0.0, min(float(corner_radius), R))

    if rc <= 1e-4:
        # Flat endmill or conical / chamfer / drill tool
        if taper_angle_deg > 0.01:
            # Conical tip (drill, chamfer mill, spot drill, countersink).
            # If included angle is given (> 45 deg, e.g. 90 deg for chamfer, 118 deg for drill):
            # half angle with vertical Z axis is alpha = taper_angle_deg / 2.
            # If half angle is given (<= 45 deg, e.g. 45 deg for 90 deg chamfer):
            # half angle is taper_angle_deg.
            if taper_angle_deg > 45.0:
                half_angle_deg = taper_angle_deg / 2.0
            else:
                half_angle_deg = taper_angle_deg
            half_angle_deg = min(89.5, max(0.5, half_angle_deg))
            alpha_rad = math.radians(half_angle_deg)
            # Conical geometry: r = z * tan(alpha) => z_rel = r / tan(alpha)
            return (r / math.tan(alpha_rad)).astype(np.float32)
        return np.zeros_like(r, dtype=np.float32)

    if abs(rc - R) < 1e-4:
        # Ball endmill: hemispherical bottom
        diff = np.maximum(0.0, R * R - r * r)
        return (R - np.sqrt(diff)).astype(np.float32)

    # Bull-nose / Torus endmill
    r_flat = R - rc
    z_rel = np.zeros_like(r, dtype=np.float32)
    corner_mask = r > r_flat
    if np.any(corner_mask):
        dr = r[corner_mask] - r_flat
        diff = np.maximum(0.0, rc * rc - dr * dr)
        z_rel[corner_mask] = rc - np.sqrt(diff)

    return z_rel


# Autodesk CPS movement enum subsets:
#   0=RAPID, 1=LEAD_IN, 2=CUTTING, 3=LEAD_OUT, 4=LINK_TRANSITION, 5=LINK_DIRECT,
#   6..9=RAMP*, 10=EXTENDED/PLUNGE, 11..14=CUTTING*, 15=HIGH_FEED
_LINK_OR_RAPID_MOVEMENTS = frozenset((0, 4, 5, 15))
_CORE_CUT_MOVEMENTS = frozenset((2, 6, 7, 8, 9, 11, 12, 13, 14))
_LEAD_OR_LINK_MOVEMENTS = frozenset((0, 1, 3, 4, 5, 10, 15))


def _z_overlap_ratio(
    tp_min_z: float,
    tp_max_z: float,
    st_min_z: float,
    st_max_z: float,
    z_skin_tol: float = 0.05,
) -> float:
    """Compute asymmetric Z overlap ratio in [0, 1], penalizing cuts above stock top."""
    tp_dz = max(0.0, tp_max_z - tp_min_z)
    st_dz = max(0.0, st_max_z - st_min_z)
    above_excess = max(0.0, tp_max_z - (st_max_z + z_skin_tol))
    denom_z = (min(tp_dz, st_dz) + above_excess) if st_dz > 1e-6 else tp_dz
    ov_z = max(0.0, min(tp_max_z, st_max_z + z_skin_tol) - max(tp_min_z, st_min_z - z_skin_tol))
    if denom_z > z_skin_tol:
        return min(1.0, ov_z / denom_z)
    return (
        1.0
        if (st_min_z - z_skin_tol <= tp_min_z <= st_max_z + z_skin_tol)
        and (st_min_z - z_skin_tol <= tp_max_z <= st_max_z + z_skin_tol)
        else 0.0
    )


def _compute_section_cutting_bounds(
    segments: Sequence[MotionSegment],
    st_max_z: float,
    z_skin_tol: float = 0.05,
) -> Tuple[
    Dict[int, List[MotionSegment]],
    Dict[int, str],
    Set[int],
    Dict[int, Tuple[float, float, float, float, float, float]],
]:
    """Compute per-section cutting 3D bounding boxes after trimming rapid/link/retract/lead moves."""
    by_sec: Dict[int, List[MotionSegment]] = {}
    sec_op_id: Dict[int, str] = {}
    sec_tilted_or_5x: Set[int] = set()
    for s in segments:
        if s.motion == MotionKind.RAPID:
            continue
        meta = getattr(s, "metadata", None) or {}
        sec_i = int(meta.get("sec_idx", s.sec_id))
        by_sec.setdefault(sec_i, []).append(s)
        if meta.get("tilted_section") or meta.get("multi_axis"):
            sec_tilted_or_5x.add(sec_i)
        if s.op_id and sec_i not in sec_op_id:
            sec_op_id[sec_i] = s.op_id

    sec_bounds: Dict[int, Tuple[float, float, float, float, float, float]] = {}
    for si, s_list in by_sec.items():
        no_link = [
            s
            for s in s_list
            if (getattr(s, "metadata", None) or {}).get("movement") not in _LINK_OR_RAPID_MOVEMENTS
        ]
        s_work = no_link if no_link else s_list

        if si in sec_tilted_or_5x:
            xs = [s.start.x for s in s_work] + [s.end.x for s in s_work]
            ys = [s.start.y for s in s_work] + [s.end.y for s in s_work]
            zs = [s.start.z for s in s_work] + [s.end.z for s in s_work]
            sec_bounds[si] = (min(xs), max(xs), min(ys), max(ys), min(zs), max(zs))
            continue

        horiz = [
            s
            for s in s_work
            if math.hypot(s.end.x - s.start.x, s.end.y - s.start.y) > 1e-6
        ]
        vert = [
            s
            for s in s_work
            if math.hypot(s.end.x - s.start.x, s.end.y - s.start.y) <= 1e-6
        ]

        if horiz:
            ceil_cands: List[float] = []
            for s in s_work:
                meta = getattr(s, "metadata", None) or {}
                for k in ("z_retract_mm", "z_clearance_mm"):
                    val = meta.get(k)
                    if val is not None:
                        ceil_cands.append(float(val))
            min_hz = min(min(s.start.z, s.end.z) for s in horiz)
            for vseg in vert:
                vz_hi = max(vseg.start.z, vseg.end.z)
                vz_lo = min(vseg.start.z, vseg.end.z)
                if vz_hi > st_max_z + 0.5 and vz_hi > vz_lo + 0.5 and vz_lo <= st_max_z + 1.0:
                    ceil_cands.append(vz_hi)

            if ceil_cands:
                rc = min(ceil_cands)
                if rc > min_hz + 1e-4:
                    below = [
                        s
                        for s in horiz
                        if not (s.start.z >= rc - 1e-4 and s.end.z >= rc - 1e-4)
                    ]
                    if below:
                        horiz = below

            core_cuts = [
                s
                for s in horiz
                if (getattr(s, "metadata", None) or {}).get("movement") in _CORE_CUT_MOVEMENTS
                and min(s.start.z, s.end.z) <= st_max_z + z_skin_tol
            ]
            if core_cuts:
                trimmed = [
                    s
                    for s in horiz
                    if not (
                        (
                            (getattr(s, "metadata", None) or {}).get("movement")
                            in _LEAD_OR_LINK_MOVEMENTS
                            and max(s.start.z, s.end.z) > st_max_z + z_skin_tol
                        )
                        or (
                            (getattr(s, "metadata", None) or {}).get("movement") in (None, 2)
                            and min(s.start.z, s.end.z) > st_max_z + z_skin_tol
                        )
                    )
                ]
                if trimmed:
                    horiz = trimmed

            xs = [s.start.x for s in horiz] + [s.end.x for s in horiz]
            ys = [s.start.y for s in horiz] + [s.end.y for s in horiz]
            zs = [s.start.z for s in horiz] + [s.end.z for s in horiz]
            sec_bounds[si] = (min(xs), max(xs), min(ys), max(ys), min(zs), max(zs))
        else:
            plunges = [s for s in vert if s.end.z < s.start.z - 1e-6]
            v_use = plunges if plunges else vert
            xs = [s.start.x for s in v_use] + [s.end.x for s in v_use]
            ys = [s.start.y for s in v_use] + [s.end.y for s in v_use]
            zs = []
            for s in v_use:
                z_start = (
                    min(s.start.z, st_max_z)
                    if (s.end.z <= st_max_z + z_skin_tol and s.start.z > st_max_z)
                    else s.start.z
                )
                zs.append(z_start)
                zs.append(s.end.z)
            sec_bounds[si] = (min(xs), max(xs), min(ys), max(ys), min(zs), max(zs))

    return by_sec, sec_op_id, sec_tilted_or_5x, sec_bounds


def _find_unmodeled_pattern_secs(
    sec_bounds: Dict[int, Tuple[float, float, float, float, float, float]],
    sec_op_id: Dict[int, str],
    st_min_x: float,
    st_max_x: float,
    st_min_y: float,
    st_max_y: float,
    st_min_z: float,
    st_max_z: float,
    xy_margin: float,
) -> Set[int]:
    """Identify section indices outside expanded stock bounds that share an explicit op_id with an in-stock section."""
    if len(sec_bounds) < 2:
        return set()
    z_margin = max(xy_margin, 2.0)
    eff_min_x, eff_max_x = st_min_x - xy_margin, st_max_x + xy_margin
    eff_min_y, eff_max_y = st_min_y - xy_margin, st_max_y + xy_margin
    eff_min_z, eff_max_z = st_min_z - z_margin, st_max_z + z_margin

    op_ids_overlapping: Set[str] = set()
    sec_overlaps: Dict[int, bool] = {}
    for si, (x0, x1, y0, y1, z0, z1) in sec_bounds.items():
        ov = (
            (x1 >= eff_min_x and x0 <= eff_max_x)
            and (y1 >= eff_min_y and y0 <= eff_max_y)
            and (z1 >= eff_min_z and z0 <= eff_max_z)
        )
        sec_overlaps[si] = ov
        op = sec_op_id.get(si, "")
        if ov and op and not re.fullmatch(r"Section_\d+", op):
            op_ids_overlapping.add(op)

    return {
        si
        for si in sec_bounds
        if not sec_overlaps.get(si, False) and sec_op_id.get(si, "") in op_ids_overlapping
    }


def detect_unmodeled_pattern_sections(
    segments: Sequence[MotionSegment],
    workpiece: Optional[WorkpieceDefinition],
    tools_by_op: Optional[Dict[str, ToolAssembly]] = None,
) -> Set[int]:
    """
    Identify CAM pattern / tombstone duplicate sections whose cutting geometry lies
    completely outside the modeled stock 3D envelope while a sibling section with
    the exact same explicit `op_id` overlaps the modeled stock.
    """
    if workpiece is None or not segments:
        return set()
    st = getattr(workpiece, "stock", None)
    if not (hasattr(st, "x_min") and hasattr(st, "x_max")):
        return set()

    st_min_x, st_max_x = float(st.x_min), float(st.x_max)
    st_min_y, st_max_y = float(st.y_min), float(st.y_max)
    st_min_z, st_max_z = float(st.z_min), float(st.z_max)
    if st_max_x < st_min_x or st_max_y < st_min_y or st_max_z < st_min_z:
        return set()

    max_tool_r = max(
        (
            float(getattr(getattr(t, "geometry", None), "diameter", 0.0) or 0.0) * 0.5
            for t in (tools_by_op or {}).values()
        ),
        default=0.0,
    )
    _, sec_op_id, _, sec_bounds = _compute_section_cutting_bounds(segments, st_max_z)
    return _find_unmodeled_pattern_secs(
        sec_bounds,
        sec_op_id,
        st_min_x,
        st_max_x,
        st_min_y,
        st_max_y,
        st_min_z,
        st_max_z,
        xy_margin=2.0 * max_tool_r,
    )


def check_stl_stock_alignment(
    segments: Sequence[MotionSegment],
    workpiece: WorkpieceDefinition,
    tools_by_op: Optional[Dict[str, ToolAssembly]] = None,
    is_turning: bool = False,
    workpiece_hint_mm: Optional[Tuple[float, float, float, float, float, float]] = None,
    dbg_fn: Optional[Callable[[str], None]] = None,
) -> Tuple[Optional[str], Optional[str], Set[int]]:
    """
    Phase 1 H4: Runtime coordinate frame alignment check for STL-derived stock.

    Verifies >= 50% cutting-envelope overlap against the STL stock bounding box
    and returns `(toolpath_box_log, stock_box_log, unmodeled_pattern_secs)`.
    """
    non_rapid = [s for s in segments if s.motion != MotionKind.RAPID]
    if not non_rapid:
        return None, None, set()

    st = getattr(workpiece, "stock", None)
    if hasattr(st, "x_min") and hasattr(st, "x_max"):
        st_min_x, st_max_x = float(st.x_min), float(st.x_max)
        st_min_y, st_max_y = float(st.y_min), float(st.y_max)
        st_min_z, st_max_z = float(st.z_min), float(st.z_max)
    elif hasattr(st, "outer_radius") and hasattr(st, "z_min"):
        st_r = float(st.outer_radius)
        st_min_x, st_max_x = -st_r, st_r
        st_min_y, st_max_y = -st_r, st_r
        st_min_z, st_max_z = float(st.z_min), float(st.z_max)
    else:
        st_min_x = st_max_x = st_min_y = st_max_y = st_min_z = st_max_z = 0.0

    st_box_log = (
        f"[{st_min_x:.2f}, {st_max_x:.2f}] x "
        f"[{st_min_y:.2f}, {st_max_y:.2f}] x "
        f"[{st_min_z:.2f}, {st_max_z:.2f}]"
    )
    z_skin_tol = 0.05

    if is_turning:
        lathe_segs = [
            s for s in non_rapid if (getattr(s, "metadata", None) or {}).get("sec_type", 1) == 1
        ]
        cand = lathe_segs if lathe_segs else non_rapid
        cand_no_link = [
            s
            for s in cand
            if (getattr(s, "metadata", None) or {}).get("movement") not in _LINK_OR_RAPID_MOVEMENTS
        ]
        if cand_no_link:
            cand = cand_no_link

        outer_r = float(getattr(st, "outer_radius", max(abs(st_max_x), abs(st_max_y))) or 0.0)
        if outer_r > 0.0:
            near_stock = [
                s
                for s in cand
                if (getattr(s, "metadata", None) or {}).get("movement") is not None
                or min(math.hypot(s.start.x, s.start.y), math.hypot(s.end.x, s.end.y))
                <= outer_r + 2.0
            ]
            if near_stock:
                cand = near_stock

        tp_xs = [s.start.x for s in cand] + [s.end.x for s in cand]
        tp_ys = [s.start.y for s in cand] + [s.end.y for s in cand]
        tp_zs = [s.start.z for s in cand] + [s.end.z for s in cand]
        tp_min_x, tp_max_x = min(tp_xs), max(tp_xs)
        tp_min_y, tp_max_y = min(tp_ys), max(tp_ys)
        tp_min_z, tp_max_z = min(tp_zs), max(tp_zs)
        tp_box_log = (
            f"[{tp_min_x:.2f}, {tp_max_x:.2f}] x "
            f"[{tp_min_y:.2f}, {tp_max_y:.2f}] x "
            f"[{tp_min_z:.2f}, {tp_max_z:.2f}]"
        )

        z_ratio = _z_overlap_ratio(tp_min_z, tp_max_z, st_min_z, st_max_z, z_skin_tol)
        if dbg_fn is not None:
            dbg_fn(
                f"H4 turning Z overlap: toolpath={tp_box_log}, stock={st_box_log}, ratio={z_ratio*100:.1f}%"
            )
        if z_ratio < 0.5:
            raise ValueError(
                f"Cutting toolpath Z span [{tp_min_z:.2f}, {tp_max_z:.2f}] has insufficient overlap ({z_ratio*100:.1f}% < 50%) "
                f"with stock Z span [{st_min_z:.2f}, {st_max_z:.2f}]. "
                f"Toolpath bounding box: {tp_box_log}; Stock bounding box: {st_box_log}."
            )
        return tp_box_log, st_box_log, set()

    max_tool_r = max(
        (
            float(getattr(getattr(t, "geometry", None), "diameter", 0.0) or 0.0) * 0.5
            for t in (tools_by_op or {}).values()
        ),
        default=0.0,
    )
    xy_margin = 2.0 * max_tool_r
    eff_st_min_x, eff_st_max_x = st_min_x - xy_margin, st_max_x + xy_margin
    eff_st_min_y, eff_st_max_y = st_min_y - xy_margin, st_max_y + xy_margin

    _, sec_op_id, sec_tilted_or_5x, sec_bounds = _compute_section_cutting_bounds(
        non_rapid, st_max_z, z_skin_tol
    )
    unmodeled_pattern_secs = _find_unmodeled_pattern_secs(
        sec_bounds,
        sec_op_id,
        st_min_x,
        st_max_x,
        st_min_y,
        st_max_y,
        st_min_z,
        st_max_z,
        xy_margin=xy_margin,
    )

    z_aligned_secs = [si for si in sorted(sec_bounds) if si not in sec_tilted_or_5x]
    eval_secs = z_aligned_secs if z_aligned_secs else sorted(sec_bounds)
    active_secs = [si for si in eval_secs if si not in unmodeled_pattern_secs] or eval_secs

    tp_min_x = min(sec_bounds[si][0] for si in active_secs)
    tp_max_x = max(sec_bounds[si][1] for si in active_secs)
    tp_min_y = min(sec_bounds[si][2] for si in active_secs)
    tp_max_y = max(sec_bounds[si][3] for si in active_secs)
    tp_min_z = min(sec_bounds[si][4] for si in active_secs)
    tp_max_z = max(sec_bounds[si][5] for si in active_secs)
    tp_box_log = (
        f"[{tp_min_x:.2f}, {tp_max_x:.2f}] x "
        f"[{tp_min_y:.2f}, {tp_max_y:.2f}] x "
        f"[{tp_min_z:.2f}, {tp_max_z:.2f}]"
    )

    tp_dx = max(0.0, tp_max_x - tp_min_x)
    tp_dy = max(0.0, tp_max_y - tp_min_y)
    ov_dx = max(0.0, min(tp_max_x, eff_st_max_x) - max(tp_min_x, eff_st_min_x))
    ov_dy = max(0.0, min(tp_max_y, eff_st_max_y) - max(tp_min_y, eff_st_min_y))
    r_x = (
        min(1.0, ov_dx / tp_dx)
        if tp_dx > 1e-6
        else (1.0 if eff_st_min_x <= tp_min_x <= eff_st_max_x else 0.0)
    )
    r_y = (
        min(1.0, ov_dy / tp_dy)
        if tp_dy > 1e-6
        else (1.0 if eff_st_min_y <= tp_min_y <= eff_st_max_y else 0.0)
    )
    r_z = _z_overlap_ratio(tp_min_z, tp_max_z, st_min_z, st_max_z, z_skin_tol)

    if (
        r_z < 0.5
        and workpiece_hint_mm is not None
        and r_x >= 0.8
        and r_y >= 0.8
        and tp_max_z <= st_max_z + z_skin_tol
        and tp_min_z < st_min_z
        and tp_max_z >= st_min_z + z_skin_tol
    ):
        wh_x0, wh_x1, wh_y0, wh_y1, wh_z0, wh_z1 = workpiece_hint_mm
        tol_wh = 1.0
        if (
            abs(st_min_x - wh_x0) <= tol_wh
            and abs(st_max_x - wh_x1) <= tol_wh
            and abs(st_min_y - wh_y0) <= tol_wh
            and abs(st_max_y - wh_y1) <= tol_wh
            and abs(st_max_z - wh_z1) <= tol_wh
            and st_min_z >= wh_z0 - tol_wh
            and max(0.0, tp_max_z - max(tp_min_z, st_min_z)) > z_skin_tol
        ):
            r_z = 1.0

    vol_ratio = r_x * r_y * r_z
    if dbg_fn is not None:
        dbg_fn(
            f"H4 milling volume overlap: toolpath={tp_box_log}, stock={st_box_log}, ratio={vol_ratio*100:.1f}%"
            + (f", unmodeled_pattern_secs={sorted(unmodeled_pattern_secs)}" if unmodeled_pattern_secs else "")
        )
    if vol_ratio < 0.5:
        raise ValueError(
            f"Toolpath bounding box {tp_box_log} has insufficient volume overlap ({vol_ratio*100:.1f}% < 50%) "
            f"with Stock bounding box {st_box_log}."
        )
    return tp_box_log, st_box_log, unmodeled_pattern_secs


class MillHeightfieldVerifier(BaseVerificationEngine):
    """
    3D Milling In-Process Workpiece (IPW) & Collision Verification Engine using
    a regular Heightfield Grid (Z-Map).
    Note: `promoted_events` is not length-filtered by the engine; minimum travel length
    filtering is applied by the caller (e.g. `filter_promoted_events_by_length` / `PipelineOrchestrator.run`).
    """

    def __init__(
        self,
        cell_size_mm: float = 0.5,
        pad_mm: float = 10.0,
        promote_margin_mm: float = 0.5,
        holder_margin_mm: float = 1.0,
        min_promote_len_mm: Optional[float] = None,
        z_tol_mm: float = 1e-3,
        xy_tol_mm: float = 1e-3,
        collision_lateral_tol_mm: Optional[float] = None,
        min_collision_volume_mm3: float = 0.5,
        min_mrr_duration_sec: float = 0.05,
    ) -> None:
        super().__init__(min_mrr_duration_sec=min_mrr_duration_sec)
        if np is None:
            raise ImportError("numpy is required for MillHeightfieldVerifier but could not be imported")
        if min_promote_len_mm is not None:
            warnings.warn(
                "min_promote_len_mm on MillHeightfieldVerifier is deprecated and ignored; "
                "length filtering is applied by the caller via filter_promoted_events_by_length.",
                DeprecationWarning,
                stacklevel=2,
            )
        self.grid = HeightfieldGrid(cell_size_mm=cell_size_mm, pad_mm=pad_mm)
        self.promote_margin_mm = float(promote_margin_mm)
        self.holder_margin_mm = float(holder_margin_mm)
        self.min_promote_len_mm = min_promote_len_mm
        self.z_tol_mm = float(z_tol_mm)
        self.xy_tol_mm = float(xy_tol_mm)
        cell_diag_half = self.grid.cell_size_mm * math.sqrt(2.0) / 2.0
        self.collision_lateral_tol_mm = (
            cell_diag_half if collision_lateral_tol_mm is None else max(0.0, float(collision_lateral_tol_mm))
        )
        self.min_collision_volume_mm3 = max(0.0, float(min_collision_volume_mm3))

        # Fallback tool dimensions if ToolAssembly geometry is missing
        self.default_cutter_diameter = 10.0
        self.default_flute_length = 25.0
        self.default_overall_length = 50.0
        self.default_holder_diameter = 40.0
        self.default_holder_length = 50.0
        self._cached_tool_key: Optional[Tuple[Any, ...]] = None
        self._cached_tool_params: Optional[
            Tuple[float, float, float, float, float, bool, List[Tuple[float, float]], float, float]
        ] = None

        self._sec_clearance_mm: Dict[int, float] = {}
        self._unmodeled_pattern_sec_indices: Set[int] = set()

    def reset(self, workpiece: WorkpieceDefinition) -> None:
        """Reset verifier and IPW grid to raw stock."""
        self._reset_base_state(workpiece)
        self._sec_clearance_mm.clear()
        self._unmodeled_pattern_sec_indices.clear()

        self.has_valid_stock = False
        stock = getattr(workpiece, "stock", None)
        if isinstance(stock, BoxStock3D) and stock.x_max > stock.x_min and stock.y_max > stock.y_min and stock.z_max > stock.z_min:
            self.grid.reset(stock)
            self.has_valid_stock = True
        elif isinstance(stock, SolidMeshStock3D):
            self.grid.reset_from_solid(stock)
            self.has_valid_stock = True
        elif hasattr(stock, "vertices") and hasattr(stock, "indices"):
            self.grid.reset_from_mesh(stock.vertices, stock.indices)
            self.has_valid_stock = True
        elif hasattr(stock, "x_min") and hasattr(stock, "z_max"):
            x_min, x_max = float(stock.x_min), float(stock.x_max)
            y_min, y_max = float(stock.y_min), float(stock.y_max)
            z_min, z_max = float(stock.z_min), float(stock.z_max)
            if x_max > x_min and y_max > y_min and z_max > z_min:
                box_stock = BoxStock3D(
                    x_min=x_min,
                    x_max=x_max,
                    y_min=y_min,
                    y_max=y_max,
                    z_min=z_min,
                    z_max=z_max,
                )
                self.grid.reset(box_stock)
                self.has_valid_stock = True
        else:
            raise ValueError(f"MillHeightfieldVerifier requires valid BoxStock3D or SolidMeshStock3D, got {type(stock)}")

    def reset_workpiece(self, workpiece: WorkpieceDefinition) -> None:
        """Backwards compatibility alias for reset()."""
        self.reset(workpiece)

    def load_tool(self, tool: ToolAssembly) -> None:
        """Mount active tool assembly."""
        self.active_tool = tool

    def _promotion_blocked(self, move: MotionSegment) -> bool:
        """Block rapid promotion on tilted/multi-axis, cutter-comp, or unmodeled pattern duplicate sections."""
        if super()._promotion_blocked(move):
            return True
        if self._unmodeled_pattern_sec_indices:
            meta = getattr(move, "metadata", None) or {}
            sec_idx = int(meta.get("sec_idx", move.sec_id))
            if sec_idx in self._unmodeled_pattern_sec_indices:
                return True
        return False

    def verify_toolpath(
        self,
        moves: Optional[Sequence[MotionSegment]] = None,
        tools_by_op: Optional[Dict[str, ToolAssembly]] = None,
        workpiece: Optional[WorkpieceDefinition] = None,
        segments: Optional[Sequence[MotionSegment]] = None,
        unmodeled_pattern_sec_indices: Optional[Set[int]] = None,
    ) -> VerificationReport:
        """Evaluate complete milling toolpath while blocking promotions on unmodeled CAM pattern copies."""
        moves_seq = moves if moves is not None else (segments or [])
        if workpiece is not None:
            self.reset(workpiece)
        if unmodeled_pattern_sec_indices is not None:
            self._unmodeled_pattern_sec_indices = set(unmodeled_pattern_sec_indices)
        elif self.workpiece is not None and moves_seq:
            self._unmodeled_pattern_sec_indices = detect_unmodeled_pattern_sections(
                moves_seq, self.workpiece, tools_by_op
            )
        return super().verify_toolpath(
            moves=moves_seq,
            tools_by_op=tools_by_op,
            workpiece=None,
        )

    def _get_active_mill_tool_params(
        self,
    ) -> Tuple[float, float, float, float, float, bool, List[Tuple[float, float]], float, float]:
        """
        Extract active milling tool and holder geometry:
        (tool_r, corner_r, flute_len, body_len, shank_r, has_holder, holder_sections, holder_max_r, taper_deg).
        """
        geom = getattr(self.active_tool, "geometry", None)
        if isinstance(geom, MillToolGeometry):
            raw_sections = getattr(geom, "holder_sections", None) or []
            cache_key = (
                id(geom),
                geom.diameter,
                geom.corner_radius,
                geom.flute_length,
                getattr(geom, "body_length", 0.0),
                geom.overall_length,
                getattr(geom, "shaft_diameter", 0.0),
                geom.holder_diameter,
                geom.holder_length,
                geom.taper_angle_deg,
                id(raw_sections),
                len(raw_sections),
            )
            if self._cached_tool_key == cache_key and self._cached_tool_params is not None:
                return self._cached_tool_params

            d = max(0.5, float(geom.diameter))
            tool_r = d / 2.0
            cr = max(0.0, float(geom.corner_radius))
            flute = max(0.0, float(geom.flute_length))
            raw_body = float(getattr(geom, "body_length", 0.0) or 0.0)
            body_len = raw_body if raw_body > 0.0 else max(flute, float(geom.overall_length))
            raw_shaft = float(getattr(geom, "shaft_diameter", 0.0) or 0.0)
            shank_r = (raw_shaft / 2.0) if raw_shaft > 0.0 else tool_r

            valid_sections: List[Tuple[float, float]] = []
            for sec in raw_sections:
                sec_l, sec_d = float(sec[0]), float(sec[1])
                if sec_l >= 0.0 and sec_d > 0.0:
                    valid_sections.append((sec_l, sec_d))

            if valid_sections:
                has_holder = True
                holder_sections = valid_sections
                holder_max_r = max(sec_d * 0.5 for _, sec_d in valid_sections)
            elif float(geom.holder_diameter) > 0.0:
                has_holder = True
                hl = float(geom.holder_length) if float(geom.holder_length) > 0.0 else self.default_holder_length
                holder_sections = [(hl, float(geom.holder_diameter))]
                holder_max_r = float(geom.holder_diameter) * 0.5
            else:
                has_holder = False
                holder_sections = []
                holder_max_r = 0.0

            taper = float(geom.taper_angle_deg)
            res = (tool_r, cr, flute, body_len, shank_r, has_holder, holder_sections, holder_max_r, taper)
            self._cached_tool_key = cache_key
            self._cached_tool_params = res
            return res

        # Fallback values when ToolAssembly geometry is missing
        r = self.default_cutter_diameter / 2.0
        return (
            r,
            0.0,
            self.default_flute_length,
            self.default_overall_length,
            r,
            True,
            [(self.default_holder_length, self.default_holder_diameter)],
            self.default_holder_diameter / 2.0,
            0.0,
        )

    def _sweep_subgrid(
        self,
        p1: Vector3D,
        p2: Vector3D,
        tool_r: float,
        corner_r: float,
        taper_deg: float,
        col_min: int,
        col_max: int,
        row_min: int,
        row_max: int,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Compute swept tool envelope over a subgrid window.
        Returns: (mask_inside_tool, z_sweep_cutter, t_param, dist_xy)
        """
        X, Y = self.grid.get_mesh_coordinates(col_min, col_max, row_min, row_max)

        x1, y1, z1 = float(p1.x), float(p1.y), float(p1.z)
        x2, y2, z2 = float(p2.x), float(p2.y), float(p2.z)

        vx = x2 - x1
        vy = y2 - y1
        L2 = vx * vx + vy * vy

        if L2 < 1e-10:
            # Pure vertical or dwell motion
            dist_xy = np.sqrt((X - x1) ** 2 + (Y - y1) ** 2)
            t_param = np.zeros_like(dist_xy, dtype=np.float64)
            mask = dist_xy <= (tool_r + 1e-5)
            z_tip = min(z1, z2)
            z_rel = evaluate_tool_bottom_profile(dist_xy, tool_r, corner_r, taper_deg)
            z_sweep = (np.float32(z_tip) + z_rel).astype(np.float32)
            return mask, z_sweep, t_param, dist_xy

        # Linear motion in XY
        wx = X - x1
        wy = Y - y1
        t_param = np.clip((wx * vx + wy * vy) / L2, 0.0, 1.0)

        cx = x1 + t_param * vx
        cy = y1 + t_param * vy
        dist_xy = np.sqrt((X - cx) ** 2 + (Y - cy) ** 2)

        mask = dist_xy <= (tool_r + 1e-5)
        dz = z2 - z1
        if abs(dz) <= 1e-9:
            z_rel = evaluate_tool_bottom_profile(dist_xy, tool_r, corner_r, taper_deg)
            z_sweep = (np.float32(z1) + z_rel).astype(np.float32)
            return mask, z_sweep, t_param, dist_xy

        # Sloped 3D segment: a cell (X, Y) inside mask is swept over parameter interval
        # [t_enter, t_exit] subset [0, 1] where ||(X, Y) - p(t)|| <= tool_r.
        L = math.sqrt(L2)
        u_par = (wx * vx + wy * vy) / L
        d_line_sq = np.maximum(0.0, (wx * wx + wy * wy) - u_par * u_par)
        r_eff = tool_r + 1e-5
        half_chord = np.sqrt(np.maximum(0.0, r_eff * r_eff - d_line_sq))
        t_enter = np.clip((u_par - half_chord) / L, 0.0, 1.0)
        t_exit = np.clip((u_par + half_chord) / L, 0.0, 1.0)

        if corner_r <= 1e-4 and taper_deg <= 0.01:
            # Flat endmill: z_rel == 0 everywhere inside mask, so z_tip(t) = z1 + t * dz
            # achieves its exact minimum at t_exit (if dz < 0) or t_enter (if dz > 0).
            t_opt = t_exit if dz < 0.0 else t_enter
            z_sweep = (z1 + t_opt * dz).astype(np.float32)
        else:
            # Bull-nose, ball, or tapered cutter: sample [t_enter, t_closest, t_exit] + midpoints
            t_m1 = 0.5 * (t_enter + t_param)
            t_m2 = 0.5 * (t_param + t_exit)
            z_sweep = (
                z1 + t_param * dz + evaluate_tool_bottom_profile(dist_xy, tool_r, corner_r, taper_deg)
            ).astype(np.float32)
            for t_cand in (t_enter, t_m1, t_m2, t_exit):
                d_cand = np.sqrt(d_line_sq + (u_par - t_cand * L) ** 2)
                valid_c = d_cand <= r_eff
                z_cand = (
                    z1
                    + t_cand * dz
                    + evaluate_tool_bottom_profile(np.minimum(d_cand, tool_r), tool_r, corner_r, taper_deg)
                ).astype(np.float32)
                z_sweep = np.where(valid_c & (z_cand < z_sweep), z_cand, z_sweep)

        return mask, z_sweep, t_param, dist_xy

    def step(self, move: MotionSegment) -> MoveEvaluation:
        """
        Evaluate a single kinematic move against the heightfield IPW,
        record material removal, detect collisions, check rapid promotion, and advance time.
        """
        self.current_time_sec += max(0.0, float(move.duration_sec))
        self.total_machining_time_sec += max(0.0, float(move.duration_sec))

        meta = move.metadata or {}
        if bool(meta.get("tilted_section", False)) or bool(meta.get("multi_axis", False)):
            sec_idx = meta.get("sec_idx", move.sec_id)
            event_idx = meta.get("event_idx")
            if event_idx is None:
                event_idx = move.segment_id
            self.record_event_evaluation(move, is_air_move=False, collisions=[])
            return MoveEvaluation(
                segment_id=move.segment_id,
                is_air_move=False,
                material_removed_mm3=0.0,
                instantaneous_mrr=0.0,
                collisions=[],
                sec_idx=sec_idx,
                event_idx=event_idx,
            )

        # Handle circular arcs by discretizing into linear sub-segments
        if move.is_arc and move.arc_center is not None:
            return self._step_arc(move)

        return self._step_linear(move.start, move.end, move)

    def _step_linear(
        self,
        p1: Vector3D,
        p2: Vector3D,
        move: MotionSegment,
        record_metrics: bool = True,
    ) -> MoveEvaluation:
        """Process a single linear kinematic move."""
        (
            tool_r,
            corner_r,
            flute_len,
            body_len,
            shank_r,
            has_holder,
            holder_sections,
            holder_max_r,
            taper_deg,
        ) = self._get_active_mill_tool_params()

        if not has_holder:
            op_label = move.op_id if move.op_id else f"Section_{move.sec_id}"
            self.holder_unchecked_ops.add(op_label)

        # Update per-section clearance tracking
        top_z = max(p1.z, p2.z)
        sec_id = move.sec_id
        if sec_id not in self._sec_clearance_mm or top_z > self._sec_clearance_mm[sec_id]:
            self._sec_clearance_mm[sec_id] = top_z

        move_collisions: List[CollisionRecord] = []

        # ---------------------------------------------------------------------
        # 1. Bounding Subgrid Extraction (Decoupled Core vs. Holder Envelope)
        # ---------------------------------------------------------------------
        # Phase 2 H2: Promotion clearance radius includes lateral margin and grid cell diagonal bound
        cell_diag_half = self.grid.cell_size_mm * math.sqrt(2.0) / 2.0
        clearance_r = tool_r + self.promote_margin_mm + cell_diag_half
        core_r = max(tool_r, shank_r, clearance_r) + 1e-4
        col_min, col_max, row_min, row_max = self.grid.get_subgrid_bounds(
            p1.x, p1.y, p2.x, p2.y, core_r
        )
        core_nonempty = (col_max > col_min) and (row_max > row_min)

        z_tip_min = min(float(p1.z), float(p2.z))
        z_holder_min = float(np.float32(z_tip_min + body_len)) - 1e-3
        h_cmin = h_cmax = h_rmin = h_rmax = 0
        h_sub_max = -math.inf
        holder_can_strike = False
        holder_can_violate_margin = False
        if has_holder and holder_max_r > 0.0:
            h_cmin, h_cmax, h_rmin, h_rmax = self.grid.get_subgrid_bounds(
                p1.x, p1.y, p2.x, p2.y, holder_max_r + 1e-4
            )
            if h_cmax > h_cmin and h_rmax > h_rmin:
                h_sub_max = float(np.max(self.grid.H[h_rmin:h_rmax, h_cmin:h_cmax]))
                holder_can_strike = h_sub_max > (z_holder_min + self.z_tol_mm)
                holder_can_violate_margin = h_sub_max > (z_holder_min - self.holder_margin_mm)

        # If move is completely outside both core and active holder bounds, return safe air move
        if not core_nonempty and not holder_can_strike and not holder_can_violate_margin:
            is_air_outside = not self._promotion_blocked(move)
            if record_metrics:
                self.record_event_evaluation(move, is_air_move=is_air_outside, collisions=[])
            return MoveEvaluation(
                segment_id=move.segment_id,
                is_air_move=is_air_outside,
                material_removed_mm3=0.0,
                instantaneous_mrr=0.0,
                collisions=[],
                sec_idx=move.metadata.get("sec_idx", move.sec_id) if move.metadata else move.sec_id,
                event_idx=move.metadata.get("event_idx", move.segment_id) if move.metadata else move.segment_id,
            )

        if not core_nonempty:
            col_min, col_max, row_min, row_max = h_cmin, h_cmax, h_rmin, h_rmax

        sub_H = self.grid.H[row_min:row_max, col_min:col_max]

        # ---------------------------------------------------------------------
        # 2. Swept Cutter Footprint & Height Calculation
        # ---------------------------------------------------------------------
        mask_cutter, z_sweep_cutter, t_param, dist_cutter = self._sweep_subgrid(
            p1, p2, tool_r, corner_r, taper_deg, col_min, col_max, row_min, row_max
        )

        meta_move = getattr(move, "metadata", None) or {}
        sec_idx = meta_move.get("sec_idx", move.sec_id)
        event_idx = meta_move.get("event_idx", move.segment_id)

        # ---------------------------------------------------------------------
        # 3. Holder & Shank Collision Checks
        # ---------------------------------------------------------------------
        vx = p2.x - p1.x
        vy = p2.y - p1.y
        L2 = vx * vx + vy * vy

        if holder_can_strike:
            cum_len = 0.0
            strike_secs: List[Tuple[float, float]] = []
            strike_max_r = 0.0
            for sec_len, sec_dia in holder_sections:
                if (z_holder_min + cum_len) < (h_sub_max - self.z_tol_mm):
                    sec_r = sec_dia * 0.5
                    strike_secs.append((cum_len, sec_r))
                    if sec_r > strike_max_r:
                        strike_max_r = sec_r
                cum_len += sec_len

            if strike_secs and strike_max_r > 0.0:
                s_cmin, s_cmax, s_rmin, s_rmax = self.grid.get_subgrid_bounds(
                    p1.x, p1.y, p2.x, p2.y, strike_max_r + 1e-4
                )
                if s_cmax > s_cmin and s_rmax > s_rmin:
                    sub_H_s = self.grid.H[s_rmin:s_rmax, s_cmin:s_cmax]
                    s_sub_max = float(np.max(sub_H_s))
                    if s_sub_max > (z_holder_min + self.z_tol_mm):
                        X_h, Y_h = self.grid.get_mesh_coordinates(s_cmin, s_cmax, s_rmin, s_rmax)
                        if L2 < 1e-10:
                            dist_h = np.sqrt((X_h - p1.x) ** 2 + (Y_h - p1.y) ** 2)
                            z_h_bot = np.full(dist_h.shape, z_tip_min + body_len, dtype=np.float32)
                        else:
                            t_h = np.clip(((X_h - p1.x) * vx + (Y_h - p1.y) * vy) / L2, 0.0, 1.0)
                            cx_h = p1.x + t_h * vx
                            cy_h = p1.y + t_h * vy
                            dist_h = np.sqrt((X_h - cx_h) ** 2 + (Y_h - cy_h) ** 2)
                            z_h_bot = (p1.z + t_h * (p2.z - p1.z) + body_len).astype(np.float32)

                        z_holder_env_col = np.full(dist_h.shape, np.inf, dtype=np.float32)
                        for c_len, sec_r in strike_secs:
                            if (z_holder_min + c_len) >= (s_sub_max - self.z_tol_mm):
                                continue
                            col_sec = dist_h <= (sec_r - self.collision_lateral_tol_mm)
                            if np.any(col_sec):
                                sec_z_bot = z_h_bot + np.float32(c_len)
                                z_holder_env_col = np.where(col_sec, np.minimum(z_holder_env_col, sec_z_bot), z_holder_env_col)

                        # Holder strike: stock top is higher than holder envelope bottom
                        holder_penetration = sub_H_s - z_holder_env_col
                        holder_strike_mask = np.isfinite(z_holder_env_col) & (holder_penetration > self.z_tol_mm)

                        if np.any(holder_strike_mask):
                            max_pen = float(np.max(holder_penetration[holder_strike_mask]))
                            pen_vol = float(np.sum(holder_penetration[holder_strike_mask])) * self.grid.cell_area_mm2
                            if pen_vol >= self.min_collision_volume_mm3:
                                col = CollisionRecord(
                                    segment_id=move.segment_id,
                                    kind=CollisionKind.HOLDER_STRIKE,
                                    timestamp_sec=self.current_time_sec,
                                    contact_point=Vector3D(p2.x, p2.y, float(p2.z + body_len)),
                                    penetration_volume_mm3=pen_vol,
                                    description=(
                                        f"Toolholder collided with uncut stock (max penetration: {max_pen:.3f} mm, "
                                        f"volume: {pen_vol:.2f} mm³)"
                                    ),
                                    op_id=move.op_id,
                                    sec_idx=sec_idx,
                                    event_idx=event_idx,
                                )
                                if record_metrics:
                                    self.collisions.append(col)
                                move_collisions.append(col)

        # Shank rub check: band from tip + flute_len up to holder bottom using shaft radius
        if move.motion != MotionKind.RAPID:
            core_sub_max = float(np.max(sub_H))
            z_shank_min = float(np.float32(z_tip_min + flute_len)) - 1e-3
            if core_sub_max > (z_shank_min + self.z_tol_mm):
                if L2 < 1e-10:
                    z_holder_bottom = np.full(dist_cutter.shape, z_tip_min + body_len, dtype=np.float32)
                    z_shank_bottom = np.full(dist_cutter.shape, z_tip_min + flute_len, dtype=np.float32)
                else:
                    z_tip_s = p1.z + t_param * (p2.z - p1.z)
                    z_holder_bottom = (z_tip_s + body_len).astype(np.float32)
                    z_shank_bottom = (z_tip_s + flute_len).astype(np.float32)
                mask_shank_col = dist_cutter <= (shank_r - self.collision_lateral_tol_mm)
                shank_top_in_stock = np.minimum(sub_H, z_holder_bottom)
                shank_rub_penetration = shank_top_in_stock - z_shank_bottom
                shank_rub_mask = mask_shank_col & (shank_rub_penetration > self.z_tol_mm)
                if np.any(shank_rub_mask):
                    max_pen = float(np.max(shank_rub_penetration[shank_rub_mask]))
                    pen_vol = float(np.sum(shank_rub_penetration[shank_rub_mask])) * self.grid.cell_area_mm2
                    if pen_vol >= self.min_collision_volume_mm3:
                        col = CollisionRecord(
                            segment_id=move.segment_id,
                            kind=CollisionKind.SHANK_RUB,
                            timestamp_sec=self.current_time_sec,
                            contact_point=Vector3D(p2.x, p2.y, float(p2.z + flute_len)),
                            penetration_volume_mm3=pen_vol,
                            description=(
                                f"Unfluted tool shank rubbed into raw stock (depth exceeded flute by {max_pen:.3f} mm)"
                            ),
                            op_id=move.op_id,
                            sec_idx=sec_idx,
                            event_idx=event_idx,
                        )
                        if record_metrics:
                            self.collisions.append(col)
                        move_collisions.append(col)

        # ---------------------------------------------------------------------
        # 4. Rapid Move Collision (RAPID_INTO_STOCK)
        # ---------------------------------------------------------------------
        stock_cut_depth = sub_H - z_sweep_cutter
        rapid_into_stock_mask = mask_cutter & (stock_cut_depth > self.z_tol_mm)

        if move.motion == MotionKind.RAPID:
            if np.any(rapid_into_stock_mask):
                max_pen = float(np.max(stock_cut_depth[rapid_into_stock_mask]))
                pen_vol = float(np.sum(stock_cut_depth[rapid_into_stock_mask])) * self.grid.cell_area_mm2
                col = CollisionRecord(
                    segment_id=move.segment_id,
                    kind=CollisionKind.RAPID_INTO_STOCK,
                    timestamp_sec=self.current_time_sec,
                    contact_point=Vector3D(p2.x, p2.y, p2.z),
                    penetration_volume_mm3=pen_vol,
                    description=(
                        f"Rapid move crashed into uncut stock (penetration: {max_pen:.3f} mm, "
                        f"volume: {pen_vol:.2f} mm³)"
                    ),
                    op_id=move.op_id,
                    sec_idx=sec_idx,
                    event_idx=event_idx,
                )
                if record_metrics:
                    self.collisions.append(col)
                move_collisions.append(col)

            # Rapid moves do not modify stock or get promoted
            return MoveEvaluation(
                segment_id=move.segment_id,
                is_air_move=True,
                material_removed_mm3=0.0,
                instantaneous_mrr=0.0,
                collisions=move_collisions,
                sec_idx=sec_idx,
                event_idx=event_idx,
            )

        # ---------------------------------------------------------------------
        # 5. Material Removal & Volumetric MRR Integration
        # ---------------------------------------------------------------------
        vol_removed = 0.0
        mrr_rate = 0.0
        doc_mm = 0.0
        cuts_material = np.any(rapid_into_stock_mask)

        if cuts_material:
            cut_depths = stock_cut_depth[rapid_into_stock_mask]
            vol_removed = float(np.sum(cut_depths)) * self.grid.cell_area_mm2
            doc_mm = float(np.max(cut_depths))

            # Carve heightfield stock
            sub_H[rapid_into_stock_mask] = np.maximum(
                self.grid.z_floor,
                z_sweep_cutter[rapid_into_stock_mask]
            )

            if record_metrics:
                sample = self.record_mrr_sample(
                    segment_id=move.segment_id,
                    duration_sec=move.duration_sec,
                    volume_removed_mm3=vol_removed,
                    depth_of_cut_mm=doc_mm,
                    op_id=move.op_id,
                )
                mrr_rate = sample.mrr_rate_mm3_per_min
            else:
                dur = max(self.min_mrr_duration_sec, 1e-4, float(move.duration_sec))
                mrr_rate = (vol_removed / dur) * 60.0

        # ---------------------------------------------------------------------
        # 6. Air Move & Rapid Promotion Check (for G1 feed moves)
        # ---------------------------------------------------------------------
        meta = move.metadata or {}
        in_cycle = bool(meta.get("in_cycle", False))
        dist_3d = math.sqrt((p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2 + (p2.z - p1.z) ** 2)

        # Check pure vertical Z+ clearance retract
        dx = p2.x - p1.x
        dy = p2.y - p1.y
        dz = p2.z - p1.z
        is_pure_z_plus = (math.hypot(dx, dy) <= self.xy_tol_mm) and (dz > self.z_tol_mm)
        clearance_z = meta.get("z_clearance_mm")
        if clearance_z is None:
            clearance_z = self._sec_clearance_mm.get(sec_id)

        is_retract_air = (
            (not in_cycle)
            and is_pure_z_plus
            and (not cuts_material)
            and (not move_collisions)
            and self.has_valid_stock
            and (clearance_z is not None)
            and (p2.z >= (clearance_z - self.z_tol_mm))
        )

        is_air_move = False
        if is_retract_air:
            is_air_move = True
        elif (not cuts_material) and (not in_cycle) and (dist_3d > 1e-6):
            # Check holder clearance (conservative, no collision lateral shrinkage)
            holder_margin_clear = True
            if holder_can_violate_margin:
                cum_len = 0.0
                margin_secs: List[Tuple[float, float]] = []
                margin_max_r = 0.0
                for sec_len, sec_dia in holder_sections:
                    if (z_holder_min + cum_len) < (h_sub_max + self.holder_margin_mm):
                        sec_r = sec_dia * 0.5
                        margin_secs.append((cum_len, sec_r))
                        if sec_r > margin_max_r:
                            margin_max_r = sec_r
                    cum_len += sec_len

                if margin_secs and margin_max_r > 0.0:
                    m_cmin, m_cmax, m_rmin, m_rmax = self.grid.get_subgrid_bounds(
                        p1.x, p1.y, p2.x, p2.y, margin_max_r + 1e-4
                    )
                    if m_cmax > m_cmin and m_rmax > m_rmin:
                        sub_H_m = self.grid.H[m_rmin:m_rmax, m_cmin:m_cmax]
                        m_sub_max = float(np.max(sub_H_m))
                        if m_sub_max > (z_holder_min - self.holder_margin_mm):
                            X_m, Y_m = self.grid.get_mesh_coordinates(m_cmin, m_cmax, m_rmin, m_rmax)
                            if L2 < 1e-10:
                                dist_m = np.sqrt((X_m - p1.x) ** 2 + (Y_m - p1.y) ** 2)
                                z_m_bot = np.full(dist_m.shape, z_tip_min + body_len, dtype=np.float32)
                            else:
                                t_m = np.clip(((X_m - p1.x) * vx + (Y_m - p1.y) * vy) / L2, 0.0, 1.0)
                                cx_m = p1.x + t_m * vx
                                cy_m = p1.y + t_m * vy
                                dist_m = np.sqrt((X_m - cx_m) ** 2 + (Y_m - cy_m) ** 2)
                                z_m_bot = (p1.z + t_m * (p2.z - p1.z) + body_len).astype(np.float32)
                            for c_len, sec_r in margin_secs:
                                if (z_holder_min + c_len) >= (m_sub_max + self.holder_margin_mm):
                                    continue
                                clr_sec = dist_m <= (sec_r + 1e-5)
                                if np.any(clr_sec):
                                    sec_z_bot = z_m_bot + np.float32(c_len)
                                    if np.any(sub_H_m[clr_sec] - (sec_z_bot[clr_sec] - self.holder_margin_mm) > 0.0):
                                        holder_margin_clear = False
                                        break

            # Check cutter clearance
            # Phase 2 H2: clearance check uses clearance_r (lateral margin + cell diagonal discretization bound)
            cutter_margin_clear = True
            if self.promote_margin_mm > 1e-4:
                mask_clearance = dist_cutter <= (clearance_r + 1e-5)
                if np.any(mask_clearance):
                    if L2 < 1e-10:
                        z_tip_c = np.float32(min(p1.z, p2.z))
                    else:
                        dz_c = p2.z - p1.z
                        if abs(dz_c) <= 1e-9:
                            z_tip_c = p1.z + t_param * dz_c
                        else:
                            X_sub, Y_sub = self.grid.get_mesh_coordinates(col_min, col_max, row_min, row_max)
                            L_c = math.sqrt(L2)
                            wx_c = X_sub - p1.x
                            wy_c = Y_sub - p1.y
                            u_par_c = (wx_c * vx + wy_c * vy) / L_c
                            d_line_sq_c = np.maximum(0.0, (wx_c * wx_c + wy_c * wy_c) - u_par_c * u_par_c)
                            r_clr = clearance_r + 1e-5
                            half_chord_c = np.sqrt(np.maximum(0.0, r_clr * r_clr - d_line_sq_c))
                            t_deep = (
                                np.clip((u_par_c + half_chord_c) / L_c, 0.0, 1.0)
                                if dz_c < 0.0
                                else np.clip((u_par_c - half_chord_c) / L_c, 0.0, 1.0)
                            )
                            z_tip_c = np.minimum(p1.z + t_param * dz_c, p1.z + t_deep * dz_c)
                    z_effective = z_tip_c + evaluate_tool_bottom_profile(
                        np.minimum(dist_cutter, tool_r), tool_r, corner_r, taper_deg
                    )
                    margin_cut_depth = sub_H[mask_clearance] - (z_effective[mask_clearance] - self.promote_margin_mm)
                    if np.any(margin_cut_depth > 0.0):
                        cutter_margin_clear = False

            if holder_margin_clear and cutter_margin_clear and self.has_valid_stock:
                is_air_move = True

        if self._promotion_blocked(move):
            is_air_move = False

        if record_metrics:
            self.record_event_evaluation(move, is_air_move=is_air_move, collisions=move_collisions)

        return MoveEvaluation(
            segment_id=move.segment_id,
            is_air_move=is_air_move,
            material_removed_mm3=vol_removed,
            instantaneous_mrr=mrr_rate,
            collisions=move_collisions,
            sec_idx=sec_idx,
            event_idx=event_idx,
            depth_of_cut_mm=doc_mm,
        )

    def _step_arc(self, move: MotionSegment) -> MoveEvaluation:
        """Discretize circular arc move in XY, ZX, or YZ plane into chordal segments and evaluate."""
        p1 = move.start
        p2 = move.end
        center = move.arc_center
        assert center is not None

        plane = str(getattr(move, "arc_plane", "XY") or "XY").upper()
        # Right-handed cyclic plane axes (u, v) with normal u x v and linear axis w:
        #   XY (G17): u=X, v=Y, w=Z
        #   ZX (G18): u=Z, v=X, w=Y
        #   YZ (G19): u=Y, v=Z, w=X
        if plane in ("ZX", "XZ"):
            u1, v1, w1 = float(p1.z), float(p1.x), float(p1.y)
            u2, v2, w2 = float(p2.z), float(p2.x), float(p2.y)
            cu, cv = float(center.z), float(center.x)
            def _to_vec(u: float, v: float, w: float) -> Vector3D:
                return Vector3D(v, w, u)
        elif plane in ("YZ", "ZY"):
            u1, v1, w1 = float(p1.y), float(p1.z), float(p1.x)
            u2, v2, w2 = float(p2.y), float(p2.z), float(p2.x)
            cu, cv = float(center.y), float(center.z)
            def _to_vec(u: float, v: float, w: float) -> Vector3D:
                return Vector3D(w, u, v)
        else:
            u1, v1, w1 = float(p1.x), float(p1.y), float(p1.z)
            u2, v2, w2 = float(p2.x), float(p2.y), float(p2.z)
            cu, cv = float(center.x), float(center.y)
            def _to_vec(u: float, v: float, w: float) -> Vector3D:
                return Vector3D(u, v, w)

        r1 = math.hypot(u1 - cu, v1 - cv)
        r2 = math.hypot(u2 - cu, v2 - cv)
        if r1 < 1e-6 and r2 < 1e-6:
            return self._step_linear(p1, p2, move, record_metrics=True)

        theta1 = math.atan2(v1 - cv, u1 - cu)
        theta2 = math.atan2(v2 - cv, u2 - cu)

        is_cw = (move.motion == MotionKind.ARC_CW)
        if is_cw:
            if theta2 >= theta1 - 1e-9:
                theta2 -= 2.0 * math.pi
        else:
            if theta2 <= theta1 + 1e-9:
                theta2 += 2.0 * math.pi

        delta = theta2 - theta1
        # Discretize arc with max angle step <= 10 deg and chordal deviation <= cell_size / 2
        r_max = max(r1, r2)
        max_angle_step = math.radians(10.0)
        max_chord_dev = self.grid.cell_size_mm * 0.5
        if r_max > max_chord_dev > 0.0:
            cos_half = max(-1.0, 1.0 - max_chord_dev / r_max)
            chord_step = max(math.radians(1.0), 2.0 * math.acos(cos_half))
            max_angle_step = min(max_angle_step, chord_step)
        steps = max(3, int(math.ceil(abs(delta) / max_angle_step)))

        tot_vol = 0.0
        max_doc = 0.0
        all_cols: List[CollisionRecord] = []
        is_air_all = True

        # Step each chordal sub-segment
        for i in range(steps):
            t_a = i / float(steps)
            t_b = (i + 1) / float(steps)
            ang_a = theta1 + delta * t_a
            ang_b = theta1 + delta * t_b
            r_a = r1 + t_a * (r2 - r1)
            r_b = r1 + t_b * (r2 - r1)

            sub_p1 = _to_vec(cu + r_a * math.cos(ang_a), cv + r_a * math.sin(ang_a), w1 + t_a * (w2 - w1))
            sub_p2 = _to_vec(cu + r_b * math.cos(ang_b), cv + r_b * math.sin(ang_b), w1 + t_b * (w2 - w1))

            sub_eval = self._step_linear(sub_p1, sub_p2, move, record_metrics=False)
            tot_vol += sub_eval.material_removed_mm3
            max_doc = max(max_doc, sub_eval.depth_of_cut_mm)
            if not sub_eval.is_air_move:
                is_air_all = False
            all_cols.extend(sub_eval.collisions)

        if tot_vol > 0.0:
            sample = self.record_mrr_sample(
                segment_id=move.segment_id,
                duration_sec=move.duration_sec,
                volume_removed_mm3=tot_vol,
                depth_of_cut_mm=max_doc,
                op_id=move.op_id,
            )
            mrr_rate = sample.mrr_rate_mm3_per_min
        else:
            mrr_rate = 0.0

        dedup_by_kind: Dict[CollisionKind, CollisionRecord] = {}
        for col in all_cols:
            existing = dedup_by_kind.get(col.kind)
            if existing is None or col.penetration_volume_mm3 > existing.penetration_volume_mm3:
                dedup_by_kind[col.kind] = col
        dedup_cols = list(dedup_by_kind.values())
        self.collisions.extend(dedup_cols)

        self.record_event_evaluation(move, is_air_move=is_air_all, collisions=dedup_cols)

        meta_arc = getattr(move, "metadata", None) or {}
        return MoveEvaluation(
            segment_id=move.segment_id,
            is_air_move=is_air_all,
            material_removed_mm3=tot_vol,
            instantaneous_mrr=mrr_rate,
            collisions=dedup_cols,
            sec_idx=meta_arc.get("sec_idx", move.sec_id),
            event_idx=meta_arc.get("event_idx", move.segment_id),
            depth_of_cut_mm=max_doc,
        )

    def get_heightfield_snapshot(self) -> Tuple[np.ndarray, Tuple[float, float, float, float]]:
        """Return a copy of the current 2D heightfield array and its (xmin, xmax, ymin, ymax) extents."""
        bounds = (self.grid.grid_x_min, self.grid.grid_x_max, self.grid.grid_y_min, self.grid.grid_y_max)
        return np.copy(self.grid.H), bounds
