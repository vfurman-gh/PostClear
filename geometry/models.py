"""
Core Domain Models for Workpiece, Tool, and Kinematic Verification.
"""

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any, Dict, List, Optional, Sequence, Set, Tuple, Union

try:
    from shapely.geometry import Polygon
except (ImportError, OSError):
    Polygon = Any  # Type alias fallback for Fusion 360 embedded Python



class MotionKind(Enum):
    RAPID = auto()          # G0
    LINEAR = auto()         # G1
    ARC_CW = auto()         # G2
    ARC_CCW = auto()        # G3
    PROBE = auto()          # G38.x
    DWELL = auto()          # G4


class CollisionKind(Enum):
    RAPID_INTO_STOCK = auto()
    SHANK_RUB = auto()
    HOLDER_STRIKE = auto()
    FIXTURE_COLLISION = auto()
    EXCESSIVE_ENGAGEMENT = auto()


class ToolCompensationQuadrant(Enum):
    """
    Lathe Tool Nose Radius Compensation (TNRC) Quadrants (P1 - P9).
    """
    P1 = 1  # X+ Z+ (Boring front)
    P2 = 2  # X+ Z- (Boring back)
    P3 = 3  # X- Z- (OD turning back - standard right-hand OD tool)
    P4 = 4  # X- Z+ (OD turning front)
    P5 = 5  # Top edge
    P6 = 6  # Right edge
    P7 = 7  # Bottom edge
    P8 = 8  # Left edge
    P9 = 9  # Center of tool nose arc (no directional bias)


@dataclass(frozen=True)
class Vector3D:
    x: float
    y: float
    z: float


try:
    import numpy as np
except (ImportError, OSError):
    np = None


@dataclass(frozen=True)
class AxisymmetricStock2D:
    """Lathe stock profile revolved symmetrically around the Z-axis (R >= 0)."""
    z_min: float
    z_max: float
    outer_radius: float
    inner_radius: float = 0.0
    polygon: Any = None  # Optional Shapely Polygon for arbitrary revolved solid profile


@dataclass(frozen=True)
class BoxStock3D:
    """Prismatic bounding box for milling stock."""
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float


@dataclass(frozen=True)
class SolidMeshStock3D:
    """3D solid mesh stock representation in setup WCS coordinates (mm)."""
    vertices: Any   # np.ndarray of shape (N, 3), float32 in mm
    indices: Any    # np.ndarray of shape (M, 3), int32 triangle vertex indices
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float

    @classmethod
    def from_mesh(cls, vertices: Any, indices: Any) -> "SolidMeshStock3D":
        if np is None:
            raise ImportError("numpy is required for SolidMeshStock3D")
        v = np.asarray(vertices, dtype=np.float32)
        idx = np.asarray(indices, dtype=np.int32)
        if v.size == 0 or idx.size == 0:
            raise ValueError("SolidMeshStock3D requires non-empty vertices and indices")
        min_pt = v.min(axis=0)
        max_pt = v.max(axis=0)
        return cls(
            vertices=v,
            indices=idx,
            x_min=float(min_pt[0]),
            x_max=float(max_pt[0]),
            y_min=float(min_pt[1]),
            y_max=float(max_pt[1]),
            z_min=float(min_pt[2]),
            z_max=float(max_pt[2]),
        )


@dataclass(frozen=True)
class AxisymmetricChuckObstacle:
    """Lathe chuck body and jaws modeled as an axisymmetric swept keep-out body."""
    z_min: float
    z_max: float
    outer_radius: float
    clamping_radius: float

    @classmethod
    def from_stock(
        cls,
        stock: AxisymmetricStock2D,
        clamp_len_mm: float = 0.0,
        back_ext_mm: float = 50.0,
        jaw_height_mm: float = 30.0,
        z_min: Optional[float] = None,
        z_max: Optional[float] = None,
        outer_radius: Optional[float] = None,
    ) -> "AxisymmetricChuckObstacle":
        """Build a lathe chuck obstacle anchored to an AxisymmetricStock2D back face."""
        resolved_z_min = float(z_min) if z_min is not None else float(stock.z_min) - float(back_ext_mm)
        resolved_z_max = float(z_max) if z_max is not None else float(stock.z_min) + float(clamp_len_mm)
        resolved_clamp_r = float(stock.outer_radius)
        resolved_outer_r = (
            float(outer_radius)
            if outer_radius is not None
            else resolved_clamp_r + float(jaw_height_mm)
        )
        return cls(
            z_min=resolved_z_min,
            z_max=resolved_z_max,
            outer_radius=resolved_outer_r,
            clamping_radius=resolved_clamp_r,
        )


WorkpieceGeometry = Union[AxisymmetricStock2D, BoxStock3D, SolidMeshStock3D]


@dataclass
class WorkpieceDefinition:
    stock: WorkpieceGeometry
    fixtures: List[AxisymmetricChuckObstacle] = field(default_factory=list)
    work_offset: Vector3D = field(default_factory=lambda: Vector3D(0, 0, 0))
    is_stl_derived: bool = False


@dataclass(frozen=True)
class LatheToolGeometry:
    """2D insert profile and holder geometry in the (Z, Radius) plane."""
    insert_shape: str                               # e.g., 'C', 'D', 'V', 'W', 'T', 'Groove', 'Drill'
    nose_radius: float
    hand: str                                       # 'R' or 'L'
    quadrant: ToolCompensationQuadrant
    cutter_polygon: Polygon                         # Valid Shapely Polygon at tip (0, 0)
    holder_polygon: Polygon                         # Valid Shapely Polygon for shank keep-out
    is_boring: bool = False


@dataclass(frozen=True)
class MillToolGeometry:
    """3D parametric milling tool and holder collar."""
    diameter: float
    corner_radius: float
    flute_length: float
    overall_length: float
    taper_angle_deg: float = 0.0
    holder_diameter: float = 40.0
    holder_length: float = 60.0
    body_length: float = 0.0       # mm, length below holder; <=0 unknown -> fall back to overall_length
    shoulder_length: float = 0.0
    shaft_diameter: float = 0.0    # <=0 -> same as cutter diameter
    holder_sections: Optional[List[Tuple[float, float]]] = None  # [(length_mm, diameter_mm)] stacked upward from holder bottom


ToolGeometry = Union[LatheToolGeometry, MillToolGeometry]


@dataclass
class ToolAssembly:
    tool_id: int
    name: str
    geometry: ToolGeometry


@dataclass
class MotionSegment:
    """A continuous kinematic move in workpiece coordinates."""
    segment_id: int               # Monotonic sequential identifier within operation
    op_id: str                    # Owning CAM operation identifier
    sec_id: int                   # Section index in post processor
    motion: MotionKind
    start: Vector3D
    end: Vector3D
    feedrate_mmpm: float          # Standardized metric feedrate (mm/min)
    spindle_rpm: float
    duration_sec: float           # Explicit duration (handles G4 dwells where distance = 0)
    is_arc: bool = False
    arc_center: Optional[Vector3D] = None
    arc_plane: str = "XY"         # 'XY' (G17), 'ZX' (G18), 'YZ' (G19)
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class CollisionRecord:
    segment_id: int
    kind: CollisionKind
    timestamp_sec: float
    contact_point: Vector3D
    penetration_volume_mm3: float
    description: str
    op_id: str = ""
    sec_idx: Optional[int] = None
    event_idx: Optional[int] = None


@dataclass
class MoveEvaluation:
    """Evaluation output for an atomic step."""
    segment_id: int
    is_air_move: bool             # True if move removed 0 stock and cleared obstacles by >= margin
    material_removed_mm3: float   # Volume removed during this move
    instantaneous_mrr: float      # mm^3/min
    collisions: List[CollisionRecord] = field(default_factory=list)
    sec_idx: Optional[int] = None
    event_idx: Optional[int] = None
    depth_of_cut_mm: float = 0.0


@dataclass
class MRRSample:
    timestamp_sec: float
    duration_sec: float
    segment_id: int
    tool_id: int
    volume_removed_mm3: float
    mrr_rate_mm3_per_min: float
    depth_of_cut_mm: float
    op_id: str = ""


PromotedEventKey = Tuple[int, int]  # (sec_idx, event_idx)


@dataclass
class VerificationReport:
    total_machining_time_sec: float
    total_volume_removed_mm3: float
    collisions: List[CollisionRecord]
    promoted_events: Set[PromotedEventKey] = field(default_factory=set)
    mrr_timeline: List[MRRSample] = field(default_factory=list)
    mrr_spikes: List[MRRSample] = field(default_factory=list)
    operation_names: List[str] = field(default_factory=list)
    stock_bounding_box: Optional[str] = None
    toolpath_bounding_box: Optional[str] = None
    dog_leg_warning: Optional[str] = None
    fixture_warning: Optional[str] = None
    frame_warnings: List[str] = field(default_factory=list)
    frame_mismatch_ops: List[str] = field(default_factory=list)
    tilted_ops: List[str] = field(default_factory=list)
    multi_axis_ops: List[str] = field(default_factory=list)
    holder_unchecked_ops: List[str] = field(default_factory=list)
