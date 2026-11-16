#!/usr/bin/env python3
"""
Offline Corpus & Automated Oracles Runner for PostClear.

Runs harvested Fusion 360 setup bundles (`setup_manifest.json`) and standalone
`.cnc` fixtures across the curated Top 10 hobbyist CNC post processors (8 milling,
2 turning) and evaluates automated zero-human-labeling correctness oracles:

  Oracle 1: Direct-post parity (`promote=False` vs native Autodesk `post` binary)
  Oracle 2: Promotion safety, canned-cycle/coord diff guard, & volume conservation
  Oracle 3: Metamorphic invariance (`MM <-> IN` unit scaling & `180 deg` Z-rotation)
  Oracle 4: Optional post-setup IPW `.stl` heightfield comparison (diagnostic)
"""

from __future__ import annotations

import argparse
import concurrent.futures
import copy
import dataclasses
import difflib
import hashlib
import json
import math
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
import traceback
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List, Optional, Sequence, Set, Tuple

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

from context_parser import (
    apply_promotions_to_context,
    check_context_dict_capability,
    check_dumped_sections_capability,
    extract_tools_from_context,
    extract_workpiece_from_context,
    filter_promoted_events_by_length,
    merge_context_dumps,
    parse_context_dump,
)
from geometry.models import (
    AxisymmetricChuckObstacle,
    AxisymmetricStock2D,
    BoxStock3D,
    CollisionKind,
    SolidMeshStock3D,
    VerificationReport,
    WorkpieceDefinition,
)
from geometry.solid_mesh import extract_workpiece_from_stl, load_stl_mesh
from pipeline import (
    PipelineOrchestrator,
    get_post_search_dirs as _pipeline_get_post_search_dirs,
    resolve_post_file as _pipeline_resolve_post_file,
    scan_cps_capabilities,
)

# ---------------------------------------------------------------------------
# Curated Top 10 Hobbyist Post Processors
# ---------------------------------------------------------------------------

HOBBYIST_POSTS_TOP_10: Tuple[Dict[str, Optional[str]], ...] = (
    {
        "name": "grbl.cps",
        "fallback_name": None,
        "domain": "milling",
        "controller_family": "Grbl / grblHAL / FluidNC (OpenBuilds, Genmitsu, X-Carve)",
    },
    {
        "name": "carbide3d.cps",
        "fallback_name": None,
        "domain": "milling",
        "controller_family": "Carbide 3D (Shapeoko / Nomad)",
    },
    {
        "name": "mach3mill.cps",
        "fallback_name": None,
        "domain": "milling",
        "controller_family": "Artsoft Mach3 / Mach4 Mill & Router",
    },
    {
        "name": "linuxcnc.cps",
        "fallback_name": None,
        "domain": "milling",
        "controller_family": "LinuxCNC / EMC2 (PrintNC, Onefinity, Mesa)",
    },
    {
        "name": "shopbot.cps",
        "fallback_name": None,
        "domain": "milling",
        "controller_family": "ShopBot OpenSBP",
    },
    {
        "name": "tormach.cps",
        "fallback_name": None,
        "domain": "milling",
        "controller_family": "Tormach PathPilot (PCNC 440 / 770 / 1100)",
    },
    {
        "name": "centroid.cps",
        "fallback_name": None,
        "domain": "milling",
        "controller_family": "Centroid Acorn Mill",
    },
    {
        "name": "fanuc.cps",
        "fallback_name": None,
        "domain": "milling",
        "controller_family": "Fanuc ISO (Haas / Masso / UCCNC / DDCS baseline)",
    },
    {
        "name": "fanuc turning.cps",
        "fallback_name": None,
        "domain": "turning",
        "controller_family": "Fanuc ISO Lathe (Mach Turn / LinuxCNC Lathe)",
    },
    {
        "name": "centroid_turning.cps",
        "fallback_name": "haas turning.cps",
        "domain": "turning",
        "controller_family": "Centroid Acorn Lathe (fallback: Haas Turning)",
    },
)


def get_post_search_dirs() -> List[str]:
    """Return existing user, cache, and Autodesk system directories containing `.cps` posts."""
    return _pipeline_get_post_search_dirs()


def infer_post_capabilities(cps_path: str) -> Dict[str, bool]:
    """Inspect a `.cps` file's `capabilities = ...` declaration."""
    return scan_cps_capabilities(cps_path) or {"milling": False, "turning": False}


def resolve_post_file(post_name_or_path: str, search_dirs: Optional[Sequence[str]] = None) -> Optional[str]:
    """Resolve a `.cps` filename or absolute path from user/system post directories."""
    if search_dirs is None:
        search_dirs = get_post_search_dirs()
    return _pipeline_resolve_post_file(post_name_or_path, search_dirs=search_dirs)


def resolve_hobbyist_posts(
    extra_posts: Sequence[str] = (),
    only_posts: Optional[Sequence[str]] = None,
) -> Dict[str, List[Tuple[str, str]]]:
    """
    Resolve the curated Top 10 hobbyist posts (plus any `--extra-posts`) into
    `{"milling": [(label, abs_path), ...], "turning": [(label, abs_path), ...]}`.
    """
    search_dirs = get_post_search_dirs()
    only_filter = {p.lower().strip() for p in only_posts if p.strip()} if only_posts else None
    result: Dict[str, List[Tuple[str, str]]] = {"milling": [], "turning": []}
    seen_paths: Dict[str, Set[str]] = {"milling": set(), "turning": set()}

    for entry in HOBBYIST_POSTS_TOP_10:
        primary = entry["name"] or ""
        fallback = entry["fallback_name"]
        domain = str(entry["domain"])

        resolved = resolve_post_file(primary, search_dirs=search_dirs)
        chosen_name = primary
        if not resolved and fallback:
            resolved = resolve_post_file(fallback, search_dirs=search_dirs)
            chosen_name = fallback
        if not resolved:
            raise FileNotFoundError(
                f"Could not locate hobbyist post '{primary}'"
                + (f" or fallback '{fallback}'" if fallback else "")
                + f" in post directories: {search_dirs}"
            )
        if only_filter is not None and chosen_name.lower() not in only_filter and primary.lower() not in only_filter:
            continue
        if resolved not in seen_paths[domain]:
            seen_paths[domain].add(resolved)
            result[domain].append((chosen_name, resolved))

    for extra in extra_posts:
        resolved = resolve_post_file(extra, search_dirs=search_dirs)
        if not resolved:
            raise FileNotFoundError(f"Could not locate extra post '{extra}' in {search_dirs}")
        label = os.path.basename(resolved)
        if only_filter is not None and label.lower() not in only_filter:
            continue
        caps = infer_post_capabilities(resolved)
        domains: List[str] = []
        if caps["milling"]:
            domains.append("milling")
        if caps["turning"]:
            domains.append("turning")
        if not domains:
            domains = ["turning" if "turn" in label.lower() else "milling"]
        for dom in domains:
            if resolved not in seen_paths[dom]:
                seen_paths[dom].add(resolved)
                result[dom].append((label, resolved))

    return result


# ---------------------------------------------------------------------------
# Fixture Discovery & Workpiece Reconstruction
# ---------------------------------------------------------------------------


@dataclasses.dataclass
class CorpusFixture:
    name: str
    source_path: str
    cnc_batches: List[str]
    is_turning: bool
    stock_mode: str = ""
    unit: Optional[int] = None
    stock_unit: Optional[str] = None
    configured_post: Optional[str] = None
    post_properties: Dict[str, Any] = dataclasses.field(default_factory=dict)
    prev_stock_stl: Optional[str] = None
    prev_stock_is_inch: bool = False
    solid_stock_stl: Optional[str] = None
    post_setup_ipw_stl: Optional[str] = None
    workpiece_mm: Optional[Dict[str, Any]] = None
    explicit_workpiece: Optional[WorkpieceDefinition] = None
    fallback_workpiece: Optional[WorkpieceDefinition] = None
    metadata: Dict[str, Any] = dataclasses.field(default_factory=dict)


def _fixtures_from_manifest(wp_dict: Optional[Dict[str, Any]]) -> List[AxisymmetricChuckObstacle]:
    out: List[AxisymmetricChuckObstacle] = []
    if not isinstance(wp_dict, dict):
        return out
    for fx in wp_dict.get("fixtures") or []:
        if not isinstance(fx, dict):
            continue
        try:
            out.append(
                AxisymmetricChuckObstacle(
                    z_min=float(fx["z_min"]),
                    z_max=float(fx["z_max"]),
                    outer_radius=float(fx["outer_radius"]),
                    clamping_radius=float(fx["clamping_radius"]),
                )
            )
        except (KeyError, TypeError, ValueError):
            pass
    return out


def _build_workpiece_from_manifest_dict(wp_dict: Optional[Dict[str, Any]], is_turning: bool) -> Optional[WorkpieceDefinition]:
    if not isinstance(wp_dict, dict):
        return None
    fixtures = _fixtures_from_manifest(wp_dict)
    kind = str(wp_dict.get("kind") or ("turning" if is_turning else "box")).lower()
    if kind == "turning":
        try:
            stock = AxisymmetricStock2D(
                z_min=float(wp_dict["z_min"]),
                z_max=float(wp_dict["z_max"]),
                outer_radius=float(wp_dict["outer_radius"]),
                inner_radius=float(wp_dict.get("inner_radius", 0.0)),
            )
            return WorkpieceDefinition(stock=stock, fixtures=fixtures)
        except (KeyError, TypeError, ValueError):
            return None
    if kind == "box":
        try:
            stock = BoxStock3D(
                x_min=float(wp_dict["x_min"]),
                x_max=float(wp_dict["x_max"]),
                y_min=float(wp_dict["y_min"]),
                y_max=float(wp_dict["y_max"]),
                z_min=float(wp_dict["z_min"]),
                z_max=float(wp_dict["z_max"]),
            )
            return WorkpieceDefinition(stock=stock, fixtures=[])
        except (KeyError, TypeError, ValueError):
            return None
    return None


def _synthesize_fallback_workpiece_from_context(
    ctx: Dict[str, Any],
    is_turning: bool,
) -> WorkpieceDefinition:
    """Create a deterministic bounding stock when a standalone `.cnc` lacks workpiece extents."""
    segments = parse_context_dump(ctx)
    tools_by_op = extract_tools_from_context(ctx, is_turning=is_turning)
    cut_segs = [s for s in segments if s.motion != s.motion.RAPID] or segments
    if not cut_segs:
        if is_turning:
            return WorkpieceDefinition(stock=AxisymmetricStock2D(z_min=-50.0, z_max=5.0, outer_radius=25.0))
        return WorkpieceDefinition(stock=BoxStock3D(-50.0, 50.0, -50.0, 50.0, -25.0, 5.0))

    xs = [s.start.x for s in cut_segs] + [s.end.x for s in cut_segs]
    ys = [s.start.y for s in cut_segs] + [s.end.y for s in cut_segs]
    zs = [s.start.z for s in cut_segs] + [s.end.z for s in cut_segs]

    if is_turning:
        r_outer = max(10.0, max(abs(x) for x in xs) + 5.0)
        z_min = min(zs) - 10.0
        z_max = max(0.0, max(zs) + 1.0)
        return WorkpieceDefinition(stock=AxisymmetricStock2D(z_min=z_min, z_max=z_max, outer_radius=r_outer, inner_radius=0.0))

    max_tool_r = max(
        (float(getattr(getattr(t, "geometry", None), "diameter", 0.0) or 0.0) * 0.5 for t in tools_by_op.values()),
        default=5.0,
    )
    pad_xy = max(5.0, max_tool_r + 2.0)
    z_min = min(zs) - 5.0
    z_max = max(zs)
    if z_max <= z_min + 1e-3:
        z_max = z_min + 10.0
    return WorkpieceDefinition(
        stock=BoxStock3D(
            x_min=min(xs) - pad_xy,
            x_max=max(xs) + pad_xy,
            y_min=min(ys) - pad_xy,
            y_max=max(ys) + pad_xy,
            z_min=z_min,
            z_max=z_max,
        )
    )


def _resolve_bundle_path(bundle_dir: str, rel_or_abs: Optional[str]) -> Optional[str]:
    if not rel_or_abs:
        return None
    p = rel_or_abs if os.path.isabs(rel_or_abs) else os.path.join(bundle_dir, rel_or_abs)
    return os.path.abspath(p) if os.path.isfile(p) else None


def _manifest_bundle_is_unsupported(manifest_path: str) -> bool:
    """Return True if a harvested setup bundle contains unsupported 3+2, multi-axis, or mill-turn sections."""
    try:
        bundle_dir = os.path.dirname(os.path.abspath(manifest_path))
        with open(manifest_path, "r", encoding="utf-8") as f:
            data = json.load(f)
        if not isinstance(data, dict):
            return False
        json_paths = [
            bp
            for rel_j in (data.get("json_batches") or [])
            for bp in (_resolve_bundle_path(bundle_dir, rel_j),)
            if bp is not None
        ]
        if not json_paths:
            return False
        setup_name = str(data.get("setup_name") or os.path.basename(bundle_dir))
        is_turning = bool(data.get("is_turning", False))
        return (
            check_dumped_sections_capability(
                json_paths,
                setup_name=setup_name,
                is_turning=is_turning,
                check_probing=False,
            )
            is not None
        )
    except Exception:
        return False


def load_fixture_from_manifest(manifest_path: str) -> CorpusFixture:
    """Load a harvested setup bundle manifest into a `CorpusFixture`."""
    bundle_dir = os.path.dirname(os.path.abspath(manifest_path))
    with open(manifest_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    batches: List[str] = []
    for rel_b in data.get("cnc_batches") or []:
        bp = _resolve_bundle_path(bundle_dir, rel_b)
        if bp:
            batches.append(bp)
    if not batches:
        raise FileNotFoundError(f"Manifest {manifest_path} lists no existing cnc_batches")

    is_turning = bool(data.get("is_turning", False))
    prev_stl = _resolve_bundle_path(bundle_dir, data.get("prev_stock_stl"))
    solid_stl = _resolve_bundle_path(bundle_dir, data.get("solid_stock_stl"))
    post_ipw_stl = _resolve_bundle_path(bundle_dir, data.get("post_setup_ipw_stl"))

    wp_dict = data.get("workpiece_mm")
    fixtures = _fixtures_from_manifest(wp_dict)
    fallback_wp = _build_workpiece_from_manifest_dict(wp_dict, is_turning=is_turning)
    explicit_wp: Optional[WorkpieceDefinition] = None
    if prev_stl:
        explicit_wp = extract_workpiece_from_stl(
            prev_stl,
            is_turning=is_turning,
            is_inch=bool(data.get("prev_stock_is_inch", False)),
            fixtures=fixtures,
        )
    elif solid_stl:
        explicit_wp = extract_workpiece_from_stl(
            solid_stl,
            is_turning=is_turning,
            is_inch=False,
            fixtures=fixtures,
        )

    doc_name = data.get("document_name") or ""
    setup_name = data.get("setup_name") or os.path.basename(bundle_dir)
    proj_name = data.get("project_name") or ""
    display_parts = [p for p in (proj_name, doc_name, setup_name) if p]
    display_name = "/".join(display_parts) if display_parts else os.path.basename(bundle_dir)

    return CorpusFixture(
        name=display_name,
        source_path=bundle_dir,
        cnc_batches=batches,
        is_turning=is_turning,
        stock_mode=str(data.get("stock_mode") or ""),
        unit=data.get("unit"),
        stock_unit=data.get("stock_unit"),
        configured_post=data.get("configured_post"),
        post_properties=dict(data.get("post_properties") or {}),
        prev_stock_stl=prev_stl,
        prev_stock_is_inch=bool(data.get("prev_stock_is_inch", False)),
        solid_stock_stl=solid_stl,
        post_setup_ipw_stl=post_ipw_stl,
        workpiece_mm=wp_dict if isinstance(wp_dict, dict) else None,
        explicit_workpiece=explicit_wp,
        fallback_workpiece=fallback_wp,
        metadata=data,
    )


def load_standalone_cnc_fixture(
    cnc_path: str,
    probe_post_path: str,
    orchestrator: Optional[PipelineOrchestrator] = None,
) -> CorpusFixture:
    """Inspect a standalone `.cnc` file once to determine domain and stock bounds."""
    abs_cnc = os.path.abspath(cnc_path)
    orch = orchestrator or PipelineOrchestrator()
    with tempfile.TemporaryDirectory() as td:
        probe_json = os.path.join(td, "probe.json")
        orch.dump_intermediate_to_json(abs_cnc, probe_post_path, probe_json)
        with open(probe_json, "r", encoding="utf-8") as f:
            ctx = json.load(f)
    is_turning = any(
        s.get("type", 0) == 1 or bool(s.get("tool", {}).get("_isTurningTool", False))
        for s in ctx.get("sections", [])
    )
    cap_err = check_context_dict_capability(
        ctx,
        setup_name=os.path.basename(abs_cnc),
        is_turning=is_turning,
        check_probing=False,
    )
    wp = extract_workpiece_from_context(ctx, is_turning=is_turning)
    if wp is None:
        wp = _synthesize_fallback_workpiece_from_context(ctx, is_turning=is_turning)
    return CorpusFixture(
        name=os.path.basename(abs_cnc),
        source_path=abs_cnc,
        cnc_batches=[abs_cnc],
        is_turning=is_turning,
        unit=ctx.get("unit"),
        fallback_workpiece=wp,
        metadata={"capability_error": cap_err} if cap_err else {},
    )


def discover_fixtures(
    corpus_dir: Optional[str] = None,
    extra_paths: Sequence[str] = (),
    include_builtin_fixtures: bool = True,
    max_fixtures: Optional[int] = None,
) -> List[CorpusFixture]:
    """
    Discover harvested setup bundles (`setup_manifest.json`) and standalone `.cnc` files.
    Standalone `.cnc` files inside an already-discovered bundle directory are ignored.
    """
    manifests: List[str] = []
    standalone_cncs: List[str] = []

    search_roots: List[str] = []
    if corpus_dir and os.path.exists(corpus_dir):
        search_roots.append(os.path.abspath(corpus_dir))
    for ep in extra_paths:
        if ep and os.path.exists(ep):
            search_roots.append(os.path.abspath(ep))
    if include_builtin_fixtures:
        builtin_dir = os.path.join(REPO_ROOT, "tests", "fixtures")
        if os.path.isdir(builtin_dir):
            search_roots.append(builtin_dir)

    seen_roots: Set[str] = set()
    bundle_dirs: Set[str] = set()
    raw_cnc_candidates: List[str] = []

    for root in search_roots:
        if root in seen_roots:
            continue
        seen_roots.add(root)
        if os.path.isfile(root):
            if os.path.basename(root) == "setup_manifest.json":
                manifests.append(root)
                bundle_dirs.add(os.path.dirname(root))
            elif root.lower().endswith(".cnc"):
                raw_cnc_candidates.append(root)
            continue

        for dirpath, dirnames, filenames in os.walk(root):
            dirnames[:] = sorted(d for d in dirnames if not d.startswith(".") and d != "__pycache__")
            if "setup_manifest.json" in filenames:
                mpath = os.path.join(dirpath, "setup_manifest.json")
                manifests.append(mpath)
                bundle_dirs.add(os.path.abspath(dirpath))
                continue
            has_incomplete_harvest = (
                "prev_stock.stl" in filenames
                or "solid_stock.stl" in filenames
                or "post_setup_ipw.stl" in filenames
                or any(fn.startswith("batch_") and fn.endswith(".json") for fn in filenames)
            )
            if has_incomplete_harvest:
                continue
            for fn in sorted(filenames):
                if fn.lower().endswith(".cnc"):
                    raw_cnc_candidates.append(os.path.join(dirpath, fn))

    for cnc in raw_cnc_candidates:
        if os.path.abspath(os.path.dirname(cnc)) not in bundle_dirs:
            standalone_cncs.append(cnc)

    fixtures: List[CorpusFixture] = []
    for mp in sorted(manifests):
        if _manifest_bundle_is_unsupported(mp):
            continue
        fixtures.append(load_fixture_from_manifest(mp))

    if standalone_cncs:
        sdirs = get_post_search_dirs()
        probe_post = resolve_post_file("shopbot.cps", sdirs) or resolve_post_file("grbl.cps", sdirs)
        if not probe_post:
            raise FileNotFoundError("Could not find a probe post processor (shopbot.cps/grbl.cps) for standalone .cnc inspection")
        orch = PipelineOrchestrator()
        for cnc in sorted(standalone_cncs):
            fx = load_standalone_cnc_fixture(cnc, probe_post_path=probe_post, orchestrator=orch)
            if fx.metadata.get("capability_error"):
                continue
            fixtures.append(fx)

    if max_fixtures is not None and max_fixtures > 0:
        fixtures = fixtures[:max_fixtures]
    return fixtures


# ---------------------------------------------------------------------------
# Automated Oracles
# ---------------------------------------------------------------------------

_TIMESTAMP_DATE_RE = re.compile(r"\b(?:19|20)\d{2}[-/]\d{2}[-/]\d{2}\b")
_TIMESTAMP_TIME_RE = re.compile(r"\b\d{1,2}:\d{2}(?::\d{2})?\b")
_CANNED_CYCLE_RE = re.compile(r"\bG0*(?:32|33|73|74|76|8[1-9])\b", re.IGNORECASE)
_PURE_WRAPPER_COMMENT_RE = re.compile(
    r"^\s*(?:N\d+\s+)?(?:[;'(]\s*)?(?:UNIVERSAL WRAPPER:?\s+RESTORED \d+ G0 RAPID MOVES|RAPID RESTORED)\s*\)?\s*$",
    re.IGNORECASE,
)
_AXIS_TOKEN_RE = re.compile(r"([XYZABC])\s*([+-]?(?:\d+\.?\d*|\.\d+))", re.IGNORECASE)
_OPENSBP_MOVE_RE = re.compile(r"^\s*([MJ][23XYZ])\s*,(.*)$", re.IGNORECASE)
_OPENSBP_AXIS_MAP = {
    "X": ("X",),
    "Y": ("Y",),
    "Z": ("Z",),
    "2": ("X", "Y"),
    "3": ("X", "Y", "Z"),
}


def _normalize_post_basename(name_or_path: Optional[str]) -> str:
    if not name_or_path:
        return ""
    bname = os.path.basename(str(name_or_path).replace("\\", "/")).strip().lower()
    return bname[:-4] if bname.endswith(".cps") else bname


def _effective_post_properties(fixture: CorpusFixture, post_path: str) -> Optional[Dict[str, Any]]:
    """Return fixture post_properties only when testing its configured vendor post."""
    if not fixture.post_properties:
        return None
    if fixture.configured_post:
        if _normalize_post_basename(fixture.configured_post) != _normalize_post_basename(post_path):
            return None
    return dict(fixture.post_properties)


def normalize_nc_lines(text: str) -> List[str]:
    """
    Normalize NC program text into non-empty lines while masking wall-clock
    dates and times inside pure comment lines only.
    """
    out: List[str] = []
    for raw_ln in text.splitlines():
        ln = raw_ln.rstrip()
        if not ln.strip():
            continue
        body = re.sub(r"^N\d+\s+", "", ln.lstrip(), flags=re.IGNORECASE)
        is_pure_comment = (
            body.startswith((";", "'"))
            or (body.startswith("(") and body.endswith(")"))
        )
        if is_pure_comment:
            ln = _TIMESTAMP_DATE_RE.sub("<DATE>", ln)
            ln = _TIMESTAMP_TIME_RE.sub("<TIME>", ln)
        out.append(ln)
    return out


def _strip_comments_and_n(line: str) -> str:
    s = re.sub(r"\([^)]*\)", "", line)
    s = re.sub(r";.*$", "", s)
    s = re.sub(r"^'.*$", "", s)
    s = re.sub(r"^\s*N\d+\s+", "", s, flags=re.IGNORECASE)
    return s.strip()


def _extract_move_coordinates(lines: Sequence[str]) -> List[Tuple[Tuple[str, float], ...]]:
    """Extract ordered coordinate tuples from NC or OpenSBP move lines."""
    coords_list: List[Tuple[Tuple[str, float], ...]] = []
    axes_order = ("X", "Y", "Z")
    for raw in lines:
        code = _strip_comments_and_n(raw)
        if not code:
            continue
        m_sbp = _OPENSBP_MOVE_RE.match(code)
        if m_sbp:
            cmd = m_sbp.group(1).upper()
            axes_for_cmd = _OPENSBP_AXIS_MAP.get(cmd[1:], axes_order)
            parts = [p.strip() for p in m_sbp.group(2).split(",")]
            tuple_items: List[Tuple[str, float]] = []
            for idx, val_str in enumerate(parts[: len(axes_for_cmd)]):
                if val_str:
                    try:
                        tuple_items.append((axes_for_cmd[idx], round(float(val_str), 4)))
                    except ValueError:
                        pass
            if tuple_items:
                coords_list.append(tuple(tuple_items))
            continue

        matches = _AXIS_TOKEN_RE.findall(code)
        if matches:
            coords_list.append(tuple((ax.upper(), round(float(num), 4)) for ax, num in matches))
    return coords_list


def run_direct_post_with_shim(
    post_bin: str,
    base_post_path: str,
    cnc_path: str,
    out_nc_path: str,
    tmp_dir: str,
    program_name: str = "1001",
    timeout: float = 120.0,
) -> str:
    """Invoke Autodesk `post` directly on one `.cnc` with a minimal `programName` shim."""
    shim_path = os.path.join(tmp_dir, "direct_shim.cps")
    esc_post = base_post_path.replace("\\", "\\\\").replace('"', '\\"')
    esc_prog = str(program_name).replace("\\", "\\\\").replace('"', '\\"')
    shim_code = (
        f'include("{esc_post}");\n'
        'var _pcOrigOpen = (typeof onOpen === "function") ? onOpen : null;\n'
        "onOpen = function() {\n"
        f'  programName = "{esc_prog}";\n'
        "  if (_pcOrigOpen) { _pcOrigOpen(); }\n"
        "};\n"
    )
    with open(shim_path, "w", encoding="utf-8") as sf:
        sf.write(shim_code)

    res = subprocess.run(
        [post_bin, "--quiet", shim_path, cnc_path, out_nc_path],
        capture_output=True,
        text=True,
        timeout=timeout,
    )
    if res.returncode != 0:
        raise RuntimeError(
            f"Direct post binary exited with rc={res.returncode}: "
            f"{res.stderr.strip() or res.stdout.strip()}"
        )
    with open(out_nc_path, "r", encoding="utf-8", errors="replace") as f:
        return f.read()


def check_oracle1_direct_parity(
    direct_nc: str,
    pipeline_noprom_nc: str,
    max_diff_lines: int = 20,
) -> Tuple[bool, str]:
    """
    Oracle 1: Verify `PipelineOrchestrator.run(..., promote=False)` matches direct
    vendor post execution line-for-line (modulo comment wall-clock timestamps).
    """
    d_lines = normalize_nc_lines(direct_nc)
    p_lines = normalize_nc_lines(pipeline_noprom_nc)
    if d_lines == p_lines:
        return True, "exact match"
    diff = list(
        difflib.unified_diff(
            d_lines,
            p_lines,
            fromfile="direct_post",
            tofile="postclear_noprom",
            lineterm="",
            n=1,
        )
    )
    sample = "\n".join(diff[:max_diff_lines])
    return False, f"NC mismatch ({len(diff)} diff lines):\n{sample}"


def _all_sections_blocked_from_dumps(
    dump_files: Sequence[str],
    report_prom: Any,
    replay_diag: Optional[Dict[str, Any]] = None,
) -> bool:
    """Determine from section metadata whether every section is tilted, multi-axis, or frame-mismatched."""
    total_sections = 0
    blocked_sections = 0
    use_replay_diag = replay_diag is not None
    diag_blocked_sec_indices: Set[int] = set()
    if use_replay_diag and isinstance(replay_diag, dict):
        for s_idx in replay_diag.get("frameMismatchSections") or []:
            if isinstance(s_idx, int):
                diag_blocked_sec_indices.add(s_idx)
        for fm in replay_diag.get("frameMismatches") or []:
            if isinstance(fm, dict) and isinstance(fm.get("secIdx"), int):
                diag_blocked_sec_indices.add(fm["secIdx"])
        for ma in replay_diag.get("multiAxisSections") or []:
            if isinstance(ma, dict) and isinstance(ma.get("secIdx"), int):
                diag_blocked_sec_indices.add(ma["secIdx"])
    frame_mismatch_ops = set(getattr(report_prom, "frame_mismatch_ops", []) or [])
    for dp in dump_files:
        try:
            with open(dp, "r", encoding="utf-8") as df:
                ctx = json.load(df)
        except Exception:
            return False
        for raw_s in ctx.get("sections", []):
            sec_idx = total_sections
            total_sections += 1
            s_type = raw_s.get("type", 0)
            wp_f = (raw_s.get("workPlane") or {}).get("forward") or {}
            fx = float(wp_f.get("x", 0.0))
            fy = float(wp_f.get("y", 0.0))
            fz = float(wp_f.get("z", 1.0))
            is_t = (s_type != 1) and not (abs(fx) < 1e-5 and abs(fy) < 1e-5 and abs(fz - 1.0) < 1e-5)
            is_ma = bool(raw_s.get("isMultiAxis") or raw_s.get("has5DMoves"))
            if use_replay_diag:
                sec_blocked = is_t or is_ma or (sec_idx in diag_blocked_sec_indices)
            else:
                params = raw_s.get("params") or {}
                op_name = params.get("operation-comment") or f"Section_{raw_s.get('id', sec_idx)}"
                sec_blocked = is_t or is_ma or (op_name in frame_mismatch_ops)
            if sec_blocked:
                blocked_sections += 1
    return total_sections > 0 and blocked_sections >= total_sections


def check_oracle2_nc_diff(
    noprom_nc: str,
    prom_nc: str,
    report_prom: Any,
    section_count: Optional[int] = None,
    all_sections_blocked_override: Optional[bool] = None,
) -> Tuple[bool, str]:
    """
    Oracle 2a/b/c: Verify promotion fail-closed rules, canned-cycle preservation,
    and coordinate preservation between `promote=False` and `promote=True` NC outputs.
    """
    n_lines = normalize_nc_lines(noprom_nc)
    p_lines = normalize_nc_lines(prom_nc)

    if all_sections_blocked_override is not None:
        all_sections_blocked = bool(all_sections_blocked_override)
        fail_closed = bool(getattr(report_prom, "dog_leg_warning", None)) or all_sections_blocked
    else:
        blocked_ops = (
            set(getattr(report_prom, "tilted_ops", []) or [])
            | set(getattr(report_prom, "multi_axis_ops", []) or [])
            | set(getattr(report_prom, "frame_mismatch_ops", []) or [])
        )
        all_ops = list(getattr(report_prom, "operation_names", []) or [])
        all_sections_blocked = bool(all_ops) and all(op in blocked_ops for op in all_ops)
        if section_count is not None and section_count > 0 and len(blocked_ops) >= section_count and all_sections_blocked:
            fail_closed = True
        else:
            fail_closed = bool(getattr(report_prom, "dog_leg_warning", None)) or all_sections_blocked

    promoted_events = set(getattr(report_prom, "promoted_events", set()) or set())
    if fail_closed:
        if promoted_events:
            return False, f"Fail-closed violation: {len(promoted_events)} promoted events despite block"
        if n_lines != p_lines:
            return False, "Fail-closed violation: NC differs between promote=False and promote=True despite block"
        return True, "fail-closed verified (0 promotions)"

    if not promoted_events:
        if n_lines != p_lines:
            return False, "No promoted_events, yet promote=True altered NC output"
        return True, "no-op verified (0 promotions)"

    # Strip wrapper summary comments from promoted output before prefix/suffix trim & alignment
    p_lines = [ln for ln in p_lines if not _PURE_WRAPPER_COMMENT_RE.match(ln)]

    # Trim identical leading/trailing NC lines (where nl == pl is a guaranteed no-op)
    len_n, len_p = len(n_lines), len(p_lines)
    prefix = 0
    min_len = min(len_n, len_p)
    while prefix < min_len and n_lines[prefix] == p_lines[prefix]:
        prefix += 1
    suffix = 0
    while suffix < (min_len - prefix) and n_lines[len_n - 1 - suffix] == p_lines[len_p - 1 - suffix]:
        suffix += 1
    n_lines = n_lines[prefix : len_n - suffix]
    p_lines = p_lines[prefix : len_p - suffix]
    if not n_lines and not p_lines:
        return True, f"diff guard passed ({len(promoted_events)} promoted events)"

    key_cache: Dict[str, str] = {}

    def _align_key(ln: str) -> str:
        cached = key_cache.get(ln)
        if cached is not None:
            return cached
        stripped = _strip_comments_and_n(ln)
        if not _CANNED_CYCLE_RE.search(stripped):
            coords = _extract_move_coordinates([ln])
            if coords:
                res = f"MOVE:{coords[0]}"
                key_cache[ln] = res
                return res
        key_cache[ln] = ln
        return ln

    k0 = [_align_key(x) for x in n_lines]
    k1 = [_align_key(x) for x in p_lines]
    if k0 == k1:
        for nl, pl in zip(n_lines, p_lines):
            if nl == pl:
                continue
            if _CANNED_CYCLE_RE.search(_strip_comments_and_n(nl)):
                return False, f"Canned cycle or threading move altered by promotion: {nl}"
            if _extract_move_coordinates([nl]) != _extract_move_coordinates([pl]):
                return False, f"Coordinate mismatch in promoted line: before={nl!r} after={pl!r}"
        return True, f"diff guard passed ({len(promoted_events)} promoted events)"

    matcher = difflib.SequenceMatcher(
        None,
        k0,
        k1,
        autojunk=False,
    )
    for tag, i1, i2, j1, j2 in matcher.get_opcodes():
        if tag == "equal":
            for nl, pl in zip(n_lines[i1:i2], p_lines[j1:j2]):
                if nl == pl:
                    continue
                if _CANNED_CYCLE_RE.search(_strip_comments_and_n(nl)):
                    return False, f"Canned cycle or threading move altered by promotion: {nl}"
                if _extract_move_coordinates([nl]) != _extract_move_coordinates([pl]):
                    return False, f"Coordinate mismatch in promoted line: before={nl!r} after={pl!r}"
            continue
        del_slice = n_lines[i1:i2]
        ins_slice = [ln for ln in p_lines[j1:j2] if not _PURE_WRAPPER_COMMENT_RE.match(ln)]
        if tag == "delete":
            return False, f"Unexpected deleted NC lines on promote=True: {del_slice[:3]}"
        if tag == "insert":
            if ins_slice:
                return False, f"Unexpected non-wrapper inserted NC lines: {ins_slice[:3]}"
            continue
        for dl in del_slice:
            if _CANNED_CYCLE_RE.search(_strip_comments_and_n(dl)):
                return False, f"Canned cycle or threading move altered by promotion: {dl}"
        del_coords = _extract_move_coordinates(del_slice)
        ins_coords = _extract_move_coordinates(ins_slice)
        if del_coords != ins_coords:
            return (
                False,
                f"Coordinate mismatch in promoted block: before={del_slice[:3]} after={ins_slice[:3]}",
            )

    return True, f"diff guard passed ({len(promoted_events)} promoted events)"


def _new_verifier(is_turning: bool) -> Any:
    if is_turning:
        from geometry.lathe_engine import Lathe2DIPWVerifier
        return Lathe2DIPWVerifier(tool_scale_to_mm=1.0)
    from geometry.heightfield_engine import MillHeightfieldVerifier
    return MillHeightfieldVerifier()


class _NoOpVerifier:
    """Fast verifier stub for `promote=False` parity runs where IPW output is unused."""

    def __init__(self) -> None:
        self.holder_unchecked_ops: Set[str] = set()

    def verify_toolpath(
        self,
        segments: Sequence[Any],
        tools_by_op: Dict[str, Any],
        workpiece: Any,
    ) -> VerificationReport:
        seen_ops: Set[str] = set()
        op_names: List[str] = []
        for seg in segments:
            op_id = getattr(seg, "op_id", "")
            if op_id and op_id not in seen_ops:
                seen_ops.add(op_id)
                op_names.append(op_id)
        return VerificationReport(
            total_machining_time_sec=0.0,
            total_volume_removed_mm3=0.0,
            collisions=[],
            promoted_events=set(),
            operation_names=op_names,
        )


def _digest_verification_inputs(
    is_turning: bool,
    segments: Sequence[Any],
    tools_by_op: Dict[Any, Any],
    workpiece: Any,
) -> str:
    """Build a deterministic Blake2b digest of `(workpiece, tools_by_op, segments)`."""
    h = hashlib.blake2b(digest_size=32)
    h.update(f"turn={int(bool(is_turning))}|".encode("utf-8"))

    if workpiece is not None:
        wo = getattr(workpiece, "work_offset", None)
        if wo is not None:
            h.update(f"wo=({wo.x:.5f},{wo.y:.5f},{wo.z:.5f})|".encode("utf-8"))
        h.update(f"stl_derived={int(bool(getattr(workpiece, 'is_stl_derived', False)))}|".encode("utf-8"))
        for fx in getattr(workpiece, "fixtures", []) or []:
            h.update(
                f"fx=({fx.z_min:.4f},{fx.z_max:.4f},{fx.outer_radius:.4f},{fx.clamping_radius:.4f})|".encode("utf-8")
            )
        st = getattr(workpiece, "stock", None)
        if st is not None:
            st_cls = type(st).__name__
            h.update(f"st={st_cls}:".encode("utf-8"))
            if isinstance(st, SolidMeshStock3D):
                h.update(
                    f"({st.x_min:.4f},{st.x_max:.4f},{st.y_min:.4f},{st.y_max:.4f},{st.z_min:.4f},{st.z_max:.4f})".encode("utf-8")
                )
                verts = getattr(st, "vertices", None)
                indices = getattr(st, "indices", None)
                if verts is not None:
                    h.update(f"v{getattr(verts, 'shape', '')}".encode("utf-8"))
                    if hasattr(verts, "tobytes"):
                        h.update(verts.tobytes())
                if indices is not None:
                    h.update(f"i{getattr(indices, 'shape', '')}".encode("utf-8"))
                    if hasattr(indices, "tobytes"):
                        h.update(indices.tobytes())
            elif isinstance(st, BoxStock3D):
                h.update(
                    f"({st.x_min:.4f},{st.x_max:.4f},{st.y_min:.4f},{st.y_max:.4f},{st.z_min:.4f},{st.z_max:.4f})".encode("utf-8")
                )
            elif isinstance(st, AxisymmetricStock2D):
                h.update(
                    f"({st.z_min:.4f},{st.z_max:.4f},{st.outer_radius:.4f},{st.inner_radius:.4f})".encode("utf-8")
                )
                poly = getattr(st, "polygon", None)
                if poly is not None and hasattr(poly, "wkt"):
                    h.update(poly.wkt.encode("utf-8"))
            else:
                h.update(repr(st).encode("utf-8"))

    for op_key, tool in sorted((tools_by_op or {}).items(), key=lambda kv: str(kv[0])):
        g = getattr(tool, "geometry", None)
        g_cls = type(g).__name__ if g is not None else "None"
        h.update(f"|t={op_key}:{getattr(tool, 'tool_id', '')}:{getattr(tool, 'name', '')}:{g_cls}:".encode("utf-8"))
        if g is not None:
            for attr in (
                "diameter",
                "corner_radius",
                "flute_length",
                "overall_length",
                "taper_angle_deg",
                "holder_diameter",
                "holder_length",
                "body_length",
                "shoulder_length",
                "shaft_diameter",
                "holder_sections",
                "insert_shape",
                "nose_radius",
                "hand",
                "is_boring",
            ):
                if hasattr(g, attr):
                    h.update(f"{attr}={getattr(g, attr)!r};".encode("utf-8"))
            q = getattr(g, "quadrant", None)
            if q is not None:
                h.update(f"quad={getattr(q, 'value', q)!r};".encode("utf-8"))
            for poly_attr in ("cutter_polygon", "holder_polygon"):
                poly = getattr(g, poly_attr, None)
                if poly is not None and hasattr(poly, "bounds"):
                    h.update(f"{poly_attr}={tuple(round(v, 5) for v in poly.bounds)!r};".encode("utf-8"))

    h.update(f"|nseg={len(segments)}|".encode("utf-8"))
    for seg in segments:
        mot = getattr(seg, "motion", None)
        mot_v = getattr(mot, "value", mot)
        st_pt = seg.start
        en_pt = seg.end
        ac = getattr(seg, "arc_center", None)
        ac_s = f"({ac.x:.5f},{ac.y:.5f},{ac.z:.5f})" if ac is not None else "N"
        meta = getattr(seg, "metadata", None) or {}
        h.update(
            f"{seg.sec_id}|{seg.op_id}|{mot_v}|"
            f"{st_pt.x:.5f},{st_pt.y:.5f},{st_pt.z:.5f}->"
            f"{en_pt.x:.5f},{en_pt.y:.5f},{en_pt.z:.5f}|"
            f"{float(seg.feedrate_mmpm):.4f}|{float(seg.spindle_rpm):.2f}|"
            f"{float(seg.duration_sec):.6f}|{int(bool(seg.is_arc))}|{seg.arc_plane}|{ac_s}|"
            f"{meta.get('sec_idx')},{meta.get('event_idx')},"
            f"{int(bool(meta.get('tilted_section')))},{int(bool(meta.get('multi_axis')))};".encode("utf-8")
        )
    return h.hexdigest()


class _MemoizedVerifier:
    """
    Per-fixture verification cache that computes the IPW verification once per unique
    `(workpiece, tools_by_op, segments)` signature and returns deep-copied reports so
    per-post mutations in `PipelineOrchestrator.run` stay completely isolated.
    """

    def __init__(self, is_turning: bool) -> None:
        self.is_turning = bool(is_turning)
        self.holder_unchecked_ops: Set[str] = set()
        self._cache: Dict[str, Tuple[VerificationReport, Set[str]]] = {}
        self.cache_hits = 0
        self.cache_misses = 0

    def verify_toolpath(
        self,
        segments: Sequence[Any],
        tools_by_op: Dict[Any, Any],
        workpiece: Any,
    ) -> VerificationReport:
        key = _digest_verification_inputs(self.is_turning, segments, tools_by_op, workpiece)
        cached = self._cache.get(key)
        if cached is not None:
            self.cache_hits += 1
            cached_rep, cached_unchecked = cached
            self.holder_unchecked_ops = set(cached_unchecked)
            return copy.deepcopy(cached_rep)

        self.cache_misses += 1
        real_verifier = _new_verifier(self.is_turning)
        rep = real_verifier.verify_toolpath(segments, tools_by_op, workpiece)
        unchecked = set(
            getattr(real_verifier, "holder_unchecked_ops", None)
            or getattr(rep, "holder_unchecked_ops", None)
            or set()
        )
        self._cache[key] = (copy.deepcopy(rep), set(unchecked))
        self.holder_unchecked_ops = set(unchecked)
        return copy.deepcopy(rep)


def _materialize_workpiece(
    fixture: CorpusFixture,
    ctx: Dict[str, Any],
) -> WorkpieceDefinition:
    if fixture.explicit_workpiece is not None:
        return fixture.explicit_workpiece
    wp = extract_workpiece_from_context(
        ctx,
        fallback_workpiece=fixture.fallback_workpiece,
        is_turning=fixture.is_turning,
    )
    if wp is not None and wp.stock is not None:
        return wp
    return _synthesize_fallback_workpiece_from_context(ctx, is_turning=fixture.is_turning)


def check_oracle2_conservation(
    combined_ctx: Dict[str, Any],
    report_prom: Any,
    workpiece: WorkpieceDefinition,
    is_turning: bool,
    min_promote_len_mm: float = 1.5,
) -> Tuple[bool, str, Optional[Any]]:
    """
    Oracle 2d: Re-simulate promoted events explicitly converted from `onLinear`
    to `onRapid`, verifying identical material volume removed and zero
    `RAPID_INTO_STOCK` collisions on promoted moves.
    """
    tools_by_op = extract_tools_from_context(combined_ctx, is_turning=is_turning)
    colliding_events = {
        (c.sec_idx, c.event_idx)
        for c in getattr(report_prom, "collisions", [])
        if c.sec_idx is not None and c.event_idx is not None
    }
    safe_promoted = {ev for ev in (getattr(report_prom, "promoted_events", set()) or set()) if ev not in colliding_events}

    ctx_mut = copy.deepcopy(combined_ctx)
    if safe_promoted:
        apply_promotions_to_context(ctx_mut, safe_promoted)
        for sec in ctx_mut.get("sections", []):
            for ev in sec.get("events", []):
                if ev.get("promote") and ev.get("type") == "onLinear":
                    ev["type"] = "onRapid"

    re_segments = parse_context_dump(ctx_mut)
    re_verifier = _new_verifier(is_turning)
    re_report = re_verifier.verify_toolpath(re_segments, tools_by_op, workpiece)
    if re_report.promoted_events:
        re_report.promoted_events = filter_promoted_events_by_length(
            re_report.promoted_events, re_segments, min_len_mm=min_promote_len_mm
        )

    vol_before = float(getattr(report_prom, "total_volume_removed_mm3", 0.0))
    vol_after = float(re_report.total_volume_removed_mm3)
    vol_tol = max(1e-2, 1e-4 * abs(vol_before))
    if abs(vol_after - vol_before) > vol_tol:
        return (
            False,
            f"Post-promotion volume drifted: before={vol_before:.3f} mm^3 vs after={vol_after:.3f} mm^3",
            re_verifier,
        )

    bad_promoted_cols = [
        c for c in re_report.collisions
        if c.kind == CollisionKind.RAPID_INTO_STOCK
        and (c.sec_idx, c.event_idx) in safe_promoted
    ]
    if bad_promoted_cols:
        sample = bad_promoted_cols[0]
        return (
            False,
            f"Promoted move caused {len(bad_promoted_cols)} RAPID_INTO_STOCK collision(s) (first: {sample.description})",
            re_verifier,
        )

    return True, f"conservation verified (vol={vol_before:.2f} mm^3, promoted={len(safe_promoted)})", re_verifier


# ---------------------------------------------------------------------------
# Oracle 3: Metamorphic Transform & Unit Invariance
# ---------------------------------------------------------------------------

_SVG_TOKEN_RE = re.compile(r"[MLAZ]|[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?")


def _scale_svg_path(svg_path: str, factor: float) -> str:
    """Scale coordinate and radius operands inside an Autodesk CAM SVG path string."""
    if not svg_path:
        return svg_path
    tokens = _SVG_TOKEN_RE.findall(svg_path)
    out: List[str] = []
    i = 0
    while i < len(tokens):
        tok = tokens[i]
        if tok in ("M", "L") and i + 2 < len(tokens):
            out.append(tok)
            out.append(f"{float(tokens[i + 1]) * factor:.9g}")
            out.append(f"{float(tokens[i + 2]) * factor:.9g}")
            i += 3
        elif tok == "A" and i + 7 < len(tokens):
            out.append("A")
            out.append(f"{float(tokens[i + 1]) * factor:.9g}")
            out.append(f"{float(tokens[i + 2]) * factor:.9g}")
            out.append(tokens[i + 3])
            out.append(tokens[i + 4])
            out.append(tokens[i + 5])
            out.append(f"{float(tokens[i + 6]) * factor:.9g}")
            out.append(f"{float(tokens[i + 7]) * factor:.9g}")
            i += 8
        else:
            out.append(tok)
            i += 1
    return " ".join(out)


def _transform_context_unit_flip(ctx: Dict[str, Any]) -> Dict[str, Any]:
    """Return a deep copy of `ctx` converted between IN (unit=0) and MM (unit=1)."""
    out = copy.deepcopy(ctx)
    orig_unit = int(out.get("unit", 0))
    new_unit = 1 if orig_unit == 0 else 0
    k = 25.4 if orig_unit == 0 else (1.0 / 25.4)
    out["unit"] = new_unit

    for wp_key in ("workpiece", "workpieceHint"):
        wp = out.get(wp_key)
        if isinstance(wp, dict):
            for bound_key in ("lower", "upper"):
                pt = wp.get(bound_key)
                if isinstance(pt, dict):
                    for ax in ("x", "y", "z"):
                        if ax in pt and pt[ax] is not None:
                            pt[ax] = float(pt[ax]) * k

    g_params = out.get("globalParams")
    if isinstance(g_params, dict):
        for gk in ("stock-diameter-inner", "job:stockDiameterInner"):
            if gk in g_params and g_params[gk]:
                try:
                    g_params[gk] = float(g_params[gk]) * k
                except (TypeError, ValueError):
                    pass

    coord_keys = ("sx", "sy", "sz", "x", "y", "z", "cx", "cy", "cz", "feed", "radius")
    param_len_keys = (
        "operation:zClearance",
        "operation:clearanceHeight_value",
        "operation:zRetract",
        "operation:retractHeight_value",
        "clearance",
        "operation:tool_surfaceSpeed",
    )
    tool_len_keys = (
        "diameter", "cornerRadius", "corner_radius", "noseRadius",
        "fluteLength", "flute_length", "overallLength", "overall_length",
        "bodyLength", "body_length", "shoulderLength", "shoulder_length",
        "shaftDiameter", "shaft_diameter", "holderDiameter", "holder_diameter",
        "holderLength", "holder_length", "surfaceSpeed", "_surfaceSpeed",
    )

    for sec in out.get("sections", []):
        wo = sec.get("workOrigin")
        if isinstance(wo, dict):
            for ax in ("x", "y", "z"):
                if ax in wo and wo[ax] is not None:
                    wo[ax] = float(wo[ax]) * k

        params = sec.get("params")
        if isinstance(params, dict):
            for pk in param_len_keys:
                if pk in params and params[pk] is not None:
                    try:
                        params[pk] = float(params[pk]) * k
                    except (TypeError, ValueError):
                        pass

        tool = sec.get("tool")
        if isinstance(tool, dict):
            for tk in tool_len_keys:
                if tk in tool and tool[tk] is not None:
                    try:
                        tool[tk] = float(tool[tk]) * k
                    except (TypeError, ValueError):
                        pass
            for sec_list_key in ("_holderSections", "holder_sections"):
                s_list = tool.get(sec_list_key)
                if isinstance(s_list, list):
                    scaled_list = []
                    for item in s_list:
                        if isinstance(item, dict):
                            item_c = dict(item)
                            if "length" in item_c and item_c["length"] is not None:
                                item_c["length"] = float(item_c["length"]) * k
                            if "diameter" in item_c and item_c["diameter"] is not None:
                                item_c["diameter"] = float(item_c["diameter"]) * k
                            scaled_list.append(item_c)
                        elif isinstance(item, list) and len(item) >= 2:
                            scaled_list.append([float(item[0]) * k, float(item[1]) * k, *item[2:]])
                        else:
                            scaled_list.append(item)
                    tool[sec_list_key] = scaled_list
            for sh_key in ("_shaftSections", "shaft_sections"):
                sh_list = tool.get(sh_key)
                if isinstance(sh_list, list):
                    scaled_sh = []
                    for item in sh_list:
                        if isinstance(item, dict):
                            item_c = dict(item)
                            if "length" in item_c and item_c["length"] is not None:
                                item_c["length"] = float(item_c["length"]) * k
                            if "diameter" in item_c and item_c["diameter"] is not None:
                                item_c["diameter"] = float(item_c["diameter"]) * k
                            scaled_sh.append(item_c)
                        elif isinstance(item, list) and len(item) >= 2:
                            scaled_sh.append([float(item[0]) * k, float(item[1]) * k, *item[2:]])
                        else:
                            scaled_sh.append(item)
                    tool[sh_key] = scaled_sh
            comp = tool.get("_compDisplacement")
            if isinstance(comp, dict):
                for ax in ("x", "y"):
                    if ax in comp and comp[ax] is not None:
                        comp[ax] = float(comp[ax]) * k
            for svg_key in ("_cutterSVG", "_holderSVG"):
                if isinstance(tool.get(svg_key), str) and tool[svg_key]:
                    tool[svg_key] = _scale_svg_path(tool[svg_key], k)

        for ev in sec.get("events", []):
            for ck in coord_keys:
                if ck in ev and ev[ck] is not None:
                    ev[ck] = float(ev[ck]) * k
            if isinstance(ev.get("linearizedMoves"), list):
                for lm in ev["linearizedMoves"]:
                    for ck in coord_keys:
                        if ck in lm and lm[ck] is not None:
                            lm[ck] = float(lm[ck]) * k
            for exp_key in ("reposMoves", "afterReposMoves", "expandedMoves"):
                if isinstance(ev.get(exp_key), list):
                    for em in ev[exp_key]:
                        for ck in coord_keys:
                            if ck in em and em[ck] is not None:
                                em[ck] = float(em[ck]) * k

    return out


def _transform_180_z(
    ctx: Dict[str, Any],
    workpiece: WorkpieceDefinition,
) -> Tuple[Dict[str, Any], WorkpieceDefinition]:
    """Rotate a milling context and stock 180 degrees around the setup Z axis."""
    out = copy.deepcopy(ctx)
    for wp_key in ("workpiece", "workpieceHint"):
        wp_box = out.get(wp_key)
        if isinstance(wp_box, dict):
            lo = wp_box.get("lower")
            hi = wp_box.get("upper")
            if isinstance(lo, dict) and isinstance(hi, dict):
                if "x" in lo and "x" in hi and lo["x"] is not None and hi["x"] is not None:
                    nx0, nx1 = -float(hi["x"]), -float(lo["x"])
                    lo["x"], hi["x"] = nx0, nx1
                if "y" in lo and "y" in hi and lo["y"] is not None and hi["y"] is not None:
                    ny0, ny1 = -float(hi["y"]), -float(lo["y"])
                    lo["y"], hi["y"] = ny0, ny1
    for sec in out.get("sections", []):
        wp = sec.get("workPlane")
        has_wp = isinstance(wp, dict)
        if has_wp:
            for vec_key in ("right", "up", "forward"):
                v = wp.get(vec_key)
                if isinstance(v, dict):
                    v["x"] = -float(v.get("x", 0.0))
                    v["y"] = -float(v.get("y", 0.0))
            wo = sec.get("workOrigin")
            if isinstance(wo, dict):
                wo["x"] = -float(wo.get("x", 0.0))
                wo["y"] = -float(wo.get("y", 0.0))
        else:
            def _negate_xy_dict(d: Dict[str, Any]) -> None:
                for k in ("sx", "x", "cx"):
                    if k in d and d[k] is not None:
                        d[k] = -float(d[k])
                for k in ("sy", "y", "cy"):
                    if k in d and d[k] is not None:
                        d[k] = -float(d[k])

            for ev in sec.get("events", []):
                _negate_xy_dict(ev)
                for sub_key in ("linearizedMoves", "reposMoves", "afterReposMoves", "expandedMoves"):
                    for sub_ev in ev.get(sub_key) or []:
                        if isinstance(sub_ev, dict):
                            _negate_xy_dict(sub_ev)

    st = workpiece.stock
    if isinstance(st, BoxStock3D):
        rot_stock = BoxStock3D(
            x_min=-st.x_max,
            x_max=-st.x_min,
            y_min=-st.y_max,
            y_max=-st.y_min,
            z_min=st.z_min,
            z_max=st.z_max,
        )
        rot_wp = WorkpieceDefinition(stock=rot_stock, fixtures=list(workpiece.fixtures))
    elif isinstance(st, SolidMeshStock3D):
        verts = st.vertices.copy()
        verts[:, 0] = -verts[:, 0]
        verts[:, 1] = -verts[:, 1]
        rot_stock = SolidMeshStock3D.from_mesh(verts, st.indices)
        rot_wp = WorkpieceDefinition(stock=rot_stock, fixtures=list(workpiece.fixtures), is_stl_derived=True)
    else:
        rot_wp = workpiece
    return out, rot_wp


def _collision_signature(collisions: Sequence[Any]) -> Dict[str, int]:
    counts: Dict[str, int] = {}
    for c in collisions:
        k = c.kind.name if hasattr(c.kind, "name") else str(c.kind)
        counts[k] = counts.get(k, 0) + 1
    return counts


def _block_tilted_or_multiaxis_promotions(
    combined_ctx: Dict[str, Any],
    segments: Sequence[Any],
    promoted_events: Set[Tuple[int, int]],
) -> Set[Tuple[int, int]]:
    blocked: Set[int] = set()
    for sec_idx, sec in enumerate(combined_ctx.get("sections", [])):
        sec_type = sec.get("type", 0)
        wp_f = (sec.get("workPlane") or {}).get("forward") or {}
        fx = float(wp_f.get("x", 0.0))
        fy = float(wp_f.get("y", 0.0))
        fz = float(wp_f.get("z", 1.0))
        if sec_type != 1 and not (abs(fx) < 1e-5 and abs(fy) < 1e-5 and abs(fz - 1.0) < 1e-5):
            blocked.add(sec_idx)
        if sec.get("isMultiAxis") or sec.get("has5DMoves"):
            blocked.add(sec_idx)
    for seg in segments:
        s_meta = getattr(seg, "metadata", None) or {}
        if s_meta.get("tilted_section") or s_meta.get("multi_axis"):
            blocked.add(s_meta.get("sec_idx", seg.sec_id))
    return {ev for ev in promoted_events if ev[0] not in blocked}


def check_oracle3_metamorphic(
    combined_ctx: Dict[str, Any],
    workpiece: WorkpieceDefinition,
    base_report: Any,
    is_turning: bool,
    min_promote_len_mm: float = 1.5,
) -> Tuple[bool, str]:
    """
    Oracle 3: Verify simulation invariance under unit flip (`MM <-> IN`) and, for
    planar 3-axis milling setups, under 180-degree Z rotation.
    """
    base_vol = float(getattr(base_report, "total_volume_removed_mm3", 0.0))
    base_cols = _collision_signature(getattr(base_report, "collisions", []))
    base_prom = set(getattr(base_report, "promoted_events", set()) or set())

    # 3a. MM <-> IN unit flip invariance
    ctx_unit = _transform_context_unit_flip(combined_ctx)
    tools_unit = extract_tools_from_context(ctx_unit, is_turning=is_turning)
    segs_unit = parse_context_dump(ctx_unit)
    v_unit = _new_verifier(is_turning)
    rep_unit = v_unit.verify_toolpath(segs_unit, tools_unit, workpiece)
    if rep_unit.promoted_events:
        rep_unit.promoted_events = filter_promoted_events_by_length(
            rep_unit.promoted_events, segs_unit, min_len_mm=min_promote_len_mm
        )
    rep_unit.promoted_events = _block_tilted_or_multiaxis_promotions(ctx_unit, segs_unit, rep_unit.promoted_events)

    unit_vol_tol = max(0.5, 0.002 * abs(base_vol))
    if abs(rep_unit.total_volume_removed_mm3 - base_vol) > unit_vol_tol:
        return (
            False,
            f"Unit-flip volume mismatch: base={base_vol:.3f} vs flipped={rep_unit.total_volume_removed_mm3:.3f}",
        )
    unit_cols = _collision_signature(rep_unit.collisions)
    if unit_cols != base_cols:
        return False, f"Unit-flip collision mismatch: base={base_cols} vs flipped={unit_cols}"
    if rep_unit.promoted_events != base_prom:
        return (
            False,
            f"Unit-flip promoted_events mismatch: base={len(base_prom)} vs flipped={len(rep_unit.promoted_events)}",
        )

    # 3b. 180-degree Z-rotation invariance for Z-aligned 3-axis milling
    has_tilted_or_ma = bool(getattr(base_report, "tilted_ops", []) or getattr(base_report, "multi_axis_ops", []))
    if not is_turning and not has_tilted_or_ma and isinstance(workpiece.stock, (BoxStock3D, SolidMeshStock3D)):
        segs_base = parse_context_dump(combined_ctx)
        ctx_rot, wp_rot = _transform_180_z(combined_ctx, workpiece)
        tools_rot = extract_tools_from_context(ctx_rot, is_turning=False)
        segs_rot = parse_context_dump(ctx_rot)
        if len(segs_rot) != len(segs_base):
            return (
                False,
                f"180-deg Z-rot segment count mismatch: base={len(segs_base)} vs rotated={len(segs_rot)}",
            )
        for idx, (sb, sr) in enumerate(zip(segs_base, segs_rot)):
            if (sb.is_arc, sb.arc_plane) != (sr.is_arc, sr.arc_plane):
                return (
                    False,
                    f"180-deg Z-rot segment {idx} arc mismatch: base={(sb.is_arc, sb.arc_plane)} vs rotated={(sr.is_arc, sr.arc_plane)}",
                )

        v_rot = _new_verifier(False)
        rep_rot = v_rot.verify_toolpath(segs_rot, tools_rot, wp_rot)
        if rep_rot.promoted_events:
            rep_rot.promoted_events = filter_promoted_events_by_length(
                rep_rot.promoted_events, segs_rot, min_len_mm=min_promote_len_mm
            )
        rep_rot.promoted_events = _block_tilted_or_multiaxis_promotions(ctx_rot, segs_rot, rep_rot.promoted_events)

        is_mesh = isinstance(workpiece.stock, SolidMeshStock3D)
        rot_vol_tol = max(5.0, 0.03 * abs(base_vol)) if is_mesh else max(2.0, 0.01 * abs(base_vol))
        if abs(rep_rot.total_volume_removed_mm3 - base_vol) > rot_vol_tol:
            return (
                False,
                f"180-deg Z-rot volume mismatch: base={base_vol:.3f} vs rotated={rep_rot.total_volume_removed_mm3:.3f}",
            )
        rot_cols = _collision_signature(rep_rot.collisions)
        if rot_cols != base_cols:
            return False, f"180-deg Z-rot collision mismatch: base={base_cols} vs rotated={rot_cols}"
        prom_cnt_tol = max(3, int(math.ceil(0.10 * len(base_prom)))) if is_mesh else max(2, int(math.ceil(0.03 * len(base_prom))))
        if abs(len(rep_rot.promoted_events) - len(base_prom)) > prom_cnt_tol:
            return (
                False,
                f"180-deg Z-rot promoted count mismatch: base={len(base_prom)} vs rotated={len(rep_rot.promoted_events)} (tol={prom_cnt_tol})",
            )

    return True, "metamorphic unit + Z-rotation invariance verified"


def check_oracle4_ipw_diagnostic(
    verifier: Any,
    post_setup_ipw_stl: Optional[str],
    is_turning: bool,
) -> Optional[Dict[str, float]]:
    """
    Oracle 4 (informational): Compare final carved milling heightfield `grid.H`
    against a rasterized post-setup IPW STL mesh when present.
    """
    if is_turning or not post_setup_ipw_stl or not os.path.isfile(post_setup_ipw_stl):
        return None
    grid = getattr(verifier, "grid", None)
    if grid is None or getattr(grid, "H", None) is None or grid.H.size == 0:
        return None
    try:
        import numpy as np
        from geometry.heightfield_engine import HeightfieldGrid
        from geometry.solid_mesh import rasterize_triangles_to_grid

        verts, indices = load_stl_mesh(post_setup_ipw_stl, is_inch=False)
        ref_grid = HeightfieldGrid(cell_size_mm=grid.cell_size_mm, pad_mm=grid.pad_mm)
        ref_grid.stock_x_min = grid.stock_x_min
        ref_grid.stock_x_max = grid.stock_x_max
        ref_grid.stock_y_min = grid.stock_y_min
        ref_grid.stock_y_max = grid.stock_y_max
        ref_grid.stock_z_min = grid.stock_z_min
        ref_grid.stock_z_max = grid.stock_z_max
        ref_grid.grid_x_min = grid.grid_x_min
        ref_grid.grid_x_max = grid.grid_x_max
        ref_grid.grid_y_min = grid.grid_y_min
        ref_grid.grid_y_max = grid.grid_y_max
        ref_grid.z_floor = grid.z_floor
        ref_grid.nx = grid.nx
        ref_grid.ny = grid.ny
        ref_grid.H = np.full((grid.ny, grid.nx), grid.z_floor, dtype=np.float32)
        rasterize_triangles_to_grid(ref_grid, verts, indices, tol_pad_z_mm=0.0)

        xs = grid.grid_x_min + (np.arange(grid.nx, dtype=np.float32) + 0.5) * grid.cell_size_mm
        ys = grid.grid_y_min + (np.arange(grid.ny, dtype=np.float32) + 0.5) * grid.cell_size_mm
        col_mask = (xs >= grid.stock_x_min + grid.cell_size_mm) & (xs <= grid.stock_x_max - grid.cell_size_mm)
        row_mask = (ys >= grid.stock_y_min + grid.cell_size_mm) & (ys <= grid.stock_y_max - grid.cell_size_mm)
        inner_mask = np.outer(row_mask, col_mask)
        if not np.any(inner_mask):
            return None
        diff = np.abs(grid.H[inner_mask] - ref_grid.H[inner_mask])
        return {
            "p95_z_diff_mm": float(np.percentile(diff, 95)),
            "max_z_diff_mm": float(np.max(diff)),
        }
    except Exception:
        return None


# ---------------------------------------------------------------------------
# Matrix Execution & Reporting
# ---------------------------------------------------------------------------


def _evaluate_fixture_metamorphic_and_conservation(
    fixture: CorpusFixture,
    probe_post_path: str,
    run_metamorphic: bool = True,
    memo_verifier: Optional[_MemoizedVerifier] = None,
) -> Dict[str, Any]:
    """Run fixture-level Oracle 2d (conservation) + Oracle 3 (metamorphic) + Oracle 4."""
    orch = PipelineOrchestrator()
    out: Dict[str, Any] = {
        "fixture": fixture.name,
        "oracle2_conservation_ok": True,
        "oracle2_conservation_detail": "",
        "oracle3_ok": True if not run_metamorphic else False,
        "oracle3_detail": "skipped" if not run_metamorphic else "",
        "oracle4_diagnostic": None,
        "volume_mm3": 0.0,
        "promoted_count": 0,
    }
    try:
        with tempfile.TemporaryDirectory() as td:
            dump_files: List[str] = []
            for idx, cnc_p in enumerate(fixture.cnc_batches):
                dp = os.path.join(td, f"batch_{idx}.json")
                orch.dump_intermediate_to_json(cnc_p, probe_post_path, dp)
                dump_files.append(dp)
            combined_ctx = merge_context_dumps(dump_files)
            eff_props = _effective_post_properties(fixture, probe_post_path)
            if eff_props:
                combined_ctx["postProperties"] = dict(eff_props)

            wp = _materialize_workpiece(fixture, combined_ctx)
            rep_prom, _ = orch.run(
                context_dump_paths=dump_files,
                base_post_path=probe_post_path,
                bootstrap_cnc_path=fixture.cnc_batches[0],
                workpiece=fixture.explicit_workpiece,
                fallback_workpiece=wp if fixture.explicit_workpiece is None else None,
                final_cnc_path=os.path.join(td, "fixture_check.nc"),
                verifier=memo_verifier,
                promote=True,
                program_name="1001",
                post_properties=eff_props or None,
                is_turning=fixture.is_turning,
            )
            out["volume_mm3"] = float(rep_prom.total_volume_removed_mm3)
            out["promoted_count"] = len(rep_prom.promoted_events)

            ok2d, det2d, re_verifier = check_oracle2_conservation(
                combined_ctx=combined_ctx,
                report_prom=rep_prom,
                workpiece=wp,
                is_turning=fixture.is_turning,
                min_promote_len_mm=orch.min_promote_len_mm,
            )
            out["oracle2_conservation_ok"] = ok2d
            out["oracle2_conservation_detail"] = det2d
            out["oracle4_diagnostic"] = check_oracle4_ipw_diagnostic(
                re_verifier, fixture.post_setup_ipw_stl, fixture.is_turning
            )

            if run_metamorphic:
                tools = extract_tools_from_context(combined_ctx, is_turning=fixture.is_turning)
                segs = parse_context_dump(combined_ctx)
                base_verifier = memo_verifier if memo_verifier is not None else _new_verifier(fixture.is_turning)
                base_rep = base_verifier.verify_toolpath(segs, tools, wp)
                if base_rep.promoted_events:
                    base_rep.promoted_events = filter_promoted_events_by_length(
                        base_rep.promoted_events, segs, min_len_mm=orch.min_promote_len_mm
                    )
                base_rep.promoted_events = _block_tilted_or_multiaxis_promotions(
                    combined_ctx, segs, base_rep.promoted_events
                )
                ok3, det3 = check_oracle3_metamorphic(
                    combined_ctx=combined_ctx,
                    workpiece=wp,
                    base_report=base_rep,
                    is_turning=fixture.is_turning,
                    min_promote_len_mm=orch.min_promote_len_mm,
                )
                out["oracle3_ok"] = ok3
                out["oracle3_detail"] = det3
    except Exception as exc:
        out["oracle2_conservation_ok"] = False
        out["oracle2_conservation_detail"] = f"Exception: {exc}\n{traceback.format_exc()}"
        if run_metamorphic:
            out["oracle3_ok"] = False
            out["oracle3_detail"] = f"Exception: {exc}"
    return out


def _is_natively_unsupported_by_post(
    post_bin: str,
    post_path: str,
    cnc_batches: Sequence[str],
    tmp_dir: str,
    program_name: str = "1001",
    timeout: float = 120.0,
) -> Optional[str]:
    """
    Probe whether Autodesk's native `post` binary rejects this `.cnc` directly with
    `base_post_path` (e.g., 4-axis/tilted operations on a 3-axis hobbyist `.cps`).
    """
    for idx, cnc_p in enumerate(cnc_batches):
        out_nc = os.path.join(tmp_dir, f"probe_{idx}.nc")
        try:
            run_direct_post_with_shim(
                post_bin=post_bin,
                base_post_path=post_path,
                cnc_path=cnc_p,
                out_nc_path=out_nc,
                tmp_dir=tmp_dir,
                program_name=program_name,
                timeout=timeout,
            )
        except subprocess.TimeoutExpired:
            return None
        except RuntimeError as exc:
            msg = str(exc)
            if "Direct post binary exited with rc=" in msg:
                return msg.splitlines()[0][:240]
            raise
    return None


def _evaluate_cell(
    fixture: CorpusFixture,
    post_label: str,
    post_path: str,
    run_direct: bool = True,
    memo_verifier: Optional[_MemoizedVerifier] = None,
) -> Dict[str, Any]:
    """Run Oracle 1 and Oracle 2 on one `(fixture, post)` pair with a fresh orchestrator."""
    cell: Dict[str, Any] = {
        "fixture": fixture.name,
        "domain": "turning" if fixture.is_turning else "milling",
        "post": post_label,
        "post_path": post_path,
        "status": "failed",
        "passed": False,
        "unsupported_by_post": False,
        "unsupported_detail": "",
        "oracle1_ok": True,
        "oracle1_detail": "skipped (multi-batch or custom postProperties)",
        "oracle2_ok": False,
        "oracle2_detail": "",
        "volume_mm3": 0.0,
        "collisions": 0,
        "promoted_events": 0,
        "dog_leg_warning": False,
        "error": None,
    }
    orch = PipelineOrchestrator()
    try:
        wp_explicit = fixture.explicit_workpiece
        wp_fallback = None if wp_explicit is not None else fixture.fallback_workpiece

        with tempfile.TemporaryDirectory() as td:
            dump_files: List[str] = []
            for idx, cnc_p in enumerate(fixture.cnc_batches):
                dp = os.path.join(td, f"batch_{idx}.json")
                orch.dump_intermediate_to_json(cnc_p, post_path, dp)
                dump_files.append(dp)

            out_noprom = os.path.join(td, "noprom.nc")
            out_prom = os.path.join(td, "prom.nc")
            bootstrap_cnc = fixture.cnc_batches[0]

            eff_props = _effective_post_properties(fixture, post_path)
            rep0, g0 = orch.run(
                context_dump_paths=dump_files,
                base_post_path=post_path,
                bootstrap_cnc_path=bootstrap_cnc,
                workpiece=wp_explicit,
                fallback_workpiece=wp_fallback,
                final_cnc_path=out_noprom,
                verifier=_NoOpVerifier(),
                promote=False,
                program_name="1001",
                post_properties=eff_props or None,
                is_turning=fixture.is_turning,
            )
            rep1, g1 = orch.run(
                context_dump_paths=dump_files,
                base_post_path=post_path,
                bootstrap_cnc_path=bootstrap_cnc,
                workpiece=wp_explicit,
                fallback_workpiece=wp_fallback,
                final_cnc_path=out_prom,
                verifier=memo_verifier,
                promote=True,
                program_name="1001",
                post_properties=eff_props or None,
                is_turning=fixture.is_turning,
            )

            cell["volume_mm3"] = float(rep1.total_volume_removed_mm3)
            cell["collisions"] = len(rep1.collisions)
            cell["promoted_events"] = len(rep1.promoted_events)
            cell["dog_leg_warning"] = bool(getattr(rep1, "dog_leg_warning", None))

            if run_direct and len(fixture.cnc_batches) == 1 and not eff_props:
                out_direct = os.path.join(td, "direct.nc")
                direct_nc = run_direct_post_with_shim(
                    post_bin=orch.post_bin,
                    base_post_path=post_path,
                    cnc_path=bootstrap_cnc,
                    out_nc_path=out_direct,
                    tmp_dir=td,
                    program_name="1001",
                    timeout=orch.timeout,
                )
                ok1, det1 = check_oracle1_direct_parity(direct_nc, g0)
                cell["oracle1_ok"] = ok1
                cell["oracle1_detail"] = det1
            elif len(fixture.cnc_batches) > 1:
                cell["oracle1_detail"] = "skipped (multi-batch)"
            elif eff_props:
                cell["oracle1_detail"] = "skipped (custom postProperties)"
            else:
                cell["oracle1_detail"] = "skipped (direct check disabled)"

            all_blocked = _all_sections_blocked_from_dumps(
                dump_files, rep1, replay_diag=orch.last_replay_diag
            )
            ok2, det2 = check_oracle2_nc_diff(
                g0, g1, rep1, all_sections_blocked_override=all_blocked
            )
            cell["oracle2_ok"] = ok2
            cell["oracle2_detail"] = det2
            cell["passed"] = bool(cell["oracle1_ok"] and cell["oracle2_ok"])
            cell["status"] = "passed" if cell["passed"] else "failed"
    except Exception as exc:
        try:
            with tempfile.TemporaryDirectory() as probe_td:
                unsup_reason = _is_natively_unsupported_by_post(
                    post_bin=orch.post_bin,
                    post_path=post_path,
                    cnc_batches=fixture.cnc_batches,
                    tmp_dir=probe_td,
                    program_name="1001",
                    timeout=orch.timeout,
                )
        except Exception:
            unsup_reason = None

        if unsup_reason:
            cell["status"] = "unsupported_by_post"
            cell["unsupported_by_post"] = True
            cell["unsupported_detail"] = unsup_reason
            cell["error"] = None
            cell["passed"] = False
            cell["oracle1_ok"] = True
            cell["oracle1_detail"] = "natively unsupported by post"
            cell["oracle2_ok"] = True
            cell["oracle2_detail"] = "natively unsupported by post"
        else:
            cell["status"] = "failed"
            cell["error"] = f"{type(exc).__name__}: {exc}"
            cell["oracle1_ok"] = False
            cell["oracle2_ok"] = False
            cell["passed"] = False
    return cell


def _normalize_error_signature(msg: str) -> str:
    first_line = (msg or "unknown").splitlines()[0]
    sig = re.sub(r"\d+(?:\.\d+)?", "<N>", first_line)
    sig = re.sub(r"/[^\s]+", "<PATH>", sig)
    return sig[:140]


def _write_json_atomic(path: str, data: Dict[str, Any]) -> None:
    """Atomically write `data` as JSON using a PID-scoped temporary file."""
    parent = os.path.dirname(os.path.abspath(path))
    if parent:
        os.makedirs(parent, exist_ok=True)
    tmp_path = f"{path}.tmp.{os.getpid()}"
    with open(tmp_path, "w", encoding="utf-8") as tf:
        json.dump(data, tf, indent=2)
    os.replace(tmp_path, path)


def _evaluate_fixture_worker(
    task: Tuple[Any, ...],
) -> Dict[str, Any]:
    """Evaluate all assigned posts plus conservation/metamorphic checks for one fixture."""
    if len(task) >= 5:
        fx, post_list, run_direct, run_metamorphic, active_dir = task[:5]
    else:
        fx, post_list, run_direct, run_metamorphic = task[:4]
        active_dir = None

    t0 = time.monotonic()
    started_epoch = time.time()
    memo_verifier = _MemoizedVerifier(fx.is_turning)
    cells: List[Dict[str, Any]] = []
    worker_file = os.path.join(active_dir, f"worker_{os.getpid()}.json") if active_dir else None

    def _update_worker_status(phase: str) -> None:
        if not worker_file:
            return
        ok_cnt = sum(1 for c in cells if c.get("status") == "passed")
        unsup_cnt = sum(1 for c in cells if c.get("unsupported_by_post"))
        fail_cnt = len(cells) - ok_cnt - unsup_cnt
        try:
            _write_json_atomic(
                worker_file,
                {
                    "pid": os.getpid(),
                    "fixture": fx.name,
                    "phase": phase,
                    "posts_total": len(post_list),
                    "posts_done": len(cells),
                    "posts_passed": ok_cnt,
                    "posts_unsupported": unsup_cnt,
                    "posts_failed": fail_cnt,
                    "started_epoch": started_epoch,
                    "updated_epoch": time.time(),
                },
            )
        except OSError:
            pass

    try:
        _update_worker_status("post_matrix")
        for p_label, p_path in post_list:
            cells.append(
                _evaluate_cell(
                    fx,
                    p_label,
                    p_path,
                    run_direct=run_direct,
                    memo_verifier=memo_verifier,
                )
            )
            _update_worker_status("post_matrix")

        _update_worker_status("conservation_metamorphic")
        probe_post_path = next(
            (c["post_path"] for c in cells if c.get("status") != "unsupported_by_post"),
            None,
        )
        if probe_post_path is not None:
            fx_check = _evaluate_fixture_metamorphic_and_conservation(
                fx,
                probe_post_path,
                run_metamorphic=run_metamorphic,
                memo_verifier=memo_verifier,
            )
        else:
            fx_check = {
                "fixture": fx.name,
                "oracle2_conservation_ok": True,
                "oracle2_conservation_detail": "skipped (unsupported by all posts)",
                "oracle3_ok": True,
                "oracle3_detail": "skipped (unsupported by all posts)",
                "oracle4_diagnostic": None,
                "volume_mm3": 0.0,
                "promoted_count": 0,
            }
    finally:
        if worker_file:
            try:
                os.remove(worker_file)
            except OSError:
                pass

    elapsed = time.monotonic() - t0
    fx_check["elapsed_sec"] = round(elapsed, 3)
    fx_check["verifier_cache_hits"] = memo_verifier.cache_hits
    fx_check["verifier_cache_misses"] = memo_verifier.cache_misses
    return {
        "fixture": fx.name,
        "elapsed_sec": elapsed,
        "cells": cells,
        "fixture_check": fx_check,
    }


def _fixture_cnc_total_bytes(fixture: CorpusFixture) -> int:
    total = 0
    for p in fixture.cnc_batches:
        try:
            total += os.path.getsize(p)
        except OSError:
            pass
    return total


def run_corpus_matrix(
    fixtures: Sequence[CorpusFixture],
    posts_by_domain: Optional[Dict[str, List[Tuple[str, str]]]] = None,
    workers: int = 4,
    run_direct: bool = True,
    run_metamorphic: bool = True,
    progress_file: Optional[str] = None,
) -> Dict[str, Any]:
    """
    Execute the full `(fixture, compatible_post)` matrix in parallel across fixture
    worker processes (largest-first, per-fixture memoized IPW simulation) and return
    structured results with deduplicated failure buckets.
    """
    if posts_by_domain is None:
        posts_by_domain = resolve_hobbyist_posts()

    search_dirs = get_post_search_dirs()
    fixture_tasks_sized: List[Tuple[int, CorpusFixture, List[Tuple[str, str]]]] = []

    for fx in fixtures:
        domain = "turning" if fx.is_turning else "milling"
        post_list = list(posts_by_domain.get(domain, []))
        seen_p = {p for _, p in post_list}
        if fx.configured_post:
            cfg_resolved = resolve_post_file(fx.configured_post, search_dirs=search_dirs)
            if cfg_resolved and cfg_resolved not in seen_p:
                seen_p.add(cfg_resolved)
                post_list.append((os.path.basename(cfg_resolved), cfg_resolved))
        if post_list:
            fixture_tasks_sized.append((_fixture_cnc_total_bytes(fx), fx, post_list))

    fixture_tasks_sized.sort(key=lambda item: (-item[0], item[1].name))

    active_dir: Optional[str] = None
    if progress_file:
        active_dir = os.path.join(
            os.path.dirname(os.path.abspath(progress_file)), ".active_workers"
        )
        if os.path.isdir(active_dir):
            shutil.rmtree(active_dir, ignore_errors=True)
        os.makedirs(active_dir, exist_ok=True)

    worker_args: List[Tuple[CorpusFixture, List[Tuple[str, str]], bool, bool, Optional[str]]] = [
        (fx, post_list, run_direct, run_metamorphic, active_dir)
        for _, fx, post_list in fixture_tasks_sized
    ]

    max_workers = max(1, int(workers))
    cell_results: List[Dict[str, Any]] = []
    fixture_checks: List[Dict[str, Any]] = []
    total_fx = len(worker_args)
    done_fx = 0
    run_t0 = time.monotonic()
    fx_passed_cnt = 0
    fx_failed_cnt = 0
    completed_runs = 0
    completed_passed = 0
    completed_unsup = 0
    completed_failed = 0
    recent_completed: List[Dict[str, Any]] = []

    def _flush_progress(status: str) -> None:
        if not progress_file:
            return
        try:
            _write_json_atomic(
                progress_file,
                {
                    "status": status,
                    "elapsed_sec": round(time.monotonic() - run_t0, 2),
                    "updated_epoch": time.time(),
                    "fixtures_total": total_fx,
                    "fixtures_completed": done_fx,
                    "fixtures_passed": fx_passed_cnt,
                    "fixtures_failed": fx_failed_cnt,
                    "completed_matrix_runs": completed_runs,
                    "completed_matrix_passed": completed_passed,
                    "completed_matrix_unsupported": completed_unsup,
                    "completed_matrix_failed": completed_failed,
                    "recent_completed": list(recent_completed[-5:]),
                },
            )
        except OSError:
            pass

    def _record_worker_result(res: Dict[str, Any]) -> None:
        nonlocal done_fx, fx_passed_cnt, fx_failed_cnt
        nonlocal completed_runs, completed_passed, completed_unsup, completed_failed
        done_fx += 1
        cells = res["cells"]
        fc = res["fixture_check"]
        cell_results.extend(cells)
        fixture_checks.append(fc)
        ok_cnt = sum(1 for c in cells if c.get("status") == "passed")
        unsup_cnt = sum(1 for c in cells if c.get("unsupported_by_post"))
        fail_cnt = len(cells) - ok_cnt - unsup_cnt
        fx_ok = bool(fc.get("oracle2_conservation_ok") and fc.get("oracle3_ok"))
        if fx_ok:
            fx_passed_cnt += 1
        else:
            fx_failed_cnt += 1
        completed_runs += len(cells)
        completed_passed += ok_cnt
        completed_unsup += unsup_cnt
        completed_failed += fail_cnt
        recent_completed.append(
            {
                "fixture": res["fixture"],
                "elapsed_sec": round(res["elapsed_sec"], 1),
                "passed": ok_cnt,
                "unsupported": unsup_cnt,
                "failed": fail_cnt,
                "fixture_ok": fx_ok,
            }
        )
        _flush_progress("running")
        sys.stderr.write(
            f"[{done_fx}/{total_fx}] {res['fixture']}: "
            f"{ok_cnt} passed, {unsup_cnt} unsupported, {fail_cnt} failed ({res['elapsed_sec']:.1f}s)\n"
        )
        sys.stderr.flush()

    try:
        _flush_progress("running")
        if worker_args:
            if max_workers == 1 or len(worker_args) <= 1:
                for arg in worker_args:
                    _record_worker_result(_evaluate_fixture_worker(arg))
            else:
                pool_workers = min(max_workers, len(worker_args))
                with concurrent.futures.ProcessPoolExecutor(max_workers=pool_workers) as executor:
                    futs = [executor.submit(_evaluate_fixture_worker, arg) for arg in worker_args]
                    for fut in concurrent.futures.as_completed(futs):
                        _record_worker_result(fut.result())
        _flush_progress("completed")
    finally:
        if active_dir and os.path.isdir(active_dir):
            shutil.rmtree(active_dir, ignore_errors=True)

    cell_results.sort(key=lambda r: (r["fixture"], r["post"]))
    fixture_checks.sort(key=lambda r: r["fixture"])

    failure_buckets: Dict[Tuple[str, str, str], Dict[str, Any]] = {}

    def _record_bucket(oracle: str, post: str, msg: str, fixture_name: str) -> None:
        sig = _normalize_error_signature(msg)
        key = (oracle, post, sig)
        if key not in failure_buckets:
            failure_buckets[key] = {
                "oracle": oracle,
                "post": post,
                "signature": sig,
                "count": 0,
                "examples": [],
                "sample_detail": msg[:800],
            }
        failure_buckets[key]["count"] += 1
        if len(failure_buckets[key]["examples"]) < 5:
            failure_buckets[key]["examples"].append(fixture_name)

    for c in cell_results:
        if c.get("unsupported_by_post"):
            continue
        if c.get("error"):
            _record_bucket("pipeline_runtime", c["post"], str(c["error"]), c["fixture"])
        else:
            if not c["oracle1_ok"]:
                _record_bucket("oracle1_direct_parity", c["post"], c["oracle1_detail"], c["fixture"])
            if not c["oracle2_ok"]:
                _record_bucket("oracle2_diff_guard", c["post"], c["oracle2_detail"], c["fixture"])

    for fc in fixture_checks:
        if not fc["oracle2_conservation_ok"]:
            _record_bucket("oracle2_conservation", "fixture", fc["oracle2_conservation_detail"], fc["fixture"])
        if not fc["oracle3_ok"]:
            _record_bucket("oracle3_metamorphic", "fixture", fc["oracle3_detail"], fc["fixture"])

    total_cells = len(cell_results)
    passed_cells = sum(1 for c in cell_results if c.get("status") == "passed")
    unsupported_cells = sum(1 for c in cell_results if c.get("unsupported_by_post"))
    failed_cells = total_cells - passed_cells - unsupported_cells
    passed_fixtures = sum(1 for fc in fixture_checks if fc["oracle2_conservation_ok"] and fc["oracle3_ok"])

    return {
        "summary": {
            "fixtures_tested": len(fixtures),
            "fixtures_passed": passed_fixtures,
            "matrix_runs": total_cells,
            "matrix_passed": passed_cells,
            "matrix_unsupported": unsupported_cells,
            "matrix_failed": failed_cells,
            "unique_failures": len(failure_buckets),
            "all_passed": (
                failed_cells == 0
                and passed_fixtures == len(fixture_checks)
                and len(failure_buckets) == 0
            ),
        },
        "posts_by_domain": {
            dom: [{"label": lbl, "path": p} for lbl, p in plist]
            for dom, plist in posts_by_domain.items()
        },
        "cell_results": cell_results,
        "fixture_checks": fixture_checks,
        "deduplicated_failures": sorted(
            failure_buckets.values(), key=lambda b: (-b["count"], b["oracle"], b["post"])
        ),
    }


def format_corpus_report(results: Dict[str, Any]) -> str:
    """Format a concise Markdown report for corpus run results."""
    s = results["summary"]
    lines = [
        "# PostClear Corpus Verification Report",
        "",
        f"Tested **{s['fixtures_tested']}** fixture(s) across **{s['matrix_runs']}** fixture-post combinations. "
        f"Overall status: **{'PASS' if s['all_passed'] else 'FAIL'}**.",
        "",
        "| Metric | Value |",
        "| :--- | :--- |",
        f"| Fixtures Passed (Conservation + Metamorphic) | {s['fixtures_passed']} / {s['fixtures_tested']} |",
        f"| Post Matrix Passed (Parity + Diff Guard) | {s['matrix_passed']} / {s['matrix_runs']} |",
        f"| Natively Unsupported by Post (Skipped) | {s.get('matrix_unsupported', 0)} |",
        f"| Post Matrix Failed | {s.get('matrix_failed', 0)} |",
        f"| Deduplicated Failure Signatures | {s['unique_failures']} |",
        "",
        "## Per-Post Summary",
        "| Post | Domain | Runs | Passed | Unsupported | Failed |",
        "| :--- | :--- | ---: | ---: | ---: | ---: |",
    ]

    by_post: Dict[Tuple[str, str], List[Dict[str, Any]]] = {}
    for c in results["cell_results"]:
        by_post.setdefault((c["post"], c["domain"]), []).append(c)
    for (p_name, dom), items in sorted(by_post.items()):
        ok_cnt = sum(1 for i in items if i.get("status") == "passed")
        unsup_cnt = sum(1 for i in items if i.get("unsupported_by_post"))
        fail_cnt = len(items) - ok_cnt - unsup_cnt
        lines.append(f"| `{p_name}` | {dom} | {len(items)} | {ok_cnt} | {unsup_cnt} | {fail_cnt} |")

    if results["deduplicated_failures"]:
        lines.extend([
            "",
            "## Deduplicated Failures",
            "| Oracle | Post | Count | Signature | Example Fixtures |",
            "| :--- | :--- | ---: | :--- | :--- |",
        ])
        for b in results["deduplicated_failures"]:
            exs = ", ".join(f"`{e}`" for e in b["examples"])
            lines.append(f"| {b['oracle']} | `{b['post']}` | {b['count']} | `{b['signature']}` | {exs} |")

    diag_rows = [fc for fc in results["fixture_checks"] if fc.get("oracle4_diagnostic")]
    if diag_rows:
        lines.extend([
            "",
            "## Post-Setup IPW STL Cross-Check (Oracle 4 Diagnostic)",
            "| Fixture | p95 Z Diff (mm) | Max Z Diff (mm) |",
            "| :--- | ---: | ---: |",
        ])
        for fc in diag_rows:
            d = fc["oracle4_diagnostic"]
            lines.append(f"| `{fc['fixture']}` | {d['p95_z_diff_mm']:.3f} | {d['max_z_diff_mm']:.3f} |")

    return "\n".join(lines) + "\n"


def write_corpus_reports(results: Dict[str, Any], output_dir: str) -> Tuple[str, str]:
    """Write `corpus_results.json` and `corpus_report.md` into `output_dir`."""
    os.makedirs(output_dir, exist_ok=True)
    json_path = os.path.join(output_dir, "corpus_results.json")
    md_path = os.path.join(output_dir, "corpus_report.md")
    with open(json_path, "w", encoding="utf-8") as jf:
        json.dump(results, jf, indent=2)
    with open(md_path, "w", encoding="utf-8") as mf:
        mf.write(format_corpus_report(results))
    return md_path, json_path


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Run PostClear corpus verification matrix across Top 10 hobbyist posts")
    parser.add_argument("--corpus-dir", default=os.path.join(REPO_ROOT, "corpus"), help="Directory of harvested setup bundles (default: corpus/)")
    parser.add_argument("--extra-fixtures", nargs="*", default=(), help="Additional .cnc files or fixture directories")
    parser.add_argument("--extra-posts", nargs="*", default=(), help="Additional .cps post filenames or paths to include")
    parser.add_argument("--only-posts", nargs="*", default=None, help="Restrict run to specific post filenames")
    parser.add_argument("--no-builtin-fixtures", action="store_true", help="Exclude tests/fixtures/*.cnc built-in fixtures")
    parser.add_argument("--output-dir", default=None, help="Report output directory (defaults to --corpus-dir)")
    parser.add_argument("--workers", type=int, default=4, help="Parallel worker count (default: 4)")
    parser.add_argument("--skip-direct", action="store_true", help="Skip Oracle 1 direct post parity check")
    parser.add_argument("--skip-metamorphic", action="store_true", help="Skip Oracle 3 metamorphic transform checks")
    parser.add_argument("--max-fixtures", type=int, default=None, help="Cap number of discovered fixtures")
    args = parser.parse_args(argv)

    posts_by_domain = resolve_hobbyist_posts(extra_posts=args.extra_posts, only_posts=args.only_posts)
    fixtures = discover_fixtures(
        corpus_dir=args.corpus_dir,
        extra_paths=args.extra_fixtures,
        include_builtin_fixtures=not args.no_builtin_fixtures,
        max_fixtures=args.max_fixtures,
    )
    if not fixtures:
        print("No fixtures found to run.", file=sys.stderr)
        return 1

    out_dir = args.output_dir or args.corpus_dir
    results = run_corpus_matrix(
        fixtures=fixtures,
        posts_by_domain=posts_by_domain,
        workers=args.workers,
        run_direct=not args.skip_direct,
        run_metamorphic=not args.skip_metamorphic,
        progress_file=os.path.join(out_dir, "corpus_progress.json"),
    )
    md_path, json_path = write_corpus_reports(results, out_dir)
    print(format_corpus_report(results))
    print(f"Reports written to: {md_path} and {json_path}")
    return 0 if results["summary"]["all_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
