#!/usr/bin/env python3
"""
In-Fusion 360 CAM Corpus Harvester for PostClear.

Crawls Fusion 360 cloud projects, active project, currently open documents, or
a local folder of `.f3d`/`.f3z` archives with minimal interaction, exporting each
unsuppressed CAM Setup into a self-contained offline fixture bundle under
`corpus/<project>/<document>/<setup>/`:
  - `setup_manifest.json`: domain, stock mode, units, configured post & props, workpiece metadata
  - `batch_0.cnc .. batch_k.cnc`: single-tool intermediate Compact-NCI files (numeric `programName`)
  - `batch_0.json .. batch_k.json`: structured dumper JSON snapshots
  - `prev_stock.stl`: exported in-process stock STL (when `PreviousSetupStock`)
  - `solid_stock.stl`: exported Setup WCS mm binary STL (when `SolidStock`)
  - `post_setup_ipw.stl`: preceding setup's ground-truth IPW STL when chaining matches WCS
"""

from __future__ import annotations

import argparse
import datetime
import json
import os
import re
import shutil
import struct
import sys
import tempfile
import time
import traceback
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)


def write_binary_stl(stl_path: str, vertices_mm: Any, indices: Any, header_text: str = "PostClear Binary STL (mm)") -> str:
    """Write `(vertices_mm, indices)` triangle mesh to a standard binary STL file."""
    import numpy as np

    verts = np.asarray(vertices_mm, dtype=np.float32)
    tris_idx = np.asarray(indices, dtype=np.int32).reshape(-1, 3)
    if verts.ndim != 2 or verts.shape[1] != 3 or tris_idx.size == 0:
        raise ValueError("write_binary_stl requires non-empty (N, 3) vertices and (M, 3) indices")

    tri_verts = verts[tris_idx]  # (M, 3, 3)
    e1 = tri_verts[:, 1, :] - tri_verts[:, 0, :]
    e2 = tri_verts[:, 2, :] - tri_verts[:, 0, :]
    normals = np.cross(e1, e2)
    norms = np.linalg.norm(normals, axis=1, keepdims=True)
    safe_norms = np.where(norms > 1e-12, normals / np.maximum(norms, 1e-12), 0.0).astype(np.float32)

    record_dtype = np.dtype([
        ("normal", "<f4", (3,)),
        ("v1", "<f4", (3,)),
        ("v2", "<f4", (3,)),
        ("v3", "<f4", (3,)),
        ("attr", "<u2"),
    ])
    records = np.empty(len(tri_verts), dtype=record_dtype)
    records["normal"] = safe_norms
    records["v1"] = tri_verts[:, 0, :]
    records["v2"] = tri_verts[:, 1, :]
    records["v3"] = tri_verts[:, 2, :]
    records["attr"] = 0

    os.makedirs(os.path.dirname(os.path.abspath(stl_path)), exist_ok=True)
    hdr = header_text.encode("ascii", errors="ignore")[:80].ljust(80, b"\x00")
    with open(stl_path, "wb") as f:
        f.write(hdr)
        f.write(struct.pack("<I", len(tri_verts)))
        records.tofile(f)
    return os.path.abspath(stl_path)


def detect_stl_is_inch(stl_path: str, dump_data: Dict[str, Any]) -> bool:
    """Infer whether an exported `PreviousSetupStock` STL uses inch or millimeter coordinates."""
    from context_parser import extract_workpiece_hint_bbox_mm, infer_stl_is_inch
    from geometry.solid_mesh import load_stl_mesh

    if extract_workpiece_hint_bbox_mm(dump_data) is None:
        return infer_stl_is_inch(None, dump_data, strict=False)
    raw_verts, _ = load_stl_mesh(stl_path, is_inch=False)
    return infer_stl_is_inch(raw_verts, dump_data, strict=False)


def extract_wcs_dict(setup: Any) -> Optional[Dict[str, List[float]]]:
    """Extract normalized Setup WCS axes and translation from `setup.workCoordinateSystem`."""
    try:
        wcs = getattr(setup, "workCoordinateSystem", None)
        if wcs is None or not hasattr(wcs, "getAsCoordinateSystem"):
            return None
        origin, x_ax, y_ax, z_ax = wcs.getAsCoordinateSystem()
        return {
            "origin": [float(origin.x), float(origin.y), float(origin.z)],
            "x_axis": [float(x_ax.x), float(x_ax.y), float(x_ax.z)],
            "y_axis": [float(y_ax.x), float(y_ax.y), float(y_ax.z)],
            "z_axis": [float(z_ax.x), float(z_ax.y), float(z_ax.z)],
        }
    except Exception:
        return None


def _wcs_matches(w1: Optional[Dict[str, List[float]]], w2: Optional[Dict[str, List[float]]]) -> bool:
    if not w1 or not w2:
        return False
    for k in ("origin", "x_axis", "y_axis", "z_axis"):
        a = w1.get(k) or []
        b = w2.get(k) or []
        if len(a) != 3 or len(b) != 3:
            return False
        tol = 0.01 if k == "origin" else 1e-4
        if any(abs(a[i] - b[i]) > tol for i in range(3)):
            return False
    return True


def _sanitize_name(text: str, fallback: str = "unnamed") -> str:
    clean = re.sub(r'[<>:"/\\|?*\x00-\x1f]+', "_", str(text or "")).strip("._ ")
    return clean[:80] or fallback


def _get_postclear_module() -> Any:
    if REPO_ROOT not in sys.path:
        sys.path.insert(0, REPO_ROOT)
    import PostClear
    return PostClear


def _find_nc_program_for_setup(cam: Any, setup: Any, is_turning: bool, PostClear: Any) -> Tuple[Optional[Any], str, Dict[str, Any]]:
    """Find the most specific valid NCProgram targeting `setup`, else fall back to a compatible post."""
    nc_progs = PostClear._extract_cam_collection(getattr(cam, "ncPrograms", None))
    setup_name = getattr(setup, "name", None)
    setup_ops = PostClear._extract_cam_collection(getattr(setup, "allOperations", []))
    setup_op_ids = {getattr(op, "operationId", None) for op in setup_ops if getattr(op, "operationId", None) is not None}

    matched_prog = None
    fallback_prog = None
    for prog in nc_progs:
        try:
            base_p = PostClear._ResolveBasePostPath(prog, is_turning=is_turning)
        except Exception:
            base_p = ""
        if not base_p or not os.path.isfile(base_p):
            continue
        if fallback_prog is None:
            fallback_prog = (prog, base_p)
        prog_targets = (
            PostClear._extract_cam_collection(getattr(prog, "setups", []))
            + PostClear._extract_cam_collection(getattr(prog, "operations", []))
        )
        for item in prog_targets:
            if item == setup or (setup_name and getattr(item, "name", None) == setup_name):
                matched_prog = (prog, base_p)
                break
            if getattr(item, "operationId", None) in setup_op_ids:
                matched_prog = (prog, base_p)
                break
        if matched_prog is not None:
            break

    chosen = matched_prog or fallback_prog
    if chosen is not None:
        prog_obj, base_path = chosen
        return prog_obj, base_path, PostClear._ExtractPostProperties(prog_obj)

    from pipeline import get_post_search_dirs, resolve_post_file
    sdirs = get_post_search_dirs()
    fallback_names = ("centroid_turning.cps", "fanuc turning.cps") if is_turning else ("grbl.cps", "shopbot.cps", "fanuc.cps")
    for nm in fallback_names:
        resolved = resolve_post_file(nm, search_dirs=sdirs)
        if resolved:
            return None, resolved, {}
    raise FileNotFoundError(f"Could not find any fallback {'turning' if is_turning else 'milling'} post processor for dumping")


def _serialize_workpiece(wp: Any, kind: str) -> Optional[Dict[str, Any]]:
    if wp is None:
        return None
    fixtures_list = []
    for fx in getattr(wp, "fixtures", None) or []:
        if hasattr(fx, "z_min"):
            fixtures_list.append({
                "z_min": float(fx.z_min),
                "z_max": float(fx.z_max),
                "outer_radius": float(fx.outer_radius),
                "clamping_radius": float(fx.clamping_radius),
            })
    st = getattr(wp, "stock", None)
    if kind == "solid_turning":
        return {"kind": "solid_turning", "fixtures": fixtures_list}
    if st is not None and hasattr(st, "outer_radius") and hasattr(st, "z_min"):
        return {
            "kind": "turning",
            "z_min": float(st.z_min),
            "z_max": float(st.z_max),
            "outer_radius": float(st.outer_radius),
            "inner_radius": float(getattr(st, "inner_radius", 0.0)),
            "fixtures": fixtures_list,
        }
    if st is not None and hasattr(st, "x_min"):
        return {
            "kind": "box",
            "x_min": float(st.x_min),
            "x_max": float(st.x_max),
            "y_min": float(st.y_min),
            "y_max": float(st.y_max),
            "z_min": float(st.z_min),
            "z_max": float(st.z_max),
            "fixtures": [],
        }
    return None


def _is_stale_or_temp_path(path_val: Any, temp_root: Optional[str]) -> bool:
    if not path_val:
        return False
    p_str = str(path_val)
    if not os.path.isfile(p_str):
        return True
    if temp_root:
        abs_tmp = os.path.abspath(temp_root)
        abs_p = os.path.abspath(p_str)
        if abs_p == abs_tmp or abs_p.startswith(abs_tmp + os.sep):
            return True
    return False


def _rewrite_dump_stock_paths(
    dump_data: Dict[str, Any],
    rel_stock_path: Optional[str],
    json_path: str,
    temp_root: Optional[str] = None,
) -> None:
    """Persist relative STL references or remove stale/temp `stockPath` entries from a dump JSON."""
    changed = False
    if rel_stock_path:
        if dump_data.get("stockPath") != rel_stock_path:
            dump_data["stockPath"] = rel_stock_path
            changed = True
        se = dump_data.get("stockExport")
        if isinstance(se, dict) and se.get("destPath") != rel_stock_path:
            se["destPath"] = rel_stock_path
            changed = True
    else:
        if _is_stale_or_temp_path(dump_data.get("stockPath"), temp_root):
            dump_data.pop("stockPath", None)
            changed = True
        se = dump_data.get("stockExport")
        if isinstance(se, dict) and _is_stale_or_temp_path(se.get("destPath"), temp_root):
            se.pop("destPath", None)
            changed = True
    if changed:
        with open(json_path, "w", encoding="utf-8") as jf:
            json.dump(dump_data, jf)


def _workpiece_dict_from_stl(
    stl_path: str,
    is_turning: bool,
    is_inch: bool,
    fixtures: Optional[List[Any]] = None,
    kind_override: Optional[str] = None,
) -> Optional[Dict[str, Any]]:
    """Build and serialize a metric `workpiece_mm` dictionary from an STL mesh file."""
    from geometry.solid_mesh import extract_workpiece_from_stl

    wp = extract_workpiece_from_stl(
        stl_path,
        is_turning=is_turning,
        is_inch=is_inch,
        fixtures=fixtures,
    )
    kind = kind_override or ("turning" if is_turning else "box")
    return _serialize_workpiece(wp, kind=kind)


def _build_setup_manifest(
    *,
    project_name: str,
    doc_name: str,
    setup_name: str,
    setup_index: int,
    is_turning: bool,
    stock_mode: str,
    unit: Any,
    stock_unit: Any,
    configured_post: Optional[str],
    post_props: Dict[str, Any],
    cnc_batches: List[str],
    json_batches: List[str],
    prev_stock_rel: Optional[str],
    prev_stock_is_inch: bool,
    solid_stock_rel: Optional[str],
    workpiece_mm_dict: Optional[Dict[str, Any]],
    wcs_dict: Optional[Dict[str, List[float]]],
) -> Dict[str, Any]:
    """Build a canonical schema v1 `setup_manifest.json` dictionary."""
    return {
        "schema_version": 1,
        "harvested_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "project_name": project_name,
        "document_name": doc_name,
        "setup_name": setup_name,
        "setup_index": setup_index,
        "is_turning": bool(is_turning),
        "stock_mode": stock_mode,
        "unit": unit,
        "stock_unit": stock_unit,
        "configured_post": configured_post,
        "post_properties": post_props,
        "cnc_batches": cnc_batches,
        "json_batches": json_batches,
        "prev_stock_stl": prev_stock_rel,
        "prev_stock_is_inch": bool(prev_stock_is_inch),
        "solid_stock_stl": solid_stock_rel,
        "post_setup_ipw_stl": None,
        "workpiece_mm": workpiece_mm_dict,
        "wcs": wcs_dict,
    }


def _get_param_raw_value(params: Any, name: str) -> Any:
    if params is None or not hasattr(params, "itemByName"):
        return None
    try:
        p = params.itemByName(name)
        if p is None:
            return None
        v_obj = getattr(p, "value", None)
        return getattr(v_obj, "value", v_obj)
    except Exception:
        return None


def _is_supported_cutting_op(op: Any) -> bool:
    """Return False for non-cutting (probing, inspection, jet, or additive) operations."""
    from context_parser import is_truthy_flag

    if getattr(op, "isProbing", False) is True:
        return False
    op_params = getattr(op, "parameters", None)
    for flag_name in (
        "operation:isProbingStrategy",
        "isProbingStrategy",
        "operation:isInspectionStrategy",
        "isInspectionStrategy",
        "operation:isJetStrategy",
        "isJetStrategy",
        "operation:isAdditiveStrategy",
        "isAdditiveStrategy",
    ):
        if is_truthy_flag(_get_param_raw_value(op_params, flag_name)):
            return False
    tool = getattr(op, "tool", None)
    tool_params = getattr(tool, "parameters", None) if tool is not None else None
    return str(_get_param_raw_value(tool_params, "tool_type") or "").strip().lower() != "probe"


def _partition_tool_batches(setup: Any, PostClear: Any) -> List[Tuple[int, List[Any]]]:
    cand_list = PostClear._extract_cam_collection(getattr(setup, "allOperations", []))
    active_ops = [
        op for op in cand_list
        if not getattr(op, "isSuppressed", False)
        and getattr(op, "hasToolpath", True)
        and getattr(op, "isToolpathValid", True)
        and _is_supported_cutting_op(op)
    ]
    return PostClear.partition_operations_by_tool(active_ops, default_tool_number=0, coerce_int=True)


def _read_post_failure_hint(b_tmp: str) -> str:
    try:
        for fn in sorted(os.listdir(b_tmp)):
            if fn.lower().endswith(".failed"):
                with open(os.path.join(b_tmp, fn), "r", encoding="utf-8", errors="replace") as ff:
                    txt = " ".join(ff.read(600).split())
                if txt:
                    return txt
    except Exception:
        pass
    return ""


class _SkipSetupCapability(Exception):
    """Internal signal raised when a setup uses commercial-only multi-axis / 3+2 / mill-turn features."""


def _is_personal_use_multiaxis_post_block(hint: str) -> bool:
    """Return True if a post failure hint reflects Fusion Personal Use blocking 3+2 / multi-axis."""
    if not hint:
        return False
    low = hint.lower()
    return any(
        tok in low
        for tok in (
            "cannot locate the simple error log generated from the post processing",
            "tool orientation is not supported",
            "work plane is tilted",
            "multi-axis",
            "multiaxis",
            "5-axis",
            "4-axis",
        )
    )


def _setup_has_unsupported_capability(setup: Any, PostClear: Any, adsk_cam: Any = None) -> Optional[str]:
    """
    Return an error reason if `setup` uses an unsupported setup type or any unsuppressed
    operation uses 3+2 Tool Orientation, 4-axis rotary wrap, simultaneous 5-axis, or mill-turn.
    """
    check_setup_fn = getattr(PostClear, "_CheckSetupCapability", None)
    if callable(check_setup_fn):
        setup_err = check_setup_fn(setup)
        if setup_err:
            return setup_err
    op_enum = getattr(adsk_cam, "OperationTypes", None) if adsk_cam is not None else None
    turn_val = getattr(op_enum, "TurningOperation", 1) if op_enum is not None else 1
    op_type = getattr(setup, "operationType", None)
    is_turning = (op_type in (turn_val, 1)) and not isinstance(op_type, bool)

    check_op_fn = getattr(PostClear, "_CheckOperationCapability", None)
    if not callable(check_op_fn):
        return None
    for op in PostClear._extract_cam_collection(getattr(setup, "allOperations", [])):
        if getattr(op, "isSuppressed", False):
            continue
        if not _is_supported_cutting_op(op):
            continue
        err = check_op_fn(op, setup=setup, is_turning=is_turning, check_probing=False)
        if err:
            return err
    return None


def harvest_setup(
    doc: Any,
    cam: Any,
    setup: Any,
    setup_idx: int,
    bundle_dir: str,
    project_name: str = "",
    doc_name: str = "",
    prev_setup_record: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """
    Export one active CAM Setup into `bundle_dir`, producing `batch_*.cnc`,
    `batch_*.json`, stock `.stl` files, and `setup_manifest.json`.
    """
    import adsk.cam
    import adsk.core

    setup_name = getattr(setup, "name", f"Setup_{setup_idx}")
    PostClear = _get_postclear_module()
    if _setup_has_unsupported_capability(setup, PostClear, adsk.cam) is not None:
        return {"status": "skipped_capability", "setup_name": setup_name}

    is_turning = (getattr(setup, "operationType", None) == adsk.cam.OperationTypes.TurningOperation)
    stock_mode = PostClear._GetSetupStockMode(setup)
    batches = _partition_tool_batches(setup, PostClear)
    if not batches:
        return {"status": "no_active_ops", "setup_name": setup_name}

    prog_obj, dump_post_path, post_props = _find_nc_program_for_setup(cam, setup, is_turning, PostClear)
    configured_post = os.path.basename(dump_post_path) if prog_obj is not None else None

    os.makedirs(bundle_dir, exist_ok=True)
    manifest_path = os.path.join(bundle_dir, "setup_manifest.json")

    try:
        enum_cls = getattr(adsk.cam, "PostOutputUnitOptions", None)
        mm_opt = getattr(enum_cls, "MillimetersOutput", 2) if enum_cls else 2
        in_opt = getattr(enum_cls, "InchesOutput", 1) if enum_cls else 1
        units = PostClear._GetNCProgramOutputUnits(prog_obj)
        dumper_unit = 1 if units == mm_opt else (0 if units == in_opt else None)
        stock_unit = PostClear._GetDocumentStockUnit(doc)
        is_prev_stock = any(k in stock_mode for k in ("previous", "preceding", "prev", "rest"))
        is_solid_stock = any(k in stock_mode for k in ("solid", "body"))

        cnc_batches: List[str] = []
        json_batches: List[str] = []
        prev_stock_rel: Optional[str] = None
        prev_stock_is_inch = False
        first_dump_data: Optional[Dict[str, Any]] = None

        with tempfile.TemporaryDirectory(prefix="pc_harvest_") as tmp_root:
            out_batch_idx = 0
            last_batch_err: Optional[Exception] = None

            def _attempt_post_ops(
                sub_ops: List[Any],
                attempt_tag: str,
                target_cnc_path: str,
                want_export_flag: bool,
                base_post_override: str,
            ) -> Tuple[Optional[str], str, Optional[Exception]]:
                b_tmp = os.path.join(tmp_root, f"b_{attempt_tag}")
                os.makedirs(b_tmp, exist_ok=True)
                num_name = str(1001 + out_batch_idx)
                cfg_path = PostClear._WriteDumperConfig(
                    base_post_path=base_post_override,
                    target_cnc_path=target_cnc_path,
                    post_properties=post_props if base_post_override else {},
                    target_dir=b_tmp,
                    stock_mode=stock_mode,
                    unit=dumper_unit,
                    stock_unit=stock_unit,
                    set_env=False,
                    export_stock=want_export_flag,
                )
                dumper_cps = PostClear.constDumperScript
                if want_export_flag:
                    staged = os.path.join(b_tmp, "universal_dumper_export_stock.cps")
                    shutil.copy2(PostClear.constDumperScript, staged)
                    dumper_cps = staged

                if len(sub_ops) == 1:
                    target_ops = sub_ops[0]
                else:
                    target_ops = adsk.core.ObjectCollection.create()
                    for op in sub_ops:
                        target_ops.add(op)

                env_updates = {
                    "BASE_POST": os.path.abspath(base_post_override) if base_post_override else "",
                    "TARGET_INTERMEDIATE_CNC": os.path.abspath(target_cnc_path),
                    "DUMPER_CONFIG_PATH": cfg_path,
                }
                post_exc: Optional[Exception] = None
                with PostClear._ScopedEnv(env_updates):
                    post_input = adsk.cam.PostProcessInput.create(num_name, dumper_cps, b_tmp, units)
                    post_input.isOpenInEditor = False
                    try:
                        cam.postProcess(target_ops, post_input)
                    except Exception as exc:
                        post_exc = exc

                json_cand = os.path.join(b_tmp, f"{num_name}.json")
                if not os.path.isfile(json_cand):
                    pattern = re.compile(rf"^{re.escape(num_name)}(_\d+)?\.json$")
                    for fn in sorted(os.listdir(b_tmp)):
                        if pattern.match(fn):
                            json_cand = os.path.join(b_tmp, fn)
                            break
                if os.path.isfile(json_cand) and os.path.getsize(json_cand) > 0:
                    return json_cand, b_tmp, None
                return None, b_tmp, post_exc

            def _run_with_base_fallback(
                sub_ops: List[Any],
                attempt_tag: str,
                target_cnc_path: str,
                want_export_flag: bool,
            ) -> Tuple[Optional[str], str, Optional[Exception]]:
                jc, bt, p_exc = _attempt_post_ops(
                    sub_ops, attempt_tag, target_cnc_path, want_export_flag, dump_post_path
                )
                if jc is not None:
                    return jc, bt, None
                if dump_post_path:
                    jc2, bt2, p_exc2 = _attempt_post_ops(
                        sub_ops, f"{attempt_tag}_nobase", target_cnc_path, want_export_flag, ""
                    )
                    if jc2 is not None:
                        return jc2, bt2, None
                    return None, bt, p_exc or p_exc2
                return None, bt, p_exc

            def _commit_batch(json_cand: str, b_tmp: str, target_cnc: str, want_export: bool) -> None:
                nonlocal out_batch_idx, prev_stock_rel, prev_stock_is_inch, first_dump_data
                num_name = str(1001 + out_batch_idx)
                out_json = os.path.join(bundle_dir, f"batch_{out_batch_idx}.json")
                shutil.copy2(json_cand, out_json)

                if not os.path.isfile(target_cnc):
                    fallback_cnc = json_cand + ".intermediate.cnc"
                    if os.path.isfile(fallback_cnc):
                        shutil.copy2(fallback_cnc, target_cnc)
                if not os.path.isfile(target_cnc):
                    os.remove(out_json)
                    raise RuntimeError(
                        f"Harvester failed to capture .cnc intermediate file for batch {out_batch_idx} of '{setup_name}'"
                    )

                json_batches.append(f"batch_{out_batch_idx}.json")
                cnc_batches.append(f"batch_{out_batch_idx}.cnc")

                with open(out_json, "r", encoding="utf-8") as jf:
                    dump_data = json.load(jf)

                from context_parser import check_context_dict_capability

                cap_err = check_context_dict_capability(
                    dump_data,
                    setup_name=setup_name,
                    is_turning=bool(is_turning),
                    check_probing=False,
                )
                if cap_err:
                    raise _SkipSetupCapability(cap_err)

                if want_export:
                    src_stl = None
                    dest_in_dump = (dump_data.get("stockExport") or {}).get("destPath") or dump_data.get("stockPath")
                    if dest_in_dump and os.path.isfile(dest_in_dump):
                        src_stl = dest_in_dump
                    else:
                        fallback_stl = os.path.join(b_tmp, f"{num_name}_stock.stl")
                        if os.path.isfile(fallback_stl):
                            src_stl = fallback_stl
                    if src_stl and os.path.isfile(src_stl):
                        out_prev_stl = os.path.join(bundle_dir, "prev_stock.stl")
                        shutil.copy2(src_stl, out_prev_stl)
                        prev_stock_rel = "prev_stock.stl"
                        prev_stock_is_inch = detect_stl_is_inch(out_prev_stl, dump_data)
                        _rewrite_dump_stock_paths(dump_data, prev_stock_rel, out_json, temp_root=tmp_root)
                    else:
                        _rewrite_dump_stock_paths(dump_data, None, out_json, temp_root=tmp_root)
                else:
                    _rewrite_dump_stock_paths(dump_data, None, out_json, temp_root=tmp_root)

                if first_dump_data is None:
                    first_dump_data = dump_data
                out_batch_idx += 1

            for b_idx, (_t_num, op_list) in enumerate(batches):
                want_export = bool(is_prev_stock and out_batch_idx == 0)
                target_cnc = os.path.join(bundle_dir, f"batch_{out_batch_idx}.cnc")
                json_cand, b_tmp, p_exc = _run_with_base_fallback(
                    op_list, str(b_idx), target_cnc, want_export
                )
                if json_cand is not None:
                    _commit_batch(json_cand, b_tmp, target_cnc, want_export)
                    continue

                # If a multi-op tool batch failed, check whether Fusion Personal blocked a 3+2/multi-axis op
                batch_hint = _read_post_failure_hint(b_tmp) or (str(p_exc) if p_exc is not None else "")
                if _is_personal_use_multiaxis_post_block(batch_hint):
                    raise _SkipSetupCapability(batch_hint)

                if len(op_list) > 1:
                    recovered_any = False
                    for sub_i, single_op in enumerate(op_list):
                        sub_want_export = bool(is_prev_stock and out_batch_idx == 0)
                        sub_cnc = os.path.join(bundle_dir, f"batch_{out_batch_idx}.cnc")
                        s_jc, s_bt, s_exc = _run_with_base_fallback(
                            [single_op], f"{b_idx}_s{sub_i}", sub_cnc, sub_want_export
                        )
                        if s_jc is not None:
                            _commit_batch(s_jc, s_bt, sub_cnc, sub_want_export)
                            recovered_any = True
                        else:
                            sub_hint = _read_post_failure_hint(s_bt) or (str(s_exc) if s_exc is not None else "")
                            if _is_personal_use_multiaxis_post_block(sub_hint):
                                raise _SkipSetupCapability(sub_hint)
                            if s_exc is not None:
                                last_batch_err = s_exc
                    if recovered_any:
                        continue

                hint = _read_post_failure_hint(b_tmp)
                if not hint and p_exc is not None:
                    hint = str(p_exc)
                if _is_personal_use_multiaxis_post_block(hint):
                    raise _SkipSetupCapability(hint)
                suffix = f" ({hint})" if hint else ""
                last_batch_err = RuntimeError(
                    f"Harvester failed to emit JSON context for batch {b_idx} of '{setup_name}'{suffix}"
                )
                raise last_batch_err

            if not json_batches and last_batch_err is not None:
                raise last_batch_err

        solid_stock_rel: Optional[str] = None
        workpiece_mm_dict: Optional[Dict[str, Any]] = None

        if is_solid_stock:
            from geometry.solid_mesh import extract_solid_workpiece_from_fusion_setup

            solid_3d_wp = extract_solid_workpiece_from_fusion_setup(setup, is_turning=False)
            if solid_3d_wp is not None and getattr(solid_3d_wp, "stock", None) is not None:
                out_solid_stl = os.path.join(bundle_dir, "solid_stock.stl")
                write_binary_stl(out_solid_stl, solid_3d_wp.stock.vertices, solid_3d_wp.stock.indices)
                solid_stock_rel = "solid_stock.stl"
            if is_turning:
                solid_turn_wp = extract_solid_workpiece_from_fusion_setup(setup, is_turning=True)
                workpiece_mm_dict = _serialize_workpiece(solid_turn_wp, kind="solid_turning")
        elif prev_stock_rel:
            turn_wp = PostClear._ExtractWorkpieceFromSetup(setup) if is_turning else None
            prev_fixtures = list(getattr(turn_wp, "fixtures", None) or []) if turn_wp else []
            workpiece_mm_dict = _workpiece_dict_from_stl(
                os.path.join(bundle_dir, prev_stock_rel),
                is_turning=is_turning,
                is_inch=prev_stock_is_inch,
                fixtures=prev_fixtures,
            )
        elif is_turning:
            turn_wp = PostClear._ExtractWorkpieceFromSetup(setup)
            workpiece_mm_dict = _serialize_workpiece(turn_wp, kind="turning")
        elif first_dump_data is not None:
            from context_parser import extract_workpiece_from_context
            box_wp = extract_workpiece_from_context(first_dump_data, is_turning=False)
            workpiece_mm_dict = _serialize_workpiece(box_wp, kind="box")

        wcs_dict = extract_wcs_dict(setup)

        manifest = _build_setup_manifest(
            project_name=project_name,
            doc_name=doc_name,
            setup_name=setup_name,
            setup_index=setup_idx,
            is_turning=bool(is_turning),
            stock_mode=stock_mode,
            unit=first_dump_data.get("unit") if first_dump_data else None,
            stock_unit=stock_unit,
            configured_post=configured_post,
            post_props=post_props,
            cnc_batches=cnc_batches,
            json_batches=json_batches,
            prev_stock_rel=prev_stock_rel,
            prev_stock_is_inch=bool(prev_stock_is_inch),
            solid_stock_rel=solid_stock_rel,
            workpiece_mm_dict=workpiece_mm_dict,
            wcs_dict=wcs_dict,
        )
        with open(manifest_path, "w", encoding="utf-8") as mf:
            json.dump(manifest, mf, indent=2)
    except _SkipSetupCapability:
        shutil.rmtree(bundle_dir, ignore_errors=True)
        return {"status": "skipped_capability", "setup_name": setup_name}
    except Exception:
        if not os.path.isfile(manifest_path):
            shutil.rmtree(bundle_dir, ignore_errors=True)
        raise

    # Link previous setup's post-setup IPW when this setup exports PreviousSetupStock in the same WCS
    if (
        prev_setup_record
        and prev_stock_rel
        and not is_turning
        and _wcs_matches(prev_setup_record.get("wcs"), wcs_dict)
    ):
        prev_bundle = prev_setup_record["bundle_dir"]
        prev_manifest_path = os.path.join(prev_bundle, "setup_manifest.json")
        if os.path.isfile(prev_manifest_path):
            from geometry.solid_mesh import load_stl_mesh
            verts_mm, idxs = load_stl_mesh(os.path.join(bundle_dir, prev_stock_rel), is_inch=prev_stock_is_inch)
            ipw_dest = os.path.join(prev_bundle, "post_setup_ipw.stl")
            write_binary_stl(ipw_dest, verts_mm, idxs, header_text="PostClear Post-Setup IPW (mm)")
            with open(prev_manifest_path, "r", encoding="utf-8") as pmf:
                prev_m = json.load(pmf)
            prev_m["post_setup_ipw_stl"] = "post_setup_ipw.stl"
            with open(prev_manifest_path, "w", encoding="utf-8") as pmf:
                json.dump(prev_m, pmf, indent=2)

    return {
        "status": "harvested",
        "setup_name": manifest["setup_name"],
        "bundle_dir": bundle_dir,
        "manifest_path": manifest_path,
        "wcs": wcs_dict,
    }


def _collect_datafiles_in_folder(folder: Any, project_name: str, folder_rel: str = "") -> List[Tuple[str, str, Any]]:
    items: List[Tuple[str, str, Any]] = []
    for df in getattr(folder, "dataFiles", None) or []:
        ext = str(getattr(df, "fileExtension", "") or "").lstrip(".").lower()
        name = str(getattr(df, "name", "") or "")
        if not ext and "." in name:
            ext = name.rsplit(".", 1)[-1].lower()
        if ext in ("f3d", "f3z"):
            items.append((project_name, os.path.join(folder_rel, name) if folder_rel else name, df))
    for sub in getattr(folder, "dataFolders", None) or []:
        sub_name = _sanitize_name(getattr(sub, "name", "folder"))
        next_rel = os.path.join(folder_rel, sub_name) if folder_rel else sub_name
        items.extend(_collect_datafiles_in_folder(sub, project_name, next_rel))
    return items


def _ensure_setups_generated(cam: Any, PostClear: Any, progress: Any = None, timeout_sec: float = 180.0) -> bool:
    """Generate missing/invalid toolpaths on unsuppressed Milling/Turning setups with cancel + timeout guards."""
    adsk_cam = sys.modules.get("adsk.cam")
    adsk_core = sys.modules.get("adsk.core")
    ops_to_gen: List[Any] = []
    for s in PostClear._extract_cam_collection(getattr(cam, "setups", None)):
        if getattr(s, "isSuppressed", False):
            continue
        if _setup_has_unsupported_capability(s, PostClear, adsk_cam) is not None:
            continue
        ops = PostClear._extract_cam_collection(getattr(s, "allOperations", None))
        for op in ops:
            if getattr(op, "isSuppressed", False):
                continue
            if not _is_supported_cutting_op(op):
                continue
            if not getattr(op, "hasToolpath", True) or not getattr(op, "isToolpathValid", True):
                ops_to_gen.append(op)
    if not ops_to_gen:
        return True

    try:
        gen_fn = getattr(cam, "generateToolpath", None)
        if callable(gen_fn):
            if len(ops_to_gen) == 1:
                future = gen_fn(ops_to_gen[0])
            elif adsk_core is not None and hasattr(adsk_core, "ObjectCollection"):
                col = adsk_core.ObjectCollection.create()
                for op in ops_to_gen:
                    col.add(op)
                future = gen_fn(col)
            else:
                future = cam.generateAllToolpaths(False)
        else:
            future = cam.generateAllToolpaths(False)

        deadline = time.time() + timeout_sec
        while not getattr(future, "isGenerationCompleted", True):
            if time.time() > deadline:
                return False
            if progress and getattr(progress, "wasCancelled", False):
                return False
            if "adsk" in sys.modules and hasattr(sys.modules["adsk"], "doEvents"):
                sys.modules["adsk"].doEvents()
            time.sleep(0.1)
    except Exception:
        return False
    return True


def _load_index(index_path: str) -> Dict[str, Any]:
    if os.path.isfile(index_path):
        try:
            with open(index_path, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            pass
    return {"schema_version": 1, "documents": {}}


def _save_index(index_path: str, index_data: Dict[str, Any]) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(index_path)), exist_ok=True)
    with open(index_path, "w", encoding="utf-8") as f:
        json.dump(index_data, f, indent=2)


def _harvest_document(
    doc: Any,
    doc_key: str,
    project_name: str,
    doc_name: str,
    corpus_dir: str,
    progress: Any = None,
) -> Dict[str, Any]:
    import adsk.cam

    PostClear = _get_postclear_module()
    products = getattr(doc, "products", None)
    cam_prod = products.itemByProductType("CAMProductType") if products else None
    cam = adsk.cam.CAM.cast(cam_prod) if hasattr(adsk.cam.CAM, "cast") else cam_prod
    setups_list = PostClear._extract_cam_collection(getattr(cam, "setups", None)) if cam is not None else []
    if cam is None or not setups_list:
        return {"status": "no_cam", "setups": [], "errors": []}

    _ensure_setups_generated(cam, PostClear=PostClear, progress=progress)

    doc_dir = os.path.join(
        corpus_dir,
        _sanitize_name(project_name, "OpenDocs"),
        _sanitize_name(doc_name, "Untitled"),
    )
    results: List[Dict[str, Any]] = []
    setup_errors: List[str] = []
    prev_record: Optional[Dict[str, Any]] = None

    for s_idx, setup in enumerate(setups_list):
        if getattr(setup, "isSuppressed", False):
            continue
        s_name = getattr(setup, "name", f"Setup_{s_idx}")
        setup_slug = f"{s_idx:02d}_{_sanitize_name(s_name)}"
        bundle_dir = os.path.join(doc_dir, setup_slug)
        try:
            res = harvest_setup(
                doc=doc,
                cam=cam,
                setup=setup,
                setup_idx=s_idx,
                bundle_dir=bundle_dir,
                project_name=project_name,
                doc_name=doc_name,
                prev_setup_record=prev_record,
            )
            results.append(res)
            if res.get("status") == "harvested":
                prev_record = res
            else:
                prev_record = None
        except Exception as exc:
            prev_record = None
            results.append({"status": "failed", "setup_name": s_name, "error": str(exc)})
            setup_errors.append(f"{doc_name}/{s_name}: {exc}")

    harvested_count = sum(1 for r in results if r.get("status") == "harvested")
    if harvested_count == 0 and os.path.isdir(doc_dir) and not os.listdir(doc_dir):
        try:
            os.rmdir(doc_dir)
        except OSError:
            pass

    if harvested_count > 0:
        doc_status = "harvested"
    elif setup_errors:
        doc_status = "failed"
    elif any(r.get("status") == "skipped_capability" for r in results):
        doc_status = "skipped_capability"
    else:
        doc_status = "no_active_ops"

    return {
        "status": doc_status,
        "setups": results,
        "errors": setup_errors,
    }


def harvest_projects(
    app: Any,
    ui: Any,
    scope_choice: str,
    corpus_dir: Optional[str] = None,
    force: bool = False,
    local_folder: Optional[str] = None,
) -> Dict[str, Any]:
    """
    Crawl the chosen Fusion 360 scope (`"1"` open docs, `"2"` active project,
    `"3"` all hub projects, `"4"` local archive folder) and harvest CAM setups.
    """
    target_corpus = os.path.abspath(corpus_dir or os.path.join(REPO_ROOT, "corpus"))
    os.makedirs(target_corpus, exist_ok=True)
    index_path = os.path.join(target_corpus, "harvest_index.json")
    index_data = _load_index(index_path)

    stats: Dict[str, Any] = {
        "corpus_dir": target_corpus,
        "docs_processed": 0,
        "docs_skipped_cached": 0,
        "docs_no_cam": 0,
        "docs_failed": 0,
        "setups_harvested": 0,
        "cancelled": False,
        "errors": [],
    }

    open_by_id: Dict[str, Any] = {}
    for d in getattr(app, "documents", None) or []:
        df = getattr(d, "dataFile", None)
        df_id = getattr(df, "id", None)
        if df_id:
            open_by_id[df_id] = d

    progress = None
    if ui and hasattr(ui, "createProgressDialog"):
        try:
            progress = ui.createProgressDialog()
            progress.isCancelButtonShown = True
        except Exception:
            progress = None

    cached_ok_statuses = ("harvested", "no_cam", "no_active_ops", "skipped_capability")

    try:
        scope = str(scope_choice).strip()
        if scope == "1":
            docs = list(getattr(app, "documents", None) or [])
            if progress:
                progress.show("PostClear Harvester", f"Scanning {len(docs)} open document(s)...", 0, max(1, len(docs)), 0)
            for idx, doc in enumerate(docs):
                if progress and progress.wasCancelled:
                    stats["cancelled"] = True
                    break
                doc_name = getattr(doc, "name", f"OpenDoc_{idx}")
                if progress:
                    progress.message = f"Harvesting {doc_name} ({idx + 1}/{len(docs)})..."
                    progress.progressValue = idx
                try:
                    doc.activate()
                    res = _harvest_document(doc, f"open:{doc_name}", "OpenDocs", doc_name, target_corpus, progress)
                    stats["docs_processed"] += 1
                    h_cnt = sum(1 for r in res["setups"] if r.get("status") == "harvested")
                    stats["setups_harvested"] += h_cnt
                    if res.get("errors"):
                        stats["docs_failed"] += 1
                        stats["errors"].extend(res["errors"])
                    elif res["status"] != "harvested":
                        stats["docs_no_cam"] += 1
                except Exception as exc:
                    stats["docs_failed"] += 1
                    stats["errors"].append(f"{doc_name}: {exc}")

        elif scope in ("2", "3"):
            data_mgr = getattr(app, "data", None)
            projects: List[Any] = []
            if scope == "2":
                active_proj = getattr(data_mgr, "activeProject", None)
                if active_proj is not None:
                    projects.append(active_proj)
            else:
                active_hub = getattr(data_mgr, "activeHub", None)
                hub_projs = getattr(active_hub, "dataProjects", None) or getattr(data_mgr, "dataProjects", None) or []
                projects = list(hub_projs)

            all_files: List[Tuple[str, str, Any]] = []
            for proj in projects:
                p_name = getattr(proj, "name", "Project")
                root_folder = getattr(proj, "rootFolder", None)
                if root_folder is not None:
                    all_files.extend(_collect_datafiles_in_folder(root_folder, p_name))

            if progress:
                progress.show("PostClear Harvester", f"Scanning {len(all_files)} Fusion file(s)...", 0, max(1, len(all_files)), 0)

            for idx, (proj_name, rel_name, df) in enumerate(all_files):
                if progress and progress.wasCancelled:
                    stats["cancelled"] = True
                    break
                if progress:
                    progress.message = f"[{idx + 1}/{len(all_files)}] {proj_name}/{rel_name}"
                    progress.progressValue = idx

                df_id = getattr(df, "id", None) or f"{proj_name}:{rel_name}"
                ver = getattr(df, "versionNumber", 0)
                cached = index_data["documents"].get(df_id)
                if (
                    not force
                    and isinstance(cached, dict)
                    and cached.get("version") == ver
                    and cached.get("status") in cached_ok_statuses
                ):
                    stats["docs_skipped_cached"] += 1
                    continue

                was_open = df_id in open_by_id
                doc = open_by_id.get(df_id)
                opened_here = False
                try:
                    if doc is None:
                        doc = app.documents.open(df, True)
                        opened_here = True
                    else:
                        doc.activate()

                    res = _harvest_document(
                        doc=doc,
                        doc_key=df_id,
                        project_name=proj_name,
                        doc_name=rel_name.rsplit(".", 1)[0],
                        corpus_dir=target_corpus,
                        progress=progress,
                    )
                    stats["docs_processed"] += 1
                    h_cnt = sum(1 for r in res["setups"] if r.get("status") == "harvested")
                    stats["setups_harvested"] += h_cnt
                    if res.get("errors"):
                        stats["docs_failed"] += 1
                        stats["errors"].extend(f"{proj_name}/{e}" for e in res["errors"])
                    elif res["status"] != "harvested":
                        stats["docs_no_cam"] += 1
                    if not res.get("errors"):
                        index_data["documents"][df_id] = {
                            "version": ver,
                            "project": proj_name,
                            "name": rel_name,
                            "status": res["status"],
                            "harvested_setups": h_cnt,
                            "updated_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
                        }
                        _save_index(index_path, index_data)
                except Exception as exc:
                    stats["docs_failed"] += 1
                    stats["errors"].append(f"{proj_name}/{rel_name}: {exc}")
                finally:
                    if opened_here and not was_open and doc is not None:
                        try:
                            doc.close(False)
                        except Exception:
                            pass

        elif scope == "4":
            folder_path = local_folder
            if not folder_path and ui and hasattr(ui, "createFolderDialog"):
                fd = ui.createFolderDialog()
                fd.title = "Select folder containing .f3d / .f3z files"
                import adsk.core
                ok_val = getattr(getattr(adsk.core, "DialogResults", None), "DialogOK", 0)
                if fd.showDialog() in (ok_val, 0):
                    folder_path = fd.folder
            if folder_path and os.path.isdir(folder_path):
                local_files: List[str] = []
                for dp, _, fns in os.walk(folder_path):
                    for fn in sorted(fns):
                        if fn.lower().endswith((".f3d", ".f3z")):
                            local_files.append(os.path.join(dp, fn))
                if progress:
                    progress.show("PostClear Harvester", f"Importing {len(local_files)} archive(s)...", 0, max(1, len(local_files)), 0)
                for idx, fpath in enumerate(local_files):
                    if progress and progress.wasCancelled:
                        stats["cancelled"] = True
                        break
                    rel_f = os.path.relpath(fpath, folder_path)
                    if progress:
                        progress.message = f"[{idx + 1}/{len(local_files)}] {rel_f}"
                        progress.progressValue = idx
                    st = os.stat(fpath)
                    ver_tag = f"{st.st_mtime_ns}:{st.st_size}"
                    doc_key = f"local:{os.path.abspath(fpath)}"
                    cached = index_data["documents"].get(doc_key)
                    if (
                        not force
                        and isinstance(cached, dict)
                        and cached.get("version") == ver_tag
                        and cached.get("status") in cached_ok_statuses
                    ):
                        stats["docs_skipped_cached"] += 1
                        continue
                    doc = None
                    try:
                        imp_mgr = app.importManager
                        opts = imp_mgr.createFusionArchiveImportOptions(fpath)
                        doc = imp_mgr.importToNewDocument(opts)
                        res = _harvest_document(
                            doc=doc,
                            doc_key=doc_key,
                            project_name="LocalArchives",
                            doc_name=os.path.splitext(rel_f)[0],
                            corpus_dir=target_corpus,
                            progress=progress,
                        )
                        stats["docs_processed"] += 1
                        h_cnt = sum(1 for r in res["setups"] if r.get("status") == "harvested")
                        stats["setups_harvested"] += h_cnt
                        if res.get("errors"):
                            stats["docs_failed"] += 1
                            stats["errors"].extend(res["errors"])
                        elif res["status"] != "harvested":
                            stats["docs_no_cam"] += 1
                        if not res.get("errors"):
                            index_data["documents"][doc_key] = {
                                "version": ver_tag,
                                "project": "LocalArchives",
                                "name": rel_f,
                                "status": res["status"],
                                "harvested_setups": h_cnt,
                            }
                            _save_index(index_path, index_data)
                    except Exception as exc:
                        stats["docs_failed"] += 1
                        stats["errors"].append(f"{rel_f}: {exc}")
                    finally:
                        if doc is not None:
                            try:
                                doc.close(False)
                            except Exception:
                                pass
        else:
            raise ValueError(f"Unknown harvest scope choice: {scope_choice!r}")
    finally:
        if progress:
            try:
                progress.hide()
            except Exception:
                pass

    return stats


def run(context: Any) -> None:
    """Fusion 360 Script entrypoint."""
    ui = None
    try:
        import adsk.core
        app = adsk.core.Application.get()
        ui = app.userInterface

        prompt = (
            "Choose harvest scope (append ! to force re-export cached versions):\n\n"
            "  1 = Currently open document(s) only\n"
            "  2 = Active Data Panel Project\n"
            "  3 = All Projects in Active Hub (recommended)\n"
            "  4 = Local folder of .f3d / .f3z files"
        )
        raw_choice, cancelled = ui.inputBox(prompt, "PostClear Corpus Harvester", "3")
        if cancelled:
            return
        choice_str = str(raw_choice or "").strip()
        force = choice_str.endswith("!") or ("force" in choice_str.lower())
        scope_digit = re.sub(r"[^1-4]", "", choice_str)[:1] or "3"

        stats = harvest_projects(app=app, ui=ui, scope_choice=scope_digit, force=force)
        err_note = ""
        if stats["errors"]:
            sample_errs = "\n".join(f"  - {e}" for e in stats["errors"][:5])
            err_note = f"\n\nErrors ({stats['docs_failed']}):\n{sample_errs}"

        summary = (
            f"Corpus harvest {'cancelled' if stats['cancelled'] else 'complete'}.\n\n"
            f"Setups exported: {stats['setups_harvested']}\n"
            f"Documents processed: {stats['docs_processed']}\n"
            f"Documents skipped (cached): {stats['docs_skipped_cached']}\n"
            f"Documents without active CAM: {stats['docs_no_cam']}\n"
            f"Failures: {stats['docs_failed']}\n\n"
            f"Output folder:\n  {stats['corpus_dir']}\n\n"
            f"Run offline matrix test in terminal:\n"
            f"  {os.path.join(REPO_ROOT, '.venv', 'bin', 'python3')} "
            f"{os.path.join(REPO_ROOT, 'scripts', 'run_corpus.py')}"
            f"{err_note}"
        )
        ui.messageBox(summary, "PostClear Corpus Harvester")
    except Exception:
        if ui:
            ui.messageBox(f"HarvestCorpus failed:\n{traceback.format_exc()}", "PostClear Corpus Harvester")


def _find_sibling_manifest(parent_dir: str) -> Optional[Dict[str, Any]]:
    """Return the first readable sibling `setup_manifest.json` under `parent_dir`, if any."""
    if not parent_dir or not os.path.isdir(parent_dir):
        return None
    for entry in sorted(os.listdir(parent_dir)):
        cand = os.path.join(parent_dir, entry, "setup_manifest.json")
        if os.path.isfile(cand):
            try:
                with open(cand, "r", encoding="utf-8") as mf:
                    data = json.load(mf)
                if isinstance(data, dict):
                    return data
            except Exception:
                continue
    return None


def _collect_contiguous_batches(dirpath: str, filenames: Sequence[str]) -> Optional[Tuple[List[str], List[str]]]:
    """Collect matching `batch_0..N-1.{cnc,json}` files from a bundle directory."""
    cnc_map: Dict[int, str] = {}
    json_map: Dict[int, str] = {}
    for fn in filenames:
        mc = re.match(r"^batch_(\d+)\.cnc$", fn)
        if mc:
            cnc_map[int(mc.group(1))] = fn
            continue
        mj = re.match(r"^batch_(\d+)\.json$", fn)
        if mj:
            json_map[int(mj.group(1))] = fn
    if not cnc_map or set(cnc_map.keys()) != set(json_map.keys()):
        return None
    expected = set(range(len(cnc_map)))
    if set(cnc_map.keys()) != expected:
        return None
    n = len(cnc_map)
    return [cnc_map[i] for i in range(n)], [json_map[i] for i in range(n)]


def recover_incomplete_bundles(corpus_dir: str) -> List[str]:
    """
    Synthesize `setup_manifest.json` and sanitize stale temp `stockPath` values
    for setup bundles that captured `batch_*.cnc` / `batch_*.json` (and optional STLs)
    but aborted before writing their manifest.

    Returns a sorted list of newly written `setup_manifest.json` paths.
    """
    from context_parser import extract_workpiece_from_context

    corpus_abs = os.path.abspath(corpus_dir)
    if not os.path.isdir(corpus_abs):
        return []

    recovered: List[str] = []
    for dirpath, dirs, files in os.walk(corpus_abs):
        dirs.sort()
        files.sort()
        if "setup_manifest.json" in files:
            continue
        if "batch_0.cnc" not in files or "batch_0.json" not in files:
            continue

        batch_pair = _collect_contiguous_batches(dirpath, files)
        if batch_pair is None:
            continue
        cnc_batches, json_batches = batch_pair

        sibling_manifest = _find_sibling_manifest(os.path.dirname(dirpath))
        rel_parts = os.path.relpath(dirpath, corpus_abs).split(os.sep)
        fallback_proj = rel_parts[0] if len(rel_parts) >= 3 else "Recovered"
        fallback_doc = (
            "/".join(rel_parts[1:-1])
            if len(rel_parts) >= 3
            else (rel_parts[0] if len(rel_parts) == 2 else "Untitled")
        )
        project_name = str((sibling_manifest or {}).get("project_name") or fallback_proj)
        doc_name = str((sibling_manifest or {}).get("document_name") or fallback_doc)

        setup_slug = os.path.basename(dirpath)
        slug_match = re.match(r"^(\d+)_(.+)$", setup_slug)
        if slug_match:
            setup_index = int(slug_match.group(1))
            setup_name = slug_match.group(2)
        else:
            setup_index = 0
            setup_name = setup_slug

        all_dumps: List[Dict[str, Any]] = []
        for jb in json_batches:
            with open(os.path.join(dirpath, jb), "r", encoding="utf-8") as jf:
                all_dumps.append(json.load(jf))
        first_dump_data = all_dumps[0]

        is_turning = any(
            sec.get("type") == 1 or bool((sec.get("tool") or {}).get("_isTurningTool", False))
            for dump in all_dumps
            for sec in (dump.get("sections") or [])
        )
        from context_parser import check_context_dict_capability

        if any(
            check_context_dict_capability(
                dump,
                setup_name=setup_name,
                is_turning=is_turning,
                check_probing=False,
            )
            for dump in all_dumps
        ):
            continue
        unit = first_dump_data.get("unit")
        stock_unit = first_dump_data.get("stockUnit")
        post_props = dict(first_dump_data.get("postProperties") or {})

        if "SB3v36" in post_props:
            configured_post: Optional[str] = "shopbot.cps"
        elif "smartCoolEquipped" in post_props or "useG64" in post_props:
            configured_post = "tormach.cps"
        elif sibling_manifest and sibling_manifest.get("configured_post"):
            configured_post = str(sibling_manifest["configured_post"])
        else:
            configured_post = "centroid_turning.cps" if is_turning else "grbl.cps"

        batch0_path = os.path.join(dirpath, json_batches[0])
        prev_stl_abs = os.path.join(dirpath, "prev_stock.stl")
        if os.path.isfile(prev_stl_abs):
            prev_stock_rel: Optional[str] = "prev_stock.stl"
            prev_stock_is_inch = detect_stl_is_inch(prev_stl_abs, first_dump_data)
            _rewrite_dump_stock_paths(first_dump_data, prev_stock_rel, batch0_path)
        else:
            prev_stock_rel = None
            prev_stock_is_inch = False
            _rewrite_dump_stock_paths(first_dump_data, None, batch0_path)

        solid_stl_abs = os.path.join(dirpath, "solid_stock.stl")
        solid_stock_rel: Optional[str] = "solid_stock.stl" if os.path.isfile(solid_stl_abs) else None
        stock_mode = str(
            first_dump_data.get("stockMode")
            or ("previous-setup" if prev_stock_rel else ("solid" if solid_stock_rel else "default"))
        )

        if prev_stock_rel:
            workpiece_mm_dict = _workpiece_dict_from_stl(
                prev_stl_abs,
                is_turning=is_turning,
                is_inch=prev_stock_is_inch,
            )
        elif solid_stock_rel:
            workpiece_mm_dict = _workpiece_dict_from_stl(
                solid_stl_abs,
                is_turning=is_turning,
                is_inch=False,
                kind_override="solid_turning" if is_turning else "box",
            )
        else:
            wp = extract_workpiece_from_context(first_dump_data, is_turning=is_turning)
            workpiece_mm_dict = _serialize_workpiece(wp, kind="turning" if is_turning else "box")

        manifest = _build_setup_manifest(
            project_name=project_name,
            doc_name=doc_name,
            setup_name=setup_name,
            setup_index=setup_index,
            is_turning=is_turning,
            stock_mode=stock_mode,
            unit=unit,
            stock_unit=stock_unit,
            configured_post=configured_post,
            post_props=post_props,
            cnc_batches=cnc_batches,
            json_batches=json_batches,
            prev_stock_rel=prev_stock_rel,
            prev_stock_is_inch=prev_stock_is_inch,
            solid_stock_rel=solid_stock_rel,
            workpiece_mm_dict=workpiece_mm_dict,
            wcs_dict=None,
        )
        manifest_path = os.path.join(dirpath, "setup_manifest.json")
        with open(manifest_path, "w", encoding="utf-8") as mf:
            json.dump(manifest, mf, indent=2)
        recovered.append(manifest_path)

    return recovered


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="PostClear Corpus Harvester utilities")
    parser.add_argument(
        "--recover",
        metavar="CORPUS_DIR",
        help="Recover incomplete corpus bundles missing setup_manifest.json",
    )
    args = parser.parse_args(argv)
    if args.recover:
        manifests = recover_incomplete_bundles(args.recover)
        print(f"Recovered {len(manifests)} setup manifest(s):")
        for mp in manifests:
            print(f"  - {mp}")
        return 0
    parser.print_help()
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
