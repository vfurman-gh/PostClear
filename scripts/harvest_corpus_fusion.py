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
    import numpy as np
    from geometry.solid_mesh import load_stl_mesh

    stock_unit_str = dump_data.get("stockUnit")
    if stock_unit_str:
        is_inch = str(stock_unit_str).lower() in ("in", "inch", "inches")
    else:
        is_inch = (dump_data.get("unit", 0) == 0)

    hint = dump_data.get("workpieceHint")
    if not (isinstance(hint, dict) and "lower" in hint and "upper" in hint):
        return is_inch

    raw_verts, _ = load_stl_mesh(stl_path, is_inch=False)
    stl_extents = raw_verts.max(axis=0) - raw_verts.min(axis=0)
    hl, hu = hint["lower"], hint["upper"]
    hint_extents = np.array([
        abs(float(hu.get("x", 0.0)) - float(hl.get("x", 0.0))),
        abs(float(hu.get("y", 0.0)) - float(hl.get("y", 0.0))),
        abs(float(hu.get("z", 0.0)) - float(hl.get("z", 0.0))),
    ], dtype=np.float64)
    active = (hint_extents > 1e-4) & (stl_extents > 1e-4)
    if not np.any(active):
        return is_inch

    best_scale = 1.0
    min_err = float("inf")
    for s in (1.0, 25.4, 1.0 / 25.4):
        scaled = stl_extents[active] * s
        target = hint_extents[active]
        rel = np.abs(scaled - target) / np.maximum(target, 1e-6)
        err = float(np.min(rel)) if np.all(scaled <= target * 1.05) else float(np.max(rel))
        if err < min_err:
            min_err = err
            best_scale = s

    if abs(best_scale - 25.4) < 1e-3:
        return True
    if abs(best_scale - (1.0 / 25.4)) < 1e-3:
        return False
    return bool(dump_data.get("unit", 0) == 0)


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

    from scripts.run_corpus import get_post_search_dirs, resolve_post_file
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


def _get_op_tool_number(op: Any) -> int:
    try:
        tool = getattr(op, "tool", None)
        params = getattr(tool, "parameters", None) if tool else None
        p = params.itemByName("tool_number") if params else None
        val = getattr(getattr(p, "value", None), "value", None)
        if val is not None:
            return int(val)
    except Exception:
        pass
    return 0


def _partition_tool_batches(setup: Any, PostClear: Any) -> List[Tuple[int, List[Any]]]:
    cand_list = PostClear._extract_cam_collection(getattr(setup, "allOperations", []))
    active_ops = [
        op for op in cand_list
        if not getattr(op, "isSuppressed", False)
        and getattr(op, "hasToolpath", True)
        and getattr(op, "isToolpathValid", True)
    ]
    batches: List[Tuple[int, List[Any]]] = []
    i = 0
    while i < len(active_ops):
        op = active_ops[i]
        cur_tool = _get_op_tool_number(op)
        op_list = [op]
        i += 1
        while i < len(active_ops):
            next_op = active_ops[i]
            t_num = _get_op_tool_number(next_op)
            if t_num != cur_tool:
                break
            op_list.append(next_op)
            i += 1
        batches.append((cur_tool, op_list))
    return batches


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

    PostClear = _get_postclear_module()
    is_turning = (getattr(setup, "operationType", None) == adsk.cam.OperationTypes.TurningOperation)
    stock_mode = PostClear._GetSetupStockMode(setup)
    batches = _partition_tool_batches(setup, PostClear)
    if not batches:
        return {"status": "no_active_ops", "setup_name": getattr(setup, "name", f"Setup_{setup_idx}")}

    prog_obj, dump_post_path, post_props = _find_nc_program_for_setup(cam, setup, is_turning, PostClear)
    configured_post = os.path.basename(dump_post_path) if prog_obj is not None else None

    os.makedirs(bundle_dir, exist_ok=True)

    enum_cls = getattr(adsk.cam, "PostOutputUnitOptions", None)
    mm_opt = getattr(enum_cls, "MillimetersOutput", 2) if enum_cls else 2
    in_opt = getattr(enum_cls, "InchesOutput", 1) if enum_cls else 1
    try:
        doc_settings = PostClear.loadSettings(doc) if hasattr(PostClear, "loadSettings") else {}
    except Exception:
        doc_settings = {}
    units = PostClear._GetNCProgramOutputUnits(prog_obj, docSettings=doc_settings)
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
        for b_idx, (_t_num, op_list) in enumerate(batches):
            b_tmp = os.path.join(tmp_root, f"b_{b_idx}")
            os.makedirs(b_tmp, exist_ok=True)
            num_name = str(1001 + b_idx)
            target_cnc = os.path.join(bundle_dir, f"batch_{b_idx}.cnc")
            want_export = bool(is_prev_stock and b_idx == 0)

            cfg_path = PostClear._WriteDumperConfig(
                base_post_path=dump_post_path,
                target_cnc_path=target_cnc,
                post_properties=post_props,
                target_dir=b_tmp,
                stock_mode=stock_mode,
                unit=dumper_unit,
                stock_unit=stock_unit,
                set_env=False,
                export_stock=want_export,
            )

            dumper_cps = PostClear.constDumperScript
            if want_export:
                staged = os.path.join(b_tmp, "universal_dumper_export_stock.cps")
                shutil.copy2(PostClear.constDumperScript, staged)
                dumper_cps = staged

            if len(op_list) == 1:
                target_ops = op_list[0]
            else:
                target_ops = adsk.core.ObjectCollection.create()
                for op in op_list:
                    target_ops.add(op)

            env_updates = {
                "BASE_POST": os.path.abspath(dump_post_path),
                "TARGET_INTERMEDIATE_CNC": os.path.abspath(target_cnc),
                "DUMPER_CONFIG_PATH": cfg_path,
            }
            with PostClear._ScopedEnv(env_updates):
                post_input = adsk.cam.PostProcessInput.create(num_name, dumper_cps, b_tmp, units)
                post_input.isOpenInEditor = False
                cam.postProcess(target_ops, post_input)

            json_cand = os.path.join(b_tmp, f"{num_name}.json")
            if not os.path.isfile(json_cand):
                pattern = re.compile(rf"^{re.escape(num_name)}(_\d+)?\.json$")
                for fn in sorted(os.listdir(b_tmp)):
                    if pattern.match(fn):
                        json_cand = os.path.join(b_tmp, fn)
                        break
            if not os.path.isfile(json_cand):
                raise RuntimeError(f"Harvester failed to emit JSON context for batch {b_idx} of '{setup.name}'")

            out_json = os.path.join(bundle_dir, f"batch_{b_idx}.json")
            shutil.copy2(json_cand, out_json)
            json_batches.append(f"batch_{b_idx}.json")

            if not os.path.isfile(target_cnc):
                fallback_cnc = json_cand + ".intermediate.cnc"
                if os.path.isfile(fallback_cnc):
                    shutil.copy2(fallback_cnc, target_cnc)
            if not os.path.isfile(target_cnc):
                raise RuntimeError(f"Harvester failed to capture .cnc intermediate file for batch {b_idx} of '{setup.name}'")
            cnc_batches.append(f"batch_{b_idx}.cnc")

            with open(out_json, "r", encoding="utf-8") as jf:
                dump_data = json.load(jf)
            if first_dump_data is None:
                first_dump_data = dump_data

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
    elif is_turning:
        turn_wp = PostClear._ExtractWorkpieceFromSetup(setup)
        workpiece_mm_dict = _serialize_workpiece(turn_wp, kind="turning")
    elif first_dump_data is not None:
        from context_parser import extract_workpiece_from_context
        box_wp = extract_workpiece_from_context(first_dump_data, is_turning=False)
        workpiece_mm_dict = _serialize_workpiece(box_wp, kind="box")

    wcs_dict = extract_wcs_dict(setup)

    manifest = {
        "schema_version": 1,
        "harvested_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "project_name": project_name,
        "document_name": doc_name,
        "setup_name": getattr(setup, "name", f"Setup_{setup_idx}"),
        "setup_index": setup_idx,
        "is_turning": bool(is_turning),
        "stock_mode": stock_mode,
        "unit": first_dump_data.get("unit") if first_dump_data else None,
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
    manifest_path = os.path.join(bundle_dir, "setup_manifest.json")
    with open(manifest_path, "w", encoding="utf-8") as mf:
        json.dump(manifest, mf, indent=2)

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
    """Generate missing/invalid toolpaths on unsuppressed CAM setups with cancel + timeout guards."""
    needs_gen = False
    for s in PostClear._extract_cam_collection(getattr(cam, "setups", None)):
        if getattr(s, "isSuppressed", False):
            continue
        ops = PostClear._extract_cam_collection(getattr(s, "allOperations", None))
        for op in ops:
            if getattr(op, "isSuppressed", False):
                continue
            if not getattr(op, "hasToolpath", True) or not getattr(op, "isToolpathValid", True):
                needs_gen = True
                break
        if needs_gen:
            break
    if not needs_gen:
        return True

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
        return {"status": "no_cam", "setups": []}

    _ensure_setups_generated(cam, PostClear=PostClear, progress=progress)

    doc_dir = os.path.join(
        corpus_dir,
        _sanitize_name(project_name, "OpenDocs"),
        _sanitize_name(doc_name, "Untitled"),
    )
    results: List[Dict[str, Any]] = []
    prev_record: Optional[Dict[str, Any]] = None

    for s_idx, setup in enumerate(setups_list):
        if getattr(setup, "isSuppressed", False):
            continue
        setup_slug = f"{s_idx:02d}_{_sanitize_name(getattr(setup, 'name', f'Setup_{s_idx}'))}"
        bundle_dir = os.path.join(doc_dir, setup_slug)
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

    harvested_count = sum(1 for r in results if r.get("status") == "harvested")
    return {
        "status": "harvested" if harvested_count > 0 else "no_active_ops",
        "setups": results,
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
                    if res["status"] == "harvested":
                        stats["setups_harvested"] += sum(1 for r in res["setups"] if r.get("status") == "harvested")
                    else:
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
                    and cached.get("status") in ("harvested", "no_cam", "no_active_ops")
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
                    if res["status"] != "harvested":
                        stats["docs_no_cam"] += 1
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
                        and cached.get("status") in ("harvested", "no_cam", "no_active_ops")
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
                        if res["status"] != "harvested":
                            stats["docs_no_cam"] += 1
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
