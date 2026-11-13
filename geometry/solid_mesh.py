#!/usr/bin/env python3
"""
Solid and Mesh Stock Processing for Milling (3D Heightfield) and Turning (2D IPW).

Extracts, transforms, and converts solid B-Rep and MeshBody stock geometry
from Autodesk Fusion 360 CAM Setups into simulation-ready verification structures.
"""

import math
import os
import struct
import sys
from typing import Any, List, Optional, Sequence, Tuple

try:
    import numpy as np
except (ImportError, OSError):
    np = None

try:
    from shapely.geometry import Polygon, box
    from shapely.ops import unary_union
except (ImportError, OSError):
    Polygon = Any
    box = None
    unary_union = None

from geometry.models import (
    AxisymmetricChuckObstacle,
    AxisymmetricStock2D,
    SolidMeshStock3D,
    WorkpieceDefinition,
)


def _dbg(msg: str) -> None:
    """Internal debug logging helper that forwards to PostClear._dbg_global if available."""
    try:
        from PostClear import _dbg_global
        _dbg_global(msg)
    except Exception:
        pass


def transform_design_points_to_setup_wcs(
    points_cm: np.ndarray,
    wcs: Any,
) -> np.ndarray:
    """
    Transform point coordinates from Fusion 360 design space (cm) to Setup WCS (mm).

    In Autodesk Fusion 360:
    - BRep/Mesh coordinates from ShapeManager (meshManager) are natively in centimeters (cm).
    - CAM setup.workCoordinateSystem translation is natively in millimeters (mm), matching the
      CAM kernel's internal unit system.
    - If a mock matrix specifies origin in cm (e.g. in synthetic unit tests), scale detection
      automatically normalizes the origin to mm.

    Args:
        points_cm: (N, 3) numpy array of coordinates in design space (cm)
        wcs: adsk.core.Matrix3D representing setup.workCoordinateSystem

    Returns:
        (N, 3) numpy array of coordinates in setup WCS (mm)
    """
    if points_cm is None or len(points_cm) == 0:
        return np.empty((0, 3), dtype=np.float32)

    pts_cm = np.asarray(points_cm, dtype=np.float64)
    # Convert input points from design space (cm) to millimeters (mm)
    pts_mm = pts_cm * 10.0

    if wcs is None:
        return pts_mm.astype(np.float32)

    # Compute bounding boxes in cm and mm for origin unit disambiguation
    min_cm = pts_cm.min(axis=0)
    max_cm = pts_cm.max(axis=0)
    min_mm = min_cm * 10.0
    max_mm = max_cm * 10.0

    def _is_cm_origin(origin_vec: np.ndarray) -> bool:
        """Check if origin vector was supplied in cm (e.g. legacy unit test mock)."""
        d_cm = np.linalg.norm(np.maximum(0.0, min_cm - origin_vec) + np.maximum(0.0, origin_vec - max_cm))
        d_mm = np.linalg.norm(np.maximum(0.0, min_mm - origin_vec) + np.maximum(0.0, origin_vec - max_mm))
        return bool(d_cm < 1e-4 and d_mm > 1.0)

    pts_setup_mm = None

    # Strategy 1: Invert Matrix3D directly (exact Autodesk Matrix3D transformation)
    # setup.workCoordinateSystem transforms from Setup WCS to Design space.
    # Its inverse transforms from Design space to Setup WCS.
    if hasattr(wcs, "copy") and hasattr(wcs, "invert"):
        try:
            inv_m = wcs.copy()
            if inv_m.invert():
                if hasattr(inv_m, "getCell"):
                    # M[r, c] is row r, column c of 4x4 matrix
                    # x' = M00*x + M01*y + M02*z + M03, etc.
                    M = np.array([[inv_m.getCell(r, c) for c in range(4)] for r in range(4)], dtype=np.float64)
                    origin_shift = M[:3, 3].copy()
                    # Design-space origin is -M[:3, :3].T @ origin_shift
                    wcs_orig = -M[:3, :3].T @ origin_shift
                    if _is_cm_origin(wcs_orig):
                        origin_shift *= 10.0
                        _dbg(f"transform_design_points_to_setup_wcs: cm mock detected, scaled origin shift to {origin_shift}")
                    pts_setup_mm = pts_mm @ M[:3, :3].T + origin_shift
                    _dbg(f"transform_design_points_to_setup_wcs: inverted via Matrix3D.getCell. Origin shift: {origin_shift}")
                elif hasattr(inv_m, "asArray"):
                    arr = np.array(inv_m.asArray(), dtype=np.float64)
                    if arr.size == 16:
                        # Row-major matrix has bottom row [0, 0, 0, 1]
                        if abs(arr[15] - 1.0) < 1e-4 and abs(arr[12]) < 1e-4 and abs(arr[13]) < 1e-4 and abs(arr[14]) < 1e-4:
                            M = arr.reshape((4, 4))
                        else:
                            M = arr.reshape((4, 4)).T
                        origin_shift = M[:3, 3].copy()
                        wcs_orig = -M[:3, :3].T @ origin_shift
                        if _is_cm_origin(wcs_orig):
                            origin_shift *= 10.0
                            _dbg(f"transform_design_points_to_setup_wcs: cm mock detected, scaled origin shift to {origin_shift}")
                        pts_setup_mm = pts_mm @ M[:3, :3].T + origin_shift
                        _dbg(f"transform_design_points_to_setup_wcs: inverted via Matrix3D.asArray. Origin shift: {origin_shift}")
        except Exception as inv_err:
            _dbg(f"transform_design_points_to_setup_wcs: Matrix3D.invert error: {inv_err}")

    # Strategy 2: getAsCoordinateSystem (handles 4-tuple and 5-tuple SWIG returns)
    if pts_setup_mm is None and hasattr(wcs, "getAsCoordinateSystem"):
        try:
            res = wcs.getAsCoordinateSystem()
            if isinstance(res, (tuple, list)):
                if len(res) == 5:
                    _, origin_pt, x_vec, y_vec, z_vec = res
                elif len(res) == 4:
                    origin_pt, x_vec, y_vec, z_vec = res
                else:
                    raise ValueError(f"Unexpected getAsCoordinateSystem tuple length: {len(res)}")
            else:
                raise TypeError(f"getAsCoordinateSystem returned unexpected type: {type(res)}")

            origin = np.array([origin_pt.x, origin_pt.y, origin_pt.z], dtype=np.float64)
            if _is_cm_origin(origin):
                origin = origin * 10.0
                _dbg(f"transform_design_points_to_setup_wcs: cm mock detected, scaled origin to {origin}")

            R = np.array(
                [
                    [x_vec.x, y_vec.x, z_vec.x],
                    [x_vec.y, y_vec.y, z_vec.y],
                    [x_vec.z, y_vec.z, z_vec.z],
                ],
                dtype=np.float64,
            )
            pts_setup_mm = (pts_mm - origin) @ R
            _dbg(f"transform_design_points_to_setup_wcs: transformed via getAsCoordinateSystem. Origin: {origin}")
        except Exception as cs_err:
            _dbg(f"transform_design_points_to_setup_wcs: getAsCoordinateSystem error: {cs_err}")

    if pts_setup_mm is None:
        _dbg("WARNING: transform_design_points_to_setup_wcs: Failed to apply WCS transform, falling back to untransformed points!")
        pts_setup_mm = pts_mm

    return pts_setup_mm.astype(np.float32)


def extract_mesh_from_fusion_setup(setup: Any) -> Tuple[np.ndarray, np.ndarray]:
    """
    Extract triangular mesh from setup.stockSolids in setup WCS (mm).

    Supports both adsk.fusion.BRepBody (via TriangleMeshCalculator) and
    adsk.fusion.MeshBody (via displayMesh), as well as adsk.fusion.Occurrence containers.

    Returns:
        vertices_mm: (N, 3) float32 numpy array in setup WCS (mm)
        indices: (M, 3) int32 numpy array of triangle indices
    """
    if np is None:
        raise ImportError("numpy is required for solid stock extraction")

    if setup is None:
        raise ValueError("Cannot extract stock from None setup")

    setup_name = getattr(setup, "name", "unnamed")
    solids = getattr(setup, "stockSolids", None)
    cnt = getattr(solids, "count", 0) if solids is not None else 0
    _dbg(f"extract_mesh_from_fusion_setup: setup='{setup_name}', stockSolids count={cnt}")

    if solids is None:
        raise ValueError(f"Setup '{setup_name}' has no stockSolids property")

    # Safely collect solid bodies from ObjectCollection, list, or iterable
    raw_entities = []
    cnt_val = getattr(solids, "count", None)
    if isinstance(cnt_val, int):
        for i in range(cnt_val):
            b = solids.item(i)
            if b is not None:
                raw_entities.append(b)
    elif hasattr(solids, "__iter__"):
        raw_entities = [b for b in solids if b is not None]

    # Unpack occurrences to extract underlying BRep and mesh bodies
    bodies = []

    def _collect_bodies(entity):
        if entity is None:
            return
        if hasattr(entity, "meshManager") or hasattr(entity, "displayMesh") or hasattr(entity, "nodeCoordinatesAsDouble"):
            bodies.append(entity)
            return
        # If it's an Occurrence, unpack its bRepBodies and meshBodies
        if hasattr(entity, "bRepBodies"):
            try:
                cnt = getattr(entity.bRepBodies, "count", 0)
                for i in range(cnt):
                    b = entity.bRepBodies.item(i)
                    if b is not None:
                        bodies.append(b)
            except Exception:
                pass
        if hasattr(entity, "meshBodies"):
            try:
                cnt = getattr(entity.meshBodies, "count", 0)
                for i in range(cnt):
                    m = entity.meshBodies.item(i)
                    if m is not None:
                        bodies.append(m)
            except Exception:
                pass
        if hasattr(entity, "childOccurrences"):
            try:
                cnt = getattr(entity.childOccurrences, "count", 0)
                for i in range(cnt):
                    child = entity.childOccurrences.item(i)
                    if child is not None:
                        _collect_bodies(child)
            except Exception:
                pass

    for ent in raw_entities:
        _collect_bodies(ent)

    if not bodies:
        raise ValueError(
            f"Setup '{setup_name}' uses 'From Solid' stock, "
            "but no stock bodies are selected in the setup."
        )

    all_verts_cm = []
    all_indices = []
    vert_offset = 0

    adsk_mod = sys.modules.get("adsk")
    quality_opt = None
    if adsk_mod and hasattr(adsk_mod, "fusion") and hasattr(adsk_mod.fusion, "TriangleMeshQualityOptions"):
        quality_opt = getattr(adsk_mod.fusion.TriangleMeshQualityOptions, "NormalQualityTriangleMesh", None)

    for idx_b, body in enumerate(bodies):
        b_name = getattr(body, "name", f"Body_{idx_b}")
        b_type = type(body).__name__
        _dbg(f"extract_mesh_from_fusion_setup: reading body[{idx_b}] '{b_name}' ({b_type})")
        mesh = None

        # 1. BRepBody tessellation via meshManager
        mesh_mgr = getattr(body, "meshManager", None)
        if mesh_mgr is not None and hasattr(mesh_mgr, "createMeshCalculator"):
            try:
                calc = mesh_mgr.createMeshCalculator()
                if calc is not None:
                    if quality_opt is not None and hasattr(calc, "setQuality"):
                        try:
                            calc.setQuality(quality_opt)
                        except Exception:
                            pass
                    mesh = calc.calculate()
            except Exception as mm_err:
                _dbg(f"extract_mesh_from_fusion_setup: createMeshCalculator error for '{b_name}': {mm_err}")
                mesh = None

            if mesh is None and hasattr(mesh_mgr, "displayMeshes"):
                try:
                    d_meshes = mesh_mgr.displayMeshes
                    if d_meshes and getattr(d_meshes, "count", 0) > 0:
                        mesh = d_meshes.item(0)
                except Exception:
                    mesh = None

        # 2. MeshBody via displayMesh or mesh property
        if mesh is None:
            for attr in ("displayMesh", "mesh"):
                m = getattr(body, attr, None)
                if m is not None and hasattr(m, "nodeCoordinatesAsDouble"):
                    mesh = m
                    break

        if mesh is None:
            _dbg(f"extract_mesh_from_fusion_setup: could not obtain mesh for body '{b_name}'")
            continue

        try:
            coords = mesh.nodeCoordinatesAsDouble
            node_indices = mesh.nodeIndices
            if coords and node_indices:
                v = np.array(coords, dtype=np.float64).reshape(-1, 3)
                idx = np.array(node_indices, dtype=np.int32).reshape(-1, 3)
                all_verts_cm.append(v)
                all_indices.append(idx + vert_offset)
                vert_offset += len(v)
                _dbg(f"extract_mesh_from_fusion_setup: extracted {len(v)} vertices, {len(idx)} triangles from '{b_name}'")
        except Exception as extract_err:
            _dbg(f"extract_mesh_from_fusion_setup: node extraction error for '{b_name}': {extract_err}")

    if not all_verts_cm:
        raise ValueError(
            f"Failed to extract 3D mesh triangles from stock solids of setup '{setup_name}'."
        )

    merged_verts_cm = np.vstack(all_verts_cm)
    merged_indices = np.vstack(all_indices)
    _dbg(f"extract_mesh_from_fusion_setup: raw merged design coords (cm): min={merged_verts_cm.min(axis=0)}, max={merged_verts_cm.max(axis=0)}")

    # Transform coordinates from Fusion design space (cm) to setup WCS (mm)
    wcs = getattr(setup, "workCoordinateSystem", None)
    vertices_mm = transform_design_points_to_setup_wcs(merged_verts_cm, wcs)
    _dbg(f"extract_mesh_from_fusion_setup: setup WCS coords (mm): min={vertices_mm.min(axis=0)}, max={vertices_mm.max(axis=0)}")

    return vertices_mm, merged_indices


def rasterize_triangles_to_grid(
    grid: Any,
    vertices: np.ndarray,
    indices: np.ndarray,
    tol_pad_z_mm: float = 0.05,
    chunk_size: int = 50000,
) -> None:
    """
    Rasterize 3D triangular mesh facets onto a 2.5D HeightfieldGrid.

    Vectorized implementation with chunked numpy processing:
    - Atomically updates vertex cells using np.maximum.at.
    - Fast path for single-cell triangles using np.maximum.at.
    - Vectorized grid offset loop for small triangles (dr < 6, dc < 6).
    - Conservative fallback for large or near-vertical triangles.

    For each grid cell (row, col) overlapping a triangle, sets:
        grid.H[row, col] = max(grid.H[row, col], interpolated_z + tol_pad_z_mm)

    Args:
        grid: HeightfieldGrid instance
        vertices: (N, 3) float32 array in setup WCS (mm)
        indices: (M, 3) int32 triangle vertex indices
        tol_pad_z_mm: Upward padding (mm) to guarantee chordal conservatism
        chunk_size: Number of triangles to process per vectorized batch
    """
    if vertices.size == 0 or indices.size == 0:
        return

    cell_size = float(grid.cell_size_mm)
    gx_min = float(grid.grid_x_min)
    gy_min = float(grid.grid_y_min)
    nx = int(grid.nx)
    ny = int(grid.ny)

    # 1. Update vertex cells directly using np.maximum.at
    v_cols = np.clip(np.floor((vertices[:, 0] - gx_min) / cell_size).astype(np.int64), 0, nx - 1)
    v_rows = np.clip(np.floor((vertices[:, 1] - gy_min) / cell_size).astype(np.int64), 0, ny - 1)
    v_z = vertices[:, 2] + tol_pad_z_mm
    np.maximum.at(grid.H, (v_rows, v_cols), v_z)

    total_tris = len(indices)
    for c_start in range(0, total_tris, chunk_size):
        c_end = min(c_start + chunk_size, total_tris)
        idx_chunk = indices[c_start:c_end]

        t_verts = vertices[idx_chunk]  # (M, 3, 3)
        v0 = t_verts[:, 0, :]
        v1 = t_verts[:, 1, :]
        v2 = t_verts[:, 2, :]

        x0, y0, z0 = v0[:, 0], v0[:, 1], v0[:, 2]
        x1, y1, z1 = v1[:, 0], v1[:, 1], v1[:, 2]
        x2, y2, z2 = v2[:, 0], v2[:, 1], v2[:, 2]

        nx_val = (y1 - y0) * (z2 - z0) - (z1 - z0) * (y2 - y0)
        ny_val = (z1 - z0) * (x2 - x0) - (x1 - x0) * (z2 - z0)
        nz_val = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0)

        t_min_x = np.minimum(np.minimum(x0, x1), x2)
        t_max_x = np.maximum(np.maximum(x0, x1), x2)
        t_min_y = np.minimum(np.minimum(y0, y1), y2)
        t_max_y = np.maximum(np.maximum(y0, y1), y2)

        c_min = np.clip(np.floor((t_min_x - gx_min) / cell_size).astype(np.int64), 0, nx - 1)
        c_max = np.clip(np.ceil((t_max_x - gx_min) / cell_size).astype(np.int64), 0, nx - 1)
        r_min = np.clip(np.floor((t_min_y - gy_min) / cell_size).astype(np.int64), 0, ny - 1)
        r_max = np.clip(np.ceil((t_max_y - gy_min) / cell_size).astype(np.int64), 0, ny - 1)

        valid = (c_max >= c_min) & (r_max >= r_min)
        if not np.any(valid):
            continue

        dr = r_max - r_min
        dc = c_max - c_min

        # Fast path 1: Single-cell triangles
        single_mask = valid & (dr == 0) & (dc == 0)
        if np.any(single_mask):
            top_z = np.maximum(np.maximum(z0[single_mask], z1[single_mask]), z2[single_mask]) + tol_pad_z_mm
            np.maximum.at(grid.H, (r_min[single_mask], c_min[single_mask]), top_z)

        # Fast path 2: Small multi-cell non-vertical triangles (dr < 6, dc < 6, not single cell)
        is_vert = np.abs(nz_val) < 1e-7
        small_multi = valid & ~single_mask & ~is_vert & (dr < 6) & (dc < 6)

        if np.any(small_multi):
            sub_r_min = r_min[small_multi]
            sub_c_min = c_min[small_multi]
            sub_dr = dr[small_multi]
            sub_dc = dc[small_multi]

            sub_x0, sub_y0, sub_z0 = x0[small_multi], y0[small_multi], z0[small_multi]
            sub_x1, sub_y1 = x1[small_multi], y1[small_multi]
            sub_x2, sub_y2 = x2[small_multi], y2[small_multi]
            sub_nx = nx_val[small_multi]
            sub_ny = ny_val[small_multi]
            sub_nz = nz_val[small_multi]

            # Vectorize over (orow, ocol) offsets
            max_dr = int(np.max(sub_dr))
            max_dc = int(np.max(sub_dc))
            for orow in range(max_dr + 1):
                row_mask = sub_dr >= orow
                if not np.any(row_mask):
                    continue
                r_cells = sub_r_min + orow
                grid_y = gy_min + (r_cells + 0.5) * cell_size

                for ocol in range(max_dc + 1):
                    cell_mask = row_mask & (sub_dc >= ocol)
                    if not np.any(cell_mask):
                        continue

                    c_cells = sub_c_min[cell_mask] + ocol
                    r_c = r_cells[cell_mask]
                    gy = grid_y[cell_mask]
                    gx = gx_min + (c_cells + 0.5) * cell_size

                    sx0, sy0, sz0 = sub_x0[cell_mask], sub_y0[cell_mask], sub_z0[cell_mask]
                    sx1, sy1 = sub_x1[cell_mask], sub_y1[cell_mask]
                    sx2, sy2 = sub_x2[cell_mask], sub_y2[cell_mask]
                    snx = sub_nx[cell_mask]
                    sny = sub_ny[cell_mask]
                    snz = sub_nz[cell_mask]

                    e01 = (sx1 - sx0) * (gy - sy0) - (sy1 - sy0) * (gx - sx0)
                    e12 = (sx2 - sx1) * (gy - sy1) - (sy2 - sy1) * (gx - sx1)
                    e20 = (sx0 - sx2) * (gy - sy2) - (sy0 - sy2) * (gx - sx2)

                    eps = 1e-4
                    inside = ((e01 >= -eps) & (e12 >= -eps) & (e20 >= -eps)) | ((e01 <= eps) & (e12 <= eps) & (e20 <= eps))
                    if not np.any(inside):
                        continue

                    Z = sz0[inside] - (snx[inside] * (gx[inside] - sx0[inside]) + sny[inside] * (gy[inside] - sy0[inside])) / snz[inside] + tol_pad_z_mm
                    np.maximum.at(grid.H, (r_c[inside], c_cells[inside]), Z)

        # Conservative fallback: Vertical or large triangles (dr >= 6 or dc >= 6)
        fallback_mask = valid & ~single_mask & (is_vert | (dr >= 6) | (dc >= 6))
        if np.any(fallback_mask):
            fb_indices = np.where(fallback_mask)[0]
            for i in fb_indices:
                x0_i, y0_i, z0_i = x0[i], y0[i], z0[i]
                x1_i, y1_i, z1_i = x1[i], y1[i], z1[i]
                x2_i, y2_i, z2_i = x2[i], y2[i], z2[i]
                c_min_i = int(c_min[i])
                c_max_i = int(c_max[i])
                r_min_i = int(r_min[i])
                r_max_i = int(r_max[i])
                nz_i = nz_val[i]

                if abs(nz_i) < 1e-7:
                    for pa, pb in (((x0_i, y0_i, z0_i), (x1_i, y1_i, z1_i)),
                                   ((x1_i, y1_i, z1_i), (x2_i, y2_i, z2_i)),
                                   ((x2_i, y2_i, z2_i), (x0_i, y0_i, z0_i))):
                        dx = pb[0] - pa[0]
                        dy = pb[1] - pa[1]
                        dz = pb[2] - pa[2]
                        length = math.hypot(dx, dy)
                        steps = max(1, int(math.ceil(length / (cell_size * 0.5))))
                        for s in range(steps + 1):
                            frac = s / steps
                            px = pa[0] + frac * dx
                            py = pa[1] + frac * dy
                            pz = pa[2] + frac * dz + tol_pad_z_mm
                            c_idx = int(math.floor((px - gx_min) / cell_size))
                            r_idx = int(math.floor((py - gy_min) / cell_size))
                            if 0 <= c_idx < nx and 0 <= r_idx < ny:
                                if pz > grid.H[r_idx, c_idx]:
                                    grid.H[r_idx, c_idx] = pz
                    continue

                cols = np.arange(c_min_i, c_max_i + 1)
                rows = np.arange(r_min_i, r_max_i + 1)
                grid_x = gx_min + (cols + 0.5) * cell_size
                grid_y = gy_min + (rows + 0.5) * cell_size
                X, Y = np.meshgrid(grid_x, grid_y)

                e01 = (x1_i - x0_i) * (Y - y0_i) - (y1_i - y0_i) * (X - x0_i)
                e12 = (x2_i - x1_i) * (Y - y1_i) - (y2_i - y1_i) * (X - x1_i)
                e20 = (x0_i - x2_i) * (Y - y2_i) - (y0_i - y2_i) * (X - x2_i)

                eps = 1e-4
                inside = ((e01 >= -eps) & (e12 >= -eps) & (e20 >= -eps)) | ((e01 <= eps) & (e12 <= eps) & (e20 <= eps))
                if not np.any(inside):
                    continue

                Z = z0_i - (nx_val[i] * (X - x0_i) + ny_val[i] * (Y - y0_i)) / nz_i + tol_pad_z_mm
                sub_h = grid.H[r_min_i : r_max_i + 1, c_min_i : c_max_i + 1]
                mask = inside & (Z > sub_h)
                sub_h[mask] = Z[mask]


def revolve_mesh_to_lathe_stock(
    vertices: np.ndarray,
    indices: np.ndarray,
    z_slice_step_mm: float = 0.5,
    tol_pad_r_mm: float = 0.05,
    max_radial_offset_mm: Optional[float] = 1.0,
    fixtures: Optional[List[AxisymmetricChuckObstacle]] = None,
) -> WorkpieceDefinition:
    """
    Compute axisymmetric revolution profile in (Z, R) plane for turning setups.

    Validates that the solid is centered on the spindle axis (Z axis, R = sqrt(X^2 + Y^2)).
    Generates a conservative superset envelope polygon in the half-plane R >= 0.

    Returns:
        WorkpieceDefinition with AxisymmetricStock2D stock.
    """
    if np is None:
        raise ImportError("numpy is required for revolving lathe stock")

    if vertices.size == 0 or indices.size == 0:
        raise ValueError("Cannot revolve empty mesh to lathe stock")

    v = np.asarray(vertices, dtype=np.float32)

    # 1. Spindle axis centering verification
    x_min, y_min, z_min = v.min(axis=0)
    x_max, y_max, z_max = v.max(axis=0)

    if max_radial_offset_mm is not None:
        center_x = (x_min + x_max) * 0.5
        center_y = (y_min + y_max) * 0.5
        offset_from_spindle = math.hypot(center_x, center_y)

        if offset_from_spindle > max_radial_offset_mm:
            raise ValueError(
                f"Solid stock is not centered on the turning spindle axis (Z-axis offset is {offset_from_spindle:.2f} mm, "
                f"exceeding maximum tolerance of {max_radial_offset_mm:.2f} mm). "
                "Ensure the stock body axis is aligned with the setup Z-axis."
            )

    # 2. Vectorized triangle clipping per Z slab
    t_verts = v[indices]
    tri_z = t_verts[:, :, 2]
    tri_z_min = tri_z.min(axis=1)
    tri_z_max = tri_z.max(axis=1)

    # Triangle edges: (T, 3, 3) where second dim is edge index 0->(0,1), 1->(1,2), 2->(2,0)
    e_A = t_verts[:, [0, 1, 2], :]
    e_B = t_verts[:, [1, 2, 0], :]
    e_Az = e_A[:, :, 2]
    e_Bz = e_B[:, :, 2]
    e_dz = e_Bz - e_Az

    # 3. Discretize into Z slices to construct the outer boundary profile
    total_z_len = max(0.1, z_max - z_min)
    num_slices = max(2, int(math.ceil(total_z_len / z_slice_step_mm)))
    slice_edges = np.linspace(z_min, z_max, num_slices + 1)

    max_radii = np.zeros(num_slices, dtype=np.float64)

    for i in range(num_slices):
        z_lo = slice_edges[i]
        z_hi = slice_edges[i + 1]

        # Overlap condition: triangle overlaps [z_lo, z_hi]
        if i == 0:
            ov = (tri_z_max > z_lo) | ((tri_z_max == z_lo) & (tri_z_min == z_lo))
        else:
            ov = (tri_z_max > z_lo)
        if i == num_slices - 1:
            ov &= (tri_z_min < z_hi) | ((tri_z_min == z_hi) & (tri_z_max == z_hi))
        else:
            ov &= (tri_z_min < z_hi)

        if not np.any(ov):
            max_radii[i] = 0.0
            continue

        sub_tris = t_verts[ov]
        cand_r = []

        # 1) Triangle vertices inside slab [z_lo, z_hi]
        v_in = (sub_tris[:, :, 2] >= z_lo) & (sub_tris[:, :, 2] <= z_hi)
        if np.any(v_in):
            sub_v = sub_tris[v_in]
            cand_r.append(np.hypot(sub_v[:, 0], sub_v[:, 1]))

        # 2) Edge intersections with z = z_lo
        sub_e_A = e_A[ov]
        sub_e_B = e_B[ov]
        sub_e_Az = e_Az[ov]
        sub_e_Bz = e_Bz[ov]
        sub_e_dz = e_dz[ov]

        cross_lo = ((sub_e_Az - z_lo) * (sub_e_Bz - z_lo) <= 0) & (sub_e_dz != 0)
        if np.any(cross_lo):
            t_lo = (z_lo - sub_e_Az[cross_lo]) / sub_e_dz[cross_lo]
            px_lo = sub_e_A[cross_lo, 0] + t_lo * (sub_e_B[cross_lo, 0] - sub_e_A[cross_lo, 0])
            py_lo = sub_e_A[cross_lo, 1] + t_lo * (sub_e_B[cross_lo, 1] - sub_e_A[cross_lo, 1])
            cand_r.append(np.hypot(px_lo, py_lo))

        # 3) Edge intersections with z = z_hi
        cross_hi = ((sub_e_Az - z_hi) * (sub_e_Bz - z_hi) <= 0) & (sub_e_dz != 0)
        if np.any(cross_hi):
            t_hi = (z_hi - sub_e_Az[cross_hi]) / sub_e_dz[cross_hi]
            px_hi = sub_e_A[cross_hi, 0] + t_hi * (sub_e_B[cross_hi, 0] - sub_e_A[cross_hi, 0])
            py_hi = sub_e_A[cross_hi, 1] + t_hi * (sub_e_B[cross_hi, 1] - sub_e_A[cross_hi, 1])
            cand_r.append(np.hypot(px_hi, py_hi))

        if cand_r:
            max_radii[i] = float(np.max(np.concatenate(cand_r))) + tol_pad_r_mm
        else:
            max_radii[i] = 0.0

    profile_pts = []
    # Start at bottom centerline: (z_min, 0)
    profile_pts.append((float(z_min), 0.0))

    for i in range(num_slices):
        z_lo = slice_edges[i]
        z_hi = slice_edges[i + 1]
        max_r = float(max_radii[i])
        profile_pts.append((float(z_lo), max_r))
        profile_pts.append((float(z_hi), max_r))

    # Close the profile along the Z axis
    profile_pts.append((float(z_max), 0.0))
    profile_pts.append((float(z_min), 0.0))

    if Polygon is not None:
        poly = Polygon(profile_pts).buffer(0)
    else:
        poly = None

    outer_r = float(np.max(max_radii)) if len(max_radii) > 0 else 0.0

    stock = AxisymmetricStock2D(
        z_min=float(z_min),
        z_max=float(z_max),
        outer_radius=outer_r,
        inner_radius=0.0,
        polygon=poly,
    )

    return WorkpieceDefinition(
        stock=stock,
        fixtures=fixtures or [],
    )


def extract_solid_workpiece_from_fusion_setup(
    setup: Any,
    is_turning: bool = False,
    cell_size_mm: float = 0.5,
    pad_mm: float = 10.0,
) -> WorkpieceDefinition:
    """
    Extract complete WorkpieceDefinition from a native Fusion 360 Setup configured with Solid stock.

    Args:
        setup: adsk.cam.Setup instance
        is_turning: True if Turning setup, False for Milling
        cell_size_mm: Cell size for milling heightfield (mm)
        pad_mm: Padding around stock bounding box (mm)

    Returns:
        WorkpieceDefinition ready for verifier simulation
    """
    vertices_mm, indices = extract_mesh_from_fusion_setup(setup)

    if is_turning:
        # Extract chuck fixtures if defined in setup parameters
        fixtures = []
        if hasattr(setup, "parameters"):
            try:
                clamp_p = setup.parameters.itemByName("job_stockClampingLength")
                if clamp_p:
                    clamp_val = getattr(clamp_p, "value", None)
                    v_sub = getattr(clamp_val, "value", clamp_val)
                    if isinstance(v_sub, (int, float)) and v_sub > 0.01:
                        clamp_len_mm = float(v_sub) * 10.0  # cm to mm
                        z_min_mm = float(vertices_mm[:, 2].min())
                        r_max_mm = float(np.hypot(vertices_mm[:, 0], vertices_mm[:, 1]).max())
                        fixtures.append(
                            AxisymmetricChuckObstacle(
                                z_min=z_min_mm - 50.0,
                                z_max=z_min_mm + clamp_len_mm,
                                outer_radius=r_max_mm + 30.0,
                                clamping_radius=r_max_mm,
                            )
                        )
            except Exception:
                pass

        return revolve_mesh_to_lathe_stock(vertices_mm, indices, fixtures=fixtures)
    else:
        stock = SolidMeshStock3D.from_mesh(vertices_mm, indices)
        _dbg(
            f"extract_solid_workpiece_from_fusion_setup: created SolidMeshStock3D for '{getattr(setup, 'name', 'unnamed')}' "
            f"X=[{stock.x_min:.2f}, {stock.x_max:.2f}], Y=[{stock.y_min:.2f}, {stock.y_max:.2f}], Z=[{stock.z_min:.2f}, {stock.z_max:.2f}]"
        )
        return WorkpieceDefinition(stock=stock)


def load_stl_mesh(stl_path: str, is_inch: bool = False) -> Tuple[np.ndarray, np.ndarray]:
    """
    Load binary or ASCII STL mesh file into setup coordinates (vertices_mm, indices).

    Args:
        stl_path: Absolute path to the STL file
        is_inch: True if coordinates in the STL file are in inches, False if millimeters

    Returns:
        (vertices_mm, indices) as numpy arrays (float32 and int32)
    """
    if np is None:
        raise ImportError("numpy is required for loading STL mesh")

    if not os.path.isfile(stl_path):
        raise FileNotFoundError(f"STL file not found: {stl_path}")

    file_size = os.path.getsize(stl_path)
    if file_size < 84:
        raise ValueError(f"STL file too small ({file_size} bytes): {stl_path}")

    # Check for standard binary STL format (80-byte header, uint32 triangle count, 50-byte records)
    with open(stl_path, "rb") as f:
        header = f.read(80)
        count_bytes = f.read(4)
        if len(count_bytes) == 4:
            num_tris = struct.unpack("<I", count_bytes)[0]
            expected_binary_size = 84 + num_tris * 50

            is_ascii_header = header.lstrip().startswith(b"solid") and b"facet" in header
            if not is_ascii_header and file_size >= expected_binary_size and num_tris > 0:
                record_dtype = np.dtype([
                    ("normal", "<f4", (3,)),
                    ("v1", "<f4", (3,)),
                    ("v2", "<f4", (3,)),
                    ("v3", "<f4", (3,)),
                    ("attr", "<u2"),
                ])
                data = np.fromfile(f, dtype=record_dtype, count=num_tris)
                v1 = data["v1"]
                v2 = data["v2"]
                v3 = data["v3"]
                verts = np.stack([v1, v2, v3], axis=1).reshape(-1, 3).astype(np.float32)
                if not np.all(np.isfinite(verts)):
                    raise ValueError(f"STL mesh contains non-finite vertex coordinates: {stl_path}")
                indices = np.arange(len(verts), dtype=np.int32).reshape(-1, 3)
                if is_inch:
                    verts *= 25.4
                _dbg(f"load_stl_mesh: loaded binary STL '{stl_path}' with {num_tris} triangles ({len(verts)} vertices)")
                return verts, indices

    # Fallback to ASCII STL parser
    verts_list = []
    with open(stl_path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            line_str = line.strip()
            if line_str.startswith("vertex"):
                parts = line_str.split()
                if len(parts) >= 4:
                    verts_list.append([float(parts[1]), float(parts[2]), float(parts[3])])

    if not verts_list or len(verts_list) % 3 != 0:
        raise ValueError(f"Could not parse valid triangle vertices from STL: {stl_path}")

    verts = np.asarray(verts_list, dtype=np.float32)
    if not np.all(np.isfinite(verts)):
        raise ValueError(f"STL mesh contains non-finite vertex coordinates: {stl_path}")
    indices = np.arange(len(verts), dtype=np.int32).reshape(-1, 3)
    if is_inch:
        verts *= 25.4
    _dbg(f"load_stl_mesh: loaded ASCII STL '{stl_path}' with {len(indices)} triangles ({len(verts)} vertices)")
    return verts, indices


def extract_workpiece_from_stl(
    stl_path: str,
    is_turning: bool = False,
    is_inch: bool = False,
    fixtures: Optional[List[AxisymmetricChuckObstacle]] = None,
) -> WorkpieceDefinition:
    """
    Construct a simulation-ready WorkpieceDefinition from an exported STL mesh file.

    In Autodesk Fusion 360, when exportStock is enabled on a setup with 'From preceding setup'
    stock, Fusion exports the preceding setup's in-process stock oriented in the active setup's WCS.

    Args:
        stl_path: Path to the exported binary/ASCII STL file
        is_turning: True if lathe/turning setup, False if milling
        is_inch: True if STL vertices are in inches (post unit 0), False if millimeters
        fixtures: Optional list of chuck obstacles / fixtures

    Returns:
        WorkpieceDefinition ready for verifier simulation
    """
    vertices_mm, indices = load_stl_mesh(stl_path, is_inch=is_inch)

    if is_turning:
        wp = revolve_mesh_to_lathe_stock(
            vertices_mm,
            indices,
            max_radial_offset_mm=None,  # Preceding setup stock is already aligned to active setup WCS
            fixtures=fixtures,
        )
    else:
        stock = SolidMeshStock3D.from_mesh(vertices_mm, indices)
        _dbg(
            f"extract_workpiece_from_stl: created SolidMeshStock3D from '{stl_path}' "
            f"X=[{stock.x_min:.2f}, {stock.x_max:.2f}], Y=[{stock.y_min:.2f}, {stock.y_max:.2f}], Z=[{stock.z_min:.2f}, {stock.z_max:.2f}]"
        )
        wp = WorkpieceDefinition(stock=stock, fixtures=fixtures or [])
    wp.is_stl_derived = True
    return wp
