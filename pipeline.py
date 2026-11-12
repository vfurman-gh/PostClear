"""
End-to-End Toolpath Verification, Multi-Batch Context Merging & Replay Pipeline.

Orchestrates the G-code-dialect-independent flow:
1. Context Extraction (`universal_dumper.cps`): Dumps structured JSON context and motion events
   from one or more single-tool Compact-NCI (`.cnc`) intermediate files while inheriting the
   target vendor post's tolerance and circular arc parameters.
2. Context Merging & Normalization (`context_parser.py`): Merges multiple single-tool batch
   JSON dumps into a unified multi-section context (`sections[0..N-1]`) and normalizes motions
   into canonical metric `MotionSegment` sequences (`mm`, `mm/min`, `sec`).
3. Unified IPW Verification (`IVerificationEngine`): Simulates stock removal, detects collisions,
   and computes occurrence-gated rapid-promotion rules across the combined setup in a single pass.
4. Direct Promotion Annotation: Annotates authorized air moves in the combined JSON context
   (`ev["promote"] = True`).
5. Single-Pass Vendor Post Replay (`universal_replayer.cps`): Executes the unmodified vendor
   `.cps` post processor (`BASE_POST`) once across all merged sections, producing a single
   dialect-native multi-tool NC program with native tool changes and rapid promotions.
6. Report & Visualization Generation: Compiles markdown reports and high-res verification plots.
"""

import glob
import json
import math
import os
from pathlib import PurePosixPath, PureWindowsPath
import re
import subprocess
import sys
import tempfile
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple, Union

from geometry.models import (
    MotionKind,
    MotionSegment,
    ToolAssembly,
    WorkpieceDefinition,
    VerificationReport,
)
from geometry.engine_interface import IVerificationEngine
from context_parser import (
    DEFAULT_MIN_PROMOTE_LEN_MM,
    merge_context_dumps,
    parse_context_dump,
    apply_promotions_to_context,
    filter_promoted_events_by_length,
    extract_workpiece_from_context,
    extract_tools_from_context,
)


def find_fusion_root_from_path(p: Optional[str]) -> Optional[str]:
    """Extract Fusion webdeploy root or .app directory from a path using path components."""
    if not p or not isinstance(p, (str, bytes)):
        return None
    if isinstance(p, bytes):
        p = p.decode("utf-8", errors="replace")
    p = p.strip().strip('"\'')
    if not p:
        return None

    # Determine whether Windows or POSIX path representation
    is_win = ("\\" in p) or (len(p) >= 2 and p[1] == ":" and p[0].isalpha())
    cls = PureWindowsPath if is_win else PurePosixPath

    # If there are leading tokens before an absolute path (e.g. from command lines), strip to path start
    if not is_win:
        slash_idx = p.find("/")
        if slash_idx > 0:
            p = p[slash_idx:]
    else:
        m = re.search(r"([a-zA-Z]:[\\/]|\\\\[^\\/]+[\\/])", p)
        if m and m.start() > 0:
            p = p[m.start():]

    path_obj = cls(p)
    parts = path_obj.parts

    # 1. Search for 'webdeploy' -> 'production' -> <hash>
    for i in range(len(parts) - 2):
        if parts[i].lower() == "webdeploy" and parts[i + 1].lower() == "production":
            hash_comp = parts[i + 2]
            if hash_comp and hash_comp not in (".", ".."):
                return str(cls(*parts[: i + 3]))

    # 2. Search for '*.app' (e.g. standalone /Applications/Autodesk Fusion 360.app)
    for i in range(len(parts)):
        if parts[i].lower().endswith(".app"):
            return str(cls(*parts[: i + 1]))

    return None


def get_post_binary_from_root(root: Optional[str]) -> Optional[str]:
    """Locate CAM post processor binary inside a specific Fusion root directory."""
    if not root:
        return None
    candidates = [
        os.path.join(root, "Autodesk Fusion.app", "Contents", "Libraries", "Applications", "CAM360", "post"),
        os.path.join(root, "Contents", "Libraries", "Applications", "CAM360", "post"),
        os.path.join(root, "Libraries", "Applications", "CAM360", "post"),
        os.path.join(root, "Applications", "CAM360", "post.exe"),
        os.path.join(root, "post.exe"),
    ]
    for c in candidates:
        if os.path.isfile(c):
            return c
    return None


def get_system_posts_dir_from_root(root: Optional[str]) -> Optional[str]:
    """Locate system Posts directory inside a specific Fusion root directory."""
    if not root:
        return None
    candidates = [
        os.path.join(root, "Autodesk Fusion.app", "Contents", "Libraries", "Applications", "CAM360", "Data", "Posts"),
        os.path.join(root, "Contents", "Libraries", "Applications", "CAM360", "Data", "Posts"),
        os.path.join(root, "Libraries", "Applications", "CAM360", "Data", "Posts"),
        os.path.join(root, "Applications", "CAM360", "Data", "Posts"),
        os.path.join(root, "Data", "Posts"),
    ]
    for c in candidates:
        if os.path.isdir(c):
            return c
    return None


def find_autodesk_fusion_invoking_root() -> Optional[str]:
    """
    Determine the Fusion installation that invoked the script (or is currently running).
    Do NOT blindly select the newest folder by mtime, as a background update script
    may have written an unactivated newer build.
    """
    # 0. Explicit environment override
    env_root = os.environ.get("AUTODESK_FUSION_ROOT")
    if env_root and (os.path.isdir(env_root) or os.path.isfile(env_root)):
        return env_root

    # 1. Inspect loaded adsk modules (when running inside Fusion 360)
    for mod_name in ("adsk", "adsk.core", "adsk.cam", "adsk.fusion"):
        mod = sys.modules.get(mod_name)
        if mod:
            f = getattr(mod, "__file__", None)
            if f:
                root = find_fusion_root_from_path(f)
                if root and os.path.isdir(root):
                    return root
            p = getattr(mod, "__path__", None)
            if p:
                for entry in p:
                    root = find_fusion_root_from_path(entry)
                    if root and os.path.isdir(root):
                        return root

    # 2. Inspect sys.executable
    root = find_fusion_root_from_path(sys.executable)
    if root and os.path.isdir(root):
        return root

    # 3. Inspect sys.prefix and sys.base_prefix
    for pfx in (sys.prefix, getattr(sys, "base_prefix", None)):
        if pfx:
            root = find_fusion_root_from_path(pfx)
            if root and os.path.isdir(root):
                return root

    # 4. Inspect sys.path
    for entry in sys.path:
        root = find_fusion_root_from_path(entry)
        if root and os.path.isdir(root):
            return root

    # 5. Inspect currently running Fusion processes on the host system
    if sys.platform == "darwin" and "LOCALAPPDATA" not in os.environ:
        try:
            with subprocess.Popen(["ps", "-ax", "-o", "command"], stdout=subprocess.PIPE, text=True) as proc:
                out, _ = proc.communicate(timeout=5)
                for line in out.splitlines():
                    if "webdeploy/production/" in line and ("Autodesk Fusion" in line or "QtWebEngineProcess" in line):
                        root = find_fusion_root_from_path(line)
                        if root and os.path.isdir(root):
                            return root
        except Exception:
            pass
    elif sys.platform == "win32":
        # Query running Fusion360.exe process via PowerShell Get-CimInstance (replaces deprecated wmic)
        cmd_ps = [
            "powershell.exe",
            "-NoProfile",
            "-NonInteractive",
            "-Command",
            'Get-CimInstance Win32_Process -Filter "name = \'Fusion360.exe\'" | Select-Object -ExpandProperty ExecutablePath',
        ]
        try:
            with subprocess.Popen(cmd_ps, stdout=subprocess.PIPE, text=True) as proc:
                out, _ = proc.communicate(timeout=5)
                for line in out.splitlines():
                    line = line.strip()
                    if line:
                        root = find_fusion_root_from_path(line)
                        if root and os.path.isdir(root):
                            return root
        except Exception:
            pass
        # Fallback to wmic for legacy Windows environments
        try:
            with subprocess.Popen(["wmic", "process", "where", "name='Fusion360.exe'", "get", "ExecutablePath"], stdout=subprocess.PIPE, text=True) as proc:
                out, _ = proc.communicate(timeout=5)
                for line in out.splitlines():
                    line = line.strip()
                    if line:
                        root = find_fusion_root_from_path(line)
                        if root and os.path.isdir(root):
                            return root
        except Exception:
            pass

    return None


def find_autodesk_post_binary() -> str:
    """
    Locate the standalone Autodesk CAM post processor binary (macOS post or Windows post.exe).
    Fails with FileNotFoundError if undetectable.
    """
    env_post = os.environ.get("AUTODESK_POST_BINARY")
    if env_post and os.path.isfile(env_post):
        return env_post

    # 1. Prefer the specific Fusion installation that invoked the script or is currently running
    invoking_root = find_autodesk_fusion_invoking_root()
    if invoking_root:
        post_bin = get_post_binary_from_root(invoking_root)
        if post_bin and os.path.isfile(post_bin):
            return post_bin

    # No mtime fallback or /Users/* glob: fail closed with clear diagnostics
    inspected = [
        f"sys.executable: '{sys.executable}'",
        f"sys.prefix: '{sys.prefix}'",
        f"sys.path ({len(sys.path)} entries)",
        "loaded adsk modules (adsk, adsk.core, adsk.cam, adsk.fusion)",
        "running host processes (ps / tasklist)",
    ]
    raise FileNotFoundError(
        "Could not determine invoking Autodesk Fusion installation or locate CAM post processor binary. "
        f"Inspected paths: {'; '.join(inspected)}. "
        "Please launch from within Autodesk Fusion 360, or set the 'AUTODESK_FUSION_ROOT' or "
        "'AUTODESK_POST_BINARY' environment variable to override."
    )


def find_autodesk_system_posts_dir() -> str:
    """
    Locate Autodesk Fusion's system Posts directory.
    Fails with FileNotFoundError if undetectable.
    """
    env_dir = os.environ.get("AUTODESK_SYSTEM_POSTS_DIR")
    if env_dir and os.path.isdir(env_dir):
        return env_dir

    # 1. Prefer the specific Fusion installation that invoked the script or is currently running
    invoking_root = find_autodesk_fusion_invoking_root()
    if invoking_root:
        posts_dir = get_system_posts_dir_from_root(invoking_root)
        if posts_dir and os.path.isdir(posts_dir):
            return posts_dir

    # 2. Check if AUTODESK_POST_BINARY is set and system posts live beside it
    env_post = os.environ.get("AUTODESK_POST_BINARY")
    if env_post and os.path.isfile(env_post):
        cam_dir = os.path.dirname(env_post)
        for sub in ("Data/Posts", "Posts"):
            cand = os.path.join(cam_dir, sub)
            if os.path.isdir(cand):
                return cand

    # No mtime fallback or /Users/* glob: fail closed with clear diagnostics
    inspected = [
        f"sys.executable: '{sys.executable}'",
        f"sys.prefix: '{sys.prefix}'",
        f"sys.path ({len(sys.path)} entries)",
        "loaded adsk modules (adsk, adsk.core, adsk.cam, adsk.fusion)",
        "running host processes (ps / tasklist)",
    ]
    raise FileNotFoundError(
        "Could not determine invoking Autodesk Fusion installation or locate system Posts directory. "
        f"Inspected paths: {'; '.join(inspected)}. "
        "Please launch from within Autodesk Fusion 360, or set the 'AUTODESK_FUSION_ROOT' or "
        "'AUTODESK_SYSTEM_POSTS_DIR' environment variable to override."
    )


def find_autodesk_apt_cps() -> Optional[str]:
    """Locate Autodesk's standard apt.cps configuration (deprecated, use find_autodesk_system_posts_dir)."""
    env_apt = os.environ.get("AUTODESK_APT_CPS")
    if env_apt and os.path.isfile(env_apt):
        return env_apt

    try:
        posts_dir = find_autodesk_system_posts_dir()
    except (FileNotFoundError, RuntimeError):
        posts_dir = None

    if posts_dir:
        cand = os.path.join(posts_dir, "apt.cps")
        if os.path.isfile(cand):
            return cand
    return None


class PipelineOrchestrator:
    def __init__(
        self,
        post_bin: Optional[str] = None,
        dumper_cps: Optional[str] = None,
        replayer_cps: Optional[str] = None,
        timeout: float = 120.0,
        *args,
        min_promote_len_mm: float = DEFAULT_MIN_PROMOTE_LEN_MM,
        **kwargs,
    ):
        min_len_val = float(min_promote_len_mm)
        if not math.isfinite(min_len_val) or min_len_val < 0.0:
            raise ValueError(f"min_promote_len_mm must be a finite number >= 0, got {min_promote_len_mm!r}")
        self.post_bin = post_bin or find_autodesk_post_binary()
        if not self.post_bin or not os.path.isfile(self.post_bin):
            raise FileNotFoundError(f"Autodesk post executable not found: {self.post_bin}")
        self.post_bin_path = self.post_bin
        self.timeout = float(timeout)
        self.min_promote_len_mm = min_len_val

        repo_root = os.path.dirname(os.path.abspath(__file__))
        self.dumper_cps = dumper_cps or os.path.join(repo_root, "universal_dumper.cps")
        if not os.path.isfile(self.dumper_cps):
            raise FileNotFoundError(f"universal_dumper.cps not found: {self.dumper_cps}")

        self.replayer_cps = replayer_cps or os.path.join(repo_root, "universal_replayer.cps")
        if not os.path.isfile(self.replayer_cps):
            raise FileNotFoundError(f"universal_replayer.cps not found: {self.replayer_cps}")

    def dump_intermediate_to_json(
        self,
        intermediate_cnc_path: str,
        base_post_path: str,
        output_json_path: str,
    ) -> str:
        """Run Pass 1 (`universal_dumper.cps`) on a binary Compact-NCI `.cnc` file."""
        env = os.environ.copy()
        env["BASE_POST"] = os.path.abspath(base_post_path)
        cmd = [self.post_bin, "--quiet", self.dumper_cps, intermediate_cnc_path, output_json_path]
        try:
            res = subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=self.timeout)
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError(
                f"Context dump extraction timed out after {self.timeout}s (Autodesk post binary at '{self.post_bin}' may be unactivated or hanging): {exc}"
            ) from exc
        if res.returncode != 0:
            err_detail = f"exit code {res.returncode}\nSTDOUT:\n{res.stdout}\nSTDERR:\n{res.stderr}"
            raise RuntimeError(f"Context dump extraction failed: {err_detail}")
        return output_json_path

    def dump_context(
        self,
        intermediate_cnc_path: str,
        base_post_path: str,
        output_json_path: str,
    ) -> Dict:
        """Run Pass 1 (`universal_dumper.cps`) and return the parsed context dictionary."""
        self.dump_intermediate_to_json(intermediate_cnc_path, base_post_path, output_json_path)
        with open(output_json_path, "r", encoding="utf-8") as f:
            return json.load(f)

    def replay_combined_context(
        self,
        combined_context: Union[str, Dict],
        bootstrap_cnc_path: str,
        base_post_path: str,
        final_cnc_path: str,
    ) -> str:
        """
        Run Pass 2 (`universal_replayer.cps`) with the unmodified `base_post_path`
        against `combined_context` (path or dict) and write the native machine program
        to `final_cnc_path`.
        """
        with tempfile.TemporaryDirectory() as tmp_dir:
            if isinstance(combined_context, dict):
                ctx_path = os.path.join(tmp_dir, "combined_context.json")
                with open(ctx_path, "w", encoding="utf-8") as f:
                    json.dump(combined_context, f, separators=(',', ':'))
            else:
                ctx_path = combined_context

            env = os.environ.copy()
            env["BASE_POST"] = os.path.abspath(base_post_path)
            env["CONTEXT_JSON"] = os.path.abspath(ctx_path)

            cmd = [self.post_bin, "--quiet", self.replayer_cps, bootstrap_cnc_path, final_cnc_path]
            try:
                res = subprocess.run(cmd, env=env, capture_output=True, text=True, timeout=self.timeout)
            except subprocess.TimeoutExpired as exc:
                raise RuntimeError(
                    f"Universal post replayer execution timed out after {self.timeout}s (Autodesk post binary at '{self.post_bin}' may be unactivated or hanging): {exc}"
                ) from exc
            if res.returncode != 0:
                err_detail = f"exit code {res.returncode}\nSTDOUT:\n{res.stdout}\nSTDERR:\n{res.stderr}"
                raise RuntimeError(f"Universal post replayer execution failed: {err_detail}")

            with open(final_cnc_path, "r", encoding="utf-8", errors="replace") as f:
                return f.read()

    def run(
        self,
        intermediate_cnc_path: Optional[Union[str, Sequence[str]]] = None,
        base_post_path: str = "",
        workpiece: Optional[WorkpieceDefinition] = None,
        tools_by_op: Optional[Dict[str, ToolAssembly]] = None,
        verifier: Optional[IVerificationEngine] = None,
        final_cnc_path: str = "",
        report_output_path: Optional[str] = None,
        plot_output_path: Optional[str] = None,
        context_dump_paths: Optional[Sequence[str]] = None,
        promote: bool = True,
        bootstrap_cnc_path: Optional[str] = None,
        progress_cb: Optional[Any] = None,
        post_properties: Optional[dict] = None,
        program_name: Optional[str] = None,
        fallback_workpiece: Optional[WorkpieceDefinition] = None,
        is_turning: Optional[bool] = None,
        unit: Optional[int] = None,
        min_promote_len_mm: Optional[float] = None,
        **kwargs,
    ) -> Tuple[VerificationReport, str]:
        """
        Execute the complete dump -> merge -> verify -> replay pipeline on one or more
        intermediate `.cnc` files (or pre-extracted `context_dump_paths`).
        Returns `(VerificationReport, final_gcode_content)`.
        """
        def _notify(msg: str, frac: float):
            if progress_cb and not progress_cb(msg, frac):
                raise RuntimeError("Operation cancelled by user.")

        eff_min_promote_len = (
            float(getattr(self, "min_promote_len_mm", DEFAULT_MIN_PROMOTE_LEN_MM))
            if min_promote_len_mm is None
            else float(min_promote_len_mm)
        )
        if not math.isfinite(eff_min_promote_len) or eff_min_promote_len < 0.0:
            raise ValueError(f"min_promote_len_mm must be a finite number >= 0, got {eff_min_promote_len!r}")

        if context_dump_paths:
            dump_files = list(context_dump_paths)
            bootstrap_cnc = bootstrap_cnc_path or ""
        elif intermediate_cnc_path:
            if isinstance(intermediate_cnc_path, str):
                cnc_paths = [intermediate_cnc_path]
            else:
                cnc_paths = list(intermediate_cnc_path)
            if not cnc_paths or not os.path.isfile(cnc_paths[0]):
                raise FileNotFoundError(f"Intermediate NC file not found: {intermediate_cnc_path}")
            dump_files = []
            bootstrap_cnc = bootstrap_cnc_path or cnc_paths[0]
        else:
            raise ValueError("Either context_dump_paths or intermediate_cnc_path must be provided")

        if not base_post_path or not os.path.isfile(base_post_path):
            raise FileNotFoundError(f"Base machine post not found: {base_post_path}")

        tools_by_op = dict(tools_by_op) if tools_by_op else {}

        for out_p in (final_cnc_path, report_output_path, plot_output_path):
            if out_p:
                out_dir = os.path.dirname(os.path.abspath(out_p))
                if out_dir:
                    os.makedirs(out_dir, exist_ok=True)

        with tempfile.TemporaryDirectory() as tmp_dir:
            # -----------------------------------------------------------------
            # Step 1: Dump each intermediate .cnc to JSON (if not pre-dumped)
            # -----------------------------------------------------------------
            if not context_dump_paths:
                for idx, cnc_p in enumerate(cnc_paths):
                    dmp_p = os.path.join(tmp_dir, f"batch_{idx}.json")
                    self.dump_intermediate_to_json(cnc_p, base_post_path, dmp_p)
                    dump_files.append(dmp_p)

            # -----------------------------------------------------------------
            # Step 2: Merge JSON Contexts & Domain Extraction
            # -----------------------------------------------------------------
            _notify("Pass 2: Merging toolpath context...", 0.42)
            combined_ctx = merge_context_dumps(dump_files)
            if program_name:
                combined_ctx["programName"] = str(program_name)
            if post_properties:
                combined_ctx["postProperties"] = post_properties
                if "maximumSpindleSpeed" in post_properties:
                    try:
                        combined_ctx["maximumSpindleSpeed"] = float(post_properties["maximumSpindleSpeed"])
                    except (TypeError, ValueError):
                        pass
            if unit is not None:
                combined_ctx["unit"] = int(unit)

            if is_turning is None:
                is_turning = any(
                    s.get("type", 0) == 1 or bool(s.get("tool", {}).get("_isTurningTool", False))
                    for s in combined_ctx.get("sections", [])
                )
            stock_mode = str(
                combined_ctx.get("stockMode")
                or combined_ctx.get("globalParams", {}).get("job:stockMode")
                or combined_ctx.get("globalParams", {}).get("stockMode")
                or ""
            ).lower()

            def _dbg_pipe(msg: str) -> None:
                try:
                    from PostProcessAll import _dbg_global
                    _dbg_global(msg)
                except Exception:
                    pass

            explicit_workpiece = workpiece is not None
            if explicit_workpiece and (combined_ctx.get("stockPath") or (isinstance(combined_ctx.get("stockExport"), dict) and combined_ctx["stockExport"].get("destPath"))):
                _dbg_pipe(
                    f"PipelineOrchestrator.run: workpiece provided explicitly; ignoring stockPath '{combined_ctx.get('stockPath')}'"
                )

            if workpiece is None:
                workpiece = extract_workpiece_from_context(combined_ctx, fallback_workpiece=fallback_workpiece, is_turning=is_turning)

            setup_label = f"'{program_name}'" if program_name else "this setup"
            if any(k in stock_mode for k in ("previous", "preceding", "prev", "rest")):
                if not explicit_workpiece and (workpiece is None or not getattr(workpiece, "is_stl_derived", False)):
                    se = combined_ctx.get("stockExport")
                    if isinstance(se, dict):
                        diag = (
                            f" [stockExport: enabled={se.get('enabled')}, "
                            f"hasParam={se.get('hasParam')}, "
                            f"srcPath={se.get('srcPath')!r}, "
                            f"srcExists={se.get('srcExists')}, "
                            f"destPath={se.get('destPath')!r}, "
                            f"error={se.get('error')!r}]"
                        )
                    else:
                        diag = " [stockExport: none]"
                    raise ValueError(
                        f"Setup {setup_label} uses 'From Previous Setup' stock, "
                        "but Fusion 360 did not produce a preceding stock mesh. "
                        "Please verify that the preceding setup has generated toolpaths "
                        f"and valid in-process stock before posting.{diag}"
                    )
            if any(k in stock_mode for k in ("solid", "body")):
                if workpiece is None and fallback_workpiece is None:
                    raise ValueError(
                        f"Setup {setup_label} uses 'From Solid' stock, "
                        "but no solid body geometry was provided or extracted."
                    )

            _dbg_pipe(f"PipelineOrchestrator.run: workpiece stock={getattr(workpiece, 'stock', None)}")

            if workpiece is None or getattr(workpiece, "stock", None) is None:
                setup_label = f"'{program_name}'" if program_name else "this setup"
                raise ValueError(
                    f"No stock detected or defined for {setup_label}. "
                    "A valid stock definition is required for toolpath verification and simulation."
                )
            if not tools_by_op:
                tools_by_op = extract_tools_from_context(combined_ctx, is_turning=is_turning)
            if is_turning and not tools_by_op:
                raise ValueError("No lathe operations with valid tools found in context dump.")

            segments = parse_context_dump(combined_ctx)

            # -----------------------------------------------------------------
            # Step 3: Run Unified Verification Engine on Combined Setup
            # -----------------------------------------------------------------
            _notify(f"Pass 2: Simulating stock & collisions ({'turning' if is_turning else 'milling'})...", 0.50)
            if verifier is None:
                if is_turning:
                    from geometry.lathe_engine import Lathe2DIPWVerifier
                    verifier = Lathe2DIPWVerifier(tool_scale_to_mm=1.0)
                else:
                    try:
                        from geometry.heightfield_engine import MillHeightfieldVerifier
                        verifier = MillHeightfieldVerifier()
                    except ImportError:
                        from geometry.mill_engine import MillClearanceVerifier
                        verifier = MillClearanceVerifier()

            # Phase 1 H4: Runtime coordinate frame check for STL-derived stock
            is_stl_stock = (
                getattr(workpiece, "is_stl_derived", False)
                or getattr(getattr(workpiece, "stock", None), "is_stl_derived", False)
                or bool(combined_ctx.get("stockPath"))
            )
            tp_box_log = None
            st_box_log = None
            if is_stl_stock and segments:
                cutting_segments = [s for s in segments if s.motion != MotionKind.RAPID]
                if cutting_segments:
                    tp_xs = [s.start.x for s in cutting_segments] + [s.end.x for s in cutting_segments]
                    tp_ys = [s.start.y for s in cutting_segments] + [s.end.y for s in cutting_segments]
                    tp_zs = [s.start.z for s in cutting_segments] + [s.end.z for s in cutting_segments]
                    tp_min_x, tp_max_x = min(tp_xs), max(tp_xs)
                    tp_min_y, tp_max_y = min(tp_ys), max(tp_ys)
                    tp_min_z, tp_max_z = min(tp_zs), max(tp_zs)

                    st = getattr(workpiece, "stock", None)
                    if hasattr(st, "x_min") and hasattr(st, "x_max"):
                        st_min_x, st_max_x = st.x_min, st.x_max
                        st_min_y, st_max_y = st.y_min, st.y_max
                        st_min_z, st_max_z = st.z_min, st.z_max
                    elif hasattr(st, "outer_radius") and hasattr(st, "z_min"):
                        st_r = st.outer_radius
                        st_min_x, st_max_x = -st_r, st_r
                        st_min_y, st_max_y = -st_r, st_r
                        st_min_z, st_max_z = st.z_min, st.z_max
                    else:
                        st_min_x = st_max_x = st_min_y = st_max_y = st_min_z = st_max_z = 0.0

                    tp_box_log = f"[{tp_min_x:.2f}, {tp_max_x:.2f}] x [{tp_min_y:.2f}, {tp_max_y:.2f}] x [{tp_min_z:.2f}, {tp_max_z:.2f}]"
                    st_box_log = f"[{st_min_x:.2f}, {st_max_x:.2f}] x [{st_min_y:.2f}, {st_max_y:.2f}] x [{st_min_z:.2f}, {st_max_z:.2f}]"

                    if is_turning:
                        tp_z_span = max(0.0, tp_max_z - tp_min_z)
                        ov_z = max(0.0, min(tp_max_z, st_max_z) - max(tp_min_z, st_min_z))
                        if tp_z_span > 1e-6:
                            z_ratio = ov_z / tp_z_span
                        else:
                            z_ratio = 1.0 if (st_min_z <= tp_min_z <= st_max_z) else 0.0
                        _dbg_pipe(f"H4 turning Z overlap: toolpath={tp_box_log}, stock={st_box_log}, ratio={z_ratio*100:.1f}%")
                        if z_ratio < 0.5:
                            raise ValueError(
                                f"Cutting toolpath Z span [{tp_min_z:.2f}, {tp_max_z:.2f}] has insufficient overlap ({z_ratio*100:.1f}% < 50%) "
                                f"with stock Z span [{st_min_z:.2f}, {st_max_z:.2f}]. "
                                f"Toolpath bounding box: {tp_box_log}; Stock bounding box: {st_box_log}."
                            )
                    else:
                        max_tool_r = max(
                            (
                                float(getattr(getattr(t, "geometry", None), "diameter", 0.0) or 0.0) * 0.5
                                for t in (tools_by_op or {}).values()
                            ),
                            default=0.0,
                        )
                        eff_st_min_x = st_min_x - max_tool_r
                        eff_st_max_x = st_max_x + max_tool_r
                        eff_st_min_y = st_min_y - max_tool_r
                        eff_st_max_y = st_max_y + max_tool_r
                        # When the toolpath enters below the top of the stock, vertical G1 plunges/retracts
                        # to the clearance plane above st_max_z are in-air positioning moves and should not
                        # inflate the cutting Z span.
                        eff_tp_max_z = min(tp_max_z, st_max_z) if tp_min_z < st_max_z else tp_max_z

                        tp_dx = max(0.0, tp_max_x - tp_min_x)
                        tp_dy = max(0.0, tp_max_y - tp_min_y)
                        tp_dz = max(0.0, eff_tp_max_z - tp_min_z)

                        ov_dx = max(0.0, min(tp_max_x, eff_st_max_x) - max(tp_min_x, eff_st_min_x))
                        ov_dy = max(0.0, min(tp_max_y, eff_st_max_y) - max(tp_min_y, eff_st_min_y))
                        ov_dz = max(0.0, min(eff_tp_max_z, st_max_z) - max(tp_min_z, st_min_z))

                        r_x = (ov_dx / tp_dx) if tp_dx > 1e-6 else (1.0 if eff_st_min_x <= tp_min_x <= eff_st_max_x else 0.0)
                        r_y = (ov_dy / tp_dy) if tp_dy > 1e-6 else (1.0 if eff_st_min_y <= tp_min_y <= eff_st_max_y else 0.0)
                        r_z = (ov_dz / tp_dz) if tp_dz > 1e-6 else (1.0 if st_min_z <= tp_min_z <= st_max_z else 0.0)
                        vol_ratio = r_x * r_y * r_z

                        _dbg_pipe(f"H4 milling volume overlap: toolpath={tp_box_log}, stock={st_box_log}, ratio={vol_ratio*100:.1f}%")
                        if vol_ratio < 0.5:
                            raise ValueError(
                                f"Toolpath bounding box {tp_box_log} has insufficient volume overlap ({vol_ratio*100:.1f}% < 50%) "
                                f"with Stock bounding box {st_box_log}."
                            )

            if segments:
                all_x = [s.start.x for s in segments] + [s.end.x for s in segments]
                all_y = [s.start.y for s in segments] + [s.end.y for s in segments]
                all_z = [s.start.z for s in segments] + [s.end.z for s in segments]
                _dbg_pipe(
                    f"PipelineOrchestrator.run: toolpath bounds (mm) "
                    f"X=[{min(all_x):.2f}, {max(all_x):.2f}], "
                    f"Y=[{min(all_y):.2f}, {max(all_y):.2f}], "
                    f"Z=[{min(all_z):.2f}, {max(all_z):.2f}]"
                )

            report = verifier.verify_toolpath(segments, tools_by_op, workpiece)
            if report.promoted_events:
                report.promoted_events = filter_promoted_events_by_length(
                    report.promoted_events,
                    segments,
                    min_len_mm=eff_min_promote_len,
                )
            if tp_box_log and st_box_log:
                report.stock_bounding_box = st_box_log
                report.toolpath_bounding_box = tp_box_log

            # Phase 1 M1: When previous-mode turning has no fixtures, warn user
            if is_turning and any(k in stock_mode for k in ("previous", "preceding", "prev", "rest")):
                if not getattr(workpiece, "fixtures", None):
                    report.fixture_warning = (
                        "Fixture checks disabled: previous-mode turning has no fixture/chuck definitions."
                    )
                    _dbg_pipe(f"PipelineOrchestrator.run: {report.fixture_warning}")

            # Phase 2 C2: Rapid promotion dog-leg safety check
            # Indicator: post-kernel global highFeedMapping and vendor post property useG0.
            # Missing rapidMode (e.g. legacy dumps from older universal_dumper versions)
            # defaults to keeping promotion enabled for backward compatibility.
            rapid_mode = combined_ctx.get("rapidMode")
            if rapid_mode is not None:
                hfm = rapid_mode.get("highFeedMapping")
                use_g0 = rapid_mode.get("useG0")
                is_dog_leg = (hfm is not None and hfm != 0) or (use_g0 is False)
                if is_dog_leg:
                    promote = False
                    hfm_val = hfm if (hfm is not None and hfm != 0) else (hfm if hfm is not None else 0)
                    dog_leg_msg = (
                        f"Rapid promotion disabled: post High feedrate mapping = {hfm_val} / useG0=false "
                        "(controller may dog-leg rapids)"
                    )
                    report.dog_leg_warning = dog_leg_msg
                    report.promoted_events = set()
                    _dbg_pipe(f"PipelineOrchestrator.run: {dog_leg_msg}")

            _dbg_pipe(
                f"PipelineOrchestrator.run: report.volume={report.total_volume_removed_mm3:.3f} mm³, "
                f"segments={len(segments)}, promoted_events={len(report.promoted_events)}"
            )

            # -----------------------------------------------------------------
            # Step 4: Annotate Promoted Air Moves in Combined Context
            # -----------------------------------------------------------------
            restored_count = 0
            has_stock = (workpiece is not None and getattr(workpiece, "stock", None) is not None)
            if promote and has_stock and report.promoted_events:
                # Q2 confirmed: collision filter blocks ONLY the colliding event (sec_idx, event_idx) from promoted_events
                colliding_events = {
                    (c.sec_idx, c.event_idx)
                    for c in report.collisions
                    if c.sec_idx is not None and c.event_idx is not None
                }
                safe_promoted_events = {
                    ev for ev in report.promoted_events
                    if ev not in colliding_events
                }
                if safe_promoted_events:
                    restored_count = apply_promotions_to_context(combined_ctx, safe_promoted_events)

            # -----------------------------------------------------------------
            # Step 5: Execute Single-Pass Vendor Post Replay
            # -----------------------------------------------------------------
            _notify("Pass 2: Emitting final NC program...", 0.85)
            if not bootstrap_cnc or not os.path.isfile(bootstrap_cnc):
                bootstrap_cnc = os.path.join(tmp_dir, "bootstrap.intermediate.cnc")
                with open(bootstrap_cnc, "wb") as bf:
                    bf.write(b"")

            effective_final_cnc = final_cnc_path or os.path.join(tmp_dir, "replayed_output.cnc")
            final_gcode = self.replay_combined_context(
                combined_context=combined_ctx,
                bootstrap_cnc_path=bootstrap_cnc,
                base_post_path=base_post_path,
                final_cnc_path=effective_final_cnc,
            )

            # -----------------------------------------------------------------
            # Step 6: Generate Summary Report & Verification Plot
            # -----------------------------------------------------------------
            if plot_output_path:
                _notify("Pass 2: Generating verification plot & report...", 0.70)
                try:
                    is_inch = (combined_ctx.get("unit", 0) == 0)
                    if is_turning:
                        from geometry.plotter import generate_verification_plot
                        final_ipw = getattr(verifier, "ipw", None)
                        chuck_obs = getattr(verifier, "chuck_obstacle", None)
                        generate_verification_plot(
                            workpiece=workpiece,
                            final_ipw=final_ipw,
                            chuck_obstacle=chuck_obs,
                            segments=segments,
                            report=report,
                            tools_by_op=tools_by_op,
                            out_png_path=plot_output_path,
                            unit_is_inch=is_inch,
                        )
                    else:
                        from geometry.plotter import generate_mill_verification_plot
                        generate_mill_verification_plot(
                            workpiece=workpiece,
                            grid=getattr(verifier, "grid", None),
                            segments=segments,
                            report=report,
                            tools_by_op=tools_by_op,
                            out_png_path=plot_output_path,
                            unit_is_inch=is_inch,
                        )
                except Exception as exc:
                    print(f"WARNING: Verification plot generation failed: {exc}", file=sys.stderr)

            report_summary = self.format_summary_report(
                report, final_gcode, base_post_path, restored_count=restored_count
            )
            if report_output_path:
                with open(report_output_path, "w", encoding="utf-8") as f:
                    f.write(report_summary)

            return report, final_gcode

    def format_summary_report(
        self,
        report: VerificationReport,
        final_gcode: str,
        base_post_path: str,
        restored_count: Optional[int] = None,
    ) -> str:
        """Format a human-readable and structured markdown verification report."""
        promoted_count = len(report.promoted_events)
        collision_count = len(report.collisions)

        wrapper_restored_count = restored_count if restored_count is not None else 0
        if restored_count is None:
            for line in final_gcode.splitlines():
                if "UNIVERSAL WRAPPER" in line and "RESTORED" in line:
                    try:
                        parts = line.split("RESTORED")[1].split("G0")[0].strip()
                        wrapper_restored_count = int(parts)
                    except (ValueError, IndexError):
                        pass

        lines = [
            "# Toolpath Verification & Rapid-Promotion Report",
            "",
            "## Summary Metrics",
            f"- **Target Base Post**: `{os.path.basename(base_post_path)}`",
            f"- **Total Machining Time**: {report.total_machining_time_sec:.2f} s ({report.total_machining_time_sec/60:.2f} min)",
            f"- **Total Material Removed**: {report.total_volume_removed_mm3:.2f} mm³",
            f"- **Collisions Detected**: {collision_count}",
            f"- **Flagged MRR Spikes**: {len(report.mrr_spikes)}",
            f"- **Authorized Air Moves**: {promoted_count}",
            f"- **G0 Moves Restored in G-code**: {wrapper_restored_count}",
        ]

        if getattr(report, "dog_leg_warning", None):
            lines.append(f"- **Promotion Status**: {report.dog_leg_warning}")
            lines.extend([
                "",
                f"> **Notice**: {report.dog_leg_warning}",
            ])

        if getattr(report, "fixture_warning", None):
            lines.append(f"- **Fixture Status**: {report.fixture_warning}")
            lines.extend([
                "",
                f"> **Warning**: {report.fixture_warning}",
            ])

        if getattr(report, "stock_bounding_box", None) and getattr(report, "toolpath_bounding_box", None):
            lines.extend([
                "",
                "## Stock & Toolpath Alignment",
                f"- **Stock Bounding Box**: `{report.stock_bounding_box}`",
                f"- **Toolpath Cutting Bounding Box**: `{report.toolpath_bounding_box}`",
            ])

        lines.extend([
            "",
            "## Collision Status",
        ])

        # Collate collisions by operation
        cols_by_op: Dict[str, List[CollisionRecord]] = {}
        for c in report.collisions:
            op_label = c.op_id if c.op_id else "General"
            cols_by_op.setdefault(op_label, []).append(c)

        all_ops = list(report.operation_names) if getattr(report, "operation_names", None) else []
        for op in cols_by_op.keys():
            if op not in all_ops:
                all_ops.append(op)
        if not all_ops:
            all_ops = list(cols_by_op.keys()) or ["All Operations"]

        if collision_count == 0:
            lines.append("✅ **Zero collisions detected.** All rapid moves and holder clearances are verified safe across all operations.")
            lines.append("")
            lines.append("### Operations Verification Summary")
            lines.append("| Operation | Status | Collisions | Details by Type |")
            lines.append("| :--- | :--- | :--- | :--- |")
            for op in all_ops:
                lines.append(f"| {op} | ✅ Safe | 0 | None |")
        else:
            affected_ops = [op for op, clist in cols_by_op.items() if len(clist) > 0]
            lines.append(
                f"⚠️ **{collision_count} Collision(s) Detected across {len(affected_ops)} operation(s)!**  \n"
                "Final NC code generation completed with warnings. Review the table and detailed move listings below before machining."
            )
            lines.append("")
            lines.append("### Collisions by Operation")
            lines.append("| Operation | Status | Collisions | Details by Type |")
            lines.append("| :--- | :--- | :--- | :--- |")
            for op in all_ops:
                op_cols = cols_by_op.get(op, [])
                if op_cols:
                    type_counts: Dict[str, int] = {}
                    for c in op_cols:
                        k_name = c.kind.name if hasattr(c.kind, "name") else str(c.kind)
                        type_counts[k_name] = type_counts.get(k_name, 0) + 1
                    type_str = ", ".join(f"`{k}` ({n})" for k, n in sorted(type_counts.items()))
                    lines.append(f"| **{op}** | ⚠️ **Collision** | **{len(op_cols)}** | {type_str} |")
                else:
                    lines.append(f"| {op} | ✅ Safe | 0 | None |")
            lines.append(f"| **Total** | | **{collision_count}** | |")
            lines.append("")

            lines.append("### Detailed Collision Breakdown")
            for op in all_ops:
                op_cols = cols_by_op.get(op, [])
                if not op_cols:
                    continue
                lines.append(f"#### ⚠️ Operation: {op} ({len(op_cols)} collision{'s' if len(op_cols) != 1 else ''})")
                for c in op_cols:
                    k_name = c.kind.name if hasattr(c.kind, "name") else str(c.kind)
                    lines.append(
                        f"- **Move {c.segment_id}** (`{k_name}`): {c.description} "
                        f"at contact `({c.contact_point.x:.2f}, {c.contact_point.y:.2f}, {c.contact_point.z:.2f})`"
                    )
                lines.append("")

        lines.append("")
        lines.append("## Material Removal Rate (MRR) Spikes")
        if not report.mrr_spikes:
            lines.append("✅ **Zero MRR spikes detected.** All cutting passes remained within normal statistical limits.")
        else:
            lines.append(
                f"⚠️ **{len(report.mrr_spikes)} MRR spike(s) flagged (top 3 per operation shown).**"
            )
            lines.append("")
            lines.append("| Operation | Move | Start Time | MRR Rate (mm³/min) | Volume Removed (mm³) | Depth of Cut (mm) |")
            lines.append("| :--- | :--- | :--- | :--- | :--- | :--- |")

            spikes_by_op: Dict[str, List[MRRSample]] = {}
            for sp in report.mrr_spikes:
                op_name = sp.op_id if sp.op_id else "General"
                spikes_by_op.setdefault(op_name, []).append(sp)

            for op in all_ops:
                op_spikes = spikes_by_op.get(op, [])
                for sp in op_spikes:
                    lines.append(
                        f"| {op} | Move {sp.segment_id} | {sp.timestamp_sec:.2f} s | "
                        f"{sp.mrr_rate_mm3_per_min:.2f} | {sp.volume_removed_mm3:.2f} | {sp.depth_of_cut_mm:.2f} |"
                    )
            for op, op_spikes in spikes_by_op.items():
                if op not in all_ops:
                    for sp in op_spikes:
                        lines.append(
                            f"| {op} | Move {sp.segment_id} | {sp.timestamp_sec:.2f} s | "
                            f"{sp.mrr_rate_mm3_per_min:.2f} | {sp.volume_removed_mm3:.2f} | {sp.depth_of_cut_mm:.2f} |"
                        )

        return "\n".join(lines)

    _format_summary_report = format_summary_report
