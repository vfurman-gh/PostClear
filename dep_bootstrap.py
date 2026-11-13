"""Cross-platform Fusion 360 Python & runtime dependency bootstrap utilities.

Pure Python standard library module shared by PostProcessAll.py, install_addin.py,
and scripts/build_release.py across macOS and Windows.
"""

import glob
import importlib
import importlib.machinery
import importlib.util
import json
import os
import platform
import subprocess
import sys
from typing import Dict, List, Optional, Sequence, Tuple, Union


REQUIRED_RUNTIME_PACKAGES: Tuple[str, ...] = ("numpy", "shapely", "matplotlib")
CREATE_NO_WINDOW: int = getattr(subprocess, "CREATE_NO_WINDOW", 0x08000000)


def _subprocess_kwargs() -> Dict[str, int]:
    """Return platform-specific subprocess flags (suppress console flash on Windows)."""
    if sys.platform == "win32" or os.name == "nt":
        return {"creationflags": CREATE_NO_WINDOW}
    return {}


def _parse_py_ver(py_ver: Optional[Union[Tuple[int, int], Sequence[int], str]]) -> Tuple[int, int]:
    if py_ver is None:
        return (sys.version_info[0], sys.version_info[1])
    if isinstance(py_ver, str):
        parts = [p for p in py_ver.strip().split(".") if p.isdigit()]
        if len(parts) >= 2:
            return (int(parts[0]), int(parts[1]))
        if len(parts) == 1 and len(parts[0]) >= 2:
            return (int(parts[0][0]), int(parts[0][1:]))
        raise ValueError(f"Invalid Python version string: {py_ver!r}")
    return (int(py_ver[0]), int(py_ver[1]))


def get_abi_tag(
    py_ver: Optional[Union[Tuple[int, int], Sequence[int], str]] = None,
    plat: Optional[str] = None,
    machine: Optional[str] = None,
) -> str:
    """Return normalized ABI directory tag such as 'darwin_arm64_py314' or 'win32_amd64_py314'."""
    maj, min_ = _parse_py_ver(py_ver)
    raw_plat = (plat if plat is not None else sys.platform).strip().lower()
    if raw_plat.startswith("win"):
        norm_plat = "win32"
    elif raw_plat.startswith("darwin") or raw_plat == "mac":
        norm_plat = "darwin"
    elif raw_plat.startswith("linux"):
        norm_plat = "linux"
    else:
        norm_plat = raw_plat

    raw_mach = (machine if machine is not None else platform.machine()).strip().lower()
    if norm_plat == "win32":
        if raw_mach in ("amd64", "x86_64", "x64", "em64t", ""):
            norm_mach = "amd64"
        elif raw_mach in ("arm64", "aarch64"):
            norm_mach = "arm64"
        elif raw_mach in ("x86", "i386", "i686"):
            norm_mach = "x86"
        else:
            norm_mach = raw_mach
    else:
        if raw_mach in ("arm64", "aarch64", "arm64e"):
            norm_mach = "arm64"
        elif raw_mach in ("x86_64", "amd64", "x64"):
            norm_mach = "x86_64"
        else:
            norm_mach = raw_mach or "unknown"

    return f"{norm_plat}_{norm_mach}_py{maj}{min_}"


def get_abi_packages_dir(
    addon_dir: str,
    py_ver: Optional[Union[Tuple[int, int], Sequence[int], str]] = None,
    plat: Optional[str] = None,
    machine: Optional[str] = None,
) -> str:
    """Return ABI-scoped packages directory under addon_dir."""
    return os.path.join(addon_dir, "packages", get_abi_tag(py_ver=py_ver, plat=plat, machine=machine))


def configure_packages_sys_path(addon_dir: str) -> List[str]:
    """Ensure the ABI-scoped (or legacy flat) packages folder is present on sys.path."""
    added: List[str] = []
    if not addon_dir:
        return added

    abi_dir = get_abi_packages_dir(addon_dir)
    flat_dir = os.path.join(addon_dir, "packages")
    abi_populated = os.path.isdir(abi_dir) and any(
        os.path.exists(os.path.join(abi_dir, pkg)) for pkg in REQUIRED_RUNTIME_PACKAGES
    )
    if abi_populated:
        while flat_dir in sys.path:
            sys.path.remove(flat_dir)
        while abi_dir in sys.path:
            sys.path.remove(abi_dir)
        insert_idx = 1 if (sys.path and sys.path[0] == addon_dir) else 0
        sys.path.insert(insert_idx, abi_dir)
        added.append(abi_dir)
        return added

    if os.path.isdir(flat_dir) and any(
        os.path.exists(os.path.join(flat_dir, pkg)) for pkg in ("numpy", "shapely")
    ):
        while flat_dir in sys.path:
            sys.path.remove(flat_dir)
        insert_idx = 1 if (sys.path and sys.path[0] == addon_dir) else 0
        sys.path.insert(insert_idx, flat_dir)
        added.append(flat_dir)

    return added


def _is_python_executable(path: Optional[str]) -> bool:
    """Return True iff path points to a real Python interpreter (never Fusion360.exe or WindowsApps stub)."""
    if not path or not isinstance(path, str):
        return False
    norm = os.path.normpath(path)
    if "windowsapps" in norm.lower():
        return False
    base = os.path.basename(norm).lower()
    if not base.startswith("python"):
        return False
    return os.path.isfile(norm)


def _prefix_python_candidates(prefix: str, plat: str) -> List[str]:
    if not prefix:
        return []
    if plat == "win32":
        return [os.path.join(prefix, "python.exe")]
    return [
        os.path.join(prefix, "bin", "python3"),
        os.path.join(prefix, "bin", "python"),
    ]


def _webdeploy_glob_patterns(plat: str, env: Optional[Dict[str, str]] = None) -> List[str]:
    env_map = env if env is not None else os.environ
    if plat == "win32":
        patterns: List[str] = []
        local_app_data = env_map.get("LOCALAPPDATA", "")
        prog_files = env_map.get("PROGRAMFILES", "") or env_map.get("ProgramFiles", "")
        if local_app_data:
            patterns.append(os.path.join(local_app_data, "Autodesk", "webdeploy", "production", "*", "Python", "python.exe"))
            patterns.append(os.path.join(local_app_data, "Autodesk", "webdeploy", "shared", "PYTHON", "*", "python.exe"))
        if prog_files:
            patterns.append(os.path.join(prog_files, "Autodesk", "webdeploy", "production", "*", "Python", "python.exe"))
        return patterns

    if plat == "darwin":
        return [
            os.path.expanduser(
                "~/Library/Application Support/Autodesk/webdeploy/production/*/Autodesk Fusion.app/Contents/Frameworks/Python.framework/Versions/*/bin/python3"
            ),
            "/Applications/Autodesk Fusion.app/Contents/Frameworks/Python.framework/Versions/*/bin/python3",
        ]

    return []


def _is_fusion_prefix(path: Optional[str]) -> bool:
    """Return True iff path points inside an Autodesk / Fusion 360 runtime prefix."""
    if not path or not isinstance(path, str):
        return False
    norm = os.path.normpath(path).lower()
    return "autodesk" in norm or "fusion" in norm


def find_fusion_python_executable() -> Optional[str]:
    """Locate Fusion 360's embedded Python interpreter (never returning Fusion360.exe or host binary)."""
    plat = "win32" if (sys.platform == "win32" or os.name == "nt") else sys.platform
    prefixes = tuple(p for p in (getattr(sys, "exec_prefix", None), getattr(sys, "prefix", None)) if p)

    # Step A: current process is already running under Fusion's embedded Python prefix
    for prefix in prefixes:
        if not _is_fusion_prefix(prefix):
            continue
        for cand in _prefix_python_candidates(prefix, plat):
            if _is_python_executable(cand):
                return cand

    # Step B: Autodesk webdeploy scan, sorted newest mtime first
    found: List[Tuple[float, str]] = []
    for pattern in _webdeploy_glob_patterns(plat):
        for match in glob.glob(pattern):
            if _is_python_executable(match):
                try:
                    mtime = os.path.getmtime(match)
                except OSError:
                    mtime = 0.0
                found.append((mtime, match))
    if found:
        found.sort(key=lambda item: item[0], reverse=True)
        return found[0][1]

    # Step C: fallback outside Fusion if Fusion is not installed on disk
    for prefix in prefixes:
        for cand in _prefix_python_candidates(prefix, plat):
            if _is_python_executable(cand):
                return cand

    if _is_python_executable(getattr(sys, "executable", None)):
        return sys.executable

    return None


def _find_ensurepip_wheels(python_exe: str) -> List[str]:
    """Locate bundled ensurepip pip-*.whl files for python_exe without modifying its site-packages."""
    wheels: List[str] = []
    try:
        proc = subprocess.run(
            [python_exe, "-c", "import ensurepip, os; print(os.path.dirname(ensurepip.__file__))"],
            capture_output=True,
            text=True,
            timeout=15,
            **_subprocess_kwargs(),
        )
        if proc.returncode == 0:
            ensurepip_dir = proc.stdout.strip().splitlines()[-1].strip() if proc.stdout.strip() else ""
            if ensurepip_dir:
                bundled = os.path.join(ensurepip_dir, "_bundled")
                wheels.extend(sorted(glob.glob(os.path.join(bundled, "pip-*.whl"))))
    except Exception:
        pass

    if not wheels and python_exe:
        exe_dir = os.path.dirname(os.path.abspath(python_exe))
        candidates = [
            os.path.join(exe_dir, "Lib", "ensurepip", "_bundled", "pip-*.whl"),
            os.path.join(os.path.dirname(exe_dir), "Lib", "ensurepip", "_bundled", "pip-*.whl"),
            os.path.join(os.path.dirname(exe_dir), "lib", "python*", "ensurepip", "_bundled", "pip-*.whl"),
        ]
        for pat in candidates:
            wheels.extend(sorted(glob.glob(pat)))

    # Deduplicate preserving order
    seen = set()
    unique: List[str] = []
    for w in wheels:
        norm = os.path.abspath(w)
        if norm not in seen and os.path.isfile(norm):
            seen.add(norm)
            unique.append(norm)
    return unique


def build_pip_command_and_env(python_exe: str) -> Tuple[List[str], Dict[str, str]]:
    """Build `[python_exe, '-m', 'pip']` and env using ensurepip's bundled wheel if pip is absent."""
    env = os.environ.copy()
    try:
        probe = subprocess.run(
            [python_exe, "-c", "import pip"],
            env=env,
            capture_output=True,
            text=True,
            timeout=15,
            **_subprocess_kwargs(),
        )
        if probe.returncode == 0:
            return [python_exe, "-m", "pip"], env
    except Exception:
        pass

    wheels = _find_ensurepip_wheels(python_exe)
    if wheels:
        existing_pp = env.get("PYTHONPATH", "")
        parts = list(wheels) + ([existing_pp] if existing_pp else [])
        env["PYTHONPATH"] = os.pathsep.join(parts)

    return [python_exe, "-m", "pip"], env


def query_python_abi_tag(python_exe: str) -> str:
    """Query the target Python executable's platform, machine, and major/minor version ABI tag."""
    try:
        proc = subprocess.run(
            [
                python_exe,
                "-c",
                "import sys, platform, json; print(json.dumps([sys.version_info[0], sys.version_info[1], sys.platform, platform.machine()]))",
            ],
            capture_output=True,
            text=True,
            timeout=15,
            **_subprocess_kwargs(),
        )
        if proc.returncode == 0 and proc.stdout.strip():
            maj, min_, plat, mach = json.loads(proc.stdout.strip().splitlines()[-1])
            return get_abi_tag((maj, min_), plat=plat, machine=mach)
    except Exception:
        pass
    return get_abi_tag()


def _purge_module(pkg: str) -> None:
    """Remove pkg and any loaded submodules from sys.modules."""
    prefix = f"{pkg}."
    for mod_name in [k for k in sys.modules if k == pkg or k.startswith(prefix)]:
        sys.modules.pop(mod_name, None)


def check_missing_runtime_packages(
    python_exe: Optional[str] = None,
    extra_sys_paths: Optional[Sequence[str]] = None,
) -> List[str]:
    """Return list of required runtime packages not genuinely importable (including C extensions)."""
    if python_exe is None:
        missing: List[str] = []
        added_paths: List[str] = []
        try:
            for p in reversed(list(extra_sys_paths or ())):
                if p and p not in sys.path:
                    sys.path.insert(0, p)
                    added_paths.append(p)
            importlib.invalidate_caches()
            for pkg in REQUIRED_RUNTIME_PACKAGES:
                if pkg in sys.modules:
                    mod_file = getattr(sys.modules[pkg], "__file__", None)
                    if isinstance(mod_file, str) and not os.path.exists(mod_file):
                        disk_spec = importlib.machinery.PathFinder.find_spec(pkg, sys.path)
                        if disk_spec is None or not disk_spec.origin or not os.path.exists(disk_spec.origin):
                            missing.append(pkg)
                        continue
                try:
                    importlib.import_module(pkg)
                except Exception:
                    _purge_module(pkg)
                    missing.append(pkg)
        finally:
            for p in added_paths:
                try:
                    sys.path.remove(p)
                except ValueError:
                    pass
        return missing

    paths_json = json.dumps(list(extra_sys_paths or ()))
    pkgs_json = json.dumps(list(REQUIRED_RUNTIME_PACKAGES))
    snippet = (
        "import sys, importlib, json\n"
        f"sys.path[:0] = {paths_json}\n"
        "missing = []\n"
        f"for p in {pkgs_json}:\n"
        "    try:\n"
        "        importlib.import_module(p)\n"
        "    except Exception:\n"
        "        missing.append(p)\n"
        "print(json.dumps(missing))\n"
    )
    try:
        proc = subprocess.run(
            [python_exe, "-c", snippet],
            capture_output=True,
            text=True,
            timeout=20,
            **_subprocess_kwargs(),
        )
        if proc.returncode == 0 and proc.stdout.strip():
            return list(json.loads(proc.stdout.strip().splitlines()[-1]))
    except Exception:
        pass
    return list(REQUIRED_RUNTIME_PACKAGES)


def install_runtime_packages(
    addon_dir: str,
    python_exe: Optional[str] = None,
    req_file: Optional[str] = None,
) -> Tuple[bool, str, str]:
    """Install binary runtime dependencies into addon_dir/packages/<abi_tag> using Fusion Python."""
    resolved_py = python_exe or find_fusion_python_executable()
    if not resolved_py:
        return (False, "", "Could not locate Fusion 360 Python executable.")

    resolved_req = req_file or os.path.join(addon_dir, "requirements.txt")
    if not os.path.isfile(resolved_req):
        return (False, "", f"Requirements file not found: {resolved_req}")

    abi_tag = query_python_abi_tag(resolved_py)
    target_dir = os.path.join(addon_dir, "packages", abi_tag)
    os.makedirs(target_dir, exist_ok=True)

    pip_cmd, env = build_pip_command_and_env(resolved_py)
    cmd = pip_cmd + [
        "install",
        "--retries",
        "1",
        "--timeout",
        "5",
        "--only-binary=:all:",
        "--no-compile",
        "--upgrade",
        "--disable-pip-version-check",
        "--no-warn-script-location",
        "--target",
        target_dir,
        "-r",
        resolved_req,
    ]
    try:
        proc = subprocess.run(
            cmd,
            env=env,
            capture_output=True,
            text=True,
            timeout=300,
            **_subprocess_kwargs(),
        )
    except Exception as exc:
        return (False, target_dir, f"pip execution failed: {exc}")

    output = ((proc.stdout or "") + "\n" + (proc.stderr or "")).strip()
    if proc.returncode != 0:
        return (False, target_dir, output or f"pip exited with code {proc.returncode}")

    if sys.platform == "darwin":
        try:
            subprocess.run(["xattr", "-cr", target_dir], capture_output=True, timeout=30)
        except Exception:
            pass

    return (True, target_dir, output or f"Installed dependencies into {target_dir}")
