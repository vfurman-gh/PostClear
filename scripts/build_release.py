#!/usr/bin/env python3
"""Standalone cross-platform release packager for PostClear.

Creates dist/PostClear-v<version>.zip (or PostClear-v<version>-<abi>.zip
with --bundle-wheels) containing clean runtime source files under top-level
PostClear/ without any installer scripts or dev-only directories.
"""

import argparse
import json
import os
import shutil
import stat
import subprocess
import sys
import tempfile
import zipfile
from typing import Optional

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

import dep_bootstrap


ADDIN_NAME = "PostClear"

RUNTIME_TOP_FILES = (
    "PostClear.py",
    "PostClear.manifest",
    "dep_bootstrap.py",
    "context_parser.py",
    "pipeline.py",
    "universal_dumper.cps",
    "universal_replayer.cps",
    "requirements.txt",
    "README.md",
    "LICENSE",
)

LF_TEXT_EXTS = (".py", ".cps", ".manifest", ".md", ".txt", ".svg")


def _read_manifest_version(source_dir: str) -> str:
    manifest_path = os.path.join(source_dir, "PostClear.manifest")
    if os.path.isfile(manifest_path):
        try:
            with open(manifest_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            ver = str(data.get("version", "")).strip()
            if ver:
                return ver
        except Exception:
            pass
    return "3.0.0"


def _abi_from_platform_tag(platform_tag: Optional[str], python_version: str) -> str:
    if not platform_tag:
        return dep_bootstrap.get_abi_tag(py_ver=python_version)
    tag = platform_tag.strip().lower()
    if "win" in tag:
        mach = "arm64" if "arm64" in tag else "amd64"
        return dep_bootstrap.get_abi_tag(py_ver=python_version, plat="win32", machine=mach)
    if "macosx" in tag or "darwin" in tag:
        mach = "arm64" if ("arm64" in tag or "aarch64" in tag) else "x86_64"
        return dep_bootstrap.get_abi_tag(py_ver=python_version, plat="darwin", machine=mach)
    mach = "arm64" if ("arm64" in tag or "aarch64" in tag) else "x86_64"
    return dep_bootstrap.get_abi_tag(py_ver=python_version, plat="linux", machine=mach)


def _copy_filtered_tree(src_root: str, dst_root: str, allowed_ext: Optional[str] = None) -> None:
    """Copy a directory tree skipping .DS_Store, __pycache__, and *.pyc/*.pyo files."""
    os.makedirs(dst_root, exist_ok=True)
    for root, dirs, files in os.walk(src_root):
        dirs[:] = [d for d in dirs if d not in ("__pycache__", ".git", ".venv")]
        rel = os.path.relpath(root, src_root)
        target_sub = dst_root if rel == "." else os.path.join(dst_root, rel)
        os.makedirs(target_sub, exist_ok=True)
        for fname in files:
            if fname == ".DS_Store" or fname.endswith((".pyc", ".pyo")):
                continue
            if allowed_ext and not fname.endswith(allowed_ext):
                continue
            shutil.copy2(os.path.join(root, fname), os.path.join(target_sub, fname))


def _stage_runtime_files(source_dir: str, dest_dir: str) -> None:
    """Stage only runtime files (excluding tests, .venv, docs, scripts, .settings, legacy posts)."""
    os.makedirs(dest_dir, exist_ok=True)

    for fname in RUNTIME_TOP_FILES:
        src_file = os.path.join(source_dir, fname)
        if os.path.isfile(src_file):
            shutil.copy2(src_file, os.path.join(dest_dir, fname))

    geom_src = os.path.join(source_dir, "geometry")
    if os.path.isdir(geom_src):
        _copy_filtered_tree(geom_src, os.path.join(dest_dir, "geometry"), allowed_ext=".py")

    res_src = os.path.join(source_dir, "resources")
    if os.path.isdir(res_src):
        _copy_filtered_tree(res_src, os.path.join(dest_dir, "resources"))


def _write_zip_entry(zf: zipfile.ZipFile, src_file: str, arcname: str) -> None:
    info = zipfile.ZipInfo.from_file(src_file, arcname)
    info.create_system = 3
    info.external_attr = ((stat.S_IFREG | 0o644) << 16)
    with open(src_file, "rb") as f:
        data = f.read()
    if arcname.endswith(LF_TEXT_EXTS):
        data = data.replace(b"\r\n", b"\n")
    zf.writestr(info, data, compress_type=zipfile.ZIP_DEFLATED)


def build_release(
    source_dir: str = REPO_ROOT,
    output_dir: Optional[str] = None,
    bundle_wheels: bool = False,
    platform_tag: Optional[str] = None,
    python_version: str = "3.14",
) -> str:
    """Build clean release zip and return its absolute file path."""
    source_dir = os.path.abspath(source_dir)
    dist_dir = os.path.abspath(output_dir if output_dir else os.path.join(source_dir, "dist"))
    os.makedirs(dist_dir, exist_ok=True)

    version = _read_manifest_version(source_dir)
    abi_tag = _abi_from_platform_tag(platform_tag, python_version) if bundle_wheels else None
    zip_name = f"{ADDIN_NAME}-v{version}-{abi_tag}.zip" if abi_tag else f"{ADDIN_NAME}-v{version}.zip"
    zip_path = os.path.join(dist_dir, zip_name)

    with tempfile.TemporaryDirectory() as tmp_stage:
        stage_root = os.path.join(tmp_stage, ADDIN_NAME)
        _stage_runtime_files(source_dir, stage_root)

        if bundle_wheels and abi_tag:
            req_file = os.path.join(source_dir, "requirements.txt")
            wheel_dest = os.path.join(stage_root, "packages", abi_tag)
            os.makedirs(wheel_dest, exist_ok=True)
            pip_cmd = [
                sys.executable,
                "-m",
                "pip",
                "install",
                "--target",
                wheel_dest,
                "--python-version",
                python_version,
                "--implementation",
                "cp",
                "--only-binary=:all:",
                "--no-compile",
                "--upgrade",
                "-r",
                req_file,
            ]
            if platform_tag:
                pip_cmd.extend(["--platform", platform_tag])
            subprocess.run(pip_cmd, check=True, **dep_bootstrap._subprocess_kwargs())

        with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
            for root, dirs, files in os.walk(stage_root):
                dirs[:] = sorted(d for d in dirs if d not in ("__pycache__", ".git", ".venv"))
                for fname in sorted(files):
                    if fname == ".DS_Store" or fname.endswith((".pyc", ".pyo")):
                        continue
                    full_path = os.path.join(root, fname)
                    rel_path = os.path.relpath(full_path, tmp_stage).replace(os.sep, "/")
                    _write_zip_entry(zf, full_path, rel_path)

    return zip_path


def main() -> int:
    parser = argparse.ArgumentParser(description="Build PostClear release archive")
    parser.add_argument("--output-dir", default=None, help="Output directory (defaults to dist/)")
    parser.add_argument("--bundle-wheels", action="store_true", help="Bundle binary wheels into packages/<abi>/")
    parser.add_argument("--platform-tag", default=None, help="pip --platform tag for --bundle-wheels")
    parser.add_argument("--python-version", default="3.14", help="pip --python-version for --bundle-wheels")
    args = parser.parse_args()

    out_zip = build_release(
        source_dir=REPO_ROOT,
        output_dir=args.output_dir,
        bundle_wheels=args.bundle_wheels,
        platform_tag=args.platform_tag,
        python_version=args.python_version,
    )
    print(f"[+] Built release archive: {out_zip}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
