import os
import sys
import tempfile
import unittest
import zipfile
from unittest.mock import MagicMock, patch

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)

import dep_bootstrap
from scripts import build_release


class TestDepBootstrapAndRelease(unittest.TestCase):
    """Unit tests for dep_bootstrap.py, _EnsureLocalModulesLoaded lifecycle, and scripts/build_release.py."""

    def test_get_abi_tag_cross_platform(self):
        self.assertEqual(
            dep_bootstrap.get_abi_tag(py_ver=(3, 14), plat="darwin", machine="arm64"),
            "darwin_arm64_py314",
        )
        self.assertEqual(
            dep_bootstrap.get_abi_tag(py_ver=(3, 12), plat="darwin", machine="x86_64"),
            "darwin_x86_64_py312",
        )
        self.assertEqual(
            dep_bootstrap.get_abi_tag(py_ver="3.15", plat="win32", machine="AMD64"),
            "win32_amd64_py315",
        )
        self.assertEqual(
            dep_bootstrap.get_abi_tag(py_ver=(3, 14), plat="win32", machine="x86_64"),
            "win32_amd64_py314",
        )

    def test_configure_packages_sys_path_prefers_abi_and_falls_back_to_legacy(self):
        with tempfile.TemporaryDirectory() as addon_dir:
            abi_dir = dep_bootstrap.get_abi_packages_dir(addon_dir)
            flat_dir = os.path.join(addon_dir, "packages")
            os.makedirs(os.path.join(flat_dir, "numpy"), exist_ok=True)

            saved_path = list(sys.path)
            try:
                sys.path[:] = [addon_dir]
                added_legacy = dep_bootstrap.configure_packages_sys_path(addon_dir)
                self.assertEqual(added_legacy, [flat_dir])
                self.assertIn(flat_dir, sys.path)

                # 2. Empty abi_dir still falls back to legacy flat_dir until populated;
                # once populated, abi_dir moves to front and flat_dir is removed from sys.path
                os.makedirs(abi_dir, exist_ok=True)
                self.assertEqual(dep_bootstrap.configure_packages_sys_path(addon_dir), [flat_dir])
                self.assertIn(flat_dir, sys.path)

                os.makedirs(os.path.join(abi_dir, "numpy"), exist_ok=True)
                added_abi = dep_bootstrap.configure_packages_sys_path(addon_dir)
                self.assertEqual(added_abi, [abi_dir])
                self.assertEqual(sys.path[1], abi_dir)
                self.assertNotIn(flat_dir, sys.path)
            finally:
                sys.path[:] = saved_path

    def test_check_missing_runtime_packages_detects_abi_mismatch_and_deleted_cache(self):
        with tempfile.TemporaryDirectory() as addon_dir:
            flat_dir = os.path.join(addon_dir, "packages")
            fake_numpy_dir = os.path.join(flat_dir, "numpy")
            os.makedirs(fake_numpy_dir, exist_ok=True)
            with open(os.path.join(fake_numpy_dir, "__init__.py"), "w", encoding="utf-8") as f:
                f.write('raise ImportError("ABI mismatch: compiled for Python 3.13")\n')

            # Subprocess check must attempt real import and report numpy as missing despite __init__.py existing
            missing_sub = dep_bootstrap.check_missing_runtime_packages(
                python_exe=sys.executable,
                extra_sys_paths=[flat_dir],
            )
            self.assertIn("numpy", missing_sub)

            # In-process check must also detect numpy ABI failure and not leave broken module cached
            saved_mods = {
                k: v for k, v in sys.modules.items()
                if k == "numpy" or k.startswith("numpy.")
            }
            try:
                for k in list(saved_mods):
                    sys.modules.pop(k, None)
                missing_inproc = dep_bootstrap.check_missing_runtime_packages(extra_sys_paths=[flat_dir])
                self.assertIn("numpy", missing_inproc)
                self.assertNotIn("numpy", sys.modules)

                # Simulate Fusion's running process where numpy was already loaded in RAM,
                # then packages/ was moved away on disk (mod_file no longer exists) and later repaired in abi_dir
                import types
                cached_numpy = types.ModuleType("numpy")
                cached_numpy.__file__ = os.path.join(addon_dir, "packages.sav", "numpy", "__init__.py")
                sys.modules["numpy"] = cached_numpy

                saved_sys_path = list(sys.path)
                try:
                    sys.path[:] = []
                    missing_moved = dep_bootstrap.check_missing_runtime_packages()
                    self.assertIn("numpy", missing_moved)
                    # Must leave already-dlopen'd module intact in sys.modules to avoid C-ext conflicts
                    self.assertIs(sys.modules.get("numpy"), cached_numpy)

                    abi_dir = os.path.join(addon_dir, "packages", dep_bootstrap.get_abi_tag())
                    abi_numpy_dir = os.path.join(abi_dir, "numpy")
                    os.makedirs(abi_numpy_dir, exist_ok=True)
                    with open(os.path.join(abi_numpy_dir, "__init__.py"), "w", encoding="utf-8") as f:
                        f.write("__version__ = '2.0.0'\n")

                    missing_repaired = dep_bootstrap.check_missing_runtime_packages(extra_sys_paths=[abi_dir])
                    self.assertNotIn("numpy", missing_repaired)
                    self.assertIs(sys.modules.get("numpy"), cached_numpy)
                finally:
                    sys.path[:] = saved_sys_path
            finally:
                for k in [m for m in sys.modules if m == "numpy" or m.startswith("numpy.")]:
                    sys.modules.pop(k, None)
                sys.modules.update(saved_mods)

    def test_find_fusion_python_executable_external_vs_active_vs_newest_webdeploy(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            # Host binaries (Fusion360.exe / Autodesk Fusion) must be rejected
            fake_prefix = os.path.join(tmp_dir, "host_prefix")
            os.makedirs(os.path.join(fake_prefix, "bin"), exist_ok=True)
            fake_fusion_bin = os.path.join(fake_prefix, "bin", "Autodesk Fusion")
            with open(fake_fusion_bin, "w", encoding="utf-8") as f:
                f.write("host")

            old_py = os.path.join(
                tmp_dir, "mac", "webdeploy", "production", "build1",
                "Autodesk Fusion.app", "Contents", "Frameworks", "Python.framework",
                "Versions", "3.14", "bin", "python3",
            )
            new_py = os.path.join(
                tmp_dir, "mac", "webdeploy", "production", "build2",
                "Autodesk Fusion.app", "Contents", "Frameworks", "Python.framework",
                "Versions", "3.14", "bin", "python3",
            )
            for p, ts in ((old_py, 1000.0), (new_py, 2000.0)):
                os.makedirs(os.path.dirname(p), exist_ok=True)
                with open(p, "w", encoding="utf-8") as f:
                    f.write("#!/bin/sh\n")
                os.utime(p, (ts, ts))

            mac_glob = os.path.join(
                tmp_dir, "mac", "webdeploy", "production", "*",
                "Autodesk Fusion.app", "Contents", "Frameworks", "Python.framework",
                "Versions", "*", "bin", "python3",
            )
            with patch.object(dep_bootstrap.sys, "platform", "darwin"), \
                 patch.object(dep_bootstrap.os, "name", "posix"), \
                 patch.object(dep_bootstrap.sys, "exec_prefix", fake_prefix), \
                 patch.object(dep_bootstrap.sys, "prefix", fake_prefix), \
                 patch.object(dep_bootstrap.sys, "executable", fake_fusion_bin), \
                 patch("dep_bootstrap._webdeploy_glob_patterns", return_value=[mac_glob]):
                self.assertEqual(dep_bootstrap.find_fusion_python_executable(), new_py)

            # Windows webdeploy scan rejects Fusion360.exe and picks newest Python/python.exe
            win_prefix = os.path.join(tmp_dir, "win_host")
            os.makedirs(win_prefix, exist_ok=True)
            win_host_exe = os.path.join(win_prefix, "Fusion360.exe")
            with open(win_host_exe, "w", encoding="utf-8") as f:
                f.write("exe")
            win_old = os.path.join(tmp_dir, "win", "webdeploy", "production", "id1", "Python", "python.exe")
            win_new = os.path.join(tmp_dir, "win", "webdeploy", "production", "id2", "Python", "python.exe")
            for p, ts in ((win_old, 1000.0), (win_new, 3000.0)):
                os.makedirs(os.path.dirname(p), exist_ok=True)
                with open(p, "w", encoding="utf-8") as f:
                    f.write("exe")
                os.utime(p, (ts, ts))

            win_glob = os.path.join(tmp_dir, "win", "webdeploy", "production", "*", "Python", "python.exe")
            with patch.object(dep_bootstrap.sys, "platform", "win32"), \
                 patch.object(dep_bootstrap.sys, "exec_prefix", win_prefix), \
                 patch.object(dep_bootstrap.sys, "prefix", win_prefix), \
                 patch.object(dep_bootstrap.sys, "executable", win_host_exe), \
                 patch("dep_bootstrap._webdeploy_glob_patterns", return_value=[win_glob]):
                self.assertEqual(dep_bootstrap.find_fusion_python_executable(), win_new)

            self.assertFalse(dep_bootstrap._is_python_executable(win_host_exe))
            self.assertFalse(dep_bootstrap._is_python_executable(fake_fusion_bin))

            # External non-Fusion Python prefix must NOT shadow Fusion webdeploy Python
            ext_prefix = os.path.join(tmp_dir, "external_venv_py39")
            os.makedirs(os.path.join(ext_prefix, "bin"), exist_ok=True)
            ext_py = os.path.join(ext_prefix, "bin", "python3")
            with open(ext_py, "w", encoding="utf-8") as f:
                f.write("#!/bin/sh\n")

            with patch.object(dep_bootstrap.sys, "platform", "darwin"), \
                 patch.object(dep_bootstrap.os, "name", "posix"), \
                 patch.object(dep_bootstrap.sys, "exec_prefix", ext_prefix), \
                 patch.object(dep_bootstrap.sys, "prefix", ext_prefix), \
                 patch.object(dep_bootstrap.sys, "executable", ext_py), \
                 patch("dep_bootstrap._webdeploy_glob_patterns", return_value=[mac_glob]):
                self.assertEqual(dep_bootstrap.find_fusion_python_executable(), new_py)

            # Active Fusion prefix (Step A) returns that exact active build even if build2 is newer
            active_fusion_prefix = os.path.dirname(os.path.dirname(old_py))
            with patch.object(dep_bootstrap.sys, "platform", "darwin"), \
                 patch.object(dep_bootstrap.os, "name", "posix"), \
                 patch.object(dep_bootstrap.sys, "exec_prefix", active_fusion_prefix), \
                 patch.object(dep_bootstrap.sys, "prefix", active_fusion_prefix), \
                 patch("dep_bootstrap._webdeploy_glob_patterns", return_value=[mac_glob]):
                self.assertEqual(dep_bootstrap.find_fusion_python_executable(), old_py)

    def test_build_pip_command_and_env_injects_bundled_wheel_without_mutating_python(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            py_dir = os.path.join(tmp_dir, "Python", "bin")
            bundled_dir = os.path.join(tmp_dir, "Python", "lib", "python3.14", "ensurepip", "_bundled")
            os.makedirs(py_dir, exist_ok=True)
            os.makedirs(bundled_dir, exist_ok=True)
            fake_py = os.path.join(py_dir, "python3")
            with open(fake_py, "w", encoding="utf-8") as f:
                f.write("")
            fake_whl = os.path.join(bundled_dir, "pip-24.2-py3-none-any.whl")
            with open(fake_whl, "w", encoding="utf-8") as f:
                f.write("zip")

            def _fake_run(cmd, **kwargs):
                if cmd[1:3] == ["-c", "import pip"]:
                    return MagicMock(returncode=1, stdout="", stderr="No module named pip")
                if "ensurepip" in cmd[2]:
                    return MagicMock(returncode=0, stdout=os.path.dirname(bundled_dir) + "\n", stderr="")
                return MagicMock(returncode=0, stdout="", stderr="")

            with patch("dep_bootstrap.subprocess.run", side_effect=_fake_run), \
                 patch.dict(os.environ, {"PYTHONPATH": "/existing/path"}, clear=False):
                cmd, env = dep_bootstrap.build_pip_command_and_env(fake_py)

            self.assertEqual(cmd, [fake_py, "-m", "pip"])
            self.assertIn("PYTHONPATH", env)
            pp_parts = env["PYTHONPATH"].split(os.pathsep)
            self.assertEqual(os.path.abspath(pp_parts[0]), os.path.abspath(fake_whl))
            self.assertEqual(pp_parts[-1], "/existing/path")

    def test_ensure_local_modules_loaded_startup_vs_interactive_and_retry(self):
        # Ensure PostClear's stub adsk is loaded via test_postclear_pipeline_integration first or directly
        import tests.test_postclear_pipeline_integration  # noqa: F401
        import PostClear

        adsk_mod = sys.modules["adsk"]
        adsk_core_mod = sys.modules.get("adsk.core") or getattr(adsk_mod, "core", None)
        had_file = hasattr(adsk_mod, "__file__")
        prev_file = getattr(adsk_mod, "__file__", None)
        had_core_file = hasattr(adsk_core_mod, "__file__") if adsk_core_mod is not None else False
        prev_core_file = getattr(adsk_core_mod, "__file__", None) if adsk_core_mod is not None else None
        saved_attempted = PostClear._deps_bootstrap_attempted
        saved_warned = PostClear._deps_bootstrap_warned_session
        saved_loaded = PostClear._modules_loaded_session
        saved_pytest_env = os.environ.pop("PYTEST_CURRENT_TEST", None)
        saved_autodesk_env = {
            k: os.environ.get(k)
            for k in ("AUTODESK_FUSION_ROOT", "AUTODESK_POST_BINARY", "AUTODESK_SYSTEM_POSTS_DIR")
        }

        mock_progress = MagicMock()
        mock_ui = MagicMock()
        mock_ui.createProgressDialog.return_value = mock_progress
        mock_app = MagicMock()
        mock_app.userInterface = mock_ui

        try:
            # Simulate real Fusion 360 PEP 420 namespace package:
            # adsk.__file__ is None while adsk.core.__file__ is a path on disk, even with "unittest" in sys.modules
            adsk_mod.__file__ = None
            if adsk_core_mod is not None:
                adsk_core_mod.__file__ = "/Applications/Autodesk Fusion.app/Contents/Api/Python/packages/adsk/core.py"
            PostClear._deps_bootstrap_attempted = False
            PostClear._deps_bootstrap_warned_session = False
            PostClear._modules_loaded_session = True  # Avoid live reload of modules during test

            with patch.object(PostClear.adsk.core.Application, "get", return_value=mock_app), \
                 patch("PostClear.InitAddIn"), \
                 patch("PostClear.SettingsManager"), \
                 patch("importlib.reload") as mock_reload, \
                 patch("importlib.invalidate_caches"), \
                 patch("pipeline.find_autodesk_fusion_invoking_root", return_value=None), \
                 patch("pipeline.find_autodesk_post_binary", return_value=None), \
                 patch("pipeline.find_autodesk_system_posts_dir", return_value=None), \
                 patch("dep_bootstrap.configure_packages_sys_path"), \
                 patch("dep_bootstrap.check_missing_runtime_packages", return_value=["shapely"]), \
                 patch("dep_bootstrap.install_runtime_packages", side_effect=[
                     (False, "/tmp/pkg", "offline-1"),
                     (False, "/tmp/pkg", "offline-2"),
                     (True, "/tmp/pkg", "ok"),
                     (True, "/tmp/pkg", "ok"),
                 ]) as mock_install:
                # 1. IsApplicationStartup=True skips bootstrap during splash startup
                PostClear.run({"IsApplicationStartup": True})
                mock_install.assert_not_called()
                self.assertFalse(PostClear._deps_bootstrap_attempted)
                mock_ui.messageBox.assert_not_called()

                # 2. First allow_bootstrap=True call hits offline failure -> hides progress, warns once, stays retryable
                PostClear._EnsureLocalModulesLoaded(force=False, allow_bootstrap=True)
                self.assertEqual(mock_install.call_count, 1)
                self.assertFalse(PostClear._deps_bootstrap_attempted)
                self.assertTrue(PostClear._deps_bootstrap_warned_session)
                mock_progress.hide.assert_called_once()
                mock_ui.messageBox.assert_called_once()
                self.assertIn("could not download verification libraries", mock_ui.messageBox.call_args[0][0])

                # 3. Repeated offline call retries pip, still hides progress, does NOT spam messageBox
                mock_progress.reset_mock()
                PostClear._EnsureLocalModulesLoaded(force=False, allow_bootstrap=True)
                self.assertEqual(mock_install.call_count, 2)
                self.assertFalse(PostClear._deps_bootstrap_attempted)
                mock_progress.hide.assert_called_once()
                self.assertEqual(mock_ui.messageBox.call_count, 1)

                # 4. Interactive run (IsApplicationStartup=False) once online retries, succeeds, marks attempted, clears warning flag
                mock_progress.reset_mock()
                PostClear.run({"IsApplicationStartup": False})
                self.assertEqual(mock_install.call_count, 3)
                self.assertTrue(PostClear._deps_bootstrap_attempted)
                self.assertFalse(PostClear._deps_bootstrap_warned_session)
                mock_progress.hide.assert_called_once()

                # 5. Subsequent non-forced call skips install once _deps_bootstrap_attempted is True
                PostClear._EnsureLocalModulesLoaded(force=False, allow_bootstrap=True)
                self.assertEqual(mock_install.call_count, 3)

                # 6. Regression: when _modules_loaded_session is already True and force=False with progress_cb,
                # a successful bootstrap uses progress_cb (without createProgressDialog) and reloads ordered_mods!
                mock_ui.createProgressDialog.reset_mock()
                mock_reload.reset_mock()
                PostClear._deps_bootstrap_attempted = False
                PostClear._modules_loaded_session = True
                mock_cb = MagicMock(return_value=True)
                PostClear._EnsureLocalModulesLoaded(
                    force=False,
                    allow_bootstrap=True,
                    progress_cb=mock_cb,
                )
                self.assertEqual(mock_install.call_count, 4)
                mock_ui.createProgressDialog.assert_not_called()
                mock_cb.assert_called()
                self.assertIn("First-time setup", mock_cb.call_args[0][0])
                reloaded_names = {
                    getattr(call.args[0], "__name__", "")
                    for call in mock_reload.call_args_list
                    if call.args
                }
                self.assertIn("context_parser", reloaded_names)
                self.assertIn("pipeline", reloaded_names)
        finally:
            if had_file:
                adsk_mod.__file__ = prev_file
            else:
                try:
                    delattr(adsk_mod, "__file__")
                except AttributeError:
                    pass
            if adsk_core_mod is not None:
                if had_core_file:
                    adsk_core_mod.__file__ = prev_core_file
                else:
                    try:
                        delattr(adsk_core_mod, "__file__")
                    except AttributeError:
                        pass
            if saved_pytest_env is not None:
                os.environ["PYTEST_CURRENT_TEST"] = saved_pytest_env
            for k, v in saved_autodesk_env.items():
                if v is None:
                    os.environ.pop(k, None)
                else:
                    os.environ[k] = v
            PostClear._deps_bootstrap_attempted = saved_attempted
            PostClear._deps_bootstrap_warned_session = saved_warned
            PostClear._modules_loaded_session = saved_loaded

    def test_install_runtime_packages_offline_fast_fail_and_no_index(self):
        with tempfile.TemporaryDirectory() as addon_dir, \
             tempfile.TemporaryDirectory() as empty_links:
            req_path = os.path.join(addon_dir, "requirements.txt")
            with open(req_path, "w", encoding="utf-8") as f:
                f.write("shapely>=2.0\nmatplotlib>=3.7\n")

            # 1. Verify fast-fail pip network flags are always passed and errors return (False, target_dir, detail)
            captured_cmds = []

            def _fake_subprocess_run(cmd, **kwargs):
                captured_cmds.append(list(cmd))
                return MagicMock(returncode=1, stdout="", stderr="NewConnectionError: Failed to establish a new connection")

            with patch("dep_bootstrap.build_pip_command_and_env", return_value=([sys.executable, "-m", "pip"], dict(os.environ))), \
                 patch("dep_bootstrap.query_python_abi_tag", return_value="darwin_arm64_py314"), \
                 patch("dep_bootstrap.subprocess.run", side_effect=_fake_subprocess_run):
                ok, target_dir, detail = dep_bootstrap.install_runtime_packages(addon_dir, python_exe=sys.executable)

            self.assertFalse(ok)
            self.assertTrue(target_dir.endswith(os.path.join("packages", "darwin_arm64_py314")))
            self.assertIn("NewConnectionError", detail)
            self.assertEqual(len(captured_cmds), 1)
            cmd = captured_cmds[0]
            self.assertIn("--retries", cmd)
            self.assertEqual(cmd[cmd.index("--retries") + 1], "1")
            self.assertIn("--timeout", cmd)
            self.assertEqual(cmd[cmd.index("--timeout") + 1], "5")

            # 2. Real pip execution against empty find-links directory (offline simulation)
            if dep_bootstrap.build_pip_command_and_env(sys.executable)[0]:
                with patch.dict(os.environ, {"PIP_NO_INDEX": "1", "PIP_FIND_LINKS": empty_links}, clear=False):
                    ok_real, target_real, detail_real = dep_bootstrap.install_runtime_packages(
                        addon_dir,
                        python_exe=sys.executable,
                    )
                self.assertFalse(ok_real)
                self.assertTrue(os.path.isdir(target_real))
                self.assertTrue(len(detail_real) > 0)

    def test_pipeline_offline_fallback_and_turning_error_when_deps_blocked(self):
        import builtins
        import json
        from geometry.models import (
            AxisymmetricStock2D,
            BoxStock3D,
            LatheToolGeometry,
            MillToolGeometry,
            ToolAssembly,
            ToolCompensationQuadrant,
            WorkpieceDefinition,
        )
        from pipeline import PipelineOrchestrator

        blocked_roots = {"numpy", "shapely", "matplotlib"}
        real_import = builtins.__import__
        saved_modules = {}
        mods_to_clear = [
            m for m in list(sys.modules.keys())
            if m in blocked_roots
            or m.startswith(("numpy.", "shapely.", "matplotlib.", "geometry.heightfield_engine", "geometry.lathe_engine", "geometry.mill_engine", "geometry.plotter"))
        ]
        for m in mods_to_clear:
            saved_modules[m] = sys.modules.pop(m)

        def _blocking_import(name, globals=None, locals=None, fromlist=(), level=0):
            root = name.split(".", 1)[0]
            if root in blocked_roots:
                raise ImportError(f"Blocked offline import: {name}")
            return real_import(name, globals, locals, fromlist, level)

        with tempfile.TemporaryDirectory() as tmpdir:
            mill_dump = os.path.join(tmpdir, "mill_ctx.json")
            with open(mill_dump, "w", encoding="utf-8") as f:
                json.dump(
                    {
                        "unit": 0,
                        "stock": {"minX": -1.0, "minY": -1.0, "minZ": -1.0, "maxX": 1.0, "maxY": 1.0, "maxZ": 0.0},
                        "sections": [
                            {
                                "id": 0,
                                "type": 0,
                                "unit": 0,
                                "operationName": "Face1",
                                "zClearance": 0.5,
                                "tool": {"number": 1, "diameter": 0.5, "fluteLength": 1.0, "bodyLength": 1.5, "isMill": True},
                                "events": [
                                    {"idx": 0, "type": "linear", "motion": "rapid", "x": 0.0, "y": 0.0, "z": 0.6},
                                    {"idx": 1, "type": "linear", "motion": "feed", "x": 0.5, "y": 0.0, "z": -0.1, "feed": 20.0},
                                ],
                            }
                        ],
                    },
                    f,
                )

            turn_dump = os.path.join(tmpdir, "turn_ctx.json")
            with open(turn_dump, "w", encoding="utf-8") as f:
                json.dump(
                    {
                        "unit": 0,
                        "sections": [
                            {
                                "id": 0,
                                "type": 1,
                                "unit": 0,
                                "operationName": "Profile1",
                                "tool": {"number": 1, "_isTurningTool": True},
                                "events": [
                                    {"idx": 0, "type": "linear", "motion": "rapid", "x": 1.0, "y": 0.0, "z": 0.5},
                                ],
                            }
                        ],
                    },
                    f,
                )

            mill_wp = WorkpieceDefinition(stock=BoxStock3D(x_min=-25.4, x_max=25.4, y_min=-25.4, y_max=25.4, z_min=-25.4, z_max=0.0))
            mill_tool = ToolAssembly(
                tool_id=1,
                name="T1",
                geometry=MillToolGeometry(diameter=12.7, corner_radius=0.0, flute_length=25.4, overall_length=38.1),
            )
            turn_wp = WorkpieceDefinition(stock=AxisymmetricStock2D(z_min=-50.0, z_max=0.0, outer_radius=25.0))
            turn_tool = ToolAssembly(
                tool_id=1,
                name="T1",
                geometry=LatheToolGeometry(
                    insert_shape="C",
                    nose_radius=0.4,
                    hand="R",
                    quadrant=ToolCompensationQuadrant.P3,
                    cutter_polygon=None,
                    holder_polygon=None,
                ),
            )

            base_post = os.path.join(REPO_ROOT, "universal_replayer.cps")
            final_cnc = os.path.join(tmpdir, "out.nc")
            plot_png = os.path.join(tmpdir, "out.png")

            orchestrator = PipelineOrchestrator()
            try:
                with patch.object(builtins, "__import__", side_effect=_blocking_import), \
                     patch.object(orchestrator, "replay_combined_context", return_value="G0 Z0.6\nM30\n"):
                    # Milling falls back cleanly to MillClearanceVerifier and does not crash on plot generation
                    report, gcode = orchestrator.run(
                        context_dump_paths=[mill_dump],
                        base_post_path=base_post,
                        workpiece=mill_wp,
                        tools_by_op={"Face1": mill_tool},
                        final_cnc_path=final_cnc,
                        plot_output_path=plot_png,
                        is_turning=False,
                    )
                    self.assertIn("M30", gcode)
                    self.assertIsNotNone(report)

                    # Turning raises clean actionable RuntimeError instead of raw ModuleNotFoundError
                    with self.assertRaises(RuntimeError) as ctx:
                        orchestrator.run(
                            context_dump_paths=[turn_dump],
                            base_post_path=base_post,
                            workpiece=turn_wp,
                            tools_by_op={"Profile1": turn_tool},
                            final_cnc_path=final_cnc,
                            is_turning=True,
                        )
                    self.assertIn("Turning 2D IPW verification requires 'shapely'", str(ctx.exception))
                    self.assertIn("offline first-time setup failed", str(ctx.exception))
            finally:
                for m in list(sys.modules.keys()):
                    if m.startswith(("geometry.heightfield_engine", "geometry.lathe_engine", "geometry.mill_engine", "geometry.plotter")):
                        sys.modules.pop(m, None)
                sys.modules.update(saved_modules)

    def test_build_release_creates_script_free_runtime_zip(self):
        with tempfile.TemporaryDirectory() as out_dir:
            zip_path = build_release.build_release(source_dir=REPO_ROOT, output_dir=out_dir)
            self.assertTrue(os.path.isfile(zip_path))
            self.assertEqual(os.path.basename(zip_path), "PostClear-v3.0.0.zip")

            with zipfile.ZipFile(zip_path, "r") as zf:
                names = set(zf.namelist())
                for req_name in (
                    "PostClear/PostClear.py",
                    "PostClear/PostClear.manifest",
                    "PostClear/dep_bootstrap.py",
                    "PostClear/context_parser.py",
                    "PostClear/pipeline.py",
                    "PostClear/universal_dumper.cps",
                    "PostClear/universal_replayer.cps",
                    "PostClear/requirements.txt",
                    "PostClear/README.md",
                ):
                    self.assertIn(req_name, names)

                # Must exclude all installer scripts, tests, dev files, legacy posts, settings, and local packages/
                self.assertFalse(any(n.startswith("PostClear/packages/") for n in names))
                self.assertFalse(any(n.endswith((".bat", ".command")) for n in names))
                self.assertFalse(any(os.path.basename(n).startswith("install_") for n in names))
                self.assertFalse(any(n.startswith("PostClear/tests/") for n in names))
                self.assertFalse(any(n.startswith("PostClear/.venv/") for n in names))
                self.assertFalse(any(n.startswith("PostClear/docs/") for n in names))
                self.assertFalse(any(n.endswith("centroid_turning.cps") for n in names))
                self.assertFalse(any(n.endswith(".settings") for n in names))

                # Text files normalized to LF
                self.assertNotIn(b"\r\n", zf.read("PostClear/PostClear.py"))

                # Manifest UUID v4, author, and version checks
                import json as _json
                import uuid as _uuid
                manifest = _json.loads(zf.read("PostClear/PostClear.manifest").decode("utf-8"))
                self.assertEqual(manifest["author"], "Vadim Furman, agy")
                self.assertEqual(manifest["version"], "3.0.0")
                self.assertNotEqual(manifest["id"], "fec886de-a491-46e4-84c5-2bc0d4549040")
                parsed_uuid = _uuid.UUID(manifest["id"])
                self.assertEqual(parsed_uuid.version, 4)

                # Core PostClear / Post Clear constants
                import tests.test_postclear_pipeline_integration  # noqa: F401
                import PostClear
                self.assertEqual(PostClear.constCmdName, "Post Clear")
                self.assertEqual(PostClear.constCmdDefId, "PostClear_Cmd")
                self.assertFalse(hasattr(PostClear, "constNcProgramName"))
                self.assertEqual(PostClear.constNotificationPaletteId, "PostClear_NotificationPalette")


if __name__ == "__main__":
    unittest.main()
