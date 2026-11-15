#!/usr/bin/env python3
"""
Test Suite: Batch 5 Environment & Selection Correctness (Phase 2 H3, H4, M1).

Validates:
- H3: find_fusion_root_from_path resolves usernames with dots (e.g. 'john.doe')
      and non-ASCII characters (e.g. 'Jörg') across macOS and Windows paths.
      find_autodesk_post_binary and find_autodesk_system_posts_dir fail closed with
      FileNotFoundError (naming inspected paths and override env vars) when undetectable.
- H4: _ResolveBasePostPath treats exact paths and file:// URLs as authoritative.
      User/cloud post directories (Tier 1) are preferred over system posts (Tier 2).
      Ambiguous name or description matches within the same tier raise ValueError
      naming all candidate files.
      _GetPostCapabilities scans the entire file and detects capabilities past 16 KB.
- M1: _WriteDumperConfig writes exclusively to the per-run 0o700 mkdtemp directory
      passed via DUMPER_CONFIG_PATH, without writing to $TMPDIR or /tmp.
      universal_dumper.cps reads only DUMPER_CONFIG_PATH without $TMPDIR//tmp fallbacks.
"""

import os
import shutil
import stat
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

# Mock adsk namespace if running outside Fusion
if "adsk" not in sys.modules:
    adsk = types.ModuleType("adsk")
    adsk.cam = types.ModuleType("adsk.cam")
    adsk.cam.OperationTypes = types.SimpleNamespace(TurningOperation=1, MillingOperation=0)
    adsk.cam.Setup = type("Setup", (), {})
    adsk.cam.Operation = type("Operation", (), {})
    adsk.cam.CAMFolder = type("CAMFolder", (), {})
    adsk.cam.CAMPattern = type("CAMPattern", (adsk.cam.CAMFolder,), {})
    adsk.cam.CAM = types.SimpleNamespace(cast=lambda x: x)
    adsk.core = types.ModuleType("adsk.core")
    adsk.core.CommandCreatedEventHandler = type("CommandCreatedEventHandler", (), {})
    adsk.core.InputChangedEventHandler = type("InputChangedEventHandler", (), {})
    adsk.core.ValidateInputsEventHandler = type("ValidateInputsEventHandler", (), {})
    adsk.core.CommandEventHandler = type("CommandEventHandler", (), {})
    adsk.core.Application = types.SimpleNamespace(get=lambda: types.SimpleNamespace(log=lambda msg: None))
    adsk.fusion = types.ModuleType("adsk.fusion")
    sys.modules["adsk"] = adsk
    sys.modules["adsk.cam"] = adsk.cam
    sys.modules["adsk.core"] = adsk.core
    sys.modules["adsk.fusion"] = adsk.fusion
else:
    import adsk
    import adsk.cam
    if not hasattr(adsk.cam, "CAMFolder"):
        adsk.cam.CAMFolder = type("CAMFolder", (), {})
    if not hasattr(adsk.cam, "CAMPattern"):
        adsk.cam.CAMPattern = type("CAMPattern", (adsk.cam.CAMFolder,), {})

from pipeline import (
    find_fusion_root_from_path,
    find_autodesk_post_binary,
    find_autodesk_system_posts_dir,
    find_autodesk_fusion_invoking_root,
)
import PostClear


class TestBatch5EnvironmentAndRootDetection(unittest.TestCase):
    """H3 tests: path component parsing and fail-closed install detection."""

    def test_h3_macos_dotted_username_resolves(self):
        """Verify dotted usernames on macOS (e.g. john.doe) resolve to webdeploy root."""
        p = "/Users/john.doe/Library/Application Support/Autodesk/webdeploy/production/b43a07003c01dd54425db484d23b1c5e6248a21d/Autodesk Fusion.app/Contents/MacOS/Fusion360"
        root = find_fusion_root_from_path(p)
        self.assertEqual(
            root,
            "/Users/john.doe/Library/Application Support/Autodesk/webdeploy/production/b43a07003c01dd54425db484d23b1c5e6248a21d",
        )

    def test_h3_macos_multiple_dots_username_resolves(self):
        """Verify usernames with multiple dots or symbols resolve cleanly."""
        p = "/Users/jane.m.doe_v2/Library/Application Support/Autodesk/webdeploy/production/hash_build_999/Autodesk Fusion.app"
        root = find_fusion_root_from_path(p)
        self.assertEqual(
            root,
            "/Users/jane.m.doe_v2/Library/Application Support/Autodesk/webdeploy/production/hash_build_999",
        )

    def test_h3_windows_non_ascii_username_resolves(self):
        """Verify non-ASCII usernames on Windows (e.g. Jörg, José) resolve to webdeploy root."""
        p_win = r"C:\Users\Jörg\AppData\Local\Autodesk\webdeploy\production\b43a07003c01dd54425db484d23b1c5e6248a21d\Applications\CAM360\post.exe"
        root = find_fusion_root_from_path(p_win)
        self.assertEqual(
            root,
            r"C:\Users\Jörg\AppData\Local\Autodesk\webdeploy\production\b43a07003c01dd54425db484d23b1c5e6248a21d",
        )

    def test_h3_windows_forward_slash_non_ascii_resolves(self):
        """Verify Windows paths with forward slashes and non-ASCII characters resolve."""
        p_win_fwd = "C:/Users/José/AppData/Local/Autodesk/webdeploy/production/abc123456789/post.exe"
        root = find_fusion_root_from_path(p_win_fwd)
        self.assertEqual(
            root,
            r"C:\Users\José\AppData\Local\Autodesk\webdeploy\production\abc123456789",
        )

    def test_h3_standalone_app_bundle_resolves(self):
        """Verify standalone /Applications/Autodesk Fusion 360.app resolves to app directory."""
        p_app = "/Applications/Autodesk Fusion 360.app/Contents/Libraries/Applications/CAM360/post"
        root = find_fusion_root_from_path(p_app)
        self.assertEqual(root, "/Applications/Autodesk Fusion 360.app")

    def test_h3_ps_command_line_resolves(self):
        """Verify ps command line with trailing arguments resolves to webdeploy root."""
        p_cmd = "/Users/john.doe/Library/Application Support/Autodesk/webdeploy/production/hash123/Autodesk Fusion.app/Contents/MacOS/Fusion360 --type=renderer --no-sandbox"
        root = find_fusion_root_from_path(p_cmd)
        self.assertEqual(
            root,
            "/Users/john.doe/Library/Application Support/Autodesk/webdeploy/production/hash123",
        )

    def test_h3_undetectable_post_binary_raises_filenotfounderror_with_diagnostics(self):
        """Verify find_autodesk_post_binary fails closed with clear error naming inspected paths and override env vars."""
        with patch.dict(os.environ, {}, clear=True), \
             patch("pipeline.find_autodesk_fusion_invoking_root", return_value=None):
            with self.assertRaises(FileNotFoundError) as cm:
                find_autodesk_post_binary()
            err_msg = str(cm.exception)
            self.assertIn("Could not determine invoking Autodesk Fusion installation", err_msg)
            self.assertIn("AUTODESK_FUSION_ROOT", err_msg)
            self.assertIn("AUTODESK_POST_BINARY", err_msg)
            self.assertIn("sys.executable", err_msg)
            self.assertIn("sys.prefix", err_msg)
            self.assertIn("sys.path", err_msg)

    def test_h3_undetectable_system_posts_raises_filenotfounderror_with_diagnostics(self):
        """Verify find_autodesk_system_posts_dir fails closed with clear error naming inspected paths and override env vars."""
        with patch.dict(os.environ, {}, clear=True), \
             patch("pipeline.find_autodesk_fusion_invoking_root", return_value=None):
            with self.assertRaises(FileNotFoundError) as cm:
                find_autodesk_system_posts_dir()
            err_msg = str(cm.exception)
            self.assertIn("Could not determine invoking Autodesk Fusion installation", err_msg)
            self.assertIn("AUTODESK_FUSION_ROOT", err_msg)
            self.assertIn("AUTODESK_SYSTEM_POSTS_DIR", err_msg)
            self.assertIn("sys.executable", err_msg)


class TestBatch5BasePostResolution(unittest.TestCase):
    """H4 tests: post resolution authority, priority tiers, ambiguity rejection, full-file capabilities."""

    def test_h4_capabilities_defined_past_16kb_detected(self):
        """Verify _GetPostCapabilities scans the whole file and detects capabilities past 16 KB."""
        with tempfile.TemporaryDirectory() as tmp_dir:
            cps_path = os.path.join(tmp_dir, "long_header.cps")
            with open(cps_path, "w", encoding="utf-8") as f:
                # Write 25 KB of header comments / changelog
                f.write("// Post processor header\n")
                f.write("// " + ("A" * 100 + "\n") * 250)
                # Capabilities defined at ~25 KB
                f.write('capabilities = CAPABILITY_TURNING;\n')
                f.write('description = "Long Header Turning Post";\n')

            caps = PostClear._GetPostCapabilities(cps_path)
            self.assertIsNotNone(caps)
            self.assertTrue(caps["turning"])
            self.assertFalse(caps["milling"])

            # Verify _IsPostCompatible detects it
            self.assertTrue(PostClear._IsPostCompatible(cps_path, is_turning=True))
            self.assertFalse(PostClear._IsPostCompatible(cps_path, is_turning=False))

    def test_h4_exact_path_authoritative(self):
        """Verify exact path to .cps file is authoritative and resolves immediately."""
        with tempfile.TemporaryDirectory() as tmp_dir:
            cps_path = os.path.join(tmp_dir, "my_custom_post.cps")
            with open(cps_path, "w", encoding="utf-8") as f:
                f.write('capabilities = CAPABILITY_MILLING;\ndescription = "Custom Post";\n')

            class MockPostCfg:
                postPath = cps_path
                location = None
                name = "unrelated_name"
                description = None
                url = None

            class MockProgram:
                postConfiguration = MockPostCfg()

            resolved = PostClear._ResolveBasePostPath(MockProgram(), is_turning=False)
            self.assertEqual(resolved, os.path.abspath(cps_path))

    def test_h4_file_url_authoritative(self):
        """Verify file:// URL is authoritative and resolves directly without search dir scanning."""
        with tempfile.TemporaryDirectory() as tmp_dir:
            cps_path = os.path.join(tmp_dir, "my_url_post.cps")
            with open(cps_path, "w", encoding="utf-8") as f:
                f.write('capabilities = CAPABILITY_MILLING;\ndescription = "URL Post";\n')

            class MockUrl:
                def toString(self):
                    return f"file://{cps_path}"

            class MockPostCfg:
                postPath = None
                location = None
                name = None
                description = None
                url = MockUrl()

            class MockProgram:
                postConfiguration = MockPostCfg()

            resolved = PostClear._ResolveBasePostPath(MockProgram(), is_turning=False)
            self.assertEqual(resolved, os.path.abspath(cps_path))

    def test_h4_user_cloud_preferred_over_system(self):
        """Verify user/cloud post directories (Tier 1) take precedence over system posts (Tier 2)."""
        with tempfile.TemporaryDirectory() as tmp_dir:
            user_dir = os.path.join(tmp_dir, "user_posts")
            sys_dir = os.path.join(tmp_dir, "sys_posts")
            os.makedirs(user_dir)
            os.makedirs(sys_dir)

            user_post = os.path.join(user_dir, "haas.cps")
            sys_post = os.path.join(sys_dir, "haas.cps")

            with open(user_post, "w", encoding="utf-8") as f:
                f.write('capabilities = CAPABILITY_MILLING;\ndescription = "User Haas";\n')
            with open(sys_post, "w", encoding="utf-8") as f:
                f.write('capabilities = CAPABILITY_MILLING;\ndescription = "System Haas";\n')

            class MockPostCfg:
                postPath = None
                location = None
                name = "haas"
                description = None
                url = None

            class MockProgram:
                postConfiguration = MockPostCfg()

            with patch("PostClear._GetPostSearchDirsByTier", return_value=([user_dir], [sys_dir], [])):
                resolved = PostClear._ResolveBasePostPath(MockProgram(), is_turning=False)
                self.assertEqual(resolved, os.path.abspath(user_post))

    def test_h4_ambiguous_name_match_raises_error_naming_candidates(self):
        """Verify ambiguous basename matches within the same tier raise ValueError naming all candidates."""
        with tempfile.TemporaryDirectory() as tmp_dir:
            user_dir1 = os.path.join(tmp_dir, "user1")
            user_dir2 = os.path.join(tmp_dir, "user2")
            os.makedirs(user_dir1)
            os.makedirs(user_dir2)

            post1 = os.path.join(user_dir1, "lathe_custom.cps")
            post2 = os.path.join(user_dir2, "lathe_custom.cps")

            with open(post1, "w", encoding="utf-8") as f:
                f.write('capabilities = CAPABILITY_TURNING;\ndescription = "Lathe Custom 1";\n')
            with open(post2, "w", encoding="utf-8") as f:
                f.write('capabilities = CAPABILITY_TURNING;\ndescription = "Lathe Custom 2";\n')

            class MockPostCfg:
                postPath = None
                location = None
                name = "lathe_custom"
                description = None
                url = None

            class MockProgram:
                postConfiguration = MockPostCfg()

            with patch("PostClear._GetPostSearchDirsByTier", return_value=([user_dir1, user_dir2], [], [])):
                with self.assertRaises(ValueError) as cm:
                    PostClear._ResolveBasePostPath(MockProgram(), is_turning=True)
                err_msg = str(cm.exception)
                self.assertIn("Ambiguous post processor match", err_msg)
                self.assertIn("lathe_custom", err_msg)
                self.assertIn(os.path.abspath(post1), err_msg)
                self.assertIn(os.path.abspath(post2), err_msg)

    def test_h4_ambiguous_description_match_raises_error_naming_candidates(self):
        """Verify ambiguous description substring matches within the same tier raise ValueError naming all candidates."""
        with tempfile.TemporaryDirectory() as tmp_dir:
            sys_dir = os.path.join(tmp_dir, "sys_posts")
            os.makedirs(sys_dir)

            post1 = os.path.join(sys_dir, "post_a.cps")
            post2 = os.path.join(sys_dir, "post_b.cps")

            with open(post1, "w", encoding="utf-8") as f:
                f.write('capabilities = CAPABILITY_MILLING;\ndescription = "Tormach PathPilot 1100";\n')
            with open(post2, "w", encoding="utf-8") as f:
                f.write('capabilities = CAPABILITY_MILLING;\ndescription = "Tormach PathPilot 770";\n')

            class MockPostCfg:
                postPath = None
                location = None
                name = "tormach"
                description = "tormach"
                url = None

            class MockProgram:
                postConfiguration = MockPostCfg()

            with patch("PostClear._GetPostSearchDirsByTier", return_value=([], [sys_dir], [])):
                with self.assertRaises(ValueError) as cm:
                    PostClear._ResolveBasePostPath(MockProgram(), is_turning=False)
                err_msg = str(cm.exception)
                self.assertIn("Ambiguous post processor match for description", err_msg)
                self.assertIn(os.path.abspath(post1), err_msg)
                self.assertIn(os.path.abspath(post2), err_msg)


class TestBatch5DumperConfigSecurity(unittest.TestCase):
    """M1 tests: 0o700 permissions, isolated DUMPER_CONFIG_PATH, no $TMPDIR//tmp leakage."""

    def test_m1_mkdtemp_0o700_permissions(self):
        """Verify per-run temp directory created for dumper config has mode 0o700."""
        orig_env = os.environ.get("DUMPER_CONFIG_PATH")
        dir_path = None
        try:
            with tempfile.NamedTemporaryFile(suffix=".cps") as tmp_cps:
                cfg_path = PostClear._WriteDumperConfig(base_post_path=tmp_cps.name)
                self.assertTrue(os.path.isfile(cfg_path))
                dir_path = os.path.dirname(cfg_path)
                # Check filesystem permission bits
                mode = stat.S_IMODE(os.stat(dir_path).st_mode)
                self.assertEqual(mode, 0o700, f"Expected 0o700 directory mode, got {oct(mode)}")
        finally:
            if orig_env is None:
                os.environ.pop("DUMPER_CONFIG_PATH", None)
            else:
                os.environ["DUMPER_CONFIG_PATH"] = orig_env
            os.environ.pop("BASE_POST", None)
            os.environ.pop("TARGET_INTERMEDIATE_CNC", None)
            if dir_path:
                shutil.rmtree(dir_path, ignore_errors=True)

    def test_m1_dumper_config_only_written_to_dumper_config_path(self):
        """Verify _WriteDumperConfig writes ONLY to DUMPER_CONFIG_PATH and never $TMPDIR or /tmp."""
        tmp_sys = tempfile.gettempdir()
        stray_tmpdir = os.path.join(tmp_sys, "universal_dumper_config.json")
        stray_tmp = "/tmp/universal_dumper_config.json"

        # Remove any pre-existing files if present
        for p in (stray_tmpdir, stray_tmp):
            if os.path.isfile(p):
                try:
                    os.remove(p)
                except Exception:
                    pass

        orig_env = os.environ.get("DUMPER_CONFIG_PATH")
        try:
            with tempfile.TemporaryDirectory() as target_dir:
                os.chmod(target_dir, 0o700)
                with tempfile.NamedTemporaryFile(suffix=".cps") as tmp_cps:
                    cfg_path = PostClear._WriteDumperConfig(
                        base_post_path=tmp_cps.name,
                        target_dir=target_dir,
                    )
                    self.assertEqual(os.environ.get("DUMPER_CONFIG_PATH"), cfg_path)
                    self.assertTrue(os.path.isfile(cfg_path))

                    # Verify neither $TMPDIR nor /tmp received the config
                    self.assertFalse(os.path.isfile(stray_tmpdir), "Dumper config must NOT be written to $TMPDIR")
                    if os.path.isdir("/tmp") and os.path.abspath(tmp_sys) != "/tmp":
                        self.assertFalse(os.path.isfile(stray_tmp), "Dumper config must NOT be written to /tmp")
        finally:
            if orig_env is None:
                os.environ.pop("DUMPER_CONFIG_PATH", None)
            else:
                os.environ["DUMPER_CONFIG_PATH"] = orig_env
            os.environ.pop("BASE_POST", None)
            os.environ.pop("TARGET_INTERMEDIATE_CNC", None)

    def test_m1_universal_dumper_ignores_tmp_fallbacks(self):
        """Verify universal_dumper.cps contains no /tmp or getTemporaryFolder fallback candidates in _getDumperConfig."""
        import re
        repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        dumper_cps = os.path.join(repo_root, "universal_dumper.cps")
        self.assertTrue(os.path.isfile(dumper_cps))

        with open(dumper_cps, "r", encoding="utf-8") as f:
            content = f.read()

        # Extract _getDumperConfig function implementation
        start = content.find("function _getDumperConfig()")
        self.assertGreater(start, 0)
        end = content.find("var basePostPath", start)
        self.assertGreater(end, start)
        dumper_cfg_func = content[start:end]

        # The config loader must NOT contain /tmp, $TMPDIR, or getTemporaryFolder fallback candidates
        self.assertNotIn("/tmp", dumper_cfg_func)
        self.assertNotIn("getTemporaryFolder", dumper_cfg_func)
        self.assertIn("DUMPER_CONFIG_PATH", dumper_cfg_func)


if __name__ == "__main__":
    unittest.main()
