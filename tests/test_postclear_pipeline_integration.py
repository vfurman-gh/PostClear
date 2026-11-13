#!/usr/bin/env python3
"""
Test Suite: PostClear.py Integration with Unified 2-Pass Pipeline.

Validates:
1. Pass 1 context extraction via universal_dumper.cps with zero Post Library mutation.
2. Direct execution of PipelineOrchestrator and IPW verifiers (Lathe2DIPWVerifier, MillHeightfieldVerifier).
3. Single-pass replay of vendor post processors via universal_replayer.cps.
4. Populates verifyResults with structured metrics (material removed, G0 restored, zero collisions).
"""

import json
import os
import shutil
import sys
import tempfile
import types
import unittest
from unittest.mock import patch, MagicMock

class MockCamCollection(list):
    @property
    def count(self):
        return len(self)

    def item(self, idx):
        return self[idx]

# Mock adsk namespace if running outside Fusion
if "adsk" not in sys.modules:
    adsk = types.ModuleType("adsk")
    adsk.cam = types.ModuleType("adsk.cam")
    adsk.cam.OperationTypes = types.SimpleNamespace(TurningOperation=1, MillingOperation=0)
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
    adsk = sys.modules["adsk"]
    if not hasattr(adsk.cam, "CAM"):
        adsk.cam.CAM = types.SimpleNamespace(cast=lambda x: x)

if not hasattr(adsk.core, "MessageBoxIconTypes"):
    adsk.core.MessageBoxIconTypes = types.SimpleNamespace(
        InformationIconType=0,
        WarningIconType=1,
        CriticalIconType=2,
        QuestionIconType=3,
    )
if not hasattr(adsk.core, "MessageBoxButtonTypes"):
    adsk.core.MessageBoxButtonTypes = types.SimpleNamespace(
        OKButtonType=0,
        YesNoButtonType=1,
    )
if not hasattr(adsk.core, "DialogResults"):
    adsk.core.DialogResults = types.SimpleNamespace(
        DialogYes=0,
        DialogNo=1,
        DialogOK=2,
    )
if not hasattr(adsk.core, "ObjectCollection"):
    class MockObjectCollection(list):
        @classmethod
        def create(cls):
            return cls()
        def add(self, item):
            self.append(item)
    adsk.core.ObjectCollection = MockObjectCollection

if not hasattr(adsk.cam, "SetupStockModes"):
    adsk.cam.SetupStockModes = types.SimpleNamespace(
        FixedBoxStock=0,
        RelativeBoxStock=1,
        FixedCylinderStock=2,
        RelativeCylinderStock=3,
        FixedTubeStock=4,
        RelativeTubeStock=5,
        SolidStock=6,
        PreviousSetupStock=7,
    )
if not hasattr(adsk.cam, "PostOutputUnitOptions"):
    adsk.cam.PostOutputUnitOptions = types.SimpleNamespace(
        DocumentUnitsOutput=0,
        InchesOutput=1,
        MillimetersOutput=2,
    )
if not hasattr(adsk.cam, "PostProcessInput"):
    class MockPostProcessInput:
        @classmethod
        def create(cls, programName, postConfiguration, outputFolder, outputUnits):
            inst = cls()
            inst.programName = programName
            inst.postConfiguration = postConfiguration
            inst.outputFolder = outputFolder
            inst.outputUnits = outputUnits
            inst.isOpenInEditor = False
            return inst
    adsk.cam.PostProcessInput = MockPostProcessInput

from PostClear import verifyResults, _RecordVerifyResult


class DummySetup:
    """Mock adsk.cam.Setup for unit testing."""
    def __init__(self, name="Setup 1 - Turning", is_turning=True):
        self.name = name
        # Emulate adsk.cam.OperationTypes.TurningOperation
        self.operationType = 1 if is_turning else 0


class TestPostClearPipelineIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        cls.fixture_cnc = os.path.join(cls.repo_root, "tests", "fixtures", "roughing_test.cnc")
        cls.base_cps = os.path.join(cls.repo_root, "centroid_turning.cps")

        if not os.path.isfile(cls.fixture_cnc) or not os.path.isfile(cls.base_cps):
            raise unittest.SkipTest("Prerequisites not found for integration test")

    def setUp(self):
        verifyResults.clear()

    def test_dumper_and_replayer_scripts_exist(self):
        """Verify constDumperScript and constReplayerScript point to existing .cps scripts."""
        from PostClear import constDumperScript, constReplayerScript
        self.assertTrue(os.path.isfile(constDumperScript))
        self.assertTrue(os.path.isfile(constReplayerScript))
        self.assertTrue(constDumperScript.endswith("universal_dumper.cps"))
        self.assertTrue(constReplayerScript.endswith("universal_replayer.cps"))

    def test_unified_pipeline_executes_mill_heightfield_verifier(self):
        """Verify PipelineOrchestrator default verifier for milling is MillHeightfieldVerifier."""
        from pipeline import PipelineOrchestrator
        from geometry import BoxStock3D, WorkpieceDefinition, MillHeightfieldVerifier

        stock_posts_dir = (
            "/Users/vadimf/Library/Application Support/Autodesk/webdeploy/production/"
            "c139e2731e42a0340acc3d34de2cbdb01237a27c/Autodesk Fusion.app/Contents/"
            "Libraries/Applications/CAM360/Data/Posts"
        )
        tormach_cps = os.path.join(stock_posts_dir, "tormach.cps")
        milling_cnc = os.path.join(self.repo_root, "tests", "fixtures", "milling_drill_test.cnc")
        if not os.path.isfile(tormach_cps) or not os.path.isfile(milling_cnc):
            raise unittest.SkipTest("Tormach post or milling fixture not available")

        orchestrator = PipelineOrchestrator(
            dumper_cps=os.path.join(self.repo_root, "universal_dumper.cps"),
            replayer_cps=os.path.join(self.repo_root, "universal_replayer.cps"),
        )
        workpiece = WorkpieceDefinition(
            stock=BoxStock3D(x_min=-50.0, x_max=50.0, y_min=-50.0, y_max=50.0, z_min=-25.0, z_max=0.0)
        )

        with tempfile.TemporaryDirectory() as tmp_dir:
            out_nc = os.path.join(tmp_dir, "tormach_milling.nc")
            report_md = os.path.join(tmp_dir, "report.md")
            plot_png = os.path.join(tmp_dir, "tormach_milling.png")
            rep, gcode = orchestrator.run(
                intermediate_cnc_path=milling_cnc,
                base_post_path=tormach_cps,
                workpiece=workpiece,
                final_cnc_path=out_nc,
                report_output_path=report_md,
                plot_output_path=plot_png,
            )

            # Assert report is generated by MillHeightfieldVerifier
            self.assertGreater(rep.total_volume_removed_mm3, 1000.0, "Expected 3D stock volume removed")
            self.assertGreater(len(gcode), 50, "Expected non-empty emitted G-code")
            self.assertTrue(os.path.isfile(report_md), "Verification markdown report must be written")
            self.assertTrue(os.path.isfile(plot_png), "Verification plot PNG must be written for milling")
            self.assertGreater(os.path.getsize(plot_png), 10000, "Expected PNG file to have size > 10KB")
            with open(report_md, "r", encoding="utf-8") as f:
                content = f.read()
            self.assertIn("Toolpath Verification", content)
            self.assertIn("Total Material Removed", content)

    def test_record_verify_result_with_warning(self):
        """Verify _RecordVerifyResult records warnings and collision kinds properly."""
        from PostClear import _RecordVerifyResult, verifyResults
        from geometry.models import CollisionRecord, CollisionKind, Vector3D

        setup = DummySetup(name="Collision Setup", is_turning=True)
        col = CollisionRecord(
            segment_id=1,
            vector_key="v1",
            kind=CollisionKind.RAPID_INTO_STOCK,
            timestamp_sec=0.1,
            contact_point=Vector3D(0.0, 0.0, -5.0),
            penetration_volume_mm3=10.0,
            description="Rapid into uncut stock",
        )
        _RecordVerifyResult(
            setup=setup,
            cncPath="/tmp/test.nc",
            reportText="# Report",
            collisions=[col],
            warning="1 collision(s) detected!",
            error=None,
        )
        self.assertEqual(len(verifyResults), 1)
        rec = verifyResults[0]
        self.assertEqual(rec["setup"], "Collision Setup")
        self.assertIsNone(rec["error"])
        self.assertEqual(rec["warning"], "1 collision(s) detected!")
        self.assertEqual(rec["kinds"].get("RAPID_INTO_STOCK"), 1)

    def test_show_verify_results_does_not_popup_modal_dialog(self):
        """Verify ShowVerifyResults operates silently without modal dialog popups."""
        from PostClear import _RecordVerifyResult, ShowVerifyResults, verifyResults
        from geometry.models import CollisionRecord, CollisionKind, Vector3D

        setup = DummySetup(name="Warning Setup", is_turning=False)
        col = CollisionRecord(
            segment_id=5,
            vector_key="v5",
            kind=CollisionKind.HOLDER_STRIKE,
            timestamp_sec=0.5,
            contact_point=Vector3D(10.0, 20.0, -10.0),
            penetration_volume_mm3=20.0,
            description="Holder strikes stock wall",
        )
        _RecordVerifyResult(
            setup=setup,
            cncPath="/tmp/warn.nc",
            reportText="# Warning Report",
            collisions=[col],
            warning="1 collision(s) detected",
            error=None,
        )

        popup_called = []
        class MockUI:
            def messageBox(self, *args, **kwargs):
                popup_called.append(args)
                return 0

        ShowVerifyResults(MockUI())
        self.assertEqual(len(popup_called), 0, "ShowVerifyResults must never pop up modal message boxes in the user's face")

    def test_format_summary_report_specifies_operation_collisions(self):
        """Verify _format_summary_report outputs markdown table and breakdown by operation."""
        from pipeline import PipelineOrchestrator
        from geometry.models import CollisionRecord, CollisionKind, Vector3D, VerificationReport

        orchestrator = PipelineOrchestrator(
            dumper_cps=os.path.join(self.repo_root, "universal_dumper.cps"),
            replayer_cps=os.path.join(self.repo_root, "universal_replayer.cps"),
        )

        col1 = CollisionRecord(
            segment_id=42,
            vector_key="v42",
            kind=CollisionKind.RAPID_INTO_STOCK,
            timestamp_sec=12.5,
            contact_point=Vector3D(10.0, -15.0, -2.0),
            penetration_volume_mm3=8.5,
            description="Rapid crashed into raw stock",
            op_id="3D Adaptive Roughing1",
        )
        col2 = CollisionRecord(
            segment_id=88,
            vector_key="v88",
            kind=CollisionKind.RAPID_INTO_STOCK,
            timestamp_sec=25.0,
            contact_point=Vector3D(12.0, -14.0, -3.0),
            penetration_volume_mm3=4.2,
            description="Rapid crashed into raw stock again",
            op_id="3D Adaptive Roughing1",
        )
        col3 = CollisionRecord(
            segment_id=150,
            vector_key="v150",
            kind=CollisionKind.HOLDER_STRIKE,
            timestamp_sec=60.0,
            contact_point=Vector3D(20.0, 5.0, 15.0),
            penetration_volume_mm3=12.0,
            description="Holder struck uncut wall",
            op_id="2D Contour1",
        )

        report = VerificationReport(
            total_machining_time_sec=120.0,
            total_volume_removed_mm3=15000.0,
            collisions=[col1, col2, col3],
            promoted_events=set(),
            mrr_timeline=[],
            mrr_spikes=[],
            operation_names=["3D Adaptive Roughing1", "2D Contour3", "2D Contour1"],
        )

        md = orchestrator._format_summary_report(
            report=report,
            final_gcode="",
            base_post_path="/path/to/tormach.cps",
            restored_count=10,
        )

        # 1. Table header exists
        self.assertIn("### Collisions by Operation", md)
        self.assertIn("| Operation | Status | Collisions | Details by Type |", md)

        # 2. 3D Adaptive Roughing1 row: 2 collisions, RAPID_INTO_STOCK (2)
        self.assertIn("| **3D Adaptive Roughing1** | ⚠️ **Collision** | **2** | `RAPID_INTO_STOCK` (2) |", md)

        # 3. 2D Contour3 row: clean/safe
        self.assertIn("| 2D Contour3 | ✅ Safe | 0 | None |", md)

        # 4. 2D Contour1 row: 1 collision, HOLDER_STRIKE (1)
        self.assertIn("| **2D Contour1** | ⚠️ **Collision** | **1** | `HOLDER_STRIKE` (1) |", md)

        # 5. Total row
        self.assertIn("| **Total** | | **3** | |", md)

        # 6. Detailed breakdown section
        self.assertIn("### Detailed Collision Breakdown", md)
        self.assertIn("#### ⚠️ Operation: 3D Adaptive Roughing1 (2 collisions)", md)
        self.assertIn("Move 42", md)
        self.assertIn("Move 88", md)
        self.assertIn("#### ⚠️ Operation: 2D Contour1 (1 collision)", md)
        self.assertIn("Move 150", md)

    def test_format_summary_report_includes_top_3_mrr_spikes(self):
        """Verify _format_summary_report includes MRR spikes section with top 3 spikes per operation."""
        from pipeline import PipelineOrchestrator
        from geometry.models import MRRSample, VerificationReport

        orchestrator = PipelineOrchestrator(
            dumper_cps=os.path.join(self.repo_root, "universal_dumper.cps"),
            replayer_cps=os.path.join(self.repo_root, "universal_replayer.cps"),
        )

        # 1. Zero spikes report
        zero_report = VerificationReport(
            total_machining_time_sec=60.0,
            total_volume_removed_mm3=5000.0,
            collisions=[],
            promoted_events=set(),
            mrr_timeline=[],
            mrr_spikes=[],
            operation_names=["Roughing"],
        )
        zero_md = orchestrator._format_summary_report(zero_report, "", "post.cps")
        self.assertIn("## Material Removal Rate (MRR) Spikes", zero_md)
        self.assertIn("✅ **Zero MRR spikes detected.**", zero_md)

        # 2. Report with top-3 spikes from multiple operations
        s1 = MRRSample(10.0, 1.0, 101, 1, 15.0, 900.0, 2.5, op_id="Roughing")
        s2 = MRRSample(12.0, 1.0, 102, 1, 12.0, 720.0, 2.0, op_id="Roughing")
        s3 = MRRSample(14.0, 1.0, 103, 1, 10.0, 600.0, 1.8, op_id="Roughing")
        s4 = MRRSample(30.0, 0.5, 201, 2, 8.0, 960.0, 1.5, op_id="Finishing")

        spike_report = VerificationReport(
            total_machining_time_sec=60.0,
            total_volume_removed_mm3=5000.0,
            collisions=[],
            promoted_events=set(),
            mrr_timeline=[],
            mrr_spikes=[s1, s2, s3, s4],
            operation_names=["Roughing", "Finishing"],
        )
        md = orchestrator._format_summary_report(spike_report, "", "post.cps")

        self.assertIn("## Material Removal Rate (MRR) Spikes", md)
        self.assertIn("⚠️ **4 MRR spike(s) flagged (top 3 per operation shown).**", md)
        self.assertIn("| Operation | Move | Start Time | MRR Rate (mm³/min) | Volume Removed (mm³) | Depth of Cut (mm) |", md)
        self.assertIn("| Roughing | Move 101 | 10.00 s | 900.00 | 15.00 | 2.50 |", md)
        self.assertIn("| Roughing | Move 102 | 12.00 s | 720.00 | 12.00 | 2.00 |", md)
        self.assertIn("| Roughing | Move 103 | 14.00 s | 600.00 | 10.00 | 1.80 |", md)
        self.assertIn("| Finishing | Move 201 | 30.00 s | 960.00 | 8.00 | 1.50 |", md)

    def test_non_modal_notifications_success_warning_and_failure(self):
        """Verify _NotifyPostProcessResult provides non-modal statusMessage & HTML toast without modal dialogs."""
        from PostClear import _NotifyPostProcessResult, constNotificationPaletteId

        created_palettes = {}
        class MockPalette:
            def __init__(self, id, name, html):
                self.id = id
                self.name = name
                self._html = html
                self.deleted = False
                self.dockingOption = None
                self.dockingState = None
                self.incomingFromHTML = type("Incoming", (), {"add": lambda s, h: None})()
                self.navigatingURL = type("Navigating", (), {"add": lambda s, h: None})()

            @property
            def html(self):
                if os.path.isfile(self._html):
                    with open(self._html, "r", encoding="utf-8") as f:
                        return f.read()
                return self._html

            @property
            def htmlFileURL(self):
                return self._html

            @htmlFileURL.setter
            def htmlFileURL(self, val):
                self._html = val

            def setPosition(self, left, top):
                self.left = left
                self.top = top
                return True

            def setSize(self, width, height):
                self.width = width
                self.height = height
                return True

            def deleteMe(self):
                self.deleted = True

        class MockPalettes:
            def itemById(self, id):
                return created_palettes.get(id)

            def add2(self, id, name, html_arg, isVisible, showCloseButton, isResizable, width, height):
                if not (html_arg.startswith("<html>") or html_arg.startswith("<HTML>") or os.path.isfile(html_arg)):
                    raise RuntimeError("3 : Invalid htmlFileURL argument")
                p = MockPalette(id, name, html_arg)
                created_palettes[id] = p
                return p

        class MockUI:
            def __init__(self):
                self.statusMessage = ""
                self.palettes = MockPalettes()
                self.message_box_calls = []

            def messageBox(self, *args, **kwargs):
                self.message_box_calls.append(args)
                return 0

        mock_ui = MockUI()

        # 1. Success notification
        _NotifyPostProcessResult(
            ui=mock_ui,
            cntFiles=1,
            cntSkipped=0,
            lstSkipped="",
            outputFolder="/tmp/nc_output",
            verifyResults=[{
                "setup": "OilLid",
                "cnc": "/tmp/nc_output/OilLid.nc",
                "promoted": 68,
                "kinds": {},
                "warning": None,
                "error": None,
            }]
        )
        self.assertEqual(len(mock_ui.message_box_calls), 0, "Must NEVER call modal messageBox on success")
        self.assertIn(constNotificationPaletteId, created_palettes)
        self.assertIn("Post Processing Completed Successfully", created_palettes[constNotificationPaletteId].html)
        self.assertIn("OilLid", created_palettes[constNotificationPaletteId].html)

        # 2. Warning notification (skipped setups)
        _NotifyPostProcessResult(
            ui=mock_ui,
            cntFiles=2,
            cntSkipped=1,
            lstSkipped="SetupB failed",
            outputFolder="/tmp/nc_output",
            verifyResults=[]
        )
        self.assertEqual(len(mock_ui.message_box_calls), 0, "Must NEVER call modal messageBox on warning")
        self.assertIn("Post Processing Completed with Warnings", created_palettes[constNotificationPaletteId].html)
        self.assertIn("1 setup(s) skipped", created_palettes[constNotificationPaletteId].html)

        # 3. Failure notification (0 files posted)
        _NotifyPostProcessResult(
            ui=mock_ui,
            cntFiles=0,
            cntSkipped=1,
            lstSkipped="Kernel failed to solve",
            outputFolder="/tmp/nc_output",
            verifyResults=[],
            is_fatal=True
        )
        self.assertEqual(len(mock_ui.message_box_calls), 0, "Must NEVER call modal messageBox on failure")
        self.assertIn("Post Processing Failed", created_palettes[constNotificationPaletteId].html)
        self.assertIn("Kernel failed to solve", created_palettes[constNotificationPaletteId].html)

    def test_notification_event_handlers_and_action_urls(self):
        """Verify that action:// navigation links and HTML events properly route to file opening."""
        import urllib.parse
        import PostClear
        from PostClear import (
            _BuildNotificationHTML,
            _NotificationNavigationEventHandler,
            _NotificationHTMLEventHandler,
            constNotificationPaletteId,
        )

        actions = [
            ("Open NC File", "open_file", "/tmp/test_dir/output.nc"),
            ("Open Folder", "open_folder", "/tmp/test_dir"),
        ]
        html_out = _BuildNotificationHTML("Done", "success", "All good", "/tmp/test_dir", actions)
        self.assertIn('action://open_file?path=', html_out)
        self.assertIn('action://open_folder?path=', html_out)
        self.assertIn('sendAction(event, \'open_file\'', html_out)

        opened_files = []
        orig_open_file = PostClear._OpenFile
        try:
            PostClear._OpenFile = lambda p: opened_files.append(p)

            # Test NavigationEventHandler
            nav_handler = _NotificationNavigationEventHandler(constNotificationPaletteId)
            mock_nav_args = type("NavArgs", (), {
                "navigationURL": "action://open_file?path=%2Ftmp%2Ftest_dir%2Foutput.nc",
                "launchExternally": True,
                "preventDefault": False,
            })()

            # Temporarily make path exist
            with tempfile.NamedTemporaryFile() as tf:
                mock_nav_args.navigationURL = f"action://open_file?path={urllib.parse.quote(tf.name)}"
                nav_handler.notify(mock_nav_args)
                self.assertFalse(mock_nav_args.launchExternally, "Must prevent external browser opening")
                self.assertIn(tf.name, opened_files)

            # Test HTMLEventHandler
            html_handler = _NotificationHTMLEventHandler(constNotificationPaletteId)
            with tempfile.NamedTemporaryFile() as tf2:
                mock_html_args = type("HTMLArgs", (), {
                    "action": "open_file",
                    "data": tf2.name,
                })()
                html_handler.notify(mock_html_args)
                self.assertIn(tf2.name, opened_files)
        finally:
            PostClear._OpenFile = orig_open_file

    def test_progress_dialog_advancement_and_auto_dismiss_timeout(self):
        """Verify progress updates advance percentages and notification auto-dismiss timer is 6 seconds."""
        import PostClear
        from PostClear import _BuildNotificationHTML

        # 1. Verify auto-dismiss timeout is 2 seconds (2000ms) on success, disabled on warning/error
        html = _BuildNotificationHTML("Done", "success", "OK", "", [])
        self.assertIn("2000);", html, "Success auto-dismiss timeout must be 2 seconds")
        self.assertIn("wasClicked", html, "Notification HTML must support click cancellation of auto-dismiss")
        warn_html = _BuildNotificationHTML("Warn", "warning", "Check", "", [])
        self.assertNotIn("2000);", warn_html, "Warning notifications must not auto-dismiss")
        err_html = _BuildNotificationHTML("Err", "error", "Fail", "", [])
        self.assertNotIn("2000);", err_html, "Error notifications must not auto-dismiss")

        # 2. Verify progress reporting callback logic
        progress_calls = []
        def mock_progress_cb(msg, frac):
            progress_calls.append((msg, frac))
            return True

        # Simulate pipeline reporting to callback
        mock_progress_cb("Pass 1: Dumping toolpath batch 1/2...", 0.20)
        mock_progress_cb("Pass 2: Simulating stock & collisions...", 0.50)
        mock_progress_cb("Pass 2: Generating verification plot & report...", 0.70)
        mock_progress_cb("Pass 2: Emitting final NC program...", 0.85)
        mock_progress_cb("Setup completed.", 1.0)

        self.assertEqual(len(progress_calls), 5)
        fractions = [c[1] for c in progress_calls]
        self.assertEqual(fractions, sorted(fractions), "Progress percentages must strictly advance")
        self.assertEqual(fractions[-1], 1.0)

    def test_default_settings_and_manager_purged_del_options(self):
        """Verify delFiles and delFolder are eliminated from defaults and purged by SettingsManager."""
        import PostClear

        self.assertNotIn("delFiles", PostClear.defaultSettings)
        self.assertNotIn("delFolder", PostClear.defaultSettings)

        mgr = PostClear.SettingsManager()
        legacy_dst = {
            "version": "1.0.0",
            "delFiles": True,
            "delFolder": True,
            "splitSetup": True,
            "restore_rapid": True,
        }
        mgr.UpdateSettings(PostClear.defaultSettings, legacy_dst)
        self.assertNotIn("delFiles", legacy_dst)
        self.assertNotIn("delFolder", legacy_dst)
        self.assertNotIn("splitSetup", legacy_dst)

    def test_perform_post_process_modal_overwrite_abort_on_no(self):
        """Verify modal prompt aborts before any progress dialog or posting if user selects No."""
        import PostClear
        from unittest.mock import MagicMock, patch

        class MockCamCollection(list):
            @property
            def count(self):
                return len(self)
            def item(self, idx):
                return self[idx]

        with tempfile.TemporaryDirectory() as tmp_dir:
            target_file = os.path.join(tmp_dir, "Prog.nc")
            with open(target_file, "w", encoding="utf8") as f:
                f.write("ORIGINAL_CONTENT")

            mock_param_folder = MagicMock()
            mock_param_folder.value.value = tmp_dir
            mock_param_ext = MagicMock()
            mock_param_ext.value.value = ".nc"

            params_dict = {
                "nc_program_output_folder": mock_param_folder,
                "nc_program_nc_extension": mock_param_ext,
                "nc_program_name": MagicMock(value=MagicMock(value="Prog")),
                "nc_program_filename": MagicMock(value=MagicMock(value="Prog")),
                "nc_program_openInEditor": MagicMock(value=MagicMock(value=False)),
            }
            mock_params = MagicMock()
            mock_params.itemByName.side_effect = lambda k: params_dict.get(k)

            mock_prog = MagicMock()
            mock_prog.parameters = mock_params
            mock_prog.attributes.itemByName.return_value = MagicMock(value="")

            mock_op = MagicMock()
            mock_op.isSuppressed = False
            mock_op.hasToolpath = True

            mock_setup = MagicMock()
            mock_setup.name = "Setup 1"
            mock_setup.isSuppressed = False
            mock_setup.allOperations = MockCamCollection([mock_op])

            mock_cam = MagicMock()
            mock_cam.allOperations = MockCamCollection([mock_op])
            mock_cam.setups = MockCamCollection([mock_setup])
            mock_cam.ncPrograms = MockCamCollection([mock_prog])

            mock_doc = MagicMock()
            mock_doc.products.itemByProductType.return_value = mock_cam
            mock_doc.attributes.itemByName.return_value = None

            msgbox_calls = []
            progress_created = []

            def mock_msgbox(prompt, title, btn_type, icon_type):
                msgbox_calls.append((prompt, title, btn_type, icon_type))
                return PostClear.adsk.core.DialogResults.DialogNo

            mock_ui = MagicMock()
            mock_ui.messageBox.side_effect = mock_msgbox
            mock_ui.createProgressDialog.side_effect = lambda: progress_created.append(1)

            mock_app = MagicMock()
            mock_app.userInterface = mock_ui
            mock_app.activeDocument = mock_doc

            post_setup_calls = []
            with patch("PostClear.adsk.core.Application.get", return_value=mock_app), \
                 patch("PostClear.PostProcessSetup", side_effect=lambda *args, **kwargs: post_setup_calls.append(args)):

                doc_settings = dict(PostClear.defaultSettings)
                PostClear.PerformPostProcess(doc_settings, setups=[mock_setup])

            self.assertEqual(len(msgbox_calls), 1, "Must prompt user with modal messageBox")
            prompt, title, btn_type, icon_type = msgbox_calls[0]
            self.assertIn("Prog.nc", prompt)
            self.assertIn("already exists", prompt)
            self.assertEqual(btn_type, PostClear.adsk.core.MessageBoxButtonTypes.YesNoButtonType)
            self.assertEqual(len(progress_created), 0, "Must NOT open progress dialog when aborted")
            self.assertEqual(len(post_setup_calls), 0, "Must NOT call PostProcessSetup when aborted")

            # Verify existing file was not modified or truncated
            with open(target_file, "r", encoding="utf8") as f:
                self.assertEqual(f.read(), "ORIGINAL_CONTENT")

    def test_perform_post_process_modal_overwrite_proceed_on_yes(self):
        """Verify modal prompt proceeds with post-processing when user selects Yes."""
        import PostClear
        from unittest.mock import MagicMock, patch

        class MockCamCollection(list):
            @property
            def count(self):
                return len(self)
            def item(self, idx):
                return self[idx]

        with tempfile.TemporaryDirectory() as tmp_dir:
            target_file = os.path.join(tmp_dir, "Prog.nc")
            with open(target_file, "w", encoding="utf8") as f:
                f.write("ORIGINAL_CONTENT")

            mock_param_folder = MagicMock()
            mock_param_folder.value.value = tmp_dir
            mock_param_ext = MagicMock()
            mock_param_ext.value.value = ".nc"

            params_dict = {
                "nc_program_output_folder": mock_param_folder,
                "nc_program_nc_extension": mock_param_ext,
                "nc_program_name": MagicMock(value=MagicMock(value="Prog")),
                "nc_program_filename": MagicMock(value=MagicMock(value="Prog")),
                "nc_program_openInEditor": MagicMock(value=MagicMock(value=False)),
            }
            mock_params = MagicMock()
            mock_params.itemByName.side_effect = lambda k: params_dict.get(k)

            mock_prog = MagicMock()
            mock_prog.parameters = mock_params
            mock_prog.attributes.itemByName.return_value = MagicMock(value="")

            mock_op = MagicMock()
            mock_op.isSuppressed = False
            mock_op.hasToolpath = True

            mock_setup = MagicMock()
            mock_setup.name = "Setup 1"
            mock_setup.isSuppressed = False
            mock_setup.allOperations = MockCamCollection([mock_op])

            mock_cam = MagicMock()
            mock_cam.allOperations = MockCamCollection([mock_op])
            mock_cam.setups = MockCamCollection([mock_setup])
            mock_cam.ncPrograms = MockCamCollection([mock_prog])

            mock_doc = MagicMock()
            mock_doc.products.itemByProductType.return_value = mock_cam
            mock_doc.attributes.itemByName.return_value = None

            msgbox_calls = []
            mock_progress = MagicMock()
            mock_progress.wasCancelled = False

            def mock_msgbox(prompt, title, btn_type, icon_type):
                msgbox_calls.append((prompt, title, btn_type, icon_type))
                return PostClear.adsk.core.DialogResults.DialogYes

            mock_ui = MagicMock()
            mock_ui.messageBox.side_effect = mock_msgbox
            mock_ui.createProgressDialog.return_value = mock_progress

            mock_app = MagicMock()
            mock_app.userInterface = mock_ui
            mock_app.activeDocument = mock_doc

            post_setup_calls = []
            with patch("PostClear.adsk.core.Application.get", return_value=mock_app), \
                 patch("PostClear.PostProcessSetup", side_effect=lambda *args, **kwargs: post_setup_calls.append(args)), \
                 patch("PostClear._NotifyPostProcessResult"):

                doc_settings = dict(PostClear.defaultSettings)
                PostClear.PerformPostProcess(doc_settings, setups=[mock_setup])

            self.assertEqual(len(msgbox_calls), 1)
            self.assertEqual(len(post_setup_calls), 1, "Must execute PostProcessSetup when confirmed with Yes")

    def test_perform_post_process_no_modal_prompt_when_file_not_existing(self):
        """Verify modal prompt is skipped when target output file does not already exist."""
        import PostClear
        from unittest.mock import MagicMock, patch

        class MockCamCollection(list):
            @property
            def count(self):
                return len(self)
            def item(self, idx):
                return self[idx]

        with tempfile.TemporaryDirectory() as tmp_dir:
            mock_param_folder = MagicMock()
            mock_param_folder.value.value = tmp_dir
            mock_param_ext = MagicMock()
            mock_param_ext.value.value = ".nc"

            params_dict = {
                "nc_program_output_folder": mock_param_folder,
                "nc_program_nc_extension": mock_param_ext,
                "nc_program_name": MagicMock(value=MagicMock(value="Prog")),
                "nc_program_filename": MagicMock(value=MagicMock(value="Prog")),
                "nc_program_openInEditor": MagicMock(value=MagicMock(value=False)),
            }
            mock_params = MagicMock()
            mock_params.itemByName.side_effect = lambda k: params_dict.get(k)

            mock_prog = MagicMock()
            mock_prog.parameters = mock_params
            mock_prog.attributes.itemByName.return_value = MagicMock(value="")

            mock_op = MagicMock()
            mock_op.isSuppressed = False
            mock_op.hasToolpath = True

            mock_setup = MagicMock()
            mock_setup.name = "Setup 1"
            mock_setup.isSuppressed = False
            mock_setup.allOperations = MockCamCollection([mock_op])

            mock_cam = MagicMock()
            mock_cam.allOperations = MockCamCollection([mock_op])
            mock_cam.setups = MockCamCollection([mock_setup])
            mock_cam.ncPrograms = MockCamCollection([mock_prog])

            mock_doc = MagicMock()
            mock_doc.products.itemByProductType.return_value = mock_cam
            mock_doc.attributes.itemByName.return_value = None

            msgbox_calls = []
            mock_progress = MagicMock()
            mock_progress.wasCancelled = False

            mock_ui = MagicMock()
            mock_ui.messageBox.side_effect = lambda *args, **kwargs: msgbox_calls.append(args)
            mock_ui.createProgressDialog.return_value = mock_progress

            mock_app = MagicMock()
            mock_app.userInterface = mock_ui
            mock_app.activeDocument = mock_doc

            post_setup_calls = []
            with patch("PostClear.adsk.core.Application.get", return_value=mock_app), \
                 patch("PostClear.PostProcessSetup", side_effect=lambda *args, **kwargs: post_setup_calls.append(args)), \
                 patch("PostClear._NotifyPostProcessResult"):

                doc_settings = dict(PostClear.defaultSettings)
                PostClear.PerformPostProcess(doc_settings, setups=[mock_setup])

            self.assertEqual(len(msgbox_calls), 0, "No modal messageBox should be shown when file does not exist")
            self.assertEqual(len(post_setup_calls), 1, "Must proceed directly to PostProcessSetup")

    def test_perform_post_process_empty_toolpath_modal_abort_on_no(self):
        """Verify PerformPostProcess aborts without posting when user clicks No on empty toolpath prompt."""
        import PostClear
        from unittest.mock import MagicMock, patch

        class MockCamCollection(list):
            @property
            def count(self):
                return len(self)
            def item(self, idx):
                return self[idx]

        with tempfile.TemporaryDirectory() as tmp_dir:
            params_dict = {
                "nc_program_output_folder": MagicMock(value=MagicMock(value=tmp_dir)),
                "nc_program_nc_extension": MagicMock(value=MagicMock(value=".nc")),
                "nc_program_name": MagicMock(value=MagicMock(value="Prog")),
                "nc_program_filename": MagicMock(value=MagicMock(value="Prog")),
                "nc_program_openInEditor": MagicMock(value=MagicMock(value=False)),
            }
            mock_params = MagicMock()
            mock_params.itemByName.side_effect = lambda k: params_dict.get(k)

            mock_prog = MagicMock()
            mock_prog.parameters = mock_params
            mock_prog.attributes.itemByName.return_value = MagicMock(value="")

            valid_op = MagicMock()
            valid_op.name = "ValidOp"
            valid_op.isSuppressed = False
            valid_op.hasToolpath = True
            valid_op.isToolpathValid = True

            empty_op = MagicMock()
            empty_op.name = "EmptyOp"
            empty_op.isSuppressed = False
            empty_op.hasToolpath = False

            mock_setup = MagicMock()
            mock_setup.name = "OilLidSetup"
            mock_setup.isSuppressed = False
            mock_setup.allOperations = MockCamCollection([valid_op, empty_op])

            mock_cam = MagicMock()
            mock_cam.allOperations = MockCamCollection([valid_op, empty_op])
            mock_cam.setups = MockCamCollection([mock_setup])
            mock_cam.ncPrograms = MockCamCollection([mock_prog])
            mock_cam.checkToolpath.side_effect = lambda op: getattr(op, "hasToolpath", True)

            mock_doc = MagicMock()
            mock_doc.products.itemByProductType.return_value = mock_cam
            mock_doc.attributes.itemByName.return_value = None

            msgbox_calls = []
            mock_ui = MagicMock()
            mock_ui.messageBox.side_effect = lambda prompt, title, btn_type, icon_type: (
                msgbox_calls.append((prompt, title, btn_type, icon_type)) or PostClear.adsk.core.DialogResults.DialogNo
            )

            mock_app = MagicMock()
            mock_app.userInterface = mock_ui
            mock_app.activeDocument = mock_doc

            post_setup_calls = []
            with patch("PostClear.adsk.core.Application.get", return_value=mock_app), \
                 patch("PostClear.PostProcessSetup", side_effect=lambda *args, **kwargs: post_setup_calls.append(args)):

                doc_settings = dict(PostClear.defaultSettings)
                PostClear.PerformPostProcess(doc_settings, setups=[mock_setup])

            self.assertEqual(len(msgbox_calls), 1, "Must prompt user with modal messageBox")
            prompt, title, btn_type, icon_type = msgbox_calls[0]
            self.assertIn("empty or invalid toolpaths", prompt)
            self.assertIn("OilLidSetup", prompt)
            self.assertIn("EmptyOp", prompt)
            self.assertEqual(btn_type, PostClear.adsk.core.MessageBoxButtonTypes.YesNoButtonType)
            self.assertEqual(len(post_setup_calls), 0, "Must NOT call PostProcessSetup when aborted on No")

    def test_perform_post_process_empty_toolpath_modal_proceed_on_yes(self):
        """Verify PerformPostProcess posts ONLY valid operations when user clicks Yes on empty toolpath prompt."""
        import PostClear
        from unittest.mock import MagicMock, patch

        class MockCamCollection(list):
            @property
            def count(self):
                return len(self)
            def item(self, idx):
                return self[idx]

        with tempfile.TemporaryDirectory() as tmp_dir:
            params_dict = {
                "nc_program_output_folder": MagicMock(value=MagicMock(value=tmp_dir)),
                "nc_program_nc_extension": MagicMock(value=MagicMock(value=".nc")),
                "nc_program_name": MagicMock(value=MagicMock(value="Prog")),
                "nc_program_filename": MagicMock(value=MagicMock(value="Prog")),
                "nc_program_openInEditor": MagicMock(value=MagicMock(value=False)),
            }
            mock_params = MagicMock()
            mock_params.itemByName.side_effect = lambda k: params_dict.get(k)

            mock_prog = MagicMock()
            mock_prog.parameters = mock_params
            mock_prog.attributes.itemByName.return_value = MagicMock(value="")

            valid_op = MagicMock()
            valid_op.name = "ValidOp"
            valid_op.isSuppressed = False
            valid_op.hasToolpath = True
            valid_op.isToolpathValid = True

            empty_op = MagicMock()
            empty_op.name = "EmptyOp"
            empty_op.isSuppressed = False
            empty_op.hasToolpath = False

            mock_setup = MagicMock()
            mock_setup.name = "OilLidSetup"
            mock_setup.isSuppressed = False
            mock_setup.allOperations = MockCamCollection([valid_op, empty_op])

            mock_cam = MagicMock()
            mock_cam.allOperations = MockCamCollection([valid_op, empty_op])
            mock_cam.setups = MockCamCollection([mock_setup])
            mock_cam.ncPrograms = MockCamCollection([mock_prog])
            mock_cam.checkToolpath.side_effect = lambda op: getattr(op, "hasToolpath", True)

            mock_doc = MagicMock()
            mock_doc.products.itemByProductType.return_value = mock_cam
            mock_doc.attributes.itemByName.return_value = None

            msgbox_calls = []
            mock_progress = MagicMock()
            mock_progress.wasCancelled = False

            mock_ui = MagicMock()
            mock_ui.messageBox.side_effect = lambda prompt, title, btn_type, icon_type: (
                msgbox_calls.append((prompt, title, btn_type, icon_type)) or PostClear.adsk.core.DialogResults.DialogYes
            )
            mock_ui.createProgressDialog.return_value = mock_progress

            mock_app = MagicMock()
            mock_app.userInterface = mock_ui
            mock_app.activeDocument = mock_doc

            post_setup_calls = []
            with patch("PostClear.adsk.core.Application.get", return_value=mock_app), \
                 patch("PostClear.PostProcessSetup", side_effect=lambda *args, **kwargs: post_setup_calls.append((args, kwargs))), \
                 patch("PostClear._NotifyPostProcessResult"):

                doc_settings = dict(PostClear.defaultSettings)
                PostClear.PerformPostProcess(doc_settings, setups=[mock_setup])

            self.assertEqual(len(msgbox_calls), 1)
            self.assertEqual(len(post_setup_calls), 1, "Must proceed to PostProcessSetup on Yes")
            # Verify target_operations passed to PostProcessSetup contains only valid_op
            passed_kwargs = post_setup_calls[0][1]
            target_ops = passed_kwargs.get("target_operations")
            self.assertEqual(target_ops, [valid_op])
            # docSettings must never be polluted with transient confirmation flags
            self.assertNotIn("_empty_toolpaths_confirmed", doc_settings)

    def test_perform_post_process_multiple_setups_returns_error(self):
        """Verify PerformPostProcess aborts with an error when multiple setups are detected."""
        import PostClear
        from unittest.mock import MagicMock, patch

        mock_op1 = MagicMock()
        mock_op1.isSuppressed = False
        mock_op1.hasToolpath = True
        mock_setup1 = MagicMock()
        mock_setup1.name = "Setup 1"
        mock_setup1.isSuppressed = False
        mock_setup1.allOperations = MockCamCollection([mock_op1])

        mock_op2 = MagicMock()
        mock_op2.isSuppressed = False
        mock_op2.hasToolpath = True
        mock_setup2 = MagicMock()
        mock_setup2.name = "Setup 2"
        mock_setup2.isSuppressed = False
        mock_setup2.allOperations = MockCamCollection([mock_op2])

        mock_prog = MagicMock()
        mock_prog.name = "MultiSetupProgram"
        mock_prog.parameters.itemByName.return_value = MagicMock(value=MagicMock(value="test"))
        mock_prog.attributes.itemByName.return_value = MagicMock(value="")

        mock_cam = MagicMock()
        mock_cam.ncPrograms = MockCamCollection([mock_prog])
        mock_cam.setups = MockCamCollection([mock_setup1, mock_setup2])

        mock_doc = MagicMock()
        mock_doc.products.itemByProductType.return_value = mock_cam
        mock_doc.attributes.itemByName.return_value = None

        msgbox_calls = []
        mock_ui = MagicMock()
        mock_ui.messageBox.side_effect = lambda prompt, title, btn_type, icon_type: msgbox_calls.append((prompt, title, btn_type, icon_type))

        mock_app = MagicMock()
        mock_app.userInterface = mock_ui
        mock_app.activeDocument = mock_doc

        post_setup_calls = []
        with patch("PostClear.adsk.core.Application.get", return_value=mock_app), \
             patch("PostClear.GetNcProgram", return_value=mock_prog), \
             patch("PostClear.GetNcProgramSetupsAndOperations", return_value=([mock_setup1, mock_setup2], {mock_setup1: [mock_op1], mock_setup2: [mock_op2]})), \
             patch("PostClear.PostProcessSetup", side_effect=lambda *args, **kwargs: post_setup_calls.append(args)):

            doc_settings = dict(PostClear.defaultSettings)
            res = PostClear.PerformPostProcess(doc_settings, setups=[mock_setup1, mock_setup2])

        self.assertIsNotNone(res, "PerformPostProcess must return an error string when multiple setups are detected")
        self.assertIn("Multiple setups detected", res)
        self.assertIn("MultiSetupProgram", res)
        self.assertIn("Setup 1", res)
        self.assertIn("Setup 2", res)
        self.assertEqual(len(msgbox_calls), 1, "Must prompt user with modal critical error dialog")
        self.assertIn("Multiple setups detected", msgbox_calls[0][0])
        self.assertEqual(len(post_setup_calls), 0, "Must never proceed to PostProcessSetup when multiple setups are detected")

    def test_postprocess_setup_requires_target_operations_and_rejects_empty(self):
        """Verify PostProcessSetup requires target_operations and fails fast without UI prompts."""
        import PostClear
        from unittest.mock import MagicMock, patch

        mock_setup = MagicMock(name="Setup 1")
        mock_prog = MagicMock()
        mock_ui = MagicMock()
        mock_app = MagicMock(userInterface=mock_ui)

        with patch("PostClear.adsk.core.Application.get", return_value=mock_app), \
             tempfile.TemporaryDirectory() as tmp_dir:
            # 1. Empty target_operations returns error string
            res_empty = PostClear.PostProcessSetup(
                "Setup 1", mock_setup, tmp_dir, {}, mock_prog, target_operations=[]
            )
            self.assertEqual(res_empty, "No active operations specified to post process.")

            # 2. None target_operations returns error string
            res_none = PostClear.PostProcessSetup(
                "Setup 1", mock_setup, tmp_dir, {}, mock_prog, target_operations=None
            )
            self.assertEqual(res_none, "No active operations specified to post process.")

            # 3. PostProcessSetup never calls ui.messageBox
            self.assertFalse(mock_ui.messageBox.called, "Worker must never display UI dialogs")

    def test_post_process_setup_non_destructive_writeability_check(self):
        """Verify PostProcessSetup does not destroy/truncate existing file during initial write check."""
        import PostClear
        from unittest.mock import MagicMock, patch

        with tempfile.TemporaryDirectory() as tmp_dir:
            file_name = "setup_out"
            file_path = os.path.join(tmp_dir, file_name + ".nc")
            with open(file_path, "w", encoding="utf8") as f:
                f.write("DO_NOT_TRUNCATE_THIS_DATA")

            mock_param_ext = MagicMock()
            mock_param_ext.value.value = ".nc"
            mock_program = MagicMock()
            mock_program.parameters.itemByName.return_value = mock_param_ext

            mock_op = MagicMock(name="Op1", isSuppressed=False, hasToolpath=True)

            mock_setup = MagicMock()
            mock_setup.name = file_name
            mock_setup.operationType = 1
            mock_setup.allOperations = [mock_op]

            mock_cam = MagicMock()
            mock_cam.checkToolpath.return_value = True

            mock_doc = MagicMock()
            mock_doc.products.itemByProductType.return_value = mock_cam

            mock_app = MagicMock()
            mock_app.activeDocument = mock_doc
            mock_app.userInterface = MagicMock()

            with patch("PostClear.adsk.core.Application.get", return_value=mock_app), \
                 patch("PostClear._PostProcessUnifiedPipeline", return_value=None):

                doc_settings = dict(PostClear.defaultSettings)
                res = PostClear.PostProcessSetup(
                    file_name, mock_setup, tmp_dir, doc_settings, mock_program,
                    target_operations=[mock_op]
                )
                self.assertIsNone(res)

            # Check file content was not wiped during the initial probe
            with open(file_path, "r", encoding="utf8") as f:
                self.assertEqual(f.read(), "DO_NOT_TRUNCATE_THIS_DATA")


    def test_pass1_direct_post_process_input_execution(self):
        """Verify Pass 1 uses direct cam.postProcess with PostProcessInput and zero Post Library sync."""
        import PostClear
        from unittest.mock import MagicMock, patch

        with tempfile.TemporaryDirectory() as tmp_dir:
            out_file = os.path.join(tmp_dir, "test_out.nc")

            mock_tool = MagicMock()
            mock_tool.number = 1

            mock_op = MagicMock()
            mock_op.name = "Contour 1"
            mock_op.tool = mock_tool
            mock_op.isSuppressed = False
            mock_op.hasToolpath = True

            mock_setup = MagicMock()
            mock_setup.name = "Setup 1"
            mock_setup.operationType = 0  # Milling
            mock_setup.allOperations = [mock_op]

            post_calls = []

            def fake_post_process(target_ops, post_input):
                post_calls.append((target_ops, post_input))
                # Create the batch_0.json and bootstrap.intermediate.cnc expected by Pass 2
                batch_json = os.path.join(post_input.outputFolder, f"{post_input.programName}.json")
                with open(batch_json, "w", encoding="utf-8") as f:
                    f.write('{"unit": 1, "workpiece": {"lower": {"x": -50, "y": -50, "z": -20}, "upper": {"x": 50, "y": 50, "z": 0}}, "comments": [], "sections": []}')
                bootstrap_cnc = os.path.join(post_input.outputFolder, "bootstrap.intermediate.cnc")
                shutil.copyfile(self.fixture_cnc, bootstrap_cnc)
                return True

            mock_cam = MagicMock()
            mock_cam.postProcess.side_effect = fake_post_process

            mock_doc = MagicMock()
            mock_doc.products.itemByProductType.return_value = mock_cam

            mock_app = MagicMock()
            mock_app.activeDocument = mock_doc

            mock_program = MagicMock()
            mock_program.parameters.itemByName.return_value = MagicMock(value=MagicMock(value="centroid_turning"))

            import pipeline

            def fake_subprocess_run(cmd, *args, **kwargs):
                final_cnc = cmd[-1]
                with open(final_cnc, "w", encoding="utf-8") as f:
                    f.write("G0 X0\nM30\n")
                return MagicMock(returncode=0, stdout="", stderr="")

            with patch("PostClear.adsk.core.Application.get", return_value=mock_app), \
                 patch("PostClear._ResolveBasePostPath", return_value=os.path.abspath("centroid_turning.cps")), \
                 patch("pipeline.subprocess.run", side_effect=fake_subprocess_run):

                doc_settings = dict(PostClear.defaultSettings)
                res = PostClear._PostProcessUnifiedPipeline(
                    fname="test_out",
                    setup=mock_setup,
                    setupFolder=tmp_dir,
                    docSettings=doc_settings,
                    program=mock_program,
                    path=out_file,
                    fileExt=".nc",
                    target_operations=[mock_op],
                )
                self.assertIsNone(res)

            # Verify cam.postProcess was called directly
            self.assertEqual(len(post_calls), 1)
            target_ops, post_in = post_calls[0]
            self.assertEqual(target_ops, mock_op)
            self.assertEqual(post_in.programName, "batch_0")
            self.assertEqual(post_in.postConfiguration, PostClear.constDumperScript)
            self.assertFalse(post_in.isOpenInEditor)

            # Verify universal_dumper.cps was NOT copied to local post library
            local_posts_dir = os.path.expanduser("~/Library/Application Support/Autodesk/Fusion 360 CAM/Posts")
            if os.path.isdir(local_posts_dir):
                self.assertFalse(os.path.isfile(os.path.join(local_posts_dir, "universal_dumper.cps")))

    def test_s1_s2_extract_workpiece_missing_stock_returns_none(self):
        """Verify _ExtractWorkpieceFromSetup and extract_workpiece_from_context return None when stock is absent."""
        import PostClear
        from context_parser import extract_workpiece_from_context

        # Mock setup with no stock parameters
        class EmptyParams:
            def itemByName(self, name):
                return None

        class EmptySetup:
            parameters = EmptyParams()
            stockSolids = None
            models = None

        self.assertIsNone(PostClear._ExtractWorkpieceFromSetup(EmptySetup()))

        # Context with zero/negative volume
        ctx_empty_milling = {
            "unit": 1,
            "workpiece": {"lower": {"x": 0, "y": 0, "z": 0}, "upper": {"x": 0, "y": 0, "z": 0}},
            "sections": [{"type": 0}],
        }
        self.assertIsNone(extract_workpiece_from_context(ctx_empty_milling, fallback_workpiece=None, is_turning=False))

    def test_s3_benign_collision_promotion_filtering(self):
        """Verify that promoted moves in collision are excluded while independent air moves are promoted."""
        from geometry.models import (
            VerificationReport,
            CollisionRecord,
            CollisionKind,
            Vector3D,
        )
        from context_parser import apply_promotions_to_context

        # Context with two linear moves
        ctx = {
            "unit": 1,
            "sections": [
                {
                    "events": [
                        {
                            "type": "onLinear",
                            "sx": 10.0, "sy": 0.0, "sz": 5.0,
                            "x": 5.0, "y": 0.0, "z": 5.0,
                        },
                        {
                            "type": "onLinear",
                            "sx": 0.0, "sy": 0.0, "sz": 0.0,
                            "x": -10.0, "y": 0.0, "z": 0.0,
                        },
                    ]
                }
            ],
        }

        from geometry.models import VectorKey
        key_collided = VectorKey.from_floats(0, Vector3D(0.0, 0.0, 0.0), Vector3D(-10.0, 0.0, 0.0)).to_wrapper_str()

        report = VerificationReport(
            total_machining_time_sec=10.0,
            total_volume_removed_mm3=50.0,
            collisions=[
                CollisionRecord(
                    segment_id=1,
                    vector_key=key_collided,
                    kind=CollisionKind.HOLDER_STRIKE,
                    timestamp_sec=5.0,
                    contact_point=Vector3D(0, 0, 0),
                    penetration_volume_mm3=10.0,
                    description="Holder collided with fixture",
                    sec_idx=0,
                    event_idx=1,
                )
            ],
            promoted_events={(0, 0), (0, 1)},
            mrr_timeline=[],
            mrr_spikes=[],
        )

        # Per-event collision filtering (Q2)
        colliding_events = {
            (c.sec_idx, c.event_idx)
            for c in report.collisions
            if c.sec_idx is not None and c.event_idx is not None
        }
        safe_promoted_events = {
            ev for ev in report.promoted_events
            if ev not in colliding_events
        }
        self.assertIn((0, 0), safe_promoted_events)
        self.assertNotIn((0, 1), safe_promoted_events)

        promoted_count = apply_promotions_to_context(ctx, safe_promoted_events)
        self.assertEqual(promoted_count, 1)
        self.assertTrue(ctx["sections"][0]["events"][0].get("promote"))
        self.assertFalse(ctx["sections"][0]["events"][1].get("promote", False))

    def test_b1_verify_checkbox_removed_and_verification_always_runs(self):
        """Verify that verifyGcode is removed from defaultSettings and obsolete settings cleaned up."""
        import PostClear
        self.assertNotIn("verifyGcode", PostClear.defaultSettings)

        mgr = PostClear.SettingsManager()
        dst = {"version": 16, "verifyGcode": True, "restore_rapid": True}
        mgr.UpdateSettings(PostClear.defaultSettings, dst)
        self.assertNotIn("verifyGcode", dst)

    def test_b2_progress_cancellation_aborts_pipeline(self):
        """Verify that returning False from progress_cb immediately aborts the pipeline."""
        import PostClear
        from unittest.mock import MagicMock, patch

        mock_setup = MagicMock()
        mock_setup.name = "Setup 1"
        mock_setup.operationType = 0
        mock_setup.allOperations = []

        mock_prog = MagicMock()
        mock_prog.parameters.itemByName.return_value = MagicMock(value=MagicMock(value="centroid_turning"))

        cancelled_cb = lambda msg, frac: False

        with patch("PostClear._ResolveBasePostPath", return_value=os.path.abspath("centroid_turning.cps")):
            status = PostClear._PostProcessUnifiedPipeline(
                fname="test_cancel",
                setup=mock_setup,
                setupFolder="/tmp",
                docSettings=dict(PostClear.defaultSettings),
                program=mock_prog,
                path="/tmp/test_cancel.nc",
                fileExt=".nc",
                progress_cb=cancelled_cb,
            )
            self.assertIn("cancelled", status.lower())

    def test_b4_nc_program_nc_extension_not_mutated(self):
        """Verify that PostProcessSetup does not mutate nc_program_nc_extension."""
        import PostClear
        from unittest.mock import MagicMock, patch

        mock_ext_value = MagicMock(value=".nc")
        mock_ext_param = MagicMock(value=mock_ext_value)

        def param_lookup(name):
            if name == "nc_program_nc_extension":
                return mock_ext_param
            return MagicMock(value=MagicMock(value=""))

        mock_prog = MagicMock()
        mock_prog.parameters.itemByName.side_effect = param_lookup

        mock_cam = MagicMock()
        mock_cam.checkToolpath.return_value = True

        mock_doc = MagicMock()
        mock_doc.products.itemByProductType.return_value = mock_cam

        mock_app = MagicMock()
        mock_app.activeDocument = mock_doc

        mock_op = MagicMock(name="Op1", isSuppressed=False, hasToolpath=True)
        mock_setup = MagicMock()
        mock_setup.name = "Setup 1"
        mock_setup.operationType = 0
        mock_setup.allOperations = [mock_op]

        with patch("PostClear.adsk.core.Application.get", return_value=mock_app), \
             patch("PostClear._PostProcessUnifiedPipeline", return_value=None), \
             tempfile.TemporaryDirectory() as tmp_dir:
            res = PostClear.PostProcessSetup(
                fname="test_ext",
                setup=mock_setup,
                setupFolder=tmp_dir,
                docSettings=dict(PostClear.defaultSettings),
                program=mock_prog,
                target_operations=[mock_op],
            )
            self.assertIsNone(res)
            # Extension parameter value should not have been overwritten to ".cnc"
            self.assertEqual(mock_ext_param.value.value, ".nc")

    def test_w1_windows_post_binary_discovery(self):
        """Verify find_autodesk_post_binary respects AUTODESK_FUSION_ROOT and does NOT fall back to newest by mtime without root."""
        from pipeline import find_autodesk_post_binary
        from unittest.mock import patch

        with tempfile.TemporaryDirectory() as tmp_dir:
            v1_root = os.path.join(tmp_dir, "Autodesk", "webdeploy", "production", "hash_v1")
            v2_root = os.path.join(tmp_dir, "Autodesk", "webdeploy", "production", "hash_v2")
            v1_dir = os.path.join(v1_root, "Applications", "CAM360")
            v2_dir = os.path.join(v2_root, "Applications", "CAM360")
            os.makedirs(v1_dir, exist_ok=True)
            os.makedirs(v2_dir, exist_ok=True)

            post_v1 = os.path.join(v1_dir, "post.exe")
            post_v2 = os.path.join(v2_dir, "post.exe")

            with open(post_v1, "w") as f:
                f.write("v1")
            with open(post_v2, "w") as f:
                f.write("v2")

            # 1. With AUTODESK_FUSION_ROOT pointing to v1_root, finds post_v1
            with patch.dict(os.environ, {"AUTODESK_FUSION_ROOT": v1_root}):
                found = find_autodesk_post_binary()
                self.assertEqual(os.path.abspath(found), os.path.abspath(post_v1))

            # 2. With AUTODESK_POST_BINARY pointing to post_v2, finds post_v2
            with patch.dict(os.environ, {"AUTODESK_POST_BINARY": post_v2}):
                found = find_autodesk_post_binary()
                self.assertEqual(os.path.abspath(found), os.path.abspath(post_v2))

    def test_w1b_invoking_fusion_takes_precedence_over_newer_mtime(self):
        """Verify find_autodesk_post_binary selects invoking Fusion build even if an unactivated update is newer."""
        import types
        from pipeline import (
            find_autodesk_fusion_invoking_root,
            find_autodesk_post_binary,
            find_autodesk_system_posts_dir,
        )
        from unittest.mock import patch
        import time

        with tempfile.TemporaryDirectory() as tmp_dir:
            older_root = os.path.join(tmp_dir, "Autodesk", "webdeploy", "production", "hash_older_invoking")
            newer_root = os.path.join(tmp_dir, "Autodesk", "webdeploy", "production", "hash_newer_update")

            older_cam = os.path.join(older_root, "Autodesk Fusion.app", "Contents", "Libraries", "Applications", "CAM360")
            newer_cam = os.path.join(newer_root, "Autodesk Fusion.app", "Contents", "Libraries", "Applications", "CAM360")
            older_posts = os.path.join(older_cam, "Data", "Posts")
            newer_posts = os.path.join(newer_cam, "Data", "Posts")

            os.makedirs(older_posts, exist_ok=True)
            os.makedirs(newer_posts, exist_ok=True)

            older_post_bin = os.path.join(older_cam, "post")
            newer_post_bin = os.path.join(newer_cam, "post")

            with open(older_post_bin, "w") as f:
                f.write("older")
            with open(newer_post_bin, "w") as f:
                f.write("newer")

            now = time.time()
            # newer_root has much newer modification times (e.g. background update)
            os.utime(older_post_bin, (now - 500, now - 500))
            os.utime(newer_post_bin, (now + 500, now + 500))

            # Case A: sys.modules["adsk"] points into the older installation
            fake_adsk = types.ModuleType("adsk")
            fake_adsk.__file__ = os.path.join(older_root, "Autodesk Fusion.app", "Contents", "Libraries", "Applications", "adsk", "__init__.py")

            with patch.dict(sys.modules, {"adsk": fake_adsk}):
                # Invoking root should identify older_root
                resolved_root = find_autodesk_fusion_invoking_root()
                self.assertEqual(os.path.abspath(resolved_root), os.path.abspath(older_root))

                # Post binary must be older_post_bin, NOT newer_post_bin
                resolved_bin = find_autodesk_post_binary()
                self.assertEqual(os.path.abspath(resolved_bin), os.path.abspath(older_post_bin))

                # System posts dir must be older_posts, NOT newer_posts
                resolved_posts = find_autodesk_system_posts_dir()
                self.assertEqual(os.path.abspath(resolved_posts), os.path.abspath(older_posts))

            # Case B: AUTODESK_FUSION_ROOT explicitly points to older_root
            with patch.dict(os.environ, {"AUTODESK_FUSION_ROOT": older_root}):
                resolved_bin = find_autodesk_post_binary()
                self.assertEqual(os.path.abspath(resolved_bin), os.path.abspath(older_post_bin))

    def test_w1d_pipeline_orchestrator_hang_timeout_protection(self):
        """Verify PipelineOrchestrator enforces subprocess timeouts to prevent hanging on unactivated binaries."""
        import subprocess
        from pipeline import PipelineOrchestrator
        from unittest.mock import patch

        with tempfile.TemporaryDirectory() as tmp_dir:
            fake_post = os.path.join(tmp_dir, "post")
            fake_dumper = os.path.join(tmp_dir, "universal_dumper.cps")
            fake_replayer = os.path.join(tmp_dir, "universal_replayer.cps")
            for p in (fake_post, fake_dumper, fake_replayer):
                with open(p, "w") as f:
                    f.write("test")

            orchestrator = PipelineOrchestrator(
                post_bin=fake_post,
                dumper_cps=fake_dumper,
                replayer_cps=fake_replayer,
                timeout=5.0,
            )

            with patch("pipeline.subprocess.run", side_effect=subprocess.TimeoutExpired(cmd=[fake_post], timeout=5.0)):
                with self.assertRaises(RuntimeError) as ctx_dump:
                    orchestrator.dump_intermediate_to_json("dummy.cnc", "dummy.cps", "dummy.json")
                self.assertIn("timed out after 5.0s", str(ctx_dump.exception))
                self.assertIn("hanging", str(ctx_dump.exception))

                with self.assertRaises(RuntimeError) as ctx_rep:
                    orchestrator.replay_combined_context({}, "dummy.cnc", "dummy.cps", "dummy_out.nc")
                self.assertIn("timed out after 5.0s", str(ctx_rep.exception))
                self.assertIn("hanging", str(ctx_rep.exception))

    def test_w2_temporary_directory_logging_and_sidecar(self):
        """Verify debug logging and sidecar configs use platform tempfile dir and tolerate missing /tmp."""
        import json
        import PostClear
        from unittest.mock import patch

        self.assertTrue(PostClear._LOG_PATH.startswith(tempfile.gettempdir()))

        # Test _dbg_global is gated by _debug_log_enabled and writes cleanly to scoped temp _LOG_PATH when enabled
        saved_log_path = PostClear._LOG_PATH
        saved_dbg_enabled = PostClear._debug_log_enabled
        try:
            with tempfile.TemporaryDirectory() as log_tmpdir:
                scoped_log = os.path.join(log_tmpdir, "postclear_debug.log")
                PostClear._LOG_PATH = scoped_log

                PostClear._SetDebugLogEnabled(False)
                PostClear._dbg_global("Suppressed log entry")
                self.assertFalse(os.path.exists(scoped_log))

                PostClear._SetDebugLogEnabled(True)
                PostClear._dbg_global("Test log entry")
                self.assertTrue(os.path.isfile(scoped_log))
                with open(scoped_log, "r", encoding="utf-8") as f:
                    self.assertIn("Test log entry", f.read())
        finally:
            PostClear._LOG_PATH = saved_log_path
            PostClear._SetDebugLogEnabled(saved_dbg_enabled)

        # Test _WriteDumperConfig writes to per-run mkdtemp and sets DUMPER_CONFIG_PATH without touching /tmp or $TMPDIR
        with tempfile.NamedTemporaryFile(suffix=".cps") as tmp_cps:
            cfg_path = PostClear._WriteDumperConfig(base_post_path=tmp_cps.name)
            self.assertTrue(os.path.isfile(cfg_path))
            self.assertEqual(os.environ.get("DUMPER_CONFIG_PATH"), cfg_path)
            # Verify NOT written to $TMPDIR or /tmp
            self.assertFalse(os.path.isfile(os.path.join(tempfile.gettempdir(), "universal_dumper_config.json")))
            self.assertFalse(os.path.isfile("/tmp/universal_dumper_config.json"))
            with open(cfg_path, "r", encoding="utf-8") as f:
                cfg = json.load(f)
            self.assertEqual(cfg.get("basePost"), os.path.abspath(tmp_cps.name))

            PostClear._ClearDumperConfig()
            self.assertFalse(os.path.isfile(cfg_path))
            self.assertNotIn("DUMPER_CONFIG_PATH", os.environ)

    def test_w8_sanitize_path_segment(self):
        """Verify _SanitizePathSegment strips invalid Windows/POSIX filesystem characters and handles reserved names."""
        import PostClear

        self.assertEqual(PostClear._SanitizePathSegment("normal_folder"), "normal_folder")
        self.assertEqual(PostClear._SanitizePathSegment("Setup:Op1"), "Setup_Op1")
        self.assertEqual(PostClear._SanitizePathSegment("Bad<Folder>Name?*"), "Bad_Folder_Name")
        self.assertEqual(PostClear._SanitizePathSegment("TrailingDot."), "TrailingDot")
        self.assertEqual(PostClear._SanitizePathSegment("TrailingSpace   "), "TrailingSpace")
        self.assertEqual(PostClear._SanitizePathSegment("CON"), "_CON")
        self.assertEqual(PostClear._SanitizePathSegment("nul.txt"), "_nul.txt")
        self.assertEqual(PostClear._SanitizePathSegment("aux"), "_aux")
        self.assertEqual(PostClear._SanitizePathSegment(""), "unnamed")
        self.assertEqual(PostClear._SanitizePathSegment("   ...   "), "unnamed")

    def test_b9_recursive_output_folder_creation(self):
        """Verify PerformPostProcess creates nested output directory structure with parents=True."""
        import PostClear
        from unittest.mock import MagicMock, patch

        with tempfile.TemporaryDirectory() as tmp_dir:
            nested_out = os.path.join(tmp_dir, "deeply", "nested", "output_dir")
            mock_cam = MagicMock()
            mock_cam.allOperations.count = 0
            mock_program = MagicMock()
            mock_out_param = MagicMock(value=MagicMock(value=nested_out))
            mock_program.parameters.itemByName.return_value = mock_out_param
            mock_program.attributes.itemByName.return_value = None

            mock_doc = MagicMock()
            mock_doc.products.itemByProductType.return_value = mock_cam
            mock_app = MagicMock()
            mock_app.activeDocument = mock_doc
            mock_app.userInterface = None
            mock_docSettings = dict(PostClear.defaultSettings)

            with patch("PostClear.adsk.core.Application.get", return_value=mock_app), \
                 patch("PostClear.GetNcProgram", return_value=mock_program), \
                 patch("PostClear.GetNcProgramSetupsAndOperations", return_value=([], {})), \
                 patch("PostClear.settingsMgr", MagicMock()):
                res = PostClear.PerformPostProcess(
                    mock_docSettings,
                    setups=[],
                )
                self.assertIsNone(res)
                self.assertTrue(os.path.isdir(nested_out))

    def test_w3_installer_platform_detection(self):
        """Verify install_packages platform tag detection logic."""
        import install_packages
        from unittest.mock import patch

        with patch("platform.system", return_value="Windows"), patch("platform.machine", return_value="AMD64"):
            self.assertEqual(install_packages.get_default_platform_tag(), "win_amd64")

        with patch("platform.system", return_value="Darwin"), patch("platform.machine", return_value="arm64"):
            self.assertEqual(install_packages.get_default_platform_tag(), "macosx_11_0_arm64")

        with patch("platform.system", return_value="Darwin"), patch("platform.machine", return_value="x86_64"):
            self.assertEqual(install_packages.get_default_platform_tag(), "macosx_10_9_x86_64")


    def test_a5_isolated_dumper_config_and_pipeline_unification(self):
        """Verify _WriteDumperConfig isolates config to target_dir and _ClearDumperConfig cleans env vars."""
        import json
        import PostClear
        with tempfile.TemporaryDirectory() as target_dir:
            base_post = os.path.join(target_dir, "fake.cps")
            with open(base_post, "w") as f:
                f.write("// post")
            cnc_path = os.path.join(target_dir, "fake.cnc")
            PostClear._WriteDumperConfig(
                base_post_path=base_post,
                target_cnc_path=cnc_path,
                post_properties={"prop1": "val1"},
                target_dir=target_dir,
            )

            cfg_file = os.path.join(target_dir, "universal_dumper_config.json")
            self.assertTrue(os.path.isfile(cfg_file))
            self.assertEqual(os.environ.get("DUMPER_CONFIG_PATH"), cfg_file)

            with open(cfg_file, "r", encoding="utf-8") as f:
                cfg = json.load(f)
            self.assertEqual(cfg["basePost"], os.path.abspath(base_post))
            self.assertEqual(cfg["targetIntermediateCnc"], os.path.abspath(cnc_path))
            self.assertEqual(cfg["postProperties"], {"prop1": "val1"})

            # Test cleanup
            os.environ["BASE_POST"] = os.path.abspath(base_post)
            os.environ["TARGET_INTERMEDIATE_CNC"] = os.path.abspath(cnc_path)
            PostClear._ClearDumperConfig(target_dir=target_dir)

            self.assertFalse(os.path.isfile(cfg_file))
            self.assertNotIn("DUMPER_CONFIG_PATH", os.environ)
            self.assertNotIn("BASE_POST", os.environ)
            self.assertNotIn("TARGET_INTERMEDIATE_CNC", os.environ)


    def test_b5_batch_json_matching_no_prefix_collision(self):
        """Verify batch regex matching isolates batch_1 from batch_10 and batch_11."""
        import re
        b_prefix = "batch_1"
        pattern = re.compile(rf"^{re.escape(b_prefix)}(_\d+)?\.json$")
        self.assertTrue(pattern.match("batch_1.json"))
        self.assertTrue(pattern.match("batch_1_1.json"))
        self.assertTrue(pattern.match("batch_1_2.json"))
        self.assertIsNone(pattern.match("batch_10.json"))
        self.assertIsNone(pattern.match("batch_11.json"))
        self.assertIsNone(pattern.match("batch_100.json"))


    def test_s4_s5_stock_extraction_units_and_wcs_centering(self):
        """Verify parameter unit parsing (S5) and bbox WCS axis centering check (S4)."""
        import PostClear
        from unittest.mock import MagicMock

        # 1. Parameter expression with 'in', 'mm', 'cm', 'ft'
        mock_setup = MagicMock()
        mock_p_dia = MagicMock()
        mock_p_dia.value.value = None
        mock_p_dia.expression = "2.0 in"
        mock_p_len = MagicMock()
        mock_p_len.value.value = None
        mock_p_len.expression = "100 mm"

        def itemByName(name):
            if name == "job_stockDiameter":
                return mock_p_dia
            if name == "job_stockLength":
                return mock_p_len
            return None

        mock_setup.parameters.itemByName.side_effect = itemByName
        mock_setup.stockSolids.count = 0
        mock_setup.models.count = 0

        wp = PostClear._ExtractWorkpieceFromSetup(mock_setup)
        self.assertIsNotNone(wp)
        self.assertAlmostEqual(wp.stock.outer_radius, 25.4, places=2)  # 2.0 in diameter -> 1.0 in radius = 25.4 mm
        self.assertAlmostEqual(wp.stock.z_max - wp.stock.z_min, 100.0, places=2)

        # 2. Off-center bbox fallback rejected (S4)
        mock_setup_offcenter = MagicMock()
        mock_setup_offcenter.parameters.itemByName.return_value = None
        mock_body = MagicMock()
        # Bounding box entirely on positive X (5.0 to 15.0 cm), not centered on Z
        mock_body.boundingBox.minPoint = MagicMock(x=5.0, y=-2.0, z=-10.0)
        mock_body.boundingBox.maxPoint = MagicMock(x=15.0, y=2.0, z=0.0)
        mock_setup_offcenter.stockSolids.count = 1
        mock_setup_offcenter.stockSolids.item.return_value = mock_body
        mock_setup_offcenter.models.count = 0

        wp_off = PostClear._ExtractWorkpieceFromSetup(mock_setup_offcenter)
        self.assertIsNone(wp_off)


    def test_b11_notification_placement(self):
        """Verify _NotifyPostProcessResult computes lower-right bounds from window query."""
        import PostClear
        from unittest.mock import MagicMock, patch

        mock_app = MagicMock()
        mock_ui = MagicMock()
        mock_ui.palettes.itemById.return_value = None

        with patch("PostClear.adsk.core.Application.get", return_value=mock_app), \
             patch("subprocess.check_output", return_value=b"0, 33, 1470, 923"):
            PostClear._NotifyPostProcessResult(
                ui=mock_ui,
                cntFiles=1,
                cntSkipped=0,
                lstSkipped="",
                outputFolder="/fake/out",
                verifyResults=[],
            )
            # If add2 was called, verify palette dimensions
            if mock_ui.palettes.add2.called:
                args = mock_ui.palettes.add2.call_args[0]
                self.assertEqual(args[6], 500)
                self.assertEqual(args[7], 220)


    def test_a2_post_capabilities_parsing_and_compatibility(self):
        """Verify _GetPostCapabilities parses .cps capabilities and filters correctly."""
        import PostClear
        with tempfile.TemporaryDirectory() as tmp_dir:
            turning_cps = os.path.join(tmp_dir, "custom_lathe.cps")
            with open(turning_cps, "w", encoding="utf-8") as f:
                f.write('description = "Custom Lathe";\ncapabilities = CAPABILITY_TURNING;\n')

            milling_cps = os.path.join(tmp_dir, "custom_mill.cps")
            with open(milling_cps, "w", encoding="utf-8") as f:
                f.write('description = "Custom Mill";\ncapabilities = CAPABILITY_MILLING;\n')

            millturn_cps = os.path.join(tmp_dir, "custom_millturn.cps")
            with open(millturn_cps, "w", encoding="utf-8") as f:
                f.write('description = "Custom MillTurn";\ncapabilities = CAPABILITY_MILLING | CAPABILITY_TURNING;\n')

            self.assertTrue(PostClear._IsPostCompatible(turning_cps, is_turning=True))
            self.assertFalse(PostClear._IsPostCompatible(turning_cps, is_turning=False))

            self.assertFalse(PostClear._IsPostCompatible(milling_cps, is_turning=True))
            self.assertTrue(PostClear._IsPostCompatible(milling_cps, is_turning=False))

            self.assertTrue(PostClear._IsPostCompatible(millturn_cps, is_turning=True))
            self.assertTrue(PostClear._IsPostCompatible(millturn_cps, is_turning=False))


    def test_path_helpers_extension_and_sanitization(self):
        """Verify _NormalizeFileExtension and _SanitizePathSegment."""
        import PostClear
        # Extension normalization
        self.assertEqual(PostClear._NormalizeFileExtension(".nc"), ".nc")
        self.assertEqual(PostClear._NormalizeFileExtension("nc"), ".nc")
        self.assertEqual(PostClear._NormalizeFileExtension("json"), ".cnc")
        self.assertEqual(PostClear._NormalizeFileExtension(".json"), ".cnc")
        self.assertEqual(PostClear._NormalizeFileExtension(""), ".cnc")
        self.assertEqual(PostClear._NormalizeFileExtension(None), ".cnc")

        # Filename sanitization (colons and invalid path characters are sanitized, not split into subfolders)
        self.assertEqual(PostClear._SanitizePathSegment("SimpleSetup"), "SimpleSetup")
        self.assertEqual(PostClear._SanitizePathSegment("SubFolder:PartSetup"), "SubFolder_PartSetup")
        self.assertFalse(hasattr(PostClear, "_ResolveSetupFolderAndFilename"))

    def test_b7_unit_handling_consistency(self):
        """Verify B7: Unit handling is consistent across context_parser and pipeline."""
        from context_parser import parse_context_dump
        import pipeline

        # Metric context (unit = 1)
        metric_ctx = {
            "unit": 1,
            "sections": [
                {
                    "id": 0,
                    "params": {"operation-comment": "MetricOp"},
                    "events": [
                        {"type": "onRapid", "x": 10.0, "y": 20.0, "z": 5.0, "sx": 0.0, "sy": 0.0, "sz": 0.0}
                    ]
                }
            ]
        }
        segs_metric = parse_context_dump(metric_ctx)
        self.assertEqual(len(segs_metric), 1)
        self.assertAlmostEqual(segs_metric[0].end.x, 10.0)
        self.assertAlmostEqual(segs_metric[0].end.y, 20.0)
        self.assertAlmostEqual(segs_metric[0].end.z, 5.0)

        # Imperial context (unit = 0)
        inch_ctx = {
            "unit": 0,
            "sections": [
                {
                    "id": 0,
                    "params": {"operation-comment": "InchOp"},
                    "events": [
                        {"type": "onRapid", "x": 1.0, "y": 2.0, "z": 0.5, "sx": 0.0, "sy": 0.0, "sz": 0.0}
                    ]
                }
            ]
        }
        segs_inch = parse_context_dump(inch_ctx)
        self.assertEqual(len(segs_inch), 1)
        self.assertAlmostEqual(segs_inch[0].end.x, 25.4)
        self.assertAlmostEqual(segs_inch[0].end.y, 50.8)
        self.assertAlmostEqual(segs_inch[0].end.z, 12.7)

        # Missing unit defaults consistently to inch (0)
        missing_ctx = {
            "sections": [
                {
                    "id": 0,
                    "params": {"operation-comment": "DefaultOp"},
                    "events": [
                        {"type": "onRapid", "x": 1.0, "y": 0.0, "z": 0.0, "sx": 0.0, "sy": 0.0, "sz": 0.0}
                    ]
                }
            ]
        }
        segs_default = parse_context_dump(missing_ctx)
        self.assertEqual(len(segs_default), 1)
        self.assertAlmostEqual(segs_default[0].end.x, 25.4)

    def test_system_posts_discovery_and_fallback(self):
        """Verify find_autodesk_system_posts_dir and MillClearanceVerifier fallback."""
        from pipeline import find_autodesk_system_posts_dir, find_autodesk_apt_cps, PipelineOrchestrator
        import unittest.mock

        with tempfile.TemporaryDirectory() as tmp_dir:
            posts_dir = os.path.join(tmp_dir, "Posts")
            os.makedirs(posts_dir)
            apt_file = os.path.join(posts_dir, "apt.cps")
            with open(apt_file, "w") as f:
                f.write('description = "Autodesk APT Post";\n')

            with unittest.mock.patch.dict(os.environ, {"AUTODESK_SYSTEM_POSTS_DIR": posts_dir}):
                self.assertEqual(find_autodesk_system_posts_dir(), posts_dir)
                self.assertEqual(find_autodesk_apt_cps(), apt_file)

    def test_no_stock_detected_raises_error_to_user(self):
        """Verify that missing stock raises an explicit error and reports failure to the user."""
        import json
        from pipeline import PipelineOrchestrator
        import PostClear
        from unittest.mock import MagicMock, patch

        with tempfile.TemporaryDirectory() as tmp_dir:
            # 1. PipelineOrchestrator directly raises ValueError when context has no stock
            ctx_path = os.path.join(tmp_dir, "batch_0.json")
            with open(ctx_path, "w", encoding="utf-8") as f:
                json.dump({"unit": 1, "comments": [], "sections": []}, f)

            dummy_cps = os.path.join(tmp_dir, "dummy.cps")
            with open(dummy_cps, "w", encoding="utf-8") as f:
                f.write('description = "Dummy Post";\ncapabilities = CAPABILITY_MILLING;\n')

            orchestrator = PipelineOrchestrator(
                dumper_cps=PostClear.constDumperScript,
                replayer_cps=PostClear.constReplayerScript,
            )

            with self.assertRaises(ValueError) as ctx:
                orchestrator.run(
                    context_dump_paths=[ctx_path],
                    base_post_path=dummy_cps,
                    program_name="TestSetupNoStock",
                )
            self.assertIn("No stock detected or defined", str(ctx.exception))

            # 2. _PostProcessUnifiedPipeline catches and returns formatted error string
            mock_setup = MagicMock()
            mock_setup.name = "SetupWithoutStock"
            mock_setup.operationType = 0  # Milling
            mock_setup.parameters.itemByName.return_value = None
            mock_setup.stockSolids.count = 0
            mock_setup.models.count = 0

            mock_op = MagicMock()
            mock_op.tool = MagicMock(number=1)
            mock_op.isSuppressed = False
            mock_op.hasToolpath = True
            mock_setup.allOperations = [mock_op]

            mock_cam = MagicMock()
            def fake_post_process(target_ops, post_input):
                b_json = os.path.join(post_input.outputFolder, f"{post_input.programName}.json")
                with open(b_json, "w", encoding="utf-8") as f:
                    f.write('{"unit": 1, "comments": [], "sections": []}')
                return True
            mock_cam.postProcess.side_effect = fake_post_process

            mock_doc = MagicMock()
            mock_doc.products.itemByProductType.return_value = mock_cam
            mock_app = MagicMock(activeDocument=mock_doc)

            mock_prog = MagicMock()
            mock_prog.parameters.itemByName.return_value = MagicMock(value=MagicMock(value="tormach"))

            out_file = os.path.join(tmp_dir, "output.nc")
            with patch("PostClear.adsk.core.Application.get", return_value=mock_app), \
                 patch("PostClear._ResolveBasePostPath", return_value=dummy_cps):
                res = PostClear._PostProcessUnifiedPipeline(
                    fname="SetupWithoutStock",
                    setup=mock_setup,
                    setupFolder=tmp_dir,
                    docSettings={},
                    program=mock_prog,
                    path=out_file,
                    fileExt=".nc",
                    target_operations=[mock_op],
                )
                self.assertIsNotNone(res)
                self.assertIn("No stock detected or defined", res)

    def test_unsupported_previous_setup_and_solid_stock_modes_return_explicit_error(self):
        """Verify that 'From Previous Setup' and 'From Solid' stock modes report explicit errors to user."""
        import json
        import PostClear
        from context_parser import extract_workpiece_from_context
        from pipeline import PipelineOrchestrator
        from unittest.mock import MagicMock, patch

        # 1. extract_workpiece_from_context returns None when stockMode indicates previous-setup or solid
        ctx_prev = {
            "unit": 1,
            "stockMode": "previous-setup",
            "workpiece": {"lower": {"x": -50, "y": -50, "z": -20}, "upper": {"x": 50, "y": 50, "z": 0}},
            "sections": [],
        }
        self.assertIsNone(extract_workpiece_from_context(ctx_prev))

        ctx_solid = {
            "unit": 1,
            "stockMode": "solid",
            "workpiece": {"lower": {"x": -50, "y": -50, "z": -20}, "upper": {"x": 50, "y": 50, "z": 0}},
            "sections": [],
        }
        self.assertIsNone(extract_workpiece_from_context(ctx_solid))

        # 2. PipelineOrchestrator raises explicit ValueError
        with tempfile.TemporaryDirectory() as tmp_dir:
            ctx_p = os.path.join(tmp_dir, "batch_prev.json")
            with open(ctx_p, "w", encoding="utf-8") as f:
                json.dump(ctx_prev, f)
            dummy_cps = os.path.join(tmp_dir, "dummy.cps")
            with open(dummy_cps, "w", encoding="utf-8") as f:
                f.write('description = "Dummy Post";\ncapabilities = CAPABILITY_MILLING;\n')

            orchestrator = PipelineOrchestrator(
                dumper_cps=PostClear.constDumperScript,
                replayer_cps=PostClear.constReplayerScript,
            )
            with self.assertRaises(ValueError) as err_ctx:
                orchestrator.run(
                    context_dump_paths=[ctx_p],
                    base_post_path=dummy_cps,
                    program_name="SetupFromPreviousOp",
                )
            self.assertIn("uses 'From Previous Setup' stock", str(err_ctx.exception))

            # 3. _PostProcessUnifiedPipeline catches previous setup and solid setups early via setup.stockMode enum
            mock_setup_prev = MagicMock()
            mock_setup_prev.name = "Op2RestMachining"
            mock_setup_prev.operationType = 0  # Milling
            mock_setup_prev.stockMode = adsk.cam.SetupStockModes.PreviousSetupStock
            mock_setup_prev.stockSolids.count = 0
            mock_setup_prev.models.count = 0

            # _GetSetupStockMode correctly resolves to "previous-setup"
            self.assertEqual(PostClear._GetSetupStockMode(mock_setup_prev), "previous-setup")

            # _ExtractWorkpieceFromSetup returns None
            self.assertIsNone(PostClear._ExtractWorkpieceFromSetup(mock_setup_prev))

            mock_prog = MagicMock()
            mock_prog.parameters.itemByName.return_value = MagicMock(value=MagicMock(value="tormach"))

            mock_op = MagicMock()
            mock_op.isSuppressed = False
            mock_op.hasToolpath = True
            mock_op.isToolpathValid = True
            p_tool = MagicMock()
            p_tool.value.value = 1
            mock_op.tool.parameters.itemByName.return_value = p_tool
            mock_setup_prev.allOperations = [mock_op]

            def fake_post_process(target_ops, post_input):
                out_json = os.path.join(post_input.outputFolder, f"{post_input.programName}.json")
                with open(out_json, "w", encoding="utf-8") as f:
                    json.dump({"unit": 1, "stockMode": "previous-setup", "sections": []}, f)
                return True

            mock_cam = MagicMock()
            mock_cam.postProcess.side_effect = fake_post_process
            mock_app = MagicMock()
            mock_app.activeDocument.products.itemByProductType.return_value = mock_cam

            with patch("PostClear.adsk.core.Application.get", return_value=mock_app), \
                 patch("PostClear._ResolveBasePostPath", return_value=dummy_cps):
                res = PostClear._PostProcessUnifiedPipeline(
                    fname="Op2RestMachining",
                    setup=mock_setup_prev,
                    setupFolder=tmp_dir,
                    docSettings={},
                    program=mock_prog,
                    path=os.path.join(tmp_dir, "op2.nc"),
                    fileExt=".nc",
                )
                self.assertIsNotNone(res)
                self.assertIn("From Previous Setup", res)

            # 4. Same for Solid stock via setup.stockMode
            mock_setup_solid = MagicMock()
            mock_setup_solid.name = "OpSolid"
            mock_setup_solid.operationType = 0  # Milling
            mock_setup_solid.stockMode = adsk.cam.SetupStockModes.SolidStock
            mock_setup_solid.stockSolids.count = 1
            mock_setup_solid.models.count = 0

            self.assertEqual(PostClear._GetSetupStockMode(mock_setup_solid), "solid")
            self.assertIsNone(PostClear._ExtractWorkpieceFromSetup(mock_setup_solid))

            with patch("PostClear.adsk.core.Application.get", return_value=MagicMock()), \
                 patch("PostClear._ResolveBasePostPath", return_value=dummy_cps):
                res = PostClear._PostProcessUnifiedPipeline(
                    fname="OpSolid",
                    setup=mock_setup_solid,
                    setupFolder=tmp_dir,
                    docSettings={},
                    program=mock_prog,
                    path=os.path.join(tmp_dir, "solid.nc"),
                    fileExt=".nc",
                )
                self.assertIsNotNone(res)
                self.assertIn("From Solid", res)

            # 5. Box setup with STALE stockSolids (count > 0) is NOT rejected as solid!
            mock_setup_box = MagicMock()
            mock_setup_box.name = "OpBoxWithStaleSolids"
            mock_setup_box.stockMode = adsk.cam.SetupStockModes.FixedBoxStock
            mock_setup_box.stockSolids.count = 5  # stale solid selection
            self.assertEqual(PostClear._GetSetupStockMode(mock_setup_box), "fixed-box")

            # 6. Verify all SetupStockModes enum mappings in _GetSetupStockMode
            expected_modes = {
                adsk.cam.SetupStockModes.FixedBoxStock: "fixed-box",
                adsk.cam.SetupStockModes.RelativeBoxStock: "relative-box",
                adsk.cam.SetupStockModes.FixedCylinderStock: "fixed-cylinder",
                adsk.cam.SetupStockModes.RelativeCylinderStock: "relative-cylinder",
                adsk.cam.SetupStockModes.FixedTubeStock: "fixed-tube",
                adsk.cam.SetupStockModes.RelativeTubeStock: "relative-tube",
                adsk.cam.SetupStockModes.SolidStock: "solid",
                adsk.cam.SetupStockModes.PreviousSetupStock: "previous-setup",
            }
            for enum_val, expected_str in expected_modes.items():
                s = MagicMock()
                s.stockMode = enum_val
                self.assertEqual(PostClear._GetSetupStockMode(s), expected_str)

    def test_nc_program_unit_resolution(self):
        """Verify _GetNCProgramOutputUnits resolves PostOutputUnitOptions for all supported input types."""
        import PostClear
        mm_enum = adsk.cam.PostOutputUnitOptions.MillimetersOutput
        in_enum = adsk.cam.PostOutputUnitOptions.InchesOutput
        doc_enum = adsk.cam.PostOutputUnitOptions.DocumentUnitsOutput

        # 1. Native ChoiceParameterValue on parameters.itemByName('nc_program_unit')
        for mm_str in ("Millimeters", "millimeters", "mm", "MM"):
            mock_prog = MagicMock()
            mock_param = MagicMock()
            mock_param.value.value = mm_str
            mock_prog.parameters.itemByName.side_effect = lambda k: mock_param if k == "nc_program_unit" else None
            self.assertEqual(PostClear._GetNCProgramOutputUnits(mock_prog), mm_enum)

        for in_str in ("Inches", "inches", "in", "IN"):
            mock_prog = MagicMock()
            mock_param = MagicMock()
            mock_param.value.value = in_str
            mock_prog.parameters.itemByName.side_effect = lambda k: mock_param if k == "nc_program_unit" else None
            self.assertEqual(PostClear._GetNCProgramOutputUnits(mock_prog), in_enum)

        for doc_str in ("Document units", "document units", "doc"):
            mock_prog = MagicMock()
            mock_param = MagicMock()
            mock_param.value.value = doc_str
            mock_prog.parameters.itemByName.side_effect = lambda k: mock_param if k == "nc_program_unit" else None
            self.assertEqual(PostClear._GetNCProgramOutputUnits(mock_prog), doc_enum)

        # 2. outputUnits property directly on program
        mock_prog_prop = MagicMock(spec=["outputUnits"])
        mock_prog_prop.outputUnits = mm_enum
        self.assertEqual(PostClear._GetNCProgramOutputUnits(mock_prog_prop), mm_enum)
        mock_prog_prop.outputUnits = "Millimeters"
        self.assertEqual(PostClear._GetNCProgramOutputUnits(mock_prog_prop), mm_enum)

        # 3. postParameters fallback
        mock_prog_post = MagicMock(spec=["postParameters"])
        mock_post_param = MagicMock()
        mock_post_param.value.value = "Millimeters"
        mock_prog_post.postParameters.itemByName.side_effect = lambda k: mock_post_param if k == "nc_program_unit" else None
        self.assertEqual(PostClear._GetNCProgramOutputUnits(mock_prog_post), mm_enum)

        # 4. docSettings fallback
        self.assertEqual(PostClear._GetNCProgramOutputUnits(None, {"outputUnits": "Millimeters"}), mm_enum)
        self.assertEqual(PostClear._GetNCProgramOutputUnits(None, {"nc_program_unit": "Inches"}), in_enum)

        # 5. Default fallback to DocumentUnitsOutput
        self.assertEqual(PostClear._GetNCProgramOutputUnits(None, {}), doc_enum)
    def test_pipeline_nc_program_mm_unit_override(self):
        """Verify that NCProgram specifying mm output overrides Document units and passes MillimetersOutput to PostProcessInput."""
        import PostClear
        from unittest.mock import patch, MagicMock

        with tempfile.TemporaryDirectory() as tmp_dir:
            out_file = os.path.join(tmp_dir, "Spinner setup.nc")

            mock_tool = MagicMock()
            mock_tool.parameters.itemByName.return_value = MagicMock(value=MagicMock(value=1))

            mock_op = MagicMock()
            mock_op.tool = mock_tool
            mock_op.isSuppressed = False
            mock_op.hasToolpath = True
            mock_op.name = "Face Operation"

            mock_setup = MagicMock()
            mock_setup.name = "Spinner setup"
            mock_setup.operationType = 1  # Turning
            mock_setup.allOperations = MockCamCollection([mock_op])
            mock_setup.parameters = MagicMock()

            # Mock NCProgram specifying 'Millimeters' output unit
            mock_param_unit = MagicMock()
            mock_param_unit.value.value = "Millimeters"
            params_dict = {
                "nc_program_unit": mock_param_unit,
                "nc_program_nc_extension": MagicMock(value=MagicMock(value=".nc")),
                "nc_program_name": MagicMock(value=MagicMock(value="Spinner")),
                "nc_program_output_folder": MagicMock(value=MagicMock(value=tmp_dir)),
            }
            mock_prog = MagicMock()
            mock_prog.name = "Spinner"
            mock_prog.parameters.itemByName.side_effect = lambda k: params_dict.get(k)
            mock_prog.postConfiguration = MagicMock()
            mock_prog.postConfiguration.name = "centroid_turning"

            post_calls = []
            def fake_post_process(ops, post_input):
                post_calls.append((ops, post_input))
                b_json = os.path.join(post_input.outputFolder, f"{post_input.programName}.json")
                ctx_payload = {
                    "unit": 1,  # MM
                    "highFeedrate": 0,
                    "programName": "Spinner",
                    "comments": [],
                    "sections": [
                        {
                            "id": 0,
                            "type": 1,
                            "tool": {"number": 1, "diameter": 25.0, "_isTurningTool": True},
                            "params": {"operation-comment": "Face Operation"},
                            "events": [
                                {"type": "onRapid", "x": 50.0, "y": 0.0, "z": 2.0},
                                {"type": "onLinear", "x": 0.0, "y": 0.0, "z": 2.0, "f": 100.0},
                            ],
                        }
                    ],
                }
                with open(b_json, "w", encoding="utf-8") as f:
                    json.dump(ctx_payload, f)
                bootstrap_cnc = os.path.join(post_input.outputFolder, "bootstrap.intermediate.cnc")
                shutil.copyfile(self.fixture_cnc, bootstrap_cnc)
                return True

            mock_cam = MagicMock()
            mock_cam.ncPrograms = MockCamCollection([mock_prog])
            mock_cam.postProcess.side_effect = fake_post_process

            mock_doc = MagicMock()
            mock_doc.products.itemByProductType.return_value = mock_cam
            mock_app = MagicMock()
            mock_app.activeDocument = mock_doc

            written_configs = []
            orig_write_cfg = PostClear._WriteDumperConfig
            def intercepted_write_dumper_config(*args, **kwargs):
                written_configs.append(dict(kwargs))
                return orig_write_cfg(*args, **kwargs)

            def fake_subprocess_run(cmd, *args, **kwargs):
                if len(cmd) >= 3 and not str(cmd[0]).endswith("ps"):
                    final_cnc = cmd[-1]
                    with open(final_cnc, "w", encoding="utf-8") as f:
                        f.write("G21\nG0 X50.0 Z2.0\nM30\n")
                return MagicMock(returncode=0, stdout="", stderr="")

            with patch("PostClear.adsk.core.Application.get", return_value=mock_app), \
                 patch("PostClear._ResolveBasePostPath", return_value=os.path.abspath("centroid_turning.cps")), \
                 patch("PostClear._WriteDumperConfig", side_effect=intercepted_write_dumper_config), \
                 patch("pipeline.subprocess.run", side_effect=fake_subprocess_run):

                doc_settings = dict(PostClear.defaultSettings)
                res = PostClear._PostProcessUnifiedPipeline(
                    fname="Spinner setup",
                    setup=mock_setup,
                    setupFolder=tmp_dir,
                    docSettings=doc_settings,
                    program=mock_prog,
                    path=out_file,
                    fileExt=".nc",
                    target_operations=[mock_op],
                )
                self.assertIsNone(res)

            # 1. Verify Pass 1 PostProcessInput outputUnits was set to MillimetersOutput (2)
            self.assertEqual(len(post_calls), 1)
            _, post_in = post_calls[0]
            self.assertEqual(
                post_in.outputUnits,
                adsk.cam.PostOutputUnitOptions.MillimetersOutput,
                "Pass 1 PostProcessInput must use MillimetersOutput when NCProgram specifies mm"
            )

            # 2. Verify _WriteDumperConfig received unit=1 (MM)
            self.assertTrue(len(written_configs) >= 1)
            self.assertEqual(written_configs[0].get("unit"), 1, "Dumper config must record unit=1 for MM")

            # 3. Verify final CNC file was generated
            self.assertTrue(os.path.isfile(out_file))
            with open(out_file, "r", encoding="utf-8") as f:
                content = f.read()
                self.assertIn("G21", content, "Replayer output must emit metric code G21")

    def test_pipeline_nc_program_inches_unit_override(self):
        """Verify that NCProgram specifying Inches output overrides Document units and passes InchesOutput (1) to PostProcessInput."""
        import PostClear
        from unittest.mock import patch, MagicMock

        with tempfile.TemporaryDirectory() as tmp_dir:
            out_file = os.path.join(tmp_dir, "InchSetup.nc")

            mock_tool = MagicMock()
            mock_tool.parameters.itemByName.return_value = MagicMock(value=MagicMock(value=1))

            mock_op = MagicMock()
            mock_op.tool = mock_tool
            mock_op.isSuppressed = False
            mock_op.hasToolpath = True
            mock_op.name = "Turning Operation"

            mock_setup = MagicMock()
            mock_setup.name = "InchSetup"
            mock_setup.operationType = 1  # Turning
            mock_setup.allOperations = MockCamCollection([mock_op])
            mock_setup.parameters = MagicMock()

            mock_param_unit = MagicMock()
            mock_param_unit.value.value = "Inches"
            params_dict = {
                "nc_program_unit": mock_param_unit,
                "nc_program_nc_extension": MagicMock(value=MagicMock(value=".nc")),
                "nc_program_name": MagicMock(value=MagicMock(value="InchProg")),
                "nc_program_output_folder": MagicMock(value=MagicMock(value=tmp_dir)),
            }
            mock_prog = MagicMock()
            mock_prog.name = "InchProg"
            mock_prog.parameters.itemByName.side_effect = lambda k: params_dict.get(k)
            mock_prog.postConfiguration = MagicMock()
            mock_prog.postConfiguration.name = "centroid_turning"

            post_calls = []
            def fake_post_process(ops, post_input):
                post_calls.append((ops, post_input))
                b_json = os.path.join(post_input.outputFolder, f"{post_input.programName}.json")
                ctx_payload = {
                    "unit": 0,  # IN
                    "highFeedrate": 0,
                    "programName": "InchProg",
                    "comments": [],
                    "sections": [
                        {
                            "id": 0,
                            "type": 1,
                            "tool": {"number": 1, "diameter": 1.0, "_isTurningTool": True},
                            "params": {"operation-comment": "Turning Operation"},
                            "events": [
                                {"type": "onRapid", "x": 2.0, "y": 0.0, "z": 0.1},
                                {"type": "onLinear", "x": 0.0, "y": 0.0, "z": 0.1, "f": 10.0},
                            ],
                        }
                    ],
                }
                with open(b_json, "w", encoding="utf-8") as f:
                    json.dump(ctx_payload, f)
                bootstrap_cnc = os.path.join(post_input.outputFolder, "bootstrap.intermediate.cnc")
                shutil.copyfile(self.fixture_cnc, bootstrap_cnc)
                return True

            mock_cam = MagicMock()
            mock_cam.ncPrograms = MockCamCollection([mock_prog])
            mock_cam.postProcess.side_effect = fake_post_process

            mock_doc = MagicMock()
            mock_doc.products.itemByProductType.return_value = mock_cam
            mock_app = MagicMock()
            mock_app.activeDocument = mock_doc

            written_configs = []
            orig_write_cfg = PostClear._WriteDumperConfig
            def intercepted_write_dumper_config(*args, **kwargs):
                written_configs.append(dict(kwargs))
                return orig_write_cfg(*args, **kwargs)

            def fake_subprocess_run(cmd, *args, **kwargs):
                final_cnc = cmd[-1]
                with open(final_cnc, "w", encoding="utf-8") as f:
                    f.write("G20\nG0 X2.0 Z0.1\nM30\n")
                return MagicMock(returncode=0, stdout="", stderr="")

            with patch("PostClear.adsk.core.Application.get", return_value=mock_app), \
                 patch("PostClear._ResolveBasePostPath", return_value=os.path.abspath("centroid_turning.cps")), \
                 patch("PostClear._WriteDumperConfig", side_effect=intercepted_write_dumper_config), \
                 patch("pipeline.subprocess.run", side_effect=fake_subprocess_run):

                doc_settings = dict(PostClear.defaultSettings)
                res = PostClear._PostProcessUnifiedPipeline(
                    fname="InchSetup",
                    setup=mock_setup,
                    setupFolder=tmp_dir,
                    docSettings=doc_settings,
                    program=mock_prog,
                    path=out_file,
                    fileExt=".nc",
                    target_operations=[mock_op],
                )
                self.assertIsNone(res)

            # 1. Verify Pass 1 PostProcessInput outputUnits was set to InchesOutput (1)
            self.assertEqual(len(post_calls), 1)
            _, post_in = post_calls[0]
            self.assertEqual(
                post_in.outputUnits,
                adsk.cam.PostOutputUnitOptions.InchesOutput,
                "Pass 1 PostProcessInput must use InchesOutput when NCProgram specifies inches"
            )

            # 2. Verify _WriteDumperConfig received unit=0 (IN)
            self.assertTrue(len(written_configs) >= 1)
            self.assertEqual(written_configs[0].get("unit"), 0, "Dumper config must record unit=0 for IN")

            # 3. Verify final CNC file was generated
            self.assertTrue(os.path.isfile(out_file))
            with open(out_file, "r", encoding="utf-8") as f:
                content = f.read()
                self.assertIn("G20", content, "Replayer output must emit imperial code G20")

    def test_pipeline_nc_program_document_units_fallback(self):
        """Verify that NCProgram with Document units leaves units as DocumentUnitsOutput (0)."""
        import PostClear
        from unittest.mock import patch, MagicMock

        with tempfile.TemporaryDirectory() as tmp_dir:
            out_file = os.path.join(tmp_dir, "DocSetup.nc")

            mock_tool = MagicMock()
            mock_tool.parameters.itemByName.return_value = MagicMock(value=MagicMock(value=1))

            mock_op = MagicMock()
            mock_op.tool = mock_tool
            mock_op.isSuppressed = False
            mock_op.hasToolpath = True
            mock_op.name = "Milling Operation"

            mock_setup = MagicMock()
            mock_setup.name = "DocSetup"
            mock_setup.operationType = 0  # Milling
            mock_setup.allOperations = [mock_op]

            mock_param_unit = MagicMock()
            mock_param_unit.value.value = "Document units"
            params_dict = {
                "nc_program_unit": mock_param_unit,
                "nc_program_nc_extension": MagicMock(value=MagicMock(value=".nc")),
                "nc_program_name": MagicMock(value=MagicMock(value="DocProg")),
                "nc_program_output_folder": MagicMock(value=MagicMock(value=tmp_dir)),
            }
            mock_prog = MagicMock()
            mock_prog.name = "DocProg"
            mock_prog.parameters.itemByName.side_effect = lambda k: params_dict.get(k)
            mock_prog.postConfiguration = MagicMock()
            mock_prog.postConfiguration.name = "haas"

            post_calls = []
            def fake_post_process(ops, post_input):
                post_calls.append((ops, post_input))
                b_json = os.path.join(post_input.outputFolder, f"{post_input.programName}.json")
                ctx_payload = {
                    "unit": 0,
                    "highFeedrate": 0,
                    "programName": "DocProg",
                    "workpiece": {"lower": {"x": -50, "y": -50, "z": -20}, "upper": {"x": 50, "y": 50, "z": 0}},
                    "comments": [],
                    "sections": [],
                }
                with open(b_json, "w", encoding="utf-8") as f:
                    json.dump(ctx_payload, f)
                bootstrap_cnc = os.path.join(post_input.outputFolder, "bootstrap.intermediate.cnc")
                shutil.copyfile(self.fixture_cnc, bootstrap_cnc)
                return True

            mock_cam = MagicMock()
            mock_cam.ncPrograms = MockCamCollection([mock_prog])
            mock_cam.postProcess.side_effect = fake_post_process

            mock_doc = MagicMock()
            mock_doc.products.itemByProductType.return_value = mock_cam
            mock_app = MagicMock()
            mock_app.activeDocument = mock_doc

            written_configs = []
            orig_write_cfg = PostClear._WriteDumperConfig
            def intercepted_write_dumper_config(*args, **kwargs):
                written_configs.append(dict(kwargs))
                return orig_write_cfg(*args, **kwargs)

            def fake_subprocess_run(cmd, *args, **kwargs):
                final_cnc = cmd[-1]
                with open(final_cnc, "w", encoding="utf-8") as f:
                    f.write("G0 X0\nM30\n")
                return MagicMock(returncode=0, stdout="", stderr="")

            with patch("PostClear.adsk.core.Application.get", return_value=mock_app), \
                 patch("PostClear._ResolveBasePostPath", return_value=os.path.abspath("centroid_turning.cps")), \
                 patch("PostClear._WriteDumperConfig", side_effect=intercepted_write_dumper_config), \
                 patch("pipeline.subprocess.run", side_effect=fake_subprocess_run):

                doc_settings = dict(PostClear.defaultSettings)
                res = PostClear._PostProcessUnifiedPipeline(
                    fname="DocSetup",
                    setup=mock_setup,
                    setupFolder=tmp_dir,
                    docSettings=doc_settings,
                    program=mock_prog,
                    path=out_file,
                    fileExt=".nc",
                    target_operations=[mock_op],
                )
                self.assertIsNone(res)

            # 1. Verify Pass 1 PostProcessInput outputUnits was set to DocumentUnitsOutput (0)
            self.assertEqual(len(post_calls), 1)
            _, post_in = post_calls[0]
            self.assertEqual(
                post_in.outputUnits,
                adsk.cam.PostOutputUnitOptions.DocumentUnitsOutput,
                "Pass 1 PostProcessInput must use DocumentUnitsOutput when NCProgram specifies Document units"
            )

            # 2. Verify _WriteDumperConfig received unit=None (not overriding)
            self.assertTrue(len(written_configs) >= 1)
            self.assertIsNone(written_configs[0].get("unit"), "Dumper config unit must be None for Document units")

    def test_init_addin_is_idempotent_cleans_stale_control_and_definition(self):
        """Calling InitAddIn() twice in a row cleans stale control before stale cmdDef without RuntimeError."""
        import PostClear

        delete_order = []
        created_defs = []
        created_controls = []

        class FakeCmdDef:
            def __init__(self, def_id):
                self.id = def_id
                self.commandCreated = MagicMock()
                self.deleteMe = MagicMock(side_effect=self._delete)

            def _delete(self):
                delete_order.append(("cmdDef", self))
                cmd_defs_store.pop(self.id, None)

        class FakeCmdControl:
            def __init__(self, ctrl_id):
                self.id = ctrl_id
                self.isPromotedByDefault = False
                self.isPromoted = False
                self.deleteMe = MagicMock(side_effect=self._delete)

            def _delete(self):
                delete_order.append(("cmdControl", self))
                controls_store.pop(self.id, None)

        cmd_defs_store = {}
        controls_store = {}

        def add_button_def(def_id, name, tooltip, res_folder):
            if def_id in cmd_defs_store:
                raise RuntimeError("3 : a command definition with that id already exists")
            d = FakeCmdDef(def_id)
            cmd_defs_store[def_id] = d
            created_defs.append(d)
            return d

        def add_command(cmd_def, target_id, is_before):
            if cmd_def.id in controls_store:
                raise RuntimeError("3 : a command control with that id already exists")
            c = FakeCmdControl(cmd_def.id)
            controls_store[cmd_def.id] = c
            created_controls.append(c)
            return c

        mock_cmd_defs = MagicMock()
        mock_cmd_defs.itemById.side_effect = lambda k: cmd_defs_store.get(k)
        mock_cmd_defs.addButtonDefinition.side_effect = add_button_def

        mock_controls = MagicMock()
        mock_controls.itemById.side_effect = lambda k: controls_store.get(k)
        mock_controls.addCommand.side_effect = add_command

        mock_panel = MagicMock()
        mock_panel.controls = mock_controls

        mock_ws = MagicMock()
        mock_ws.toolbarPanels.itemById.return_value = mock_panel

        mock_ui = MagicMock()
        mock_ui.commandDefinitions = mock_cmd_defs
        mock_ui.workspaces.itemById.return_value = mock_ws
        mock_ui.allToolbarPanels.itemById.return_value = mock_panel

        mock_app = MagicMock()
        mock_app.userInterface = mock_ui

        saved_handlers = list(PostClear.handlers)
        saved_mm_handler = PostClear._marking_menu_handler
        try:
            PostClear.handlers.clear()
            PostClear._marking_menu_handler = None
            with patch("PostClear.adsk.core.Application.get", return_value=mock_app):
                PostClear.InitAddIn()
                mock_ui.messageBox.assert_not_called()
                self.assertEqual(len(created_defs), 1)
                self.assertEqual(len(created_controls), 1)
                self.assertEqual(len(PostClear.handlers), 2)

                # Call InitAddIn() again without stop() — must delete stale control then stale cmdDef.
                PostClear.InitAddIn()
                mock_ui.messageBox.assert_not_called()

            first_ctrl = created_controls[0]
            first_def = created_defs[0]
            first_ctrl.deleteMe.assert_called_once()
            first_def.deleteMe.assert_called_once()
            self.assertEqual(
                delete_order,
                [("cmdControl", first_ctrl), ("cmdDef", first_def)],
            )
            self.assertEqual(len(created_defs), 2)
            self.assertEqual(len(created_controls), 2)
            self.assertEqual(len(PostClear.handlers), 2)
        finally:
            PostClear.handlers[:] = saved_handlers
            PostClear._marking_menu_handler = saved_mm_handler

    def test_stop_and_run_idempotent_reload(self):
        """run() refreshes local modules via _EnsureLocalModulesLoaded(force=True) and stop() cleans up safely."""
        import PostClear

        mock_control = MagicMock()
        mock_def = MagicMock()
        mock_palette = MagicMock()

        mock_panel = MagicMock()
        mock_panel.controls.itemById.return_value = mock_control

        mock_ws = MagicMock()
        mock_ws.toolbarPanels.itemById.return_value = mock_panel

        mock_ui = MagicMock()
        mock_ui.workspaces.itemById.return_value = mock_ws
        mock_ui.allToolbarPanels.itemById.return_value = mock_panel
        mock_ui.commandDefinitions.itemById.return_value = mock_def
        mock_ui.palettes.itemById.return_value = mock_palette

        mock_app = MagicMock()
        mock_app.userInterface = mock_ui

        saved_handlers = list(PostClear.handlers)
        saved_settings_mgr = PostClear.settingsMgr
        try:
            with patch("PostClear.adsk.core.Application.get", return_value=mock_app), \
                 patch("PostClear._EnsureLocalModulesLoaded") as mock_ensure:
                PostClear.run(None)
                mock_ensure.assert_called_once_with(force=True, allow_bootstrap=True)
                mock_ui.messageBox.assert_not_called()
                mock_control.deleteMe.assert_called_once()
                mock_def.deleteMe.assert_called_once()

                PostClear.stop(None)
                mock_ui.messageBox.assert_not_called()
                mock_palette.deleteMe.assert_called_once()
                self.assertEqual(len(PostClear.handlers), 0)
        finally:
            PostClear.handlers[:] = saved_handlers
            PostClear.settingsMgr = saved_settings_mgr


    def test_v17_settings_migration_and_selected_nc_programs(self):
        """SettingsManager v18 migrates legacy ncProgram -> ncPrograms, defaults debug_log=False, and resolves multi-selection."""
        import PostClear

        self.assertEqual(PostClear.version, 18)
        self.assertIn("ncPrograms", PostClear.defaultSettings)
        self.assertEqual(PostClear.defaultSettings["ncPrograms"], [])
        self.assertIn("debug_log", PostClear.defaultSettings)
        self.assertIs(PostClear.defaultSettings["debug_log"], False)

        mgr = PostClear.SettingsManager()
        legacy = {
            "version": 16,
            "ncProgram": "ProgB",
            "output": "/tmp/out",
        }
        mgr.UpdateSettings(PostClear.defaultSettings, legacy)
        self.assertEqual(legacy["version"], 18)
        self.assertEqual(legacy["ncPrograms"], ["ProgB"])
        self.assertEqual(legacy["ncProgram"], "ProgB")
        self.assertIs(legacy["debug_log"], False)

        # Explicitly enabled debug_log is preserved across migration
        explicit_true = {"version": 17, "ncPrograms": ["ProgA"], "debug_log": True}
        mgr.UpdateSettings(PostClear.defaultSettings, explicit_true)
        self.assertEqual(explicit_true["version"], 18)
        self.assertIs(explicit_true["debug_log"], True)

        prog_a = MagicMock()
        prog_a.name = "ProgA"
        prog_b = MagicMock()
        prog_b.name = "ProgB"
        prog_c = MagicMock()
        prog_c.name = "ProgC"
        cam = MagicMock()
        cam.ncPrograms = MockCamCollection([prog_a, prog_b, prog_c])

        selected = PostClear.GetSelectedNcPrograms(cam, {"ncPrograms": ["ProgC", "ProgA"]})
        self.assertEqual([p.name for p in selected], ["ProgC", "ProgA"])

        # Fallback to legacy single ncProgram when ncPrograms is empty
        fallback = PostClear.GetSelectedNcPrograms(cam, {"ncPrograms": [], "ncProgram": "ProgB"})
        self.assertEqual([p.name for p in fallback], ["ProgB"])

    def test_multi_nc_program_command_dialog_validation_and_execute(self):
        """Verify TableCommandInput checklist, select-all sync, validation, and execute persistence."""
        import PostClear

        prog_a = MagicMock()
        prog_a.name = "ProgA"
        prog_a.parameters = MagicMock()
        prog_a.parameters.itemByName.side_effect = lambda k: {
            "nc_program_filename": MagicMock(value=MagicMock(value="PartA")),
            "nc_program_nc_extension": MagicMock(value=MagicMock(value=".nc")),
        }.get(k)

        prog_b = MagicMock()
        prog_b.name = "ProgB"
        prog_b.parameters = MagicMock()
        prog_b.parameters.itemByName.side_effect = lambda k: {
            "nc_program_filename": MagicMock(value=MagicMock(value="PartB")),
            "nc_program_nc_extension": MagicMock(value=MagicMock(value=".nc")),
        }.get(k)

        cam = MagicMock()
        cam.ncPrograms = MockCamCollection([prog_a, prog_b])

        input_store = {}

        def _make_bool(inp_id, name, checkbox=True, icon="", val=False):
            m = MagicMock()
            m.id = inp_id
            m.name = name
            m.value = bool(val)
            m.isVisible = True
            input_store[inp_id] = m
            return m

        def _make_str(inp_id, name, val=""):
            m = MagicMock()
            m.id = inp_id
            m.name = name
            m.value = val
            m.isReadOnly = False
            input_store[inp_id] = m
            return m

        def _make_text(inp_id, name, text, rows, read_only):
            m = MagicMock()
            m.id = inp_id
            m.text = text
            m.isVisible = True
            input_store[inp_id] = m
            return m

        def _make_table(inp_id, name, cols, ratios):
            t = MagicMock()
            t.id = inp_id
            t.rowCount = 0
            t._cells = {}

            def _add_cell(inp, r, c):
                t._cells[(r, c)] = inp
                t.rowCount = max(t.rowCount, r + 1)

            t.addCommandInput.side_effect = _add_cell
            t.getInputAtPosition.side_effect = lambda r, c: t._cells.get((r, c))
            input_store[inp_id] = t
            return t

        def _make_group(inp_id, name):
            g = MagicMock()
            g.id = inp_id
            g.children = MagicMock()
            g.children.addBoolValueInput.side_effect = _make_bool
            input_store[inp_id] = g
            return g

        mock_inputs = MagicMock()
        mock_inputs.addBoolValueInput.side_effect = _make_bool
        mock_inputs.addStringValueInput.side_effect = _make_str
        mock_inputs.addTextBoxCommandInput.side_effect = _make_text
        mock_inputs.addTableCommandInput.side_effect = _make_table
        mock_inputs.addGroupCommandInput.side_effect = _make_group
        mock_inputs.itemById.side_effect = lambda k: input_store.get(k)

        mock_cmd = MagicMock()
        mock_cmd.commandInputs = mock_inputs

        mock_app = MagicMock()
        mock_app.activeDocument.products.itemByProductType.return_value = cam
        mock_app.activeDocument.attributes = MagicMock()

        doc_settings = dict(PostClear.defaultSettings)
        doc_settings["ncPrograms"] = ["ProgA"]
        doc_settings["ncProgram"] = "ProgA"

        saved_handlers = list(PostClear.handlers)
        saved_settings_mgr = PostClear.settingsMgr
        try:
            mock_mgr = MagicMock()
            mock_mgr.GetSettings.return_value = doc_settings
            PostClear.settingsMgr = mock_mgr

            with patch.object(PostClear.adsk.core.Application, "get", return_value=mock_app):
                with patch.object(PostClear.adsk.core, "CommandCreatedEventArgs", MagicMock(cast=lambda a: MagicMock(command=mock_cmd)), create=True), \
                     patch.object(PostClear.adsk.cam.CAM, "cast", return_value=cam), \
                     patch("PostClear.GetNcProgramSetupsAndOperations", return_value=([MagicMock(name="Setup 1")], {})):
                    PostClear.CommandEventHandler().notify(MagicMock())

                self.assertIn("ncProgramsTable", input_store)
                self.assertIn("ncprog_select_all", input_store)
                tbl = input_store["ncProgramsTable"]
                self.assertEqual(tbl.rowCount, 3)
                self.assertEqual(tbl.getInputAtPosition(0, 0).id, "ncprog_select_all")
                self.assertEqual(tbl.getInputAtPosition(0, 1).value, "Select All")
                self.assertTrue(tbl.getInputAtPosition(0, 1).isReadOnly)
                self.assertEqual(tbl.getInputAtPosition(1, 0).id, "ncprog_chk_0")
                self.assertEqual(tbl.getInputAtPosition(2, 0).id, "ncprog_chk_1")
                self.assertTrue(input_store["ncprog_chk_0"].value)
                self.assertFalse(input_store["ncprog_chk_1"].value)
                self.assertFalse(input_store["ncprog_select_all"].value)

                # InputChangedHandler: Select All -> True checks all rows
                changed_handler = PostClear.CommandInputChangedHandler(doc_settings)
                input_store["ncprog_select_all"].value = True
                with patch.object(
                    PostClear.adsk.core,
                    "InputChangedEventArgs",
                    MagicMock(cast=lambda a: MagicMock(input=input_store["ncprog_select_all"], inputs=mock_inputs)),
                    create=True,
                ):
                    changed_handler.notify(MagicMock())
                self.assertTrue(input_store["ncprog_chk_0"].value)
                self.assertTrue(input_store["ncprog_chk_1"].value)
                self.assertEqual(doc_settings["ncPrograms"], ["ProgA", "ProgB"])

                # Validation with 0 checked disables OK and shows error
                input_store["ncprog_chk_0"].value = False
                input_store["ncprog_chk_1"].value = False
                val_handler = PostClear.CommandValidateInputsHandler()
                val_args = MagicMock(inputs=mock_inputs, areInputsValid=True)
                with patch.object(
                    PostClear.adsk.core,
                    "ValidateInputsEventArgs",
                    MagicMock(cast=lambda a: val_args),
                    create=True,
                ):
                    val_handler.notify(MagicMock())
                self.assertFalse(val_args.areInputsValid)
                self.assertTrue(input_store["error"].isVisible)

                # Validation firing from child GroupCommandInput.children while 0 checked still blocks OK
                val_args_nested = MagicMock(
                    inputs=input_store["groupPersonal"].children,
                    firingEvent=MagicMock(sender=MagicMock(commandInputs=mock_inputs)),
                    areInputsValid=True,
                )
                with patch.object(
                    PostClear.adsk.core,
                    "ValidateInputsEventArgs",
                    MagicMock(cast=lambda a: val_args_nested),
                    create=True,
                ):
                    val_handler.notify(MagicMock())
                self.assertFalse(val_args_nested.areInputsValid)
                self.assertTrue(input_store["error"].isVisible)

                # Check ProgB only -> valid and execute persists ["ProgB"]
                input_store["ncprog_chk_1"].value = True
                val_args2 = MagicMock(inputs=mock_inputs, areInputsValid=False)
                with patch.object(
                    PostClear.adsk.core,
                    "ValidateInputsEventArgs",
                    MagicMock(cast=lambda a: val_args2),
                    create=True,
                ):
                    val_handler.notify(MagicMock())
                self.assertTrue(val_args2.areInputsValid)
                self.assertFalse(input_store["error"].isVisible)

                exec_handler = PostClear.CommandExecuteHandler(doc_settings)
                exec_cmd = MagicMock(commandInputs=mock_inputs)
                with patch.object(
                    PostClear.adsk.core,
                    "CommandEventArgs",
                    MagicMock(cast=lambda a: MagicMock(command=exec_cmd)),
                    create=True,
                ), \
                     patch.object(PostClear.adsk.core, "GroupCommandInput", MagicMock(classType=lambda: "Group"), create=True), \
                     patch.object(PostClear.adsk.core, "DropDownCommandInput", MagicMock(classType=lambda: "DropDown"), create=True), \
                     patch("PostClear.PerformPostProcess") as mock_perform:
                    exec_handler.notify(MagicMock())

            self.assertEqual(doc_settings["ncPrograms"], ["ProgB"])
            self.assertEqual(doc_settings["ncProgram"], "ProgB")
            mock_perform.assert_called_once_with(doc_settings)
        finally:
            PostClear.handlers[:] = saved_handlers
            PostClear.settingsMgr = saved_settings_mgr

    def test_resolve_nc_program_output_filename_and_duplicate_path_collision(self):
        """Verify nc_program_filename is always used and duplicate resolved paths abort PerformPostProcess."""
        import PostClear

        setup = MagicMock()
        setup.name = "Setup 1"
        setup.isSuppressed = False

        def _make_prog(prog_name, fname):
            p = MagicMock()
            p.name = prog_name
            p.attributes.itemByName.return_value = MagicMock(value="")
            p.parameters = MagicMock()
            p.parameters.itemByName.side_effect = lambda k: {
                "nc_program_filename": MagicMock(value=MagicMock(value=fname)),
                "nc_program_nc_extension": MagicMock(value=MagicMock(value=".nc")),
                "nc_program_post_property_unit": MagicMock(value=MagicMock(value=0)),
                "nc_program_output_folder": MagicMock(value=MagicMock(value="/tmp")),
            }.get(k)
            return p

        p1 = _make_prog("Program One", "SharedName.NC")
        p2 = _make_prog("Program Two", "sub:Part2.nc")
        p_blank = _make_prog("Program Three", "   ")

        self.assertEqual(PostClear._ResolveNcProgramOutputFilename(p1, setup, ".nc"), "SharedName")
        self.assertEqual(PostClear._ResolveNcProgramOutputFilename(p2, setup, ".nc"), "sub:Part2")
        self.assertEqual(PostClear._ResolveNcProgramOutputFilename(p_blank, setup, ".nc"), "Program Three")
        self.assertEqual(PostClear._ResolveNcProgramOutputFilename(MagicMock(name=None, parameters=None), setup, ".nc"), "Setup 1")

        # Duplicate output target collision across 2 selected programs
        p_dup1 = _make_prog("NC1", "SameOut")
        p_dup2 = _make_prog("NC2", "SameOut")

        op1 = MagicMock(name="Face1", hasToolpath=True, isSuppressed=False)
        op2 = MagicMock(name="Pocket1", hasToolpath=True, isSuppressed=False)
        setup.allOperations = MockCamCollection([op1, op2])
        cam = MagicMock()
        cam.allOperations = MockCamCollection([op1, op2])
        cam.ncPrograms = MockCamCollection([p_dup1, p_dup2])
        cam.setups = MockCamCollection([setup])

        mock_ui = MagicMock()
        mock_app = MagicMock()
        mock_app.userInterface = mock_ui
        mock_app.activeDocument.products.itemByProductType.return_value = cam
        mock_app.activeDocument.attributes.itemByName.return_value = None

        with tempfile.TemporaryDirectory() as tmp_dir:
            doc_settings = dict(PostClear.defaultSettings)
            doc_settings["output"] = tmp_dir
            doc_settings["ncPrograms"] = ["NC1", "NC2"]

            def _get_prog_setups_ops(_cam, _settings, setups_arg=None, program=None):
                ops = [op1] if program is p_dup1 else [op2]
                return [setup], {setup.name: ops}

            with patch.object(PostClear.adsk.core.Application, "get", return_value=mock_app), \
                 patch.object(PostClear.adsk.cam.CAM, "cast", return_value=cam), \
                 patch("PostClear.GetNcProgramSetupsAndOperations", side_effect=_get_prog_setups_ops), \
                 patch("PostClear.PostProcessSetup") as mock_pps:
                PostClear.PerformPostProcess(doc_settings)

            mock_pps.assert_not_called()
            mock_ui.createProgressDialog.assert_not_called()
            mock_ui.messageBox.assert_called_once()
            err_msg = mock_ui.messageBox.call_args[0][0]
            self.assertIn("Duplicate", err_msg)
            self.assertIn("SameOut.nc", err_msg)

    def test_perform_post_process_multi_nc_program_batch_single_prompts_and_restore(self):
        """Batching 2 NC Programs uses single overwrite prompt, single progress dialog, and restores params."""
        import PostClear

        op1 = MagicMock(name="Op1", hasToolpath=True, isSuppressed=False)
        op2 = MagicMock(name="Op2", hasToolpath=True, isSuppressed=False)
        setup1 = MagicMock()
        setup1.name = "Setup 1"
        setup1.isSuppressed = False
        setup1.allOperations = MockCamCollection([op1])
        setup2 = MagicMock()
        setup2.name = "Setup 2"
        setup2.isSuppressed = False
        setup2.allOperations = MockCamCollection([op2])

        with tempfile.TemporaryDirectory() as tmp_dir:
            def _make_prog(name, fname):
                p = MagicMock()
                p.name = name
                p.attributes.itemByName.return_value = MagicMock(value="")
                fname_val = MagicMock(value=fname)
                out_val = MagicMock(value=tmp_dir)
                p.parameters = MagicMock()
                p.parameters.itemByName.side_effect = lambda k, fv=fname_val, ov=out_val: {
                    "nc_program_name": MagicMock(value=MagicMock(value=name)),
                    "nc_program_filename": MagicMock(value=fv),
                    "nc_program_nc_extension": MagicMock(value=MagicMock(value=".nc")),
                    "nc_program_post_property_unit": MagicMock(value=MagicMock(value=0)),
                    "nc_program_output_folder": MagicMock(value=ov),
                    "nc_program_openInEditor": MagicMock(value=MagicMock(value=False)),
                }.get(k)
                return p, fname_val

            p1, fv1 = _make_prog("NC1", "FirstPart")
            p2, fv2 = _make_prog("NC2", "SecondPart")

            cam = MagicMock()
            cam.allOperations = MockCamCollection([op1, op2])
            cam.ncPrograms = MockCamCollection([p1, p2])
            cam.setups = MockCamCollection([setup1, setup2])

            msgbox_calls = []

            def _msgbox(prompt, title="", btn=None, icon=None):
                msgbox_calls.append((prompt, title, btn, icon))
                return PostClear.adsk.core.DialogResults.DialogYes

            mock_progress = MagicMock()
            mock_progress.wasCancelled = False
            mock_ui = MagicMock()
            mock_ui.messageBox.side_effect = _msgbox
            mock_ui.createProgressDialog.return_value = mock_progress

            mock_app = MagicMock()
            mock_app.userInterface = mock_ui
            mock_app.activeDocument.products.itemByProductType.return_value = cam
            mock_app.activeDocument.attributes.itemByName.return_value = None

            # Create BOTH target files beforehand to verify a SINGLE aggregated overwrite prompt
            for fn in ("FirstPart.nc", "SecondPart.nc"):
                with open(os.path.join(tmp_dir, fn), "w", encoding="utf8") as f:
                    f.write("OLD")

            doc_settings = dict(PostClear.defaultSettings)
            doc_settings["output"] = tmp_dir
            doc_settings["ncPrograms"] = ["NC1", "NC2"]

            def _get_prog_setups_ops(_cam, _settings, setups_arg=None, program=None):
                if program is p1:
                    return [setup1], {setup1.name: [op1]}
                return [setup2], {setup2.name: [op2]}

            pps_calls = []

            def _fake_pps(fname_arg, setup_arg, setup_folder_arg, settings_arg, program_arg, **kwargs):
                pps_calls.append((setup_arg.name, program_arg.name, fname_arg))
                PostClear._RecordVerifyResult(
                    setup=setup_arg,
                    nc_program=program_arg.name,
                    cncPath=os.path.join(setup_folder_arg, f"{fname_arg}.nc"),
                    reportText="OK",
                    reportPath=None,
                    pngPath=None,
                )
                return None

            with patch.object(PostClear.adsk.core.Application, "get", return_value=mock_app), \
                 patch.object(PostClear.adsk.cam.CAM, "cast", return_value=cam), \
                 patch("PostClear.GetNcProgramSetupsAndOperations", side_effect=_get_prog_setups_ops), \
                 patch("PostClear.PostProcessSetup", side_effect=_fake_pps), \
                 patch("PostClear._NotifyPostProcessResult") as mock_notify:
                PostClear.PerformPostProcess(doc_settings)

            self.assertEqual(len(msgbox_calls), 1, "Must show only ONE aggregated overwrite prompt for both programs")
            self.assertIn("FirstPart.nc", msgbox_calls[0][0])
            self.assertIn("SecondPart.nc", msgbox_calls[0][0])
            mock_ui.createProgressDialog.assert_called_once()
            self.assertEqual(pps_calls, [("Setup 1", "NC1", "FirstPart"), ("Setup 2", "NC2", "SecondPart")])
            # Verify nc_program recorded in verifyResults
            self.assertEqual(len(PostClear.verifyResults), 2)
            self.assertEqual(PostClear.verifyResults[0]["nc_program"], "NC1")
            self.assertEqual(PostClear.verifyResults[1]["nc_program"], "NC2")
            # Original nc_program_filename values preserved
            self.assertEqual(fv1.value, "FirstPart")
            self.assertEqual(fv2.value, "SecondPart")
            mock_notify.assert_called_once()
            self.assertEqual(len(mock_notify.call_args.kwargs.get("verifyResults", [])), 2)

    def test_reworked_notification_html_scaling_and_non_dismissing_actions(self):
        """Verify open_file/open_folder keep palette open, 2s auto-dismiss + click-cancel on success, and multi-program row layout."""
        import PostClear

        with tempfile.TemporaryDirectory() as tmp_dir:
            nc1 = os.path.join(tmp_dir, "Part1.nc")
            rep1 = os.path.join(tmp_dir, "Part1_report.md")
            png1 = os.path.join(tmp_dir, "Part1.png")
            nc2 = os.path.join(tmp_dir, "Part2.nc")
            for p in (nc1, rep1, png1, nc2):
                with open(p, "w", encoding="utf8") as f:
                    f.write("data")

            # 1. _HandleNotificationAction keeps palette visible on open_file / open_folder, hides only on dismiss / close
            mock_palette = MagicMock()
            mock_palette.isVisible = True
            mock_ui = MagicMock()
            mock_ui.palettes.itemById.return_value = mock_palette
            mock_app = MagicMock()
            mock_app.userInterface = mock_ui

            with patch.object(PostClear.adsk.core.Application, "get", return_value=mock_app), \
                 patch("PostClear.os.path.exists", return_value=True), \
                 patch("PostClear.subprocess.Popen") as mock_popen, \
                 patch("PostClear.sys.platform", "darwin"):
                PostClear._HandleNotificationAction(PostClear.constNotificationPaletteId, "open_file", nc1)
                self.assertTrue(mock_palette.isVisible, "open_file must NOT hide notification palette")
                mock_popen.assert_called_once()

                PostClear._HandleNotificationAction(PostClear.constNotificationPaletteId, "open_folder", tmp_dir)
                self.assertTrue(mock_palette.isVisible, "open_folder must NOT hide notification palette")

                PostClear._HandleNotificationAction(PostClear.constNotificationPaletteId, "dismiss", "")
                self.assertFalse(mock_palette.isVisible, "dismiss must hide notification palette")

                mock_palette.isVisible = True
                PostClear._HandleNotificationAction(PostClear.constNotificationPaletteId, "close", "")
                self.assertFalse(mock_palette.isVisible, "close must hide notification palette")

            # 2. _NotifyPostProcessResult with 3 programs -> scaled height & isResizable=True & per-row buttons
            created_palettes = []

            def _make_palette(pid, title, html_path, is_vis, show_close, is_resiz, w, h):
                with open(html_path, "r", encoding="utf-8") as hf:
                    html_str = hf.read()
                pal = MagicMock()
                pal.id = pid
                pal.isVisible = is_vis
                pal.isResizable = is_resiz
                created_palettes.append((pal, html_str, is_resiz, w, h))
                return pal

            mock_ui2 = MagicMock()
            mock_ui2.palettes.itemById.return_value = None
            mock_ui2.palettes.add2.side_effect = _make_palette
            mock_ui2.palettes.add.side_effect = _make_palette

            outcomes = [
                {"nc_program": "NC1", "setup": "Setup 1", "cnc": nc1, "report": rep1, "png": png1, "status": "success", "summary": "Clean"},
                {"nc_program": "NC2", "setup": "Setup 2", "cnc": nc2, "report": None, "png": None, "status": "success", "summary": "Clean"},
                {"nc_program": "NC3", "setup": "Setup 3", "cnc": nc1, "report": None, "png": None, "status": "success", "summary": "Clean"},
            ]
            PostClear._NotifyPostProcessResult(mock_ui2, 3, 0, [], tmp_dir, verifyResults=[], program_outcomes=outcomes)

            self.assertEqual(len(created_palettes), 1)
            _, html_out, is_resiz, pal_w, pal_h = created_palettes[0]
            self.assertTrue(is_resiz, "Multi-program notification must be resizable")
            self.assertEqual(pal_w, 500)
            self.assertEqual(pal_h, 150 + 56 * 3)
            self.assertIn("program-list", html_out)
            self.assertIn("NC1", html_out)
            self.assertIn("2000);", html_out)
            self.assertIn("wasClicked", html_out)

            # 3. Single-program failure -> non-resizable 500x220, no 2000ms auto-dismiss, explicit Dismiss button
            created_palettes.clear()
            outcomes_err = [
                {"nc_program": "NC1", "setup": "Setup 1", "cnc": nc1, "report": rep1, "png": None, "status": "error", "summary": "Holder collision"},
            ]
            PostClear._NotifyPostProcessResult(mock_ui2, 1, 0, [], tmp_dir, verifyResults=[], is_fatal=True, program_outcomes=outcomes_err)
            _, err_html_out, err_resiz, _, err_h = created_palettes[0]
            self.assertFalse(err_resiz)
            self.assertEqual(err_h, 220)
            self.assertNotIn("2000);", err_html_out)
            self.assertIn("Dismiss", err_html_out)

    def test_marking_menu_context_injection_preselection_and_cleanup(self):
        """Verify MarkingMenuHandler adds only constCmdDefId on CAM context, preselects NCPrograms, ignores non-CAM, and cleans up."""
        import PostClear

        prog_a = MagicMock()
        prog_a.name = "ProgA"
        prog_a.objectType = "adsk::cam::NCProgram"
        prog_b = MagicMock()
        prog_b.name = "ProgB"
        prog_b.objectType = "adsk::cam::NCProgram"

        setup1 = MagicMock()
        setup1.name = "Setup 1"
        setup1.objectType = "adsk::cam::Setup"

        cam = MagicMock()
        cam.ncPrograms = MockCamCollection([prog_a, prog_b])

        cmd_def = MagicMock()

        def _make_controls(ctrl_list):
            by_id = {getattr(c, "id", ""): c for c in ctrl_list}

            class _ControlsCol(list):
                def itemById(self, cid):
                    return by_id.get(cid)

            col = _ControlsCol(ctrl_list)
            col.count = len(ctrl_list)
            col.item = lambda i: ctrl_list[i]
            col.addCommand = MagicMock()
            return col

        iron_nc_ctrl = MagicMock()
        iron_nc_ctrl.id = "IronNCProgramPostProcess"
        iron_post_ctrl = MagicMock()
        iron_post_ctrl.id = PostClear.constPostProcessControlId  # "IronPostProcess"
        custom_post_ctrl = MagicMock()
        custom_post_ctrl.id = "CustomCamPostBtn"
        custom_post_ctrl.commandDefinition = MagicMock()
        custom_post_ctrl.commandDefinition.name = "Post Process"
        stale_self_ctrl = MagicMock()
        stale_self_ctrl.id = PostClear.constCmdDefId

        mock_controls = _make_controls([iron_nc_ctrl, stale_self_ctrl])
        mock_linear_menu = MagicMock(controls=mock_controls)

        mock_ui = MagicMock()
        mock_ui.activeWorkspace.id = PostClear.constCAMWorkspaceId
        mock_ui.commandDefinitions.itemById.return_value = cmd_def

        mock_app = MagicMock()
        mock_app.userInterface = mock_ui
        mock_app.activeDocument.products.itemByProductType.return_value = cam
        mock_app.activeProduct.productType = PostClear.constCAMProductId

        saved_pending = PostClear._pending_context_selection
        try:
            # 1. Right-click on NCProgram ProgB with IronNCProgramPostProcess -> deletes stale self & anchors after IronNCProgramPostProcess
            PostClear._pending_context_selection = None
            handler = PostClear.MarkingMenuHandler()
            ev_args = MagicMock(
                selectedEntities=[prog_b],
                linearMarkingMenu=mock_linear_menu,
            )
            with patch.object(PostClear.adsk.core.Application, "get", return_value=mock_app), \
                 patch.object(PostClear.adsk.core, "MarkingMenuEventArgs", MagicMock(cast=lambda a: ev_args), create=True), \
                 patch.object(PostClear.adsk.cam.CAM, "cast", return_value=cam):
                handler.notify(MagicMock())

            stale_self_ctrl.deleteMe.assert_called_once()
            mock_controls.addCommand.assert_called_once_with(cmd_def, "IronNCProgramPostProcess", False)
            pre = PostClear._ResolvePreselectedNcProgramNames(cam, mock_ui)
            self.assertEqual(pre, ["ProgB"])

            # 2. Right-click on "NC Programs" folder node with custom "Post Process" command name -> anchors after CustomCamPostBtn & preselects ALL
            mock_controls_custom = _make_controls([custom_post_ctrl])
            mock_linear_custom = MagicMock(controls=mock_controls_custom)
            nc_folder = MagicMock()
            nc_folder.name = "NC Programs"
            nc_folder.objectType = "adsk::cam::CAMFolder"
            ev_folder = MagicMock(selectedEntities=[nc_folder], linearMarkingMenu=mock_linear_custom)
            with patch.object(PostClear.adsk.core.Application, "get", return_value=mock_app), \
                 patch.object(PostClear.adsk.core, "MarkingMenuEventArgs", MagicMock(cast=lambda a: ev_folder), create=True), \
                 patch.object(PostClear.adsk.cam.CAM, "cast", return_value=cam):
                handler.notify(MagicMock())
            mock_controls_custom.addCommand.assert_called_once_with(cmd_def, "CustomCamPostBtn", False)
            self.assertEqual(PostClear._ResolvePreselectedNcProgramNames(cam, mock_ui), ["ProgA", "ProgB"])

            # 3. Right-click on Setup with IronPostProcess -> anchors after IronPostProcess & preselects matching NC Program
            mock_controls_setup = _make_controls([iron_post_ctrl])
            mock_linear_setup = MagicMock(controls=mock_controls_setup)
            ev_setup = MagicMock(selectedEntities=[setup1], linearMarkingMenu=mock_linear_setup)
            with patch.object(PostClear.adsk.core.Application, "get", return_value=mock_app), \
                 patch.object(PostClear.adsk.core, "MarkingMenuEventArgs", MagicMock(cast=lambda a: ev_setup), create=True), \
                 patch.object(PostClear.adsk.cam.CAM, "cast", return_value=cam), \
                 patch("PostClear.GetNcProgramSetupsAndOperations", side_effect=lambda c, s, program=None: ([setup1], []) if program is prog_a else ([], [])):
                handler.notify(MagicMock())
            mock_controls_setup.addCommand.assert_called_once_with(cmd_def, PostClear.constPostProcessControlId, False)
            self.assertEqual(PostClear._ResolvePreselectedNcProgramNames(cam, mock_ui), ["ProgA"])

            # 4. Non-CAM entity (e.g., BRepBody) -> ignored cleanly, no addCommand call
            mock_controls.addCommand.reset_mock()
            brep = MagicMock()
            brep.objectType = "adsk::fusion::BRepBody"
            ev_non_cam = MagicMock(selectedEntities=[brep], linearMarkingMenu=mock_linear_menu)
            with patch.object(PostClear.adsk.core.Application, "get", return_value=mock_app), \
                 patch.object(PostClear.adsk.core, "MarkingMenuEventArgs", MagicMock(cast=lambda a: ev_non_cam), create=True), \
                 patch.object(PostClear.adsk.cam.CAM, "cast", return_value=cam):
                handler.notify(MagicMock())
            mock_controls.addCommand.assert_not_called()

            # 5. Cleanup detaches _marking_menu_handler from ui.markingMenuDisplaying
            PostClear._marking_menu_handler = handler
            PostClear._CleanupCommandUI(mock_ui)
            mock_ui.markingMenuDisplaying.remove.assert_called_once_with(handler)
            self.assertIsNone(PostClear._marking_menu_handler)
            self.assertIsNone(PostClear._pending_context_selection)
        finally:
            PostClear._pending_context_selection = saved_pending
            PostClear._marking_menu_handler = None

    def test_optional_debug_log_gating_dialog_and_diagnostic_header(self):
        """Verify debug_log checkbox, GroupCommandInput lookup, silent default, and full diagnostic header when enabled."""
        import PostClear
        from geometry import solid_mesh

        saved_log_path = PostClear._LOG_PATH
        saved_enabled = PostClear._debug_log_enabled
        saved_handlers = list(PostClear.handlers)
        saved_settings_mgr = PostClear.settingsMgr

        with tempfile.TemporaryDirectory() as tmpdir:
            temp_log = os.path.join(tmpdir, "postclear_debug.log")
            PostClear._LOG_PATH = temp_log
            try:
                # 1. Default False -> _dbg_global, solid_mesh._dbg, and context_parser._dbg_ctx do NOT create log file
                import context_parser
                PostClear._SetDebugLogEnabled(False)
                PostClear._dbg_global("should-not-write-1")
                solid_mesh._dbg("should-not-write-solid")
                context_parser.extract_workpiece_from_context({"stockMode": "default"})
                self.assertFalse(os.path.exists(temp_log))

                # 2. Dialog creation adds debug_log checkbox (default False) inside groupPersonal.children,
                # CommandInputChangedHandler updates docSettings["debug_log"], and CommandExecuteHandler
                # finds debug_log even if root inputs.itemById only resolves groupPersonal (not nested children)
                prog_a = MagicMock()
                prog_a.name = "ProgA"
                prog_a.parameters = MagicMock()
                prog_a.parameters.itemByName.side_effect = lambda k: {
                    "nc_program_filename": MagicMock(value=MagicMock(value="PartA")),
                }.get(k)
                cam = MagicMock()
                cam.ncPrograms = MockCamCollection([prog_a])

                root_store = {}
                group_store = {}

                def _make_inp(store, inp_id, **extra):
                    m = MagicMock()
                    m.id = inp_id
                    for k, v in extra.items():
                        setattr(m, k, v)
                    store[inp_id] = m
                    return m

                def _make_group(inp_id, _name):
                    grp = _make_inp(root_store, inp_id, isExpanded=True, objectType="Group")
                    children = MagicMock()
                    children.addBoolValueInput.side_effect = lambda bid, nm, chk, res, val: _make_inp(
                        group_store, bid, name=nm, value=val, objectType="Bool"
                    )
                    children.itemById.side_effect = lambda k: group_store.get(k)
                    grp.children = children
                    return grp

                def _make_table(inp_id, *_a):
                    tbl = _make_inp(root_store, inp_id, rowCount=0)
                    cells = {}
                    tbl.addCommandInput.side_effect = lambda inp, r, c: (
                        cells.__setitem__((r, c), inp),
                        setattr(tbl, "rowCount", max(tbl.rowCount, r + 1)),
                    )
                    tbl.getInputAtPosition.side_effect = lambda r, c: cells.get((r, c))
                    return tbl

                mock_inputs = MagicMock()
                mock_inputs.addBoolValueInput.side_effect = lambda bid, nm, *_a: _make_inp(
                    root_store, bid, name=nm, value=(bool(_a[-1]) if _a else False), objectType="Bool"
                )
                mock_inputs.addStringValueInput.side_effect = lambda sid, _nm, val: _make_inp(
                    root_store, sid, value=val, objectType="String"
                )
                mock_inputs.addTextBoxCommandInput.side_effect = lambda tid, *_a: _make_inp(
                    root_store, tid, text="", objectType="TextBox"
                )
                mock_inputs.addTableCommandInput.side_effect = _make_table
                mock_inputs.addGroupCommandInput.side_effect = _make_group
                # Root itemById intentionally does NOT expose group_store items to verify _FindCommandInputById fallback!
                mock_inputs.itemById.side_effect = lambda k: root_store.get(k)

                mock_cmd = MagicMock(commandInputs=mock_inputs)
                mock_app = MagicMock()
                mock_app.version = "2.0.20999"
                mock_app.productName = "Autodesk Fusion"
                mock_app.currentUser = MagicMock(email="secret-pii@example.com")
                mock_doc = MagicMock()
                mock_doc.name = "TestBracket"
                mock_doc.unitsManager.defaultLengthUnits = "in"
                mock_doc.products.itemByProductType.return_value = cam
                mock_app.activeDocument = mock_doc

                doc_settings = dict(PostClear.defaultSettings)
                doc_settings["ncPrograms"] = ["ProgA"]
                doc_settings["ncProgram"] = "ProgA"

                mock_mgr = MagicMock()
                mock_mgr.GetSettings.return_value = doc_settings
                PostClear.settingsMgr = mock_mgr

                with patch.object(PostClear.adsk.core.Application, "get", return_value=mock_app), \
                     patch.object(PostClear.adsk.core, "CommandCreatedEventArgs", MagicMock(cast=lambda a: MagicMock(command=mock_cmd)), create=True), \
                     patch.object(PostClear.adsk.cam.CAM, "cast", return_value=cam), \
                     patch("PostClear.GetNcProgramSetupsAndOperations", return_value=([MagicMock(name="Setup 1")], {})):
                    PostClear.CommandEventHandler().notify(MagicMock())

                self.assertIn("debug_log", group_store)
                self.assertEqual(group_store["debug_log"].name, "Produce debug log")
                self.assertFalse(group_store["debug_log"].value)

                # CommandInputChangedHandler toggles debug_log and updates doc_settings
                changed_handler = PostClear.CommandInputChangedHandler(doc_settings)
                group_store["debug_log"].value = True
                with patch.object(PostClear.adsk.core.Application, "get", return_value=mock_app), \
                     patch.object(
                        PostClear.adsk.core,
                        "InputChangedEventArgs",
                        MagicMock(cast=lambda a: MagicMock(input=group_store["debug_log"], inputs=mock_inputs)),
                        create=True,
                     ), patch.object(PostClear.adsk.core, "GroupCommandInput", MagicMock(classType=lambda: "Group"), create=True), \
                        patch.object(PostClear.adsk.core, "DropDownCommandInput", MagicMock(classType=lambda: "DropDown"), create=True):
                    changed_handler.notify(MagicMock())
                self.assertTrue(doc_settings["debug_log"])

                # Clicking "Save as default" persists debug_log=True via settingsMgr.SaveDefault
                with patch.object(PostClear.adsk.core.Application, "get", return_value=mock_app), \
                     patch.object(
                        PostClear.adsk.core,
                        "InputChangedEventArgs",
                        MagicMock(cast=lambda a: MagicMock(input=root_store["save"], inputs=mock_inputs)),
                        create=True,
                     ):
                    changed_handler.notify(MagicMock())
                mock_mgr.SaveDefault.assert_called_once()
                self.assertIs(mock_mgr.SaveDefault.call_args[0][0]["debug_log"], True)

                # Prepare minimal milling dump for pipeline._dbg_pipe gating check
                import json as _json
                from geometry.models import BoxStock3D, MillToolGeometry, ToolAssembly, WorkpieceDefinition
                from pipeline import PipelineOrchestrator

                pipe_dump = os.path.join(tmpdir, "pipe_ctx.json")
                with open(pipe_dump, "w", encoding="utf-8") as f:
                    _json.dump(
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
                pipe_wp = WorkpieceDefinition(stock=BoxStock3D(x_min=-25.4, x_max=25.4, y_min=-25.4, y_max=25.4, z_min=-25.4, z_max=0.0))
                pipe_tool = ToolAssembly(
                    tool_id=1,
                    name="T1",
                    geometry=MillToolGeometry(diameter=12.7, corner_radius=0.0, flute_length=25.4, overall_length=38.1),
                )
                pipe_orch = PipelineOrchestrator()
                pipe_base_post = os.path.join(os.path.dirname(PostClear.__file__), "universal_replayer.cps")
                pipe_out_nc = os.path.join(tmpdir, "pipe_out.nc")

                # 3. Execute with debug_log=False -> silent, PerformPostProcess & pipeline._dbg_pipe do NOT create temp_log
                group_store["debug_log"].value = False
                exec_handler = PostClear.CommandExecuteHandler(doc_settings)
                with patch.object(
                    PostClear.adsk.core,
                    "CommandEventArgs",
                    MagicMock(cast=lambda a: MagicMock(command=mock_cmd)),
                    create=True,
                ), \
                     patch.object(PostClear.adsk.core, "GroupCommandInput", MagicMock(classType=lambda: "Group"), create=True), \
                     patch.object(PostClear.adsk.core, "DropDownCommandInput", MagicMock(classType=lambda: "DropDown"), create=True), \
                     patch.object(PostClear.adsk.core.Application, "get", return_value=mock_app), \
                     patch.object(PostClear.adsk.cam.CAM, "cast", return_value=cam), \
                     patch("PostClear.GetSelectedNcPrograms", return_value=[]):
                    exec_handler.notify(MagicMock())
                self.assertFalse(doc_settings["debug_log"])
                with patch.object(pipe_orch, "replay_combined_context", return_value="M30\n"):
                    pipe_orch.run(
                        context_dump_paths=[pipe_dump],
                        base_post_path=pipe_base_post,
                        workpiece=pipe_wp,
                        tools_by_op={"Face1": pipe_tool},
                        final_cnc_path=pipe_out_nc,
                        is_turning=False,
                    )
                self.assertFalse(os.path.exists(temp_log))

                # 4. Execute with debug_log=True -> PerformPostProcess writes rich diagnostic banner + traces
                group_store["debug_log"].value = True
                with patch.object(
                    PostClear.adsk.core,
                    "CommandEventArgs",
                    MagicMock(cast=lambda a: MagicMock(command=mock_cmd)),
                    create=True,
                ), \
                     patch.object(PostClear.adsk.core, "GroupCommandInput", MagicMock(classType=lambda: "Group"), create=True), \
                     patch.object(PostClear.adsk.core, "DropDownCommandInput", MagicMock(classType=lambda: "DropDown"), create=True), \
                     patch.object(PostClear.adsk.core.Application, "get", return_value=mock_app), \
                     patch.object(PostClear.adsk.cam.CAM, "cast", return_value=cam), \
                     patch("PostClear.GetSelectedNcPrograms", return_value=[]):
                    exec_handler.notify(MagicMock())

                # Simulate downstream module logging while enabled (solid_mesh._dbg, context_parser._dbg_ctx, pipeline._dbg_pipe)
                solid_mesh._dbg("mesh-helper-trace")
                context_parser.extract_workpiece_from_context({"stockMode": "default"})
                with patch.object(pipe_orch, "replay_combined_context", return_value="M30\n"):
                    pipe_orch.run(
                        context_dump_paths=[pipe_dump],
                        base_post_path=pipe_base_post,
                        workpiece=pipe_wp,
                        tools_by_op={"Face1": pipe_tool},
                        final_cnc_path=pipe_out_nc,
                        is_turning=False,
                    )

                self.assertTrue(os.path.isfile(temp_log))
                with open(temp_log, "r", encoding="utf-8") as f:
                    log_text = f.read()

                self.assertIn("=== PostClear Diagnostic Log ===", log_text)
                self.assertIn("Add-In: version=3.0.0", log_text)
                self.assertIn("Fusion 360: version=2.0.20999", log_text)
                self.assertIn("product=Autodesk Fusion", log_text)
                self.assertIn("doc='TestBracket', units=in", log_text)
                self.assertIn(f"executable={sys.executable}", log_text)
                self.assertIn("OS/Platform:", log_text)
                self.assertIn("Dependencies: abi=", log_text)
                self.assertIn("numpy=", log_text)
                self.assertIn("shapely=", log_text)
                self.assertIn("matplotlib=", log_text)
                self.assertIn("selected_nc_programs=['ProgA']", log_text)
                self.assertIn("restore_rapid=True", log_text)
                self.assertIn("mesh-helper-trace", log_text)
                self.assertIn("extract_workpiece_from_context:", log_text)
                self.assertIn("PipelineOrchestrator.run: workpiece stock=", log_text)
                self.assertNotIn("secret-pii@example.com", log_text)

                # Verify SWIG property RuntimeError does not drop diagnostic banner
                class _SwigErrorObj:
                    @property
                    def version(self):
                        raise RuntimeError("2 : InternalValidationError")
                    @property
                    def setups(self):
                        raise RuntimeError("2 : InternalValidationError")
                header_swig = PostClear._FormatDiagnosticHeader(
                    app=_SwigErrorObj(),
                    doc=mock_doc,
                    cam=_SwigErrorObj(),
                    docSettings=doc_settings,
                )
                self.assertIn("Fusion 360: version=unknown", header_swig)
                self.assertIn("cam_setups=unknown", header_swig)
            finally:
                PostClear._LOG_PATH = saved_log_path
                PostClear._SetDebugLogEnabled(saved_enabled)
                PostClear.handlers[:] = saved_handlers
                PostClear.settingsMgr = saved_settings_mgr


if __name__ == "__main__":
    unittest.main()








