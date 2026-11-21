#!/usr/bin/env python3
"""
Focused Stage 2 Host tests covering changes in PostClear.py:
1. Toast palette uses single-transport incomingFromHTML (no action://, no _NotificationNavigationEventHandler,
   no _last_action debouncer, no osascript), and stop() clears _palette_handlers.
2. PerformPostProcess never snapshots/writes back NCProgram parameters, and UpdateSettings drops the 19-key strip loop.
3. PostProcessSetup skips empty-file create/delete probe, skips 300s generateToolpath loop,
   passes _skip_cap_check=True to _PostProcessUnifiedPipeline while direct calls validate by default.
4. _RecordVerifyResult avoids overwriting existing reportPath when error is falsy.
5. _ResolveBasePostPath iterates _GetPostSearchDirsByTier() directly without calling _GetPostSearchDirs().
6. Consolidated parameter extraction (_extract_scalar_param, _extract_float_param_mm, _ExtractPostProperties,
   _GetSetupStockMode, safe tool_number) and _record_failure closure preserving nc_program.
7. _same_cam_entity and simplified GetNcProgram / GetSelectedNcPrograms.
"""

import os
import sys
import tempfile
import types
import unittest
from unittest.mock import MagicMock, patch

if "adsk" not in sys.modules:
    adsk = types.ModuleType("adsk")
    adsk.cam = types.ModuleType("adsk.cam")
    adsk.core = types.ModuleType("adsk.core")
    adsk.fusion = types.ModuleType("adsk.fusion")
    sys.modules["adsk"] = adsk
    sys.modules["adsk.cam"] = adsk.cam
    sys.modules["adsk.core"] = adsk.core
    sys.modules["adsk.fusion"] = adsk.fusion
else:
    adsk = sys.modules["adsk"]
    for sub in ("cam", "core", "fusion"):
        if not hasattr(adsk, sub):
            mod = types.ModuleType(f"adsk.{sub}")
            setattr(adsk, sub, mod)
            sys.modules[f"adsk.{sub}"] = mod

for _handler_cls in (
    "CommandCreatedEventHandler",
    "InputChangedEventHandler",
    "ValidateInputsEventHandler",
    "CommandEventHandler",
    "HTMLEventHandler",
):
    if not hasattr(adsk.core, _handler_cls):
        setattr(adsk.core, _handler_cls, type(_handler_cls, (), {}))
if not hasattr(adsk.core, "Application"):
    adsk.core.Application = types.SimpleNamespace(get=lambda: types.SimpleNamespace(log=lambda _msg: None))
if not hasattr(adsk.core, "MessageBoxButtonTypes"):
    adsk.core.MessageBoxButtonTypes = types.SimpleNamespace(OKButtonType=0, YesNoButtonType=1)
if not hasattr(adsk.core, "MessageBoxIconTypes"):
    adsk.core.MessageBoxIconTypes = types.SimpleNamespace(
        InformationIconType=0,
        WarningIconType=1,
        CriticalIconType=2,
        QuestionIconType=3,
    )
if not hasattr(adsk.core, "DialogResults"):
    adsk.core.DialogResults = types.SimpleNamespace(DialogYes=0, DialogNo=1, DialogOK=2)
if not hasattr(adsk.core, "ObjectCollection"):
    class MockObjectCollection(list):
        @classmethod
        def create(cls):
            return cls()
        def add(self, item):
            self.append(item)
    adsk.core.ObjectCollection = MockObjectCollection
if not hasattr(adsk.cam, "OperationTypes"):
    adsk.cam.OperationTypes = types.SimpleNamespace(TurningOperation=1, MillingOperation=0, JetOperation=2, AdditiveOperation=3)
if not hasattr(adsk.cam, "CAM"):
    adsk.cam.CAM = types.SimpleNamespace(cast=lambda x: x)
if not hasattr(adsk.cam, "PostOutputUnitOptions"):
    adsk.cam.PostOutputUnitOptions = types.SimpleNamespace(
        DocumentUnitsOutput=0, InchesOutput=1, MillimetersOutput=2
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

import PostClear


class TestStage2ToastAndLifecycle(unittest.TestCase):
    def tearDown(self):
        PostClear.verifyResults.clear()
        PostClear._palette_handlers.clear()
        PostClear.handlers.clear()

    def test_toast_single_transport_no_osascript(self):
        self.assertFalse(hasattr(PostClear, "_NotificationNavigationEventHandler"))
        self.assertFalse(hasattr(PostClear, "_NavigationEventHandlerBase"))
        self.assertFalse(hasattr(PostClear, "_last_action_timestamp"))
        self.assertFalse(hasattr(PostClear, "_last_action_key"))

        html_out = PostClear._BuildNotificationHTML(
            "Done",
            "success",
            "Completed",
            "",
            [("Open NC", "open_file", "/tmp/a.nc"), ("Dismiss", "dismiss", "")],
        )
        self.assertNotIn("action://", html_out)
        self.assertNotIn("window.location.href", html_out)
        self.assertIn('href="#"', html_out)
        self.assertIn("return false;", html_out)
        self.assertIn("adsk.fusionSendData", html_out)

        nav_added = []
        html_added = []

        class MockPalette:
            def __init__(self, pid, pname, html_arg, *_a):
                self.id = pid
                self.html_arg = html_arg
                self.incomingFromHTML = types.SimpleNamespace(add=lambda h: html_added.append(h))
                self.navigatingURL = types.SimpleNamespace(add=lambda h: nav_added.append(h))
                self.isVisible = False
            def setSize(self, *_a): return True
            def setPosition(self, *_a): return True
            def deleteMe(self): pass

        palettes = {}
        mock_ui = types.SimpleNamespace(
            palettes=types.SimpleNamespace(
                itemById=lambda pid: palettes.get(pid),
                add2=lambda pid, pname, h, *a: palettes.setdefault(pid, MockPalette(pid, pname, h, *a)),
            )
        )
        mock_vp = types.SimpleNamespace(
            width=1000, height=800,
            viewToScreen=lambda pt: types.SimpleNamespace(x=1200, y=900),
        )
        mock_app = types.SimpleNamespace(activeViewport=mock_vp, log=lambda _m: None)

        with patch("PostClear.sys.platform", "darwin"), \
             patch("PostClear.subprocess.check_output", side_effect=AssertionError("osascript must not run")), \
             patch.object(PostClear.adsk.core, "Point2D", types.SimpleNamespace(create=lambda x, y: (x, y)), create=True), \
             patch.object(PostClear.adsk.core.Application, "get", return_value=mock_app):
            PostClear._NotifyPostProcessResult(mock_ui, 1, 0, "", "/tmp", verifyResults=[])

        self.assertEqual(len(nav_added), 0, "navigatingURL must not be attached")
        self.assertEqual(len(html_added), 1, "incomingFromHTML must be attached")
        self.assertTrue(palettes[PostClear.constNotificationPaletteId].html_arg.startswith("file:///"))
        self.assertNotIn("\\", palettes[PostClear.constNotificationPaletteId].html_arg)

    def test_stop_clears_palette_handlers(self):
        fake_handler = object()
        PostClear._palette_handlers.append(fake_handler)
        PostClear.handlers.append(fake_handler)
        deleted = []
        mock_palette = types.SimpleNamespace(deleteMe=lambda: deleted.append(True))
        mock_ui = types.SimpleNamespace(
            workspaces=types.SimpleNamespace(itemById=lambda _id: None),
            allToolbarPanels=types.SimpleNamespace(itemById=lambda _id: None),
            commandDefinitions=types.SimpleNamespace(itemById=lambda _id: None),
            palettes=types.SimpleNamespace(itemById=lambda pid: mock_palette if pid == PostClear.constNotificationPaletteId else None),
        )
        mock_app = types.SimpleNamespace(userInterface=mock_ui)
        with patch.object(PostClear.adsk.core.Application, "get", return_value=mock_app):
            PostClear.stop({})
        self.assertEqual(deleted, [True])
        self.assertEqual(PostClear._palette_handlers, [])
        self.assertEqual(PostClear.handlers, [])


class TestStage2SetupAndPipelineChecks(unittest.TestCase):
    def tearDown(self):
        PostClear.verifyResults.clear()

    def test_record_verify_result_avoids_double_write_unless_error(self):
        setup = types.SimpleNamespace(name="Setup 1")
        with tempfile.TemporaryDirectory() as tmp_dir:
            report_path = os.path.join(tmp_dir, "out_report.md")
            cnc_path = os.path.join(tmp_dir, "out.nc")
            with open(report_path, "w", encoding="utf-8") as f:
                f.write("ORIGINAL_FROM_ORCHESTRATOR")

            PostClear._RecordVerifyResult(
                setup=setup, cncPath=cnc_path, reportText="DUPLICATE_TEXT",
                reportPath=report_path, error=None, nc_program="Prog1",
            )
            with open(report_path, "r", encoding="utf-8") as f:
                self.assertEqual(f.read(), "ORIGINAL_FROM_ORCHESTRATOR")

            # On error, reportPath IS updated with the failure markdown
            PostClear._RecordVerifyResult(
                setup=setup, cncPath=cnc_path, reportText="FAILURE_TEXT",
                reportPath=report_path, error="boom", nc_program="Prog1",
            )
            with open(report_path, "r", encoding="utf-8") as f:
                self.assertEqual(f.read(), "FAILURE_TEXT")

    def test_post_process_setup_skips_probe_toolpath_wait_and_redundant_cap_check(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            target_nc = os.path.join(tmp_dir, "part.nc")
            op = types.SimpleNamespace(name="Op1", isSuppressed=False, hasToolpath=True, isToolpathValid=True, strategy="adaptive2d")
            setup = types.SimpleNamespace(name="Setup 1", operationType=0, allOperations=[op], parameters=None)
            prog = types.SimpleNamespace(name="ProgA", parameters=None)

            cam = types.SimpleNamespace(
                checkToolpath=lambda _item: False,
                generateToolpath=MagicMock(side_effect=AssertionError("must not call generateToolpath")),
            )
            doc = types.SimpleNamespace(
                products=types.SimpleNamespace(itemByProductType=lambda _id: cam),
                attributes=MagicMock(),
            )
            app = types.SimpleNamespace(activeDocument=doc, userInterface=MagicMock(), log=lambda _m: None)

            captured_kwargs = []
            def fake_unified(*args, **kwargs):
                captured_kwargs.append(kwargs)
                return None

            open_calls = []
            orig_open = open
            def tracking_open(file, mode="r", *a, **kw):
                if os.path.abspath(str(file)) == os.path.abspath(target_nc) and "w" in mode:
                    open_calls.append((str(file), mode))
                return orig_open(file, mode, *a, **kw)

            with patch.object(PostClear.adsk.core.Application, "get", return_value=app), \
                 patch("PostClear._PostProcessUnifiedPipeline", side_effect=fake_unified), \
                 patch("builtins.open", side_effect=tracking_open):
                err = PostClear.PostProcessSetup(
                    "part", setup, tmp_dir, dict(PostClear.defaultSettings), prog, target_operations=[op]
                )

            self.assertIsNone(err)
            self.assertFalse(os.path.exists(target_nc), "probe must not create target file")
            self.assertEqual(open_calls, [], "must not open(target_nc, 'w') probe")
            self.assertFalse(cam.generateToolpath.called)
            self.assertEqual(len(captured_kwargs), 1)
            self.assertTrue(captured_kwargs[0].get("_skip_cap_check"))

    def test_unified_pipeline_validates_capability_by_default_and_handles_missing_tool_number(self):
        jet_setup = types.SimpleNamespace(name="Jet Setup", operationType=2, allOperations=[])
        res = PostClear._PostProcessUnifiedPipeline(
            "part", jet_setup, dict(PostClear.defaultSettings), None, "/tmp/part.nc", ".nc"
        )
        self.assertIsNotNone(res)
        self.assertIn("cutting / jet", res.lower())

        # Direct call with _skip_cap_check=False still rejects unsupported op capability,
        # while tool_number missing does not crash batching when ops pass cap check.
        op_no_tool = types.SimpleNamespace(
            name="OpNoTool", isSuppressed=False, hasToolpath=True, isToolpathValid=True,
            strategy="adaptive2d", tool=None,
        )
        mill_setup = types.SimpleNamespace(name="Mill Setup", operationType=0, allOperations=[op_no_tool], parameters=None)
        with tempfile.TemporaryDirectory() as tmp_dir:
            cps_path = os.path.join(tmp_dir, "mill.cps")
            with open(cps_path, "w", encoding="utf-8") as f:
                f.write('capabilities = CAPABILITY_MILLING;\n')
            prog = types.SimpleNamespace(name="ProgMill", postConfiguration=types.SimpleNamespace(name="mill"), parameters=None)
            out_nc = os.path.join(tmp_dir, "part.nc")

            mock_cam = MagicMock()
            app = types.SimpleNamespace(
                activeDocument=types.SimpleNamespace(products=types.SimpleNamespace(itemByProductType=lambda _id: mock_cam)),
                log=lambda _m: None,
            )
            with patch("PostClear._ResolveBasePostPath", return_value=cps_path), \
                 patch.object(PostClear.adsk.core.Application, "get", return_value=app), \
                 patch("PostClear._WriteDumperConfig", side_effect=RuntimeError("simulated dumper cfg fail")):
                err = PostClear._PostProcessUnifiedPipeline(
                    "part", mill_setup, dict(PostClear.defaultSettings), prog, out_nc, ".nc",
                    target_operations=[op_no_tool],
                )
            # Reaches Pass 1 without AttributeError on op.tool.parameters, and _record_failure stamps nc_program
            self.assertIsNotNone(err)
            self.assertIn("simulated dumper cfg fail", err)
            self.assertGreaterEqual(len(PostClear.verifyResults), 1)
            self.assertEqual(PostClear.verifyResults[-1]["nc_program"], "ProgMill")
            self.assertEqual(PostClear.verifyResults[-1]["error"], "simulated dumper cfg fail")

    def test_perform_post_process_does_not_mutate_program_parameters(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            writes = []
            class ReadOnlyValue:
                def __init__(self, v):
                    self._v = v
                @property
                def value(self):
                    return self._v
                @value.setter
                def value(self, new_v):
                    writes.append(new_v)
                    self._v = new_v

            param_map = {
                "nc_program_output_folder": types.SimpleNamespace(value=ReadOnlyValue(tmp_dir)),
                "nc_program_nc_extension": types.SimpleNamespace(value=ReadOnlyValue(".nc")),
                "nc_program_name": types.SimpleNamespace(value=ReadOnlyValue("ProgA")),
            }
            params = types.SimpleNamespace(itemByName=lambda k: param_map.get(k))
            op = types.SimpleNamespace(name="Op1", isSuppressed=False, hasToolpath=True, isToolpathValid=True, strategy="adaptive2d")
            setup = types.SimpleNamespace(name="Setup1", operationType=0, allOperations=[op], parameters=None)
            prog = types.SimpleNamespace(name="ProgA", parameters=params, setups=[setup])
            cam = types.SimpleNamespace(ncPrograms=[prog], allOperations=types.SimpleNamespace(count=1), setups=[setup])
            doc = types.SimpleNamespace(
                products=types.SimpleNamespace(itemByProductType=lambda _id: cam),
                attributes=MagicMock(),
            )
            mock_ui = MagicMock()
            mock_ui.createProgressDialog.return_value.wasCancelled = False
            app = types.SimpleNamespace(activeDocument=doc, userInterface=mock_ui, log=lambda _m: None)

            with patch.object(PostClear.adsk.core.Application, "get", return_value=app), \
                 patch("PostClear.GetNcProgramSetupsAndOperations", return_value=([setup], {setup.name: [op]})), \
                 patch("PostClear.PostProcessSetup", return_value=None), \
                 patch("PostClear._NotifyPostProcessResult"):
                settings = dict(PostClear.defaultSettings)
                settings["ncPrograms"] = ["ProgA"]
                PostClear.PerformPostProcess(settings)

            self.assertEqual(writes, [], "PerformPostProcess must never write back to NCProgram parameters")

    def test_perform_post_process_unnamed_program_threads_label_and_no_redundant_get_nc_program(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            cps_path = os.path.join(tmp_dir, "mill.cps")
            with open(cps_path, "w", encoding="utf-8") as f:
                f.write('capabilities = CAPABILITY_MILLING;\n')
            param_map = {
                "nc_program_output_folder": types.SimpleNamespace(value=types.SimpleNamespace(value=tmp_dir)),
                "nc_program_nc_extension": types.SimpleNamespace(value=types.SimpleNamespace(value=".nc")),
                "nc_program_filename": types.SimpleNamespace(value=types.SimpleNamespace(value="unnamed_out")),
            }
            params = types.SimpleNamespace(itemByName=lambda k: param_map.get(k))
            op = types.SimpleNamespace(name="Op1", isSuppressed=False, hasToolpath=True, isToolpathValid=True, strategy="adaptive2d", tool=None)
            setup = types.SimpleNamespace(name="Setup1", operationType=0, allOperations=[op], parameters=None)
            unnamed_prog = types.SimpleNamespace(
                name="",
                postConfiguration=types.SimpleNamespace(name="mill"),
                parameters=params,
                setups=[setup],
            )
            cam = types.SimpleNamespace(
                ncPrograms=[unnamed_prog],
                allOperations=types.SimpleNamespace(count=1),
                setups=[setup],
                checkToolpath=lambda _item: True,
            )
            doc = types.SimpleNamespace(
                products=types.SimpleNamespace(itemByProductType=lambda _id: cam),
                attributes=MagicMock(),
            )
            mock_ui = MagicMock()
            mock_ui.createProgressDialog.return_value.wasCancelled = False
            app = types.SimpleNamespace(activeDocument=doc, userInterface=mock_ui, log=lambda _m: None)

            # 1. GetNcProgram shim removed; GetSelectedNcPrograms([]) fails closed
            self.assertFalse(hasattr(PostClear, "GetNcProgram"))
            with patch.object(PostClear.adsk.core.Application, "get", return_value=app), \
                 patch("PostClear.GetSelectedNcPrograms", return_value=[]), \
                 patch("PostClear._NotifyPostProcessResult"):
                err = PostClear.PerformPostProcess(dict(PostClear.defaultSettings))
            self.assertEqual(err, "No NC Programs found to post process.")

            # 2. Unnamed NCProgram gets "NC Program 1" threaded into verifyResults["nc_program"]
            with patch.object(PostClear.adsk.core.Application, "get", return_value=app), \
                 patch("PostClear.GetSelectedNcPrograms", return_value=[unnamed_prog]), \
                 patch("PostClear.GetNcProgramSetupsAndOperations", return_value=([setup], {setup.name: [op]})), \
                 patch("PostClear._ResolveBasePostPath", return_value=cps_path), \
                 patch("PostClear._WriteDumperConfig", side_effect=RuntimeError("simulated fail")), \
                 patch("PostClear._NotifyPostProcessResult"):
                PostClear.PerformPostProcess(dict(PostClear.defaultSettings))

            self.assertGreaterEqual(len(PostClear.verifyResults), 1)
            self.assertEqual(PostClear.verifyResults[-1]["nc_program"], "NC Program 1")


class _MockTableCommandInput:
    """Faithful mock of Fusion's adsk.core.TableCommandInput visible-row invariant."""

    def __init__(self, inp_id, name):
        self.id = inp_id
        self.name = name
        self._min_visible_rows = 2
        self._max_visible_rows = 4
        self.hasGrid = True
        self.rowCount = 0
        self._rows = {}

    @property
    def minimumVisibleRows(self):
        return self._min_visible_rows

    @minimumVisibleRows.setter
    def minimumVisibleRows(self, val):
        if val < 0 or self._max_visible_rows < val:
            raise RuntimeError("3 : invalid value")
        self._min_visible_rows = val

    @property
    def maximumVisibleRows(self):
        return self._max_visible_rows

    @maximumVisibleRows.setter
    def maximumVisibleRows(self, val):
        if self._min_visible_rows > val:
            raise RuntimeError("3 : invalid value")
        self._max_visible_rows = val

    def addCommandInput(self, child, r, c):
        self._rows[(r, c)] = child
        self.rowCount = max(self.rowCount, r + 1)

    def getInputAtPosition(self, r, c):
        return self._rows.get((r, c))


class TestStage2DeduplicationHelpers(unittest.TestCase):
    def test_resolve_base_post_path_uses_tier_helper_directly(self):
        with tempfile.TemporaryDirectory() as tmp_dir:
            cps_path = os.path.join(tmp_dir, "grbl.cps")
            with open(cps_path, "w", encoding="utf-8") as f:
                f.write('capabilities = CAPABILITY_MILLING;\n')
            prog = types.SimpleNamespace(postConfiguration=types.SimpleNamespace(name="grbl"))
            with patch("PostClear._GetPostSearchDirs", side_effect=AssertionError("should call _GetPostSearchDirsByTier directly")), \
                 patch("PostClear._GetPostSearchDirsByTier", return_value=([tmp_dir], [], [])):
                res = PostClear._ResolveBasePostPath(prog, is_turning=False)
            self.assertEqual(os.path.abspath(res), os.path.abspath(cps_path))

    def test_consolidated_param_extractors_and_stock_mode(self):
        params = {
            "stockDiameter": types.SimpleNamespace(value=types.SimpleNamespace(value=2.5), expression="25 mm"),
            "exprOnly": types.SimpleNamespace(value=types.SimpleNamespace(value=MagicMock()), expression="1.0 in"),
            "job_stockMode": types.SimpleNamespace(value=types.SimpleNamespace(value=MagicMock()), expression="'solid'"),
        }
        self.assertAlmostEqual(PostClear._extract_float_param_mm(params, ["stockDiameter"]), 25.0)
        self.assertAlmostEqual(PostClear._extract_float_param_mm(params, ["exprOnly"]), 25.4)
        self.assertIsNone(PostClear._extract_scalar_param({"m": MagicMock()}, "m"))
        self.assertEqual(PostClear._extract_choice_value(MagicMock()), "")
        self.assertEqual(PostClear._extract_choice_value(None), "")

        setup = types.SimpleNamespace(name="S1", stockMode=MagicMock(), parameters=types.SimpleNamespace(itemByName=params.get))
        self.assertEqual(PostClear._GetSetupStockMode(setup), "solid")

        prog = types.SimpleNamespace(
            postParameters=types.SimpleNamespace(
                count=1,
                item=lambda _i: types.SimpleNamespace(name="useCycles", value=types.SimpleNamespace(value=True)),
            ),
            parameters=types.SimpleNamespace(
                itemByName=lambda k: types.SimpleNamespace(value=types.SimpleNamespace(value=4500))
                if k == "nc_program_maximum_spindle_speed" else None
            ),
        )
        props = PostClear._ExtractPostProperties(prog)
        self.assertEqual(props, {"useCycles": True, "maximumSpindleSpeed": 4500})

    def test_same_cam_entity_and_get_selected_nc_programs(self):
        a = types.SimpleNamespace(entityToken="tok-1", operationId=-1, name="Setup A")
        b = types.SimpleNamespace(entityToken="tok-1", operationId=-1, name="Renamed")
        op1 = types.SimpleNamespace(entityToken="", operationId=42, name="Drill")
        op2 = types.SimpleNamespace(entityToken="", operationId=42, name="Drill (2)")
        s_same_name = types.SimpleNamespace(name="ProgA", objectType="adsk::cam::Setup")

        self.assertTrue(PostClear._same_cam_entity(a, b))
        self.assertTrue(PostClear._same_cam_entity(op1, op2))
        self.assertFalse(PostClear._same_cam_entity(a, s_same_name, check_name=False))

        p1 = types.SimpleNamespace(name="ProgA", entityToken="p1")
        p2 = types.SimpleNamespace(name="ProgB", entityToken="p2")
        cam = types.SimpleNamespace(ncPrograms=[p1, p2])
        self.assertFalse(PostClear._is_nc_program_object(s_same_name, cam))

        self.assertFalse(hasattr(PostClear, "GetNcProgram"))
        self.assertEqual(PostClear.GetSelectedNcPrograms(cam, {"ncPrograms": ["ProgB"]}), [p2])
        self.assertEqual(PostClear.GetSelectedNcPrograms(cam, {"ncPrograms": []}), [p1])

    def test_command_dialog_and_validate_inputs_when_no_nc_programs(self):
        input_store = {}
        table_rows = {}

        def _make_bool(inp_id, name, is_cb, res="", val=False):
            obj = types.SimpleNamespace(id=inp_id, name=name, value=val, isEnabled=True, tooltip="", resourceFolder="")
            input_store[inp_id] = obj
            return obj

        def _make_str(inp_id, name, val):
            obj = types.SimpleNamespace(id=inp_id, name=name, value=val, isReadOnly=False)
            input_store[inp_id] = obj
            return obj

        def _make_text(inp_id, name, text, rows, read_only):
            obj = types.SimpleNamespace(id=inp_id, name=name, text=text, isFullWidth=False, isVisible=False)
            input_store[inp_id] = obj
            return obj

        def _make_group(inp_id, name):
            grp_children = types.SimpleNamespace(
                addBoolValueInput=lambda iid, n, cb, res, v: _make_bool(iid, n, cb, res, v),
                itemById=lambda iid: input_store.get(iid),
            )
            obj = types.SimpleNamespace(id=inp_id, name=name, isExpanded=True, children=grp_children)
            input_store[inp_id] = obj
            return obj

        def _make_table(inp_id, name, cols, col_ratios):
            tbl = _MockTableCommandInput(inp_id, name)
            input_store[inp_id] = tbl
            return tbl

        mock_inputs = types.SimpleNamespace(
            addTableCommandInput=_make_table,
            addBoolValueInput=_make_bool,
            addStringValueInput=_make_str,
            addTextBoxCommandInput=_make_text,
            addGroupCommandInput=_make_group,
            itemById=lambda iid: input_store.get(iid),
        )
        mock_cmd = types.SimpleNamespace(
            commandInputs=mock_inputs,
            execute=types.SimpleNamespace(add=lambda h: None),
            inputChanged=types.SimpleNamespace(add=lambda h: None),
            validateInputs=types.SimpleNamespace(add=lambda h: None),
        )

        empty_programs = MagicMock()
        empty_programs.count = 0
        empty_programs.__iter__ = lambda _self: iter(())
        cam = types.SimpleNamespace(ncPrograms=empty_programs)
        doc_settings = dict(PostClear.defaultSettings)

        mock_app = types.SimpleNamespace(
            userInterface=MagicMock(),
            activeDocument=types.SimpleNamespace(
                products=types.SimpleNamespace(itemByProductType=lambda _id: cam),
                attributes=MagicMock(),
            ),
        )

        saved_handlers = list(PostClear.handlers)
        saved_settings_mgr = PostClear.settingsMgr
        try:
            mock_mgr = MagicMock()
            mock_mgr.GetSettings.return_value = doc_settings
            PostClear.settingsMgr = mock_mgr

            with patch.object(PostClear.adsk.core.Application, "get", return_value=mock_app), \
                 patch.object(PostClear.adsk.core, "CommandCreatedEventArgs", MagicMock(cast=lambda a: MagicMock(command=mock_cmd)), create=True), \
                 patch.object(PostClear.adsk.cam.CAM, "cast", return_value=cam):
                PostClear.CommandEventHandler().notify(MagicMock())

            empty_programs.createInput.assert_not_called()
            empty_programs.add.assert_not_called()
            mock_app.userInterface.messageBox.assert_not_called()
            self.assertFalse(input_store["ncprog_select_all"].isEnabled)
            self.assertTrue(input_store["error"].isVisible)
            self.assertEqual(input_store["error"].text, "No NC Programs found. Create an NC Program first.")

            val_args = types.SimpleNamespace(inputs=mock_inputs, areInputsValid=True)
            input_store["error"].text = ""
            input_store["error"].isVisible = False
            with patch.object(PostClear.adsk.core.Application, "get", return_value=mock_app), \
                 patch.object(PostClear.adsk.core, "ValidateInputsEventArgs", MagicMock(cast=lambda a: val_args), create=True):
                PostClear.CommandValidateInputsHandler().notify(MagicMock())

            self.assertFalse(val_args.areInputsValid)
            self.assertTrue(input_store["error"].isVisible)
            self.assertEqual(input_store["error"].text, "No NC Programs found. Create an NC Program first.")
        finally:
            PostClear.handlers[:] = saved_handlers
            PostClear.settingsMgr = saved_settings_mgr

    def test_compute_toast_bottom_right_single_origin_and_fallbacks(self):
        saved_point2d = getattr(PostClear.adsk.core, "Point2D", None)
        PostClear.adsk.core.Point2D = types.SimpleNamespace(
            create=lambda x, y: types.SimpleNamespace(x=x, y=y)
        )
        try:
            queried_pts = []

            def make_vp(w, h, ox, oy):
                def _vts(pt):
                    queried_pts.append((pt.x, pt.y))
                    return types.SimpleNamespace(x=ox, y=oy)

                return types.SimpleNamespace(width=w, height=h, viewToScreen=_vts)

            # (a) Retina Mac / standard viewport: calls viewToScreen ONLY with (0, 0)
            vp_std = make_vp(1410, 740, 60.0, 148.0)
            self.assertEqual(
                PostClear._ComputeToastBottomRight(500, 220, vp=vp_std),
                (60 + 1410 - 500 - 20, 148 + 740 - 220 - 24),
            )
            self.assertEqual(queried_pts, [(0, 0)])

            # (b) Left-side secondary monitor preserves negative coordinates
            vp_neg = make_vp(1800, 900, -1860.0, 30.0)
            self.assertEqual(
                PostClear._ComputeToastBottomRight(500, 220, vp=vp_neg),
                (-1860 + 1800 - 500 - 20, 30 + 900 - 220 - 24),
            )
            self.assertEqual(queried_pts, [(0, 0), (0, 0)])

            # (c) Clamps relative to (ox + 12, oy + 12) when viewport is tiny (still preserves negative monitor coords)
            vp_tiny = make_vp(200, 100, -1000.0, -400.0)
            self.assertEqual(
                PostClear._ComputeToastBottomRight(500, 220, vp=vp_tiny),
                (-1000 + 12, -400 + 12),
            )

            # (d) Fallback (940, 680) when vp is None or origin is None
            self.assertEqual(PostClear._ComputeToastBottomRight(500, 220, vp=None), (940, 680))
            vp_none_origin = types.SimpleNamespace(width=1400, height=800, viewToScreen=lambda _pt: None)
            self.assertEqual(PostClear._ComputeToastBottomRight(500, 220, vp=vp_none_origin), (940, 680))
            self.assertFalse(hasattr(PostClear, "_cached_screen_metrics"))
            self.assertFalse(hasattr(PostClear, "_last_toast_size"))

            html_str = PostClear._BuildNotificationHTML("Done", "success", "1 file", "", [])
            self.assertNotIn("screen_metrics", html_str)
            self.assertNotIn("reportScreenMetrics", html_str)
        finally:
            if saved_point2d is None and hasattr(PostClear.adsk.core, "Point2D"):
                delattr(PostClear.adsk.core, "Point2D")
            else:
                PostClear.adsk.core.Point2D = saved_point2d

    def test_oillid_dialog_table_visibility_header_count_warn_and_abort_no_save(self):
        """Reproduce 9-setup/10-NCProgram OilLid dialog, live header counter, ⚠ prefix, and abort SaveSettings behavior."""
        input_store = {}
        table_rows = {}

        def _make_bool(inp_id, name, is_cb, res="", val=False):
            obj = types.SimpleNamespace(id=inp_id, name=name, value=val, isEnabled=True, tooltip="", resourceFolder="")
            input_store[inp_id] = obj
            return obj

        def _make_str(inp_id, name, val):
            obj = types.SimpleNamespace(id=inp_id, name=name, value=val, isReadOnly=False)
            input_store[inp_id] = obj
            return obj

        def _make_text(inp_id, name, text, rows, read_only):
            obj = types.SimpleNamespace(id=inp_id, name=name, text=text, isFullWidth=False, isVisible=False)
            input_store[inp_id] = obj
            return obj

        def _make_group(inp_id, name):
            grp_children = types.SimpleNamespace(
                addBoolValueInput=lambda iid, n, cb, res, v: _make_bool(iid, n, cb, res, v),
                itemById=lambda iid: input_store.get(iid),
            )
            obj = types.SimpleNamespace(id=inp_id, name=name, isExpanded=True, children=grp_children)
            input_store[inp_id] = obj
            return obj

        def _make_table(inp_id, name, cols, col_ratios):
            tbl = _MockTableCommandInput(inp_id, name)
            input_store[inp_id] = tbl
            return tbl

        mock_inputs = types.SimpleNamespace(
            addTableCommandInput=_make_table,
            addBoolValueInput=_make_bool,
            addStringValueInput=_make_str,
            addTextBoxCommandInput=_make_text,
            addGroupCommandInput=_make_group,
            itemById=lambda iid: input_store.get(iid),
        )
        added_changed_handlers = []
        mock_cmd = types.SimpleNamespace(
            commandInputs=mock_inputs,
            execute=types.SimpleNamespace(add=lambda h: None),
            inputChanged=types.SimpleNamespace(add=lambda h: added_changed_handlers.append(h)),
            validateInputs=types.SimpleNamespace(add=lambda h: None),
        )

        def _mk_op(name, op_id, has_tp=True, is_suppressed=False):
            return types.SimpleNamespace(
                name=name,
                operationId=op_id,
                entityToken=f"op-{op_id}",
                objectType="adsk::cam::Operation",
                hasToolpath=has_tp,
                isToolpathValid=has_tp,
                isSuppressed=is_suppressed,
                isSelected=False,
                parent=None,
                tool=types.SimpleNamespace(parameters=types.SimpleNamespace(itemByName=lambda _k: None)),
            )

        def _mk_setup(name, tok, ops, is_suppressed=False):
            s = types.SimpleNamespace(
                name=name,
                entityToken=tok,
                objectType="adsk::cam::Setup",
                operationType=0,
                isSuppressed=is_suppressed,
                isSelected=False,
                allOperations=list(ops),
            )
            for op in ops:
                op.parent = s
                op.parentSetup = s
            return s

        flip_op = _mk_op("Face1", 101, has_tp=False)
        setup_flip = _mk_setup("OilLidFlip", "set-flip", [flip_op])
        rs1_op = _mk_op("Face1", 201, has_tp=True)
        setup_rs1 = _mk_setup("RoundStock1", "set-rs1", [rs1_op])
        other_setups = [
            _mk_setup(f"Setup{i}", f"set-{i}", [_mk_op(f"Op{i}", 300 + i, has_tp=True)])
            for i in range(2, 9)
        ]
        all_setups = [setup_flip, setup_rs1] + other_setups  # 9 setups

        def _mk_prog(name, setups_list, is_selected=False):
            return types.SimpleNamespace(
                name=name,
                entityToken=f"prog-{name}",
                objectType="adsk::cam::NCProgram",
                isSelected=is_selected,
                setups=list(setups_list),
                operations=[],
                postConfiguration=types.SimpleNamespace(extension=".nc"),
                parameters=types.SimpleNamespace(itemByName=lambda _k: None),
            )

        prog_rs1 = _mk_prog("RoundStock1", [setup_rs1], is_selected=True)
        prog_empty = _mk_prog("EmptyProg", [], is_selected=False)
        prog_nc10 = _mk_prog("NCProgram10", [setup_flip], is_selected=False)
        other_progs = [_mk_prog(f"Prog{i}", [other_setups[i - 2]]) for i in range(2, 9)]  # 7 programs
        all_progs = [prog_rs1] + other_progs + [prog_empty, prog_nc10]  # 10 NCPrograms

        cam = types.SimpleNamespace(
            setups=all_setups,
            ncPrograms=all_progs,
            allOperations=types.SimpleNamespace(count=sum(len(s.allOperations) for s in all_setups)),
        )
        # Stale saved settings included NCProgram10 from an earlier session
        doc_settings = dict(PostClear.defaultSettings)
        doc_settings["ncPrograms"] = ["RoundStock1", "NCProgram10"]

        mock_ui = MagicMock()
        mock_ui.activeSelections = []
        mock_ui.createProgressDialog.return_value.wasCancelled = False
        mock_doc = types.SimpleNamespace(
            products=types.SimpleNamespace(itemByProductType=lambda _id: cam),
            attributes=MagicMock(),
        )
        mock_app = types.SimpleNamespace(userInterface=mock_ui, activeDocument=mock_doc)

        saved_handlers = list(PostClear.handlers)
        saved_settings_mgr = PostClear.settingsMgr
        saved_pending = PostClear._pending_context_selection
        try:
            PostClear._pending_context_selection = None
            mock_mgr = MagicMock()
            mock_mgr.GetSettings.return_value = doc_settings
            PostClear.settingsMgr = mock_mgr

            # Verify _MockTableCommandInput enforces Fusion's setter ordering invariant
            probe_tbl = _MockTableCommandInput("probe", "Probe")
            with self.assertRaises(RuntimeError) as cm_min:
                probe_tbl.minimumVisibleRows = 8
            self.assertIn("3 : invalid value", str(cm_min.exception))
            with self.assertRaises(RuntimeError) as cm_max:
                probe_tbl.maximumVisibleRows = 1
            self.assertIn("3 : invalid value", str(cm_max.exception))

            with patch.object(PostClear.adsk.core.Application, "get", return_value=mock_app), \
                 patch.object(PostClear.adsk.core, "CommandCreatedEventArgs", MagicMock(cast=lambda a: MagicMock(command=mock_cmd)), create=True), \
                 patch.object(PostClear.adsk.cam.CAM, "cast", return_value=cam):
                PostClear.CommandEventHandler().notify(MagicMock())

            # 1. Table shows up to 16 rows, only RoundStock1 is checked, header count is "1 of 10 selected",
            #    and NCProgram10 (index 9) has "⚠ " prefix
            tbl = input_store["ncProgramsTable"]
            self.assertEqual(tbl.maximumVisibleRows, 16)
            self.assertEqual(tbl.minimumVisibleRows, 8)
            self.assertTrue(input_store["ncprog_chk_0"].value)
            self.assertFalse(input_store["ncprog_chk_9"].value)
            self.assertEqual(input_store["ncprog_hdr_info"].value, "1 of 10 selected")
            self.assertFalse(input_store["ncprog_info_0"].value.startswith("⚠"))
            self.assertTrue(input_store["ncprog_info_9"].value.startswith("⚠ "))

            # Also verify 1-program dialog against strict _MockTableCommandInput
            one_prog_store = {}
            one_mock_inputs = types.SimpleNamespace(
                addTableCommandInput=lambda iid, n, c, cr: one_prog_store.setdefault(iid, _MockTableCommandInput(iid, n)),
                addBoolValueInput=lambda iid, n, cb, res="", v=False: one_prog_store.setdefault(
                    iid, types.SimpleNamespace(id=iid, name=n, value=v, isEnabled=True, tooltip="", resourceFolder="")
                ),
                addStringValueInput=lambda iid, n, v: one_prog_store.setdefault(
                    iid, types.SimpleNamespace(id=iid, name=n, value=v, isReadOnly=False)
                ),
                addTextBoxCommandInput=lambda iid, n, txt, r, ro: one_prog_store.setdefault(
                    iid, types.SimpleNamespace(id=iid, name=n, text=txt, isFullWidth=False, isVisible=False)
                ),
                addGroupCommandInput=lambda iid, n: one_prog_store.setdefault(
                    iid,
                    types.SimpleNamespace(
                        id=iid,
                        name=n,
                        isExpanded=True,
                        children=types.SimpleNamespace(
                            addBoolValueInput=lambda ciid, cn, ccb, cres="", cv=False: one_prog_store.setdefault(
                                ciid, types.SimpleNamespace(id=ciid, name=cn, value=cv, isEnabled=True, tooltip="", resourceFolder="")
                            ),
                            itemById=lambda ciid: one_prog_store.get(ciid),
                        ),
                    ),
                ),
                itemById=lambda iid: one_prog_store.get(iid),
            )
            one_cmd = types.SimpleNamespace(
                commandInputs=one_mock_inputs,
                execute=types.SimpleNamespace(add=lambda h: None),
                inputChanged=types.SimpleNamespace(add=lambda h: None),
                validateInputs=types.SimpleNamespace(add=lambda h: None),
            )
            one_cam = types.SimpleNamespace(setups=[setup_rs1], ncPrograms=[prog_rs1])
            one_doc = types.SimpleNamespace(
                products=types.SimpleNamespace(itemByProductType=lambda _id: one_cam),
                attributes=MagicMock(),
            )
            one_app = types.SimpleNamespace(userInterface=mock_ui, activeDocument=one_doc)
            with patch.object(PostClear.adsk.core.Application, "get", return_value=one_app), \
                 patch.object(PostClear.adsk.core, "CommandCreatedEventArgs", MagicMock(cast=lambda a: MagicMock(command=one_cmd)), create=True), \
                 patch.object(PostClear.adsk.cam.CAM, "cast", return_value=one_cam):
                PostClear.CommandEventHandler().notify(MagicMock())
            self.assertEqual(one_prog_store["ncProgramsTable"].maximumVisibleRows, 16)
            self.assertEqual(one_prog_store["ncProgramsTable"].minimumVisibleRows, 2)
            self.assertEqual(one_prog_store["ncprog_hdr_info"].value, "1 of 1 selected")

            # 2. Live CommandInputChangedHandler updates ncprog_hdr_info on Select All & row checkbox toggles
            changed_handler = added_changed_handlers[0]
            input_store["ncprog_select_all"].value = True
            with patch.object(PostClear.adsk.core.Application, "get", return_value=mock_app), \
                 patch.object(
                     PostClear.adsk.core,
                     "InputChangedEventArgs",
                     MagicMock(cast=lambda a: types.SimpleNamespace(input=input_store["ncprog_select_all"], inputs=mock_inputs)),
                     create=True,
                 ):
                changed_handler.notify(MagicMock())
            self.assertEqual(input_store["ncprog_hdr_info"].value, "10 of 10 selected")

            input_store["ncprog_chk_0"].value = False
            with patch.object(PostClear.adsk.core.Application, "get", return_value=mock_app), \
                 patch.object(
                     PostClear.adsk.core,
                     "InputChangedEventArgs",
                     MagicMock(cast=lambda a: types.SimpleNamespace(input=input_store["ncprog_chk_0"], inputs=mock_inputs)),
                     create=True,
                 ):
                changed_handler.notify(MagicMock())
            self.assertEqual(input_store["ncprog_hdr_info"].value, "9 of 10 selected")

            # 3. PerformPostProcess on ['RoundStock1'] emits zero OilLidFlip warnings and saves settings
            with tempfile.TemporaryDirectory() as out_dir:
                for p in all_progs:
                    p.parameters = types.SimpleNamespace(
                        itemByName=lambda k: types.SimpleNamespace(
                            value=types.SimpleNamespace(value=out_dir)
                        ) if k == "nc_program_output_folder" else None
                    )
                with patch.object(PostClear.adsk.core.Application, "get", return_value=mock_app), \
                     patch.object(PostClear.adsk.cam.CAM, "cast", return_value=cam), \
                     patch("PostClear._EnsureLocalModulesLoaded", return_value=(True, "")), \
                     patch("PostClear._NotifyPostProcessResult"), \
                     patch("PostClear.PostProcessSetup", return_value=None) as mock_pp:
                    mock_ui.messageBox.reset_mock()
                    mock_mgr.SaveSettings.reset_mock()
                    PostClear.PerformPostProcess({"ncPrograms": ["RoundStock1"], "restore_rapid": False, "debug_log": False})
                    mock_ui.messageBox.assert_not_called()
                    mock_mgr.SaveSettings.assert_called_once()
                    self.assertEqual(mock_pp.call_count, 1)
                    self.assertEqual(mock_pp.call_args[0][1].name, "RoundStock1")

                # 4. PerformPostProcess on ['RoundStock1', 'NCProgram10'] prompts for OilLidFlip empty op;
                #    aborting on DialogNo does NOT call SaveSettings
                mock_ui.messageBox.reset_mock()
                mock_ui.messageBox.return_value = PostClear.adsk.core.DialogResults.DialogNo
                mock_mgr.SaveSettings.reset_mock()
                with patch.object(PostClear.adsk.core.Application, "get", return_value=mock_app), \
                     patch.object(PostClear.adsk.cam.CAM, "cast", return_value=cam), \
                     patch("PostClear._EnsureLocalModulesLoaded", return_value=(True, "")), \
                     patch("PostClear.PostProcessSetup") as mock_pp_abort:
                    PostClear.PerformPostProcess({"ncPrograms": ["RoundStock1", "NCProgram10"], "restore_rapid": False, "debug_log": False})
                    mock_ui.messageBox.assert_called_once()
                    self.assertIn("OilLidFlip", mock_ui.messageBox.call_args[0][0])
                    mock_mgr.SaveSettings.assert_not_called()
                    mock_pp_abort.assert_not_called()

                # 5. When OilLidFlip is suppressed, ['RoundStock1', 'NCProgram10'] triggers ZERO pre-validation prompts
                setup_flip.isSuppressed = True
                mock_ui.messageBox.reset_mock()
                mock_mgr.SaveSettings.reset_mock()
                with patch.object(PostClear.adsk.core.Application, "get", return_value=mock_app), \
                     patch.object(PostClear.adsk.cam.CAM, "cast", return_value=cam), \
                     patch("PostClear._EnsureLocalModulesLoaded", return_value=(True, "")), \
                     patch("PostClear._NotifyPostProcessResult"), \
                     patch("PostClear.PostProcessSetup", return_value=None) as mock_pp_sup:
                    PostClear.PerformPostProcess({"ncPrograms": ["RoundStock1", "NCProgram10"], "restore_rapid": False, "debug_log": False})
                    mock_ui.messageBox.assert_not_called()
                    mock_mgr.SaveSettings.assert_called_once()
                    self.assertEqual(mock_pp_sup.call_count, 1)
        finally:
            PostClear.handlers[:] = saved_handlers
            PostClear.settingsMgr = saved_settings_mgr
            PostClear._pending_context_selection = saved_pending


if __name__ == "__main__":
    unittest.main()
