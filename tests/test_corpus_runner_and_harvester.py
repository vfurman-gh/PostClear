#!/usr/bin/env python3
"""
Tests for Top-10 Hobbyist Corpus Runner and In-Fusion Corpus Harvester.
"""

import json
import os
import shutil
import sys
import tempfile
import types
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)


def _ensure_adsk_stubs():
    """Populate missing `adsk` stubs without overwriting existing test stubs."""
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

    if not hasattr(adsk, "doEvents"):
        adsk.doEvents = lambda: None

    cam = adsk.cam
    if not hasattr(cam, "OperationTypes"):
        cam.OperationTypes = types.SimpleNamespace(MillingOperation=0, TurningOperation=1)
    if not hasattr(cam, "SetupStockModes"):
        cam.SetupStockModes = types.SimpleNamespace(
            FixedBoxStock=0,
            RelativeBoxStock=1,
            FixedCylinderStock=2,
            RelativeCylinderStock=3,
            FixedTubeStock=4,
            RelativeTubeStock=5,
            SolidStock=6,
            PreviousSetupStock=7,
        )
    if not hasattr(cam, "PostOutputUnitOptions"):
        cam.PostOutputUnitOptions = types.SimpleNamespace(
            DocumentUnitsOutput=0,
            InchesOutput=1,
            MillimetersOutput=2,
        )
    if not hasattr(cam, "CAM"):
        cam.CAM = types.SimpleNamespace(cast=lambda x: x)
    if not hasattr(cam, "PostProcessInput"):
        class _MockPostProcessInput:
            def __init__(self, program_name, post_config, output_folder, output_units):
                self.programName = program_name
                self.postConfiguration = post_config
                self.outputFolder = output_folder
                self.outputUnits = output_units
                self.isOpenInEditor = False

            @classmethod
            def create(cls, program_name, post_config, output_folder, output_units):
                return cls(program_name, post_config, output_folder, output_units)

        cam.PostProcessInput = _MockPostProcessInput

    core = adsk.core
    for h_name in (
        "CommandCreatedEventHandler",
        "InputChangedEventHandler",
        "ValidateInputsEventHandler",
        "CommandEventHandler",
        "HTMLEventHandler",
        "NavigationEventHandler",
    ):
        if not hasattr(core, h_name):
            setattr(core, h_name, type(h_name, (), {}))
    if not hasattr(core, "Application"):
        core.Application = types.SimpleNamespace(get=lambda: types.SimpleNamespace(log=lambda msg: None))
    if not hasattr(core, "ObjectCollection"):
        class _MockObjectCollection(list):
            @classmethod
            def create(cls):
                return cls()

            def add(self, item):
                self.append(item)

        core.ObjectCollection = _MockObjectCollection
    if not hasattr(core, "DialogResults"):
        core.DialogResults = types.SimpleNamespace(DialogYes=0, DialogNo=1, DialogOK=0)

    fusion = adsk.fusion
    if not hasattr(fusion, "TriangleMeshQualityOptions"):
        fusion.TriangleMeshQualityOptions = types.SimpleNamespace(
            LowQualityTriangleMesh=8,
            NormalQualityTriangleMesh=11,
            HighQualityTriangleMesh=13,
            VeryHighQualityTriangleMesh=15,
        )


_ensure_adsk_stubs()

from geometry.models import (
    BoxStock3D,
    MillToolGeometry,
    MotionKind,
    MotionSegment,
    SolidMeshStock3D,
    ToolAssembly,
    Vector3D,
    VectorKey,
    VerificationReport,
    WorkpieceDefinition,
)
from geometry.solid_mesh import load_stl_mesh
from scripts import harvest_corpus_fusion, run_corpus


class TestCorpusRunnerTop10AndOracles(unittest.TestCase):
    def test_top10_hobbyist_posts_resolve_cleanly(self):
        posts = run_corpus.resolve_hobbyist_posts()
        self.assertEqual(len(posts["milling"]), 8)
        self.assertEqual(len(posts["turning"]), 2)
        milling_names = [lbl for lbl, _ in posts["milling"]]
        for expected in (
            "grbl.cps",
            "carbide3d.cps",
            "mach3mill.cps",
            "linuxcnc.cps",
            "shopbot.cps",
            "tormach.cps",
            "centroid.cps",
            "fanuc.cps",
        ):
            self.assertIn(expected, milling_names)
        for dom, items in posts.items():
            for lbl, ppath in items:
                self.assertTrue(os.path.isfile(ppath), f"Missing resolved {dom} post {lbl}: {ppath}")

    def test_end_to_end_matrix_on_builtin_fixtures_across_10_posts(self):
        fixtures = run_corpus.discover_fixtures(corpus_dir=None, include_builtin_fixtures=True)
        names = [fx.name for fx in fixtures]
        self.assertIn("milling_drill_test.cnc", names)
        self.assertIn("roughing_test.cnc", names)

        with tempfile.TemporaryDirectory() as td:
            progress_p = os.path.join(td, "corpus_progress.json")
            results = run_corpus.run_corpus_matrix(
                fixtures=fixtures,
                workers=4,
                run_direct=True,
                run_metamorphic=True,
                progress_file=progress_p,
            )
            summary = results["summary"]
            self.assertTrue(
                summary["all_passed"],
                f"Corpus matrix failed: {json.dumps(results['deduplicated_failures'], indent=2)}",
            )
            self.assertEqual(summary["matrix_runs"], 10)
            self.assertEqual(summary["matrix_passed"], 10)
            self.assertEqual(summary["fixtures_tested"], 2)
            self.assertEqual(summary["fixtures_passed"], 2)

            self.assertTrue(os.path.isfile(progress_p))
            self.assertFalse(os.path.exists(os.path.join(td, ".active_workers")))
            with open(progress_p, "r", encoding="utf-8") as pf:
                prog = json.load(pf)
            self.assertEqual(prog["status"], "completed")
            self.assertEqual(prog["fixtures_total"], 2)
            self.assertEqual(prog["fixtures_completed"], 2)
            self.assertEqual(prog["completed_matrix_runs"], 10)
            self.assertEqual(prog["completed_matrix_passed"], 10)

            md_path, json_path = run_corpus.write_corpus_reports(results, td)
            self.assertTrue(os.path.isfile(md_path))
            self.assertTrue(os.path.isfile(json_path))
            with open(md_path, "r", encoding="utf-8") as mf:
                md_text = mf.read()
            self.assertIn("Overall status: **PASS**", md_text)
            self.assertIn("grbl.cps", md_text)
            self.assertIn("fanuc turning.cps", md_text)

    def test_normalize_and_oracle_failure_detectors(self):
        a = "N10 (2026-11-13 18:01:02)\nN20 G1 X10.5 Y20.0 F500.\n\n"
        b = "N10 (2026-11-14 09:59:59)\nN20 G1 X10.5 Y20.0 F500.\n"
        self.assertEqual(run_corpus.normalize_nc_lines(a), run_corpus.normalize_nc_lines(b))

        ok1, _ = run_corpus.check_oracle1_direct_parity(a, b)
        self.assertTrue(ok1)

        bad_coord = "N10 (2026-11-14 09:59:59)\nN20 G1 X10.6 Y20.0 F500.\n"
        ok1_bad, detail = run_corpus.check_oracle1_direct_parity(a, bad_coord)
        self.assertFalse(ok1_bad)
        self.assertIn("X10.6", detail)

        # Fail-closed guard catches leaked promotion when dog_leg_warning is set
        rep_dog = types.SimpleNamespace(
            dog_leg_warning="highFeedMapping=1",
            tilted_ops=[],
            multi_axis_ops=[],
            frame_mismatch_ops=[],
            operation_names=["Op1"],
            promoted_events={(0, 3)},
        )
        ok_dog, _ = run_corpus.check_oracle2_nc_diff(a, a, rep_dog)
        self.assertFalse(ok_dog)

        # Coordinate/cycle guard catches canned cycle mutation
        rep_prom = types.SimpleNamespace(
            dog_leg_warning=None,
            tilted_ops=[],
            multi_axis_ops=[],
            frame_mismatch_ops=[],
            operation_names=["Op1"],
            promoted_events={(0, 2)},
        )
        ok_cyc, cyc_msg = run_corpus.check_oracle2_nc_diff(
            "N10 G81 X1.0 Y2.0 Z-5.0 R1.0 F100.\n",
            "N10 G0 X1.0 Y2.0 Z-5.0\n",
            rep_prom,
        )
        self.assertFalse(ok_cyc)
        self.assertIn("Canned cycle", cyc_msg)

    def test_noop_verifier_returns_empty_report_with_operation_names(self):
        vk0 = VectorKey.from_floats(0, Vector3D(0, 0, 10), Vector3D(10, 0, 10))
        vk1 = VectorKey.from_floats(1, Vector3D(10, 0, 10), Vector3D(20, 0, 10))
        segs = [
            MotionSegment(
                segment_id=0,
                op_id="OpA",
                sec_id=0,
                motion=MotionKind.LINEAR,
                start=Vector3D(0, 0, 10),
                end=Vector3D(10, 0, 10),
                feedrate_mmpm=500.0,
                spindle_rpm=10000.0,
                duration_sec=1.2,
                vector_key=vk0,
            ),
            MotionSegment(
                segment_id=1,
                op_id="OpA",
                sec_id=0,
                motion=MotionKind.LINEAR,
                start=Vector3D(10, 0, 10),
                end=Vector3D(15, 0, 10),
                feedrate_mmpm=500.0,
                spindle_rpm=10000.0,
                duration_sec=0.6,
                vector_key=vk0,
            ),
            MotionSegment(
                segment_id=2,
                op_id="OpB",
                sec_id=1,
                motion=MotionKind.RAPID,
                start=Vector3D(15, 0, 10),
                end=Vector3D(20, 0, 10),
                feedrate_mmpm=5000.0,
                spindle_rpm=10000.0,
                duration_sec=0.1,
                vector_key=vk1,
            ),
        ]
        v = run_corpus._NoOpVerifier()
        rep = v.verify_toolpath(segs, {}, None)
        self.assertEqual(rep.total_machining_time_sec, 0.0)
        self.assertEqual(rep.total_volume_removed_mm3, 0.0)
        self.assertEqual(rep.collisions, [])
        self.assertEqual(rep.promoted_events, set())
        self.assertEqual(rep.operation_names, ["OpA", "OpB"])
        self.assertEqual(v.holder_unchecked_ops, set())

    def test_memoized_verifier_caches_and_isolates_reports(self):
        wp = WorkpieceDefinition(stock=BoxStock3D(0.0, 50.0, 0.0, 50.0, -10.0, 0.0))
        tools = {
            "Op1": ToolAssembly(
                tool_id=1,
                name="1/4 Flat",
                geometry=MillToolGeometry(
                    diameter=6.35,
                    corner_radius=0.0,
                    flute_length=19.0,
                    overall_length=50.0,
                ),
            )
        }
        vk = VectorKey.from_floats(0, Vector3D(0, 0, 5), Vector3D(20, 0, 5))
        seg = MotionSegment(
            segment_id=0,
            op_id="Op1",
            sec_id=0,
            motion=MotionKind.LINEAR,
            start=Vector3D(0, 0, 5),
            end=Vector3D(20, 0, 5),
            feedrate_mmpm=600.0,
            spindle_rpm=12000.0,
            duration_sec=2.0,
            vector_key=vk,
            metadata={"sec_idx": 0, "event_idx": 1},
        )

        class _StubRealVerifier:
            def __init__(self):
                self.holder_unchecked_ops = {"Op1"}

            def verify_toolpath(self, segments, tools_by_op, workpiece):
                return VerificationReport(
                    total_machining_time_sec=2.0,
                    total_volume_removed_mm3=123.4,
                    collisions=[],
                    promoted_events={(0, 1)},
                    operation_names=["Op1"],
                    holder_unchecked_ops=["Op1"],
                )

        with patch("scripts.run_corpus._new_verifier", side_effect=lambda is_t: _StubRealVerifier()) as mock_new:
            mv = run_corpus._MemoizedVerifier(is_turning=False)
            rep1 = mv.verify_toolpath([seg], tools, wp)
            self.assertEqual(mock_new.call_count, 1)
            self.assertEqual(mv.cache_misses, 1)
            self.assertEqual(mv.cache_hits, 0)
            self.assertEqual(mv.holder_unchecked_ops, {"Op1"})

            # Simulate downstream orchestrator mutations on rep1
            rep1.promoted_events.clear()
            rep1.dog_leg_warning = "mutated_by_post_A"
            rep1.multi_axis_ops.append("Op1")
            mv.holder_unchecked_ops.clear()

            # Second identical call must hit cache and remain unaffected by mutations
            rep2 = mv.verify_toolpath([seg], tools, wp)
            self.assertEqual(mock_new.call_count, 1)
            self.assertEqual(mv.cache_misses, 1)
            self.assertEqual(mv.cache_hits, 1)
            self.assertEqual(mv.holder_unchecked_ops, {"Op1"})
            self.assertEqual(rep2.promoted_events, {(0, 1)})
            self.assertIsNone(rep2.dog_leg_warning)
            self.assertEqual(rep2.multi_axis_ops, [])

            # Different segment coordinate must miss cache cleanly
            seg_diff = MotionSegment(
                segment_id=0,
                op_id="Op1",
                sec_id=0,
                motion=MotionKind.LINEAR,
                start=Vector3D(0, 0, 5),
                end=Vector3D(25, 0, 5),
                feedrate_mmpm=600.0,
                spindle_rpm=12000.0,
                duration_sec=2.5,
                vector_key=vk,
                metadata={"sec_idx": 0, "event_idx": 1},
            )
            rep3 = mv.verify_toolpath([seg_diff], tools, wp)
            self.assertEqual(mock_new.call_count, 2)
            self.assertEqual(mv.cache_misses, 2)
            self.assertEqual(rep3.total_volume_removed_mm3, 123.4)

    def test_unsupported_by_post_classified_separately_from_failures(self):
        with tempfile.TemporaryDirectory() as td:
            cnc_p = os.path.join(td, "rotary.cnc")
            cps_p = os.path.join(td, "grbl_stub.cps")
            with open(cnc_p, "w", encoding="utf-8") as f:
                f.write("@\n")
            with open(cps_p, "w", encoding="utf-8") as f:
                f.write("description = 'stub';\n")

            fx = run_corpus.CorpusFixture(
                name="rotary_fixture",
                source_path=cnc_p,
                cnc_batches=[cnc_p],
                is_turning=False,
            )

            with (
                patch(
                    "pipeline.PipelineOrchestrator.dump_intermediate_to_json",
                    side_effect=RuntimeError("Post error during replay"),
                ),
                patch(
                    "scripts.run_corpus.run_direct_post_with_shim",
                    side_effect=RuntimeError(
                        "Direct post binary exited with rc=244: Tool orientation is not supported"
                    ),
                ),
            ):
                results = run_corpus.run_corpus_matrix(
                    fixtures=[fx],
                    posts_by_domain={"milling": [("grbl.cps", cps_p)], "turning": []},
                    workers=1,
                    run_direct=True,
                    run_metamorphic=False,
                )

        summary = results["summary"]
        self.assertEqual(summary["matrix_runs"], 1)
        self.assertEqual(summary["matrix_passed"], 0)
        self.assertEqual(summary["matrix_unsupported"], 1)
        self.assertEqual(summary["matrix_failed"], 0)
        self.assertEqual(summary["unique_failures"], 0)
        self.assertTrue(summary["all_passed"])

        cell = results["cell_results"][0]
        self.assertEqual(cell["status"], "unsupported_by_post")
        self.assertTrue(cell["unsupported_by_post"])
        self.assertIn("Tool orientation is not supported", cell["unsupported_detail"])
        self.assertEqual(
            results["fixture_checks"][0]["oracle2_conservation_detail"],
            "skipped (unsupported by all posts)",
        )
        report_md = run_corpus.format_corpus_report(results)
        self.assertIn("Natively Unsupported by Post (Skipped)", report_md)
        self.assertIn("| `grbl.cps` | milling | 1 | 0 | 1 | 0 |", report_md)


class TestHarvestCorpusFusion(unittest.TestCase):
    def test_write_binary_stl_round_trip(self):
        verts = np.array(
            [
                [0.0, 0.0, 0.0],
                [10.0, 0.0, 0.0],
                [10.0, 20.0, 0.0],
                [0.0, 20.0, 5.0],
            ],
            dtype=np.float32,
        )
        indices = np.array([[0, 1, 2], [0, 2, 3]], dtype=np.int32)
        with tempfile.TemporaryDirectory() as td:
            stl_p = os.path.join(td, "solid_stock.stl")
            harvest_corpus_fusion.write_binary_stl(stl_p, verts, indices)
            loaded_v, loaded_idx = load_stl_mesh(stl_p, is_inch=False)
            self.assertEqual(loaded_idx.shape, (2, 3))
            np.testing.assert_allclose(loaded_v.min(axis=0), verts.min(axis=0), atol=1e-5)
            np.testing.assert_allclose(loaded_v.max(axis=0), verts.max(axis=0), atol=1e-5)

    def test_harvest_projects_crawls_skips_cached_and_respects_open_docs(self):
        sample_cnc = os.path.join(REPO_ROOT, "tests", "fixtures", "milling_drill_test.cnc")
        sample_dump = {
            "unit": 1,
            "stockMode": "solid",
            "sections": [
                {
                    "id": 0,
                    "type": 0,
                    "tool": {"number": 1, "diameter": 6.35},
                    "params": {"operation-comment": "Pocket1"},
                    "events": [],
                }
            ],
        }

        posted_names = []

        class _FakeParamVal:
            def __init__(self, v):
                self.value = v

        class _FakeParams:
            def __init__(self, d):
                self._d = d

            def itemByName(self, k):
                return _FakeParamVal(_FakeParamVal(self._d[k])) if k in self._d else None

        class _FakeCAM:
            def __init__(self, setup):
                self.setups = [setup]
                self.ncPrograms = []

            def generateAllToolpaths(self, skip_valid):
                return types.SimpleNamespace(isGenerationCompleted=True)

            def postProcess(self, target_ops, post_input):
                posted_names.append(post_input.programName)
                p_name = post_input.programName
                out_json = os.path.join(post_input.outputFolder, f"{p_name}.json")
                with open(out_json, "w", encoding="utf-8") as jf:
                    json.dump(sample_dump, jf)
                target_cnc = os.environ.get("TARGET_INTERMEDIATE_CNC")
                if target_cnc:
                    shutil.copy2(sample_cnc, target_cnc)
                else:
                    shutil.copy2(sample_cnc, out_json + ".intermediate.cnc")
                return True

        def _make_doc(df_id, name):
            op = types.SimpleNamespace(
                isSuppressed=False,
                hasToolpath=True,
                isToolpathValid=True,
                operationId=11,
                tool=types.SimpleNamespace(parameters=_FakeParams({"tool_number": 1})),
            )
            setup = types.SimpleNamespace(
                name="Setup1",
                isSuppressed=False,
                operationType=0,  # MillingOperation
                stockMode=6,      # SolidStock
                allOperations=[op],
                parameters=_FakeParams({}),
                workCoordinateSystem=None,
            )
            cam = _FakeCAM(setup)
            doc = MagicMock()
            doc.name = name
            doc.dataFile = types.SimpleNamespace(id=df_id)
            doc.products.itemByProductType.side_effect = lambda pt: cam if pt == "CAMProductType" else None
            doc.unitsManager = types.SimpleNamespace(defaultLengthUnits="mm")
            return doc

        df_open = types.SimpleNamespace(id="id-open", name="AlreadyOpen.f3d", fileExtension="f3d", versionNumber=3)
        df_closed = types.SimpleNamespace(id="id-closed", name="ClosedPart.f3z", fileExtension="f3z", versionNumber=2)
        df_drawing = types.SimpleNamespace(id="id-draw", name="PartDrawing.f2d", fileExtension="f2d", versionNumber=1)

        doc_already_open = _make_doc("id-open", "AlreadyOpen")
        doc_newly_opened = _make_doc("id-closed", "ClosedPart")

        root_folder = types.SimpleNamespace(
            name="Root",
            dataFiles=[df_open, df_drawing],
            dataFolders=[types.SimpleNamespace(name="Sub", dataFiles=[df_closed], dataFolders=[])],
        )
        proj = types.SimpleNamespace(name="Workshop", rootFolder=root_folder)

        docs_collection = [doc_already_open]

        class _FakeDocuments(list):
            def open(self, df, visible=True):
                self.opened_ids.append(df.id)
                return doc_newly_opened

        fake_docs = _FakeDocuments(docs_collection)
        fake_docs.opened_ids = []

        app = types.SimpleNamespace(
            documents=fake_docs,
            data=types.SimpleNamespace(
                activeProject=proj,
                activeHub=types.SimpleNamespace(dataProjects=[proj]),
            ),
        )
        prog_dialog = types.SimpleNamespace(isCancelButtonShown=False, wasCancelled=False, message="", progressValue=0, show=lambda *a: None, hide=lambda: None)
        ui = types.SimpleNamespace(createProgressDialog=lambda: prog_dialog)

        cube_verts = np.array(
            [
                [-40.0, -40.0, -20.0], [40.0, -40.0, -20.0], [40.0, 40.0, -20.0], [-40.0, 40.0, -20.0],
                [-40.0, -40.0, 5.0], [40.0, -40.0, 5.0], [40.0, 40.0, 5.0], [-40.0, 40.0, 5.0],
            ],
            dtype=np.float32,
        )
        cube_idx = np.array(
            [
                [0, 1, 2], [0, 2, 3],
                [4, 6, 5], [4, 7, 6],
                [0, 4, 5], [0, 5, 1],
                [1, 5, 6], [1, 6, 2],
                [2, 6, 7], [2, 7, 3],
                [3, 7, 4], [3, 4, 0],
            ],
            dtype=np.int32,
        )
        solid_wp = WorkpieceDefinition(stock=SolidMeshStock3D.from_mesh(cube_verts, cube_idx))

        with tempfile.TemporaryDirectory() as corpus_dir:
            with patch("geometry.solid_mesh.extract_solid_workpiece_from_fusion_setup", return_value=solid_wp):
                stats1 = harvest_corpus_fusion.harvest_projects(
                    app=app,
                    ui=ui,
                    scope_choice="3",
                    corpus_dir=corpus_dir,
                    force=False,
                )

            # Drawing .f2d was skipped; 2 CAD docs processed -> 2 setups exported
            self.assertEqual(stats1["docs_processed"], 2)
            self.assertEqual(stats1["setups_harvested"], 2)
            self.assertEqual(stats1["docs_failed"], 0, f"Unexpected errors: {stats1['errors']}")
            self.assertEqual(fake_docs.opened_ids, ["id-closed"])
            # Pre-opened doc was activated, never closed; newly opened doc was closed(False)
            doc_already_open.close.assert_not_called()
            doc_newly_opened.close.assert_called_once_with(False)
            # Numeric programName ("1001") was used for every PostProcessInput
            self.assertTrue(posted_names and all(n.isdigit() for n in posted_names))

            # Discover harvested bundles via run_corpus and verify solid_stock.stl loaded
            discovered = run_corpus.discover_fixtures(corpus_dir=corpus_dir, include_builtin_fixtures=False)
            self.assertEqual(len(discovered), 2)
            for fx in discovered:
                self.assertIsNotNone(fx.solid_stock_stl)
                self.assertIsInstance(fx.explicit_workpiece.stock, SolidMeshStock3D)

            # Second harvest without force skips both cached doc versions
            with patch("geometry.solid_mesh.extract_solid_workpiece_from_fusion_setup", return_value=solid_wp):
                stats2 = harvest_corpus_fusion.harvest_projects(
                    app=app,
                    ui=ui,
                    scope_choice="3",
                    corpus_dir=corpus_dir,
                    force=False,
                )
            self.assertEqual(stats2["docs_skipped_cached"], 2)
            self.assertEqual(stats2["docs_processed"], 0)

    def test_wrapper_comment_with_colon_allowed_in_oracle2_nc_diff(self):
        self.assertIsNotNone(
            run_corpus._PURE_WRAPPER_COMMENT_RE.match("(UNIVERSAL WRAPPER: Restored 1 G0 rapid moves)")
        )
        self.assertIsNotNone(
            run_corpus._PURE_WRAPPER_COMMENT_RE.match("UNIVERSAL WRAPPER Restored 3 G0 rapid moves")
        )
        self.assertIsNotNone(
            run_corpus._PURE_WRAPPER_COMMENT_RE.match("; UNIVERSAL WRAPPER: Restored 2 G0 rapid moves")
        )
        self.assertIsNotNone(
            run_corpus._PURE_WRAPPER_COMMENT_RE.match("' UNIVERSAL WRAPPER: Restored 1 G0 rapid moves")
        )
        ok, detail = run_corpus.check_oracle2_nc_diff(
            "G1 X10 Y0 Z5 F500\nM30\n",
            "(UNIVERSAL WRAPPER: Restored 1 G0 rapid moves)\nG0 X10 Y0 Z5\nM30\n",
            types.SimpleNamespace(
                promoted_events={(0, 0)},
                tilted_ops=[],
                multi_axis_ops=[],
                frame_mismatch_ops=[],
            ),
            section_count=1,
        )
        self.assertTrue(ok, detail)

    def test_discover_fixtures_skips_incomplete_harvest_folders(self):
        with tempfile.TemporaryDirectory() as corpus_dir:
            incomplete = os.path.join(corpus_dir, "ProjA", "Doc1", "Setup_incomplete")
            os.makedirs(incomplete)
            for fname in ("prev_stock.stl", "solid_stock.stl", "post_setup_ipw.stl", "batch_0.cnc"):
                with open(os.path.join(incomplete, fname), "w", encoding="utf-8") as f:
                    f.write("stub")

            valid = os.path.join(corpus_dir, "ProjA", "Doc1", "Setup_valid")
            os.makedirs(valid)
            cnc_path = os.path.join(valid, "batch_0.cnc")
            with open(cnc_path, "w", encoding="utf-8") as f:
                f.write("@0 section\n")
            with open(os.path.join(valid, "setup_manifest.json"), "w", encoding="utf-8") as f:
                json.dump({"setup_name": "Setup_valid", "cnc_batches": ["batch_0.cnc"]}, f)

            discovered = run_corpus.discover_fixtures(
                corpus_dir=corpus_dir, include_builtin_fixtures=False
            )
            self.assertEqual(len(discovered), 1)
            self.assertIn("Setup_valid", discovered[0].name)

    def test_effective_post_properties_scoped_to_configured_post(self):
        fx = run_corpus.CorpusFixture(
            name="ButtRouter/Top",
            source_path="/tmp/ButtRouter/Top",
            is_turning=False,
            cnc_batches=["batch_0.cnc"],
            configured_post="shopbot.cps",
            post_properties={"useZ": True, "unit": "mm"},
        )
        self.assertEqual(
            run_corpus._effective_post_properties(fx, "/Posts/ShopBot.cps"),
            {"useZ": True, "unit": "mm"},
        )
        self.assertIsNone(run_corpus._effective_post_properties(fx, "/Posts/mach3mill.cps"))

    def test_oracle2_multi_pass_coordinate_alignment(self):
        noprom = (
            "G90\n"
            "G1 X0 Y0 Z5.08 F500\n"
            "G1 X10 Y0 Z-1.0 F300\n"
            "G1 X10 Y0 Z5.08 F500\n"
            "G1 X0 Y0 Z5.08 F500\n"
            "M30\n"
        )
        prom_ok = (
            "G90\n"
            "G0 X0 Y0 Z5.08\n"
            "G1 X10 Y0 Z-1.0 F300\n"
            "G0 X10 Y0 Z5.08\n"
            "G0 X0 Y0 Z5.08\n"
            "M30\n"
        )
        rep_stub = types.SimpleNamespace(
            promoted_events={(0, 1), (0, 3), (0, 4)},
            tilted_ops=[],
            multi_axis_ops=[],
            frame_mismatch_ops=[],
        )
        ok, det = run_corpus.check_oracle2_nc_diff(noprom, prom_ok, rep_stub, section_count=1)
        self.assertTrue(ok, det)

        prom_bad_coord = prom_ok.replace("G0 X10 Y0 Z5.08", "G0 X10 Y1 Z5.08")
        ok_bad, _ = run_corpus.check_oracle2_nc_diff(
            noprom, prom_bad_coord, rep_stub, section_count=1
        )
        self.assertFalse(ok_bad)

    def test_all_sections_blocked_from_dumps_uses_per_section_replay_diag(self):
        with tempfile.TemporaryDirectory() as td:
            dump_path = os.path.join(td, "batch_0.json")
            with open(dump_path, "w", encoding="utf-8") as df:
                json.dump(
                    {
                        "sections": [
                            {
                                "id": 0,
                                "type": 0,
                                "workPlane": {"forward": {"x": 0.0, "y": 0.0, "z": 1.0}},
                                "params": {"operation-comment": "Face7"},
                            },
                            {
                                "id": 1,
                                "type": 0,
                                "workPlane": {"forward": {"x": 0.0, "y": -0.866, "z": -0.5}},
                                "params": {"operation-comment": "Face7"},
                            },
                        ]
                    },
                    df,
                )
            rep_stub = types.SimpleNamespace(frame_mismatch_ops=["Face7"])
            partial_diag = {
                "frameMismatches": [{"secIdx": 1, "id": 1, "op": "Face7"}],
                "multiAxisSections": [],
            }
            self.assertFalse(
                run_corpus._all_sections_blocked_from_dumps(
                    [dump_path], rep_stub, replay_diag=partial_diag
                )
            )
            full_diag = {
                "frameMismatches": [
                    {"secIdx": 0, "id": 0, "op": "Face7"},
                    {"secIdx": 1, "id": 1, "op": "Face7"},
                ],
                "multiAxisSections": [],
            }
            self.assertTrue(
                run_corpus._all_sections_blocked_from_dumps(
                    [dump_path], rep_stub, replay_diag=full_diag
                )
            )

    def test_harvest_setup_previous_setup_stock_writes_manifest_after_temp_cleanup(self):
        sample_cnc = os.path.join(REPO_ROOT, "tests", "fixtures", "milling_drill_test.cnc")
        verts_in = np.array(
            [
                [0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [2.0, 1.0, 0.0], [0.0, 1.0, 0.0],
                [0.0, 0.0, 0.5], [2.0, 0.0, 0.5], [2.0, 1.0, 0.5], [0.0, 1.0, 0.5],
            ],
            dtype=np.float32,
        )
        idxs = np.array(
            [
                [0, 1, 2], [0, 2, 3],
                [4, 6, 5], [4, 7, 6],
                [0, 4, 5], [0, 5, 1],
                [1, 5, 6], [1, 6, 2],
                [2, 6, 7], [2, 7, 3],
                [3, 7, 4], [3, 4, 0],
            ],
            dtype=np.int32,
        )

        class _FakeParamVal:
            def __init__(self, v):
                self.value = v

        class _FakeParams:
            def __init__(self, d):
                self._d = d

            def itemByName(self, k):
                return _FakeParamVal(_FakeParamVal(self._d[k])) if k in self._d else None

        op = types.SimpleNamespace(
            isSuppressed=False,
            hasToolpath=True,
            isToolpathValid=True,
            operationId=21,
            tool=types.SimpleNamespace(parameters=_FakeParams({"tool_number": 1})),
        )
        setup = types.SimpleNamespace(
            name="Flip",
            isSuppressed=False,
            operationType=0,  # MillingOperation
            stockMode=7,      # PreviousSetupStock
            allOperations=[op],
            parameters=_FakeParams({}),
            workCoordinateSystem=None,
        )

        class _FakeCAM:
            def __init__(self):
                self.setups = [setup]
                self.ncPrograms = []

            def postProcess(self, target_ops, post_input):
                p_name = post_input.programName
                temp_stl = os.path.join(post_input.outputFolder, f"{p_name}_stock.stl")
                harvest_corpus_fusion.write_binary_stl(temp_stl, verts_in, idxs)
                dump_payload = {
                    "unit": 0,
                    "stockUnit": "mm",
                    "stockMode": "previous-setup",
                    "stockPath": temp_stl,
                    "stockExport": {"enabled": True, "destPath": temp_stl},
                    "workpieceHint": {
                        "lower": {"x": 0.0, "y": 0.0, "z": 0.0},
                        "upper": {"x": 2.0, "y": 1.0, "z": 0.5},
                    },
                    "sections": [
                        {
                            "id": 0,
                            "type": 0,
                            "tool": {"number": 1, "diameter": 0.25},
                            "params": {"operation-comment": "Pocket1"},
                            "events": [],
                        }
                    ],
                }
                out_json = os.path.join(post_input.outputFolder, f"{p_name}.json")
                with open(out_json, "w", encoding="utf-8") as jf:
                    json.dump(dump_payload, jf)
                target_cnc = os.environ.get("TARGET_INTERMEDIATE_CNC")
                shutil.copy2(sample_cnc, target_cnc or (out_json + ".intermediate.cnc"))
                return True

        cam = _FakeCAM()
        doc = MagicMock()
        doc.unitsManager = types.SimpleNamespace(defaultLengthUnits="mm")

        with tempfile.TemporaryDirectory() as td:
            bundle_dir = os.path.join(td, "Admin Project", "Part", "01_Flip")
            res = harvest_corpus_fusion.harvest_setup(
                doc=doc,
                cam=cam,
                setup=setup,
                setup_idx=1,
                bundle_dir=bundle_dir,
                project_name="Admin Project",
                doc_name="Part",
            )
            self.assertEqual(res["status"], "harvested")
            manifest_path = os.path.join(bundle_dir, "setup_manifest.json")
            self.assertTrue(os.path.isfile(manifest_path))

            with open(manifest_path, "r", encoding="utf-8") as mf:
                manifest = json.load(mf)
            self.assertTrue(manifest["prev_stock_is_inch"])
            self.assertEqual(manifest["prev_stock_stl"], "prev_stock.stl")
            self.assertIsNotNone(manifest["workpiece_mm"])
            self.assertAlmostEqual(manifest["workpiece_mm"]["x_max"], 50.8, places=2)
            self.assertAlmostEqual(manifest["workpiece_mm"]["y_max"], 25.4, places=2)
            self.assertAlmostEqual(manifest["workpiece_mm"]["z_max"], 12.7, places=2)

            with open(os.path.join(bundle_dir, "batch_0.json"), "r", encoding="utf-8") as bf:
                b0 = json.load(bf)
            self.assertEqual(b0["stockPath"], "prev_stock.stl")
            self.assertEqual(b0["stockExport"]["destPath"], "prev_stock.stl")

    def test_recover_incomplete_bundles_recovers_previous_setup_and_rotary(self):
        sample_cnc = os.path.join(REPO_ROOT, "tests", "fixtures", "milling_drill_test.cnc")
        verts_in = np.array(
            [
                [0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [2.0, 1.0, 0.0], [0.0, 1.0, 0.0],
                [0.0, 0.0, 0.5], [2.0, 0.0, 0.5], [2.0, 1.0, 0.5], [0.0, 1.0, 0.5],
            ],
            dtype=np.float32,
        )
        idxs = np.array(
            [
                [0, 1, 2], [0, 2, 3],
                [4, 6, 5], [4, 7, 6],
                [0, 4, 5], [0, 5, 1],
                [1, 5, 6], [1, 6, 2],
                [2, 6, 7], [2, 7, 3],
                [3, 7, 4], [3, 4, 0],
            ],
            dtype=np.int32,
        )

        with tempfile.TemporaryDirectory() as corpus_dir:
            # 1. Existing sibling bundle
            sib_dir = os.path.join(corpus_dir, "Admin Project", "Butt_Plate", "01_Setup1")
            os.makedirs(sib_dir)
            shutil.copy2(sample_cnc, os.path.join(sib_dir, "batch_0.cnc"))
            with open(os.path.join(sib_dir, "setup_manifest.json"), "w", encoding="utf-8") as mf:
                json.dump(
                    {
                        "schema_version": 1,
                        "project_name": "Admin Project",
                        "document_name": "Butt/Plate",
                        "setup_name": "Setup1",
                        "setup_index": 1,
                        "is_turning": False,
                        "stock_mode": "fixed-box",
                        "configured_post": "tormach.cps",
                        "cnc_batches": ["batch_0.cnc"],
                        "json_batches": ["batch_0.json"],
                    },
                    mf,
                )

            # 2. Incomplete prev_stock bundle with stale temp stockPath
            flip_dir = os.path.join(corpus_dir, "Admin Project", "Butt_Plate", "02_Flip")
            os.makedirs(flip_dir)
            shutil.copy2(sample_cnc, os.path.join(flip_dir, "batch_0.cnc"))
            harvest_corpus_fusion.write_binary_stl(os.path.join(flip_dir, "prev_stock.stl"), verts_in, idxs)
            with open(os.path.join(flip_dir, "batch_0.json"), "w", encoding="utf-8") as bf:
                json.dump(
                    {
                        "unit": 0,
                        "stockUnit": "in",
                        "stockMode": "previous-setup",
                        "stockPath": "/tmp/nonexistent/1001_stock.stl",
                        "stockExport": {"destPath": "/tmp/nonexistent/1001_stock.stl"},
                        "postProperties": {"smartCoolEquipped": False},
                        "sections": [
                            {"id": 0, "type": 0, "tool": {"number": 1, "diameter": 0.25}, "events": []}
                        ],
                    },
                    bf,
                )

            # 3. Incomplete fixed-cylinder rotary bundle
            rot_dir = os.path.join(corpus_dir, "Admin Project", "Butt_Front_proj", "05_Rotary")
            os.makedirs(rot_dir)
            shutil.copy2(sample_cnc, os.path.join(rot_dir, "batch_0.cnc"))
            with open(os.path.join(rot_dir, "batch_0.json"), "w", encoding="utf-8") as bf:
                json.dump(
                    {
                        "unit": 0,
                        "stockMode": "fixed-cylinder",
                        "workpiece": {
                            "lower": {"x": -2.0, "y": -1.5, "z": -1.5},
                            "upper": {"x": 0.0, "y": 1.5, "z": 1.5},
                        },
                        "sections": [
                            {"id": 0, "type": 0, "tool": {"number": 2, "diameter": 0.375}, "events": []}
                        ],
                    },
                    bf,
                )

            recovered = harvest_corpus_fusion.recover_incomplete_bundles(corpus_dir)
            self.assertEqual(len(recovered), 2)

            with open(os.path.join(flip_dir, "setup_manifest.json"), "r", encoding="utf-8") as mf:
                flip_m = json.load(mf)
            self.assertEqual(flip_m["document_name"], "Butt/Plate")
            self.assertEqual(flip_m["setup_index"], 2)
            self.assertEqual(flip_m["setup_name"], "Flip")
            self.assertEqual(flip_m["configured_post"], "tormach.cps")
            self.assertTrue(flip_m["prev_stock_is_inch"])
            self.assertAlmostEqual(flip_m["workpiece_mm"]["x_max"], 50.8, places=2)

            with open(os.path.join(flip_dir, "batch_0.json"), "r", encoding="utf-8") as bf:
                flip_b0 = json.load(bf)
            self.assertEqual(flip_b0["stockPath"], "prev_stock.stl")
            self.assertEqual(flip_b0["stockExport"]["destPath"], "prev_stock.stl")

            with open(os.path.join(rot_dir, "setup_manifest.json"), "r", encoding="utf-8") as mf:
                rot_m = json.load(mf)
            self.assertEqual(rot_m["setup_index"], 5)
            self.assertEqual(rot_m["setup_name"], "Rotary")
            self.assertEqual(rot_m["stock_mode"], "fixed-cylinder")
            self.assertAlmostEqual(rot_m["workpiece_mm"]["x_min"], -50.8, places=2)

            discovered = run_corpus.discover_fixtures(corpus_dir=corpus_dir, include_builtin_fixtures=False)
            self.assertEqual(len(discovered), 3)

    def test_harvester_skips_jet_setups_and_probing_ops_and_isolates_setup_errors(self):
        class _FakeParamVal:
            def __init__(self, v):
                self.value = v

        class _FakeParams:
            def __init__(self, d):
                self._d = d

            def itemByName(self, k):
                return _FakeParamVal(_FakeParamVal(self._d[k])) if k in self._d else None

        probe_op = types.SimpleNamespace(
            isSuppressed=False,
            hasToolpath=True,
            isToolpathValid=True,
            strategy="probe",
            parameters=_FakeParams({"operation:isProbingStrategy": 1}),
            tool=types.SimpleNamespace(parameters=_FakeParams({"tool_number": 99, "tool_type": "probe"})),
        )
        mill_op = types.SimpleNamespace(
            isSuppressed=False,
            hasToolpath=True,
            isToolpathValid=True,
            strategy="face",
            parameters=_FakeParams({"operation:isProbingStrategy": 0}),
            tool=types.SimpleNamespace(parameters=_FakeParams({"tool_number": 1, "tool_type": "face mill"})),
        )

        PostClear = harvest_corpus_fusion._get_postclear_module()

        # 1. Mixed Probe + Mill setup keeps only Milling op batch
        mixed_setup = types.SimpleNamespace(
            name="1st op",
            operationType=0,
            allOperations=[probe_op, mill_op],
        )
        batches = harvest_corpus_fusion._partition_tool_batches(mixed_setup, PostClear)
        self.assertEqual(len(batches), 1)
        self.assertEqual(batches[0][0], 1)
        self.assertEqual(batches[0][1], [mill_op])

        # 2. Pure Probing setup returns empty batches -> no_active_ops without creating bundle_dir
        pure_probe_setup = types.SimpleNamespace(
            name="ProbeOnly",
            operationType=0,
            parameters=_FakeParams({"job_stockMode": "default"}),
            allOperations=[probe_op],
        )
        # 3. JetOperation (operationType=2) is skipped_capability without creating bundle_dir
        jet_setup = types.SimpleNamespace(
            name="PlasmaSign",
            operationType=2,
            allOperations=[mill_op],
        )
        # 4. Setup whose post writes 0-byte JSON cleans up partial bundle_dir and does not block next setup
        bad_setup = types.SimpleNamespace(
            name="BadSetup",
            operationType=0,
            isSuppressed=False,
            parameters=_FakeParams({"job_stockMode": "default"}),
            allOperations=[mill_op],
            workCoordinateSystem=None,
        )
        good_setup = types.SimpleNamespace(
            name="GoodSetup",
            operationType=0,
            isSuppressed=False,
            parameters=_FakeParams({"job_stockMode": "default"}),
            allOperations=[mill_op],
            workCoordinateSystem=None,
        )

        sample_cnc = os.path.join(REPO_ROOT, "tests", "fixtures", "milling_drill_test.cnc")

        def _fake_post_process(ops, p_in):
            out_json = os.path.join(p_in.outputFolder, f"{p_in.programName}.json")
            cfg_path = os.environ.get("DUMPER_CONFIG_PATH", "")
            target_cnc = os.environ.get("TARGET_INTERMEDIATE_CNC", "")
            if "00_BadSetup" in target_cnc:
                with open(out_json, "w", encoding="utf-8") as f:
                    f.write("")  # 0-byte failed post output
                with open(os.path.join(p_in.outputFolder, f"{p_in.programName}.failed"), "w", encoding="utf-8") as ff:
                    ff.write("Error: simulated post kernel abort")
                return
            shutil.copy2(sample_cnc, target_cnc)
            with open(out_json, "w", encoding="utf-8") as f:
                json.dump(
                    {
                        "unit": 1,
                        "stockMode": "default",
                        "workpiece": {"lower": {"x": 0, "y": 0, "z": -10}, "upper": {"x": 50, "y": 50, "z": 0}},
                        "sections": [{"id": 0, "type": 0, "tool": {"number": 1, "diameter": 6.0}, "events": []}],
                    },
                    f,
                )
            _ = cfg_path

        cam = types.SimpleNamespace(
            setups=[bad_setup, good_setup, jet_setup, pure_probe_setup],
            ncPrograms=[],
            postProcess=_fake_post_process,
        )
        doc = types.SimpleNamespace(
            name="MultiSetupSample",
            products=types.SimpleNamespace(itemByProductType=lambda pt: cam if pt == "CAMProductType" else None),
            unitsManager=types.SimpleNamespace(defaultLengthUnits="mm"),
        )

        with tempfile.TemporaryDirectory() as corpus_dir:
            jet_bundle = os.path.join(corpus_dir, "jet_bundle")
            jet_res = harvest_corpus_fusion.harvest_setup(doc, cam, jet_setup, 0, jet_bundle)
            self.assertEqual(jet_res["status"], "skipped_capability")
            self.assertFalse(os.path.exists(jet_bundle))

            probe_bundle = os.path.join(corpus_dir, "probe_bundle")
            probe_res = harvest_corpus_fusion.harvest_setup(doc, cam, pure_probe_setup, 0, probe_bundle)
            self.assertEqual(probe_res["status"], "no_active_ops")
            self.assertFalse(os.path.exists(probe_bundle))

            doc_res = harvest_corpus_fusion._harvest_document(
                doc=doc,
                doc_key="open:MultiSetupSample",
                project_name="OpenDocs",
                doc_name="MultiSetupSample",
                corpus_dir=corpus_dir,
            )
            self.assertEqual(doc_res["status"], "harvested")
            self.assertEqual(len(doc_res["errors"]), 1)
            self.assertIn("simulated post kernel abort", doc_res["errors"][0])

            doc_root = os.path.join(corpus_dir, "OpenDocs", "MultiSetupSample")
            self.assertFalse(os.path.exists(os.path.join(doc_root, "00_BadSetup")))
            self.assertTrue(os.path.isfile(os.path.join(doc_root, "01_GoodSetup", "setup_manifest.json")))

    def test_universal_dumper_guards_unoptimized_5x_abc_and_probing_cycles(self):
        dumper_path = os.path.join(REPO_ROOT, "universal_dumper.cps")
        with open(dumper_path, "r", encoding="utf-8") as f:
            src = f.read()
        self.assertIn("_safeToolAxisABC", src)
        self.assertIn("isOptimizedForMachine", src)
        self.assertIn("isProbeOrInspect", src)

    def test_harvester_handles_generation_not_started_and_postprocess_fallbacks(self):
        class _BrokenFuture:
            @property
            def isGenerationCompleted(self):
                raise RuntimeError("3 : Generation not started")

        class _FakeParamVal:
            def __init__(self, v):
                self.value = v

        class _FakeParams:
            def __init__(self, d):
                self._d = d

            def itemByName(self, k):
                return _FakeParamVal(_FakeParamVal(self._d[k])) if k in self._d else None

        # Structured boolean taxonomy detection (no dependency on concrete strategy names)
        surf_inspect_op = types.SimpleNamespace(
            isSuppressed=False,
            hasToolpath=False,
            isToolpathValid=False,
            parameters=_FakeParams({"operation:isInspectionStrategy": 1}),
            tool=types.SimpleNamespace(parameters=_FakeParams({"tool_number": 99})),
        )
        self.assertFalse(harvest_corpus_fusion._is_supported_cutting_op(surf_inspect_op))

        ungenerated_mill = types.SimpleNamespace(
            isSuppressed=False,
            hasToolpath=False,
            isToolpathValid=False,
            strategy="adaptive2d",
            parameters=_FakeParams({}),
            tool=types.SimpleNamespace(parameters=_FakeParams({"tool_number": 2})),
        )
        valid_mill_1 = types.SimpleNamespace(
            name="ValidOp1",
            isSuppressed=False,
            hasToolpath=True,
            isToolpathValid=True,
            strategy="face",
            parameters=_FakeParams({}),
            tool=types.SimpleNamespace(parameters=_FakeParams({"tool_number": 1})),
        )
        blocked_mill_2 = types.SimpleNamespace(
            name="BlockedOp2",
            isSuppressed=False,
            hasToolpath=True,
            isToolpathValid=True,
            strategy="contour2d",
            parameters=_FakeParams({}),
            tool=types.SimpleNamespace(parameters=_FakeParams({"tool_number": 1})),
        )

        setup_with_mixed_batch = types.SimpleNamespace(
            name="1st op",
            operationType=0,
            isSuppressed=False,
            parameters=_FakeParams({"job_stockMode": "default"}),
            allOperations=[surf_inspect_op, ungenerated_mill, valid_mill_1, blocked_mill_2],
            workCoordinateSystem=None,
        )
        fallback_3axis_setup = types.SimpleNamespace(
            name="Pure3AxisFallback",
            operationType=0,
            isSuppressed=False,
            parameters=_FakeParams({"job_stockMode": "default"}),
            allOperations=[surf_inspect_op, ungenerated_mill, valid_mill_1],
            workCoordinateSystem=None,
        )

        sample_cnc = os.path.join(REPO_ROOT, "tests", "fixtures", "milling_drill_test.cnc")

        def _fake_post_process(ops, p_in):
            # Multi-op collection or BlockedOp2 throws Fusion Personal's 3+2 C++ exception;
            # ValidOp1 fails only when a broken vendor BASE_POST is passed, succeeding with BASE_POST="".
            is_single_valid = getattr(ops, "name", None) == "ValidOp1"
            base_post_env = os.environ.get("BASE_POST", "")
            if not is_single_valid or base_post_env:
                raise RuntimeError("3 : Cannot locate the simple error log generated from the Post Processing.")
            out_json = os.path.join(p_in.outputFolder, f"{p_in.programName}.json")
            target_cnc = os.environ.get("TARGET_INTERMEDIATE_CNC", "")
            shutil.copy2(sample_cnc, target_cnc)
            with open(out_json, "w", encoding="utf-8") as f:
                json.dump(
                    {
                        "unit": 1,
                        "stockMode": "default",
                        "workpiece": {"lower": {"x": 0, "y": 0, "z": -10}, "upper": {"x": 50, "y": 50, "z": 0}},
                        "sections": [{"id": 0, "type": 0, "tool": {"number": 1, "diameter": 6.0}, "events": []}],
                    },
                    f,
                )

        cam = types.SimpleNamespace(
            setups=[setup_with_mixed_batch, fallback_3axis_setup],
            ncPrograms=[],
            generateToolpath=lambda _ops: _BrokenFuture(),
            postProcess=_fake_post_process,
        )
        doc = types.SimpleNamespace(
            name="MotorBracket",
            products=types.SimpleNamespace(itemByProductType=lambda pt: cam if pt == "CAMProductType" else None),
            unitsManager=types.SimpleNamespace(defaultLengthUnits="mm"),
        )

        with tempfile.TemporaryDirectory() as corpus_dir:
            doc_res = harvest_corpus_fusion._harvest_document(
                doc=doc,
                doc_key="open:MotorBracket",
                project_name="OpenDocs",
                doc_name="MotorBracket",
                corpus_dir=corpus_dir,
            )
            self.assertEqual(doc_res["status"], "harvested")
            self.assertEqual(doc_res["errors"], [])
            self.assertEqual(doc_res["setups"][0]["status"], "skipped_capability")
            self.assertEqual(doc_res["setups"][1]["status"], "harvested")
            skipped_dir = os.path.join(corpus_dir, "OpenDocs", "MotorBracket", "00_1st op")
            self.assertFalse(os.path.exists(skipped_dir))
            manifest_p = os.path.join(corpus_dir, "OpenDocs", "MotorBracket", "01_Pure3AxisFallback", "setup_manifest.json")
            self.assertTrue(os.path.isfile(manifest_p))

    def test_harvester_and_corpus_runner_skip_3plus2_multiaxis_and_millturn(self):
        class _FakeParamVal:
            def __init__(self, v):
                self.value = v

        class _FakeParams:
            def __init__(self, d):
                self._d = dict(d)

            def itemByName(self, k):
                return _FakeParamVal(_FakeParamVal(self._d[k])) if k in self._d else None

        tilted_op = types.SimpleNamespace(
            name="TiltedPocket",
            isSuppressed=False,
            hasToolpath=True,
            isToolpathValid=True,
            strategy="pocket2d",
            parameters=_FakeParams({"overrideToolView": 1, "view_orientation_mode": "axesZX"}),
            tool=types.SimpleNamespace(parameters=_FakeParams({"tool_number": 1})),
        )
        wrap_op = types.SimpleNamespace(
            name="WrapAdaptive",
            isSuppressed=False,
            hasToolpath=True,
            isToolpathValid=True,
            strategy="adaptive2d",
            parameters=_FakeParams({"operation:multiAxisMachiningType": "wrapped"}),
            tool=types.SimpleNamespace(parameters=_FakeParams({"tool_number": 2})),
        )
        millturn_op = types.SimpleNamespace(
            name="LiveHexMill",
            isSuppressed=False,
            hasToolpath=True,
            isToolpathValid=True,
            strategy="contour2d",
            parameters=_FakeParams({"operation:isMillingStrategy": 1}),
            tool=types.SimpleNamespace(parameters=_FakeParams({"tool_number": 3})),
        )
        pattern_child_op = types.SimpleNamespace(
            name="PatternFace",
            isSuppressed=False,
            hasToolpath=True,
            isToolpathValid=True,
            strategy="face",
            parameters=_FakeParams({"operation:view_orientation_mode": "useWCS"}),
            tool=types.SimpleNamespace(parameters=_FakeParams({"tool_number": 4})),
        )
        clean_mill_op = types.SimpleNamespace(
            name="TopFace",
            isSuppressed=False,
            hasToolpath=True,
            isToolpathValid=True,
            strategy="face",
            parameters=_FakeParams({"operation:isRotaryStrategy": 1, "operation:view_orientation_mode": "useWCS"}),
            tool=types.SimpleNamespace(parameters=_FakeParams({"tool_number": 5})),
        )

        s_3plus2 = types.SimpleNamespace(
            name="Setup3Plus2",
            operationType=0,
            isSuppressed=False,
            parameters=_FakeParams({"job_stockMode": "default"}),
            allOperations=[clean_mill_op, tilted_op],
        )
        s_wrap = types.SimpleNamespace(
            name="SetupWrap",
            operationType=0,
            isSuppressed=False,
            parameters=_FakeParams({"job_stockMode": "default"}),
            allOperations=[wrap_op],
        )
        s_millturn = types.SimpleNamespace(
            name="SetupMillTurn",
            operationType=1,
            isSuppressed=False,
            parameters=_FakeParams({"job_stockMode": "default"}),
            allOperations=[millturn_op],
        )
        s_rotary_pattern = types.SimpleNamespace(
            name="SetupRotaryPattern",
            operationType=0,
            isSuppressed=False,
            parameters=_FakeParams({"job_stockMode": "default"}),
            allOperations=[clean_mill_op, pattern_child_op],
        )

        sample_cnc = os.path.join(REPO_ROOT, "tests", "fixtures", "milling_drill_test.cnc")
        post_calls = []

        def _fake_post(ops, p_in):
            post_calls.append(getattr(ops, "name", "col"))
            out_json = os.path.join(p_in.outputFolder, f"{p_in.programName}.json")
            target_cnc = os.environ.get("TARGET_INTERMEDIATE_CNC", "")
            shutil.copy2(sample_cnc, target_cnc)
            is_pat = getattr(ops, "name", "") == "PatternFace"
            fwd = {"x": 0.0, "y": -0.8660254, "z": -0.5} if is_pat else {"x": 0.0, "y": 0.0, "z": 1.0}
            with open(out_json, "w", encoding="utf-8") as jf:
                json.dump(
                    {
                        "unit": 1,
                        "stockMode": "default",
                        "workpiece": {"lower": {"x": 0, "y": 0, "z": -10}, "upper": {"x": 50, "y": 50, "z": 0}},
                        "sections": [
                            {
                                "id": 0,
                                "type": 0,
                                "workPlane": {"forward": fwd},
                                "params": {"operation-comment": getattr(ops, "name", "Op")},
                                "events": [],
                            }
                        ],
                    },
                    jf,
                )

        cam = types.SimpleNamespace(
            setups=[s_3plus2, s_wrap, s_millturn, s_rotary_pattern],
            ncPrograms=[],
            postProcess=_fake_post,
        )
        doc = types.SimpleNamespace(
            name="CommercialSample",
            products=types.SimpleNamespace(itemByProductType=lambda pt: cam if pt == "CAMProductType" else None),
            unitsManager=types.SimpleNamespace(defaultLengthUnits="mm"),
        )

        with tempfile.TemporaryDirectory() as corpus_dir:
            # 1-3: Pre-post capability check skips without invoking cam.postProcess or creating bundle_dir
            for idx, s_obj in enumerate((s_3plus2, s_wrap, s_millturn)):
                b_dir = os.path.join(corpus_dir, f"b_{idx}")
                res = harvest_corpus_fusion.harvest_setup(doc, cam, s_obj, idx, b_dir)
                self.assertEqual(res["status"], "skipped_capability")
                self.assertFalse(os.path.exists(b_dir))
            self.assertEqual(post_calls, [])

            # 4: Post-dump capability check catches rotary CAMPattern on batch 1, cleans up bundle_dir, 0 errors
            doc_res = harvest_corpus_fusion._harvest_document(
                doc=doc,
                doc_key="open:CommercialSample",
                project_name="OpenDocs",
                doc_name="CommercialSample",
                corpus_dir=corpus_dir,
            )
            self.assertEqual(doc_res["status"], "skipped_capability")
            self.assertEqual(doc_res["errors"], [])
            self.assertFalse(os.path.exists(os.path.join(corpus_dir, "OpenDocs", "CommercialSample", "03_SetupRotaryPattern")))

            # 5: run_corpus.discover_fixtures skips pre-harvested 3+2 bundle while keeping valid 2-axis centerline-drill lathe bundle
            tilted_bundle = os.path.join(corpus_dir, "PreCached", "Sample3Plus2", "00_Setup1")
            lathe_bundle = os.path.join(corpus_dir, "PreCached", "LatheCenterDrill", "00_Setup1")
            os.makedirs(tilted_bundle)
            os.makedirs(lathe_bundle)
            shutil.copy2(sample_cnc, os.path.join(tilted_bundle, "batch_0.cnc"))
            shutil.copy2(sample_cnc, os.path.join(lathe_bundle, "batch_0.cnc"))
            with open(os.path.join(tilted_bundle, "batch_0.json"), "w", encoding="utf-8") as f:
                json.dump(
                    {
                        "unit": 1,
                        "sections": [{"id": 0, "type": 0, "workPlane": {"forward": {"x": 1.0, "y": 0.0, "z": 0.0}}, "events": []}],
                    },
                    f,
                )
            with open(os.path.join(tilted_bundle, "setup_manifest.json"), "w", encoding="utf-8") as f:
                json.dump(
                    {
                        "schema_version": 1,
                        "project_name": "PreCached",
                        "document_name": "Sample3Plus2",
                        "setup_name": "Setup1",
                        "is_turning": False,
                        "cnc_batches": ["batch_0.cnc"],
                        "json_batches": ["batch_0.json"],
                        "workpiece_mm": {"kind": "box", "x_min": 0, "x_max": 50, "y_min": 0, "y_max": 50, "z_min": -10, "z_max": 0},
                    },
                    f,
                )
            with open(os.path.join(lathe_bundle, "batch_0.json"), "w", encoding="utf-8") as f:
                json.dump(
                    {
                        "unit": 1,
                        "sections": [
                            {
                                "id": 0,
                                "type": 0,
                                "workPlane": {"forward": {"x": 0.0, "y": 0.0, "z": 1.0}},
                                "params": {"operation:strategy": "drill", "operation:isDrillingStrategy": 1},
                                "events": [{"type": "onCyclePoint", "x": 0.0, "y": 0.0, "z": -15.0}],
                            }
                        ],
                    },
                    f,
                )
            with open(os.path.join(lathe_bundle, "setup_manifest.json"), "w", encoding="utf-8") as f:
                json.dump(
                    {
                        "schema_version": 1,
                        "project_name": "PreCached",
                        "document_name": "LatheCenterDrill",
                        "setup_name": "Setup1",
                        "is_turning": True,
                        "cnc_batches": ["batch_0.cnc"],
                        "json_batches": ["batch_0.json"],
                        "workpiece_mm": {"kind": "turning", "z_min": -30, "z_max": 0, "outer_radius": 20, "inner_radius": 0},
                    },
                    f,
                )

            additive_bundle = os.path.join(corpus_dir, "PreCached", "StockFenceAdditive", "00_Setup1")
            os.makedirs(additive_bundle)
            shutil.copy2(sample_cnc, os.path.join(additive_bundle, "batch_0.cnc"))
            with open(os.path.join(additive_bundle, "batch_0.json"), "w", encoding="utf-8") as f:
                json.dump(
                    {
                        "unit": 1,
                        "sections": [{"id": 0, "type": 4, "strategy": "additive_buildstyle", "events": []}],
                    },
                    f,
                )
            with open(os.path.join(additive_bundle, "setup_manifest.json"), "w", encoding="utf-8") as f:
                json.dump(
                    {
                        "schema_version": 1,
                        "project_name": "PreCached",
                        "document_name": "StockFenceAdditive",
                        "setup_name": "Setup1",
                        "is_turning": False,
                        "cnc_batches": ["batch_0.cnc"],
                        "json_batches": ["batch_0.json"],
                        "workpiece_mm": {"kind": "box", "x_min": 0, "x_max": 50, "y_min": 0, "y_max": 50, "z_min": -10, "z_max": 0},
                    },
                    f,
                )

            discovered = run_corpus.discover_fixtures(corpus_dir=corpus_dir, include_builtin_fixtures=False)
            self.assertEqual([fx.name for fx in discovered], ["PreCached/LatheCenterDrill/Setup1"])


if __name__ == "__main__":
    unittest.main()
