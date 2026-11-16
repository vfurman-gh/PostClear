#!/usr/bin/env python3
"""
Stage 3 tests for `universal_replayer.cps` after un-stringifying `buildVendor`
onto a top-level `__host` object inside `onClose` and unifying Manual NC
expanded-event dispatch.
"""

import json
import os
import re
import subprocess
import tempfile
import unittest

from pipeline import find_autodesk_post_binary


def _identity_wp():
    return {
        "right": {"x": 1.0, "y": 0.0, "z": 0.0},
        "up": {"x": 0.0, "y": 1.0, "z": 0.0},
        "forward": {"x": 0.0, "y": 0.0, "z": 1.0},
    }


def _zero_wo():
    return {"x": 0.0, "y": 0.0, "z": 0.0}


class TestStage3ReplayerSourceAndBehavior(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        cls.replayer_cps = os.path.join(cls.repo_root, "universal_replayer.cps")
        cls.milling_cnc = os.path.join(cls.repo_root, "tests", "fixtures", "milling_drill_test.cnc")
        try:
            cls.post_bin = find_autodesk_post_binary()
        except FileNotFoundError:
            cls.post_bin = None

    def _run_replayer(self, tmp_dir, vendor_code, ctx, out_name="out.nc"):
        if not self.post_bin or not os.path.isfile(self.milling_cnc):
            self.skipTest("Autodesk post binary or milling_drill_test.cnc fixture not found")
        vendor_cps = os.path.join(tmp_dir, "probe_vendor.cps")
        with open(vendor_cps, "w", encoding="utf-8") as f:
            f.write(vendor_code)
        ctx_path = os.path.join(tmp_dir, "ctx.json")
        with open(ctx_path, "w", encoding="utf-8") as f:
            json.dump(ctx, f)
        out_nc = os.path.join(tmp_dir, out_name)
        diag_json = os.path.join(tmp_dir, "diag.json")
        env = os.environ.copy()
        env["BASE_POST"] = vendor_cps
        env["CONTEXT_JSON"] = ctx_path
        env["REPLAY_DIAG_JSON"] = diag_json
        res = subprocess.run(
            [self.post_bin, "--quiet", self.replayer_cps, self.milling_cnc, out_nc],
            env=env,
            capture_output=True,
            text=True,
            timeout=30,
        )
        self.assertEqual(res.returncode, 0, f"post failed:\nSTDOUT:{res.stdout}\nSTDERR:{res.stderr}")
        with open(out_nc, "r", encoding="utf-8") as f:
            return f.read()

    def test_01_replayer_source_thin_build_vendor_and_shared_manual_nc_helper(self):
        with open(self.replayer_cps, "r", encoding="utf-8") as f:
            src = f.read()

        # Standalone CPS check: no include(...) or external universal_common.js
        self.assertNotIn("universal_common", src)
        self.assertNotRegex(src, r"(?m)^\s*include\s*\(")

        # Thin new Function("__host", ...) block (~40-60 lines)
        m = re.search(
            r'var buildVendor = new Function\("__host",\n(.*?)\n  \);',
            src,
            re.DOTALL,
        )
        self.assertIsNotNone(m, "Expected `var buildVendor = new Function(\"__host\", ...)`")
        fn_lines = m.group(0).splitlines()
        self.assertLessEqual(len(fn_lines), 60, f"buildVendor grew too large: {len(fn_lines)} lines")

        # Verify lexical forwarders delegate to __host rather than duplicating bodies
        for helper in (
            "getCurrentSectionId",
            "getPreviousSection",
            "getNextSection",
            "isNewWorkPlane",
            "isNewWorkOffset",
            "isToolChangeNeeded",
            "getNextTool",
            "isSpindleSpeedDifferent",
            "isProbeOperation",
            "isDrillingCycle",
            "hasParameter",
            "getParameter",
            "getProperty",
            "setProperty",
            "linearize",
            "expandCyclePoint",
            "repositionToCycleClearance",
            "expandManualNC",
        ):
            self.assertIn(f"__host.{helper}(", m.group(1), f"Expected forwarder to __host.{helper}")

        # Verify shared Manual NC expanded-event dispatcher is used in both places
        self.assertIn("function _dispatchManualNCExpanded(", src)
        self.assertGreaterEqual(src.count("_dispatchManualNCExpanded("), 3)  # 1 def + 2 call sites

    def test_02_vendor_section_and_tool_lexical_helpers(self):
        vendor_code = (
            'description = "Stage3 Section/Tool Probe";\n'
            'capabilities = CAPABILITY_MILLING | CAPABILITY_TURNING;\n'
            'extension = "nc";\n'
            'function onOpen() {\n'
            '  writeln("OPEN_CURSEC:" + getCurrentSectionId());\n'
            '}\n'
            'function onSection() {\n'
            '  var id = currentSection.getId();\n'
            '  var nt0 = getNextTool();\n'
            '  var nt1 = getNextTool(true);\n'
            '  var ntSec = getNextTool(currentSection, true);\n'
            '  writeln("SEC:" + id\n'
            '    + "|CUR:" + getCurrentSectionId()\n'
            '    + "|FIRST:" + isFirstSection()\n'
            '    + "|LAST:" + isLastSection()\n'
            '    + "|HPREV:" + hasPreviousSection()\n'
            '    + "|HNEXT:" + hasNextSection()\n'
            '    + "|PREV:" + getPreviousSection().getId()\n'
            '    + "|NEXT:" + getNextSection().getId()\n'
            '    + "|WP:" + isNewWorkPlane()\n'
            '    + "|WO:" + isNewWorkOffset()\n'
            '    + "|TC0:" + isToolChangeNeeded()\n'
            '    + "|TC_DIA:" + isToolChangeNeeded("diameter")\n'
            '    + "|TC_FL:" + isToolChangeNeeded("fluteLength")\n'
            '    + "|TC_SEC:" + isToolChangeNeeded(currentSection, "fluteLength")\n'
            '    + "|NT0:" + (nt0 ? nt0.number : "none")\n'
            '    + "|NT1:" + (nt1 ? nt1.number : "none")\n'
            '    + "|NTS:" + (ntSec ? ntSec.number : "none")\n'
            '    + "|SPD:" + isSpindleSpeedDifferent()\n'
            '    + "|PROBE:" + isProbeOperation()\n'
            '    + "|DRILL:" + isDrillingCycle()\n'
            '    + "|P_SEC:" + hasParameter("secOnly") + ":" + getParameter("secOnly", "d")\n'
            '    + "|P_GLB:" + hasParameter("globalOnly") + ":" + getParameter("globalOnly", "d")\n'
            '    + "|P_MIS:" + hasParameter("noSuchParam") + ":" + getParameter("noSuchParam", "fb")\n'
            '  );\n'
            '}\n'
            'function onSectionEnd() { writeln("END_CURSEC:" + getCurrentSectionId()); }\n'
            'function onClose() { writeln("CLOSE_CURSEC:" + getCurrentSectionId()); }\n'
        )

        def _sec(
            sid,
            tool_num,
            flute_len,
            work_offset=1,
            init_axis=None,
            final_axis=None,
            spindle_mode=0,
            rpm=3000,
            css=0,
            strategy="adaptive",
            params=None,
        ):
            p = {"operation-comment": f"Op{sid}", "operation-strategy": strategy}
            if params:
                p.update(params)
            return {
                "id": sid,
                "type": 0,
                "workOffset": work_offset,
                "workPlane": _identity_wp(),
                "workOrigin": _zero_wo(),
                "initialPosition": {"x": 0.0, "y": 0.0, "z": 5.0},
                "finalPosition": {"x": 0.0, "y": 0.0, "z": 5.0},
                "initialToolAxis": init_axis or {"x": 0.0, "y": 0.0, "z": 1.0},
                "finalToolAxis": final_axis or {"x": 0.0, "y": 0.0, "z": 1.0},
                "strategy": strategy,
                "initSpindleSpeed": rpm,
                "initMovement": 0,
                "initRadiusComp": 0,
                "params": p,
                "tool": {
                    "number": tool_num,
                    "diameter": 10.0,
                    "fluteLength": flute_len,
                    "_spindleMode": spindle_mode,
                    "spindleRPM": rpm,
                    "surfaceSpeed": css,
                    "clockwise": True,
                },
                "events": [],
            }

        ctx = {
            "unit": 1,
            "programName": "Stage3Probe",
            "programComment": "",
            "globalParams": {"globalOnly": "gval"},
            "comments": [],
            "manualNC": [],
            "sections": [
                # Sec 0: T1 RPM 3000, ends with tilted tool axis (0,1,0)
                _sec(0, 1, 20.0, work_offset=1, final_axis={"x": 0.0, "y": 1.0, "z": 0.0}, params={"secOnly": "s0"}),
                # Sec 1: T1 RPM 3000 same diameter, different fluteLength, new workPlane, same workOffset
                _sec(1, 1, 35.0, work_offset=1, init_axis={"x": 0.0, "y": 0.0, "z": 1.0}),
                # Sec 2: T2 RPM 4500 (RPM speed diff!), new workOffset=2, probe strategy
                _sec(2, 2, 35.0, work_offset=2, rpm=4500, strategy="probe"),
                # Sec 3: T2 CSS 150 (mode change -> speed diff!), same workOffset=2
                _sec(3, 2, 35.0, work_offset=2, spindle_mode=1, css=150, strategy="adaptive"),
                # Sec 4: T2 CSS 220 (CSS speed diff!), same workOffset=2, drill strategy
                _sec(4, 2, 35.0, work_offset=2, spindle_mode=1, css=220, strategy="drill"),
            ],
        }

        with tempfile.TemporaryDirectory() as td:
            out = self._run_replayer(td, vendor_code, ctx)

        self.assertIn("OPEN_CURSEC:-1", out)
        self.assertIn("END_CURSEC:0", out)
        self.assertIn("CLOSE_CURSEC:-1", out)
        self.assertIn(
            "SEC:0|CUR:0|FIRST:true|LAST:false|HPREV:false|HNEXT:true|PREV:0|NEXT:1"
            "|WP:true|WO:true|TC0:true|TC_DIA:true|TC_FL:true|TC_SEC:true"
            "|NT0:2|NT1:2|NTS:2|SPD:true|PROBE:false|DRILL:false"
            "|P_SEC:true:s0|P_GLB:true:gval|P_MIS:false:fb",
            out,
        )
        self.assertIn(
            "SEC:1|CUR:1|FIRST:false|LAST:false|HPREV:true|HNEXT:true|PREV:0|NEXT:2"
            "|WP:true|WO:false|TC0:false|TC_DIA:false|TC_FL:true|TC_SEC:true"
            "|NT0:2|NT1:2|NTS:2|SPD:false|PROBE:false|DRILL:false"
            "|P_SEC:false:d|P_GLB:true:gval|P_MIS:false:fb",
            out,
        )
        self.assertIn(
            "SEC:2|CUR:2|FIRST:false|LAST:false|HPREV:true|HNEXT:true|PREV:1|NEXT:3"
            "|WP:false|WO:true|TC0:true|TC_DIA:false|TC_FL:false|TC_SEC:false"
            "|NT0:none|NT1:1|NTS:1|SPD:true|PROBE:true|DRILL:false",
            out,
        )
        self.assertIn(
            "SEC:3|CUR:3|FIRST:false|LAST:false|HPREV:true|HNEXT:true|PREV:2|NEXT:4"
            "|WP:false|WO:false|TC0:false|TC_DIA:false|TC_FL:false|TC_SEC:false"
            "|NT0:none|NT1:1|NTS:1|SPD:true|PROBE:false|DRILL:false",
            out,
        )
        self.assertIn(
            "SEC:4|CUR:4|FIRST:false|LAST:true|HPREV:true|HNEXT:false|PREV:3|NEXT:4"
            "|WP:false|WO:false|TC0:false|TC_DIA:false|TC_FL:false|TC_SEC:false"
            "|NT0:none|NT1:1|NTS:1|SPD:true|PROBE:false|DRILL:true",
            out,
        )

    def test_03_properties_linearize_cycles_manual_nc_and_rapid_restored(self):
        vendor_code = (
            'description = "Stage3 Motion/Cycle/ManualNC Probe";\n'
            'capabilities = CAPABILITY_MILLING;\n'
            'extension = "nc";\n'
            'properties = { propObj: { title: "Obj", type: "integer", value: 10 }, propRaw: 5 };\n'
            'function writeComment(text) { writeln("(" + text + ")"); }\n'
            'function onOpen() {\n'
            '  writeln("PROPS_INIT:" + getProperty("propObj") + ":" + getProperty("propRaw") + ":" + getProperty("missing", 99));\n'
            '  setProperty("propRaw", 77);\n'
            '  writeln("PROPS_MUT:" + getProperty("propRaw"));\n'
            '}\n'
            'function onComment(text) { writeln("CMT:" + text); }\n'
            'function onPassThrough(text) { writeln("PT:" + text); }\n'
            'function onParameter(name, value) {\n'
            '  if (name === "mncParam" || name === "display") writeln("PARAM:" + name + "=" + value);\n'
            '}\n'
            'function onCommand(cmd) { writeln("CMD:" + cmd); }\n'
            'function onDwell(sec) { writeln("DWELL:" + sec); }\n'
            'function onManualNC(command, value) {\n'
            '  if (command === 9001) { expandManualNC(command, value); }\n'
            '  else if (command === 9002) { expandManualNC(COMMAND_COMMENT, value); }\n'
            '  else if (command === 9003) { expandManualNC(COMMAND_PASS_THROUGH, value); }\n'
            '  else if (command === 9004) { expandManualNC(COMMAND_DWELL, value); }\n'
            '  else if (command === 9005) { expandManualNC(COMMAND_DISPLAY_MESSAGE, value); }\n'
            '  else { expandManualNC(command, value); }\n'
            '}\n'
            'function onRapid(x, y, z) { writeln("G0:" + Math.round(x) + "," + Math.round(y) + "," + Math.round(z) + "|M=" + movement); }\n'
            'function onLinear(x, y, z, feed) { writeln("G1:" + Math.round(x) + "," + Math.round(y) + "," + Math.round(z) + "|F=" + feed + "|M=" + movement); }\n'
            'function onCircular(cw, cx, cy, cz, x, y, z, feed) { linearize(0.01); }\n'
            'function onCyclePoint(x, y, z) {\n'
            '  if (isFirstCyclePoint()) { repositionToCycleClearance(cycle, x, y, z); }\n'
            '  expandCyclePoint(x, y, z);\n'
            '  writeln("CEXP:" + cycleExpanded);\n'
            '}\n'
        )

        ctx = {
            "unit": 1,
            "programName": "Stage3Replay",
            "programComment": "",
            "postProperties": {"propObj": 55},
            "globalParams": {},
            "comments": [],
            "manualNC": [
                {
                    "command": 9001,
                    "value": "queuedVal",
                    "expanded": [
                        {"type": "onComment", "c": "q_cmt"},
                        {"type": "onPassThrough", "text": "q_pt"},
                        {"type": "onParameter", "name": "mncParam", "value": "q_prm"},
                        {"type": "onCommand", "cmd": 42},
                        {"type": "onDwell", "sec": 0.5},
                    ],
                },
                {"command": 9002, "value": "fb_cmt"},
                {"command": 9003, "value": "fb_pt"},
                {"command": 9004, "value": 1.25},
                {"command": 9005, "value": "fb_disp"},
                {"command": 9099, "value": "fb_raw_cmd"},
            ],
            "sections": [
                {
                    "id": 0,
                    "type": 0,
                    "workOffset": 1,
                    "workPlane": _identity_wp(),
                    "workOrigin": _zero_wo(),
                    "initialPosition": {"x": 0.0, "y": 0.0, "z": 5.0},
                    "finalPosition": {"x": 20.0, "y": 0.0, "z": 5.0},
                    "initSpindleSpeed": 2000,
                    "initMovement": 0,
                    "initRadiusComp": 0,
                    "params": {"operation-comment": "Stage3Op"},
                    "tool": {"number": 1, "diameter": 6.0, "spindleRPM": 2000},
                    "events": [
                        # Circular with linearizedMoves
                        {
                            "type": "onCircular",
                            "cw": True, "plane": 0,
                            "sx": 0.0, "sy": 0.0, "sz": 5.0,
                            "cx": 5.0, "cy": 0.0, "cz": 5.0,
                            "x": 10.0, "y": 0.0, "z": 5.0, "feed": 300,
                            "radius": 5.0, "sweep": 3.14, "nx": 0.0, "ny": 0.0, "nz": 1.0,
                            "linearizedMoves": [
                                {"type": "onLinear", "x": 5.0, "y": 5.0, "z": 5.0, "feed": 300, "movement": 1},
                                {"type": "onLinear", "x": 10.0, "y": 0.0, "z": 5.0, "feed": 300, "movement": 1},
                            ],
                        },
                        # Circular fallback when linearizedMoves absent
                        {
                            "type": "onCircular",
                            "cw": False, "plane": 0,
                            "sx": 10.0, "sy": 0.0, "sz": 5.0,
                            "cx": 12.0, "cy": 0.0, "cz": 5.0,
                            "x": 14.0, "y": 0.0, "z": 5.0, "feed": 250, "movement": 1,
                            "radius": 2.0, "sweep": 3.14, "nx": 0.0, "ny": 0.0, "nz": 1.0,
                        },
                        # Promoted rapid -> full-line Rapid restored comment
                        {
                            "type": "onLinear",
                            "sx": 14.0, "sy": 0.0, "sz": 5.0,
                            "x": 14.0, "y": 0.0, "z": 15.0,
                            "feed": 5000, "movement": 0, "rc": 0, "rpm": 2000,
                            "promote": True,
                        },
                        {"type": "onCycle", "cycleType": "drilling", "cycle": {}, "points": [
                            {"x": 14.0, "y": 0.0, "z": -5.0},
                            {"x": 20.0, "y": 0.0, "z": -5.0},
                        ]},
                        # First point calls repositionToCycleClearance -> uses reposMoves + afterReposMoves
                        {
                            "type": "onCyclePoint",
                            "sx": 14.0, "sy": 0.0, "sz": 15.0,
                            "x": 14.0, "y": 0.0, "z": -5.0,
                            "first": True, "last": False,
                            "reposMoves": [{"type": "onRapid", "x": 14.0, "y": 0.0, "z": 20.0, "movement": 0}],
                            "afterReposMoves": [
                                {"type": "onLinear", "x": 14.0, "y": 0.0, "z": -5.0, "feed": 120, "movement": 1},
                                {"type": "onDwell", "sec": 0.3},
                            ],
                            "expandedMoves": [
                                {"type": "onRapid", "x": 99.0, "y": 99.0, "z": 99.0, "movement": 0},
                            ],
                        },
                        # Second point skips repositionToCycleClearance -> uses expandedMoves (not afterReposMoves)
                        {
                            "type": "onCyclePoint",
                            "sx": 14.0, "sy": 0.0, "sz": -5.0,
                            "x": 20.0, "y": 0.0, "z": -5.0,
                            "first": False, "last": True,
                            "reposMoves": [{"type": "onRapid", "x": 20.0, "y": 0.0, "z": 20.0, "movement": 0}],
                            "afterReposMoves": [
                                {"type": "onLinear", "x": 88.0, "y": 88.0, "z": 88.0, "feed": 120, "movement": 1},
                            ],
                            "expandedMoves": [
                                {"type": "onLinear", "x": 20.0, "y": 0.0, "z": -5.0, "feed": 180, "movement": 1},
                            ],
                        },
                        {"type": "onCycleEnd"},
                    ],
                }
            ],
        }

        with tempfile.TemporaryDirectory() as td:
            out = self._run_replayer(td, vendor_code, ctx)

        # Properties (applyPostProp + getProperty + setProperty)
        self.assertIn("PROPS_INIT:55:5:99", out)
        self.assertIn("PROPS_MUT:77", out)

        # Manual NC queued-expanded & fallback COMMAND_*
        for expected in (
            "CMT:q_cmt", "PT:q_pt", "PARAM:mncParam=q_prm", "CMD:42", "DWELL:0.5",
            "CMT:fb_cmt", "PT:fb_pt", "DWELL:1.25", "PARAM:display=fb_disp", "CMD:9099",
        ):
            self.assertIn(expected, out)

        # Linearize (both linearizedMoves and endpoint fallback)
        self.assertIn("G1:5,5,5|F=300|M=1", out)
        self.assertIn("G1:10,0,5|F=300|M=1", out)
        self.assertIn("G1:14,0,5|F=250|M=1", out)

        # Full-line Rapid restored comment + G0 promotion
        self.assertRegex(out, r"(?m)^\(Rapid restored\)$")
        self.assertIn("G0:14,0,15|M=0", out)

        # Cycle expansion with and without repositionToCycleClearance
        self.assertIn("G0:14,0,20|M=0", out)
        self.assertIn("G1:14,0,-5|F=120|M=1", out)
        self.assertIn("DWELL:0.3", out)
        self.assertNotIn("99,99,99", out)
        self.assertIn("G1:20,0,-5|F=180|M=1", out)
        self.assertNotIn("88,88,88", out)
        self.assertIn("CEXP:true", out)

    def test_04_manual_nc_without_vendor_onManualNC_and_vendor_setProperty_override(self):
        # Vendor defines custom setProperty/getProperty (like centroid_turning) and omits onManualNC
        vendor_code = (
            'description = "Stage3 No-onManualNC & Custom setProperty Probe";\n'
            'capabilities = CAPABILITY_MILLING;\n'
            'extension = "nc";\n'
            'properties = { customP: { value: "orig", current: "orig" } };\n'
            'function setProperty(name, value) { properties[name].current = "custom:" + value; }\n'
            'function getProperty(name) { return properties[name] ? properties[name].current : undefined; }\n'
            'function onOpen() { writeln("CUST_PROP:" + getProperty("customP")); }\n'
            'function onComment(text) { writeln("NO_MNC_CMT:" + text); }\n'
            'function onPassThrough(text) { writeln("NO_MNC_PT:" + text); }\n'
            'function onParameter(name, value) { if (name === "p1") writeln("NO_MNC_PRM:" + value); }\n'
            'function onCommand(cmd) { writeln("NO_MNC_CMD:" + cmd); }\n'
            'function onDwell(sec) { writeln("NO_MNC_DWL:" + sec); }\n'
        )
        ctx = {
            "unit": 1,
            "programName": "NoMnc",
            "programComment": "",
            "postProperties": {"customP": "override"},
            "globalParams": {},
            "comments": [],
            "manualNC": [
                {
                    "command": 1,
                    "value": "ignored",
                    "expanded": [
                        {"type": "onComment", "c": "c1"},
                        {"type": "onPassThrough", "text": "pt1"},
                        {"type": "onParameter", "name": "p1", "value": "v1"},
                        {"type": "onCommand", "cmd": 7},
                        {"type": "onDwell", "sec": 0.75},
                    ],
                }
            ],
            "sections": [],
        }
        with tempfile.TemporaryDirectory() as td:
            out = self._run_replayer(td, vendor_code, ctx)

        self.assertIn("CUST_PROP:custom:override", out)
        for expected in ("NO_MNC_CMT:c1", "NO_MNC_PT:pt1", "NO_MNC_PRM:v1", "NO_MNC_CMD:7", "NO_MNC_DWL:0.75"):
            self.assertIn(expected, out)


if __name__ == "__main__":
    unittest.main()
