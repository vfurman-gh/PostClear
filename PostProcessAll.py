#Author-Tim Paterson
#Description-Post process all CAM setups, using the setup name as the output file name.

import collections
import contextlib
import datetime
import enum
import glob
import html
import importlib
import io
import json
import math
import os
import pathlib
import re
import shutil
import subprocess
import sys
import tempfile
import time
import traceback
import urllib.parse
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union

import adsk.cam
import adsk.core
import adsk.fusion

if not hasattr(adsk.cam, "SetupStockModes"):
    class SetupStockModes:
        FixedBoxStock = 0
        RelativeBoxStock = 1
        FixedCylinderStock = 2
        RelativeCylinderStock = 3
        FixedTubeStock = 4
        RelativeTubeStock = 5
        SolidStock = 6
        PreviousSetupStock = 7
    adsk.cam.SetupStockModes = SetupStockModes

if not hasattr(adsk.cam, "PostOutputUnitOptions"):
    class PostOutputUnitOptions:
        DocumentUnitsOutput = 0
        InchesOutput = 1
        MillimetersOutput = 2
    adsk.cam.PostOutputUnitOptions = PostOutputUnitOptions


# Version number of settings as saved in documents and settings file
# update this whenever settings content changes
version = 17

# Initial default values of settings
defaultSettings = {
    "version" : version,
    "ncProgram": "",
    "ncPrograms": [],
    "output" : "",
    "restore_rapid" : True,
    # Groups are expanded or not
    "groupPersonal" : True
}

# Constants
constCmdName = "Post Process All"
constCmdDefId = "PatersonTech_PostProcessAll"
constCAMWorkspaceId = "CAMEnvironment"
constCAMActionsPanelId = "CAMActionPanel"
constPostProcessControlId = "IronPostProcess"
constCAMProductId = "CAMProductType"
constAttrGroup = constCmdDefId
constAttrName = "settings"
constAttrCompressedName = "CompressedName"
constSettingsFileExt = ".settings"
constNcProgramName = "PostProcessAll NC Program"
constAddonDir = os.path.dirname(os.path.abspath(__file__))
constDumperScript = os.path.join(constAddonDir, "universal_dumper.cps")
constReplayerScript = os.path.join(constAddonDir, "universal_replayer.cps")
constPackagesDir = os.path.join(constAddonDir, "packages")

_LOG_PATH = os.path.join(tempfile.gettempdir(), "postprocess_debug.log")
_MAX_LOG_SIZE_BYTES = 5 * 1024 * 1024  # 5 MB capped debug log

def _RotateLogIfNeeded(log_path: str = _LOG_PATH, max_bytes: int = _MAX_LOG_SIZE_BYTES) -> None:
    """Rotate log file to .1 if size exceeds max_bytes."""
    try:
        if os.path.isfile(log_path) and os.path.getsize(log_path) >= max_bytes:
            rot_path = log_path + ".1"
            if os.path.isfile(rot_path):
                os.remove(rot_path)
            os.replace(log_path, rot_path)
    except Exception:
        pass

def _dbg_global(msg):
    try:
        _RotateLogIfNeeded(_LOG_PATH)
        t = time.strftime('%H:%M:%S')
        with open(_LOG_PATH, "a", encoding="utf8") as f:
            f.write(f"[{t}] {msg}\n")
            f.flush()
    except Exception:
        pass

_INVALID_PATH_CHARS = re.compile(r'[<>:"/\\|?*\x00-\x1f]')
_RESERVED_DEVICE_NAMES = {
    "CON", "PRN", "AUX", "NUL",
    *(f"COM{i}" for i in range(1, 10)),
    *(f"LPT{i}" for i in range(1, 10))
}

def _SanitizePathSegment(segment: str) -> str:
    """Sanitize a file or folder segment for cross-platform compatibility (Windows/POSIX)."""
    if not segment:
        return "unnamed"
    clean = _INVALID_PATH_CHARS.sub("_", segment).strip()
    clean = re.sub(r"_+", "_", clean)
    clean = clean.strip("._ ")
    if not clean:
        clean = "unnamed"
    stem = clean.split(".")[0].upper()
    if stem in _RESERVED_DEVICE_NAMES:
        clean = f"_{clean}"
    return clean


def _NormalizeFileExtension(ext: Optional[str]) -> str:
    """Normalize file extension ensuring leading dot and replacing .json with .cnc."""
    if not ext:
        return ".cnc"
    clean = ext.strip()
    if clean.lower().lstrip(".") == "json":
        clean = ".cnc"
    if not clean.startswith("."):
        clean = "." + clean
    return clean


def _ResolveSetupFolderAndFilename(outputFolder: str, setup_name: str) -> Tuple[str, str]:
    """Split and sanitize setup name (supporting ':' subfolders) into (setupFolder, filename)."""
    raw_segments = [s.strip() for s in setup_name.split(':')]
    clean_segments = [_SanitizePathSegment(s) for s in raw_segments if s]
    if not clean_segments:
        clean_segments = ["unnamed"]
    sFolder = os.path.join(outputFolder, *clean_segments[:-1]) if len(clean_segments) > 1 else outputFolder
    sFname = clean_segments[-1]
    return sFolder, sFname

# Tool tip text
toolTip = (
    "Post process all setups into G-code for your machine.\n\n"
    "The name of the setup is used for the name of the output "
    "file adding the appropriate extension. A colon (':') in the name indicates "
    "the preceding portion is the name of a subfolder. Multiple "
    "colons can be used to nest subfolders. Spaces around colons "
    "are removed."
    )

# Global list to keep all event handlers in scope.
# This is only needed with Python.
handlers = []

# Global settingsMgr object
settingsMgr = None

_MarkingMenuEventHandlerBase = (
    getattr(adsk.core, "MarkingMenuEventHandler", object)
    if ("adsk" in sys.modules and hasattr(adsk, "core") and hasattr(adsk.core, "MarkingMenuEventHandler"))
    else object
)
_marking_menu_handler = None
_pending_context_selection = None  # Optional[Tuple[float, List[str]]]


def _CleanupCommandUI(ui) -> None:
    """Safely remove any existing toolbar controls, marking-menu handler, command definition, and handlers."""
    global _marking_menu_handler, _pending_context_selection
    _pending_context_selection = None

    if _marking_menu_handler is not None:
        try:
            if ui and hasattr(ui, "markingMenuDisplaying") and ui.markingMenuDisplaying:
                ui.markingMenuDisplaying.remove(_marking_menu_handler)
        except Exception:
            pass
        _marking_menu_handler = None

    if not ui:
        handlers.clear()
        return

    # Delete existing toolbar controls first, before deleting the command definition.
    panels = []
    try:
        workspaces = getattr(ui, "workspaces", None)
        cam_ws = workspaces.itemById(constCAMWorkspaceId) if workspaces else None
        ws_panels = getattr(cam_ws, "toolbarPanels", None) if cam_ws else None
        ws_panel = ws_panels.itemById(constCAMActionsPanelId) if ws_panels else None
        if ws_panel is not None:
            panels.append(ws_panel)
    except Exception:
        pass

    try:
        if hasattr(ui, "allToolbarPanels") and ui.allToolbarPanels:
            all_panel = ui.allToolbarPanels.itemById(constCAMActionsPanelId)
            if all_panel is not None and all_panel not in panels:
                panels.append(all_panel)
    except Exception:
        pass

    for panel in panels:
        try:
            controls = getattr(panel, "controls", None)
            cmd_control = controls.itemById(constCmdDefId) if controls else None
            if cmd_control:
                cmd_control.deleteMe()
        except Exception:
            pass

    try:
        cmd_defs = getattr(ui, "commandDefinitions", None)
        existing_cmd_def = cmd_defs.itemById(constCmdDefId) if cmd_defs else None
        if existing_cmd_def:
            existing_cmd_def.deleteMe()
    except Exception:
        pass

    handlers.clear()


def run(context):
    global settingsMgr
    ui = None
    try:
        _EnsureLocalModulesLoaded(force=True)
        settingsMgr = SettingsManager()
        app = adsk.core.Application.get()
        ui  = app.userInterface
        InitAddIn()

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


def stop(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        # Clean up the UI: delete controls first, then command definition, then palette.
        _CleanupCommandUI(ui)

        if ui and hasattr(ui, "palettes") and ui.palettes:
            try:
                p = ui.palettes.itemById(constNotificationPaletteId)
                if p:
                    p.deleteMe()
            except Exception:
                pass
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))	


class SettingsManager:
    def __init__(self):
        self.default = None
        self.path = None
        self.fMustSave = False
        self.inputs = None

    def GetSettings(self, docAttr):
        docSettings = None
        attr = docAttr.itemByName(constAttrGroup, constAttrName)
        if attr:
            try:
                docSettings = json.loads(attr.value)
                if docSettings["version"] == version:
                    return docSettings
            except Exception:
                pass
            
        # Document does not have valid settings, get defaults
        if not self.default:
            # Haven't read the settings file yet
            try:
                with open(self.GetPath(), "r", encoding="utf-8") as file:
                    self.default = json.load(file)
                if self.default["version"] != version:
                    self.UpdateSettings(defaultSettings, self.default)
            except Exception:
                self.default = {
                    k: (list(v) if isinstance(v, list) else v)
                    for k, v in defaultSettings.items()
                }
                self.fMustSave = True
        
        if not docSettings:
            docSettings = {
                k: (list(v) if isinstance(v, list) else v)
                for k, v in self.default.items()
            }
        else:
            self.UpdateSettings(self.default, docSettings)
        if "ncPrograms" in docSettings and isinstance(docSettings["ncPrograms"], list):
            docSettings["ncPrograms"] = list(docSettings["ncPrograms"])
        return docSettings

    def SaveDefault(self, docSettings):
        self.fMustSave = False
        self.default = {
            k: (list(v) if isinstance(v, list) else v)
            for k, v in docSettings.items()
        }
        try:
            strSettings = json.dumps(docSettings)
            with open(self.GetPath(), "w", encoding="utf-8") as file:
                file.write(strSettings)
        except Exception:
            pass

    def SaveSettings(self, docAttr, docSettings):
        if self.fMustSave:
            self.SaveDefault(docSettings)
        docAttr.add(constAttrGroup, constAttrName, json.dumps(docSettings))
            
    def UpdateSettings(self, src, dst):
        if "restore_rapid" not in dst:
            dst["restore_rapid"] = dst.get("fastZ", dst.get("fastZHeuristic", True))
        if "ncPrograms" not in dst:
            legacy_prog = dst.get("ncProgram", "")
            dst["ncPrograms"] = [legacy_prog] if isinstance(legacy_prog, str) and legacy_prog else []
        elif isinstance(dst.get("ncPrograms"), list):
            dst["ncPrograms"] = [str(x) for x in dst["ncPrograms"] if x]
        else:
            dst["ncPrograms"] = []
        if dst["ncPrograms"] and not dst.get("ncProgram"):
            dst["ncProgram"] = dst["ncPrograms"][0]
        for k in (
            "verifyGcode", "splitSetup", "fastZ", "fastZHeuristic", "toolChange", "endCodes", "homeEndsOp",
            "sequence", "twoDigits", "onlySelected", "groupRename", "groupPost", "numericName",
            "groupAdvanced", "initialDelay", "postRetries", "delFiles", "delFolder", "combineTool"
        ):
            dst.pop(k, None)
        for item in src:
            if not (item in dst):
                val = src[item]
                dst[item] = list(val) if isinstance(val, list) else val
        dst["version"] = src["version"]

    def GetPath(self):
        if not self.path:
            pos = __file__.rfind(".")
            if pos == -1:
                pos = len(__file__)
            self.path = __file__[0:pos] + constSettingsFileExt
        return self.path


def InitAddIn():
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface

        # Ensure idempotent registration if a stale control or command definition exists.
        _CleanupCommandUI(ui)

        # Create a button command definition.
        cmdDefs = ui.commandDefinitions
        cmdResFolder = os.path.join(constAddonDir, "resources", "Command")
        cmdDef = cmdDefs.addButtonDefinition(constCmdDefId, constCmdName, toolTip, cmdResFolder)
        
        # Connect to the commandCreated event.
        commandEventHandler = CommandEventHandler()
        cmdDef.commandCreated.add(commandEventHandler)
        handlers.append(commandEventHandler)
        
        # Get the Actions panel in the Manufacture workspace.
        workSpace = ui.workspaces.itemById(constCAMWorkspaceId)
        addInsPanel = workSpace.toolbarPanels.itemById(constCAMActionsPanelId)
        
        # Add the button right after the Post Process command.
        cmdControl = addInsPanel.controls.addCommand(cmdDef, constPostProcessControlId, False)
        cmdControl.isPromotedByDefault = True
        cmdControl.isPromoted = True

        # Register right-click marking menu handler for CAM entities.
        global _marking_menu_handler
        if hasattr(ui, "markingMenuDisplaying") and ui.markingMenuDisplaying is not None:
            try:
                onMarkingMenu = MarkingMenuHandler()
                ui.markingMenuDisplaying.add(onMarkingMenu)
                _marking_menu_handler = onMarkingMenu
                handlers.append(onMarkingMenu)
            except Exception:
                pass

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


def ExpandFileName(file):
    return os.path.expanduser(file).replace("\\", "/")


def CompressFileName(file):
    # normalize whacks 
    base = os.path.expanduser("~").replace("\\", "/")
    newFile = file.replace("\\", "/").removeprefix(base)
    if len(file) != len(newFile) and newFile[0] == "/":
        file = "~" + newFile
    return file


def AssignOutputFolder(parameters, folder):
    parameters.itemByName("nc_program_output_folder").value.value = folder
    result = parameters.itemByName("nc_program_output_folder").value.value
    if result != folder and folder[0:2] == "\\\\":
        parameters.itemByName("nc_program_output_folder").value.value = "\\\\" + folder    # double up leading "\"
    return None


def _is_setup_cam_object(obj, cam=None):
    if obj is None:
        return False
    obj_type = getattr(obj, "objectType", None)
    if isinstance(obj_type, str) and obj_type:
        if obj_type == "adsk::cam::Setup":
            return True
        if not obj_type.startswith("adsk::cam::"):
            return False
    if hasattr(adsk.cam, "Setup") and isinstance(obj, adsk.cam.Setup):
        return True
    if hasattr(obj, "operationType"):
        return True
    if cam and hasattr(cam, "setups"):
        for s in cam.setups:
            if s == obj or (getattr(s, "name", None) is not None and getattr(s, "name", None) == getattr(obj, "name", None)):
                return True
    return False


def _is_folder_cam_object(obj):
    if obj is None:
        return False
    if hasattr(adsk.cam, "CAMFolder") and isinstance(obj, adsk.cam.CAMFolder):
        return True
    if hasattr(adsk.cam, "CAMPattern") and isinstance(obj, adsk.cam.CAMPattern):
        return True
    if getattr(obj, "objectType", "") in ("adsk::cam::CAMFolder", "adsk::cam::CAMPattern"):
        return True
    if hasattr(obj, "folders") and not hasattr(obj, "operationType"):
        return True
    return False


def _extract_cam_collection(collection):
    """Safely extract items from a Fusion CAM collection, list, tuple, or iterable."""
    if collection is None:
        return []
    if isinstance(collection, (list, tuple)):
        return list(collection)
    cnt = getattr(collection, "count", None)
    if isinstance(cnt, int):
        return [collection.item(i) for i in range(cnt)]
    if hasattr(collection, "__iter__"):
        return list(collection)
    return []


def _resolve_setup_for_object(obj, cam=None):
    """Find the owning adsk.cam.Setup for a Setup, Folder, or Operation."""
    if obj is None:
        return None
    if _is_setup_cam_object(obj, cam):
        if cam and hasattr(cam, "setups"):
            for s in cam.setups:
                if s == obj or getattr(s, "name", None) == getattr(obj, "name", None):
                    return s
        return obj

    # Walk up parent chain
    curr = getattr(obj, "parent", None)
    while curr is not None:
        if _is_setup_cam_object(curr, cam):
            if cam and hasattr(cam, "setups"):
                for s in cam.setups:
                    if s == curr or getattr(s, "name", None) == getattr(curr, "name", None):
                        return s
            return curr
        curr = getattr(curr, "parent", None)

    # Search in cam.setups by checking operations
    if cam and hasattr(cam, "setups"):
        for s in cam.setups:
            all_ops = getattr(s, "allOperations", [])
            op_list = _extract_cam_collection(all_ops)
            for op in op_list:
                if op == obj:
                    return s
                if (getattr(op, "operationId", -1) != -1 and 
                    getattr(op, "operationId", -1) == getattr(obj, "operationId", -2)):
                    return s
                if (getattr(op, "name", None) is not None and 
                    getattr(op, "name", None) == getattr(obj, "name", None)):
                    return s
    return None


class SetupOperationsMap(dict):
    """Dictionary mapping Setup to operations list, safe for unhashable adsk.cam.Setup objects."""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.empty_or_invalid_ops = []

    def __setitem__(self, key, value):
        k = getattr(key, "name", key) if not isinstance(key, str) else key
        super().__setitem__(k, value)

    def __getitem__(self, key):
        k = getattr(key, "name", key) if not isinstance(key, str) else key
        return super().__getitem__(k)

    def get(self, key, default=None):
        k = getattr(key, "name", key) if not isinstance(key, str) else key
        return super().get(k, default)

    def __contains__(self, key):
        k = getattr(key, "name", key) if not isinstance(key, str) else key
        return super().__contains__(k)


class SetupListDefaultDict(collections.defaultdict):
    """defaultdict(list) keyed by Setup or setup name, safe for unhashable adsk.cam.Setup objects."""
    def __init__(self):
        super().__init__(list)

    def __setitem__(self, key, value):
        k = getattr(key, "name", key) if not isinstance(key, str) else key
        super().__setitem__(k, value)

    def __getitem__(self, key):
        k = getattr(key, "name", key) if not isinstance(key, str) else key
        return super().__getitem__(k)

    def get(self, key, default=None):
        k = getattr(key, "name", key) if not isinstance(key, str) else key
        return super().get(k, default)

    def __contains__(self, key):
        k = getattr(key, "name", key) if not isinstance(key, str) else key
        return super().__contains__(k)


def _lookup_setup_ops(setup, setup_operations_map):
    """Retrieve operations list for setup from setup_operations_map regardless of wrapper identity."""
    if setup is None or setup_operations_map is None:
        return None
    name = getattr(setup, "name", None)
    if name is not None and name in setup_operations_map:
        return setup_operations_map[name]
    if hasattr(setup_operations_map, "get"):
        return setup_operations_map.get(setup, None)
    return None


def _check_operation_toolpath_status(op, cam=None):
    """
    Check if a CAM operation has an empty or invalid toolpath.

    Returns:
        (is_empty_or_invalid: bool, reason: str)
        where reason is 'Empty toolpath', 'Invalid toolpath', or '' if valid.
    """
    if getattr(op, "isSuppressed", False):
        return False, ""

    # 1. Check hasToolpath
    has_tp = getattr(op, "hasToolpath", None)
    if has_tp is not None:
        try:
            if not bool(has_tp):
                return True, "Empty toolpath"
        except Exception:
            pass

    # 2. Check operationState if available
    op_state = getattr(op, "operationState", None)
    if op_state is not None:
        try:
            val = getattr(op_state, "value", op_state)
            if val == 3:
                return True, "Empty toolpath"
            if val == 1:
                return True, "Invalid toolpath"
            cam_mod = sys.modules.get("adsk.cam")
            states_enum = getattr(cam_mod, "OperationStates", None)
            if states_enum is not None:
                if op_state == getattr(states_enum, "NoToolpathOperationState", None):
                    return True, "Empty toolpath"
                if op_state == getattr(states_enum, "IsInvalidOperationState", None):
                    return True, "Invalid toolpath"
        except Exception:
            pass

    # 3. Check isToolpathValid
    tp_valid = getattr(op, "isToolpathValid", None)
    if tp_valid is not None:
        try:
            if not bool(tp_valid):
                return True, "Invalid toolpath"
        except Exception:
            pass

    # 4. Check cam.checkToolpath(op) if available
    if cam is not None and hasattr(cam, "checkToolpath"):
        try:
            if not cam.checkToolpath(op):
                if has_tp is False:
                    return True, "Empty toolpath"
                return True, "Invalid toolpath"
        except Exception:
            pass

    return False, ""


def GetNcProgramSetupsAndOperations(cam, settings, setups=None, program=None):
    """
    Look in the selected NCProgram's setup and operations and derive the setups and operations to process.
    Returns:
        target_setups: list of adsk.cam.Setup objects
        setup_operations_map: SetupOperationsMap mapping Setup -> list[Operation]
    """
    if program is None and cam is not None and hasattr(cam, "ncPrograms"):
        program = GetNcProgram(cam, settings)

    whole_setups = []
    selected_ops_by_setup = SetupListDefaultDict()
    nc_prog_setups = []

    def add_setup_ref(s):
        if s and not any(s == existing or getattr(s, "name", None) == getattr(existing, "name", None) for existing in nc_prog_setups):
            nc_prog_setups.append(s)

    # 1. Check if program has direct setups collection
    if program is not None and hasattr(program, "setups"):
        try:
            items = _extract_cam_collection(program.setups)
            for s in items:
                resolved_s = _resolve_setup_for_object(s, cam)
                if resolved_s:
                    add_setup_ref(resolved_s)
                    if not any(resolved_s == ws or getattr(resolved_s, "name", None) == getattr(ws, "name", None) for ws in whole_setups):
                        whole_setups.append(resolved_s)
        except Exception as exc:
            _dbg_global(f"GetNcProgramSetupsAndOperations: error inspecting program.setups: {exc}")

    # 2. Check program.operations collection
    if program is not None and hasattr(program, "operations"):
        try:
            op_items = _extract_cam_collection(program.operations)
            for item in op_items:
                if _is_setup_cam_object(item, cam):
                    resolved_s = _resolve_setup_for_object(item, cam)
                    if resolved_s:
                        add_setup_ref(resolved_s)
                        if not any(resolved_s == ws or getattr(resolved_s, "name", None) == getattr(ws, "name", None) for ws in whole_setups):
                            whole_setups.append(resolved_s)
                elif _is_folder_cam_object(item):
                    resolved_s = _resolve_setup_for_object(item, cam)
                    if resolved_s:
                        add_setup_ref(resolved_s)
                        f_ops = getattr(item, "allOperations", getattr(item, "operations", []))
                        for f_op in _extract_cam_collection(f_ops):
                            selected_ops_by_setup[resolved_s].append(f_op)
                else:
                    # Individual Operation
                    resolved_s = _resolve_setup_for_object(item, cam)
                    if resolved_s:
                        add_setup_ref(resolved_s)
                        selected_ops_by_setup[resolved_s].append(item)
        except Exception as exc:
            _dbg_global(f"GetNcProgramSetupsAndOperations: error inspecting program.operations: {exc}")

    # Fallback if NCProgram specifies no setups/operations
    if not nc_prog_setups:
        if cam and hasattr(cam, "setups"):
            for s in cam.setups:
                add_setup_ref(s)
                if not any(s == ws or getattr(s, "name", None) == getattr(ws, "name", None) for ws in whole_setups):
                    whole_setups.append(s)

    target_setups = list(nc_prog_setups)

    setup_operations_map = SetupOperationsMap()
    all_empty_or_invalid = []

    for s in target_setups:
        all_s_ops = getattr(s, "allOperations", [])
        all_s_ops_list = _extract_cam_collection(all_s_ops)

        is_whole = any(
            s == ws or getattr(s, "name", None) == getattr(ws, "name", None)
            for ws in whole_setups
        )
        if is_whole:
            cand_ops = [
                op for op in all_s_ops_list
                if not getattr(op, "isSuppressed", False)
            ]
        else:
            chosen_ops = selected_ops_by_setup.get(s, [])
            cand_ops = []
            for op in all_s_ops_list:
                is_match = any(
                    op == c_op or
                    (getattr(op, "operationId", -1) != -1 and getattr(op, "operationId", -1) == getattr(c_op, "operationId", -2)) or
                    (getattr(op, "name", None) is not None and getattr(op, "name", None) == getattr(c_op, "name", None))
                    for c_op in chosen_ops
                )
                if is_match and not getattr(op, "isSuppressed", False):
                    cand_ops.append(op)

        valid_ops = []
        for op in cand_ops:
            is_bad, reason = _check_operation_toolpath_status(op, cam=cam)
            if is_bad:
                all_empty_or_invalid.append((s, op, reason))
            else:
                valid_ops.append(op)

        setup_operations_map[s] = valid_ops

    setup_operations_map.empty_or_invalid_ops = all_empty_or_invalid
    return target_setups, setup_operations_map


def GetNcProgram(cam, settings):
    if not cam or not hasattr(cam, "ncPrograms"):
        return None
    target = None
    prog_name = settings.get("ncProgram") if settings else None
    if not prog_name and settings and isinstance(settings.get("ncPrograms"), (list, tuple)) and settings["ncPrograms"]:
        prog_name = settings["ncPrograms"][0]
    if prog_name:
        for program in cam.ncPrograms:
            if program.name == prog_name:
                target = program
                break
    if target is None and cam.ncPrograms.count > 0:
        target = cam.ncPrograms.item(0)
    return target


def GetSelectedNcPrograms(cam, settings) -> List[Any]:
    """Resolve the list of selected NCProgram objects from settings (with legacy ncProgram fallback)."""
    if not cam or not hasattr(cam, "ncPrograms"):
        return []
    all_progs = _extract_cam_collection(cam.ncPrograms)
    selected: List[Any] = []

    prog_names = settings.get("ncPrograms") if settings else None
    if isinstance(prog_names, (list, tuple)) and len(prog_names) > 0:
        for name in prog_names:
            for p in all_progs:
                if getattr(p, "name", None) == name and p not in selected:
                    selected.append(p)
                    break

    if not selected and settings and settings.get("ncProgram"):
        target = GetNcProgram(cam, settings)
        if target is not None:
            selected.append(target)

    if not selected:
        target = GetNcProgram(cam, settings)
        if target is not None:
            selected.append(target)
        elif all_progs:
            selected.append(all_progs[0])

    return selected


def _ResolveNcProgramOutputFilename(program, setup=None, fileExt: str = ".nc") -> str:
    """
    Resolve output base filename from program's nc_program_filename parameter,
    falling back to program.name then setup.name only when missing or whitespace.
    Strips trailing fileExt (case-insensitive) while preserving ':' subfolder syntax.
    """
    raw_name = ""
    if program is not None and hasattr(program, "parameters") and program.parameters is not None:
        try:
            params = program.parameters
            p_fn = params.itemByName("nc_program_filename") if hasattr(params, "itemByName") else params.get("nc_program_filename")
            if p_fn is not None:
                val = _extract_choice_value(p_fn)
                if val and val.strip():
                    raw_name = val.strip()
        except Exception:
            pass

    if not raw_name and program is not None:
        p_name = getattr(program, "name", None)
        if isinstance(p_name, str) and p_name.strip():
            raw_name = p_name.strip()

    if not raw_name and setup is not None:
        s_name = getattr(setup, "name", None)
        if isinstance(s_name, str) and s_name.strip():
            raw_name = s_name.strip()

    if not raw_name:
        raw_name = "unnamed"

    norm_ext = _NormalizeFileExtension(fileExt) if fileExt else ""
    if norm_ext and raw_name.lower().endswith(norm_ext.lower()):
        stripped = raw_name[:-len(norm_ext)].strip()
        if stripped:
            raw_name = stripped

    return raw_name


def _is_nc_program_object(obj, cam=None) -> bool:
    if obj is None:
        return False
    obj_type = getattr(obj, "objectType", None)
    if isinstance(obj_type, str) and obj_type:
        if obj_type == "adsk::cam::NCProgram":
            return True
        if not obj_type.startswith("adsk::cam::"):
            return False
    if hasattr(adsk.cam, "NCProgram") and isinstance(obj, adsk.cam.NCProgram):
        return True
    if cam and hasattr(cam, "ncPrograms"):
        for p in _extract_cam_collection(cam.ncPrograms):
            if p == obj:
                return True
            p_tok = getattr(p, "entityToken", None)
            o_tok = getattr(obj, "entityToken", None)
            if isinstance(p_tok, str) and p_tok and p_tok == o_tok:
                return True
    return False


def _is_cam_context_entity(obj, cam=None) -> bool:
    if obj is None:
        return False
    obj_type = getattr(obj, "objectType", None)
    if isinstance(obj_type, str) and obj_type:
        return obj_type.startswith("adsk::cam::")
    obj_name = getattr(obj, "name", None)
    if isinstance(obj_name, str) and obj_name in ("NC Programs", "Setups"):
        return True
    if _is_nc_program_object(obj, cam) or _is_setup_cam_object(obj, cam) or _is_folder_cam_object(obj):
        return True
    return False


def _ResolveNcProgramNamesFromEntities(cam, entities) -> List[str]:
    """Resolve matching NCProgram names from browser/context selected CAM entities."""
    if not cam or not hasattr(cam, "ncPrograms") or not entities:
        return []
    all_progs = _extract_cam_collection(cam.ncPrograms)
    if not all_progs:
        return []

    matched_names: List[str] = []

    def _add_name(name):
        if isinstance(name, str) and name and name not in matched_names:
            matched_names.append(name)

    selected_setups = []
    for ent in entities:
        if ent is None:
            continue
        obj_type = getattr(ent, "objectType", None)
        obj_type_str = obj_type if isinstance(obj_type, str) else ""
        ent_name = getattr(ent, "name", None)
        ent_name_str = ent_name if isinstance(ent_name, str) else ""

        if _is_nc_program_object(ent, cam):
            for p in all_progs:
                p_name = getattr(p, "name", None)
                if p == ent or (isinstance(p_name, str) and p_name == ent_name_str):
                    _add_name(p_name)
                    break
        elif ("NCPrograms" in obj_type_str) or ent_name_str == "NC Programs":
            for p in all_progs:
                _add_name(getattr(p, "name", None))
        elif ("Setups" in obj_type_str) or ent_name_str == "Setups":
            for p in all_progs:
                _add_name(getattr(p, "name", None))
        elif _is_setup_cam_object(ent, cam):
            selected_setups.append(ent)

    if selected_setups:
        for p in all_progs:
            try:
                p_setups, _ = GetNcProgramSetupsAndOperations(cam, {}, program=p)
            except Exception:
                p_setups = []
            for ps in p_setups:
                ps_name = getattr(ps, "name", None)
                if any(
                    ps == s
                    or (isinstance(ps_name, str) and ps_name and ps_name == getattr(s, "name", None))
                    for s in selected_setups
                ):
                    _add_name(getattr(p, "name", None))
                    break

    return matched_names


def _ResolvePreselectedNcProgramNames(cam, ui) -> List[str]:
    """Resolve initially checked NC Program names from transient context-menu selection or ui.activeSelections."""
    global _pending_context_selection
    all_progs = _extract_cam_collection(getattr(cam, "ncPrograms", None)) if cam else []
    valid_names = {getattr(p, "name", None) for p in all_progs if isinstance(getattr(p, "name", None), str)}

    if _pending_context_selection is not None:
        ts, names = _pending_context_selection
        _pending_context_selection = None
        if (time.time() - ts) < 3.0 and names:
            filtered = [n for n in names if n in valid_names]
            if filtered:
                return filtered

    if ui and hasattr(ui, "activeSelections") and ui.activeSelections is not None:
        try:
            sels = _extract_cam_collection(ui.activeSelections)
            entities = [getattr(sel, "entity", None) for sel in sels if getattr(sel, "entity", None) is not None]
            if entities:
                matched = _ResolveNcProgramNamesFromEntities(cam, entities)
                if matched:
                    return matched
        except Exception:
            pass

    return []


def _GetRootCommandInputs(eventArgs):
    """Resolve the top-level CommandInputs even when eventArgs.inputs is a child group or table collection."""
    candidates = [
        getattr(getattr(getattr(eventArgs, "firingEvent", None), "sender", None), "commandInputs", None),
        getattr(getattr(getattr(eventArgs, "input", None), "parentCommand", None), "commandInputs", None),
        getattr(eventArgs, "inputs", None),
    ]
    for c in candidates:
        if c is not None:
            try:
                tbl = c.itemById("ncProgramsTable")
                if tbl is not None and getattr(tbl, "id", None) == "ncProgramsTable":
                    return c
            except Exception:
                pass
    return getattr(eventArgs, "inputs", None)


def _TableHasSelectAllHeader(tbl) -> bool:
    """Return True if row 0, col 0 of tbl holds the ncprog_select_all header checkbox."""
    if tbl is None or not hasattr(tbl, "getInputAtPosition"):
        return False
    try:
        cell0 = tbl.getInputAtPosition(0, 0)
        return getattr(cell0, "id", None) == "ncprog_select_all"
    except Exception:
        return False


def _GetNcProgramRowInputs(inputs, tbl, idx: int, row_offset: int = 0):
    """Look up (chk_inp, name_inp) for data index idx via itemById with tbl.getInputAtPosition fallback."""
    expected_id = f"ncprog_chk_{idx}"
    tbl_row = idx + row_offset
    chk = inputs.itemById(expected_id) if inputs is not None else None
    if (chk is None or getattr(chk, "id", None) != expected_id) and tbl is not None and hasattr(tbl, "getInputAtPosition"):
        try:
            chk = tbl.getInputAtPosition(tbl_row, 0)
        except Exception:
            chk = None
    if chk is None or getattr(chk, "id", None) != expected_id:
        return None, None
    name_id = f"ncprog_name_{idx}"
    name_inp = inputs.itemById(name_id) if inputs is not None else None
    if name_inp is None and tbl is not None and hasattr(tbl, "getInputAtPosition"):
        try:
            name_inp = tbl.getInputAtPosition(tbl_row, 1)
        except Exception:
            name_inp = None
    return chk, name_inp


def _CollectCheckedNcProgramsFromInputs(inputs) -> Tuple[List[str], int]:
    """Return (checked_program_names, total_rows) from the NC Programs TableCommandInput rows."""
    selected: List[str] = []
    total = 0
    if inputs is None:
        return selected, total
    tbl = inputs.itemById("ncProgramsTable")
    row_offset = 1 if _TableHasSelectAllHeader(tbl) else 0
    row_count = getattr(tbl, "rowCount", None) if tbl is not None else None
    indices = range(max(0, row_count - row_offset)) if isinstance(row_count, int) else range(500)
    for i in indices:
        chk, name_inp = _GetNcProgramRowInputs(inputs, tbl, i, row_offset=row_offset)
        if chk is None:
            break
        total += 1
        prog_name = getattr(name_inp, "value", "") if name_inp is not None else ""
        if getattr(chk, "value", False) and isinstance(prog_name, str) and prog_name:
            selected.append(prog_name)
    return selected, total


def _extract_choice_value(param) -> str:
    """Extract string value from a CAM ParameterValue, ChoiceParameterValue, or mock."""
    if param is None:
        return ""
    val = getattr(param, "value", param)
    if hasattr(val, "value"):
        val = getattr(val, "value", val)
    if isinstance(val, str):
        return val.strip()
    expr = getattr(param, "expression", None)
    if isinstance(expr, str) and expr.strip():
        return expr.strip()
    return str(val).strip() if val is not None else ""


def _ResolveCamOutputUnit(val):
    """
    Map string, int, or enum to adsk.cam.PostOutputUnitOptions enum value:
      MillimetersOutput = 2
      InchesOutput = 1
      DocumentUnitsOutput = 0
    """
    cam_mod = sys.modules.get("adsk.cam")
    enum_cls = getattr(cam_mod, "PostOutputUnitOptions", None)
    mm_opt = getattr(enum_cls, "MillimetersOutput", 2) if enum_cls else 2
    in_opt = getattr(enum_cls, "InchesOutput", 1) if enum_cls else 1
    doc_opt = getattr(enum_cls, "DocumentUnitsOutput", 0) if enum_cls else 0

    if val is None:
        return doc_opt

    if isinstance(val, (int, float)):
        int_val = int(val)
        if int_val == mm_opt:
            return mm_opt
        elif int_val == in_opt:
            return in_opt
        elif int_val == doc_opt:
            return doc_opt

    s = str(val).strip().lower()
    if s in ("millimeters", "millimeter", "mm", "millimetersoutput", "metric"):
        return mm_opt
    elif s in ("inches", "inch", "in", "inchesoutput", "imperial"):
        return in_opt
    elif s in ("document units", "documentunits", "documentunitsoutput", "document", "doc"):
        return doc_opt

    if enum_cls is not None and isinstance(val, enum_cls):
        return val

    return doc_opt


def _safe_str(val) -> str:
    """Safely return stripped string or empty string if not a string."""
    return val.strip() if isinstance(val, str) else ""


def _GetNCProgramOutputUnits(program, docSettings=None):
    """
    Resolve the CAM PostOutputUnitOptions enum value from the NCProgram or its parameters.
    Checks:
      1. program.parameters.itemByName("nc_program_unit") (standard Fusion CAM choice parameter: "Millimeters", "Inches", "Document units")
      2. program.outputUnits property (if present)
      3. program.postParameters ("nc_program_unit" or "unit")
      4. docSettings fallback ("nc_program_unit" or "outputUnits" or "unit")
    Returns:
      adsk.cam.PostOutputUnitOptions enum value (MillimetersOutput=2, InchesOutput=1, or DocumentUnitsOutput=0)
    """
    cam_mod = sys.modules.get("adsk.cam")
    enum_cls = getattr(cam_mod, "PostOutputUnitOptions", None)
    doc_opt = getattr(enum_cls, "DocumentUnitsOutput", 0) if enum_cls else 0

    if program is not None:
        # 1. Native NCProgram parameter: nc_program_unit
        if hasattr(program, "parameters"):
            try:
                params = program.parameters
                p = params.itemByName("nc_program_unit") if hasattr(params, "itemByName") else params.get("nc_program_unit")
                if p is not None:
                    raw_val = _extract_choice_value(p)
                    if raw_val:
                        return _ResolveCamOutputUnit(raw_val)
            except Exception:
                pass

        # 2. program.outputUnits property
        if hasattr(program, "outputUnits"):
            try:
                raw_val = getattr(program, "outputUnits")
                if raw_val is not None:
                    return _ResolveCamOutputUnit(raw_val)
            except Exception:
                pass

        # 3. program.postParameters
        if hasattr(program, "postParameters"):
            try:
                post_params = program.postParameters
                for k in ("nc_program_unit", "unit", "outputUnits"):
                    p = post_params.itemByName(k) if hasattr(post_params, "itemByName") else post_params.get(k)
                    if p is not None:
                        raw_val = _extract_choice_value(p)
                        if raw_val:
                            return _ResolveCamOutputUnit(raw_val)
            except Exception:
                pass

    # 4. docSettings fallback
    if docSettings and isinstance(docSettings, dict):
        for k in ("nc_program_unit", "outputUnits", "unit"):
            if k in docSettings and docSettings[k] is not None:
                return _ResolveCamOutputUnit(docSettings[k])

    return doc_opt


# Event handler for right-click markingMenuDisplaying event.
def _FindPostProcessAnchorControlId(controls) -> Optional[str]:
    """Locate Fusion's built-in Post Process control ID in a marking menu control list."""
    if controls is None:
        return None
    known_ids = (
        constPostProcessControlId,
        "IronNCProgramPostProcess",
        "CAMPostProcess",
        "PostProcess",
        "PostProcessCommand",
    )
    for cid in known_ids:
        try:
            ctrl = controls.itemById(cid)
            if ctrl is not None and isinstance(getattr(ctrl, "id", None), str) and ctrl.id == cid:
                return cid
        except Exception:
            pass

    all_ctrls = _extract_cam_collection(controls)
    for ctrl in all_ctrls:
        ctrl_id = getattr(ctrl, "id", None)
        if ctrl_id == constCmdDefId:
            continue
        cmd_def = getattr(ctrl, "commandDefinition", None)
        cmd_name = getattr(cmd_def, "name", "") if cmd_def is not None else ""
        if isinstance(cmd_name, str) and cmd_name.strip().lower() == "post process":
            if isinstance(ctrl_id, str) and ctrl_id:
                return ctrl_id

    for ctrl in all_ctrls:
        ctrl_id = getattr(ctrl, "id", None)
        if ctrl_id == constCmdDefId:
            continue
        if isinstance(ctrl_id, str) and "postprocess" in ctrl_id.lower():
            return ctrl_id

    return None


class MarkingMenuHandler(_MarkingMenuEventHandlerBase):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        global _pending_context_selection
        try:
            eventArgs = args
            if "adsk.core" in sys.modules and hasattr(adsk.core, "MarkingMenuEventArgs"):
                try:
                    casted = adsk.core.MarkingMenuEventArgs.cast(args)
                    if casted:
                        eventArgs = casted
                except Exception:
                    pass

            app = adsk.core.Application.get()
            ui = getattr(app, "userInterface", None) if app else None
            doc = getattr(app, "activeDocument", None) if app else None
            if not ui:
                return

            sel_ents = _extract_cam_collection(getattr(eventArgs, "selectedEntities", None))
            cam = None
            if doc and hasattr(doc, "products") and doc.products:
                try:
                    cam = adsk.cam.CAM.cast(doc.products.itemByProductType(constCAMProductId))
                except Exception:
                    cam = None

            has_cam_entity = any(_is_cam_context_entity(ent, cam) for ent in sel_ents)
            if sel_ents and not has_cam_entity:
                return

            ws_id = getattr(getattr(ui, "activeWorkspace", None), "id", None)
            act_prod = getattr(app, "activeProduct", None)
            in_cam_context = (
                ws_id == constCAMWorkspaceId
                or getattr(act_prod, "productType", None) == constCAMProductId
                or has_cam_entity
            )
            if not in_cam_context:
                return

            matched_names = _ResolveNcProgramNamesFromEntities(cam, sel_ents)
            if matched_names:
                _pending_context_selection = (time.time(), matched_names)
            else:
                _pending_context_selection = None

            linear_menu = getattr(eventArgs, "linearMarkingMenu", None)
            controls = getattr(linear_menu, "controls", None) if linear_menu else None
            cmd_defs = getattr(ui, "commandDefinitions", None)
            cmd_def = cmd_defs.itemById(constCmdDefId) if cmd_defs else None
            if controls is not None and cmd_def is not None:
                try:
                    existing_ctrl = controls.itemById(constCmdDefId)
                    if existing_ctrl is not None and getattr(existing_ctrl, "id", None) == constCmdDefId:
                        existing_ctrl.deleteMe()
                except Exception:
                    pass
                anchor_id = _FindPostProcessAnchorControlId(controls)
                if anchor_id:
                    controls.addCommand(cmd_def, anchor_id, False)
                else:
                    controls.addCommand(cmd_def)
        except Exception as exc:
            _dbg_global(f"MarkingMenuHandler exception: {exc}")


# Event handler for the commandCreated event.
class CommandEventHandler(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            eventArgs = adsk.core.CommandCreatedEventArgs.cast(args)
            cmd = eventArgs.command

            # Get document attributes that will set initial values
            app = adsk.core.Application.get()
            ui = app.userInterface
            cam = adsk.cam.CAM.cast(app.activeDocument.products.itemByProductType(constCAMProductId))
            docSettings = settingsMgr.GetSettings(app.activeDocument.attributes)

            # Get the NCPrograms
            programs = cam.ncPrograms
            if programs.count == 0:
                ncInput = programs.createInput()
                ncInput.displayName = constNcProgramName
                program = programs.add(ncInput)
                program.postConfiguration = program.postConfiguration
                outputFolder = docSettings["output"]
                program.attributes.add(constAttrGroup, constAttrCompressedName, outputFolder)
                AssignOutputFolder(program.parameters, ExpandFileName(outputFolder))
                program.parameters.itemByName("nc_program_createInBrowser").value.value = True

            all_progs = _extract_cam_collection(programs)
            all_prog_names = [p.name for p in all_progs if getattr(p, "name", None)]
            valid_name_set = set(all_prog_names)

            # Determine initial checked NC Programs: live context/selection first, then saved settings, else first program
            preselected = [n for n in _ResolvePreselectedNcProgramNames(cam, ui) if n in valid_name_set]
            if preselected:
                checked_names = preselected
            else:
                saved_progs = docSettings.get("ncPrograms")
                if isinstance(saved_progs, (list, tuple)) and saved_progs:
                    checked_names = [n for n in saved_progs if n in valid_name_set]
                elif docSettings.get("ncProgram") in valid_name_set:
                    checked_names = [docSettings["ncProgram"]]
                else:
                    checked_names = []
                if not checked_names and all_prog_names:
                    checked_names = [all_prog_names[0]]

            checked_set = set(checked_names)
            ordered_checked = [n for n in all_prog_names if n in checked_set]
            docSettings["ncPrograms"] = ordered_checked
            docSettings["ncProgram"] = ordered_checked[0] if ordered_checked else ""

            # Connect to the execute event.
            onExecute = CommandExecuteHandler(docSettings)
            cmd.execute.add(onExecute)
            handlers.append(onExecute)

            # Add inputs that will appear in a dialog
            inputs = cmd.commandInputs

            tableInput = inputs.addTableCommandInput("ncProgramsTable", "NC Programs", 3, "1:5:4")
            tableInput.minimumVisibleRows = min(4, max(2, programs.count + 1))
            tableInput.maximumVisibleRows = 9
            tableInput.hasGrid = False

            # Row 0: Select All header aligned cleanly on top of column 0 checkboxes
            sel_all_inp = inputs.addBoolValueInput(
                "ncprog_select_all",
                "",
                True,
                "",
                len(ordered_checked) == len(all_prog_names) and len(all_prog_names) > 0,
            )
            sel_all_inp.tooltip = "Select or deselect all NC Programs"
            hdr_name_inp = inputs.addStringValueInput("ncprog_hdr_label", "", "Select All")
            hdr_name_inp.isReadOnly = True
            hdr_info_inp = inputs.addStringValueInput("ncprog_hdr_info", "", "")
            hdr_info_inp.isReadOnly = True
            tableInput.addCommandInput(sel_all_inp, 0, 0)
            tableInput.addCommandInput(hdr_name_inp, 0, 1)
            tableInput.addCommandInput(hdr_info_inp, 0, 2)

            for idx, prog in enumerate(all_progs):
                p_name = getattr(prog, "name", f"NC Program {idx + 1}")
                chk_inp = inputs.addBoolValueInput(f"ncprog_chk_{idx}", "", True, "", p_name in checked_set)
                name_inp = inputs.addStringValueInput(f"ncprog_name_{idx}", "", p_name)
                name_inp.isReadOnly = True

                # Build setup & output filename summary
                try:
                    p_setups, _ = GetNcProgramSetupsAndOperations(cam, docSettings, program=prog)
                except Exception:
                    p_setups = []

                if len(p_setups) > 1:
                    info_str = "⚠ Multiple setups"
                else:
                    setup_obj = p_setups[0] if len(p_setups) == 1 else None
                    setup_name = _safe_str(getattr(setup_obj, "name", None)) or "No setup"
                    raw_ext = ""
                    p_params = getattr(prog, "parameters", None)
                    if p_params is not None:
                        try:
                            ext_param = p_params.itemByName("nc_program_nc_extension")
                            if ext_param is not None:
                                raw_ext = _extract_choice_value(ext_param)
                        except Exception:
                            pass
                    file_ext = _NormalizeFileExtension(raw_ext) if raw_ext else ".nc"
                    raw_fname = _ResolveNcProgramOutputFilename(prog, setup_obj, file_ext)
                    _, leaf_fname = _ResolveSetupFolderAndFilename("", raw_fname)
                    info_str = f"{setup_name} -> {leaf_fname}{file_ext}"

                info_inp = inputs.addStringValueInput(f"ncprog_info_{idx}", "", info_str)
                info_inp.isReadOnly = True

                tableInput.addCommandInput(chk_inp, idx + 1, 0)
                tableInput.addCommandInput(name_inp, idx + 1, 1)
                tableInput.addCommandInput(info_inp, idx + 1, 2)

            # Optimization & Verification group
            inputGroup = inputs.addGroupCommandInput("groupPersonal", "Optimization & Verification")

            # check box to restore rapid moves
            input = inputGroup.children.addBoolValueInput("restore_rapid",
                                                          "Restore rapid moves (G0)",
                                                          True,
                                                          "",
                                                          docSettings.get("restore_rapid", True))
            input.tooltip = "Restore Safe Rapid Moves (G0)"
            input.tooltipDescription = (
                "Analyze toolpaths and restore safe rapid (G0) moves.<br>"
                "• <b>Turning:</b> Promotes collision-free air moves verified against dynamic 2D IPW simulation.<br>"
                "• <b>Milling:</b> Promotes upward vertical Z+ retract moves reaching or exceeding clearance height.<br>"
                "Restored moves are annotated with comments in the output program.")
           
            inputGroup.isExpanded = docSettings["groupPersonal"]

            # button to save default settings
            input = inputs.addBoolValueInput("save", "Save as default", False)
            input.resourceFolder = os.path.join(constAddonDir, "resources", "Save")
            input.tooltip = "Save These Settings as System Default"
            input.tooltipDescription = (
                "Save these settings to use as the default for each new design.")

            # text box for error messages
            input = inputs.addTextBoxCommandInput("error", "", "", 3, True)
            input.isFullWidth = True
            input.isVisible = False

            # Connect to the inputChanged event.
            onInputChanged = CommandInputChangedHandler(docSettings)
            cmd.inputChanged.add(onInputChanged)
            handlers.append(onInputChanged)

            # Connect to the validateInputs event.
            onValidateInputs = CommandValidateInputsHandler()
            cmd.validateInputs.add(onValidateInputs)
            handlers.append(onValidateInputs)
        except:
            ui = app.userInterface
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

# Event handler for the inputChanged event.
class CommandInputChangedHandler(adsk.core.InputChangedEventHandler):
    def __init__(self, docSettings):
        self.docSettings = docSettings
        self._updating_select_all = False
        super().__init__()

    def notify(self, args):
        app = adsk.core.Application.get()
        ui  = app.userInterface
        try:
            eventArgs = adsk.core.InputChangedEventArgs.cast(args)
            inputs = _GetRootCommandInputs(eventArgs)

            # See if button clicked
            input = eventArgs.input
            if input.id == "save":
                settingsMgr.SaveDefault(self.docSettings)

            elif input.id == "ncprog_select_all":
                if self._updating_select_all:
                    return
                self._updating_select_all = True
                try:
                    target_val = bool(input.value)
                    selected = []
                    if inputs is not None:
                        tbl = inputs.itemById("ncProgramsTable")
                        row_offset = 1 if _TableHasSelectAllHeader(tbl) else 0
                        row_count = getattr(tbl, "rowCount", None) if tbl is not None else None
                        indices = range(max(0, row_count - row_offset)) if isinstance(row_count, int) else range(500)
                        for i in indices:
                            chk, name_inp = _GetNcProgramRowInputs(inputs, tbl, i, row_offset=row_offset)
                            if chk is None:
                                break
                            chk.value = target_val
                            if target_val:
                                p_name = getattr(name_inp, "value", "") if name_inp is not None else ""
                                if isinstance(p_name, str) and p_name:
                                    selected.append(p_name)
                    self.docSettings["ncPrograms"] = selected
                    self.docSettings["ncProgram"] = selected[0] if selected else ""
                finally:
                    self._updating_select_all = False

            elif input.id.startswith("ncprog_chk_"):
                if self._updating_select_all:
                    return
                self._updating_select_all = True
                try:
                    selected, total = _CollectCheckedNcProgramsFromInputs(inputs)
                    sel_all = inputs.itemById("ncprog_select_all") if inputs is not None else None
                    if sel_all is None and inputs is not None:
                        tbl = inputs.itemById("ncProgramsTable")
                        if _TableHasSelectAllHeader(tbl):
                            sel_all = tbl.getInputAtPosition(0, 0)
                    if sel_all is not None and total > 0:
                        sel_all.value = (len(selected) == total)
                    self.docSettings["ncPrograms"] = selected
                    self.docSettings["ncProgram"] = selected[0] if selected else ""
                finally:
                    self._updating_select_all = False

            elif input.id in self.docSettings:
                if input.objectType == adsk.core.GroupCommandInput.classType():
                    self.docSettings[input.id] = input.isExpanded
                elif input.objectType == adsk.core.DropDownCommandInput.classType():
                    self.docSettings[input.id] = input.selectedItem.name
                else:
                    self.docSettings[input.id] = input.value

        except:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


# Event handler for the validateInputs event.
class CommandValidateInputsHandler(adsk.core.ValidateInputsEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        app = adsk.core.Application.get()
        ui  = app.userInterface

        try:
            eventArgs = adsk.core.ValidateInputsEventArgs.cast(args)
            inputs = _GetRootCommandInputs(eventArgs)
            selected, total = _CollectCheckedNcProgramsFromInputs(inputs)
            err_box = inputs.itemById("error") if inputs is not None else None
            if total > 0 and not selected:
                eventArgs.areInputsValid = False
                if err_box is not None:
                    err_box.text = "Select at least one NC Program."
                    err_box.isVisible = True
            else:
                eventArgs.areInputsValid = True
                if err_box is not None:
                    err_box.text = ""
                    err_box.isVisible = False

        except:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))


# Event handler for the execute event.
class CommandExecuteHandler(adsk.core.CommandEventHandler):
    def __init__(self, docSettings):
        self.docSettings = docSettings
        super().__init__()

    def notify(self, args):
        eventArgs = adsk.core.CommandEventArgs.cast(args)
        cmd = eventArgs.command
        inputs = cmd.commandInputs

        selected, total = _CollectCheckedNcProgramsFromInputs(inputs)
        if total > 0:
            self.docSettings["ncPrograms"] = selected
            self.docSettings["ncProgram"] = selected[0] if selected else ""

        # Code to react to the event.
        for key in list(self.docSettings.keys()):
            if key in ("ncPrograms", "ncProgram"):
                continue
            inp = inputs.itemById(key)
            if inp:
                if inp.objectType == adsk.core.GroupCommandInput.classType():
                    self.docSettings[key] = inp.isExpanded
                elif inp.objectType == adsk.core.DropDownCommandInput.classType():
                    if inp.selectedItem:
                        self.docSettings[key] = inp.selectedItem.name
                elif hasattr(inp, "value"):
                    self.docSettings[key] = inp.value
        _dbg_global(
            f"CommandExecuteHandler execute: ncPrograms={self.docSettings.get('ncPrograms')}, "
            f"restore_rapid={self.docSettings.get('restore_rapid')}"
        )
        PerformPostProcess(self.docSettings)



def PerformPostProcess(docSettings, setups=None):
    ui = None
    progress = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface
        doc = app.activeDocument
        cam = adsk.cam.CAM.cast(doc.products.itemByProductType(constCAMProductId))

        cntFiles = 0
        cntSkipped = 0
        lstSkipped = ""
        verifyResults.clear()
        _dbg_global(f"PerformPostProcess entered: incoming setups count={len(setups) if setups else 0}")

        global settingsMgr
        if settingsMgr is None:
            settingsMgr = SettingsManager()

        selected_programs = GetSelectedNcPrograms(cam, docSettings)
        if not selected_programs:
            fallback_prog = GetNcProgram(cam, docSettings)
            if fallback_prog is not None:
                selected_programs = [fallback_prog]

        if not selected_programs:
            err = "No NC Programs found to post process."
            _NotifyPostProcessResult(
                ui=ui,
                cntFiles=0,
                cntSkipped=1,
                lstSkipped=err,
                outputFolder="",
                verifyResults=verifyResults,
                is_fatal=True,
            )
            return err

        # Pre-flight across all selected NC Programs before starting progress dialog
        all_empty_invalid = []
        existing_files = []
        runnable_tasks = []
        program_snapshots = {}
        seen_targets = {}
        first_output_folder = ""
        all_output_folders = set()

        for prog_idx, program in enumerate(selected_programs):
            prog_name = _safe_str(getattr(program, "name", None)) or f"NC Program {prog_idx + 1}"
            parameters = getattr(program, "parameters", None)
            setups_override = setups if (setups is not None and len(selected_programs) == 1) else None
            p_setups, setup_operations_map = GetNcProgramSetupsAndOperations(cam, docSettings, setups_override, program)

            # Abort with error if multiple setups are detected in a single NC Program
            if len(p_setups) > 1:
                setup_names = ", ".join(f"'{getattr(s, 'name', 'Setup')}'" for s in p_setups)
                err = (
                    f"Multiple setups detected in NC Program '{prog_name}' ({setup_names}). "
                    f"Processing multiple setups in a single NC Program is not supported."
                )
                _dbg_global(f"PerformPostProcess: {err}")
                if ui:
                    ui.messageBox(
                        err,
                        constCmdName,
                        adsk.core.MessageBoxButtonTypes.OKButtonType,
                        adsk.core.MessageBoxIconTypes.CriticalIconType,
                    )
                return err

            # Normalize output folder for this user ("\" -> "/")
            raw_folder = ""
            if parameters is not None:
                try:
                    p_folder = parameters.itemByName("nc_program_output_folder")
                    raw_folder = _extract_choice_value(p_folder)
                except Exception:
                    raw_folder = ""
            outputFolder = raw_folder.replace("\\", "/")
            # Keep leading "\\" for file share
            if outputFolder[0:2] == "//":
                outputFolder = "\\\\" + outputFolder[2:]
            try:
                pathlib.Path(outputFolder).mkdir(parents=True, exist_ok=True)
            except Exception:
                # See if we can map it to folder with compressed user
                try:
                    attr = program.attributes.itemByName(constAttrGroup, constAttrCompressedName)
                    if attr and attr.value:
                        compressedName = attr.value
                        if compressedName and compressedName[0] == "~" and compressedName[1:] == outputFolder[-(len(compressedName) - 1):]:
                            outputFolder = ExpandFileName(compressedName)
                except Exception:
                    pass
                try:
                    pathlib.Path(outputFolder).mkdir(parents=True, exist_ok=True)
                except Exception:
                    pass

            if not first_output_folder:
                first_output_folder = outputFolder
            if outputFolder:
                all_output_folders.add(outputFolder)

            compressedName = CompressFileName(outputFolder)
            try:
                program.attributes.add(constAttrGroup, constAttrCompressedName, compressedName)
            except Exception:
                pass
            docSettings["output"] = compressedName

            empty_invalid = getattr(setup_operations_map, "empty_or_invalid_ops", [])
            if empty_invalid:
                all_empty_invalid.extend(empty_invalid)

            # Snapshot original NCProgram parameters so the user's NCProgram is never left mutated
            orig_prog_params = {}
            if parameters is not None:
                for p_key in (
                    "nc_program_name",
                    "nc_program_filename",
                    "nc_program_openInEditor",
                    "nc_program_nc_extension",
                    "nc_program_output_folder",
                ):
                    try:
                        p_item = parameters.itemByName(p_key)
                        if p_item is not None:
                            orig_prog_params[p_key] = p_item.value.value
                    except Exception:
                        pass
            program_snapshots[id(program)] = (program, parameters, outputFolder, orig_prog_params)

            if len(p_setups) != 0 and cam.allOperations.count != 0:
                raw_ext = ""
                if parameters is not None:
                    try:
                        ext_param = parameters.itemByName("nc_program_nc_extension")
                        if ext_param is not None:
                            raw_ext = _extract_choice_value(ext_param)
                    except Exception:
                        pass
                fileExt = _NormalizeFileExtension(raw_ext)

                for setup in p_setups:
                    target_ops = _lookup_setup_ops(setup, setup_operations_map)
                    if not target_ops or getattr(setup, "isSuppressed", False):
                        continue
                    if getattr(setup, "allOperations", None) is not None and getattr(setup.allOperations, "count", 1) == 0:
                        continue

                    raw_fname = _ResolveNcProgramOutputFilename(program, setup, fileExt)
                    sFolder, sFname = _ResolveSetupFolderAndFilename(outputFolder, raw_fname)
                    target_path = os.path.join(sFolder, sFname + fileExt)
                    abs_target = os.path.abspath(target_path)

                    if abs_target in seen_targets:
                        other_prog = seen_targets[abs_target]
                        err = (
                            f"Duplicate output filename detected: NC Programs '{other_prog}' and '{prog_name}' "
                            f"both target '{abs_target}'. Please give each NC Program a unique filename."
                        )
                        _dbg_global(f"PerformPostProcess: {err}")
                        if ui:
                            ui.messageBox(
                                err,
                                constCmdName,
                                adsk.core.MessageBoxButtonTypes.OKButtonType,
                                adsk.core.MessageBoxIconTypes.CriticalIconType,
                            )
                        return err

                    seen_targets[abs_target] = prog_name
                    if os.path.isfile(target_path):
                        existing_files.append(target_path)

                    runnable_tasks.append({
                        "program": program,
                        "prog_name": prog_name,
                        "parameters": parameters,
                        "setup": setup,
                        "target_ops": target_ops,
                        "setupFolder": sFolder,
                        "fname": sFname,
                        "outputFolder": outputFolder,
                        "target_path": target_path,
                    })

        # Save settings in document attributes
        settingsMgr.SaveSettings(doc.attributes, docSettings)

        # Prompt once for any empty/invalid toolpaths across all selected NC Programs
        if all_empty_invalid and ui:
            lines = []
            for s, op, reason in all_empty_invalid:
                s_name = getattr(s, "name", "Setup")
                op_name = getattr(op, "name", "Operation")
                lines.append(f"• Setup '{s_name}' -> Operation '{op_name}' ({reason})")

            op_list_str = "\n".join(lines)
            prompt = (
                f"The following operation(s) have empty or invalid toolpaths:\n\n"
                f"{op_list_str}\n\n"
                f"Do you want to proceed?"
            )
            res = ui.messageBox(
                prompt,
                constCmdName,
                adsk.core.MessageBoxButtonTypes.YesNoButtonType,
                adsk.core.MessageBoxIconTypes.WarningIconType
            )
            if res != adsk.core.DialogResults.DialogYes:
                _dbg_global("PerformPostProcess: user chose not to proceed with empty/invalid toolpaths; aborting.")
                return  # abort!

        # Prompt once for any existing target output files across all selected NC Programs
        if existing_files and ui:
            if len(existing_files) == 1:
                target = existing_files[0]
                prompt = (
                    f"The output file already exists:\n\n"
                    f"{target}\n\n"
                    f"Do you want to overwrite it?"
                )
            else:
                file_lines = "\n".join(f"• {f}" for f in existing_files)
                prompt = (
                    f"The following output file(s) already exist:\n\n"
                    f"{file_lines}\n\n"
                    f"Do you want to overwrite them?"
                )
            res = ui.messageBox(
                prompt,
                constCmdName,
                adsk.core.MessageBoxButtonTypes.YesNoButtonType,
                adsk.core.MessageBoxIconTypes.QuestionIconType
            )
            if res != adsk.core.DialogResults.DialogYes:
                _dbg_global("PerformPostProcess: user chose not to overwrite existing file; aborting.")
                return  # abort!

        program_outcomes = []
        if runnable_tasks:
            progress = ui.createProgressDialog() if (ui and hasattr(ui, "createProgressDialog")) else None
            total_tasks = max(1, len(runnable_tasks))
            if progress:
                progress.isCancelButtonShown = True
                progress.show("Post Processing...", "Initializing post processing... (%p%)", 0, 100)
                progress.progressValue = 0
                if "adsk" in sys.modules and hasattr(adsk, "doEvents"):
                    try:
                        adsk.doEvents()
                    except Exception:
                        pass

            def make_setup_progress_cb(t_idx, label):
                def _cb(step_msg, frac):
                    if progress:
                        if progress.wasCancelled:
                            return False
                        pct = int(((t_idx + max(0.0, min(1.0, frac))) / total_tasks) * 100)
                        progress.message = f"{label}: {step_msg} (%p%)"
                        progress.progressValue = max(0, min(100, pct))
                        if "adsk" in sys.modules and hasattr(adsk, "doEvents"):
                            try:
                                adsk.doEvents()
                            except Exception:
                                pass
                    return True
                _cb.progress_dialog = progress
                return _cb

            try:
                for task_idx, task in enumerate(runnable_tasks):
                    if progress and progress.wasCancelled:
                        break
                    program = task["program"]
                    prog_name = task["prog_name"]
                    setup = task["setup"]
                    target_ops = task["target_ops"]
                    setupFolder = task["setupFolder"]
                    fname = task["fname"]
                    label = f"{prog_name} / {setup.name}" if len(selected_programs) > 1 else setup.name

                    setup_cb = make_setup_progress_cb(task_idx, label)
                    setup_cb("Starting setup...", 0.0)

                    prev_len = len(verifyResults)
                    status = PostProcessSetup(
                        fname, setup, setupFolder, docSettings, program,
                        target_operations=target_ops, progress_cb=setup_cb
                    )
                    # Backfill nc_program on any verifyResult appended during this task
                    for vr in verifyResults[prev_len:]:
                        if not vr.get("nc_program"):
                            vr["nc_program"] = prog_name
                        if not vr.get("folder"):
                            vr["folder"] = task["outputFolder"]

                    if status is None:
                        cntFiles += 1
                        setup_cb("Completed.", 1.0)
                        if len(verifyResults) == prev_len:
                            program_outcomes.append({
                                "nc_program": prog_name,
                                "setup": setup.name,
                                "cnc": task["target_path"],
                                "folder": task["outputFolder"],
                                "status": "success",
                                "summary": "Completed",
                            })
                    elif "cancelled" in status.lower():
                        cntSkipped += 1
                        lstSkipped += f"\nCancelled by user on setup {setup.name}"
                        program_outcomes.append({
                            "nc_program": prog_name,
                            "setup": setup.name,
                            "cnc": task["target_path"],
                            "folder": task["outputFolder"],
                            "status": "warning",
                            "summary": status,
                        })
                        break
                    else:
                        cntSkipped += 1
                        lstSkipped += f"\nFailed on setup {setup.name}: {status}"
                        if len(verifyResults) == prev_len:
                            program_outcomes.append({
                                "nc_program": prog_name,
                                "setup": setup.name,
                                "cnc": task["target_path"],
                                "folder": task["outputFolder"],
                                "status": "error",
                                "summary": status,
                            })
            finally:
                if progress:
                    try:
                        progress.hide()
                    except Exception:
                        pass
                # Restore every program's output folder and original parameters
                for _, parameters, orig_folder, orig_prog_params in program_snapshots.values():
                    if parameters is not None:
                        try:
                            AssignOutputFolder(parameters, orig_folder)
                        except Exception:
                            pass
                        for p_key, p_val in orig_prog_params.items():
                            try:
                                p_item = parameters.itemByName(p_key)
                                if p_item is not None:
                                    p_item.value.value = p_val
                            except Exception:
                                pass

        shared_folder = first_output_folder if len(all_output_folders) <= 1 else ""
        # done with setups, report results non-modally
        _NotifyPostProcessResult(
            ui=ui,
            cntFiles=cntFiles,
            cntSkipped=cntSkipped,
            lstSkipped=lstSkipped,
            outputFolder=shared_folder or first_output_folder,
            verifyResults=verifyResults,
            program_outcomes=program_outcomes,
        )
        ShowVerifyResults(ui)

    except Exception as exc:
        if progress:
            progress.hide()
        err_msg = traceback.format_exc()
        _dbg_global("PerformPostProcess exception:\n" + err_msg)
        user_err = f"Post processing encountered an error: {exc}"
        _NotifyPostProcessResult(
            ui=ui,
            cntFiles=0,
            cntSkipped=1,
            lstSkipped=user_err,
            outputFolder=first_output_folder if 'first_output_folder' in locals() else "",
            verifyResults=verifyResults,
            is_fatal=True,
        )

# Per-run list of lathe verification summaries, shown in a modal dialog at the end
verifyResults = []


def _RecordVerifyResult(setup, cncPath, reportText, reportPath=None, pngPath=None, promoted=None, spikes=None, volume_mm3=None, collisions=None, error=None, warning=None, nc_program=None):
    basePath = os.path.splitext(cncPath)[0]
    if not reportPath:
        reportPath = basePath + "_report.md"
    if not pngPath:
        pngPath = basePath + ".png"
    try:
        with open(reportPath, "w", encoding="utf8") as f:
            f.write(reportText)
    except Exception:
        pass
    kinds = {}
    for c in (collisions or []):
        kind_str = c.kind.name if hasattr(c.kind, "name") else str(c.kind)
        kinds[kind_str] = kinds.get(kind_str, 0) + 1
    verifyResults.append({
        "nc_program": nc_program or "",
        "setup": setup.name,
        "cnc": cncPath,
        "png": pngPath,
        "report": reportPath,
        "promoted": promoted,
        "spikes": spikes,
        "volume_mm3": volume_mm3,
        "kinds": kinds,
        "error": error,
        "warning": warning,
    })
    try:
        col_count = len(collisions or [])
        adsk.core.Application.get().log(
            f"[PostProcessAll] Verified '{setup.name}': {promoted} rapid moves restored, {col_count} collision(s) | Report: {reportPath}"
        )
    except Exception:
        pass


_INTERNAL_HELPER_POSTS = (
    "universal_dumper",
    "universal_replayer",
)


def _IsInternalHelperPost(post_cfg) -> bool:
    """Return True if post_cfg refers to one of our internal wrapper/dumper/replayer .cps scripts."""
    if not post_cfg:
        return False
    parts = []
    for attr in ("description", "name", "postPath", "location", "extension"):
        val = getattr(post_cfg, attr, None)
        if val and isinstance(val, str):
            parts.append(val.lower())
    try:
        if hasattr(post_cfg, "url") and post_cfg.url:
            u_str = post_cfg.url.toString() if hasattr(post_cfg.url, "toString") else str(post_cfg.url)
            parts.append(u_str.lower())
    except Exception:
        pass
    joined = " | ".join(parts)
    if any(tag in joined for tag in _INTERNAL_HELPER_POSTS):
        return True
    if "context dumper" in joined or getattr(post_cfg, "extension", "").lower().lstrip(".") == "json":
        return True
    return False


def _GetPostSearchDirsByTier() -> Tuple[List[str], List[str], List[str]]:
    """
    Return post search directories partitioned by priority tier:
    Tier 1 (User / Cloud): Personal and cloud post libraries.
    Tier 2 (System): Autodesk installation system posts.
    Tier 3 (Add-on): Fallback add-in local directory.
    """
    user_dirs: List[str] = []
    system_dirs: List[str] = []
    addon_dirs: List[str] = []

    def _add(lst: List[str], path: Optional[str]):
        if not path:
            return
        norm = os.path.normpath(os.path.abspath(path))
        if os.path.isdir(norm) and norm not in lst:
            lst.append(norm)

    # --- Tier 1: User / Personal / Cloud Post Directories ---
    _add(user_dirs, os.path.expanduser("~/Library/Application Support/Autodesk/Fusion 360 CAM/Posts"))
    _add(user_dirs, os.path.expanduser("~/Library/Application Support/Autodesk/Autodesk Fusion 360/CAM/Posts"))
    _add(user_dirs, os.path.expanduser("~/Library/Application Support/Autodesk/Autodesk Fusion 360/CAM/cache/posts"))

    appdata = os.environ.get("APPDATA", "")
    if appdata:
        _add(user_dirs, os.path.join(appdata, "Autodesk", "Fusion 360 CAM", "Posts"))
        _add(user_dirs, os.path.join(appdata, "Autodesk", "Autodesk Fusion 360", "CAM", "Posts"))
    localappdata = os.environ.get("LOCALAPPDATA", "")
    if localappdata:
        _add(user_dirs, os.path.join(localappdata, "Autodesk", "Autodesk Fusion 360", "CAM", "cache", "posts"))

    for pattern in (
        os.path.expanduser("~/Library/Application Support/Autodesk/Autodesk Fusion 360/*/W.login/*/CAMPosts"),
        os.path.join(localappdata, "Autodesk", "Autodesk Fusion 360", "*", "W.login", "*", "CAMPosts") if localappdata else "",
    ):
        if pattern:
            for cd in glob.glob(pattern):
                _add(user_dirs, cd)

    # --- Tier 2: System Post Directories ---
    try:
        from pipeline import find_autodesk_system_posts_dir, find_autodesk_post_binary
        try:
            sys_posts_dir = find_autodesk_system_posts_dir()
            if sys_posts_dir:
                _add(system_dirs, sys_posts_dir)
        except Exception:
            pass
        try:
            post_b = find_autodesk_post_binary()
            if post_b:
                cam_dir = os.path.dirname(post_b)
                for sub in ("Data/Posts", "Posts"):
                    _add(system_dirs, os.path.join(cam_dir, sub))
        except Exception:
            pass
    except Exception as exc:
        _dbg_global(f"_GetPostSearchDirsByTier pipeline search exception: {exc}")

    # --- Tier 3: Add-on Directory Fallback ---
    _add(addon_dirs, constAddonDir)

    return user_dirs, system_dirs, addon_dirs


def _GetPostSearchDirs() -> List[str]:
    """Return all directories on this machine where Autodesk Fusion and user post processors live."""
    user_dirs, sys_dirs, addon_dirs = _GetPostSearchDirsByTier()
    combined = []
    for d in user_dirs + sys_dirs + addon_dirs:
        if d not in combined:
            combined.append(d)
    return combined


def _GetPostCapabilities(cps_path: str) -> Optional[Dict[str, bool]]:
    """Parse capabilities declared in a .cps post processor file (scans whole file)."""
    if not cps_path or not os.path.isfile(cps_path):
        return None
    try:
        with open(cps_path, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                m = re.search(r"\bcapabilities\s*=\s*([^;]+);", line)
                if m:
                    expr = m.group(1)
                    return {
                        "turning": "CAPABILITY_TURNING" in expr,
                        "milling": "CAPABILITY_MILLING" in expr or "CAPABILITY_ROUTER" in expr,
                    }
    except Exception:
        pass
    return None


def _IsPostCompatible(cps_path: str, is_turning: Optional[bool] = None) -> bool:
    """Check whether a post processor's declared capabilities match setup operation type."""
    if is_turning is None:
        return True
    caps = _GetPostCapabilities(cps_path)
    if not caps:
        return True  # If capabilities not declared or readable, allow
    if is_turning:
        return caps["turning"]
    else:
        return caps["milling"]


def _ResolveBasePostPath(program=None, is_turning: Optional[bool] = None) -> str:
    """
    Resolve target machine post processor path from NCProgram without mutating user settings.
    - Exact path or file:// URL is authoritative.
    - For name/description matches, user/cloud post directories (Tier 1) are preferred over system posts (Tier 2).
    - If more than one compatible file matches within the same priority tier, raises ValueError naming candidates.
    """
    if not program or not hasattr(program, "postConfiguration") or not program.postConfiguration:
        return ""

    post_cfg = program.postConfiguration
    if _IsInternalHelperPost(post_cfg):
        return ""

    # 1. Authoritative check: Exact path or file:// URL
    for attr in ("postPath", "location"):
        val = getattr(post_cfg, attr, None)
        if val and isinstance(val, str) and (os.path.isabs(val) or val.endswith(".cps")):
            if os.path.isfile(val):
                if _IsPostCompatible(val, is_turning):
                    _dbg_global(f"_ResolveBasePostPath: authoritative exact path via postConfiguration.{attr} -> {val}")
                    return os.path.abspath(val)

    try:
        if hasattr(post_cfg, "url") and post_cfg.url:
            u_str = post_cfg.url.toString() if hasattr(post_cfg.url, "toString") else str(post_cfg.url)
            if u_str and u_str.startswith("file://"):
                import urllib.parse
                f_path = urllib.parse.unquote(u_str[len("file://"):])
                if os.path.isfile(f_path):
                    if _IsPostCompatible(f_path, is_turning):
                        _dbg_global(f"_ResolveBasePostPath: authoritative exact path via url file:// -> {f_path}")
                        return os.path.abspath(f_path)
    except Exception as u_exc:
        _dbg_global(f"_ResolveBasePostPath url inspection exception: {u_exc}")

    # Gather search candidate strings (names / descriptions)
    candidates = []
    for attr in ("postPath", "location", "name", "description"):
        val = getattr(post_cfg, attr, None)
        if val and isinstance(val, str):
            candidates.append(val)
    if hasattr(post_cfg, "url") and post_cfg.url:
        u_str = post_cfg.url.toString() if hasattr(post_cfg.url, "toString") else str(post_cfg.url)
        if u_str:
            candidates.append(os.path.basename(u_str))

    search_dirs = _GetPostSearchDirs()
    user_dirs_def, sys_dirs_def, addon_dirs_def = _GetPostSearchDirsByTier()
    sys_set = set(sys_dirs_def)
    addon_set = set(addon_dirs_def)

    user_dirs = [d for d in search_dirs if d not in sys_set and d not in addon_set]
    sys_dirs = [d for d in search_dirs if d in sys_set]
    addon_dirs = [d for d in search_dirs if d in addon_set]

    tiers = [
        ("user/cloud", user_dirs),
        ("system", sys_dirs),
        ("add-on", addon_dirs),
    ]

    for tier_name, sdirs in tiers:
        if not sdirs:
            continue

        # A. Basename search within this tier
        for cand in candidates:
            bname = os.path.basename(cand).split("?")[0].strip()
            if not bname:
                continue
            names_to_try = [bname]
            if not bname.lower().endswith(".cps"):
                names_to_try.append(bname + ".cps")

            matched_files = set()
            for name_try in names_to_try:
                for sdir in sdirs:
                    p_try = os.path.join(sdir, name_try)
                    if os.path.isfile(p_try) and _IsPostCompatible(p_try, is_turning):
                        matched_files.add(os.path.abspath(p_try))

            if len(matched_files) == 1:
                res = list(matched_files)[0]
                _dbg_global(f"_ResolveBasePostPath: matched {bname} in {tier_name} -> {res}")
                return res
            elif len(matched_files) > 1:
                cand_list = ", ".join(f"'{p}'" for p in sorted(matched_files))
                raise ValueError(
                    f"Ambiguous post processor match for '{bname}' in {tier_name} post directories: "
                    f"multiple compatible candidate files found ({cand_list}). Please specify an exact path."
                )

        # B. Description substring search within this tier
        for cand in candidates:
            cand_clean = cand.strip().lower()
            if not cand_clean or len(cand_clean) < 3:
                continue

            matched_files = set()
            for sdir in sdirs:
                if not os.path.isdir(sdir):
                    continue
                try:
                    for entry in os.listdir(sdir):
                        if entry.lower().endswith(".cps"):
                            entry_path = os.path.join(sdir, entry)
                            if not _IsPostCompatible(entry_path, is_turning):
                                continue
                            try:
                                with open(entry_path, "r", encoding="utf-8", errors="ignore") as f:
                                    for line in f:
                                        if "description" in line and cand_clean in line.lower():
                                            matched_files.add(os.path.abspath(entry_path))
                                            break
                            except Exception:
                                pass
                except Exception:
                    pass

            if len(matched_files) == 1:
                res = list(matched_files)[0]
                _dbg_global(f"_ResolveBasePostPath: matched description '{cand}' in {tier_name} -> {res}")
                return res
            elif len(matched_files) > 1:
                cand_list = ", ".join(f"'{p}'" for p in sorted(matched_files))
                raise ValueError(
                    f"Ambiguous post processor match for description '{cand}' in {tier_name} post directories: "
                    f"multiple compatible candidate files found ({cand_list}). Please specify an exact path."
                )

    return ""


def _ExtractPostProperties(program=None) -> dict:
    """Extract custom post processor properties configured on the user's NCProgram."""
    props = {}
    if not program:
        return props
    try:
        post_params = getattr(program, "postParameters", None)
        if not post_params:
            return props
        cnt = getattr(post_params, "count", 0)
        for idx in range(cnt):
            try:
                param = post_params.item(idx)
                p_name = getattr(param, "name", None)
                if not p_name:
                    continue
                val_obj = getattr(param, "value", None)
                val = getattr(val_obj, "value", None) if val_obj is not None else None
                if isinstance(val, (bool, int, float, str)):
                    props[p_name] = val
            except Exception:
                pass

        # Check standard NCProgram parameters if maximumSpindleSpeed was not in postParameters
        if "maximumSpindleSpeed" not in props:
            prog_params = getattr(program, "parameters", None)
            if prog_params:
                for candidate_name in ("maximumSpindleSpeed", "maxSpindleSpeed", "nc_program_maximum_spindle_speed"):
                    try:
                        p_obj = prog_params.itemByName(candidate_name)
                        if p_obj:
                            val_obj = getattr(p_obj, "value", None)
                            val = getattr(val_obj, "value", None) if val_obj is not None else getattr(p_obj, "value", None)
                            if isinstance(val, (int, float)) and val > 0:
                                props["maximumSpindleSpeed"] = val
                                break
                    except Exception:
                        pass
    except Exception as exc:
        _dbg_global(f"_ExtractPostProperties warning: {exc}")
    return props


_modules_loaded_session = False


def _EnsureLocalModulesLoaded(force=False):
    """Ensure Add-In directory and packages are on sys.path and reload local modules once per session."""
    global _modules_loaded_session
    import sys
    if constAddonDir not in sys.path:
        sys.path.insert(0, constAddonDir)
    if sys.version_info >= (3, 14) and os.path.isdir(constPackagesDir) and constPackagesDir not in sys.path:
        sys.path.insert(0, constPackagesDir)

    # Do not reload modules during automated test execution to preserve Enum and Class identities.
    # Note: vendored pyparsing.testing imports 'unittest' in production Fusion 360 when plotting,
    # so check whether 'adsk' is a real C++ extension module on disk (__file__ is a string) vs a test mock.
    _adsk_mod = sys.modules.get("adsk")
    _is_test_env = bool(
        os.environ.get("PYTEST_CURRENT_TEST")
        or (
            "unittest" in sys.modules
            and not isinstance(getattr(_adsk_mod, "__file__", None), str)
        )
    )
    if _is_test_env:
        return

    if _modules_loaded_session and not force:
        return
    _modules_loaded_session = True

    import importlib
    # Reload in strict topological dependency order starting with geometry.models
    # so all downstream modules share identical Enum and dataclass type objects.
    ordered_mods = (
        "geometry.models",
        "geometry.engine_interface",
        "geometry.stock_generator",
        "geometry.tool_generator",
        "geometry.legacy_adapter",
        "geometry.lathe_engine",
        "geometry.solid_mesh",
        "geometry.mesh_utils",
        "geometry.heightfield_engine",
        "geometry.mill_engine",
        "geometry.plotter",
        "geometry",
        "context_parser",
        "pipeline",
    )
    for mod_name in ordered_mods:
        if mod_name in sys.modules:
            try:
                importlib.reload(sys.modules[mod_name])
            except Exception as exc:
                _dbg_global(f"Reload of {mod_name} failed: {exc}")

    try:
        from pipeline import find_autodesk_fusion_invoking_root, find_autodesk_post_binary, find_autodesk_system_posts_dir
        root = find_autodesk_fusion_invoking_root()
        if root and "AUTODESK_FUSION_ROOT" not in os.environ:
            os.environ["AUTODESK_FUSION_ROOT"] = root
        post_b = find_autodesk_post_binary()
        if post_b and "AUTODESK_POST_BINARY" not in os.environ:
            os.environ["AUTODESK_POST_BINARY"] = post_b
        sys_p = find_autodesk_system_posts_dir()
        if sys_p and "AUTODESK_SYSTEM_POSTS_DIR" not in os.environ:
            os.environ["AUTODESK_SYSTEM_POSTS_DIR"] = sys_p
    except Exception as exc:
        _dbg_global(f"_EnsureLocalModulesLoaded invoking root discovery failed: {exc}")


def _GetDocumentStockUnit(doc=None) -> str:
    """
    Get the document length unit ('mm' or 'in') from the CAM or Design units manager.
    Defaults to 'mm' if not determinable.
    """
    try:
        if doc is None and "adsk" in sys.modules and hasattr(adsk, "core"):
            app = adsk.core.Application.get()
            doc = getattr(app, "activeDocument", None) if app else None

        if doc is not None:
            units_mgr = getattr(doc, "unitsManager", None)
            if not units_mgr:
                products = getattr(doc, "products", None)
                if products:
                    for p_type in ("DesignProductType", "CAMProductType"):
                        try:
                            prod = products.itemByProductType(p_type) if hasattr(products, "itemByProductType") else None
                            if prod and hasattr(prod, "unitsManager"):
                                units_mgr = prod.unitsManager
                                break
                        except Exception:
                            pass
            if units_mgr:
                unit_str = getattr(units_mgr, "defaultLengthUnits", None)
                if isinstance(unit_str, str):
                    u = unit_str.strip().lower()
                    if u in ("in", "inch", "inches", "ft", "foot", "feet"):
                        return "in"
                    if u in ("mm", "cm", "m", "millimeter", "millimeters"):
                        return "mm"
    except Exception as exc:
        _dbg_global(f"_GetDocumentStockUnit failed: {exc}")
    return "mm"


@contextlib.contextmanager
def _ScopedEnv(env_updates: Dict[str, Optional[str]]):
    """
    Temporarily apply environment variable updates, saving previous values
    and restoring them strictly in a finally block.
    """
    saved = {}
    for k, v in env_updates.items():
        saved[k] = os.environ.get(k)
        if v is None:
            os.environ.pop(k, None)
        else:
            os.environ[k] = str(v)
    try:
        yield
    finally:
        for k, prev in saved.items():
            if prev is None:
                os.environ.pop(k, None)
            else:
                os.environ[k] = prev


def _WriteDumperConfig(base_post_path=None, target_cnc_path=None, post_properties=None, target_dir=None, stock_mode=None, unit=None, stock_unit=None, set_env=True, export_stock=None):
    """Write sidecar config for universal_dumper.cps exclusively into the per-run 0o700 mkdtemp directory."""
    cfg = {}
    if base_post_path and os.path.isfile(base_post_path):
        cfg["basePost"] = os.path.abspath(base_post_path)
    if target_cnc_path:
        cfg["targetIntermediateCnc"] = os.path.abspath(target_cnc_path)
    if post_properties and isinstance(post_properties, dict):
        cfg["postProperties"] = post_properties
        if "maximumSpindleSpeed" in post_properties:
            try:
                cfg["maximumSpindleSpeed"] = float(post_properties["maximumSpindleSpeed"])
            except (TypeError, ValueError):
                pass
    if stock_mode:
        cfg["stockMode"] = str(stock_mode)
    if export_stock is not None:
        cfg["export_stock"] = bool(export_stock)
    if unit is not None:
        cfg["unit"] = int(unit)
    if stock_unit:
        cfg["stockUnit"] = str(stock_unit)

    if not target_dir or not os.path.isdir(target_dir):
        target_dir = tempfile.mkdtemp(prefix="dumper_cfg_")
        try:
            os.chmod(target_dir, 0o700)
        except Exception:
            pass
    else:
        try:
            os.chmod(target_dir, 0o700)
        except Exception:
            pass

    cfg_path = os.path.join(target_dir, "universal_dumper_config.json")
    with open(cfg_path, "w", encoding="utf-8") as f:
        json.dump(cfg, f)
    if set_env:
        os.environ["DUMPER_CONFIG_PATH"] = cfg_path
    return cfg_path


def _ClearDumperConfig(target_dir=None):
    """Remove temporary sidecar config and clean environment variables for universal_dumper.cps."""
    if target_dir and os.path.isdir(target_dir):
        cfg_path = os.path.join(target_dir, "universal_dumper_config.json")
        if os.path.isfile(cfg_path):
            try:
                os.remove(cfg_path)
            except Exception:
                pass

    env_cfg = os.environ.get("DUMPER_CONFIG_PATH")
    if env_cfg and os.path.isfile(env_cfg):
        try:
            os.remove(env_cfg)
        except Exception:
            pass

    os.environ.pop("DUMPER_CONFIG_PATH", None)
    os.environ.pop("BASE_POST", None)
    os.environ.pop("TARGET_INTERMEDIATE_CNC", None)


def _GetSetupStockMode(setup) -> str:
    """Detect the stock definition mode from a Fusion CAM Setup."""
    if not setup:
        return ""

    res = ""
    # 1. Primary check: setup.stockMode enum property (Autodesk Fusion typed API)
    val = getattr(setup, "stockMode", None)
    if val is not None and not hasattr(val, "_mock_return_value"):
        v = getattr(val, "value", val)
        if isinstance(v, int):
            modes_enum = getattr(adsk.cam, "SetupStockModes", None)
            if modes_enum:
                if v == getattr(modes_enum, "PreviousSetupStock", 7):
                    res = "previous-setup"
                elif v == getattr(modes_enum, "SolidStock", 6):
                    res = "solid"
                elif v == getattr(modes_enum, "FixedBoxStock", 0):
                    res = "fixed-box"
                elif v == getattr(modes_enum, "RelativeBoxStock", 1):
                    res = "relative-box"
                elif v == getattr(modes_enum, "FixedCylinderStock", 2):
                    res = "fixed-cylinder"
                elif v == getattr(modes_enum, "RelativeCylinderStock", 3):
                    res = "relative-cylinder"
                elif v == getattr(modes_enum, "FixedTubeStock", 4):
                    res = "fixed-tube"
                elif v == getattr(modes_enum, "RelativeTubeStock", 5):
                    res = "relative-tube"
        elif isinstance(v, str):
            s = v.lower().strip()
            if "previous" in s or "preceding" in s or "rest" in s:
                res = "previous-setup"
            elif "solid" in s or "body" in s:
                res = "solid"
            else:
                res = s

    # 2. Fallback check for headless test setups or mock objects
    if not res and hasattr(setup, "parameters") and setup.parameters is not None:
        for p_name in ("job_stockMode", "stockMode", "job_stockType", "stockType"):
            try:
                p = setup.parameters.itemByName(p_name)
                if p is not None:
                    for attr in ("value", "expression"):
                        pv = getattr(p, attr, None)
                        if pv is not None:
                            pv_sub = getattr(pv, "value", pv)
                            if isinstance(pv_sub, (str, int)) and not hasattr(pv_sub, "_mock_return_value"):
                                s = str(pv_sub).lower().strip()
                                if "previous" in s or "preceding" in s or "rest" in s:
                                    res = "previous-setup"
                                elif "solid" in s or "body" in s:
                                    res = "solid"
                                else:
                                    res = s
                                break
                    if res:
                        break
            except Exception:
                pass

    _dbg_global(f"_GetSetupStockMode: setup='{getattr(setup, 'name', '')}' val={val} -> mode='{res}'")
    return res


def _ExtractWorkpieceFromSetup(setup):
    """Extract axisymmetric stock and chuck geometry directly from native Fusion 360 Setup."""
    try:
        stock_mode = _GetSetupStockMode(setup)
        if any(k in stock_mode for k in ("previous", "preceding", "prev", "rest")):
            _dbg_global(f"_ExtractWorkpieceFromSetup: setup uses stock mode '{stock_mode}'; relying on Pass 1 STL stock dump")
            return None

        if any(k in stock_mode for k in ("solid", "body")):
            try:
                from geometry.solid_mesh import extract_solid_workpiece_from_fusion_setup
                return extract_solid_workpiece_from_fusion_setup(setup, is_turning=True)
            except Exception as e:
                _dbg_global(f"_ExtractWorkpieceFromSetup: solid turning stock extraction failed: {e}")
                return None

        if not hasattr(setup, "parameters"):
            return None
        params = setup.parameters

        def _get_float_param(names, default=None):
            for n in names:
                try:
                    p = params.itemByName(n)
                    if p is not None:
                        expr = None
                        if hasattr(p, "expression") and p.expression:
                            expr = str(p.expression).strip()
                        val = p.value.value if hasattr(p, "value") and hasattr(p.value, "value") else getattr(p, "value", p)
                        if isinstance(val, (int, float)):
                            if not math.isnan(val) and not math.isinf(val):
                                return float(val) * 10.0  # Fusion internal length unit (cm) to mm

                        s = expr or (str(val).strip() if val is not None else "")
                        if s:
                            # 1. Try Fusion native unitsManager expression evaluation if in Fusion process
                            try:
                                app = adsk.core.Application.get() if ("adsk" in sys.modules and hasattr(adsk, "core")) else None
                                doc = getattr(app, "activeDocument", None) if app else None
                                units_mgr = getattr(doc, "unitsManager", None)
                                if units_mgr and hasattr(units_mgr, "evaluateExpression"):
                                    fval = units_mgr.evaluateExpression(s, "mm")
                                    if not math.isnan(fval) and not math.isinf(fval):
                                        return float(fval)
                            except Exception:
                                pass

                            # 2. Robust unit regex parsing (S5)
                            s_clean = s.lower().strip()
                            m = re.match(r"^([+-]?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)\s*([a-z'\"]*)$", s_clean)
                            if m:
                                num = float(m.group(1))
                                unit = m.group(2)
                                if unit in ("in", "inch", "inches", '"'):
                                    fval = num * 25.4
                                elif unit in ("ft", "feet", "'"):
                                    fval = num * 304.8
                                elif unit in ("cm",):
                                    fval = num * 10.0
                                elif unit in ("mm",):
                                    fval = num
                                elif unit == "":
                                    fval = num * 10.0
                                else:
                                    fval = None
                                if fval is not None and not math.isnan(fval) and not math.isinf(fval):
                                    return fval
                except Exception:
                    pass
            return default

        d_outer_mm = _get_float_param(["job_stockDiameter", "stockDiameter", "job_stockDimensionsX", "job_stockDimensionsY", "stock_diameter"])
        length_mm = _get_float_param(["job_stockLength", "stockLength", "job_stockDimensionsZ", "stock_length"])
        z_max_mm = _get_float_param(["job_stockOffsetFront", "stockOffsetFront", "job_stockOffsetZPositive"], default=0.0)
        if z_max_mm is None or math.isnan(z_max_mm):
            z_max_mm = 0.0

        if d_outer_mm is None or math.isnan(d_outer_mm) or d_outer_mm <= 0.0:
            _dbg_global("_ExtractWorkpieceFromSetup: stock diameter is missing or non-positive; returning None")
            return None
        if length_mm is None or math.isnan(length_mm) or length_mm <= 0.0:
            _dbg_global("_ExtractWorkpieceFromSetup: stock length is missing or non-positive; returning None")
            return None

        d_inner_mm = _get_float_param(["job_stockDiameterInner", "stockDiameterInner"], default=0.0)
        if d_inner_mm is None or math.isnan(d_inner_mm) or d_inner_mm < 0.0:
            d_inner_mm = 0.0
        z_min_mm = z_max_mm - length_mm


        fixtures = []
        clamp_len_mm = _get_float_param(["job_stockClampingLength", "stockClampingLength"], default=0.0)
        if clamp_len_mm and clamp_len_mm > 0.1:
            from geometry.models import AxisymmetricChuckObstacle
            fixtures.append(
                AxisymmetricChuckObstacle(
                    z_min=z_min_mm - 50.0,
                    z_max=z_min_mm + clamp_len_mm,
                    outer_radius=(d_outer_mm * 0.5) + 30.0,
                    clamping_radius=d_outer_mm * 0.5,
                )
            )

        from geometry.models import AxisymmetricStock2D, WorkpieceDefinition
        return WorkpieceDefinition(
            stock=AxisymmetricStock2D(
                z_min=z_min_mm,
                z_max=z_max_mm,
                outer_radius=d_outer_mm * 0.5,
                inner_radius=d_inner_mm * 0.5,
            ),
            fixtures=fixtures,
        )
    except Exception as exc:
        _dbg_global(f"_ExtractWorkpieceFromSetup exception: {exc}")
        return None





_palette_handlers = []
_last_action_timestamp = 0
_last_action_key = ""
constNotificationPaletteId = "PostProcessAll_NotificationPalette"

_HTMLEventHandlerBase = (
    getattr(adsk.core, "HTMLEventHandler", object)
    if ("adsk" in sys.modules and hasattr(adsk, "core") and hasattr(adsk.core, "HTMLEventHandler"))
    else object
)

_NavigationEventHandlerBase = (
    getattr(adsk.core, "NavigationEventHandler", object)
    if ("adsk" in sys.modules and hasattr(adsk, "core") and hasattr(adsk.core, "NavigationEventHandler"))
    else object
)


def _HandleNotificationAction(palette_id, action, data):
    global _last_action_timestamp, _last_action_key
    now = time.time()
    key = f"{action}:{data}"
    # Debounce duplicate events if both Navigation and HTML handlers fire for the same click
    if key == _last_action_key and (now - _last_action_timestamp) < 0.8:
        _dbg_global(f"_HandleNotificationAction: debouncing duplicate action={action}")
        return
    _last_action_key = key
    _last_action_timestamp = now

    _dbg_global(f"_HandleNotificationAction: executing action={action}, data={data}")

    # 1. Execute OS action first (independent of Fusion UI state); keep palette open
    if action in ("open_file", "open_folder"):
        if data and os.path.exists(data):
            res = _OpenFile(data)
            _dbg_global(f"_HandleNotificationAction: _OpenFile({data}) returned {res}")
        else:
            _dbg_global(f"_HandleNotificationAction: path does not exist: '{data}'")
        return

    # 2. Hide palette only when explicitly dismissed/closed
    if action in ("dismiss", "close"):
        try:
            app = adsk.core.Application.get() if ("adsk" in sys.modules and hasattr(adsk, "core") and hasattr(adsk.core, "Application")) else None
            ui = getattr(app, "userInterface", None) if app else None
            if ui and hasattr(ui, "palettes") and ui.palettes:
                p = ui.palettes.itemById(palette_id)
                if p:
                    p.isVisible = False
        except Exception as hide_exc:
            _dbg_global(f"_HandleNotificationAction hide palette note: {hide_exc}")


class _NotificationNavigationEventHandler(_NavigationEventHandlerBase):
    def __init__(self, palette_id):
        super().__init__()
        self.palette_id = palette_id

    def notify(self, args):
        try:
            url = getattr(args, "navigationURL", "")
            if "adsk" in sys.modules and hasattr(adsk, "core") and hasattr(adsk.core, "NavigationEventArgs"):
                try:
                    c = adsk.core.NavigationEventArgs.cast(args)
                    if c:
                        url = c.navigationURL
                except Exception:
                    pass

            _dbg_global(f"NotificationNavigationEventHandler notify: url={url}")

            if url and url.startswith("action://"):
                try:
                    args.launchExternally = False
                except Exception:
                    pass
                try:
                    if hasattr(args, "preventDefault"):
                        args.preventDefault = True
                except Exception:
                    pass

                p = urllib.parse.urlparse(url)
                action = (p.netloc or p.path).strip("/")
                q = urllib.parse.parse_qs(p.query)
                data = q.get("path", [""])[0] or q.get("data", [""])[0]
                _dbg_global(f"NotificationNavigationEventHandler parsed: action={action}, data={data}")
                _HandleNotificationAction(self.palette_id, action, data)
        except Exception as exc:
            _dbg_global(f"NotificationNavigationEventHandler exception: {exc}\n{traceback.format_exc()}")


class _NotificationHTMLEventHandler(_HTMLEventHandlerBase):
    def __init__(self, palette_id):
        super().__init__()
        self.palette_id = palette_id

    def notify(self, args):
        try:
            action = getattr(args, "action", "")
            data = getattr(args, "data", "")
            if "adsk" in sys.modules and hasattr(adsk, "core") and hasattr(adsk.core, "HTMLEventArgs"):
                try:
                    c = adsk.core.HTMLEventArgs.cast(args)
                    if c:
                        action = c.action
                        data = c.data
                except Exception:
                    pass

            _dbg_global(f"NotificationHTMLEventHandler notify: action={action}, data={data}")
            _HandleNotificationAction(self.palette_id, action, data)
        except Exception as exc:
            _dbg_global(f"NotificationHTMLEventHandler exception: {exc}")


def _BuildNotificationHTML(title, status_type, message, details, actions, program_rows=None):
    """
    Generate self-contained HTML for non-modal Fusion toast notification.
    Note: Fusion 360's palettes.add2 requires the HTML string to strictly start
    with <html> (no <!DOCTYPE html> or leading whitespace).
    """
    icon_map = {
        'success': '✓',
        'warning': '⚠',
        'error': '✕',
    }
    color_map = {
        'success': '#2e7d32',
        'warning': '#ed6c02',
        'error': '#d32f2f',
    }
    bg_accent = color_map.get(status_type, '#2e7d32')
    icon = icon_map.get(status_type, '✓')

    def _make_btn(label, action_name, action_data, css_cls="action-btn"):
        esc_data = html.escape(action_data, quote=True).replace('\\', '\\\\').replace("'", "\\'")
        nav_href = f"action://{action_name}?path={urllib.parse.quote(action_data)}"
        return (
            f'<a class="{css_cls}" href="{nav_href}" '
            f'onclick="sendAction(event, \'{action_name}\', \'{esc_data}\')">{html.escape(label)}</a>'
        )

    action_buttons = [_make_btn(label, action_name, action_data) for label, action_name, action_data in actions]
    actions_html = "\n".join(action_buttons)

    rows_html = ""
    if program_rows:
        row_items = []
        row_folders = {
            (r.get("folder") or (os.path.dirname(r["cnc"]) if r.get("cnc") else ""))
            for r in program_rows
            if (r.get("folder") or r.get("cnc"))
        }
        show_row_folder = len(row_folders) > 1
        for r in program_rows:
            r_status = r.get("status", "success")
            r_icon = icon_map.get(r_status, "✓")
            r_color = color_map.get(r_status, "#2e7d32")
            nc_prog = (r.get("nc_program") or "").strip()
            setup_name = (r.get("setup") or "").strip()
            if nc_prog and setup_name and nc_prog != setup_name:
                row_label = f"{nc_prog} · {setup_name}"
            else:
                row_label = nc_prog or setup_name or "Program"
            row_summary = r.get("summary") or ""

            mini_btns = []
            if r.get("cnc") and os.path.isfile(r["cnc"]):
                mini_btns.append(_make_btn("NC", "open_file", r["cnc"], css_cls="mini-btn"))
            if r.get("report") and os.path.isfile(r["report"]):
                mini_btns.append(_make_btn("Report", "open_file", r["report"], css_cls="mini-btn"))
            if r.get("png") and os.path.isfile(r["png"]):
                mini_btns.append(_make_btn("Plot", "open_file", r["png"], css_cls="mini-btn"))
            row_folder = r.get("folder") or (os.path.dirname(r["cnc"]) if r.get("cnc") else "")
            if show_row_folder and row_folder and os.path.isdir(row_folder):
                mini_btns.append(_make_btn("Folder", "open_folder", row_folder, css_cls="mini-btn"))

            summary_div = f'<div class="row-summary">{html.escape(row_summary)}</div>' if row_summary else ""
            row_items.append(
                f'<div class="program-row">'
                f'  <span class="row-icon" style="background-color:{r_color};">{r_icon}</span>'
                f'  <div class="row-info">'
                f'    <div class="row-title">{html.escape(row_label)}</div>'
                f'    {summary_div}'
                f'  </div>'
                f'  <div class="row-actions">{"".join(mini_btns)}</div>'
                f'</div>'
            )
        rows_html = f'<div class="program-list">{"".join(row_items)}</div>'

    if status_type == "success":
        auto_dismiss_js = """
  var wasClicked = false;
  var dismissTimer = setTimeout(function() {
    if (!wasClicked) sendAction(null, 'dismiss', '');
  }, 2000);
  function cancelAutoDismiss() {
    wasClicked = true;
    if (dismissTimer) {
      clearTimeout(dismissTimer);
      dismissTimer = null;
    }
  }
  document.addEventListener('mousedown', cancelAutoDismiss, true);
  document.addEventListener('click', cancelAutoDismiss, true);
  document.addEventListener('mouseenter', function() {
    if (dismissTimer) {
      clearTimeout(dismissTimer);
      dismissTimer = null;
    }
  });
  document.addEventListener('mouseleave', function() {
    if (!wasClicked) {
      if (dismissTimer) clearTimeout(dismissTimer);
      dismissTimer = setTimeout(function() {
        if (!wasClicked) sendAction(null, 'dismiss', '');
      }, 2000);
    }
  });"""
    else:
        auto_dismiss_js = """
  var wasClicked = true;
  var dismissTimer = null;
  function cancelAutoDismiss() {}"""

    # Note: Must strictly begin with <html> (no <!DOCTYPE html>) for Fusion 360 C++ add2 parser
    html_content = f"""<html>
<head>
<meta charset="utf-8">
<style>
  * {{ box-sizing: border-box; margin: 0; padding: 0; }}
  body {{
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
    background: #2b2b2b;
    color: #f0f0f0;
    margin: 0;
    padding: 12px 16px;
    font-size: 12px;
    line-height: 1.45;
    overflow-y: auto;
    overflow-x: hidden;
    user-select: none;
    -webkit-user-select: none;
  }}
  @media (prefers-color-scheme: light) {{
    body {{ background: #fdfdfd; color: #202020; border: 1px solid #d0d0d0; }}
  }}
  .toast-header {{
    display: flex;
    align-items: center;
    font-size: 13.5px;
    font-weight: 600;
    margin-bottom: 8px;
  }}
  .toast-icon {{
    display: inline-flex;
    align-items: center;
    justify-content: center;
    width: 20px;
    height: 20px;
    border-radius: 50%;
    background-color: {bg_accent};
    color: #ffffff;
    font-size: 12px;
    font-weight: bold;
    margin-right: 8px;
    flex-shrink: 0;
  }}
  .toast-title {{
    flex-grow: 1;
    font-weight: 600;
  }}
  .toast-body {{
    font-size: 12px;
    margin-bottom: 6px;
    color: #e0e0e0;
    word-break: break-word;
  }}
  @media (prefers-color-scheme: light) {{
    .toast-body {{ color: #333333; }}
  }}
  .toast-details {{
    font-size: 11px;
    color: #9e9e9e;
    margin-bottom: 10px;
    word-break: break-word;
    white-space: pre-wrap;
  }}
  .program-list {{
    max-height: 250px;
    overflow-y: auto;
    margin: 6px 0 10px 0;
    border: 1px solid #3c3c3c;
    border-radius: 4px;
    background: #242424;
  }}
  @media (prefers-color-scheme: light) {{
    .program-list {{ border-color: #dcdcdc; background: #f6f6f6; }}
  }}
  .program-row {{
    display: flex;
    align-items: center;
    gap: 8px;
    padding: 6px 8px;
    border-bottom: 1px solid #353535;
  }}
  .program-row:last-child {{ border-bottom: none; }}
  @media (prefers-color-scheme: light) {{
    .program-row {{ border-bottom-color: #e4e4e4; }}
  }}
  .row-icon {{
    display: inline-flex;
    align-items: center;
    justify-content: center;
    width: 16px;
    height: 16px;
    border-radius: 50%;
    color: #ffffff;
    font-size: 10px;
    font-weight: bold;
    flex-shrink: 0;
  }}
  .row-info {{
    flex-grow: 1;
    min-width: 0;
  }}
  .row-title {{
    font-size: 11.5px;
    font-weight: 600;
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }}
  .row-summary {{
    font-size: 10.5px;
    color: #b0b0b0;
    white-space: nowrap;
    overflow: hidden;
    text-overflow: ellipsis;
  }}
  @media (prefers-color-scheme: light) {{
    .row-summary {{ color: #555555; }}
  }}
  .row-actions {{
    display: flex;
    gap: 4px;
    flex-shrink: 0;
  }}
  a.mini-btn {{
    display: inline-flex;
    align-items: center;
    justify-content: center;
    background: #3e3e3e;
    color: #ffffff !important;
    text-decoration: none !important;
    border: 1px solid #555555;
    padding: 2px 6px;
    border-radius: 3px;
    font-size: 10.5px;
    font-weight: 500;
    cursor: pointer;
  }}
  @media (prefers-color-scheme: light) {{
    a.mini-btn {{ background: #eeeeee; color: #111111 !important; border-color: #cccccc; }}
  }}
  a.mini-btn:hover {{
    background: #007acc;
    border-color: #007acc;
    color: #ffffff !important;
  }}
  .toast-actions {{
    display: flex;
    gap: 8px;
    flex-wrap: wrap;
    margin-top: 4px;
  }}
  a.action-btn, button.action-btn {{
    display: inline-flex;
    align-items: center;
    justify-content: center;
    background: #3e3e3e;
    color: #ffffff !important;
    text-decoration: none !important;
    border: 1px solid #555555;
    padding: 6px 12px;
    border-radius: 4px;
    font-size: 11.5px;
    font-weight: 500;
    cursor: pointer;
    transition: background 0.15s, border-color 0.15s;
    user-select: none;
    -webkit-user-select: none;
  }}
  @media (prefers-color-scheme: light) {{
    a.action-btn, button.action-btn {{ background: #eeeeee; color: #111111 !important; border-color: #cccccc; }}
  }}
  a.action-btn:hover, button.action-btn:hover {{
    background: #007acc;
    border-color: #007acc;
    color: #ffffff !important;
  }}
</style>
<script>
  function sendAction(e, action, data) {{
    if (action !== 'dismiss' && action !== 'close') {{
      cancelAutoDismiss();
    }}
    // 1. Try adsk.fusionSendData if available
    try {{
      if (typeof adsk !== 'undefined' && adsk && adsk.fusionSendData) {{
        adsk.fusionSendData(action, data);
      }} else if (window.adsk && window.adsk.fusionSendData) {{
        window.adsk.fusionSendData(action, data);
      }}
    }} catch (err) {{}}

    // 2. Direct location navigation to fire navigatingURL (universal fail-safe)
    try {{
      var navUrl = 'action://' + encodeURIComponent(action) + '?path=' + encodeURIComponent(data) + '&t=' + Date.now();
      window.location.href = navUrl;
    }} catch (err2) {{}}
  }}{auto_dismiss_js}
</script>
</head>
<body>
  <div class="toast-header">
    <div class="toast-icon">{icon}</div>
    <div class="toast-title">{html.escape(title)}</div>
  </div>
  <div class="toast-body">{html.escape(message)}</div>
  {rows_html}
  {f'<div class="toast-details">{html.escape(details)}</div>' if details else ''}
  <div class="toast-actions">
    {actions_html}
  </div>
</body>
</html>"""
    return html_content.strip()


def _NotifyPostProcessResult(ui, cntFiles, cntSkipped, lstSkipped, outputFolder, verifyResults=None, is_fatal=False, program_outcomes=None):
    """
    Provide non-modal feedback via Fusion statusMessage and non-modal Palette toast,
    matching Fusion native post-processing behavior without blocking modal message boxes.
    """
    global _notification_handler
    verifyResults = verifyResults or []
    program_outcomes = program_outcomes or []

    # 1. Determine status type and headline
    has_errors = (cntFiles == 0 or is_fatal)
    has_collisions = any(bool(r.get("kinds")) for r in verifyResults)
    has_warnings = (cntSkipped > 0 or has_collisions or any(bool(r.get("warning")) for r in verifyResults))

    if has_errors:
        status_type = "error"
        title = "Post Processing Failed"
        msg = lstSkipped.strip() if lstSkipped else "No CAM operations posted."
    elif has_warnings:
        status_type = "warning"
        title = "Post Processing Completed with Warnings"
        if cntSkipped > 0:
            msg = f"{cntFiles} file(s) written. {cntSkipped} setup(s) skipped due to errors."
        else:
            msg = f"{cntFiles} file(s) written. Collision warnings detected during verification."
    else:
        status_type = "success"
        title = "Post Processing Completed Successfully"
        if len(verifyResults) == 1:
            r0 = verifyResults[0]
            prom = r0.get("promoted", 0) or 0
            msg = f"{r0['setup']}: {cntFiles} file written ({prom} rapid moves restored, 0 collisions)."
        else:
            total_prom = sum((r.get("promoted", 0) or 0) for r in verifyResults)
            if len(verifyResults) > 1:
                msg = f"{cntFiles} NC programs completed ({total_prom} G0 restored, 0 collisions)."
            else:
                msg = f"{cntFiles} file(s) successfully written."

    # Build per-program rows from verifyResults and any non-verified outcomes
    program_rows = []
    seen_row_keys = set()
    for r in verifyResults:
        total_cols = sum(r["kinds"].values()) if r.get("kinds") else 0
        if r.get("error"):
            r_status = "error"
            r_summary = str(r["error"])
        elif r.get("warning") or total_cols > 0:
            r_status = "warning"
            r_summary = str(r.get("warning") or f"{total_cols} collision(s) detected")
        else:
            r_status = "success"
            prom = r.get("promoted", 0) or 0
            r_summary = f"{prom} G0 restored · 0 collisions"
        cnc_p = r.get("cnc", "")
        folder_p = r.get("folder") or (os.path.dirname(cnc_p) if cnc_p else "")
        nc_prog = r.get("nc_program") or ""
        setup_name = r.get("setup") or ""
        seen_row_keys.add((nc_prog, setup_name, cnc_p))
        program_rows.append({
            "status": r_status,
            "nc_program": nc_prog,
            "setup": setup_name,
            "summary": r_summary,
            "cnc": cnc_p,
            "report": r.get("report", ""),
            "png": r.get("png", ""),
            "folder": folder_p,
        })
    for po in program_outcomes:
        key = (po.get("nc_program", ""), po.get("setup", ""), po.get("cnc", ""))
        if key not in seen_row_keys:
            program_rows.append(po)

    details = outputFolder if outputFolder else ""
    if cntSkipped > 0 and lstSkipped:
        skip_msg = lstSkipped.strip()
        details = f"{skip_msg}\n\nFolder: {outputFolder}" if outputFolder else skip_msg
    if len(details) > 600:
        details = details[:600].rstrip() + "\n... (see report for full details)"

    # 2. Log status summary to Application console (without flickering status bar)
    status_summary = f"[PostProcessAll] {title}: {msg}"
    try:
        if "adsk.core" in sys.modules and hasattr(adsk.core, "Application"):
            adsk.core.Application.get().log(status_summary)
    except Exception:
        pass

    # 3. Build Action buttons for the non-modal toast
    actions = []
    show_program_rows = len(program_rows) > 1
    if not show_program_rows:
        last_res = verifyResults[-1] if verifyResults else None
        if last_res:
            if last_res.get("cnc") and os.path.isfile(last_res["cnc"]):
                actions.append(("Open NC File", "open_file", last_res["cnc"]))
        if outputFolder and os.path.isdir(outputFolder):
            actions.append(("Open Folder", "open_folder", outputFolder))
        if last_res:
            if last_res.get("report") and os.path.isfile(last_res["report"]):
                actions.append(("View Report", "open_file", last_res["report"]))
            if last_res.get("png") and os.path.isfile(last_res["png"]):
                actions.append(("View Plot", "open_file", last_res["png"]))
    else:
        row_folders = {
            r.get("folder") for r in program_rows if r.get("folder")
        }
        if len(row_folders) <= 1:
            shared = next(iter(row_folders), outputFolder)
            if shared and os.path.isdir(shared):
                actions.append(("Open Folder", "open_folder", shared))
    actions.append(("Dismiss", "dismiss", ""))

    # 4. Display non-modal Palette toast if supported by ui
    if ui and hasattr(ui, "palettes") and ui.palettes is not None:
        html_text = _BuildNotificationHTML(
            title,
            status_type,
            msg,
            details,
            actions,
            program_rows=program_rows if show_program_rows else None,
        )
        tmp_html = os.path.join(tempfile.gettempdir(), "postprocess_toast.html")
        try:
            with open(tmp_html, "w", encoding="utf-8") as f:
                f.write(html_text)
        except Exception as fe:
            _dbg_global(f"_NotifyPostProcessResult write tmp_html note: {fe}")

        palette_w = 500
        if len(program_rows) <= 1:
            palette_h = 220
            is_resizable = False
        else:
            palette_h = min(440, 150 + 56 * len(program_rows))
            is_resizable = True
        target_x = 940
        target_y = 680

        # Query macOS Fusion window bounds for pixel-perfect lower-right placement
        if sys.platform == "darwin":
            try:
                import subprocess
                cmd = 'tell application "System Events" to tell process "Autodesk Fusion" to repeat with w in windows\nif name of w contains "Autodesk Fusion" then\nreturn {position of w, size of w}\nend if\nend repeat'
                raw = subprocess.check_output(['osascript', '-e', cmd], timeout=0.4).decode().strip()
                parts = [int(p.strip()) for p in raw.split(',') if p.strip().lstrip('-').isdigit()]
                if len(parts) >= 4:
                    win_x, win_y, win_w, win_h = parts[0], parts[1], parts[2], parts[3]
                    target_x = max(20, win_x + win_w - palette_w - 24)
                    target_y = max(20, win_y + win_h - palette_h - 45)
                    _dbg_global(f"_NotifyPostProcessResult: macOS window bounds ({win_x},{win_y},{win_w},{win_h}) -> target ({target_x}, {target_y})")
            except Exception as e_mac:
                _dbg_global(f"_NotifyPostProcessResult: mac window query note: {e_mac}")
        else:
            app = adsk.core.Application.get() if ("adsk" in sys.modules and hasattr(adsk, "core")) else None
            if app and hasattr(app, "activeViewport") and app.activeViewport:
                vp = app.activeViewport
                try:
                    if hasattr(adsk.core, "Point2D") and hasattr(adsk.core.Point2D, "create"):
                        pt = vp.viewToScreen(adsk.core.Point2D.create(vp.width, vp.height))
                        if pt:
                            target_x = max(50, int(pt.x) - palette_w - 20)
                            target_y = max(50, int(pt.y) - palette_h - 20)
                except Exception as pt_exc:
                    _dbg_global(f"_NotifyPostProcessResult: viewToScreen note: {pt_exc}")

        palette = None

        # Check if existing notification palette exists and recreate cleanly
        existing = None
        try:
            existing = ui.palettes.itemById(constNotificationPaletteId)
        except Exception:
            existing = None

        if existing:
            try:
                _dbg_global(f"_NotifyPostProcessResult: cleaning up previous palette id={constNotificationPaletteId}")
                existing.deleteMe()
            except Exception as rex:
                _dbg_global(f"_NotifyPostProcessResult: palette cleanup error: {rex}")
            existing = None

        if not palette and os.path.isfile(tmp_html):
            if hasattr(ui.palettes, "add2"):
                try:
                    _dbg_global("_NotifyPostProcessResult: attempting ui.palettes.add2 with tmp_html")
                    palette = ui.palettes.add2(
                        constNotificationPaletteId,
                        "Post Processing",
                        tmp_html,
                        True,   # isVisible=True so native Qt window is allocated for setPosition/setSize
                        True,   # showCloseButton
                        is_resizable,
                        palette_w,
                        palette_h,
                    )
                except Exception as e_add2:
                    _dbg_global(f"_NotifyPostProcessResult: add2 with tmp_html failed: {e_add2}")

            if not palette and hasattr(ui.palettes, "add"):
                try:
                    _dbg_global("_NotifyPostProcessResult: attempting ui.palettes.add with tmp_html")
                    palette = ui.palettes.add(
                        constNotificationPaletteId,
                        "Post Processing",
                        tmp_html,
                        True,
                        True,
                        is_resizable,
                        palette_w,
                        palette_h,
                    )
                except Exception as e_add:
                    _dbg_global(f"_NotifyPostProcessResult: add with tmp_html failed: {e_add}")

        if palette:
            _dbg_global(f"_NotifyPostProcessResult: palette active (id={constNotificationPaletteId})")
            try:
                palette.isVisible = True
            except Exception:
                pass

            try:
                if "adsk.core" in sys.modules:
                    if hasattr(adsk.core, "PaletteDockingOptions") and hasattr(adsk.core.PaletteDockingOptions, "PaletteDockOptionsNone"):
                        palette.dockingOption = adsk.core.PaletteDockingOptions.PaletteDockOptionsNone
                    if hasattr(adsk.core, "PaletteDockingStates") and hasattr(adsk.core.PaletteDockingStates, "PaletteDockStateFloating"):
                        palette.dockingState = adsk.core.PaletteDockingStates.PaletteDockStateFloating
            except Exception as dock_exc:
                _dbg_global(f"_NotifyPostProcessResult docking state note: {dock_exc}")

            try:
                res_size = palette.setSize(palette_w, palette_h)
                _dbg_global(f"_NotifyPostProcessResult: palette.setSize({palette_w}, {palette_h}) returned {res_size}")
            except Exception as size_exc:
                _dbg_global(f"_NotifyPostProcessResult: setSize note: {size_exc}")

            try:
                res_pos = palette.setPosition(target_x, target_y)
                _dbg_global(f"_NotifyPostProcessResult: palette.setPosition({target_x}, {target_y}) returned {res_pos}")
            except Exception as pos_exc:
                _dbg_global(f"_NotifyPostProcessResult: setPosition failed: {pos_exc}")

            # Remove any stale notification handlers from global handlers
            global _palette_handlers, handlers
            for h in _palette_handlers:
                if h in handlers:
                    try:
                        handlers.remove(h)
                    except ValueError:
                        pass
            _palette_handlers = []

            # 1. Navigation Event Handler (handles action:// links from <a> or window.location.href)
            if hasattr(palette, "navigatingURL") and hasattr(palette.navigatingURL, "add"):
                try:
                    nav_handler = _NotificationNavigationEventHandler(constNotificationPaletteId)
                    palette.navigatingURL.add(nav_handler)
                    _palette_handlers.append(nav_handler)
                    handlers.append(nav_handler)
                    _dbg_global("_NotifyPostProcessResult: navigatingURL handler attached successfully")
                except Exception as n_exc:
                    _dbg_global(f"_NotifyPostProcessResult: navigatingURL attach note: {n_exc}")

            # 2. HTML Event Handler (handles adsk.fusionSendData)
            if hasattr(palette, "incomingFromHTML") and hasattr(palette.incomingFromHTML, "add"):
                try:
                    html_handler = _NotificationHTMLEventHandler(constNotificationPaletteId)
                    palette.incomingFromHTML.add(html_handler)
                    _palette_handlers.append(html_handler)
                    handlers.append(html_handler)
                    _dbg_global("_NotifyPostProcessResult: incomingFromHTML handler attached successfully")
                except Exception as h_exc:
                    _dbg_global(f"_NotifyPostProcessResult: incomingFromHTML attach note: {h_exc}")
        else:
            _dbg_global("_NotifyPostProcessResult: failed to create palette through all attempts")




def ShowVerifyResults(ui):
    """
    Log verification summaries to Application log without modal dialog interruption.
    Diagnostic details and collision breakdowns are saved to the XXX_report.md files.
    """
    _dbg_global(f"ShowVerifyResults called: verifyResults count={len(verifyResults)}")
    if not verifyResults:
        return
    for r in verifyResults:
        total_cols = sum(r["kinds"].values()) if r.get("kinds") else 0
        if r.get("error"):
            status = "FAILED: " + r["error"]
        elif r.get("warning") or total_cols > 0:
            warn_str = r.get("warning") or f"{total_cols} collision(s) detected"
            status = f"WARNING: {warn_str}"
        else:
            status = "OK"
        msg = f"[PostProcessAll] Verification for '{r['setup']}': {status} | NC: {r.get('cnc')} | Report: {r.get('report')}"
        _dbg_global(msg)
        try:
            adsk.core.Application.get().log(msg)
        except Exception:
            pass


def _OpenFile(path):
    """Open a file in the OS viewer. Returns None on success or an error string."""
    _dbg_global(f"_OpenFile: requested for '{path}' (exists={os.path.exists(path)})")
    try:
        if hasattr(os, "startfile"):
            os.startfile(path)
            _dbg_global(f"_OpenFile: os.startfile succeeded for '{path}'")
            return None
        if sys.platform == "win32":
            subprocess.run(["cmd", "/c", "start", "", path], check=True)
            _dbg_global(f"_OpenFile: cmd start succeeded for '{path}'")
            return None
        # Strip Fusion's embedded-Python/dyld variables so LaunchServices helpers start cleanly
        env = {k: v for k, v in os.environ.items() if not k.startswith(("PYTHON", "DYLD_"))}
        attempts = [["open", path]]
        if path.lower().endswith(".png"):
            attempts.insert(0, ["open", "-a", "Preview", path])
        last = ""
        for cmd in attempts:
            proc = subprocess.run(cmd, capture_output=True, text=True, timeout=15, env=env)
            if proc.returncode == 0:
                _dbg_global(f"_OpenFile: command {cmd} succeeded")
                return None
            last = (proc.stderr or proc.stdout).strip() or f"exit code {proc.returncode}"
            _dbg_global(f"_OpenFile: command {cmd} failed with {last}")
        return last
    except Exception as exc:
        err = f"{exc}\n{traceback.format_exc()}"
        _dbg_global(f"_OpenFile: exception: {err}")
        return str(exc)


def _PostProcessUnifiedPipeline(fname, setup, setupFolder, docSettings, program, path, fileExt, target_operations=None, progress_cb=None):
    """
    Unified 2-pass Context-Dump & Single-Pass Replay pipeline for Milling and Turning setups:
    Pass 1 (Context Extraction): Post each single-tool batch inside Fusion using universal_dumper.cps
            (inheriting vendor BASE_POST settings) to emit batch_i.json and capture bootstrap .cnc.
    Pass 2 (Merge, Universal Domain Extraction, Unified Verification & Single-Pass Replay):
            - Merge all batch_i.json files into a unified multi-section context dump.
            - Extract domain WorkpieceDefinition & tools from context:
              * Turning: AxisymmetricStock2D + exact kernel SVG profiles.
              * Milling: BoxStock3D + parametric MillToolGeometry.
            - Verify toolpath:
              * Turning: Lathe2DIPWVerifier (dynamic 2D IPW simulation + chuck guard).
              * Milling: MillHeightfieldVerifier (3D Heightfield Grid IPW simulation with gouge & collision detection).
            - Universal collision check (RAPID_INTO_STOCK, HOLDER_STRIKE, FIXTURE_COLLISION, SHANK_RUB).
              Issues warnings, logs diagnostics, and highlights collisions while continuing final NC generation.
            - If restore_rapid is enabled, stamp promotions onto merged context.
            - Single-pass replay of unmodified vendor post via universal_replayer.cps.
            - Generate summary report (_report.md) and record verify results for UI.
    """
    _dbg = _dbg_global
    _notify_p = progress_cb or (lambda msg, frac: True)
    if not _notify_p("Initializing pipeline...", 0.02):
        return "Operation cancelled by user."
    is_turning = (setup.operationType == adsk.cam.OperationTypes.TurningOperation)
    _dbg(f"_PostProcessUnifiedPipeline entered for {fname}, path={path}, is_turning={is_turning}")

    # 0. Ensure Add-In modules are on sys.path and reloaded in Fusion's persistent interpreter
    _EnsureLocalModulesLoaded()

    if program is None:
        try:
            app = adsk.core.Application.get()
            doc = getattr(app, "activeDocument", None)
            cam = adsk.cam.CAM.cast(doc.products.itemByProductType(constCAMProductId)) if (doc and hasattr(doc, "products")) else None
            program = GetNcProgram(cam, docSettings)
        except Exception:
            pass
    try:
        base_post_path = _ResolveBasePostPath(program, is_turning=is_turning)
    except (ValueError, RuntimeError) as err:
        return str(err)
    if not base_post_path or not os.path.isfile(base_post_path):
        prog_name = getattr(program, "name", "None")
        post_cfg = getattr(program, "postConfiguration", None)
        post_name = getattr(post_cfg, "name", None) or getattr(post_cfg, "description", None) or "None"
        setup_type = "Turning" if is_turning else "Milling"
        try:
            raw_post = _ResolveBasePostPath(program, is_turning=None)
        except (ValueError, RuntimeError):
            raw_post = ""
        if raw_post and os.path.isfile(raw_post):
            caps = _GetPostCapabilities(raw_post)
            if caps:
                if is_turning and not caps.get("turning", False) and caps.get("milling", False):
                    return (
                        f"NC Program '{prog_name}' is configured with milling post '{post_name}', "
                        f"which does not support turning toolpaths. "
                        f"Please select or configure a turning NC Program for setup '{setup.name}'."
                    )
                elif not is_turning and not caps.get("milling", False) and caps.get("turning", False):
                    return (
                        f"NC Program '{prog_name}' is configured with turning post '{post_name}', "
                        f"which does not support milling toolpaths. "
                        f"Please select or configure a milling NC Program for setup '{setup.name}'."
                    )
        return (
            f"Could not locate machine post processor for {setup_type} setup '{setup.name}' "
            f"(NC Program '{prog_name}', post '{post_name}'). "
            f"Please verify the post processor configuration in your NC Program."
        )
    post_props = _ExtractPostProperties(program)
    _dbg(f"_PostProcessUnifiedPipeline: base_post_path={base_post_path}, post_props={post_props}")

    # Ensure output path uses normalized extension (never .json)
    fileExt = _NormalizeFileExtension(fileExt)
    if path.lower().endswith(".json"):
        path = os.path.splitext(path)[0] + fileExt

    base_no_ext = os.path.splitext(path)[0]
    report_path = base_no_ext + "_report.md"
    plot_path = base_no_ext + ".png"

    stock_mode = _GetSetupStockMode(setup)
    _dbg(f"_PostProcessUnifiedPipeline: setup '{setup.name}' stock_mode='{stock_mode}'")

    solid_wp = None
    if any(k in stock_mode for k in ("solid", "body")):
        try:
            from geometry.solid_mesh import extract_solid_workpiece_from_fusion_setup
            solid_wp = extract_solid_workpiece_from_fusion_setup(setup, is_turning=is_turning)
        except Exception as solid_exc:
            err_msg = (
                f"Setup '{setup.name}' uses 'From Solid' stock, but extraction failed: {solid_exc}"
            )
            _dbg(f"_PostProcessUnifiedPipeline: {err_msg}")
            _RecordVerifyResult(
                setup=setup,
                cncPath=path,
                reportText=f"# Verification Failed\n\nError: {err_msg}",
                reportPath=report_path,
                pngPath=plot_path,
                promoted=0,
                spikes=0,
                volume_mm3=0.0,
                collisions=[],
                error=err_msg,
                warning=None,
            )
            return err_msg

        if solid_wp is None:
            err_msg = (
                f"Setup '{setup.name}' uses 'From Solid' stock, but no valid stock bodies were found or extracted."
            )
            _dbg(f"_PostProcessUnifiedPipeline: {err_msg}")
            _RecordVerifyResult(
                setup=setup,
                cncPath=path,
                reportText=f"# Verification Failed\n\nError: {err_msg}",
                reportPath=report_path,
                pngPath=plot_path,
                promoted=0,
                spikes=0,
                volume_mm3=0.0,
                collisions=[],
                error=err_msg,
                warning=None,
            )
            return err_msg

    # 2. Partition setup operations into single-tool batches (mandatory for Personal Edition compatibility)
    cand_ops = target_operations if target_operations is not None else getattr(setup, "allOperations", [])
    cand_list = _extract_cam_collection(cand_ops)

    active_ops = [
        op for op in cand_list
        if not getattr(op, "isSuppressed", False)
        and getattr(op, "hasToolpath", True)
        and getattr(op, "isToolpathValid", True)
    ]
    batches = []
    i = 0
    while i < len(active_ops):
        op = active_ops[i]
        curTool = op.tool.parameters.itemByName("tool_number").value.value
        opList = [op]
        i += 1
        while i < len(active_ops):
            next_op = active_ops[i]
            t_num = next_op.tool.parameters.itemByName("tool_number").value.value
            if t_num != curTool:
                break
            opList.append(next_op)
            i += 1
        batches.append((curTool, opList))

    _dbg(f"_PostProcessUnifiedPipeline: partitioned into {len(batches)} batches: {[b[0] for b in batches]}")
    if not batches:
        return "No active operations to post process."

    tmp_dir = tempfile.mkdtemp(prefix="unified_pipe_")
    try:
        os.chmod(tmp_dir, 0o700)
    except Exception:
        pass
    try:
        # -------------------------------------------------------------
        # PASS 1: Extract full post context (.json) per single-tool batch
        # Direct execution via cam.postProcess using constDumperScript path
        # Zero NCProgram creation and zero registration in Post Library required.
        # -------------------------------------------------------------
        app = adsk.core.Application.get()
        cam = adsk.cam.CAM.cast(app.activeDocument.products.itemByProductType(constCAMProductId))

        batch_json_files = []
        bootstrap_cnc = os.path.join(tmp_dir, "bootstrap.intermediate.cnc")
        units = _GetNCProgramOutputUnits(program, docSettings=docSettings)
        cam_mod = sys.modules.get("adsk.cam")
        enum_cls = getattr(cam_mod, "PostOutputUnitOptions", None)
        mm_opt = getattr(enum_cls, "MillimetersOutput", 2) if enum_cls else 2
        in_opt = getattr(enum_cls, "InchesOutput", 1) if enum_cls else 1
        dumper_unit = None
        if units == mm_opt:
            dumper_unit = 1  # MM in CPS post engine
        elif units == in_opt:
            dumper_unit = 0  # IN in CPS post engine

        stock_unit = _GetDocumentStockUnit(getattr(app, "activeDocument", None) if app else None)
        is_prev_stock = any(k in stock_mode for k in ("previous", "preceding", "prev", "rest"))

        n_batches = max(1, len(batches))
        for b_idx, (t_num, op_list) in enumerate(batches):
            b_frac = 0.08 + 0.32 * (b_idx / n_batches)
            if not _notify_p(f"Pass 1: Dumping toolpath batch {b_idx + 1} of {n_batches} (Tool {t_num})...", b_frac):
                return "Operation cancelled by user."
            b_prefix = f"batch_{b_idx}"
            b_json = os.path.join(tmp_dir, f"{b_prefix}.json")
            want_export_stock = bool(is_prev_stock and b_idx == 0)

            cfg_path = _WriteDumperConfig(
                base_post_path=base_post_path,
                target_cnc_path=bootstrap_cnc if b_idx == 0 else None,
                post_properties=post_props,
                target_dir=tmp_dir,
                stock_mode=stock_mode,
                unit=dumper_unit,
                stock_unit=stock_unit,
                set_env=False,
                export_stock=want_export_stock,
            )

            dumper_cps_for_batch = constDumperScript
            if want_export_stock:
                staged_dumper = os.path.join(tmp_dir, "universal_dumper_export_stock.cps")
                try:
                    shutil.copy2(constDumperScript, staged_dumper)
                    dumper_cps_for_batch = staged_dumper
                except Exception as cp_exc:
                    _dbg(f"Pass 1: Failed to stage {staged_dumper}: {cp_exc}")

            _dbg(f"Pass 1: Posting batch {b_idx} (Tool {t_num}, {len(op_list)} ops, exportStock={want_export_stock}) with {os.path.basename(dumper_cps_for_batch)} via direct PostProcessInput...")
            if len(op_list) == 1:
                target_ops = op_list[0]
            else:
                target_ops = adsk.core.ObjectCollection.create()
                for op in op_list:
                    target_ops.add(op)

            # Ensure no stale output file exists before invoking dumper
            if os.path.isfile(b_json):
                try:
                    os.remove(b_json)
                except Exception:
                    pass

            env_updates = {
                "BASE_POST": os.path.abspath(base_post_path),
                "TARGET_INTERMEDIATE_CNC": bootstrap_cnc if b_idx == 0 else None,
                "DUMPER_CONFIG_PATH": cfg_path,
            }
            post_ok = False
            with _ScopedEnv(env_updates):
                post_input = adsk.cam.PostProcessInput.create(
                    b_prefix,
                    dumper_cps_for_batch,
                    tmp_dir,
                    units,
                )
                post_input.isOpenInEditor = False
                try:
                    post_ok = bool(cam.postProcess(target_ops, post_input))
                except Exception as pp_exc:
                    _dbg(f"Pass 1: cam.postProcess exception for batch {b_idx}: {pp_exc}")
                    post_ok = False

            if not os.path.isfile(b_json):
                # Search tmp_dir strictly for batch_{b_idx}.json or batch_{b_idx}_<digits>.json
                # to prevent batch_1 from matching batch_10..batch_19 (Fix B5)
                pattern = re.compile(rf"^{re.escape(b_prefix)}(_\d+)?\.json$")
                for f in os.listdir(tmp_dir):
                    if pattern.match(f):
                        b_json = os.path.join(tmp_dir, f)
                        break

            if not os.path.isfile(b_json):
                _dbg(f"Pass 1: Failed to produce {b_json} (post_ok={post_ok})!")
                return f"Failed to dump post context for batch {b_idx + 1} (Tool {t_num})."

            if not post_ok:
                _dbg(f"Pass 1: cam.postProcess returned False for batch {b_idx}, but {b_json} was created.")

            if b_idx == 0 and not os.path.isfile(bootstrap_cnc):
                fallback_inter = b_json + ".intermediate.cnc"
                if os.path.isfile(fallback_inter):
                    shutil.copy2(fallback_inter, bootstrap_cnc)

            batch_json_files.append(b_json)

        # -------------------------------------------------------------
        # PASS 2: Merge Contexts, Universal Domain Extraction & Single-Pass Replay
        # Delegated directly to PipelineOrchestrator to eliminate duplication between Fusion and CLI/test
        # -------------------------------------------------------------
        from pipeline import PipelineOrchestrator
        from geometry import CollisionKind

        base_no_ext = os.path.splitext(path)[0]
        report_path = base_no_ext + "_report.md"
        plot_path = base_no_ext + ".png"

        if solid_wp is not None:
            fallback_wp = solid_wp
        else:
            fallback_wp = _ExtractWorkpieceFromSetup(setup) if is_turning else None
        do_promote = bool(docSettings.get("restore_rapid", True))
        _dbg(f"Pass 2: invoking orchestrator.run with fallback_wp={fallback_wp}")

        orchestrator = PipelineOrchestrator(
            dumper_cps=constDumperScript,
            replayer_cps=constReplayerScript,
        )

        report, final_gcode = orchestrator.run(
            context_dump_paths=batch_json_files,
            base_post_path=base_post_path,
            bootstrap_cnc_path=bootstrap_cnc,
            final_cnc_path=path,
            report_output_path=report_path,
            plot_output_path=plot_path,
            promote=do_promote,
            progress_cb=_notify_p,
            post_properties=post_props,
            program_name=str(fname),
            fallback_workpiece=fallback_wp,
            is_turning=is_turning,
        )
        _dbg(
            f"Pass 2: orchestrator.run complete. Vol={report.total_volume_removed_mm3:.3f} mm³, "
            f"collisions={len(report.collisions)}"
        )

        # Universal collision check: issue warnings and record collision metrics while continuing NC generation
        fatal_cols = [
            c for c in report.collisions
            if c.kind in (CollisionKind.RAPID_INTO_STOCK, CollisionKind.HOLDER_STRIKE, CollisionKind.FIXTURE_COLLISION, CollisionKind.SHANK_RUB)
        ]
        warning_msg = None
        if fatal_cols:
            warning_msg = f"{len(fatal_cols)} collision(s) detected"
            _dbg(f"Pass 2: WARNING: {warning_msg} in setup {setup.name}! Continuing with final NC code generation.")
            try:
                adsk.core.Application.get().log(f"[PostProcessAll] WARNING: {warning_msg} in setup {setup.name}!")
            except Exception:
                pass

        if getattr(report, "fixture_warning", None):
            _dbg(f"Pass 2: WARNING: {report.fixture_warning} in setup {setup.name}!")
            try:
                adsk.core.Application.get().log(f"[PostProcessAll] WARNING: {report.fixture_warning} in setup {setup.name}!")
            except Exception:
                pass
            if warning_msg:
                warning_msg += f"; {report.fixture_warning}"
            else:
                warning_msg = report.fixture_warning

        _notify_p("Setup completed.", 1.0)

        if do_promote and getattr(report, "promoted_events", None):
            colliding_events = {
                (c.sec_idx, c.event_idx)
                for c in report.collisions
                if c.sec_idx is not None and c.event_idx is not None
            }
            promoted_total = len([ev for ev in report.promoted_events if ev not in colliding_events])
        else:
            promoted_total = 0

        summary_md = ""
        if os.path.isfile(report_path):
            with open(report_path, "r", encoding="utf-8") as rf:
                summary_md = rf.read()
        else:
            summary_md = orchestrator.format_summary_report(
                report, final_gcode=final_gcode, base_post_path=base_post_path, restored_count=promoted_total
            )

        _RecordVerifyResult(
            setup=setup,
            cncPath=path,
            reportText=summary_md,
            reportPath=report_path,
            pngPath=plot_path,
            promoted=promoted_total,
            spikes=len(report.mrr_spikes),
            volume_mm3=report.total_volume_removed_mm3,
            collisions=report.collisions,
            error=None,
            warning=warning_msg,
            nc_program=_safe_str(getattr(program, "name", None)),
        )

        return None

    except Exception as exc:
        err_msg = str(exc)
        _dbg(f"_PostProcessUnifiedPipeline exception: {traceback.format_exc()}")
        _RecordVerifyResult(
            setup=setup,
            cncPath=path,
            reportText=f"# Verification Failed\n\nError: {err_msg}",
            reportPath=report_path,
            pngPath=plot_path,
            promoted=0,
            spikes=0,
            volume_mm3=0.0,
            collisions=[],
            error=err_msg,
            warning=None,
            nc_program=_safe_str(getattr(program, "name", None)),
        )
        return f"PostProcess pipeline error: {err_msg}"
    finally:
        _ClearDumperConfig(target_dir=tmp_dir)
        try:
            shutil.rmtree(tmp_dir, ignore_errors=True)
        except Exception:
            pass



def PostProcessSetup(fname, setup, setupFolder, docSettings, program, target_operations, progress_cb=None):
    ui = None
    retVal = "Fusion reported an exception"

    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface
        doc = getattr(app, "activeDocument", None)
        cam = adsk.cam.CAM.cast(doc.products.itemByProductType(constCAMProductId)) if (doc and hasattr(doc, "products")) else None
        if program is None and cam is not None:
            program = GetNcProgram(cam, docSettings)
        parameters = getattr(program, "parameters", None)

        if not target_operations:
            return "No active operations specified to post process."

        # Verify file name is valid by creating it now
        raw_ext = ""
        if parameters is not None:
            try:
                ext_param = parameters.itemByName("nc_program_nc_extension") if hasattr(parameters, "itemByName") else parameters.get("nc_program_nc_extension")
                if ext_param is not None:
                    raw_ext = _extract_choice_value(ext_param)
            except Exception:
                pass
        fileExt = _NormalizeFileExtension(raw_ext)
        path = os.path.join(setupFolder, fname + fileExt)
        try:
            pathlib.Path(setupFolder).mkdir(parents=True, exist_ok=True)
            if os.path.exists(path):
                if not os.access(path, os.W_OK):
                    raise PermissionError(f"File {path} is not writable")
            else:
                with open(path, "w", encoding="utf8"):
                    pass
                os.remove(path)
        except Exception as exc:
            return "Unable to create output file '" + path + "'. Make sure the setup name is valid as a file name."

        # For setups using 'From preceding setup' stock, warn if preceding setup stock is likely missing
        stock_mode = _GetSetupStockMode(setup)
        if any(k in stock_mode for k in ("previous", "preceding", "prev", "rest")):
            prior_missing = False
            try:
                if cam and hasattr(cam, "setups"):
                    all_setups = list(cam.setups)
                    s_idx = -1
                    for idx, s in enumerate(all_setups):
                        if s == setup or (
                            hasattr(setup, "entityToken") and hasattr(s, "entityToken") and setup.entityToken == s.entityToken
                        ) or getattr(s, "name", None) == getattr(setup, "name", None):
                            s_idx = idx
                            break

                    if s_idx > 0:
                        # Walk all earlier setups (not just the immediate predecessor)
                        earlier_setups = [s for s in all_setups[:s_idx] if not getattr(s, "isSuppressed", False)]
                        if not earlier_setups:
                            prior_missing = True
                        else:
                            for prior_setup in earlier_setups:
                                prior_ops = [
                                    p_op for p_op in getattr(prior_setup, "allOperations", [])
                                    if not getattr(p_op, "isSuppressed", False)
                                ]
                                if not prior_ops:
                                    prior_missing = True
                                    break
                                for p_op in prior_ops:
                                    if not cam.checkToolpath(p_op):
                                        prior_missing = True
                                        break
                                if prior_missing:
                                    break
                    else:
                        prior_missing = True
                else:
                    prior_missing = True
            except Exception as prior_exc:
                _dbg_global(f"PostProcessSetup: prior setup check exception: {prior_exc}")
                prior_missing = True  # Fail closed on any exception

            if prior_missing and ui:
                prog_dialog = getattr(progress_cb, "progress_dialog", None)
                if prog_dialog and hasattr(prog_dialog, "hide"):
                    try:
                        prog_dialog.hide()
                    except Exception:
                        pass
                try:
                    res = ui.messageBox(
                        "Preceding setup stock is likely missing, proceed?",
                        constCmdName,
                        adsk.core.MessageBoxButtonTypes.YesNoButtonType,
                        adsk.core.MessageBoxIconTypes.WarningIconType,
                    )
                finally:
                    if prog_dialog and hasattr(prog_dialog, "show"):
                        try:
                            prog_dialog.show("Post Processing...", "Initializing post processing... (%p%)", 0, 100)
                        except Exception:
                            pass
                if res != adsk.core.DialogResults.DialogYes:
                    _dbg_global(f"PostProcessSetup: user chose not to proceed with setup '{setup.name}' due to missing preceding stock")
                    # Answering "No" returns "Operation cancelled by user.", which causes the caller loop to abort all remaining setups.
                    return "Operation cancelled by user."

        # Ensure toolpaths are generated for active operations
        for item in target_operations:
            if not cam.checkToolpath(item):
                if progress_cb and not progress_cb("Generating missing toolpath...", 0.02):
                    return "Operation cancelled by user."
                genStat = cam.generateToolpath(item)
                start_wait = time.time()
                while not genStat.isGenerationCompleted:
                    time.sleep(.1)
                    if progress_cb and not progress_cb("Generating missing toolpath...", 0.02):
                        return "Operation cancelled by user."
                    if time.time() - start_wait > 300.0:
                        return "Toolpath generation timed out."
                    if "adsk" in sys.modules and hasattr(adsk, "doEvents"):
                        try:
                            adsk.doEvents()
                        except Exception:
                            pass

        _dbg = _dbg_global
        _dbg(f"PostProcessSetup called: fname={fname}, setup={setup.name}, opType={setup.operationType}, target_ops_cnt={len(target_operations)}")

        # Unified 2-pass dump & single-pass replay pipeline for all setups
        return _PostProcessUnifiedPipeline(fname, setup, setupFolder, docSettings, program, path, fileExt, target_operations=target_operations, progress_cb=progress_cb)

    except Exception as exc:
        _dbg_global(f"PostProcessSetup exception: {traceback.format_exc()}")
        return f"Fusion reported an exception: {exc}"
