/**
 * Universal Context & Event Dumper (universal_dumper.cps)
 *
 * Pass 1 extractor for the G-code-dialect-independent verification & replay pipeline:
 * 1. Dynamically includes BASE_POST (if provided) so the C++ Post Kernel inherits the
 *    target vendor post's exact tolerance, chord length, and circular plane settings.
 * 2. Serializes all global metadata (units, machine configuration, workpiece bounding box,
 *    global parameters, comments) and per-section metadata (Tool, WCS, workPlane, Z-range,
 *    parameters, and ordered motion/state events) into structured JSON.
 * 3. Pre-captures C++ kernel-computed expansions during active records:
 *    - `linearizedMoves` inside `onCircular()` via `linearize(tolerance)`
 *    - `reposMoves`, `afterReposMoves`, and `expandedMoves` inside `onCyclePoint()`
 *      via `repositionToCycleClearance()` and `expandCyclePoint()`
 * 4. Optionally copies `getIntermediatePath()` to `TARGET_INTERMEDIATE_CNC` so the
 *    standalone Autodesk `post` binary has a bootstrap `.cnc` file for Pass 2 replay.
 */

description = "Universal Context & Event Dumper";
vendor = "Universal";
certificationLevel = 2;
minimumRevision = 45000;
capabilities = CAPABILITY_TURNING | CAPABILITY_MILLING | CAPABILITY_INTERMEDIATE;
extension = "json";

var _dumperCfg = null;
function _getDumperConfig() {
  if (_dumperCfg !== null) return _dumperCfg;
  _dumperCfg = {};
  var cfgCandidates = [];
  try {
    var cfgEnv = getEnvironmentVariable("DUMPER_CONFIG_PATH");
    if (cfgEnv) {
      cfgCandidates.push(cfgEnv);
    }
  } catch (e) {}
  try {
    var outP = getOutputPath();
    if (outP) {
      var folder = FileSystem.getFolderPath(outP);
      if (folder) {
        cfgCandidates.push(FileSystem.getCombinedPath(folder, "universal_dumper_config.json"));
      }
    }
  } catch (e) {}

  for (var i = 0; i < cfgCandidates.length; ++i) {
    try {
      if (FileSystem.isFile(cfgCandidates[i])) {
        _dumperCfg = JSON.parse(loadText(cfgCandidates[i], "utf-8"));
        break;
      }
    } catch (e) {}
  }
  return _dumperCfg;
}

var basePostPath = "";
try {
  var cfg = _getDumperConfig();
  if (cfg && cfg.basePost) {
    basePostPath = cfg.basePost;
  }
} catch (e) {}
if (!basePostPath) {
  try {
    basePostPath = getEnvironmentVariable("BASE_POST");
  } catch (e) {}
}
if (basePostPath && FileSystem.isFile(basePostPath)) {
  try {
    include(basePostPath);
  } catch (e) {}
}

function _setVendorProperty(name, val) {
  try {
    if (typeof properties === "object" && properties && Object.prototype.hasOwnProperty.call(properties, name)) {
      if (properties[name] !== null && typeof properties[name] === "object") {
        properties[name].value = val;
        properties[name].current = val;
      } else {
        properties[name] = val;
      }
    }
  } catch (e) {}
}

function _readVendorProperty(name, expectedType, validFn) {
  function _ok(v) {
    return (typeof v === expectedType) && (!validFn || validFn(v));
  }
  try {
    if (typeof getProperty === "function") {
      var gp = getProperty(name);
      if (_ok(gp)) return gp;
    }
  } catch (e) {}
  try {
    if (typeof properties === "object" && properties && typeof properties[name] !== "undefined") {
      var p = properties[name];
      var val = (p !== null && typeof p === "object")
        ? (("current" in p && p.current !== undefined) ? p.current : p.value)
        : p;
      if (_ok(val)) return val;
    }
  } catch (e) {}
  try {
    if (typeof _cfgProps === "object" && _cfgProps && _ok(_cfgProps[name])) {
      return _cfgProps[name];
    }
  } catch (e) {}
  return null;
}

var _cfgProps = {};
try {
  var _dc = _getDumperConfig();
  if (_dc && _dc.postProperties && typeof _dc.postProperties === "object") {
    _cfgProps = _dc.postProperties;
    for (var _pk in _cfgProps) {
      if (Object.prototype.hasOwnProperty.call(_cfgProps, _pk)) {
        _setVendorProperty(_pk, _cfgProps[_pk]);
      }
    }
  }
} catch (e) {}

// Enable native stock export in Autodesk CAM post engine (CAMplete-compatible mechanism)
var _wantExportStock = false;
try {
  var _scfg = _getDumperConfig();
  if (_scfg && typeof _scfg.export_stock === "boolean") {
    _wantExportStock = _scfg.export_stock;
  } else if (_scfg && typeof _scfg.stockMode === "string") {
    var _sm = _scfg.stockMode.toLowerCase();
    if (_sm.indexOf("previous") !== -1 || _sm.indexOf("preceding") !== -1 || _sm.indexOf("prev") !== -1 || _sm.indexOf("rest") !== -1) {
      _wantExportStock = true;
    }
  } else if (typeof getConfigurationPath === "function") {
    var _cp = String(getConfigurationPath()).toLowerCase();
    if (_cp.indexOf("export_stock") !== -1) {
      _wantExportStock = true;
    }
  }
} catch (e) {}

if (typeof properties === "undefined" || !properties) {
  properties = {};
}
properties.exportStock = {
  title      : "Export stock",
  description: "Specifies whether or not to export the stock model with the project.",
  group      : "preferences",
  type       : "boolean",
  value      : _wantExportStock,
  scope      : "post"
};
try {
  exportStock = _wantExportStock;
} catch (e) {}

// Re-assert universal capabilities and JSON extension after including BASE_POST
capabilities = CAPABILITY_TURNING | CAPABILITY_MILLING | CAPABILITY_INTERMEDIATE;
extension = "json";
description = "Universal Context & Event Dumper";
vendor = "Universal";

// Reset any output formatting filters from BASE_POST so raw JSON is written cleanly
try {
  setCodePage("utf-8");
} catch (e) {}

function _resolveUseG0() {
  return _readVendorProperty("useG0", "boolean");
}

function _resolveHighFeedMapping() {
  try {
    if (typeof highFeedMapping !== "undefined") {
      return highFeedMapping;
    }
  } catch (e) {}
  return null;
}

function _getRapidMode() {
  return {
    highFeedMapping: _resolveHighFeedMapping(),
    useG0: _resolveUseG0()
  };
}

var dumpData = {
  unit: 0,
  highFeedrate: 0,
  programName: "",
  programComment: "",
  postProperties: _cfgProps,
  rapidMode: null,
  workpiece: null,
  stockPath: "",
  stockExport: {
    enabled: _wantExportStock,
    hasParam: false,
    srcPath: "",
    srcExists: false,
    destPath: "",
    error: ""
  },
  globalParams: {},
  comments: [],
  manualNC: [],
  sections: []
};

function _captureStockFile() {
  if (dumpData.stockPath && FileSystem.isFile(dumpData.stockPath)) {
    return;
  }
  var se = dumpData.stockExport || {
    enabled: !!_wantExportStock,
    hasParam: false,
    srcPath: "",
    srcExists: false,
    destPath: "",
    error: ""
  };
  dumpData.stockExport = se;
  se.enabled = !!_wantExportStock;
  se.error = "";

  try {
    var srcStock = "";
    var hasParam = false;
    if (typeof hasGlobalParameter === "function" && hasGlobalParameter("autodeskcam:stock-path")) {
      hasParam = true;
      srcStock = getGlobalParameter("autodeskcam:stock-path") || "";
    }
    se.hasParam = hasParam;
    se.srcPath = srcStock || "";

    var srcExists = false;
    if (srcStock) {
      srcExists = FileSystem.isFile(srcStock);
    }
    se.srcExists = srcExists;

    if (srcStock && srcExists) {
      var outDir = "";
      try {
        var outP = getOutputPath();
        if (outP) {
          outDir = FileSystem.getFolderPath(outP);
        }
      } catch (e) {}
      if (!outDir) {
        try {
          outDir = FileSystem.getTemporaryFolder();
        } catch (e) {}
      }
      if (!outDir) {
        outDir = "/tmp";
      }
      var pName = (typeof programFilename === "string" && programFilename) ? programFilename :
                  ((typeof programName === "string" && programName) ? programName : "setup");
      var destStock = FileSystem.getCombinedPath(outDir, pName + "_stock.stl");
      se.destPath = destStock;
      if (FileSystem.isFile(destStock)) {
        try {
          FileSystem.remove(destStock);
        } catch (ignored) {}
      }
      FileSystem.copyFile(srcStock, destStock);
      dumpData.stockPath = destStock;
      se.error = "";
    }
  } catch (e) {
    se.error = (e && e.message) ? e.message : String(e);
  }
}

var curSec = null;
var opened = false;
var pendingParams = {};
var activeCycleObj = null;
var capturingExpanded = null;

function serializeVector(v) {
  if (!v) return {x: 0, y: 0, z: 0};
  return {x: v.x, y: v.y, z: v.z};
}

function _safeToolAxisABC(sec, getterName) {
  try {
    if (
      sec &&
      sec.isMultiAxis && sec.isMultiAxis() &&
      sec.isOptimizedForMachine && sec.isOptimizedForMachine() &&
      typeof sec[getterName] === "function"
    ) {
      return serializeVector(sec[getterName]());
    }
  } catch (e) {}
  return {x: 0, y: 0, z: 0};
}

function serializeMatrix(m) {
  if (!m) {
    return {
      right: {x: 1, y: 0, z: 0},
      up: {x: 0, y: 1, z: 0},
      forward: {x: 0, y: 0, z: 1}
    };
  }
  return {
    right: serializeVector(m.right),
    up: serializeVector(m.up),
    forward: serializeVector(m.forward)
  };
}

function serializeBox(b) {
  if (!b) return null;
  return {
    lower: serializeVector(b.lower),
    upper: serializeVector(b.upper)
  };
}

onOpen = function() {
  opened = true;
  dumpData.unit = unit;
  dumpData.highFeedrate = (typeof highFeedrate === "number") ? highFeedrate : 0;
  dumpData.programName = programName;
  dumpData.programComment = programComment;
  dumpData.stockMode = "";
  dumpData.maximumSpindleSpeed = _readVendorProperty(
    "maximumSpindleSpeed",
    "number",
    function(v) { return v > 0; }
  ) || 0;
  try {
    var cfg = _getDumperConfig();
    if (cfg && cfg.stockMode) {
      dumpData.stockMode = cfg.stockMode;
    }
    if (cfg && typeof cfg.unit !== "undefined" && cfg.unit !== null) {
      unit = cfg.unit;
      dumpData.unit = unit;
    }
    if (cfg && cfg.stockUnit) {
      dumpData.stockUnit = cfg.stockUnit;
    }
    if (cfg && typeof cfg.maximumSpindleSpeed === "number" && cfg.maximumSpindleSpeed > 0) {
      dumpData.maximumSpindleSpeed = cfg.maximumSpindleSpeed;
    }
  } catch (e) {}
  dumpData.rapidMode = _getRapidMode();
  try {
    try {
      dumpData.workpieceHint = serializeBox(getWorkpiece());
    } catch (e) {
      dumpData.workpieceHint = null;
    }
    var sMode = (dumpData.stockMode || "").toLowerCase();
    var isUnsupported = sMode.indexOf("previous") !== -1 ||
                        sMode.indexOf("preceding") !== -1 ||
                        sMode.indexOf("prev") !== -1 ||
                        sMode.indexOf("rest") !== -1 ||
                        sMode.indexOf("solid") !== -1 ||
                        sMode.indexOf("body") !== -1;
    if (!isUnsupported) {
      dumpData.workpiece = dumpData.workpieceHint || serializeBox(getWorkpiece());
    } else {
      dumpData.workpiece = null;
    }
  } catch (e) {}
};

onComment = function(c) {
  if (capturingExpanded) {
    capturingExpanded.push({type: "onComment", c: c});
    return;
  }
  if (!curSec) {
    if (dumpData.sections.length === 0) {
      dumpData.comments.push(c);
    } else {
      dumpData.sections[dumpData.sections.length - 1].postComments.push(c);
    }
  } else {
    curSec.events.push({type: "onComment", c: c});
  }
};

onPassThrough = function(text) {
  if (capturingExpanded) {
    capturingExpanded.push({type: "onPassThrough", text: text});
    return;
  }
  if (curSec) {
    curSec.events.push({type: "onPassThrough", text: text});
  }
};

onParameter = function(name, value) {
  if (capturingExpanded) {
    capturingExpanded.push({type: "onParameter", name: name, value: value});
    return;
  }
  if (!opened) {
    dumpData.globalParams[name] = value;
  } else if (!curSec) {
    pendingParams[name] = value;
  } else {
    curSec.params[name] = value;
    curSec.events.push({type: "onParameter", name: name, value: value});
  }
};

onSection = function() {
  if (typeof cancelTransformation === "function") {
    try {
      cancelTransformation();
    } catch (e) {}
  }
  var t = {};
  for (var k in tool) {
    if (typeof tool[k] !== "function" && k !== "shaft" && k !== "holder") {
      t[k] = tool[k];
    }
  }
  t._manualToolChange = tool.getManualToolChange ? tool.getManualToolChange() : !!tool.manualToolChange;
  t._spindleMode = tool.getSpindleMode ? tool.getSpindleMode() : 0;
  t._taperAngle = tool.getTaperAngle ? tool.getTaperAngle() : (tool.taperAngle || 0);
  t._threadPitch = tool.getThreadPitch ? tool.getThreadPitch() : (tool.threadPitch || 0);
  t._tappingFeedrate = tool.getTappingFeedrate ? tool.getTappingFeedrate() : (tool.tappingFeedrate || 0);
  t._isTurningTool = tool.isTurningTool ? tool.isTurningTool() : false;
  t._isLiveTool = tool.isLiveTool ? tool.isLiveTool() : !!tool.liveTool;
  t._toolId = tool.getToolId ? tool.getToolId() : String(tool.number || "");
  t._description = tool.getDescription ? tool.getDescription() : (tool.description || "");
  t._comment = tool.getComment ? tool.getComment() : (tool.comment || "");
  t._extent = tool.getExtent ? serializeBox(tool.getExtent(false)) : null;
  t._cutterSVG = tool.getCutterProfileAsSVGPath ? tool.getCutterProfileAsSVGPath() : "";
  t._holderSVG = tool.getHolderProfileAsSVGPath ? tool.getHolderProfileAsSVGPath() : "";
  t._compDisplacement = tool.getCompensationDisplacement ? serializeVector(tool.getCompensationDisplacement()) : {x: 0, y: 0, z: 0};
  t._toolTypeName = (typeof getToolTypeName === "function" && tool.type !== undefined) ? getToolTypeName(tool.type) : "";

  function _serializeToolSections(secObj) {
    var secs = [];
    try {
      if (secObj && (typeof secObj.hasSections !== "function" || secObj.hasSections())) {
        var n = (typeof secObj.getNumberOfSections === "function") ? secObj.getNumberOfSections() : 0;
        for (var idx = 0; idx < n; ++idx) {
          var sLen = (typeof secObj.getLength === "function") ? secObj.getLength(idx) : 0;
          var sDia = (typeof secObj.getDiameter === "function") ? secObj.getDiameter(idx) : 0;
          secs.push({length: sLen, diameter: sDia});
        }
      }
    } catch (e) {}
    return secs;
  }
  t._holderSections = _serializeToolSections(tool.holder);
  t._shaftSections = _serializeToolSections(tool.shaft);

  var secMaxRpm = 0;
  if (typeof tool.maximumSpindleSpeed === "number" && tool.maximumSpindleSpeed > 0) {
    secMaxRpm = tool.maximumSpindleSpeed;
  } else if (typeof tool.getMaximumSpindleSpeed === "function") {
    try {
      secMaxRpm = tool.getMaximumSpindleSpeed() || 0;
    } catch (e) {}
  }
  if (!secMaxRpm && dumpData.maximumSpindleSpeed) {
    secMaxRpm = dumpData.maximumSpindleSpeed;
  }
  var secSurfSpeed = 0;
  if (typeof tool.surfaceSpeed === "number" && tool.surfaceSpeed > 0) {
    secSurfSpeed = tool.surfaceSpeed;
  } else if (typeof tool.getSurfaceSpeed === "function") {
    try {
      secSurfSpeed = tool.getSurfaceSpeed() || 0;
    } catch (e) {}
  }
  t.maximumSpindleSpeed = secMaxRpm;
  t.surfaceSpeed = secSurfSpeed;

  var zRange = currentSection.getGlobalZRange ? currentSection.getGlobalZRange() : null;
  var isMulti = currentSection.isMultiAxis ? currentSection.isMultiAxis() : false;

  curSec = {
    id: currentSection.getId(),
    type: currentSection.getType(),
    quality: currentSection.quality || 0,
    jetMode: currentSection.jetMode || 0,
    workOffset: currentSection.workOffset,
    wcs: currentSection.wcs,
    probeWorkOffset: currentSection.probeWorkOffset,
    workOrigin: serializeVector(currentSection.workOrigin),
    workPlane: serializeMatrix(currentSection.workPlane),
    spindle: currentSection.spindle,
    feedMode: currentSection.feedMode,
    maximumSpindleSpeed: secMaxRpm,
    surfaceSpeed: secSurfSpeed,
    toolOrientation: currentSection.toolOrientation || 0,
    partCatcher: currentSection.partCatcher,
    tailstock: currentSection.tailstock,
    strategy: currentSection.strategy,
    initialPosition: serializeVector(currentSection.getInitialPosition()),
    globalInitialPosition: serializeVector(
      currentSection.getGlobalPosition
        ? currentSection.getGlobalPosition(currentSection.getInitialPosition())
        : (currentSection.getGlobalInitialPosition ? currentSection.getGlobalInitialPosition() : currentSection.getInitialPosition())
    ),
    finalPosition: serializeVector(currentSection.getFinalPosition()),
    globalFinalPosition: serializeVector(
      currentSection.getGlobalPosition
        ? currentSection.getGlobalPosition(currentSection.getFinalPosition())
        : (currentSection.getGlobalFinalPosition ? currentSection.getGlobalFinalPosition() : currentSection.getFinalPosition())
    ),
    initialToolAxis: currentSection.getInitialToolAxis ? serializeVector(currentSection.getInitialToolAxis()) : {x: 0, y: 0, z: 1},
    globalInitialToolAxis: currentSection.getGlobalInitialToolAxis ? serializeVector(currentSection.getGlobalInitialToolAxis()) : {x: 0, y: 0, z: 1},
    finalToolAxis: currentSection.getFinalToolAxis ? serializeVector(currentSection.getFinalToolAxis()) : {x: 0, y: 0, z: 1},
    globalFinalToolAxis: currentSection.getGlobalFinalToolAxis ? serializeVector(currentSection.getGlobalFinalToolAxis()) : {x: 0, y: 0, z: 1},
    initialToolAxisABC: _safeToolAxisABC(currentSection, "getInitialToolAxisABC"),
    finalToolAxisABC: _safeToolAxisABC(currentSection, "getFinalToolAxisABC"),
    forceToolChange: currentSection.getForceToolChange ? currentSection.getForceToolChange() : false,
    movements: currentSection.getMovements ? currentSection.getMovements() : 0,
    patternId: currentSection.getPatternId ? currentSection.getPatternId() : 0,
    internalPatternId: currentSection.getInternalPatternId ? currentSection.getInternalPatternId() : 0,
    numberOfPatternInstances: currentSection.getNumberOfPatternInstances ? currentSection.getNumberOfPatternInstances() : 1,
    isPatterned: currentSection.isPatterned ? currentSection.isPatterned() : false,
    hasAnyCycle: currentSection.hasAnyCycle ? currentSection.hasAnyCycle() : false,
    isOptional: currentSection.isOptional ? currentSection.isOptional() : false,
    isMultiAxis: isMulti,
    isOptimizedForMachine: currentSection.isOptimizedForMachine ? currentSection.isOptimizedForMachine() : false,
    optimizedTCPMode: currentSection.getOptimizedTCPMode ? currentSection.getOptimizedTCPMode() : 0,
    zRangeMin: zRange ? zRange.getMinimum() : 0,
    zRangeMax: zRange ? zRange.getMaximum() : 0,
    initSpindleSpeed: spindleSpeed,
    initMovement: movement,
    initRadiusComp: radiusCompensation,
    params: pendingParams,
    tool: t,
    events: [],
    postComments: [],
    postManualNC: []
  };
  pendingParams = {};
  dumpData.sections.push(curSec);
};

onSpindleSpeed = function(s) {
  curSec.events.push({type: "onSpindleSpeed", s: s});
};

onOrientateSpindle = function(angle) {
  curSec.events.push({type: "onOrientateSpindle", angle: angle});
};

onMovement = function(m) {
  if (capturingExpanded) return;
  curSec.events.push({type: "onMovement", m: m});
};

onFeedMode = function(fm) {
  curSec.events.push({type: "onFeedMode", fm: fm});
};

onRadiusCompensation = function() {
  curSec.events.push({type: "onRadiusCompensation", rc: radiusCompensation});
};

onDwell = function(sec) {
  if (capturingExpanded) {
    capturingExpanded.push({type: "onDwell", sec: sec});
    return;
  }
  curSec.events.push({type: "onDwell", sec: sec});
};

onCommand = function(cmd) {
  if (capturingExpanded) {
    capturingExpanded.push({type: "onCommand", cmd: cmd});
    return;
  }
  curSec.events.push({type: "onCommand", cmd: cmd});
};

onManualNC = function(command, value) {
  var exp = [];
  capturingExpanded = exp;
  try {
    if (typeof expandManualNC === "function") {
      expandManualNC(command, value);
    }
  } catch (e) {}
  capturingExpanded = null;
  var rec = {type: "onManualNC", command: command, value: value, expanded: exp};
  if (!curSec) {
    if (dumpData.sections.length === 0) {
      dumpData.manualNC.push(rec);
    } else {
      dumpData.sections[dumpData.sections.length - 1].postManualNC.push(rec);
    }
  } else {
    curSec.events.push(rec);
  }
};

onRapid = function(x, y, z) {
  if (capturingExpanded) {
    capturingExpanded.push({type: "onRapid", x: x, y: y, z: z, movement: movement});
    return;
  }
  var start = getCurrentPosition();
  curSec.events.push({
    type: "onRapid", x: x, y: y, z: z,
    sx: start.x, sy: start.y, sz: start.z,
    movement: movement, rc: radiusCompensation, rpm: spindleSpeed
  });
};

onLinear = function(x, y, z, feed) {
  if (capturingExpanded) {
    capturingExpanded.push({type: "onLinear", x: x, y: y, z: z, feed: feed, movement: movement});
    return;
  }
  var start = getCurrentPosition();
  curSec.events.push({
    type: "onLinear", x: x, y: y, z: z, feed: feed,
    sx: start.x, sy: start.y, sz: start.z,
    movement: movement, rc: radiusCompensation, rpm: spindleSpeed,
    sync: (typeof isSpeedFeedSynchronizationActive === "function") ? isSpeedFeedSynchronizationActive() : false
  });
};

onRapid5D = function(x, y, z, a, b, c) {
  if (curSec) {
    curSec.has5DMoves = true;
  }
};

onLinear5D = function(x, y, z, a, b, c, feed) {
  if (curSec) {
    curSec.has5DMoves = true;
  }
};

onCircular = function(cw, cx, cy, cz, x, y, z, feed) {
  var start = getCurrentPosition();
  var n = getCircularNormal();
  var plane = getCircularPlane();
  var radius = getCircularRadius();
  var sweep = getCircularSweep();
  var fullCircle = isFullCircle();
  var helical = isHelical();
  var spiral = isSpiral();
  var curMov = movement;
  var curRc = radiusCompensation;
  var curRpm = spindleSpeed;

  var linMoves = [];
  capturingExpanded = linMoves;
  try {
    var tol = (typeof tolerance === "number" && tolerance > 0) ? tolerance : spatial(0.002, MM);
    linearize(tol);
  } catch (e) {}
  capturingExpanded = null;
  setCurrentPosition(new Vector(x, y, z));

  curSec.events.push({
    type: "onCircular", cw: cw, cx: cx, cy: cy, cz: cz, x: x, y: y, z: z, feed: feed,
    sx: start.x, sy: start.y, sz: start.z,
    plane: plane,
    radius: radius,
    sweep: sweep,
    nx: n.x, ny: n.y, nz: n.z,
    fullCircle: fullCircle,
    helical: helical,
    spiral: spiral,
    movement: curMov, rc: curRc, rpm: curRpm,
    linearizedMoves: linMoves
  });
};

onCycle = function() {
  var c = {};
  for (var k in cycle) {
    if (typeof cycle[k] !== "function") {
      var val = cycle[k];
      if (val && typeof val === "object" && ("x" in val) && ("y" in val) && ("z" in val)) {
        c[k] = {x: val.x, y: val.y, z: val.z, _isVec: true};
      } else {
        c[k] = val;
      }
    }
  }
  activeCycleObj = {type: "onCycle", cycleType: cycleType, cycle: c, points: []};
  curSec.events.push(activeCycleObj);
};

onCyclePoint = function(x, y, z) {
  var start = getCurrentPosition();
  var first = isFirstCyclePoint();
  var last = isLastCyclePoint();
  var expMoves = [];
  var reposMoves = [];
  var afterReposMoves = [];

  var ctStr = (typeof cycleType === "string") ? cycleType.toLowerCase() : "";
  var isProbeOrInspect = false;
  try {
    isProbeOrInspect = (ctStr.indexOf("prob") !== -1 || ctStr.indexOf("inspect") !== -1) ||
      (typeof isProbeOperation === "function" && isProbeOperation()) ||
      (typeof isInspectionOperation === "function" && isInspectionOperation()) ||
      !!(curSec && curSec.params && (curSec.params["operation:isProbingStrategy"] == 1 || curSec.params["operation:isInspectionStrategy"] == 1));
  } catch (e) {
    isProbeOrInspect = (ctStr.indexOf("prob") !== -1 || ctStr.indexOf("inspect") !== -1) ||
      !!(curSec && curSec.params && (curSec.params["operation:isProbingStrategy"] == 1 || curSec.params["operation:isInspectionStrategy"] == 1));
  }

  if (!isProbeOrInspect) {
    capturingExpanded = reposMoves;
    try {
      repositionToCycleClearance(cycle, x, y, z);
    } catch (e) {}
    if (reposMoves.length > 0) {
      capturingExpanded = afterReposMoves;
      try {
        expandCyclePoint(x, y, z);
      } catch (e) {}
    }
    capturingExpanded = null;

    setCurrentPosition(start);
    capturingExpanded = expMoves;
    try {
      expandCyclePoint(x, y, z);
    } catch (e) {}
    capturingExpanded = null;
  }
  setCurrentPosition(new Vector(x, y, z));

  var pt = {
    type: "onCyclePoint", x: x, y: y, z: z,
    sx: start.x, sy: start.y, sz: start.z,
    first: first, last: last,
    reposMoves: reposMoves,
    expandedMoves: expMoves
  };
  if (reposMoves.length > 0) {
    pt.afterReposMoves = afterReposMoves;
  }
  if (activeCycleObj) {
    activeCycleObj.points.push({x: x, y: y, z: z});
  }
  curSec.events.push(pt);
};

onCycleEnd = function() {
  activeCycleObj = null;
  curSec.events.push({type: "onCycleEnd"});
};

onSectionEnd = function() {
  var endPos = getCurrentPosition();
  curSec.endPos = serializeVector(endPos);
  curSec.endSpindleSpeed = spindleSpeed;
  if (typeof currentSection !== "undefined" && currentSection) {
    if (currentSection.getFinalPosition) {
      var fpEnd = currentSection.getFinalPosition();
      curSec.finalPosition = serializeVector(fpEnd);
      if (currentSection.getGlobalPosition) {
        curSec.globalFinalPosition = serializeVector(currentSection.getGlobalPosition(fpEnd));
      } else if (currentSection.getGlobalFinalPosition) {
        curSec.globalFinalPosition = serializeVector(currentSection.getGlobalFinalPosition());
      }
    } else if (currentSection.getGlobalFinalPosition) {
      curSec.globalFinalPosition = serializeVector(currentSection.getGlobalFinalPosition());
    }
    if (currentSection.getFinalToolAxis) {
      curSec.finalToolAxis = serializeVector(currentSection.getFinalToolAxis());
    }
    if (currentSection.getGlobalFinalToolAxis) {
      curSec.globalFinalToolAxis = serializeVector(currentSection.getGlobalFinalToolAxis());
    }
    curSec.finalToolAxisABC = _safeToolAxisABC(currentSection, "getFinalToolAxisABC");
  }
  curSec = null;
};

onClose = function() {
  // Capture native exported stock STL if available before writing dumpData
  _captureStockFile();

  // Write raw JSON without any comment or line filters
  writeln(JSON.stringify(dumpData));

  // Optionally copy the binary Compact-NCI intermediate file for bootstrap replay
  var targetCnc = "";
  try {
    var cfg = _getDumperConfig();
    if (cfg && cfg.targetIntermediateCnc) {
      targetCnc = cfg.targetIntermediateCnc;
    }
  } catch (e) {}
  if (!targetCnc) {
    try {
      targetCnc = getEnvironmentVariable("TARGET_INTERMEDIATE_CNC");
    } catch (e) {}
  }
  if (targetCnc) {
    try {
      var intermediateSrc = getIntermediatePath();
      if (intermediateSrc && FileSystem.isFile(intermediateSrc)) {
        if (FileSystem.isFile(targetCnc)) {
          try {
            FileSystem.remove(targetCnc);
          } catch (ignored) {}
        }
        FileSystem.copyFile(intermediateSrc, targetCnc);
      }
    } catch (e) {}
  }
};
