/**
 * Universal Context & Event Replayer (universal_replayer.cps)
 *
 * Executes any unmodified vendor `.cps` post processor (`BASE_POST`) against a
 * combined JSON context (`CONTEXT_JSON`) produced by merging one or more
 * `universal_dumper.cps` outputs.
 *
 * Key capabilities:
 * - Evaluates the stock vendor `.cps` inside a lexical closure (`buildVendor`)
 *   that shadows read-only C++ Post Kernel globals (`currentSection`, `tool`,
 *   `movement`, `radiusCompensation`, `spindleSpeed`, `cycle`, `cycleType`,
 *   `cycleExpanded`, `programName`, `programComment`) while preserving direct
 *   access to all C++ Post Kernel APIs (`Vector`, `Matrix`, `BoundingBox`,
 *   `createFormat`, `createModal`, `createOutputVariable`, `writeBlock`, etc.).
 * - Synthesizes `getNumberOfSections()`, `getSection()`, `getToolTable()`,
 *   `getWorkpiece()`, `getProperty()`, `setProperty()`, `expandCyclePoint()`,
 *   `repositionToCycleClearance()`, and `linearize()` across merged multi-tool
 *   sections.
 * - Supports rapid promotion via direct event annotation (`ev.promote = true`).
 */

description = "Universal Context & Event Replayer";
vendor = "Universal";
certificationLevel = 2;
minimumRevision = 45000;
capabilities = CAPABILITY_TURNING | CAPABILITY_MILLING | CAPABILITY_INTERMEDIATE;
extension = "nc";

function loadLargeText(path, encoding) {
  if (!path) return "";
  try {
    if (typeof FileSystem !== "undefined" && FileSystem.isFile(path) && FileSystem.getFileSize(path) < 800000) {
      return loadText(path, encoding || "utf-8");
    }
  } catch (e) {
    // Fall back to streaming TextFile
  }

  // Stream via TextFile to bypass the Autodesk Post Processor kernel 1 MB buffer limit
  var file = new TextFile(path, false, encoding || "utf-8");
  var lines = [];
  while (true) {
    try {
      var l = file.readln();
      lines.push(l);
    } catch (e) {
      break;
    }
  }
  file.close();
  return lines.join("\n");
}

function toVec(v) {
  if (!v) return new Vector(0, 0, 0);
  return new Vector(v.x, v.y, v.z);
}

function toMat(m) {
  if (!m) return new Matrix();
  return new Matrix(toVec(m.right), toVec(m.up), toVec(m.forward));
}

function toBox(b) {
  if (!b) return new BoundingBox(new Vector(0, 0, 0), new Vector(0, 0, 0));
  return new BoundingBox(toVec(b.lower), toVec(b.upper));
}

function makeTool(t) {
  var obj = {};
  for (var k in t) {
    obj[k] = t[k];
  }
  obj.getNumber = function() { return t.number; };
  obj.getCompensationOffset = function() { return t.compensationOffset; };
  obj.getLengthOffset = function() { return t.lengthOffset; };
  obj.getDiameterOffset = function() { return t.diameterOffset; };
  obj.getTurret = function() { return t.turret; };
  obj.getType = function() { return t.type; };
  obj.getDescription = function() { return t._description; };
  obj.getComment = function() { return t._comment; };
  obj.getHolderDescription = function() { return t.holderDescription || ""; };
  obj.getDiameter = function() { return t.diameter; };
  obj.getTipAngle = function() { return t.tipAngle; };
  obj.getCornerRadius = function() { return t.cornerRadius; };
  obj.getFluteLength = function() { return t.fluteLength; };
  obj.getShoulderLength = function() { return t.shoulderLength; };
  obj.getShaftDiameter = function() { return t.shaftDiameter; };
  obj.getBodyLength = function() { return t.bodyLength; };
  obj.getOverallLength = function() { return t.overallLength; };
  obj.getHolderLength = function() { return t.holderLength; };
  obj.getNumberOfFlutes = function() { return t.numberOfFlutes; };
  obj.getSpindleRPM = function() { return t.spindleRPM; };
  obj.getSurfaceSpeed = function() { return t.surfaceSpeed; };
  obj.getMaximumSpindleSpeed = function() { return t.maximumSpindleSpeed; };
  obj.isClockwise = function() { return t.clockwise; };
  obj.getCoolant = function() { return t.coolant; };
  obj.getMaterial = function() { return t.material; };
  obj.getManualToolChange = function() { return t._manualToolChange; };
  obj.isManualToolChange = function() { return t._manualToolChange; };
  obj.hasBreakControl = function() { return !!t.breakControl; };
  obj.getSpindleMode = function() { return t._spindleMode; };
  obj.getTaperAngle = function() { return t._taperAngle; };
  obj.getThreadPitch = function() { return t._threadPitch; };
  obj.getTappingFeedrate = function() { return t._tappingFeedrate; };
  obj.isTurningTool = function() { return t._isTurningTool; };
  obj.isLiveTool = function() { return t._isLiveTool; };
  obj.getToolId = function() { return t._toolId; };
  obj.getInscribedCircleDiameter = function() { return t.inscribedCircleDiameter; };
  obj.getEdgeLength = function() { return t.edgeLength; };
  obj.getNoseRadius = function() { return t.noseRadius; };
  obj.getReliefAngle = function() { return t.reliefAngle; };
  obj.getThickness = function() { return t.thickness; };
  obj.getGrooveWidth = function() { return t.grooveWidth; };
  obj.getCrossSection = function() { return t.crossSection || ""; };
  obj.getTolerance = function() { return t.tolerance || ""; };
  obj.getHand = function() { return t.hand || ""; };
  obj.getClamping = function() { return t.clamping || ""; };
  obj.getExtent = function() { return toBox(t._extent); };
  return obj;
}

function makeSection(s, idx, allRawSections) {
  var toolObj = makeTool(s.tool);
  var wp = toMat(s.workPlane);
  var wo = toVec(s.workOrigin);
  return {
    id: idx,
    type: s.type,
    quality: s.quality || 0,
    jetMode: s.jetMode || 0,
    workOffset: s.workOffset,
    wcs: s.wcs,
    probeWorkOffset: s.probeWorkOffset,
    workOrigin: wo,
    workPlane: wp,
    spindle: s.spindle,
    feedMode: s.feedMode,
    toolOrientation: s.toolOrientation || 0,
    partCatcher: s.partCatcher,
    tailstock: s.tailstock,
    strategy: s.strategy,
    getId: function() { return idx; },
    getType: function() { return s.type; },
    getMachiningType: function() { return s.type; },
    getQuality: function() { return s.quality || 0; },
    getJetMode: function() { return s.jetMode || 0; },
    getWorkOffset: function() { return s.workOffset; },
    getWCS: function() { return s.wcs; },
    getProbeWorkOffset: function() { return s.probeWorkOffset; },
    getWorkOrigin: function() { return wo; },
    getGlobalWorkOrigin: function() { return wo; },
    getWorkPlane: function() { return wp; },
    getGlobalWorkPlane: function() { return wp; },
    getWCSOrigin: function() { return wo; },
    getWCSPlane: function() { return wp; },
    getSpindle: function() { return s.spindle; },
    getFeedMode: function() { return s.feedMode; },
    toolOrientation: s.toolOrientation || 0,
    getToolOrientation: function() { return s.toolOrientation || 0; },
    getPartCatcher: function() { return s.partCatcher; },
    getTailstock: function() { return s.tailstock; },
    getStrategy: function() { return s.strategy; },
    getTool: function() { return toolObj; },
    getInitialPosition: function() { return toVec(s.initialPosition); },
    getGlobalInitialPosition: function() {
      var ip = toVec(s.initialPosition);
      var calcGlobal = Vector.sum(wp.multiply(ip), wo);
      if (s.globalInitialPosition) {
        var gip = toVec(s.globalInitialPosition);
        // Legacy dumps before getGlobalPosition() fix stored raw initialPosition in globalInitialPosition;
        // prefer calculated global position if globalInitialPosition equals raw initialPosition under a non-trivial frame.
        if (Vector.diff(gip, ip).length > 1e-6 || Vector.diff(calcGlobal, ip).length <= 1e-6) {
          return gip;
        }
      }
      return calcGlobal;
    },
    getFinalPosition: function() { return toVec(s.finalPosition || s.endPos || s.initialPosition); },
    getGlobalFinalPosition: function() {
      var fp = toVec(s.finalPosition || s.endPos || s.initialPosition);
      var calcGlobal = Vector.sum(wp.multiply(fp), wo);
      if (s.globalFinalPosition) {
        var gfp = toVec(s.globalFinalPosition);
        if (Vector.diff(gfp, fp).length > 1e-6 || Vector.diff(calcGlobal, fp).length <= 1e-6) {
          return gfp;
        }
      }
      return calcGlobal;
    },
    getFirstPosition: function() { return toVec(s.initialPosition); },
    getGlobalPosition: function(v) {
      if (!v) return Vector.sum(wp.multiply(toVec(s.initialPosition)), wo);
      return Vector.sum(wp.multiply(v), wo);
    },
    getWorkPosition: function(v) {
      if (!v) return new Vector(0, 0, 0);
      return wp.getTransposed().multiply(Vector.diff(v, wo));
    },
    getBoundingBox: function() {
      var ip = toVec(s.initialPosition);
      var minX = ip.x, minY = ip.y, minZ = ip.z;
      var maxX = ip.x, maxY = ip.y, maxZ = ip.z;
      function _expandPt(px, py, pz) {
        if (typeof px === "number" && typeof py === "number" && typeof pz === "number") {
          if (px < minX) minX = px;
          if (py < minY) minY = py;
          if (pz < minZ) minZ = pz;
          if (px > maxX) maxX = px;
          if (py > maxY) maxY = py;
          if (pz > maxZ) maxZ = pz;
        }
      }
      var evs = s.events || [];
      for (var bi = 0; bi < evs.length; ++bi) {
        var bev = evs[bi];
        if (!bev) continue;
        _expandPt(bev.sx, bev.sy, bev.sz);
        _expandPt(bev.x, bev.y, bev.z);
        if (bev.type === "onCircular") {
          var lms = bev.linearizedMoves;
          if (lms && lms.length > 0) {
            for (var lmi = 0; lmi < lms.length; ++lmi) {
              var lm = lms[lmi];
              if (lm) {
                _expandPt(lm.sx, lm.sy, lm.sz);
                _expandPt(lm.x, lm.y, lm.z);
              }
            }
          } else if (typeof bev.cx === "number" && typeof bev.cy === "number" && typeof bev.cz === "number") {
            var r = (typeof bev.radius === "number" && bev.radius > 0) ? bev.radius : 0;
            var plane = bev.plane;
            if (plane === undefined || plane === null || plane === -1) {
              var nx = (typeof bev.nx === "number") ? Math.abs(bev.nx) : 0;
              var ny = (typeof bev.ny === "number") ? Math.abs(bev.ny) : 0;
              var nz = (typeof bev.nz === "number") ? Math.abs(bev.nz) : 1;
              if (nx < 1e-5 && ny < 1e-5 && Math.abs(nz - 1) < 1e-4) plane = 0;
              else if (nx < 1e-5 && nz < 1e-5 && Math.abs(ny - 1) < 1e-4) plane = 1;
              else if (ny < 1e-5 && nz < 1e-5 && Math.abs(nx - 1) < 1e-4) plane = 2;
              else plane = -1;
            }
            if (r <= 0 && typeof bev.sx === "number" && typeof bev.sy === "number" && typeof bev.sz === "number") {
              var dx = bev.sx - bev.cx, dy = bev.sy - bev.cy, dz = bev.sz - bev.cz;
              r = Math.sqrt(dx * dx + dy * dy + dz * dz);
            }
            if (r > 0) {
              if (plane === 0) {
                _expandPt(bev.cx - r, bev.cy - r, bev.cz);
                _expandPt(bev.cx + r, bev.cy + r, bev.cz);
              } else if (plane === 1) {
                _expandPt(bev.cx - r, bev.cy, bev.cz - r);
                _expandPt(bev.cx + r, bev.cy, bev.cz + r);
              } else if (plane === 2) {
                _expandPt(bev.cx, bev.cy - r, bev.cz - r);
                _expandPt(bev.cx, bev.cy + r, bev.cz + r);
              } else {
                _expandPt(bev.cx - r, bev.cy - r, bev.cz - r);
                _expandPt(bev.cx + r, bev.cy + r, bev.cz + r);
              }
            }
          }
        } else if (bev.type === "onCyclePoint") {
          var ems = bev.expandedMoves;
          if (ems && ems.length > 0) {
            for (var emi = 0; emi < ems.length; ++emi) {
              var em = ems[emi];
              if (em) {
                _expandPt(em.sx, em.sy, em.sz);
                _expandPt(em.x, em.y, em.z);
              }
            }
          }
        }
      }
      return new BoundingBox(new Vector(minX, minY, minZ), new Vector(maxX, maxY, maxZ));
    },
    getGlobalBoundingBox: function() {
      var lbb = this.getBoundingBox();
      var corners = [
        new Vector(lbb.lower.x, lbb.lower.y, lbb.lower.z),
        new Vector(lbb.upper.x, lbb.lower.y, lbb.lower.z),
        new Vector(lbb.lower.x, lbb.upper.y, lbb.lower.z),
        new Vector(lbb.upper.x, lbb.upper.y, lbb.lower.z),
        new Vector(lbb.lower.x, lbb.lower.y, lbb.upper.z),
        new Vector(lbb.upper.x, lbb.lower.y, lbb.upper.z),
        new Vector(lbb.lower.x, lbb.upper.y, lbb.upper.z),
        new Vector(lbb.upper.x, lbb.upper.y, lbb.upper.z)
      ];
      var g0 = Vector.sum(wp.multiply(corners[0]), wo);
      var minX = g0.x, minY = g0.y, minZ = g0.z;
      var maxX = g0.x, maxY = g0.y, maxZ = g0.z;
      for (var ci = 1; ci < corners.length; ++ci) {
        var gc = Vector.sum(wp.multiply(corners[ci]), wo);
        if (gc.x < minX) minX = gc.x;
        if (gc.y < minY) minY = gc.y;
        if (gc.z < minZ) minZ = gc.z;
        if (gc.x > maxX) maxX = gc.x;
        if (gc.y > maxY) maxY = gc.y;
        if (gc.z > maxZ) maxZ = gc.z;
      }
      return new BoundingBox(new Vector(minX, minY, minZ), new Vector(maxX, maxY, maxZ));
    },
    getInitialToolAxis: function() { return toVec(s.initialToolAxis || {x: 0, y: 0, z: 1}); },
    getGlobalInitialToolAxis: function() { return toVec(s.globalInitialToolAxis || s.initialToolAxis || {x: 0, y: 0, z: 1}); },
    getInitialToolAxisABC: function() { return toVec(s.initialToolAxisABC || {x: 0, y: 0, z: 0}); },
    getFinalToolAxis: function() { return toVec(s.finalToolAxis || s.initialToolAxis || {x: 0, y: 0, z: 1}); },
    getGlobalFinalToolAxis: function() { return toVec(s.globalFinalToolAxis || s.globalInitialToolAxis || s.finalToolAxis || s.initialToolAxis || {x: 0, y: 0, z: 1}); },
    getFinalToolAxisABC: function() { return toVec(s.finalToolAxisABC || s.initialToolAxisABC || {x: 0, y: 0, z: 0}); },
    getABCByPreference: function() { return toVec(s.initialToolAxisABC || {x: 0, y: 0, z: 0}); },
    getForceToolChange: function() { return !!s.forceToolChange; },
    getMovements: function() { return s.movements || 0; },
    getPatternId: function() { return s.patternId || 0; },
    getInternalPatternId: function() { return s.internalPatternId || 0; },
    getNumberOfPatternInstances: function() {
      if (s.numberOfPatternInstances && s.numberOfPatternInstances > 1) {
        return s.numberOfPatternInstances;
      }
      if (s.patternId && allRawSections && allRawSections.length) {
        var count = 0;
        for (var pi = 0; pi < allRawSections.length; ++pi) {
          if (allRawSections[pi].patternId === s.patternId) count++;
        }
        if (count > 0) return count;
      }
      return 1;
    },
    isPatterned: function() { return !!s.isPatterned; },
    hasAnyCycle: function() { return !!s.hasAnyCycle; },
    hasCycle: function() { return !!s.hasAnyCycle; },
    isOptional: function() { return !!s.isOptional; },
    isMultiAxis: function() { return !!s.isMultiAxis; },
    isOptimizedForMachine: function() { return !!s.isOptimizedForMachine; },
    getOptimizedTCPMode: function() { return s.optimizedTCPMode || 0; },
    isConnectionSection: function() { return false; },
    hasRewind: function() { return false; },
    isCuttingMotionAwayFromRotary: function() { return false; },
    doesToolpathFitWithinLimits: function() { return true; },
    optimize3DPositionsByMachine: function() {},
    optimizeMachineAnglesByMachine: function() {},
    getGlobalZRange: function() {
      var minVal = (s.zRangeMin !== undefined) ? s.zRangeMin : 0;
      var maxVal = (s.zRangeMax !== undefined) ? s.zRangeMax : 0;
      return {
        getMinimum: function() { return minVal; },
        getMaximum: function() { return maxVal; },
        expandToRange: function(other) {
          if (other.getMinimum() < minVal) minVal = other.getMinimum();
          if (other.getMaximum() > maxVal) maxVal = other.getMaximum();
        }
      };
    },
    getZRange: function() {
      var minVal = (s.zRangeMin !== undefined) ? s.zRangeMin : 0;
      var maxVal = (s.zRangeMax !== undefined) ? s.zRangeMax : 0;
      return {
        getMinimum: function() { return minVal; },
        getMaximum: function() { return maxVal; },
        expandToRange: function(other) {
          if (other.getMinimum() < minVal) minVal = other.getMinimum();
          if (other.getMaximum() > maxVal) maxVal = other.getMaximum();
        }
      };
    },
    hasParameter: function(name) {
      return !!(s.params && Object.prototype.hasOwnProperty.call(s.params, name));
    },
    getParameter: function(name, def) {
      return (s.params && Object.prototype.hasOwnProperty.call(s.params, name)) ? s.params[name] : def;
    },
    getProperty: function(name) { return undefined; },
    checkGroup: function() { return false; }
  };
}

function hydrateCycle(c) {
  if (!c) return undefined;
  var res = {};
  for (var k in c) {
    var val = c[k];
    if (val && typeof val === "object" && val._isVec) {
      res[k] = new Vector(val.x, val.y, val.z);
    } else {
      res[k] = val;
    }
  }
  return res;
}

// Suppress C++ kernel callbacks on the bootstrap .cnc file until onClose()
function onOpen() {}
function onSection() {}
function onSpindleSpeed() {}
function onOrientateSpindle() {}
function onMovement() {}
function onFeedMode() {}
function onRadiusCompensation() {}
function onDwell() {}
function onCommand() {}
function onRapid() {}
function onLinear() {}
function onCircular() {}
function onCycle() {}
function onCyclePoint() {}
function onCycleEnd() {}
function onSectionEnd() {}

function onClose() {
  var jsonPath = getEnvironmentVariable("CONTEXT_JSON");
  var basePostPath = getEnvironmentVariable("BASE_POST");

  var data = JSON.parse(loadLargeText(jsonPath, "utf-8"));
  var vendorCode = loadLargeText(basePostPath, "utf-8");

  var sections = [];
  for (var i = 0; i < data.sections.length; ++i) {
    sections.push(makeSection(data.sections[i], i, data.sections));
  }

  var uniqueTools = [];
  var seenToolNums = {};
  for (var ti = 0; ti < sections.length; ++ti) {
    var st = sections[ti].getTool();
    if (!seenToolNums[st.number]) {
      seenToolNums[st.number] = true;
      uniqueTools.push(st);
    }
  }

  var curLocalPos = new Vector(0, 0, 0);
  var curEv = null;
  var curCyclePoints = [];
  var curSync = false;

  function _fp(x, y, z) {
    var v = new Vector(x, y, z);
    return (typeof getFramePosition === "function") ? getFramePosition(v) : v;
  }
  function _fpVec(v) {
    var vec = toVec(v);
    return (typeof getFramePosition === "function") ? getFramePosition(vec) : vec;
  }
  function _invFpVec(v) {
    var vec = toVec(v);
    var t = (typeof getTranslation === "function") ? getTranslation() : new Vector(0, 0, 0);
    var shifted = Vector.diff(vec, t);
    if (typeof getRotation === "function") {
      var r = getRotation();
      if (r && typeof r.getTransposed === "function") {
        return r.getTransposed().multiply(shifted);
      }
    }
    return shifted;
  }
  function _fd(x, y, z) {
    var v = new Vector(x, y, z);
    return (typeof getFrameDirection === "function") ? getFrameDirection(v) : v;
  }
  function _rotDet() {
    if (typeof getRotation !== "function") return 1;
    var m = getRotation();
    if (!m || !m.right || !m.up || !m.forward) return 1;
    return (
      m.right.x * (m.up.y * m.forward.z - m.up.z * m.forward.y) -
      m.up.x * (m.right.y * m.forward.z - m.right.z * m.forward.y) +
      m.forward.x * (m.right.y * m.up.z - m.right.z * m.up.y)
    );
  }
  function _transformCircular(ev) {
    var nx0 = (ev && typeof ev.nx === "number") ? ev.nx : 0;
    var ny0 = (ev && typeof ev.ny === "number") ? ev.ny : 0;
    var nz0 = (ev && typeof ev.nz === "number") ? ev.nz : 1;
    var n = _fd(nx0, ny0, nz0);
    var cw = !!(ev && ev.cw);
    if (_rotDet() < 0) {
      cw = !cw;
    }
    var plane = -1;
    if (Math.abs(n.x) < 1e-5 && Math.abs(n.y) < 1e-5 && Math.abs(Math.abs(n.z) - 1) < 1e-4) {
      if (n.z < 0) cw = !cw;
      n = new Vector(0, 0, 1);
      plane = 0;
    } else if (Math.abs(n.x) < 1e-5 && Math.abs(n.z) < 1e-5 && Math.abs(Math.abs(n.y) - 1) < 1e-4) {
      if (n.y < 0) cw = !cw;
      n = new Vector(0, 1, 0);
      plane = 1;
    } else if (Math.abs(n.y) < 1e-5 && Math.abs(n.z) < 1e-5 && Math.abs(Math.abs(n.x) - 1) < 1e-4) {
      if (n.x < 0) cw = !cw;
      n = new Vector(1, 0, 0);
      plane = 2;
    }
    return { cw: cw, normal: n, plane: plane };
  }

  getToolTable = function() {
    return {
      getNumberOfTools: function() { return uniqueTools.length; },
      getTool: function(idx) { return uniqueTools[idx]; }
    };
  };
  getNumberOfSections = function() { return sections.length; };
  getSection = function(id) { return sections[id]; };
  hasGlobalParameter = function(name) {
    return Object.prototype.hasOwnProperty.call(data.globalParams, name);
  };
  getGlobalParameter = function(name, def) {
    return Object.prototype.hasOwnProperty.call(data.globalParams, name) ? data.globalParams[name] : def;
  };
  getWorkpiece = function() { return toBox(data.workpiece); };
  getGlobalPosition = function(v) {
    if (currentSection && typeof currentSection.getGlobalPosition === "function") {
      return currentSection.getGlobalPosition(v);
    }
    return toVec(v);
  };

  var nextEvIsMotion = true;
  getNextRecord = function() {
    return {
      isMotion: function() { return nextEvIsMotion; }
    };
  };
  getCurrentPosition = function() { return _fpVec(curLocalPos); };
  setCurrentPosition = function(v) { curLocalPos = _invFpVec(v); };
  isSpeedFeedSynchronizationActive = function() { return curSync; };

  getCircularPlane = function() { return _transformCircular(curEv).plane; };
  getCircularRadius = function() { return curEv.radius; };
  getCircularSweep = function() { return curEv.sweep; };
  getCircularNormal = function() { return _transformCircular(curEv).normal; };
  getCircularStart = function() { return _fp(curEv.sx, curEv.sy, curEv.sz); };
  getCircularCenter = function() { return _fp(curEv.cx, curEv.cy, curEv.cz); };
  getCircularEnd = function() { return _fp(curEv.x, curEv.y, curEv.z); };
  isFullCircle = function() { return !!curEv.fullCircle; };
  isHelical = function() { return !!curEv.helical; };
  isSpiral = function() { return !!curEv.spiral; };

  isFirstCyclePoint = function() { return !!curEv.first; };
  isLastCyclePoint = function() { return !!curEv.last; };
  getNumberOfCyclePoints = function() { return curCyclePoints.length; };
  getCyclePoint = function(i) { return _fpVec(curCyclePoints[i]); };

  __getCurEv = function() { return curEv; };
  __fp = _fp;
  __setLocalPos = function(v) { curLocalPos = new Vector(v.x, v.y, v.z); };

  unit = data.unit;
  if (data.highFeedrate) {
    highFeedrate = data.highFeedrate;
  }

  var buildVendor = new Function(
    "writeln",
    "writeWords",
    "writeWords2",
    "var currentSection, tool, movement, radiusCompensation, spindleSpeed, cycle, cycleType, cycleExpanded, programName, programComment;\n" +
    "var onOpen, onComment, onPassThrough, onParameter, onSection, onSpindleSpeed, onOrientateSpindle, onMovement, onFeedMode, onRadiusCompensation, onDwell, onCommand, onRapid, onLinear, onCircular, onCycle, onCyclePoint, onCycleEnd, onSectionEnd, onClose;\n" +
    "var __didRepos = false;\n" +
    "function getCurrentSectionId() { return currentSection.getId(); }\n" +
    "function isFirstSection() { return currentSection.getId() === 0; }\n" +
    "function isLastSection() { return currentSection.getId() === (getNumberOfSections() - 1); }\n" +
    "function hasPreviousSection() { return currentSection.getId() > 0; }\n" +
    "function hasNextSection() { return (currentSection.getId() + 1) < getNumberOfSections(); }\n" +
    "function getPreviousSection() {\n" +
    "  if (!currentSection || currentSection.getId() <= 0) return currentSection;\n" +
    "  return getSection(currentSection.getId() - 1);\n" +
    "}\n" +
    "function getNextSection() {\n" +
    "  if (!currentSection || (currentSection.getId() + 1) >= getNumberOfSections()) return currentSection;\n" +
    "  return getSection(currentSection.getId() + 1);\n" +
    "}\n" +
    "function isNewWorkPlane(sec) {\n" +
    "  var section = (sec && typeof sec === 'object') ? sec : currentSection;\n" +
    "  if (!section) return false;\n" +
    "  var firstSection = section.getId() === 0;\n" +
    "  var newWorkPlane = firstSection;\n" +
    "  if (!firstSection) {\n" +
    "    var previousSection = getSection(section.getId() - 1);\n" +
    "    if (previousSection) {\n" +
    "      var isOptimized = section.isOptimizedForMachine() && previousSection.isOptimizedForMachine();\n" +
    "      newWorkPlane = newWorkPlane || !isSameDirection(previousSection.getGlobalFinalToolAxis(), section.getGlobalInitialToolAxis());\n" +
    "      newWorkPlane = newWorkPlane || (isOptimized && Vector.diff(previousSection.getFinalToolAxisABC(), section.getInitialToolAxisABC()).length > 1e-4);\n" +
    "      newWorkPlane = newWorkPlane || (typeof machineConfiguration !== 'undefined' && machineConfiguration && !machineConfiguration.isMultiAxisConfiguration() && section.isMultiAxis());\n" +
    "      newWorkPlane = newWorkPlane || (previousSection.isMultiAxis() !== section.isMultiAxis());\n" +
    "    }\n" +
    "  }\n" +
    "  return newWorkPlane;\n" +
    "}\n" +
    "function isNewWorkOffset(sec) {\n" +
    "  var section = (sec && typeof sec === 'object') ? sec : currentSection;\n" +
    "  if (!section) return false;\n" +
    "  var firstSection = section.getId() === 0;\n" +
    "  var newWorkOffset = firstSection;\n" +
    "  if (!firstSection) {\n" +
    "    var previousSection = getSection(section.getId() - 1);\n" +
    "    if (previousSection) {\n" +
    "      newWorkOffset = newWorkOffset || (previousSection.workOffset !== section.workOffset);\n" +
    "    }\n" +
    "  }\n" +
    "  return newWorkOffset;\n" +
    "}\n" +
    "function isToolChangeNeeded() {\n" +
    "  var start = (arguments.length > 0 && typeof arguments[0] === 'object') ? 1 : 0;\n" +
    "  var section = (arguments.length > 0 && typeof arguments[0] === 'object') ? arguments[0] : currentSection;\n" +
    "  if (!section) return false;\n" +
    "  var firstSection = section.getId() === 0;\n" +
    "  var currentTool = section.getTool();\n" +
    "  if (firstSection || section.getForceToolChange()) return true;\n" +
    "  var previousSection = getSection(section.getId() - 1);\n" +
    "  if (!previousSection) return true;\n" +
    "  var previousTool = previousSection.getTool();\n" +
    "  if (!previousTool) return true;\n" +
    "  if (arguments.length === start) {\n" +
    "    return currentTool.number !== previousTool.number;\n" +
    "  } else {\n" +
    "    for (var i = start; i < arguments.length; ++i) {\n" +
    "      var prop = arguments[i];\n" +
    "      if (previousTool[prop] !== currentTool[prop]) return true;\n" +
    "    }\n" +
    "  }\n" +
    "  return false;\n" +
    "}\n" +
    "function getNextTool() {\n" +
    "  var start = (arguments.length > 0 && typeof arguments[0] === 'object') ? 1 : 0;\n" +
    "  var section = (arguments.length > 0 && typeof arguments[0] === 'object') ? arguments[0] : currentSection;\n" +
    "  if (!section) return undefined;\n" +
    "  var currentSectionId = section.getId();\n" +
    "  var numSections = getNumberOfSections();\n" +
    "  var returnFirstTool = (arguments.length > start && typeof arguments[start] === 'boolean') ? arguments[start] : false;\n" +
    "  for (var i = currentSectionId + 1; i < numSections; ++i) {\n" +
    "    var nextSec = getSection(i);\n" +
    "    if (nextSec && nextSec.getTool().number !== section.getTool().number) {\n" +
    "      return nextSec.getTool();\n" +
    "    }\n" +
    "  }\n" +
    "  return returnFirstTool ? getSection(0).getTool() : undefined;\n" +
    "}\n" +
    "function isSpindleSpeedDifferent(sec) {\n" +
    "  var section = (sec && typeof sec === 'object') ? sec : currentSection;\n" +
    "  if (!section || section.getId() === 0) return true;\n" +
    "  var previousSection = getSection(section.getId() - 1);\n" +
    "  if (!previousSection) return true;\n" +
    "  if (previousSection.getTool().clockwise !== section.getTool().clockwise) return true;\n" +
    "  if (section.getTool().getSpindleMode() === SPINDLE_CONSTANT_SURFACE_SPEED) {\n" +
    "    return (previousSection.getTool().getSpindleMode() !== SPINDLE_CONSTANT_SURFACE_SPEED) ||\n" +
    "      Math.round(previousSection.getTool().surfaceSpeed - section.getTool().surfaceSpeed) !== 0;\n" +
    "  } else {\n" +
    "    return (previousSection.getTool().getSpindleMode() !== SPINDLE_CONSTANT_SPINDLE_SPEED) ||\n" +
    "      Math.round(previousSection.getTool().spindleRPM - section.getTool().spindleRPM) !== 0;\n" +
    "  }\n" +
    "}\n" +
    "function isProbeOperation(sec) {\n" +
    "  var section = (sec && typeof sec === 'object') ? sec : currentSection;\n" +
    "  if (!section) return false;\n" +
    "  var strat = section.getParameter('operation-strategy', '');\n" +
    "  return strat === 'probe' || strat === 'probe_geometry';\n" +
    "}\n" +
    "function isDrillingCycle(sec) {\n" +
    "  var section = (sec && typeof sec === 'object') ? sec : currentSection;\n" +
    "  if (!section) return false;\n" +
    "  return section.strategy === 'drill';\n" +
    "}\n" +
    "function hasParameter(name) { return (currentSection && currentSection.hasParameter(name)) || hasGlobalParameter(name); }\n" +
    "function getParameter(name, def) { return (currentSection && currentSection.hasParameter(name)) ? currentSection.getParameter(name, def) : getGlobalParameter(name, def); }\n" +
    "function getProperty(name, def) {\n" +
    "  if (typeof properties !== 'undefined' && properties && Object.prototype.hasOwnProperty.call(properties, name)) {\n" +
    "    var p = properties[name];\n" +
    "    if (p !== null && typeof p === 'object') {\n" +
    "      if ('current' in p && p.current !== undefined) return p.current;\n" +
    "      if ('value' in p) return p.value;\n" +
    "    }\n" +
    "    return p;\n" +
    "  }\n" +
    "  return def;\n" +
    "}\n" +
    "function setProperty(name, val) {\n" +
    "  if (typeof properties !== 'undefined' && properties && Object.prototype.hasOwnProperty.call(properties, name)) {\n" +
    "    if (properties[name] !== null && typeof properties[name] === 'object') {\n" +
    "      properties[name].value = val;\n" +
    "      properties[name].current = val;\n" +
    "    } else {\n" +
    "      properties[name] = val;\n" +
    "    }\n" +
    "  }\n" +
    "}\n" +
    "function __applyPostProp(name, val) {\n" +
    "  if (typeof properties !== 'undefined' && properties && Object.prototype.hasOwnProperty.call(properties, name)) {\n" +
    "    if (properties[name] !== null && typeof properties[name] === 'object') {\n" +
    "      properties[name].value = val;\n" +
    "      properties[name].current = val;\n" +
    "    } else {\n" +
    "      properties[name] = val;\n" +
    "    }\n" +
    "  }\n" +
    "  if (typeof setProperty === 'function') {\n" +
    "    try { setProperty(name, val); } catch (e) {}\n" +
    "  }\n" +
    "}\n" +
    vendorCode + "\n" +
    "if (typeof gMotionModal !== 'undefined') this.gMotionModal = gMotionModal;\n" +
    "if (typeof gPlaneModal !== 'undefined') this.gPlaneModal = gPlaneModal;\n" +
    "if (typeof gAbsIncModal !== 'undefined') this.gAbsIncModal = gAbsIncModal;\n" +
    "if (typeof gFeedModeModal !== 'undefined') this.gFeedModeModal = gFeedModeModal;\n" +
    "if (typeof feedOutput !== 'undefined') this.feedOutput = feedOutput;\n" +
    "if (typeof fourthAxisClamp !== 'undefined') this.fourthAxisClamp = fourthAxisClamp;\n" +
    "if (typeof fifthAxisClamp !== 'undefined') this.fifthAxisClamp = fifthAxisClamp;\n" +
    "if (typeof sixthAxisClamp !== 'undefined') this.sixthAxisClamp = sixthAxisClamp;\n" +
    "function linearize(tol) {\n" +
    "  var cEv = __getCurEv();\n" +
    "  if (cEv && cEv.linearizedMoves && cEv.linearizedMoves.length > 0) {\n" +
    "    for (var m = 0; m < cEv.linearizedMoves.length; ++m) {\n" +
    "      var mv = cEv.linearizedMoves[m];\n" +
    "      if (mv.movement !== undefined) movement = mv.movement;\n" +
    "      var p = __fp(mv.x, mv.y, mv.z);\n" +
    "      onLinear(p.x, p.y, p.z, mv.feed);\n" +
    "      __setLocalPos(mv);\n" +
    "    }\n" +
    "  } else if (cEv) {\n" +
    "    var p2 = __fp(cEv.x, cEv.y, cEv.z);\n" +
    "    onLinear(p2.x, p2.y, p2.z, cEv.feed);\n" +
    "    __setLocalPos(cEv);\n" +
    "  }\n" +
    "}\n" +
    "function expandCyclePoint(x, y, z) {\n" +
    "  cycleExpanded = true;\n" +
    "  var cEv = __getCurEv();\n" +
    "  var list = (cEv && __didRepos) ? cEv.afterReposMoves : (cEv ? cEv.expandedMoves : null);\n" +
    "  if (list) {\n" +
    "    for (var m = 0; m < list.length; ++m) {\n" +
    "      var mv = list[m];\n" +
    "      if (mv.movement !== undefined) movement = mv.movement;\n" +
    "      if (mv.type === 'onRapid') {\n" +
    "        var pr = __fp(mv.x, mv.y, mv.z);\n" +
    "        onRapid(pr.x, pr.y, pr.z);\n" +
    "        __setLocalPos(mv);\n" +
    "      } else if (mv.type === 'onLinear') {\n" +
    "        var pl = __fp(mv.x, mv.y, mv.z);\n" +
    "        onLinear(pl.x, pl.y, pl.z, mv.feed);\n" +
    "        __setLocalPos(mv);\n" +
    "      } else if (mv.type === 'onDwell') {\n" +
    "        if (typeof onDwell === 'function') onDwell(mv.sec);\n" +
    "      }\n" +
    "    }\n" +
    "  }\n" +
    "}\n" +
    "function repositionToCycleClearance(cycle, x, y, z) {\n" +
    "  __didRepos = true;\n" +
    "  var cEv = __getCurEv();\n" +
    "  if (cEv && cEv.reposMoves) {\n" +
    "    for (var m = 0; m < cEv.reposMoves.length; ++m) {\n" +
    "      var mv = cEv.reposMoves[m];\n" +
    "      if (mv.movement !== undefined) movement = mv.movement;\n" +
    "      if (mv.type === 'onRapid') {\n" +
    "        var pr = __fp(mv.x, mv.y, mv.z);\n" +
    "        onRapid(pr.x, pr.y, pr.z);\n" +
    "        __setLocalPos(mv);\n" +
    "      } else if (mv.type === 'onLinear') {\n" +
    "        var pl = __fp(mv.x, mv.y, mv.z);\n" +
    "        onLinear(pl.x, pl.y, pl.z, mv.feed);\n" +
    "        __setLocalPos(mv);\n" +
    "      }\n" +
    "    }\n" +
    "  }\n" +
    "}\n" +
    "return {\n" +
    "  setGlobals: function(pn, pc) { programName = pn; programComment = pc; },\n" +
    "  setState: function(st) {\n" +
    "    if ('currentSection' in st) currentSection = st.currentSection;\n" +
    "    if ('tool' in st) tool = st.tool;\n" +
    "    if ('movement' in st) movement = st.movement;\n" +
    "    if ('radiusCompensation' in st) radiusCompensation = st.radiusCompensation;\n" +
    "    if ('spindleSpeed' in st) spindleSpeed = st.spindleSpeed;\n" +
    "    if ('cycle' in st) cycle = st.cycle;\n" +
    "    if ('cycleType' in st) cycleType = st.cycleType;\n" +
    "    if ('cycleExpanded' in st) cycleExpanded = st.cycleExpanded;\n" +
    "  },\n" +
    "  setProperty: setProperty,\n" +
    "  applyPostProp: __applyPostProp,\n" +
    "  getRapidMode: function() {\n" +
    "    var hfm = (typeof highFeedMapping !== 'undefined') ? highFeedMapping : null;\n" +
    "    var ug = null;\n" +
    "    if (typeof getProperty === 'function') {\n" +
    "      try {\n" +
    "        var p = getProperty('useG0');\n" +
    "        if (typeof p === 'boolean') ug = p;\n" +
    "      } catch (e) {}\n" +
    "    }\n" +
    "    if (ug === null && typeof properties !== 'undefined' && properties && typeof properties.useG0 !== 'undefined') {\n" +
    "      try {\n" +
    "        var val = (typeof properties.useG0 === 'object' && properties.useG0 !== null) ? properties.useG0.value : properties.useG0;\n" +
    "        if (typeof val === 'boolean') ug = val;\n" +
    "      } catch (e) {}\n" +
    "    }\n" +
    "    return { highFeedMapping: hfm, useG0: ug };\n" +
    "  },\n" +
    "  formatComment: (typeof formatComment === 'function') ? formatComment : null,\n" +
    "  writeComment: (typeof writeComment === 'function') ? writeComment : null,\n" +
    "  onOpen: (typeof onOpen === 'function') ? onOpen : null,\n" +
    "  onComment: (typeof onComment === 'function') ? onComment : null,\n" +
    "  onPassThrough: (typeof onPassThrough === 'function') ? onPassThrough : null,\n" +
    "  onParameter: (typeof onParameter === 'function') ? onParameter : null,\n" +
    "  onSection: (typeof onSection === 'function') ? onSection : null,\n" +
    "  onSpindleSpeed: (typeof onSpindleSpeed === 'function') ? onSpindleSpeed : null,\n" +
    "  onOrientateSpindle: (typeof onOrientateSpindle === 'function') ? onOrientateSpindle : null,\n" +
    "  onMovement: (typeof onMovement === 'function') ? onMovement : null,\n" +
    "  onFeedMode: (typeof onFeedMode === 'function') ? onFeedMode : null,\n" +
    "  onRadiusCompensation: (typeof onRadiusCompensation === 'function') ? onRadiusCompensation : null,\n" +
    "  onDwell: (typeof onDwell === 'function') ? onDwell : null,\n" +
    "  onCommand: (typeof onCommand === 'function') ? onCommand : null,\n" +
    "  onRapid: (typeof onRapid === 'function') ? onRapid : null,\n" +
    "  onLinear: (typeof onLinear === 'function') ? onLinear : null,\n" +
    "  onCircular: (typeof onCircular === 'function') ? onCircular : null,\n" +
    "  onCycle: (typeof onCycle === 'function') ? onCycle : null,\n" +
    "  onCyclePoint: (typeof onCyclePoint === 'function') ? function(x,y,z){ __didRepos = false; onCyclePoint(x,y,z); } : function(x,y,z){ __didRepos = false; expandCyclePoint(x,y,z); },\n" +
    "  onCycleEnd: (typeof onCycleEnd === 'function') ? onCycleEnd : null,\n" +
    "  onSectionEnd: (typeof onSectionEnd === 'function') ? onSectionEnd : null,\n" +
    "  onClose: (typeof onClose === 'function') ? onClose : null\n" +
    "};"
  );

  var _origWriteln = writeln;
  var _origWriteWords = writeWords;
  var _origWriteWords2 = writeWords2;
  var _promoCommentSuffix = "";
  var _customWriteln = function(text) {
    if (_promoCommentSuffix && typeof text === "string" && text.trim().length > 0) {
      var sfx = _promoCommentSuffix;
      _promoCommentSuffix = "";
      _origWriteln(text + " " + sfx);
    } else {
      _origWriteln(text);
    }
  };
  var _customWriteWords = function() {
    var args = Array.prototype.slice.call(arguments);
    if (_promoCommentSuffix) {
      var sfx = _promoCommentSuffix;
      _promoCommentSuffix = "";
      args.push(sfx);
    }
    _origWriteWords.apply(null, args);
  };
  var _customWriteWords2 = function() {
    var args = Array.prototype.slice.call(arguments);
    if (_promoCommentSuffix) {
      var sfx = _promoCommentSuffix;
      _promoCommentSuffix = "";
      args.push(sfx);
    }
    _origWriteWords2.apply(null, args);
  };

  var vendor = buildVendor(_customWriteln, _customWriteWords, _customWriteWords2);
  vendor.setGlobals(data.programName, data.programComment);

  if (data.postProperties && typeof data.postProperties === "object" && vendor.applyPostProp) {
    for (var ppk in data.postProperties) {
      if (Object.prototype.hasOwnProperty.call(data.postProperties, ppk)) {
        try {
          vendor.applyPostProp(ppk, data.postProperties[ppk]);
        } catch (e) {}
      }
    }
  }

  for (var gk in data.globalParams) {
    if (vendor.onParameter) vendor.onParameter(gk, data.globalParams[gk]);
  }

  if (sections.length > 0) {
    try {
      currentSection = sections[0];
      tool = sections[0].getTool();
    } catch (e) {}
    vendor.setState({
      currentSection: sections[0],
      tool: sections[0].getTool(),
      spindleSpeed: data.sections[0].initSpindleSpeed,
      movement: data.sections[0].initMovement,
      radiusCompensation: data.sections[0].initRadiusComp
    });
  }

  if (typeof cancelTransformation === "function") {
    try {
      cancelTransformation();
    } catch (e) {}
  }
  if (vendor.onOpen) vendor.onOpen();

  function checkVendorDogLeg() {
    var hfm = null;
    var ug = null;
    if (vendor && typeof vendor.getRapidMode === "function") {
      try {
        var rm = vendor.getRapidMode();
        if (rm) {
          hfm = rm.highFeedMapping;
          ug = rm.useG0;
        }
      } catch (e) {}
    }
    if (hfm === null && typeof highFeedMapping !== "undefined") {
      hfm = highFeedMapping;
    }
    if (ug === null && typeof getProperty === "function") {
      try {
        var p = getProperty("useG0");
        if (typeof p === "boolean") ug = p;
      } catch (e) {}
    }
    var isDogLeg = false;
    if (hfm !== null && hfm !== 0) {
      isDogLeg = true;
    }
    if (ug === false) {
      isDogLeg = true;
    }
    return { isDogLeg: isDogLeg, highFeedMapping: hfm, useG0: ug };
  }

  function _logReplayerComment(msg) {
    try {
      if (vendor && typeof vendor.writeComment === "function") {
        vendor.writeComment(msg);
        return;
      }
    } catch (e) {}
    try {
      if (typeof writeComment === "function") {
        writeComment(msg);
        return;
      }
    } catch (e) {}
    try {
      if (typeof writeln === "function") {
        writeln("(" + msg + ")");
        return;
      }
    } catch (e) {}
  }

  var vendorDogLeg = checkVendorDogLeg();
  var dogLegCommentLogged = false;

  for (var ci = 0; ci < data.comments.length; ++ci) {
    if (vendor.onComment) vendor.onComment(data.comments[ci]);
  }

  function _matricesApproxEqual(a, b, tol) {
    if (!a || !b || !a.right || !a.up || !a.forward || !b.right || !b.up || !b.forward) {
      return false;
    }
    return (
      Vector.diff(a.right, b.right).length <= tol &&
      Vector.diff(a.up, b.up).length <= tol &&
      Vector.diff(a.forward, b.forward).length <= tol
    );
  }

  var restoredCount = 0;
  var inCycle = false;
  var frameMismatchSections = [];
  var multiAxisSections = [];

  for (var si = 0; si < sections.length; ++si) {
    var sec = sections[si];
    var rawSec = data.sections[si];
    var opName = (rawSec.params && rawSec.params["operation-comment"])
      ? String(rawSec.params["operation-comment"])
      : ("Section_" + (rawSec.id !== undefined ? rawSec.id : si));

    for (var pk in rawSec.params) {
      if (vendor.onParameter) vendor.onParameter(pk, rawSec.params[pk]);
    }

    try {
      currentSection = sec;
      tool = sec.getTool();
    } catch (e) {}

    vendor.setState({
      currentSection: sec,
      tool: sec.getTool(),
      spindleSpeed: rawSec.initSpindleSpeed,
      movement: rawSec.initMovement,
      radiusCompensation: rawSec.initRadiusComp,
      cycle: undefined,
      cycleType: "",
      cycleExpanded: false
    });
    // R5: First section starts at its initialPosition; subsequent sections inherit
    // the previous section's local end position (mapped by current frame in getCurrentPosition).
    if (si === 0) {
      curLocalPos = sec.getInitialPosition();
    }
    inCycle = false;

    if (vendor.onSection) vendor.onSection();

    // R1: Verify vendor post's active kernel frame (R, T) matches verifier's (workPlane, workOrigin).
    var sectionFrameConsistent = true;
    try {
      var curR = (typeof getRotation === "function") ? getRotation() : new Matrix();
      var curT = (typeof getTranslation === "function") ? getTranslation() : new Vector(0, 0, 0);
      var expW = sec.getWorkPlane();
      var expO = sec.getWorkOrigin();
      var rOk = _matricesApproxEqual(curR, expW, 1e-4);
      var tOk = Vector.diff(curT, expO).length <= 1e-4;
      if (!rOk || !tOk) {
        sectionFrameConsistent = false;
        frameMismatchSections.push({
          secIdx: si,
          id: rawSec.id !== undefined ? rawSec.id : si,
          op: opName,
          rOk: rOk,
          tOk: tOk
        });
      }
    } catch (e) {
      sectionFrameConsistent = false;
      frameMismatchSections.push({
        secIdx: si,
        id: rawSec.id !== undefined ? rawSec.id : si,
        op: opName,
        error: String(e)
      });
    }

    // R8: Fail closed on simultaneous multi-axis sections.
    var isMultiAxisSec = !!(rawSec.isMultiAxis || rawSec.has5DMoves);
    if (isMultiAxisSec) {
      multiAxisSections.push({
        secIdx: si,
        id: rawSec.id !== undefined ? rawSec.id : si,
        op: opName
      });
    }

    for (var ei = 0; ei < rawSec.events.length; ++ei) {
      var ev = rawSec.events[ei];
      curEv = ev;
      nextEvIsMotion = false;
      for (var nei = ei + 1; nei < rawSec.events.length; ++nei) {
        var nt = rawSec.events[nei].type;
        if (nt === "onRapid" || nt === "onLinear" || nt === "onCircular" || nt === "onCyclePoint") {
          nextEvIsMotion = true;
          break;
        }
      }

      switch (ev.type) {
        case "onComment":
          if (vendor.onComment) vendor.onComment(ev.c);
          break;
        case "onPassThrough":
          if (vendor.onPassThrough) vendor.onPassThrough(ev.text);
          break;
        case "onParameter":
          if (vendor.onParameter) vendor.onParameter(ev.name, ev.value);
          break;
        case "onSpindleSpeed":
          vendor.setState({spindleSpeed: ev.s});
          if (vendor.onSpindleSpeed) vendor.onSpindleSpeed(ev.s);
          break;
        case "onOrientateSpindle":
          if (vendor.onOrientateSpindle) vendor.onOrientateSpindle(ev.angle);
          break;
        case "onMovement":
          vendor.setState({movement: ev.m});
          if (vendor.onMovement) vendor.onMovement(ev.m);
          break;
        case "onFeedMode":
          if (vendor.onFeedMode) vendor.onFeedMode(ev.fm);
          break;
        case "onRadiusCompensation":
          vendor.setState({radiusCompensation: ev.rc});
          if (vendor.onRadiusCompensation) vendor.onRadiusCompensation();
          break;
        case "onDwell":
          if (vendor.onDwell) vendor.onDwell(ev.sec);
          break;
        case "onCommand":
          if (vendor.onCommand) vendor.onCommand(ev.cmd);
          break;
        case "onRapid": {
          curLocalPos = new Vector(ev.sx, ev.sy, ev.sz);
          var rEnd = _fp(ev.x, ev.y, ev.z);
          vendor.setState({movement: ev.movement, radiusCompensation: ev.rc, spindleSpeed: ev.rpm});
          if (vendor.onRapid) vendor.onRapid(rEnd.x, rEnd.y, rEnd.z);
          curLocalPos = new Vector(ev.x, ev.y, ev.z);
          break;
        }
        case "onLinear": {
          curLocalPos = new Vector(ev.sx, ev.sy, ev.sz);
          var lEnd = _fp(ev.x, ev.y, ev.z);
          curSync = !!ev.sync;
          vendor.setState({movement: ev.movement, radiusCompensation: ev.rc, spindleSpeed: ev.rpm});
          var shouldPromote = !!ev.promote;
          if (shouldPromote && (!sectionFrameConsistent || isMultiAxisSec)) {
            shouldPromote = false;
          }
          if (shouldPromote && vendorDogLeg.isDogLeg) {
            shouldPromote = false;
            if (!dogLegCommentLogged) {
              dogLegCommentLogged = true;
              var reason = "Rapid promotion disabled: vendor post indicates dog-leg rapids (highFeedMapping=" +
                vendorDogLeg.highFeedMapping + ", useG0=" + vendorDogLeg.useG0 + ")";
              _logReplayerComment(reason);
            }
          }
          if (shouldPromote && !inCycle && !(ev.rc) && !curSync) {
            restoredCount += 1;
            _promoCommentSuffix = (typeof vendor.formatComment === "function")
              ? vendor.formatComment("Rapid restored")
              : "(Rapid restored)";
            try {
              if (vendor.onRapid) vendor.onRapid(lEnd.x, lEnd.y, lEnd.z);
            } finally {
              _promoCommentSuffix = "";
            }
          } else {
            if (vendor.onLinear) vendor.onLinear(lEnd.x, lEnd.y, lEnd.z, ev.feed);
          }
          curLocalPos = new Vector(ev.x, ev.y, ev.z);
          curSync = false;
          break;
        }
        case "onCircular": {
          curLocalPos = new Vector(ev.sx, ev.sy, ev.sz);
          var cCenter = _fp(ev.cx, ev.cy, ev.cz);
          var cEnd = _fp(ev.x, ev.y, ev.z);
          var cInfo = _transformCircular(ev);
          vendor.setState({movement: ev.movement, radiusCompensation: ev.rc, spindleSpeed: ev.rpm});
          if (vendor.onCircular) {
            vendor.onCircular(cInfo.cw, cCenter.x, cCenter.y, cCenter.z, cEnd.x, cEnd.y, cEnd.z, ev.feed);
          }
          curLocalPos = new Vector(ev.x, ev.y, ev.z);
          break;
        }
        case "onCycle":
          inCycle = true;
          curCyclePoints = ev.points || [];
          vendor.setState({cycleType: ev.cycleType, cycle: hydrateCycle(ev.cycle), cycleExpanded: false});
          if (vendor.onCycle) vendor.onCycle();
          break;
        case "onCyclePoint": {
          curLocalPos = new Vector(ev.sx, ev.sy, ev.sz);
          var cpEnd = _fp(ev.x, ev.y, ev.z);
          if (vendor.onCyclePoint) vendor.onCyclePoint(cpEnd.x, cpEnd.y, cpEnd.z);
          curLocalPos = new Vector(ev.x, ev.y, ev.z);
          break;
        }
        case "onCycleEnd":
          if (vendor.onCycleEnd) vendor.onCycleEnd();
          vendor.setState({cycle: undefined, cycleType: "", cycleExpanded: false});
          curCyclePoints = [];
          inCycle = false;
          break;
      }
    }

    if (rawSec.endPos) {
      curLocalPos = toVec(rawSec.endPos);
    } else if (rawSec.finalPosition) {
      curLocalPos = toVec(rawSec.finalPosition);
    }
    vendor.setState({spindleSpeed: rawSec.endSpindleSpeed});
    if (vendor.onSectionEnd) vendor.onSectionEnd();

    if (rawSec.postComments) {
      for (var pci = 0; pci < rawSec.postComments.length; ++pci) {
        if (vendor.onComment) vendor.onComment(rawSec.postComments[pci]);
      }
    }
  }

  if (vendor.onClose) vendor.onClose();

  if (restoredCount > 0 && vendor.writeComment) {
    try {
      vendor.writeComment("UNIVERSAL WRAPPER: RESTORED " + restoredCount + " G0 RAPID MOVES");
    } catch (e) {}
  }

  var replayDiag = {
    frameMismatches: frameMismatchSections,
    multiAxisSections: multiAxisSections
  };
  var diagPath = "";
  try {
    diagPath = getEnvironmentVariable("REPLAY_DIAG_JSON");
  } catch (e) {}
  var diagWritten = false;
  if (diagPath) {
    try {
      var tf = new TextFile(diagPath, true, "utf-8");
      tf.writeln(JSON.stringify(replayDiag));
      tf.close();
      diagWritten = true;
    } catch (e) {}
  }
  if (!diagWritten && (frameMismatchSections.length > 0 || multiAxisSections.length > 0 || diagPath)) {
    try {
      _origWriteln("(POSTCLEAR_REPLAY_DIAG:" + JSON.stringify(replayDiag) + ")");
    } catch (e) {}
  }
}
