/**
 * SMGLib Interactive Simulator
 * ─────────────────────────────
 * Implements Social-ORCA (RVO2) and a simplified IMPC-DR in pure JS.
 * Renders on an HTML5 Canvas with click-to-place interaction.
 */

/* ══════════════════════════════════════════════════════════════
   0.  VECTOR HELPERS
   ══════════════════════════════════════════════════════════════ */
function v2(x, y) { return { x: x, y: y }; }
function vAdd(a, b) { return { x: a.x + b.x, y: a.y + b.y }; }
function vSub(a, b) { return { x: a.x - b.x, y: a.y - b.y }; }
function vScale(a, s) { return { x: a.x * s, y: a.y * s }; }
function vDot(a, b) { return a.x * b.x + a.y * b.y; }
function vDet(a, b) { return a.x * b.y - a.y * b.x; }
function vLen(a) { return Math.sqrt(a.x * a.x + a.y * a.y); }
function vLenSq(a) { return a.x * a.x + a.y * a.y; }
function vNorm(a) { var l = vLen(a); return l > 1e-8 ? vScale(a, 1 / l) : v2(0, 0); }
function vPerp(a) { return { x: -a.y, y: a.x }; } // 90° CCW
function vDist(a, b) { return vLen(vSub(a, b)); }
function vDistSq(a, b) { return vLenSq(vSub(a, b)); }

/* ══════════════════════════════════════════════════════════════
   1.  CONSTANTS
   ══════════════════════════════════════════════════════════════ */
var SIM = {
  DT: 0.1,                // simulation timestep (seconds)
  AGENT_RADIUS: 0.8,      // visual/collision radius (world units)
  AGENT_REPS: 0.15,       // radius epsilon for ORCA
  MAX_SPEED: 4.0,         // max agent speed (world units / s)
  TIME_HORIZON: 5.4,      // ORCA time horizon for agents
  OBS_TIME_HORIZON: 10.0, // ORCA time horizon for obstacles
  GOAL_TOL: 0.5,          // goal reached tolerance
  EPS: 1e-5,
  MAX_STEPS: 3000,        // safety cap
};

/* ══════════════════════════════════════════════════════════════
   2.  ORCA LINEAR PROGRAM  (RVO2 port)
   ══════════════════════════════════════════════════════════════ */

/**
 * Solve 1D LP along line `lines[curr]` subject to half-planes lines[0..curr-1].
 * Returns { ok, result }.
 */
function linearProgram1(lines, curr, radius, optVelocity, dirOpt) {
  var line = lines[curr];
  var dotProduct = vDot(line.point, line.dir);
  var discriminant = dotProduct * dotProduct + radius * radius - vLenSq(line.point);
  if (discriminant < 0) return { ok: false, result: v2(0, 0) };

  var sqrtDisc = Math.sqrt(discriminant);
  var tLeft = -dotProduct - sqrtDisc;
  var tRight = -dotProduct + sqrtDisc;

  for (var i = 0; i < curr; i++) {
    var denom = vDet(line.dir, lines[i].dir);
    var numer = vDet(lines[i].dir, vSub(line.point, lines[i].point));

    if (Math.abs(denom) <= SIM.EPS) {
      // Lines are (almost) parallel
      if (numer < 0) return { ok: false, result: v2(0, 0) };
      continue;
    }
    var t = numer / denom;
    if (denom >= 0) { tRight = Math.min(tRight, t); }
    else            { tLeft  = Math.max(tLeft, t);  }
    if (tLeft > tRight) return { ok: false, result: v2(0, 0) };
  }

  var t;
  if (dirOpt) {
    // Optimise direction
    t = (vDot(optVelocity, line.dir) > 0) ? tRight : tLeft;
  } else {
    t = vDot(line.dir, vSub(optVelocity, line.point));
    t = Math.max(tLeft, Math.min(tRight, t));
  }
  return { ok: true, result: vAdd(line.point, vScale(line.dir, t)) };
}

/**
 * 2D LP: find velocity closest to optVelocity satisfying all half-planes.
 * Returns { failLine, result }.  failLine === lines.length means success.
 */
function linearProgram2(lines, radius, optVelocity, dirOpt) {
  var result;
  if (dirOpt) {
    result = vScale(optVelocity, radius);
  } else if (vLenSq(optVelocity) > radius * radius) {
    result = vScale(vNorm(optVelocity), radius);
  } else {
    result = { x: optVelocity.x, y: optVelocity.y };
  }

  for (var i = 0; i < lines.length; i++) {
    if (vDet(lines[i].dir, vSub(lines[i].point, result)) > 0) {
      // result violates constraint i
      var lp1 = linearProgram1(lines, i, radius, optVelocity, dirOpt);
      if (!lp1.ok) {
        return { failLine: i, result: result };
      }
      result = lp1.result;
    }
  }
  return { failLine: lines.length, result: result };
}

/**
 * 3D fallback: when LP2 fails, iteratively project the result onto
 * each violated constraint. Simplified from RVO2's linearProgram3.
 */
function linearProgram3(lines, numObstLines, beginLine, radius, resultIn) {
  var result = { x: resultIn.x, y: resultIn.y };

  for (var i = beginLine; i < lines.length; i++) {
    if (vDet(lines[i].dir, vSub(lines[i].point, result)) > 0) {
      // Constraint i not satisfied — project result onto this line
      var dp = vDot(lines[i].dir, vSub(result, lines[i].point));
      var projected = vAdd(lines[i].point, vScale(lines[i].dir, dp));
      // Clamp to speed limit
      if (vLenSq(projected) > radius * radius) {
        projected = vScale(vNorm(projected), radius);
      }
      result = projected;
    }
  }
  return result;
}

/* ── Build ORCA half-plane for a pair of agents ─────────────── */
function computeAgentORCALine(agent, other) {
  var relPos = vSub(other.pos, agent.pos);
  var relVel = vSub(agent.vel, other.vel);
  var distSq = vLenSq(relPos);
  var combinedRadius = agent.radius + other.radius + 2 * SIM.AGENT_REPS;
  var combinedRadiusSq = combinedRadius * combinedRadius;

  var line = { point: v2(0, 0), dir: v2(0, 0) };
  var u;

  if (distSq > combinedRadiusSq) {
    // No collision
    var w = vSub(relVel, vScale(relPos, 1.0 / SIM.TIME_HORIZON));
    var wLenSq = vLenSq(w);
    var dotProduct1 = vDot(w, relPos);

    if (dotProduct1 < 0 && dotProduct1 * dotProduct1 > combinedRadiusSq * wLenSq) {
      // Project on cut-off circle
      var wLen = Math.sqrt(wLenSq);
      var unitW = vScale(w, 1.0 / wLen);
      line.dir = v2(unitW.y, -unitW.x);
      u = vScale(unitW, combinedRadius / SIM.TIME_HORIZON - wLen);
    } else {
      // Project on legs
      var leg = Math.sqrt(distSq - combinedRadiusSq);
      if (vDet(relPos, w) > 0) {
        // Left leg
        line.dir = vScale(
          v2(relPos.x * leg - relPos.y * combinedRadius,
             relPos.x * combinedRadius + relPos.y * leg),
          1.0 / distSq);
      } else {
        // Right leg
        line.dir = vScale(
          v2(relPos.x * leg + relPos.y * combinedRadius,
             -relPos.x * combinedRadius + relPos.y * leg),
          -1.0 / distSq);
      }
      var dotProduct2 = vDot(relVel, line.dir);
      u = vSub(vScale(line.dir, dotProduct2), relVel);
    }
  } else {
    // Collision — project on cut-off circle at time step
    var invTimeStep = 1.0 / SIM.DT;
    var w = vSub(relVel, vScale(relPos, invTimeStep));
    var wLen = vLen(w);
    if (wLen < SIM.EPS) return null;
    var unitW = vScale(w, 1.0 / wLen);
    line.dir = v2(unitW.y, -unitW.x);
    u = vScale(unitW, combinedRadius * invTimeStep - wLen);
  }

  line.point = vAdd(agent.vel, vScale(u, 0.5));
  // Store normal for LP3 fallback
  line.normal = v2(-line.dir.y, line.dir.x);
  return line;
}

/* ── Build ORCA half-planes for rectangular obstacles ──────── */
function computeObstacleORCALines(agent, obstacles) {
  var lines = [];
  var expandedRadius = agent.radius + SIM.AGENT_REPS;

  for (var oi = 0; oi < obstacles.length; oi++) {
    var obs = obstacles[oi];

    // Find closest point on rectangle to agent
    var cx = Math.max(obs.x, Math.min(obs.x + obs.w, agent.pos.x));
    var cy = Math.max(obs.y, Math.min(obs.y + obs.h, agent.pos.y));
    var diff = vSub(agent.pos, v2(cx, cy));
    var dist = vLen(diff);

    // Only generate constraint if agent is close to obstacle
    var safetyMargin = expandedRadius + 0.5;
    if (dist > safetyMargin) continue;

    var normal;
    if (dist < SIM.EPS) {
      // Agent center is inside obstacle — push toward nearest edge
      var dLeft   = agent.pos.x - obs.x;
      var dRight  = obs.x + obs.w - agent.pos.x;
      var dTop    = agent.pos.y - obs.y;
      var dBottom = obs.y + obs.h - agent.pos.y;
      var minD = Math.min(dLeft, dRight, dTop, dBottom);
      if      (minD === dLeft)   normal = v2(-1, 0);
      else if (minD === dRight)  normal = v2(1, 0);
      else if (minD === dTop)    normal = v2(0, -1);
      else                       normal = v2(0, 1);
    } else {
      normal = vScale(diff, 1.0 / dist);
    }

    var penetration = expandedRadius - dist;
    var pushMag;
    if (penetration > 0) {
      // Overlapping — strong push
      pushMag = penetration / SIM.DT + 0.5;
    } else {
      // Close but not overlapping — gentle repulsion
      var gap = -penetration; // positive distance from surface
      pushMag = Math.max(0, (0.5 - gap) * 2.0);
    }

    if (pushMag > SIM.EPS) {
      lines.push({
        point: vScale(normal, pushMag),
        dir: v2(-normal.y, normal.x),
        normal: normal,
      });
    }
  }
  return lines;
}

/* ══════════════════════════════════════════════════════════════
   3.  ORCA ALGORITHM — compute new velocity for one agent
   ══════════════════════════════════════════════════════════════ */
function orcaComputeVelocity(agent, agents, obstacles) {
  var orcaLines = [];

  // Obstacle lines first
  var obsLines = computeObstacleORCALines(agent, obstacles);
  for (var i = 0; i < obsLines.length; i++) orcaLines.push(obsLines[i]);
  var numObstLines = orcaLines.length;

  // Agent-agent lines
  for (var i = 0; i < agents.length; i++) {
    if (agents[i] === agent || agents[i].reached) continue;
    var line = computeAgentORCALine(agent, agents[i]);
    if (line) orcaLines.push(line);
  }

  // Preferred velocity toward goal with slight perpendicular bias to break symmetry
  var toGoal = vSub(agent.goal, agent.pos);
  var distToGoal = vLen(toGoal);
  var prefVel;
  if (distToGoal < SIM.GOAL_TOL) {
    prefVel = v2(0, 0);
  } else {
    prefVel = vScale(toGoal, Math.min(SIM.MAX_SPEED, distToGoal) / distToGoal);
    // Symmetry breaker: small perpendicular nudge based on agent ID
    var perpNudge = vPerp(vNorm(toGoal));
    var nudgeSign = (agent.id % 2 === 0) ? 1 : -1;
    prefVel = vAdd(prefVel, vScale(perpNudge, nudgeSign * SIM.MAX_SPEED * 0.08));
  }

  var lp = linearProgram2(orcaLines, SIM.MAX_SPEED, prefVel, false);
  if (lp.failLine < orcaLines.length) {
    lp.result = linearProgram3(orcaLines, numObstLines, lp.failLine, SIM.MAX_SPEED, lp.result);
  }

  return lp.result;
}

/* ══════════════════════════════════════════════════════════════
   4.  SIMPLIFIED IMPC-DR
   ══════════════════════════════════════════════════════════════
   Inspired by Buffered-Voronoi + MPC: at each step compute preferred
   velocity, then project it away from other agents and obstacles using
   a simple iterative constraint projection.
   ══════════════════════════════════════════════════════════════ */
function impcdrComputeVelocity(agent, agents, obstacles) {
  // Preferred velocity toward goal
  var toGoal = vSub(agent.goal, agent.pos);
  var distToGoal = vLen(toGoal);
  var prefVel;
  if (distToGoal < SIM.GOAL_TOL) {
    return v2(0, 0);
  }
  var goalDir = vNorm(toGoal);
  prefVel = vScale(goalDir, Math.min(SIM.MAX_SPEED, distToGoal));

  // Add symmetry-breaking perpendicular bias
  var perpDir = vPerp(goalDir);
  var nudgeSign = (agent.id % 2 === 0) ? 1 : -1;
  prefVel = vAdd(prefVel, vScale(perpDir, nudgeSign * SIM.MAX_SPEED * 0.05));

  var vel = { x: prefVel.x, y: prefVel.y };

  // Iterative projection (multiple passes for convergence)
  for (var iter = 0; iter < 6; iter++) {
    // Buffered Voronoi cell constraints from other agents
    for (var i = 0; i < agents.length; i++) {
      if (agents[i] === agent || agents[i].reached) continue;
      var other = agents[i];
      var relPos = vSub(other.pos, agent.pos);
      var dist = vLen(relPos);
      if (dist < SIM.EPS) continue;

      var normal = vScale(relPos, 1.0 / dist);
      var safetyBuffer = (agent.radius + other.radius) * 2.0;

      // Buffered Voronoi boundary: agent must stay at least safetyBuffer/2
      // from the midpoint, on its own side
      var boundary = (dist - safetyBuffer) * 0.5;
      if (boundary < 0) boundary = 0;

      // How far the next position would be along normal direction from current pos
      var velAlongNormal = vDot(vel, normal);
      var nextDist = velAlongNormal * SIM.DT;

      // Max allowed movement toward other agent
      var maxMove = boundary;
      if (nextDist > maxMove) {
        // Need to reduce velocity component toward other agent
        var excess = nextDist - maxMove;
        vel = vSub(vel, vScale(normal, excess / SIM.DT));

        // Deflect laterally to maintain progress
        var lateralDir = vPerp(normal);
        var goalComponent = vDot(prefVel, lateralDir);
        if (Math.abs(goalComponent) < 0.1) {
          // Goal is nearly aligned with obstacle — add lateral deflection
          goalComponent = nudgeSign * SIM.MAX_SPEED * 0.5;
        }
        vel = vAdd(vel, vScale(lateralDir, goalComponent * 0.3));
      }
    }

    // Obstacle avoidance
    for (var oi = 0; oi < obstacles.length; oi++) {
      var obs = obstacles[oi];
      var nextPos = vAdd(agent.pos, vScale(vel, SIM.DT));

      var cx = Math.max(obs.x, Math.min(obs.x + obs.w, nextPos.x));
      var cy = Math.max(obs.y, Math.min(obs.y + obs.h, nextPos.y));
      var diff = vSub(nextPos, v2(cx, cy));
      var d = vLen(diff);
      var minDist = agent.radius + 0.3;

      if (d < minDist) {
        var pushDir = d > SIM.EPS ? vScale(diff, 1.0 / d) : v2(1, 0);
        var correction = (minDist - d) / SIM.DT;
        vel = vAdd(vel, vScale(pushDir, correction * 0.7));
      }
    }

    // Clamp to max speed
    if (vLen(vel) > SIM.MAX_SPEED) {
      vel = vScale(vNorm(vel), SIM.MAX_SPEED);
    }
  }

  // Deadlock detection: very slow but far from goal
  if (vLen(vel) < 0.3 && distToGoal > SIM.GOAL_TOL * 3) {
    var escapeDir = vPerp(goalDir);
    vel = vAdd(vel, vScale(escapeDir, nudgeSign * SIM.MAX_SPEED * 0.6));
    if (vLen(vel) > SIM.MAX_SPEED) vel = vScale(vNorm(vel), SIM.MAX_SPEED);
  }

  return vel;
}

/* ══════════════════════════════════════════════════════════════
   5.  COLLISION CHECKER  (for metrics — post-hoc)
   ══════════════════════════════════════════════════════════════ */
function checkCollisions(agents) {
  var count = 0;
  for (var i = 0; i < agents.length; i++) {
    for (var j = i + 1; j < agents.length; j++) {
      if (vDist(agents[i].pos, agents[j].pos) < agents[i].radius + agents[j].radius) {
        count++;
      }
    }
  }
  return count;
}

/* ══════════════════════════════════════════════════════════════
   6.  SIMULATOR STATE MACHINE
   ══════════════════════════════════════════════════════════════ */
var sim = {
  canvas: null,
  ctx: null,
  scenario: null,       // current SCENARIOS[key]
  scenarioKey: 'doorway',
  algorithm: 'orca',    // 'orca' or 'impcdr'
  agentCount: 2,
  agents: [],
  running: false,
  finished: false,
  step: 0,
  animId: null,
  // Metrics accumulators
  metrics: { totalDeltaV: 0, totalSteps: 0, collisions: 0, agentMetrics: [] },
  // Interaction state
  placingAgent: -1,     // which agent index is being placed (-1 = none)
  placingWhat: '',      // 'start' or 'goal'
  // View transform
  scale: 1,
  offsetX: 0,
  offsetY: 0,
};

/* ── Coordinate transforms ─────────────────────────────────── */
function worldToCanvas(wx, wy) {
  return {
    x: (wx - sim.scenario.view.xMin) * sim.scale + sim.offsetX,
    y: (wy - sim.scenario.view.yMin) * sim.scale + sim.offsetY,
  };
}

function canvasToWorld(cx, cy) {
  return {
    x: (cx - sim.offsetX) / sim.scale + sim.scenario.view.xMin,
    y: (cy - sim.offsetY) / sim.scale + sim.scenario.view.yMin,
  };
}

function updateTransform() {
  if (!sim.canvas || !sim.scenario) return;
  var view = sim.scenario.view;
  var worldW = view.xMax - view.xMin;
  var worldH = view.yMax - view.yMin;
  var canvasW = sim.canvas.width;
  var canvasH = sim.canvas.height;
  var padding = 20;
  sim.scale = Math.min((canvasW - 2 * padding) / worldW, (canvasH - 2 * padding) / worldH);
  sim.offsetX = (canvasW - worldW * sim.scale) / 2;
  sim.offsetY = (canvasH - worldH * sim.scale) / 2;
}

/* ── Initialise agents from scenario preset ────────────────── */
function initAgents() {
  sim.agents = [];
  var preset = sim.scenario.presets[sim.agentCount];
  if (!preset) preset = sim.scenario.presets[2];

  for (var i = 0; i < preset.length; i++) {
    sim.agents.push({
      id: i,
      pos: { x: preset[i].start.x, y: preset[i].start.y },
      vel: v2(0, 0),
      goal: { x: preset[i].goal.x, y: preset[i].goal.y },
      startPos: { x: preset[i].start.x, y: preset[i].start.y },
      radius: SIM.AGENT_RADIUS,
      color: preset[i].color,
      reached: false,
      trail: [],
      idealDist: 0,
      actualDist: 0,
      prevVel: v2(0, 0),
      startTime: 0,
      endTime: -1,
    });
    sim.agents[i].idealDist = vDist(sim.agents[i].pos, sim.agents[i].goal);
  }
  sim.step = 0;
  sim.finished = false;
  sim.metrics = { totalDeltaV: 0, totalSteps: 0, collisions: 0, agentMetrics: [] };
}

/* ── One simulation step ───────────────────────────────────── */
function simulationStep() {
  var allDone = true;
  var obstacles = sim.scenario.obstacles;

  // Compute new velocities for all agents first
  var newVels = [];
  for (var i = 0; i < sim.agents.length; i++) {
    var a = sim.agents[i];
    if (a.reached) {
      newVels.push(v2(0, 0));
      continue;
    }
    allDone = false;
    var newVel;
    if (sim.algorithm === 'orca') {
      newVel = orcaComputeVelocity(a, sim.agents, obstacles);
    } else {
      newVel = impcdrComputeVelocity(a, sim.agents, obstacles);
    }
    newVels.push(newVel);
  }

  // Apply velocities simultaneously
  for (var i = 0; i < sim.agents.length; i++) {
    var a = sim.agents[i];
    if (a.reached) continue;

    var oldVel = { x: a.vel.x, y: a.vel.y };
    a.vel = newVels[i];

    // Track delta-V
    sim.metrics.totalDeltaV += vDist(a.vel, oldVel);
    sim.metrics.totalSteps++;
    a.prevVel = oldVel;

    // Integrate position
    var newPos = vAdd(a.pos, vScale(a.vel, SIM.DT));

    // Hard obstacle collision — clamp position outside obstacles
    for (var oi = 0; oi < obstacles.length; oi++) {
      var obs = obstacles[oi];
      newPos = resolveObstacleCollision(newPos, a.radius, obs);
    }

    a.pos = newPos;
    a.trail.push({ x: a.pos.x, y: a.pos.y });
    a.actualDist += vLen(a.vel) * SIM.DT;

    // Check goal
    if (vDist(a.pos, a.goal) < SIM.GOAL_TOL) {
      a.reached = true;
      a.vel = v2(0, 0);
      a.endTime = sim.step * SIM.DT;
    }
  }

  // Count collisions
  sim.metrics.collisions += checkCollisions(sim.agents);

  sim.step++;

  if (allDone || sim.step >= SIM.MAX_STEPS) {
    sim.finished = true;
    sim.running = false;
    // Mark unreached agents
    for (var i = 0; i < sim.agents.length; i++) {
      if (sim.agents[i].endTime < 0) sim.agents[i].endTime = sim.step * SIM.DT;
    }
    updateButtonStates();
  }
}

/* ── Hard obstacle position resolution ─────────────────────── */
function resolveObstacleCollision(pos, radius, obs) {
  // Check if pos+radius overlaps the rectangle
  var cx = Math.max(obs.x, Math.min(obs.x + obs.w, pos.x));
  var cy = Math.max(obs.y, Math.min(obs.y + obs.h, pos.y));
  var diff = vSub(pos, v2(cx, cy));
  var dist = vLen(diff);

  if (dist < radius) {
    if (dist < SIM.EPS) {
      // Inside obstacle — push out via nearest edge
      var pushes = [
        { d: pos.x - obs.x, dir: v2(-1, 0) },
        { d: obs.x + obs.w - pos.x, dir: v2(1, 0) },
        { d: pos.y - obs.y, dir: v2(0, -1) },
        { d: obs.y + obs.h - pos.y, dir: v2(0, 1) },
      ];
      pushes.sort(function(a, b) { return a.d - b.d; });
      return vAdd(pos, vScale(pushes[0].dir, radius + pushes[0].d));
    }
    var pushDir = vScale(diff, 1.0 / dist);
    return vAdd(v2(cx, cy), vScale(pushDir, radius));
  }
  return pos;
}

/* ══════════════════════════════════════════════════════════════
   7.  CANVAS RENDERING
   ══════════════════════════════════════════════════════════════ */
function render() {
  var ctx = sim.ctx;
  var W = sim.canvas.width;
  var H = sim.canvas.height;

  // Background
  ctx.fillStyle = '#1a1a2e';
  ctx.fillRect(0, 0, W, H);

  // Grid (subtle)
  ctx.strokeStyle = 'rgba(255,255,255,0.04)';
  ctx.lineWidth = 1;
  var view = sim.scenario.view;
  for (var gx = Math.ceil(view.xMin); gx <= view.xMax; gx += 2) {
    var p = worldToCanvas(gx, view.yMin);
    var p2 = worldToCanvas(gx, view.yMax);
    ctx.beginPath(); ctx.moveTo(p.x, p.y); ctx.lineTo(p2.x, p2.y); ctx.stroke();
  }
  for (var gy = Math.ceil(view.yMin); gy <= view.yMax; gy += 2) {
    var p = worldToCanvas(view.xMin, gy);
    var p2 = worldToCanvas(view.xMax, gy);
    ctx.beginPath(); ctx.moveTo(p.x, p.y); ctx.lineTo(p2.x, p2.y); ctx.stroke();
  }

  // Obstacles
  ctx.fillStyle = '#3d3d5c';
  ctx.strokeStyle = '#5a5a8a';
  ctx.lineWidth = 1;
  var obstacles = sim.scenario.obstacles;
  for (var i = 0; i < obstacles.length; i++) {
    var tl = worldToCanvas(obstacles[i].x, obstacles[i].y);
    var br = worldToCanvas(obstacles[i].x + obstacles[i].w, obstacles[i].y + obstacles[i].h);
    var rw = br.x - tl.x;
    var rh = br.y - tl.y;
    ctx.fillRect(tl.x, tl.y, rw, rh);
    ctx.strokeRect(tl.x, tl.y, rw, rh);
  }

  // Agent trails
  for (var i = 0; i < sim.agents.length; i++) {
    var a = sim.agents[i];
    if (a.trail.length < 2) continue;
    ctx.strokeStyle = a.color;
    ctx.lineWidth = 2;
    ctx.globalAlpha = 0.4;
    ctx.beginPath();
    var p0 = worldToCanvas(a.trail[0].x, a.trail[0].y);
    ctx.moveTo(p0.x, p0.y);
    for (var j = 1; j < a.trail.length; j++) {
      var p = worldToCanvas(a.trail[j].x, a.trail[j].y);
      ctx.lineTo(p.x, p.y);
    }
    ctx.stroke();
    ctx.globalAlpha = 1;
  }

  // Goals (X marks)
  for (var i = 0; i < sim.agents.length; i++) {
    var a = sim.agents[i];
    var gp = worldToCanvas(a.goal.x, a.goal.y);
    var sz = sim.scale * a.radius * 0.6;
    ctx.strokeStyle = a.color;
    ctx.lineWidth = 2.5;
    ctx.globalAlpha = 0.6;
    ctx.beginPath();
    ctx.moveTo(gp.x - sz, gp.y - sz); ctx.lineTo(gp.x + sz, gp.y + sz);
    ctx.moveTo(gp.x + sz, gp.y - sz); ctx.lineTo(gp.x - sz, gp.y + sz);
    ctx.stroke();
    ctx.globalAlpha = 1;
    // Label
    ctx.fillStyle = a.color;
    ctx.font = 'bold 10px sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('G' + (i + 1), gp.x, gp.y - sz - 4);
  }

  // Start positions (hollow circle)
  for (var i = 0; i < sim.agents.length; i++) {
    var a = sim.agents[i];
    var sp = worldToCanvas(a.startPos.x, a.startPos.y);
    var r = sim.scale * a.radius;
    ctx.strokeStyle = a.color;
    ctx.lineWidth = 1.5;
    ctx.globalAlpha = 0.3;
    ctx.setLineDash([4, 4]);
    ctx.beginPath();
    ctx.arc(sp.x, sp.y, r, 0, Math.PI * 2);
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.globalAlpha = 1;
  }

  // Agents (filled circles with direction arrow)
  for (var i = 0; i < sim.agents.length; i++) {
    var a = sim.agents[i];
    var cp = worldToCanvas(a.pos.x, a.pos.y);
    var r = sim.scale * a.radius;

    // Glow
    ctx.shadowColor = a.color;
    ctx.shadowBlur = a.reached ? 0 : 8;

    // Body
    ctx.fillStyle = a.color;
    ctx.globalAlpha = a.reached ? 0.5 : 0.9;
    ctx.beginPath();
    ctx.arc(cp.x, cp.y, r, 0, Math.PI * 2);
    ctx.fill();
    ctx.globalAlpha = 1;
    ctx.shadowBlur = 0;

    // Border
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(cp.x, cp.y, r, 0, Math.PI * 2);
    ctx.stroke();

    // Label
    ctx.fillStyle = '#fff';
    ctx.font = 'bold ' + Math.max(10, r * 0.9) + 'px sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText('' + (i + 1), cp.x, cp.y);

    // Velocity arrow
    if (vLen(a.vel) > 0.1 && !a.reached) {
      var arrowEnd = vAdd(a.pos, vScale(vNorm(a.vel), a.radius * 2.5));
      var ae = worldToCanvas(arrowEnd.x, arrowEnd.y);
      ctx.strokeStyle = '#fff';
      ctx.lineWidth = 2;
      ctx.globalAlpha = 0.7;
      ctx.beginPath();
      ctx.moveTo(cp.x, cp.y);
      ctx.lineTo(ae.x, ae.y);
      ctx.stroke();
      // Arrowhead
      var angle = Math.atan2(ae.y - cp.y, ae.x - cp.x);
      ctx.beginPath();
      ctx.moveTo(ae.x, ae.y);
      ctx.lineTo(ae.x - 6 * Math.cos(angle - 0.4), ae.y - 6 * Math.sin(angle - 0.4));
      ctx.lineTo(ae.x - 6 * Math.cos(angle + 0.4), ae.y - 6 * Math.sin(angle + 0.4));
      ctx.closePath();
      ctx.fillStyle = '#fff';
      ctx.fill();
      ctx.globalAlpha = 1;
    }
  }

  // Status text
  ctx.fillStyle = 'rgba(255,255,255,0.6)';
  ctx.font = '12px monospace';
  ctx.textAlign = 'left';
  ctx.textBaseline = 'top';
  ctx.fillText('t = ' + (sim.step * SIM.DT).toFixed(1) + 's  |  step ' + sim.step, 8, 8);

  if (sim.finished) {
    ctx.fillStyle = 'rgba(72,199,116,0.85)';
    ctx.font = 'bold 16px sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('Simulation Complete', W / 2, H - 24);
  }
}

/* ══════════════════════════════════════════════════════════════
   8.  ANIMATION LOOP
   ══════════════════════════════════════════════════════════════ */
function animLoop() {
  if (sim.running && !sim.finished) {
    simulationStep();
  }
  render();
  updateMetricsPanel();
  sim.animId = requestAnimationFrame(animLoop);
}

/* ══════════════════════════════════════════════════════════════
   9.  METRICS PANEL UPDATE
   ══════════════════════════════════════════════════════════════ */
function updateMetricsPanel() {
  var el = document.getElementById('metrics-panel');
  if (!el) return;

  var avgDV = sim.metrics.totalSteps > 0
    ? (sim.metrics.totalDeltaV / sim.metrics.totalSteps).toFixed(3)
    : '0.000';

  // Average delay: (actual_time - ideal_time) averaged over agents
  var totalDelay = 0;
  var reachedCount = 0;
  var totalDeviation = 0;
  for (var i = 0; i < sim.agents.length; i++) {
    var a = sim.agents[i];
    var idealTime = a.idealDist / SIM.MAX_SPEED;
    if (a.reached || sim.finished) {
      totalDelay += a.endTime - idealTime;
      reachedCount++;
    }
    // Path deviation: extra distance
    totalDeviation += Math.max(0, a.actualDist - a.idealDist);
  }
  var avgDelay = reachedCount > 0 ? (totalDelay / reachedCount).toFixed(2) : '-';
  var avgDeviation = sim.agents.length > 0
    ? (totalDeviation / sim.agents.length).toFixed(2)
    : '0.00';

  el.innerHTML =
    '<div class="metric-item"><span class="metric-label">Avg &Delta;V</span><span class="metric-value">' + avgDV + '</span></div>' +
    '<div class="metric-item"><span class="metric-label">Avg Delay</span><span class="metric-value">' + avgDelay + 's</span></div>' +
    '<div class="metric-item"><span class="metric-label">Path Dev.</span><span class="metric-value">' + avgDeviation + '</span></div>' +
    '<div class="metric-item"><span class="metric-label">Collisions</span><span class="metric-value">' + sim.metrics.collisions + '</span></div>' +
    '<div class="metric-item"><span class="metric-label">Time</span><span class="metric-value">' + (sim.step * SIM.DT).toFixed(1) + 's</span></div>';
}

/* ══════════════════════════════════════════════════════════════
   10.  UI CONTROLS
   ══════════════════════════════════════════════════════════════ */
function updateButtonStates() {
  var runBtn = document.getElementById('btn-run');
  var stepBtn = document.getElementById('btn-step');
  var resetBtn = document.getElementById('btn-reset');
  if (!runBtn) return;

  if (sim.running) {
    runBtn.textContent = 'Pause';
    runBtn.classList.remove('is-success');
    runBtn.classList.add('is-warning');
  } else {
    runBtn.textContent = sim.finished ? 'Done' : 'Run';
    runBtn.classList.remove('is-warning');
    runBtn.classList.add('is-success');
  }
  runBtn.disabled = sim.finished;
  stepBtn.disabled = sim.running || sim.finished;
}

function onRun() {
  if (sim.finished) return;
  sim.running = !sim.running;
  updateButtonStates();
}

function onStep() {
  if (sim.running || sim.finished) return;
  simulationStep();
}

function onReset() {
  sim.running = false;
  sim.finished = false;
  initAgents();
  updateButtonStates();
}

function onScenarioChange(key) {
  sim.scenarioKey = key;
  sim.scenario = SCENARIOS[key];
  updateTransform();
  onReset();
}

function onAlgorithmChange(alg) {
  sim.algorithm = alg;
  // Highlight active button
  document.querySelectorAll('.algo-btn').forEach(function(btn) {
    btn.classList.remove('is-info', 'is-selected');
    btn.classList.add('is-light');
  });
  var activeBtn = document.querySelector('.algo-btn[data-algo="' + alg + '"]');
  if (activeBtn) {
    activeBtn.classList.remove('is-light');
    activeBtn.classList.add('is-info', 'is-selected');
  }
  onReset();
}

function onAgentCountChange(n) {
  sim.agentCount = n;
  // Highlight active button
  document.querySelectorAll('.count-btn').forEach(function(btn) {
    btn.classList.remove('is-info', 'is-selected');
    btn.classList.add('is-light');
  });
  var activeBtn = document.querySelector('.count-btn[data-count="' + n + '"]');
  if (activeBtn) {
    activeBtn.classList.remove('is-light');
    activeBtn.classList.add('is-info', 'is-selected');
  }
  onReset();
}

/* ── Canvas click: place agent start/goal ──────────────────── */
function onCanvasClick(e) {
  if (sim.running) return;

  var rect = sim.canvas.getBoundingClientRect();
  var scaleX = sim.canvas.width / rect.width;
  var scaleY = sim.canvas.height / rect.height;
  var cx = (e.clientX - rect.left) * scaleX;
  var cy = (e.clientY - rect.top) * scaleY;
  var world = canvasToWorld(cx, cy);

  // Check if clicking near an existing agent or goal to drag
  var closestAgent = -1;
  var closestDist = Infinity;
  var closestIsGoal = false;

  for (var i = 0; i < sim.agents.length; i++) {
    var a = sim.agents[i];
    var d = vDist(world, a.pos);
    if (d < a.radius * 2 && d < closestDist) {
      closestDist = d;
      closestAgent = i;
      closestIsGoal = false;
    }
    var dg = vDist(world, a.goal);
    if (dg < a.radius * 2 && dg < closestDist) {
      closestDist = dg;
      closestAgent = i;
      closestIsGoal = true;
    }
  }

  if (closestAgent >= 0) {
    // Start drag
    sim.placingAgent = closestAgent;
    sim.placingWhat = closestIsGoal ? 'goal' : 'start';
  }
}

function onCanvasMouseMove(e) {
  if (sim.placingAgent < 0) return;

  var rect = sim.canvas.getBoundingClientRect();
  var scaleX = sim.canvas.width / rect.width;
  var scaleY = sim.canvas.height / rect.height;
  var cx = (e.clientX - rect.left) * scaleX;
  var cy = (e.clientY - rect.top) * scaleY;
  var world = canvasToWorld(cx, cy);

  var a = sim.agents[sim.placingAgent];
  if (sim.placingWhat === 'goal') {
    a.goal = world;
  } else {
    a.pos = world;
    a.startPos = { x: world.x, y: world.y };
    a.trail = [];
    a.idealDist = vDist(a.pos, a.goal);
  }
}

function onCanvasMouseUp() {
  if (sim.placingAgent >= 0) {
    var a = sim.agents[sim.placingAgent];
    a.idealDist = vDist(a.startPos, a.goal);
    sim.placingAgent = -1;
    sim.placingWhat = '';
  }
}

/* ══════════════════════════════════════════════════════════════
   11.  INITIALISATION
   ══════════════════════════════════════════════════════════════ */
function initSimulator() {
  sim.canvas = document.getElementById('sim-canvas');
  if (!sim.canvas) return;
  sim.ctx = sim.canvas.getContext('2d');

  // Set canvas resolution (2x for sharpness on most screens)
  var rect = sim.canvas.getBoundingClientRect();
  sim.canvas.width = rect.width * 2;
  sim.canvas.height = rect.height * 2;

  sim.scenario = SCENARIOS[sim.scenarioKey];
  updateTransform();
  initAgents();

  // Event listeners
  sim.canvas.addEventListener('mousedown', onCanvasClick);
  sim.canvas.addEventListener('mousemove', onCanvasMouseMove);
  sim.canvas.addEventListener('mouseup', onCanvasMouseUp);
  sim.canvas.addEventListener('mouseleave', onCanvasMouseUp);

  // Touch support
  sim.canvas.addEventListener('touchstart', function(e) {
    e.preventDefault();
    var touch = e.touches[0];
    onCanvasClick({ clientX: touch.clientX, clientY: touch.clientY });
  }, { passive: false });
  sim.canvas.addEventListener('touchmove', function(e) {
    e.preventDefault();
    var touch = e.touches[0];
    onCanvasMouseMove({ clientX: touch.clientX, clientY: touch.clientY });
  }, { passive: false });
  sim.canvas.addEventListener('touchend', function(e) {
    e.preventDefault();
    onCanvasMouseUp();
  }, { passive: false });

  // Buttons
  document.getElementById('btn-run').addEventListener('click', onRun);
  document.getElementById('btn-step').addEventListener('click', onStep);
  document.getElementById('btn-reset').addEventListener('click', onReset);

  // Scenario selector
  document.querySelectorAll('.scenario-btn').forEach(function(btn) {
    btn.addEventListener('click', function() {
      document.querySelectorAll('.scenario-btn').forEach(function(b) {
        b.classList.remove('is-info', 'is-selected');
        b.classList.add('is-light');
      });
      btn.classList.remove('is-light');
      btn.classList.add('is-info', 'is-selected');
      onScenarioChange(btn.dataset.scenario);
    });
  });

  // Algorithm buttons
  document.querySelectorAll('.algo-btn').forEach(function(btn) {
    btn.addEventListener('click', function() {
      onAlgorithmChange(btn.dataset.algo);
    });
  });

  // Agent count buttons
  document.querySelectorAll('.count-btn').forEach(function(btn) {
    btn.addEventListener('click', function() {
      onAgentCountChange(parseInt(btn.dataset.count));
    });
  });

  // Quick-start presets
  document.querySelectorAll('.preset-btn').forEach(function(btn) {
    btn.addEventListener('click', function() {
      var s = btn.dataset.scenario;
      var n = parseInt(btn.dataset.count);
      var a = btn.dataset.algo;

      // Update scenario
      document.querySelectorAll('.scenario-btn').forEach(function(b) {
        b.classList.remove('is-info', 'is-selected');
        b.classList.add('is-light');
      });
      var sBtn = document.querySelector('.scenario-btn[data-scenario="' + s + '"]');
      if (sBtn) { sBtn.classList.remove('is-light'); sBtn.classList.add('is-info', 'is-selected'); }

      sim.scenarioKey = s;
      sim.scenario = SCENARIOS[s];
      sim.agentCount = n;
      sim.algorithm = a;
      updateTransform();
      onReset();

      // Update UI highlights
      onAlgorithmChange(a);
      onAgentCountChange(n);

      // Auto-run
      sim.running = true;
      updateButtonStates();
    });
  });

  updateButtonStates();
  animLoop();
}

/* ── Handle window resize ──────────────────────────────────── */
function onResize() {
  if (!sim.canvas) return;
  var rect = sim.canvas.getBoundingClientRect();
  sim.canvas.width = rect.width * 2;
  sim.canvas.height = rect.height * 2;
  updateTransform();
}

window.addEventListener('resize', onResize);

/* ── Start when DOM is ready ───────────────────────────────── */
if (document.readyState === 'loading') {
  document.addEventListener('DOMContentLoaded', initSimulator);
} else {
  initSimulator();
}
