import { Sim, policies, vantageForFrontier, bfsDist } from './simlib.js';
// auto-load fusion data

import { ThreeScene } from './three_scene.js';
import * as THREE from 'three';
let scored, goodList;

;(async function loadFusionData(){
  [scored, goodList] = await Promise.all([
    fetch('scored.json').then(r=>r.json()),
    fetch('good_rewrites.json').then(r=>r.json())
  ]);
  fusion.best_ms = Math.min(...scored.map(r=>r.pred_ms));
  fusion.good    = goodList;
  buildBoth();   // only now initialize sim/scene
})();

const $ = (id) => document.getElementById(id);

// DOM (two panels)
const canvasA = $('threeCanvasA');
const povA    = $('robotViewA');
const canvasB = $('threeCanvasB');
const povB    = $('robotViewB');

// Controls
const runBtn   = $('runBtn');
const pauseBtn = $('pauseBtn');
const resetBtn = $('resetBtn');

const sizeEl = $('size');
// missing DOM refs

const FIXED = {
    nRobots: 1,
    // 10× slower so motion and scanning are readable
    robotSpeed: 0.1,
    infoRadius: 4,
    target: 0.95,
    obstacleProb: 0.12
};
    
const useFusionEl = $('useFusion');
const covVal = $('covVal');
const timeVal = $('timeVal');
const distVal = $('distVal');
const stepsVal = $('stepsVal');



// 3D scenes
const sceneA = new ThreeScene(canvasA, null);   // Baseline
const sceneB = new ThreeScene(canvasB, null);   // Fusion

// State
let simB = null;         // baseline
let simF = null;         // fusion
let running = false;
let lastTick = 0;
let accum = 0;
const STEP_HZ = 3;            // slow, readable sim rate
const STEP_DT = 1 / STEP_HZ;

let fusion = { best_ms: null, good: null };
let llmCache = new Map(); // (ctxSig, ids[]) -> new order


// always render so you can move the camera even when paused
function draw() {
  sceneA.tick(); sceneB.tick();
  sceneA.render(true); sceneB.render(true);
  requestAnimationFrame(draw);
}
requestAnimationFrame(draw);

function buildBoth() {
    const cfg = {
    size: parseInt(sizeEl.value, 10),
    obstacleProb: FIXED.obstacleProb,
    nRobots: FIXED.nRobots,
    robotSpeed: FIXED.robotSpeed,
    infoRadius: FIXED.infoRadius,
    target: FIXED.target,
    seed: 0,
    simpleRoom: true
    };
    simB = new Sim(cfg);           // Baseline (NF)
    simF = simB.cloneForAB(1);     // Fusion (different seed to avoid lockstep ties)
    // Build scenes using the same GT
    sceneA.buildObstaclesFromGrid(simB.gt, { simpleRoom: simB.cfg.simpleRoom });
    sceneB.buildObstaclesFromGrid(simF.gt, { simpleRoom: simF.cfg.simpleRoom });
    sceneA.setRobotGridPose(simB.robots[0][0], simB.robots[0][1], 0);
    sceneB.setRobotGridPose(simF.robots[0][0], simF.robots[0][1], 0);
    if (sceneA.updateCoverageFromSim) sceneA.updateCoverageFromSim(simB);
    if (sceneB.updateCoverageFromSim) sceneB.updateCoverageFromSim(simF);
    updateMetrics(0);
}
    
resetBtn.addEventListener('click', () => buildBoth());

function updateFusionExplain(ctx5, scanS, order) {
    const box = document.getElementById('fusionExplain');
    if (!box) return;
    const [size, clutter, heat, gas, noise] = ctx5;
    const estScanMs = Math.round((scanS || 0) * 1000);
    const choice = order && order[0] !== undefined ? order[0] : null;
    box.textContent =
    `Fusion: clutter=${clutter.toFixed(2)}, scan≈${estScanMs}ms` +
    (choice !== null ? `, selected composite #${choice}` : '');
}
    
runBtn.addEventListener('click', () => { running = true; lastTick = performance.now(); requestAnimationFrame(loop); });
pauseBtn.addEventListener('click', () => { running = false; });

function loop(ts) {
  if (!running) return;
  const dt = (ts - lastTick) / 1000;
  lastTick = ts;

  accum += dt;
  while (accum >= STEP_DT) {
    stepOnce();            // advance the sim at 1 Hz
    accum -= STEP_DT;
  }
  requestAnimationFrame(loop);
}


function ctxSig(size, clutter, heat, gas, noise) {
  return `${Math.round(size)},${clutter.toFixed(2)},${heat|0},${gas|0},${noise|0}`;
}
    
async function maybeLLMRerank(ctx5, topK) {
  const url = 'http://localhost:8000/api/rerank';
  if (!url || !fusion.good) return null;
  // Minimal candidate set: ids 0..topK-1 by predicted ms (we don’t have per-composite y_ms in browser; use ranks from scored.json if supplied).
  // Fallback: just 0..(K-1).
  const pool = Array.from({length: topK}, (_,i)=>i);
  const key = ctxSig(...ctx5) + '|' + pool.join(',');
  if (llmCache.has(key)) return llmCache.get(key);

  try{
    const payload = {
      ctx: { size: ctx5[0], clutter: ctx5[1], heat: !!ctx5[2], gas: !!ctx5[3], noise: !!ctx5[4] },
      candidates: pool.map(i => ({
        index: i,
        pred_ms: fusion.best_ms ?? 1000,
        // send compact signature if good.json present
        steps: fusion.good ? fusion.good[i]?.parsed_plan : null
      }))
    };
    const resp = await fetch(url, { method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify(payload) });
    if (!resp.ok) throw new Error(await resp.text());
    const order = await resp.json(); // [ids...]
    if (Array.isArray(order) && order.every(n=>Number.isInteger(n))) {
      llmCache.set(key, order);
      return order;
    }
  }catch(e){ console.warn('[LLM] rerank failed', e); }
  return null;
}

function stepOnce() {
  const pol = policies.nf;       // always Nearest Frontier
  const polB = pol, polF = pol;
  if (!simB || !simF) buildBoth();
  // baseline
  runOne(simB, polB, { fusionOn: false, useLLM: false, scene: sceneA });

  if (sceneA.updateCoverageFromSim) sceneA.updateCoverageFromSim(simB);
  // fusion
  runOne(simF,
    useFusionEl.checked ? pickFusionScanGuided : policies.nf,
    { fusionOn: useFusionEl.checked, useLLM: false, scene: sceneB });

  if (sceneB.updateCoverageFromSim) sceneB.updateCoverageFromSim(simF);
  updateMetrics(Math.max(simB.step, simF.step));
  if ((simB.coverage() >= simB.cfg.target && simF.coverage() >= simF.cfg.target) ||
      (simB.step > simB.cfg.maxSteps && simF.step > simF.cfg.maxSteps)) running = false;
}

function runOne(simObj, pol, { fusionOn, useLLM, scene }) {
    if (!simObj) { console.warn('runOne: simObj is null'); return; }
  if (typeof pol !== 'function') { console.warn('runOne: policy is not a function'); return; }
  const goals = pol(simObj);
  for (let i = 0; i < simObj.robots.length; i++) {
    const g = goals[i] || simObj.robots[i];
    const prev = [...simObj.robots[i]];
    // Build 5D ctx (approximation)
    const size_m2 = simObj.cfg.size;
    const clutter = simObj.clutter();
    const heat = 0.0, gas = 0.0, noise = 0.0;
    const ctx5 = [size_m2, clutter, heat, gas, noise];

    // Scan time
    let scanS = 1.0;
    if (fusionOn) {
      if (fusion.best_ms != null) {
        // Use model’s best as a *hint*, but keep within a fast, realistic band.
        // Make it mildly depend on clutter so it adapts but stays <= baseline.
        const hintS = (fusion.best_ms / 1000.0) * (0.6 + 0.4 * clutter);
        scanS = Math.min(0.80, Math.max(0.15, hintS));
      } else {
        // Fallback heuristic: faster than baseline on average.
        scanS = 0.25 + 0.50 * clutter;  // ∈ [0.25, 0.75]
      }
    }

    // Optional LLM every N steps
    // Fusion cue & explanation only
    if (fusionOn) {
      playSensorCue(0, scene);
      // removed the “predicted scan times” explanation
    }
            
    simObj.moveAndScan(i, g, scanS);

    // Update robot pose in 3D (first robot only for POV)
    if (i === 0) {
      const [nx, ny] = simObj.robots[0];
      const dx = nx - prev[0], dy = ny - prev[1];
      // heading basis: forward = (sin θ, 0, -cos θ)
      const heading = (dx === 0 && dy === 0) ? scene.robot.rotation.y : Math.atan2(dx, -dy);
      scene.setRobotGridPose(nx, ny, heading);
      // LiDAR animation each decision
      scene.playLidarPulse(0);
    }
  }
}

function playSensorCue(kind, scene) {
    // 0 -> LiDAR pulse (already handled by scene.playLidarPulse),
    // 1 -> thermal glow, 2 -> gas puff, 3 -> (reserved)
    if (!scene) return;
  
    if (kind === 1) {
      const s = new THREE.Mesh(
        new THREE.SphereGeometry(0.6, 16, 16),
        new THREE.MeshBasicMaterial({ color: 0xff7a00, transparent: true, opacity: 0.6 })
      );
      s.position.copy(scene.robot.position).add({ x: 0, y: 1.2, z: -1 });
      scene.sensorLayer.add(s);
      setTimeout(() => {
        s.material.opacity = 0.0;
        scene.sensorLayer.remove(s);
        s.geometry.dispose(); s.material.dispose();
      }, 450);
    } else if (kind === 2) {
      const g = new THREE.Mesh(
        new THREE.SphereGeometry(1.1, 16, 16),
        new THREE.MeshStandardMaterial({ color: 0x44ff88, transparent: true, opacity: 0.25, emissive: 0x115522 })
      );
      g.position.copy(scene.robot.position).add({ x: 0, y: 0.6, z: -0.8 });
      scene.sensorLayer.add(g);
      setTimeout(() => {
        g.material.opacity = 0.0;
        scene.sensorLayer.remove(g);
        g.geometry.dispose(); g.material.dispose();
      }, 650);
    }
  }
  
function updateMetrics(step) {
    if (!simB || !simF) return;
    covVal.textContent  = `B ${(simB.coverage()*100).toFixed(1)}% | F ${(simF.coverage()*100).toFixed(1)}%`;
    timeVal.textContent = `B ${simB.time.toFixed(1)}s | F ${simF.time.toFixed(1)}s`;
    distVal.textContent = `B ${simB.totalDistance().toFixed(0)} | F ${simF.totalDistance().toFixed(0)}`;
    stepsVal.textContent = `${step}`;
}
// New: pick frontiers by minimizing (travelDistance + scanTimeHint)
function pickFusionScanGuided(sim) {
    const clutter = sim.clutter();
    // same hint logic you already have in runOne:
    const hintS = (fusion.best_ms != null)
    ? (fusion.best_ms / 1000) * (0.6 + 0.4 * clutter)
    : (0.25 + 0.5 * clutter);
    const fr = sim.frontiers();
    return sim.robots.map(([rx, ry]) => {
    let best = [rx, ry], bestCost = Infinity;
    for (const [fx, fy] of fr) {
      const v = vantageForFrontier(sim, rx, ry, fx, fy);
      const d = bfsDist(sim, rx, ry, v[0], v[1]);
      const cost = d + hintS;
      if (cost < bestCost) {
        bestCost = cost;
        best = v;
      }
    }
    return best;
  });
}


