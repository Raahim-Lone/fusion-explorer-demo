// simlib.js (ESM) — library-only (no DOM). Exports: Sim, policies.
const FREE = 0, OBST = 1, UNK = -1;
const EPS_JITTER = 0;
export function vantageForFrontier(sim, rx, ry, fx, fy) {
  const neigh = sim._neighbors4(fx, fy);
  let best = null, bestD = Infinity;
  for (const [nx, ny] of neigh) {
    if (sim.map[nx][ny] !== FREE) continue;
    const d = bfsDist(sim, rx, ry, nx, ny);
    if (d !== null && d < bestD) {
      bestD = d;
      best = [nx, ny];
    }
  }
  return best;
}
    
// helper: true shortest-path distance (ignores diagonal, respects sim.gt obstacles)
export function bfsDist(sim, sx, sy, tx, ty) {
  const N = sim.cfg.size, OBST = 1;
  const enc = (x,y) => y * N + x;
  const seen = new Uint8Array(N * N);
  const q = [[sx, sy, 0]];
  seen[enc(sx, sy)] = 1;
  const dirs = [[1,0],[-1,0],[0,1],[0,-1]];
  while (q.length) {
    const [x, y, d] = q.shift();
    if (x === tx && y === ty) return d;
    for (const [dx, dy] of dirs) {
      const nx = x + dx, ny = y + dy;
      if (nx<0||ny<0||nx>=N||ny>=N) continue;
      if (sim.gt[nx][ny] === OBST) continue;
      const id = enc(nx, ny);
      if (!seen[id]) { seen[id] = 1; q.push([nx, ny, d+1]); }
    }
  }
  return null;
}
    
export class Sim {
  constructor(cfg) {
    this.cfg = {
      size: cfg.size ?? 80,
      obstacleProb: cfg.obstacleProb ?? 0.08,
      nRobots: cfg.nRobots ?? 2,
      robotSpeed: cfg.robotSpeed ?? 1.0,
      infoRadius: cfg.infoRadius ?? 4,
      target: cfg.target ?? 0.95,
      maxSteps: cfg.maxSteps ?? 25000,
      seed: cfg.seed ?? 0,
      simpleRoom: !!cfg.simpleRoom
    };
    this._rng = mulberry32(this.cfg.seed);
    const N = this.cfg.size;
    this.gt = Array.from({ length: N }, () =>
      Array.from({ length: N }, () => (this._rng() < this.cfg.obstacleProb ? OBST : FREE))
    );
    // --- Enclose with walls, but carve door gaps so the robot can explore ---
    for (let i = 0; i < N; i++) {
      this.gt[i][0] = OBST; this.gt[i][N-1] = OBST;
      this.gt[0][i] = OBST; this.gt[N-1][i] = OBST;
    }
    // 3-cell doors on each side (centered)
    const mid = Math.floor(N/2);
    if (this.cfg.simpleRoom) {
      // One interior wall that splits the space with a single doorway opening.
      // Choose vertical wall (along y) at x = mid; doorway centered at y = mid.
      // doorway: 2–3 cells wide
      // No perimeter doors, no cross-corridors.
      // Simple layout: remove random clutter inside the perimeter,
      // then add one divider with a 3-cell doorway in the middle.
      for (let x = 1; x < N-1; x++)
        for (let y = 1; y < N-1; y++)
          this.gt[x][y] = FREE;
      for (let y = 1; y < N-1; y++) this.gt[mid][y] = OBST;  // divider
      for (let k = -1; k <= +1; k++) this.gt[mid][mid+k] = FREE; // doorway
    } else {
      // Original: carve small doors on each side (centered) and cross corridor
      for (let k = -1; k <= 1; k++) {
        this.gt[mid+k][0]     = FREE;  // north
        this.gt[mid+k][N-1]   = FREE;  // south
        this.gt[0][mid+k]     = FREE;  // west
        this.gt[N-1][mid+k]   = FREE;  // east
      }
      for (let x = 1; x < N-1; x++) this.gt[x][mid] = FREE;
      for (let y = 1; y < N-1; y++) this.gt[mid][y] = FREE;
    }
    this.map = Array.from({ length: N }, () => Array.from({ length: N }, () => UNK)); // obstacles NOT known initially
    const s = Math.max(2, Math.floor(N / 10));
    // Ensure a clear 2–3 cell patch around spawn for valid frontiers
    for (let x = 0; x < Math.min(3, N); x++)
      for (let y = 0; y < Math.min(3, N); y++)
        this.gt[x][y] = FREE;
    for (let x = 0; x < s; x++) for (let y = 0; y < s; y++) this.map[x][y] = this.gt[x][y];
    this.robots = Array.from({ length: Math.max(1, this.cfg.nRobots) }, (_, i) => i === 0 ? [0,0] : [s-1, s-1]);
    this.time = 0;
    this.step = 0;
    this._dist = Array(this.robots.length).fill(0);
  }
  cloneForAB() {
    // Pure clone: shares RNG, GT, map, robot poses, distance & time counters
    const b = Object.create(Sim.prototype);
    b.cfg    = { ...this.cfg };
    b._rng   = this._rng;
    b.gt     = this.gt.map(row => row.slice());
    b.map    = this.map.map(row => row.slice());
    b.robots = this.robots.map(r => r.slice());
    b.time   = this.time;
    b.step   = this.step;
    b._dist  = this._dist.slice();
    return b;
  }
  totalDistance(){ return this._dist.reduce((a,b)=>a+b,0); }
  clutter() {
    const N=this.cfg.size; let obst=0;
    for (let x=0;x<N;x++) for (let y=0;y<N;y++) if (this.gt[x][y]===OBST) obst++;
    return obst/(N*N);
  }
  coverage(){
    const N=this.cfg.size; let known=0;
    for (let x=0;x<N;x++) for (let y=0;y<N;y++) if (this.map[x][y]!==UNK) known++;
    return known / Math.max(1, N*N);
  }
  _neighbors4(x,y){
    const N=this.cfg.size, out=[];
    if (x+1<N) out.push([x+1,y]);
    if (x-1>=0) out.push([x-1,y]);
    if (y+1<N) out.push([x,y+1]);
    if (y-1>=0) out.push([x,y-1]);
    return out;
  }
  frontiers(){
    const N=this.cfg.size, fr=[];
    for (let x=0;x<N;x++) for (let y=0;y<N;y++){
      if (this.map[x][y]!==UNK) continue;
      const neigh=this._neighbors4(x,y);
      if (neigh.some(([nx,ny])=>this.map[nx][ny]===FREE)) fr.push([x,y]);
    }
    return fr;
  }
  aStarDist(ax,ay,bx,by){
    const N=this.cfg.size;
    if (ax<0||ay<0||bx<0||by<0) return null;
    if (ax>=N||ay>=N||bx>=N||by>=N) return null;
    return Math.abs(ax-bx)+Math.abs(ay-by);
  }
  _reveal(cx,cy,r=4){
    const N=this.cfg.size;
    for (let x=Math.max(0,cx-r); x<=Math.min(N-1,cx+r); x++){
      for (let y=Math.max(0,cy-r); y<=Math.min(N-1,cy+r); y++){
        this.map[x][y] = this.gt[x][y];
      }
    }
  }
  moveAndScan(robotIdx, goal, scanS){
    const r=this.robots[robotIdx];
    const d=this.aStarDist(r[0],r[1],goal[0],goal[1]);
    if (d==null){ return; }
    if (d===0){
      this.time += Math.max(0, scanS||0);
      this._reveal(r[0], r[1], 3);
      this.step++; return;
    }
    // move exactly ONE grid cell toward the goal (4-connected)
    let nx=r[0], ny=r[1];
    const dx = Math.sign(goal[0]-r[0]);
    const dy = Math.sign(goal[1]-r[1]);
    // try axis with greater |distance| first
    const tryXFirst = Math.abs(goal[0]-r[0]) >= Math.abs(goal[1]-r[1]);
    const trySteps = tryXFirst ? [[dx,0],[0,dy]] : [[0,dy],[dx,0]];
    let moved=false;
    for (const [sx,sy] of trySteps){
      const tx = r[0]+sx, ty = r[1]+sy;
      if (tx<0||ty<0||tx>=this.cfg.size||ty>=this.cfg.size) continue;
      if (this.gt[tx][ty]===OBST) continue;  // don’t step into obstacle
      nx=tx; ny=ty; moved=true; break;
    }
    if (!moved){
      // stuck: just “scan” in place so we still gain info
      this.time += Math.max(0, scanS || 0);
      this._reveal(r[0], r[1], 2);
      this.step++; return;
    }
    nx=Math.max(0, Math.min(this.cfg.size-1, nx));
    ny=Math.max(0, Math.min(this.cfg.size-1, ny));
    this._dist[robotIdx]+=1;
    this.time += 1 / Math.max(1e-6, (this.cfg.robotSpeed||1.0)); // robotSpeed slows sim time
    r[0]=nx; r[1]=ny;
    this._reveal(r[0], r[1], 2);
    this.time += Math.max(0, scanS || 0);
    this._reveal(r[0], r[1], 3);
    this.step++;
  }
}

// BFS to the nearest unknown cell (map==UNK), traversing cells where gt!=OBST.
function nearestUnknown(sim, sx, sy) {
  const N = sim.cfg.size, UNK = -1, OBST = 1;
  const q = [[sx, sy]];
  const seen = new Uint8Array(N * N);
  const enc = (x, y) => y * N + x;
  seen[enc(sx, sy)] = 1;
  const dirs = [[1,0],[-1,0],[0,1],[0,-1]];
  while (q.length) {
    const [x, y] = q.shift();
    if (sim.map[x][y] === UNK) return [x, y];
    for (const [dx, dy] of dirs) {
      const nx = x + dx, ny = y + dy;
      if (nx < 0 || ny < 0 || nx >= N || ny >= N) continue;
      if (sim.gt[nx][ny] === OBST) continue;
      const id = enc(nx, ny);
      if (!seen[id]) { seen[id] = 1; q.push([nx, ny]); }
    }
  }
  return [sx, sy];
}

export const policies = {
  nf(sim){
    const goals=[], fr=sim.frontiers();
    if (fr.length === 0) {
      for (const [rx, ry] of sim.robots) goals.push(nearestUnknown(sim, rx, ry));
      return goals;
    }
    for (const [rx,ry] of sim.robots){
      let best=[rx,ry], bestD=Infinity;
      for (const [fx,fy] of fr){
        const v = vantageForFrontier(sim, rx, ry, fx, fy);
        if (!v) continue;
        const d = bfsDist(sim, rx, ry, v[0], v[1]);
        if (d == null) continue;
        const dj = d;
        if (dj < bestD){ bestD = dj; best = v; }
      }
      if (best[0] === rx && best[1] === ry) {
        best = nearestUnknown(sim, rx, ry);
      }
      goals.push(best);
    }
    return goals;
  },
  igRatio(sim){
    const goals=[], fr=sim.frontiers();
    if (fr.length === 0) {
      for (const [rx, ry] of sim.robots) goals.push(nearestUnknown(sim, rx, ry));
      return goals;
    }
    for (const [rx,ry] of sim.robots){
      let best=[rx,ry], bestScore=-1e9;
      for (const [fx,fy] of fr){
        const v = vantageForFrontier(sim, rx, ry, fx, fy);
        if (!v) continue;
        const d = bfsDist(sim, rx, ry, v[0], v[1]);
        if (d == null) continue;
        const ig=infoGain(sim,fx,fy,sim.cfg.infoRadius);
        const score = ig / d + 1e-4*(sim._rng()-0.5);
        if (score>bestScore){ bestScore=score; best=v; }
      }
      goals.push(best);
    }
    return goals;
  },
  igMinus(sim, lambda=1.0){
    const goals=[], fr=sim.frontiers();
    if (fr.length === 0) {
      for (const [rx, ry] of sim.robots) goals.push(nearestUnknown(sim, rx, ry));
      return goals;
    }
    for (const [rx,ry] of sim.robots){
      let best=[rx,ry], bestScore=-1e9;
      for (const [fx,fy] of fr){
        const v = vantageForFrontier(sim, rx, ry, fx, fy);
        if (!v) continue;
        const d = bfsDist(sim, rx, ry, v[0], v[1]);
        if (d == null) continue;
        const ig=infoGain(sim,fx,fy,sim.cfg.infoRadius);
        const score = ig - lambda * d + 1e-4*(sim._rng()-0.5);
        if (score>bestScore){ bestScore=score; best=v; }
      }
      goals.push(best);
    }
    return goals;
  },
  cqliteLite(sim, params={}){
    const hystR=params.hystRadius ?? 3.0;
    const hystG=params.hystGain ?? 2.0;
    const goals=[], fr=sim.frontiers();
    if (fr.length === 0) {
      for (const [rx, ry] of sim.robots) goals.push(nearestUnknown(sim, rx, ry));
      return goals;
    }
    for (const [rx,ry] of sim.robots){
      let best=[rx,ry], bestScore=-1e9;
      for (const [fx,fy] of fr){
        const v = vantageForFrontier(sim, rx, ry, fx, fy);
        if (!v) continue;
        const d = bfsDist(sim, rx, ry, v[0], v[1]);
        if (d == null) continue;
        const ig=infoGain(sim,fx,fy,sim.cfg.infoRadius);
        const hyst = (Math.abs(rx-fx)+Math.abs(ry-fy)) <= hystR ? hystG : 0.0;
        const score = 3.0*ig - d + hyst + 1e-4*(sim._rng()-0.5);
        if (score>bestScore){ bestScore=score; best=v; }
      }
      goals.push(best);
    }
    return goals;
  },
};

// helpers
function infoGain(sim,x,y,r){
  let ig=0, N=sim.cfg.size;
  for (let i=Math.max(0,x-r); i<=Math.min(N-1,x+r); i++){
    for (let j=Math.max(0,y-r); j<=Math.min(N-1,y+r); j++){
      if (sim.map[i][j]===UNK) ig++;
    }
  }
  return ig;
}
// Choose a reachable FREE vantage cell adjacent to a frontier (fx,fy)
function mulberry32(a){return function(){let t=a+=0x6D2B79F5;t=Math.imul(t^t>>>15,t|1);t^=t+Math.imul(t^t>>>7,t|61);return ((t^t>>>14)>>>0)/4294967296;}}
