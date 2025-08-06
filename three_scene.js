// three_scene.js — Three.js scene, robot+obstacles, sensors, POV inset, simple post
import * as THREE from 'three';
import { OrbitControls } from "https://cdn.jsdelivr.net/npm/three@0.161/examples/jsm/controls/OrbitControls.js";
import { GLTFLoader } from "https://cdn.jsdelivr.net/npm/three@0.161/examples/jsm/loaders/GLTFLoader.js";
import { EffectComposer } from "https://cdn.jsdelivr.net/npm/three@0.161/examples/jsm/postprocessing/EffectComposer.js";
import { RenderPass } from "https://cdn.jsdelivr.net/npm/three@0.161/examples/jsm/postprocessing/RenderPass.js";
import { UnrealBloomPass } from "https://cdn.jsdelivr.net/npm/three@0.161/examples/jsm/postprocessing/UnrealBloomPass.js";
// helper: find contiguous runs of 1’s in a [0/1] array
function runs(arr) {
  const out = [];
  let s = -1;
  for (let i = 0; i < arr.length; i++) {
    if (arr[i] === 1 && s < 0) s = i;
    if ((arr[i] !== 1 || i === arr.length - 1) && s >= 0) {
      const e = (arr[i] === 1 && i === arr.length - 1) ? i : i - 1;
      out.push([s, e]);
      s = -1;
    }
  }
  return out;
}
    
    
export class ThreeScene {
  /**
   * Creates a ThreeScene object, which encapsulates a 3D scene, main renderer,
   * and POV inset renderer.
   * @param {HTMLCanvasElement} mainCanvas - main canvas to render to
   * @param {HTMLCanvasElement} insetCanvas - inset canvas to render to
   */

  constructor(mainCanvas, insetCanvas) {
    this.canvas = mainCanvas;
    this.inset = insetCanvas || null;
    this.scene = new THREE.Scene();
    this.scene.fog = new THREE.FogExp2(0x0b0f16, 0.04); // indoor haze
    
    this.renderer = new THREE.WebGLRenderer({ canvas: mainCanvas, antialias: false });
    this.renderer.setPixelRatio(1);
    const mw = (mainCanvas && mainCanvas.clientWidth)  ? mainCanvas.clientWidth  : 800;
    const mh = (mainCanvas && mainCanvas.clientHeight) ? mainCanvas.clientHeight : 600;
    this.renderer.setSize(mw, mh, false);
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.renderer.physicallyCorrectLights = true;
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.35;
    this.renderer.setClearColor(0x0b0f16, 1);  // dark interior
    
    // Cameras
    this.camMain = new THREE.PerspectiveCamera(72, mw / mh, 0.05, 200); // main chase camera
    
    const iw = (insetCanvas && insetCanvas.clientWidth)  ? insetCanvas.clientWidth  : 320;
    const ih = (insetCanvas && insetCanvas.clientHeight) ? insetCanvas.clientHeight : 200;
    this.camPOV = new THREE.PerspectiveCamera(70, iw / ih, 0.1, 200);
    
    // Post
    // Controls (orbit around robot; chase by default)
    this.controls = new OrbitControls(this.camMain, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.enablePan = false;
    this.controls.minDistance = 4;
    this.controls.maxDistance = 28;
    this.controls.minPolarAngle = Math.PI/6;
    this.controls.maxPolarAngle = Math.PI/2.2;
    this._userOrbiting = false;
    this.controls.addEventListener('start', ()=>{ this._userOrbiting = true; });
    this.controls.addEventListener('end',   ()=>{ this._userOrbiting = false; });
    this._chaseOffset = new THREE.Vector3(0, 5.5, 10.5); // a touch higher/farther for clearer context
    
    this.composer = new EffectComposer(this.renderer);
    this.composer.addPass(new RenderPass(this.scene, this.camMain));
    this.bloom = new UnrealBloomPass(new THREE.Vector2(mainCanvas.width, mainCanvas.height), 0.25, 0.7, 0.9);
    this.composer.addPass(this.bloom);

    // Lights
    const hemi = new THREE.HemisphereLight(0x6b91a3, 0x1a1f22, 0.3);
    this.scene.add(hemi);
    const sun = new THREE.DirectionalLight(0xffffff, 1.2);
    sun.position.set(40, 80, 10);
    sun.castShadow = true;
    sun.shadow.mapSize.set(1024,1024);
    this.scene.add(sun);
    this.scene.add(new THREE.AmbientLight(0xffffff, 0.35));
    // warm fill from a “broken roof” opening
    const fill = new THREE.DirectionalLight(0xffe0b0, 0.4);
    fill.position.set(-20, 40, -30);
    this.scene.add(fill);
    // orientation/grid to give depth reference
    // ---- World scale / centering (set when we know grid size) ----
    this.cell = 2.0;          // world units per grid cell
    this.halfSpan = 0.0;      // computed from N: ((N-1)*cell)/2
    this.gridHelper = null;
    this.ground = null;
    
    // Objs
    this.solids = new THREE.Group();       // walls/roof etc for raycasts
    this.scene.add(this.solids);
    this.obstacles = new THREE.Group();     // crates/hard obstacles
    this.scene.add(this.obstacles);
    this.dressing = new THREE.Group();      // broken walls/beams/debris/fires
    this.scene.add(this.dressing);
    
    // Robot & headlamp
    this.robot = new THREE.Group();
    this.scene.add(this.robot);
    this._hasPose = false;
    this.motionSlowdown = 1.0;  // lerp divisor
    this._targetPos = new THREE.Vector3();
    this._targetHeading = 0;
    
    this.headlamp = new THREE.SpotLight(0xffffff, 1.2, 40, Math.PI/6, 0.4, 1.0);
    this.headlamp.castShadow = true;
    this.robot.add(this.headlamp);
    this.headlamp.position.set(0, 2, 0);
    this.headlamp.target.position.set(0, 0, -3);
    this.robot.add(this.headlamp.target);

    // LiDAR / thermal / gas containers
    this.sensorLayer = new THREE.Group(); this.scene.add(this.sensorLayer);
    this.lidarPoints = makePointCloud(4000); this.sensorLayer.add(this.lidarPoints);
    this.scanRing = makeScanRing(); this.sensorLayer.add(this.scanRing);
    this.coverCanvas = null; this.coverCtx = null; this.coverTex = null; this.coverMesh = null;

    // Attempt to load robot glTF
    this._loadRobot();

    // Resize
    window.addEventListener("resize", () => this._resize());
    this._resize();

    // State
    this.cellToWorld = (x,y) => new THREE.Vector3(
      x*this.cell - this.halfSpan,
      0,
      y*this.cell - this.halfSpan
    ); // centered grid → world
    this.showPOV = true;
    this.clock = new THREE.Clock();
  }

  async _loadRobot() {
    try {
      const loader = new GLTFLoader();
      const glb = await loader.loadAsync("./assets/robot.glb");
      const model = glb.scene;
      model.traverse(o => { if (o.isMesh) { o.castShadow = true; o.receiveShadow = true; o.material.metalness = 0.4; o.material.roughness = 0.6; }});
      model.scale.set(1,1,1);
      model.position.set(0,0,0);
      this.robot.add(model);
    } catch {
      // fallback: capsule + wheels
      const body = new THREE.Mesh(new THREE.CapsuleGeometry(0.6, 1.1, 8, 16),
        new THREE.MeshStandardMaterial({ color: 0x2895ff, metalness: 0.3, roughness: 0.5 }));
      body.castShadow = true; this.robot.add(body);
      const wheelGeo = new THREE.CylinderGeometry(0.25,0.25,0.5,16);
      const wheelMat = new THREE.MeshStandardMaterial({ color: 0x111111, metalness:0, roughness:0.9 });
      for (const off of [[0.6,-0.1,0.4],[-0.6,-0.1,0.4],[0.6,-0.1,-0.4],[-0.6,-0.1,-0.4]]) {
        const wh = new THREE.Mesh(wheelGeo, wheelMat); wh.rotation.z = Math.PI/2; wh.position.set(...off);
        wh.castShadow = true; this.robot.add(wh);
      }
    }
  }

  setRobotGridPose(x, y, headingRad=0) {
    const p = this.cellToWorld(x,y);
    p.y = 0.2;
    this._targetPos.copy(p);
    this._targetHeading = headingRad;
    if (!this._hasPose) {
      this.robot.position.copy(this._targetPos);
      this.robot.rotation.y = this._targetHeading;
      this._hasPose = true;
    }
  }    
  buildObstaclesFromGrid(gt, opts = {}) {
    // (Re)create ground & grid for this N and center world
    const N = gt.length;
    this._setupGroundAndGrid(N);
    this._gt = gt; // keep GT for perimeter wall gaps
    this._simpleRoom = !!opts.simpleRoom;
    while (this.solids.children.length)    this.solids.remove(this.solids.children[0]);
    while (this.obstacles.children.length) this.obstacles.remove(this.obstacles.children[0]);
    while (this.dressing.children.length)  this.dressing.remove(this.dressing.children[0]);
    
    // Clear obstacles
    if (this._simpleRoom) {
      this._buildSimpleLayout(gt.length);
      this._addSimpleRoof(gt.length);
      this._spawnFiresSimple(gt.length, 2); // a few large fires (also mark GT)
      return;
    }
    // Big visible fires where the GT has hazards (OBST = 1)
    // Clustered hazards: fewer but larger
    const fireChance = this._simpleRoom ? 0.0 : 0.08; // no fires in the simple fallback layout
    for (let x=1;x<N-1;x++){
      for (let y=1;y<N-1;y++){
        if (gt[x][y] !== 1 || Math.random()>fireChance) continue;
        const p = this.cellToWorld(x,y);
        const fire = makeFireStack(this.cell*4.0);
        fire.position.set(p.x, 0.02, p.z);
        // ── mark the sim.gt so the robot won’t step into this fire area
        const xi = Math.round((p.x + this.halfSpan) / this.cell);
        const yi = Math.round((p.z + this.halfSpan) / this.cell);
        const Ncells = this._gt.length;
        // fireGraphicDiameter = cell*4.0 → radius in cells = 4/2 = 2
        const r = 2;
        for (let dx = -r; dx <= r; dx++) {
          for (let dy = -r; dy <= r; dy++) {
            const xx = xi + dx, yy = yi + dy;
            if (xx >= 0 && yy >= 0 && xx < Ncells && yy < Ncells && dx*dx + dy*dy <= r*r) {
              this._gt[xx][yy] = 1; // OBST
            }
          }
        }
        this.obstacles.add(fire);
      }
    }
    this._buildHouseInterior(gt);
    this._buildRoomsAndFurniture(gt.length);
  }
  _box(w, h, d, mat, x, y, z, rotY=0, into='solids') {
    const g = new THREE.BoxGeometry(w, h, d);
    const m = mat.clone ? mat.clone() : mat;
    const mesh = new THREE.Mesh(g, m);
    mesh.castShadow = true; mesh.receiveShadow = true;
    mesh.position.set(x, y, z); mesh.rotation.y = rotY;
    this[into].add(mesh);
    return mesh;
  }
  _addSimpleRoof(N) {
    const size = N * this.cell;
    const H = this.cell * 4.2;
    const roofMat = new THREE.MeshStandardMaterial({
      color: 0xbfbfbf, roughness: 0.75, metalness: 0.0, side: THREE.DoubleSide
    });
    // Safe: _maybeApplyTexture ignores unknown files
    this._maybeApplyTexture(roofMat, 'roof_basecolor.jpg', 'roof_normal.jpg', 2.0);
    const roof = new THREE.Mesh(new THREE.PlaneGeometry(size * 0.92, size * 0.92), roofMat);
    roof.rotation.x = -Math.PI / 2;
    roof.position.y = H + 0.05;
    roof.receiveShadow = true;
    this.solids.add(roof);
  }
  _spawnFiresSimple(N, count = 3) {
    const span = (N * this.cell) * 0.38; // keep fires away from walls
    let placed = 0;
    while (placed < count) {
      const fx = (Math.random() * 2 - 1) * span;
      const fz = (Math.random() * 2 - 1) * span;
      // avoid blocking the doorway gap near world z≈0 on the divider x≈0
      // avoid blocking the corridor (just skip, don’t decrement)
      if (Math.abs(fz) < this.cell * 2.5 && Math.abs(fx) < this.cell * 1.2) {
        continue;
      }
      const fire = makeFireStack(this.cell * 4.0);
      fire.position.set(fx, 0.02, fz);
      this.obstacles.add(fire);
      // Write a circular obstacle patch into the GT so pathing avoids the fire
      if (this._gt) {
        const xi = Math.round((fx + this.halfSpan) / this.cell);
        const yi = Math.round((fz + this.halfSpan) / this.cell);
        const Ncells = this._gt.length;
        const r = 2; // ~fire radius in cells
        for (let dx = -r; dx <= r; dx++) {
          for (let dy = -r; dy <= r; dy++) {
            const xx = xi + dx, yy = yi + dy;
            if (xx >= 0 && yy >= 0 && xx < Ncells && yy < Ncells && dx*dx + dy*dy <= r*r) {
              this._gt[xx][yy] = 1; // OBST
            }
          }
        }
      }
      placed++;
    }
  }
  _buildRoomsAndFurniture(N){
    if (this._simpleRoom) return; // no furniture in the simple fallback layout

    const size = N*this.cell, th=this.cell*0.25, H=this.cell*3.6;
    const W = size*0.92;

    const wall = new THREE.MeshStandardMaterial({ color: 0xdad7d1, roughness: 0.9, metalness: 0.0 });
    const ct   = new THREE.MeshStandardMaterial({ color: 0xbdb8ae, roughness: 0.85, metalness: 0.05 });
    const wood = new THREE.MeshStandardMaterial({ color: 0x6e5a43, roughness: 0.85, metalness: 0.05 });
    const metal= new THREE.MeshStandardMaterial({ color: 0xb0b7c1, roughness: 0.35, metalness: 0.4 });

    // partition layout (simple cross): hallway in middle
    // North–South wall (with two interior openings)
    this._box(W*0.92, H, th, wall, 0, H/2, 0, 0);
    const gap = this.cell*1.8, seg = (W*0.92 - 2*gap)/3;
    this._box(seg, H, th, wall, -W*0.46 + seg/2, H/2, 0, 0);
    this._box(seg, H, th, wall,  W*0.46 - seg/2, H/2, 0, 0);
    // East–West wall
    this._box(W*0.92, H, th, wall, 0, H/2, 0, Math.PI/2);

    // Kitchen (NW quadrant)
    const kx = -W*0.25, kz =  W*0.25;
    this._box(this.cell*4.0, this.cell*0.9, this.cell*1.2, ct, kx, this.cell*0.45, kz - this.cell*2.0);
    this._box(this.cell*1.2, this.cell*0.9, this.cell*3.6, ct, kx - this.cell*2.4, this.cell*0.45, kz - this.cell*0.4);
    this._box(this.cell*1.0, this.cell*2.2, this.cell*1.0, metal, kx - this.cell*1.2, this.cell*1.1, kz + this.cell*0.6);
    this._box(this.cell*1.0, this.cell*1.1, this.cell*1.0, metal, kx + this.cell*1.2, this.cell*0.55, kz + this.cell*0.6);

    // Living room (NE quadrant)
    const lx =  W*0.25, lz =  W*0.25;
    this._box(this.cell*2.6, this.cell*0.9, this.cell*1.0, wood, lx, this.cell*0.45, lz - this.cell*1.2);
    this._box(this.cell*2.6, this.cell*0.5, this.cell*0.2, wood, lx, this.cell*0.95, lz - this.cell*1.5);
    this._box(this.cell*1.4, this.cell*0.45, this.cell*0.7, wood, lx, this.cell*0.225, lz - this.cell*2.6);

    // Bedroom (SW quadrant)
    const bx = -W*0.25, bz = -W*0.25;
    this._box(this.cell*2.2, this.cell*0.6, this.cell*1.4, wood, bx, this.cell*0.3, bz);
    this._box(this.cell*0.6, this.cell*0.6, this.cell*0.6, wood, bx + this.cell*1.4, this.cell*0.3, bz + this.cell*0.9);

    // Dining (SE quadrant)
    const dx =  W*0.25, dz = -W*0.25;
    this._box(this.cell*2.0, this.cell*0.8, this.cell*1.2, wood, dx, this.cell*0.4, dz);
    this._box(this.cell*0.5, this.cell*0.6, this.cell*0.5, wood, dx - this.cell*1.0, this.cell*0.3, dz + this.cell*0.9);
    this._box(this.cell*0.5, this.cell*0.6, this.cell*0.5, wood, dx + this.cell*1.0, this.cell*0.3, dz + this.cell*0.9);
  }

  playLidarPulse(dirRad=0) {
    // animate scan ring & raycast points in a cone
    this.scanRing.visible = true;
    this.scanRing.material.opacity = 0.7;
    this.scanRing.scale.set(0.01, 0.01, 0.01);
    // compute hits
    const origin = this.robot.position.clone().add(new THREE.Vector3(0, 1.0, 0));
    const rays=120, maxDist=22, spread=Math.PI/4;
    const positions = this.lidarPoints.geometry.getAttribute("position");
    const alphas = this.lidarPoints.geometry.getAttribute("alpha");
    let idx=0;
    const rc = new THREE.Raycaster();
    for (let i = 0; i < rays;  i++) {
      const a = dirRad + (i/(rays-1)-0.5)*spread;
      const dir = new THREE.Vector3(Math.sin(a), 0, -Math.cos(a)).normalize();
      rc.set(origin, dir);
      const hits = rc.intersectObjects(this.solids.children, true);
      const hit = hits.find(h => h.distance < maxDist);
      const pos = hit ? hit.point : origin.clone().add(dir.multiplyScalar(maxDist));
      positions.setXYZ(idx, pos.x, pos.y, pos.z);
      alphas.setX(idx, 1.0);
      idx++;
    }
    for (; idx<positions.count; idx++){ // clear rest
      positions.setXYZ(idx, 0, -999, 0);
      alphas.setX(idx, 0.0);
    }
    positions.needsUpdate = true;
    alphas.needsUpdate = true;
  }

  fadeSensors(dt) {
    // fade lidar points & scan ring
    const alphas = this.lidarPoints.geometry.getAttribute("alpha");
    for (let i=0;i<alphas.count;i++){
      const v = Math.max(0, alphas.getX(i) - dt*1.5);
      alphas.setX(i, v);
    }
    alphas.needsUpdate = true;
    if (this.scanRing.visible){
      this.scanRing.material.opacity = Math.max(0, this.scanRing.material.opacity - dt*1.2);
      this.scanRing.scale.multiplyScalar(1 + dt*2.5);
      if (this.scanRing.material.opacity <= 0.01) this.scanRing.visible = false;
    }
  }

  render(showPOV = true) {
    // main view with bloom/composer
    this.composer.render();

    // inset POV (reuse or lazily create its own renderer)
    if (showPOV && this.inset) {
      if (!this.povRenderer) {
        this.povRenderer = new THREE.WebGLRenderer({ canvas: this.inset, antialias: true, alpha: false });
        this.povRenderer.setPixelRatio(Math.min(devicePixelRatio, 2));
        this.povRenderer.setSize(this.inset.clientWidth, this.inset.clientHeight, false);
        this.povRenderer.shadowMap.enabled = true;
      }
      // keep its size in sync
      const iw = this.inset.clientWidth, ih = this.inset.clientHeight;
      this.povRenderer.setSize(iw, ih, false);
      this.povRenderer.render(this.scene, this.camPOV);
    }
  }

  tick() {
    const dt = this.clock.getDelta();
    this.fadeSensors(dt);
    if (this._hasPose) {
      const lerp = Math.min(1, dt / (0.12 * this.motionSlowdown));
      this.robot.position.lerp(this._targetPos, lerp);
      // shortest-arc yaw
      const cur = this.robot.rotation.y, tgt = this._targetHeading;
      const ang = Math.atan2(Math.sin(tgt-cur), Math.cos(tgt-cur));
      this.robot.rotation.y = cur + ang * lerp;
      // POV follows head
      const heading = this.robot.rotation.y;
      const desired = this.robot.position.clone()
        .add(this._chaseOffset.clone().applyAxisAngle(new THREE.Vector3(0,1,0), heading));
      if (!this._userOrbiting) this.camMain.position.lerp(desired, 0.15);

      this.controls.target.copy(this.robot.position).add(new THREE.Vector3(0,1.2,0));
      this.controls.update();
    }
  }

  _resize() {
    const w = this.canvas?.clientWidth, h = this.canvas?.clientHeight;
    if (w && h){
      this.renderer.setSize(w, h, false);
      this.camMain.aspect = w / h;
      this.camMain.updateProjectionMatrix();
    }
    if (this.inset){
      const iw = this.inset.clientWidth, ih = this.inset.clientHeight;
      if (iw && ih){
        this.camPOV.aspect = iw / ih; this.camPOV.updateProjectionMatrix();
      }
    }
  }

  _setupGroundAndGrid(N){
    // update span / centering
    this.halfSpan = ((N - 1) * this.cell) / 2;
    const size = N * this.cell;
    // grid helper
    if (this.gridHelper) {
      this.scene.remove(this.gridHelper);
      this.gridHelper.geometry?.dispose?.();
      this.gridHelper = null;
    }
    // ground
    if (this.ground) { this.scene.remove(this.ground); this.ground.geometry.dispose(); this.ground.material.dispose(); }
    const gmat = new THREE.MeshStandardMaterial({ color: 0x777777, metalness: 0.0, roughness: 0.95 });
    this._maybeApplyTexture(gmat, 'ground_basecolor.jpg', 'ground_normal.png', 8.0);
    this.ground = new THREE.Mesh(new THREE.PlaneGeometry(size, size), gmat);
    this.ground.rotation.x = -Math.PI/2;
    this.ground.receiveShadow = true;
    this.scene.add(this.ground);
    
    // coverage canvas
    if (this.coverMesh) { 
      this.scene.remove(this.coverMesh); 
      this.coverMesh.geometry.dispose(); 
      this.coverMesh.material.map.dispose(); 
      this.coverMesh.material.dispose(); 
    }
    this.coverCanvas = document.createElement('canvas');
    this.coverCanvas.width = N; this.coverCanvas.height = N;
    this.coverCtx = this.coverCanvas.getContext('2d', { willReadFrequently: true });
    this.coverTex = new THREE.CanvasTexture(this.coverCanvas);
    this.coverTex.magFilter = THREE.NearestFilter; this.coverTex.minFilter = THREE.NearestFilter;
    const m = new THREE.MeshBasicMaterial({ map: this.coverTex, transparent: true, opacity: 0.7 });
    this.coverMesh = new THREE.Mesh(new THREE.PlaneGeometry(size, size), m);
    this.coverMesh.rotation.x = -Math.PI/2; this.coverMesh.position.y = 0.012;
    this.scene.add(this.coverMesh);
  }

  _maybeApplyTexture(mat, baseName, normalNames, repeatScale=4.0){
    // Only try files you confirmed exist; ignore errors
    const ok = new Set([
      'crate_basecolor.jpg','crate_normal.jpg',
      'ground_basecolor.jpg','ground_normal.png','ground_normal.npg'
    ]);
    const loader = new THREE.TextureLoader();
    const applyRepeat = (tex) => {
      tex.wrapS = tex.wrapT = THREE.RepeatWrapping;
      tex.repeat.set(repeatScale, repeatScale);
    };
    if (ok.has(baseName)){
      loader.load(
        `./assets/${baseName}`,
        t => { applyRepeat(t); mat.map = t; mat.needsUpdate = true; },
        undefined,
        ()=>{}
      );
    }
    const normals = Array.isArray(normalNames) ? normalNames : [normalNames];
    for (const name of normals) {
      if (!name || !ok.has(name)) continue;
      loader.load(
        `./assets/${name}`,
        t => { applyRepeat(t); mat.normalMap = t; mat.needsUpdate = true; },
        undefined,
        ()=>{}
      );
    }
  }

  // Paint coverage from Sim: unknown=black, free=transparent, obst=no overlay
  updateCoverageFromSim(sim){
    if (!this.coverCtx || !sim) return;
    const N = sim.cfg.size, img = this.coverCtx.createImageData(N, N), d = img.data;
    const UNK=-1, FREE=0, OBST=1;
    let k=0;
    for (let y=0; y<N; y++){
      for (let x=0; x<N; x++){
        const v = sim.map[x][y];
        if (v===UNK){ d[k++]=5; d[k++]=7; d[k++]=10; d[k++]=255; }
        else if (v===FREE){ d[k++]=0; d[k++]=0; d[k++]=0; d[k++]=0; }
        else { d[k++]=0; d[k++]=0; d[k++]=0; d[k++]=0; }
      }
    }
    // frontiers
    for (const [fx,fy] of sim.frontiers()){
      const i = (fy*N + fx)*4; d[i+0]=32; d[i+1]=41; d[i+2]=55; d[i+3]=255;
    }
    // robots
    for (const [rx,ry] of sim.robots){
      const i = (ry*N + rx)*4; d[i+0]=255; d[i+1]=210; d[i+2]=60; d[i+3]=255;
    }
    this.coverCtx.putImageData(img, 0, 0); this.coverTex.needsUpdate = true;
  }

  // ---------------------------------------------------
  // Build exactly one big room + one divider with one doorway
  _buildSimpleLayout(N) {
    const th = this.cell * 0.6;
    const H  = this.cell * 4.2;
    const size = N * this.cell;
    const half = size / 2;
    // simple wall-material
    const wallMat = new THREE.MeshStandardMaterial({ color: 0xe7e2d9, roughness: 0.8, metalness: 0.0 });
    const mk = (w, rot, x, z) => this._box(w, H, th, wallMat, x, H/2, z, rot, 'solids');

    // 1) Perimeter (closed)
    mk(size, 0,             0, -half + th/2);             // south
    mk(size, 0,             0, +half - th/2);             // north
    mk(size, Math.PI/2, -half + th/2,  0);                 // west
    mk(size, Math.PI/2, +half - th/2,  0);                 // east

    // 2) Divider at X=0, with a 3-cell-wide doorway in the middle
    const doorCells = 6;
    const doorW = doorCells * this.cell;
    const segLen = (size - doorW) / 2;
    // lower segment (below doorway)
    mk(segLen, Math.PI/2, 0, -doorW/2 - segLen/2);
    // upper segment (above doorway)
    mk(segLen, Math.PI/2, 0, +doorW/2 + segLen/2);
  }
  
  _buildHouseInterior(gt){
    const N = gt.length;
    const size = N*this.cell, H=this.cell*4.2;  // higher walls & roof
    while (this.solids.children.length) this.solids.remove(this.solids.children[0]);
    while (this.dressing.children.length) this.dressing.remove(this.dressing.children[0]);

    const wallMat = new THREE.MeshStandardMaterial({ color: 0xe7e2d9, roughness: 0.8, metalness: 0.0 });
    this._maybeApplyTexture(wallMat, 'plaster_basecolor.jpg', 'plaster_normal.jpg', 3.0);
    const th = this.cell * 0.60;
    const W = size*0.92;
    const halfW = W*0.5;

    // Build perimeter walls as segments where GT border == OBST
    const mkSeg = (w, rot, x, z) => {
      const g = new THREE.BoxGeometry(w, H, th);
      const m = wallMat.clone();
      const mesh = new THREE.Mesh(g, m);
      mesh.castShadow = true; mesh.receiveShadow = true;
      mesh.position.set(x, H/2, z); mesh.rotation.y = rot;
      this.solids.add(mesh);
    };
    // North (y=N-1, along x), South (y=0, along x)
    const north = runs(Array.from({length:N}, (_,x)=> gt[x][N-1]===1?1:0));
    const south = runs(Array.from({length:N}, (_,x)=> gt[x][0]   ===1?1:0));
    // East (x=N-1, along y), West (x=0, along y)
    const east  = runs(Array.from({length:N}, (_,y)=> gt[N-1][y]===1?1:0));
    const west  = runs(Array.from({length:N}, (_,y)=> gt[0][y]   ===1?1:0));
    const idxToWorld = (i) => (i - (N-1)/2)*this.cell; // cell index → centered world x/z
    // Place segments
    for (const [s,e] of north){
      const wlen=(e-s+1)*this.cell, cx=(idxToWorld(s)+idxToWorld(e))/2;
      mkSeg(wlen, 0, cx, +halfW - th*0.5);
    }
    for (const [s,e] of south){
      const wlen=(e-s+1)*this.cell, cx=(idxToWorld(s)+idxToWorld(e))/2;
      mkSeg(wlen, 0, cx, -halfW + th*0.5);
    }
    for (const [s,e] of east){
      const wlen=(e-s+1)*this.cell, cz=(idxToWorld(s)+idxToWorld(e))/2;
      mkSeg(wlen, Math.PI/2, +halfW - th*0.5, cz);
    }
    for (const [s,e] of west){
      const wlen=(e-s+1)*this.cell, cz=(idxToWorld(s)+idxToWorld(e))/2;
      mkSeg(wlen, Math.PI/2, -halfW + th*0.5, cz);
    }
    // Fill tiny visual gaps at the four corners with slim "posts"
    const post = (x,z)=> {
      const g = new THREE.BoxGeometry(th*1.2, H, th*1.2);
      const m = wallMat.clone();
      const mesh = new THREE.Mesh(g,m);
      mesh.castShadow = true; mesh.receiveShadow = true;
      mesh.position.set(x, H/2, z);
      this.solids.add(mesh);
    };
    post(+halfW - th*0.5, +halfW - th*0.5);
    post(+halfW - th*0.5, -halfW + th*0.5);
    post(-halfW + th*0.5, +halfW - th*0.5);
    post(-halfW + th*0.5, -halfW + th*0.5);

    // === Interior walls from GT (non-border OBST runs) ===
    // Horizontal runs (across x for each y)
    for (let y=1;y<N-1;y++){
      const rowRuns = runs(Array.from({length:N}, (_,x)=> gt[x][y]===1?1:0));
      for (const [s,e] of rowRuns){
        const wlen=(e-s+1)*this.cell, cx=( (s+e)/2 - (N-1)/2 )*this.cell, cz=(y - (N-1)/2)*this.cell;
        mkSeg(wlen, 0, cx, cz);
      }
    }
    // Vertical runs (down y for each x) — ensures vertical walls also show
    for (let x=1;x<N-1;x++){
      const colRuns = runs(Array.from({length:N}, (_,y)=> gt[x][y]===1?1:0));
      for (const [s,e] of colRuns){
        const wlen=(e-s+1)*this.cell, cz=( (s+e)/2 - (N-1)/2 )*this.cell, cx=(x - (N-1)/2)*this.cell;
        mkSeg(wlen, Math.PI/2, cx, cz);
      }
    }
    
    // roof
    const roofMat = new THREE.MeshStandardMaterial({
      color: 0xbfbfbf, roughness: 0.75, metalness: 0.0, side: THREE.DoubleSide
    });
    this._maybeApplyTexture(roofMat, 'roof_basecolor.jpg', 'roof_normal.jpg', 2.0);
    const roof = new THREE.Mesh(new THREE.PlaneGeometry(W, W), roofMat);
    roof.rotation.x = -Math.PI/2; roof.position.y = H + 0.05;
    roof.receiveShadow = true; roof.castShadow = false;
    this.solids.add(roof);
  }
}   // <-- end class ThreeScene

// helpers
function makeCrate(size, self){
  const g = new THREE.BoxGeometry(size,size,size);
  const m = new THREE.MeshStandardMaterial({ color: 0x6b4b2a, roughness: 0.9, metalness: 0.0 });
  self._maybeApplyTexture(m, 'crate_basecolor.jpg', 'crate_normal.jpg', 1.0);
  const mesh = new THREE.Mesh(g,m);
  mesh.castShadow = true; mesh.receiveShadow = true;
  return mesh;
}

function makePointCloud(maxPoints=2000){
  const g = new THREE.BufferGeometry();
  const pos = new Float32Array(maxPoints*3), alpha = new Float32Array(maxPoints);
  g.setAttribute("position", new THREE.BufferAttribute(pos,3));
  g.setAttribute("alpha", new THREE.BufferAttribute(alpha,1));
  const m = new THREE.PointsMaterial({ size: 0.12, color: 0x35c6ff, transparent: true, opacity: 1.0 });

  m.onBeforeCompile = (shader) => {
    shader.vertexShader = shader.vertexShader
      .replace(
        '#include <common>',
        `#include <common>
         attribute float alpha;
         varying float vAlpha;`
      )
      .replace(
        '#include <begin_vertex>',
        `#include <begin_vertex>
         vAlpha = alpha;`
      );

    shader.fragmentShader = shader.fragmentShader
      .replace(
        '#include <common>',
        `#include <common>
         varying float vAlpha;`
      )
      .replace(
        'gl_FragColor = vec4( outgoingLight, diffuseColor.a );',
        `gl_FragColor = vec4( outgoingLight, diffuseColor.a * vAlpha );`
      );
  };

  const pts = new THREE.Points(g, m);
  pts.frustumCulled = false;
  return pts;
}

function makeScanRing(){
  const g = new THREE.RingGeometry(0.9,1.05,64,1);
  const m = new THREE.MeshBasicMaterial({ color:0x35c6ff, transparent:true, opacity:0.0, side:THREE.DoubleSide });
  const ring = new THREE.Mesh(g, m);
  ring.rotation.x = -Math.PI/2;
  ring.visible = false;
  return ring;
}

function makeFireStack(cell=2.0){
  const grp = new THREE.Group();
  const tex = new THREE.TextureLoader().load('https://threejs.org/examples/textures/sprites/spark1.png');
  const COUNT = 16;
  for (let i=0;i<COUNT;i++){
    const sm = new THREE.SpriteMaterial({ map: tex, color: 0xff8a2b, transparent:true, depthWrite:false, blending:THREE.AdditiveBlending });
    const sp = new THREE.Sprite(sm);
    sp.position.set((Math.random()-0.5)*0.4, 0.1+Math.random()*0.2, (Math.random()-0.5)*0.4);
    const s = 0.5 + Math.random()*0.7;
    sp.scale.set(s, s*1.6, s);
    const base = Math.random()*1000;
    sp.onBeforeRender = function(){
      const t = performance.now()*0.002 + base;
      const y = (t % 1.0);
      this.position.y = 0.1 + y*1.2;
      this.material.opacity = 0.6 + Math.sin(t*5.0)*0.25;
      this.material.color.setHSL(0.06 + 0.02*Math.sin(t*3.0), 1.0, 0.55);
    };
    grp.add(sp);
  }
  const glow = new THREE.Mesh(
    new THREE.CircleGeometry(0.8*cell*0.2, 24),
    new THREE.MeshBasicMaterial({ color:0xff6a00, transparent:true, opacity:0.6 })
  );
  glow.rotation.x = -Math.PI/2; glow.position.y = 0.01;
  grp.add(glow);

  const pl = new THREE.PointLight(0xff7a00, 2.0, cell*6);
  pl.position.set(0, 1.0, 0);
  const seed = Math.random()*1000;
  pl.onBeforeRender = function(){
    const t = (performance.now()+seed)*0.004;
    this.intensity = 1.5 + Math.sin(t*3.0)*0.6 + Math.sin(t*7.1)*0.3;
  };
  grp.add(pl);

  return grp;
}
