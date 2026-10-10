/* ==========================================================================
   NÚCLEO DEL SIMULADOR (física, sensores, fallas, reglas y ejecución del código)
   Esta función se serializa y corre dentro de un Web Worker, por eso NO puede
   usar nada de afuera: todo lo que necesita está aquí adentro.
   Unidades: mm, s, kg, N, rad (los parámetros de la interfaz usan °, %, V).
   ========================================================================== */
function SimCore() {
  'use strict';
  const DEG = Math.PI / 180, TAU = Math.PI * 2;
  const clamp = (v, a, b) => (v < a ? a : v > b ? b : v);

  /* ---------- Números aleatorios reproducibles (misma semilla = misma corrida) ---------- */
  function RNG(seed) {
    let a = (seed >>> 0) || 1;
    const f = () => { a = (a + 0x6D2B79F5) | 0; let t = Math.imul(a ^ (a >>> 15), 1 | a); t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t; return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
    f.g = () => { let u = 0; while (!u) u = f(); return Math.sqrt(-2 * Math.log(u)) * Math.cos(TAU * f()); };
    f.int = n => Math.floor(f() * n);
    return f;
  }

  /* ---------- Parámetros por defecto ---------- */
  // Motores N20 6 V. rpm y torque de atasco son valores típicos de tienda a 6 V.
  const GEARS = {
    30: { rpm6: 1000, stall6: 0.029 }, 50: { rpm6: 600, stall6: 0.049 },
    100: { rpm6: 300, stall6: 0.098 }, 150: { rpm6: 200, stall6: 0.14 }
  };
  const ROBOT_DEFAULT = {
    massKg: 0.17, gear: 100, wheelD: 34, track: 90, encPPR: 7,
    motorR: 6.0,            // ohm (corriente de atasco ~1 A a 6 V)
    batteryV: 7.8, capacity: 350, rint: 0.25,  // LiPo 2S
    regV: 6.0, regDropout: 2.0,                // LM7806
    l293: true,             // caída de voltaje del L293D
    logicA: 0.13, wheelLoad: 0.85, skidMu: 0.25, wallMu: 0.35
  };
  const FAIL_PRESETS = {
    ideal: { wallOffset: 0, wallAngle: 0, gapProb: 0, gapMax: 0, wallThickVar: 0, reflVar: 0, wallHold: 40,
      mu0: 0.9, dirtRate: 0, muMin: 0.9, wheelDiamVar: 0, motorMismatch: 0, deadband: 0,
      irNoise: 1, ambient: 20, ambFlicker: 0, irGainVar: 0, irSpike: 0, deadSensor: -1,
      encMiss: 0, gyroDrift: 0, gyroScale: 0, startPos: 0, startAng: 0, batteryV: 8.2 },
    realista: { wallOffset: 1.5, wallAngle: 0.5, gapProb: 0.15, gapMax: 2, wallThickVar: 0.5, reflVar: 0.15, wallHold: 3,
      mu0: 0.85, dirtRate: 0.05, muMin: 0.5, wheelDiamVar: 0.8, motorMismatch: 5, deadband: 4,
      irNoise: 5, ambient: 40, ambFlicker: 10, irGainVar: 12, irSpike: 0.002, deadSensor: -1,
      encMiss: 0.3, gyroDrift: 0.02, gyroScale: 1.5, startPos: 3, startAng: 2, batteryV: 7.8 },
    pesimista: { wallOffset: 3, wallAngle: 1.2, gapProb: 0.35, gapMax: 4, wallThickVar: 1, reflVar: 0.3, wallHold: 2,
      mu0: 0.7, dirtRate: 0.12, muMin: 0.35, wheelDiamVar: 1.5, motorMismatch: 10, deadband: 8,
      irNoise: 10, ambient: 90, ambFlicker: 30, irGainVar: 20, irSpike: 0.01, deadSensor: -1,
      encMiss: 1, gyroDrift: 0.05, gyroScale: 3, startPos: 6, startAng: 4, batteryV: 7.5 },
    peor: { wallOffset: 4, wallAngle: 2, gapProb: 0.5, gapMax: 6, wallThickVar: 1.5, reflVar: 0.4, wallHold: 1.5,
      mu0: 0.6, dirtRate: 0.2, muMin: 0.3, wheelDiamVar: 2.5, motorMismatch: 15, deadband: 12,
      irNoise: 15, ambient: 150, ambFlicker: 50, irGainVar: 30, irSpike: 0.03, deadSensor: 1,
      encMiss: 2, gyroDrift: 0.1, gyroScale: 4, startPos: 8, startAng: 6, batteryV: 7.3 }
  };
  const RULES = { stuckS: 30, stuckMM: 30, maxTime: 300 };

  /* ---------- Geometría del robot (sacada de la PCB final) ----------
     Marco del robot: f = hacia adelante, l = hacia la izquierda, origen en el eje de llantas.
     PCB de 100 x 100 mm, esquinas traseras R20, frente en arco (centro f=-4.13, r=53.95). */
  function robotOutline() {
    const pts = [];
    const arc = (cf, cl, r, a0, a1, n) => { for (let i = 0; i <= n; i++) { const a = a0 + (a1 - a0) * i / n; pts.push([cf + r * Math.cos(a), cl + r * Math.sin(a)]); } };
    const phi = Math.asin(50 / 53.95);
    arc(-4.13, 0, 53.95, phi, -phi, 34);                 // frente (de izquierda a derecha)
    const fSide = -4.13 + 53.95 * Math.cos(phi);
    for (let f = fSide - 6; f > -30; f -= 6) pts.push([f, -50]);  // lado derecho
    arc(-30, -30, 20, -Math.PI / 2, -Math.PI, 6);        // esquina trasera derecha
    for (let l = -24; l < 30; l += 6) pts.push([-50, l]); // parte trasera
    arc(-30, 30, 20, Math.PI, Math.PI / 2, 6);           // esquina trasera izquierda
    for (let f = -24; f < fSide; f += 6) pts.push([f, 50]); // lado izquierdo
    return pts;
  }
  const OUTLINE = robotOutline();
  const R_BOUND = Math.max(...OUTLINE.map(p => Math.hypot(p[0], p[1])));
  // Sensores IR: receptor (fototransistor) de cada par, posición y dirección según la PCB.
  const SENSORS = [
    { key: 'left',   f: 20.1, l: 36.9,  a: 90 },
    { key: 'dleft',  f: 30.8, l: 24.7,  a: 31.6 },
    { key: 'front',  f: 38.1, l: 3.9,   a: 0 },
    { key: 'dright', f: 33.1, l: -21.5, a: -42.8 },
    { key: 'right',  f: 20.2, l: -36.7, a: -90 }
  ];
  const IR_A = 1.034e6, IR_D0 = 10;   // calibrado: ~520 a 34 mm (robot centrado en el pasillo)

  /* ---------- Laberintos ---------- */
  function parseMaze(m) {
    const h = m.h.map(r => Array.from(r, c => c === '1' || c === 1 ? 1 : 0));
    const v = m.v.map(r => Array.from(r, c => c === '1' || c === 1 ? 1 : 0));
    return { name: m.name || 'Pista', cols: m.cols, rows: m.rows, cell: m.cell || 155, wall: m.wall || 12, height: m.height || 50,
      h, v, start: Object.assign({}, m.start), goal: m.goal.map(g => [g[0], g[1]]) };
  }
  function serializeMaze(mz) {
    return { name: mz.name, cols: mz.cols, rows: mz.rows, cell: mz.cell, wall: mz.wall, height: mz.height,
      h: mz.h.map(r => r.join('')), v: mz.v.map(r => r.join('')), start: mz.start, goal: mz.goal };
  }
  function wallBetween(mz, x, y, d) { // ¿hay pared en el lado d de la celda (x,y)? d: 0N 1E 2S 3O
    if (d === 0) return mz.h[y + 1][x]; if (d === 2) return mz.h[y][x];
    if (d === 1) return mz.v[y][x + 1]; return mz.v[y][x];
  }
  function setWallBetween(mz, x, y, d, val) {
    if (d === 0) mz.h[y + 1][x] = val; else if (d === 2) mz.h[y][x] = val;
    else if (d === 1) mz.v[y][x + 1] = val; else mz.v[y][x] = val;
  }
  const DX = [0, 1, 0, -1], DY = [1, 0, -1, 0];
  function reachDist(mz, sx, sy) {
    const dist = new Int16Array(mz.cols * mz.rows).fill(-1), q = [[sx, sy]]; dist[sy * mz.cols + sx] = 0;
    while (q.length) {
      const [x, y] = q.shift();
      for (let d = 0; d < 4; d++) {
        if (wallBetween(mz, x, y, d)) continue;
        const nx = x + DX[d], ny = y + DY[d];
        if (nx < 0 || ny < 0 || nx >= mz.cols || ny >= mz.rows) continue;
        const i = ny * mz.cols + nx; if (dist[i] >= 0) continue;
        dist[i] = dist[y * mz.cols + x] + 1; q.push([nx, ny]);
      }
    }
    return dist;
  }
  function shortestToGoal(mz) {
    const d = reachDist(mz, mz.start.x, mz.start.y);
    let best = -1; for (const [gx, gy] of mz.goal) { const v = d[gy * mz.cols + gx]; if (v >= 0 && (best < 0 || v < best)) best = v; }
    return best;
  }
  function isGoal(mz, x, y) { return mz.goal.some(g => g[0] === x && g[1] === y); }
  function emptyMaze(cols, rows, base) {
    const mz = { name: 'Vacía', cols, rows, cell: base ? base.cell : 155, wall: base ? base.wall : 12, height: 50,
      h: [], v: [], start: base ? Object.assign({}, base.start) : { x: cols - 1, y: 0, dir: 0 },
      goal: base ? base.goal.map(g => [g[0], g[1]]) : [[0, rows - 1]] };
    for (let y = 0; y <= rows; y++) mz.h.push(Array.from({ length: cols }, () => (y === 0 || y === rows ? 1 : 0)));
    for (let y = 0; y < rows; y++) mz.v.push(Array.from({ length: cols + 1 }, (_, x) => (x === 0 || x === cols ? 1 : 0)));
    return mz;
  }
  // Laberinto aleatorio (DFS + algunos ciclos), conservando inicio, meta y el cuarto de meta cerrado con una sola entrada.
  function randomMaze(base, seed, loops) {
    const rng = RNG(seed), mz = emptyMaze(base.cols, base.rows, base);
    mz.name = 'Aleatoria #' + seed;
    for (let y = 0; y <= mz.rows; y++) for (let x = 0; x < mz.cols; x++) mz.h[y][x] = 1;
    for (let y = 0; y < mz.rows; y++) for (let x = 0; x <= mz.cols; x++) mz.v[y][x] = 1;
    const seen = new Uint8Array(mz.cols * mz.rows), st = [[mz.start.x, mz.start.y]]; seen[mz.start.y * mz.cols + mz.start.x] = 1;
    while (st.length) {
      const [x, y] = st[st.length - 1], opts = [];
      for (let d = 0; d < 4; d++) { const nx = x + DX[d], ny = y + DY[d]; if (nx >= 0 && ny >= 0 && nx < mz.cols && ny < mz.rows && !seen[ny * mz.cols + nx]) opts.push(d); }
      if (!opts.length) { st.pop(); continue; }
      const d = opts[rng.int(opts.length)], nx = x + DX[d], ny = y + DY[d];
      setWallBetween(mz, x, y, d, 0); seen[ny * mz.cols + nx] = 1; st.push([nx, ny]);
    }
    const n = loops == null ? Math.round(mz.cols * mz.rows * 0.08) : loops;
    for (let i = 0; i < n; i++) {
      const x = rng.int(mz.cols), y = rng.int(mz.rows), d = rng.int(4), nx = x + DX[d], ny = y + DY[d];
      if (nx < 1 || ny < 0 || nx >= mz.cols - 1 || ny >= mz.rows) continue;
      setWallBetween(mz, x, y, d, 0);
    }
    return mz;
  }
  // Cambia k paredes interiores al azar manteniendo la meta alcanzable (la pista "real" puede no ser igual al dibujo).
  function mutateMaze(base, seed, k) {
    const rng = RNG(seed * 7919 + 13), mz = parseMaze(serializeMaze(base));
    mz.name = base.name + ' (+' + k + ' cambios)';
    let done = 0, tries = 0;
    while (done < k && tries++ < k * 60) {
      const horiz = rng() < 0.5;
      let x, y;
      if (horiz) { x = rng.int(mz.cols); y = 1 + rng.int(mz.rows - 1); } else { x = 1 + rng.int(mz.cols - 1); y = rng.int(mz.rows); }
      const arr = horiz ? mz.h : mz.v, old = arr[y][x];
      // no tocar las paredes del cuarto de meta ni de la celda de inicio
      const cellsTouch = horiz ? [[x, y - 1], [x, y]] : [[x - 1, y], [x, y]];
      if (cellsTouch.some(c => isGoal(mz, c[0], c[1]) || (c[0] === mz.start.x && c[1] === mz.start.y))) continue;
      arr[y][x] = old ? 0 : 1;
      if (shortestToGoal(mz) < 0) { arr[y][x] = old; continue; }
      done++;
    }
    return mz;
  }

  /* ---------- Mundo físico: paredes con defectos de impresión 3D ---------- */
  function buildWorld(mz, F, rng) {
    const c = mz.cell, W = mz.wall, walls = [];
    function add(x1, y1, x2, y2, border, kind, gx, gy) {
      const len = Math.hypot(x2 - x1, y2 - y1);
      let a = Math.atan2(y2 - y1, x2 - x1) + rng.g() * F.wallAngle * DEG * (border ? 0.3 : 1);
      const ux = Math.cos(a), uy = Math.sin(a), nx = -uy, ny = ux;
      let cx = (x1 + x2) / 2, cy = (y1 + y2) / 2;
      const off = rng.g() * F.wallOffset * (border ? 0.5 : 1); cx += nx * off; cy += ny * off;
      let s0 = 0, s1 = 0;
      if (rng() < F.gapProb) s0 = W / 2 + rng() * F.gapMax;
      if (rng() < F.gapProb) s1 = W / 2 + rng() * F.gapMax;
      const L = len + W - s0 - s1; cx += ux * (s0 - s1) / 2; cy += uy * (s0 - s1) / 2;
      const th = Math.max(4, W + rng.g() * F.wallThickVar);
      walls.push({ id: walls.length, cx, cy, ux, uy, nx, ny, hl: L / 2, ht: th / 2,
        refl: clamp(1 + rng.g() * F.reflVar, 0.4, 1.6), hold: border ? F.wallHold * 5 : F.wallHold * (0.7 + 0.6 * rng()),
        border, kind, gx, gy, over: 0, knocked: false, load: 0 });
    }
    for (let y = 0; y <= mz.rows; y++) for (let x = 0; x < mz.cols; x++) if (mz.h[y][x]) add(x * c, y * c, (x + 1) * c, y * c, y === 0 || y === mz.rows, 'h', x, y);
    for (let y = 0; y < mz.rows; y++) for (let x = 0; x <= mz.cols; x++) if (mz.v[y][x]) add(x * c, y * c, x * c, (y + 1) * c, x === 0 || x === mz.cols, 'v', x, y);
    // índice espacial por celda
    const buckets = Array.from({ length: (mz.cols + 2) * (mz.rows + 2) }, () => []);
    const bi = (cx, cy) => (cy + 1) * (mz.cols + 2) + (cx + 1);
    for (const w of walls) {
      const ex = Math.abs(w.ux) * w.hl + Math.abs(w.nx) * w.ht, ey = Math.abs(w.uy) * w.hl + Math.abs(w.ny) * w.ht;
      const x0 = Math.floor((w.cx - ex) / c), x1 = Math.floor((w.cx + ex) / c), y0 = Math.floor((w.cy - ey) / c), y1 = Math.floor((w.cy + ey) / c);
      for (let yy = Math.max(-1, y0); yy <= Math.min(mz.rows, y1); yy++) for (let xx = Math.max(-1, x0); xx <= Math.min(mz.cols, x1); xx++) buckets[bi(xx, yy)].push(w);
    }
    let stamp = 1; const marks = new Uint32Array(walls.length);
    function query(x, y, R) {
      stamp++; const out = [];
      const x0 = Math.floor((x - R) / c), x1 = Math.floor((x + R) / c), y0 = Math.floor((y - R) / c), y1 = Math.floor((y + R) / c);
      for (let yy = Math.max(-1, y0); yy <= Math.min(mz.rows, y1); yy++) for (let xx = Math.max(-1, x0); xx <= Math.min(mz.cols, x1); xx++)
        for (const w of buckets[bi(xx, yy)]) if (marks[w.id] !== stamp) { marks[w.id] = stamp; if (!w.knocked) out.push(w); }
      return out;
    }
    return { walls, query, maze: mz };
  }
  // Rayo contra un rectángulo orientado. Devuelve {t, cos} o null.
  function rayRect(ox, oy, dx, dy, w) {
    const rx = ox - w.cx, ry = oy - w.cy;
    const ou = rx * w.ux + ry * w.uy, ow = rx * w.nx + ry * w.ny;
    const du = dx * w.ux + dy * w.uy, dw = dx * w.nx + dy * w.ny;
    let tmin = -Infinity, tmax = Infinity, axis = -1;
    if (Math.abs(du) < 1e-9) { if (Math.abs(ou) > w.hl) return null; }
    else { let t1 = (-w.hl - ou) / du, t2 = (w.hl - ou) / du; if (t1 > t2) { const q = t1; t1 = t2; t2 = q; } if (t1 > tmin) { tmin = t1; axis = 0; } tmax = Math.min(tmax, t2); }
    if (Math.abs(dw) < 1e-9) { if (Math.abs(ow) > w.ht) return null; }
    else { let t1 = (-w.ht - ow) / dw, t2 = (w.ht - ow) / dw; if (t1 > t2) { const q = t1; t1 = t2; t2 = q; } if (t1 > tmin) { tmin = t1; axis = 1; } tmax = Math.min(tmax, t2); }
    if (tmax < tmin || tmin <= 0) return null;
    return { t: tmin, cos: axis === 1 ? Math.abs(dw) : Math.abs(du) };
  }
  function pointInPoly(pts, f, l) {
    let inside = false;
    for (let i = 0, j = pts.length - 1; i < pts.length; j = i++) {
      const fi = pts[i][0], li = pts[i][1], fj = pts[j][0], lj = pts[j][1];
      if ((li > l) !== (lj > l) && f < (fj - fi) * (l - li) / (lj - li) + fi) inside = !inside;
    }
    return inside;
  }

  /* ==========================================================================
     SIMULACIÓN
     ========================================================================== */
  function createSim(opts) {
    const mz = parseMaze(opts.maze), ref = parseMaze(opts.reference || opts.maze);
    const F = Object.assign({}, FAIL_PRESETS.realista, opts.fail || {});
    const RB = Object.assign({}, ROBOT_DEFAULT, opts.robot || {});
    const gear = GEARS[RB.gear] || GEARS[30];
    const seed = opts.seed | 0;
    const rngWorld = RNG(seed * 2654435761 + 1), rngRun = RNG(seed * 40503 + 7), rngSens = RNG(seed * 69069 + 3);
    const world = buildWorld(mz, F, rngWorld);
    const c = mz.cell;

    // --- parámetros del robot con variaciones de fabricación ---
    const w0 = gear.rpm6 * TAU / 60;               // rad/s de la llanta sin carga a 6 V
    const rWheel = RB.wheelD / 2;
    const wheels = [0, 1].map(k => {
      const mm = 1 + rngWorld.g() * F.motorMismatch / 100;
      const stall = gear.stall6 * mm;
      return {
        l: k === 0 ? RB.track / 2 : -RB.track / 2,
        r: rWheel * (1 + rngWorld.g() * F.wheelDiamVar / 100),
        Ke: 6 / (w0 * (1 + rngWorld.g() * F.motorMismatch / 200)),
        Kt: stall * RB.motorR / 6,
        J: 0.03 * stall / w0 + 2e-7,
        tf: stall * (0.05 + F.deadband / 100), ts: stall * (0.08 + F.deadband / 100),
        mu0: F.mu0 * (1 + rngWorld.g() * 0.05), mu: F.mu0, dist: 0,
        w: 0, I: 0, pwm: 0, acc: 0, enc: 0, ang: 0
      };
    });
    const CPR = RB.encPPR * 2 * RB.gear;            // cuentas por vuelta (interrupción CHANGE en canal A)
    const mass = RB.massKg, Iz = mass * (0.1 * 0.1 + 0.1 * 0.1) / 12;
    const g = 9.81, Nw = mass * g * RB.wheelLoad / 2, Nsk = mass * g * (1 - RB.wheelLoad) / 2;
    const irGain = SENSORS.map(() => clamp(1 + rngWorld.g() * F.irGainVar / 100, 0.4, 1.8));
    const irAmb = SENSORS.map(() => Math.max(0, F.ambient * (1 + rngWorld.g() * 0.3)));
    const irPhase = SENSORS.map(() => rngWorld() * TAU);
    const dead = F.deadSensor;                     // -1 ninguno, 0..4 sensor muerto
    const gyroScale = 1 + rngWorld.g() * F.gyroScale / 100;
    let gyroBias = rngWorld.g() * 1.5;             // °/s (se calibra en el arranque)
    // acelerómetro del MPU-6050 (±2 g, 16384 LSB/g, DLPF ~44 Hz)
    const accBias = [rngWorld.g() * 0.04, rngWorld.g() * 0.04];   // g (se calibra en el arranque)
    const accF = [0, 0];                           // salida del filtro pasa-bajas
    const DLPF_A = 0.001 / (0.001 + 1 / (TAU * 44));

    // --- estado ---
    const st = { t: 0, x: 0, y: 0, th: 0, vx: 0, vy: 0, om: 0 };
    const s = mz.start;
    st.x = (s.x + 0.5) * c + rngRun.g() * F.startPos;
    st.y = (s.y + 0.5) * c + rngRun.g() * F.startPos;
    st.th = (90 - s.dir * 90) * DEG + rngRun.g() * F.startAng * DEG;
    const x0 = st.x, y0 = st.y;
    let charge = 0, Vbat = F.batteryV, Vmot = 6, brownout = false;
    let collisions = 0, inContact = false, odo = 0;
    const res = { status: 'ready', reason: '', runs: [], firstArrival: null, best: null, tStart: null, legStart: null, inGoal: false, ended: false };
    let stuckX = st.x, stuckY = st.y, stuckT = 0;
    const logs = []; let logSeq = 0;
    let overlay = null, overlaySeq = 0, beepT = -1;
    let lastIR = null, lastRays = null;
    const trail = []; let trailT = -1;

    function log(msg, kind) {
      const line = { t: st.t, msg: String(msg), kind: kind || 'user' };
      logs.push(line); logSeq++; if (logs.length > 3000) logs.splice(0, 1000);
    }

    /* ---------- Sensores ---------- */
    let irNear = null, irNX = 1e9, irNY = 1e9;
    function readIR() {
      const cs = Math.cos(st.th), sn = Math.sin(st.th);
      if (Math.hypot(st.x - irNX, st.y - irNY) > 40) { irNear = world.query(st.x, st.y, 380); irNX = st.x; irNY = st.y; }
      const near = irNear;
      const out = {}, rays = [];
      for (let i = 0; i < SENSORS.length; i++) {
        const S = SENSORS[i];
        const ox = st.x + S.f * cs - S.l * sn, oy = st.y + S.f * sn + S.l * cs;
        let sig = 0, wsum = 0, hitD = 300;
        for (let k = -1; k <= 1; k++) {
          const a = st.th + (S.a + k * 5) * DEG, dx = Math.cos(a), dy = Math.sin(a), wk = k ? 0.6 : 1;
          let best = null, bw = null;
          for (const w of near) {
            const qx = w.cx - ox, qy = w.cy - oy, ext = w.hl + w.ht;
            const along = qx * dx + qy * dy;
            if (along < -ext || along > 300 + ext) continue;             // detrás del rayo o muy lejos
            if (Math.abs(qx * dy - qy * dx) > ext) continue;              // el rayo pasa lejos de la pared
            const h = rayRect(ox, oy, dx, dy, w); if (h && h.t < 300 && (!best || h.t < best.t)) { best = h; bw = w; }
          }
          wsum += wk;
          if (best) { sig += wk * bw.refl * Math.max(best.cos, 0.15) * IR_A / ((best.t + IR_D0) * (best.t + IR_D0)); if (k === 0) hitD = best.t; }
        }
        sig /= wsum;
        let v = irAmb[i] + F.ambFlicker * Math.sin(st.t * TAU * 0.7 + irPhase[i]) + irGain[i] * sig;
        v = 1010 * Math.tanh(Math.max(0, v) / 1010);               // saturación del fototransistor
        v += rngSens.g() * F.irNoise;
        if (rngSens() < F.irSpike) v = rngSens() * 1023;           // lectura basura
        if (i === dead) v = 3 + rngSens() * 4;                     // sensor muerto / desoldado
        v = clamp(Math.round(v), 0, 1023);
        out[S.key] = v;
        rays.push([ox, oy, ox + Math.cos(st.th + S.a * DEG) * hitD, oy + Math.sin(st.th + S.a * DEG) * hitD, v]);
      }
      lastIR = out; lastRays = rays;
      return Object.assign({}, out);
    }
    function readAccel() {
      const q = v => clamp(Math.round(v * 16384), -32768, 32767) / 16384;
      return { x: q(accF[0] + accBias[0] + rngSens.g() * 0.008), y: q(accF[1] + accBias[1] + rngSens.g() * 0.008) };
    }
    function readGyro() {
      const dps = st.om / DEG * gyroScale + gyroBias + rngSens.g() * 0.05;
      return Math.round(dps * 65.5) / 65.5;
    }

    /* ---------- API que ve el programa del robot ---------- */
    const cfg = {
      cols: mz.cols, rows: mz.rows, start: Object.assign({}, mz.start), goal: mz.goal.map(g => [g[0], g[1]]),
      cellMM: c, wheelD: RB.wheelD, track: RB.track, gear: RB.gear, countsPerRev: CPR,
      sensors: SENSORS.map(S => Object.assign({}, S)),
      reference: serializeMaze(ref)
    };
    const api = {
      cfg,
      ir: () => readIR(),
      motors: (l, r) => {
        wheels[0].pwm = clamp(Math.round(+l || 0), -255, 255);
        wheels[1].pwm = clamp(Math.round(+r || 0), -255, 255);
      },
      enc: () => ({ l: wheels[0].enc, r: wheels[1].enc }),
      gyro: () => readGyro(),
      accel: () => readAccel(),
      battery: () => Math.round(Vbat * 120 / 320 / 5 * 1023) * 5 / 1023 * 320 / 120,
      millis: () => Math.floor(st.t * 1000),
      micros: () => Math.floor(st.t * 1e6),
      log: (...a) => log(a.map(v => (typeof v === 'object' ? JSON.stringify(v) : String(v))).join(' ')),
      beep: () => { beepT = st.t; },
      show: o => { overlay = o; overlaySeq++; },
      finish: () => { res.ended = true; }
    };

    /* ---------- Programa del usuario ---------- */
    let gen = null, wake = 0, progDone = false;
    try {
      const factory = new Function('r', '"use strict";\n' + opts.code + '\n;if (typeof main !== "function") throw new Error("Falta la función main(r)."); return main;');
      const mainFn = factory(api);
      if (Object.getPrototypeOf(mainFn) !== Object.getPrototypeOf(function* () {})) throw new Error('main debe ser function* main(r) { ... } (generador, con asterisco).');
      gen = mainFn(api);
    } catch (e) { endRun('error', 'Error en el código: ' + e.message); }

    function endRun(status, reason) {
      if (res.status === 'dq' || res.status === 'error' || res.status === 'done') return;
      res.status = status; res.reason = reason; log(reason, status === 'done' ? 'ok' : 'err');
      wheels[0].pwm = wheels[1].pwm = 0;
    }

    /* ---------- Un paso de física ---------- */
    const DT = 0.001;
    let phNear = [], phNX = 1e9, phNY = 1e9;
    function physics(dt) {
      const cs = Math.cos(st.th), sn = Math.sin(st.th);
      const u = st.vx * cs + st.vy * sn, sl = -st.vx * sn + st.vy * cs;
      // electrónica: batería LiPo -> LM7806 -> L293D -> motor
      const Itot = Math.abs(wheels[0].I) + Math.abs(wheels[1].I) + RB.logicA;
      const soc = clamp(1 - charge / RB.capacity, 0, 1);
      const Voc = F.batteryV - (1 - soc) * 0.8;
      Vbat = Voc - RB.rint * Itot;
      Vmot = clamp(Math.min(RB.regV, Vbat - RB.regDropout), 0, RB.regV);
      if (Vbat < 6.2 && !brownout) { brownout = true; }
      charge += Itot * dt / 3.6;
      let Ff = 0, Fl = 0, tau = 0;
      for (const W of wheels) {
        const duty = W.pwm / 255;
        const drop = RB.l293 ? 1.2 + 1.3 * Math.min(Math.abs(W.I), 1.2) : 0;
        const Vavail = Math.max(0, Vmot - drop);
        W.I = clamp((duty * Vavail - W.Ke * W.w) / RB.motorR, -1.2, 1.2);
        let tm = W.Kt * W.I;
        // fricción interna de la caja reductora (zona muerta del PWM)
        if (Math.abs(W.w) < 0.3) { if (Math.abs(tm) < W.ts) { tm = 0; } else tm -= Math.sign(tm) * W.tf; }
        else tm -= Math.sign(W.w) * W.tf;
        // llanta contra el piso (se ensucia con la distancia recorrida)
        W.mu = Math.max(F.muMin, W.mu0 - F.dirtRate * W.dist / 1000);
        const vc = u - st.om * W.l;
        const slip = W.w * W.r - vc;
        let fl = clamp(0.008 * slip, -10, 10), fa = clamp(-0.03 * sl, -10, 10);
        const fmax = W.mu * Nw, fm = Math.hypot(fl, fa);
        if (fm > fmax) { fl *= fmax / fm; fa *= fmax / fm; }
        W.w += (tm - fl * W.r / 1000) / W.J * dt;
        if (Math.abs(W.w) < 0.3 && W.pwm === 0 && Math.abs(tm) < 1e-9) W.w *= 0.9;
        W.dist += Math.abs(W.w * W.r * dt);
        // encoder con pulsos perdidos
        W.acc += W.w * dt * CPR / TAU;
        while (W.acc >= 1) { W.acc -= 1; if (rngRun() * 100 >= F.encMiss) W.enc++; }
        while (W.acc <= -1) { W.acc += 1; if (rngRun() * 100 >= F.encMiss) W.enc--; }
        Ff += fl; Fl += fa; tau += -W.l * fl;
      }
      // patines (deslizadores adelante y atrás)
      for (const fs of [38, -42]) {
        const pf = u, pl = sl + st.om * fs, sp = Math.hypot(pf, pl);
        if (sp > 1e-6) { const k = RB.skidMu * Nsk * Math.tanh(sp / 5) / sp; Ff -= k * pf; Fl -= k * pl; tau += fs * (-k * pl); }
      }
      let Fx = Ff * cs - Fl * sn, Fy = Ff * sn + Fl * cs;
      // contactos con paredes
      let contact = false;
      if (Math.hypot(st.x - phNX, st.y - phNY) > 25) { phNear = world.query(st.x, st.y, R_BOUND + 60); phNX = st.x; phNY = st.y; }
      for (const w of phNear) {
        if (w.knocked) continue;
        const rx = st.x - w.cx, ry = st.y - w.cy;
        const cu = rx * w.ux + ry * w.uy, cw = rx * w.nx + ry * w.ny;
        if (Math.abs(cw) > w.ht + R_BOUND || Math.abs(cu) > w.hl + R_BOUND) continue;
        let load = 0;
        const apply = (px, py, nxw, nyw, depth) => {
          depth = Math.min(depth, 6);
          const ax = px - st.x, ay = py - st.y;
          const vpx = st.vx - st.om * ay, vpy = st.vy + st.om * ax;
          const vn = vpx * nxw + vpy * nyw;
          let Fn = 3.0 * depth - 0.015 * vn; if (Fn <= 0) return;
          const tx = -nyw, ty = nxw, vt = vpx * tx + vpy * ty;
          const Ft = -RB.wallMu * Fn * Math.tanh(vt / 3);
          const fx = Fn * nxw + Ft * tx, fy = Fn * nyw + Ft * ty;
          Fx += fx; Fy += fy; tau += ax * fy - ay * fx; load += Fn; contact = true;
        };
        for (const p of OUTLINE) {
          const px = st.x + p[0] * cs - p[1] * sn, py = st.y + p[0] * sn + p[1] * cs;
          const qx = px - w.cx, qy = py - w.cy, pu = qx * w.ux + qy * w.uy, pw = qx * w.nx + qy * w.ny;
          if (Math.abs(pu) >= w.hl || Math.abs(pw) >= w.ht) continue;
          const dW = w.ht - Math.abs(pw), dU = w.hl - Math.abs(pu);
          if (dW <= dU) apply(px, py, w.nx * Math.sign(pw || 1), w.ny * Math.sign(pw || 1), dW);
          else apply(px, py, w.ux * Math.sign(pu || 1), w.uy * Math.sign(pu || 1), dU);
        }
        // esquinas de la pared que se meten en el robot
        for (const su of [-1, 1]) for (const sw of [-1, 1]) {
          const kx = w.cx + w.ux * w.hl * su + w.nx * w.ht * sw, ky = w.cy + w.uy * w.hl * su + w.ny * w.ht * sw;
          const dx = kx - st.x, dy = ky - st.y, f = dx * cs + dy * sn, l = -dx * sn + dy * cs;
          if (f * f + l * l > R_BOUND * R_BOUND || !pointInPoly(OUTLINE, f, l)) continue;
          let bd = 1e9, bnf = 0, bnl = 0;
          for (let i = 0, j = OUTLINE.length - 1; i < OUTLINE.length; j = i++) {
            const af = OUTLINE[j][0], al = OUTLINE[j][1], ef = OUTLINE[i][0] - af, el = OUTLINE[i][1] - al, L2 = ef * ef + el * el;
            const tt = clamp(((f - af) * ef + (l - al) * el) / L2, 0, 1), qf = af + ef * tt - f, ql = al + el * tt - l, d = Math.hypot(qf, ql);
            if (d < bd) { bd = d; const len = Math.sqrt(L2); bnf = el / len; bnl = -ef / len; }
          }
          // el contorno va en sentido horario: (el,-ef) apunta hacia ADENTRO del robot; empujamos el robot en esa dirección
          const nwx = bnf * cs - bnl * sn, nwy = bnf * sn + bnl * cs;
          apply(kx, ky, nwx, nwy, bd);
        }
        if (load > 0) {
          w.load = load;
          if (load > w.hold) w.over += dt; else w.over = Math.max(0, w.over - dt * 0.5);
          if (w.over > 0.08 && !w.knocked) { w.knocked = true; endRun('dq', 'Descalificado: derribó una pared (empujó con ' + load.toFixed(1) + ' N)'); }
        }
      }
      // cuenta como choque sólo si llega con algo de velocidad (recargarse suave en una pared no cuenta)
      if (contact && !inContact && Math.hypot(st.vx, st.vy) > 60) { collisions++; log('Choque con pared #' + collisions + ' a ' + Math.hypot(st.vx, st.vy).toFixed(0) + ' mm/s', 'warn'); }
      inContact = contact;
      // integrar cuerpo
      st.vx += Fx / mass * 1000 * dt; st.vy += Fy / mass * 1000 * dt;
      // acelerómetro: aceleración en el marco del robot (g) + vibración de motores y patines
      {
        const af = (Fx * cs + Fy * sn) / mass / g, al = (-Fx * sn + Fy * cs) / mass / g;
        const sp = Math.abs(u), vib = 0.02 * sp / 300;
        accF[0] += DLPF_A * (af + rngRun.g() * vib - accF[0]);
        accF[1] += DLPF_A * (al + rngRun.g() * vib - accF[1]);
      }
      st.om += (tau / 1000) / Iz * dt;
      const ox = st.x, oy = st.y;
      st.x += st.vx * dt; st.y += st.vy * dt; st.th += st.om * dt;
      odo += Math.hypot(st.x - ox, st.y - oy);
      gyroBias += rngRun.g() * F.gyroDrift * Math.sqrt(dt);
      st.t += dt;
    }

    /* ---------- Reglas de la competencia ---------- */
    function rules() {
      const cx = Math.floor(st.x / c), cy = Math.floor(st.y / c);
      // el cronómetro arranca cuando el robot sale de la celda de inicio (la calibración dentro de ella no cuenta)
      if (res.tStart == null && (cx !== mz.start.x || cy !== mz.start.y)) { res.tStart = st.t; res.legStart = st.t; res.status = 'running'; log('Salió del inicio: corre el cronómetro', 'sys'); }
      const inGoal = isGoal(mz, cx, cy);
      if (inGoal && !res.inGoal && res.legStart != null) {
        const tLeg = st.t - res.legStart;
        res.runs.push(tLeg); if (res.firstArrival == null) res.firstArrival = tLeg;
        if (res.best == null || tLeg < res.best) res.best = tLeg;
        log('META en ' + tLeg.toFixed(2) + ' s', 'ok'); res.legStart = null;
        if (opts.stopAtGoal) endRun('done', '¡Llegó a la meta en ' + tLeg.toFixed(2) + ' s!');
      }
      res.inGoal = inGoal;
      const atStart = cx === mz.start.x && cy === mz.start.y;
      if (atStart && res.legStart == null && res.tStart != null && !inGoal) res.armed = true;
      if (!atStart && res.armed) { res.armed = false; res.legStart = st.t; log('Nueva salida desde INICIO', 'sys'); }
      if (res.tStart != null) {
        if (Math.hypot(st.x - stuckX, st.y - stuckY) > RULES.stuckMM) { stuckX = st.x; stuckY = st.y; stuckT = st.t; }
        else if (st.t - stuckT > RULES.stuckS && !inGoal) endRun('dq', 'Descalificado: atorado más de 30 s en un pasillo');
      } else stuckT = st.t;
      if (brownout) endRun('dq', 'Descalificado: el Arduino se reinició (batería cayó a ' + Vbat.toFixed(2) + ' V)');
      if (st.t > (opts.maxTime || RULES.maxTime)) endRun('dq', 'Tiempo límite de la simulación (' + (opts.maxTime || RULES.maxTime) + ' s)');
      if (res.ended) endRun(res.firstArrival != null ? 'done' : 'dq', res.firstArrival != null ? 'Programa terminado. Mejor llegada: ' + res.best.toFixed(2) + ' s' : 'El programa terminó sin llegar a la meta');
    }

    /* ---------- Avanzar el tiempo ---------- */
    function finished() { return res.status === 'done' || res.status === 'dq' || res.status === 'error'; }
    function advance(ms) {
      const tEnd = st.t + ms / 1000;
      const t0 = Date.now();
      while (st.t < tEnd && !finished()) {
        if (gen && !progDone && st.t * 1000 >= wake - 1e-6) {
          let r;
          try { r = gen.next(); } catch (e) { endRun('error', 'Error en el código (t=' + st.t.toFixed(2) + ' s): ' + e.message); break; }
          if (r.done) { progDone = true; res.ended = true; }
          else { const v = +r.value; wake = st.t * 1000 + (isFinite(v) && v > 0.05 ? v : 0.05); }
        }
        physics(DT);
        rules();
        if (st.t - trailT > 0.05) { trailT = st.t; trail.push([st.x, st.y]); if (trail.length > 20000) trail.splice(0, 5000); }
        if (Date.now() - t0 > 1500) break;
      }
    }
    function snapshot(sinceLog) {
      const from = Math.max(0, logs.length - (logSeq - (sinceLog || 0)));
      return {
        t: st.t, x: st.x, y: st.y, th: st.th, v: Math.hypot(st.vx, st.vy),
        pwm: [wheels[0].pwm, wheels[1].pwm], enc: [wheels[0].enc, wheels[1].enc], I: [wheels[0].I, wheels[1].I],
        mu: [wheels[0].mu, wheels[1].mu], Vbat, Vmot, acc: [accF[0], accF[1]], ir: lastIR, rays: lastRays, collisions, odo,
        status: res.status, reason: res.reason, runs: res.runs.slice(), first: res.firstArrival, best: res.best,
        tStart: res.tStart, legT: res.legStart != null ? st.t - res.legStart : null,
        stuck: res.tStart != null ? st.t - stuckT : 0,
        knocked: world.walls.filter(w => w.knocked).map(w => w.id),
        logs: logs.slice(from), logSeq, overlay: overlay, overlaySeq, beep: beepT >= 0 && st.t - beepT < 0.15,
        trail: trail.length
      };
    }
    function result() {
      return { seed, status: res.status, reason: res.reason, first: res.firstArrival, best: res.best, runs: res.runs.slice(),
        collisions, t: st.t, odo, Vbat, muL: wheels[0].mu, muR: wheels[1].mu, maze: mz.name };
    }
    return {
      advance, snapshot, result, finished, trail,
      worldInfo: () => ({ walls: world.walls.map(w => ({ id: w.id, cx: w.cx, cy: w.cy, ux: w.ux, uy: w.uy, hl: w.hl, ht: w.ht, refl: w.refl, border: w.border })),
        outline: OUTLINE, sensors: SENSORS, maze: serializeMaze(mz), cfg, wheel: { r: rWheel, l: RB.track / 2 } })
    };
  }

  /* ==========================================================================
     Manejo de mensajes (worker o hilo principal)
     ========================================================================== */
  let live = null, liveLog = 0, sentTrail = 0;
  function handle(msg, send) {
    if (msg.type === 'init') {
      live = createSim(msg.opts); liveLog = 0; sentTrail = 0;
      send({ type: 'world', world: live.worldInfo() });
      const sn = live.snapshot(0); liveLog = sn.logSeq; sn.trailPts = []; send({ type: 'state', state: sn });
    } else if (msg.type === 'advance') {
      if (!live) return;
      live.advance(msg.ms);
      const sn = live.snapshot(liveLog); liveLog = sn.logSeq;
      sn.trailPts = live.trail.slice(Math.min(sentTrail, live.trail.length)); sentTrail = live.trail.length;
      send({ type: 'state', state: sn });
    } else if (msg.type === 'batch') {
      const o = msg.opts, out = [];
      for (let i = 0; i < o.n; i++) {
        const seed = o.seed + i;
        let maze = o.maze;
        if (o.varyMaze === 'mutar') maze = serializeMaze(mutateMaze(parseMaze(o.maze), seed, o.mutations || 5));
        if (o.varyMaze === 'aleatoria') maze = serializeMaze(randomMaze(parseMaze(o.maze), seed));
        const sim = createSim(Object.assign({}, o, { seed, maze }));
        while (!sim.finished()) sim.advance(5000);
        const r = sim.result(); r.mazeData = maze; out.push(r);
        send({ type: 'batchProgress', i: i + 1, n: o.n, result: r });
      }
      send({ type: 'batchDone', results: out });
    }
  }

  return { handle, createSim, parseMaze, serializeMaze, randomMaze, mutateMaze, emptyMaze, shortestToGoal, reachDist,
    wallBetween, setWallBetween, isGoal, FAIL_PRESETS, ROBOT_DEFAULT, GEARS, OUTLINE, SENSORS, RULES };
}
