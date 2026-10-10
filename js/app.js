/* Interfaz del simulador: dibuja la pista, controla la simulación (en un Web Worker) y maneja los paneles. */
(function () {
  'use strict';
  const $ = id => document.getElementById(id);
  const CORE = SimCore();                       // copia local para utilidades (laberintos, presets)
  const store = {
    get(k, d) { try { const v = localStorage.getItem('simITM.' + k); return v == null ? d : JSON.parse(v); } catch (e) { return d; } },
    set(k, v) { try { localStorage.setItem('simITM.' + k, JSON.stringify(v)); } catch (e) { /* sin almacenamiento */ } }
  };

  /* ================= Estado ================= */
  const S = {
    prog: store.get('prog', 'previo'),
    editing: 'prog',
    progCode: store.get('progCode2', {}),
    lib: store.get('lib2', null),
    scenario: store.get('scenario', 'realista'),
    fail: store.get('fail', Object.assign({}, CORE.FAIL_PRESETS.realista)),
    robot: store.get('robot2', Object.assign({}, CORE.ROBOT_DEFAULT)),
    maze: store.get('maze', window.PISTA_OFICIAL),
    seed: store.get('seed', 1),
    running: false, pending: false, finished: false,
    world: null, st: null, trail: [], tele: [], overlay: null, overlaySeq: -1,
    editMode: 'none', hover: null, view: null
  };
  if (!window.PROGRAMAS[S.prog]) S.prog = 'previo';
  S.mzParsed = CORE.parseMaze(S.maze);

  const NAMES = { ideal: 'Ideal', realista: 'Realista', pesimista: 'Pesimista', peor: 'Peor caso', custom: 'Personalizado' };
  const codeFor = k => (S.progCode[k] != null ? S.progCode[k] : window.PROGRAMAS[k].code);
  const libCode = () => (S.lib != null ? S.lib : window.LIBRERIA);

  /* ================= Worker ================= */
  function makeWorker(onmsg) {
    const src = 'const CORE=(' + SimCore.toString() + ')();\nself.onmessage=function(e){CORE.handle(e.data,function(m){self.postMessage(m)})};';
    try {
      const w = new Worker(URL.createObjectURL(new Blob([src], { type: 'text/javascript' })));
      w.onmessage = e => onmsg(e.data);
      w.onerror = e => { logLine({ t: 0, msg: 'Error interno del simulador: ' + (e.message || e), kind: 'err' }); };
      return w;
    } catch (e) {
      // sin workers: correr en el hilo principal
      const core = SimCore();
      return { postMessage: d => setTimeout(() => core.handle(d, m => onmsg(m)), 0), terminate() {} };
    }
  }
  let live = null, watchdog = 0, sentAt = 0;
  function startLive() {
    if (live) live.terminate();
    live = makeWorker(onLive);
    S.pending = false;
  }
  function simOpts() {
    return {
      maze: S.maze, reference: window.PISTA_OFICIAL, fail: S.fail, robot: S.robot, seed: S.seed | 0,
      code: libCode() + '\n' + codeFor(S.prog), stopAtGoal: $('optStopGoal').checked, maxTime: 600
    };
  }
  function resetSim(keepRunning) {
    startLive();
    S.trail = []; S.tele = []; S.overlay = null; S.overlaySeq = -1; S.finished = false; S.st = null;
    $('banner').hidden = true;
    clearLog();
    S.pending = true; sentAt = performance.now();
    live.postMessage({ type: 'init', opts: simOpts() });
    S.running = !!keepRunning;
    updateRunBtn();
  }
  function onLive(m) {
    if (m.type === 'world') { S.world = m.world; S.view = null; renderSensTable(); updateTrackInfo(); return; }
    if (m.type === 'state') {
      S.pending = false;
      const st = m.state; S.st = st;
      if (st.trailPts) for (const p of st.trailPts) S.trail.push(p);
      if (st.overlay && st.overlaySeq !== S.overlaySeq) { S.overlay = st.overlay; S.overlaySeq = st.overlaySeq; }
      for (const l of st.logs) logLine(l);
      S.tele.push([st.t, st.v, st.ir ? st.ir.front : 0, st.pwm[0], st.pwm[1]]);
      while (S.tele.length && st.t - S.tele[0][0] > 8) S.tele.shift();
      updateStatus(st);
      if (['done', 'dq', 'error'].includes(st.status) && !S.finished) {
        S.finished = true; S.running = false; updateRunBtn();
        const b = $('banner'); b.hidden = false;
        b.className = 'banner ' + (st.status === 'done' ? 'ok' : 'bad');
        b.textContent = st.reason;
      }
    }
  }

  /* ================= Bucle de animación ================= */
  let lastFrame = performance.now();
  function frame(now) {
    const dt = Math.min(0.1, (now - lastFrame) / 1000); lastFrame = now;
    if (S.running && !S.pending && live) {
      const sp = $('speed').value;
      const ms = sp === 'max' ? 800 : Math.max(1, dt * 1000 * parseFloat(sp));
      S.pending = true; sentAt = now;
      live.postMessage({ type: 'advance', ms });
    }
    if (S.pending && now - sentAt > 5000) {
      // el código del usuario no regresa el control
      S.running = false; S.pending = false; updateRunBtn();
      live.terminate(); live = null;
      const b = $('banner'); b.hidden = false; b.className = 'banner bad';
      b.textContent = 'El programa se trabó: ¿un ciclo sin "yield"? Corrijan el código y reinicien.';
      logLine({ t: S.st ? S.st.t : 0, msg: 'El simulador detuvo el programa: no regresó el control en 5 s (bucle infinito sin yield).', kind: 'err' });
    }
    draw();
    drawTele();
    requestAnimationFrame(frame);
  }

  /* ================= Dibujo de la pista ================= */
  const cv = $('field'), ctx = cv.getContext('2d');
  let colors = {};
  function readColors() {
    const cs = getComputedStyle(document.documentElement);
    for (const k of ['floor', 'wall', 'pcb', 'accent', 'copper', 'danger', 'ok', 'muted', 'ink', 'warn'])
      colors[k] = cs.getPropertyValue('--' + k).trim();
  }
  function fitCanvas() {
    const box = cv.parentElement.clientWidth - 16;
    const mz = S.maze, W = mz.cols * mz.cell + 60, H = mz.rows * mz.cell + 60;
    const dpr = window.devicePixelRatio || 1;
    const cssW = Math.max(280, box), cssH = Math.round(cssW * H / W);
    cv.style.height = cssH + 'px';
    cv.width = Math.round(cssW * dpr); cv.height = Math.round(cssH * dpr);
    S.view = { dpr, cssW, cssH, W, H };
  }
  function transform() {
    const v = S.view, mz = S.maze;
    let scale = v.cssW / v.W * v.dpr, ox = 30, oy = 30;
    if ($('optFollow').checked && S.st) {
      scale *= 2.6;
      const cx = S.st.x, cy = S.st.y;
      return { scale, tx: cv.width / 2 - cx * scale, ty: cv.height / 2 + cy * scale };
    }
    return { scale, tx: ox * scale, ty: (mz.rows * mz.cell + oy) * scale };
  }
  const W2S = (T, x, y) => [T.tx + x * T.scale, T.ty - y * T.scale];
  function draw() {
    if (!S.view) fitCanvas();
    const T = transform(), mz = S.maze, c = mz.cell;
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.fillStyle = colors.floor; ctx.fillRect(0, 0, cv.width, cv.height);
    ctx.setTransform(T.scale, 0, 0, -T.scale, T.tx, T.ty);
    // postes (como los puntos rojos del dibujo de la convocatoria)
    ctx.fillStyle = 'rgba(255,255,255,0.10)';
    for (let y = 0; y <= mz.rows; y++) for (let x = 0; x <= mz.cols; x++) { ctx.beginPath(); ctx.arc(x * c, y * c, 3, 0, 7); ctx.fill(); }
    // meta
    ctx.fillStyle = 'rgba(255,255,255,0.07)';
    for (const g of mz.goal) ctx.fillRect(g[0] * c, g[1] * c, c, c);
    // línea blanca en la entrada de la meta
    ctx.strokeStyle = '#ffffff'; ctx.lineWidth = 6;
    for (const g of mz.goal) for (let d = 0; d < 4; d++) {
      const nx = g[0] + [0, 1, 0, -1][d], ny = g[1] + [1, 0, -1, 0][d];
      if (mz.goal.some(q => q[0] === nx && q[1] === ny)) continue;
      if (CORE.wallBetween(S.mzParsed, g[0], g[1], d)) continue;
      const x0 = g[0] * c, y0 = g[1] * c;
      ctx.beginPath();
      if (d === 0) { ctx.moveTo(x0 + 10, y0 + c - 4); ctx.lineTo(x0 + c - 10, y0 + c - 4); }
      if (d === 2) { ctx.moveTo(x0 + 10, y0 + 4); ctx.lineTo(x0 + c - 10, y0 + 4); }
      if (d === 1) { ctx.moveTo(x0 + c - 4, y0 + 10); ctx.lineTo(x0 + c - 4, y0 + c - 10); }
      if (d === 3) { ctx.moveTo(x0 + 4, y0 + 10); ctx.lineTo(x0 + 4, y0 + c - 10); }
      ctx.stroke();
    }
    // inicio: línea verde
    const s = mz.start;
    ctx.strokeStyle = '#2fbf5f'; ctx.lineWidth = 5; ctx.beginPath();
    ctx.moveTo((s.x + 0.85) * c, (s.y + 0.15) * c); ctx.lineTo((s.x + 0.85) * c, (s.y + 0.85) * c); ctx.stroke();
    // paredes físicas (con sus defectos)
    const knocked = new Set(S.st ? S.st.knocked : []);
    if (S.world) for (const w of S.world.walls) {
      const shade = Math.round(200 + 40 * Math.min(1, w.refl - 0.6));
      ctx.fillStyle = knocked.has(w.id) ? colors.danger : `rgb(${shade},${shade - 4},${shade - 14})`;
      ctx.beginPath();
      for (const [a, b] of [[-1, -1], [1, -1], [1, 1], [-1, 1]]) {
        const px = w.cx + w.ux * w.hl * a - w.uy * w.ht * b, py = w.cy + w.uy * w.hl * a + w.ux * w.ht * b;
        ctx.lineTo(px, py);
      }
      ctx.closePath(); ctx.fill();
    }
    // pared bajo el mouse en modo edición
    if (S.editMode === 'walls' && S.hover) {
      const h = S.hover; ctx.strokeStyle = colors.accent; ctx.lineWidth = 8; ctx.beginPath();
      if (h.kind === 'h') { ctx.moveTo(h.x * c, h.y * c); ctx.lineTo((h.x + 1) * c, h.y * c); } else { ctx.moveTo(h.x * c, h.y * c); ctx.lineTo(h.x * c, (h.y + 1) * c); }
      ctx.stroke();
    }
    // mapa que cree el robot
    if ($('optBelief').checked && S.overlay && S.overlay.cells) drawBelief(S.overlay, c);
    // rastro
    if ($('optTrail').checked && S.trail.length > 1) {
      ctx.strokeStyle = colors.accent; ctx.globalAlpha = 0.55; ctx.lineWidth = 2.5; ctx.beginPath();
      ctx.moveTo(S.trail[0][0], S.trail[0][1]); for (const p of S.trail) ctx.lineTo(p[0], p[1]); ctx.stroke(); ctx.globalAlpha = 1;
    }
    if (S.st && S.world) drawRobot(S.st);
    // textos (sin espejo)
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    const fs = Math.max(10, T.scale * 26);
    ctx.font = `600 ${fs}px "Chakra Petch", sans-serif`; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.fillStyle = 'rgba(255,255,255,0.85)';
    const gx = mz.goal.reduce((a, g) => a + g[0] + 0.5, 0) / mz.goal.length, gy = mz.goal.reduce((a, g) => a + g[1] + 0.5, 0) / mz.goal.length;
    let [tx, ty] = W2S(T, gx * c, gy * c); ctx.fillText('META', tx, ty);
    ctx.fillStyle = '#4fdc84'; [tx, ty] = W2S(T, (s.x + 0.5) * c, (s.y + 0.5) * c - 30); ctx.fillText('INICIO', tx, ty);
    if ($('optDist').checked && S.overlay && S.overlay.dist && T.scale * c > 30) {
      ctx.font = `500 ${Math.max(9, T.scale * 22)}px "JetBrains Mono", monospace`; ctx.fillStyle = 'rgba(120,220,255,0.75)';
      const o = S.overlay;
      for (let y = 0; y < o.rows; y++) for (let x = 0; x < o.cols; x++) {
        const d = o.dist[y * o.cols + x]; if (d < 0) continue;
        [tx, ty] = W2S(T, (x + 0.5) * c, (y + 0.5) * c + 40); ctx.fillText(String(d), tx, ty);
      }
    }
  }
  function drawBelief(o, c) {
    ctx.lineWidth = 3; ctx.strokeStyle = 'rgba(90,200,255,0.9)';
    const seg = (x0, y0, x1, y1) => { ctx.moveTo(x0, y0); ctx.lineTo(x1, y1); };
    ctx.beginPath();
    const prior = [];
    for (let y = 0; y < o.rows; y++) for (let x = 0; x < o.cols; x++) {
      const v = o.cells[y * o.cols + x], x0 = x * c, y0 = y * c, k = 16;
      for (let d = 0; d < 4; d++) {
        const present = (v >> d) & 1, known = (v >> (4 + d)) & 1;
        if (!present) continue;
        const line = d === 0 ? [x0 + k, y0 + c - k, x0 + c - k, y0 + c - k] : d === 2 ? [x0 + k, y0 + k, x0 + c - k, y0 + k] :
          d === 1 ? [x0 + c - k, y0 + k, x0 + c - k, y0 + c - k] : [x0 + k, y0 + k, x0 + k, y0 + c - k];
        if (known) seg(...line); else prior.push(line);
      }
    }
    ctx.stroke();
    if (prior.length) {
      ctx.setLineDash([6, 8]); ctx.strokeStyle = 'rgba(90,200,255,0.35)'; ctx.beginPath();
      for (const l of prior) seg(...l); ctx.stroke(); ctx.setLineDash([]);
    }
    if (o.pose) {
      const p = o.pose, cx = (p.x + 0.5) * c, cy = (p.y + 0.5) * c;
      ctx.strokeStyle = 'rgba(90,200,255,0.9)'; ctx.setLineDash([10, 6]); ctx.lineWidth = 2.5;
      ctx.strokeRect(p.x * c + 8, p.y * c + 8, c - 16, c - 16); ctx.setLineDash([]);
      const a = Math.PI / 2 - p.h * Math.PI / 2;
      ctx.beginPath(); ctx.moveTo(cx + Math.cos(a) * 30, cy + Math.sin(a) * 30);
      ctx.lineTo(cx + Math.cos(a + 2.5) * 16, cy + Math.sin(a + 2.5) * 16); ctx.lineTo(cx + Math.cos(a - 2.5) * 16, cy + Math.sin(a - 2.5) * 16); ctx.closePath();
      ctx.fillStyle = 'rgba(90,200,255,0.8)'; ctx.fill();
    }
  }
  function drawRobot(st) {
    const W = S.world, cs = Math.cos(st.th), sn = Math.sin(st.th);
    const P = (f, l) => [st.x + f * cs - l * sn, st.y + f * sn + l * cs];
    // rayos IR
    if ($('optRays').checked && st.rays) for (const r of st.rays) {
      ctx.strokeStyle = colors.copper; ctx.globalAlpha = 0.25 + 0.75 * Math.min(1, r[4] / 700); ctx.lineWidth = 2;
      ctx.beginPath(); ctx.moveTo(r[0], r[1]); ctx.lineTo(r[2], r[3]); ctx.stroke();
      ctx.beginPath(); ctx.arc(r[2], r[3], 4, 0, 7); ctx.fillStyle = colors.copper; ctx.fill();
    }
    ctx.globalAlpha = 1;
    // PCB
    ctx.beginPath(); for (const p of W.outline) ctx.lineTo(...P(p[0], p[1])); ctx.closePath();
    ctx.fillStyle = colors.pcb; ctx.globalAlpha = 0.92; ctx.fill(); ctx.globalAlpha = 1;
    ctx.strokeStyle = 'rgba(255,255,255,0.7)'; ctx.lineWidth = 1.5; ctx.stroke();
    // componentes principales (Nano y motores) como referencia visual
    ctx.fillStyle = 'rgba(10,30,60,0.85)';
    const rect = (f0, l0, f1, l1) => { ctx.beginPath(); for (const [f, l] of [[f0, l0], [f1, l0], [f1, l1], [f0, l1]]) ctx.lineTo(...P(f, l)); ctx.closePath(); ctx.fill(); };
    rect(-41, 22, -18, -22);                                   // Arduino Nano
    ctx.fillStyle = 'rgba(200,200,200,0.55)'; rect(-6, 39, 6, 2); rect(-6, -2, 6, -39);   // motores N20
    // llantas
    ctx.fillStyle = '#0b0b0b'; ctx.strokeStyle = 'rgba(255,255,255,0.35)';
    for (const side of [1, -1]) { const r = W.wheel.r; rect(-r, side * 41, r, side * 49); }
    // sensores
    for (const s of W.sensors) { const [x, y] = P(s.f, s.l); ctx.beginPath(); ctx.arc(x, y, 3.5, 0, 7); ctx.fillStyle = '#ffd38a'; ctx.fill(); }
    // frente
    ctx.beginPath(); ctx.moveTo(...P(46, 0)); ctx.lineTo(...P(30, 8)); ctx.lineTo(...P(30, -8)); ctx.closePath(); ctx.fillStyle = '#ffffff'; ctx.fill();
    if (st.beep) { ctx.beginPath(); ctx.arc(st.x, st.y, 80, 0, 7); ctx.strokeStyle = colors.warn; ctx.lineWidth = 4; ctx.stroke(); }
  }

  /* ================= Telemetría ================= */
  const tc = $('tele'), tctx = tc.getContext('2d');
  function drawTele() {
    const dpr = window.devicePixelRatio || 1, w = tc.clientWidth, h = tc.clientHeight;
    if (tc.width !== Math.round(w * dpr)) { tc.width = Math.round(w * dpr); tc.height = Math.round(h * dpr); }
    tctx.setTransform(dpr, 0, 0, dpr, 0, 0); tctx.clearRect(0, 0, w, h);
    tctx.strokeStyle = colors.muted; tctx.globalAlpha = 0.25; tctx.lineWidth = 1;
    for (let i = 1; i < 4; i++) { tctx.beginPath(); tctx.moveTo(0, h * i / 4); tctx.lineTo(w, h * i / 4); tctx.stroke(); }
    tctx.globalAlpha = 1;
    const D = S.tele; if (D.length < 2) return;
    const t1 = D[D.length - 1][0], t0 = t1 - 8, X = t => (t - t0) / 8 * w;
    const line = (idx, max, color, mid) => {
      tctx.strokeStyle = color; tctx.lineWidth = 1.6; tctx.beginPath();
      D.forEach((d, i) => { const y = mid ? h / 2 - d[idx] / max * h / 2 : h - d[idx] / max * h; i ? tctx.lineTo(X(d[0]), y) : tctx.moveTo(X(d[0]), y); });
      tctx.stroke();
    };
    tctx.globalAlpha = 0.6; line(3, 255, colors.muted, true); line(4, 255, colors.muted, true); tctx.globalAlpha = 1;
    const vTop = Math.max(400, Math.ceil(Math.max(...D.map(d => d[1])) / 100) * 100);   // escala según el robot
    line(2, 1023, colors.copper); line(1, vTop, colors.accent);
    tctx.fillStyle = colors.muted; tctx.font = '10px "JetBrains Mono", monospace'; tctx.fillText(vTop + ' mm/s', 4, 11);
  }

  /* ================= Estado en pantalla ================= */
  const IRK = [['left', 'Izq'], ['dleft', 'Diag I'], ['front', 'Frente'], ['dright', 'Diag D'], ['right', 'Der']];
  $('irbars').innerHTML = IRK.map(([k, n]) => `<div class="irbar"><div class="tube"><i id="ir_${k}"></i></div><b id="irv_${k}">—</b><span>${n}</span></div>`).join('');
  function fmtT(t) { return t == null ? '—' : t.toFixed(2) + ' s'; }
  function updateStatus(st) {
    const runningT = st.tStart == null ? 0 : (st.first != null && $('optStopGoal').checked ? st.first : st.t - st.tStart);
    $('sTimer').textContent = (st.legT != null ? st.legT : runningT).toFixed(2) + ' s';
    const chip = $('sStatus');
    const label = { ready: 'Listo', running: 'Corriendo', done: 'Meta', dq: 'Descalificado', error: 'Error' }[st.status] || st.status;
    chip.textContent = S.running || st.status !== 'ready' ? label : 'Listo'; chip.className = 'chip ' + st.status;
    $('sRuns').textContent = st.runs.length ? st.runs.map(x => x.toFixed(2)).join(' · ') + ' s' : '—';
    $('sHits').textContent = st.collisions;
    $('sStuck').textContent = st.stuck.toFixed(1) + ' s';
    $('sStuckBar').style.width = Math.min(100, st.stuck / 30 * 100) + '%';
    const bat = $('sBat'); bat.textContent = st.Vbat.toFixed(2) + ' / ' + st.Vmot.toFixed(2) + ' V'; bat.className = 'v' + (st.Vbat < 6.8 ? ' bad' : st.Vmot < 5.6 ? ' warn' : '');
    const mu = $('sMu'); mu.textContent = st.mu[0].toFixed(2) + ' / ' + st.mu[1].toFixed(2); mu.className = 'v' + (Math.min(...st.mu) < 0.45 ? ' warn' : '');
    $('sVel').textContent = st.v.toFixed(0) + ' mm/s';
    $('sSimT').textContent = st.t.toFixed(1) + ' s';
    for (const [k] of IRK) {
      const v = st.ir ? st.ir[k] : 0;
      $('ir_' + k).style.height = (v / 1023 * 100) + '%'; $('irv_' + k).textContent = st.ir ? v : '—';
    }
    $('kEncL').textContent = st.enc[0]; $('kEncR').textContent = st.enc[1];
    $('kPwmL').textContent = st.pwm[0]; $('kPwmR').textContent = st.pwm[1];
    $('kI').textContent = (Math.abs(st.I[0]) + Math.abs(st.I[1])).toFixed(2) + ' A';
    $('kOdo').textContent = (st.odo / 1000).toFixed(2) + ' m';
  }
  function updateRunBtn() {
    const b = $('btnRun');
    b.textContent = S.running ? '⏸ Pausa' : (S.finished ? '▶ Otra vez' : '▶ Iniciar');
  }

  /* ================= Registro ================= */
  const logBox = $('log'); let logCount = 0;
  function logLine(l) {
    const isUser = l.kind === 'user';
    if (isUser && !$('logUser').checked) return;
    if (!isUser && !$('logSys').checked) return;
    const d = document.createElement('div');
    d.className = l.kind === 'user' ? '' : l.kind;
    const t = document.createElement('span'); t.className = 't'; t.textContent = l.t.toFixed(2).padStart(7, ' ');
    d.appendChild(t); d.appendChild(document.createTextNode(l.msg));
    const atBottom = logBox.scrollTop + logBox.clientHeight >= logBox.scrollHeight - 20;
    logBox.appendChild(d); logCount++;
    if (logCount > 2500) { logBox.removeChild(logBox.firstChild); logCount--; }
    if (atBottom) logBox.scrollTop = logBox.scrollHeight;
  }
  function clearLog() { logBox.textContent = ''; logCount = 0; }
  $('logClear').onclick = clearLog;

  /* ================= Controles principales ================= */
  $('btnRun').onclick = () => {
    if (S.finished || !live) { resetSim(true); return; }
    S.running = !S.running; updateRunBtn();
  };
  $('btnReset').onclick = () => resetSim(false);
  $('btnStep').onclick = () => { if (!live || S.pending || S.finished) return; S.running = false; updateRunBtn(); S.pending = true; sentAt = performance.now(); live.postMessage({ type: 'advance', ms: 100 }); };
  $('seed').value = S.seed;
  $('seed').onchange = () => { S.seed = Math.max(1, parseInt($('seed').value) || 1); store.set('seed', S.seed); resetSim(false); };
  $('btnDice').onclick = () => { S.seed = 1 + Math.floor(Math.random() * 99999); $('seed').value = S.seed; store.set('seed', S.seed); resetSim(false); };
  for (const id of ['optBelief', 'optDist', 'optRays', 'optTrail', 'optFollow']) $(id).onchange = () => { };
  $('optStopGoal').onchange = () => resetSim(false);

  /* ================= Panel de código ================= */
  const progSel = $('prog');
  progSel.innerHTML = Object.entries(window.PROGRAMAS).map(([k, p]) => `<option value="${k}">${p.nombre}</option>`).join('');
  progSel.value = S.prog;
  function showCode() {
    $('progDesc').textContent = window.PROGRAMAS[S.prog].desc;
    $('code').value = S.editing === 'lib' ? libCode() : codeFor(S.prog);
    $('edProg').setAttribute('aria-pressed', S.editing === 'prog'); $('edLib').setAttribute('aria-pressed', S.editing === 'lib');
  }
  progSel.onchange = () => {
    S.prog = progSel.value; store.set('prog', S.prog);
    $('optStopGoal').checked = S.prog !== 'explorar';
    S.editing = 'prog'; showCode(); resetSim(false);
  };
  $('edProg').onclick = () => { saveEditor(); S.editing = 'prog'; showCode(); };
  $('edLib').onclick = () => { saveEditor(); S.editing = 'lib'; showCode(); };
  function saveEditor() {
    const v = $('code').value;
    if (S.editing === 'lib') { S.lib = v === window.LIBRERIA ? null : v; store.set('lib2', S.lib); }
    else { if (v === window.PROGRAMAS[S.prog].code) delete S.progCode[S.prog]; else S.progCode[S.prog] = v; store.set('progCode2', S.progCode); }
  }
  $('code').addEventListener('input', () => saveEditor());
  $('code').addEventListener('keydown', e => {
    if (e.key === 'Tab') { e.preventDefault(); const t = e.target, a = t.selectionStart, b = t.selectionEnd; t.value = t.value.slice(0, a) + '  ' + t.value.slice(b); t.selectionStart = t.selectionEnd = a + 2; saveEditor(); }
    if ((e.ctrlKey || e.metaKey) && e.key === 'Enter') { e.preventDefault(); saveEditor(); resetSim(true); }
  });
  $('btnApply').onclick = () => { saveEditor(); resetSim(true); };
  let restoreArmed = 0;
  $('btnRestore').onclick = () => {
    if (Date.now() - restoreArmed > 3000) { restoreArmed = Date.now(); $('btnRestore').textContent = '¿Seguro? clic otra vez'; setTimeout(() => { $('btnRestore').textContent = 'Restaurar original'; }, 3000); return; }
    restoreArmed = 0; $('btnRestore').textContent = 'Restaurar original';
    if (S.editing === 'lib') { S.lib = null; store.set('lib2', null); } else { delete S.progCode[S.prog]; store.set('progCode2', S.progCode); }
    showCode(); resetSim(false);
  };

  /* ================= Panel de fallas ================= */
  const FAILS = [
    ['Paredes impresas en 3D', [
      ['wallOffset', 'Desalineación de paredes', 0, 6, 0.1, 'mm', 'Cuánto se mueve cada pared de su lugar ideal (σ).'],
      ['wallAngle', 'Paredes chuecas', 0, 3, 0.1, '°', 'Ángulo de inclinación de cada pared vista desde arriba (σ).'],
      ['gapProb', 'Uniones con hueco', 0, 1, 0.01, '%', 'Probabilidad de que un extremo de pared no toque el poste. Por el hueco el sensor no ve pared.'],
      ['gapMax', 'Tamaño máximo del hueco', 0, 8, 0.5, 'mm'],
      ['wallThickVar', 'Variación de grosor', 0, 2, 0.1, 'mm'],
      ['reflVar', 'Variación de reflejo', 0, 0.5, 0.01, '%', 'Color y acabado distinto del PLA: unas paredes regresan menos luz IR.'],
      ['wallHold', 'Fuerza para tumbar una pared', 0.5, 10, 0.1, 'N', 'Si el robot empuja más que esto por 80 ms, la pared cae y lo descalifican.']]],
    ['Llantas', [
      ['mu0', 'Agarre de llanta limpia', 0.3, 1, 0.01, ''],
      ['dirtRate', 'Pérdida de agarre por metro', 0, 0.3, 0.005, '/m', 'Polvo de la pista pegado en la llanta.'],
      ['muMin', 'Agarre de llanta sucia (mínimo)', 0.2, 1, 0.01, ''],
      ['wheelDiamVar', 'Diferencia de diámetro', 0, 3, 0.1, '%', 'Una llanta un poco más grande hace que el robot se desvíe y que los encoders mientan.']]],
    ['Motores y driver', [
      ['motorMismatch', 'Diferencia entre motores', 0, 20, 0.5, '%'],
      ['deadband', 'Fricción de la caja (zona muerta)', 0, 15, 0.5, '%', 'Porcentaje extra de torque necesario para que la llanta empiece a girar.']]],
    ['Sensores IR', [
      ['ambient', 'Luz ambiental', 0, 250, 1, '', 'Sol o lámparas. Los emisores siempre están prendidos, así que esto se suma a todas las lecturas.'],
      ['ambFlicker', 'Parpadeo de la luz', 0, 60, 1, ''],
      ['irNoise', 'Ruido eléctrico', 0, 20, 0.5, ''],
      ['irGainVar', 'Diferencia entre sensores (trimpots)', 0, 40, 1, '%'],
      ['irSpike', 'Lecturas basura', 0, 0.05, 0.001, '%', 'Probabilidad por lectura de un valor al azar (falso contacto, ruido del motor).'],
      ['deadSensor', 'Sensor dañado', -1, 4, 1, 'sel']]],
    ['Encoders y giroscopio', [
      ['encMiss', 'Pulsos perdidos del encoder', 0, 5, 0.1, '%'],
      ['gyroDrift', 'Deriva del giroscopio', 0, 0.2, 0.005, '°/s√s'],
      ['gyroScale', 'Error de escala del giroscopio', 0, 5, 0.1, '%', 'Si es 2 %, un giro de 90° se mide como 88° o 92°.']]],
    ['Arranque y batería', [
      ['startPos', 'Error al colocar el robot', 0, 10, 0.5, 'mm'],
      ['startAng', 'Robot colocado chueco', 0, 8, 0.5, '°'],
      ['batteryV', 'Voltaje de la batería al empezar', 6.8, 8.4, 0.05, 'V', 'LiPo 2S: 8.4 V llena, 7.4 V nominal. El LM7806 necesita ~8 V para dar 6 V.']]]
  ];
  const SENS_NAMES = ['Ninguno', 'Izquierdo', 'Diagonal izq.', 'Frontal', 'Diagonal der.', 'Derecho'];
  // unidades '%' con máximo ≤ 1 se guardan como fracción (0.15 = 15 %)
  function fmtFail(unit, v, max) {
    if (unit === '%' && max <= 1) return (v * 100).toFixed(v < 0.1 ? 1 : 0) + ' %';
    return (Math.round(v * 1000) / 1000) + (unit ? ' ' + unit : '');
  }
  function renderFails() {
    $('failGroups').innerHTML = FAILS.map(([g, items]) => `<div class="group"><h4>${g}</h4>${items.map(([k, label, min, max, step, unit, help]) => {
      if (unit === 'sel') return `<div class="slider"><label for="f_${k}">${label}</label><span></span><select id="f_${k}" style="grid-column:1/-1">${SENS_NAMES.map((n, i) => `<option value="${i - 1}">${n}</option>`).join('')}</select></div>`;
      return `<div class="slider"><label for="f_${k}">${label}</label><output id="o_${k}"></output><input type="range" id="f_${k}" min="${min}" max="${max}" step="${step}">${help ? `<small>${help}</small>` : ''}</div>`;
    }).join('')}</div>`).join('');
    for (const [, items] of FAILS) for (const [k, , , , , unit] of items) {
      const el = $('f_' + k);
      el.value = S.fail[k];
      if ($('o_' + k)) $('o_' + k).textContent = fmtFail(unit, +el.value, +el.max);
      el.oninput = () => {
        S.fail[k] = parseFloat(el.value);
        if ($('o_' + k)) $('o_' + k).textContent = fmtFail(unit, S.fail[k], +el.max);
        if (S.scenario !== 'custom') { S.scenario = 'custom'; $('scenario').value = 'custom'; store.set('scenario', 'custom'); }
        store.set('fail', S.fail);
      };
      el.onchange = () => resetSim(false);
    }
  }
  $('scenario').value = S.scenario;
  $('scenario').onchange = () => {
    S.scenario = $('scenario').value; store.set('scenario', S.scenario);
    if (S.scenario !== 'custom') { S.fail = Object.assign({}, CORE.FAIL_PRESETS[S.scenario]); store.set('fail', S.fail); }
    renderFails(); resetSim(false);
  };

  /* ================= Panel del robot ================= */
  const RB_FIELDS = [['rGear', 'gear', 1], ['rWheel', 'wheelD', 1], ['rTrack', 'track', 1], ['rMass', 'massKg', 1000], ['rCap', 'capacity', 1], ['rRint', 'rint', 1], ['rDrop', 'regDropout', 1], ['rPPR', 'encPPR', 1]];
  function renderRobot() {
    for (const [id, k, mul] of RB_FIELDS) $(id).value = Math.round(S.robot[k] * mul * 1000) / 1000;
    $('rL293').checked = !!S.robot.l293;
  }
  $('btnRobotApply').onclick = () => {
    for (const [id, k, mul] of RB_FIELDS) { const v = parseFloat($(id).value); if (isFinite(v)) S.robot[k] = v / mul; }
    S.robot.l293 = $('rL293').checked; store.set('robot2', S.robot); resetSim(false);
  };
  $('btnRobotDefault').onclick = () => { S.robot = Object.assign({}, CORE.ROBOT_DEFAULT); store.set('robot2', S.robot); renderRobot(); resetSim(false); };
  function renderSensTable() {
    const names = { left: 'Lateral izq. (LED2)', dleft: 'Diagonal izq. (LED3)', front: 'Frontal (LED5)', dright: 'Diagonal der. (LED7)', right: 'Lateral der. (LED9)' };
    $('sensTable').innerHTML = S.world.sensors.map(s => `<tr><td>${names[s.key]}</td><td class="num">${s.f.toFixed(1)} mm</td><td class="num">${s.l.toFixed(1)} mm</td><td class="num">${s.a.toFixed(1)}°</td></tr>`).join('');
  }

  /* ================= Panel de pista ================= */
  function setMaze(m, keep) {
    S.maze = m; S.mzParsed = CORE.parseMaze(m); store.set('maze', m);
    $('mzJson').value = JSON.stringify(m);
    $('mzCell').value = m.cell; $('mzWall').value = m.wall; $('startDir').value = m.start.dir;
    S.view = null; fitCanvas(); if (!keep) resetSim(false);
  }
  function updateTrackInfo() {
    const mz = CORE.parseMaze(S.maze), d = CORE.shortestToGoal(mz);
    $('trackInfo').textContent = `${S.maze.name}: ${mz.cols} × ${mz.rows} celdas de ${mz.cell} mm (${(mz.cols * mz.cell / 1000).toFixed(2)} × ${(mz.rows * mz.cell / 1000).toFixed(2)} m). Camino más corto: ${d < 0 ? 'no hay' : d + ' celdas'}.`;
  }
  $('mzOfficial').onclick = () => setMaze(JSON.parse(JSON.stringify(window.PISTA_OFICIAL)));
  $('mzMutate').onclick = () => setMaze(CORE.serializeMaze(CORE.mutateMaze(CORE.parseMaze(window.PISTA_OFICIAL), 1 + Math.floor(Math.random() * 9999), Math.max(1, parseInt($('mzMutN').value) || 6))));
  $('mzRandom').onclick = () => setMaze(CORE.serializeMaze(CORE.randomMaze(CORE.parseMaze(S.maze), 1 + Math.floor(Math.random() * 9999))));
  $('mzEmpty').onclick = () => { const m = CORE.serializeMaze(CORE.emptyMaze(S.maze.cols, S.maze.rows, CORE.parseMaze(S.maze))); setMaze(m); };
  $('mzCell').onchange = () => { const v = parseFloat($('mzCell').value); if (v >= 140 && v <= 200) { S.maze = Object.assign({}, S.maze, { cell: v }); setMaze(S.maze); } };
  $('mzWall').onchange = () => { const v = parseFloat($('mzWall').value); if (v >= 4 && v <= 20) { S.maze = Object.assign({}, S.maze, { wall: v }); setMaze(S.maze); } };
  $('startDir').onchange = () => { S.maze = Object.assign({}, S.maze, { start: Object.assign({}, S.maze.start, { dir: +$('startDir').value }) }); setMaze(S.maze); };
  $('mzCopy').onclick = () => {
    const t = $('mzJson');
    navigator.clipboard.writeText(t.value).then(() => { $('mzCopy').textContent = 'Copiado'; setTimeout(() => $('mzCopy').textContent = 'Copiar', 1500); }).catch(() => { t.focus(); t.select(); });
  };
  $('mzDownload').onclick = () => {
    const a = document.createElement('a'); a.href = URL.createObjectURL(new Blob([JSON.stringify(S.maze, null, 1)], { type: 'application/json' }));
    a.download = (S.maze.name || 'pista').replace(/[^\w\-]+/g, '_') + '.json'; document.body.appendChild(a); a.click(); a.remove();
  };
  function loadMazeText(txt) {
    try {
      const m = JSON.parse(txt);
      if (!m.cols || !m.rows || !m.h || !m.v || !m.start || !m.goal) throw new Error('faltan campos (cols, rows, h, v, start, goal)');
      if (m.h.length !== m.rows + 1 || m.v.length !== m.rows) throw new Error('el número de filas de h o v no coincide con rows');
      setMaze(m);
    } catch (e) { logLine({ t: 0, msg: 'No se pudo cargar la pista: ' + e.message, kind: 'err' }); selectTab('log'); }
  }
  $('mzLoadText').onclick = () => loadMazeText($('mzJson').value);
  $('mzFile').onchange = e => { const f = e.target.files[0]; if (!f) return; const rd = new FileReader(); rd.onload = () => loadMazeText(rd.result); rd.readAsText(f); e.target.value = ''; };
  for (const [id, mode] of [['edNone', 'none'], ['edWalls', 'walls'], ['edStart', 'start'], ['edGoal', 'goal']]) {
    $(id).onclick = () => {
      S.editMode = mode;
      for (const b of ['edNone', 'edWalls', 'edStart', 'edGoal']) $(b).setAttribute('aria-pressed', b === id);
      $('editHint').hidden = mode === 'none';
      $('editHint').textContent = { walls: 'Modo edición: clic en una línea para poner/quitar pared', start: 'Clic en una celda para poner el INICIO', goal: 'Clic en celdas para agregar/quitar META' }[mode] || '';
    };
  }
  function mouseWorld(e) {
    const r = cv.getBoundingClientRect(), T = transform(), dpr = S.view.dpr;
    const px = (e.clientX - r.left) * dpr, py = (e.clientY - r.top) * dpr;
    return [(px - T.tx) / T.scale, (T.ty - py) / T.scale];
  }
  function edgeAt(x, y) {
    const mz = S.maze, c = mz.cell, fx = x / c, fy = y / c;
    const dh = Math.abs(fy - Math.round(fy)), dv = Math.abs(fx - Math.round(fx));
    if (Math.min(dh, dv) > 0.2) return null;
    if (dh < dv) { const ex = Math.floor(fx), ey = Math.round(fy); if (ex < 0 || ex >= mz.cols || ey <= 0 || ey >= mz.rows) return null; return { kind: 'h', x: ex, y: ey }; }
    const ex = Math.round(fx), ey = Math.floor(fy); if (ey < 0 || ey >= mz.rows || ex <= 0 || ex >= mz.cols) return null; return { kind: 'v', x: ex, y: ey };
  }
  cv.addEventListener('mousemove', e => { if (S.editMode !== 'walls') { S.hover = null; return; } const [x, y] = mouseWorld(e); S.hover = edgeAt(x, y); });
  cv.addEventListener('mouseleave', () => { S.hover = null; });
  cv.addEventListener('click', e => {
    if (S.editMode === 'none') return;
    const [x, y] = mouseWorld(e), m = JSON.parse(JSON.stringify(S.maze)), c = m.cell;
    if (S.editMode === 'walls') {
      const ed = edgeAt(x, y); if (!ed) return;
      const arr = ed.kind === 'h' ? m.h : m.v, row = arr[ed.y].split('');
      row[ed.x] = row[ed.x] === '1' ? '0' : '1'; arr[ed.y] = row.join('');
    } else {
      const cx = Math.floor(x / c), cy = Math.floor(y / c); if (cx < 0 || cy < 0 || cx >= m.cols || cy >= m.rows) return;
      if (S.editMode === 'start') m.start = { x: cx, y: cy, dir: +$('startDir').value };
      else { const i = m.goal.findIndex(g => g[0] === cx && g[1] === cy); if (i >= 0) { if (m.goal.length > 1) m.goal.splice(i, 1); } else m.goal.push([cx, cy]); }
    }
    m.name = m.name.includes('(editada)') ? m.name : m.name + ' (editada)';
    setMaze(m);
  });

  /* ================= Lote ================= */
  let batchW = null, batchRes = [];
  const history = store.get('history', []);
  function renderHistory() {
    $('bHistory').innerHTML = history.slice().reverse().map(h => `<tr><td>${h.prog}</td><td>${h.scen}</td><td>${h.track}</td><td class="num">${h.ok}/${h.n}</td><td class="num">${fmtT(h.avg)}</td><td class="num">${fmtT(h.best)}</td></tr>`).join('') || '<tr><td colspan="6" style="color:var(--muted)">Todavía no hay lotes. Corran uno para comparar estrategias.</td></tr>';
  }
  function renderBatch(done) {
    const ok = batchRes.filter(r => r.first != null), times = ok.map(r => r.first);
    const avg = times.length ? times.reduce((a, b) => a + b) / times.length : null;
    const reasons = {};
    for (const r of batchRes) if (r.first == null) { const k = r.reason.replace(/\(.*\)/, '').replace(/ en \d.*/, '').trim(); reasons[k] = (reasons[k] || 0) + 1; }
    const card = (k, v) => `<div class="stat"><div class="k">${k}</div><div class="v">${v}</div></div>`;
    $('bCards').innerHTML = card('Llegaron', batchRes.length ? `${ok.length}/${batchRes.length} (${Math.round(ok.length / batchRes.length * 100)} %)` : '—') +
      card('Tiempo prom.', fmtT(avg)) + card('Mejor', fmtT(times.length ? Math.min(...times) : null)) + card('Peor', fmtT(times.length ? Math.max(...times) : null)) +
      card('Choques prom.', batchRes.length ? (batchRes.reduce((a, r) => a + r.collisions, 0) / batchRes.length).toFixed(1) : '—');
    $('bRows').innerHTML = batchRes.map((r, i) => `<tr><td class="num">${r.seed}</td><td><span class="pill ${r.first != null ? 'ok' : 'bad'}">${r.first != null ? 'Meta' : 'Falló'}</span> <span style="color:var(--muted)">${r.first != null ? '' : r.reason.replace('Descalificado: ', '')}</span></td><td class="num">${fmtT(r.first)}</td><td class="num">${r.collisions}</td><td><button type="button" class="btn" data-i="${i}">Ver</button></td></tr>`).join('');
    drawHist(times);
    if (done && batchRes.length) {
      history.push({ prog: window.PROGRAMAS[S.prog].nombre.split(' (')[0], scen: NAMES[S.scenario], track: $('bVary').selectedOptions[0].textContent.replace('Siempre la actual', S.maze.name.split(' (')[0]), ok: ok.length, n: batchRes.length, avg, best: times.length ? Math.min(...times) : null });
      if (history.length > 30) history.shift();
      store.set('history', history); renderHistory();
      const rs = Object.entries(reasons).sort((a, b) => b[1] - a[1]).map(([k, v]) => `${v}× ${k}`).join(' · ');
      $('bMsg').textContent = rs ? 'Fallas: ' + rs : 'Todas llegaron.';
    }
  }
  const hc = $('hist'), hctx = hc.getContext('2d');
  function drawHist(times) {
    const dpr = window.devicePixelRatio || 1, w = hc.clientWidth, h = hc.clientHeight;
    hc.width = Math.round(w * dpr); hc.height = Math.round(h * dpr); hctx.setTransform(dpr, 0, 0, dpr, 0, 0); hctx.clearRect(0, 0, w, h);
    if (!times.length) { hctx.fillStyle = colors.muted; hctx.font = '12px "IBM Plex Sans", sans-serif'; hctx.fillText('Histograma de tiempos a la meta', 6, 16); return; }
    const lo = Math.floor(Math.min(...times)), hi = Math.ceil(Math.max(...times)) + 1, nb = Math.min(20, Math.max(4, hi - lo)), bw = (hi - lo) / nb;
    const bins = new Array(nb).fill(0); for (const t of times) bins[Math.min(nb - 1, Math.floor((t - lo) / bw))]++;
    const mx = Math.max(...bins), pad = 18, gw = (w - 8) / nb;
    hctx.fillStyle = colors.accent;
    bins.forEach((b, i) => { const bh = (h - pad - 6) * b / mx; hctx.fillRect(4 + i * gw + 1, h - pad - bh, gw - 2, bh); });
    hctx.fillStyle = colors.muted; hctx.font = '10px "JetBrains Mono", monospace';
    hctx.textAlign = 'left'; hctx.fillText(lo.toFixed(0) + ' s', 4, h - 4);
    hctx.textAlign = 'right'; hctx.fillText((lo + nb * bw).toFixed(0) + ' s', w - 4, h - 4);
  }
  $('bRows').addEventListener('click', e => {
    const b = e.target.closest('button[data-i]'); if (!b) return;
    const r = batchRes[+b.dataset.i];
    if (r.mazeData && JSON.stringify(r.mazeData) !== JSON.stringify(S.maze)) setMaze(r.mazeData, true);
    S.seed = r.seed; $('seed').value = r.seed; store.set('seed', r.seed);
    resetSim(true);
  });
  $('btnBatch').onclick = () => {
    if (batchW) batchW.terminate();
    batchRes = []; renderBatch(false); $('bMsg').textContent = '';
    const n = Math.max(1, Math.min(500, parseInt($('bN').value) || 20));
    $('btnBatch').hidden = true; $('btnBatchStop').hidden = false; $('bProg').style.width = '0';
    batchW = makeWorker(m => {
      if (m.type === 'batchProgress') { batchRes.push(m.result); $('bProg').style.width = (m.i / m.n * 100) + '%'; renderBatch(false); }
      if (m.type === 'batchDone') { renderBatch(true); $('btnBatch').hidden = false; $('btnBatchStop').hidden = true; batchW.terminate(); batchW = null; }
    });
    const o = simOpts(); o.n = n; o.seed = Math.max(1, parseInt($('bSeed').value) || 1); o.varyMaze = $('bVary').value; o.mutations = Math.max(1, parseInt($('mzMutN').value) || 6);
    o.maxTime = Math.max(30, parseInt($('bMax').value) || 180); o.stopAtGoal = true;
    batchW.postMessage({ type: 'batch', opts: o });
  };
  $('btnBatchStop').onclick = () => { if (batchW) batchW.terminate(); batchW = null; $('btnBatch').hidden = false; $('btnBatchStop').hidden = true; renderBatch(true); };

  /* ================= Pestañas ================= */
  const TABS = ['code', 'fail', 'robot', 'track', 'batch', 'log', 'help'];
  function selectTab(k) {
    for (const t of TABS) { $('t-' + t).setAttribute('aria-selected', t === k); $('p-' + t).hidden = t !== k; }
    store.set('tab', k);
    if (k === 'batch') renderBatch(false);
  }
  for (const t of TABS) $('t-' + t).onclick = () => selectTab(t);

  /* ================= Inicio ================= */
  readColors();
  const mq = window.matchMedia('(prefers-color-scheme: dark)');
  if (mq.addEventListener) mq.addEventListener('change', readColors);
  new MutationObserver(readColors).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] });
  window.addEventListener('resize', () => { S.view = null; });
  $('optStopGoal').checked = S.prog !== 'explorar';
  showCode(); renderFails(); renderRobot(); renderHistory();
  setMaze(S.maze, true);
  selectTab(store.get('tab', 'code'));
  resetSim(false);
  requestAnimationFrame(frame);
})();
