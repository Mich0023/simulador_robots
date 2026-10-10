/* ==========================================================================
   Código del robot que se puede editar desde la página.
   LIBRERIA = equivalente a pins.h + motion.cpp + maze.cpp del firmware.
   PROGRAMAS = estrategias (equivalente a strategy.cpp + loop()).
   El programa es un generador: "yield 5" = esperar 5 ms (como delay(5)).
   ========================================================================== */
window.LIBRERIA = String.raw`// ================== CONFIGURACIÓN (como pins.h) ==================
// Ajusten estos valores igual que en el robot real.
// Robot: llanta Pololu 34 x 7 mm, N20 100:1 (~300 rpm) con encoder de 7 PPR, L293D a 6 V.
const CONF = {
  CELL: 155,            // mm de centro a centro de celda (¡midan la pista real!)
  WHEEL_D: 34,          // diámetro de llanta en mm (el que creen que tiene)
  TRACK: 90,            // distancia entre llantas en mm (centro a centro de la llanta)
  SEARCH_SPEED: 240,    // mm/s explorando (el motor de 300 rpm llega a ~330 mm/s con el L293D)
  FAST_SPEED: 280,      // mm/s en carrera rápida y con mapa previo
  ACCEL: 2000,          // mm/s²
  TURN_DPS: 300,        // °/s al girar en su lugar
  TURN_ACCEL: 3000,     // °/s²
  KP_POS: 12, KD_POS: 0.3, KV_FF: 0.66, KS_FF: 30,   // avance: PWM por mm, por mm/s; KS_FF = empuje fijo para vencer la fricción
  KP_ANG: 12, KD_ANG: 0.6, KW_FF: 0.35,              // giro: PWM por °, por °/s
  KP_WALL: 0.6,         // corrección para centrarse entre paredes
  KSLOPE: 0.5,          // corrección del rumbo midiendo si la distancia a una pared lateral crece o baja (0 = apagado)
  PWM_MIN: 45,          // PWM mínimo que vence la fricción de la caja reductora (zona muerta); se ajusta solo
  SETTLE_MS: 400,       // tiempo máximo para acomodarse al final de un movimiento
  WALL_TH: 0.40,        // umbral de pared = 40 % de la lectura con pared cerca
  PWM_MAX: 250,
  VCAP_MIN: 200,        // nunca bajar la velocidad máxima aprendida de aquí (si empuja una pared también se satura)
  BACK_ALIGN: true,     // al arrancar, recargarse en la pared de atrás para quedar derecho
  CAL_SPIN: true,       // al arrancar, girar en la celda de inicio para calibrar sensores y escala del giroscopio
  // --- vueltas suaves (curva sin detenerse) ---
  SMOOTH_TURNS: true,   // false = frenar en el centro y girar en su lugar (modo anterior)
  ARC_SPEED: 150,       // mm/s en la curva (la llanta de afuera va a ~1.9x)
  SENSE_OFS: 15,        // mm después de entrar a la celda donde se leen las paredes y se decide
  ARC_SHIFT: 0,         // 0..1: si entra a la curva cargado de un lado, corregir la distancia de salida (0 = apagado)
  ARC_SHIFT_BOTH: true, // ...sólo si ve pared de los dos lados (más confiable)
  ARC_RAMP: 0.2,        // fracción de la curva para subir/bajar la curvatura (curva suave, sin tirones)
  // --- corrección con los postes (fin de pared) ---
  EDGE_CORR: true,      // corregir la distancia cuando un sensor lateral ve que se acaba una pared
  EDGE_OFS: -9.5,       // mm: dónde está el eje cuando se detecta el fin de pared, respecto a la orilla de la celda
  // --- MPU-6050: detección de choques ---
  IMPACT: true,
  IMPACT_G: 0.45,       // g de diferencia entre lo medido y lo esperado para decir "choqué"
  IMPACT_HDG: 1.5,      // ° que se corrige el rumbo cuando roza una pared de lado
  DT: 5                 // ms del lazo de control (200 Hz)
};

// ================== ESTADO DEL MOVIMIENTO ==================
const M = { angle: 0, bias: 0, target: 0, refL: 420, refR: 420, thL: 170, thR: 170, amb: 40,
            frontRef: 420, thFront: 150, cpmm: r.cfg.countsPerRev / (Math.PI * CONF.WHEEL_D), v: 0,
            carry: 0, ax0: 0, ay0: 0, impN: 0, impacts: 0, edges: 0,
            vcap: 9999 };   // velocidad máxima que de verdad alcanzan los motores (se aprende: baja con la batería)
// Velocidad de curva: la llanta de afuera va ~1.9 veces más rápido, así que no puede pasar de lo que dan los motores.
function arcSpeed() { return Math.min(CONF.ARC_SPEED, 0.95 * M.vcap / (1 + CONF.TRACK / 2 / 52)); }

// El giro tiene prioridad: si la suma pasa del PWM máximo, se le quita al avance (así el robot sigue pudiendo corregir).
function drive(fwd, rot) {
  rot = Math.max(-CONF.PWM_MAX, Math.min(CONF.PWM_MAX, rot));
  const room = CONF.PWM_MAX - Math.abs(rot);
  M.sat = Math.abs(fwd) > room;
  if (M.sat) fwd = Math.sign(fwd) * room;
  const l = Math.max(-CONF.PWM_MAX, Math.min(CONF.PWM_MAX, fwd - rot));
  const d = Math.max(-CONF.PWM_MAX, Math.min(CONF.PWM_MAX, fwd + rot));
  r.motors(l, d);
}
// Un ciclo de control: espera DT ms e integra el giroscopio con el tiempo REAL transcurrido (como gyroUpdate()).
let lastUs = 0;
function* tick() {
  yield CONF.DT;
  const now = r.micros(), dt = lastUs ? (now - lastUs) / 1e6 : CONF.DT / 1000;
  lastUs = now;
  M.angle += (r.gyro() - M.bias) * dt / (M.gs || 1);
}
// Mediana de n lecturas: elimina lecturas basura (picos) de los sensores.
function medianIR(n) {
  const all = [];
  for (let i = 0; i < n; i++) all.push(r.ir());
  const out = {};
  for (const k of ['left', 'dleft', 'front', 'dright', 'right']) {
    const v = all.map(a => a[k]).sort((a, b) => a - b);
    out[k] = v[Math.floor(n / 2)];
  }
  return out;
}
// Lectura filtrada del ciclo de control: mediana de las últimas 3 lecturas (una por ciclo).
const HIST = [];
function filteredIR() {
  HIST.push(r.ir()); if (HIST.length > 3) HIST.shift();
  const out = {};
  for (const k of ['left', 'dleft', 'front', 'dright', 'right']) {
    const v = HIST.map(a => a[k]).sort((a, b) => a - b);
    out[k] = v[Math.floor(v.length / 2)];
  }
  return out;
}
// MPU-6050: compara la aceleración medida con la que dicen los encoders. Si las llantas siguen girando pero el
// cuerpo del robot frena de golpe (o lo empujan de lado), chocó. Regresa null, 'front', 'left' o 'right'
// ('left' = la pared que tocó está a la izquierda). Pide 2 lecturas seguidas para no confundir vibración con choque.
const ENC_HIST = [];
function impact(ayExp) {
  const e = r.enc(), now = r.millis();
  if (now - (M.impT || 0) > 1.5 * CONF.DT) ENC_HIST.length = 0;   // hubo una pausa: empezar de nuevo
  M.impT = now;
  ENC_HIST.push((e.l + e.r) / 2 / M.cpmm); if (ENC_HIST.length > 5) ENC_HIST.shift();
  if (!CONF.IMPACT || ENC_HIST.length < 5) return null;
  const h = 2 * CONF.DT / 1000;                                     // 10 ms entre muestras
  const aEnc = (ENC_HIST[4] - 2 * ENC_HIST[2] + ENC_HIST[0]) / (h * h);   // aceleración según encoders (mm/s²)
  const a = r.accel();
  const dx = a.x - M.ax0 - aEnc / 9810, dy = a.y - M.ay0 - ayExp / 9810;
  const hit = dx < -CONF.IMPACT_G || Math.abs(dy) > CONF.IMPACT_G;
  M.impN = hit ? M.impN + 1 : 0;
  if (M.impN < 2) return null;
  M.impN = 0; M.impacts++;
  if (dx < -CONF.IMPACT_G && -dx > Math.abs(dy)) return 'front';
  return dy > 0 ? 'right' : 'left';
}
// Calibración en la celda de inicio (tiene pared a la izquierda, a la derecha y atrás). Todo pasa DENTRO de la
// celda de inicio, así que no cuenta para el tiempo (el cronómetro arranca al salir de ella).
//  1) giroscopio y acelerómetro quietos
//  2) recargarse atrás: queda derecho y a la distancia exacta de la pared de atrás
//  3) girar 90° y recargarse en la pared de un lado: lo que se tuerce al recargarse = error de escala del giroscopio;
//     además queda centrado de lado
//  4) girar otros 90°: el sensor de enfrente mide la pared de atrás a la distancia exacta; los laterales, las paredes al revés
//  5) regresar -180° y recargarse otra vez (afina la escala) y medir los laterales de nuevo
//  6) referencia lateral = media geométrica de las dos lecturas de cada sensor
//  7) vueltas de 360° + recargarse hasta que quede derecho (afina la escala)
function* backAlign() {
  yield* forward(-19, 50);                      // casi hasta la pared, despacio
  const a0 = M.angle;
  for (let i = 0; i < 140; i++) { const p = -Math.min(140, 50 + i); r.motors(p, p); yield* tick(); }   // recargarse 0.7 s: la orilla trasera recta de la PCB endereza al robot
  for (let i = 0; i < 30; i++) { r.motors(0, 0); yield* tick(); }
  const twist = M.angle - a0;                   // cuánto se torció al enderezarse
  M.angle = M.target;                           // ahora sí está derecho
  M.v = 0; M.carry = 0; M.sLead = 0;
  yield* forward((CONF.CELL - 12) / 2 - 50, 100);  // del borde trasero al centro de la celda
  return twist;
}
function* calibrate() {
  let g = 0, ax = 0, ay = 0;
  for (let i = 0; i < 200; i++) { g += r.gyro(); const a = r.accel(); ax += a.x; ay += a.y; yield 2; }
  M.bias = g / 200; M.ax0 = ax / 200; M.ay0 = ay / 200; M.angle = 0; M.target = 0;
  const avg = function* (n) {
    const o = { left: 0, right: 0, front: 0, diag: 0 };
    for (let i = 0; i < n; i++) { const v = r.ir(); o.left += v.left; o.right += v.right; o.front += v.front; o.diag += (v.dleft + v.dright) / 2; yield 2; }
    for (const k in o) o[k] /= n; return o;
  };
  let A = yield* avg(20);
  M.amb = A.front < 0.5 * Math.min(A.left, A.right) ? A.front : 40;        // luz ambiente: frente abierto
  const scale = (tw, deg) => { if (Math.abs(tw) < 15) M.gs = (M.gs || 1) * (1 + tw / deg); };
  let L = A.left, R = A.right, F = 0;
  if (CONF.BACK_ALIGN) { yield* backAlign(); M.angle = 0; M.target = 0; }
  if (CONF.BACK_ALIGN && CONF.CAL_SPIN) {
    yield* turn(90);                              // la pared derecha queda atrás
    scale(yield* backAlign(), 90);
    yield* turn(90);                              // mirando a la pared de atrás
    const B = yield* avg(20);
    yield* turn(-180);
    scale(yield* backAlign(), 180);
    M.angle = 0; M.target = 0;
    A = yield* avg(20);
    // girado 180°, cada sensor ve la pared contraria: si el robot estaba cargado a un lado, una lectura sube y la otra baja
    L = Math.sqrt(A.left * B.left); R = Math.sqrt(A.right * B.right); F = B.front;
    // vueltas completas para afinar la escala (si ya está bien, sólo se hace una)
    for (let k = 0; k < 2; k++) {
      yield* turn(360);
      const tw = yield* backAlign();
      scale(tw, 360);
      M.angle = 0; M.target = 0;
      if (Math.abs(tw) < 1.5) break;
    }
    r.log('Escala del giroscopio: ' + (M.gs || 1).toFixed(4));
  }
  if (L < 120 || R < 120) { r.log('⚠ Calibración rara: L=' + L.toFixed(0) + ' R=' + R.toFixed(0) + ' (uso valores por defecto)'); L = Math.max(L, 420); R = Math.max(R, 420); }
  M.refL = L; M.refR = R;
  M.diagTouch = Math.max(500, 3.5 * A.diag);     // diagonal tan alta = la esquina del robot está tocando una pared
  M.thL = M.amb + (L - M.amb) * CONF.WALL_TH; M.thR = M.amb + (R - M.amb) * CONF.WALL_TH;
  M.frontRef = F > 1.5 * M.amb + 60 ? F : (L + R) / 2;
  M.thFront = M.amb + (M.frontRef - M.amb) * CONF.WALL_TH;
  r.log('Calibrado: giro ' + M.bias.toFixed(2) + ' °/s, acel ' + M.ax0.toFixed(2) + '/' + M.ay0.toFixed(2) + ' g, pared izq ' + L.toFixed(0) + ', der ' + R.toFixed(0) + ', frente ' + M.frontRef.toFixed(0) + ', ambiente ' + M.amb.toFixed(0) + ', batería ' + r.battery().toFixed(2) + ' V');
}
// Error de centrado: positivo = más cerca de la pared izquierda.
function wallError(v) {
  v = v || filteredIR();
  const hasL = v.left > M.thL, hasR = v.right > M.thR;
  M.bothWalls = hasL && hasR;
  const eL = Math.max(-60, Math.min(60, (v.left - M.refL) / M.refL * 100));
  const eR = Math.max(-60, Math.min(60, (v.right - M.refR) / M.refR * 100));
  if (hasL && hasR) return eL - eR;
  if (hasL) return 2 * eL;
  if (hasR) return -2 * eR;
  return 0;
}

// Distancia aproximada (mm) a la pared de enfrente según el sensor frontal (la señal cae con d²).
function frontDistance(reading) {
  const ratio = (reading - M.amb) / Math.max(30, M.frontRef - M.amb);
  if (ratio <= 0.02) return 999;
  return 43.4 / Math.sqrt(ratio) - 10;
}
// Distancia (mm) del sensor lateral a la pared, a partir de la lectura (la señal cae con el cuadrado de la distancia).
// Con el robot centrado en la celda de inicio el sensor está a ~34.5 mm de la pared.
function sideDistance(reading, ref) {
  const ratio = (reading - M.amb) / Math.max(30, ref - M.amb);
  if (ratio <= 0.05) return 999;
  return 44.5 / Math.sqrt(ratio) - 10;
}
// ¿Qué tan cargado está el robot hacia un lado? (+ = hacia la izquierda, mm). 0 si no hay paredes.
function lateralShift() {
  const v = medianIR(3), d0 = 34.5;              // distancia sensor-pared con el robot centrado
  let n = 0, sh = 0;
  if (v.left > M.thL * 1.2) { const d = sideDistance(v.left, M.refL); if (d < 60) { sh += d0 - d; n++; } }
  if (v.right > M.thR * 1.2) { const d = sideDistance(v.right, M.refR); if (d < 60) { sh += d - d0; n++; } }
  if (CONF.ARC_SHIFT_BOTH && n < 2) return 0;
  return n ? Math.max(-25, Math.min(25, sh / n)) : 0;
}
// Estimador del rumbo con una pared lateral: ajusta una recta a (distancia avanzada, distancia a la pared).
// Si la distancia a la pared cambia mientras avanzamos, el robot va chueco: comparamos con lo que dice el giroscopio.
function slopeWin() { return { n: 0, sx: 0, sy: 0, sxx: 0, sxy: 0, syy: 0, sg: 0, x0: 0 }; }
function slopeAdd(w, x, y, g) { if (!w.n) w.x0 = x; x -= w.x0; w.n++; w.sx += x; w.sy += y; w.sxx += x * x; w.sxy += x * y; w.syy += y * y; w.sg += g; }
function slopeFit(w) {
  const den = w.n * w.sxx - w.sx * w.sx;
  if (den < 1e-6) return { m: 0, rms: 99 };
  const m = (w.n * w.sxy - w.sx * w.sy) / den, b = (w.sy - m * w.sx) / w.n;
  const sse = w.syy - 2 * m * w.sxy - 2 * b * w.sy + m * m * w.sxx + 2 * m * b * w.sx + w.n * b * b;
  return { m, rms: Math.sqrt(Math.max(0, sse) / w.n) };   // rms: qué tan recta es (si es grande, hay hueco o poste)
}
// Avanza "mm" con perfil trapezoidal. Opciones:
//   walls: centrarse con las paredes   vEnd: velocidad al terminar (para encadenar sin frenar)
//   extend(pos, total, v): se llama en cada ciclo con la distancia real (encoders); si regresa un número, ese es el nuevo total
//   guard: si el sensor frontal ve una pared que no esperábamos, frena y se detiene a la distancia correcta
//   edges: mm desde el inicio hasta la siguiente orilla de celda (para corregir la distancia con el fin de las paredes)
// Regresa false si detecta un choque (MPU o llantas que no avanzan).
function* forward(mm, vmax, o) {
  o = o || {};
  const sg = mm < 0 ? -1 : 1;                       // reversa: mismo perfil, motores al revés (sin paredes ni choques)
  if (sg < 0) { mm = -mm; o = { vEnd: 0 }; M.v = 0; M.carry = 0; M.sLead = 0; }
  const dt = CONF.DT / 1000, e0 = r.enc(), base = (e0.l + e0.r) / 2;
  let off = M.carry || 0, s0 = M.sLead || 0; M.carry = 0; M.sLead = 0;   // dónde quedaron robot y perfil del tramo anterior
  let target = mm, s = s0, v = M.v, prevE = null, prevA = null, settle = 0, stall = 0, lastPos = off, near = 0, pos = off;
  let vEnd = o.vEnd || 0, satT = 0, vAvg = v, touch = 0;
  vmax = Math.min(vmax, M.vcap); vEnd = Math.min(vEnd, M.vcap);
  const seen = { left: 0, right: 0 }, gone = { left: 0, right: 0 }, win = { left: slopeWin(), right: slopeWin() };
  HIST.length = 0; M.blocked = false; M.impact = null;
  for (let it = 0; it < 20000; it++) {
    yield* tick();
    const e = r.enc(), ir = filteredIR();
    pos = sg * ((e.l + e.r) / 2 - base) / M.cpmm + off; M.lastDist = sg * pos;
    const vm = (pos - lastPos) / dt;
    // fin de pared: el sensor lateral pasa el poste -> sabemos exactamente dónde está el robot
    if (o.edges != null && CONF.EDGE_CORR && v > 80) {
      for (const k of ['left', 'right']) {
        const th = k === 'left' ? M.thL : M.thR;
        if (ir[k] > th) { seen[k] += pos - lastPos; gone[k] = 0; }
        else if (seen[k] > 40 && ++gone[k] === 2) {
          const n = Math.round((pos - o.edges - CONF.EDGE_OFS) / CONF.CELL);
          const err = o.edges + n * CONF.CELL + CONF.EDGE_OFS - pos;
          if (CONF.DEBUG) r.log('EDGE ' + k + ' err ' + err.toFixed(1));
          if (Math.abs(err) < 25) { off += err; pos += err; s += err; M.edges++; win.left = slopeWin(); win.right = slopeWin(); }
          seen[k] = 0;
        } else if (gone[k] > 2) seen[k] = 0;
      }
    }
    lastPos = pos;
    if (o.extend) { const nt = o.extend(pos, target, v); if (typeof nt === 'number') target = nt; }
    if (o.guard && v > 60) {
      const d = frontDistance(ir.front), stopAt = d - 33;        // 33 mm = distancia con el robot centrado
      near = d < 33 + v * v / (2 * CONF.ACCEL) + 30 ? near + 1 : 0;
      if (near >= 3 && s + stopAt < target - 20) {
        target = s + Math.max(0, stopAt); vEnd = 0; M.blocked = true;
        r.log('⚠ Pared inesperada enfrente: freno');
      }
    }
    const rem = target - s;
    if (rem > 0) {
      const brake = (v * v - vEnd * vEnd) / (2 * CONF.ACCEL);
      if (rem <= brake) v = Math.max(v - CONF.ACCEL * dt, Math.max(vEnd, 60)); else v = Math.min(v + CONF.ACCEL * dt, Math.max(vmax, vEnd));
      s += v * dt; if (s >= target) { s = target; v = vEnd; }
    } else if (vEnd > 0) {
      // el perfil ya llegó; seguimos a vEnd hasta que el ROBOT llegue (si va atrasado, no decidir antes de tiempo)
      s += vEnd * dt;
      if (pos >= target) { M.v = vEnd; M.carry = pos - target; M.sLead = s - target; return true; }
    }
    else { settle += CONF.DT; if ((Math.abs(s - pos) < 1.5 && settle > 40) || settle > CONF.SETTLE_MS) break; }
    const eP = s - pos;
    let fwd = CONF.KP_POS * eP + (prevE == null ? 0 : CONF.KD_POS * (eP - prevE) / dt) + CONF.KV_FF * v + (v > 5 ? CONF.KS_FF : 0); prevE = eP;
    const pm = M.pwmMin || CONF.PWM_MIN;
    if (rem <= 0 && Math.abs(eP) > 1 && Math.abs(fwd) < pm) fwd = Math.sign(eP) * pm;   // zona muerta
    const eA = M.target - M.angle;
    let rot = CONF.KP_ANG * eA + (prevA == null ? 0 : CONF.KD_ANG * (eA - prevA) / dt); prevA = eA;
    if (o.walls && v > 80) {
      const we = wallError(ir);
      rot -= CONF.KP_WALL * we;
      // rumbo por la pendiente de la distancia a cada pared (funciona aunque sólo haya pared de un lado)
      if (CONF.KSLOPE > 0) for (const k of ['left', 'right']) {
        const ref = k === 'left' ? M.refL : M.refR, th = k === 'left' ? M.thL : M.thR, w = win[k];
        const dW = sideDistance(ir[k], ref);
        if (ir[k] > th * 1.2 && dW < 60) slopeAdd(w, pos, dW, M.angle - M.target);
        else { win[k] = slopeWin(); continue; }
        if (w.n >= 30 && pos - w.x0 >= 50) {
          const fit = slopeFit(w), alpha = (k === 'left' ? -1 : 1) * fit.m * 180 / Math.PI;   // + = apunta a la izquierda (atan(m) ≈ m)
          const dif = alpha - w.sg / w.n;
          // sólo si la pared se ve recta (sin huecos) y la diferencia es creíble
          if (fit.rms < 1.2 && Math.abs(dif) < 5) {
            const c = Math.max(-2, Math.min(2, dif)) * CONF.KSLOPE;
            M.angle += c; M.slopeCorr = (M.slopeCorr || 0) + Math.abs(c);
            if (CONF.DEBUG) r.log('SLOPE ' + k + ' ' + alpha.toFixed(2) + ' gyro ' + (w.sg / w.n).toFixed(2) + ' rms ' + fit.rms.toFixed(2) + ' corr ' + c.toFixed(2));
          }
          win[k] = slopeWin();
        }
      }
    }
    drive(sg * fwd, rot);
    if (sg < 0) continue;
    // ¿los motores ya no dan más? (PWM al tope, velocidad constante pedida, pero el robot va más lento): aprender el límite
    vAvg += 0.1 * (vm - vAvg);
    if (M.sat && rem > 0 && v >= vmax - 1 && vAvg < v - 15) {
      satT += CONF.DT;
      if (satT >= 150) { M.vcap = Math.min(M.vcap, Math.max(CONF.VCAP_MIN, Math.round(vAvg - 5))); vmax = M.vcap; vEnd = Math.min(vEnd, M.vcap); satT = 0; r.log('Motores al límite: velocidad máxima ' + M.vcap + ' mm/s'); }
    } else satT = 0;
    // MPU: ¿la aceleración medida es la que pedimos?
    const wz = (r.gyro() - M.bias) * Math.PI / 180;
    const hit = impact(vm * wz);
    if (hit === 'front' && vm > 40) { drive(0, 0); M.v = 0; M.impact = 'front'; r.log('💥 Choque de frente (MPU): freno'); return false; }
    if (hit === 'left' || hit === 'right') {
      // rozó una pared de lado: el rumbo estimado está chueco hacia esa pared, corregirlo para alejarse
      M.angle += (hit === 'left' ? 1 : -1) * CONF.IMPACT_HDG;
      r.log('💥 Rozó la pared ' + (hit === 'left' ? 'izquierda' : 'derecha') + ' (MPU): corrijo el rumbo');
    }
    if (Math.abs(fwd) > 150 && Math.abs(vm) < 10) stall += CONF.DT; else stall = 0;
    if (stall > 300) { drive(0, 0); M.v = 0; r.log('⚠ Choque: las llantas no avanzan'); return false; }
    // esquina del robot pegada a una pared (las llantas pueden seguir girando y patinando)
    if (v > 50 && M.diagTouch && (ir.dleft > M.diagTouch || ir.dright > M.diagTouch)) { if (++touch >= 4) { drive(0, 0); M.v = 0; M.impact = 'diag'; r.log('⚠ La esquina del robot está tocando una pared'); return false; } }
    else touch = 0;
    // empujando una pared: el sensor frontal está saturado aunque los encoders digan que avanzamos
    if (v > 0 && ir.front > M.frontRef * 1.6 && pos > 20) { drive(0, 0); M.v = 0; r.log('⚠ Pegado a una pared enfrente'); return false; }
  }
  drive(0, 0); M.v = 0; M.lastDist = sg * pos;
  return true;
}
// Gira en su lugar (positivo = izquierda) usando el giroscopio.
function* turn(deg) {
  const dt = CONF.DT / 1000, start = M.target, dir = Math.sign(deg), tot = Math.abs(deg);
  M.target += deg;
  const e0 = r.enc(), base = (e0.l + e0.r) / 2;
  let a = 0, w = 0, prevE = null, prevP = null, settle = 0;
  for (let i = 0; i < 400; i++) {
    yield* tick();
    if (a < tot) {
      const br = w * w / (2 * CONF.TURN_ACCEL);
      if (tot - a <= br) w = Math.max(w - CONF.TURN_ACCEL * dt, 30); else w = Math.min(w + CONF.TURN_ACCEL * dt, CONF.TURN_DPS);
      a += w * dt; if (a >= tot) { a = tot; w = 0; }
    } else {
      settle += CONF.DT;
      const rate = Math.abs(r.gyro() - M.bias);
      if (Math.abs(M.target - M.angle) < 1.5 && rate < 15 && settle > 40) break;
      if (settle > CONF.SETTLE_MS) break;
      // si no se mueve, subir poco a poco el empuje mínimo (zona muerta del motor real)
      if (settle > 80 && Math.abs(M.target - M.angle) > 1.5) M.pwmMin = Math.min(110, (M.pwmMin || CONF.PWM_MIN) + 0.5);
    }
    const e = start + dir * a - M.angle;
    let rot = CONF.KP_ANG * e + (prevE == null ? 0 : CONF.KD_ANG * (e - prevE) / dt) + dir * (w * CONF.KW_FF + (w > 5 ? CONF.KS_FF : 0)); prevE = e;
    // al final, si queda error, empujar con el PWM mínimo para vencer la zona muerta
    const pm = M.pwmMin || CONF.PWM_MIN;
    if (a >= tot && Math.abs(e) > 0.7 && Math.abs(rot) < pm) rot = Math.sign(e) * pm;
    const en = r.enc(), p = ((en.l + en.r) / 2 - base) / M.cpmm;
    const fwd = -CONF.KP_POS * p - (prevP == null ? 0 : CONF.KD_POS * (p - prevP) / dt); prevP = p;
    drive(fwd, rot);
  }
  drive(0, 0); M.v = 0; M.carry = 0; M.sLead = 0;
}
// ---------- Vuelta suave: curva de 90° sin detenerse ----------
// Empieza SENSE_OFS mm después de entrar a la celda y termina mirando a la celda de al lado, SENSE_OFS mm antes de su orilla.
// La curvatura sube y baja en rampa (ARC_RAMP), así las llantas no reciben un tirón.
// El giroscopio sigue el ángulo de la curva con PD + anticipación (feedforward); los encoders, la distancia.
const ARC_UNIT = (function () {             // avance de una curva de 90° de largo 1 (forma normalizada)
  const N = 2000, RF = CONF.ARC_RAMP, k0 = (Math.PI / 2) / (1 - RF);
  let th = 0, x = 0;
  for (let i = 0; i < N; i++) { const s = (i + 0.5) / N; const k = s < RF ? k0 * s / RF : s > 1 - RF ? k0 * (1 - s) / RF : k0; x += Math.cos(th) / N; th += k / N; }
  return x;
})();
// X = cuánto avanza (y se recorre de lado) la curva; normalmente CELL/2 - SENSE_OFS.
function* arc(deg, v, X) {
  const dt = CONF.DT / 1000, dir = Math.sign(deg), tot = Math.abs(deg), RF = CONF.ARC_RAMP;
  const L = (X || CONF.CELL / 2 - CONF.SENSE_OFS) / ARC_UNIT * tot / 90, k0 = tot / (L * (1 - RF));   // °/mm en el centro
  const ang = s => s < RF * L ? k0 * s * s / (2 * RF * L)
    : s <= L * (1 - RF) ? k0 * (RF * L / 2 + s - RF * L) : tot - k0 * (L - s) * (L - s) / (2 * RF * L);
  const curv = s => s < RF * L ? k0 * s / (RF * L) : s > L * (1 - RF) ? k0 * (L - s) / (RF * L) : k0;
  const start = M.target, e0 = r.enc(), base = (e0.l + e0.r) / 2;
  let off = M.carry || 0, s = M.sLead || 0; M.carry = 0; M.sLead = 0;
  let prevE = null, prevA = null, pos = off, lastPos = off;
  HIST.length = 0; M.impact = null;
  for (let it = 0; it < 2000; it++) {
    yield* tick();
    s += v * dt;
    const e = r.enc();
    pos = ((e.l + e.r) / 2 - base) / M.cpmm + off;
    const vm = (pos - lastPos) / dt; lastPos = pos;
    // el ángulo se sigue según la distancia REAL recorrida: así la forma de la curva no depende de si el robot va atrasado
    const sg = Math.max(0, Math.min(L, pos)), wRef = Math.max(vm, 0) * curv(sg);   // °/s que pide la curva
    const eP = s - pos;
    const fwd = CONF.KP_POS * eP + (prevE == null ? 0 : CONF.KD_POS * (eP - prevE) / dt) + CONF.KV_FF * v + CONF.KS_FF; prevE = eP;
    const eA = start + dir * ang(sg) - M.angle;
    const rot = CONF.KP_ANG * eA + (prevA == null ? 0 : CONF.KD_ANG * (eA - prevA) / dt) + dir * wRef * CONF.KW_FF; prevA = eA;
    drive(fwd, rot);
    const wz = (r.gyro() - M.bias) * Math.PI / 180;
    const hit = impact(vm * wz);
    if (hit) {
      // choque en la curva: detenerse, terminar el giro en su lugar y seguir derecho desde ahí
      drive(0, 0); M.v = 0; M.impact = hit; M.arcDone = s / L;
      r.log('💥 Choque en la curva (' + hit + ', MPU): termino el giro en su lugar');
      M.target = start + dir * ang(sg);
      yield 60;
      yield* turn(deg - dir * ang(sg));
      return false;
    }
    if (pos >= L) break;
  }
  M.target = start + deg; M.carry = pos - L; M.sLead = s - L; M.v = v;
  return true;
}
// Si hay pared enfrente, se acomoda a la distancia correcta (corrige el error de los encoders).
function* alignFront() {
  for (let i = 0; i < 60; i++) {
    const f = medianIR(3).front;
    if (f < M.thFront) break;
    const err = (M.frontRef - f) / M.frontRef * 40;     // aprox. mm que faltan
    if (Math.abs(err) < 1.5) break;
    const eA = M.target - M.angle;
    const pm = M.pwmMin || CONF.PWM_MIN;
    let f2 = Math.max(-90, Math.min(90, err * 8)); if (Math.abs(f2) < pm) f2 = Math.sign(err) * pm;
    drive(f2, CONF.KP_ANG * eA);
    yield* tick();
  }
  drive(0, 0); M.v = 0; M.carry = 0; M.sLead = 0;
}
// Re-centrarse en la celda usando las paredes: la de enfrente da la posición hacia adelante y una lateral
// (girando para verla de frente) da la posición de lado. Sirve después de un choque, cuando ya no sabemos bien dónde estamos.
function* recenter() {
  const v = medianIR(5);
  if (v.front > M.thFront) yield* alignFront();
  if (v.left > M.thL) { yield* turn(90); yield* alignFront(); yield* turn(-90); }
  else if (v.right > M.thR) { yield* turn(-90); yield* alignFront(); yield* turn(90); }
}
// Después de un choque: se echa para atrás y recupera el rumbo.
function* recover() {
  M.v = 0; M.carry = 0; M.sLead = 0;
  yield 80;
  yield* forward(-30, 120);
  yield* turn(0);
}

// ================== MAPA Y FLOOD FILL (como maze.cpp) ==================
// Direcciones: 0 = Norte (+y), 1 = Este (+x), 2 = Sur, 3 = Oeste. (0,0) = esquina inferior izquierda.
const DX = [0, 1, 0, -1], DY = [1, 0, -1, 0];
class Maze {
  constructor(cols, rows) {
    this.cols = cols; this.rows = rows;
    this.cells = new Uint8Array(cols * rows);   // bits 0-3: hay pared, bits 4-7: pared medida
    this.dist = new Int16Array(cols * rows);
    for (let x = 0; x < cols; x++) { this.setWall(x, 0, 2, true); this.setWall(x, rows - 1, 0, true); }
    for (let y = 0; y < rows; y++) { this.setWall(0, y, 3, true); this.setWall(cols - 1, y, 1, true); }
  }
  inside(x, y) { return x >= 0 && y >= 0 && x < this.cols && y < this.rows; }
  i(x, y) { return y * this.cols + x; }
  hasWall(x, y, d) { return (this.cells[this.i(x, y)] >> d) & 1; }
  isKnown(x, y, d) { return (this.cells[this.i(x, y)] >> (4 + d)) & 1; }
  setOne(x, y, d, p, known) {
    const k = this.i(x, y);
    if (known) this.cells[k] |= 16 << d;
    if (p) this.cells[k] |= 1 << d; else this.cells[k] &= ~(1 << d);
  }
  setWall(x, y, d, p, known = true) {
    if (!this.inside(x, y)) return;
    this.setOne(x, y, d, p, known);
    const nx = x + DX[d], ny = y + DY[d];
    if (this.inside(nx, ny)) this.setOne(nx, ny, (d + 2) & 3, p, known);
  }
  // Carga un mapa "previo" (lo que creemos que es la pista) como paredes NO medidas.
  loadPrior(m) {
    for (let y = 0; y < this.rows; y++) for (let x = 0; x < this.cols; x++) {
      const w = [m.h[y + 1][x], m.v[y][x + 1], m.h[y][x], m.v[y][x]];
      for (let d = 0; d < 4; d++) if (!this.isKnown(x, y, d)) this.setOne(x, y, d, w[d] === '1' || w[d] === 1, false);
    }
  }
  blocked(x, y, d, unknownIsWall) { return this.hasWall(x, y, d) || (unknownIsWall && !this.isKnown(x, y, d)); }
  flood(goals, unknownIsWall) {
    this.dist.fill(-1); const q = [];
    for (const g of goals) { this.dist[this.i(g[0], g[1])] = 0; q.push(g[0], g[1]); }
    for (let h = 0; h < q.length; h += 2) {
      const x = q[h], y = q[h + 1], nd = this.dist[this.i(x, y)] + 1;
      for (let d = 0; d < 4; d++) {
        if (this.blocked(x, y, d, unknownIsWall)) continue;
        const nx = x + DX[d], ny = y + DY[d];
        if (!this.inside(nx, ny) || this.dist[this.i(nx, ny)] >= 0) continue;
        this.dist[this.i(nx, ny)] = nd; q.push(nx, ny);
      }
    }
  }
  // Mejor dirección: la vecina con menor distancia; en empate, seguir derecho.
  bestDir(x, y, heading, unknownIsWall) {
    let best = -1, bd = 1e9;
    for (const d of [heading, (heading + 3) & 3, (heading + 1) & 3, (heading + 2) & 3]) {
      if (this.blocked(x, y, d, unknownIsWall)) continue;
      const nx = x + DX[d], ny = y + DY[d];
      if (!this.inside(nx, ny)) continue;
      const v = this.dist[this.i(nx, ny)];
      if (v >= 0 && v < bd) { bd = v; best = d; }
    }
    return best;
  }
  isGoal(x, y, goals) { return goals.some(g => g[0] === x && g[1] === y); }
}
function publish(mz, p) { r.show({ cols: mz.cols, rows: mz.rows, cells: Array.from(mz.cells), dist: Array.from(mz.dist), pose: { x: p.x, y: p.y, h: p.h } }); }

// Lee paredes (mediana de 5 lecturas) y las guarda en el mapa.
// frontDist: a qué distancia (mm) estaría la pared de enfrente; si se lee en movimiento, la pared está más lejos
// y la señal es más débil, así que el umbral se ajusta (la señal cae con el cuadrado de la distancia).
const FORCED = new Set();   // paredes "invisibles" donde ya chocamos 2 veces: los sensores no las borran
function senseCell(mz, p, frontDist) {
  const v = medianIR(5);
  const keep = d => FORCED.has(p.x + ',' + p.y + ',' + d);
  if (frontDist) {
    // de lejos la señal es débil: sólo se anota si es claro; si es dudoso se deja como "no medida"
    const k = Math.pow(43.4 / (frontDist + 10), 2), exp = (M.frontRef - M.amb) * k;
    if (v.front > M.amb + 0.6 * exp + 25) mz.setWall(p.x, p.y, p.h, true);
    else if (v.front < M.amb + 0.25 * exp && !keep(p.h)) mz.setWall(p.x, p.y, p.h, false);
  } else mz.setWall(p.x, p.y, p.h, v.front > M.thFront || keep(p.h));
  mz.setWall(p.x, p.y, (p.h + 3) & 3, v.left > M.thL || keep((p.h + 3) & 3));
  mz.setWall(p.x, p.y, (p.h + 1) & 3, v.right > M.thR || keep((p.h + 1) & 3));
}
function* faceDir(p, d) {
  const diff = (d - p.h + 4) & 3;
  if (diff === 1) yield* turn(-90); else if (diff === 3) yield* turn(90); else if (diff === 2) yield* turn(180);
  p.h = d;
}

// Búsqueda con flood fill hacia "goals" (modo anterior): avanza sin detenerse en las rectas,
// pero frena en el centro de la celda para cada vuelta y gira en su lugar.
function* searchStop(mz, p, goals, speed) {
  senseCell(mz, p);
  for (let steps = 0; steps < 600; steps++) {
    if (mz.isGoal(p.x, p.y, goals)) { publish(mz, p); return true; }
    mz.flood(goals, false);
    publish(mz, p);
    let d = mz.bestDir(p.x, p.y, p.h, false);
    if (d < 0) { r.log('Sin salida: el mapa dice que no hay camino'); return false; }
    if (!mz.isKnown(p.x, p.y, d)) { yield* faceDir(p, d); senseCell(mz, p); continue; }
    if (d !== p.h) { yield* faceDir(p, d); yield* alignFront(); }
    let cellsDone = 0;
    const ok = yield* forward(CONF.CELL, speed, {
      walls: true, guard: true,
      extend: (s, target, v) => {   // s = distancia real recorrida según encoders
        const before = v * v / (2 * CONF.ACCEL) + 12, next = (cellsDone + 1) * CONF.CELL;
        if (s >= next - before && next <= target + 1) {
          p.x += DX[p.h]; p.y += DY[p.h]; cellsDone++;
          mz.setWall(p.x, p.y, (p.h + 2) & 3, false);
          senseCell(mz, p, (CONF.CELL - 12) / 2 + before - 38);
          if (mz.isGoal(p.x, p.y, goals)) return;
          mz.flood(goals, false);
          const nd = mz.bestDir(p.x, p.y, p.h, false);
          if (nd === p.h && !mz.hasWall(p.x, p.y, p.h)) return target + CONF.CELL;   // seguir derecho
        }
      }
    });
    const moved = Math.max(0, Math.round(M.lastDist / CONF.CELL));
    if (moved !== cellsDone) {
      const back = cellsDone - moved;
      p.x -= DX[p.h] * back; p.y -= DY[p.h] * back;
      if (back) r.log('Ajuste de posición: avanzó ' + moved + ' celdas, no ' + cellsDone);
    }
    if (!ok) { yield* recover(); }
    senseCell(mz, p);
    if (mz.hasWall(p.x, p.y, p.h)) yield* alignFront();
  }
  return false;
}

// Búsqueda con vueltas suaves. El robot decide en el "punto de sensado" de cada celda
// (SENSE_OFS mm después de entrar): ahí los sensores laterales ya ven las paredes de esa celda.
//   recta -> sigue a velocidad de búsqueda      vuelta -> curva de 90° sin detenerse
//   callejón -> frena en el centro, gira 180°   pared inesperada enfrente -> frena en el centro y replanea
// known = true: sólo usa paredes ya medidas (carrera rápida).
function* searchSmooth(mz, p, goals, speed, known) {
  const SO = CONF.SENSE_OFS, HALF = CONF.CELL / 2;
  let moving = false;                    // false: detenido en el centro de p;  true: en el punto de sensado de p
  const hits = {};                        // choques por celda y dirección
  let resets = 0;
  senseCell(mz, p);
  // ¿Qué haremos en la celda de enfrente según lo que creemos ahora? Para llegar con la velocidad correcta.
  const nextSpeed = () => {
    const nx = p.x + DX[p.h], ny = p.y + DY[p.h];
    if (!mz.inside(nx, ny) || mz.isGoal(nx, ny, goals)) return arcSpeed();
    return mz.bestDir(nx, ny, p.h, known) === p.h ? speed : arcSpeed();
  };
  // Avanza hasta el punto de sensado de la celda de enfrente. startOfs = dónde empieza, medido desde el centro de p.
  function* cruise(dist, startOfs) {
    const ok = yield* forward(dist, speed, { walls: true, guard: true, vEnd: nextSpeed(), edges: dist - SO });
    if (ok && !M.blocked) {
      p.x += DX[p.h]; p.y += DY[p.h];
      mz.setWall(p.x, p.y, (p.h + 2) & 3, false);
      return true;
    }
    // se detuvo antes (pared inesperada o choque): ¿en qué celda quedó?
    const k = Math.max(0, Math.round((startOfs + M.lastDist) / CONF.CELL));
    for (let i = 0; i < k; i++) { mz.setWall(p.x, p.y, p.h, false); p.x += DX[p.h]; p.y += DY[p.h]; }
    if (!ok) {
      yield* recover();
      yield* recenter();
      // si ya chocó dos veces saliendo de esta misma celda hacia el mismo lado, tratarlo como pared y buscar otra ruta
      const key = p.x + ',' + p.y + ',' + p.h;
      hits[key] = (hits[key] || 0) + 1;
      if (hits[key] >= 2) {
        mz.setWall(p.x, p.y, p.h, true); FORCED.add(key);
        const nx = p.x + DX[p.h], ny = p.y + DY[p.h]; FORCED.add(nx + ',' + ny + ',' + ((p.h + 2) & 3));
        r.log('Choqué 2 veces aquí: lo marco como pared');
      }
    }
    return false;
  }
  for (let steps = 0; steps < 900; steps++) {
    if (moving) senseCell(mz, p, CONF.CELL - 6 - SO - 38);
    if (mz.isGoal(p.x, p.y, goals)) {
      if (moving) yield* forward(HALF - SO, Math.max(M.v, 100), { walls: true });   // frenar en el centro de la meta
      publish(mz, p); return true;
    }
    mz.flood(goals, known); publish(mz, p);
    const d = mz.bestDir(p.x, p.y, p.h, known);
    if (d < 0) {
      if (moving) { yield* forward(HALF - SO, Math.max(M.v, 100)); moving = false; }
      if (resets++ >= 2) { r.log('Sin salida: el mapa dice que no hay camino'); return false; }
      // el mapa quedó inconsistente (seguramente nos perdimos): olvidar lo medido y volver a explorar desde aquí
      r.log('⚠ El mapa no tiene salida: lo borro y vuelvo a explorar');
      const fresh = new Maze(mz.cols, mz.rows); mz.cells.set(fresh.cells);
      FORCED.clear(); for (const k in hits) delete hits[k];
      senseCell(mz, p); if (mz.hasWall(p.x, p.y, p.h)) yield* alignFront();
      continue;
    }
    const rel = (d - p.h + 4) & 3;
    if (moving) {
      if (rel === 0) { moving = yield* cruise(CONF.CELL, SO - HALF); }
      else if (rel !== 2 && M.v < arcSpeed() * 1.15) {
        // vuelta suave. Si el robot va cargado hacia un lado, al girar ese error se vuelve "adelantado/atrasado"
        // en la dirección nueva: lo medimos con las paredes laterales y lo pasamos a la cuenta de distancia.
        const shift = lateralShift();                       // + = cargado a la izquierda
        const ok = yield* arc(rel === 3 ? 90 : -90, arcSpeed());
        if (ok) M.carry += (rel === 3 ? 1 : -1) * shift * CONF.ARC_SHIFT;
        p.h = d;
        if (ok) {
          // de donde acaba la curva al punto de sensado de la celda nueva: 2 x SENSE_OFS
          const ok2 = yield* forward(2 * SO, arcSpeed(), { walls: true, vEnd: nextSpeed() });
          if (ok2) { p.x += DX[p.h]; p.y += DY[p.h]; mz.setWall(p.x, p.y, (p.h + 2) & 3, false); }
          else { yield* recover(); moving = false; senseCell(mz, p); }
        } else {
          // se interrumpió la curva: quedó girado cerca del centro de p; seguir desde ahí
          moving = yield* cruise(HALF + SO, 0);
        }
      } else {
        // callejón sin salida, o venía muy rápido para la curva: frenar en el centro
        yield* forward(HALF - SO, Math.max(M.v, 100), { walls: true });
        moving = false;
        senseCell(mz, p);
        if (mz.hasWall(p.x, p.y, p.h)) yield* alignFront();
      }
    } else {
      // detenido en el centro de la celda
      if (!mz.isKnown(p.x, p.y, d)) { yield* faceDir(p, d); senseCell(mz, p); continue; }
      if (rel) yield* faceDir(p, d);
      moving = yield* cruise(HALF + SO, 0);
      if (!moving) { senseCell(mz, p); if (mz.hasWall(p.x, p.y, p.h)) yield* alignFront(); }
    }
    if (!moving && M.blocked) { senseCell(mz, p); if (mz.hasWall(p.x, p.y, p.h)) yield* alignFront(); }
  }
  return false;
}
function* searchTo(mz, p, goals, speed) {
  return CONF.SMOOTH_TURNS ? yield* searchSmooth(mz, p, goals, speed, false) : yield* searchStop(mz, p, goals, speed);
}
// ¿Ya se conoce el camino más corto? (optimista == solo paredes medidas)
function shortestKnown(mz, from, goals) {
  mz.flood(goals, false); const a = mz.dist[mz.i(from.x, from.y)];
  mz.flood(goals, true); const b = mz.dist[mz.i(from.x, from.y)];
  return b >= 0 && a === b;
}
// Carrera rápida por el camino ya conocido.
function* speedRun(mz, p, goals) {
  mz.flood(goals, true);
  if (mz.dist[mz.i(p.x, p.y)] < 0) { r.log('No hay ruta conocida'); return false; }
  if (CONF.SMOOTH_TURNS) return yield* searchSmooth(mz, p, goals, CONF.FAST_SPEED, true);
  while (!mz.isGoal(p.x, p.y, goals)) {
    const d = mz.bestDir(p.x, p.y, p.h, true);
    if (d < 0) return false;
    yield* faceDir(p, d);
    let n = 0, x = p.x, y = p.y;
    while (!mz.hasWall(x, y, d) && mz.isKnown(x, y, d)) {
      const nx = x + DX[d], ny = y + DY[d];
      if (mz.dist[mz.i(nx, ny)] !== mz.dist[mz.i(x, y)] - 1) break;
      x = nx; y = ny; n++;
      if (mz.isGoal(x, y, goals) || mz.bestDir(x, y, d, true) !== d) break;
    }
    const ok = yield* forward(n * CONF.CELL, CONF.FAST_SPEED, { walls: true });
    if (!ok) { yield* recover(); return false; }
    p.x = x; p.y = y; publish(mz, p);
  }
  return true;
}
`;

window.PROGRAMAS = {
  directo: {
    nombre: 'Flood fill directo a la meta (1 intento)',
    desc: 'No conoce la pista. Explora con flood fill sólo lo necesario para llegar a la meta. Es lo más seguro si la competencia mide un solo recorrido.',
    code: String.raw`// Estrategia 1: flood fill directo a la meta (no conoce la pista)
function* main(r) {
  const mz = new Maze(r.cfg.cols, r.cfg.rows);
  const p = { x: r.cfg.start.x, y: r.cfg.start.y, h: r.cfg.start.dir };
  yield* calibrate();
  const ok = yield* searchTo(mz, p, r.cfg.goal, CONF.SEARCH_SPEED);
  r.log(ok ? '¡Meta alcanzada!' : 'No llegó');
  drive(0, 0);
}
`
  },
  previo: {
    nombre: 'Mapa previo + verificación',
    desc: 'Carga la pista de la convocatoria como "creencia". Va directo por la ruta más corta del dibujo, pero mide cada pared y vuelve a planear si la pista real es distinta.',
    code: String.raw`// Estrategia 2: usar el dibujo de la convocatoria como mapa previo, verificando con sensores.
// Si la pista real coincide, va directo por la ruta más corta; si no, se corrige sola.
function* main(r) {
  const mz = new Maze(r.cfg.cols, r.cfg.rows);
  mz.loadPrior(r.cfg.reference);          // paredes "creídas" (no medidas)
  const p = { x: r.cfg.start.x, y: r.cfg.start.y, h: r.cfg.start.dir };
  yield* calibrate();
  const ok = yield* searchTo(mz, p, r.cfg.goal, CONF.FAST_SPEED);
  r.log(ok ? '¡Meta alcanzada!' : 'No llegó');
  drive(0, 0);
}
`
  },
  explorar: {
    nombre: 'Explorar ida y vuelta + carrera rápida',
    desc: 'Para competencias con varios intentos: va a la meta, regresa por otro camino hasta confirmar la ruta más corta y luego hace la carrera rápida.',
    code: String.raw`// Estrategia 3: explorar (ida y vuelta) y luego carrera rápida.
// El simulador mide cada salida desde INICIO hasta la META por separado.
function* main(r) {
  const mz = new Maze(r.cfg.cols, r.cfg.rows);
  const start = [[r.cfg.start.x, r.cfg.start.y]];
  const p = { x: r.cfg.start.x, y: r.cfg.start.y, h: r.cfg.start.dir };
  yield* calibrate();
  for (let viaje = 1; viaje <= 3; viaje++) {
    if (!(yield* searchTo(mz, p, r.cfg.goal, CONF.SEARCH_SPEED))) return;
    if (!(yield* searchTo(mz, p, start, CONF.SEARCH_SPEED))) return;
    if (shortestKnown(mz, p, r.cfg.goal)) { r.log('Ruta más corta confirmada en el viaje ' + viaje); break; }
  }
  yield* faceDir(p, r.cfg.start.dir);
  yield 500;                                  // pausa en el inicio
  const ok = yield* speedRun(mz, p, r.cfg.goal);
  r.log(ok ? 'Carrera rápida terminada' : 'Falló la carrera rápida');
  drive(0, 0);
}
`
  },
  pared: {
    nombre: 'Seguir la pared derecha (referencia)',
    desc: 'Estrategia simple para comparar. Puede dar vueltas sin llegar si la meta no está pegada a una pared conectada con el inicio.',
    code: String.raw`// Estrategia de referencia: regla de la mano derecha.
function* main(r) {
  const p = { x: r.cfg.start.x, y: r.cfg.start.y, h: r.cfg.start.dir };
  const mz = new Maze(r.cfg.cols, r.cfg.rows);
  yield* calibrate();
  for (let i = 0; i < 800; i++) {
    if (mz.isGoal(p.x, p.y, r.cfg.goal)) { r.log('¡Meta!'); break; }
    senseCell(mz, p); publish(mz, p);
    let d;
    if (!mz.hasWall(p.x, p.y, (p.h + 1) & 3)) d = (p.h + 1) & 3;
    else if (!mz.hasWall(p.x, p.y, p.h)) d = p.h;
    else if (!mz.hasWall(p.x, p.y, (p.h + 3) & 3)) d = (p.h + 3) & 3;
    else d = (p.h + 2) & 3;
    yield* faceDir(p, d);
    const ok = yield* forward(CONF.CELL, CONF.SEARCH_SPEED, { walls: true });
    if (!ok) { yield* recover(); continue; }
    p.x += DX[p.h]; p.y += DY[p.h];
    if (mz.hasWall(p.x, p.y, p.h)) yield* alignFront();
  }
  drive(0, 0);
}
`
  },
  vacio: {
    nombre: 'Programa en blanco (para empezar el suyo)',
    desc: 'Esqueleto mínimo con la API disponible.',
    code: String.raw`// Programa propio. API del robot:
//   r.ir()            -> {left, dleft, front, dright, right}  (0..1023)
//   r.motors(izq, der) -> PWM -255..255
//   r.enc()           -> {l, r} cuentas de encoder
//   r.gyro()          -> °/s del MPU-6050 (con su error)
//   r.accel()         -> {x, y} en g del MPU-6050 (x = adelante, y = izquierda)
//   r.battery()       -> volts medidos con el divisor
//   r.millis(), r.log(...), r.beep(), r.show({...}), r.finish()
//   yield ms          -> esperar (como delay)
// También pueden usar la librería: forward(mm, vel), turn(grados), arc(±90, vel), calibrate(), Maze, searchTo...
function* main(r) {
  yield* calibrate();
  yield* forward(CONF.CELL * 3, 300, { walls: true });
  yield* turn(90);
  r.log('Listo', r.ir());
}
`
  }
};
