# Simulador de Robots

Simulador web para probar el código del **micromouse** antes de llevarlo al robot real. Usa la pista de la convocatoria, el robot de nuestra PCB final y muchas de las fallas que pueden pasar el día de la competencia.

Es una sola página estática, sin servidor ni instalación: se abre en el navegador y ya. Está pensada para crecer después con el **seguidor de línea** y el **minisumo**.

## Cómo usarlo

**En su computadora:** abran `index.html` con doble clic (Chrome, Edge o Firefox).

**Publicarlo para el equipo (GitHub Pages):**
1. Suban esta carpeta a un repositorio de GitHub.
2. En el repositorio: *Settings → Pages → Build and deployment → Source: Deploy from a branch*, rama `main`, carpeta `/ (root)`.
3. En un minuto queda en `https://<usuario>.github.io/<repositorio>/`.

## Qué se simula

| Parte | Modelo |
|---|---|
| Pista | La de la convocatoria transcrita a 19 × 10 celdas (`pistas/oficial.json`). Pasillo de ~14 cm, celda de 155 mm, pared de 12 mm y 5 cm de alto. Inicio abajo a la derecha mirando al norte; meta en el cuarto de 2 × 2 con entrada por arriba. |
| Robot | Contorno real de la PCB (100 × 100 mm, esquinas traseras R20, frente en arco), eje de llantas al centro, llantas Pololu de 34 mm separadas 90 mm. |
| Sensores IR | 5 pares en la posición y ángulo de la PCB. La señal cae con el cuadrado de la distancia, depende del ángulo y del color de la pared, el fototransistor se satura, y la luz ambiental se suma porque los emisores siempre están prendidos. |
| Motores | N20 con reductor (por defecto 100:1 ≈ 300 rpm; también 30:1, 50:1 y 150:1), corriente, back-EMF, fricción de la caja (zona muerta del PWM). Con 300 rpm y el L293D el robot llega a ~330 mm/s. |
| Electrónica | LiPo 2S con resistencia interna que se descarga → LM7806 (necesita ~2 V de margen) → L293D (pierde 1.2–2.8 V según la corriente). Si la batería baja de 6.2 V, el Arduino se reinicia. |
| Llantas | Patinan cuando se pide más fuerza de la que da el agarre, y se ensucian con cada metro recorrido. Los encoders cuentan lo que gira la llanta, no lo que avanza el robot. |
| Encoders y MPU-6050 | Encoders con pulsos perdidos. Giroscopio con sesgo, deriva y error de escala. Acelerómetro con filtro de 44 Hz, ruido y vibración de los motores: siente los choques. |
| Paredes impresas en 3D | Desalineadas, chuecas, con huecos en las uniones, de distinto grosor y reflejo. Si el robot las empuja con demasiada fuerza, se caen. |
| Reglas | Gana el menor tiempo de INICIO a META; el cronómetro arranca cuando el robot sale de la celda de inicio. Descalifica atorarse 30 s en un pasillo o tumbar una pared. |

Todas las fallas se sortean con la **semilla**: la misma semilla repite exactamente la misma corrida, así que se puede comparar un cambio de código contra otro de forma justa.

## Estrategias incluidas

Las estrategias de la pestaña **Código** son editables:

- **Mapa previo + verificación:** carga el dibujo de la convocatoria como "creencia", va directo por la ruta más corta y mide cada pared. Si la pista real es distinta, vuelve a planear.
- **Flood fill directo a la meta:** no conoce la pista y explora sólo lo necesario.
- **Explorar ida y vuelta + carrera rápida:** sirve si la competencia da varios intentos.
- **Seguir la pared derecha:** sólo como referencia; en las pruebas no llegó a la meta en esta pista (se queda dando vueltas).

## Cómo se mueve el robot (librería)

- **Vueltas suaves:** el robot decide en el "punto de sensado" de cada celda (15 mm después de entrar, cuando los sensores laterales ya ven las paredes). Si toca dar vuelta, hace una curva de 90° sin detenerse (~150 mm/s; la llanta de afuera va a ~1.9 veces). La curvatura sube y baja en rampa, y el giroscopio sigue el ángulo de la curva según la distancia real de los encoders (PD + anticipación). Los callejones sin salida y las paredes inesperadas siguen siendo "frenar en el centro y girar en su lugar". Con `CONF.SMOOTH_TURNS = false` se usa el modo anterior.
- **Choques con el MPU-6050:** compara la aceleración del acelerómetro con la que dicen los encoders. Si las llantas siguen girando pero el cuerpo frena de golpe, chocó de frente: frena, se echa para atrás, se re-centra con las paredes y vuelve a planear (si choca 2 veces en el mismo lugar, lo marca como pared). Si lo empujan de lado, rozó una pared: corrige el rumbo para alejarse. También detecta la esquina del robot pegada a una pared con los sensores diagonales.
- **Rumbo con las paredes:** mientras avanza, ajusta una recta a la distancia contra cada pared lateral; si la distancia cambia, el robot va chueco y se corrige el ángulo del giroscopio (funciona aunque haya pared de un solo lado).
- **Postes:** cuando un sensor lateral ve que se acaba una pared, el robot sabe exactamente dónde está y corrige la distancia de los encoders.
- **Calibración en la celda de inicio (~10 s, no cuenta para el tiempo):** se recarga atrás para quedar derecho, gira 90° y se recarga de lado (lo que se tuerce = error de escala del giroscopio), mide la pared de atrás con el sensor de enfrente y cada pared lateral con los dos sensores (media geométrica: no importa si quedó cargado a un lado).
- **Límite de los motores:** si el PWM llega al tope y el robot no alcanza la velocidad pedida (batería baja), aprende la velocidad máxima real y baja la velocidad de rectas y curvas.

## Resultados de referencia

Corridas en lote con la librería incluida, robot de 300 rpm con llanta de 34 mm, pista oficial. "Girar en su lugar" es la misma librería con `SMOOTH_TURNS = false`.

| Estrategia / escenario | Vueltas suaves | Girar en su lugar |
|---|---|---|
| Mapa previo · ideal | 22.8 s | 31.1 s |
| Mapa previo · realista (20 corridas) | 20/20 · 23.0 s | 20/20 · 32.9 s |
| Mapa previo · realista con 6 paredes cambiadas (10) | 10/10 · 25.4 s | 10/10 · 35.7 s |
| Mapa previo · pesimista | 29/40 · 31.0 s | 11/20 · 35.0 s |
| Flood fill directo · realista (10) | 10/10 · 73.0 s | 10/10 · 88.0 s |
| Flood fill directo · pesimista (10) | 7/10 · 81.2 s | 1/10 |

Las vueltas suaves ahorran ~10 s en la ruta de la convocatoria (32 celdas, 18 vueltas). Con el MPU y la calibración nueva, el escenario pesimista pasó de 0 de 10 (versión anterior) a ~3 de cada 4. Lo que todavía falla en el pesimista es que el robot se pierde en las escaleras (vueltas izquierda-derecha seguidas) con paredes muy chuecas y mucha luz ambiental.

En la exploración (flood fill directo) el sensor de enfrente lee débil a una celda de distancia, así que muchas paredes de enfrente se descubren ya cerca y el robot tiene que frenar y girar en su lugar. Por eso la estrategia con mapa previo es mucho más rápida.

## API del robot

El programa es una función generadora `function* main(r)`. `yield 5` espera 5 ms (como `delay(5)`). Un ciclo sin `yield` congela al robot, y el simulador lo detiene a los 5 s.

```js
r.ir()             // {left, dleft, front, dright, right}  0..1023
r.motors(izq, der) // PWM -255..255 (como analogWrite en las entradas del L293D)
r.enc()            // {l, r} cuentas de encoder (7 PPR × 2 × relación)
r.gyro()           // °/s del MPU-6050, con su sesgo
r.accel()          // {x, y} en g del MPU-6050 (x = adelante, y = izquierda)
r.battery()        // volts leídos con el divisor 200k/120k
r.millis(), r.micros(), r.log(...), r.beep(), r.finish()
r.show({...})      // dibuja el mapa que cree el robot
r.cfg              // cols, rows, start, goal, cellMM, countsPerRev, sensors, reference (pista oficial)
```

La **librería** (botón "Librería" en la pestaña Código) es el equivalente en JavaScript del firmware v2 del Nano:

| Simulador | Firmware |
|---|---|
| `CONF` | `pins.h` |
| `forward()`, `turn()`, `arc()`, `calibrate()`, `impact()` | `motion.cpp` |
| `Maze`, `flood()`, `bestDir()`, `loadPrior()` | `maze.cpp` + `prior_map.h` |
| `searchSmooth()` | `strategy.cpp` |
| programas "Mapa previo" y "Explorar" | modos 1 y 2 del menú en `micromouse.ino` |

Cuando una mejora funcione en el simulador, hay que pasarla a mano al código C++.

## Estructura

```
index.html          página (interfaz)
js/core.js          física, sensores, fallas, reglas (corre en un Web Worker)
js/programas.js     librería del robot y estrategias
js/pistas.js        pista oficial
js/app.js           interfaz: dibujo, paneles, lotes
pistas/oficial.json la misma pista en archivo
```

## Limitaciones conocidas

- Es en 2D, visto desde arriba: no simula la altura de los sensores, los reflejos del piso ni las paredes inclinadas hacia adentro.
- El modelo de llanta (patinamiento proporcional limitado por el agarre) es simplificado.
- Los valores de los motores N20 son típicos de tienda. Midan los suyos (rpm sin carga y corriente de atasco) y ajústenlos en la pestaña Robot.
- La señal de los sensores IR sigue un modelo de 1/d². Las constantes de distancia de la librería (`frontDistance`, `sideDistance`, `EDGE_OFS`) salen de ese modelo; en el robot real hay que revisarlas con el modo de telemetría.

## Pendiente

- Pista y robot del **seguidor de línea** (BlackPill, regleta de 16 sensores, TB6612).
- Dohyo y robot del **minisumo** (Nano, TB6612, sensores E18/Sharp).
