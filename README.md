# poking-robot

Robot autónomo para el **Hackatón Arduino UNO Q**: busca con la cámara los globos del color asignado y los pincha con la chincheta del extremo, en el menor tiempo posible y sin tocar los del equipo rival.

La app se ejecuta en **Arduino App Lab** y tiene dos partes que se comunican por el **Bridge** (RPC sobre `arduino-router`):

| Parte | Dónde corre | Responsabilidad |
|---|---|---|
| `sketch/` | MCU (STM32) del UNO Q | Pulsadores, LEDs, motores, arranque reglamentario (P1 + 5 s), límite de 3 min, menú de tests |
| `python/` | Linux (MPU) del UNO Q | Cámara y visión, máquina de estados de alto nivel, controlador PD |

```
             P1 / P2 / P3                                  cámara USB
                  │                                            │
┌─────────────────▼───────────────┐   Bridge (RPC)   ┌─────────▼──────────────────────────┐
│ MCU  sketch/sketch_prueba.ino   │ ◄──────────────► │ Python  python/main.py             │
│  · estados de arranque          │  set_goals       │  · vision/vision.py   (detección)  │
│  · aplica velocidades a motores │  transition      │  · control/fsm.py     (FSM)        │
│  · watchdog + fin de partida    │  set_wheels      │  · control/controller.py (PD + RPC)│
└─────────────────┬───────────────┘  confirmar_goals └────────────────────────────────────┘
                  │                  set_detection
           Motor Shield Rev3
```

## Estructura

```
app.yaml                    manifiesto de la app de App Lab
requirements.txt            dependencias de Python (placa y desarrollo local)
sketch/
  sketch_prueba.ino         programa del MCU
  config.h                  pines (reglas.pdf), typedefs, tiempos y nombres RPC
  vision_link.h             última detección recibida de Python (modo CAMARA)
  sketch.yaml               perfil de compilación (plataforma arduino:zephyr)
python/
  main.py                   bucle principal: cámara → visión → controlador, a FREQUENCY Hz
  control/fsm.py            FSM de alto nivel: estados, transiciones y colores objetivo
  control/controller.py     controlador PD y comunicación RPC con el MCU
  vision/vision.py          segmentación por color y cálculo del error horizontal
```

## Uso en competición

1. **Encender** el robot y lanzar la app. LED rojo parpadeando lento = MCU y Python sincronizando colores.
2. **Elegir colores** (LED rojo fijo = listo). Por defecto ambos son **azul**:
   - **P2** cambia el color **inicial**: azul → rojo → verde → azul…
   - **P3** cambia el color **final**: azul → verde → rojo → azul…

   Ejemplo: una pulsación de P2 = buscar **rojo**; una pulsación de P3 = color final **verde**. Los colores elegidos se muestran en el Monitor.
3. **Arrancar con P1**: el LED rojo parpadea (cada 500 ms) durante **5 s** y después el robot empieza solo (transición `START`, IDLE → WANDER). LED verde parpadeando = en marcha.
4. A los **3 min** el MCU para los motores (LED verde fijo). **P1** vuelve a reposo para otro intento.

| LED | Estado del MCU |
|---|---|
| Rojo parpadeo lento (1 s) | `SINCRONIZANDO` — esperando que Python confirme los colores |
| Rojo fijo | `ESPERANDO_INICIO` — P1 arranca, P2/P3 cambian colores |
| Rojo parpadeo 500 ms | `CUENTA_ATRAS` — 5 s reglamentarios |
| Verde parpadeo 500 ms | `EN_MARCHA` |
| Verde fijo | `FIN` — partida terminada |

## Hardware y pines

Fuente de verdad: `reglas.pdf`. Declarados en [sketch/config.h](sketch/config.h).

| Elemento | Pines |
|---|---|
| Rueda izquierda (Motor Shield, canal B) | dirección D13, PWM D11, freno D8 |
| Rueda derecha (Motor Shield, canal A) | dirección D12, PWM D3, freno D9 |
| Pulsadores (`INPUT_PULLUP`) | P1 = D10 (inicio), P2 = D2 (color inicial), P3 = D0 (color final) |
| LEDs | rojo D7, verde D6 |
| Sigue líneas (analógico, der. → izq.) | A0–A5 (sensores ópticos 2º–7º) |
| Cámara Logitech 720p | USB-A 3.0 del hub |

A0/A1 son también la medida de corriente del Motor Shield; en este robot se usan para el sigue líneas. Con velocidad 0 se activa el freno del canal.

## MCU: máquina de estados de arranque

```
SINCRONIZANDO ──(Python confirma colores)──► ESPERANDO_INICIO ──(P1)──► CUENTA_ATRAS
   ▲   ▲                                          │                          │ 5 s
   │   └──────────(P2/P3: cambia color)───────────┘                          ▼
   └──────────────────────(P1)──────────── FIN ◄────(3 min)──────────── EN_MARCHA
```

- `SINCRONIZANDO` y `ESPERANDO_INICIO` equivalen al estado **IDLE** de la FSM de Python.
- En `SINCRONIZANDO` el MCU envía `transition("RESET")` + `set_goals(inicial, final)` al entrar y cada segundo, hasta que Python responde con `confirmar_goals` y los colores coinciden. Así el MCU nunca arranca con colores distintos a los de Python.
- Al entrar en `EN_MARCHA` envía `transition("START")` y desde ese momento aplica las velocidades `set_wheels` de Python. Si dejan de llegar durante `TIMEOUT_COMANDO_MS` (5 periodos = 250 ms) para los motores.
- Al entrar en `FIN` envía `transition("RESET")`; Python vuelve a IDLE.
- Fuera de `EN_MARCHA` el MCU ignora `set_wheels`: es la última autoridad sobre los motores.

## Python: FSM de alto nivel

[python/control/fsm.py](python/control/fsm.py). Las transiciones no listadas se ignoran.

| Estado actual | Transición | Siguiente | Quién la lanza |
|---|---|---|---|
| IDLE | `START` | WANDER | MCU, tras la cuenta atrás |
| WANDER | `DETECTED_TARGET` | APPROACH | visión: se ve el globo del color objetivo |
| WANDER / APPROACH | `ROBOT_PRESENCE` | AVOID | RPC `obstacle(True)` |
| APPROACH | `POKED` | WANDER | controlador: globo centrado `MIN_VISUAL_TRACK_ITER` ciclos |
| AVOID | `CLEAR_PATH` | WANDER | RPC `obstacle(False)` |
| WANDER / APPROACH / AVOID | `RESET` | IDLE | MCU, al volver a reposo |

- **Colores**: `current_goal` (inicial) y `next_goal` (final). Solo se cambian en IDLE (`set_goals`). Tras `MAX_TARGETS` = 3 globos pinchados se intercambian. `RESET` pone a 0 el contador.

## Python: controlador

[python/control/controller.py](python/control/controller.py), ejecutado por `main.py` a `FREQUENCY` = 20 Hz.

Cada ciclo: captura un frame → detecta el globo del color objetivo actual → `vision_callback` (lanza `DETECTED_TARGET` en WANDER y actualiza el error) → `control_loop`:

| Estado | Velocidad de ruedas |
|---|---|
| IDLE, AVOID | 0, 0 |
| WANDER | giro sobre sí mismo (`WANDER_THETA`) |
| APPROACH | según `CONTROLLER_TYPE`: **PD** `VCONS ∓ u` con `u = Kp·e + Kd·de/dt`, o **LQR** (ver abajo); limitado a `[0, 2·VCONS]` |

- **Tipo de controlador**: `CONTROLLER_TYPE` en controller.py (`ControllerType.PD` por defecto, o `ControllerType.LQR`), o el argumento `controller_type` de `Controller`.
- **LQR**: modelo lineal de robot diferencial con rueda loca, estado `x = [φ, ω, v]` (rumbo al globo, velocidad angular, velocidad lineal) y entradas `[u_izq, u_der]`; cada rueda es un primer orden de constante `WHEEL_TIME_CONSTANT` y el rozamiento de la rueda loca va incluido en ella. Se discretiza (ZOH) a `FREQUENCY`, la ganancia sale de la ecuación de Riccati discreta (pesos por la regla de Bryson) y `ω`, `v` se estiman con un observador de Kalman estacionario porque solo se mide `φ = atan(e / f_px)`. Los parámetros físicos (`WHEEL_RADIUS`, `TRACK_WIDTH`, `WHEEL_TIME_CONSTANT`, `CAMERA_HFOV_DEG`) son estimaciones a medir en el robot.

- **Error visual** `e = ancho/2 − cx` en píxeles del frame reducido (320 px): positivo = globo a la izquierda → `u > 0` acelera la rueda derecha y el robot gira a la izquierda.
- Las referencias se envían al MCU en **% de PWM** (`2·VCONS` = 100 %).
- El bucle espera hasta el siguiente instante fijo (descuenta el tiempo de cómputo), así que el periodo real es `1/FREQUENCY`.

## Visión

[python/vision/vision.py](python/vision/vision.py): `resize (320×180) → blur → HSV + CLAHE → inRange por color → cierre morfológico → contornos → filtros de forma → centroide → error`. Los rangos HSV de rojo, verde y azul están calibrados con fotos tomadas desde el robot. Solo se busca en la parte superior del frame (`ROI_BOTTOM`) para evitar falsos positivos del suelo. Se puede probar suelto con `python vision.py [color] [imagen|carpeta]`.

## Protocolo RPC (Bridge)

Los nombres están en `sketch/config.h` (`RPC_*`) y en `python/control/controller.py` (`BRIDGE_*`); deben coincidir.

| Nombre | Dirección | Argumentos | Uso |
|---|---|---|---|
| `set_goals` | MCU → Python | `(inicial: str, final: str)` | colores elegidos con P2/P3 (`blue`/`red`/`green`) |
| `confirmar_goals` | Python → MCU (`call`) | `(inicial: str, final: str) -> bool` | Python confirma los colores aplicados en su FSM |
| `transition` | MCU → Python | `(nombre: str)` | `START` al acabar la cuenta atrás, `RESET` al volver a reposo |
| `set_wheels` | Python → MCU | `(izq: int, der: int)` en % [-100, 100] | velocidades de rueda, cada ciclo |
| `set_detection` | Python → MCU | `(error_px: int, encontrado: bool)` | depuración (modo CAMARA) |
| `obstacle` | → Python | `(presente: bool)` | `ROBOT_PRESENCE` / `CLEAR_PATH` (aún no lo envía nadie) |

Python responde a `set_goals` desde el bucle principal, no desde el callback del Bridge.

## Menú de tests (Monitor de App Lab)

Con el robot en reposo, escribir en el Monitor:

| Opción | Test |
|---|---|
| `1` | Motores: `IZQ DER` en %, p. ej. `30 30`, `-30 -30`, `0 0` |
| `2` | Pulsadores: muestra pulsaciones/liberaciones de P1–P3 |
| `3` | Sensores sigue líneas: lecturas de A0–A5 |
| `4` | Cámara: error recibido de Python (LED verde = centrado, rojo = descentrado) |
| `5` | LEDs: parpadeo alterno |
| `0` / `MENU` | volver al menú (el robot vuelve a `SINCRONIZANDO`) |

## Parámetros a calibrar

| Parámetro | Fichero | Valor | Nota |
|---|---|---|---|
| `VCONS`, `WANDER_THETA` | controller.py | 0.25 (50 % PWM), -0.1 (20 % PWM) | comprobar con el test de motores que el 20 % mueve las ruedas |
| `steering_gains` (Kp, Kd) | controller.py | 0.001, 0.0005 | Kd domina cuando el error cambia rápido |
| `EPSILON`, `MIN_VISUAL_TRACK_ITER` | controller.py | 4 px, 15 ciclos | criterio de `POKED` |
| `FREQUENCY` | main.py | 20 Hz | debe coincidir con `FRECUENCIA_SISTEMA_HZ` de config.h |
| `COLOR_RANGES`, `ROI_BOTTOM` | vision.py | — | recalibrar con la luz del recinto |
| `MOTOR_IZQ/DER.adelanteEsHigh` | config.h | true / false | invertir si una rueda gira al revés |

## Limitaciones conocidas

- **Globo perdido en APPROACH**: si el globo desaparece (pinchado o fuera de imagen) antes de cumplirse el criterio de `POKED`, el robot se queda parado en APPROACH. Opciones: usar `PopCounter` de `vision.py` o añadir una transición `LOST_TARGET` → WANDER.
- **Fin de partida**: según las reglas la partida acaba al pinchar los 3 globos propios, pero la FSM cambia al color final y sigue; pinchar globos rivales resta puntos.
- **AVOID** no tiene comportamiento todavía y nadie envía `obstacle`.
- Los rangos HSV y las ganancias no se han validado en el campo.
