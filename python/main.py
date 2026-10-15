import os
import time

import cv2 as cv
import numpy as np
from arduino.app_utils import App

from control.controller import Controller
from control.fsm import Goal
from vision.vision import BalloonDetector, Detection, configure_camera, open_camera

FREQUENCY = 10.0 # Hz - frecuencia del sistema, debe coincidir con la del MCU
CAMERA_INDEX = 0 # primera camara que se prueba; si no da frames open_camera prueba las demas
CAMERA_EXPOSURE = None # None = exposicion automatica; si no, valor manual (V4L2: unidades de 100 us)
REOPEN_AFTER_FAILURES = 10 # lecturas fallidas seguidas tras las que se cierra y se vuelve a abrir la camara

# diagnostico de vision: cada DEBUG_PERIOD_S se imprime el brillo del frame y los px de la mascara
# y, si DEBUG_DIR no es None, se guarda el frame reducido junto a la mascara para ver que ve el robot
DEBUG_PERIOD_S = 1.0
DEBUG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "debug") # None = no se guardan imagenes
DEBUG_MAX_IMAGES = 50 # se sobrescriben en ciclo para no llenar el disco

controller = Controller(Goal.BLUE, Goal.BLUE, FREQUENCY) # colores por defecto; se cambian con P2/P3 en el MCU
detector = BalloonDetector(controller.get_goal().value)



def start_camera(index):
    """Abre la camara (comprobando que da frames) y aplica la exposicion. Devuelve (camara o None, indice)."""
    cam, found = open_camera(index)
    if cam is None:
        print("ERROR: ninguna camara devuelve frames; se reintenta tras", REOPEN_AFTER_FAILURES, "ciclos")
        return None, index
    print("camara", found, ":", configure_camera(cam, CAMERA_EXPOSURE))
    return cam, found


camera, camera_index = start_camera(CAMERA_INDEX)
if DEBUG_DIR is not None:
    os.makedirs(DEBUG_DIR, exist_ok=True)

_last_debug = 0.0
_debug_index = 0
_read_failures = 0


def debug_vision(det, small, mask):
    """Imprime y guarda lo que ve la camara como mucho cada DEBUG_PERIOD_S.

    - brillo: media del canal V (0-255). Cerca de 255 = sobreexpuesta (globos blancos), cerca de 0 = negra.
    - quemado: % de px con V > 250, saturados por la exposicion.
    - mascara: px del color objetivo tras el filtrado; si es 0 el rango HSV no encaja con lo que ve la camara.
    """
    global _last_debug, _debug_index
    now = time.monotonic()
    if now - _last_debug < DEBUG_PERIOD_S:
        return
    _last_debug = now

    v = cv.cvtColor(small, cv.COLOR_BGR2HSV)[:, :, 2]
    print(f"vision [{detector.segmenter.color}]: brillo = {v.mean():.0f} | quemado = {100 * np.mean(v > 250):.0f} % "
          f"| mascara = {cv.countNonZero(mask)} px | {det}")

    if DEBUG_DIR is None:
        return
    vis = small.copy()
    if det.found:
        cv.drawContours(vis, [det.contour], -1, (255, 255, 255), 2)
    path = os.path.join(DEBUG_DIR, f"{_debug_index:02d}.jpg")
    cv.imwrite(path, np.hstack([vis, cv.cvtColor(mask, cv.COLOR_GRAY2BGR)]))
    _debug_index = (_debug_index + 1) % DEBUG_MAX_IMAGES

period = controller.fsm.get_period() # el bucle usa la misma frecuencia que la FSM
next_tick = time.monotonic()


def detect_goal():
    """Captura un frame y busca el globo del color objetivo actual de la FSM."""
    global _read_failures, camera, camera_index
    detector.set_color(controller.get_goal().value) # el objetivo cambia tras cada lote de globos
    ok, frame = camera.read() if camera is not None else (False, None)
    if not ok:
        _read_failures += 1
        if camera is not None and _read_failures == 1:
            print("ERROR: la camara ha dejado de devolver frames")
        if _read_failures >= REOPEN_AFTER_FAILURES:
            # p. ej. la webcam se ha reiniciado por un pico de consumo de los motores; mientras se
            # reabre no se envian referencias y el MCU para los motores por timeout
            print("reabriendo la camara...")
            if camera is not None:
                camera.release()
            camera, camera_index = start_camera(camera_index)
            _read_failures = 0
        return Detection(False)
    _read_failures = 0
    det, small, mask = detector.detect(frame)
    debug_vision(det, small, mask)
    return det


last_state = None


def loop():
    global next_tick, last_state
    controller.vision_callback(detect_goal())
    controller.control_loop()

    # solo se imprime al cambiar de estado para no saturar la consola a 20 Hz
    state = controller.get_state()
    if state != last_state:
        print("estado:", state.value, "| objetivo:", controller.get_goal().value)
        last_state = state

    # espera hasta el siguiente instante fijo descontando el tiempo de computo del ciclo
    next_tick += period
    remaining = next_tick - time.monotonic()
    if remaining > 0:
        time.sleep(remaining)
    else:
        next_tick = time.monotonic() # ciclo retrasado: se resincroniza sin acumular deuda


try:
    App.run(user_loop=loop)
finally:
    if camera is not None:
        camera.release()
