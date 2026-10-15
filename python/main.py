import time

import cv2 as cv
from arduino.app_utils import App

from control.controller import Controller
from control.fsm import Goal
from vision.vision import BalloonDetector, Detection, configure_camera

FREQUENCY = 10.0 # Hz - frecuencia del sistema, debe coincidir con la del MCU
CAMERA_INDEX = 0
CAMERA_EXPOSURE = None # None = no se toca la exposicion, solo se pasa a manual

controller = Controller(Goal.BLUE, Goal.BLUE, FREQUENCY) # colores por defecto; se cambian con P2/P3 en el MCU
detector = BalloonDetector(controller.get_goal().value)

camera = cv.VideoCapture(CAMERA_INDEX)
print("camara:", configure_camera(camera, CAMERA_EXPOSURE))
camera.set(cv.CAP_PROP_BUFFERSIZE, 1) # evita procesar frames antiguos del buffer

period = controller.fsm.get_period() # el bucle usa la misma frecuencia que la FSM
next_tick = time.monotonic()


def detect_goal():
    """Captura un frame y busca el globo del color objetivo actual de la FSM."""
    detector.set_color(controller.get_goal().value) # el objetivo cambia tras cada lote de globos
    ok, frame = camera.read()
    if not ok:
        return Detection(False)
    det, _, _ = detector.detect(frame)
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
    camera.release()
