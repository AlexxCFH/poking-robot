import time

from arduino.app_utils import App

from control.controller import Controller
from control.fsm import Goal

FREQUENCY = 20.0 # Hz - frecuencia del sistema, debe coincidir con la del MCU

controller = Controller(Goal.RED, Goal.BLUE, FREQUENCY)
period = controller.fsm.get_period() # el bucle usa la misma frecuencia que la FSM
next_tick = time.monotonic()


def loop():
    global next_tick
    controller.control_loop()

    # espera hasta el siguiente instante fijo descontando el tiempo de computo del ciclo
    next_tick += period
    remaining = next_tick - time.monotonic()
    if remaining > 0:
        time.sleep(remaining)
    else:
        next_tick = time.monotonic() # ciclo retrasado: se resincroniza sin acumular deuda


App.run(user_loop=loop)
