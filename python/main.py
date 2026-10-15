import time

from arduino.app_utils import App

from control.controller import Controller
from control.fsm import FREQUENCY, Goal

controller = Controller(Goal.RED, Goal.BLUE)


def loop():
    controller.control_loop()
    time.sleep(1.0 / FREQUENCY)


App.run(user_loop=loop)
