import threading

from arduino.app_utils import Bridge

from .fsm import FiniteStateMachine, State, Transition

VELOCITY_TOPIC = '/cmd_vel'  # Topic para publicar comandos de velocidad
VCONS = 0.25 # velocidad lineal de los motores (unit? probably rev/s)
EPSILON = 1 # visual error in pixels admited
MIN_VISUAL_TRACK_ITER = 15 # iterations of the visual controller to consider it successful and launch transition POKED
WANDER_THETA = -0.1 # velocidad de giro sobre si mismo en WANDER (negativo = horario)

# Nombres de las funciones RPC del Bridge (deben coincidir con el sketch del MCU)
BRIDGE_SET_WHEELS = 'set_wheels'        # Python -> MCU: (left: float, right: float)
BRIDGE_OBSTACLE = 'obstacle'            # -> Python: (present: bool)
BRIDGE_TRANSITION = 'transition'        # -> Python: (transition: str), p.ej. 'START', 'DETECTED_TARGET'


class PDControllerParams():

    def __init__(self, kp, kd, is_steering):
        self.kp = kp
        self.kd = kd
        self.is_steering = is_steering  # True para control de dirección, False para control de velocidad

    def getKp(self):
        return self.kp

    def getKd(self):
        return self.kd

    def setKp(self, kp):
        self.kp = kp

    def setKd(self, kd):
        self.kd = kd


class Controller():
    def __init__(self, goal1, goal2, frequency):
        # PD controller gains
        self.steering_gains = PDControllerParams(kp=0.001, kd=0.0005, is_steering=True)
        self.lineal_gains = PDControllerParams(kp=1.0, kd=0.1, is_steering=False)

        self.error = None
        self.previous_error = 0.0
        self.controller_consecutive_actions_sent = 0 # detectar si ha conseguido minimizar el error visual
        self.fsm = FiniteStateMachine(goal1, goal2, frequency)
        self.transition_poked_sent = False  # Flag para evitar publicar múltiples transiciones
        self.there_is_obstacle = False # la presencia de un robot se considera obstaculo

        # los callbacks del Bridge se ejecutan en otro hilo distinto al del control_loop
        self._lock = threading.Lock()
        self._register_bridge_callbacks()

    def _register_bridge_callbacks(self):
        Bridge.provide(BRIDGE_OBSTACLE, self.obstacle_callback)
        Bridge.provide(BRIDGE_TRANSITION, self.fsm_callback)

    def _apply_transition(self, transition):
        """Aplica una transicion a la FSM y resetea el estado del controlador visual al entrar en APPROACH."""
        with self._lock:
            changed = self.fsm.update_state(transition)
            if changed and self.fsm.get_current_state() == State.APPROACH:
                self.controller_consecutive_actions_sent = 0
                self.transition_poked_sent = False
                self.previous_error = None # evita el pico de la derivada en la primera muestra
                self.error = None
            return changed

    def get_state(self):
        with self._lock:
            return self.fsm.get_current_state()

    def get_goal(self):
        with self._lock:
            return self.fsm.get_goal()

    def error_callback(self, msg):
        """Actualiza el error visual (px). None si no se ve el globo objetivo."""
        self.error = None if msg is None else float(msg)

    def vision_callback(self, detection):
        """Recibe la Detection de vision (vision.BalloonDetector) del globo del color objetivo.

        En WANDER, ver el globo lanza DETECTED_TARGET. La transicion se aplica antes de actualizar
        el error porque al entrar en APPROACH se resetea el estado del controlador visual.
        """
        if detection.found and self.get_state() == State.WANDER:
            self._apply_transition(Transition.DETECTED_TARGET)
        self.error_callback(detection.error if detection.found else None)

    def obstacle_callback(self, present):
        """Callback para la detección de obstáculos (Bridge)."""
        if present:
            self.send_obstacle_transition()
        else:
            self.there_is_obstacle = False
            self._apply_transition(Transition.CLEAR_PATH)

    def send_poked_transition(self):
        """Si se cumplen el minimo de acciones consecutivas con error dentro de EPSILON se lanza la transicion POKED."""
        if self.error is not None and abs(self.error) <= EPSILON:
            self.controller_consecutive_actions_sent += 1
        else:
            self.controller_consecutive_actions_sent = 0

        if self.controller_consecutive_actions_sent >= MIN_VISUAL_TRACK_ITER and not self.transition_poked_sent:
            self.transition_poked_sent = True
            self._apply_transition(Transition.POKED)

    def send_obstacle_transition(self):
        """Callback para la detección de obstáculos: se envia transicion de ROBOT_PRESENCE"""
        self.there_is_obstacle = True
        self._apply_transition(Transition.ROBOT_PRESENCE)

    def fsm_callback(self, msg):
        """Callback para las transiciones de estado (Bridge)."""
        self._apply_transition(msg)

    def send_bridge(self, tau):
        """Envia a Arduino Q UNO la referencia de control de cada rueda."""
        left_wheel = float(tau["left"])
        right_wheel = float(tau["right"])
        Bridge.notify(BRIDGE_SET_WHEELS, left_wheel, right_wheel)

    @staticmethod
    def _clamp(value, low=0.0, high=2 * VCONS):
        return max(low, min(high, value))

    def _wander_tau(self):
        if self.there_is_obstacle: # logica por implementar
            return {"left": 0.0, "right": 0.0}

        theta = WANDER_THETA # girar sobre sí mismo
        if theta == 0: # avanza recto
            return {"left": VCONS, "right": VCONS}
        # a implementar a futuro: girar sin frenar a 0 cada rueda
        if theta > 0: # regla de la mano izq. (giro antihorario)
            return {"left": 0.0, "right": theta}
        return {"left": -theta, "right": 0.0}

    def _approach_tau(self):
        # si hay obstaculo de momento se fija a 0, esa logica se implementa a futuro
        if self.there_is_obstacle or self.error is None:
            return {"left": 0.0, "right": 0.0}

        previous = self.error if self.previous_error is None else self.previous_error
        derivative = (self.error - previous) * self.fsm.get_frequency()
        # el error de vision ya es referencia - medida (width/2 - cx, positivo = globo a la izquierda),
        # por lo que no hace falta invertir el signo de la ley de control
        control_law = (self.steering_gains.getKp() * self.error) + (self.steering_gains.getKd() * derivative)

        # velocidad diferencial: control_law > 0 acelera la rueda derecha y gira a la izquierda (antihorario)
        return {
            "left": self._clamp(VCONS - control_law),
            "right": self._clamp(VCONS + control_law),
        }

    def control_loop(self):
        """Bucle de control principal que se ejecuta periódicamente a la frecuencia de la FSM."""
        state = self.get_state()

        if state == State.WANDER:
            tau = self._wander_tau()
        elif state == State.APPROACH:
            tau = self._approach_tau()
            self.send_poked_transition()
        else: # IDLE, AVOID (avoid state to be implemented)
            tau = {"left": 0.0, "right": 0.0}

        self.send_bridge(tau)

        if self.error is not None:
            self.previous_error = self.error
