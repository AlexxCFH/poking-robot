import math
import threading
from enum import Enum

import numpy as np
from arduino.app_utils import Bridge

from .fsm import FiniteStateMachine, State, Transition

VELOCITY_TOPIC = '/cmd_vel'  # Topic para publicar comandos de velocidad
VCONS = 0.25 # velocidad lineal de los motores (unit? probably rev/s)
EPSILON = 4 # error visual admitido en pixeles (~0.8 grados con 320 px y 60 de FOV; holgura frente al ruido de la camara)
MIN_VISUAL_TRACK_ITER = 15 # iterations of the visual controller to consider it successful and launch transition POKED
WANDER_THETA = -0.1 # velocidad de giro sobre si mismo en WANDER (negativo = horario)

MAX_WHEEL_REF = 2 * VCONS # referencia de rueda que corresponde al 100% de PWM en el MCU


class ControllerType(str, Enum):
    PD = 'PD'   # PD sobre el error visual en pixeles
    LQR = 'LQR' # LQR sobre el modelo lineal en espacio de estados del robot diferencial


CONTROLLER_TYPE = ControllerType.PD # controlador usado en APPROACH

# Camara: pasar el error en pixeles a angulo de rumbo hacia el globo
IMAGE_WIDTH_PX = 320    # ancho del frame procesado (vision.PROCESS_SIZE)
CAMERA_HFOV_DEG = 60.0  # campo de vision horizontal de la Logitech 720p (a medir)

# Parametros fisicos del robot para el modelo LQR (a identificar en el robot real)
WHEEL_RADIUS = 0.035      # m
TRACK_WIDTH = 0.15        # m, distancia entre las ruedas motrices
WHEEL_TIME_CONSTANT = 0.15 # s, respuesta de primer orden motor + inercia + rozamiento (incluye la rueda loca)
WHEEL_SPEED_GAIN = 2 * math.pi * WHEEL_RADIUS # m/s de rueda por unidad de referencia (referencias en rev/s)

# Nombres de las funciones RPC del Bridge (deben coincidir con sketch/config.h)
BRIDGE_SET_WHEELS = 'set_wheels'        # Python -> MCU: (left: int, right: int) en % [-100, 100]
BRIDGE_CONFIRM_GOALS = 'confirmar_goals' # Python -> MCU: (goal: str, next_goal: str) -> bool, colores aplicados
BRIDGE_DETECTION = 'set_detection'      # Python -> MCU: (error: int, found: bool), solo para depurar
BRIDGE_SET_GOALS = 'set_goals'          # MCU -> Python: (goal: str, next_goal: str), elegidos con P2/P3
BRIDGE_OBSTACLE = 'obstacle'            # -> Python: (present: bool)
BRIDGE_TRANSITION = 'transition'        # MCU -> Python: (transition: str), 'START' tras la cuenta atras o 'RESET'


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


def solve_dare(A, B, Q, R, max_iter=10000, tol=1e-10):
    """Resuelve la ecuacion algebraica de Riccati discreta por iteracion. Devuelve P."""
    P = Q.copy()
    for _ in range(max_iter):
        K = np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)
        P_next = Q + A.T @ P @ (A - B @ K)
        if np.max(np.abs(P_next - P)) < tol:
            return P_next
        P = P_next
    raise RuntimeError('La ecuacion de Riccati no converge')


def discretize_zoh(A, B, dt, terms=20):
    """Discretizacion exacta con retenedor de orden cero: exp([[A, B], [0, 0]] * dt)."""
    n, m = B.shape
    M = np.zeros((n + m, n + m))
    M[:n, :n] = A
    M[:n, n:] = B
    M *= dt
    # serie de Taylor con escalado y cuadrado para la exponencial de la matriz
    squarings = max(0, int(np.ceil(np.log2(max(np.linalg.norm(M, np.inf), 1e-12)))) + 1)
    M /= 2 ** squarings
    expm = np.eye(n + m)
    term = np.eye(n + m)
    for k in range(1, terms):
        term = term @ M / k
        expm = expm + term
    for _ in range(squarings):
        expm = expm @ expm
    return expm[:n, :n], expm[:n, n:]


class DifferentialDriveModel():
    """Modelo dinamico lineal en espacio de estados de un robot diferencial con rueda loca.

    Estado x = [phi, omega, v]:
        phi   rumbo del globo respecto al eje del robot (rad, positivo = a la izquierda)
        omega velocidad angular del robot (rad/s, positivo = antihorario)
        v     velocidad lineal del robot (m/s)
    Entrada u = [u_izq, u_der]: referencias de rueda (mismas unidades que VCONS).
    Salida y = phi, medida por la camara.

    Cada rueda motriz responde como un sistema de primer orden: tau * dv_i/dt = -v_i + k * u_i.
    La rueda loca no impone restriccion cinematica; su rozamiento se incluye en tau. Con
    omega = (v_der - v_izq) / L y v = (v_der + v_izq) / 2, y suponiendo el globo estatico y
    lejano (dphi/dt = -omega), el modelo continuo es:

        dphi/dt   = -omega
        domega/dt = -omega / tau + k / (L * tau) * (u_der - u_izq)
        dv/dt     = -v / tau     + k / (2 * tau) * (u_der + u_izq)
    """

    def __init__(self, wheel_speed_gain=WHEEL_SPEED_GAIN, track_width=TRACK_WIDTH,
                 time_constant=WHEEL_TIME_CONSTANT):
        k, L, tau = wheel_speed_gain, track_width, time_constant
        self.k = k
        self.A = np.array([
            [0.0, -1.0, 0.0],
            [0.0, -1.0 / tau, 0.0],
            [0.0, 0.0, -1.0 / tau],
        ])
        self.B = np.array([
            [0.0, 0.0],
            [-k / (L * tau), k / (L * tau)],
            [k / (2 * tau), k / (2 * tau)],
        ])
        self.C = np.array([[1.0, 0.0, 0.0]])

    def discretize(self, frequency):
        return discretize_zoh(self.A, self.B, 1.0 / frequency)

    def equilibrium(self, wheel_ref):
        """Punto de equilibrio avanzando recto hacia el globo con ambas ruedas a wheel_ref."""
        x_eq = np.array([0.0, 0.0, self.k * wheel_ref])
        u_eq = np.array([wheel_ref, wheel_ref])
        return x_eq, u_eq


class LQRController():
    """LQR discreto en torno al equilibrio (avanzar recto a VCONS con el globo centrado).

    u = u_eq - K (x_est - x_eq), con K de la ecuacion de Riccati discreta. Solo se mide phi,
    asi que omega y v se estiman con un observador de Kalman estacionario sobre el mismo modelo.
    Pesos por la regla de Bryson: 1 / (valor maximo admisible)^2.
    """

    def __init__(self, frequency, model=None, max_phi=math.radians(3), max_omega=1.0, max_v_error=0.05,
                 max_wheel_delta=VCONS, process_noise=(1e-4, 1e-2, 1e-3), measurement_noise=math.radians(1) ** 2):
        self.model = model if model is not None else DifferentialDriveModel()
        self.Ad, self.Bd = self.model.discretize(frequency)
        C = self.model.C

        Q = np.diag([1 / max_phi ** 2, 1 / max_omega ** 2, 1 / max_v_error ** 2])
        R = np.diag([1 / max_wheel_delta ** 2] * 2)
        P = solve_dare(self.Ad, self.Bd, Q, R)
        self.K = np.linalg.solve(R + self.Bd.T @ P @ self.Bd, self.Bd.T @ P @ self.Ad)

        # observador: Riccati dual (covarianza de prediccion) -> ganancia de correccion
        Qn = np.diag(process_noise)
        Rn = np.array([[measurement_noise]])
        S = solve_dare(self.Ad.T, C.T, Qn, Rn)
        self.M = S @ C.T @ np.linalg.inv(C @ S @ C.T + Rn)
        self.C = C

        self.x_eq, self.u_eq = self.model.equilibrium(VCONS)
        self.f_px = (IMAGE_WIDTH_PX / 2) / math.tan(math.radians(CAMERA_HFOV_DEG) / 2)
        self.x_pred = None

    def reset(self):
        self.x_pred = None

    def pixel_error_to_bearing(self, error_px):
        """Error visual (width/2 - cx, positivo = globo a la izquierda) -> rumbo phi en rad."""
        return math.atan2(error_px, self.f_px)

    def compute(self, error_px, saturate):
        """Calcula las referencias de rueda (izq, der) a partir del error visual en pixeles."""
        phi = self.pixel_error_to_bearing(error_px)
        if self.x_pred is None:
            self.x_pred = np.array([phi, 0.0, 0.0])

        # correccion con la medida de la camara
        x_est = self.x_pred + self.M @ (np.array([phi]) - self.C @ self.x_pred)
        u = self.u_eq - self.K @ (x_est - self.x_eq)
        u = np.array([saturate(u[0]), saturate(u[1])])

        # prediccion con la entrada realmente aplicada (tras saturar)
        self.x_pred = self.Ad @ x_est + self.Bd @ u
        return float(u[0]), float(u[1])


class Controller():
    def __init__(self, goal1, goal2, frequency, controller_type=CONTROLLER_TYPE):
        self.controller_type = ControllerType(controller_type)
        # PD controller gains
        self.steering_gains = PDControllerParams(kp=0.001, kd=0.0005, is_steering=True)
        self.lineal_gains = PDControllerParams(kp=1.0, kd=0.1, is_steering=False)
        self.lqr = LQRController(frequency) if self.controller_type == ControllerType.LQR else None

        self.error = None
        self.previous_error = 0.0
        self.controller_consecutive_actions_sent = 0 # detectar si ha conseguido minimizar el error visual
        self.fsm = FiniteStateMachine(goal1, goal2, frequency)
        self.transition_poked_sent = False  # Flag para evitar publicar múltiples transiciones
        self.there_is_obstacle = False # la presencia de un robot se considera obstaculo
        self.goals_ack_pending = False # hay que confirmar al MCU los colores recibidos

        # los callbacks del Bridge se ejecutan en otro hilo distinto al del control_loop
        self._lock = threading.Lock()
        self._register_bridge_callbacks()

    def _register_bridge_callbacks(self):
        Bridge.provide(BRIDGE_OBSTACLE, self.obstacle_callback)
        Bridge.provide(BRIDGE_TRANSITION, self.fsm_callback)
        Bridge.provide(BRIDGE_SET_GOALS, self.set_goals_callback)

    def _apply_transition(self, transition):
        """Aplica una transicion a la FSM y resetea el estado del controlador visual al entrar en APPROACH o IDLE."""
        with self._lock:
            changed = self.fsm.update_state(transition)
            if changed and self.fsm.get_current_state() in (State.APPROACH, State.IDLE):
                self.controller_consecutive_actions_sent = 0
                self.transition_poked_sent = False
                self.previous_error = None # evita el pico de la derivada en la primera muestra
                self.error = None
                if self.lqr is not None:
                    self.lqr.reset()
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
        Bridge.notify(BRIDGE_DETECTION, int(detection.error) if detection.found else 0, bool(detection.found))

    def set_goals_callback(self, goal, next_goal):
        """El MCU envia los colores elegidos con P2/P3 (Bridge). Solo se aplican en IDLE.

        La confirmacion se envia desde control_loop, fuera del callback del Bridge.
        """
        with self._lock:
            applied = self.fsm.set_goals(goal, next_goal)
        if applied:
            self.goals_ack_pending = True
        else:
            print('Colores rechazados (FSM fuera de IDLE o color no valido):', goal, next_goal)

    def send_goals_ack(self):
        """Confirma al MCU los colores aplicados; si falla, el MCU los vuelve a enviar."""
        if not self.goals_ack_pending:
            return
        self.goals_ack_pending = False
        with self._lock:
            goal, next_goal = self.fsm.get_goal(), self.fsm.get_next_goal()
        try:
            Bridge.call(BRIDGE_CONFIRM_GOALS, goal.value, next_goal.value)
        except Exception as e:
            print('Error confirmando los colores al MCU:', e)

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
        """Envia a Arduino Q UNO la referencia de control de cada rueda en % de PWM."""
        left_wheel = self._to_percent(tau["left"])
        right_wheel = self._to_percent(tau["right"])
        Bridge.notify(BRIDGE_SET_WHEELS, left_wheel, right_wheel)

    @staticmethod
    def _to_percent(reference):
        return int(round(max(-1.0, min(1.0, reference / MAX_WHEEL_REF)) * 100))

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
            if self.lqr is not None:
                self.lqr.reset()
            return {"left": 0.0, "right": 0.0}

        if self.controller_type == ControllerType.LQR:
            return self._lqr_tau()
        return self._pd_tau()

    def _lqr_tau(self):
        left, right = self.lqr.compute(self.error, self._clamp)
        return {"left": left, "right": right}

    def _pd_tau(self):
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
        self.send_goals_ack()
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
