import threading
import time

from arduino.app_utils import Bridge

from vision.vision import POP_MIN_AREA

from .fsm import FiniteStateMachine, State, Transition

VELOCITY_TOPIC = '/cmd_vel'  # Topic para publicar comandos de velocidad
VCONS = 8.00 # velocidad lineal de los motores (unit? probably rev/s)
EPSILON = 4 # error visual (px) por debajo del cual el globo se considera fijado (centrado)
STEERING_DEADBAND = 2 # zona muerta del error (px): por debajo se va recto; menor que EPSILON para converger dentro
# criterio de POKED: el objetivo es estatico, asi que si un contorno grande y fijado desaparece de golpe
# es que se ha pinchado
POKE_MIN_AREA = POP_MIN_AREA # area (px del frame reducido) a partir de la que el contorno se considera grande
MIN_LOCKED_FRAMES = 3 # frames seguidos con el contorno grande y |error| <= EPSILON para darlo por fijado
POKED_LOST_FRAMES = 2 # frames seguidos sin contorno tras estar fijado para lanzar POKED (filtra parpadeos de 1 frame)
ERROR_FILTER_ALPHA = 0.5 # filtro paso bajo del error visual (1 = sin filtro); quita el ruido del centroide
MAX_STEERING = 0.6 * VCONS # saturacion de la ley de control para no bloquear una rueda
LOST_FRAMES_TO_WANDER = 10 # frames seguidos sin ver el globo en APPROACH para volver a buscar (1 s a 10 Hz)
WANDER_THETA = 7.00 # velocidad de giro sobre si mismo en WANDER (negativo = horario)
# frenada al salir de APPROACH (POKED, TARGET_LOST, ROBOT_PRESENCE): primero un pulso de contramarcha
# proporcional a la velocidad que llevaba cada rueda y despues ruedas a 0 (el MCU activa el freno del shield)
BRAKE_REVERSE_GAIN = 0.6 # fraccion de la velocidad previa que se aplica en sentido contrario
BRAKE_REVERSE_CYCLES = 1 # ciclos de contramarcha (0.1 s a 10 Hz); mas ciclos haria retroceder al robot
BRAKE_HOLD_CYCLES = 2 # ciclos con las ruedas a 0 antes de empezar a girar en WANDER

PRINT_PERIOD_S = 0.5 # segundos entre prints de la deteccion de la camara

MAX_WHEEL_REF = 1.2 * VCONS # referencia de rueda que corresponde al 100% de PWM en el MCU

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


class Controller():
    def __init__(self, goal1, goal2, frequency):
        # PD controller gains
        # kd multiplica a la derivada en px/s (diferencia * frecuencia): 0.002 * 20 Hz = 0.04 por px de cambio entre frames
        self.steering_gains = PDControllerParams(kp=0.04, kd=0.002, is_steering=True)
        self.lineal_gains = PDControllerParams(kp=1.0, kd=0.0, is_steering=False)

        self.error = None
        self.previous_error = None
        self.fsm = FiniteStateMachine(goal1, goal2, frequency)
        self.locked_frames = 0 # frames seguidos con el contorno objetivo grande y centrado (fijado)
        self.lost_frames = 0 # frames seguidos sin ver el globo en APPROACH
        self._last_tau = {"left": 0.0, "right": 0.0}
        self._brake_cycles = 0 # ciclos de frenada que quedan (contramarcha + parada)
        self._brake_from = None # velocidad de las ruedas al empezar la frenada
        self.there_is_obstacle = False # la presencia de un robot se considera obstaculo
        self.goals_ack_pending = False # hay que confirmar al MCU los colores recibidos
        self._last_print = 0.0 # instante del ultimo print de la camara

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
            previous_state = self.fsm.get_current_state()
            changed = self.fsm.update_state(transition)
            if changed and previous_state == State.APPROACH:
                # sale de APPROACH a velocidad de crucero: se frena antes de girar en WANDER
                self._brake_cycles = BRAKE_REVERSE_CYCLES + BRAKE_HOLD_CYCLES
                self._brake_from = dict(self._last_tau)
            if changed and self.fsm.get_current_state() in (State.APPROACH, State.IDLE):
                self.locked_frames = 0
                self.lost_frames = 0
                self.previous_error = None # evita el pico de la derivada en la primera muestra
                self.error = None
            if changed and self.fsm.get_current_state() == State.IDLE:
                self.there_is_obstacle = False # al volver a reposo no se arrastra el obstaculo de la partida
                self._brake_cycles = 0 # en IDLE las ruedas ya van a 0
            return changed

    def get_state(self):
        with self._lock:
            return self.fsm.get_current_state()

    def get_goal(self):
        with self._lock:
            return self.fsm.get_goal()

    def error_callback(self, msg):
        """Actualiza el error visual (px) filtrado. None si no se ve el globo objetivo."""
        if msg is None:
            self.error = None
            return
        raw = float(msg)
        last = self.error if self.error is not None else self.previous_error
        # filtro paso bajo: si no hay muestra anterior se toma la medida tal cual
        self.error = raw if last is None else ERROR_FILTER_ALPHA * raw + (1 - ERROR_FILTER_ALPHA) * last

    def vision_callback(self, detection):
        """Recibe la Detection de vision (vision.BalloonDetector) del globo del color objetivo.

        En WANDER, ver el globo lanza DETECTED_TARGET. La transicion se aplica antes de actualizar
        el error porque al entrar en APPROACH se resetea el estado del controlador visual.
        En APPROACH el error se actualiza antes de comprobar si el objetivo sigue fijado (check_target).
        """
        state = self.get_state()
        if detection.found and state == State.WANDER:
            self._apply_transition(Transition.DETECTED_TARGET)
        self.error_callback(detection.error if detection.found else None)
        if state == State.APPROACH:
            self.check_target(detection)
        self._print_detection(detection)
        Bridge.notify(BRIDGE_DETECTION, int(detection.error) if detection.found else 0, bool(detection.found))

    def check_target(self, detection):
        """Decide en APPROACH si el globo se ha pinchado (POKED) o se ha perdido (TARGET_LOST).

        El contorno queda fijado si durante MIN_LOCKED_FRAMES frames seguidos es grande (area >= POKE_MIN_AREA)
        y esta centrado (|error| <= EPSILON). Como el objetivo es estatico, si un contorno fijado desaparece de
        golpe (POKED_LOST_FRAMES frames sin verlo) se da por pinchado. Si desaparece sin estar fijado (lejos o
        descentrado) durante LOST_FRAMES_TO_WANDER frames se vuelve a buscar.
        """
        if detection.found:
            self.lost_frames = 0
            locked = detection.area >= POKE_MIN_AREA and abs(self.error) <= EPSILON
            self.locked_frames = self.locked_frames + 1 if locked else 0
            return

        self.lost_frames += 1
        if self.locked_frames >= MIN_LOCKED_FRAMES and self.lost_frames >= POKED_LOST_FRAMES:
            print('objetivo pinchado: el contorno fijado ha desaparecido')
            self._apply_transition(Transition.POKED)
        elif self.lost_frames >= LOST_FRAMES_TO_WANDER:
            print('objetivo perdido: se vuelve a buscar')
            self._apply_transition(Transition.TARGET_LOST)

    def _print_detection(self, detection):
        """Imprime el error visual (px) y en que mitad de la imagen esta el centroide, como mucho cada PRINT_PERIOD_S."""
        now = time.monotonic()
        if now - self._last_print < PRINT_PERIOD_S:
            return
        self._last_print = now

        if not detection.found:
            print('camara: sin globo')
            return
        # error = width/2 - cx: positivo = centroide en la mitad izquierda
        if detection.error > 0:
            half = 'izquierda'
        elif detection.error < 0:
            half = 'derecha'
        else:
            half = 'centro'
        print(f'camara: error = {detection.error:.1f} px | centroide en la mitad {half} (cx = {detection.cx} px)')

    def set_goals_callback(self, goal, next_goal):
        """El MCU envia los colores elegidos con P2/P3 (Bridge). Solo se aplican en IDLE.

        La confirmacion se envia desde control_loop, fuera del callback del Bridge.
        """
        with self._lock:
            applied = self.fsm.set_goals(goal, next_goal)
        if applied:
            self.goals_ack_pending = True
            print('Colores aplicados desde el MCU: inicial =', goal, '| final =', next_goal)
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
    def _clamp(value, low=0.0, high=1.2 * VCONS):
        return max(low, min(high, value))

    def _wander_tau(self):
        if self.there_is_obstacle: # logica por implementar
            return {"left": 0.0, "right": 0.0}

        theta = WANDER_THETA # girar sobre sí mismo
        if theta == 0: # avanza recto
            return {"left": VCONS, "right": VCONS}
        # giro sobre si mismo: ruedas con velocidades opuestas (pueden ser negativas, el MCU las admite)
        speed = self._clamp(abs(theta))
        if theta > 0: # regla de la mano izq. (giro antihorario)
            return {"left": -speed, "right": speed}
        return {"left": speed, "right": -speed}

    def _approach_tau(self):
        # si hay obstaculo de momento se fija a 0, esa logica se implementa a futuro
        if self.there_is_obstacle:
            return {"left": 0.0, "right": 0.0}
        if self.error is None:
            # globo fijado que desaparece: se esta pinchando, se mantiene la ultima orden para empujar
            # hasta que vision_callback lance POKED. Si no estaba fijado se frena mientras se decide
            # si es un parpadeo de la vision o TARGET_LOST, en vez de seguir a ciegas a toda velocidad
            if self.locked_frames >= MIN_LOCKED_FRAMES:
                return self._last_tau
            return {"left": 0.0, "right": 0.0}

        previous = self.error if self.previous_error is None else self.previous_error
        derivative = (self.error - previous) * self.fsm.get_frequency()
        # el error de vision ya es referencia - medida (width/2 - cx, positivo = globo a la izquierda),
        # por lo que no hace falta invertir el signo de la ley de control
        error = 0.0 if abs(self.error) <= STEERING_DEADBAND else self.error
        control_law = (self.steering_gains.getKp() * error) + (self.steering_gains.getKd() * derivative)
        control_law = max(-MAX_STEERING, min(MAX_STEERING, control_law))

        # velocidad diferencial: control_law > 0 acelera la rueda derecha y gira a la izquierda (antihorario)
        return {
            "left": self._clamp(VCONS - control_law),
            "right": self._clamp(VCONS + control_law),
        }

    def _brake_tau(self):
        """Frenada al salir de APPROACH: BRAKE_REVERSE_CYCLES de contramarcha y despues ruedas a 0."""
        with self._lock:
            cycles, start = self._brake_cycles, self._brake_from
            self._brake_cycles -= 1
        if cycles > BRAKE_HOLD_CYCLES:
            return {wheel: -BRAKE_REVERSE_GAIN * speed for wheel, speed in start.items()}
        return {"left": 0.0, "right": 0.0}

    def control_loop(self):
        """Bucle de control principal que se ejecuta periódicamente a la frecuencia de la FSM."""
        self.send_goals_ack()
        state = self.get_state()

        if self._brake_cycles > 0 and state != State.IDLE:
            tau = self._brake_tau()
        elif state == State.WANDER:
            tau = self._wander_tau()
        elif state == State.APPROACH:
            tau = self._approach_tau()
        else: # IDLE, AVOID (avoid state to be implemented)
            tau = {"left": 0.0, "right": 0.0}

        self.send_bridge(tau)
        self._last_tau = tau

        if self.error is not None:
            self.previous_error = self.error
