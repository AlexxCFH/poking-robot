from enum import Enum

GOAL_TOPIC = '/goal'
TRANSITION_TOPIC = '/transition'
STATE_TOPIC = '/state'
MAX_TARGETS = 3 # Lote de globos a pinchar


class Goal(str, Enum):
    GREEN = 'green'
    RED = 'red'
    BLUE = 'blue'


class State(str, Enum):
    IDLE = 'IDLE'
    WANDER = 'WANDER'
    APPROACH = 'APPROACH'
    AVOID = 'AVOID'


class Transition(str, Enum):
    START = 'START'
    DETECTED_TARGET = 'DETECTED_TARGET'
    POKED = 'POKED'
    TARGET_LOST = 'TARGET_LOST' # se pierde el globo en APPROACH sin haberlo pinchado
    ROBOT_PRESENCE = 'ROBOT_PRESENCE'
    CLEAR_PATH = 'CLEAR_PATH'
    RESET = 'RESET' # el MCU vuelve a reposo (fin de partida o menu de tests)
    STOP = 'STOP' # parada manual: P1 pulsado de nuevo con el robot en marcha


# Tabla de transiciones: (estado actual, transicion) -> estado siguiente
# Cualquier combinacion que no aparezca aqui se ignora
TRANSITION_TABLE = {
    (State.IDLE, Transition.START): State.WANDER,
    (State.WANDER, Transition.DETECTED_TARGET): State.APPROACH,
    (State.WANDER, Transition.ROBOT_PRESENCE): State.AVOID,
    (State.APPROACH, Transition.POKED): State.WANDER,
    (State.APPROACH, Transition.TARGET_LOST): State.WANDER,
    (State.APPROACH, Transition.ROBOT_PRESENCE): State.AVOID,
    (State.AVOID, Transition.CLEAR_PATH): State.WANDER,
    (State.WANDER, Transition.RESET): State.IDLE,
    (State.APPROACH, Transition.RESET): State.IDLE,
    (State.AVOID, Transition.RESET): State.IDLE,
    (State.WANDER, Transition.STOP): State.IDLE,
    (State.APPROACH, Transition.STOP): State.IDLE,
    (State.AVOID, Transition.STOP): State.IDLE,
}


class FiniteStateMachine():
    def __init__(self, goal1, goal2, frequency):
        # atributos de clase
        if frequency <= 0:
            raise ValueError('System frequency must be positive')
        self.frequency = float(frequency) # Hz - frecuencia del sistema, debe coincidir con la del MCU
        self.current_state = State.IDLE
        self.transition = None
        self.num_baloons_poked = 0 # globos pinchados counter

        self.current_goal, self.next_goal = self._parse_goals(goal1, goal2)
        self.initial_goal, self.final_goal = self.current_goal, self.next_goal # colores elegidos, para restaurarlos al volver a IDLE

    @staticmethod
    def _parse_goals(goal1, goal2):
        try:
            return Goal(goal1), Goal(goal2)
        except ValueError:
            raise ValueError('Wrong string passed as goal value')

    def set_goals(self, goal1, goal2):
        """Cambia el color objetivo inicial y el final. Solo se permite en IDLE; devuelve True si se aplica."""
        if self.current_state != State.IDLE:
            return False
        try:
            self.current_goal, self.next_goal = self._parse_goals(goal1, goal2)
        except ValueError:
            return False
        self.initial_goal, self.final_goal = self.current_goal, self.next_goal
        return True

    def update_state(self, transition):
        """
        Aplica la transicion (Transition o str) sobre el estado actual segun TRANSITION_TABLE.
        Devuelve True si el estado ha cambiado, False si la transicion no es valida desde el estado actual.
        """
        try:
            transition = Transition(transition)
        except ValueError:
            return False

        next_state = TRANSITION_TABLE.get((self.current_state, transition))
        if next_state is None:
            return False

        self.transition = transition
        self.current_state = next_state

        if transition == Transition.POKED:
            self.baloon_poked()
        elif transition in (Transition.RESET, Transition.STOP):
            self.reset()
        return True

    def reset(self):
        """Vuelve a la situacion de partida: contador de globos a 0 y colores objetivo los elegidos (sin el intercambio de lotes)."""
        self.baloon_poked(reset=True)
        self.current_goal, self.next_goal = self.initial_goal, self.final_goal

    def _update_goal(self):
        # intercambia objetivo actual y siguiente para poder seguir alternando lotes
        self.current_goal, self.next_goal = self.next_goal, self.current_goal

    def get_current_state(self):
        return self.current_state

    def get_frequency(self):
        return self.frequency

    def get_period(self):
        return 1.0 / self.frequency

    def get_goal(self):
        return self.current_goal

    def get_next_goal(self):
        return self.next_goal

    def baloon_poked(self, reset=False):
        if reset:
            self.num_baloons_poked = 0
            return
        self.num_baloons_poked +=1
        if self.num_baloons_poked == MAX_TARGETS:
            self._update_goal() # actualizar al siguiente color objetivo
            self.baloon_poked(reset=True) # reset del contador
