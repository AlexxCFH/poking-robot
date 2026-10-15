// Sketch del MCU del Arduino UNO Q.
// La logica de alto nivel (FSM WANDER/APPROACH/AVOID, vision y controlador PD) corre en Python
// (python/main.py). Este sketch gestiona la eleccion de colores (P2/P3), el arranque reglamentario
// (P1 + 5 s), la duracion de la partida, los tests de hardware desde el Monitor y aplica las
// velocidades de rueda de Python.
// Pines y tipos en config.h (fuente: reglas.pdf).
#include <Arduino.h>
#include <Arduino_RouterBridge.h>

#include "config.h"
#include "vision_link.h"


// ============================================================
//                       PULSADORES
// ============================================================

Pulsador pulsador1 = { PIN_PULSADOR_1, "PULSADOR 1 - INICIO (PIN 10)",       HIGH, HIGH, 0, SIN_CAMBIO };
Pulsador pulsador2 = { PIN_PULSADOR_2, "PULSADOR 2 - COLOR INICIAL (PIN 2)", HIGH, HIGH, 0, SIN_CAMBIO };
Pulsador pulsador3 = { PIN_PULSADOR_3, "PULSADOR 3 - COLOR FINAL (PIN 0)",   HIGH, HIGH, 0, SIN_CAMBIO };


// ============================================================
//                COLORES OBJETIVO (P2 / P3)
// ============================================================

// Se eligen con P2 (inicial) y P3 (final) y se envian a Python con set_goals
ColorGlobo colorInicial = COLOR_POR_DEFECTO;
ColorGlobo colorFinal = COLOR_POR_DEFECTO;

unsigned long tiempoSincronizacion = 0;


// ============================================================
//                    MAQUINA DE ESTADOS
// ============================================================

EstadoRobot estado = SINCRONIZANDO;

unsigned long tiempoEstado = 0;


// ============================================================
//              REFERENCIAS Y DETECCION (DESDE PYTHON)
// ============================================================

// Velocidades de rueda en % que calcula el controlador de Python
int velIzqRef = 0;
int velDerRef = 0;
unsigned long tiempoComando = 0;

// Ultima deteccion del globo objetivo (solo para el modo CAMARA)
VisionLink vision;


// ============================================================
//                         VARIABLES
// ============================================================

ModoMenu modo = MODO_ESTRATEGIA;

const unsigned long PERIODO_IMPRESION_MS = 200;

unsigned long tiempoSensores = 0;
unsigned long tiempoCamara = 0;
unsigned long tiempoLeds = 0;

bool estadoLed = false;


// ============================================================
//                          MOTORES
// ============================================================

void ConfigurarMotor(const Motor &m) {

  pinMode(m.pinDireccion, OUTPUT);
  pinMode(m.pinPWM, OUTPUT);
  pinMode(m.pinFreno, OUTPUT);
}


// velocidad en % [-100, 100]; 0 activa el freno del Motor Shield
void MoverMotor(const Motor &m, int velocidad) {

  velocidad = constrain(velocidad, -VELOCIDAD_MAX, VELOCIDAD_MAX);

  if (velocidad == 0) {
    analogWrite(m.pinPWM, 0);
    digitalWrite(m.pinFreno, HIGH);
    return;
  }

  bool adelante = velocidad > 0;

  digitalWrite(m.pinFreno, LOW);
  digitalWrite(m.pinDireccion, (adelante == m.adelanteEsHigh) ? HIGH : LOW);

  int pwm = map(abs(velocidad), 0, VELOCIDAD_MAX, 0, PWM_MAX);

  analogWrite(m.pinPWM, pwm);
}


void MotorIzquierdo(int velocidad) {

  MoverMotor(MOTOR_IZQ, velocidad);
}


void MotorDerecho(int velocidad) {

  MoverMotor(MOTOR_DER, velocidad);
}


void PararMotores() {

  MotorIzquierdo(0);
  MotorDerecho(0);
}


// Aplica las velocidades de Python; si dejan de llegar (Python caido o bloqueado) se para
void AplicarReferencias() {

  if (millis() - tiempoComando > TIMEOUT_COMANDO_MS) {
    PararMotores();
    return;
  }

  MotorIzquierdo(velIzqRef);
  MotorDerecho(velDerRef);
}


// ============================================================
//                           LEDS
// ============================================================

void EscribirLed(int pin, bool encendido) {

  digitalWrite(pin, encendido ? HIGH : LOW);
}


void ApagarLeds() {

  EscribirLed(PIN_LED_ROJO, false);
  EscribirLed(PIN_LED_VERDE, false);
}


void TestLeds() {

  if (millis() - tiempoLeds >= 500) {

    tiempoLeds = millis();

    estadoLed = !estadoLed;

    EscribirLed(PIN_LED_ROJO, estadoLed);
    EscribirLed(PIN_LED_VERDE, !estadoLed);
  }
}


void ParpadearLed(int pin, unsigned long periodo) {

  if (millis() - tiempoLeds >= periodo) {

    tiempoLeds = millis();

    estadoLed = !estadoLed;

    EscribirLed(pin, estadoLed);
  }
}


// ============================================================
//                    LECTURA PULSADORES
// ============================================================

// Antirrebote sin bloquear: un cambio solo se confirma si la
// lectura se mantiene estable durante DEBOUNCE_MS.
void ActualizarPulsador(Pulsador &p) {

  p.evento = SIN_CAMBIO;

  bool lectura = digitalRead(p.pin) == HIGH;

  if (lectura != p.lectura) {
    p.lectura = lectura;
    p.tiempoCambio = millis();
  }

  if (millis() - p.tiempoCambio >= DEBOUNCE_MS && p.lectura != p.estable) {

    p.estable = p.lectura;

    // INPUT_PULLUP: pulsado = LOW
    p.evento = p.estable ? LIBERADO : PRESIONADO;
  }
}


void ActualizarPulsadores() {

  ActualizarPulsador(pulsador1);
  ActualizarPulsador(pulsador2);
  ActualizarPulsador(pulsador3);
}


// ============================================================
//                       MENU INICIAL
// ============================================================

void MenuInicial() {

  Monitor.println();
  Monitor.println("================================");
  Monitor.println("           MENU ROBOT");
  Monitor.println("================================");
  Monitor.println("1 - TEST MOTORES");
  Monitor.println("2 - TEST PULSADORES");
  Monitor.println("3 - TEST SENSORES");
  Monitor.println("4 - CAMARA");
  Monitor.println("5 - TEST LEDS");
  Monitor.println("0 - PARAR / MENU");
  Monitor.println("================================");
  Monitor.println("En el menu: P1 = arrancar, P2 = color inicial, P3 = color final");
  Monitor.println("Introduce una opcion:");
  Monitor.println();
}


// ============================================================
//                    TEST PULSADORES
// ============================================================

void MostrarEventoPulsador(const Pulsador &p) {

  if (p.evento == PRESIONADO) {
    Monitor.print(p.nombre);
    Monitor.println(": PRESIONADO");
  } else if (p.evento == LIBERADO) {
    Monitor.print(p.nombre);
    Monitor.println(": LIBERADO");
  }
}


void TestPulsadores() {

  MostrarEventoPulsador(pulsador1);
  MostrarEventoPulsador(pulsador2);
  MostrarEventoPulsador(pulsador3);
}


// ============================================================
//                    TEST SENSORES
// ============================================================

void TestSensores() {

  if (millis() - tiempoSensores >= 100) {

    tiempoSensores = millis();

    for (int i = 0; i < NUM_SENSORES_LINEA; i++) {
      Monitor.print(i == 0 ? "S" : "  S");
      Monitor.print(i);
      Monitor.print("=");
      Monitor.print(analogRead(PINES_SENSORES_LINEA[i]));
    }

    Monitor.println();
  }
}


// ============================================================
//                      TEST MOTORES
// ============================================================

void TestMotores(String comando) {

  int espacio = comando.indexOf(' ');

  if (espacio == -1) {

    Monitor.println("ERROR");
    Monitor.println("Ejemplo: 30 30");
    Monitor.println("Ejemplo: -30 -30");

    return;
  }


  int izquierda = constrain(comando.substring(0, espacio).toInt(), -VELOCIDAD_MAX, VELOCIDAD_MAX);
  int derecha = constrain(comando.substring(espacio + 1).toInt(), -VELOCIDAD_MAX, VELOCIDAD_MAX);


  MotorIzquierdo(izquierda);
  MotorDerecho(derecha);


  Monitor.print("IZQ = ");
  Monitor.print(izquierda);

  Monitor.print("    DER = ");
  Monitor.println(derecha);
}


// ============================================================
//                       TEST CAMARA
// ============================================================

// Muestra la deteccion que envia Python: LED verde = globo centrado, LED rojo = descentrado
void TestCamara() {

  VisionLink::Direction dir = vision.direction();

  EscribirLed(PIN_LED_VERDE, dir == VisionLink::CENTER);
  EscribirLed(PIN_LED_ROJO, dir == VisionLink::LEFT || dir == VisionLink::RIGHT);

  if (millis() - tiempoCamara < PERIODO_IMPRESION_MS) {
    return;
  }

  tiempoCamara = millis();

  if (dir == VisionLink::NONE) {
    Monitor.println("sin globo");
    return;
  }

  Monitor.print("error: ");
  Monitor.print(vision.error());
  Monitor.println(dir == VisionLink::LEFT ? " (izquierda)"
                  : dir == VisionLink::RIGHT ? " (derecha)" : " (centrado)");
}


// ============================================================
//                     COLORES OBJETIVO
// ============================================================

// Siguiente color del ciclo propio de cada pulsador
ColorGlobo SiguienteColor(ColorGlobo actual, const ColorGlobo ciclo[NUM_COLORES]) {

  for (int i = 0; i < NUM_COLORES; i++) {
    if (ciclo[i] == actual) {
      return ciclo[(i + 1) % NUM_COLORES];
    }
  }

  return ciclo[0];
}


void MostrarColores() {

  Monitor.print("COLOR INICIAL: ");
  Monitor.print(NOMBRES_COLOR[colorInicial]);
  Monitor.print("  COLOR FINAL: ");
  Monitor.println(NOMBRES_COLOR[colorFinal]);
}


// P2 / P3 en reposo: cambia el color y devuelve true si hay que volver a sincronizar con Python
bool ActualizarColores() {

  bool cambio = false;

  if (pulsador2.evento == PRESIONADO) {
    colorInicial = SiguienteColor(colorInicial, CICLO_COLOR_P2);
    cambio = true;
  }

  if (pulsador3.evento == PRESIONADO) {
    colorFinal = SiguienteColor(colorFinal, CICLO_COLOR_P3);
    cambio = true;
  }

  if (cambio) {
    MostrarColores();
  }

  return cambio;
}


// Python vuelve a IDLE (RESET) y recibe los colores; se repite hasta que confirma
void EnviarColores() {

  tiempoSincronizacion = millis();

  Bridge.notify(RPC_TRANSITION, String(TRANSICION_RESET));
  Bridge.notify(RPC_SET_GOALS, String(NOMBRES_COLOR[colorInicial]), String(NOMBRES_COLOR[colorFinal]));
}


// ============================================================
//                  MAQUINA DE ESTADOS ROBOT
// ============================================================

const char* NombreEstado(EstadoRobot e) {

  switch (e) {
    case SINCRONIZANDO:    return "SINCRONIZANDO";
    case ESPERANDO_INICIO: return "ESPERANDO_INICIO";
    case CUENTA_ATRAS:     return "CUENTA_ATRAS";
    case EN_MARCHA:        return "EN_MARCHA";
    case FIN:              return "FIN";
  }

  return "?";
}


// Acciones de entrada de cada estado
void CambiarEstado(EstadoRobot nuevo) {

  estado = nuevo;
  tiempoEstado = millis();

  PararMotores();
  ApagarLeds();

  estadoLed = false;
  tiempoLeds = millis();

  Monitor.print("ESTADO -> ");
  Monitor.println(NombreEstado(estado));

  switch (estado) {

    case SINCRONIZANDO:
      EnviarColores();
      break;

    case ESPERANDO_INICIO:
      EscribirLed(PIN_LED_ROJO, true);
      MostrarColores();
      Monitor.println("P1 = arrancar, P2/P3 = cambiar colores");
      break;

    case EN_MARCHA:
      // no se aplica ninguna referencia anterior al START
      velIzqRef = 0;
      velDerRef = 0;
      tiempoComando = 0;

      // la FSM de Python pasa de IDLE a WANDER
      Bridge.notify(RPC_TRANSITION, String(TRANSICION_START));
      break;

    case FIN:
      EscribirLed(PIN_LED_VERDE, true);
      Monitor.println("FIN DE PARTIDA: motores parados (P1 = volver a reposo)");

      // la FSM de Python vuelve a IDLE y deja de enviar referencias
      Bridge.notify(RPC_TRANSITION, String(TRANSICION_RESET));
      break;

    default:
      break;
  }
}


// Vuelve a reposo conservando los colores; se resincronizan con Python (que vuelve a IDLE)
void ReiniciarEstrategia() {

  CambiarEstado(SINCRONIZANDO);
}


void ActualizarEstrategia() {

  switch (estado) {

    case SINCRONIZANDO:

      ParpadearLed(PIN_LED_ROJO, 1000);

      if (ActualizarColores() || millis() - tiempoSincronizacion >= PERIODO_SINCRONIZACION_MS) {
        EnviarColores();
      }
      break;


    case ESPERANDO_INICIO:

      if (pulsador1.evento == PRESIONADO) {
        CambiarEstado(CUENTA_ATRAS);
      } else if (ActualizarColores()) {
        CambiarEstado(SINCRONIZANDO);
      }
      break;


    case CUENTA_ATRAS:

      ParpadearLed(PIN_LED_ROJO, PARPADEO_CUENTA_ATRAS_MS);

      if (millis() - tiempoEstado >= ESPERA_INICIO_MS) {
        CambiarEstado(EN_MARCHA);
      }
      break;


    case EN_MARCHA:

      ParpadearLed(PIN_LED_VERDE, 500);

      if (millis() - tiempoEstado >= DURACION_PARTIDA_MS) {
        CambiarEstado(FIN);
        break;
      }

      AplicarReferencias();
      break;


    case FIN:

      if (pulsador1.evento == PRESIONADO) {
        CambiarEstado(SINCRONIZANDO);
      }
      break;
  }
}


// ============================================================
//                 FUNCIONES QUE LLAMA PYTHON
// ============================================================

// Registradas con provide_safe: se ejecutan dentro del loop(),
// asi que pueden tocar las variables del sketch sin problemas.
// No hay que llamar a Bridge.call() desde aqui (bloqueo).

// Bridge.call("confirmar_goals", inicial, final): colores que Python ha aplicado en su FSM.
// Devuelve true si coinciden con los elegidos en el MCU; si no, se siguen reenviando.
bool ConfirmarGoals(String inicial, String final_) {

  if (estado != SINCRONIZANDO) {
    return false;
  }

  bool coinciden = inicial == NOMBRES_COLOR[colorInicial] && final_ == NOMBRES_COLOR[colorFinal];

  if (coinciden && modo == MODO_ESTRATEGIA) {
    Monitor.println("COLORES SINCRONIZADOS CON PYTHON");
    CambiarEstado(ESPERANDO_INICIO);
  }

  return coinciden;
}


// Bridge.notify("set_wheels", izquierda, derecha): velocidades de rueda en % [-100, 100].
// Solo se aplican en EN_MARCHA (ver AplicarReferencias).
void SetWheels(int izquierda, int derecha) {

  velIzqRef = constrain(izquierda, -VELOCIDAD_MAX, VELOCIDAD_MAX);
  velDerRef = constrain(derecha, -VELOCIDAD_MAX, VELOCIDAD_MAX);
  tiempoComando = millis();
}


// Bridge.notify("set_detection", error, encontrado): error en px (positivo = globo a la izquierda)
void SetDetection(int error, bool encontrado) {

  vision.update(error, encontrado);
}


// ============================================================
//                    PROCESAR COMANDOS
// ============================================================

void EntrarModo(ModoMenu nuevo) {

  modo = nuevo;

  ReiniciarEstrategia();
  ApagarLeds();

  Monitor.println();
  Monitor.println("================================");

  switch (modo) {

    case MODO_TEST_MOTORES:
      Monitor.println("          TEST MOTORES");
      Monitor.println("================================");
      Monitor.println("IZQ DER en % (-100 a 100)");
      Monitor.println("Ejemplo: 30 30");
      Monitor.println("Ejemplo: -30 -30");
      Monitor.println("0 0 = parar");
      break;

    case MODO_TEST_PULSADORES:
      Monitor.println("        TEST PULSADORES");
      Monitor.println("================================");
      Monitor.println(pulsador1.nombre);
      Monitor.println(pulsador2.nombre);
      Monitor.println(pulsador3.nombre);
      break;

    case MODO_TEST_SENSORES:
      Monitor.println("          TEST SENSORES");
      Monitor.println("================================");
      Monitor.println("S0..S5 -> A0..A5 (de derecha a izquierda)");
      break;

    case MODO_CAMARA:
      Monitor.println("             CAMARA");
      Monitor.println("================================");
      Monitor.println("Deteccion enviada por Python");
      Monitor.println("LED VERDE = centrado, LED ROJO = descentrado");
      break;

    case MODO_TEST_LEDS:
      Monitor.println("           TEST LEDS");
      Monitor.println("================================");
      Monitor.print("LED ROJO  -> PIN ");
      Monitor.println(PIN_LED_ROJO);
      Monitor.print("LED VERDE -> PIN ");
      Monitor.println(PIN_LED_VERDE);
      break;

    default:
      break;
  }

  Monitor.println("MENU = volver al menu");
  Monitor.println("================================");
}


void ProcesarComando() {

  if (!Monitor.available()) {
    return;
  }


  String comando = Monitor.readStringUntil('\n');

  comando.trim();


  // ==========================================================
  //                         MENU
  // ==========================================================

  if (modo == MODO_ESTRATEGIA) {

    int opcion = comando.toInt();

    if (comando.length() == 1 && opcion >= MODO_TEST_MOTORES && opcion <= MODO_TEST_LEDS) {
      EntrarModo((ModoMenu)opcion);
    }

    return;
  }


  // ==========================================================
  //                    VOLVER AL MENU
  // ==========================================================

  if (comando == "MENU" || comando == "0") {

    modo = MODO_ESTRATEGIA;

    Monitor.println();
    Monitor.println("VOLVIENDO AL MENU...");

    ReiniciarEstrategia();

    MenuInicial();

    return;
  }


  // ==========================================================
  //                       MOTORES
  // ==========================================================

  if (modo == MODO_TEST_MOTORES) {

    TestMotores(comando);
  }
}


// ============================================================
//                            SETUP
// ============================================================

void setup() {

  // ----------------------------------------------------------
  // MOTORES
  // ----------------------------------------------------------

  ConfigurarMotor(MOTOR_IZQ);
  ConfigurarMotor(MOTOR_DER);


  // ----------------------------------------------------------
  // PULSADORES
  // ----------------------------------------------------------

  pinMode(PIN_PULSADOR_1, INPUT_PULLUP);
  pinMode(PIN_PULSADOR_2, INPUT_PULLUP);
  pinMode(PIN_PULSADOR_3, INPUT_PULLUP);


  // ----------------------------------------------------------
  // SENSORES
  // ----------------------------------------------------------

  for (int i = 0; i < NUM_SENSORES_LINEA; i++) {
    pinMode(PINES_SENSORES_LINEA[i], INPUT);
  }


  // ----------------------------------------------------------
  // LEDS
  // ----------------------------------------------------------

  pinMode(PIN_LED_ROJO, OUTPUT);
  pinMode(PIN_LED_VERDE, OUTPUT);


  // ----------------------------------------------------------
  // PARAR TODO
  // ----------------------------------------------------------

  PararMotores();
  ApagarLeds();


  // ----------------------------------------------------------
  // BRIDGE (PYTHON) Y MONITOR
  // ----------------------------------------------------------

  Bridge.begin();
  Monitor.begin(115200);

  Bridge.provide_safe(RPC_CONFIRMAR_GOALS, ConfirmarGoals);
  Bridge.provide_safe(RPC_SET_WHEELS, SetWheels);
  Bridge.provide_safe(RPC_SET_DETECTION, SetDetection);

  delay(1000);


  // ----------------------------------------------------------
  // MENU INICIAL
  // ----------------------------------------------------------

  Monitor.println();
  Monitor.println("================================");
  Monitor.println("          ROBOT INICIADO");
  Monitor.println("================================");

  // Hasta que Python confirme los colores no se puede arrancar
  CambiarEstado(SINCRONIZANDO);

  MenuInicial();
}


// ============================================================
//                            LOOP
// ============================================================

void loop() {

  ProcesarComando();

  ActualizarPulsadores();

  switch (modo) {

    case MODO_ESTRATEGIA:
      ActualizarEstrategia();
      break;

    case MODO_TEST_PULSADORES:
      TestPulsadores();
      break;

    case MODO_TEST_SENSORES:
      TestSensores();
      break;

    case MODO_CAMARA:
      TestCamara();
      break;

    case MODO_TEST_LEDS:
      TestLeds();
      break;

    case MODO_TEST_MOTORES:
      break;  // los motores se mueven con los comandos del Monitor
  }

  delay(10);
}
