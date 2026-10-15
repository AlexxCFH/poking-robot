#pragma once
#include <Arduino.h>

// ============================================================
//   CONFIGURACION DEL ROBOT (Arduino UNO Q + Motor Shield Rev3)
//   Fuente de verdad de los pines: reglas.pdf (seccion 4)
// ============================================================


// ============================================================
//                          SISTEMA
// ============================================================

// Debe coincidir con FREQUENCY de python/main.py (frecuencia a la que Python envia set_wheels)
const unsigned long FRECUENCIA_SISTEMA_HZ = 20;
const unsigned long PERIODO_SISTEMA_MS = 1000 / FRECUENCIA_SISTEMA_HZ;

// Si Python deja de enviar referencias durante este tiempo se paran los motores
const unsigned long TIMEOUT_COMANDO_MS = 5 * PERIODO_SISTEMA_MS;

const unsigned long ESPERA_INICIO_MS = 5000;       // reglas: 5 s de espera tras el pulsador de inicio
const unsigned long PARPADEO_CUENTA_ATRAS_MS = 500; // el LED rojo cambia de estado cada 500 ms
const unsigned long DURACION_PARTIDA_MS = 180000;  // reglas: 3 min maximo de partida

const unsigned long DEBOUNCE_MS = 20;
const unsigned long PERIODO_SINCRONIZACION_MS = 1000;  // reenvio de colores hasta que Python confirma


// ============================================================
//                  NOMBRES RPC DEL BRIDGE
//   Deben coincidir con las constantes BRIDGE_* de python/control/controller.py
// ============================================================

// Python -> MCU (Bridge.provide_safe en el sketch)
#define RPC_SET_WHEELS       "set_wheels"       // (int izq, int der) en % [-100, 100]
#define RPC_CONFIRMAR_GOALS  "confirmar_goals"  // (String inicial, String final) -> bool, colores aplicados en Python
#define RPC_SET_DETECTION    "set_detection"    // (int error_px, bool encontrado)

// MCU -> Python (Bridge.provide en Python)
#define RPC_SET_GOALS        "set_goals"        // (String inicial, String final), elegidos con P2/P3
#define RPC_TRANSITION       "transition"       // (String transicion)

#define TRANSICION_START "START"  // IDLE -> WANDER al terminar la cuenta atras
#define TRANSICION_RESET "RESET"  // cualquier estado -> IDLE al volver a reposo
#define TRANSICION_STOP  "STOP"   // WANDER/APPROACH/AVOID -> IDLE al pulsar P1 con el robot en marcha


// ============================================================
//                 MOTORES (Arduino Motor Shield Rev3)
// ============================================================
//
//  Funcion          Canal A   Canal B
//  Direccion        D12       D13
//  PWM (velocidad)  D3        D11
//  Freno            D9        D8
//
//  Canal B -> rueda izquierda, canal A -> rueda derecha. Los motores estan montados en
//  espejo, por eso el nivel de direccion que hace avanzar cada rueda es distinto.

typedef struct {
  int pinDireccion;
  int pinPWM;
  int pinFreno;
  bool adelanteEsHigh;  // true si HIGH en el pin de direccion hace avanzar la rueda
} Motor;

const Motor MOTOR_IZQ = { 13, 11, 8, true  };  // canal B
const Motor MOTOR_DER = { 12,  3, 9, false };  // canal A

const int VELOCIDAD_MAX = 100;  // las velocidades de rueda van en % [-VELOCIDAD_MAX, VELOCIDAD_MAX]
const int PWM_MAX = 255;


// ============================================================
//                         PULSADORES
// ============================================================

const int PIN_PULSADOR_1 = 10;  // P1: puesta en marcha (cuenta atras de 5 s)
const int PIN_PULSADOR_2 = 2;   // P2: cambia el color objetivo inicial (solo en reposo)
const int PIN_PULSADOR_3 = 0;   // P3: cambia el color objetivo final (solo en reposo)

typedef enum {
  SIN_CAMBIO,
  PRESIONADO,
  LIBERADO
} EventoPulsador;

typedef struct {
  int pin;
  const char* nombre;
  bool estable;               // ultimo estado confirmado
  bool lectura;               // ultima lectura (puede estar rebotando)
  unsigned long tiempoCambio;
  EventoPulsador evento;      // evento de esta vuelta del loop
} Pulsador;


// ============================================================
//                    SENSOR SIGUE LINEAS
// ============================================================
//
//  Entradas analogicas, de derecha a izquierda: A0 = 2o sensor optico ... A5 = 7o.
//  Nota: A0/A1 son tambien la medida de corriente del Motor Shield; en este robot se usan
//  para el sigue lineas, asi que la medida de corriente no esta disponible.

const int NUM_SENSORES_LINEA = 6;
const int PINES_SENSORES_LINEA[NUM_SENSORES_LINEA] = { A0, A1, A2, A3, A4, A5 };


// ============================================================
//                            LEDS
// ============================================================

const int PIN_LED_ROJO = 7;
const int PIN_LED_VERDE = 6;


// ============================================================
//                     COLORES OBJETIVO
// ============================================================

typedef enum {
  COLOR_AZUL,
  COLOR_ROJO,
  COLOR_VERDE,
  NUM_COLORES
} ColorGlobo;

// Mismos valores que el enum Goal de python/control/fsm.py
const char* const NOMBRES_COLOR[NUM_COLORES] = { "blue", "red", "green" };

const ColorGlobo COLOR_POR_DEFECTO = COLOR_AZUL;

// Orden en que cada pulsador recorre los colores (empezando en azul):
// P2: azul -> rojo -> verde -> azul ...   P3: azul -> verde -> rojo -> azul ...
const ColorGlobo CICLO_COLOR_P2[NUM_COLORES] = { COLOR_AZUL, COLOR_ROJO, COLOR_VERDE };
const ColorGlobo CICLO_COLOR_P3[NUM_COLORES] = { COLOR_AZUL, COLOR_VERDE, COLOR_ROJO };


// ============================================================
//                     ESTADOS Y MODOS
// ============================================================

// Estados de bajo nivel del MCU. La logica de alto nivel (WANDER, APPROACH, ...) la lleva la
// FSM de Python; el MCU solo gestiona el arranque, la duracion de la partida y aplica las
// velocidades de rueda que envia Python mientras esta EN_MARCHA.
//
//  SINCRONIZANDO --(Python confirma colores)--> ESPERANDO_INICIO --(P1)--> CUENTA_ATRAS
//        ^   ^                                                                | 5 s
//        |   +-------------------(P1: parada manual, STOP)---------------+    v
//        +-----------------------(P1)----------- FIN <--(3 min)---------- EN_MARCHA
//
//  P2/P3 en cualquier estado salvo EN_MARCHA (motores parados) -> cambia color -> SINCRONIZANDO
//  SINCRONIZANDO y ESPERANDO_INICIO equivalen al estado IDLE de la FSM de Python.
typedef enum {
  SINCRONIZANDO,     // envia RESET + colores a Python hasta que los confirma
  ESPERANDO_INICIO,  // colores sincronizados, esperando P1
  CUENTA_ATRAS,      // 5 s de espera reglamentaria con el LED rojo parpadeando (P2/P3 la cancelan)
  EN_MARCHA,         // se aplican las referencias de Python (envia START al entrar; P1 = parada)
  FIN                // tiempo de partida agotado, motores parados (envia RESET)
} EstadoRobot;

// Modos del menu del Monitor
typedef enum {
  MODO_ESTRATEGIA = 0,
  MODO_TEST_MOTORES = 1,
  MODO_TEST_PULSADORES = 2,
  MODO_TEST_SENSORES = 3,
  MODO_CAMARA = 4,
  MODO_TEST_LEDS = 5
} ModoMenu;
