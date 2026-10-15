// Recibe de Python (por el Bridge) el error del globo y lo muestra en los LEDs y el Monitor.
// No mueve los motores.
#include "Arduino_RouterBridge.h"
#include "vision_link.h"

const int LED_RED = 7;    // globo detectado pero descentrado
const int LED_GREEN = 6;  // globo centrado
const unsigned long PRINT_PERIOD_MS = 200;

VisionLink vision;
unsigned long lastPrint = 0;

// Python: Bridge.notify("set_detection", error, found)
void set_detection(int error, bool found) {
  vision.update(error, found);
}

void setup() {
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  Monitor.begin();
  Bridge.begin();
  Bridge.provide_safe("set_detection", set_detection);
}

void loop() {
  VisionLink::Direction dir = vision.direction();
  digitalWrite(LED_GREEN, dir == VisionLink::CENTER);
  digitalWrite(LED_RED, dir == VisionLink::LEFT || dir == VisionLink::RIGHT);

  if (millis() - lastPrint >= PRINT_PERIOD_MS) {
    lastPrint = millis();
    if (dir == VisionLink::NONE) {
      Monitor.println("sin globo");
    } else {
      Monitor.print("error: ");
      Monitor.print(vision.error());
      Monitor.println(dir == VisionLink::LEFT ? " (izquierda)"
                      : dir == VisionLink::RIGHT ? " (derecha)" : " (centrado)");
    }
  }
}
