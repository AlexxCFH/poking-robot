#pragma once
#include <Arduino.h>

class VisionLink {
public:
  enum Direction { NONE, LEFT, CENTER, RIGHT };

  VisionLink(unsigned long timeoutMs = 500, int centerTol = 20)
      : timeoutMs_(timeoutMs), centerTol_(centerTol) {}

  void update(int error, bool found) {
    error_ = error;
    found_ = found;
    lastMs_ = millis();
  }

  bool fresh() const { return received_() && (millis() - lastMs_) < timeoutMs_; }
  bool hasBalloon() const { return fresh() && found_; }
  int error() const { return error_; }

  Direction direction() const {
    if (!hasBalloon()) return NONE;
    if (abs(error_) <= centerTol_) return CENTER;
    return error_ > 0 ? LEFT : RIGHT;
  }

private:
  bool received_() const { return lastMs_ != 0; }

  volatile int error_ = 0;
  volatile bool found_ = false;
  volatile unsigned long lastMs_ = 0;
  unsigned long timeoutMs_;
  int centerTol_;
};
