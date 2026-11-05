#include "touch_sensor.h"
AutoTouchSensor touchSensor;
AutoTouchSensor::AutoTouchSensor() : _lastState(false) {}
void AutoTouchSensor::begin() {
  pinMode(PIN_TOUCH_SEND, OUTPUT);
  pinMode(PIN_TOUCH_RECV, INPUT);
  digitalWrite(PIN_TOUCH_SEND, LOW);
}
uint32_t AutoTouchSensor::measureChargeCycles() {
  digitalWrite(PIN_TOUCH_SEND, LOW);
  delayMicroseconds(10);
  digitalWrite(PIN_TOUCH_SEND, HIGH);
  uint32_t count = 0;
  // Beyond the decision threshold further counting adds no information.
  // The old million-iteration timeout could starve USB on a held-low pin.
  const uint32_t limit = TOUCH_THRESHOLD_SAMPLES + TOUCH_HYSTERESIS + 1U;
  while (count < limit && digitalRead(PIN_TOUCH_RECV) == LOW) ++count;
  digitalWrite(PIN_TOUCH_SEND, LOW);
  return count;
}
bool AutoTouchSensor::readTouch() {
  const uint32_t cycles = measureChargeCycles();
  const uint32_t release = TOUCH_THRESHOLD_SAMPLES > TOUCH_HYSTERESIS
      ? TOUCH_THRESHOLD_SAMPLES - TOUCH_HYSTERESIS : 0;
  if (!_lastState && cycles > TOUCH_THRESHOLD_SAMPLES + TOUCH_HYSTERESIS) _lastState = true;
  else if (_lastState && cycles < release) _lastState = false;
  return _lastState;
}

